#include <linux/string.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <stdarg.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
MODULE_LICENSE("Dual BSD/GPL");

#define MAX_FREQ_NODE "scaling_max_freq"
#define MIN_FREQ_NODE "scaling_min_freq"
#define MAX_CPU_CNT 1 // fix 1 CPU policy and other CPU's policy can be set auto.
#define SYSFS_PATH_MAX 255
#define PATH_TO_CPU "/sys/devices/system/cpu/"
#define STRING_SIZE 16

#define CPUD_DEBUG(fmt, arg...)  \
	pr_debug("CPUD_DEBUG: [%s] " fmt , __func__ , ##arg)
#define CPUD_INFO(fmt, arg...)   \
	pr_info("CPUD_INFO: [%s] " fmt , __func__ , ##arg)
#define CPUD_ERROR(fmt, arg...)  \
	pr_err("CPUD_ERROR: [%s] " fmt , __func__ , ##arg)

struct cpufreq_policy pre_policy[MAX_CPU_CNT]; // save previous policy of 1 CPUs.
static int cpud_Enable = 0; // cpud turn on/off
struct kobject *cpud_kobj;
/* helper function to write a new value to a /sys file */
/* fname is a relative path under "cpuX/cpufreq" dir */
static unsigned int sysfs_cpufreq_write_file(unsigned int cpu,
						 const char *fname,
						 const char *value, size_t len)
{
	char path[SYSFS_PATH_MAX];
	struct file *fd;
	mm_segment_t fs;
	loff_t ppos = 0;
	int err;

	snprintf(path, sizeof(path), PATH_TO_CPU "cpu%u/cpufreq/%s", cpu, fname);
	CPUD_DEBUG( "enter sysfs_cpufreq_write_file(), path=%s\n", path );
	fd = filp_open( path, O_RDWR, 0 );
	if(IS_ERR(fd) || (fd == NULL)) {
		CPUD_ERROR(KERN_ERR "filp_open() error:%ld\n",PTR_ERR(fd));
		return 0;
	}
	if(fd)
	{
		fs = get_fs();
		set_fs(KERNEL_DS);
		err = fd->f_op->write(fd, value, strlen(value), &fd->f_pos);
		CPUD_INFO("write cpufreq done!\n" );
		set_fs(fs);
		filp_close( fd, NULL );
	}
	return 0;
}
/*set platform_max_freq value*/
static unsigned int platform_max_freq_write(
						 const char *fname,
						 const char *value, size_t len)
{
	char path[SYSFS_PATH_MAX];
	struct file *fd;
	mm_segment_t fs;
	loff_t ppos = 0;
	int err;

	snprintf(path, sizeof(path), PATH_TO_CPU "cpufreq/%s", fname);
	CPUD_DEBUG("platform_max_freq_write(), path=%s\n", path );
	fd = filp_open( path, O_RDWR, 0 );
	if(IS_ERR(fd) || (fd == NULL)) {
		CPUD_ERROR("filp_open() error:%ld\n",PTR_ERR(fd));
		return 0;
	}
	if(fd)
	{
		fs = get_fs();
		set_fs(KERNEL_DS);
		err = fd->f_op->write(fd, value, strlen(value), &fd->f_pos);
		CPUD_INFO("write cpufreq done!\n" );
		set_fs(fs);
		filp_close( fd, NULL );
	}
	return 0;
}

static ssize_t show_attrs_handler(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int ret;

	ret = sprintf(buf,"%u\n",cpud_Enable );

	CPUD_DEBUG("show attr buf=%s\n",buf);

	return ret;
}

static ssize_t store_attrs_handler(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{

	unsigned int value;
	struct cpufreq_policy policy;
	char max[STRING_SIZE];
	int ret;

	value = simple_strtoul( buf, NULL, 0 );

	if( value == 1 )
	{
		cpud_Enable = value;
		CPUD_INFO("set cpud enable!\n");
	}
	else if( value == 0 )
	{
		cpud_Enable = 0;
		CPUD_INFO("set cpud disable!\n");
		ret = cpufreq_get_policy(&policy,0);
		CPUD_DEBUG("refcount=%d\n", policy.kobj.kref.refcount );
		if (!ret)
		{
			snprintf(max, STRING_SIZE, "%u", policy.cpuinfo.max_freq );
			// set platform_max_freq at max while turn off CPUD
			platform_max_freq_write("platform_max_freq",max, strlen(max) );
			cpufreq_cpu_put(&policy); // release ref count of kobject of cpufreq_policy
		}
		else
			CPUD_INFO("can't set platform_max_freq\n");
	}
	else
	{
		CPUD_INFO("store attr set value is not correct!\n");
	}

	return strnlen(buf, count);
}
DEVICE_ATTR(cpud_enable, 0644, show_attrs_handler, store_attrs_handler);

static struct attribute *group[] = {
   &dev_attr_cpud_enable.attr,
   NULL,
};

static struct attribute_group attr_group = {
   .attrs = group,
};

static int cpud_early_suspend(void)
{
	unsigned int cpu;
	struct cpufreq_policy policy;
	char min[STRING_SIZE];
	int ret;
	unsigned long min_freq = 0; // minimum freq from cpuinfo

	// cpud disable
	if( !cpud_Enable )
	{
		CPUD_INFO( "cpud is disable,skip store pre-cpufreq policy\n");
		goto exit;
	}

	CPUD_INFO("CPU_Daemon early-suspend\n");

	for (cpu = 0; cpu < MAX_CPU_CNT; cpu++) {
		ret = cpufreq_get_policy(&policy,cpu);
		CPUD_DEBUG("refcount=%d\n", policy.kobj.kref.refcount );
		if (!ret)
		{
			if(cpu == 0 )// get minmum freq of cpuinfo
			{
				min_freq = policy.cpuinfo.min_freq;
				CPUD_INFO("min_freq=%u,cpu cnt=%d\n",min_freq,nr_cpu_ids);
			}
			CPUD_INFO("cpufreq_get_policy() CPU%3d	%9lu kHz  -  %9lu kHz\n", cpu , policy.min, policy.max );
			memcpy(&pre_policy[cpu],&policy,sizeof(struct cpufreq_policy)); // log the previous policy
			CPUD_DEBUG("pre_policy[%d] refcount=%d\n", cpu, pre_policy[cpu].kobj.kref.refcount );

			cpufreq_cpu_put(&policy); // release ref count of kobject of cpufreq_policy
			CPUD_DEBUG("refcount=%d\n", policy.kobj.kref.refcount );

			// ref count-- for pre_policy
			cpufreq_cpu_put(&pre_policy[cpu]);
		}
		else
		{
			CPUD_INFO("get now policy failed!\n");
		}
	}

	// set platform_max_freq at early suspend
	snprintf(min, STRING_SIZE, "%u", min_freq );
	platform_max_freq_write("platform_max_freq", min, strlen(min) );

exit:
	return 0;
}

static int cpud_late_resume(void)
{
	int i,ret;
	char max[STRING_SIZE],min[STRING_SIZE];
	struct cpufreq_policy policy;
	char platform_max_freq[STRING_SIZE];
	unsigned long max_freq = 0; // minimum freq from cpuinfo

	// cpud disable
	if( !cpud_Enable )
	{
		CPUD_INFO("cpud is disable, skip cpufreq restore\n");
		goto exit;
	}

	CPUD_INFO("CPU_Daemon late-resume\n");

	// set platform_max_freq as cpuinfo max freq
	ret = cpufreq_get_policy(&policy,0);
	CPUD_DEBUG("refcount=%d\n", policy.kobj.kref.refcount );
	if (!ret)
	{
		max_freq = policy.cpuinfo.max_freq;
		CPUD_INFO("set platform_max_freq=%u\n",max_freq );
		snprintf(platform_max_freq, STRING_SIZE, "%u", max_freq );
		platform_max_freq_write("platform_max_freq", platform_max_freq, strlen(platform_max_freq) );

		cpufreq_cpu_put(&policy); // release ref count of kobject of cpufreq_policy
		CPUD_DEBUG("refcount=%d\n", policy.kobj.kref.refcount );
	}
	else
		CPUD_INFO("can't set platform_max_freq!\n");

	for( i = 0; i < MAX_CPU_CNT; i++ )
		CPUD_INFO("CPU%d pre_max=%u, pre_min=%u\n",i,pre_policy[i].max, pre_policy[i].min);

	// in byt, we just set policy to 1 CPU and other CPU can bet set auto.
	for( i = 0; i < MAX_CPU_CNT; i++ ) {
		//if( cpu_is_offline(i) ) continue; // cpu offline check
		cpufreq_verify_within_limits( &pre_policy[i], pre_policy[i].cpuinfo.min_freq, pre_policy[i].cpuinfo.max_freq );
		snprintf(max, STRING_SIZE, "%u", pre_policy[i].max );
		snprintf(min, STRING_SIZE, "%u", pre_policy[i].min );
		sysfs_cpufreq_write_file(pre_policy[i].cpu, MAX_FREQ_NODE, max, strlen(max) ); // set pre-policy(max)
		sysfs_cpufreq_write_file(pre_policy[i].cpu, MIN_FREQ_NODE, min, strlen(min) ); // set pre-policy(min)
	}

exit:
	return 0;
}

static struct early_suspend early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 100 + 1, // suspend after display driver
	.suspend = cpud_early_suspend,
	.resume = cpud_late_resume,
};
static int cpud_dev_suspend(void)
{
	CPUD_DEBUG("CPU_Daemon  suspend\n");
	return 0;
}

static int cpud_dev_resume(void)
{
	CPUD_DEBUG("CPU_Daemon resume\n");
	return 0;
}

static int cpud_init(void)
{
	int i,ret;

	for( i = 0; i < MAX_CPU_CNT; i++ )
		memset(&pre_policy[i],0,sizeof(struct cpufreq_policy));

	cpud_kobj = kobject_create_and_add("cpud", NULL);
	if (!cpud_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(cpud_kobj, &attr_group);
	if (ret) {
		kobject_put(cpud_kobj);
		CPUD_ERROR("cpud_init: sysfs_create_group failed\n");
		return ret;
	}

	register_early_suspend(&early_suspend);

	CPUD_INFO("cpud init\n");

	return 0;
}

static void cpud_exit(void)
{
	if(cpud_kobj)
	{
		sysfs_remove_group(cpud_kobj, &attr_group);

		unregister_early_suspend(&early_suspend);
		kobject_put(cpud_kobj);
	}

	CPUD_INFO("cpud exit\n");
}

module_init(cpud_init);
module_exit(cpud_exit);
