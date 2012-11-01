/*
 *  HID driver for N-Trig touchscreens
 *
 *  Copyright (c) 2011 N-TRIG
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */


#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/string.h>

#include "typedef-ntrig.h"
#include "ntrig-common.h"
#include "ntrig-dispatcher.h"

#include "ntrig_virtual_keys_def.h"

#define DISPATCHER_VER_FILE "/etc/ntrig_version"

config_callback		read_config_dispatcher;
config_callback		write_config_dispatcher;
message_callback	read_config_sensor;
message_callback_count	write_config_sensor;
message_callback	read_get_bus_interface;
message_callback	write_get_bus_interface;
read_counters_callback  read_get_counters;
reset_counters_callback write_reset_counters;

/**
 * virtual keys functionallity
 */
#ifdef VIRTUAL_KEYS_SUPPORTED
/* no error */
#define EOK              0
/* we don't expect numbers larger than 10 digits... */
#define CHARS_PER_NUMBER 10
/* This is the constant part of the file name,
 * will be concatenate with user defined part */
#define VIRTUAL_KEYS_FILE_PREFIX "virtualkeys."

/* sysfs Local API */
static ssize_t ntrig_virtual_keys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf);

/* attributes buffer size - make sure it doens't exceed PAGE_SIZE
 * (On i386, this is 4096 On i386). Buffer holding the virutal keys
 * definitions(6 values:reserved:scan-code:x:y:w:h; 5 delimiters +
 * \n" +final \0) number of keys */
static char virtual_keys_attr_buf[(CHARS_PER_NUMBER * 6 + sizeof(char)*7) *
							VIRTUAL_KEYS_NUM];
static char virtual_keys_file_name[sizeof(VIRTUAL_KEYS_FILE_PREFIX) +
				   sizeof(NTRIG_DEVICE_NAME) + 1];

/* These structures support single virtual keys device. If more device
 * should be supported in the futures, more static kobj_attribute can be
 * added, ntrig_properties_attrs should be replaced by an array and
 * corresponding code should be change accordingle. */
static struct kobj_attribute ntrig_virtualkeys_attrs =
	 __ATTR(NULL, S_IRUGO, ntrig_virtual_keys_show, NULL);
/* NOTE: actual filename in virtual_keys_file_name will be written later */

static struct attribute *ntrig_properties_attrs = &ntrig_virtualkeys_attrs.attr;
static struct kobject *board_properties_kobj;


/*
 * Read (show) data from the sysfs file
 */
static ssize_t ntrig_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	ntrig_dbg("%s: attributes = %s\n", __func__, virtual_keys_attr_buf);
	return snprintf(buf, PAGE_SIZE, "%s\n", virtual_keys_attr_buf);
}

/*
 * Release sysfs files
 */
static void ntrig_board_properties_release(void)
{
	ntrig_dbg("inside %s\n", __func__);

	if (board_properties_kobj == NULL) {
		ntrig_dbg("inside %s, kobj==NULL\n", __func__);
		return;
	}

	/* remove file virtualkeys.<device>
	 * under folder /sys/board_properties */
	sysfs_remove_file(board_properties_kobj, ntrig_properties_attrs);

	/* remove folder /sys/board_properties
	 * (this function also handle reference count and memroy free) */
	kobject_put(board_properties_kobj);
	board_properties_kobj = NULL;
	ntrig_dbg("inside %s After call to kobject_put\n", __func__);
}

/*
 * Create folder "board_properties" under sysfs,
 * and create file virtualskeys.<dev-name> under this folder
 */
static void ntrig_create_virtualkeys_file(void)
{
	int retval;
	int i;
	char tmpbuf[CHARS_PER_NUMBER*6+sizeof(char)*7];

	ntrig_dbg("inside %s\n", __func__);

	/* At this stage we support a sinlge device for virtual keys. Hence
	 * leaving this function if file for this device is already created.
	 * If we support more devices in the future - this is the place to
	 * create a new file. */
	if (board_properties_kobj) {
		ntrig_dbg("%s: virtualkey file alreay created!\n", __func__);
		return;
	}

	/* 1. Create folder "board_properties" under "sys"
	 * allocate the memory for the whole object */
	board_properties_kobj =
		kobject_create_and_add("board_properties", NULL);
	ntrig_dbg("inside %s\n", __func__);

	if (!board_properties_kobj) {
		ntrig_dbg("%s: kobject_create_and_add FAILED\n", __func__);
		return;
	}

	/* We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	kobject_uevent(board_properties_kobj, KOBJ_ADD);

	/* 2. Create virtual keys file name */
	sprintf(virtual_keys_file_name, "%s%s",
			VIRTUAL_KEYS_FILE_PREFIX, NTRIG_DEVICE_NAME);
	ntrig_properties_attrs->name = virtual_keys_file_name;

	/* 3. create file "virtualkeys" under folder "board_properties",
	 * create the file "virtualkeys.<ntrig-dev-name>" under
	 * "board_properties" folder */
	retval = sysfs_create_file(board_properties_kobj,
						ntrig_properties_attrs);

	if (retval != EOK) {
		ntrig_dbg("inside %s - sysfs_create_file FAILED\n", __func__);

		/* remove properties_kobj */
		kobject_put(board_properties_kobj);
		return;
	}

	/* 3. Write Virtual keys definitions into the file */
	/* There is no actual writing to file system.  Instead, we create a
	 * char buffer containing the data entered by the user, in the format
	 * expected by the reading module. The buffer is global to this source
	 * file, so it can be accessed by the "show" function. */

	/* see expected format of the buffer in function readVirtualKeys in
	 * file KeyInputQueue.java. Data for all keys is written into the
	 * same buffer. Format of data for each key is:
	 * scan-kode:center-x:center-y:width:hight\n */

	virtual_keys_attr_buf[0] = '\0';
	/* a tmp buffer to contain data of one key in string format */
	for (i = 0; i < VIRTUAL_KEYS_NUM; i++) {
		sprintf(tmpbuf, "0x01:%d:%d:%d:%d:%d:",
			_ntrig_virt_keys[i].scan_code,
			_ntrig_virt_keys[i].center_x,
			_ntrig_virt_keys[i].center_y,
			_ntrig_virt_keys[i].width,
			_ntrig_virt_keys[i].height);

		strcat(virtual_keys_attr_buf, tmpbuf);
	}

	/* remove last ':' */
	virtual_keys_attr_buf[strlen(virtual_keys_attr_buf)-1] = 0;

	ntrig_dbg("%s: attributes = %s, len = %d\n", __func__,
			virtual_keys_attr_buf, strlen(virtual_keys_attr_buf));
}

#endif  /* VIRTUAL_KEYS_SUPPORTED */

/* Virtual Keys Exported APIs */
int get_virt_keys_scan_code(int i)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return _ntrig_virt_keys[i].scan_code;
#else
	return false;
#endif
}

int get_virtual_keys_num(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return VIRTUAL_KEYS_NUM;
#else
	return false;
#endif
}

bool virtual_keys_supported(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return true;
#else
	return false;
#endif
}

int get_touch_screen_border_left(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_LEFT;
#else
	return 0;
#endif
}

int get_touch_screen_border_right(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_RIGHT;
#else
	return 0;
#endif
}

int get_touch_screen_border_down(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_DOWN;
#else
	return 0;
#endif
}

int get_touch_screen_border_up(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_UP;
#else
	return 0;
#endif
}

int get_touch_screen_border_pen_left(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_PEN_LEFT;
#else
	return 0;
#endif
}

int get_touch_screen_border_pen_right(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_PEN_RIGHT;
#else
	return 0;
#endif
}

int get_touch_screen_border_pen_down(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_PEN_DOWN;
#else
	return 0;
#endif
}

int get_touch_screen_border_pen_up(void)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	return TOUCH_SCREEN_BORDER_PEN_UP;
#else
	return 0;
#endif
}

int get_virtual_keys(char *buf)
{
#ifdef VIRTUAL_KEYS_SUPPORTED
	char tmp_buf[1024];
#endif
	ntrig_dbg("ntrig-sysfs-dispatcher: inside %s\n", __func__);
#ifdef VIRTUAL_KEYS_SUPPORTED
	snprintf(tmp_buf, PAGE_SIZE, "%s\n", virtual_keys_attr_buf);
	ntrig_dbg("%s: inside %s, keys=%s\n", __func__, tmp_buf);
	return snprintf(buf, PAGE_SIZE, "%s\n", virtual_keys_attr_buf);
#else
	return 0;
#endif
}


/* Dispatcher configuration files */
static ssize_t
config_dispatcher_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval = 0;

	if (strcmp(attr->attr.name, "tracklib_status") == 0) {
		ntrig_dbg("%s: CONFIG_TRACKLIB\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_TRACKLIB);
	} else if (strcmp(attr->attr.name, "pen_status") == 0) {
		ntrig_dbg("%s: CONFIG_PEN\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_PEN);
	} else if (strcmp(attr->attr.name, "ntrig_virtual_keys_show") == 0) {
		ntrig_dbg("%s: CONFIG_VIRTUAL_KEYS\n", __func__);
		/* Virtual keys are held locally in this file */
		retval = get_virtual_keys(buf);
	} else if (strcmp(attr->attr.name, "num_sensors") == 0) {
		ntrig_dbg("%s: CONFIG_NUM_SENSORS\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_NUM_SENSORS);
	} else if (strcmp(attr->attr.name, "touch_screen_border") == 0) {
		ntrig_dbg("%s: CONFIG_TOUCH_SCREEN_BORDER\n", __func__);
		retval = read_config_dispatcher(buf,
					CONFIG_TOUCH_SCREEN_BORDER);
	} else if (strcmp(attr->attr.name, "driver_version") == 0) {
		ntrig_dbg("%s: CONFIG_DRIVER_VERSION\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_DRIVER_VERSION);
	} else if (strcmp(attr->attr.name, "debug_print") == 0) {
		ntrig_dbg("%s: CONFIG_DEBUG_PRINT\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_DEBUG_PRINT);
	} else if (strcmp(attr->attr.name, "bus_interface") == 0) {
		ntrig_dbg("%s: CONFIG_BUS_INTERFACE\n", __func__);
		retval = read_config_dispatcher(buf, CONFIG_BUS_INTERFACE);
	} else {
		ntrig_dbg("%s: unknown configuration\n", __func__);
		retval = DTRG_FAILED;
	}

	return retval;
}

static ssize_t
config_dispatcher_store(struct kobject *kobj, struct kobj_attribute *attr,
						 const char *buf, size_t count)
{
	int retval = 0;
	unsigned char val;

	/*sscanf(buf, "%cu", &val); */
	val = *buf;
	if (strcmp(attr->attr.name, "tracklib_status") == 0) {
		ntrig_dbg("%s: CONFIG_TRACKLIB, data = %d\n", __func__, val);
		retval = write_config_dispatcher(&val, CONFIG_TRACKLIB);
	} else if (strcmp(attr->attr.name, "pen_status") == 0) {
		ntrig_dbg("%s: CONFIG_PEN, data = %d\n", __func__, val);
		retval = write_config_dispatcher(&val, CONFIG_PEN);
	} else if (strcmp(attr->attr.name, "bus_interface") == 0) {
		ntrig_dbg("%s: CONFIG_BUS_INTERFACE, data = %d\n",
								__func__, val);
		retval = write_config_dispatcher(&val, CONFIG_BUS_INTERFACE);
	} else if (strcmp(attr->attr.name, "touch_screen_border") == 0) {
		ntrig_dbg("%s: CONFIG_TOUCH_SCREEN_BORDER, data = %d\n",
							__func__, (int)val);
		retval = write_config_dispatcher(&val,
				CONFIG_TOUCH_SCREEN_BORDER);
	} else if (strcmp(attr->attr.name, "debug_print") == 0) {
		ntrig_dbg("%s: CONFIG_DEBUG_PRINT, data = %d\n",
							__func__, (int)val);
		retval = write_config_dispatcher(&val, CONFIG_DEBUG_PRINT);
	} else {
		ntrig_dbg("%s: unknown configuration\n", __func__);
		retval = DTRG_FAILED;
	}

	return retval;
}

/* get bus interface */
static ssize_t
get_bus_interface_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return read_get_bus_interface((void *)buf);
}

static ssize_t
get_bus_interface_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval = write_get_bus_interface((void *)buf);
	if (retval >= 0)
		return count;
	else
		return retval;
}

static ssize_t
reset_counters_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 1;
	if (buf[0] == '0') {
		if (write_reset_counters != NULL)
			write_reset_counters();
	} else {
		ntrig_dbg("%s: RESET_COUNTERS - wrong data=%d the only valid data is 0\n",
							__func__, *buf);
	}
	return ret;
}

static ssize_t
get_counters_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i, res;
	struct ntrig_counter *counters_list;
	int length;

	ntrig_dbg("inside %s\n", __func__);

	if (read_get_counters == NULL) {
		pr_err("%s: read_get_counters is NULL\n", __func__);
		return -1;
	}
	res = 0;

	read_get_counters(&counters_list, &length);

	for (i = 0; i < length; i++) {
		res += snprintf(buf + res, PAGE_SIZE - res,
			"%s \t %d\n",
			counters_list[i].name, counters_list[i].count);
	}

	return res;
}

static struct kobj_attribute tracklib_status_attribute =
	__ATTR(tracklib_status, S_IRUGO | S_IWUGO,
			config_dispatcher_show, config_dispatcher_store);
static struct kobj_attribute pen_status_attribute =
	__ATTR(pen_status, S_IRUGO | S_IWUGO,
			config_dispatcher_show, config_dispatcher_store);
/* Setting of virtual keys is not allowed */
static struct kobj_attribute ntrig_virtual_keys_show_attribute =
	__ATTR(ntrig_virtual_keys_show, S_IRUGO, config_dispatcher_show, NULL);
/* Setting of sensors number is not allowed */
static struct kobj_attribute num_sensors_attribute =
	__ATTR(num_sensors, S_IRUGO, config_dispatcher_show, NULL);
/* Store function sets sensor_id for reading screen border */
static struct kobj_attribute touch_screen_border_attribute =
	__ATTR(touch_screen_border, S_IRUGO | S_IWUGO,
			config_dispatcher_show, config_dispatcher_store);
static struct kobj_attribute driver_version_attribute =
	__ATTR(driver_version, S_IRUGO, config_dispatcher_show, NULL);
/* Control debug prints */
static struct kobj_attribute debug_print_attribute =
	__ATTR(debug_print, S_IRUGO | S_IWUGO,
			config_dispatcher_show, config_dispatcher_store);
/* TODO check why it is needed */
/*
static struct kobj_attribute bus_interface_attribute =
	__ATTR(bus_interface, S_IRUGO | S_IWUGO,
			get_bus_interface_show, get_bus_interface_store);
*/

/* create sysfs file named "driver_counters"
 * 'cat driver_counters' will print the counters
 * 'echo 0 > driver_counters' will reset all counters to 0
 */
static struct kobj_attribute get_counters_attribute =
	__ATTR(driver_counters, S_IRUGO | S_IWUGO,
			get_counters_show, reset_counters_store);

/* Sensor Configuration file */
static ssize_t
config_sensor_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return read_config_sensor(buf);
}

static ssize_t
config_sensor_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int retval;
	ntrig_dbg("%s: data = %d\n", __func__, buf[1]);

	retval = write_config_sensor((void *) buf, count);

	if (retval)
		return count;
	else
		return retval;
}

static struct kobj_attribute config_sensor_attribute =
	__ATTR(config_sensor, S_IRUGO | S_IWUGO,
			config_sensor_show, config_sensor_store);

static struct kobj_attribute get_bus_interface_attribute =
	__ATTR(get_bus_interface, S_IRUGO | S_IWUGO,
			get_bus_interface_show, get_bus_interface_store);

static struct attribute *attrs[] = {
	&tracklib_status_attribute.attr,
	&pen_status_attribute.attr,
	&ntrig_virtual_keys_show_attribute.attr,
	&num_sensors_attribute.attr,
	&touch_screen_border_attribute.attr,
	&driver_version_attribute.attr,
	&debug_print_attribute.attr,
	/* &bus_interface_attribute.attr, */
	&config_sensor_attribute.attr,
	&get_bus_interface_attribute.attr,
	&get_counters_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs
};

static struct kobject *dispatcher_kobj;

static void dispatcher_files_release(void)
{
	sysfs_remove_group(dispatcher_kobj, &attr_group);
	kobject_put(dispatcher_kobj);
	dispatcher_kobj = NULL;

#ifdef VIRTUAL_KEYS_SUPPORTED
	ntrig_board_properties_release();
#endif
}

int ntrig_dispathcer_sysfs_init(void)
{
	int retval;

	ntrig_dbg("inside %s\n", __func__);

	dispatcher_kobj = kobject_create_and_add("ntrig", NULL);
	if (!dispatcher_kobj) {
		ntrig_dbg("%s: failed to create dispatcher_kobj\n", __func__);
		return -ENOMEM;
	}
	/* Create the files associated with this kobject */
	retval = sysfs_create_group(dispatcher_kobj, &attr_group);
	if (retval) {
		ntrig_dbg("%s: failed to create sysfs_group", __func__);
		kobject_put(dispatcher_kobj);
	}

#ifdef VIRTUAL_KEYS_SUPPORTED
	ntrig_create_virtualkeys_file();
#endif

	ntrig_dbg("ntrig-dispatcher-sysfs inside %s\n", __func__);

	if (setup_config_dispatcher(&read_config_dispatcher,
						&write_config_dispatcher))
		ntrig_dbg("%s: cannot setup_get_bus_interface\n", __func__);
	if (setup_config_sensor(&read_config_sensor, &write_config_sensor))
		ntrig_dbg("%s: cannot setup_config_sensor\n", __func__);
	if (setup_get_bus_interface(&read_get_bus_interface,
						&write_get_bus_interface))
		ntrig_dbg("%s: cannot setup_get_bus_interface\n", __func__);
	if (setup_config_counters(&read_get_counters, &write_reset_counters))
		ntrig_dbg("%s: cannot setup_get_counters\n", __func__);

	return retval;
}

void ntrig_dispathcer_sysfs_exit(void)
{
	ntrig_dbg("inside %s\n", __func__);
	dispatcher_files_release();
}
