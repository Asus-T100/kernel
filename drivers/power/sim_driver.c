/*
 * sim_driver.c - Intel Merrifield Platform Charging Simulation Driver
 *
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ajay Thomas D <ajay.thomas.david.rajamanickam@intel.com>
 *
 * DEVICE_NAME: Intel Merrifield platform - Simulation module
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/power/sim_driver.h>
#include <linux/usb/penwell_otg.h>
#include <linux/usb/otg.h>

#define DRIVER_NAME "basin_cove_sim"
#define DEVICE_NAME "bc_sim"
#define MODNAME "bc_sim"

static bool access_count;
struct device *bc_batt_sim_dev;
static struct sim_drv_prop *sim_prop;

struct otg_transceiver *sim_otg;

void batt_sim_register_callback(
	void (*charger_irq_callback)(
		uint16_t chgrirq0, uint16_t schgrirq0, uint8_t ext_chrgr_stat))
{
	sim_prop->chgr_irq_callback = charger_irq_callback;

}
EXPORT_SYMBOL_GPL(batt_sim_register_callback);

/*charger event*/
static ssize_t chrgr_event_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t chrgr_event_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(charger_event, S_IRUGO | S_IWUSR, chrgr_event_get,
			chrgr_event_set);

/*charger schrgrirq+chrgrirq*/
static ssize_t chrgr_intr_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t chrgr_intr_set(struct device *device,
		struct device_attribute *attr, const char *buf,
				size_t count);
static DEVICE_ATTR(chrgr_intr, S_IRUGO | S_IWUSR, chrgr_intr_get,
					chrgr_intr_set);

/*external charger control/status reg*/
static ssize_t ext_chrgr_stat_get(struct device *device,
			struct device_attribute *attr, char *buf);

static ssize_t ext_chrgr_stat_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(ext_chrgr_stat, S_IRUGO | S_IWUSR, ext_chrgr_stat_get,
				ext_chrgr_stat_set);


/*charger initial voltage level after charger event*/
static ssize_t chrgr_init_vol_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t chrgr_init_vol_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(chrgr_init_vol, S_IRUGO | S_IWUSR, chrgr_init_vol_get,
				chrgr_init_vol_set);

/*charger initial current after charger event*/
static ssize_t chrgr_init_cur_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t chrgr_init_cur_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(chrgr_init_cur, S_IRUGO | S_IWUSR, chrgr_init_cur_get,
				chrgr_init_cur_set);

/*Battery temperature - Read by charger driver from here
instead of reading from ADC*/

static ssize_t battery_temp_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t battery_temp_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(sim_battery_temp, S_IRUGO | S_IWUSR, battery_temp_get,
				battery_temp_set);

/*Battery voltage delta - change in voltage with after every delay time*/

static ssize_t batt_vol_del_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(sim_batt_vol_del, S_IWUSR, NULL, batt_vol_del_set);

/*Battery current delta - change(decrease) in current while charging*/

static ssize_t batt_cur_del_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(sim_batt_cur_del, S_IWUSR, NULL, batt_cur_del_set);

/*Battery current delta - change(decrease) in current while charging*/

static ssize_t batt_cur_dis_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(sim_batt_cur_dis, S_IWUSR, NULL, batt_cur_dis_set);

/*VBUS voltage*/
static ssize_t vbus_vol_get(struct device *device,
		struct device_attribute *attr, char *buf);

static ssize_t vbus_vol_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count);
static DEVICE_ATTR(sim_vbus_vol, S_IRUGO | S_IWUSR, vbus_vol_get, vbus_vol_set);


static ssize_t chrgr_event_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count)
{
	unsigned long event_type;
	struct otg_transceiver *otg;

	if (strict_strtoul(buf, 10, &event_type))
		return -EINVAL;

		switch (event_type) {

		case CHRG_DISCONN:
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
			break;
		case CHRG_SUSPEND:
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_SUSPEND;
			break;
		case CHRG_CONNECT_SDP_LOW:
			sim_prop->ps_change_cap.chrg_type
						= POWER_SUPPLY_TYPE_USB;
			/*The current limit macro taken from penwell_otg.h*/
			sim_prop->ps_change_cap.mA = CHRG_CURR_SDP_LOW;
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			break;
		case CHRG_CONNECT_SDP_HIGH:
			sim_prop->ps_change_cap.chrg_type
					= POWER_SUPPLY_TYPE_USB;
			sim_prop->ps_change_cap.mA = CHRG_CURR_SDP_HIGH;
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			break;
		case CHRG_CONNECT_DCP:
			sim_prop->ps_change_cap.chrg_type
					= POWER_SUPPLY_TYPE_USB_DCP;
			sim_prop->ps_change_cap.mA = CHRG_CURR_DCP;
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			break;
		case CHRG_CONNECT_CDP:
			sim_prop->ps_change_cap.chrg_type
					= POWER_SUPPLY_TYPE_USB_CDP;
			sim_prop->ps_change_cap.mA = CHRG_CURR_CDP;
			sim_prop->ps_change_cap.chrg_evt
				= POWER_SUPPLY_CHARGER_EVENT_CONNECT;
			break;
		default:
			return -EINVAL;

		}
	sim_prop->event_val = event_type;
	/*Publish notification to the registered drivers on
					charger event change*/
	otg = otg_get_transceiver();
	if (otg == NULL) {
		dev_err(bc_batt_sim_dev, "Failed to get otg transceiver\n");
		return -EINVAL;
	}
	atomic_notifier_call_chain(&otg->notifier,
				0, &(sim_prop->ps_change_cap));

	if (otg)
		otg_put_transceiver(otg);

	return count;

}

static ssize_t chrgr_event_get(struct device *device,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_prop->event_val);
}

/*set initial voltage during any event*/
static ssize_t chrgr_init_vol_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	sim_prop->volt_now = value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

static ssize_t chrgr_init_vol_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_prop->volt_now);
}

/*increase/decrease voltage by delta*/
static ssize_t batt_vol_del_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	if (sim_prop->ps_change_cap.chrg_evt
		== POWER_SUPPLY_CHARGER_EVENT_CONNECT)
		sim_prop->volt_now += value;
	else
		sim_prop->volt_now -= value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;

}

/*set initial current during connect event*/
static ssize_t chrgr_init_cur_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	if (sim_prop->ps_change_cap.chrg_evt
		== POWER_SUPPLY_CHARGER_EVENT_CONNECT)
		sim_prop->curr_now = value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

static ssize_t chrgr_init_cur_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_prop->curr_now);
}

/*decrease current by delta when charging*/
static ssize_t batt_cur_del_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	if (sim_prop->ps_change_cap.chrg_evt
		== POWER_SUPPLY_CHARGER_EVENT_CONNECT)
		sim_prop->curr_now -= value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

/*set as current_now when discharging*/
static ssize_t batt_cur_dis_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	if (sim_prop->ps_change_cap.chrg_evt
				!= POWER_SUPPLY_CHARGER_EVENT_CONNECT)
		sim_prop->curr_now = value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

/*set battery temperature-for charger driver*/
static ssize_t battery_temp_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	sim_prop->batt_temp = value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

static ssize_t battery_temp_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sim_prop->batt_temp);
}

/*set vbus voltage*/
static ssize_t vbus_vol_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)

{
	unsigned long value;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&sim_prop->prop_lock);
	sim_prop->vbus_volt_val = value;
	mutex_unlock(&sim_prop->prop_lock);

	return count;
}

static ssize_t vbus_vol_get(struct device *device,
		struct device_attribute *attr, char *buf)\
{
	return sprintf(buf, "%d\n", sim_prop->vbus_volt_val);
}


static ssize_t chrgr_intr_set(struct device *device,
		struct device_attribute *attr, const char *buf,
			size_t count)
{
	long unsigned int value;

	if (strict_strtoul(buf, 16, &value))
		return -EINVAL;

	sim_prop->chgrirq0_val = value & 0xFF;
	sim_prop->schgrirq0_val = (value & 0xFF00) >> 8;
	/*callback will happen once data written into ext_chgr_stat,
	write should happen in the order:
		1. chrgr_intr and 2.ext_chgr_stat*/

	return count;
}

static ssize_t chrgr_intr_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x%x\n", sim_prop->schgrirq0_val,
					sim_prop->chgrirq0_val);
}

static ssize_t ext_chrgr_stat_set(struct device *device,
			struct device_attribute *attr, const char *buf,
				size_t count)
{
	long unsigned int value;

	if (strict_strtoul(buf, 16, &value))
		return -EINVAL;

	sim_prop->ext_chgr_stat = value;

	if (sim_prop->chgr_irq_callback)
		sim_prop->chgr_irq_callback(sim_prop->chgrirq0_val,
			sim_prop->schgrirq0_val, sim_prop->ext_chgr_stat);
	else
		dev_err(bc_batt_sim_dev, "%s: No IRQ registration %d\n",
					__func__, __LINE__);

	return count;
}
static ssize_t ext_chrgr_stat_get(struct device *device,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", sim_prop->ext_chgr_stat);
}

/*APIs to be exported to the charger driver*/
int batt_sim_get_batt_temp(int *tmp)
{
	mutex_lock(&sim_prop->prop_lock);
	*tmp = sim_prop->batt_temp;
	mutex_unlock(&sim_prop->prop_lock);
	return 0;
}
EXPORT_SYMBOL(batt_sim_get_batt_temp);

int batt_sim_get_batt_current(int *cur)
{
	mutex_lock(&sim_prop->prop_lock);
	*cur = sim_prop->curr_now;
	mutex_unlock(&sim_prop->prop_lock);
	return 0;
}
EXPORT_SYMBOL(batt_sim_get_batt_current);

int batt_sim_get_batt_volt(int *volt)
{
	mutex_lock(&sim_prop->prop_lock);
	*volt = sim_prop->volt_now;
	mutex_unlock(&sim_prop->prop_lock);
	return 0;
}
EXPORT_SYMBOL(batt_sim_get_batt_volt);

int batt_sim_get_vbus_volt(int *vbus_volt)
{
	mutex_lock(&sim_prop->prop_lock);
	*vbus_volt = sim_prop->vbus_volt_val;
	mutex_unlock(&sim_prop->prop_lock);
	return 0;
}
EXPORT_SYMBOL(batt_sim_get_vbus_volt);

/*SYSFS GROUPING*/

static struct attribute *sim_attrs[] = {
	&dev_attr_charger_event.attr,
	&dev_attr_chrgr_intr.attr,
	&dev_attr_ext_chrgr_stat.attr,
	&dev_attr_chrgr_init_vol.attr,
	&dev_attr_chrgr_init_cur.attr,
	&dev_attr_sim_battery_temp.attr,
	&dev_attr_sim_batt_vol_del.attr,
	&dev_attr_sim_batt_cur_del.attr,
	&dev_attr_sim_batt_cur_dis.attr,
	&dev_attr_sim_vbus_vol.attr,
	NULL,
};

static struct attribute_group sim_attr_gr = {
	.name = "mrfl_sim_drv",
	.attrs = sim_attrs
};

static int sim_open(struct inode *pinode, struct file *pfile)
{
	mutex_lock(&sim_prop->access_lock);
	if (access_count) {
		mutex_unlock(&sim_prop->access_lock);
		return -EBUSY;
	}
	access_count = 1;
	mutex_unlock(&sim_prop->access_lock);

	return 0;
}

static int sim_release(struct inode *pinode, struct file *pfile)
{
	mutex_lock(&sim_prop->access_lock);
	access_count = 0;
	mutex_unlock(&sim_prop->access_lock);

	return 0;
}

const struct file_operations sim_fops = {
owner: THIS_MODULE,
open : sim_open,
release : sim_release,
};

struct miscdevice sim_dev = {
minor: MISC_DYNAMIC_MINOR,
name : MODNAME,
fops : &sim_fops,
};

static int __init bc_sim_init(void)
{
	int ret;

	ret = misc_register(&sim_dev);
	if (ret)
		goto exit;

	bc_batt_sim_dev = sim_dev.this_device;
	sim_prop = devm_kzalloc(bc_batt_sim_dev, sizeof(struct sim_drv_prop),
					GFP_KERNEL);
	if (sim_prop == NULL)
		return -ENOMEM;

	sim_otg = devm_kzalloc(bc_batt_sim_dev,
				sizeof(struct otg_transceiver), GFP_KERNEL);
	if (sim_otg == NULL)
		return -ENOMEM;

	sim_otg->dev = bc_batt_sim_dev;
	sim_otg->label = "bc_batt_sim_dev";

	ret = otg_set_transceiver(sim_otg);
	if (ret)
		dev_err(bc_batt_sim_dev, "set trans failed\n");

	 ATOMIC_INIT_NOTIFIER_HEAD(&(sim_otg->notifier));

	/*Default initial values for capability*/
	sim_prop->ps_change_cap.mA = CHRG_CURR_DISCONN;
	sim_prop->ps_change_cap.chrg_type = POWER_SUPPLY_TYPE_BATTERY;
	sim_prop->ps_change_cap.chrg_evt =
				POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;

	/* Creating a sysfs group with sim_attr_gr attributes */
	ret = sysfs_create_group(&sim_dev.this_device->kobj, &sim_attr_gr);
	if (ret) {
		dev_err(bc_batt_sim_dev, "sysfs create group failed\n");
		goto exit;
	}
	mutex_init(&sim_prop->prop_lock);
	mutex_init(&sim_prop->access_lock);
exit:
	return ret;
}

static void __exit bc_sim_exit(void)
{
	sysfs_remove_group(&sim_dev.this_device->kobj, &sim_attr_gr);
	misc_deregister(&sim_dev);
}

module_init(bc_sim_init);
module_exit(bc_sim_exit);

MODULE_DESCRIPTION("BC_Charger Simulation Driver");
MODULE_LICENSE("GPL");
