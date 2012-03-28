/*
 * platform_camera.c: Camera platform library file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/atomisp_platform.h>
#include <media/v4l2-subdev.h>
#include <asm/intel-mid.h>
#include "platform_camera.h"

/*
 * One-time gpio initialization.
 * @name: gpio name: coded in SFI table
 * @gpio: gpio pin number (bypass @name)
 * @dir: GPIOF_DIR_IN or GPIOF_DIR_OUT
 * @value: if dir = GPIOF_DIR_OUT, this is the init value for output pin
 * if dir = GPIOF_DIR_IN, this argument is ignored
 * return: a positive pin number if succeeds, otherwise a negative value
 */
int camera_sensor_gpio(int gpio, char *name, int dir, int value)
{
	int ret, pin;

	if (gpio == -1) {
		pin = get_gpio_by_name(name);
		if (pin == -1) {
			pr_err("%s: failed to get gpio(name: %s)\n",
						__func__, name);
			return -EINVAL;
		}
	} else {
		pin = gpio;
	}

	ret = gpio_request(pin, name);
	if (ret) {
		pr_err("%s: failed to request gpio(pin %d)\n", __func__, pin);
		return -EINVAL;
	}

	if (dir == GPIOF_DIR_OUT)
		ret = gpio_direction_output(pin, value);
	else
		ret = gpio_direction_input(pin);

	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
							__func__, pin);
		gpio_free(pin);
	}

	return ret ? ret : pin;
}

/*
 * Configure MIPI CSI physical parameters.
 * @port: ATOMISP_CAMERA_PORT_PRIMARY or ATOMISP_CAMERA_PORT_SECONDARY
 * @lanes: for ATOMISP_CAMERA_PORT_PRIMARY, there could be 2 or 4 lanes
 * for ATOMISP_CAMERA_PORT_SECONDARY, there is only one lane.
 * @format: MIPI CSI pixel format, see include/linux/atomisp_platform.h
 * @bayer_order: MIPI CSI bayer order, see include/linux/atomisp_platform.h
 */
int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *csi = NULL;

	if (flag) {
		csi = kzalloc(sizeof(*csi), GFP_KERNEL);
		if (!csi) {
			dev_err(&client->dev, "out of memory\n");
			return -ENOMEM;
		}
		csi->port = port;
		csi->num_lanes = lanes;
		csi->input_format = format;
		csi->raw_bayer_order = bayer_order;
		v4l2_set_subdev_hostdata(sd, (void *)csi);
	} else {
		csi = v4l2_get_subdev_hostdata(sd);
		kfree(csi);
	}

	return 0;
}
static int no_v4l2_dev_ids(void)
{
	const struct intel_v4l2_subdev_id *v4l2_ids_ptr = v4l2_ids;
	int no_v4l2_ids = 0;

	while (v4l2_ids_ptr->name[0]) {
		no_v4l2_ids++;
		v4l2_ids_ptr++;
	}

	return no_v4l2_ids;
}
static const struct intel_v4l2_subdev_id *get_v4l2_ids(int *n_subdev)
{
	if (n_subdev && v4l2_ids)
		*n_subdev = no_v4l2_dev_ids();
	return v4l2_ids;
}

static struct atomisp_platform_data *v4l2_subdev_table_head;

void intel_ignore_i2c_device_register(struct sfi_device_table_entry *pentry,
					struct devs_id *dev)
{
	struct i2c_board_info i2c_info;
	struct i2c_board_info *idev = &i2c_info;
	int bus = pentry->host_num;
	void *pdata = NULL;
	int n_subdev;
	const struct intel_v4l2_subdev_id *vdev = get_v4l2_ids(&n_subdev);
	struct intel_v4l2_subdev_i2c_board_info *info;
	static struct intel_v4l2_subdev_table *subdev_table;
	enum intel_v4l2_subdev_type type = 0;
	enum atomisp_camera_port port;
	static int i;

	if (vdev == NULL) {
		pr_info("ERROR: camera vdev list is NULL\n");
		return;
	}

	memset(&i2c_info, 0, sizeof(i2c_info));
	strncpy(i2c_info.type, pentry->name, SFI_NAME_LEN);
	i2c_info.irq = ((pentry->irq == (u8)0xff) ? 0 : pentry->irq);
	i2c_info.addr = pentry->addr;
	pr_info("I2C bus = %d, name = %16.16s, "
		"irq = 0x%2x, addr = 0x%x\n",
		pentry->host_num,
		i2c_info.type,
		i2c_info.irq,
		i2c_info.addr);
	pdata = dev->get_platform_data(&i2c_info);
	i2c_info.platform_data = pdata;


	while (vdev->name[0]) {
		if (!strncmp(vdev->name, idev->type, 16)) {
			/* compare name */
			type = vdev->type;
			port = vdev->port;
			break;
		}
		vdev++;
	}

	if (!type) /* not found */
		return;

	info = kzalloc(sizeof(struct intel_v4l2_subdev_i2c_board_info),
		       GFP_KERNEL);
	if (!info) {
		pr_err("MRST: fail to alloc mem for ignored i2c dev %s\n",
		       idev->type);
		return;
	}

	info->i2c_adapter_id = bus;
	/* set platform data */
	memcpy(&info->board_info, idev, sizeof(*idev));

	if (v4l2_subdev_table_head == NULL) {
		subdev_table = kzalloc(sizeof(struct intel_v4l2_subdev_table)
			* n_subdev, GFP_KERNEL);

		if (!subdev_table) {
			pr_err("MRST: fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			return;
		}

		v4l2_subdev_table_head = kzalloc(
			sizeof(struct atomisp_platform_data), GFP_KERNEL);
		if (!v4l2_subdev_table_head) {
			pr_err("MRST: fail to alloc mem for v4l2_subdev_table %s\n",
			       idev->type);
			kfree(info);
			kfree(subdev_table);
			return;
		}
		v4l2_subdev_table_head->subdevs = subdev_table;
	}

	memcpy(&subdev_table[i].v4l2_subdev, info, sizeof(*info));
	subdev_table[i].type = type;
	subdev_table[i].port = port;
	i++;
	kfree(info);
	return;
}

const struct atomisp_platform_data *intel_get_v4l2_subdev_table(void)
{
	if (v4l2_subdev_table_head)
		return v4l2_subdev_table_head;
	else {
		pr_err("MRST: no camera device in the SFI table\n");
		return NULL;
	}
}
EXPORT_SYMBOL_GPL(intel_get_v4l2_subdev_table);

static int camera_af_power_gpio = -1;

static int camera_af_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	return gpio_direction_output(camera_af_power_gpio, flag);
}

const struct camera_af_platform_data *camera_get_af_platform_data(void)
{
	static const int GP_CORE = 96;
	static const int GPIO_DEFAULT = GP_CORE + 76;
	static const char gpio_name[] = "CAM_0_AF_EN";
	static const struct camera_af_platform_data platform_data = {
		.power_ctrl = camera_af_power_ctrl
	};
	int gpio, r;

	if (camera_af_power_gpio == -1) {
		gpio = get_gpio_by_name(gpio_name);
		if (gpio < 0) {
			pr_err("%s: can not find gpio `%s', using default\n",
				__func__, gpio_name);
			gpio = GPIO_DEFAULT;
		}
		r = gpio_request(gpio, gpio_name);
		if (r)
			return NULL;
		r = gpio_direction_output(gpio, 0);
		if (r)
			return NULL;
		pr_info("%s: using gpio %i\n", __func__, gpio);
		camera_af_power_gpio = gpio;
	}

	return &platform_data;
}
EXPORT_SYMBOL_GPL(camera_get_af_platform_data);
