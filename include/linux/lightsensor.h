/* include/linux/lightsensor.h
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Iliyan Malchev <malchev@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_LIGHTSENSOR_H
#define __LINUX_LIGHTSENSOR_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)
//<ASUS-Hollie 20130912+>
#define ASUS_LSENSOR_SETCALI_DATA _IOW(LIGHTSENSOR_IOCTL_MAGIC, 3, uint32_t)		//<ASUS-Bob20130930+>uint16_t
#define ASUS_LSENSOR_GETCALI_DATA _IOR(LIGHTSENSOR_IOCTL_MAGIC, 5, uint32_t)		//<ASUS-Bob20130930+>uint16_t
#define ASUS_LSENSOR_IOCTL_GETLUXDATA _IOR(LIGHTSENSOR_IOCTL_MAGIC, 4, uint32_t)	//<ASUS-Bob20130930+>int
//<ASUS-Hollie 20130912->

#endif
