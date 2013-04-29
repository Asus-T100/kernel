/*
 * include/linux/cmos_osnib_ilb.h
 *
 * Copyright (C) 2013 Intel Corp
 * Author: Asutosh Pathak <asutosh.pathak@intel.com>
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
 */

#ifndef __CMOS_OSNIB_ILB_H
#define __CMOS_OSNIB_ILB_H

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mc146818rtc.h>
#
/* Size (bytes) of Intel Debug */
#define OSNIB_DEBUG_SIZE	16

/* Size (bytes) of OEM RESERVED */
#define OSNIB_OEM_RSVD_SIZE	32

#define CMOS_OSNIB_BASE_ADDR	0x10

/* fw_update is one of the os_flags attribute */
#define OSNIB_FW_UPDATE_BIT 1

/*
 * OSNIB allocation.
 *
*/

struct cmos_osnib {
	u8 magic[4];
	u8 version_major;
	u8 version_minor;
	u8 header_reserved1;
	u8 header_reserved2;
	u8 intel_area_size;
	u8 oem_area_size;
	u8 boot_flow_type;
	u8 watchdog_flags;
	u8 wake_source;
	u8 debug[OSNIB_DEBUG_SIZE];
	u8 target_mode_attr;
	u8 os_flags;
	u8 checksum;
	u8 oem_reserved[OSNIB_OEM_RSVD_SIZE];
};



int intel_mid_ilb_read_osnib_rr(u8 *rr);
int intel_mid_ilb_write_osnib_rr(u8 rr);

#endif /* __CMOS_OSNIB_ILB_H */
