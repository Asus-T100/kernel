/*
 * platform_vlv2_plat_clk.c - VLV2 platform clock driver
 * Copyright (C) 2013 Intel Corporation
 *
 * Author: Asutosh Pathak <asutosh.pathak@intel.com>
 * Author: Chandra Sekhar Anagani <chandra.sekhar.anagani@intel.com>
 * Author: Sergio Aguirre <sergio.a.aguirre.rodriguez@intel.com>
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>

void __init *vlv2_plat_clk_device_platform_data(void *info)
{
	int ret;
	struct platform_device *pdev;
	struct sfi_device_table_entry *pentry = info;

	pdev = platform_device_register_simple(pentry->name, -1, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		pr_err("platform_vlv2_plat_clk:register failed: %d\n", ret);
	}

	return NULL;
}

