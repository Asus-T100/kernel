/*
 * platform_panel.c: panel platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/intel-mid.h>
#include <linux/string.h>
#include <linux/sfi.h>
#include <linux/panel_psb_drv.h>

void panel_handler(struct sfi_device_table_entry *pentry,
				struct devs_id *dev) {
	void *pdata = NULL;

	pr_info("Panel name = %16.16s\n", pentry->name);

	if (!strcmp(pentry->name, "PANEL_CMI_CMD"))
		PanelID = CMI_CMD;
	else if (!strcmp(pentry->name, "PANEL_JDI_VID"))
		PanelID = JDI_VID;
	else if (!strcmp(pentry->name, "PANEL_JDI_CMD"))
		PanelID = JDI_CMD;
}
