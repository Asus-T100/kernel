/**************************************************************************
 * Copyright (c) 2007-2008, Intel Corporation.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#ifndef _PANEL_PSB_DRV_H_
#define _PANEL_PSB_DRV_H_

extern int PanelID;

enum panel_type {
	TPO_CMD,
	TPO_VID,
	TMD_CMD,
	TMD_VID,
	TMD_6X10_VID,
	PYR_CMD,
	PYR_VID,
	TPO,
	TMD,
	PYR,
	HDMI,
	JDI_VID,
	JDI_CMD,
	CMI_VID,
	CMI_CMD,
	GCT_DETECT
};

#endif
