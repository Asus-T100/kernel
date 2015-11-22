/*
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ASUSTEK_BOARDINFO_H_
#define _ASUSTEK_BOARDINFO_H_

enum {
	FUN_IFWI_ID = 0,
	FUN_HARDWARE_ID,
	FUN_PROJECT_ID,
	FUN_LCD_ID,
	FUN_TOUCH_ID,
	FUN_CAMERA_SKU_ID,
	FUN_CAMERA_REAR_ID,
	FUN_CAMERA_FRONT_ID,
	FUN_RF_ID,
	FUN_SIM_ID,
	FUN_DRAM_ID,
	FUN_EMMC_ID,
	FUN_SKU_ID,
	RAW_HARDWARE_ID,
	RAW_PROJECT_ID,
	FUN_ID_MAX
};

extern int asustek_boardinfo_get(int fun_num);

/* The defination of function return values
 * Module Naming rule: <Vendor>_<P/N code>[_<other info>]
 * For example:
 * - Camera vendor: Chicony
 * - P/N code: CIFE02920003870LH
 * - Camera Sensor(other info): GC0310
 * => CHICONY_CIFE02920003870LH_GC0310
 */
#define UNKNOWN (-1)
// Hardware ID: Return value of FUN_HARDWARE_ID
enum {
	SR1,
	ER1,
	ER2,
	PR,
};

// Project ID: Return value of FUN_PROJECT_ID
enum {
	TF303CL,
	ME176C,  // K013
	ME176C_L, // ME176C-L (K013A)
	ME176CX_L,  // ME176CX-L (K013B)
	ME176CE,  // ME176CE (K013C)
	TF103CE,  // TF103CE
	TF103C,  //TF103C
	ME181C,  //ME181C(K011)		
	ME176CX,  //K013_1
};

// DRAM ID: Return values of FUN_DRAM_ID
enum {
	HYNIX_H9CCNNNBPTALBR_NTD,
	SAMSUNG_K3QF2F20DM_AGCE,  //TF103CE_SAMSUNG_2G
	SAMSUNG_H9CCNNN8KTALBR_NTD,  //TF103CE_SAMASUNG_1G
	HYNIX_H9CCNNNBKTMLBR_NTD,  //TF103CE_HYNIX_2G
	HYNIX_H9CCNNN8KTALBR_NTD, //TF103CE_HYNIX_1G
};

// TOUCH ID: return values of FUN_TOUCH_ID
enum {
	JTOUCH_8610120765AC05N,
	JTOUCH_8210130763AC05N,  //TF103CE_JTOUCH_BLACK
	OFILM_KTF_101_1521_02AO,  //TF103CE_OFILM_BLACK
};

// LCD ID: return values of FUN_LCD_ID
enum {
	CPT_CLAA101FP05_XG,
	AUO_B101UAN01_7,
	AUO_B101EAN01_6,  // TF103CE_AUO_B101EAN01_6
};

// Camera ID: FUN_CAMERA_REAR_ID, FUN_CAMERA_FRONT_ID
enum {
	CHICONY_CIFDH6520003871LH_MI1040, // 1.2M
	CHICONY_CIFE02920003870LH_GC0310, // 0.3M
	CHICONY_CJAE51120003870LH_AR0543, // 5M
	LITEON_3SF103T2_MI1040, // 1.2M
	LITEON_4SF223T2A_GC2155M, // 2M
	LITEON_4SF227T2A_GC2155M, // 2M
	ABILITY_SS2BF220,  // 2M
	AZWAVE_AM_2F035,  // 2M
	CHICONY_CIFD01820003870LH,  // 0.3M
	SUNWIN_SW69572176CB_VB,  // 0.3M
};

// *** Camera SKU ID:
// Camera SKU ID depends on the combination of Camera ID in each project, we do not define the name.

// eMMC ID: return values of FUN_EMMC_ID
enum {
	HYNIX_H26M52103FMR,  //TF103CE_HYNIX_16G
	KSI_MMC16G_S100_A08,  //TF103CE_KINGSTON_16G
};

// SKU ID: return values of FUN_EMMC_ID
enum {
	CHANNEL,  // OPEN CHANNEL SKU
	EDU,  // GOOGLE EDU SKU
};

#define CAMERA_SKU_ID(rcamid, fcamid) \
	((unsigned short)(0xFFFF & rcamid) << 16 | (unsigned short)(0xFFFF & fcamid))

#endif
