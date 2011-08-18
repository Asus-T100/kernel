/*
 * intel_mid_pm.h
 * Copyright (c) 2010, Intel Corporation.
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
 */
#ifndef INTEL_MID_PM_H
#define INTEL_MID_PM_H

#include <asm/mrst.h>
#include <linux/init.h>
#include <linux/pci.h>

/* Register Type definitions */
#define OSPM_REG_TYPE          0x0
#define APM_REG_TYPE           0x1
#define OSPM_MAX_POWER_ISLANDS 16
#define OSPM_ISLAND_UP         0x0
#define OSPM_ISLAND_DOWN       0x1

/* North complex power islands definitions for APM block*/
#define APM_GRAPHICS_ISLAND    0x1
#define APM_VIDEO_DEC_ISLAND   0x2
#define APM_VIDEO_ENC_ISLAND   0x4
#define APM_GL3_CACHE_ISLAND   0x8
#define APM_ISP_ISLAND         0x10
#define APM_IPH_ISLAND         0x20

/* North complex power islands definitions for OSPM block*/
#define OSPM_DISPLAY_A_ISLAND  0x2
#define OSPM_DISPLAY_B_ISLAND  0x80
#define OSPM_DISPLAY_C_ISLAND  0x100
#define OSPM_MIPI_ISLAND       0x200

#define C4_HINT	(0x30)
#define C6_HINT	(0x52)

#ifdef CONFIG_X86_MDFLD

#define PMU1_MAX_PENWELL_DEVS   8
#define PMU2_MAX_PENWELL_DEVS   55
#define PMU1_MAX_MRST_DEVS   2
#define PMU2_MAX_MRST_DEVS   15
#define MAX_DEVICES	(PMU1_MAX_PENWELL_DEVS + PMU2_MAX_PENWELL_DEVS)
#define WAKE_CAPABLE	0x80000000
#define PMU_MAX_LSS_SHARE 4
#define AUTO_CLK_GATE_VALUE	0x555551
#define SUB_SYS_D0I2_VALUE	0xaaaaaa
#define WAKE_ENABLE_VALUE	0x4786
#define SUSPEND_GFX             0xc

/* Error codes for pmu */
#define	PMU_SUCCESS			0
#define PMU_FAILED			-1
#define PMU_BUSY_STATUS			0
#define PMU_MODE_ID			1
#define	SET_MODE			1
#define	SET_AOAC_S0i1			2
#define	SET_AOAC_S0i3			3
#define	SET_LPAUDIO			4
#define	SET_AOAC_S0i2			7

struct pci_dev_info {
	u8 ss_pos;
	u8 ss_idx;
	u8 pmu_num;

	u32 log_id;
	u32 cap;
	struct pci_dev *dev_driver[PMU_MAX_LSS_SHARE];
	pci_power_t dev_power_state[PMU_MAX_LSS_SHARE];
};

struct wk_data {
	u32 word0;
	u32 word1;
};

union wake_config {
	struct wk_data data;
	u64  long_word;
};

struct pmu_wake_ss_states {
	unsigned long wake_enable[2];
	unsigned long pmu1_wake_states;
	unsigned long pmu2_wake_states[4];
};

struct pmu_ss_states {
	unsigned long pmu1_states;
	unsigned long pmu2_states[4];
};

struct pmu_suspend_config {
	struct pmu_ss_states ss_state;
	struct pmu_wake_ss_states wake_state;
};

enum pmu_number {
	PMU_NUM_1,
	PMU_NUM_2,
	PMU_MAX_DEVS
};

enum pmu_ss_state {
	SS_STATE_D0I0 = 0,
	SS_STATE_D0I1 = 1,
	SS_STATE_D0I2 = 2,
	SS_STATE_D0I3 = 3
};

/* PMU event */
#define	PMU_SUBSYS_WAKE		0
#define PMU_CMD_SUCCESS		1
#define	PMU_CMD_ERROR		2
#define PMU_CMD_NO_C6_ERROR	3

#define EVENT_HANDLER_PATH	"/etc/pmu/pmu_event_handler"

#define C7_HINT	(0x200)
#define C8_HINT	(0x201)

#define MID_S0I1_STATE		1
#define MID_S0I3_STATE		3
#define MID_S0IX_STATE		4

extern int mfld_s0i1_enter(void);
extern void mfld_s0i3_enter(void);
extern int get_target_platform_state(void);
extern void pmu_enable_forward_msi(void);
extern unsigned long pmu_get_cstate(unsigned long eax);
extern int pmu_nc_set_power_state
	(int islands, int state_type, int reg_type);

#else

#define TEMP_DTS_ID     43

/*
 * If CONFIG_X86_MDFLD is not defined
 * fall back to C6
 */
#define C7_HINT	C6_HINT
#define C8_HINT	C6_HINT

static inline int pmu_nc_set_power_state
	(int islands, int state_type, int reg_type) { return 0; }

static unsigned long pmu_get_cstate(unsigned long eax) { return eax; }

#endif /* #ifdef CONFIG_X86_MDFLD */

#endif /* #ifndef INTEL_MID_PM_H */
