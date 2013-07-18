/*
 * intel_soc_pmc.h
 * Copyright (c) 2013, Intel Corporation.
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

#ifndef __INTEL_SOC_PMC_H
#define __INTEL_SOC_PMC_H

#define		MAX_PLATFORM_STATES 5
#define		S0I3	3

#define		S0IX_REGISTERS_OFFSET	0x80

#define		S0IR_TMR_OFFSET		0x80
#define		S0I1_TMR_OFFSET		0x84
#define		S0I2_TMR_OFFSET		0x88
#define		S0I3_TMR_OFFSET		0x8c
#define		S0_TMR_OFFSET		0x90

#define		S0IX_WAKE_EN		0x3c

#define		PMC_MMIO_BAR		1
#define		BASE_ADDRESS_MASK	0xFFFFFFFE00
#define		DISABLE_LPC_CLK_WAKE_EN 0xffffef



#define   PM_SUPPORT			0x21

#define ISP_POS				7
#define ISP_SUB_CLASS			0x80

#define PUNIT_PORT			0x04
#define PWRGT_CNT			0x60
#define PWRGT_STATUS			0x61
#define VED_SS_PM0			0x32
#define ISP_SS_PM0			0x39
#define MIO_SS_PM			0x3B
#define SSS_SHIFT			24
#define RENDER_POS			0
#define MEDIA_POS			2
#define DISPLAY_POS			6

/* Soft reset mask */
#define SR_MASK				0x2

#define BYT_S3_HINT			0x64

#define NC_PM_SSS			0x3F

#define GFX_LSS_INDEX			1

#define D0I0_MASK			0
#define D0I1_MASK			1
#define D0I2_MASK			2
#define D0I3_MASK			3

#define BITS_PER_LSS			2
#define PCI_ID_ANY			(~0)
#define SUB_CLASS_MASK			0xFF00

struct mid_pmc_dev {
	u32 base_address;
	u32 __iomem *pmc_registers;
	u32 __iomem *s0ix_wake_en;
	struct pci_dev const *pdev;
	struct semaphore nc_ready_lock;
	u32 s3_residency;
};

extern void cstate_ignore_add_init(void);
#endif
