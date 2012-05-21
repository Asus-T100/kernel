/*!****************************************************************************
@File			emu.h
@Title			Emulator PCI header
@Author			Copyright (C) Imagination Technologies Limited.
				All rights reserved. Strictly Confidential.
@Description	Defines emulator PCI registers
******************************************************************************/

/* Atlas reg on base register 0 */
#define EMULATOR_ATLAS_REG_PCI_BASENUM			0
#define EMULATOR_ATLAS_REG_PCI_OFFSET			(EMULATOR_ATLAS_REG_PCI_BASENUM + PCI_BASEREG_OFFSET_DWORDS)

#define EMULATOR_ATLAS_REG_OFFSET				0x0
#define EMULATOR_ATLAS_REG_SIZE					0x10000

/* RGX reg on base register 1 */
#define EMULATOR_RGX_REG_PCI_BASENUM			1
#define EMULATOR_RGX_REG_PCI_OFFSET				(EMULATOR_RGX_REG_PCI_BASENUM + PCI_BASEREG_OFFSET_DWORDS)

#define EMULATOR_RGX_REG_OFFSET					0x0
#define EMULATOR_RGX_REG_SIZE					0x20000

#define EMULATOR_RGX_REG_WRAPPER_OFFSET			0x20000
#define EMULATOR_RGX_REG_WRAPPER_SIZE			0x4000

/* RGX mem (including HP mapping) on base register 2 */
#define EMULATOR_RGX_MEM_PCI_BASENUM			2
#define EMULATOR_RGX_MEM_PCI_OFFSET				(EMULATOR_RGX_MEM_PCI_BASENUM + PCI_BASEREG_OFFSET_DWORDS)

#define EMULATOR_RGX_MEM_REGION_SIZE			0x20000000

#define EMULATOR_LOCALMEM_FOR_RGX_RESERVE_SIZE	(224*1024*1024)
