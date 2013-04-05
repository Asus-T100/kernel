/******************************************************************************
 * Intel Management Engine Interface (Intel TXEI) Linux driver
 * Intel TXEI Interface Header
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2003 - 2012 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *	Intel Corporation.
 *	linux-txei@linux.intel.com
 *	http://www.intel.com
 *
 * BSD LICENSE
 *
 * Copyright(c) 2003 - 2012 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifndef _TXEI_TXE_REGS_H_
#define _TXEI_TXE_REGS_H_

#include "hw.h"

#ifdef IPC_DEBUG
/* TODO: Check if spec specify any timeout for aliveness timer mechanism */
#define SEC_ALIVENESS_TIMER_TIMEOUT        (6 * MSEC_PER_SEC)
#define SEC_ALIVENESS_WAIT_TIMEOUT         (6 * MSEC_PER_SEC)
#define RESET_CANCEL_WAIT_TIMEOUT          (1 * MSEC_PER_SEC)

#if IPC_EMULATION
#define SEC_RESET_WAIT_TIMEOUT             (2 * MSEC_PER_SEC)
#define SEC_READY_WAIT_TIMEOUT             (2 * MSEC_PER_SEC)
#define START_MESSAGE_RESPONSE_WAIT_TIMEOUT (2 * MSEC_PER_SEC)
#else /* IPC_EMULATION */
#define SEC_RESET_WAIT_TIMEOUT             (8 * MSEC_PER_SEC)
#define SEC_READY_WAIT_TIMEOUT             (6 * MSEC_PER_SEC)
#define START_MESSAGE_RESPONSE_WAIT_TIMEOUT (6 * MSEC_PER_SEC)
#endif /* IPC_EMULATION */
#else /* IPC_DEBUG */
/* TODO: Check if spec specify any timeout for aliveness timer mechanism */
#define SEC_ALIVENESS_TIMER_TIMEOUT        (5 * MSEC_PER_SEC)
#define SEC_ALIVENESS_WAIT_TIMEOUT         (1 * MSEC_PER_SEC)
#define SEC_RESET_WAIT_TIMEOUT             (1 * MSEC_PER_SEC0)
#define SEC_READY_WAIT_TIMEOUT             (5 * MSEC_PER_SEC)
#define START_MESSAGE_RESPONSE_WAIT_TIMEOUT (5 * MSEC_PER_SEC)
#define RESET_CANCEL_WAIT_TIMEOUT          (1 * MSEC_PER_SEC)
#endif /* IPC_DEBUG */

enum {
#ifdef FPGA
	FPGA_BAR,
#endif
	SEC_BAR,
	BRIDGE_BAR,

	NUM_OF_MEM_BARS
};

/*
 * IPC device IDs
 */
#define VLV_IPC_FPGA_DEV_ID                     0x6024  /*  */
#define VLV_IPC_DEV_ID                          0x0F18  /*  */

/*
 * TXEI HW Section
 */

/* TXEI registers */

#define IPC_BASE_ADDR                           0x80400
/* SeC IPC Base Address */

#define SEC_IPC_INPUT_DOORBELL_OFFSET           (0x0000 + IPC_BASE_ADDR)

/**
 * Security Controller IPC Input Doorbell Register
 * Moved to Bridge == > #define SEC_IPC_OUTPUT_DOORBELL_OFFSET 0x0004
 * + IPC_BASE_ADDR // Security Controller IPC Output Doorbell Register
 */


#define SEC_IPC_INPUT_STATUS_OFFSET             (0x0008 + IPC_BASE_ADDR)
/**
 * Security Controller IPC Input Status Register
 * Moved to Bridge == > #define SEC_IPC_OUTPUT_STATUS_OFFSET 0x000C
 * + IPC_BASE_ADD // Security Controller IPC Output Status Register
 */

#define SEC_IPC_HOST_INT_STATUS_OFFSET          (0x0010 + IPC_BASE_ADDR)
/* Security Controller IPC Host Interrupt Status Register */

#define SEC_IPC_HOST_INT_MASK_OFFSET            (0x0014 + IPC_BASE_ADDR)
/* Security Controller IPC Host Interrupt Mask Register */

#define SEC_IPC_INPUT_PAYLOAD_OFFSET            (0x0100 + IPC_BASE_ADDR)
/**
 * Security Controller IPC Input Payload RAM
 * Moved to Bridge == > #define SEC_IPC_OUTPUT_PAYLOAD_OFFSET 0x0180
 * + IPC_BASE_ADDR // Security Controller IPC Output Payload RAM
 */

#define IPC_SHARED_PAYLOAD_OFFSET               (0x0200 + IPC_BASE_ADDR)
/* Security Controller IPC Shared Payload RAM */

#define SATT2_CTRL_OFFSET                       0x1040
/* SeC Address Translation Table Entry 2 - Ctrl */

#define SATT2_SAP_BA_OFFSET                     0x1044
/* SATT Table Entry 2 SAP Base Address Register */

#define SATT2_SAP_SIZE_OFFSET                   0x1048
/* SATT Table Entry 2 SAP Size Register. */

#define SATT2_BRG_BA_LSB_OFFSET                 0x104C
/* SATT Table Entry 2 SAP Bridge Address - LSB Register */

#define HHISR_OFFSET                            0x2020
/* Host High-level Interrupt Status Register */

#define HHIER_OFFSET                            0x2024
/* Host High-level Interrupt Enable Register */

#define HHIMR_OFFSET                            0x2028
/* Host High-level Interrupt Mask Register */

#define HHIRQSR_OFFSET                          0x202C
/* Host High-level IRQ Status Register */

#define HICR_SEC_IPC_READINESS_OFFSET           0x2040
/* Host Interrupt Cause Register 0 - SeC IPC Readiness */

#define HICR_HOST_ALIVENESS_RESP_OFFSET         0x2044
/* Host Interrupt Cause Register 1 - Aliveness Response */

#define HICR_SEC_IPC_OUTPUT_DOORBELL_OFFSET     0x2048
/* Host Interrupt Cause Register 2 - SeC IPC Output Doorbell */

#define HISR_OFFSET                             0x2060
/* Host Interrupt Status Register */

#define HIER_OFFSET                             0x2064
/* Host Interrupt Enable Register */

#define BRIDGE_IPC_OUTPUT_PAYLOAD_OFFSET        0x20C0
/** SEC Memory Space IPC output payload.
 * This register is part of the output
 * payload which SEC provides to host.
 */

#define SICR_HOST_ALIVENESS_REQ_OFFSET          0x214C
/* SeC Interrupt Cause Register - Host Aliveness Request */

#define SICR_HOST_IPC_READINESS_REQ_OFFSET      0x2150
/* SeC Interrupt Cause Register - Host IPC Readiness */

#define SICR_SEC_IPC_OUTPUT_STATUS_OFFSET       0x2154
/* SeC Interrupt Cause Register - SeC IPC Output Status */

/* SEC_IPC_HOST_INT_STATUS register bits */
#define TXEI_SEC_IPC_HOST_INT_STATUS_OU_DB              BIT(0)
#define TXEI_SEC_IPC_HOST_INT_STATUS_IN_RDY             BIT(1)
#define TXEI_SEC_IPC_HOST_INT_STATUS_HDCP_M0_RCVD       BIT(5)
#define TXEI_SEC_IPC_HOST_INT_STATUS_ILL_MEM_ACCESS     BIT(17)
#define TXEI_SEC_IPC_HOST_INT_STATUS_AES_HKEY_ERR       BIT(18)
#define TXEI_SEC_IPC_HOST_INT_STATUS_DES_HKEY_ERR       BIT(19)
#define TXEI_SEC_IPC_HOST_INT_STATUS_TMRMTB_OVERFLOW    BIT(21)

/* Convenient mask for pending interrupts */
#define TXEI_SEC_IPC_HOST_INT_STATUS_PENDING \
		(TXEI_SEC_IPC_HOST_INT_STATUS_OU_DB| \
		TXEI_SEC_IPC_HOST_INT_STATUS_IN_RDY)

/*
 * SeC FW uses this register in order to report its status to host (e.g BIOS).
 * This register resdies in PCI-E config space.
 */
#define SEC_IPC_HOST_INT_MASK_OUT_DB	BIT(0) /* IPCOuputDoorbellIntMask */
#define SEC_IPC_HOST_INT_MASK_IN_RDY	BIT(1) /* IPCInputReadyIntMask */

/**
 * SATT Table Entry 2 Control Register.
 * This register resides also in SeC's PCI-E Memory space.
 */
#define SATT2_CTRL_VALID_MSK                   BIT(0)
#define SATT2_CTRL_BRIDGE_BASE_ADDR_OFFSET     8
#define SATT2_CTRL_BRIDGE_HOST_EN_MSK          BIT(12)

/**
 * Host High-level Interrupt Status Register. Resides in PCI memory space.
 * This is the top hierarchy for interrupts
 * from SeC to host, aggregating both interrupts
 * that arrive through HICR registers as well as interrupts that arrive via IPC.
 */

#define TXEI_IPC_HHIER_SEC	BIT(0)
#define TXEI_IPC_HHIER_BRIDGE	BIT(1)
#define TXEI_IPC_HHIER_MSK	(TXEI_IPC_HHIER_SEC | TXEI_IPC_HHIER_BRIDGE)

/**
 * Host High-level Interrupt Mask Register. Resides in PCI memory space.
 * This is the top hierarchy for masking interrupts from SeC to host.
 */
#define TXEI_IPC_HHIMR_SEC       BIT(0)
#define TXEI_IPC_HHIMR_BRIDGE    BIT(1)

/**
 * This register is both an ICR to Host from PCI Memory Space
 * and it is also exposed in the SeC memory space.
 * This register is used by SeC's IPC driver uses in order
 * to synchornize with host about IPC interface state.
 */
#define TXEI_HICR_SEC_IPC_READINESS_HOST_RDY  BIT(0)
#define TXEI_HICR_SEC_IPC_READINESS_SEC_RDY   BIT(1)
#define TXEI_HICR_SEC_IPC_READINESS_SYS_RDY     \
	(TXEI_HICR_SEC_IPC_READINESS_HOST_RDY | \
	 TXEI_HICR_SEC_IPC_READINESS_SEC_RDY)
#define TXEI_HICR_SEC_IPC_READINESS_RDY_CLR   BIT(2)

/**
 * This register is both an ICR to Host from PCI Memory Space
 * and it is also exposed in the SeC memory space.
 * The register may be used by SeC to ACK a host request for aliveness.
 */
#define TXEI_HICR_HOST_ALIVENESS_RESP_ACK    BIT(0)

/**
 * Host Interrupt Status Register. Resides in PCI memory space.
 * This is the main register involved in
 * generating interrupts from SeC to host via HICRs.
 * The interrupt generation rules are as follows:
 * An interrupt will be generated whenever for any i,
 * there is a transition from a state where at least one of
 * the following conditions did not hold,
 * to a state where ALL the following conditions hold:
 * A) HISR.INT[i]_STS == 1.
 * B) HIER.INT[i]_EN == 1.
 */
#define TXEI_HISR_INT_0_STS      BIT(0)
#define TXEI_HISR_INT_1_STS      BIT(1)
#define TXEI_HISR_INT_2_STS      BIT(2)
#define TXEI_HISR_INT_3_STS      BIT(3)
#define TXEI_HISR_INT_4_STS      BIT(4)
#define TXEI_HISR_INT_5_STS      BIT(5)
#define TXEI_HISR_INT_6_STS      BIT(6)
#define TXEI_HISR_INT_7_STS      BIT(7)
#define TXEI_HISR_INT_STS_MSK \
	(TXEI_HISR_INT_0_STS | TXEI_HISR_INT_1_STS | TXEI_HISR_INT_2_STS)

/* Host Interrupt Enable Register. Resides in PCI memory space. */

#define TXEI_HIER_INT_0_EN      BIT(0)
#define TXEI_HIER_INT_1_EN      BIT(1)
#define TXEI_HIER_INT_2_EN      BIT(2)
#define TXEI_HIER_INT_3_EN      BIT(3)
#define TXEI_HIER_INT_4_EN      BIT(4)
#define TXEI_HIER_INT_5_EN      BIT(5)
#define TXEI_HIER_INT_6_EN      BIT(6)
#define TXEI_HIER_INT_7_EN      BIT(7)

#define TXEI_HIER_INT_EN_MSK \
	(TXEI_HIER_INT_0_EN | TXEI_HIER_INT_1_EN | TXEI_HIER_INT_2_EN)

/**
 * This register is both an ICR to SeC andit is also exposed
 * in the host-visible PCI memory space.
 * The registe is used by host to request SeC aliveness.
 */
#define TXEI_SICR_HOST_ALIVENESS_REQ_REQUESTED        BIT(0)

/**
 * This register is both an ICR to SeC and it is also exposed
 * in the host-visible PCI memory space.
 * This register is used by the host's SeC driver uses in order
 * to synchornize with SeC about IPC interface state.
 */

#define TXEI_SICR_HOST_IPC_READINESS_HOST_RDY  BIT(0)
#define TXEI_SICR_HOST_IPC_READINESS_SEC_RDY   BIT(1)
#define TXEI_SICR_HOST_IPC_READINESS_SYS_RDY     \
	(TXEI_SICR_HOST_IPC_READINESS_HOST_RDY | \
	 TXEI_SICR_HOST_IPC_READINESS_SEC_RDY)
#define TXEI_SICR_HOST_IPC_READINESS_RDY_CLR   BIT(2)

/**
 * This register indicates whether or not processing of the most recent
 * command has been completed by the Host.
 * New commands and payloads should not be written by SeC until this
 * register indicates that the previous command has been processed.
 */
#define SEC_IPC_OUTPUT_STATUS_RDY BIT(0)

/**
 * This register indicates whether or not processing of
 * the most recent command has been completed by the SEC RISC
 * New commands and payloads should not be written by the Host
 * until this indicates that the previous command has been processed.
 */
#define SEC_IPC_INPUT_STATUS_RDY	BIT(0)

#define PAYLOAD_SIZE            64
/*  TXEI IPC Message payload size 64 bytes */

#endif /* _TXEI_TXE_REGS_H_ */
