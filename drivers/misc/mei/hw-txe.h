/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
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
 */

#ifndef _MEI_HW_TXE_H_
#define _MEI_HW_TXE_H_

#include "hw.h"
#include "hw-txe-regs.h"

extern bool nopg;



/* Flatten Hierarchy interrupt cuase */
#define TXE_INTR_READINESS_BIT  0 /* HISR_INT_0_STS */
#define TXE_INTR_READINESS      HISR_INT_0_STS
#define TXE_INTR_ALIVENESS_BIT  1 /* HISR_INT_1_STS */
#define TXE_INTR_ALIVENESS      HISR_INT_1_STS
#define TXE_INTR_OUT_DB_BIT     2 /* HISR_INT_2_STS */
#define TXE_INTR_OUT_DB         HISR_INT_2_STS
#define TXE_INTR_IN_READY_BIT   8 /* beyond HISR */
#define TXE_INTR_IN_READY       BIT(8)

struct mei_txe_hw {
	void __iomem *mem_addr[NUM_OF_MEM_BARS];
	u32 aliveness;
	unsigned long aliveness_atime;
	unsigned long aliveness_timeout;
	bool recvd_aliv_resp;
	wait_queue_head_t wait_aliveness_resp;
	struct delayed_work aliveness_timer;

	u32 readiness_state;
	unsigned long intr_cause;
};

#define to_txe_hw(dev) (struct mei_txe_hw *)((dev)->hw)

static inline struct mei_device *hw_txe_to_mei(struct mei_txe_hw *hw)
{
	return container_of((void *)hw, struct mei_device, hw);
}

struct mei_device *mei_txe_dev_init(struct pci_dev *pdev);

irqreturn_t mei_txe_irq_quick_handler(int irq, void *dev_id);
irqreturn_t mei_txe_irq_thread_handler(int irq, void *dev_id);

#endif /* _MEI_HW_TXE_H_ */

