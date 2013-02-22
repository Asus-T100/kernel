#ifndef __IRQ_LOCAL_H_INCLUDED__
#define __IRQ_LOCAL_H_INCLUDED__

#include "irq_global.h"

#include <irq_controller_defs.h>

struct irq_controller_state_s {
	unsigned int	irq_edge;
	unsigned int	irq_mask;
	unsigned int	irq_status;
	unsigned int	irq_enable;
	unsigned int	irq_level_not_pulse;
};

#endif /* __IRQ_LOCAL_H_INCLUDED__ */
