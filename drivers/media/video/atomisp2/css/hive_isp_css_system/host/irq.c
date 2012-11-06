#include "irq.h"
#define __INLINE_GP_DEVICE__
#include "gp_device.h"

/* MW: This is an HRT backend function from "thread" */
#include "platform_support.h"	/* hrt_sleep() */

/* STORAGE_CLASS_INLINE void irq_wait_for_write_complete( */
static void irq_wait_for_write_complete(
	const irq_ID_t		ID);

#ifndef __INLINE_IRQ__
#include "irq_private.h"
#endif /* __INLINE_IRQ__ */

void irq_clear_all(
	const irq_ID_t				ID)
{
/*
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX,
		~((~(hrt_data)0)>>hrt_isp_css_irq_num_irqs));
 */
assert(hrt_isp_css_irq_num_irqs == HRT_DATA_WIDTH);

	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX, (hrt_data)0xFFFFFFFF);
return;
}

/*
 * Do we want the user to be able to set the signalling method ?
 */
void irq_enable_channel(
	const irq_ID_t				ID,
    const enum hrt_isp_css_irq	irq_id)
{
	unsigned int mask = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX);
	unsigned int enable = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX);
	unsigned int edge_in = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_EDGE_REG_IDX);
	unsigned int edge_out = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_EDGE_NOT_PULSE_REG_IDX);
	unsigned int me = 1U << irq_id;

assert(irq_id < hrt_isp_css_irq_num_irqs);

	mask |= me;
	enable |= me;
	edge_in |= me;	/* rising edge */
/*	edge_out |= me;  use pulse, not edge */

/* to avoid mishaps configuration must follow the following order */

/* mask this interrupt */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX, mask & ~me);
/* rising edge at input */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_EDGE_REG_IDX, edge_in);
/* enable interrupt to output */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX, enable);
/* output is given as edge, not pulse */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_EDGE_NOT_PULSE_REG_IDX, edge_out);
/* clear current irq only */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX, me);
/* unmask interrupt from input */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX, mask);

	irq_wait_for_write_complete(ID);

return;
}

void irq_disable_channel(
	const irq_ID_t				ID,
	const enum hrt_isp_css_irq	irq_id)
{
	unsigned int mask = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX);
	unsigned int enable = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX);
	unsigned int me = 1U << irq_id;

assert(irq_id < hrt_isp_css_irq_num_irqs);

	mask &= ~me;
	enable &= ~me;

/* enable interrupt to output */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX, enable);
/* unmask interrupt from input */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX, mask);
/* clear current irq only */
	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX, me);

	irq_wait_for_write_complete(ID);

return;
}

enum hrt_isp_css_irq_status irq_get_channel_id(
	const irq_ID_t				ID,
	enum hrt_isp_css_irq		*irq_id)
{
	unsigned int irq_status = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_STATUS_REG_IDX);
	enum hrt_isp_css_irq idx = hrt_isp_css_irq_num_irqs;
	enum hrt_isp_css_irq_status status = hrt_isp_css_irq_status_success;

assert(irq_id != NULL);

/* find the first irq bit */
	for (idx = 0; idx < hrt_isp_css_irq_num_irqs; idx++) {
		if (irq_status & (1U << idx))
			break;
	}
	if (idx == hrt_isp_css_irq_num_irqs)
		return hrt_isp_css_irq_status_error;

/* now check whether there are more bits set */
	if (irq_status != (1U << idx))
		status = hrt_isp_css_irq_status_more_irqs;


	irq_reg_store(ID,
		_HRT_IRQ_CONTROLLER_CLEAR_REG_IDX, 1U << idx);

	irq_wait_for_write_complete(ID);

	if (irq_id)
		*irq_id = (enum hrt_isp_css_irq)idx;

return status;
}

void _irq_raise(
	const irq_ID_t				ID,
	const irq_sw_channel_id_t	irq_id)
{
assert(ID == IRQ0_ID);
assert(IRQ_BASE[ID] != (hrt_address)-1);
assert(irq_id < N_IRQ_SW_CHANNEL_ID);
	(void)ID;

/* The SW IRQ pins are remapped to offset zero */
	gp_device_reg_store(GP_DEVICE0_ID,
		_REG_GP_IRQ_REQUEST_ADDR,
		(1U<<(irq_id - hrt_isp_css_irq_sw_0)));
#ifdef HRT_CSIM
	hrt_sleep();
#endif
	gp_device_reg_store(GP_DEVICE0_ID,
		_REG_GP_IRQ_REQUEST_ADDR, 0);
return;
}

void irq_controller_get_state(
	const irq_ID_t				ID,
	irq_controller_state_t		*state)
{
assert(ID < N_IRQ_ID);
assert(state != NULL);

	state->irq_edge = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_EDGE_REG_IDX);
	state->irq_mask = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_MASK_REG_IDX);
	state->irq_status = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_STATUS_REG_IDX);
	state->irq_enable = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX);
	state->irq_level_not_pulse = irq_reg_load(ID,
		_HRT_IRQ_CONTROLLER_EDGE_NOT_PULSE_REG_IDX);
return;
}

/* STORAGE_CLASS_INLINE void irq_wait_for_write_complete( */
static void irq_wait_for_write_complete(
	const irq_ID_t		ID)
{
assert(ID < N_IRQ_ID);
assert(IRQ_BASE[ID] != (hrt_address)-1);
	(void)device_load_uint32(IRQ_BASE[ID] +
		_HRT_IRQ_CONTROLLER_ENABLE_REG_IDX*sizeof(hrt_data));
#ifdef HRT_CSIM
	hrt_sleep();
#endif
return;
}

