#ifndef __IRQ_PUBLIC_H_INCLUDED__
#define __IRQ_PUBLIC_H_INCLUDED__

#include "system_types.h"

#include "hive_isp_css_irq_types_hrt.h"	/* enum	hrt_isp_css_irq */
typedef struct irq_controller_state_s		irq_controller_state_t;

/*! Read the control registers of IRQ[ID]

 \param	ID[in]				IRQ identifier
 \param	state[out]			irq controller state structure

 \return none, state = IRQ[ID].state
 */
extern void irq_controller_get_state(
	const irq_ID_t				ID,
	irq_controller_state_t		*state);

/*! Write to a control register of IRQ[ID]

 \param	ID[in]				IRQ identifier
 \param	reg[in]				register index
 \param value[in]			The data to be written

 \return none, IRQ[ID].ctrl[reg] = value
 */
STORAGE_CLASS_IRQ_H void irq_reg_store(
	const irq_ID_t		ID,
	const unsigned int	reg,
	const hrt_data		value);

/*! Read from a control register of IRQ[ID]

 \param	ID[in]				IRQ identifier
 \param	reg[in]				register index
 \param value[in]			The data to be written

 \return IRQ[ID].ctrl[reg]
 */
STORAGE_CLASS_IRQ_H hrt_data irq_reg_load(
	const irq_ID_t		ID,
	const unsigned int	reg);

/*! Enable an IRQ channel of IRQ[ID] with a mode

 \param	ID[in]				IRQ (device) identifier
 \param	irq[in]				IRQ (channel) identifier

 \return none, enable(IRQ[ID].channel[irq_ID])
 */
extern void irq_enable_channel(
	const irq_ID_t				ID,
	const enum hrt_isp_css_irq	irq_ID);

/*! Disable an IRQ channel of IRQ[ID]

 \param	ID[in]				IRQ (device) identifier
 \param	irq[in]				IRQ (channel) identifier

 \return none, disable(IRQ[ID].channel[irq_ID])
 */
extern void irq_disable_channel(
	const irq_ID_t				ID,
	enum hrt_isp_css_irq		irq);

/*! Clear the state of all IRQ channels of IRQ[ID]

 \param	ID[in]				IRQ (device) identifier

 \return none, clear(IRQ[ID].channel[])
 */
extern void irq_clear_all(
		const irq_ID_t			ID);

/*! Return the ID of a signalling IRQ channel of IRQ[ID]

 \param	ID[in]				IRQ (device) identifier
 \param irq[out]			active IRQ (channel) identifier

 \Note: This function operates as strtok(), based on the return
  state the user is informed if there are additional signalling
  channels

 \return state(IRQ[ID])
 */
extern enum hrt_isp_css_irq_status irq_get_channel_id(
	const irq_ID_t				ID,
	enum hrt_isp_css_irq		*irq);

/*! Raise an interrupt on channel irq_id of device IRQ[ID]

 \param	ID[in]				IRQ (device) identifier
 \param	irq_id[in]			IRQ (channel) identifier

 \return none, signal(IRQ[ID].channel[irq_id])
 */
extern void _irq_raise(
	const irq_ID_t				ID,
	const irq_sw_channel_id_t	irq_id);

#endif /* __IRQ_PUBLIC_H_INCLUDED__ */
