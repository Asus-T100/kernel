#ifndef __GP_DEVICE_GLOBAL_H_INCLUDED__
#define __GP_DEVICE_GLOBAL_H_INCLUDED__

#define IS_GP_DEVICE_VERSION_1

/* The SP configures FIFO switches in these registers */
#define _REG_GP_SWITCH_IF_ADDR				0x00
#define _REG_GP_SWITCH_DMA_ADDR				0x04
#define _REG_GP_SWITCH_GDC_ADDR				0x08

/* The SP sends SW interrupt info to this register */
#define _REG_GP_IRQ_REQUEST_ADDR			0x98

#endif /* __GP_DEVICE_GLOBAL_H_INCLUDED__ */
