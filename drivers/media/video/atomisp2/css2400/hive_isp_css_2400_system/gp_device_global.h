#ifndef __GP_DEVICE_GLOBAL_H_INCLUDED__
#define __GP_DEVICE_GLOBAL_H_INCLUDED__

#define IS_GP_DEVICE_VERSION_2

#define _REG_GP_IRQ_REQ0_ADDR				0x08
#define _REG_GP_IRQ_REQ1_ADDR				0x0C
/* The SP sends SW interrupt info to this register */
#define _REG_GP_IRQ_REQUEST0_ADDR			_REG_GP_IRQ_REQ0_ADDR
#define _REG_GP_IRQ_REQUEST1_ADDR			_REG_GP_IRQ_REQ1_ADDR

/* The SP configures FIFO switches in these registers */
#define _REG_GP_SWITCH_IF_ADDR						0x40
#define _REG_GP_SWITCH_GDC1_ADDR					0x44
#define _REG_GP_SWITCH_GDC2_ADDR					0x48

#endif /* __GP_DEVICE_GLOBAL_H_INCLUDED__ */
