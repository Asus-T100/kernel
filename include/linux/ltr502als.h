#ifndef __LTR502ALS_H__
#define __LTR502ALS_H__

/* IOCTL */
#define INACTIVE_PS			0
#define ACTIVE_PS			1
#define ACTIVE_LS			4
#define INACTIVE_LS			3
#define READ_LS				5
#define READ_PS				6
/* #define POWEROFF_CHIP		7 */
/* #define POWERON_CHIP			8 */
#define INIT_CHIP			9
#define SET_PS_THRESHOLD		10
#define GET_PS_THRESHOLD		11
#define SET_PS_ACCURACY			12
#define SET_LS_LEVEL			13
#define SET_LS_LOWTHRESH		14
#define SET_LS_PERSIST			15
#define SET_PS_PERSIST			16
#define SET_INTEGRA_TIME		17
#define ACTIVE_IRQ			18
#define INACTIVE_IRQ			19
#define READ_LUX			20
#define GET_LS_LOWTHRESHOLD		21

#endif /* __LTR502ALS_H__ */

