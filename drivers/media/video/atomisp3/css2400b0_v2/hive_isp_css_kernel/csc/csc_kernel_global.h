#ifndef __CSC_KERNEL_GLOBAL_H_INCLUDED__
#define __CSC_KERNEL_GLOBAL_H_INCLUDED__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/* Can we pass a pointer (that can be resolved to a constant) without ill effects */
//typedef const struct csc_kernel_param_s	*csc_kernel_param_h;
/* Else, can we pass a register struct (defined in csc_kernel_local.h) */

/* Else, can we only pass an ID / index ? */
typedef csc_kernel_param_set_t			csc_kernel_param_h;

typedef struct csc_kernel_param_s		csc_kernel_param_t;

struct csc_kernel_param_s {
	uint16_t	m_shift;
	int16_t		m00;
	int16_t		m01;
	int16_t		m02;
	int16_t		m10;
	int16_t		m11;
	int16_t		m12;
	int16_t		m20;
	int16_t		m21;
	int16_t		m22;
};

#endif /* __CSC_KERNEL_GLOBAL_H_INCLUDED__ */
