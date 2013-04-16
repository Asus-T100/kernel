#ifndef __HMEM_GLOBAL_H_INCLUDED__
#define __HMEM_GLOBAL_H_INCLUDED__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define IS_HMEM_VERSION_1

#include "isp.h"

/*
#define ISP_HIST_ADDRESS_BITS                  12
#define ISP_HIST_ALIGNMENT                     4
#define ISP_HIST_COMP_IN_PREC                  12
#define ISP_HIST_DEPTH                         1024
#define ISP_HIST_WIDTH                         24
#define ISP_HIST_COMPONENTS                    4
*/
#define ISP_HIST_ALIGNMENT_LOG2		2

#define HMEM_SIZE_LOG2		(ISP_HIST_ADDRESS_BITS-ISP_HIST_ALIGNMENT_LOG2)
#define HMEM_SIZE			ISP_HIST_DEPTH

#define HMEM_UNIT_SIZE		(HMEM_SIZE/ISP_HIST_COMPONENTS)
#define HMEM_UNIT_COUNT		ISP_HIST_COMPONENTS

#define HMEM_RANGE_LOG2		ISP_HIST_WIDTH
#define HMEM_RANGE			(1UL<<HMEM_RANGE_LOG2)

typedef uint32_t			hmem_data_t;

#endif /* __HMEM_GLOBAL_H_INCLUDED__ */
