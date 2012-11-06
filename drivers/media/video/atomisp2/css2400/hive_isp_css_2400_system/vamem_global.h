#ifndef __VAMEM_GLOBAL_H_INCLUDED__
#define __VAMEM_GLOBAL_H_INCLUDED__

#include <stdint.h>

#define IS_VAMEM_VERSION_2

/* (log) stepsize of linear interpolation */
#define VAMEM_INTERP_STEP_LOG2	4
#define VAMEM_INTERP_STEP		(1<<VAMEM_INTERP_STEP_LOG2)
/* (physical) size of the tables */
#define VAMEM_TABLE_UNIT_SIZE	((1<<(ISP_VAMEM_ADDRESS_BITS-VAMEM_INTERP_STEP_LOG2)) + 1)
/* (logical) size of the tables */
#define VAMEM_TABLE_UNIT_STEP	((VAMEM_TABLE_UNIT_SIZE-1)<<1)
/* Number of tables */
#define VAMEM_TABLE_UNIT_COUNT	(ISP_VAMEM_DEPTH/VAMEM_TABLE_UNIT_STEP)

typedef uint16_t				vamem_data_t;

#endif /* __VAMEM_GLOBAL_H_INCLUDED__ */
