#ifndef __ISP_LOCAL_H_INCLUDED__
#define __ISP_LOCAL_H_INCLUDED__

#include <stdbool.h>

#include "isp_global.h"

#define HIVE_ISP_VMEM_MASK	((1U<<ISP_VMEM_ELEMBITS)-1)

typedef struct isp_state_s		isp_state_t;
typedef struct isp_stall_s		isp_stall_t;

struct isp_state_s {
	int		pc;
	int		status_register;
	bool	is_broken;
	bool	is_idle;
	bool	is_sleeping;
	bool	is_stalling;
};

struct isp_stall_s {
	bool	fifo0;
	bool	fifo1;
	bool	fifo2;
	bool	fifo3;
	bool	fifo4;
	bool	fifo5;
	bool	fifo6;
	bool	stat_ctrl;
	bool	dmem;
	bool	vmem;
	bool	vamem1;
	bool	vamem2;
	bool	vamem3;
	bool	hmem;
	bool	pmem;
	bool	icache_master;
};

#endif /* __ISP_LOCAL_H_INCLUDED__ */
