#ifndef __PROGRAM_LOAD_H_INCLUDED__
#define __PROGRAM_LOAD_H_INCLUDED__
#ifndef __KERNEL__
#include <stdbool.h>
#endif

extern bool program_load(
	const cell_id_t			ID,
	const firmware_h		firmware);

#endif /* __PROGRAM_LOAD_H_INCLUDED__ */
