#include "input_system.h"

#ifdef HAS_INPUT_SYSTEM_VERSION_2
#include "input_system_init.h"
#include <linux/string.h>

input_system_error_t ia_css_input_system_init(void)
{
	backend_channel_cfg_t	backend_ch0; // AM: Initialize!
	backend_channel_cfg_t	backend_ch1; // AM: Initialize!
	target_cfg2400_t 	targetB; // AM: Initialize!
	target_cfg2400_t 	targetC; // AM: Initialize!
	uint32_t		acq_mem_region_size = 24;
	uint32_t		acq_nof_mem_regions = 2;
	input_system_error_t error=INPUT_SYSTEM_ERR_NO_ERROR;

	memset(&backend_ch0,0,sizeof(backend_channel_cfg_t));
	memset(&backend_ch1,0,sizeof(backend_channel_cfg_t));
	memset(&targetB,0,sizeof(targetB));
	memset(&targetC,0,sizeof(targetC));

	error=(input_system_configuration_reset());
	if( error != INPUT_SYSTEM_ERR_NO_ERROR)
		return error;

	error=(input_system_csi_xmem_channel_cfg(
		0,						//ch_id 
		INPUT_SYSTEM_PORT_A,	//port
		backend_ch0,			//backend_ch
		32,						//mem_region_size
		6,						//nof_mem_regions
		acq_mem_region_size,	//acq_mem_region_size
		acq_nof_mem_regions,	//acq_nof_mem_regions
		targetB,				//target
		3));//nof_xmem_buffers
	if( error != INPUT_SYSTEM_ERR_NO_ERROR)
		return error;

	error=(input_system_csi_xmem_channel_cfg(
		1,						//ch_id 
		INPUT_SYSTEM_PORT_B,	//port
		backend_ch0,			//backend_ch
		16,						//mem_region_size
		3,						//nof_mem_regions
		acq_mem_region_size,	//acq_mem_region_size
		acq_nof_mem_regions,	//acq_nof_mem_regions
		targetB,				//target
		3));//nof_xmem_buffers
	if(error != INPUT_SYSTEM_ERR_NO_ERROR)
		return error;

	error=(input_system_csi_xmem_channel_cfg(
		2,						
		INPUT_SYSTEM_PORT_C,	//input_system_csi_port_t port
		backend_ch1,			//backend_channel_cfg_t	backend_ch
		32,						//uint32_t 				mem_region_size
		3,						//uint32_t 				nof_mem_regions
		acq_mem_region_size,	//uint32_t 				acq_mem_region_size
		acq_nof_mem_regions,	//uint32_t 				acq_nof_mem_regions
		targetC,				//target_cfg2400_t 		target
		2));						//uint32_t 				nof_xmem_buffers
	if(error != INPUT_SYSTEM_ERR_NO_ERROR)
		return error;

	error=(input_system_configuration_commit());
    return error;
}
#endif
