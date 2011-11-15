/*
 * Support for Medfield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef _SP_MAP_H_
#define _SP_MAP_H_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) \
	_hrt_cell_load_program_from_elf_file(proc, "sp")

#define HIVE_MEM_isp_vectors_per_input_line  scalar_processor_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0xE7C
#define HIVE_SIZE_isp_vectors_per_input_line 4
#define HIVE_MEM_sp_raw_copy_thread  scalar_processor_dmem
#define HIVE_ADDR_sp_raw_copy_thread 0x4C
#define HIVE_SIZE_sp_raw_copy_thread 4
#define HIVE_MEM_sp_input_v_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_input_v_addr 0xE80
#define HIVE_SIZE_sp_input_v_addr 4
#define HIVE_MEM_dma_proxy_status  scalar_processor_dmem
#define HIVE_ADDR_dma_proxy_status 0x70
#define HIVE_SIZE_dma_proxy_status 4
#define HIVE_MEM_sp_error  scalar_processor_dmem
#define HIVE_ADDR_sp_error 0x282C
#define HIVE_SIZE_sp_error 4
#define HIVE_MEM_sp_init_dmem_data  scalar_processor_dmem
#define HIVE_ADDR_sp_init_dmem_data 0xE84
#define HIVE_SIZE_sp_init_dmem_data 24
#define HIVE_MEM_sp_per_frame_data  scalar_processor_dmem
#define HIVE_ADDR_sp_per_frame_data 0xE9C
#define HIVE_SIZE_sp_per_frame_data 20
#define HIVE_MEM_xmem_bin_addr  scalar_processor_dmem
#define HIVE_ADDR_xmem_bin_addr 0xEB0
#define HIVE_SIZE_xmem_bin_addr 4
#define HIVE_MEM_sp_uds_config  scalar_processor_dmem
#define HIVE_ADDR_sp_uds_config 0x281C
#define HIVE_SIZE_sp_uds_config 16
#define HIVE_MEM_sp_isp_started  scalar_processor_dmem
#define HIVE_ADDR_sp_isp_started 0xEB4
#define HIVE_SIZE_sp_isp_started 4
#define HIVE_MEM_sp_obarea_start_bq  scalar_processor_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0xEB8
#define HIVE_SIZE_sp_obarea_start_bq 4
#define HIVE_MEM_sp_start_isp_thread  scalar_processor_dmem
#define HIVE_ADDR_sp_start_isp_thread 0x12C
#define HIVE_SIZE_sp_start_isp_thread 4
#define HIVE_MEM_sp_proxy_thread  scalar_processor_dmem
#define HIVE_ADDR_sp_proxy_thread 0x78
#define HIVE_SIZE_sp_proxy_thread 4
#define HIVE_MEM_sp_bin_copy_bytes_copied  scalar_processor_dmem
#define HIVE_ADDR_sp_bin_copy_bytes_copied 0x280C
#define HIVE_SIZE_sp_bin_copy_bytes_copied 4
#define HIVE_MEM_isp_sh_dma_cmd_buffer  scalar_processor_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x2830
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#define HIVE_MEM_vf_pp_args  scalar_processor_dmem
#define HIVE_ADDR_vf_pp_args 0x2948
#define HIVE_SIZE_vf_pp_args 76
#define HIVE_ADDR_vf_pp_dynamic_entry 0x1732
#define HIVE_MEM_current_thread  scalar_processor_dmem
#define HIVE_ADDR_current_thread 0x2814
#define HIVE_SIZE_current_thread 4
#define HIVE_MEM_sp_dma_output_block_width_b  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_output_block_width_b 0x2874
#define HIVE_SIZE_sp_dma_output_block_width_b 4
#define HIVE_ADDR_sp_raw_copy_entry 0x4BB
#define HIVE_MEM_sp_obarea_length_bq  scalar_processor_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0xEBC
#define HIVE_SIZE_sp_obarea_length_bq 4
#define HIVE_MEM_capture_pp_args  scalar_processor_dmem
#define HIVE_ADDR_capture_pp_args 0x2994
#define HIVE_SIZE_capture_pp_args 12
#define HIVE_MEM_sp_dma_crop_block_width_b  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x2878
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#define HIVE_MEM_sp_output_v_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_output_v_addr 0xEC0
#define HIVE_SIZE_sp_output_v_addr 4
#define HIVE_ADDR_sp_start_isp_entry 0x0
#define HIVE_MEM_sp_current_isp_program  scalar_processor_dmem
#define HIVE_ADDR_sp_current_isp_program 0xE78
#define HIVE_SIZE_sp_current_isp_program 4
#define HIVE_MEM_sp_dma_crop_block_width_a  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x287C
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#define HIVE_MEM_sp_dma_c_block_width_b  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_c_block_width_b 0x2880
#define HIVE_SIZE_sp_dma_c_block_width_b 4
#define HIVE_MEM_isp_vectors_per_line  scalar_processor_dmem
#define HIVE_ADDR_isp_vectors_per_line 0xEC4
#define HIVE_SIZE_isp_vectors_per_line 4
#define HIVE_ADDR_sp_gen_histogram_entry 0x35F
#define HIVE_MEM_sp_group  scalar_processor_dmem
#define HIVE_ADDR_sp_group 0xEC8
#define HIVE_SIZE_sp_group 1424
#define HIVE_MEM_isp_uv_internal_width_vecs  scalar_processor_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x1458
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#define HIVE_MEM_sp_input_y_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_input_y_addr 0x145C
#define HIVE_SIZE_sp_input_y_addr 4
#define HIVE_MEM_sp_sw_interrupt_value  scalar_processor_dmem
#define HIVE_ADDR_sp_sw_interrupt_value 0x1460
#define HIVE_SIZE_sp_sw_interrupt_value 4
#define HIVE_MEM_histogram_args  scalar_processor_dmem
#define HIVE_ADDR_histogram_args 0x2810
#define HIVE_SIZE_histogram_args 4
#define HIVE_ADDR_capture_pp_dynamic_entry 0x3311
#define HIVE_MEM_isp_sdis_horiproj_num  scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x1464
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#define HIVE_MEM_sp_dma_output_block_width_a  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_output_block_width_a 0x2884
#define HIVE_SIZE_sp_dma_output_block_width_a 4
#define HIVE_MEM_num_sp_threads  scalar_processor_dmem
#define HIVE_ADDR_num_sp_threads 0x2818
#define HIVE_SIZE_num_sp_threads 4
#define HIVE_MEM_sp_output_y_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_output_y_addr 0x1468
#define HIVE_SIZE_sp_output_y_addr 4
#define HIVE_MEM_bayer_conf  scalar_processor_dmem
#define HIVE_ADDR_bayer_conf 0x130
#define HIVE_SIZE_bayer_conf 56
#define HIVE_MEM_isp_sdis_vertproj_num  scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x146C
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#define HIVE_MEM_sp_dma_c_block_width_a  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_c_block_width_a 0x2888
#define HIVE_SIZE_sp_dma_c_block_width_a 4
#define HIVE_MEM_sp_dma_crop_cropping_a  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x288C
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#define HIVE_MEM_sp_dma_output_skip_vecs  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_output_skip_vecs 0x2890
#define HIVE_SIZE_sp_dma_output_skip_vecs 4
#define HIVE_MEM_sp_vf_downscale_bits  scalar_processor_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x1470
#define HIVE_SIZE_sp_vf_downscale_bits 4
#define HIVE_MEM_isp_sdis_vertcoef_vectors  scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x1474
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#define HIVE_MEM_sp_output_u_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_output_u_addr 0x1478
#define HIVE_SIZE_sp_output_u_addr 4
#define HIVE_MEM_output_dma_info_descr  scalar_processor_dmem
#define HIVE_ADDR_output_dma_info_descr 0x2900
#define HIVE_SIZE_output_dma_info_descr 72
#define HIVE_MEM_sp_isp_input_stream_format  scalar_processor_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x147C
#define HIVE_SIZE_sp_isp_input_stream_format 4
#define HIVE_MEM_curr_binary_id  scalar_processor_dmem
#define HIVE_ADDR_curr_binary_id 0x0
#define HIVE_SIZE_curr_binary_id 4
#define HIVE_MEM_sp_dma_c_skip_vecs  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_c_skip_vecs 0x2894
#define HIVE_SIZE_sp_dma_c_skip_vecs 4
#define HIVE_MEM_sp_dma_vfout_cropping_a  scalar_processor_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x2898
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#define HIVE_MEM_sp_input_u_addr  scalar_processor_dmem
#define HIVE_ADDR_sp_input_u_addr 0x1480
#define HIVE_SIZE_sp_input_u_addr 4
#define HIVE_MEM_sp_debug  scalar_processor_dmem
#define HIVE_ADDR_sp_debug 0x2834
#define HIVE_SIZE_sp_debug 64
#define HIVE_MEM_isp_sdis_horicoef_vectors  scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x1484
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#define HIVE_MEM_sp_data  scalar_processor_dmem
#define HIVE_ADDR_sp_data 0x1488
#define HIVE_SIZE_sp_data 4864
#define HIVE_MEM_dma_proxy_kill_req  scalar_processor_dmem
#define HIVE_ADDR_dma_proxy_kill_req 0x4BC
#define HIVE_SIZE_dma_proxy_kill_req 1
#define HIVE_MEM_mem_map  scalar_processor_dmem
#define HIVE_ADDR_mem_map 0x289C
#define HIVE_SIZE_mem_map 100
#define HIVE_MEM_sp_proxy_running  scalar_processor_dmem
#define HIVE_ADDR_sp_proxy_running 0x4C0
#define HIVE_SIZE_sp_proxy_running 4
#define HIVE_MEM_vtmp1  scalar_processor_dmem
#define HIVE_ADDR_vtmp1 0x2788
#define HIVE_SIZE_vtmp1 128
#define HIVE_MEM_isp_vf_output_width_vecs  scalar_processor_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x2808
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#define HIVE_ADDR_sp_bin_copy_entry 0x2DD

#endif /* _SP_MAP_H_ */
