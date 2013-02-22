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

#ifndef _sp_map_h_
#define _sp_map_h_

#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_input_line
#define HIVE_MEM_isp_vectors_per_input_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0x2A48
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#error Symbol isp_vectors_per_input_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2A48
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 3992 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MASK_UNIT
#define HIVE_MEM_MASK_UNIT scalar_processor_2400A0_dmem
#define HIVE_ADDR_MASK_UNIT 0x6B8
#define HIVE_SIZE_MASK_UNIT 4
#else
#error Symbol MASK_UNIT occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_MASK_UNIT scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MASK_UNIT 0x6B8
#define HIVE_SIZE_sp_MASK_UNIT 4

/* function longjmp: 41D1 */

/* function sp_tagger_tag_exp_id: 38F6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_MASK
#define HIVE_MEM_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_MASK 0x33C
#define HIVE_SIZE_HIVE_IF_SRST_MASK 12
#else
#error Symbol HIVE_IF_SRST_MASK occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_MASK 0x33C
#define HIVE_SIZE_sp_HIVE_IF_SRST_MASK 12

/* function sp_dma_proxy_read: 2FB9 */

/* function sp_dma_proxy_is_idle: 30C5 */

/* function debug_buffer_set_ddr_addr: 68 */

/* function token_map_register: 4114 */

/* function setjmp: 41DA */

/* function decode_sw_event: 5AB */

/* function sp_isys_tokhandler_init: 3FEB */

/* function initialize_sp_stage: 2B4F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stage
#define HIVE_MEM_isp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stage 0x37A8
#define HIVE_SIZE_isp_stage 512
#else
#error Symbol isp_stage occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stage 0x37A8
#define HIVE_SIZE_sp_isp_stage 512

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_raw
#define HIVE_MEM_vbuf_raw scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_raw 0xC0
#define HIVE_SIZE_vbuf_raw 4
#else
#error Symbol vbuf_raw occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_vbuf_raw scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_raw 0xC0
#define HIVE_SIZE_sp_vbuf_raw 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_proxy_status
#define HIVE_MEM_dma_proxy_status scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_proxy_status 0x1D4
#define HIVE_SIZE_dma_proxy_status 4
#else
#error Symbol dma_proxy_status occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x1D4
#define HIVE_SIZE_sp_dma_proxy_status 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x2CFC
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#error Symbol do_mmu_invalidation occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x2CFC
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 214E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_binary_group 0x2780
#define HIVE_SIZE_sp_binary_group 72
#else
#error Symbol sp_binary_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x2780
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sw_state 0x2A4C
#define HIVE_SIZE_sp_sw_state 4
#else
#error Symbol sp_sw_state occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2A4C
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 36B9 */

/* function sp_circular_buf_pop: 3684 */

/* function token_map_empty: 4086 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stop_copy_preview
#define HIVE_MEM_sp_stop_copy_preview scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stop_copy_preview 0x2CD8
#define HIVE_SIZE_sp_stop_copy_preview 4
#else
#error Symbol sp_stop_copy_preview occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_stop_copy_preview scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_stop_copy_preview 0x2CD8
#define HIVE_SIZE_sp_sp_stop_copy_preview 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_handles 0x2580
#define HIVE_SIZE_vbuf_handles 200
#else
#error Symbol vbuf_handles occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x2580
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3664
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#error Symbol cb_elems_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3664
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x2D10
#define HIVE_SIZE_sp_dma_proxy_thread 64
#else
#error Symbol sp_dma_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x2D10
#define HIVE_SIZE_sp_sp_dma_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x264
#define HIVE_SIZE_sp_thread_ready_queue 8
#else
#error Symbol sp_thread_ready_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x264
#define HIVE_SIZE_sp_sp_thread_ready_queue 8

/* function sp_debug_mode_update_command: 3DE1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_is_pending_mask
#define HIVE_MEM_event_is_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_is_pending_mask 0x48
#define HIVE_SIZE_event_is_pending_mask 44
#else
#error Symbol event_is_pending_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_event_is_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_is_pending_mask 0x48
#define HIVE_SIZE_sp_event_is_pending_mask 44

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_com
#define HIVE_MEM_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_com 0x2A50
#define HIVE_SIZE_host_sp_com 68
#else
#error Symbol host_sp_com occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2A50
#define HIVE_SIZE_sp_host_sp_com 68

/* function exec_image_pipe: 2365 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x2A94
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#error Symbol sp_init_dmem_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x2A94
#define HIVE_SIZE_sp_sp_init_dmem_data 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_use 0x11CC
#define HIVE_SIZE_sp_flash_in_use 4
#else
#error Symbol sp_flash_in_use occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0x11CC
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_flashed_frame_cnt 0x11D4
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#error Symbol flashed_frame_cnt occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0x11D4
#define HIVE_SIZE_sp_flashed_frame_cnt 4

/* function is_isp_debug_buffer_full: 2B1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_stack_sizes 0x294
#define HIVE_SIZE_stack_sizes 24
#else
#error Symbol stack_sizes occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stack_sizes 0x294
#define HIVE_SIZE_sp_stack_sizes 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_ph 0x250C
#define HIVE_SIZE_ph 28
#else
#error Symbol ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ph 0x250C
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_per_frame_data 0x2AAC
#define HIVE_SIZE_sp_per_frame_data 4
#else
#error Symbol sp_per_frame_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x2AAC
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 4855 */

/* function sp_tagger_connect_pipes: 3BE4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x2D50
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#error Symbol sp_copy_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x2D50
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_xmem_bin_addr 0x2AB0
#define HIVE_SIZE_xmem_bin_addr 4
#else
#error Symbol xmem_bin_addr occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x2AB0
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 4264 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_threads 0x26C
#define HIVE_SIZE_pipe_threads 16
#else
#error Symbol pipe_threads occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_threads 0x26C
#define HIVE_SIZE_sp_pipe_threads 16

/* function sp_isys_tokhandler_work: 3F04 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x6C8
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#error Symbol GP_DEVICE_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x6C8
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 2EFC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_exp_id
#define HIVE_MEM_ia_css_i_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_i_exp_id 0x6D4
#define HIVE_SIZE_ia_css_i_exp_id 1
#else
#error Symbol ia_css_i_exp_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_i_exp_id 0x6D4
#define HIVE_SIZE_sp_ia_css_i_exp_id 1

/* function __divu: 41E2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_stop
#define HIVE_MEM_sem_for_cont_capt_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_cont_capt_stop 0x3674
#define HIVE_SIZE_sem_for_cont_capt_stop 20
#else
#error Symbol sem_for_cont_capt_stop occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_stop 0x3674
#define HIVE_SIZE_sp_sem_for_cont_capt_stop 20

/* function sp_dma_proxy_func: 4309 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_started 0x2AB4
#define HIVE_SIZE_sp_isp_started 4
#else
#error Symbol sp_isp_started occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x2AB4
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x2D90
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#error Symbol sp_isp_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x2D90
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x2AB8
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#error Symbol sp_obarea_start_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x2AB8
#define HIVE_SIZE_sp_sp_obarea_start_bq 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_IRQ_BASE
#define HIVE_MEM_IRQ_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_IRQ_BASE 0x28
#define HIVE_SIZE_IRQ_BASE 4
#else
#error Symbol IRQ_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_IRQ_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_IRQ_BASE 0x28
#define HIVE_SIZE_sp_IRQ_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_TIMED_CTRL_BASE
#define HIVE_MEM_TIMED_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_TIMED_CTRL_BASE 0x30
#define HIVE_SIZE_TIMED_CTRL_BASE 4
#else
#error Symbol TIMED_CTRL_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_TIMED_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_TIMED_CTRL_BASE 0x30
#define HIVE_SIZE_sp_TIMED_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_is_isp_requested
#define HIVE_MEM_is_isp_requested scalar_processor_2400A0_dmem
#define HIVE_ADDR_is_isp_requested 0x11A8
#define HIVE_SIZE_is_isp_requested 4
#else
#error Symbol is_isp_requested occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_is_isp_requested scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_is_isp_requested 0x11A8
#define HIVE_SIZE_sp_is_isp_requested 4

/* function ia_css_i_sp_rmgr_init: 393 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_thread 0x260
#define HIVE_SIZE_current_sp_thread 4
#else
#error Symbol current_sp_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x260
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x2E90
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#error Symbol h_pipe_private_ddr_ptrs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x2E90
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x760
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#error Symbol sp_capture_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x760
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 3217 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_internal_event 0x364C
#define HIVE_SIZE_sp_internal_event 4
#else
#error Symbol sp_internal_event occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x364C
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 2F10 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MASK_NOF_FRAMES
#define HIVE_MEM_MASK_NOF_FRAMES scalar_processor_2400A0_dmem
#define HIVE_ADDR_MASK_NOF_FRAMES 0x6BC
#define HIVE_SIZE_MASK_NOF_FRAMES 4
#else
#error Symbol MASK_NOF_FRAMES occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_MASK_NOF_FRAMES scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MASK_NOF_FRAMES 0x6BC
#define HIVE_SIZE_sp_MASK_NOF_FRAMES 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_last_index 0x2648
#define HIVE_SIZE_last_index 4
#else
#error Symbol last_index occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_last_index 0x2648
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_queue_tail
#define HIVE_MEM_queue_tail scalar_processor_2400A0_dmem
#define HIVE_ADDR_queue_tail 0x3A5C
#define HIVE_SIZE_queue_tail 4
#else
#error Symbol queue_tail occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_queue_tail scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_queue_tail 0x3A5C
#define HIVE_SIZE_sp_queue_tail 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x2D00
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#error Symbol isp_sh_dma_cmd_buffer occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x2D00
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_start
#define HIVE_MEM_sem_for_cont_capt_start scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_cont_capt_start 0x3688
#define HIVE_SIZE_sem_for_cont_capt_start 20
#else
#error Symbol sem_for_cont_capt_start occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_start scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_start 0x3688
#define HIVE_SIZE_sp_sem_for_cont_capt_start 20

/* function host2sp_event_queue_is_empty: 42A7 */

/* function debug_buffer_init_isp: 6F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x75C
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#error Symbol sp_preview_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x75C
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x369C
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x369C
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 30DA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_fibers 0x2AC
#define HIVE_SIZE_fibers 24
#else
#error Symbol fibers occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_fibers 0x2AC
#define HIVE_SIZE_sp_fibers 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x36B0
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#error Symbol cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x36B0
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 4AB4 */

/* function initialize_sp_group: 2B2D */

/* function start_binary: 101D */

/* function sp_isys_tokhandler_stage: 3EE9 */

/* function sp_tagger_configure: 3943 */

/* function input_system_stop: 2975 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x2D04
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#error Symbol sp_invalidate_tlb occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x2D04
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_DMEM_BASE
#define HIVE_MEM_ISP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_DMEM_BASE 0x10
#define HIVE_SIZE_ISP_DMEM_BASE 4
#else
#error Symbol ISP_DMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_DMEM_BASE 0x10
#define HIVE_SIZE_sp_ISP_DMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_DMEM_BASE
#define HIVE_MEM_SP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_SP_DMEM_BASE 0x4
#define HIVE_SIZE_SP_DMEM_BASE 4
#else
#error Symbol SP_DMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_SP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_SP_DMEM_BASE 0x4
#define HIVE_SIZE_sp_SP_DMEM_BASE 4

/* function bin_spec_init_ifs: 681 */

/* function dma_proxy_channel_release: 4841 */

/* function token_get_xmem_addr: 41B8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_can_send_token_mask
#define HIVE_MEM_event_can_send_token_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_can_send_token_mask 0x74
#define HIVE_SIZE_event_can_send_token_mask 44
#else
#error Symbol event_can_send_token_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_event_can_send_token_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_can_send_token_mask 0x74
#define HIVE_SIZE_sp_event_can_send_token_mask 44

/* function sp_dma_proxy_wait_for_ack: 466D */

/* function sp_thread_yield: 4966 */

/* function sp_circular_buf_peek: 3455 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_thread
#define HIVE_MEM_isp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_thread 0x39A8
#define HIVE_SIZE_isp_thread 4
#else
#error Symbol isp_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_thread 0x39A8
#define HIVE_SIZE_sp_isp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x2ABC
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#error Symbol sp_obarea_length_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x2ABC
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 24B */

/* function sp_uds_init: 889 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_token_map_valid
#define HIVE_MEM_token_map_valid scalar_processor_2400A0_dmem
#define HIVE_ADDR_token_map_valid 0x3A60
#define HIVE_SIZE_token_map_valid 72
#else
#error Symbol token_map_valid occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_token_map_valid scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_token_map_valid 0x3A60
#define HIVE_SIZE_sp_token_map_valid 72

/* function sp_dma_proxy_isp_write_addr: 2F5B */

/* function sp_circular_buf_create: 36C5 */

/* function token_map_dequeue: 4144 */

/* function debug_enqueue_ddr: 79 */

/* function host2sp_dequeue_buffer: 370 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_buf_swap 0x6F0
#define HIVE_SIZE_buf_swap 96
#else
#error Symbol buf_swap occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_buf_swap 0x6F0
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 38DB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_curr_nr_of_copied_frames
#define HIVE_MEM_curr_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_curr_nr_of_copied_frames 0x2CDC
#define HIVE_SIZE_curr_nr_of_copied_frames 4
#else
#error Symbol curr_nr_of_copied_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_curr_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_curr_nr_of_copied_frames 0x2CDC
#define HIVE_SIZE_sp_curr_nr_of_copied_frames 4

/* function sp2host_enqueue_buffer: 351 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x270C
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#error Symbol sp_dma_crop_block_width_b occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x270C
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x36C4
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#error Symbol cb_elems_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x36C4
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_output 0x2AC0
#define HIVE_SIZE_sp_output 16
#else
#error Symbol sp_output occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_output 0x2AC0
#define HIVE_SIZE_sp_sp_output 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_CTRL_BASE
#define HIVE_MEM_ISP_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_ISP_CTRL_BASE 4
#else
#error Symbol ISP_CTRL_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_sp_ISP_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_INPUT_FORMATTER_BASE
#define HIVE_MEM_INPUT_FORMATTER_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_INPUT_FORMATTER_BASE 0x3C
#define HIVE_SIZE_INPUT_FORMATTER_BASE 12
#else
#error Symbol INPUT_FORMATTER_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_INPUT_FORMATTER_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_INPUT_FORMATTER_BASE 0x3C
#define HIVE_SIZE_sp_INPUT_FORMATTER_BASE 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_sem
#define HIVE_MEM_raw_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_raw_frame_sem 0x264C
#define HIVE_SIZE_raw_frame_sem 20
#else
#error Symbol raw_frame_sem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x264C
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_raw_frame_pool 0x2660
#define HIVE_SIZE_raw_frame_pool 16
#else
#error Symbol raw_frame_pool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x2660
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 2BB2 */

/* function __sp_dma_proxy_configure_channel_text: 300B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3650
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#error Symbol sem_for_sp2host_event_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3650
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_request_acquire_mipi_buffer
#define HIVE_MEM_request_acquire_mipi_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_request_acquire_mipi_buffer 0x3A58
#define HIVE_SIZE_request_acquire_mipi_buffer 4
#else
#error Symbol request_acquire_mipi_buffer occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_request_acquire_mipi_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_request_acquire_mipi_buffer 0x3A58
#define HIVE_SIZE_sp_request_acquire_mipi_buffer 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_tagger 0x2CE0
#define HIVE_SIZE_tagger 20
#else
#error Symbol tagger occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_tagger 0x2CE0
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_refpool 0x2670
#define HIVE_SIZE_refpool 16
#else
#error Symbol refpool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_refpool 0x2670
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 32E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_token_map_queue
#define HIVE_MEM_token_map_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_token_map_queue 0x3AA8
#define HIVE_SIZE_token_map_queue 72
#else
#error Symbol token_map_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_token_map_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_token_map_queue 0x3AA8
#define HIVE_SIZE_sp_token_map_queue 72

/* function sp_start_isp_entry: 2144 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#error Symbol sp_start_isp_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x2144
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x2144

/* function __sp_raw_copy_func_critical: 42B1 */

/* function add_sp_command: 4823 */

/* function sp2host_enqueue_irq_event: 311 */

/* function create_sp_fibers: 317F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x2EA0
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#error Symbol pipe_private_s3a_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x2EA0
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

/* function sp_debug_mode_init: 3DEB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x2710
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#error Symbol sp_dma_crop_block_width_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x2710
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 5D5 */

/* function ia_css_i_sp_rmgr_acq_gen: 3ED */

/* function sp_turn_on_flash: 3236 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_queue_head
#define HIVE_MEM_queue_head scalar_processor_2400A0_dmem
#define HIVE_ADDR_queue_head 0x3AF0
#define HIVE_SIZE_queue_head 4
#else
#error Symbol queue_head occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_queue_head scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_queue_head 0x3AF0
#define HIVE_SIZE_sp_queue_head 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x2EC0
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#error Symbol sems_for_sp2host_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x2EC0
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x2AD0
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#error Symbol isp_vectors_per_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x2AD0
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x2AD4
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#error Symbol h_latest_parameter_set occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x2AD4
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function __sp_dma_proxy_func_text: 2E78 */

/* function sp_thread_join: 32F0 */

/* function sp_dma_proxy_configure_channel: 469C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_group 0x2AE4
#define HIVE_SIZE_sp_group 460
#else
#error Symbol sp_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_group 0x2AE4
#define HIVE_SIZE_sp_sp_group 460

/* function sp2host_buffer_queue_get_size: 345 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x2E50
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#error Symbol sp_event_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x2E50
#define HIVE_SIZE_sp_sp_event_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_pool
#define HIVE_MEM_dma_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_pool 0xB4
#define HIVE_SIZE_dma_pool 4
#else
#error Symbol dma_pool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_pool 0xB4
#define HIVE_SIZE_sp_dma_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_uv_internal_width_vecs
#define HIVE_MEM_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x2CB0
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#error Symbol isp_uv_internal_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x2CB0
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x2F4C
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#error Symbol pipe_private_exp_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x2F4C
#define HIVE_SIZE_sp_pipe_private_exp_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MMU_BASE
#define HIVE_MEM_MMU_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_MMU_BASE 0x24
#define HIVE_SIZE_MMU_BASE 4
#else
#error Symbol MMU_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_MMU_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MMU_BASE 0x24
#define HIVE_SIZE_sp_MMU_BASE 4

/* function sp_dma_proxy_configure_init_dmem_channel: 2FD3 */

/* function sp2host_event_queue_get_size: 308 */

/* function isp_hmem_load: 3EA8 */

/* function sp_dma_proxy_read_byte_addr: 4685 */

/* function sp_thread_fork: 3301 */

/* function sp_semaphore_wait: 4A27 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_buffer_ddr_address
#define HIVE_MEM_debug_buffer_ddr_address scalar_processor_2400A0_dmem
#define HIVE_ADDR_debug_buffer_ddr_address 0xA8
#define HIVE_SIZE_debug_buffer_ddr_address 4
#else
#error Symbol debug_buffer_ddr_address occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_debug_buffer_ddr_address scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_debug_buffer_ddr_address 0xA8
#define HIVE_SIZE_sp_debug_buffer_ddr_address 4

/* function input_system_run: 2996 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x36D4
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x36D4
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

/* function token_map_enqueue: 416B */

/* function token_map_update: 40C0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_request_flash 0x2D0C
#define HIVE_SIZE_sp_request_flash 4
#else
#error Symbol sp_request_flash occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x2D0C
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 3DC1 */

/* function cnd_input_system_cfg: 29D4 */

/* function sp_generate_events: 3CCE */

/* function sp_uds_configure: 712 */

/* function sp_dma_proxy_execute: 2FA6 */

/* function __modu: 4228 */

/* function sp_circular_buf_push_marked: 34C6 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x2CB4
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#error Symbol isp_sdis_horiproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x2CB4
#define HIVE_SIZE_sp_isp_sdis_horiproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GDC_BASE
#define HIVE_MEM_GDC_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GDC_BASE 0x34
#define HIVE_SIZE_GDC_BASE 8
#else
#error Symbol GDC_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_GDC_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GDC_BASE 0x34
#define HIVE_SIZE_sp_GDC_BASE 8

/* function sp_fiber_init: 3206 */

/* function ia_css_i_sp_rmgr_uninit: 38C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isys_token_handler
#define HIVE_MEM_sem_for_isys_token_handler scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isys_token_handler 0x36E8
#define HIVE_SIZE_sem_for_isys_token_handler 20
#else
#error Symbol sem_for_isys_token_handler occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_isys_token_handler scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isys_token_handler 0x36E8
#define HIVE_SIZE_sp_sem_for_isys_token_handler 20

/* function sp_thread_init: 3328 */

/* function irq_raise_set_token: 41 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GPIO_BASE
#define HIVE_MEM_GPIO_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GPIO_BASE 0x2C
#define HIVE_SIZE_GPIO_BASE 4
#else
#error Symbol GPIO_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_GPIO_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GPIO_BASE 0x2C
#define HIVE_SIZE_sp_GPIO_BASE 4

/* function _dma_proxy_dma_read_write: 472B */

/* function sp_dma_proxy_configure_init_vmem_channel: 2FEF */

/* function sp_acquire_dynamic_buf: 383C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_ph 0x2548
#define HIVE_SIZE_isp_ph 28
#else
#error Symbol isp_ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_ph 0x2548
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 3BEE */

/* function init_isp_internal_buffers: 1106 */

/* function dma_proxy_dma_set_addr_B: 302D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_captured_frames
#define HIVE_MEM_target_nr_of_captured_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_target_nr_of_captured_frames 0x2CF4
#define HIVE_SIZE_target_nr_of_captured_frames 4
#else
#error Symbol target_nr_of_captured_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_captured_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_target_nr_of_captured_frames 0x2CF4
#define HIVE_SIZE_sp_target_nr_of_captured_frames 4

/* function sp_dma_proxy_write: 2F8B */

/* function ia_css_i_sp_refcount_init_vbuf: 42E */

/* function isp_hmem_clear: 3E6D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x36FC
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x36FC
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 2F42 */

/* function sp_isys_tokhandler_isr: 3FCD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3710
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3710
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3724
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#error Symbol cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3724
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x2CB8
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#error Symbol isp_sdis_vertproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x2CB8
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3738
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#error Symbol cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3738
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queue 0x2F50
#define HIVE_SIZE_host_sp_queue 1036
#else
#error Symbol host_sp_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x2F50
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 3340 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_pool
#define HIVE_MEM_isp_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_pool 0xB8
#define HIVE_SIZE_isp_pool 4
#else
#error Symbol isp_pool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pool 0xB8
#define HIVE_SIZE_sp_isp_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_is_done_flag
#define HIVE_MEM_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_is_done_flag 0xC9C
#define HIVE_SIZE_isp_is_done_flag 1
#else
#error Symbol isp_is_done_flag occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0xC9C
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_any_pending_mask 0x6CC
#define HIVE_SIZE_event_any_pending_mask 8
#else
#error Symbol event_any_pending_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x6CC
#define HIVE_SIZE_sp_event_any_pending_mask 8

/* function ia_css_i_sp_refcount_release_vbuf: 4BD */

/* function init_isp_data_segment: 1195 */

/* function sh_css_decode_tag_descr: 587 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_configs 0x2680
#define HIVE_SIZE_dma_configs 140
#else
#error Symbol dma_configs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_configs 0x2680
#define HIVE_SIZE_sp_dma_configs 140

/* function debug_enqueue_isp: 1FA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x2714
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#error Symbol sp_dma_crop_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x2714
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x374C
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#error Symbol sem_for_isp_idle occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x374C
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_channels 0x2528
#define HIVE_SIZE_channels 32
#else
#error Symbol channels occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_channels 0x2528
#define HIVE_SIZE_sp_channels 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x2CBC
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#error Symbol sp_vf_downscale_bits occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x2CBC
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x2CC0
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#error Symbol isp_sdis_vertcoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x2CC0
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 8CF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_VAMEM_BASE
#define HIVE_MEM_ISP_VAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_ISP_VAMEM_BASE 12
#else
#error Symbol ISP_VAMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_VAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_sp_ISP_VAMEM_BASE 12

/* function sp_tagger_create: 3C0F */

/* function sp_dma_proxy_vmem_write: 2F29 */

/* function sp_thread_set_priority: 32C5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x335C
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#error Symbol pipe_private_dis_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x335C
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 49E1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MASK_NOF_XMEM_WORDS
#define HIVE_MEM_MASK_NOF_XMEM_WORDS scalar_processor_2400A0_dmem
#define HIVE_ADDR_MASK_NOF_XMEM_WORDS 0x6C0
#define HIVE_SIZE_MASK_NOF_XMEM_WORDS 4
#else
#error Symbol MASK_NOF_XMEM_WORDS occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_MASK_NOF_XMEM_WORDS scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MASK_NOF_XMEM_WORDS 0x6C0
#define HIVE_SIZE_sp_MASK_NOF_XMEM_WORDS 4

/* function token_map_is_queue_full: 418F */

/* function sp_dma_proxy_write_byte_addr: 2F73 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_PMEM_BASE
#define HIVE_MEM_SP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_SP_PMEM_BASE 0x0
#define HIVE_SIZE_SP_PMEM_BASE 4
#else
#error Symbol SP_PMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_SP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_SP_PMEM_BASE 0x0
#define HIVE_SIZE_sp_SP_PMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_input_stream_format
#define HIVE_MEM_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x2CC4
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#error Symbol sp_isp_input_stream_format occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x2CC4
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 33D0 */

/* function __mod: 4214 */

/* function __sp_event_proxy_func_critical: 4869 */

/* function sp_circular_buf_mark: 359D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_copied_frames
#define HIVE_MEM_target_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_target_nr_of_copied_frames 0x2CF8
#define HIVE_SIZE_target_nr_of_copied_frames 4
#else
#error Symbol target_nr_of_copied_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_target_nr_of_copied_frames 0x2CF8
#define HIVE_SIZE_sp_target_nr_of_copied_frames 4

/* function irq_raise: 53 */

/* function sp_circular_buf_unmark: 3575 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 47B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_ADDRESS
#define HIVE_MEM_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_ADDRESS 0x330
#define HIVE_SIZE_HIVE_IF_SRST_ADDRESS 12
#else
#error Symbol HIVE_IF_SRST_ADDRESS occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_ADDRESS 0x330
#define HIVE_SIZE_sp_HIVE_IF_SRST_ADDRESS 12

/* function _dma_proxy_dma_execute: 47E8 */

/* function timed_ctrl_snd_gpio_commnd: 2CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3760
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#error Symbol cb_elems_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3760
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3770
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#error Symbol cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3770
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

/* function set_sp_sleep_for_debug: 3DF6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_fiber 0x11C0
#define HIVE_SIZE_current_sp_fiber 4
#else
#error Symbol current_sp_fiber occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0x11C0
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x2718
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#error Symbol sp_dma_vfout_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x2718
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3784
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#error Symbol cb_elems_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3784
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 2E96 */

/* function sp_release_dynamic_buf: 36DC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_HMEM_BASE
#define HIVE_MEM_ISP_HMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_ISP_HMEM_BASE 4
#else
#error Symbol ISP_HMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_HMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_sp_ISP_HMEM_BASE 4

/* function token_cpy: 41A7 */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 4A6 */

/* function timed_ctrl_snd_commnd: 2F8 */

/* function dma_proxy_dma_set_stride_B: 3022 */

/* function end_binary: F86 */

/* function token_get_id: 41C9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_stacks 0x27C
#define HIVE_SIZE_stacks 24
#else
#error Symbol stacks occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stacks 0x27C
#define HIVE_SIZE_sp_stacks 24

/* function dma_proxy_dma_execute_split: 307F */

/* function ia_css_i_sp_refcount_dump: 456 */

/* function ia_css_i_sp_rmgr_rel_gen: 3D5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x2508
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#error Symbol irq_sw_interrupt_token occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x2508
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 32CD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x337C
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#error Symbol pipe_private_buffer_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x337C
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_addresses 0x39AC
#define HIVE_SIZE_sp_isp_addresses 172
#else
#error Symbol sp_isp_addresses occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x39AC
#define HIVE_SIZE_sp_sp_isp_addresses 172

/* function token_get_unit: 41C0 */

/* function sp_fiber_main: 3210 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_isps 0x2564
#define HIVE_SIZE_isps 28
#else
#error Symbol isps occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isps 0x2564
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x2CC8
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#error Symbol host_sp_queues_initialized occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x2CC8
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function token_map_get: 4095 */

/* function timed_ctrl_snd_sp_commnd: 2E2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_token_map
#define HIVE_MEM_token_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_token_map 0x3AF4
#define HIVE_SIZE_token_map 144
#else
#error Symbol token_map occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_token_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_token_map 0x3AF4
#define HIVE_SIZE_sp_token_map 144

/* function __sp_dma_proxy_wait_for_ack_text: 2EDC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_spref
#define HIVE_MEM_vbuf_spref scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_spref 0xBC
#define HIVE_SIZE_vbuf_spref 4
#else
#error Symbol vbuf_spref occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_vbuf_spref scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_spref 0xBC
#define HIVE_SIZE_sp_vbuf_spref 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MASK_TOKEN_ID
#define HIVE_MEM_MASK_TOKEN_ID scalar_processor_2400A0_dmem
#define HIVE_ADDR_MASK_TOKEN_ID 0x6B4
#define HIVE_SIZE_MASK_TOKEN_ID 4
#else
#error Symbol MASK_TOKEN_ID occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_MASK_TOKEN_ID scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MASK_TOKEN_ID 0x6B4
#define HIVE_SIZE_sp_MASK_TOKEN_ID 4

/* function sp_circular_buf_extract: 35C5 */

/* function output_compute_dma_info: 1B90 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x2CCC
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#error Symbol isp_sdis_horicoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x2CCC
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3794
#define HIVE_SIZE_sem_for_reading_if 20
#else
#error Symbol sem_for_reading_if occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3794
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function sp_circular_buf_pop_marked: 347A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_frames 0x33EC
#define HIVE_SIZE_pipe_private_frames 48
#else
#error Symbol pipe_private_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x33EC
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 3C37 */

/* function init_isp_vars: 1E34 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x341C
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#error Symbol sems_for_host2sp_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x341C
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_data 0x27C8
#define HIVE_SIZE_sp_data 640
#else
#error Symbol sp_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_data 0x27C8
#define HIVE_SIZE_sp_sp_data 640

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x6C4
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#error Symbol ISP_BAMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x6C4
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function acquire_isp: 3E1D */

/* function sp_circular_buf_is_marked: 3550 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_mem_map 0x271C
#define HIVE_SIZE_mem_map 100
#else
#error Symbol mem_map occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_mem_map 0x271C
#define HIVE_SIZE_sp_mem_map 100

/* function sp_init_dmem: 2AB3 */

/* function ia_css_i_sp_refcount_retain_vbuf: 4DE */

/* function init_isp_code_segment: 1040 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#error Symbol ISP_PMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function run_sp_threads: 3356 */

/* function configure_dma_channel: 2B92 */

/* function sp_thread_queue_print: 3373 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_service 0x11D0
#define HIVE_SIZE_sp_flash_in_service 4
#else
#error Symbol sp_flash_in_service occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0x11D0
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x2CD0
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#error Symbol isp_vf_output_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x2CD0
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 338C */

/* function token_map_is_queue_empty: 419E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sleep_mode 0x2CD4
#define HIVE_SIZE_sp_sleep_mode 4
#else
#error Symbol sp_sleep_mode occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x2CD4
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: 5E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stop_req 0x2D08
#define HIVE_SIZE_isp_stop_req 4
#else
#error Symbol isp_stop_req occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x2D08
#define HIVE_SIZE_sp_isp_stop_req 4

/* function release_isp: 3E08 */

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
