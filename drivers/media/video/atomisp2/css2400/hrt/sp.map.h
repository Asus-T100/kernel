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
#define HIVE_ADDR_isp_vectors_per_input_line 0x2448
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#error Symbol isp_vectors_per_input_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2448
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 45C4 */

/* function longjmp: 33DE */

/* function sp_tagger_tag_exp_id: 30F9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_MASK
#define HIVE_MEM_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_MASK 0x340
#define HIVE_SIZE_HIVE_IF_SRST_MASK 12
#else
#error Symbol HIVE_IF_SRST_MASK occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_MASK 0x340
#define HIVE_SIZE_sp_HIVE_IF_SRST_MASK 12

/* function sp_dma_proxy_read: 2987 */

/* function sp_dma_proxy_is_idle: 2B17 */

/* function debug_buffer_set_ddr_addr: 68 */

/* function __exec_image_pipe_critical: 35A5 */

/* function setjmp: 33E7 */

/* function decode_sw_event: 4F1 */

/* function initialize_sp_stage: 3910 */

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
#define HIVE_ADDR_dma_proxy_status 0x1D8
#define HIVE_SIZE_dma_proxy_status 4
#else
#error Symbol dma_proxy_status occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x1D8
#define HIVE_SIZE_sp_dma_proxy_status 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x347C
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#error Symbol do_mmu_invalidation occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x347C
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 217A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_binary_group 0x20D4
#define HIVE_SIZE_sp_binary_group 72
#else
#error Symbol sp_binary_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x20D4
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sw_state 0x244C
#define HIVE_SIZE_sp_sw_state 4
#else
#error Symbol sp_sw_state occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x244C
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 2E67 */

/* function sp_circular_buf_pop: 2E4E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stage
#define HIVE_MEM_sp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stage 0x2450
#define HIVE_SIZE_sp_stage 3328
#else
#error Symbol sp_stage occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_stage 0x2450
#define HIVE_SIZE_sp_sp_stage 3328

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_handles 0x1ED0
#define HIVE_SIZE_vbuf_handles 200
#else
#error Symbol vbuf_handles occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x1ED0
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3DE4
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#error Symbol cb_elems_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3DE4
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x3490
#define HIVE_SIZE_sp_dma_proxy_thread 64
#else
#error Symbol sp_dma_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x3490
#define HIVE_SIZE_sp_sp_dma_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x268
#define HIVE_SIZE_sp_thread_ready_queue 8
#else
#error Symbol sp_thread_ready_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x268
#define HIVE_SIZE_sp_sp_thread_ready_queue 8

/* function sp_debug_mode_update_command: 328C */

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
#define HIVE_ADDR_host_sp_com 0x3150
#define HIVE_SIZE_host_sp_com 48
#else
#error Symbol host_sp_com occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_com 0x3150
#define HIVE_SIZE_sp_host_sp_com 48

/* function exec_image_pipe: 2389 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x3180
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#error Symbol sp_init_dmem_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x3180
#define HIVE_SIZE_sp_sp_init_dmem_data 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_use 0xB60
#define HIVE_SIZE_sp_flash_in_use 4
#else
#error Symbol sp_flash_in_use occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0xB60
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_flashed_frame_cnt 0xB68
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#error Symbol flashed_frame_cnt occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0xB68
#define HIVE_SIZE_sp_flashed_frame_cnt 4

/* function is_isp_debug_buffer_full: 2B1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_stack_sizes 0x298
#define HIVE_SIZE_stack_sizes 24
#else
#error Symbol stack_sizes occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stack_sizes 0x298
#define HIVE_SIZE_sp_stack_sizes 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_ph 0x1E5C
#define HIVE_SIZE_ph 28
#else
#error Symbol ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ph 0x1E5C
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_per_frame_data 0x3198
#define HIVE_SIZE_sp_per_frame_data 4
#else
#error Symbol sp_per_frame_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x3198
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 4093 */

/* function sp_tagger_connect_pipes: 3173 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x34D0
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#error Symbol sp_copy_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x34D0
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_xmem_bin_addr 0x319C
#define HIVE_SIZE_xmem_bin_addr 4
#else
#error Symbol xmem_bin_addr occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x319C
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 3471 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_threads 0x270
#define HIVE_SIZE_pipe_threads 16
#else
#error Symbol pipe_threads occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_threads 0x270
#define HIVE_SIZE_sp_pipe_threads 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x350
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#error Symbol GP_DEVICE_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x350
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 28CA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_exp_id
#define HIVE_MEM_ia_css_i_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_i_exp_id 0x35C
#define HIVE_SIZE_ia_css_i_exp_id 1
#else
#error Symbol ia_css_i_exp_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_i_exp_id 0x35C
#define HIVE_SIZE_sp_ia_css_i_exp_id 1

/* function __divu: 33EF */

/* function sp_dma_proxy_func: 3BDB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_started 0x31A0
#define HIVE_SIZE_sp_isp_started 4
#else
#error Symbol sp_isp_started occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x31A0
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x3510
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#error Symbol sp_isp_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x3510
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x31A4
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#error Symbol sp_obarea_start_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x31A4
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

/* function ia_css_i_sp_rmgr_init: 336 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_thread 0x264
#define HIVE_SIZE_current_sp_thread 4
#else
#error Symbol current_sp_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x264
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x3610
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#error Symbol h_pipe_private_ddr_ptrs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x3610
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x3E8
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#error Symbol sp_capture_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x3E8
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 2C57 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_internal_event 0x3DCC
#define HIVE_SIZE_sp_internal_event 4
#else
#error Symbol sp_internal_event occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x3DCC
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 28DE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_last_index 0x1F98
#define HIVE_SIZE_last_index 4
#else
#error Symbol last_index occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_last_index 0x1F98
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x3480
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#error Symbol isp_sh_dma_cmd_buffer occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x3480
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

/* function host2sp_event_queue_is_empty: 34D9 */

/* function debug_buffer_init_isp: 6F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x3E4
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#error Symbol sp_preview_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x3E4
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x3DF4
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x3DF4
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 2B2C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_fibers 0x2B0
#define HIVE_SIZE_fibers 24
#else
#error Symbol fibers occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_fibers 0x2B0
#define HIVE_SIZE_sp_fibers 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x3E08
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#error Symbol cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3E08
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 4329 */

/* function initialize_sp_group: 278D */

/* function start_binary: F88 */

/* function sp_tagger_configure: 3146 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x3484
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#error Symbol sp_invalidate_tlb occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x3484
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

/* function __sp2host_enqueue_irq_event_text: 30F */

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

/* function bin_spec_init_ifs: 5C2 */

/* function dma_proxy_channel_release: 407F */

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

/* function sp_dma_proxy_wait_for_ack: 3F15 */

/* function sp_thread_yield: 41BF */

/* function sp_circular_buf_peek: 43C0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x31A8
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#error Symbol sp_obarea_length_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x31A8
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 24B */

/* function __sp_circular_buf_pop_critical: 4577 */

/* function sp_uds_init: 7C5 */

/* function __sp_thread_fork_critical: 423A */

/* function sp_dma_proxy_isp_write_addr: 2929 */

/* function sp_circular_buf_create: 2E73 */

/* function debug_enqueue_ddr: 79 */

/* function host2sp_dequeue_buffer: 352E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_buf_swap 0x378
#define HIVE_SIZE_buf_swap 96
#else
#error Symbol buf_swap occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_buf_swap 0x378
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 30DE */

/* function sp2host_enqueue_buffer: 3505 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x205C
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#error Symbol sp_dma_crop_block_width_b occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x205C
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x3E1C
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#error Symbol cb_elems_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x3E1C
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_output 0x31AC
#define HIVE_SIZE_sp_output 212
#else
#error Symbol sp_output occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_output 0x31AC
#define HIVE_SIZE_sp_sp_output 212

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
#define HIVE_ADDR_raw_frame_sem 0x1F9C
#define HIVE_SIZE_raw_frame_sem 20
#else
#error Symbol raw_frame_sem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x1F9C
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_raw_frame_pool 0x1FB0
#define HIVE_SIZE_raw_frame_pool 16
#else
#error Symbol raw_frame_pool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x1FB0
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 3954 */

/* function __sp_dma_proxy_configure_channel_text: 29D8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3DD0
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#error Symbol sem_for_sp2host_event_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3DD0
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_tagger 0x3468
#define HIVE_SIZE_tagger 20
#else
#error Symbol tagger occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_tagger 0x3468
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_refpool 0x1FC0
#define HIVE_SIZE_refpool 16
#else
#error Symbol refpool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_refpool 0x1FC0
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 34E3 */

/* function sp_start_isp_entry: 2170 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#error Symbol sp_start_isp_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x2170
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x2170

/* function add_sp_command: 4061 */

/* function sp2host_enqueue_irq_event: 34C1 */

/* function create_sp_fibers: 2BBF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x3620
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#error Symbol pipe_private_s3a_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x3620
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

/* function sp_debug_mode_init: 3296 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x2060
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#error Symbol sp_dma_crop_block_width_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x2060
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 51B */

/* function ia_css_i_sp_rmgr_acq_gen: 390 */

/* function sp_turn_on_flash: 2C76 */

/* function start_input: 2613 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x3640
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#error Symbol sems_for_sp2host_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x3640
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x3280
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#error Symbol isp_vectors_per_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x3280
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x3284
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#error Symbol h_latest_parameter_set occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x3284
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function __sp_dma_proxy_func_text: 2845 */

/* function sp_thread_join: 2D10 */

/* function sp_dma_proxy_configure_channel: 3F43 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_group 0x3294
#define HIVE_SIZE_sp_group 428
#else
#error Symbol sp_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_group 0x3294
#define HIVE_SIZE_sp_sp_group 428

/* function sp2host_buffer_queue_get_size: 319 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x35D0
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#error Symbol sp_event_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x35D0
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
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x3440
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#error Symbol isp_uv_internal_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x3440
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x36CC
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#error Symbol pipe_private_exp_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x36CC
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

/* function sp_dma_proxy_configure_init_dmem_channel: 29A1 */

/* function host2sp_buffer_queue_is_empty: 351F */

/* function sp2host_event_queue_get_size: 306 */

/* function isp_hmem_load: 339D */

/* function sp_dma_proxy_read_byte_addr: 3F2C */

/* function sp_thread_fork: 2D21 */

/* function sp_semaphore_wait: 429C */

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

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3E2C
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3E2C
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

/* function __host2sp_dequeue_sp_event_text: 314 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_request_flash 0x348C
#define HIVE_SIZE_sp_request_flash 4
#else
#error Symbol sp_request_flash occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x348C
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 326C */

/* function dma_proxy_dma_read_write_split: 2A4C */

/* function cnd_input_system_cfg: 2631 */

/* function sp_generate_events: 4819 */

/* function sp_uds_configure: 653 */

/* function sp_dma_proxy_execute: 2974 */

/* function __modu: 3435 */

/* function sp_circular_buf_push_marked: 2DD4 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x3444
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#error Symbol isp_sdis_horiproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x3444
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

/* function sp_fiber_init: 2C46 */

/* function ia_css_i_sp_rmgr_uninit: 32F */

/* function __sp_circular_buf_push_marked_critical: 4431 */

/* function sp_thread_init: 2D2C */

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

/* function _dma_proxy_dma_read_write: 3FD2 */

/* function sp_dma_proxy_configure_init_vmem_channel: 29BD */

/* function sp_acquire_dynamic_buf: 2FEA */

/* function __host2sp_dequeue_buffer_text: 32A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_ph 0x1E98
#define HIVE_SIZE_isp_ph 28
#else
#error Symbol isp_ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_ph 0x1E98
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 317D */

/* function __sp_circular_buf_extract_critical: 44E0 */

/* function init_isp_internal_buffers: 1069 */

/* function dma_proxy_dma_set_addr_B: 29FA */

/* function sp_dma_proxy_write: 2959 */

/* function ia_css_i_sp_refcount_init_vbuf: 3D1 */

/* function __sp_tagger_configure_critical: 459D */

/* function isp_hmem_clear: 3362 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3E40
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3E40
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 2910 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3E54
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3E54
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

/* function sp2host_buffer_queue_is_full: 34F6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3E68
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#error Symbol cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3E68
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x3448
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#error Symbol isp_sdis_vertproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x3448
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3E7C
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#error Symbol cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3E7C
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

/* function stop_input: 25F5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queue 0x36D0
#define HIVE_SIZE_host_sp_queue 1036
#else
#error Symbol host_sp_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x36D0
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 2D44 */

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
#define HIVE_ADDR_isp_is_done_flag 0x400
#define HIVE_SIZE_isp_is_done_flag 1
#else
#error Symbol isp_is_done_flag occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0x400
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_any_pending_mask 0x354
#define HIVE_SIZE_event_any_pending_mask 8
#else
#error Symbol event_any_pending_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x354
#define HIVE_SIZE_sp_event_any_pending_mask 8

/* function ia_css_i_sp_refcount_release_vbuf: 3547 */

/* function init_isp_data_segment: 10F8 */

/* function sh_css_decode_tag_descr: 4CD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_configs 0x1FD0
#define HIVE_SIZE_dma_configs 140
#else
#error Symbol dma_configs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_configs 0x1FD0
#define HIVE_SIZE_sp_dma_configs 140

/* function debug_enqueue_isp: 1FA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x2064
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#error Symbol sp_dma_crop_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x2064
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x3E90
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#error Symbol sem_for_isp_idle occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x3E90
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_channels 0x1E78
#define HIVE_SIZE_channels 32
#else
#error Symbol channels occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_channels 0x1E78
#define HIVE_SIZE_sp_channels 32

/* function __sp2host_enqueue_buffer_text: 325 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x344C
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#error Symbol sp_vf_downscale_bits occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x344C
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x3450
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#error Symbol isp_sdis_vertcoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x3450
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 80B */

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

/* function sp_tagger_create: 319E */

/* function sp_dma_proxy_vmem_write: 28F7 */

/* function sp_thread_set_priority: 2CE5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3ADC
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#error Symbol pipe_private_dis_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3ADC
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 4256 */

/* function sp_dma_proxy_write_byte_addr: 2941 */

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
#define HIVE_ADDR_sp_isp_input_stream_format 0x3454
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#error Symbol sp_isp_input_stream_format occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x3454
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 433B */

/* function __mod: 3421 */

/* function __sp_event_proxy_func_critical: 40A7 */

/* function sp_circular_buf_mark: 44B8 */

/* function irq_raise: 53 */

/* function sp_circular_buf_unmark: 4490 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 41F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_ADDRESS
#define HIVE_MEM_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_ADDRESS 0x334
#define HIVE_SIZE_HIVE_IF_SRST_ADDRESS 12
#else
#error Symbol HIVE_IF_SRST_ADDRESS occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_ADDRESS 0x334
#define HIVE_SIZE_sp_HIVE_IF_SRST_ADDRESS 12

/* function _dma_proxy_dma_execute: 4027 */

/* function timed_ctrl_snd_gpio_commnd: 2CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3EA4
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#error Symbol cb_elems_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3EA4
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3EB4
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#error Symbol cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3EB4
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

/* function set_sp_sleep_for_debug: 32A1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_fiber 0xB54
#define HIVE_SIZE_current_sp_fiber 4
#else
#error Symbol current_sp_fiber occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0xB54
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x2068
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#error Symbol sp_dma_vfout_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x2068
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3EC8
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#error Symbol cb_elems_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3EC8
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 2863 */

/* function sp_release_dynamic_buf: 2E8A */

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

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 44A */

/* function timed_ctrl_snd_commnd: 2F6 */

/* function dma_proxy_dma_set_stride_B: 29EF */

/* function end_binary: EF1 */

/* function sp2host_event_queue_is_full: 34B4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_stacks 0x280
#define HIVE_SIZE_stacks 24
#else
#error Symbol stacks occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stacks 0x280
#define HIVE_SIZE_sp_stacks 24

/* function dma_proxy_dma_execute_split: 2AD1 */

/* function ia_css_i_sp_refcount_dump: 3FA */

/* function ia_css_i_sp_rmgr_rel_gen: 378 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x1E58
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#error Symbol irq_sw_interrupt_token occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x1E58
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 2CED */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x3AFC
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#error Symbol pipe_private_buffer_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x3AFC
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_addresses 0x211C
#define HIVE_SIZE_sp_isp_addresses 172
#else
#error Symbol sp_isp_addresses occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x211C
#define HIVE_SIZE_sp_sp_isp_addresses 172

/* function sp_fiber_main: 2C50 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_isps 0x1EB4
#define HIVE_SIZE_isps 28
#else
#error Symbol isps occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isps 0x1EB4
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x3458
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#error Symbol host_sp_queues_initialized occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x3458
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function timed_ctrl_snd_sp_commnd: 2E1 */

/* function __sp_dma_proxy_wait_for_ack_text: 28A9 */

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

/* function sp_circular_buf_extract: 2E25 */

/* function output_compute_dma_info: 1B96 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x345C
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#error Symbol isp_sdis_horicoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x345C
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3ED8
#define HIVE_SIZE_sem_for_reading_if 20
#else
#error Symbol sem_for_reading_if occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3ED8
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function update_sp_debug_info: 32B3 */

/* function sp_circular_buf_pop_marked: 43E5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_frames 0x3B6C
#define HIVE_SIZE_pipe_private_frames 48
#else
#error Symbol pipe_private_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x3B6C
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 31C6 */

/* function init_isp_vars: 1E36 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x3B9C
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#error Symbol sems_for_host2sp_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x3B9C
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_data 0x21C8
#define HIVE_SIZE_sp_data 640
#else
#error Symbol sp_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_data 0x21C8
#define HIVE_SIZE_sp_sp_data 640

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x34C
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#error Symbol ISP_BAMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x34C
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function sp_circular_buf_is_marked: 446B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_mem_map 0x206C
#define HIVE_SIZE_mem_map 104
#else
#error Symbol mem_map occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_mem_map 0x206C
#define HIVE_SIZE_sp_mem_map 104

/* function sp_init_dmem: 2713 */

/* function ia_css_i_sp_refcount_retain_vbuf: 3568 */

/* function init_isp_code_segment: FAB */

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

/* function run_sp_threads: 2D5A */

/* function configure_dma_channel: 27AF */

/* function __sp_circular_buf_create_critical: 4593 */

/* function sp_thread_queue_print: 2D77 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_service 0xB64
#define HIVE_SIZE_sp_flash_in_service 4
#else
#error Symbol sp_flash_in_service occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0xB64
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x3460
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#error Symbol isp_vf_output_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x3460
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 2D90 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sleep_mode 0x3464
#define HIVE_SIZE_sp_sleep_mode 4
#else
#error Symbol sp_sleep_mode occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x3464
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: 5E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stop_req 0x3488
#define HIVE_SIZE_isp_stop_req 4
#else
#error Symbol isp_stop_req occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x3488
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
