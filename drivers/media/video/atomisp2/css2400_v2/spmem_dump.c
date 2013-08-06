/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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
#define HIVE_ADDR_isp_vectors_per_input_line 0x2A90
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2A90
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 42F6 */

/* function input_system_acquisition_stop: 2D80 */

/* function longjmp: 4F00 */

/* function ia_css_sp_backend_push: 4830 */

/* function sp_tagger_tag_exp_id: 4239 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_MASK
#define HIVE_MEM_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_MASK 0x2E4
#define HIVE_SIZE_HIVE_IF_SRST_MASK 16
#else
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_MASK scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_MASK 0x2E4
#define HIVE_SIZE_sp_HIVE_IF_SRST_MASK 16

/* function sp_dma_proxy_read: 359B */

/* function ia_css_sp_backend_release: 4905 */

/* function sp_dma_proxy_is_idle: 36C1 */

/* function debug_buffer_set_ddr_addr: D2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_mipi
#define HIVE_MEM_vbuf_mipi scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_mipi 0xD4
#define HIVE_SIZE_vbuf_mipi 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_mipi scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_mipi 0xD4
#define HIVE_SIZE_sp_vbuf_mipi 4

/* function setjmp: 4F09 */

/* function decode_sw_event: 682 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_map
#define HIVE_MEM_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_map 0x3CD4
#define HIVE_SIZE_map 64
#else
#endif
#endif
#define HIVE_MEM_sp_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_map 0x3CD4
#define HIVE_SIZE_sp_map 64

/* function initialize_sp_stage: 2F39 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stage
#define HIVE_MEM_isp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stage 0x3998
#define HIVE_SIZE_isp_stage 564
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stage 0x3998
#define HIVE_SIZE_sp_isp_stage 564

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_raw
#define HIVE_MEM_vbuf_raw scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_raw 0xD0
#define HIVE_SIZE_vbuf_raw 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_raw scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_raw 0xD0
#define HIVE_SIZE_sp_vbuf_raw 4

/* function release_in_param: 4074 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_proxy_status
#define HIVE_MEM_dma_proxy_status scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_proxy_status 0x1A0
#define HIVE_SIZE_dma_proxy_status 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x1A0
#define HIVE_SIZE_sp_dma_proxy_status 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x2E80
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x2E80
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 2723 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_binary_group 0x27C8
#define HIVE_SIZE_sp_binary_group 72
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x27C8
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sw_state 0x2A94
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2A94
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 3D6A */

/* function sp_circular_buf_pop: 3D28 */

/* function generate_sw_interrupt: 467D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stop_copy_preview
#define HIVE_MEM_sp_stop_copy_preview scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stop_copy_preview 0x2E58
#define HIVE_SIZE_sp_stop_copy_preview 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stop_copy_preview scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_stop_copy_preview 0x2E58
#define HIVE_SIZE_sp_sp_stop_copy_preview 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3818
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3818
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

/* function sh_css_stride_from_info: 1419 */

/* function dma_proxy_dma_set_width_exception: 3624 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_sp_input_system_error_cnt
#define HIVE_MEM_ia_css_sp_input_system_error_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_sp_input_system_error_cnt 0x3CC8
#define HIVE_SIZE_ia_css_sp_input_system_error_cnt 12
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_sp_input_system_error_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_sp_input_system_error_cnt 0x3CC8
#define HIVE_SIZE_sp_ia_css_sp_input_system_error_cnt 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mipi_frame_sem
#define HIVE_MEM_mipi_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_mipi_frame_sem 0x26EC
#define HIVE_SIZE_mipi_frame_sem 20
#else
#endif
#endif
#define HIVE_MEM_sp_mipi_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_mipi_frame_sem 0x26EC
#define HIVE_SIZE_sp_mipi_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x204
#define HIVE_SIZE_sp_thread_ready_queue 12
#else
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x204
#define HIVE_SIZE_sp_sp_thread_ready_queue 12

/* function sp_debug_mode_update_command: 47B5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_is_pending_mask
#define HIVE_MEM_event_is_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_is_pending_mask 0x58
#define HIVE_SIZE_event_is_pending_mask 44
#else
#endif
#endif
#define HIVE_MEM_sp_event_is_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_is_pending_mask 0x58
#define HIVE_SIZE_sp_event_is_pending_mask 44

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_com
#define HIVE_MEM_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_com 0x2A98
#define HIVE_SIZE_host_sp_com 104
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2A98
#define HIVE_SIZE_sp_host_sp_com 104

/* function exec_image_pipe: 28CB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x2B00
#define HIVE_SIZE_sp_init_dmem_data 36
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x2B00
#define HIVE_SIZE_sp_sp_init_dmem_data 36

/* function stop_threads: 28C4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_use 0x11FC
#define HIVE_SIZE_sp_flash_in_use 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0x11FC
#define HIVE_SIZE_sp_sp_flash_in_use 4

/* function ia_css_sp_backend_rcv_acquire_ack: 47DC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_flashed_frame_cnt 0x1204
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0x1204
#define HIVE_SIZE_sp_flashed_frame_cnt 4

/* function is_isp_debug_buffer_full: 323 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_stack_sizes 0x234
#define HIVE_SIZE_stack_sizes 20
#else
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stack_sizes 0x234
#define HIVE_SIZE_sp_stack_sizes 20

/* function ia_css_sp_backend_destroy: 492F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_ph 0x2678
#define HIVE_SIZE_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ph 0x2678
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_writing_cb_params_preview_pipe
#define HIVE_MEM_sem_for_writing_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_writing_cb_params_preview_pipe 0x3828
#define HIVE_SIZE_sem_for_writing_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_writing_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_writing_cb_params_preview_pipe 0x3828
#define HIVE_SIZE_sp_sem_for_writing_cb_params_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_per_frame_data 0x2B24
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x2B24
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 5397 */

/* function sp_tagger_connect_pipes: 459D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x2E98
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x2E98
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_xmem_bin_addr 0x2B28
#define HIVE_SIZE_xmem_bin_addr 4
#else
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x2B28
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 4F93 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_threads 0x210
#define HIVE_SIZE_pipe_threads 16
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_threads 0x210
#define HIVE_SIZE_sp_pipe_threads 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x310
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x310
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 34DE */

/* function __divu: 4F11 */

/* function __ia_css_sp_input_system_isr_text: 4D0D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_stop
#define HIVE_MEM_sem_for_cont_capt_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_cont_capt_stop 0x383C
#define HIVE_SIZE_sem_for_cont_capt_stop 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_stop 0x383C
#define HIVE_SIZE_sp_sem_for_cont_capt_stop 20

/* function sp_dma_proxy_func: 5038 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_started 0x2B2C
#define HIVE_SIZE_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x2B2C
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x2ED8
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x2ED8
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x2B30
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x2B30
#define HIVE_SIZE_sp_sp_obarea_start_bq 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_IRQ_BASE
#define HIVE_MEM_IRQ_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_IRQ_BASE 0x28
#define HIVE_SIZE_IRQ_BASE 16
#else
#endif
#endif
#define HIVE_MEM_sp_IRQ_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_IRQ_BASE 0x28
#define HIVE_SIZE_sp_IRQ_BASE 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_TIMED_CTRL_BASE
#define HIVE_MEM_TIMED_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_TIMED_CTRL_BASE 0x3C
#define HIVE_SIZE_TIMED_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_TIMED_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_TIMED_CTRL_BASE 0x3C
#define HIVE_SIZE_sp_TIMED_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_is_isp_requested
#define HIVE_MEM_is_isp_requested scalar_processor_2400A0_dmem
#define HIVE_ADDR_is_isp_requested 0x11D4
#define HIVE_SIZE_is_isp_requested 4
#else
#endif
#endif
#define HIVE_MEM_sp_is_isp_requested scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_is_isp_requested 0x11D4
#define HIVE_SIZE_sp_is_isp_requested 4

/* function ia_css_i_sp_rmgr_init: 3C9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_thread 0x200
#define HIVE_SIZE_current_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x200
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x3038
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x3038
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x500
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x500
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function ia_css_sp_input_system_token_map_is_full: 4D80 */

/* function sp_turn_off_flash: 3858 */

/* function ia_css_sp_backend_pre_acquire_request: 481C */

/* function _sp_dma_proxy_init_isp_vector: 34F2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x2E84
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x2E84
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_start
#define HIVE_MEM_sem_for_cont_capt_start scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_cont_capt_start 0x3850
#define HIVE_SIZE_sem_for_cont_capt_start 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_start scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_start 0x3850
#define HIVE_SIZE_sp_sem_for_cont_capt_start 20

/* function host2sp_event_queue_is_empty: 4FD6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_pipe_start_semaphore
#define HIVE_MEM_sp_pipe_start_semaphore scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_start_semaphore 0x2F98
#define HIVE_SIZE_sp_pipe_start_semaphore 80
#else
#endif
#endif
#define HIVE_MEM_sp_sp_pipe_start_semaphore scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_pipe_start_semaphore 0x2F98
#define HIVE_SIZE_sp_sp_pipe_start_semaphore 80

/* function debug_buffer_init_isp: D9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x4FC
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x4FC
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x3864
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x3864
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 36D6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_fibers 0x248
#define HIVE_SIZE_fibers 20
#else
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_fibers 0x248
#define HIVE_SIZE_sp_fibers 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x3878
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3878
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 5620 */

/* function initialize_sp_group: 2F17 */

/* function start_binary: 14C5 */

/* function sp_tagger_configure: 4291 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x2E88
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x2E88
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_DMEM_BASE
#define HIVE_MEM_ISP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_DMEM_BASE 0x10
#define HIVE_SIZE_ISP_DMEM_BASE 4
#else
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
#endif
#endif
#define HIVE_MEM_sp_SP_DMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_SP_DMEM_BASE 0x4
#define HIVE_SIZE_sp_SP_DMEM_BASE 4

/* function ia_css_sp_frontend_stop: 4B49 */

/* function bin_spec_init_ifs: 75D */

/* function dma_proxy_channel_release: 5383 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_sp_frontend_states
#define HIVE_MEM_ia_css_sp_frontend_states scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_sp_frontend_states 0x3C80
#define HIVE_SIZE_ia_css_sp_frontend_states 12
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_sp_frontend_states scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_sp_frontend_states 0x3C80
#define HIVE_SIZE_sp_ia_css_sp_frontend_states 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_writing_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_writing_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_writing_cb_frames_preview_pipe 0x388C
#define HIVE_SIZE_sem_for_writing_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_writing_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_writing_cb_frames_preview_pipe 0x388C
#define HIVE_SIZE_sp_sem_for_writing_cb_frames_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_can_send_token_mask
#define HIVE_MEM_event_can_send_token_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_can_send_token_mask 0x84
#define HIVE_SIZE_event_can_send_token_mask 44
#else
#endif
#endif
#define HIVE_MEM_sp_event_can_send_token_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_can_send_token_mask 0x84
#define HIVE_SIZE_sp_event_can_send_token_mask 44

/* function sp_dma_proxy_wait_for_ack: 5294 */

/* function sp_thread_yield: 54DF */

/* function sp_circular_buf_peek: 3AEA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_thread
#define HIVE_MEM_isp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_thread 0x3BCC
#define HIVE_SIZE_isp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_thread 0x3BCC
#define HIVE_SIZE_sp_isp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x2B34
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x2B34
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 2BB */

/* function sp_uds_init: 97B */

/* function sp_dma_proxy_isp_write_addr: 3535 */

/* function sp_circular_buf_create: 3D76 */

/* function encode_and_post_sp_event: 474D */

/* function debug_enqueue_ddr: E3 */

/* function host2sp_dequeue_buffer: 3A6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_buf_swap 0x478
#define HIVE_SIZE_buf_swap 96
#else
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_buf_swap 0x478
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 3F8E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_curr_nr_of_copied_frames
#define HIVE_MEM_curr_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_curr_nr_of_copied_frames 0x2E5C
#define HIVE_SIZE_curr_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_curr_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_curr_nr_of_copied_frames 0x2E5C
#define HIVE_SIZE_sp_curr_nr_of_copied_frames 4

/* function sp2host_enqueue_buffer: 387 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x2750
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x2750
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

/* function sp_event_proxy_init: 36EB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x38A0
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x38A0
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_output 0x2B38
#define HIVE_SIZE_sp_output 16
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_output 0x2B38
#define HIVE_SIZE_sp_sp_output 16

/* function init_buffer_queues: 4226 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_CTRL_BASE
#define HIVE_MEM_ISP_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_ISP_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_CTRL_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_sp_ISP_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_INPUT_FORMATTER_BASE
#define HIVE_MEM_INPUT_FORMATTER_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_INPUT_FORMATTER_BASE 0x48
#define HIVE_SIZE_INPUT_FORMATTER_BASE 16
#else
#endif
#endif
#define HIVE_MEM_sp_INPUT_FORMATTER_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_INPUT_FORMATTER_BASE 0x48
#define HIVE_SIZE_sp_INPUT_FORMATTER_BASE 16

/* function sp_raw_copy_func: 2F7D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3804
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3804
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_tagger 0x2E60
#define HIVE_SIZE_tagger 24
#else
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_tagger 0x2E60
#define HIVE_SIZE_sp_tagger 24

/* function host2sp_dequeue_sp_event: 364 */

/* function sp_start_isp_entry: 2719 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x2719
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x2719

/* function __sp_raw_copy_func_critical: 4FE0 */

/* function add_sp_command: 5367 */

/* function sp2host_enqueue_irq_event: 347 */

/* function create_sp_fibers: 37C0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x3048
#define HIVE_SIZE_pipe_private_s3a_bufs 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x3048
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 48

/* function sp_debug_mode_init: 47BF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x2754
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x2754
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 6AC */

/* function ia_css_i_sp_rmgr_acq_gen: 423 */

/* function sp_turn_on_flash: 386C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x3078
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x3078
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x2B48
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x2B48
#define HIVE_SIZE_sp_isp_vectors_per_line 4

/* function __sp_dma_proxy_func_text: 33A0 */

/* function sp_thread_join: 3977 */

/* function sp_dma_proxy_configure_channel: 52C1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_irq_raise
#define HIVE_MEM_do_irq_raise scalar_processor_2400A0_dmem
#define HIVE_ADDR_do_irq_raise 0xD58
#define HIVE_SIZE_do_irq_raise 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_irq_raise scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_do_irq_raise 0xD58
#define HIVE_SIZE_sp_do_irq_raise 4

/* function ia_css_sp_backend_flush: 488D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_group 0x2B4C
#define HIVE_SIZE_sp_group 740
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_group 0x2B4C
#define HIVE_SIZE_sp_sp_group 740

/* function sp2host_buffer_queue_get_size: 37B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x2FE8
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x2FE8
#define HIVE_SIZE_sp_sp_event_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_pool
#define HIVE_MEM_dma_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_pool 0xC4
#define HIVE_SIZE_dma_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_pool 0xC4
#define HIVE_SIZE_sp_dma_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_uv_internal_width_vecs
#define HIVE_MEM_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x2E30
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x2E30
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x3104
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x3104
#define HIVE_SIZE_sp_pipe_private_exp_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MMU_BASE
#define HIVE_MEM_MMU_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_MMU_BASE 0x24
#define HIVE_SIZE_MMU_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_MMU_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_MMU_BASE 0x24
#define HIVE_SIZE_sp_MMU_BASE 4

/* function sp_dma_proxy_configure_init_dmem_channel: 35B4 */

/* function ia_css_sp_backend_create: 4940 */

/* function sp2host_event_queue_get_size: 33E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_sp_frontend_idle
#define HIVE_MEM_ia_css_sp_frontend_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_sp_frontend_idle 0x3C8C
#define HIVE_SIZE_ia_css_sp_frontend_idle 60
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_sp_frontend_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_sp_frontend_idle 0x3C8C
#define HIVE_SIZE_sp_ia_css_sp_frontend_idle 60

/* function isp_hmem_load: 4C6A */

/* function sp_dma_proxy_read_byte_addr: 52AB */

/* function sp_thread_fork: 3988 */

/* function sp_semaphore_wait: 559A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_buffer_ddr_address
#define HIVE_MEM_debug_buffer_ddr_address scalar_processor_2400A0_dmem
#define HIVE_ADDR_debug_buffer_ddr_address 0xB8
#define HIVE_SIZE_debug_buffer_ddr_address 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_buffer_ddr_address scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_debug_buffer_ddr_address 0xB8
#define HIVE_SIZE_sp_debug_buffer_ddr_address 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_mipi_exp_id
#define HIVE_MEM_ia_css_i_mipi_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_i_mipi_exp_id 0xD8
#define HIVE_SIZE_ia_css_i_mipi_exp_id 1
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_mipi_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_i_mipi_exp_id 0xD8
#define HIVE_SIZE_sp_ia_css_i_mipi_exp_id 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x38B0
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x38B0
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

/* function encode_sp_event: 4784 */

/* function sizeof_hmem: 4D06 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_request_flash 0x2E90
#define HIVE_SIZE_sp_request_flash 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x2E90
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 4795 */

/* function cnd_input_system_cfg: 2DC0 */

/* function sp_generate_events: 4693 */

/* function sp_uds_configure: 7E1 */

/* function sp_dma_proxy_execute: 357A */

/* function __modu: 4F57 */

/* function sp_circular_buf_push_marked: 3B5B */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x2E34
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x2E34
#define HIVE_SIZE_sp_isp_sdis_horiproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GDC_BASE
#define HIVE_MEM_GDC_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GDC_BASE 0x40
#define HIVE_SIZE_GDC_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_GDC_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GDC_BASE 0x40
#define HIVE_SIZE_sp_GDC_BASE 8

/* function sp_event_proxy_callout_func: 53BE */

/* function sp_fiber_init: 3847 */

/* function ia_css_sp_input_system_token_map_destroy: 4E8B */

/* function ia_css_i_sp_rmgr_uninit: 3C2 */

/* function wait_for_in_frame: 3FA9 */

/* function ia_css_sp_input_system_token_map_init: 4ECC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isys_token_handler
#define HIVE_MEM_sem_for_isys_token_handler scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isys_token_handler 0x38C4
#define HIVE_SIZE_sem_for_isys_token_handler 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isys_token_handler scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isys_token_handler 0x38C4
#define HIVE_SIZE_sp_sem_for_isys_token_handler 20

/* function sp_thread_init: 39AF */

/* function ia_css_sp_frontend_destroy: 4BF8 */

/* function irq_raise_set_token: AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GPIO_BASE
#define HIVE_MEM_GPIO_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_GPIO_BASE 0x38
#define HIVE_SIZE_GPIO_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GPIO_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GPIO_BASE 0x38
#define HIVE_SIZE_sp_GPIO_BASE 4

/* function _dma_proxy_dma_read_write: 532A */

/* function sp_dma_proxy_configure_init_vmem_channel: 35E4 */

/* function sp_acquire_dynamic_buf: 3EEA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_ph 0x26B4
#define HIVE_SIZE_isp_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_ph 0x26B4
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 45A7 */

/* function init_isp_internal_buffers: 15AF */

/* function dma_proxy_dma_set_addr_B: 3639 */

/* function ia_css_sp_input_system_token_map_snd_capture_req: 4E45 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_captured_frames
#define HIVE_MEM_target_nr_of_captured_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_target_nr_of_captured_frames 0x2E78
#define HIVE_SIZE_target_nr_of_captured_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_captured_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_target_nr_of_captured_frames 0x2E78
#define HIVE_SIZE_sp_target_nr_of_captured_frames 4

/* function sp_dma_proxy_write: 3561 */

/* function ___dma_proxy_dma_read_write_text: 36A3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp_group
#define HIVE_MEM_sem_for_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_sp_group 0x38D8
#define HIVE_SIZE_sem_for_sp_group 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_sp_group 0x38D8
#define HIVE_SIZE_sp_sem_for_sp_group 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_raw_exp_id
#define HIVE_MEM_ia_css_i_raw_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_ia_css_i_raw_exp_id 0x31C
#define HIVE_SIZE_ia_css_i_raw_exp_id 1
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_raw_exp_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ia_css_i_raw_exp_id 0x31C
#define HIVE_SIZE_sp_ia_css_i_raw_exp_id 1

/* function ia_css_i_sp_refcount_init_vbuf: 464 */

/* function isp_hmem_clear: 4C3B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x38EC
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x38EC
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function input_system_acquisition_run: 2D9E */

/* function sp_dma_proxy_vmem_read: 351E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3900
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3900
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

/* function release_in_frame: 3FE5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3914
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3914
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x2E38
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x2E38
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

/* function wait_for_in_param: 4012 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3928
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3928
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queue 0x3108
#define HIVE_SIZE_host_sp_queue 1036
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x3108
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 39C7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_pool
#define HIVE_MEM_isp_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_pool 0xC8
#define HIVE_SIZE_isp_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pool 0xC8
#define HIVE_SIZE_sp_isp_pool 4

/* function dma_proxy_dma_set_height_exception: 3616 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_busy_frame
#define HIVE_MEM_busy_frame scalar_processor_2400A0_dmem
#define HIVE_ADDR_busy_frame 0x3D14
#define HIVE_SIZE_busy_frame 16
#else
#endif
#endif
#define HIVE_MEM_sp_busy_frame scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_busy_frame 0x3D14
#define HIVE_SIZE_sp_busy_frame 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_is_done_flag
#define HIVE_MEM_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_is_done_flag 0xD54
#define HIVE_SIZE_isp_is_done_flag 1
#else
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0xD54
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_any_pending_mask 0x314
#define HIVE_SIZE_event_any_pending_mask 8
#else
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x314
#define HIVE_SIZE_sp_event_any_pending_mask 8

/* function ia_css_sp_frontend_has_empty_mipi_buffer_cb: 4974 */

/* function ia_css_i_sp_refcount_release_vbuf: 503 */

/* function init_isp_data_segment: 165D */

/* function ia_css_sp_frontend_start: 4B77 */

/* function sh_css_decode_tag_descr: 642 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_configs 0x2700
#define HIVE_SIZE_dma_configs 80
#else
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_configs 0x2700
#define HIVE_SIZE_sp_dma_configs 80

/* function debug_enqueue_isp: 26A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x2758
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x2758
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SWITCH_CODE
#define HIVE_MEM_HIVE_IF_SWITCH_CODE scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SWITCH_CODE 0x2F4
#define HIVE_SIZE_HIVE_IF_SWITCH_CODE 4
#else
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SWITCH_CODE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SWITCH_CODE 0x2F4
#define HIVE_SIZE_sp_HIVE_IF_SWITCH_CODE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x393C
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x393C
#define HIVE_SIZE_sp_sem_for_isp_idle 20

/* function sp_thread_get_state: 38FB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_channels 0x2694
#define HIVE_SIZE_channels 32
#else
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_channels 0x2694
#define HIVE_SIZE_sp_channels 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x2E3C
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x2E3C
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x2E40
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x2E40
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 9C1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_pipe_stop
#define HIVE_MEM_sp_pipe_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_stop 0x3028
#define HIVE_SIZE_sp_pipe_stop 16
#else
#endif
#endif
#define HIVE_MEM_sp_sp_pipe_stop scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_pipe_stop 0x3028
#define HIVE_SIZE_sp_sp_pipe_stop 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_VAMEM_BASE
#define HIVE_MEM_ISP_VAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_ISP_VAMEM_BASE 12
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_VAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_sp_ISP_VAMEM_BASE 12

/* function handle_parameter_sets: 40A8 */

/* function ia_css_sp_frontend_create: 4C0E */

/* function sp_tagger_create: 45C8 */

/* function sp_dma_proxy_vmem_write: 3508 */

/* function sp_thread_set_priority: 3946 */

/* function ia_css_sp_input_system_token_map_get_exp_id: 4E24 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3514
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3514
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_callout_sp_thread
#define HIVE_MEM_callout_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_callout_sp_thread 0x2E94
#define HIVE_SIZE_callout_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_callout_sp_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_callout_sp_thread 0x2E94
#define HIVE_SIZE_sp_callout_sp_thread 4

/* function sp_semaphore_signal: 5554 */

/* function ia_css_sp_input_system_token_map_snd_acquire_req: 4E0D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_ref_out_buf
#define HIVE_MEM_sp_ref_out_buf scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ref_out_buf 0x45C
#define HIVE_SIZE_sp_ref_out_buf 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_ref_out_buf scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_ref_out_buf 0x45C
#define HIVE_SIZE_sp_sp_ref_out_buf 4

/* function sp_dma_proxy_write_byte_addr: 354B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_PMEM_BASE
#define HIVE_MEM_SP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_SP_PMEM_BASE 0x0
#define HIVE_SIZE_SP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_SP_PMEM_BASE 0x0
#define HIVE_SIZE_sp_SP_PMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_input_stream_format
#define HIVE_MEM_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x2E44
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x2E44
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 3A58 */

/* function __mod: 4F43 */

/* function __sp_event_proxy_func_critical: 53AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_curr_binary_id
#define HIVE_MEM_curr_binary_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_curr_binary_id 0x154
#define HIVE_SIZE_curr_binary_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_curr_binary_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_curr_binary_id 0x154
#define HIVE_SIZE_sp_curr_binary_id 4

/* function sp_circular_buf_mark: 3C3F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_copied_frames
#define HIVE_MEM_target_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_target_nr_of_copied_frames 0x2E7C
#define HIVE_SIZE_target_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_copied_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_target_nr_of_copied_frames 0x2E7C
#define HIVE_SIZE_sp_target_nr_of_copied_frames 4

/* function ia_css_sp_input_system_token_map_reset_capturing_buffer_on_error: 4D56 */

/* function irq_raise: BD */

/* function sp_circular_buf_unmark: 3C17 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 4BF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_HIVE_IF_SRST_ADDRESS
#define HIVE_MEM_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_HIVE_IF_SRST_ADDRESS 0x2D4
#define HIVE_SIZE_HIVE_IF_SRST_ADDRESS 16
#else
#endif
#endif
#define HIVE_MEM_sp_HIVE_IF_SRST_ADDRESS scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_HIVE_IF_SRST_ADDRESS 0x2D4
#define HIVE_SIZE_sp_HIVE_IF_SRST_ADDRESS 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3950
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3950
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3960
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3960
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

/* function set_sp_sleep_for_debug: 47CA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_fiber 0x11F0
#define HIVE_SIZE_current_sp_fiber 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0x11F0
#define HIVE_SIZE_sp_current_sp_fiber 4

/* function ia_css_sp_input_system_token_map_create: 4EF9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x275C
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x275C
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3974
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3974
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 34AF */

/* function sp_release_dynamic_buf: 3D8D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_HMEM_BASE
#define HIVE_MEM_ISP_HMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_ISP_HMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_HMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_sp_ISP_HMEM_BASE 4

/* function ia_css_sp_input_system_token_map_flush: 4E6E */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 4EA */

/* function ia_css_sp_input_system_isr: 5632 */

/* function end_binary: 1428 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_stacks 0x220
#define HIVE_SIZE_stacks 20
#else
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stacks 0x220
#define HIVE_SIZE_sp_stacks 20

/* function ia_css_i_sp_refcount_dump: 49A */

/* function ia_css_i_sp_rmgr_rel_gen: 40B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x2674
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x2674
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 394E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x3534
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x3534
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_addresses 0x3BD0
#define HIVE_SIZE_sp_isp_addresses 176
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x3BD0
#define HIVE_SIZE_sp_sp_isp_addresses 176

/* function sp_fiber_main: 3851 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_isps 0x26D0
#define HIVE_SIZE_isps 28
#else
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isps 0x26D0
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x2E48
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x2E48
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function ia_css_sp_backend_acquire: 491E */

/* function dma_proxy_dma_set_increments: 362B */

/* function __sp_dma_proxy_wait_for_ack_text: 34D5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_spref
#define HIVE_MEM_vbuf_spref scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_spref 0xCC
#define HIVE_SIZE_vbuf_spref 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_spref scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_spref 0xCC
#define HIVE_SIZE_sp_vbuf_spref 4

/* function sp_circular_buf_extract: 3C67 */

/* function output_compute_dma_info: 212C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_ref_in_buf
#define HIVE_MEM_sp_ref_in_buf scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ref_in_buf 0x458
#define HIVE_SIZE_sp_ref_in_buf 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_ref_in_buf scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_ref_in_buf 0x458
#define HIVE_SIZE_sp_sp_ref_in_buf 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x2E4C
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x2E4C
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3984
#define HIVE_SIZE_sem_for_reading_if 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3984
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function sp_circular_buf_pop_marked: 3B0F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_frames 0x35A4
#define HIVE_SIZE_pipe_private_frames 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x35A4
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function ia_css_sp_frontend_rcv_capture_ack: 4A2F */

/* function sp_generate_interrupts: 45F0 */

/* function init_isp_vars: 23EF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x35D4
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x35D4
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_data 0x2810
#define HIVE_SIZE_sp_data 640
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_data 0x2810
#define HIVE_SIZE_sp_sp_data 640

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x30C
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x30C
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function sp_circular_buf_is_marked: 3BF2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_mem_map 0x2760
#define HIVE_SIZE_mem_map 104
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_mem_map 0x2760
#define HIVE_SIZE_sp_mem_map 104

/* function sp_init_dmem: 2EAD */

/* function ia_css_i_sp_refcount_retain_vbuf: 524 */

/* function init_isp_code_segment: 14E8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function run_sp_threads: 39DE */

/* function ia_css_sp_backend_snd_acquire_request: 47F6 */

/* function sp_thread_queue_print: 39FB */

/* function done_isp_data_segment: 163F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_service 0x1200
#define HIVE_SIZE_sp_flash_in_service 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0x1200
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x2E50
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x2E50
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 3A14 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sleep_mode 0x2E54
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x2E54
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: C8 */

/* function ia_css_sp_input_system_token_map_receive_ack: 4DB2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stop_req 0x2E8C
#define HIVE_SIZE_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x2E8C
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
extern void sh_css_dump_sp_init_dmem_data (void);
void sh_css_dump_sp_init_dmem_data (void)
{
  static struct ia_css_sp_init_dmem_cfg sp_init_dmem_data;
  assert(sizeof(sp_init_dmem_data) ==
         HIVE_SIZE_sp_init_dmem_data);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_init_dmem_data),
    (char*)&sp_init_dmem_data,
    sizeof(sp_init_dmem_data));
  sh_css_print("sp_init_dmem_data.done ="
    "                                  0x%x\n",
    sp_init_dmem_data.done);
  sh_css_print("sp_init_dmem_data.ddr_code_addr ="
    "                         0x%x\n",
    sp_init_dmem_data.ddr_code_addr);
  sh_css_print("sp_init_dmem_data.ddr_data_addr ="
    "                         0x%x\n",
    sp_init_dmem_data.ddr_data_addr);
  sh_css_print("sp_init_dmem_data.pmem_text_addr ="
    "                        0x%x\n",
    sp_init_dmem_data.pmem_text_addr);
  sh_css_print("sp_init_dmem_data.dmem_data_addr ="
    "                        0x%x\n",
    sp_init_dmem_data.dmem_data_addr);
  sh_css_print("sp_init_dmem_data.dmem_bss_addr ="
    "                         0x%x\n",
    sp_init_dmem_data.dmem_bss_addr);
  sh_css_print("sp_init_dmem_data.text_size ="
    "                             0x%x\n",
    sp_init_dmem_data.text_size);
  sh_css_print("sp_init_dmem_data.data_size ="
    "                             0x%x\n",
    sp_init_dmem_data.data_size);
  sh_css_print("sp_init_dmem_data.bss_size ="
    "                              0x%x\n",
    sp_init_dmem_data.bss_size);
}

extern void sh_css_dump_xmem_bin_addr (void);
void sh_css_dump_xmem_bin_addr (void)
{
  hrt_vaddress xmem_bin_addr;
  assert(sizeof(xmem_bin_addr) ==
         HIVE_SIZE_xmem_bin_addr);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(xmem_bin_addr),
    (char*)&xmem_bin_addr,
    sizeof(xmem_bin_addr));
  sh_css_print("xmem_bin_addr ="
    "                                           0x%x\n",
    xmem_bin_addr);
}

extern void sh_css_dump_sp_vf_downscale_bits (void);
void sh_css_dump_sp_vf_downscale_bits (void)
{
  unsigned int sp_vf_downscale_bits;
  assert(sizeof(sp_vf_downscale_bits) ==
         HIVE_SIZE_sp_vf_downscale_bits);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_vf_downscale_bits),
    (char*)&sp_vf_downscale_bits,
    sizeof(sp_vf_downscale_bits));
  sh_css_print("sp_vf_downscale_bits ="
    "                                    0x%x\n",
    sp_vf_downscale_bits);
}

extern void sh_css_dump_sp_per_frame_data (void);
void sh_css_dump_sp_per_frame_data (void)
{
  static struct sh_css_sp_per_frame_data sp_per_frame_data;
  assert(sizeof(sp_per_frame_data) ==
         HIVE_SIZE_sp_per_frame_data);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_per_frame_data),
    (char*)&sp_per_frame_data,
    sizeof(sp_per_frame_data));
  sh_css_print("sp_per_frame_data.sp_group_addr ="
    "                         0x%x\n",
    sp_per_frame_data.sp_group_addr);
}

extern void sh_css_dump_sp_group (void);
void sh_css_dump_sp_group (void)
{
  static struct sh_css_sp_group sp_group;
  assert(sizeof(sp_group) ==
         HIVE_SIZE_sp_group);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_group),
    (char*)&sp_group,
    sizeof(sp_group));
  sh_css_print("sp_group.config.is_offline ="
    "                              0x%x\n",
    sp_group.config.is_offline);
  sh_css_print("sp_group.config.input_needs_raw_binning ="
    "                 0x%x\n",
    sp_group.config.input_needs_raw_binning);
  sh_css_print("sp_group.config.no_isp_sync ="
    "                             0x%x\n",
    sp_group.config.no_isp_sync);
  sh_css_print("sp_group.config.input_formatter.a_changed ="
    "               0x%x\n",
    sp_group.config.input_formatter.a_changed);
  sh_css_print("sp_group.config.input_formatter.b_changed ="
    "               0x%x\n",
    sp_group.config.input_formatter.b_changed);
  sh_css_print("sp_group.config.input_formatter.isp_2ppc ="
    "                0x%x\n",
    sp_group.config.input_formatter.isp_2ppc);
  sh_css_print("sp_group.config.input_formatter.set[0].stream_format ="
    "    0x%x\n",
    sp_group.config.input_formatter.set[0].stream_format);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.start_line);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.start_column);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[0].config_a.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_a.block_no_reqs);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.start_line);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.start_column);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[0].config_b.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[0].config_b.block_no_reqs);
  sh_css_print("sp_group.config.input_formatter.set[1].stream_format ="
    "    0x%x\n",
    sp_group.config.input_formatter.set[1].stream_format);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.start_line);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.start_column);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[1].config_a.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_a.block_no_reqs);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.start_line);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.start_column);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[1].config_b.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[1].config_b.block_no_reqs);
  sh_css_print("sp_group.config.input_formatter.set[2].stream_format ="
    "    0x%x\n",
    sp_group.config.input_formatter.set[2].stream_format);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.start_line);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.start_column);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[2].config_a.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_a.block_no_reqs);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.start_line ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.start_line);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.start_column ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.start_column);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.left_padding ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.left_padding);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.cropped_height ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.cropped_height);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.cropped_width ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.cropped_width);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.deinterleaving ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.deinterleaving);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.buf_vecs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.buf_vecs);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.buf_start_index ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.buf_start_index);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.buf_increment ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.buf_increment);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.buf_eol_offset ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.buf_eol_offset);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.is_yuv420_format ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.is_yuv420_format);
  sh_css_print("sp_group.config.input_formatter.set[2].config_b.block_no_reqs ="
    "0x%x\n",
    sp_group.config.input_formatter.set[2].config_b.block_no_reqs);
  sh_css_print("sp_group.config.sync_gen.width ="
    "                          0x%x\n",
    sp_group.config.sync_gen.width);
  sh_css_print("sp_group.config.sync_gen.height ="
    "                         0x%x\n",
    sp_group.config.sync_gen.height);
  sh_css_print("sp_group.config.sync_gen.hblank_cycles ="
    "                  0x%x\n",
    sp_group.config.sync_gen.hblank_cycles);
  sh_css_print("sp_group.config.sync_gen.vblank_cycles ="
    "                  0x%x\n",
    sp_group.config.sync_gen.vblank_cycles);
  sh_css_print("sp_group.config.tpg.x_mask ="
    "                              0x%x\n",
    sp_group.config.tpg.x_mask);
  sh_css_print("sp_group.config.tpg.y_mask ="
    "                              0x%x\n",
    sp_group.config.tpg.y_mask);
  sh_css_print("sp_group.config.tpg.x_delta ="
    "                             0x%x\n",
    sp_group.config.tpg.x_delta);
  sh_css_print("sp_group.config.tpg.y_delta ="
    "                             0x%x\n",
    sp_group.config.tpg.y_delta);
  sh_css_print("sp_group.config.tpg.xy_mask ="
    "                             0x%x\n",
    sp_group.config.tpg.xy_mask);
  sh_css_print("sp_group.config.tpg.sync_gen_cfg.width ="
    "                  0x%x\n",
    sp_group.config.tpg.sync_gen_cfg.width);
  sh_css_print("sp_group.config.tpg.sync_gen_cfg.height ="
    "                 0x%x\n",
    sp_group.config.tpg.sync_gen_cfg.height);
  sh_css_print("sp_group.config.tpg.sync_gen_cfg.hblank_cycles ="
    "          0x%x\n",
    sp_group.config.tpg.sync_gen_cfg.hblank_cycles);
  sh_css_print("sp_group.config.tpg.sync_gen_cfg.vblank_cycles ="
    "          0x%x\n",
    sp_group.config.tpg.sync_gen_cfg.vblank_cycles);
  sh_css_print("sp_group.config.prbs.seed ="
    "                               0x%x\n",
    sp_group.config.prbs.seed);
  sh_css_print("sp_group.config.prbs.sync_gen_cfg.width ="
    "                 0x%x\n",
    sp_group.config.prbs.sync_gen_cfg.width);
  sh_css_print("sp_group.config.prbs.sync_gen_cfg.height ="
    "                0x%x\n",
    sp_group.config.prbs.sync_gen_cfg.height);
  sh_css_print("sp_group.config.prbs.sync_gen_cfg.hblank_cycles ="
    "         0x%x\n",
    sp_group.config.prbs.sync_gen_cfg.hblank_cycles);
  sh_css_print("sp_group.config.prbs.sync_gen_cfg.vblank_cycles ="
    "         0x%x\n",
    sp_group.config.prbs.sync_gen_cfg.vblank_cycles);
  sh_css_print("sp_group.config.input_circuit.no_side_band ="
    "              0x%x\n",
    sp_group.config.input_circuit.no_side_band);
  sh_css_print("sp_group.config.input_circuit.fmt_type ="
    "                  0x%x\n",
    sp_group.config.input_circuit.fmt_type);
  sh_css_print("sp_group.config.input_circuit.ch_id ="
    "                     0x%x\n",
    sp_group.config.input_circuit.ch_id);
  sh_css_print("sp_group.config.input_circuit.input_mode ="
    "                0x%x\n",
    sp_group.config.input_circuit.input_mode);
  sh_css_print("sp_group.config.input_circuit_cfg_changed ="
    "               0x%x\n",
    sp_group.config.input_circuit_cfg_changed);
  sh_css_print("sp_group.pipe[0].pipe_id ="
    "                                0x%x\n",
    sp_group.pipe[0].pipe_id);
  sh_css_print("sp_group.pipe[0].pipe_num ="
    "                               0x%x\n",
    sp_group.pipe[0].pipe_num);
  sh_css_print("sp_group.pipe[0].thread_id ="
    "                              0x%x\n",
    sp_group.pipe[0].thread_id);
  sh_css_print("sp_group.pipe[0].pipe_config ="
    "                            0x%x\n",
    sp_group.pipe[0].pipe_config);
  sh_css_print("sp_group.pipe[0].input_system_mode ="
    "                      0x%x\n",
    sp_group.pipe[0].input_system_mode);
  sh_css_print("sp_group.pipe[0].port_id ="
    "                                0x%x\n",
    sp_group.pipe[0].port_id);
  sh_css_print("sp_group.pipe[0].num_stages ="
    "                             0x%x\n",
    sp_group.pipe[0].num_stages);
  sh_css_print("sp_group.pipe[0].running ="
    "                                0x%x\n",
    sp_group.pipe[0].running);
  sh_css_print("sp_group.pipe[0].dvs_frame_delay ="
    "                        0x%x\n",
    sp_group.pipe[0].dvs_frame_delay);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[0] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[0]);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[1] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[1]);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[2] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[2]);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[3] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[3]);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[4] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[4]);
  sh_css_print("sp_group.pipe[0].sp_stage_addr[5] ="
    "                       0x%x\n",
    sp_group.pipe[0].sp_stage_addr[5]);
  sh_css_print("sp_group.pipe[0].stage ="
    "                                    %p\n",
    sp_group.pipe[0].stage);
  sh_css_print("sp_group.pipe[0].copy.bin.bytes_available ="
    "               0x%x\n",
    sp_group.pipe[0].copy.bin.bytes_available);
  sh_css_print("sp_group.pipe[0].copy.raw.height ="
    "                        0x%x\n",
    sp_group.pipe[0].copy.raw.height);
  sh_css_print("sp_group.pipe[0].copy.raw.width ="
    "                         0x%x\n",
    sp_group.pipe[0].copy.raw.width);
  sh_css_print("sp_group.pipe[0].copy.raw.padded_width ="
    "                  0x%x\n",
    sp_group.pipe[0].copy.raw.padded_width);
  sh_css_print("sp_group.pipe[0].copy.raw.max_input_width ="
    "               0x%x\n",
    sp_group.pipe[0].copy.raw.max_input_width);
  sh_css_print("sp_group.pipe[0].copy.raw.raw_bit_depth ="
    "                 0x%x\n",
    sp_group.pipe[0].copy.raw.raw_bit_depth);
  sh_css_print("sp_group.pipe[1].pipe_id ="
    "                                0x%x\n",
    sp_group.pipe[1].pipe_id);
  sh_css_print("sp_group.pipe[1].pipe_num ="
    "                               0x%x\n",
    sp_group.pipe[1].pipe_num);
  sh_css_print("sp_group.pipe[1].thread_id ="
    "                              0x%x\n",
    sp_group.pipe[1].thread_id);
  sh_css_print("sp_group.pipe[1].pipe_config ="
    "                            0x%x\n",
    sp_group.pipe[1].pipe_config);
  sh_css_print("sp_group.pipe[1].input_system_mode ="
    "                      0x%x\n",
    sp_group.pipe[1].input_system_mode);
  sh_css_print("sp_group.pipe[1].port_id ="
    "                                0x%x\n",
    sp_group.pipe[1].port_id);
  sh_css_print("sp_group.pipe[1].num_stages ="
    "                             0x%x\n",
    sp_group.pipe[1].num_stages);
  sh_css_print("sp_group.pipe[1].running ="
    "                                0x%x\n",
    sp_group.pipe[1].running);
  sh_css_print("sp_group.pipe[1].dvs_frame_delay ="
    "                        0x%x\n",
    sp_group.pipe[1].dvs_frame_delay);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[0] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[0]);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[1] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[1]);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[2] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[2]);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[3] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[3]);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[4] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[4]);
  sh_css_print("sp_group.pipe[1].sp_stage_addr[5] ="
    "                       0x%x\n",
    sp_group.pipe[1].sp_stage_addr[5]);
  sh_css_print("sp_group.pipe[1].stage ="
    "                                    %p\n",
    sp_group.pipe[1].stage);
  sh_css_print("sp_group.pipe[1].copy.bin.bytes_available ="
    "               0x%x\n",
    sp_group.pipe[1].copy.bin.bytes_available);
  sh_css_print("sp_group.pipe[1].copy.raw.height ="
    "                        0x%x\n",
    sp_group.pipe[1].copy.raw.height);
  sh_css_print("sp_group.pipe[1].copy.raw.width ="
    "                         0x%x\n",
    sp_group.pipe[1].copy.raw.width);
  sh_css_print("sp_group.pipe[1].copy.raw.padded_width ="
    "                  0x%x\n",
    sp_group.pipe[1].copy.raw.padded_width);
  sh_css_print("sp_group.pipe[1].copy.raw.max_input_width ="
    "               0x%x\n",
    sp_group.pipe[1].copy.raw.max_input_width);
  sh_css_print("sp_group.pipe[1].copy.raw.raw_bit_depth ="
    "                 0x%x\n",
    sp_group.pipe[1].copy.raw.raw_bit_depth);
  sh_css_print("sp_group.pipe[2].pipe_id ="
    "                                0x%x\n",
    sp_group.pipe[2].pipe_id);
  sh_css_print("sp_group.pipe[2].pipe_num ="
    "                               0x%x\n",
    sp_group.pipe[2].pipe_num);
  sh_css_print("sp_group.pipe[2].thread_id ="
    "                              0x%x\n",
    sp_group.pipe[2].thread_id);
  sh_css_print("sp_group.pipe[2].pipe_config ="
    "                            0x%x\n",
    sp_group.pipe[2].pipe_config);
  sh_css_print("sp_group.pipe[2].input_system_mode ="
    "                      0x%x\n",
    sp_group.pipe[2].input_system_mode);
  sh_css_print("sp_group.pipe[2].port_id ="
    "                                0x%x\n",
    sp_group.pipe[2].port_id);
  sh_css_print("sp_group.pipe[2].num_stages ="
    "                             0x%x\n",
    sp_group.pipe[2].num_stages);
  sh_css_print("sp_group.pipe[2].running ="
    "                                0x%x\n",
    sp_group.pipe[2].running);
  sh_css_print("sp_group.pipe[2].dvs_frame_delay ="
    "                        0x%x\n",
    sp_group.pipe[2].dvs_frame_delay);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[0] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[0]);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[1] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[1]);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[2] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[2]);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[3] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[3]);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[4] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[4]);
  sh_css_print("sp_group.pipe[2].sp_stage_addr[5] ="
    "                       0x%x\n",
    sp_group.pipe[2].sp_stage_addr[5]);
  sh_css_print("sp_group.pipe[2].stage ="
    "                                    %p\n",
    sp_group.pipe[2].stage);
  sh_css_print("sp_group.pipe[2].copy.bin.bytes_available ="
    "               0x%x\n",
    sp_group.pipe[2].copy.bin.bytes_available);
  sh_css_print("sp_group.pipe[2].copy.raw.height ="
    "                        0x%x\n",
    sp_group.pipe[2].copy.raw.height);
  sh_css_print("sp_group.pipe[2].copy.raw.width ="
    "                         0x%x\n",
    sp_group.pipe[2].copy.raw.width);
  sh_css_print("sp_group.pipe[2].copy.raw.padded_width ="
    "                  0x%x\n",
    sp_group.pipe[2].copy.raw.padded_width);
  sh_css_print("sp_group.pipe[2].copy.raw.max_input_width ="
    "               0x%x\n",
    sp_group.pipe[2].copy.raw.max_input_width);
  sh_css_print("sp_group.pipe[2].copy.raw.raw_bit_depth ="
    "                 0x%x\n",
    sp_group.pipe[2].copy.raw.raw_bit_depth);
  sh_css_print("sp_group.pipe[3].pipe_id ="
    "                                0x%x\n",
    sp_group.pipe[3].pipe_id);
  sh_css_print("sp_group.pipe[3].pipe_num ="
    "                               0x%x\n",
    sp_group.pipe[3].pipe_num);
  sh_css_print("sp_group.pipe[3].thread_id ="
    "                              0x%x\n",
    sp_group.pipe[3].thread_id);
  sh_css_print("sp_group.pipe[3].pipe_config ="
    "                            0x%x\n",
    sp_group.pipe[3].pipe_config);
  sh_css_print("sp_group.pipe[3].input_system_mode ="
    "                      0x%x\n",
    sp_group.pipe[3].input_system_mode);
  sh_css_print("sp_group.pipe[3].port_id ="
    "                                0x%x\n",
    sp_group.pipe[3].port_id);
  sh_css_print("sp_group.pipe[3].num_stages ="
    "                             0x%x\n",
    sp_group.pipe[3].num_stages);
  sh_css_print("sp_group.pipe[3].running ="
    "                                0x%x\n",
    sp_group.pipe[3].running);
  sh_css_print("sp_group.pipe[3].dvs_frame_delay ="
    "                        0x%x\n",
    sp_group.pipe[3].dvs_frame_delay);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[0] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[0]);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[1] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[1]);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[2] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[2]);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[3] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[3]);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[4] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[4]);
  sh_css_print("sp_group.pipe[3].sp_stage_addr[5] ="
    "                       0x%x\n",
    sp_group.pipe[3].sp_stage_addr[5]);
  sh_css_print("sp_group.pipe[3].stage ="
    "                                    %p\n",
    sp_group.pipe[3].stage);
  sh_css_print("sp_group.pipe[3].copy.bin.bytes_available ="
    "               0x%x\n",
    sp_group.pipe[3].copy.bin.bytes_available);
  sh_css_print("sp_group.pipe[3].copy.raw.height ="
    "                        0x%x\n",
    sp_group.pipe[3].copy.raw.height);
  sh_css_print("sp_group.pipe[3].copy.raw.width ="
    "                         0x%x\n",
    sp_group.pipe[3].copy.raw.width);
  sh_css_print("sp_group.pipe[3].copy.raw.padded_width ="
    "                  0x%x\n",
    sp_group.pipe[3].copy.raw.padded_width);
  sh_css_print("sp_group.pipe[3].copy.raw.max_input_width ="
    "               0x%x\n",
    sp_group.pipe[3].copy.raw.max_input_width);
  sh_css_print("sp_group.pipe[3].copy.raw.raw_bit_depth ="
    "                 0x%x\n",
    sp_group.pipe[3].copy.raw.raw_bit_depth);
  sh_css_print("sp_group.debug.dma_sw_reg ="
    "                               0x%x\n",
    sp_group.debug.dma_sw_reg);
}

extern void sh_css_dump_sp_output (void);
void sh_css_dump_sp_output (void)
{
  static struct sh_css_sp_output sp_output;
  assert(sizeof(sp_output) ==
         HIVE_SIZE_sp_output);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_output),
    (char*)&sp_output,
    sizeof(sp_output));
  sh_css_print("sp_output.bin_copy_bytes_copied ="
    "                         0x%x\n",
    sp_output.bin_copy_bytes_copied);
  sh_css_print("sp_output.sw_interrupt_value[0] ="
    "                         0x%x\n",
    sp_output.sw_interrupt_value[0]);
  sh_css_print("sp_output.sw_interrupt_value[1] ="
    "                         0x%x\n",
    sp_output.sw_interrupt_value[1]);
  sh_css_print("sp_output.sw_interrupt_value[2] ="
    "                         0x%x\n",
    sp_output.sw_interrupt_value[2]);
}

extern void sh_css_dump_host_sp_com (void);
void sh_css_dump_host_sp_com (void)
{
  static volatile struct host_sp_communication host_sp_com;
  assert(sizeof(host_sp_com) ==
         HIVE_SIZE_host_sp_com);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(host_sp_com),
    (char*)&host_sp_com,
    sizeof(host_sp_com));
  sh_css_print("host_sp_com.host2sp_command ="
    "                             0x%x\n",
    host_sp_com.host2sp_command);
  sh_css_print("host_sp_com.host2sp_offline_frames[0] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[0]);
  sh_css_print("host_sp_com.host2sp_offline_frames[1] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[1]);
  sh_css_print("host_sp_com.host2sp_offline_frames[2] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[2]);
  sh_css_print("host_sp_com.host2sp_offline_frames[3] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[3]);
  sh_css_print("host_sp_com.host2sp_offline_frames[4] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[4]);
  sh_css_print("host_sp_com.host2sp_offline_frames[5] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[5]);
  sh_css_print("host_sp_com.host2sp_offline_frames[6] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[6]);
  sh_css_print("host_sp_com.host2sp_offline_frames[7] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[7]);
  sh_css_print("host_sp_com.host2sp_offline_frames[8] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[8]);
  sh_css_print("host_sp_com.host2sp_offline_frames[9] ="
    "                   0x%x\n",
    host_sp_com.host2sp_offline_frames[9]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[0] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[0]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[1] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[1]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[2] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[2]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[3] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[3]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[4] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[4]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[5] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[5]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[6] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[6]);
  sh_css_print("host_sp_com.host2sp_mipi_frames[7] ="
    "                      0x%x\n",
    host_sp_com.host2sp_mipi_frames[7]);
  sh_css_print("host_sp_com.host2sp_cont_num_raw_frames ="
    "                 0x%x\n",
    host_sp_com.host2sp_cont_num_raw_frames);
  sh_css_print("host_sp_com.host2sp_cont_num_mipi_frames ="
    "                0x%x\n",
    host_sp_com.host2sp_cont_num_mipi_frames);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[0].or_mask ="
    "           0x%x\n",
    host_sp_com.host2sp_event_irq_mask[0].or_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[0].and_mask ="
    "          0x%x\n",
    host_sp_com.host2sp_event_irq_mask[0].and_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[1].or_mask ="
    "           0x%x\n",
    host_sp_com.host2sp_event_irq_mask[1].or_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[1].and_mask ="
    "          0x%x\n",
    host_sp_com.host2sp_event_irq_mask[1].and_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[2].or_mask ="
    "           0x%x\n",
    host_sp_com.host2sp_event_irq_mask[2].or_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[2].and_mask ="
    "          0x%x\n",
    host_sp_com.host2sp_event_irq_mask[2].and_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[3].or_mask ="
    "           0x%x\n",
    host_sp_com.host2sp_event_irq_mask[3].or_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[3].and_mask ="
    "          0x%x\n",
    host_sp_com.host2sp_event_irq_mask[3].and_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[4].or_mask ="
    "           0x%x\n",
    host_sp_com.host2sp_event_irq_mask[4].or_mask);
  sh_css_print("host_sp_com.host2sp_event_irq_mask[4].and_mask ="
    "          0x%x\n",
    host_sp_com.host2sp_event_irq_mask[4].and_mask);
}

extern void sh_css_dump_curr_binary_id (void);
void sh_css_dump_curr_binary_id (void)
{
  volatile int curr_binary_id;
  assert(sizeof(curr_binary_id) ==
         HIVE_SIZE_curr_binary_id);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(curr_binary_id),
    (char*)&curr_binary_id,
    sizeof(curr_binary_id));
  sh_css_print("curr_binary_id ="
    "                                          0x%x\n",
    curr_binary_id);
}

extern void sh_css_dump_sp_isp_started (void);
void sh_css_dump_sp_isp_started (void)
{
  volatile int sp_isp_started;
  assert(sizeof(sp_isp_started) ==
         HIVE_SIZE_sp_isp_started);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_isp_started),
    (char*)&sp_isp_started,
    sizeof(sp_isp_started));
  sh_css_print("sp_isp_started ="
    "                                          0x%x\n",
    sp_isp_started);
}

extern void sh_css_dump_sp_sw_state (void);
void sh_css_dump_sp_sw_state (void)
{
  volatile int sp_sw_state;
  assert(sizeof(sp_sw_state) ==
         HIVE_SIZE_sp_sw_state);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_sw_state),
    (char*)&sp_sw_state,
    sizeof(sp_sw_state));
  sh_css_print("sp_sw_state ="
    "                                             0x%x\n",
    sp_sw_state);
}

extern void sh_css_dump_host_sp_queues_initialized (void);
void sh_css_dump_host_sp_queues_initialized (void)
{
  volatile int host_sp_queues_initialized;
  assert(sizeof(host_sp_queues_initialized) ==
         HIVE_SIZE_host_sp_queues_initialized);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(host_sp_queues_initialized),
    (char*)&host_sp_queues_initialized,
    sizeof(host_sp_queues_initialized));
  sh_css_print("host_sp_queues_initialized ="
    "                              0x%x\n",
    host_sp_queues_initialized);
}

extern void sh_css_dump_sp_sleep_mode (void);
void sh_css_dump_sp_sleep_mode (void)
{
  volatile int sp_sleep_mode;
  assert(sizeof(sp_sleep_mode) ==
         HIVE_SIZE_sp_sleep_mode);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_sleep_mode),
    (char*)&sp_sleep_mode,
    sizeof(sp_sleep_mode));
  sh_css_print("sp_sleep_mode ="
    "                                           0x%x\n",
    sp_sleep_mode);
}

extern void sh_css_dump_isp_uv_internal_width_vecs (void);
void sh_css_dump_isp_uv_internal_width_vecs (void)
{
  unsigned int isp_uv_internal_width_vecs;
  assert(sizeof(isp_uv_internal_width_vecs) ==
         HIVE_SIZE_isp_uv_internal_width_vecs);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_uv_internal_width_vecs),
    (char*)&isp_uv_internal_width_vecs,
    sizeof(isp_uv_internal_width_vecs));
  sh_css_print("isp_uv_internal_width_vecs ="
    "                              0x%x\n",
    isp_uv_internal_width_vecs);
}

extern void sh_css_dump_isp_vf_output_width_vecs (void);
void sh_css_dump_isp_vf_output_width_vecs (void)
{
  unsigned int isp_vf_output_width_vecs;
  assert(sizeof(isp_vf_output_width_vecs) ==
         HIVE_SIZE_isp_vf_output_width_vecs);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_vf_output_width_vecs),
    (char*)&isp_vf_output_width_vecs,
    sizeof(isp_vf_output_width_vecs));
  sh_css_print("isp_vf_output_width_vecs ="
    "                                0x%x\n",
    isp_vf_output_width_vecs);
}

extern void sh_css_dump_isp_vectors_per_line (void);
void sh_css_dump_isp_vectors_per_line (void)
{
  unsigned int isp_vectors_per_line;
  assert(sizeof(isp_vectors_per_line) ==
         HIVE_SIZE_isp_vectors_per_line);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_vectors_per_line),
    (char*)&isp_vectors_per_line,
    sizeof(isp_vectors_per_line));
  sh_css_print("isp_vectors_per_line ="
    "                                    0x%x\n",
    isp_vectors_per_line);
}

extern void sh_css_dump_isp_vectors_per_input_line (void);
void sh_css_dump_isp_vectors_per_input_line (void)
{
  unsigned int isp_vectors_per_input_line;
  assert(sizeof(isp_vectors_per_input_line) ==
         HIVE_SIZE_isp_vectors_per_input_line);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_vectors_per_input_line),
    (char*)&isp_vectors_per_input_line,
    sizeof(isp_vectors_per_input_line));
  sh_css_print("isp_vectors_per_input_line ="
    "                              0x%x\n",
    isp_vectors_per_input_line);
}

extern void sh_css_dump_isp_sdis_horiproj_num (void);
void sh_css_dump_isp_sdis_horiproj_num (void)
{
  unsigned int isp_sdis_horiproj_num;
  assert(sizeof(isp_sdis_horiproj_num) ==
         HIVE_SIZE_isp_sdis_horiproj_num);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_sdis_horiproj_num),
    (char*)&isp_sdis_horiproj_num,
    sizeof(isp_sdis_horiproj_num));
  sh_css_print("isp_sdis_horiproj_num ="
    "                                   0x%x\n",
    isp_sdis_horiproj_num);
}

extern void sh_css_dump_isp_sdis_vertproj_num (void);
void sh_css_dump_isp_sdis_vertproj_num (void)
{
  unsigned int isp_sdis_vertproj_num;
  assert(sizeof(isp_sdis_vertproj_num) ==
         HIVE_SIZE_isp_sdis_vertproj_num);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_sdis_vertproj_num),
    (char*)&isp_sdis_vertproj_num,
    sizeof(isp_sdis_vertproj_num));
  sh_css_print("isp_sdis_vertproj_num ="
    "                                   0x%x\n",
    isp_sdis_vertproj_num);
}

extern void sh_css_dump_isp_sdis_horicoef_vectors (void);
void sh_css_dump_isp_sdis_horicoef_vectors (void)
{
  unsigned int isp_sdis_horicoef_vectors;
  assert(sizeof(isp_sdis_horicoef_vectors) ==
         HIVE_SIZE_isp_sdis_horicoef_vectors);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_sdis_horicoef_vectors),
    (char*)&isp_sdis_horicoef_vectors,
    sizeof(isp_sdis_horicoef_vectors));
  sh_css_print("isp_sdis_horicoef_vectors ="
    "                               0x%x\n",
    isp_sdis_horicoef_vectors);
}

extern void sh_css_dump_isp_sdis_vertcoef_vectors (void);
void sh_css_dump_isp_sdis_vertcoef_vectors (void)
{
  unsigned int isp_sdis_vertcoef_vectors;
  assert(sizeof(isp_sdis_vertcoef_vectors) ==
         HIVE_SIZE_isp_sdis_vertcoef_vectors);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(isp_sdis_vertcoef_vectors),
    (char*)&isp_sdis_vertcoef_vectors,
    sizeof(isp_sdis_vertcoef_vectors));
  sh_css_print("isp_sdis_vertcoef_vectors ="
    "                               0x%x\n",
    isp_sdis_vertcoef_vectors);
}

extern void sh_css_dump_sp_isp_input_stream_format (void);
void sh_css_dump_sp_isp_input_stream_format (void)
{
  enum sh_stream_format{ sh_stream_format_yuv420_legacy, sh_stream_format_yuv420, sh_stream_format_yuv422, sh_stream_format_rgb, sh_stream_format_raw, sh_stream_format_binary} sp_isp_input_stream_format;
  assert(sizeof(sp_isp_input_stream_format) ==
         HIVE_SIZE_sp_isp_input_stream_format);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_isp_input_stream_format),
    (char*)&sp_isp_input_stream_format,
    sizeof(sp_isp_input_stream_format));
  sh_css_print("sp_isp_input_stream_format ="
    "                              0x%x\n",
    sp_isp_input_stream_format);
}

extern void sh_css_dump_sp_obarea_start_bq (void);
void sh_css_dump_sp_obarea_start_bq (void)
{
  unsigned int sp_obarea_start_bq;
  assert(sizeof(sp_obarea_start_bq) ==
         HIVE_SIZE_sp_obarea_start_bq);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_obarea_start_bq),
    (char*)&sp_obarea_start_bq,
    sizeof(sp_obarea_start_bq));
  sh_css_print("sp_obarea_start_bq ="
    "                                      0x%x\n",
    sp_obarea_start_bq);
}

extern void sh_css_dump_sp_obarea_length_bq (void);
void sh_css_dump_sp_obarea_length_bq (void)
{
  unsigned int sp_obarea_length_bq;
  assert(sizeof(sp_obarea_length_bq) ==
         HIVE_SIZE_sp_obarea_length_bq);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_obarea_length_bq),
    (char*)&sp_obarea_length_bq,
    sizeof(sp_obarea_length_bq));
  sh_css_print("sp_obarea_length_bq ="
    "                                     0x%x\n",
    sp_obarea_length_bq);
}

extern void sh_css_dump_sp_preview_thread_id (void);
void sh_css_dump_sp_preview_thread_id (void)
{
  unsigned int sp_preview_thread_id;
  assert(sizeof(sp_preview_thread_id) ==
         HIVE_SIZE_sp_preview_thread_id);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_preview_thread_id),
    (char*)&sp_preview_thread_id,
    sizeof(sp_preview_thread_id));
  sh_css_print("sp_preview_thread_id ="
    "                                    0x%x\n",
    sp_preview_thread_id);
}

extern void sh_css_dump_sp_capture_thread_id (void);
void sh_css_dump_sp_capture_thread_id (void)
{
  unsigned int sp_capture_thread_id;
  assert(sizeof(sp_capture_thread_id) ==
         HIVE_SIZE_sp_capture_thread_id);
  sp_dmem_load(SP0_ID, (unsigned int)sp_address_of(sp_capture_thread_id),
    (char*)&sp_capture_thread_id,
    sizeof(sp_capture_thread_id));
  sh_css_print("sp_capture_thread_id ="
    "                                    0x%x\n",
    sp_capture_thread_id);
}

extern void sh_css_dump_sp_dmem(void);
void sh_css_dump_sp_dmem(void)
{
  sh_css_dump_sp_init_dmem_data();
  sh_css_dump_xmem_bin_addr();
  sh_css_dump_sp_vf_downscale_bits();
  sh_css_dump_sp_per_frame_data();
  sh_css_dump_sp_group();
  sh_css_dump_sp_output();
  sh_css_dump_host_sp_com();
  sh_css_dump_curr_binary_id();
  sh_css_dump_sp_isp_started();
  sh_css_dump_sp_sw_state();
  sh_css_dump_host_sp_queues_initialized();
  sh_css_dump_sp_sleep_mode();
  sh_css_dump_isp_uv_internal_width_vecs();
  sh_css_dump_isp_vf_output_width_vecs();
  sh_css_dump_isp_vectors_per_line();
  sh_css_dump_isp_vectors_per_input_line();
  sh_css_dump_isp_sdis_horiproj_num();
  sh_css_dump_isp_sdis_vertproj_num();
  sh_css_dump_isp_sdis_horicoef_vectors();
  sh_css_dump_isp_sdis_vertcoef_vectors();
  sh_css_dump_sp_isp_input_stream_format();
  sh_css_dump_sp_obarea_start_bq();
  sh_css_dump_sp_obarea_length_bq();
  sh_css_dump_sp_preview_thread_id();
  sh_css_dump_sp_capture_thread_id();
}
