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
#define HIVE_ADDR_isp_vectors_per_input_line 0x2384
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#error Symbol isp_vectors_per_input_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2384
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 388C */

/* function longjmp: 3CBA */

/* function sp_dma_proxy_read: 2F6D */

/* function sp_dma_proxy_is_idle: 30F8 */

/* function debug_buffer_set_ddr_addr: 68 */

/* function setjmp: 3CC3 */

/* function decode_sw_event: 5A2 */

/* function initialize_sp_stage: 2B6A */

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

/* function sp_start_isp: 2183 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_binary_group 0x2018
#define HIVE_SIZE_sp_binary_group 72
#else
#error Symbol sp_binary_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x2018
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sw_state 0x2388
#define HIVE_SIZE_sp_sw_state 4
#else
#error Symbol sp_sw_state occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2388
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 36A3 */

/* function sp_circular_buf_pop: 361B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stage
#define HIVE_MEM_sp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_stage 0x238C
#define HIVE_SIZE_sp_stage 3184
#else
#error Symbol sp_stage occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_stage scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_stage 0x238C
#define HIVE_SIZE_sp_sp_stage 3184

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_vbuf_handles 0x1E1C
#define HIVE_SIZE_vbuf_handles 200
#else
#error Symbol vbuf_handles occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x1E1C
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3CA8
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#error Symbol cb_elems_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3CA8
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x3340
#define HIVE_SIZE_sp_dma_proxy_thread 68
#else
#error Symbol sp_dma_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x3340
#define HIVE_SIZE_sp_sp_dma_proxy_thread 68

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

/* function sp_debug_mode_update_command: 3BE4 */

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
#define HIVE_ADDR_host_sp_com 0x2FFC
#define HIVE_SIZE_host_sp_com 44
#else
#error Symbol host_sp_com occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2FFC
#define HIVE_SIZE_sp_host_sp_com 44

/* function exec_image_pipe: 239C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x3028
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#error Symbol sp_init_dmem_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x3028
#define HIVE_SIZE_sp_sp_init_dmem_data 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_use 0xB04
#define HIVE_SIZE_sp_flash_in_use 4
#else
#error Symbol sp_flash_in_use occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0xB04
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_flashed_frame_cnt 0xB0C
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#error Symbol flashed_frame_cnt occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0xB0C
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
#define HIVE_ADDR_ph 0x1DA8
#define HIVE_SIZE_ph 28
#else
#error Symbol ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ph 0x1DA8
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_per_frame_data 0x3040
#define HIVE_SIZE_sp_per_frame_data 4
#else
#error Symbol sp_per_frame_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x3040
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 3121 */

/* function sp_tagger_connect_pipes: 39FB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x3384
#define HIVE_SIZE_sp_copy_pipe_thread 68
#else
#error Symbol sp_copy_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x3384
#define HIVE_SIZE_sp_sp_copy_pipe_thread 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_xmem_bin_addr 0x3044
#define HIVE_SIZE_xmem_bin_addr 4
#else
#error Symbol xmem_bin_addr occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x3044
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 3D4D */

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
#define HIVE_ADDR_GP_DEVICE_BASE 0x338
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#error Symbol GP_DEVICE_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x338
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 2E9E */

/* function __divu: 3CCB */

/* function sp_dma_proxy_func: 2E13 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_started 0x3048
#define HIVE_SIZE_sp_isp_started 4
#else
#error Symbol sp_isp_started occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x3048
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x33C8
#define HIVE_SIZE_sp_isp_pipe_thread 204
#else
#error Symbol sp_isp_pipe_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x33C8
#define HIVE_SIZE_sp_sp_isp_pipe_thread 204

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x304C
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#error Symbol sp_obarea_start_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x304C
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

/* function ia_css_i_sp_rmgr_init: 391 */

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
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x34D8
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#error Symbol h_pipe_private_ddr_ptrs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x34D8
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x3CC
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#error Symbol sp_capture_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x3CC
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 3265 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_internal_event 0x3C90
#define HIVE_SIZE_sp_internal_event 4
#else
#error Symbol sp_internal_event occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x3C90
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 2EB1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_last_index 0x1EE4
#define HIVE_SIZE_last_index 4
#else
#error Symbol last_index occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_last_index 0x1EE4
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x3330
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#error Symbol isp_sh_dma_cmd_buffer occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x3330
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

/* function host2sp_event_queue_is_empty: 3D9D */

/* function debug_buffer_init_isp: 6F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x3C8
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#error Symbol sp_preview_thread_id occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x3C8
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x3CB8
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x3CB8
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 3135 */

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
#define HIVE_ADDR_cb_params_preview_pipe 0x3CCC
#define HIVE_SIZE_cb_params_preview_pipe 24
#else
#error Symbol cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3CCC
#define HIVE_SIZE_sp_cb_params_preview_pipe 24

/* function sp_semaphore_init: 4500 */

/* function initialize_sp_group: 2B4C */

/* function start_binary: FE8 */

/* function sp_tagger_configure: 384B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x3334
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#error Symbol sp_invalidate_tlb occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x3334
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

/* function __sp_dma_proxy_func_critical: 3E8D */

/* function bin_spec_init_ifs: 67B */

/* function dma_proxy_channel_release: 310D */

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

/* function sp_dma_proxy_wait_for_ack: 41B8 */

/* function sp_thread_yield: 4417 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x3050
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#error Symbol sp_obarea_length_bq occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x3050
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 24B */

/* function sp_uds_init: 8AB */

/* function sp_dma_proxy_isp_write_addr: 2EFB */

/* function sp_circular_buf_create: 36AF */

/* function debug_enqueue_ddr: 79 */

/* function host2sp_dequeue_buffer: 36E */

/* function is_dynamic_buffer: 3830 */

/* function sp2host_enqueue_buffer: 34F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x1FAC
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#error Symbol sp_dma_crop_block_width_b occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x1FAC
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x3CE4
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#error Symbol cb_elems_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x3CE4
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_output 0x3054
#define HIVE_SIZE_sp_output 212
#else
#error Symbol sp_output occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_output 0x3054
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
#define HIVE_ADDR_raw_frame_sem 0x1EE8
#define HIVE_SIZE_raw_frame_sem 20
#else
#error Symbol raw_frame_sem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x1EE8
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_raw_frame_pool 0x1EFC
#define HIVE_SIZE_raw_frame_pool 16
#else
#error Symbol raw_frame_pool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x1EFC
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 2C1F */

/* function sp_circular_buf_becomes_not_empty: 4544 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3C94
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#error Symbol sem_for_sp2host_event_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3C94
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_tagger 0x331C
#define HIVE_SIZE_tagger 20
#else
#error Symbol tagger occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_tagger 0x331C
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_refpool 0x1F0C
#define HIVE_SIZE_refpool 16
#else
#error Symbol refpool occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_refpool 0x1F0C
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 32C */

/* function sp_start_isp_entry: 2179 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#error Symbol sp_start_isp_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x2179
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x2179

/* function __sp_raw_copy_func_critical: 3DC5 */

/* function add_sp_command: 42BB */

/* function sp2host_enqueue_irq_event: 30F */

/* function create_sp_fibers: 31CD */

/* function sp_circular_buf_get_pos: 33E1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x34E8
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#error Symbol pipe_private_s3a_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x34E8
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

/* function sp_debug_mode_init: 3BEE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x1FB0
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#error Symbol sp_dma_crop_block_width_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x1FB0
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 5CC */

/* function ia_css_i_sp_rmgr_acq_gen: 3EB */

/* function sp_turn_on_flash: 3284 */

/* function start_input: 29B0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x3508
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#error Symbol sems_for_sp2host_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x3508
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x3128
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#error Symbol isp_vectors_per_line occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x3128
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x312C
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#error Symbol h_latest_parameter_set occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x312C
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function sp_thread_join: 333E */

/* function sp_dma_proxy_configure_channel: 2FBC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_group 0x313C
#define HIVE_SIZE_sp_group 440
#else
#error Symbol sp_group occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_group 0x313C
#define HIVE_SIZE_sp_sp_group 440

/* function sp2host_buffer_queue_get_size: 343 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x3494
#define HIVE_SIZE_sp_event_proxy_thread 68
#else
#error Symbol sp_event_proxy_thread occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x3494
#define HIVE_SIZE_sp_sp_event_proxy_thread 68

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
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x32F4
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#error Symbol isp_uv_internal_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x32F4
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

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

/* function sp_dma_proxy_configure_init_dmem_channel: 2F87 */

/* function host2sp_buffer_queue_is_empty: 3DB6 */

/* function sp2host_event_queue_get_size: 306 */

/* function write_trunk_data_to_ddr: 2BA1 */

/* function sp_dma_proxy_read_byte_addr: 2F43 */

/* function sp_thread_fork: 334F */

/* function sp_semaphore_wait: 44A6 */

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
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3CF4
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3CF4
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_request_flash 0x333C
#define HIVE_SIZE_sp_request_flash 4
#else
#error Symbol sp_request_flash occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x333C
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 3BC4 */

/* function dma_proxy_dma_read_write_split: 304B */

/* function cnd_input_system_cfg: 29CE */

/* function sp_generate_events: 3AD7 */

/* function sp_uds_configure: 737 */

/* function sp_dma_proxy_execute: 2F5A */

/* function __modu: 3D11 */

/* function sp_circular_buf_push_marked: 34E5 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x32F8
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#error Symbol isp_sdis_horiproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x32F8
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

/* function sp_fiber_init: 3254 */

/* function ia_css_i_sp_rmgr_uninit: 38A */

/* function sp_thread_init: 336D */

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

/* function _dma_proxy_dma_read_write: 423C */

/* function sp_dma_proxy_configure_init_vmem_channel: 2FA2 */

/* function sp_acquire_dynamic_buf: 376C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_ph 0x1DE4
#define HIVE_SIZE_isp_ph 28
#else
#error Symbol isp_ph occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_ph 0x1DE4
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 3A05 */

/* function init_isp_internal_buffers: 10C9 */

/* function sp_thread_queue_push: 452E */

/* function sp_circular_buf_push: 3658 */

/* function sp_dma_proxy_write: 2F29 */

/* function ia_css_i_sp_refcount_init_vbuf: 42C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3D08
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#error Symbol sem_for_reading_cb_params_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3D08
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 2EE2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3D1C
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#error Symbol sem_for_reading_cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3D1C
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

/* function sp2host_buffer_queue_is_full: 3DA7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3D30
#define HIVE_SIZE_cb_params_capture_pipe 24
#else
#error Symbol cb_params_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3D30
#define HIVE_SIZE_sp_cb_params_capture_pipe 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x32FC
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#error Symbol isp_sdis_vertproj_num occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x32FC
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3D48
#define HIVE_SIZE_cb_frames_capture_pipe 24
#else
#error Symbol cb_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3D48
#define HIVE_SIZE_sp_cb_frames_capture_pipe 24

/* function stop_input: 2992 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queue 0x3594
#define HIVE_SIZE_host_sp_queue 1036
#else
#error Symbol host_sp_queue occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x3594
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 3388 */

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
#define HIVE_ADDR_isp_is_done_flag 0x3E4
#define HIVE_SIZE_isp_is_done_flag 1
#else
#error Symbol isp_is_done_flag occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0x3E4
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_event_any_pending_mask 0x33C
#define HIVE_SIZE_event_any_pending_mask 8
#else
#error Symbol event_any_pending_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x33C
#define HIVE_SIZE_sp_event_any_pending_mask 8

/* function ia_css_i_sp_refcount_release_vbuf: 4BD */

/* function read_trunk_data_from_ddr: 2BD0 */

/* function init_isp_data_segment: 1158 */

/* function sh_css_decode_tag_descr: 582 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_dma_configs 0x1F1C
#define HIVE_SIZE_dma_configs 144
#else
#error Symbol dma_configs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_configs 0x1F1C
#define HIVE_SIZE_sp_dma_configs 144

/* function debug_enqueue_isp: 1FA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x1FB4
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#error Symbol sp_dma_crop_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x1FB4
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x3D60
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#error Symbol sem_for_isp_idle occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x3D60
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_channels 0x1DC4
#define HIVE_SIZE_channels 32
#else
#error Symbol channels occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_channels 0x1DC4
#define HIVE_SIZE_sp_channels 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x3300
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#error Symbol sp_vf_downscale_bits occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x3300
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x3304
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#error Symbol isp_sdis_vertcoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x3304
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 8F1 */

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

/* function sp_tagger_create: 3A26 */

/* function sp_dma_proxy_vmem_write: 2ECA */

/* function sp_thread_set_priority: 3313 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x39A0
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#error Symbol pipe_private_dis_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x39A0
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 4474 */

/* function sp_dma_proxy_write_byte_addr: 2F12 */

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
#define HIVE_ADDR_sp_isp_input_stream_format 0x3308
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#error Symbol sp_isp_input_stream_format occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x3308
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 3445 */

/* function __mod: 3CFD */

/* function __sp_event_proxy_func_critical: 42D9 */

/* function sp_thread_queue_pop: 4512 */

/* function sp_circular_buf_mark: 3571 */

/* function irq_raise: 53 */

/* function sp_circular_buf_unmark: 3557 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 478 */

/* function _dma_proxy_dma_execute: 4283 */

/* function timed_ctrl_snd_gpio_commnd: 2CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3D74
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#error Symbol cb_elems_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3D74
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3D84
#define HIVE_SIZE_cb_frames_preview_pipe 24
#else
#error Symbol cb_frames_preview_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3D84
#define HIVE_SIZE_sp_cb_frames_preview_pipe 24

/* function set_sp_sleep_for_debug: 3BF9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_current_sp_fiber 0xAF8
#define HIVE_SIZE_current_sp_fiber 4
#else
#error Symbol current_sp_fiber occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0xAF8
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x1FB8
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#error Symbol sp_dma_vfout_cropping_a occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x1FB8
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3D9C
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#error Symbol cb_elems_frames_capture_pipe occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3D9C
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 2E3D */

/* function sp_circular_buf_becomes_not_full: 33D5 */

/* function sp_release_dynamic_buf: 36C9 */

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

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 4A6 */

/* function timed_ctrl_snd_commnd: 2F6 */

/* function end_binary: F51 */

/* function sp2host_event_queue_is_full: 3D90 */

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

/* function dma_proxy_dma_execute_split: 30B1 */

/* function ia_css_i_sp_refcount_dump: 453 */

/* function ia_css_i_sp_rmgr_rel_gen: 3D3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x1DA4
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#error Symbol irq_sw_interrupt_token occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x1DA4
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 331B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x39C0
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#error Symbol pipe_private_buffer_bufs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x39C0
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_addresses 0x2060
#define HIVE_SIZE_sp_isp_addresses 164
#else
#error Symbol sp_isp_addresses occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x2060
#define HIVE_SIZE_sp_sp_isp_addresses 164

/* function sp_fiber_main: 325E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_isps 0x1E00
#define HIVE_SIZE_isps 28
#else
#error Symbol isps occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isps 0x1E00
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x330C
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#error Symbol host_sp_queues_initialized occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x330C
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function timed_ctrl_snd_sp_commnd: 2E1 */

/* function __sp_dma_proxy_wait_for_ack_text: 2E83 */

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

/* function sp_circular_buf_extract: 358B */

/* function output_compute_dma_info: 1B99 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x3310
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#error Symbol isp_sdis_horicoef_vectors occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x3310
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3DAC
#define HIVE_SIZE_sem_for_reading_if 20
#else
#error Symbol sem_for_reading_if occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3DAC
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function update_sp_debug_info: 3C0B */

/* function sp_circular_buf_pop_marked: 3499 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_pipe_private_frames 0x3A30
#define HIVE_SIZE_pipe_private_frames 48
#else
#error Symbol pipe_private_frames occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x3A30
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 3A48 */

/* function init_isp_vars: 1E3A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x3A60
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#error Symbol sems_for_host2sp_buf_queues occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x3A60
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_data 0x2104
#define HIVE_SIZE_sp_data 640
#else
#error Symbol sp_data occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_data 0x2104
#define HIVE_SIZE_sp_sp_data 640

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x334
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#error Symbol ISP_BAMEM_BASE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x334
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function sp_circular_buf_is_marked: 3540 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_mem_map 0x1FBC
#define HIVE_SIZE_mem_map 92
#else
#error Symbol mem_map occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_mem_map 0x1FBC
#define HIVE_SIZE_sp_mem_map 92

/* function sp_init_dmem: 2ACD */

/* function ia_css_i_sp_refcount_retain_vbuf: 4DA */

/* function init_isp_code_segment: 100B */

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

/* function run_sp_threads: 339F */

/* function configure_dma_channel: 2BFF */

/* function sp_thread_queue_print: 33BC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_flash_in_service 0xB08
#define HIVE_SIZE_sp_flash_in_service 4
#else
#error Symbol sp_flash_in_service occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0xB08
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x3314
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#error Symbol isp_vf_output_width_vecs occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x3314
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 3401 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sleep_mode 0x3318
#define HIVE_SIZE_sp_sleep_mode 4
#else
#error Symbol sp_sleep_mode occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x3318
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: 5E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_isp_stop_req 0x3338
#define HIVE_SIZE_isp_stop_req 4
#else
#error Symbol isp_stop_req occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_2400A0_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x3338
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
