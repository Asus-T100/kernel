#ifndef _sp_map_h_
#define _sp_map_h_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_input_line
#define HIVE_MEM_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0x2220
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2220
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 3316 */

/* function longjmp: 3673 */

/* function sp_tagger_tag_exp_id: 3298 */

/* function sp_dma_proxy_read: 2A8B */

/* function sp_dma_proxy_is_idle: 2C00 */

/* function debug_buffer_set_ddr_addr: B3 */

/* function setjmp: 367A */

/* function decode_sw_event: 535 */

/* function initialize_sp_stage: 2757 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_raw
#define HIVE_MEM_vbuf_raw scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_raw 0x90
#define HIVE_SIZE_vbuf_raw 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_raw scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_raw 0x90
#define HIVE_SIZE_sp_vbuf_raw 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_proxy_status
#define HIVE_MEM_dma_proxy_status scalar_processor_demo_dmem
#define HIVE_ADDR_dma_proxy_status 0x1BC
#define HIVE_SIZE_dma_proxy_status 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x1BC
#define HIVE_SIZE_sp_dma_proxy_status 4

/* function sp_start_isp: 1F63 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_binary_group 0x1F08
#define HIVE_SIZE_sp_binary_group 72
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x1F08
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sw_state 0x2224
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2224
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 3068 */

/* function sp_circular_buf_pop: 3030 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stage
#define HIVE_MEM_sp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stage 0x2228
#define HIVE_SIZE_sp_stage 3200
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_stage 0x2228
#define HIVE_SIZE_sp_sp_stage 3200

/* function __sp_circular_buf_push_unmarked_critical: 3E40 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_handles 0x1CF0
#define HIVE_SIZE_vbuf_handles 200
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x1CF0
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3B40
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3B40
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x31EC
#define HIVE_SIZE_sp_dma_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x31EC
#define HIVE_SIZE_sp_sp_dma_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x24C
#define HIVE_SIZE_sp_thread_ready_queue 8
#else
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x24C
#define HIVE_SIZE_sp_sp_thread_ready_queue 8

/* function sp_debug_mode_update_command: 35C7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_is_pending_mask
#define HIVE_MEM_event_is_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_event_is_pending_mask 0x38
#define HIVE_SIZE_event_is_pending_mask 32
#else
#endif
#endif
#define HIVE_MEM_sp_event_is_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_is_pending_mask 0x38
#define HIVE_SIZE_sp_event_is_pending_mask 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_com
#define HIVE_MEM_host_sp_com scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_com 0x2EA8
#define HIVE_SIZE_host_sp_com 44
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2EA8
#define HIVE_SIZE_sp_host_sp_com 44

/* function exec_image_pipe: 20F2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x2ED4
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x2ED4
#define HIVE_SIZE_sp_sp_init_dmem_data 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flash_in_use 0x978
#define HIVE_SIZE_sp_flash_in_use 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0x978
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_demo_dmem
#define HIVE_ADDR_flashed_frame_cnt 0x980
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0x980
#define HIVE_SIZE_sp_flashed_frame_cnt 4

/* function is_isp_debug_buffer_full: 2C3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_demo_dmem
#define HIVE_ADDR_stack_sizes 0x27C
#define HIVE_SIZE_stack_sizes 24
#else
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stack_sizes 0x27C
#define HIVE_SIZE_sp_stack_sizes 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_demo_dmem
#define HIVE_ADDR_ph 0x1C7C
#define HIVE_SIZE_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ph 0x1C7C
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_per_frame_data 0x2EEC
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x2EEC
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 2C1E */

/* function sp_tagger_connect_pipes: 3439 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x322C
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x322C
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_xmem_bin_addr 0x2EF0
#define HIVE_SIZE_xmem_bin_addr 4
#else
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x2EF0
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 36EE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_threads 0x254
#define HIVE_SIZE_pipe_threads 16
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_threads 0x254
#define HIVE_SIZE_sp_pipe_threads 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x1C
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x1C
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 29D0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_exp_id
#define HIVE_MEM_ia_css_i_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_ia_css_i_exp_id 0x324
#define HIVE_SIZE_ia_css_i_exp_id 1
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ia_css_i_exp_id 0x324
#define HIVE_SIZE_sp_ia_css_i_exp_id 1

/* function __divu: 3680 */

/* function sp_dma_proxy_func: 38CD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_started 0x2EF4
#define HIVE_SIZE_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x2EF4
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x326C
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x326C
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x2EF8
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x2EF8
#define HIVE_SIZE_sp_sp_obarea_start_bq 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_IRQ_BASE
#define HIVE_MEM_IRQ_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_IRQ_BASE 0x18
#define HIVE_SIZE_IRQ_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_IRQ_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_IRQ_BASE 0x18
#define HIVE_SIZE_sp_IRQ_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_TIMED_CTRL_BASE
#define HIVE_MEM_TIMED_CTRL_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_TIMED_CTRL_BASE 0x24
#define HIVE_SIZE_TIMED_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_TIMED_CTRL_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_TIMED_CTRL_BASE 0x24
#define HIVE_SIZE_sp_TIMED_CTRL_BASE 4

/* function ia_css_i_sp_rmgr_init: 387 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_current_sp_thread 0x248
#define HIVE_SIZE_current_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x248
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_demo_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x336C
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x336C
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x3B0
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x3B0
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 2D17 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_internal_event 0x3B28
#define HIVE_SIZE_sp_internal_event 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x3B28
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 29E2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_last_index 0x1DB8
#define HIVE_SIZE_last_index 4
#else
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_sp_last_index 0x1DB8
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x31DC
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x31DC
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

/* function host2sp_event_queue_is_empty: 3730 */

/* function debug_buffer_init_isp: BA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x3AC
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x3AC
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x3B50
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x3B50
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 2C2E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_demo_dmem
#define HIVE_ADDR_fibers 0x294
#define HIVE_SIZE_fibers 24
#else
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_demo_dmem
#define HIVE_ADDR_sp_fibers 0x294
#define HIVE_SIZE_sp_fibers 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x3B64
#define HIVE_SIZE_cb_params_preview_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3B64
#define HIVE_SIZE_sp_cb_params_preview_pipe 24

/* function sp_semaphore_init: 3E30 */

/* function initialize_sp_group: 273A */

/* function start_binary: EBC */

/* function sp_tagger_configure: 32D3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x31E0
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x31E0
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_DMEM_BASE
#define HIVE_MEM_ISP_DMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_DMEM_BASE 0xC
#define HIVE_SIZE_ISP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_DMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_DMEM_BASE 0xC
#define HIVE_SIZE_sp_ISP_DMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_DMEM_BASE
#define HIVE_MEM_SP_DMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_SP_DMEM_BASE 0x4
#define HIVE_SIZE_SP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_DMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_SP_DMEM_BASE 0x4
#define HIVE_SIZE_sp_SP_DMEM_BASE 4

/* function bin_spec_init_ifs: 5E8 */

/* function dma_proxy_channel_release: 2C0E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_can_send_token_mask
#define HIVE_MEM_event_can_send_token_mask scalar_processor_demo_dmem
#define HIVE_ADDR_event_can_send_token_mask 0x58
#define HIVE_SIZE_event_can_send_token_mask 32
#else
#endif
#endif
#define HIVE_MEM_sp_event_can_send_token_mask scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_can_send_token_mask 0x58
#define HIVE_SIZE_sp_event_can_send_token_mask 32

/* function sp_dma_proxy_wait_for_ack: 3B7D */

/* function sp_thread_yield: 3D47 */

/* function sp_circular_buf_peek: 2E9C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x2EFC
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x2EFC
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 262 */

/* function sp_uds_init: 7F5 */

/* function __sp_thread_fork_critical: 3D93 */

/* function sp_dma_proxy_isp_write_addr: 2A25 */

/* function sp_circular_buf_create: 3072 */

/* function debug_enqueue_ddr: C6 */

/* function host2sp_dequeue_buffer: 368 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_demo_dmem
#define HIVE_ADDR_buf_swap 0x340
#define HIVE_SIZE_buf_swap 96
#else
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_demo_dmem
#define HIVE_ADDR_sp_buf_swap 0x340
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 3285 */

/* function sp2host_enqueue_buffer: 34E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x1E90
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x1E90
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x3B7C
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x3B7C
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_output 0x2F00
#define HIVE_SIZE_sp_output 212
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_output 0x2F00
#define HIVE_SIZE_sp_sp_output 212

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_CTRL_BASE
#define HIVE_MEM_ISP_CTRL_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_ISP_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_CTRL_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_sp_ISP_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_INPUT_FORMATTER_BASE
#define HIVE_MEM_INPUT_FORMATTER_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_INPUT_FORMATTER_BASE 0x2C
#define HIVE_SIZE_INPUT_FORMATTER_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_INPUT_FORMATTER_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_INPUT_FORMATTER_BASE 0x2C
#define HIVE_SIZE_sp_INPUT_FORMATTER_BASE 8

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_sem
#define HIVE_MEM_raw_frame_sem scalar_processor_demo_dmem
#define HIVE_ADDR_raw_frame_sem 0x1DBC
#define HIVE_SIZE_raw_frame_sem 20
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_demo_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x1DBC
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_raw_frame_pool 0x1DD0
#define HIVE_SIZE_raw_frame_pool 16
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x1DD0
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 27AF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3B2C
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3B2C
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_tagger 0x31C8
#define HIVE_SIZE_tagger 20
#else
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_sp_tagger 0x31C8
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_refpool 0x1DE0
#define HIVE_SIZE_refpool 16
#else
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_refpool 0x1DE0
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 32E */

/* function sp_start_isp_entry: 1F5D */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x1F5D
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x1F5D

/* function __sp_raw_copy_func_critical: 37D3 */

/* function add_sp_command: 3C50 */

/* function sp2host_enqueue_irq_event: 317 */

/* function create_sp_fibers: 2C9B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x337C
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x337C
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

/* function sp_debug_mode_init: 35D0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x1E94
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x1E94
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 55B */

/* function ia_css_i_sp_rmgr_acq_gen: 3CE */

/* function sp_turn_on_flash: 2D30 */

/* function start_input: 25CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x339C
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x339C
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x2FD4
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x2FD4
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_demo_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x2FD8
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_demo_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x2FD8
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function __sp_dma_proxy_func_text: 2958 */

/* function sp_thread_join: 2DB5 */

/* function sp_dma_proxy_configure_channel: 2AD3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_group 0x2FE8
#define HIVE_SIZE_sp_group 440
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_group 0x2FE8
#define HIVE_SIZE_sp_sp_group 440

/* function sp2host_buffer_queue_get_size: 343 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x332C
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x332C
#define HIVE_SIZE_sp_sp_event_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_pool
#define HIVE_MEM_dma_pool scalar_processor_demo_dmem
#define HIVE_ADDR_dma_pool 0x84
#define HIVE_SIZE_dma_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_pool 0x84
#define HIVE_SIZE_sp_dma_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_uv_internal_width_vecs
#define HIVE_MEM_isp_uv_internal_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x31A0
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x31A0
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x3428
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x3428
#define HIVE_SIZE_sp_pipe_private_exp_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MMU_BASE
#define HIVE_MEM_MMU_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_MMU_BASE 0x34
#define HIVE_SIZE_MMU_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_MMU_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_MMU_BASE 0x34
#define HIVE_SIZE_sp_MMU_BASE 4

/* function sp_dma_proxy_configure_init_dmem_channel: 2AA1 */

/* function host2sp_buffer_queue_is_empty: 3749 */

/* function sp2host_event_queue_get_size: 30F */

/* function sp_dma_proxy_read_byte_addr: 2A66 */

/* function sp_thread_fork: 2DC3 */

/* function sp_semaphore_wait: 3DDD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_buffer_ddr_address
#define HIVE_MEM_debug_buffer_ddr_address scalar_processor_demo_dmem
#define HIVE_ADDR_debug_buffer_ddr_address 0x78
#define HIVE_SIZE_debug_buffer_ddr_address 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_buffer_ddr_address scalar_processor_demo_dmem
#define HIVE_ADDR_sp_debug_buffer_ddr_address 0x78
#define HIVE_SIZE_sp_debug_buffer_ddr_address 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3B8C
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3B8C
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_request_flash 0x31E8
#define HIVE_SIZE_sp_request_flash 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x31E8
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 35AE */

/* function dma_proxy_dma_read_write_split: 2B5E */

/* function cnd_input_system_cfg: 25E6 */

/* function sp_generate_events: 34EE */

/* function sp_uds_configure: 691 */

/* function sp_dma_proxy_execute: 2A7A */

/* function __modu: 36BF */

/* function sp_circular_buf_push_marked: 2EF4 */

/* function __sp_tagger_propagate_frame_critical: 3EB4 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x31A4
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x31A4
#define HIVE_SIZE_sp_isp_sdis_horiproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GDC_BASE
#define HIVE_MEM_GDC_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_GDC_BASE 0x28
#define HIVE_SIZE_GDC_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GDC_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_GDC_BASE 0x28
#define HIVE_SIZE_sp_GDC_BASE 4

/* function sp_fiber_init: 2D08 */

/* function ia_css_i_sp_rmgr_uninit: 382 */

/* function __sp_circular_buf_push_marked_critical: 3E7A */

/* function sp_thread_init: 2DCD */

/* function irq_raise_set_token: 92 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GPIO_BASE
#define HIVE_MEM_GPIO_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_GPIO_BASE 0x20
#define HIVE_SIZE_GPIO_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GPIO_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_GPIO_BASE 0x20
#define HIVE_SIZE_sp_GPIO_BASE 4

/* function _dma_proxy_dma_read_write: 3BE2 */

/* function sp_dma_proxy_configure_init_vmem_channel: 2ABA */

/* function sp_acquire_dynamic_buf: 31B7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_isp_ph 0x1CB8
#define HIVE_SIZE_isp_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_ph 0x1CB8
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 3442 */

/* function init_isp_internal_buffers: FF0 */

/* function sp_dma_proxy_write: 2A4F */

/* function ia_css_i_sp_refcount_init_vbuf: 3FD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3BA0
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3BA0
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 2A0F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3BB4
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3BB4
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

/* function sp2host_buffer_queue_is_full: 3739 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3BC8
#define HIVE_SIZE_cb_params_capture_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3BC8
#define HIVE_SIZE_sp_cb_params_capture_pipe 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x31A8
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x31A8
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3BE0
#define HIVE_SIZE_cb_frames_capture_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3BE0
#define HIVE_SIZE_sp_cb_frames_capture_pipe 24

/* function stop_input: 25B2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queue 0x342C
#define HIVE_SIZE_host_sp_queue 1036
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x342C
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 2DE6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_pool
#define HIVE_MEM_isp_pool scalar_processor_demo_dmem
#define HIVE_ADDR_isp_pool 0x88
#define HIVE_SIZE_isp_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_pool 0x88
#define HIVE_SIZE_sp_isp_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_is_done_flag
#define HIVE_MEM_isp_is_done_flag scalar_processor_demo_dmem
#define HIVE_ADDR_isp_is_done_flag 0x3C8
#define HIVE_SIZE_isp_is_done_flag 1
#else
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0x3C8
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_event_any_pending_mask 0x320
#define HIVE_SIZE_event_any_pending_mask 4
#else
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x320
#define HIVE_SIZE_sp_event_any_pending_mask 4

/* function ia_css_i_sp_refcount_release_vbuf: 473 */

/* function init_isp_data_segment: 106E */

/* function sh_css_decode_tag_descr: 516 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_dma_configs 0x1DF0
#define HIVE_SIZE_dma_configs 160
#else
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_configs 0x1DF0
#define HIVE_SIZE_sp_dma_configs 160

/* function debug_enqueue_isp: 21E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x1E98
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x1E98
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x3BF8
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x3BF8
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_demo_dmem
#define HIVE_ADDR_channels 0x1C98
#define HIVE_SIZE_channels 32
#else
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_demo_dmem
#define HIVE_ADDR_sp_channels 0x1C98
#define HIVE_SIZE_sp_channels 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x31AC
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x31AC
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x31B0
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x31B0
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 832 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_VAMEM_BASE
#define HIVE_MEM_ISP_VAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_VAMEM_BASE 0x10
#define HIVE_SIZE_ISP_VAMEM_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_VAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_VAMEM_BASE 0x10
#define HIVE_SIZE_sp_ISP_VAMEM_BASE 8

/* function sp_tagger_create: 345C */

/* function sp_dma_proxy_vmem_write: 29F8 */

/* function sp_thread_set_priority: 2D91 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3838
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3838
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 3DA8 */

/* function sp_dma_proxy_write_byte_addr: 2A3A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_PMEM_BASE
#define HIVE_MEM_SP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_SP_PMEM_BASE 0x0
#define HIVE_SIZE_SP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_SP_PMEM_BASE 0x0
#define HIVE_SIZE_sp_SP_PMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_input_stream_format
#define HIVE_MEM_sp_isp_input_stream_format scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x31B4
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x31B4
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 2E62 */

/* function __mod: 36AA */

/* function __sp_event_proxy_func_critical: 3C6C */

/* function sp_circular_buf_mark: 2F74 */

/* function irq_raise: A1 */

/* function sp_circular_buf_unmark: 2F53 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 43B */

/* function _dma_proxy_dma_execute: 3C21 */

/* function timed_ctrl_snd_gpio_commnd: 2DE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3C0C
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3C0C
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3C1C
#define HIVE_SIZE_cb_frames_preview_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3C1C
#define HIVE_SIZE_sp_cb_frames_preview_pipe 24

/* function set_sp_sleep_for_debug: 35D8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_demo_dmem
#define HIVE_ADDR_current_sp_fiber 0x96C
#define HIVE_SIZE_current_sp_fiber 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_demo_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0x96C
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x1E9C
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x1E9C
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3C34
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3C34
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 297B */

/* function sp_circular_buf_becomes_not_full: 2E22 */

/* function sp_release_dynamic_buf: 308C */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 460 */

/* function timed_ctrl_snd_commnd: 300 */

/* function end_binary: E38 */

/* function sp2host_event_queue_is_full: 3723 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_demo_dmem
#define HIVE_ADDR_stacks 0x264
#define HIVE_SIZE_stacks 24
#else
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stacks 0x264
#define HIVE_SIZE_sp_stacks 24

/* function dma_proxy_dma_execute_split: 2BC2 */

/* function ia_css_i_sp_refcount_dump: 41F */

/* function ia_css_i_sp_rmgr_rel_gen: 3BB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x1C78
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x1C78
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 2D98 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x3858
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x3858
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_addresses 0x1F50
#define HIVE_SIZE_sp_isp_addresses 172
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x1F50
#define HIVE_SIZE_sp_sp_isp_addresses 172

/* function sp_fiber_main: 2D10 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_demo_dmem
#define HIVE_ADDR_isps 0x1CD4
#define HIVE_SIZE_isps 28
#else
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isps 0x1CD4
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x31B8
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x31B8
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function timed_ctrl_snd_sp_commnd: 2EF */

/* function __sp_dma_proxy_wait_for_ack_text: 29B7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_spref
#define HIVE_MEM_vbuf_spref scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_spref 0x8C
#define HIVE_SIZE_vbuf_spref 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_spref scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_spref 0x8C
#define HIVE_SIZE_sp_vbuf_spref 4

/* function sp_circular_buf_extract: 2F94 */

/* function output_compute_dma_info: 1A02 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x31BC
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x31BC
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3C44
#define HIVE_SIZE_sem_for_reading_if 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3C44
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function update_sp_debug_info: 35E5 */

/* function sp_circular_buf_pop_marked: 2EBB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_frames 0x38C8
#define HIVE_SIZE_pipe_private_frames 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x38C8
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 3478 */

/* function init_isp_vars: 1C93 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x38F8
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x38F8
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_data 0x1FFC
#define HIVE_SIZE_sp_data 548
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_data 0x1FFC
#define HIVE_SIZE_sp_sp_data 548

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x31C
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x31C
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function sp_circular_buf_is_marked: 2F34 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_mem_map 0x1EA0
#define HIVE_SIZE_mem_map 104
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_sp_mem_map 0x1EA0
#define HIVE_SIZE_sp_mem_map 104

/* function sp_init_dmem: 26D5 */

/* function ia_css_i_sp_refcount_retain_vbuf: 48B */

/* function init_isp_code_segment: EE0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0x318
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0x318
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function run_sp_threads: 2DFB */

/* function configure_dma_channel: 2792 */

/* function sp_thread_queue_print: 2E11 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flash_in_service 0x97C
#define HIVE_SIZE_sp_flash_in_service 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0x97C
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x31C0
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x31C0
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 2E2E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sleep_mode 0x31C4
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x31C4
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_isp_stop_req 0x31E4
#define HIVE_SIZE_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x31E4
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
