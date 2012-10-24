#ifndef _sp_map_h_
#define _sp_map_h_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_input_line
#define HIVE_MEM_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0x2268
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2268
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 3B95 */

/* function longjmp: 2CDD */

/* function sp_tagger_tag_exp_id: 2AEB */

/* function sp_dma_proxy_read: 24D7 */

/* function sp_dma_proxy_is_idle: 2617 */

/* function debug_buffer_set_ddr_addr: B3 */

/* function __exec_image_pipe_critical: 2E63 */

/* function setjmp: 2CE4 */

/* function decode_sw_event: 497 */

/* function initialize_sp_stage: 3145 */

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

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_demo_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x3230
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_demo_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x3230
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 1E06 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_binary_group 0x1F50
#define HIVE_SIZE_sp_binary_group 72
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x1F50
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sw_state 0x226C
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x226C
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 28B8 */

/* function sp_circular_buf_pop: 28A2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stage
#define HIVE_MEM_sp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stage 0x2270
#define HIVE_SIZE_sp_stage 3200
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_stage 0x2270
#define HIVE_SIZE_sp_sp_stage 3200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_handles 0x1D38
#define HIVE_SIZE_vbuf_handles 200
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x1D38
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3B98
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3B98
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x3244
#define HIVE_SIZE_sp_dma_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x3244
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

/* function sp_debug_mode_update_command: 2C31 */

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
#define HIVE_ADDR_host_sp_com 0x2EF0
#define HIVE_SIZE_host_sp_com 68
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2EF0
#define HIVE_SIZE_sp_host_sp_com 68

/* function exec_image_pipe: 1F95 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x2F34
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x2F34
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
#define HIVE_ADDR_ph 0x1CC4
#define HIVE_SIZE_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ph 0x1CC4
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_per_frame_data 0x2F4C
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x2F4C
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 3790 */

/* function sp_tagger_connect_pipes: 2B4B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x3284
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x3284
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_xmem_bin_addr 0x2F50
#define HIVE_SIZE_xmem_bin_addr 4
#else
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x2F50
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 2D58 */

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

/* function sp_dma_proxy_set_width_ab: 2430 */

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

/* function __divu: 2CEA */

/* function sp_dma_proxy_func: 339A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_started 0x2F54
#define HIVE_SIZE_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x2F54
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x32C4
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x32C4
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x2F58
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x2F58
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

/* function ia_css_i_sp_rmgr_init: 333 */

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
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x33C4
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x33C4
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

/* function sp_turn_off_flash: 270E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_internal_event 0x3B80
#define HIVE_SIZE_sp_internal_event 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x3B80
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 2442 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_last_index 0x1E00
#define HIVE_SIZE_last_index 4
#else
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_sp_last_index 0x1E00
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x3234
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x3234
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

/* function host2sp_event_queue_is_empty: 2DAE */

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
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x3BA8
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x3BA8
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 2625 */

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
#define HIVE_ADDR_cb_params_preview_pipe 0x3BBC
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3BBC
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 3986 */

/* function initialize_sp_group: 230D */

/* function start_binary: D69 */

/* function sp_tagger_configure: 2B26 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x3238
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x3238
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

/* function __sp2host_enqueue_irq_event_text: 317 */

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

/* function bin_spec_init_ifs: 540 */

/* function dma_proxy_channel_release: 3780 */

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

/* function sp_dma_proxy_wait_for_ack: 3654 */

/* function sp_thread_yield: 3876 */

/* function sp_circular_buf_peek: 39FF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x2F5C
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x2F5C
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 262 */

/* function __sp_circular_buf_pop_critical: 3B54 */

/* function sp_uds_init: 733 */

/* function __sp_thread_fork_critical: 38D3 */

/* function sp_dma_proxy_isp_write_addr: 2485 */

/* function sp_circular_buf_create: 28C2 */

/* function debug_enqueue_ddr: C6 */

/* function host2sp_dequeue_buffer: 2DFF */

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

/* function is_dynamic_buffer: 2AD8 */

/* function sp2host_enqueue_buffer: 2DDA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x1ED8
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x1ED8
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x3BD0
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x3BD0
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_output 0x2F60
#define HIVE_SIZE_sp_output 212
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_output 0x2F60
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
#define HIVE_ADDR_raw_frame_sem 0x1E04
#define HIVE_SIZE_raw_frame_sem 20
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_demo_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x1E04
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_raw_frame_pool 0x1E18
#define HIVE_SIZE_raw_frame_pool 16
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x1E18
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 3180 */

/* function __sp_dma_proxy_configure_channel_text: 251F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x3B84
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x3B84
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_tagger 0x321C
#define HIVE_SIZE_tagger 20
#else
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_sp_tagger 0x321C
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_refpool 0x1E28
#define HIVE_SIZE_refpool 16
#else
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_refpool 0x1E28
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 2DB7 */

/* function sp_start_isp_entry: 1E00 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x1E00
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x1E00

/* function add_sp_command: 3764 */

/* function sp2host_enqueue_irq_event: 2D9A */

/* function create_sp_fibers: 2692 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x33D4
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x33D4
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

/* function sp_debug_mode_init: 2C3A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x1EDC
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x1EDC
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 4BD */

/* function ia_css_i_sp_rmgr_acq_gen: 37A */

/* function sp_turn_on_flash: 2727 */

/* function start_input: 21B7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x33F4
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x33F4
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x3034
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x3034
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_demo_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x3038
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_demo_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x3038
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function __sp_dma_proxy_func_text: 23B8 */

/* function sp_thread_join: 27A1 */

/* function sp_dma_proxy_configure_channel: 367B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_group 0x3048
#define HIVE_SIZE_sp_group 428
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_group 0x3048
#define HIVE_SIZE_sp_sp_group 428

/* function sp2host_buffer_queue_get_size: 31D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x3384
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x3384
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
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x31F4
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x31F4
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x3480
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x3480
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

/* function sp_dma_proxy_configure_init_dmem_channel: 24ED */

/* function host2sp_buffer_queue_is_empty: 2DF1 */

/* function sp2host_event_queue_get_size: 30F */

/* function sp_dma_proxy_read_byte_addr: 3667 */

/* function sp_thread_fork: 27AF */

/* function sp_semaphore_wait: 391D */

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
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3BE0
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3BE0
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

/* function __host2sp_dequeue_sp_event_text: 31A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_request_flash 0x3240
#define HIVE_SIZE_sp_request_flash 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x3240
#define HIVE_SIZE_sp_sp_request_flash 4

/* function sp_debug_mode_is_dma_request_enabled: 2C18 */

/* function dma_proxy_dma_read_write_split: 257D */

/* function cnd_input_system_cfg: 21D1 */

/* function sp_generate_events: 3D73 */

/* function sp_uds_configure: 5CF */

/* function sp_dma_proxy_execute: 24C6 */

/* function __modu: 2D29 */

/* function sp_circular_buf_push_marked: 2842 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x31F8
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x31F8
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

/* function sp_fiber_init: 26FF */

/* function ia_css_i_sp_rmgr_uninit: 32E */

/* function __sp_circular_buf_push_marked_critical: 3A56 */

/* function sp_thread_init: 27B9 */

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

/* function _dma_proxy_dma_read_write: 36FA */

/* function sp_dma_proxy_configure_init_vmem_channel: 2506 */

/* function sp_acquire_dynamic_buf: 2A01 */

/* function __host2sp_dequeue_buffer_text: 32B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_isp_ph 0x1D00
#define HIVE_SIZE_isp_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_ph 0x1D00
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 2B54 */

/* function __sp_circular_buf_extract_critical: 3AE3 */

/* function init_isp_internal_buffers: E9D */

/* function sp_dma_proxy_write: 24AF */

/* function ia_css_i_sp_refcount_init_vbuf: 3A9 */

/* function __sp_tagger_configure_critical: 3B74 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3BF4
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3BF4
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 246F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3C08
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3C08
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

/* function sp2host_buffer_queue_is_full: 2DCA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3C1C
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3C1C
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x31FC
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x31FC
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3C30
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3C30
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

/* function stop_input: 219D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queue 0x3484
#define HIVE_SIZE_host_sp_queue 1036
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x3484
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 27D2 */

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

/* function ia_css_i_sp_refcount_release_vbuf: 2E17 */

/* function init_isp_data_segment: F1B */

/* function sh_css_decode_tag_descr: 478 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_dma_configs 0x1E38
#define HIVE_SIZE_dma_configs 160
#else
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_configs 0x1E38
#define HIVE_SIZE_sp_dma_configs 160

/* function debug_enqueue_isp: 21E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x1EE0
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x1EE0
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x3C44
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x3C44
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_demo_dmem
#define HIVE_ADDR_channels 0x1CE0
#define HIVE_SIZE_channels 32
#else
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_demo_dmem
#define HIVE_ADDR_sp_channels 0x1CE0
#define HIVE_SIZE_sp_channels 32

/* function __sp2host_enqueue_buffer_text: 328 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x3200
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x3200
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x3204
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x3204
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 770 */

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

/* function sp_tagger_create: 2B6E */

/* function sp_dma_proxy_vmem_write: 2458 */

/* function sp_thread_set_priority: 277E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3890
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3890
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 38E8 */

/* function sp_dma_proxy_write_byte_addr: 249A */

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
#define HIVE_ADDR_sp_isp_input_stream_format 0x3208
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x3208
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 3996 */

/* function __mod: 2D14 */

/* function __sp_event_proxy_func_critical: 37A0 */

/* function sp_circular_buf_mark: 3AC3 */

/* function irq_raise: A1 */

/* function sp_circular_buf_unmark: 3AA3 */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 3E9 */

/* function _dma_proxy_dma_execute: 3735 */

/* function timed_ctrl_snd_gpio_commnd: 2DE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3C58
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3C58
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3C68
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3C68
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

/* function set_sp_sleep_for_debug: 2C42 */

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
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x1EE4
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x1EE4
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x3C7C
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x3C7C
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 23DB */

/* function sp_release_dynamic_buf: 28D6 */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 40E */

/* function timed_ctrl_snd_commnd: 300 */

/* function end_binary: CE5 */

/* function sp2host_event_queue_is_full: 2D8D */

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

/* function dma_proxy_dma_execute_split: 25D9 */

/* function ia_css_i_sp_refcount_dump: 3CD */

/* function ia_css_i_sp_rmgr_rel_gen: 367 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x1CC0
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x1CC0
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 2785 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x38B0
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x38B0
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_addresses 0x1F98
#define HIVE_SIZE_sp_isp_addresses 172
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x1F98
#define HIVE_SIZE_sp_sp_isp_addresses 172

/* function sp_fiber_main: 2707 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_demo_dmem
#define HIVE_ADDR_isps 0x1D1C
#define HIVE_SIZE_isps 28
#else
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isps 0x1D1C
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x320C
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x320C
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function timed_ctrl_snd_sp_commnd: 2EF */

/* function __sp_dma_proxy_wait_for_ack_text: 2417 */

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

/* function sp_circular_buf_extract: 2882 */

/* function output_compute_dma_info: 18A3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x3210
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x3210
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3C8C
#define HIVE_SIZE_sem_for_reading_if 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3C8C
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function update_sp_debug_info: 2C4F */

/* function sp_circular_buf_pop_marked: 3A1D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_frames 0x3920
#define HIVE_SIZE_pipe_private_frames 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x3920
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 2B8E */

/* function init_isp_vars: 1B34 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x3950
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x3950
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_data 0x2044
#define HIVE_SIZE_sp_data 548
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_data 0x2044
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

/* function sp_circular_buf_is_marked: 3A85 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_mem_map 0x1EE8
#define HIVE_SIZE_mem_map 104
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_sp_mem_map 0x1EE8
#define HIVE_SIZE_sp_mem_map 104

/* function sp_init_dmem: 22A7 */

/* function ia_css_i_sp_refcount_retain_vbuf: 2E2F */

/* function init_isp_code_segment: D8D */

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

/* function run_sp_threads: 27E7 */

/* function configure_dma_channel: 232A */

/* function __sp_circular_buf_create_critical: 3B6B */

/* function sp_thread_queue_print: 27FD */

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
#define HIVE_ADDR_isp_vf_output_width_vecs 0x3214
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x3214
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 280E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sleep_mode 0x3218
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x3218
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_isp_stop_req 0x323C
#define HIVE_SIZE_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x323C
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
