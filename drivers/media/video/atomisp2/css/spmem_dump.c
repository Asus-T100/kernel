#ifndef _sp_map_h_
#define _sp_map_h_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_input_line
#define HIVE_MEM_isp_vectors_per_input_line scalar_processor_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0x2784
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2784
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 3457 */

/* function longjmp: 3814 */

/* function sp_tagger_tag_exp_id: 33D9 */

/* function sp_dma_proxy_read: 2C71 */

/* function sp_dma_proxy_is_idle: 2D56 */

/* function debug_buffer_set_ddr_addr: C4 */

/* function stop_event_proxy: 2D64 */

/* function setjmp: 381B */

/* function decode_sw_event: 54B */

/* function initialize_sp_stage: 38FF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stage
#define HIVE_MEM_isp_stage scalar_processor_dmem
#define HIVE_ADDR_isp_stage 0x34C0
#define HIVE_SIZE_isp_stage 480
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stage scalar_processor_dmem
#define HIVE_ADDR_sp_isp_stage 0x34C0
#define HIVE_SIZE_sp_isp_stage 480

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_raw
#define HIVE_MEM_vbuf_raw scalar_processor_dmem
#define HIVE_ADDR_vbuf_raw 0x104
#define HIVE_SIZE_vbuf_raw 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_raw scalar_processor_dmem
#define HIVE_ADDR_sp_vbuf_raw 0x104
#define HIVE_SIZE_sp_vbuf_raw 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_proxy_status
#define HIVE_MEM_dma_proxy_status scalar_processor_dmem
#define HIVE_ADDR_dma_proxy_status 0x278
#define HIVE_SIZE_dma_proxy_status 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x278
#define HIVE_SIZE_sp_dma_proxy_status 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x2A28
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x2A28
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 1FB9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_dmem
#define HIVE_ADDR_sp_binary_group 0x24BC
#define HIVE_SIZE_sp_binary_group 72
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x24BC
#define HIVE_SIZE_sp_sp_binary_group 72

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_dmem
#define HIVE_ADDR_sp_sw_state 0x2788
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2788
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 31EB */

/* function sp_circular_buf_pop: 31BE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stop_copy_preview
#define HIVE_MEM_sp_stop_copy_preview scalar_processor_dmem
#define HIVE_ADDR_sp_stop_copy_preview 0x2A04
#define HIVE_SIZE_sp_stop_copy_preview 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stop_copy_preview scalar_processor_dmem
#define HIVE_ADDR_sp_sp_stop_copy_preview 0x2A04
#define HIVE_SIZE_sp_sp_stop_copy_preview 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_dmem
#define HIVE_ADDR_vbuf_handles 0x22A8
#define HIVE_SIZE_vbuf_handles 200
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x22A8
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x3390
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x3390
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

/* function sh_css_stride_from_info: E94 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x2A3C
#define HIVE_SIZE_sp_dma_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x2A3C
#define HIVE_SIZE_sp_sp_dma_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x318
#define HIVE_SIZE_sp_thread_ready_queue 8
#else
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x318
#define HIVE_SIZE_sp_sp_thread_ready_queue 8

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_is_pending_mask
#define HIVE_MEM_event_is_pending_mask scalar_processor_dmem
#define HIVE_ADDR_event_is_pending_mask 0x54
#define HIVE_SIZE_event_is_pending_mask 32
#else
#endif
#endif
#define HIVE_MEM_sp_event_is_pending_mask scalar_processor_dmem
#define HIVE_ADDR_sp_event_is_pending_mask 0x54
#define HIVE_SIZE_sp_event_is_pending_mask 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_com
#define HIVE_MEM_host_sp_com scalar_processor_dmem
#define HIVE_ADDR_host_sp_com 0x278C
#define HIVE_SIZE_host_sp_com 68
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_dmem
#define HIVE_ADDR_sp_host_sp_com 0x278C
#define HIVE_SIZE_sp_host_sp_com 68

/* function exec_image_pipe: 219F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x27D0
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x27D0
#define HIVE_SIZE_sp_sp_init_dmem_data 24

/* function stop_threads: 215D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_dmem
#define HIVE_ADDR_sp_flash_in_use 0xEB0
#define HIVE_SIZE_sp_flash_in_use 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0xEB0
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_dmem
#define HIVE_ADDR_flashed_frame_cnt 0xEB8
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0xEB8
#define HIVE_SIZE_sp_flashed_frame_cnt 4

/* function is_isp_debug_buffer_full: 2D7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_dmem
#define HIVE_ADDR_stack_sizes 0x350
#define HIVE_SIZE_stack_sizes 24
#else
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_dmem
#define HIVE_ADDR_sp_stack_sizes 0x350
#define HIVE_SIZE_sp_stack_sizes 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_dmem
#define HIVE_ADDR_ph 0x2234
#define HIVE_SIZE_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_dmem
#define HIVE_ADDR_sp_ph 0x2234
#define HIVE_SIZE_sp_ph 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_dmem
#define HIVE_ADDR_sp_per_frame_data 0x27E8
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x27E8
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 3DE2 */

/* function sp_tagger_connect_pipes: 3633 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x2A7C
#define HIVE_SIZE_sp_copy_pipe_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x2A7C
#define HIVE_SIZE_sp_sp_copy_pipe_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_dmem
#define HIVE_ADDR_xmem_bin_addr 0x27EC
#define HIVE_SIZE_xmem_bin_addr 4
#else
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x27EC
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 388F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_dmem
#define HIVE_ADDR_pipe_threads 0x328
#define HIVE_SIZE_pipe_threads 16
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_threads 0x328
#define HIVE_SIZE_sp_pipe_threads 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x1C
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x1C
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

/* function sp_dma_proxy_set_width_ab: 2BCA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_exp_id
#define HIVE_MEM_ia_css_i_exp_id scalar_processor_dmem
#define HIVE_ADDR_ia_css_i_exp_id 0x428
#define HIVE_SIZE_ia_css_i_exp_id 1
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_exp_id scalar_processor_dmem
#define HIVE_ADDR_sp_ia_css_i_exp_id 0x428
#define HIVE_SIZE_sp_ia_css_i_exp_id 1

/* function __divu: 3821 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_stop
#define HIVE_MEM_sem_for_cont_capt_stop scalar_processor_dmem
#define HIVE_ADDR_sem_for_cont_capt_stop 0x33A0
#define HIVE_SIZE_sem_for_cont_capt_stop 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_stop scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_stop 0x33A0
#define HIVE_SIZE_sp_sem_for_cont_capt_stop 20

/* function sp_dma_proxy_func: 3986 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_dmem
#define HIVE_ADDR_sp_isp_started 0x27F0
#define HIVE_SIZE_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x27F0
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x2ABC
#define HIVE_SIZE_sp_isp_pipe_thread 192
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x2ABC
#define HIVE_SIZE_sp_sp_isp_pipe_thread 192

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x27F4
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x27F4
#define HIVE_SIZE_sp_sp_obarea_start_bq 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_IRQ_BASE
#define HIVE_MEM_IRQ_BASE scalar_processor_dmem
#define HIVE_ADDR_IRQ_BASE 0x18
#define HIVE_SIZE_IRQ_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_IRQ_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_IRQ_BASE 0x18
#define HIVE_SIZE_sp_IRQ_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_TIMED_CTRL_BASE
#define HIVE_MEM_TIMED_CTRL_BASE scalar_processor_dmem
#define HIVE_ADDR_TIMED_CTRL_BASE 0x24
#define HIVE_SIZE_TIMED_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_TIMED_CTRL_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_TIMED_CTRL_BASE 0x24
#define HIVE_SIZE_sp_TIMED_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_is_isp_requested
#define HIVE_MEM_is_isp_requested scalar_processor_dmem
#define HIVE_ADDR_is_isp_requested 0xE8C
#define HIVE_SIZE_is_isp_requested 4
#else
#endif
#endif
#define HIVE_MEM_sp_is_isp_requested scalar_processor_dmem
#define HIVE_ADDR_sp_is_isp_requested 0xE8C
#define HIVE_SIZE_sp_is_isp_requested 4

/* function ia_css_i_sp_rmgr_init: 39B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_dmem
#define HIVE_ADDR_current_sp_thread 0x314
#define HIVE_SIZE_current_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x314
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x2BBC
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x2BBC
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x4B4
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x4B4
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 2E3F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_dmem
#define HIVE_ADDR_sp_internal_event 0x3378
#define HIVE_SIZE_sp_internal_event 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x3378
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 2BDC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_dmem
#define HIVE_ADDR_last_index 0x2370
#define HIVE_SIZE_last_index 4
#else
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_dmem
#define HIVE_ADDR_sp_last_index 0x2370
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x2A2C
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x2A2C
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_start
#define HIVE_MEM_sem_for_cont_capt_start scalar_processor_dmem
#define HIVE_ADDR_sem_for_cont_capt_start 0x33B4
#define HIVE_SIZE_sem_for_cont_capt_start 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_start scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_start 0x33B4
#define HIVE_SIZE_sp_sem_for_cont_capt_start 20

/* function host2sp_event_queue_is_empty: 38C4 */

/* function debug_buffer_init_isp: CB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x4B0
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x4B0
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x33C8
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x33C8
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 20

/* function sp_event_proxy_func: 2D6B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_dmem
#define HIVE_ADDR_fibers 0x368
#define HIVE_SIZE_fibers 24
#else
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_dmem
#define HIVE_ADDR_sp_fibers 0x368
#define HIVE_SIZE_sp_fibers 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x33DC
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x33DC
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 3F96 */

/* function initialize_sp_group: 282E */

/* function start_binary: F26 */

/* function sp_tagger_configure: 3414 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x2A30
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x2A30
#define HIVE_SIZE_sp_sp_invalidate_tlb 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_DMEM_BASE
#define HIVE_MEM_ISP_DMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_ISP_DMEM_BASE 0xC
#define HIVE_SIZE_ISP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_DMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_ISP_DMEM_BASE 0xC
#define HIVE_SIZE_sp_ISP_DMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_DMEM_BASE
#define HIVE_MEM_SP_DMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_SP_DMEM_BASE 0x4
#define HIVE_SIZE_SP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_DMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_SP_DMEM_BASE 0x4
#define HIVE_SIZE_sp_SP_DMEM_BASE 4

/* function bin_spec_init_ifs: 5F4 */

/* function dma_proxy_channel_release: 3DD1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_can_send_token_mask
#define HIVE_MEM_event_can_send_token_mask scalar_processor_dmem
#define HIVE_ADDR_event_can_send_token_mask 0x74
#define HIVE_SIZE_event_can_send_token_mask 32
#else
#endif
#endif
#define HIVE_MEM_sp_event_can_send_token_mask scalar_processor_dmem
#define HIVE_ADDR_sp_event_can_send_token_mask 0x74
#define HIVE_SIZE_sp_event_can_send_token_mask 32

/* function sp_dma_proxy_wait_for_ack: 3C5E */

/* function sp_thread_yield: 3E9B */

/* function sp_circular_buf_peek: 3008 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_thread
#define HIVE_MEM_isp_thread scalar_processor_dmem
#define HIVE_ADDR_isp_thread 0x36A0
#define HIVE_SIZE_isp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_thread scalar_processor_dmem
#define HIVE_ADDR_sp_isp_thread 0x36A0
#define HIVE_SIZE_sp_isp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x27F8
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x27F8
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function is_ddr_debug_buffer_full: 276 */

/* function sp_uds_init: 822 */

/* function sp_dma_proxy_isp_write_addr: 2C1F */

/* function sp_circular_buf_create: 31F5 */

/* function debug_enqueue_ddr: D7 */

/* function host2sp_dequeue_buffer: 37C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_dmem
#define HIVE_ADDR_buf_swap 0x444
#define HIVE_SIZE_buf_swap 96
#else
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_dmem
#define HIVE_ADDR_sp_buf_swap 0x444
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 33C6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_curr_nr_of_copied_frames
#define HIVE_MEM_curr_nr_of_copied_frames scalar_processor_dmem
#define HIVE_ADDR_curr_nr_of_copied_frames 0x2A08
#define HIVE_SIZE_curr_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_curr_nr_of_copied_frames scalar_processor_dmem
#define HIVE_ADDR_sp_curr_nr_of_copied_frames 0x2A08
#define HIVE_SIZE_sp_curr_nr_of_copied_frames 4

/* function sp2host_enqueue_buffer: 362 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x2448
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x2448
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x33F0
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x33F0
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_dmem
#define HIVE_ADDR_sp_output 0x27FC
#define HIVE_SIZE_sp_output 16
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_dmem
#define HIVE_ADDR_sp_sp_output 0x27FC
#define HIVE_SIZE_sp_sp_output 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_CTRL_BASE
#define HIVE_MEM_ISP_CTRL_BASE scalar_processor_dmem
#define HIVE_ADDR_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_ISP_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_CTRL_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_sp_ISP_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_INPUT_FORMATTER_BASE
#define HIVE_MEM_INPUT_FORMATTER_BASE scalar_processor_dmem
#define HIVE_ADDR_INPUT_FORMATTER_BASE 0x2C
#define HIVE_SIZE_INPUT_FORMATTER_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_INPUT_FORMATTER_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_INPUT_FORMATTER_BASE 0x2C
#define HIVE_SIZE_sp_INPUT_FORMATTER_BASE 8

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_sem
#define HIVE_MEM_raw_frame_sem scalar_processor_dmem
#define HIVE_ADDR_raw_frame_sem 0x2374
#define HIVE_SIZE_raw_frame_sem 20
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_sem scalar_processor_dmem
#define HIVE_ADDR_sp_raw_frame_sem 0x2374
#define HIVE_SIZE_sp_raw_frame_sem 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_dmem
#define HIVE_ADDR_raw_frame_pool 0x2388
#define HIVE_SIZE_raw_frame_pool 16
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x2388
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 2868 */

/* function __sp_dma_proxy_configure_channel_text: 2CBA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x337C
#define HIVE_SIZE_sem_for_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x337C
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_dmem
#define HIVE_ADDR_tagger 0x2A0C
#define HIVE_SIZE_tagger 20
#else
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_dmem
#define HIVE_ADDR_sp_tagger 0x2A0C
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_dmem
#define HIVE_ADDR_refpool 0x2398
#define HIVE_SIZE_refpool 16
#else
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_dmem
#define HIVE_ADDR_sp_refpool 0x2398
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: 342 */

/* function sp_start_isp_entry: 1FB3 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x1FB3
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x1FB3

/* function __sp_raw_copy_func_critical: 3939 */

/* function add_sp_command: 3DB5 */

/* function sp2host_enqueue_irq_event: 32B */

/* function create_sp_fibers: 2DC3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x2BCC
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x2BCC
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x244C
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x244C
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 571 */

/* function ia_css_i_sp_rmgr_acq_gen: 3E2 */

/* function sp_turn_on_flash: 2E58 */

/* function start_input: 26E6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x2BEC
#define HIVE_SIZE_sems_for_sp2host_buf_queues 140
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x2BEC
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x280C
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x280C
#define HIVE_SIZE_sp_isp_vectors_per_line 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_latest_parameter_set
#define HIVE_MEM_h_latest_parameter_set scalar_processor_dmem
#define HIVE_ADDR_h_latest_parameter_set 0x2810
#define HIVE_SIZE_h_latest_parameter_set 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_latest_parameter_set scalar_processor_dmem
#define HIVE_ADDR_sp_h_latest_parameter_set 0x2810
#define HIVE_SIZE_sp_h_latest_parameter_set 16

/* function __sp_dma_proxy_func_text: 2B53 */

/* function sp_thread_join: 2EE9 */

/* function sp_dma_proxy_configure_channel: 3C87 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_dmem
#define HIVE_ADDR_sp_group 0x2820
#define HIVE_SIZE_sp_group 444
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_dmem
#define HIVE_ADDR_sp_sp_group 0x2820
#define HIVE_SIZE_sp_sp_group 444

/* function sp2host_buffer_queue_get_size: 357 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x2B7C
#define HIVE_SIZE_sp_event_proxy_thread 64
#else
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x2B7C
#define HIVE_SIZE_sp_sp_event_proxy_thread 64

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_pool
#define HIVE_MEM_dma_pool scalar_processor_dmem
#define HIVE_ADDR_dma_pool 0xE8
#define HIVE_SIZE_dma_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_pool scalar_processor_dmem
#define HIVE_ADDR_sp_dma_pool 0xE8
#define HIVE_SIZE_sp_dma_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_uv_internal_width_vecs
#define HIVE_MEM_isp_uv_internal_width_vecs scalar_processor_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x29DC
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x29DC
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x2C78
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x2C78
#define HIVE_SIZE_sp_pipe_private_exp_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MMU_BASE
#define HIVE_MEM_MMU_BASE scalar_processor_dmem
#define HIVE_ADDR_MMU_BASE 0x34
#define HIVE_SIZE_MMU_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_MMU_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_MMU_BASE 0x34
#define HIVE_SIZE_sp_MMU_BASE 4

/* function sp_dma_proxy_configure_init_dmem_channel: 2C87 */

/* function sp2host_event_queue_get_size: 323 */

/* function sp_dma_proxy_read_byte_addr: 3C73 */

/* function sp_thread_fork: 2EF7 */

/* function sp_semaphore_wait: 3F2D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_buffer_ddr_address
#define HIVE_MEM_debug_buffer_ddr_address scalar_processor_dmem
#define HIVE_ADDR_debug_buffer_ddr_address 0xAC
#define HIVE_SIZE_debug_buffer_ddr_address 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_buffer_ddr_address scalar_processor_dmem
#define HIVE_ADDR_sp_debug_buffer_ddr_address 0xAC
#define HIVE_SIZE_sp_debug_buffer_ddr_address 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3400
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3400
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_dmem
#define HIVE_ADDR_sp_request_flash 0x2A38
#define HIVE_SIZE_sp_request_flash 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x2A38
#define HIVE_SIZE_sp_sp_request_flash 4

/* function cnd_input_system_cfg: 2700 */

/* function sp_generate_events: 36FB */

/* function sp_uds_configure: 683 */

/* function sp_dma_proxy_execute: 2C60 */

/* function __modu: 3860 */

/* function sp_circular_buf_push_marked: 305F */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x29E0
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x29E0
#define HIVE_SIZE_sp_isp_sdis_horiproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GDC_BASE
#define HIVE_MEM_GDC_BASE scalar_processor_dmem
#define HIVE_ADDR_GDC_BASE 0x28
#define HIVE_SIZE_GDC_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GDC_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_GDC_BASE 0x28
#define HIVE_SIZE_sp_GDC_BASE 4

/* function sp_fiber_init: 2E30 */

/* function ia_css_i_sp_rmgr_uninit: 396 */

/* function sp_thread_init: 2F16 */

/* function irq_raise_set_token: A3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GPIO_BASE
#define HIVE_MEM_GPIO_BASE scalar_processor_dmem
#define HIVE_ADDR_GPIO_BASE 0x20
#define HIVE_SIZE_GPIO_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GPIO_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_GPIO_BASE 0x20
#define HIVE_SIZE_sp_GPIO_BASE 4

/* function _dma_proxy_dma_read_write: 3D06 */

/* function sp_dma_proxy_configure_init_vmem_channel: 2CA0 */

/* function sp_acquire_dynamic_buf: 3336 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_dmem
#define HIVE_ADDR_isp_ph 0x2270
#define HIVE_SIZE_isp_ph 28
#else
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_dmem
#define HIVE_ADDR_sp_isp_ph 0x2270
#define HIVE_SIZE_sp_isp_ph 28

/* function sp_tagger_destroy: 363C */

/* function init_isp_internal_buffers: 1066 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_captured_frames
#define HIVE_MEM_target_nr_of_captured_frames scalar_processor_dmem
#define HIVE_ADDR_target_nr_of_captured_frames 0x2A20
#define HIVE_SIZE_target_nr_of_captured_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_captured_frames scalar_processor_dmem
#define HIVE_ADDR_sp_target_nr_of_captured_frames 0x2A20
#define HIVE_SIZE_sp_target_nr_of_captured_frames 4

/* function sp_dma_proxy_write: 2C49 */

/* function ia_css_i_sp_refcount_init_vbuf: 411 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3414
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3414
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 20

/* function sp_dma_proxy_vmem_read: 2C09 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3428
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3428
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x343C
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x343C
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x29E4
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x29E4
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x3450
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x3450
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

/* function stop_input: 26CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_dmem
#define HIVE_ADDR_host_sp_queue 0x2C7C
#define HIVE_SIZE_host_sp_queue 1036
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x2C7C
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 2F2F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_pool
#define HIVE_MEM_isp_pool scalar_processor_dmem
#define HIVE_ADDR_isp_pool 0xF4
#define HIVE_SIZE_isp_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_pool scalar_processor_dmem
#define HIVE_ADDR_sp_isp_pool 0xF4
#define HIVE_SIZE_sp_isp_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_is_done_flag
#define HIVE_MEM_isp_is_done_flag scalar_processor_dmem
#define HIVE_ADDR_isp_is_done_flag 0x9F0
#define HIVE_SIZE_isp_is_done_flag 1
#else
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0x9F0
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_dmem
#define HIVE_ADDR_event_any_pending_mask 0x424
#define HIVE_SIZE_event_any_pending_mask 4
#else
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x424
#define HIVE_SIZE_sp_event_any_pending_mask 4

/* function ia_css_i_sp_refcount_release_vbuf: 489 */

/* function init_isp_data_segment: 10F3 */

/* function sh_css_decode_tag_descr: 52C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_dmem
#define HIVE_ADDR_dma_configs 0x23A8
#define HIVE_SIZE_dma_configs 160
#else
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_dmem
#define HIVE_ADDR_sp_dma_configs 0x23A8
#define HIVE_SIZE_sp_dma_configs 160

/* function debug_enqueue_isp: 232 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_RX_BASE
#define HIVE_MEM_RX_BASE scalar_processor_dmem
#define HIVE_ADDR_RX_BASE 0x38
#define HIVE_SIZE_RX_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_RX_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_RX_BASE 0x38
#define HIVE_SIZE_sp_RX_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x2450
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x2450
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x3464
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x3464
#define HIVE_SIZE_sp_sem_for_isp_idle 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_dmem
#define HIVE_ADDR_channels 0x2250
#define HIVE_SIZE_channels 32
#else
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_dmem
#define HIVE_ADDR_sp_channels 0x2250
#define HIVE_SIZE_sp_channels 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x29E8
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x29E8
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x29EC
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x29EC
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 85F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_VAMEM_BASE
#define HIVE_MEM_ISP_VAMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_ISP_VAMEM_BASE 0x10
#define HIVE_SIZE_ISP_VAMEM_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_VAMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_ISP_VAMEM_BASE 0x10
#define HIVE_SIZE_sp_ISP_VAMEM_BASE 8

/* function sp_tagger_create: 3656 */

/* function sp_dma_proxy_vmem_write: 2BF2 */

/* function sp_thread_set_priority: 2EC6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3088
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3088
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 3EF8 */

/* function sp_dma_proxy_write_byte_addr: 2C34 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_PMEM_BASE
#define HIVE_MEM_SP_PMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_SP_PMEM_BASE 0x0
#define HIVE_SIZE_SP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_PMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_SP_PMEM_BASE 0x0
#define HIVE_SIZE_sp_SP_PMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_input_stream_format
#define HIVE_MEM_sp_isp_input_stream_format scalar_processor_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x29F0
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x29F0
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 2F9F */

/* function __mod: 384B */

/* function __sp_event_proxy_func_critical: 3DF3 */

/* function sp_circular_buf_mark: 310A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_copied_frames
#define HIVE_MEM_target_nr_of_copied_frames scalar_processor_dmem
#define HIVE_ADDR_target_nr_of_copied_frames 0x2A24
#define HIVE_SIZE_target_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_copied_frames scalar_processor_dmem
#define HIVE_ADDR_sp_target_nr_of_copied_frames 0x2A24
#define HIVE_SIZE_sp_target_nr_of_copied_frames 4

/* function irq_raise: B2 */

/* function sp_circular_buf_unmark: 30EA */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 451 */

/* function _dma_proxy_dma_execute: 3D86 */

/* function timed_ctrl_snd_gpio_commnd: 2F2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x3478
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x3478
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x3488
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x3488
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

/* function set_sp_sleep_for_debug: 37B0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_dmem
#define HIVE_ADDR_current_sp_fiber 0xEA4
#define HIVE_SIZE_current_sp_fiber 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0xEA4
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x2454
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x2454
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x349C
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x349C
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 2B76 */

/* function sp_release_dynamic_buf: 320C */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 476 */

/* function timed_ctrl_snd_commnd: 314 */

/* function end_binary: EA2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_dmem
#define HIVE_ADDR_stacks 0x338
#define HIVE_SIZE_stacks 24
#else
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_dmem
#define HIVE_ADDR_sp_stacks 0x338
#define HIVE_SIZE_sp_stacks 24

/* function dma_proxy_dma_execute_split: 2D18 */

/* function ia_css_i_sp_refcount_dump: 435 */

/* function ia_css_i_sp_rmgr_rel_gen: 3CF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x2230
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x2230
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 2ECD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x30A8
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x30A8
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_dmem
#define HIVE_ADDR_sp_isp_addresses 0x36A4
#define HIVE_SIZE_sp_isp_addresses 176
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x36A4
#define HIVE_SIZE_sp_sp_isp_addresses 176

/* function sp_fiber_main: 2E38 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_dmem
#define HIVE_ADDR_isps 0x228C
#define HIVE_SIZE_isps 28
#else
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_dmem
#define HIVE_ADDR_sp_isps 0x228C
#define HIVE_SIZE_sp_isps 28

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x29F4
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x29F4
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function timed_ctrl_snd_sp_commnd: 303 */

/* function __sp_dma_proxy_wait_for_ack_text: 2BB2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_spref
#define HIVE_MEM_vbuf_spref scalar_processor_dmem
#define HIVE_ADDR_vbuf_spref 0x100
#define HIVE_SIZE_vbuf_spref 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_spref scalar_processor_dmem
#define HIVE_ADDR_sp_vbuf_spref 0x100
#define HIVE_SIZE_sp_vbuf_spref 4

/* function sp_circular_buf_extract: 312A */

/* function output_compute_dma_info: 19BC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x29F8
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x29F8
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_dmem
#define HIVE_ADDR_sem_for_reading_if 0x34AC
#define HIVE_SIZE_sem_for_reading_if 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x34AC
#define HIVE_SIZE_sp_sem_for_reading_if 20

/* function sp_circular_buf_pop_marked: 3026 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_dmem
#define HIVE_ADDR_pipe_private_frames 0x3118
#define HIVE_SIZE_pipe_private_frames 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x3118
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 3676 */

/* function init_isp_vars: 1D13 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x3148
#define HIVE_SIZE_sems_for_host2sp_buf_queues 560
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x3148
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 560

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_dmem
#define HIVE_ADDR_sp_data 0x2504
#define HIVE_SIZE_sp_data 640
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_dmem
#define HIVE_ADDR_sp_sp_data 0x2504
#define HIVE_SIZE_sp_sp_data 640

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x420
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x420
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function acquire_isp: 37CF */

/* function sp_circular_buf_is_marked: 30CC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_dmem
#define HIVE_ADDR_mem_map 0x2458
#define HIVE_SIZE_mem_map 100
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_dmem
#define HIVE_ADDR_sp_mem_map 0x2458
#define HIVE_SIZE_sp_mem_map 100

/* function sp_init_dmem: 27D6 */

/* function ia_css_i_sp_refcount_retain_vbuf: 4A1 */

/* function init_isp_code_segment: F4A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0x41C
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0x41C
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function run_sp_threads: 2F44 */

/* function configure_dma_channel: 284B */

/* function sp_thread_queue_print: 2F5A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_dmem
#define HIVE_ADDR_sp_flash_in_service 0xEB4
#define HIVE_SIZE_sp_flash_in_service 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0xEB4
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x29FC
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x29FC
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 2F6B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_dmem
#define HIVE_ADDR_sp_sleep_mode 0x2A00
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x2A00
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: BC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_dmem
#define HIVE_ADDR_isp_stop_req 0x2A34
#define HIVE_SIZE_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x2A34
#define HIVE_SIZE_sp_isp_stop_req 4

/* function release_isp: 37BD */

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
