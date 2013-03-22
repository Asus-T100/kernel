#ifndef _sp_map_h_
#define _sp_map_h_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_input_line
#define HIVE_MEM_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_input_line 0x2720
#define HIVE_SIZE_isp_vectors_per_input_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_input_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_input_line 0x2720
#define HIVE_SIZE_sp_isp_vectors_per_input_line 4

/* function sp_tagger_propagate_frame: 330E */

/* function longjmp: 364B */

/* function sp_tagger_tag_exp_id: 3288 */

/* function sp_dma_proxy_read: 285C */

/* function sp_dma_proxy_is_idle: 2971 */

/* function stop_event_proxy: 297F */

/* function setjmp: 3652 */

/* function decode_sw_event: 2EF */

/* function initialize_sp_stage: 3735 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stage
#define HIVE_MEM_isp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_isp_stage 0x351C
#define HIVE_SIZE_isp_stage 516
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stage scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_stage 0x351C
#define HIVE_SIZE_sp_isp_stage 516

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_raw
#define HIVE_MEM_vbuf_raw scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_raw 0x8C
#define HIVE_SIZE_vbuf_raw 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_raw scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_raw 0x8C
#define HIVE_SIZE_sp_vbuf_raw 4

/* function release_in_param: 304E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_proxy_status
#define HIVE_MEM_dma_proxy_status scalar_processor_demo_dmem
#define HIVE_ADDR_dma_proxy_status 0x1B0
#define HIVE_SIZE_dma_proxy_status 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_proxy_status scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_proxy_status 0x1B0
#define HIVE_SIZE_sp_dma_proxy_status 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_mmu_invalidation
#define HIVE_MEM_do_mmu_invalidation scalar_processor_demo_dmem
#define HIVE_ADDR_do_mmu_invalidation 0x29BC
#define HIVE_SIZE_do_mmu_invalidation 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_mmu_invalidation scalar_processor_demo_dmem
#define HIVE_ADDR_sp_do_mmu_invalidation 0x29BC
#define HIVE_SIZE_sp_do_mmu_invalidation 4

/* function sp_start_isp: 1E44 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_binary_group 0x24B8
#define HIVE_SIZE_sp_binary_group 68
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x24B8
#define HIVE_SIZE_sp_sp_binary_group 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sw_state 0x2724
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x2724
#define HIVE_SIZE_sp_sp_sw_state 4

/* function sp_circular_buf_destroy: 2DDE */

/* function sp_circular_buf_pop: 2DAF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stop_copy_preview
#define HIVE_MEM_sp_stop_copy_preview scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stop_copy_preview 0x2998
#define HIVE_SIZE_sp_stop_copy_preview 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stop_copy_preview scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_stop_copy_preview 0x2998
#define HIVE_SIZE_sp_sp_stop_copy_preview 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_handles
#define HIVE_MEM_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_handles 0x22B8
#define HIVE_SIZE_vbuf_handles 200
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_handles scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_handles 0x22B8
#define HIVE_SIZE_sp_vbuf_handles 200

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_capture_pipe
#define HIVE_MEM_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_capture_pipe 0x33CC
#define HIVE_SIZE_cb_elems_params_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_capture_pipe 0x33CC
#define HIVE_SIZE_sp_cb_elems_params_capture_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_proxy_thread
#define HIVE_MEM_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_proxy_thread 0x29D0
#define HIVE_SIZE_sp_dma_proxy_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_proxy_thread 0x29D0
#define HIVE_SIZE_sp_sp_dma_proxy_thread 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_thread_ready_queue
#define HIVE_MEM_sp_thread_ready_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_thread_ready_queue 0x238
#define HIVE_SIZE_sp_thread_ready_queue 8
#else
#endif
#endif
#define HIVE_MEM_sp_sp_thread_ready_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_thread_ready_queue 0x238
#define HIVE_SIZE_sp_sp_thread_ready_queue 8

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
#define HIVE_ADDR_host_sp_com 0x2728
#define HIVE_SIZE_host_sp_com 76
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_com 0x2728
#define HIVE_SIZE_sp_host_sp_com 76

/* function exec_image_pipe: 201B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x2774
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x2774
#define HIVE_SIZE_sp_sp_init_dmem_data 24

/* function stop_threads: 1FF2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_use
#define HIVE_MEM_sp_flash_in_use scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flash_in_use 0xDD0
#define HIVE_SIZE_sp_flash_in_use 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_use scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_flash_in_use 0xDD0
#define HIVE_SIZE_sp_sp_flash_in_use 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_flashed_frame_cnt
#define HIVE_MEM_flashed_frame_cnt scalar_processor_demo_dmem
#define HIVE_ADDR_flashed_frame_cnt 0xDD8
#define HIVE_SIZE_flashed_frame_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_flashed_frame_cnt scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flashed_frame_cnt 0xDD8
#define HIVE_SIZE_sp_flashed_frame_cnt 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stack_sizes
#define HIVE_MEM_stack_sizes scalar_processor_demo_dmem
#define HIVE_ADDR_stack_sizes 0x268
#define HIVE_SIZE_stack_sizes 24
#else
#endif
#endif
#define HIVE_MEM_sp_stack_sizes scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stack_sizes 0x268
#define HIVE_SIZE_sp_stack_sizes 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ph
#define HIVE_MEM_ph scalar_processor_demo_dmem
#define HIVE_ADDR_ph 0x2234
#define HIVE_SIZE_ph 32
#else
#endif
#endif
#define HIVE_MEM_sp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ph 0x2234
#define HIVE_SIZE_sp_ph 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_per_frame_data 0x278C
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x278C
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function dma_proxy_channel_acquire: 3BED */

/* function sp_tagger_connect_pipes: 34AE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pipe_thread
#define HIVE_MEM_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_copy_pipe_thread 0x2A14
#define HIVE_SIZE_sp_copy_pipe_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_copy_pipe_thread 0x2A14
#define HIVE_SIZE_sp_sp_copy_pipe_thread 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_xmem_bin_addr
#define HIVE_MEM_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_xmem_bin_addr 0x2790
#define HIVE_SIZE_xmem_bin_addr 4
#else
#endif
#endif
#define HIVE_MEM_sp_xmem_bin_addr scalar_processor_demo_dmem
#define HIVE_ADDR_sp_xmem_bin_addr 0x2790
#define HIVE_SIZE_sp_xmem_bin_addr 4

/* function memcpy: 36C6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_threads
#define HIVE_MEM_pipe_threads scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_threads 0x240
#define HIVE_SIZE_pipe_threads 16
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_threads scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_threads 0x240
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

/* function sp_dma_proxy_set_width_ab: 27B1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_i_exp_id
#define HIVE_MEM_ia_css_i_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_ia_css_i_exp_id 0x318
#define HIVE_SIZE_ia_css_i_exp_id 1
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_i_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ia_css_i_exp_id 0x318
#define HIVE_SIZE_sp_ia_css_i_exp_id 1

/* function __divu: 3658 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_stop
#define HIVE_MEM_sem_for_cont_capt_stop scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_cont_capt_stop 0x33DC
#define HIVE_SIZE_sem_for_cont_capt_stop 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_stop scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_stop 0x33DC
#define HIVE_SIZE_sp_sem_for_cont_capt_stop 24

/* function sp_dma_proxy_func: 37BD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_started
#define HIVE_MEM_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_started 0x2794
#define HIVE_SIZE_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_started scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_started 0x2794
#define HIVE_SIZE_sp_sp_isp_started 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_pipe_thread
#define HIVE_MEM_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_pipe_thread 0x2A58
#define HIVE_SIZE_sp_isp_pipe_thread 204
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_pipe_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_pipe_thread 0x2A58
#define HIVE_SIZE_sp_sp_isp_pipe_thread 204

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_start_bq
#define HIVE_MEM_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_start_bq 0x2798
#define HIVE_SIZE_sp_obarea_start_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_start_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_start_bq 0x2798
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

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_is_isp_requested
#define HIVE_MEM_is_isp_requested scalar_processor_demo_dmem
#define HIVE_ADDR_is_isp_requested 0xDAC
#define HIVE_SIZE_is_isp_requested 4
#else
#endif
#endif
#define HIVE_MEM_sp_is_isp_requested scalar_processor_demo_dmem
#define HIVE_ADDR_sp_is_isp_requested 0xDAC
#define HIVE_SIZE_sp_is_isp_requested 4

/* function ia_css_i_sp_rmgr_init: 12B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_current_sp_thread 0x234
#define HIVE_SIZE_current_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x234
#define HIVE_SIZE_sp_current_sp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_h_pipe_private_ddr_ptrs
#define HIVE_MEM_h_pipe_private_ddr_ptrs scalar_processor_demo_dmem
#define HIVE_ADDR_h_pipe_private_ddr_ptrs 0x2B68
#define HIVE_SIZE_h_pipe_private_ddr_ptrs 16
#else
#endif
#endif
#define HIVE_MEM_sp_h_pipe_private_ddr_ptrs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_h_pipe_private_ddr_ptrs 0x2B68
#define HIVE_SIZE_sp_h_pipe_private_ddr_ptrs 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_capture_thread_id
#define HIVE_MEM_sp_capture_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_capture_thread_id 0x3BC
#define HIVE_SIZE_sp_capture_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_capture_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_capture_thread_id 0x3BC
#define HIVE_SIZE_sp_sp_capture_thread_id 4

/* function sp_turn_off_flash: 2A5A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_internal_event
#define HIVE_MEM_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_internal_event 0x33B0
#define HIVE_SIZE_sp_internal_event 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_internal_event scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_internal_event 0x33B0
#define HIVE_SIZE_sp_sp_internal_event 4

/* function _sp_dma_proxy_init_isp_vector: 27C7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_last_index
#define HIVE_MEM_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_last_index 0x2380
#define HIVE_SIZE_last_index 4
#else
#endif
#endif
#define HIVE_MEM_sp_last_index scalar_processor_demo_dmem
#define HIVE_ADDR_sp_last_index 0x2380
#define HIVE_SIZE_sp_last_index 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sh_dma_cmd_buffer
#define HIVE_MEM_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sh_dma_cmd_buffer 0x29C0
#define HIVE_SIZE_isp_sh_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sh_dma_cmd_buffer scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sh_dma_cmd_buffer 0x29C0
#define HIVE_SIZE_sp_isp_sh_dma_cmd_buffer 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_cont_capt_start
#define HIVE_MEM_sem_for_cont_capt_start scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_cont_capt_start 0x33F4
#define HIVE_SIZE_sem_for_cont_capt_start 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_cont_capt_start scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_cont_capt_start 0x33F4
#define HIVE_SIZE_sp_sem_for_cont_capt_start 24

/* function host2sp_event_queue_is_empty: 36FB */

/* function ia_css_i_sp_rmgr_get_num_vbuf: 2C1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_preview_thread_id
#define HIVE_MEM_sp_preview_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_preview_thread_id 0x3B8
#define HIVE_SIZE_sp_preview_thread_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_preview_thread_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_preview_thread_id 0x3B8
#define HIVE_SIZE_sp_sp_preview_thread_id 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_capture_pipe 0x340C
#define HIVE_SIZE_sem_for_reading_cb_frames_capture_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_capture_pipe 0x340C
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_capture_pipe 24

/* function sp_event_proxy_func: 2986 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_fibers
#define HIVE_MEM_fibers scalar_processor_demo_dmem
#define HIVE_ADDR_fibers 0x280
#define HIVE_SIZE_fibers 24
#else
#endif
#endif
#define HIVE_MEM_sp_fibers scalar_processor_demo_dmem
#define HIVE_ADDR_sp_fibers 0x280
#define HIVE_SIZE_sp_fibers 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_preview_pipe
#define HIVE_MEM_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_params_preview_pipe 0x3424
#define HIVE_SIZE_cb_params_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_preview_pipe 0x3424
#define HIVE_SIZE_sp_cb_params_preview_pipe 20

/* function sp_semaphore_init: 3DBE */

/* function initialize_sp_group: 2496 */

/* function start_binary: D48 */

/* function sp_tagger_configure: 32C3 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_invalidate_tlb
#define HIVE_MEM_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_invalidate_tlb 0x29C4
#define HIVE_SIZE_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_invalidate_tlb scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_invalidate_tlb 0x29C4
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

/* function bin_spec_init_ifs: 398 */

/* function dma_proxy_channel_release: 3BDC */

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

/* function sp_dma_proxy_wait_for_ack: 3AA4 */

/* function sp_thread_yield: 3CA7 */

/* function sp_circular_buf_peek: 2C04 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_thread
#define HIVE_MEM_isp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_isp_thread 0x3720
#define HIVE_SIZE_isp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_thread 0x3720
#define HIVE_SIZE_sp_isp_thread 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_obarea_length_bq
#define HIVE_MEM_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_obarea_length_bq 0x279C
#define HIVE_SIZE_sp_obarea_length_bq 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_obarea_length_bq scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_obarea_length_bq 0x279C
#define HIVE_SIZE_sp_sp_obarea_length_bq 4

/* function sp_uds_init: 5CE */

/* function sp_dma_proxy_isp_write_addr: 280A */

/* function sp_circular_buf_create: 2DE8 */

/* function host2sp_dequeue_buffer: 10C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buf_swap
#define HIVE_MEM_buf_swap scalar_processor_demo_dmem
#define HIVE_ADDR_buf_swap 0x34C
#define HIVE_SIZE_buf_swap 96
#else
#endif
#endif
#define HIVE_MEM_sp_buf_swap scalar_processor_demo_dmem
#define HIVE_ADDR_sp_buf_swap 0x34C
#define HIVE_SIZE_sp_buf_swap 96

/* function is_dynamic_buffer: 2FB4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_curr_nr_of_copied_frames
#define HIVE_MEM_curr_nr_of_copied_frames scalar_processor_demo_dmem
#define HIVE_ADDR_curr_nr_of_copied_frames 0x299C
#define HIVE_SIZE_curr_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_curr_nr_of_copied_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_curr_nr_of_copied_frames 0x299C
#define HIVE_SIZE_sp_curr_nr_of_copied_frames 4

/* function sp2host_enqueue_buffer: F2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_b
#define HIVE_MEM_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_b 0x2444
#define HIVE_SIZE_sp_dma_crop_block_width_b 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_b scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_b 0x2444
#define HIVE_SIZE_sp_sp_dma_crop_block_width_b 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_params_preview_pipe
#define HIVE_MEM_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_params_preview_pipe 0x3438
#define HIVE_SIZE_cb_elems_params_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_params_preview_pipe 0x3438
#define HIVE_SIZE_sp_cb_elems_params_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_output 0x27A0
#define HIVE_SIZE_sp_output 16
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_output 0x27A0
#define HIVE_SIZE_sp_sp_output 16

/* function init_buffer_queues: 31AF */

/* function sp_tagger_propagate_past_frames: 31BC */

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

/* function sp_tagger_update_size: 34FB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_frame_pool
#define HIVE_MEM_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_raw_frame_pool 0x2384
#define HIVE_SIZE_raw_frame_pool 16
#else
#endif
#endif
#define HIVE_MEM_sp_raw_frame_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_raw_frame_pool 0x2384
#define HIVE_SIZE_sp_raw_frame_pool 16

/* function sp_raw_copy_func: 24D0 */

/* function __sp_dma_proxy_configure_channel_text: 28B2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_sp2host_event_queue
#define HIVE_MEM_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_sp2host_event_queue 0x33B4
#define HIVE_SIZE_sem_for_sp2host_event_queue 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_sp2host_event_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_sp2host_event_queue 0x33B4
#define HIVE_SIZE_sp_sem_for_sp2host_event_queue 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_copy_pack
#define HIVE_MEM_sp_copy_pack scalar_processor_demo_dmem
#define HIVE_ADDR_sp_copy_pack 0x29A0
#define HIVE_SIZE_sp_copy_pack 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_copy_pack scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_copy_pack 0x29A0
#define HIVE_SIZE_sp_sp_copy_pack 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_tagger
#define HIVE_MEM_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_tagger 0x29A4
#define HIVE_SIZE_tagger 20
#else
#endif
#endif
#define HIVE_MEM_sp_tagger scalar_processor_demo_dmem
#define HIVE_ADDR_sp_tagger 0x29A4
#define HIVE_SIZE_sp_tagger 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_refpool
#define HIVE_MEM_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_refpool 0x2394
#define HIVE_SIZE_refpool 16
#else
#endif
#endif
#define HIVE_MEM_sp_refpool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_refpool 0x2394
#define HIVE_SIZE_sp_refpool 16

/* function host2sp_dequeue_sp_event: D2 */

/* function sp_start_isp_entry: 1E3E */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x1E3E
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x1E3E

/* function __sp_raw_copy_func_critical: 376F */

/* function add_sp_command: 3BC0 */

/* function sp2host_enqueue_irq_event: BB */

/* function create_sp_fibers: 29DE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_s3a_bufs
#define HIVE_MEM_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_s3a_bufs 0x2B78
#define HIVE_SIZE_pipe_private_s3a_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_s3a_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_s3a_bufs 0x2B78
#define HIVE_SIZE_sp_pipe_private_s3a_bufs 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_block_width_a
#define HIVE_MEM_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_block_width_a 0x2448
#define HIVE_SIZE_sp_dma_crop_block_width_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_block_width_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_block_width_a 0x2448
#define HIVE_SIZE_sp_sp_dma_crop_block_width_a 4

/* function sp_bin_copy_func: 315 */

/* function ia_css_i_sp_rmgr_acq_gen: 16D */

/* function sp_turn_on_flash: 2A6D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_sp2host_buf_queues
#define HIVE_MEM_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_sp2host_buf_queues 0x2B98
#define HIVE_SIZE_sems_for_sp2host_buf_queues 168
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_sp2host_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_sp2host_buf_queues 0x2B98
#define HIVE_SIZE_sp_sems_for_sp2host_buf_queues 168

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vectors_per_line
#define HIVE_MEM_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vectors_per_line 0x27B0
#define HIVE_SIZE_isp_vectors_per_line 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vectors_per_line scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vectors_per_line 0x27B0
#define HIVE_SIZE_sp_isp_vectors_per_line 4

/* function __sp_dma_proxy_func_text: 2737 */

/* function sp_thread_join: 2AF4 */

/* function sp_dma_proxy_configure_channel: 3ACD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_do_irq_raise
#define HIVE_MEM_do_irq_raise scalar_processor_demo_dmem
#define HIVE_ADDR_do_irq_raise 0x8EC
#define HIVE_SIZE_do_irq_raise 4
#else
#endif
#endif
#define HIVE_MEM_sp_do_irq_raise scalar_processor_demo_dmem
#define HIVE_ADDR_sp_do_irq_raise 0x8EC
#define HIVE_SIZE_sp_do_irq_raise 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_group 0x27B4
#define HIVE_SIZE_sp_group 444
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_group 0x27B4
#define HIVE_SIZE_sp_sp_group 444

/* function sp2host_buffer_queue_get_size: E7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_event_proxy_thread
#define HIVE_MEM_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_proxy_thread 0x2B24
#define HIVE_SIZE_sp_event_proxy_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_sp_event_proxy_thread scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_event_proxy_thread 0x2B24
#define HIVE_SIZE_sp_sp_event_proxy_thread 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_pool
#define HIVE_MEM_dma_pool scalar_processor_demo_dmem
#define HIVE_ADDR_dma_pool 0x80
#define HIVE_SIZE_dma_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_dma_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_pool 0x80
#define HIVE_SIZE_sp_dma_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_uv_internal_width_vecs
#define HIVE_MEM_isp_uv_internal_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_isp_uv_internal_width_vecs 0x2970
#define HIVE_SIZE_isp_uv_internal_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_uv_internal_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_uv_internal_width_vecs 0x2970
#define HIVE_SIZE_sp_isp_uv_internal_width_vecs 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_exp_id
#define HIVE_MEM_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_exp_id 0x2C40
#define HIVE_SIZE_pipe_private_exp_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_exp_id scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_exp_id 0x2C40
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

/* function sp_dma_proxy_configure_init_dmem_channel: 2872 */

/* function sp2host_event_queue_get_size: B3 */

/* function sp_dma_proxy_read_byte_addr: 3AB9 */

/* function sp_thread_fork: 2B02 */

/* function sp_semaphore_wait: 3D39 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_frames_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_frames_preview_pipe 0x3448
#define HIVE_SIZE_sem_for_reading_cb_frames_preview_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_frames_preview_pipe 0x3448
#define HIVE_SIZE_sp_sem_for_reading_cb_frames_preview_pipe 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_request_flash
#define HIVE_MEM_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_request_flash 0x29CC
#define HIVE_SIZE_sp_request_flash 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_request_flash scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_request_flash 0x29CC
#define HIVE_SIZE_sp_sp_request_flash 4

/* function cnd_input_system_cfg: 2368 */

/* function sp_generate_events: 3587 */

/* function sp_uds_configure: 426 */

/* function sp_dma_proxy_execute: 284B */

/* function __modu: 3697 */

/* function sp_circular_buf_push_marked: 2C5C */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horiproj_num
#define HIVE_MEM_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horiproj_num 0x2974
#define HIVE_SIZE_isp_sdis_horiproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horiproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horiproj_num 0x2974
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

/* function sp_fiber_init: 2A4B */

/* function ia_css_i_sp_rmgr_uninit: 126 */

/* function wait_for_in_frame: 2FC7 */

/* function sp_thread_init: 2B22 */

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

/* function _dma_proxy_dma_read_write: 3B50 */

/* function sp_dma_proxy_configure_init_vmem_channel: 2892 */

/* function sp_acquire_dynamic_buf: 2F29 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_ph
#define HIVE_MEM_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_isp_ph 0x2278
#define HIVE_SIZE_isp_ph 32
#else
#endif
#endif
#define HIVE_MEM_sp_isp_ph scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_ph 0x2278
#define HIVE_SIZE_sp_isp_ph 32

/* function sp_tagger_destroy: 34B7 */

/* function init_isp_internal_buffers: E82 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_captured_frames
#define HIVE_MEM_target_nr_of_captured_frames scalar_processor_demo_dmem
#define HIVE_ADDR_target_nr_of_captured_frames 0x8E0
#define HIVE_SIZE_target_nr_of_captured_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_captured_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_target_nr_of_captured_frames 0x8E0
#define HIVE_SIZE_sp_target_nr_of_captured_frames 4

/* function sp_dma_proxy_write: 2834 */

/* function ia_css_i_sp_refcount_init_vbuf: 19E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_preview_pipe
#define HIVE_MEM_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_preview_pipe 0x3460
#define HIVE_SIZE_sem_for_reading_cb_params_preview_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_preview_pipe 0x3460
#define HIVE_SIZE_sp_sem_for_reading_cb_params_preview_pipe 24

/* function sp_dma_proxy_vmem_read: 27F4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_cb_params_capture_pipe
#define HIVE_MEM_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_cb_params_capture_pipe 0x3478
#define HIVE_SIZE_sem_for_reading_cb_params_capture_pipe 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_cb_params_capture_pipe 0x3478
#define HIVE_SIZE_sp_sem_for_reading_cb_params_capture_pipe 24

/* function release_in_frame: 2FEA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_params_capture_pipe
#define HIVE_MEM_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_params_capture_pipe 0x3490
#define HIVE_SIZE_cb_params_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_params_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_params_capture_pipe 0x3490
#define HIVE_SIZE_sp_cb_params_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertproj_num
#define HIVE_MEM_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertproj_num 0x2978
#define HIVE_SIZE_isp_sdis_vertproj_num 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertproj_num scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertproj_num 0x2978
#define HIVE_SIZE_sp_isp_sdis_vertproj_num 4

/* function wait_for_in_param: 300D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_capture_pipe
#define HIVE_MEM_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_capture_pipe 0x34A4
#define HIVE_SIZE_cb_frames_capture_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_capture_pipe 0x34A4
#define HIVE_SIZE_sp_cb_frames_capture_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queue
#define HIVE_MEM_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queue 0x2C44
#define HIVE_SIZE_host_sp_queue 1036
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queue scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queue 0x2C44
#define HIVE_SIZE_sp_host_sp_queue 1036

/* function sp_thread_main: 2B3B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_pool
#define HIVE_MEM_isp_pool scalar_processor_demo_dmem
#define HIVE_ADDR_isp_pool 0x84
#define HIVE_SIZE_isp_pool 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_pool scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_pool 0x84
#define HIVE_SIZE_sp_isp_pool 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_is_done_flag
#define HIVE_MEM_isp_is_done_flag scalar_processor_demo_dmem
#define HIVE_ADDR_isp_is_done_flag 0x8F0
#define HIVE_SIZE_isp_is_done_flag 1
#else
#endif
#endif
#define HIVE_MEM_sp_isp_is_done_flag scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_is_done_flag 0x8F0
#define HIVE_SIZE_sp_isp_is_done_flag 1

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_event_any_pending_mask 0x314
#define HIVE_SIZE_event_any_pending_mask 4
#else
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_demo_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x314
#define HIVE_SIZE_sp_event_any_pending_mask 4

/* function ia_css_i_sp_refcount_release_vbuf: 217 */

/* function init_isp_data_segment: F0F */

/* function sh_css_decode_tag_descr: 2C7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dma_configs
#define HIVE_MEM_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_dma_configs 0x23A4
#define HIVE_SIZE_dma_configs 160
#else
#endif
#endif
#define HIVE_MEM_sp_dma_configs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_configs 0x23A4
#define HIVE_SIZE_sp_dma_configs 160

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_crop_cropping_a
#define HIVE_MEM_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_crop_cropping_a 0x244C
#define HIVE_SIZE_sp_dma_crop_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_crop_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_crop_cropping_a 0x244C
#define HIVE_SIZE_sp_sp_dma_crop_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x34B8
#define HIVE_SIZE_sem_for_isp_idle 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x34B8
#define HIVE_SIZE_sp_sem_for_isp_idle 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_channels
#define HIVE_MEM_channels scalar_processor_demo_dmem
#define HIVE_ADDR_channels 0x2254
#define HIVE_SIZE_channels 36
#else
#endif
#endif
#define HIVE_MEM_sp_channels scalar_processor_demo_dmem
#define HIVE_ADDR_sp_channels 0x2254
#define HIVE_SIZE_sp_channels 36

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_vf_downscale_bits
#define HIVE_MEM_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vf_downscale_bits 0x297C
#define HIVE_SIZE_sp_vf_downscale_bits 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_vf_downscale_bits scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_vf_downscale_bits 0x297C
#define HIVE_SIZE_sp_sp_vf_downscale_bits 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_vertcoef_vectors
#define HIVE_MEM_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_vertcoef_vectors 0x2980
#define HIVE_SIZE_isp_sdis_vertcoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_vertcoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_vertcoef_vectors 0x2980
#define HIVE_SIZE_sp_isp_sdis_vertcoef_vectors 4

/* function dma_configure: 60B */

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

/* function handle_parameter_sets: 3077 */

/* function sp_tagger_create: 34D1 */

/* function sp_dma_proxy_vmem_write: 27DD */

/* function sp_thread_set_priority: 2AD1 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_dis_bufs
#define HIVE_MEM_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_dis_bufs 0x3050
#define HIVE_SIZE_pipe_private_dis_bufs 32
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_dis_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_dis_bufs 0x3050
#define HIVE_SIZE_sp_pipe_private_dis_bufs 32

/* function sp_semaphore_signal: 3D04 */

/* function sp_dma_proxy_write_byte_addr: 281F */

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
#define HIVE_ADDR_sp_isp_input_stream_format 0x2984
#define HIVE_SIZE_sp_isp_input_stream_format 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x2984
#define HIVE_SIZE_sp_sp_isp_input_stream_format 4

/* function sp_circular_buf_push_unmarked: 2BAB */

/* function __mod: 3682 */

/* function __sp_event_proxy_func_critical: 3BFE */

/* function sp_circular_buf_mark: 2CFB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_target_nr_of_copied_frames
#define HIVE_MEM_target_nr_of_copied_frames scalar_processor_demo_dmem
#define HIVE_ADDR_target_nr_of_copied_frames 0x29B8
#define HIVE_SIZE_target_nr_of_copied_frames 4
#else
#endif
#endif
#define HIVE_MEM_sp_target_nr_of_copied_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_target_nr_of_copied_frames 0x29B8
#define HIVE_SIZE_sp_target_nr_of_copied_frames 4

/* function irq_raise: A1 */

/* function sp_circular_buf_unmark: 2CDA */

/* function ia_css_i_sp_rmgr_vbuf_dequeue: 1DF */

/* function _dma_proxy_dma_execute: 2906 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_preview_pipe
#define HIVE_MEM_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_preview_pipe 0x34D0
#define HIVE_SIZE_cb_elems_frames_preview_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_preview_pipe 0x34D0
#define HIVE_SIZE_sp_cb_elems_frames_preview_pipe 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_frames_preview_pipe
#define HIVE_MEM_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_frames_preview_pipe 0x34E0
#define HIVE_SIZE_cb_frames_preview_pipe 20
#else
#endif
#endif
#define HIVE_MEM_sp_cb_frames_preview_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_frames_preview_pipe 0x34E0
#define HIVE_SIZE_sp_cb_frames_preview_pipe 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_fiber
#define HIVE_MEM_current_sp_fiber scalar_processor_demo_dmem
#define HIVE_ADDR_current_sp_fiber 0xDC4
#define HIVE_SIZE_current_sp_fiber 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_fiber scalar_processor_demo_dmem
#define HIVE_ADDR_sp_current_sp_fiber 0xDC4
#define HIVE_SIZE_sp_current_sp_fiber 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_dma_vfout_cropping_a
#define HIVE_MEM_sp_dma_vfout_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_dma_vfout_cropping_a 0x2450
#define HIVE_SIZE_sp_dma_vfout_cropping_a 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_dma_vfout_cropping_a scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_dma_vfout_cropping_a 0x2450
#define HIVE_SIZE_sp_sp_dma_vfout_cropping_a 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cb_elems_frames_capture_pipe
#define HIVE_MEM_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_cb_elems_frames_capture_pipe 0x34F4
#define HIVE_SIZE_cb_elems_frames_capture_pipe 16
#else
#endif
#endif
#define HIVE_MEM_sp_cb_elems_frames_capture_pipe scalar_processor_demo_dmem
#define HIVE_ADDR_sp_cb_elems_frames_capture_pipe 0x34F4
#define HIVE_SIZE_sp_cb_elems_frames_capture_pipe 16

/* function sp_dma_proxy_init: 275D */

/* function sp_release_dynamic_buf: 2DFF */

/* function ia_css_i_sp_rmgr_vbuf_enqueue: 204 */

/* function end_binary: CC4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_stacks
#define HIVE_MEM_stacks scalar_processor_demo_dmem
#define HIVE_ADDR_stacks 0x250
#define HIVE_SIZE_stacks 24
#else
#endif
#endif
#define HIVE_MEM_sp_stacks scalar_processor_demo_dmem
#define HIVE_ADDR_sp_stacks 0x250
#define HIVE_SIZE_sp_stacks 24

/* function dma_proxy_dma_execute_split: 2933 */

/* function ia_css_i_sp_refcount_dump: 1C3 */

/* function ia_css_i_sp_rmgr_rel_gen: 15A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x2230
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_demo_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x2230
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

/* function sp_thread_kill: 2AD8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_buffer_bufs
#define HIVE_MEM_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_buffer_bufs 0x3070
#define HIVE_SIZE_pipe_private_buffer_bufs 112
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_buffer_bufs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_buffer_bufs 0x3070
#define HIVE_SIZE_sp_pipe_private_buffer_bufs 112

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_addresses 0x2174
#define HIVE_SIZE_sp_isp_addresses 188
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x2174
#define HIVE_SIZE_sp_sp_isp_addresses 188

/* function sp_fiber_main: 2A53 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isps
#define HIVE_MEM_isps scalar_processor_demo_dmem
#define HIVE_ADDR_isps 0x2298
#define HIVE_SIZE_isps 32
#else
#endif
#endif
#define HIVE_MEM_sp_isps scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isps 0x2298
#define HIVE_SIZE_sp_isps 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x2988
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_demo_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x2988
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function __sp_dma_proxy_wait_for_ack_text: 2799 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vbuf_spref
#define HIVE_MEM_vbuf_spref scalar_processor_demo_dmem
#define HIVE_ADDR_vbuf_spref 0x88
#define HIVE_SIZE_vbuf_spref 4
#else
#endif
#endif
#define HIVE_MEM_sp_vbuf_spref scalar_processor_demo_dmem
#define HIVE_ADDR_sp_vbuf_spref 0x88
#define HIVE_SIZE_sp_vbuf_spref 4

/* function sp_circular_buf_extract: 2D1B */

/* function output_compute_dma_info: 1838 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_sdis_horicoef_vectors
#define HIVE_MEM_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_isp_sdis_horicoef_vectors 0x298C
#define HIVE_SIZE_isp_sdis_horicoef_vectors 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_sdis_horicoef_vectors scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_sdis_horicoef_vectors 0x298C
#define HIVE_SIZE_sp_isp_sdis_horicoef_vectors 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_reading_if
#define HIVE_MEM_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sem_for_reading_if 0x3504
#define HIVE_SIZE_sem_for_reading_if 24
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_reading_if scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sem_for_reading_if 0x3504
#define HIVE_SIZE_sp_sem_for_reading_if 24

/* function sp_circular_buf_pop_marked: 2C23 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipe_private_frames
#define HIVE_MEM_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_pipe_private_frames 0x30E0
#define HIVE_SIZE_pipe_private_frames 48
#else
#endif
#endif
#define HIVE_MEM_sp_pipe_private_frames scalar_processor_demo_dmem
#define HIVE_ADDR_sp_pipe_private_frames 0x30E0
#define HIVE_SIZE_sp_pipe_private_frames 48

/* function sp_generate_interrupts: 3502 */

/* function ia_css_i_sp_rmgr_update_num_vbuf: 2BA */

/* function init_isp_vars: 1B94 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sems_for_host2sp_buf_queues
#define HIVE_MEM_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sems_for_host2sp_buf_queues 0x3110
#define HIVE_SIZE_sems_for_host2sp_buf_queues 672
#else
#endif
#endif
#define HIVE_MEM_sp_sems_for_host2sp_buf_queues scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sems_for_host2sp_buf_queues 0x3110
#define HIVE_SIZE_sp_sems_for_host2sp_buf_queues 672

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_data 0x24FC
#define HIVE_SIZE_sp_data 548
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_data 0x24FC
#define HIVE_SIZE_sp_sp_data 548

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x310
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x310
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

/* function sp_circular_buf_is_marked: 2CBB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_mem_map 0x2454
#define HIVE_SIZE_mem_map 100
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_demo_dmem
#define HIVE_ADDR_sp_mem_map 0x2454
#define HIVE_SIZE_sp_mem_map 100

/* function sp_init_dmem: 243E */

/* function ia_css_i_sp_refcount_retain_vbuf: 22F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_thread_is_active
#define HIVE_MEM_thread_is_active scalar_processor_demo_dmem
#define HIVE_ADDR_thread_is_active 0x298
#define HIVE_SIZE_thread_is_active 16
#else
#endif
#endif
#define HIVE_MEM_sp_thread_is_active scalar_processor_demo_dmem
#define HIVE_ADDR_sp_thread_is_active 0x298
#define HIVE_SIZE_sp_thread_is_active 16

/* function init_isp_code_segment: D6C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0x30C
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_demo_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0x30C
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function run_sp_threads: 2B50 */

/* function configure_dma_channel: 24B3 */

/* function sp_thread_queue_print: 2B66 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_flash_in_service
#define HIVE_MEM_sp_flash_in_service scalar_processor_demo_dmem
#define HIVE_ADDR_sp_flash_in_service 0xDD4
#define HIVE_SIZE_sp_flash_in_service 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_flash_in_service scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_flash_in_service 0xDD4
#define HIVE_SIZE_sp_sp_flash_in_service 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_vf_output_width_vecs
#define HIVE_MEM_isp_vf_output_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_isp_vf_output_width_vecs 0x2990
#define HIVE_SIZE_isp_vf_output_width_vecs 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_vf_output_width_vecs scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_vf_output_width_vecs 0x2990
#define HIVE_SIZE_sp_isp_vf_output_width_vecs 4

/* function sp_circular_buf_pop_unmarked: 2B77 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sleep_mode 0x2994
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_demo_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x2994
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stop_req
#define HIVE_MEM_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_isp_stop_req 0x29C8
#define HIVE_SIZE_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stop_req scalar_processor_demo_dmem
#define HIVE_ADDR_sp_isp_stop_req 0x29C8
#define HIVE_SIZE_sp_isp_stop_req 4

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
