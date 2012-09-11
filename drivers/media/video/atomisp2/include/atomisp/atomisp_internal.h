/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
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
#ifndef ATOMISP_INTERNAL_H_
#define ATOMISP_INTERNAL_H_

#ifdef CONFIG_BOARD_CTP
#include <linux/intel_mid_pm.h>
#endif

#include <linux/pm_qos_params.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>
#include <media/videobuf-vmalloc.h>
#include <linux/firmware.h>
#include <media/media-device.h>
#include <linux/kernel.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>

#include <linux/atomisp.h>
#include <linux/atomisp_platform.h>

#include <sh_css_types.h>
#include "gp_device.h"
#include "irq.h"

#include "atomisp_subdev.h"
#include "atomisp_csi2.h"
#include "atomisp_tpg.h"
#include "atomisp_file.h"

#define MAGIC_NUMBER 0x73c5cc4

#define ATOMISP_MAJOR		0
#define ATOMISP_MINOR		5
#define ATOMISP_PATCHLEVEL	1

#define DRIVER_VERSION_STR	__stringify(ATOMISP_MAJOR) \
	"." __stringify(ATOMISP_MINOR) "." __stringify(ATOMISP_PATCHLEVEL)
#define DRIVER_VERSION		KERNEL_VERSION(ATOMISP_MAJOR, \
	ATOMISP_MINOR, ATOMISP_PATCHLEVEL)

#define ATOM_ISP_STEP_WIDTH	4
#define ATOM_ISP_STEP_HEIGHT	4

#define ATOM_ISP_MIN_WIDTH	4
#define ATOM_ISP_MIN_HEIGHT	4
#define ATOM_ISP_MAX_WIDTH	4352
#define ATOM_ISP_MAX_HEIGHT	3264

/* sub-QCIF resolution */
#define ATOM_RESOLUTION_SUBQCIF_WIDTH	128
#define ATOM_RESOLUTION_SUBQCIF_HEIGHT	96

#define ATOM_ISP_MAX_WIDTH_TMP	1280
#define ATOM_ISP_MAX_HEIGHT_TMP	720

#define ATOM_ISP_I2C_BUS_1	4
#define ATOM_ISP_I2C_BUS_2	5

#define ATOM_ISP_POWER_DOWN	0
#define ATOM_ISP_POWER_UP	1

#define ATOM_ISP_MAX_INPUTS	4
#define ATOMISP_ACC_FW_MAX	8

#define ATOMISP_SC_TYPE_SIZE	2

#define ATOMISP_WDT_TIMEOUT		(2 * MSEC_PER_SEC) /* msecs */
#define ATOMISP_WDT_MAX_TIMEOUTS	5

#define ATOMISP_CSS_Q_DEPTH	3
/*
 * Define how fast CPU should be able to serve ISP interrupts.
 * The bigger the value, the higher risk that the ISP is not
 * triggered sufficiently fast for it to process image during
 * vertical blanking time, increasing risk of dropped frames.
 * 1000 us is a reasonable value considering that the processing
 * time is typically ~2000 us.
 */
#ifdef CONFIG_BOARD_CTP
#define ATOMISP_MAX_ISR_LATENCY	CSTATE_EXIT_LATENCY_C1
#else
#define ATOMISP_MAX_ISR_LATENCY	1000
#endif

int atomisp_video_init(struct atomisp_video_pipe *video, const char *name);
void atomisp_video_unregister(struct atomisp_video_pipe *video);
int atomisp_video_register(struct atomisp_video_pipe *video,
	struct v4l2_device *vdev);

struct atomisp_input_subdev {
	unsigned int type;
	enum atomisp_camera_port port;
	struct v4l2_subdev *camera;
	struct v4l2_subdev *motor;
	struct sh_css_morph_table *morph_table;
	struct sh_css_shading_table *shading_table;
	struct v4l2_frmsizeenum frame_size;
};

struct atomisp_hw_contex {

	/* PCI config space info */
	u16 pcicmdsts;
	u32 ispmmadr;
	u32 msicap;
	u32 msi_addr;
	u16 msi_data;
	u8 intr;
	u32 interrupt_control;
	u32 pmcs;
	u32 cg_dis;

	/* I-Unit PHY related info */
	unsigned int csi_rcomp_config;
	unsigned int csi_afe_dly;
	unsigned int csi_control;

	/* ISP MMU base address */
	void *mmu_l1_base;

	/*
	 * PCI Bus Root
	 * Used to access internal message bus
	 */
	struct pci_dev *pci_root;
};

struct atomisp_sw_contex {
	bool init;
	bool probed;

	bool work_queued;
	bool sensor_streaming;
	bool isp_streaming;

	bool error;
	bool bypass;
	bool updating_uptr;
	bool file_input;
	bool grid_info_updated;
	bool invalid_frame;
	bool invalid_vf_frame;
	bool invalid_s3a;
	bool invalid_dis;

	int power_state;
	int run_mode;
	int output_mode;
};

/* Internal states for flash process */
enum atomisp_flash_state {
	ATOMISP_FLASH_IDLE,
	ATOMISP_FLASH_REQUESTED,
	ATOMISP_FLASH_ONGOING,
	ATOMISP_FLASH_DONE
};

struct atomisp_css_params {
	int online_process;
	int yuv_ds_en;
	unsigned int color_effect;
	bool gdc_cac_en;
	bool macc_en;
	bool bad_pixel_en;
	bool video_dis_en;
	bool sc_en;
	bool fpn_en;
	bool xnr_en;
	bool low_light;
	int false_color;
	unsigned int histogram_elenum;

	/* default configurations */
	const struct sh_css_dp_config   *default_dp_config;
	const struct sh_css_wb_config   *default_wb_config;
	const struct sh_css_cc_config   *default_cc_config;
	const struct sh_css_nr_config   *default_nr_config;
	const struct sh_css_ee_config   *default_ee_config;
	const struct sh_css_ob_config   *default_ob_config;
	const struct sh_css_de_config   *default_de_config;
	const struct sh_css_ce_config   *default_ce_config;
	const struct sh_css_gc_config   *default_gc_config;
	const struct sh_css_tnr_config  *default_tnr_config;
	const struct sh_css_3a_config   *default_3a_config;
	const struct sh_css_macc_table  *default_macc_table;
	const struct sh_css_ctc_table   *default_ctc_table;
	const struct sh_css_gamma_table *default_gamma_table;

	/* current configurations */
	struct sh_css_dp_config   dp_config;
	struct sh_css_wb_config   wb_config;
	struct sh_css_cc_config   cc_config;
	struct sh_css_nr_config   nr_config;
	struct sh_css_ee_config   ee_config;
	struct sh_css_ob_config   ob_config;
	struct sh_css_de_config   de_config;
	struct sh_css_ce_config   ce_config;
	struct sh_css_gc_config   gc_config;
	struct sh_css_tnr_config  tnr_config;
	struct sh_css_3a_config   s3a_config;
	struct sh_css_gamma_table gamma_table;
	struct sh_css_ctc_table   ctc_table;
	struct sh_css_macc_table  macc_table;
	/*
	 * overlay removed from css 1.5
	 * struct sh_css_overlay	*vf_overlay;
	 */

	/* Current grid info */
	struct sh_css_grid_info curr_grid_info;

	/* Intermediate buffers used to communicate data between
	   CSS and user space. These are needed to perform the
	   copy_to_user. */
	struct sh_css_3a_output *s3a_output_buf;
	int s3a_output_bytes;
	bool s3a_buf_data_valid;
	/* DIS Coefficients */
	short *dis_hor_coef_buf;
	int    dis_hor_coef_bytes;
	short *dis_ver_coef_buf;
	int    dis_ver_coef_bytes;
	/* DIS projections */
	int *dis_ver_proj_buf;
	int  dis_ver_proj_bytes;
	int *dis_hor_proj_buf;
	int  dis_hor_proj_bytes;
	bool dis_proj_data_valid;
	/* Flash */
	int num_flash_frames;
	enum atomisp_flash_state flash_state;
	enum atomisp_frame_status last_frame_status;
	/* Flag to check if driver needs to update params to css */
	bool css_update_params_needed;
};

struct atomisp_video_pipe_format {
	struct v4l2_pix_format out;
	struct v4l2_pix_format in;
	unsigned int out_sh_fmt;
};

enum atomisp_wdt_status {
	ATOMISP_WDT_STATUS_OK,
	ATOMISP_WDT_STATUS_ERROR_TRYAGAIN,
	ATOMISP_WDT_STATUS_ERROR_FATAL,
};

/*
 * ci device struct
 */
struct atomisp_device {
	struct pci_dev *pdev;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct atomisp_platform_data *pdata;
	const struct firmware *firmware;
	struct timer_list wdt;

	struct pm_qos_request_list pm_qos;

	struct sh_css_acc_fw *acc_fw[ATOMISP_ACC_FW_MAX];
	unsigned int acc_fw_handle;
	int acc_fw_count;
	struct sh_css_acc_fw *marked_fw_for_unload;
	unsigned int frame_bufs_in_css;
	unsigned int vf_frame_bufs_in_css;
	unsigned int s3a_bufs_in_css;
	unsigned int dis_bufs_in_css;

	/* ISP modules */
	struct atomisp_sub_device isp_subdev;
	struct atomisp_mipi_csi2_device csi2_4p;
	struct atomisp_mipi_csi2_device csi2_1p;
	struct atomisp_tpg_device tpg;
	struct atomisp_file_device file_dev;

	struct workqueue_struct *work_queue;
	struct work_struct work;
	struct completion wq_frame_complete;
	struct completion dis_state_complete;
	struct completion acc_fw_complete;
	struct completion acc_unload_fw_complete;

	spinlock_t irq_lock;
	struct list_head s3a_stats;
	struct list_head dis_stats;
	uint32_t irq_infos;
	struct mutex input_lock;
	struct mutex isp_lock;
	struct atomisp_tvnorm *tvnorm;
	bool isp3a_stat_ready;

	struct atomisp_video_pipe_format *main_format;
	struct atomisp_video_pipe_format *snapshot_format;
	struct atomisp_video_pipe_format *vf_format;
	struct atomisp_video_pipe_format *input_format;
	struct sh_css_frame *vf_frame; /* TODO: needed? */
	struct sh_css_frame *ss_frame; /* TODO: needed? */
	struct sh_css_frame *regular_output_frame; /* TODO: needed? */
	struct sh_css_frame *raw_output_frame; /* TODO: needed? */
	enum atomisp_frame_status frame_status[VIDEO_MAX_FRAME];

	int input_cnt;
	int input_curr;
	struct atomisp_input_subdev inputs[ATOM_ISP_MAX_INPUTS];
	struct v4l2_subdev *flash;
	struct v4l2_subdev *motor;

	struct atomisp_hw_contex hw_contex;
	struct atomisp_sw_contex sw_contex;
	struct atomisp_css_params params;

	__u32 snr_max_width;
	__u32 snr_max_height;
	__u32 snr_pixelformat;

	/* isp timeout status flag */
	bool isp_timeout;
	int timeout_cnt;
	enum atomisp_wdt_status wdt_status;
	enum atomisp_frame_status fr_status;

	struct videobuf_buffer *vb_capture; /* TODO: needed? */
	struct videobuf_buffer *vb_snapshot; /* TODO: needed? */
	struct videobuf_buffer *vb_preview; /* TODO: needed? */
};

#define v4l2_dev_to_atomisp_device(dev) \
	container_of(dev, struct atomisp_device, v4l2_dev)

extern struct v4l2_device atomisp_dev;

extern void *atomisp_kernel_malloc(size_t bytes);

extern void atomisp_kernel_free(void *ptr);

#define FW_PATH	"shisp_css15.bin"

#endif /* ATOMISP_INTERNAL_H_ */
