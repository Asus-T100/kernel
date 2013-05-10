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
#ifndef __ATOMISP_INTERNAL_H__
#define __ATOMISP_INTERNAL_H__

#include <linux/atomisp_platform.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/pm_qos.h>
#include <linux/idr.h>

#include <media/media-device.h>
#include <media/v4l2-subdev.h>

#ifdef ATOMISP_CSS2
#include "ia_css_types.h"
#include "sh_css_legacy.h"
#else
#include "sh_css_types.h"
#endif

#include "atomisp_csi2.h"
#include "atomisp_file.h"
#include "atomisp_subdev.h"
#include "atomisp_tpg.h"

#include "gp_device.h"
#include "irq.h"

#define ATOMISP_PCI_DEVICE_SOC_MASK	0xfff8
#define ATOMISP_PCI_DEVICE_SOC_MRFLD	0x1178
#define ATOMISP_PCI_DEVICE_SOC_BYT	0x0f38

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

#define ATOMISP_SC_TYPE_SIZE	2

#define ATOMISP_ISP_TIMEOUT_DURATION		(2 * HZ)
#define ATOMISP_ISP_FILE_TIMEOUT_DURATION	(60 * HZ)
#define ATOMISP_ISP_MAX_TIMEOUT_COUNT	2

#define ATOMISP_CSS_Q_DEPTH	3
#define ATOMISP_CSS_EVENTS_MAX  16
#define ATOMISP_CONT_RAW_FRAMES 10

#define ATOMISP_DELAYED_INIT_NOT_QUEUED	0
#define ATOMISP_DELAYED_INIT_QUEUED	1
#define ATOMISP_DELAYED_INIT_WORK_DONE	2
#define ATOMISP_DELAYED_INIT_DONE	3

/*
 * Define how fast CPU should be able to serve ISP interrupts.
 * The bigger the value, the higher risk that the ISP is not
 * triggered sufficiently fast for it to process image during
 * vertical blanking time, increasing risk of dropped frames.
 * 1000 us is a reasonable value considering that the processing
 * time is typically ~2000 us.
 */
#define ATOMISP_MAX_ISR_LATENCY	1000

struct atomisp_css_event {
	enum ia_css_pipe_id pipe;
	struct ia_css_event event;
};

struct atomisp_input_subdev {
	unsigned int type;
	enum atomisp_camera_port port;
	struct v4l2_subdev *camera;
	struct v4l2_subdev *motor;
	struct ia_css_morph_table *morph_table;
	struct ia_css_shading_table *shading_table;
	struct v4l2_frmsizeenum frame_size;
};

struct atomisp_freq_scaling_rule {
	unsigned int width;
	unsigned int height;
	unsigned short fps;
	unsigned int isp_freq;
	unsigned int run_mode;
};

enum atomisp_dfs_mode {
	ATOMISP_DFS_MODE_AUTO = 0,
	ATOMISP_DFS_MODE_LOW,
	ATOMISP_DFS_MODE_MAX,
};

struct atomisp_regs {
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
	u32 i_control;

	/* I-Unit PHY related info */
	u32 csi_rcomp_config;
	u32 csi_afe_dly;
	u32 csi_control;

	/* New for MRFLD */
	u32 csi_afe_rcomp_config;
	u32 csi_afe_hs_control;
	u32 csi_deadline_control;
	u32 csi_access_viol;
};

struct atomisp_sw_contex {
	bool file_input;
	int  invalid_frame;
	int  invalid_vf_frame;
	int  invalid_s3a;
	int  invalid_dis;

	int power_state;
	int running_freq;
};

/* Internal states for flash process */
enum atomisp_flash_state {
	ATOMISP_FLASH_IDLE,
	ATOMISP_FLASH_REQUESTED,
	ATOMISP_FLASH_ONGOING,
	ATOMISP_FLASH_DONE
};
enum atomisp_css2_stream_state {
	CSS2_STREAM_UNINIT,
	CSS2_STREAM_CREATED,
	CSS2_STREAM_STARTED,
	CSS2_STREAM_STOPPED,
};
struct atomisp_css2_basis {
	struct ia_css_stream *stream;
	struct ia_css_stream_config stream_config;
	struct ia_css_pipe *pipes[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_config pipe_configs[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_extra_config pipe_extra_configs[IA_CSS_PIPE_ID_NUM];
	bool update_pipe[IA_CSS_PIPE_ID_NUM];
	unsigned int curr_pipe;
	enum atomisp_css2_stream_state stream_state;
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
	struct ia_css_isp_config config;

	/* current configurations */

	struct ia_css_wb_config   wb_config;  /**< White Balance config */
	struct ia_css_cc_config   cc_config;  /**< Color Correction config */
	struct ia_css_tnr_config  tnr_config; /**< Temporal Noise Reduction */
	struct ia_css_ecd_config  ecd_config; /**< Eigen Color Demosaicing */
	struct ia_css_ynr_config  ynr_config; /**< Y(Luma) Noise Reduction */
	struct ia_css_fc_config   fc_config;  /**< Fringe Control */
	struct ia_css_cnr_config  cnr_config; /**< Chroma Noise Reduction */
	struct ia_css_macc_config macc_config;  /**< MACC */
	struct ia_css_ctc_config  ctc_config; /**< Chroma Tone Control */
	struct ia_css_aa_config   aa_config;  /**< Anti-Aliasing */
	struct ia_css_ce_config   ce_config;
	struct ia_css_ob_config   ob_config;  /**< Objective Black config */
	struct ia_css_dp_config   dp_config;  /**< Dead Pixel config */
	struct ia_css_nr_config   nr_config;  /**< Noise Reduction config */
	struct ia_css_ee_config   ee_config;  /**< Edge Enhancement config */
	struct ia_css_de_config   de_config;  /**< Demosaic config */
	struct ia_css_gc_config   gc_config;  /**< Gamma Correction config */
	struct ia_css_anr_config  anr_config; /**< Advanced Noise Reduction */
	struct ia_css_3a_config   s3a_config; /**< 3A Statistics config */
	struct ia_css_xnr_config  xnr_config; /**< eXtra Noise Reduction */
	struct ia_css_dz_config   dz_config;  /**< Digital Zoom */
	struct ia_css_cc_config yuv2rgb_cc_config; /**< Color
							Correction config */
	struct ia_css_cc_config rgb2yuv_cc_config; /**< Color
							Correction config */
	struct ia_css_macc_table  macc_table;
	struct ia_css_gamma_table gamma_table;
	struct ia_css_ctc_table   ctc_table;
	struct ia_css_xnr_table   xnr_table;
	struct ia_css_rgb_gamma_table r_gamma_table;
	struct ia_css_rgb_gamma_table g_gamma_table;
	struct ia_css_rgb_gamma_table b_gamma_table;
	struct ia_css_vector      motion_vector; /**< For 2-axis DVS */
	struct ia_css_shading_table shading_table;
	struct ia_css_morph_table   morph_table;
	struct ia_css_capture_config   capture_config;
	struct ia_css_anr_thres   anr_thres;
	/*
	 * overlay removed from css 1.5
	 * struct ia_css_overlay	*vf_overlay;
	 */

	/* Current grid info */
	struct ia_css_grid_info curr_grid_info;

	/* Intermediate buffers used to communicate data between
	   CSS and user space. These are needed to perform the
	   copy_to_user. */
	struct ia_css_3a_statistics *s3a_user_stat;
	int s3a_output_bytes;
	bool s3a_buf_data_valid;

	struct ia_css_dvs_coefficients *dvs_coeff;
	struct ia_css_dvs_statistics *dvs_stat;
	bool dvs_proj_data_valid;
	int  dvs_hor_coef_bytes;
	int  dvs_ver_coef_bytes;
	int  dvs_ver_proj_bytes;
	int  dvs_hor_proj_bytes;

	/* Flash */
	int num_flash_frames;
	enum atomisp_flash_state flash_state;
	enum atomisp_frame_status last_frame_status;
	/* continuous capture */
	struct atomisp_cont_capture_conf offline_parm;
	/* Flag to check if driver needs to update params to css */
	bool css_update_params_needed;
};

struct atomisp_acc_fw {
	struct sh_css_fw_info *fw;
	unsigned int handle;
	unsigned int flags;
	unsigned int type;
	struct {
		size_t length;
		unsigned long css_ptr;
	} args[ATOMISP_ACC_NR_MEMORY];
	struct list_head list;
};

struct atomisp_map {
	hrt_vaddress ptr;
	size_t length;
	struct list_head list;
	/* FIXME: should keep book which maps are currently used
	 * by binaries and not allow releasing those
	 * which are in use. Implement by reference counting.
	 */
};

#define ATOMISP_DEVICE_STREAMING_DISABLED	0
#define ATOMISP_DEVICE_STREAMING_ENABLED	1
#define ATOMISP_DEVICE_STREAMING_STOPPING	2

/*
 * ci device struct
 */
struct atomisp_device {
	struct pci_dev *pdev;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct atomisp_platform_data *pdata;
	void *mmu_l1_base;
	struct pci_dev *pci_root;
	const struct firmware *firmware;

	struct pm_qos_request pm_qos;
	s32 max_isr_latency;

	struct {
		struct list_head fw;
		struct list_head memory_maps;
		struct ia_css_pipe *pipeline;
		bool extension_mode;
		struct ida ida;
	} acc;

	unsigned int s3a_bufs_in_css[IA_CSS_PIPE_ID_NUM];
	unsigned int dis_bufs_in_css;
	struct ia_css_fw css_fw;

	/* ISP modules */
	struct atomisp_sub_device isp_subdev;
	/*
	 * MRFLD has 3 CSI ports, while MFLD has only 2.
	 */
	struct atomisp_mipi_csi2_device csi2_port[ATOMISP_CAMERA_NR_PORTS];
	struct atomisp_tpg_device tpg;
	struct atomisp_file_device file_dev;

	/* Purpose of mutex is to protect and serialize use of isp data
	 * structures and css API calls. */
	struct mutex mutex;
	/*
	 * Serialise streamoff: mutex is dropped during streamoff to
	 * cancel the watchdog queue. MUST be acquired BEFORE
	 * "mutex".
	 */
	struct mutex streamoff_mutex;
	struct list_head s3a_stats;
	struct list_head dvs_stats;

	struct ia_css_frame *vf_frame; /* TODO: needed? */
	struct ia_css_frame *raw_output_frame;
	enum atomisp_frame_status frame_status[VIDEO_MAX_FRAME];

	int input_cnt;
	int input_curr;
	struct atomisp_input_subdev inputs[ATOM_ISP_MAX_INPUTS];
	struct v4l2_subdev *flash;
	struct v4l2_subdev *motor;

	struct atomisp_regs saved_regs;
	struct atomisp_sw_contex sw_contex;
	struct atomisp_css_params params;
	struct atomisp_css2_basis css2_basis;

	/* isp timeout status flag */
	bool isp_timeout;
	bool isp_fatal_error;
	struct workqueue_struct *wdt_work_queue;
	struct work_struct wdt_work;
	struct timer_list wdt;
	atomic_t wdt_count;
	unsigned int wdt_duration;	/* in jiffies */
	enum atomisp_frame_status fr_status;

	atomic_t sof_count;
	atomic_t sequence;      /* Sequence value that is assigned to buffer. */
	atomic_t sequence_temp;

	spinlock_t lock; /* Just for streaming below */
	unsigned int streaming; /* Hold both mutex and lock to change this */

	bool need_gfx_throttle;

	/* delayed memory allocation for css */
	struct completion init_done;
	struct workqueue_struct *delayed_init_workq;
	unsigned int delayed_init;
	struct work_struct delayed_init_work;
};

#define v4l2_dev_to_atomisp_device(dev) \
	container_of(dev, struct atomisp_device, v4l2_dev)

extern struct v4l2_device atomisp_dev;

extern void *atomisp_kernel_malloc(size_t bytes);

extern void atomisp_kernel_free(void *ptr);

#define MFLD_FW_PATH	"shisp_css15.bin"
#ifdef ATOMISP_CSS2
#define ISP2400B0_FW_PATH   "shisp_2400b0_cssv2.bin"
#define ISP2400A0_FW_PATH   "shisp_2400_cssv2.bin"
#else
#define MRFLD_FW_PATH   "shisp_2400.bin"
#endif

#endif /* __ATOMISP_INTERNAL_H__ */
