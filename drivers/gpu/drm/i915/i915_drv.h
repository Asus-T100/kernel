/* i915_drv.h -- Private header for the I915 driver -*- linux-c -*-
 */
/*
 *
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _I915_DRV_H_
#define _I915_DRV_H_

#include "i915_reg.h"
#include "intel_bios.h"
#include "intel_ringbuffer.h"
#include <linux/io-mapping.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <drm/intel-gtt.h>
#include <linux/backlight.h>
#include <linux/intel-iommu.h>
#include <linux/kref.h>
#include "hdmi_audio_if.h"
#include <linux/mmu_notifier.h>

/* General customization:
 */

#define DRIVER_AUTHOR		"Tungsten Graphics, Inc."

#define DRIVER_NAME		"i915"
#define DRIVER_DESC		"Intel Graphics"
#define DRIVER_DATE		"20080730"

enum pipe {
	PIPE_A = 0,
	PIPE_B,
	PIPE_C,
	I915_MAX_PIPES
};
#define pipe_name(p) ((p) + 'A')

enum plane {
	PLANE_A = 0,
	PLANE_B,
	PLANE_C,
	I915_MAX_PLANES
};
#define plane_name(p) ((p) + 'A')

enum port {
	PORT_A = 0,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
	I915_MAX_PORTS
};
#define port_name(p) ((p) + 'A')

#define I915_GEM_GPU_DOMAINS \
	(I915_GEM_DOMAIN_RENDER | \
	I915_GEM_DOMAIN_SAMPLER | \
	I915_GEM_DOMAIN_COMMAND | \
	I915_GEM_DOMAIN_INSTRUCTION | \
	I915_GEM_DOMAIN_VERTEX)

#define for_each_pipe(p) for ((p) = 0; (p) < dev_priv->num_pipe; (p)++)

#define for_each_encoder_on_crtc(dev, __crtc, intel_encoder) \
	list_for_each_entry((intel_encoder), &(dev)->mode_config.encoder_list, base.head) \
		if ((intel_encoder)->base.crtc == (__crtc))

struct intel_pch_pll {
	int refcount; /* count of number of CRTCs sharing this PLL */
	int active; /* count of number of active CRTCs (i.e. DPMS on) */
	bool on; /* is the PLL actually active? Disabled during modeset */
	int pll_reg;
	int fp0_reg;
	int fp1_reg;
};
#define I915_NUM_PLLS 2

/* Interface history:
 *
 * 1.1: Original.
 * 1.2: Add Power Management
 * 1.3: Add vblank support
 * 1.4: Fix cmdbuffer path, add heap destroy
 * 1.5: Add vblank pipe configuration
 * 1.6: - New ioctl for scheduling buffer swaps on vertical blank
 *      - Support vertical blank on secondary display pipe
 */
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		6
#define DRIVER_PATCHLEVEL	0

#define WATCH_COHERENCY	0
#define WATCH_LISTS	0
#define WATCH_GTT	0

#define I915_GEM_PHYS_CURSOR_0 1
#define I915_GEM_PHYS_CURSOR_1 2
#define I915_GEM_PHYS_OVERLAY_REGS 3
#define I915_MAX_PHYS_OBJECT (I915_GEM_PHYS_OVERLAY_REGS)

struct drm_i915_gem_phys_object {
	int id;
	struct page **page_list;
	drm_dma_handle_t *handle;
	struct drm_i915_gem_object *cur_obj;
};

struct mem_block {
	struct mem_block *next;
	struct mem_block *prev;
	int start;
	int size;
	struct drm_file *file_priv; /* NULL: free, -1: heap, other: real files */
};

struct opregion_header;
struct opregion_acpi;
struct opregion_swsci;
struct opregion_asle;
struct drm_i915_private;

struct intel_opregion {
	struct opregion_header __iomem *header;
	struct opregion_acpi __iomem *acpi;
	struct opregion_swsci __iomem *swsci;
	struct opregion_asle __iomem *asle;
	void __iomem *vbt;
	u32 __iomem *lid_state;
};
#define OPREGION_SIZE            (8*1024)

struct intel_overlay;
struct intel_overlay_error_state;

struct drm_i915_master_private {
	drm_local_map_t *sarea;
	struct _drm_i915_sarea *sarea_priv;
};
#define I915_FENCE_REG_NONE -1
#define I915_MAX_NUM_FENCES 16
/* 16 fences + sign bit for FENCE_REG_NONE */
#define I915_MAX_NUM_FENCE_BITS 5

struct drm_i915_fence_reg {
	struct list_head lru_list;
	struct drm_i915_gem_object *obj;
	int pin_count;
};

struct sdvo_device_mapping {
	u8 initialized;
	u8 dvo_port;
	u8 slave_addr;
	u8 dvo_wiring;
	u8 i2c_pin;
	u8 ddc_pin;
};

struct intel_display_error_state;

struct drm_i915_error_state {
	struct kref ref;
	u32 eir;
	u32 pgtbl_er;
	u32 ier;
	u32 ccid;
	bool waiting[I915_NUM_RINGS];
	u32 pipestat[I915_MAX_PIPES];
	u32 tail[I915_NUM_RINGS];
	u32 head[I915_NUM_RINGS];
	u32 ipeir[I915_NUM_RINGS];
	u32 ipehr[I915_NUM_RINGS];
	u32 instdone[I915_NUM_RINGS][I915_MAX_INSTDONE_REG];
	u32 acthd[I915_NUM_RINGS];
	u32 semaphore_mboxes[I915_NUM_RINGS][I915_NUM_RINGS - 1];
	u32 rc_psmi[I915_NUM_RINGS]; /* sleep state */
	/* our own tracking of ring head and tail */
	u32 cpu_ring_head[I915_NUM_RINGS];
	u32 cpu_ring_tail[I915_NUM_RINGS];
	u32 error; /* gen6+ */
	u32 instpm[I915_NUM_RINGS];
	u32 instps[I915_NUM_RINGS];
	u32 seqno[I915_NUM_RINGS];
	u64 bbaddr;
	u32 fault_reg[I915_NUM_RINGS];
	u32 done_reg;
	u32 faddr[I915_NUM_RINGS];
	u64 fence[I915_MAX_NUM_FENCES];
	struct timeval time;
	struct drm_i915_error_ring {
		struct drm_i915_error_object {
			int page_count;
			u32 gtt_offset;
			u32 *pages[0];
		} *ringbuffer, *batchbuffer;
		struct drm_i915_error_request {
			long jiffies;
			u32 seqno;
			u32 tail;
		} *requests;
		int num_requests;
	} ring[I915_NUM_RINGS];
	struct drm_i915_error_buffer {
		u32 size;
		u32 name;
		u32 rseqno, wseqno;
		u32 gtt_offset;
		u32 read_domains;
		u32 write_domain;
		s32 fence_reg:I915_MAX_NUM_FENCE_BITS;
		s32 pinned:2;
		u32 tiling:2;
		u32 dirty:1;
		u32 purgeable:1;
		s32 ring:4;
		u32 cache_level:2;
	} *active_bo, *pinned_bo;
	u32 active_bo_count, pinned_bo_count;
	struct intel_overlay_error_state *overlay;
	struct intel_display_error_state *display;
};

struct drm_i915_display_funcs {
	void (*dpms)(struct drm_crtc *crtc, int mode);
	bool (*fbc_enabled)(struct drm_device *dev);
	void (*enable_fbc)(struct drm_crtc *crtc, unsigned long interval);
	void (*disable_fbc)(struct drm_device *dev);
	int (*get_display_clock_speed)(struct drm_device *dev);
	int (*get_fifo_size)(struct drm_device *dev, int plane);
	void (*update_wm)(struct drm_device *dev);
	void (*update_sprite_wm)(struct drm_device *dev, int pipe,
				 uint32_t sprite_width, int pixel_size);
	void (*update_linetime_wm)(struct drm_device *dev, int pipe,
				 struct drm_display_mode *mode);
	int (*crtc_mode_set)(struct drm_crtc *crtc,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode,
			     int x, int y,
			     struct drm_framebuffer *old_fb);
	void (*off)(struct drm_crtc *crtc);
	void (*write_eld)(struct drm_connector *connector,
			  struct drm_crtc *crtc);
	void (*fdi_link_train)(struct drm_crtc *crtc);
	void (*init_clock_gating)(struct drm_device *dev);
	void (*init_pch_clock_gating)(struct drm_device *dev);
	int (*queue_flip)(struct drm_device *dev, struct drm_crtc *crtc,
			  struct drm_framebuffer *fb,
			  struct drm_i915_gem_object *obj);
	int (*update_plane)(struct drm_crtc *crtc, struct drm_framebuffer *fb,
			    int x, int y);
	/* clock updates for mode set */
	/* cursor updates */
	/* render clock increase/decrease */
	/* display clock increase/decrease */
	/* pll clock increase/decrease */
};

struct drm_i915_gt_funcs {
	void (*force_wake_get)(struct drm_i915_private *dev_priv,
				int fw_engine);
	void (*force_wake_put)(struct drm_i915_private *dev_priv,
				int fw_engine);
};

struct drm_i915_pm_funcs {
	int (*drm_freeze)(struct drm_device *dev);
	int (*drm_thaw)(struct drm_device *dev, bool is_hibernate_restore);
};

#define DEV_INFO_FLAGS \
	DEV_INFO_FLAG(is_mobile) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_i85x) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_i915g) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_i945gm) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_g33) DEV_INFO_SEP \
	DEV_INFO_FLAG(need_gfx_hws) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_g4x) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_pineview) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_broadwater) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_crestline) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_ivybridge) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_valleyview) DEV_INFO_SEP \
	DEV_INFO_FLAG(is_haswell) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_force_wake) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_fbc) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_pipe_cxsr) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_hotplug) DEV_INFO_SEP \
	DEV_INFO_FLAG(cursor_needs_physical) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_overlay) DEV_INFO_SEP \
	DEV_INFO_FLAG(overlay_needs_physical) DEV_INFO_SEP \
	DEV_INFO_FLAG(supports_tv) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_bsd_ring) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_blt_ring) DEV_INFO_SEP \
	DEV_INFO_FLAG(has_llc)

struct intel_device_info {
	u8 gen;
	u8 is_mobile:1;
	u8 is_i85x:1;
	u8 is_i915g:1;
	u8 is_i945gm:1;
	u8 is_g33:1;
	u8 need_gfx_hws:1;
	u8 is_g4x:1;
	u8 is_pineview:1;
	u8 is_broadwater:1;
	u8 is_crestline:1;
	u8 is_ivybridge:1;
	u8 is_valleyview:1;
	u8 has_force_wake:1;
	u8 is_haswell:1;
	u8 has_fbc:1;
	u8 has_pipe_cxsr:1;
	u8 has_hotplug:1;
	u8 cursor_needs_physical:1;
	u8 has_overlay:1;
	u8 overlay_needs_physical:1;
	u8 supports_tv:1;
	u8 has_bsd_ring:1;
	u8 has_blt_ring:1;
	u8 has_llc:1;
};

#define I915_PPGTT_PD_ENTRIES 512
#define I915_PPGTT_PT_ENTRIES 1024
struct i915_hw_ppgtt {
	struct drm_device *dev;
	unsigned num_pd_entries;
	struct page **pt_pages;
	uint32_t pd_offset;
	dma_addr_t *pt_dma_addr;
	dma_addr_t scratch_page_dma_addr;
};


/* This must match up with the value previously used for execbuf2.rsvd1. */
#define DEFAULT_CONTEXT_ID 0
struct i915_hw_context {
	int id;
	bool is_initialized;
	struct drm_i915_file_private *file_priv;
	struct intel_ring_buffer *ring;
	struct drm_i915_gem_object *obj;
};

enum no_fbc_reason {
	FBC_NO_OUTPUT, /* no outputs enabled to compress */
	FBC_STOLEN_TOO_SMALL, /* not enough space to hold compressed buffers */
	FBC_UNSUPPORTED_MODE, /* interlace or doublescanned mode */
	FBC_MODE_TOO_LARGE, /* mode too large for compression */
	FBC_BAD_PLANE, /* fbc not supported on plane */
	FBC_NOT_TILED, /* buffer not tiled */
	FBC_MULTIPLE_PIPES, /* more than one pipe active */
	FBC_MODULE_PARAM,
};

enum intel_pch {
	PCH_NONE = 0,	/* No PCH present */
	PCH_IBX,	/* Ibexpeak PCH */
	PCH_CPT,	/* Cougarpoint PCH */
	PCH_LPT,	/* Lynxpoint PCH */
};

#define QUIRK_PIPEA_FORCE (1<<0)
#define QUIRK_LVDS_SSC_DISABLE (1<<1)
#define QUIRK_INVERT_BRIGHTNESS (1<<2)

struct intel_fbdev;
struct intel_fbc_work;

struct intel_gmbus {
	struct i2c_adapter adapter;
	bool force_bit;
	u32 reg0;
	u32 gpio_reg;
	struct i2c_algo_bit_data bit_algo;
	struct drm_i915_private *dev_priv;
};

struct intel_hangcheck {
	/* The ring being monitored*/
	uint32_t ringid;

	/* Parent drm_device*/
	struct drm_device *dev;

	/* Timer for this ring only*/
	struct timer_list timer;

	/* Count of consecutive hang detections
	 * (reset flag set once count exceeds threshold)*/
#define HANGCHECK_THRESHOLD      1
#define MBOX_HANGCHECK_THRESHOLD 4
	int count;

	/* Last sampled head and active head*/
	uint32_t last_acthd;
	uint32_t last_hd;

	/* Last recorded head index from previous ring hang.
	* This is only ever a ring index where as active
	* head may be a graphics address in a ring buffer */
	uint32_t last_head;

	/* Last recorded instdone*/
	uint32_t prev_instdone[I915_MAX_INSTDONE_REG];

	/* Flag to indicate if ring reset required*/
	atomic_t reset;

	/* Keep a record of the last time the ring was reset */
	unsigned long last_reset;

	/* Number of times this ring has been
	* reset since boot (TDR and watchdog)*/
	uint32_t total;

	/* Number of TDR hang detections for this ring */
	uint32_t tdr_count;

	/* Number of watchdog hang detections for this ring */
	uint32_t watchdog_count;
};


typedef struct drm_i915_private {
	struct drm_device *dev;

	const struct intel_device_info *info;

	int relative_constants_mode;

	void __iomem *regs;

	struct drm_i915_gt_funcs gt;

	/** related to power management */
	struct drm_i915_pm_funcs pm;
	/** gt_fifo_count and the subsequent register write are synchronized
	 * with dev->struct_mutex. */
	unsigned int gt_fifo_count;
	/** forcewake_count is protected by gt_lock */
	unsigned int forcewake_count;

	/*VLV specific FW counters. Clean this later*/
	unsigned int fw_rendercount;
	unsigned int fw_mediacount;

	/** counter for user requests to disable/re-enable RC6 */
	int rc6_user_disable_count;

	/** gt_lock is also taken in irq contexts. */
	struct spinlock gt_lock;

	struct intel_gmbus gmbus[GMBUS_NUM_PORTS];

	/** gmbus_mutex protects against concurrent usage of the single hw gmbus
	 * controller on different i2c buses. */
	struct mutex gmbus_mutex;

	/**
	 * Base address of the gmbus and gpio block.
	 */
	uint32_t gpio_mmio_base;

	struct pci_dev *bridge_dev;
	struct intel_ring_buffer ring[I915_NUM_RINGS];
	uint32_t next_seqno;

	drm_dma_handle_t *status_page_dmah;
	uint32_t counter;
	struct drm_i915_gem_object *pwrctx;
	struct drm_i915_gem_object *renderctx;

	struct resource mch_res;

	unsigned int cpp;
	int back_offset;
	int front_offset;
	int current_page;
	int page_flipping;

	atomic_t irq_received;

	/* protects the irq masks */
	spinlock_t irq_lock;

	/* DPIO indirect register protection */
	spinlock_t dpio_lock;

	/** Cached value of IMR to avoid reads in updating the bitfield */
	u32 pipestat[2];
	u32 irq_mask;
	u32 hotplugstat;
	bool disp_pm_in_progress;
	bool sprtsuspendstat[2];
	u32 gt_irq_mask;
	u32 pch_irq_mask;

	u32 hotplug_supported_mask;
	struct work_struct hotplug_work;

	unsigned int sr01, adpa, ppcr, dvob, dvoc, lvds;
	int num_pipe;
	int num_plane; /* number of sprite planes per pipe */
	int num_pch_pll;

	/* For hangcheck timer */
#define MINIMUM_HANGCHECK_PERIOD 100   /* 100ms */
#define MAXIMUM_HANGCHECK_PERIOD 30000 /* 30s */
#define DRM_I915_HANGCHECK_JIFFIES msecs_to_jiffies(i915_hangcheck_period)
	struct intel_hangcheck hangcheck[I915_NUM_RINGS];

	unsigned int stop_rings;

	unsigned long cfb_size;
	unsigned int cfb_fb;
	enum plane cfb_plane;
	int cfb_y;
	struct intel_fbc_work *fbc_work;

	struct intel_opregion opregion;

	/* overlay */
	struct intel_overlay *overlay;
	bool sprite_scaling_enabled;

	/* LVDS info */
	int backlight_level;  /* restore backlight to this value */
	bool backlight_enabled;
	struct drm_display_mode *lfp_lvds_vbt_mode; /* if any */
	struct drm_display_mode *sdvo_lvds_vbt_mode; /* if any */

	/* Feature bits from the VBIOS */
	unsigned int int_tv_support:1;
	unsigned int lvds_dither:1;
	unsigned int lvds_vbt:1;
	unsigned int int_crt_support:1;
	unsigned int lvds_use_ssc:1;
	unsigned int display_clock_mode:1;
	int lvds_ssc_freq;
	unsigned int bios_lvds_val; /* initial [PCH_]LVDS reg val in VBIOS */
	unsigned int lvds_val; /* used for checking LVDS channel mode */
	struct {
		int rate;
		int lanes;
		int preemphasis;
		int vswing;

		bool initialized;
		bool support;
		int bpp;
		struct edp_power_seq pps;
	} edp;

	/* MIPI panel info */
	struct mipi_info mipi;

	/* Either of the below will be true based on detection
	 * presence of one indicates absence of other as of now
	 * till we have proper fix for dynamic detection
	 */
	bool is_edp;
	bool is_mipi;
	bool is_hdmi;
	bool csc_enabled;
	bool gamma_enabled;
	int planeid_gamma;
	int planeid_csc;

	bool no_aux_handshake;

	struct notifier_block lid_notifier;

	int crt_ddc_pin;
	struct drm_i915_fence_reg fence_regs[I915_MAX_NUM_FENCES]; /* assume 965 */
	int fence_reg_start; /* 4 if userland hasn't ioctl'd us yet */
	int num_fence_regs; /* 8 on pre-965, 16 otherwise */

	unsigned int fsb_freq, mem_freq, is_ddr3, cck_freq, gpll;

	spinlock_t error_lock;
	/* Protected by dev->error_lock. */
	struct drm_i915_error_state *first_error;
	struct work_struct error_work;
	atomic_t full_reset;
	uint32_t total_resets;

	wait_queue_head_t error_queue;
	struct workqueue_struct *wq;
	struct workqueue_struct *flipwq;

	struct workqueue_struct *vmap_mn_unregister_wq;

	/* Display functions */
	struct drm_i915_display_funcs display;
	bool early_suspended;

	/* PCH chipset type */
	enum intel_pch pch_type;

	unsigned long quirks;

	/* Register state */
	bool modeset_on_lid;
	u8 saveLBB;
	u32 saveDSPACNTR;
	u32 saveDSPBCNTR;
	u32 saveDSPARB;
	u32 saveHWS;
	u32 savePIPEACONF;
	u32 savePIPEBCONF;
	u32 savePIPEASRC;
	u32 savePIPEBSRC;
	u32 saveFPA0;
	u32 saveFPA1;
	u32 saveDPLL_A;
	u32 saveDPLL_A_MD;
	u32 saveHTOTAL_A;
	u32 saveHBLANK_A;
	u32 saveHSYNC_A;
	u32 saveVTOTAL_A;
	u32 saveVBLANK_A;
	u32 saveVSYNC_A;
	u32 saveBCLRPAT_A;
	u32 saveTRANSACONF;
	u32 saveTRANS_HTOTAL_A;
	u32 saveTRANS_HBLANK_A;
	u32 saveTRANS_HSYNC_A;
	u32 saveTRANS_VTOTAL_A;
	u32 saveTRANS_VBLANK_A;
	u32 saveTRANS_VSYNC_A;
	u32 savePIPEASTAT;
	u32 saveDSPASTRIDE;
	u32 saveDSPASIZE;
	u32 saveDSPAPOS;
	u32 saveDSPAADDR;
	u32 saveDSPASURF;
	u32 saveDSPATILEOFF;
	u32 savePFIT_PGM_RATIOS;
	u32 saveBLC_HIST_CTL;
	u32 saveBLC_PWM_CTL;
	u32 saveBLC_PWM_CTL2;
	u32 saveBLC_CPU_PWM_CTL;
	u32 saveBLC_CPU_PWM_CTL2;
	u32 saveFPB0;
	u32 saveFPB1;
	u32 saveDPLL_B;
	u32 saveDPLL_B_MD;
	u32 saveHTOTAL_B;
	u32 saveHBLANK_B;
	u32 saveHSYNC_B;
	u32 saveVTOTAL_B;
	u32 saveVBLANK_B;
	u32 saveVSYNC_B;
	u32 saveBCLRPAT_B;
	u32 saveTRANSBCONF;
	u32 saveTRANS_HTOTAL_B;
	u32 saveTRANS_HBLANK_B;
	u32 saveTRANS_HSYNC_B;
	u32 saveTRANS_VTOTAL_B;
	u32 saveTRANS_VBLANK_B;
	u32 saveTRANS_VSYNC_B;
	u32 savePIPEBSTAT;
	u32 saveDSPBSTRIDE;
	u32 saveDSPBSIZE;
	u32 saveDSPBPOS;
	u32 saveDSPBADDR;
	u32 saveDSPBSURF;
	u32 saveDSPBTILEOFF;
	u32 saveVGA0;
	u32 saveVGA1;
	u32 saveVGA_PD;
	u32 saveVGACNTRL;
	u32 saveADPA;
	u32 saveLVDS;
	u32 savePP_ON_DELAYS;
	u32 savePP_OFF_DELAYS;
	u32 saveDVOA;
	u32 saveDVOB;
	u32 saveDVOC;
	u32 savePP_ON;
	u32 savePP_OFF;
	u32 savePP_CONTROL;
	u32 savePP_DIVISOR;
	u32 savePFIT_CONTROL;
	u32 save_palette_a[256];
	u32 save_palette_b[256];
	u32 saveDPFC_CB_BASE;
	u32 saveFBC_CFB_BASE;
	u32 saveFBC_LL_BASE;
	u32 saveFBC_CONTROL;
	u32 saveFBC_CONTROL2;
	u32 saveIER;
	u32 saveIIR;
	u32 saveIMR;
	u32 saveDEIER;
	u32 saveDEIMR;
	u32 saveGTIER;
	u32 saveGTIMR;
	u32 saveFDI_RXA_IMR;
	u32 saveFDI_RXB_IMR;
	u32 saveCACHE_MODE_0;
	u32 saveMI_ARB_STATE;
	u32 saveSWF0[16];
	u32 saveSWF1[16];
	u32 saveSWF2[3];
	u8 saveMSR;
	u8 saveSR[8];
	u8 saveGR[25];
	u8 saveAR_INDEX;
	u8 saveAR[21];
	u8 saveDACMASK;
	u8 saveCR[37];
	uint64_t saveFENCE[I915_MAX_NUM_FENCES];
	u32 saveCURACNTR;
	u32 saveCURAPOS;
	u32 saveCURABASE;
	u32 saveCURBCNTR;
	u32 saveCURBPOS;
	u32 saveCURBBASE;
	u32 saveCURSIZE;
	u32 saveDP_B;
	u32 saveDP_C;
	u32 saveDP_D;
	u32 savePIPEA_GMCH_DATA_M;
	u32 savePIPEB_GMCH_DATA_M;
	u32 savePIPEA_GMCH_DATA_N;
	u32 savePIPEB_GMCH_DATA_N;
	u32 savePIPEA_DP_LINK_M;
	u32 savePIPEB_DP_LINK_M;
	u32 savePIPEA_DP_LINK_N;
	u32 savePIPEB_DP_LINK_N;
	u32 saveFDI_RXA_CTL;
	u32 saveFDI_TXA_CTL;
	u32 saveFDI_RXB_CTL;
	u32 saveFDI_TXB_CTL;
	u32 savePFA_CTL_1;
	u32 savePFB_CTL_1;
	u32 savePFA_WIN_SZ;
	u32 savePFB_WIN_SZ;
	u32 savePFA_WIN_POS;
	u32 savePFB_WIN_POS;
	u32 savePCH_DREF_CONTROL;
	u32 saveDISP_ARB_CTL;
	u32 savePIPEA_DATA_M1;
	u32 savePIPEA_DATA_N1;
	u32 savePIPEA_LINK_M1;
	u32 savePIPEA_LINK_N1;
	u32 savePIPEB_DATA_M1;
	u32 savePIPEB_DATA_N1;
	u32 savePIPEB_LINK_M1;
	u32 savePIPEB_LINK_N1;
	u32 saveMCHBAR_RENDER_STANDBY;
	u32 savePCH_PORT_HOTPLUG;
	u32 saveGUNIT_Control;
	u32 saveGUNIT_Control2;
	u32 saveGUNIT_CZClockGatingDisable1;
	u32 saveGUNIT_CZClockGatingDisable2;
	u32 saveDPIO_CFG_DATA;
	u32 saveDPST_VLV_BTGR_DATA;
	bool saveDPSTState;

	struct {
		/** Bridge to intel-gtt-ko */
		const struct intel_gtt *gtt;
		/** Memory allocator for GTT stolen memory */
		struct drm_mm stolen;
		/** Memory allocator for GTT */
		struct drm_mm gtt_space;
		/** List of all objects in gtt_space. Used to restore gtt
		 * mappings on resume */
		struct list_head gtt_list;

		/** Usable portion of the GTT for GEM */
		unsigned long gtt_start;
		unsigned long gtt_mappable_end;
		unsigned long gtt_end;

		struct io_mapping *gtt_mapping;
		phys_addr_t gtt_base_addr;
		int gtt_mtrr;

		/** PPGTT used for aliasing the PPGTT with the GTT */
		struct i915_hw_ppgtt *aliasing_ppgtt;

		u32 *l3_remap_info;

		struct shrinker inactive_shrinker;

		/**
		 * List of objects currently involved in rendering.
		 *
		 * Includes buffers having the contents of their GPU caches
		 * flushed, not necessarily primitives.  last_rendering_seqno
		 * represents when the rendering involved will be completed.
		 *
		 * A reference is held on the buffer while on this list.
		 */
		struct list_head active_list;

		/**
		 * LRU list of objects which are not in the ringbuffer and
		 * are ready to unbind, but are still in the GTT.
		 *
		 * last_rendering_seqno is 0 while an object is in this list.
		 *
		 * A reference is not held on the buffer while on this list,
		 * as merely being GTT-bound shouldn't prevent its being
		 * freed, and we'll pull it off the list in the free path.
		 */
		struct list_head inactive_list;

		/** LRU list of objects with fence regs on them. */
		struct list_head fence_list;

		/**
		 * We leave the user IRQ off as much as possible,
		 * but this means that requests will finish and never
		 * be retired once the system goes idle. Set a timer to
		 * fire periodically while the ring is running. When it
		 * fires, go retire requests.
		 */
		struct delayed_work retire_work;

		/**
		 * Are we in a non-interruptible section of code like
		 * modesetting?
		 */
		bool interruptible;

		/**
		 * Flag if the X Server, and thus DRM, is not currently in
		 * control of the device.
		 *
		 * This is set between LeaveVT and EnterVT.  It needs to be
		 * replaced with a semaphore.  It also needs to be
		 * transitioned away from for kernel modesetting.
		 */
		int suspended;

		/**
		 * This is set when the error_recovery function is running.
		 * It prevents command submission from occurring and makes
		 * every pending request fail
		 */
		atomic_t wedged;

		/** Bit 6 swizzling required for X tiling */
		uint32_t bit_6_swizzle_x;
		/** Bit 6 swizzling required for Y tiling */
		uint32_t bit_6_swizzle_y;

		/* storage for physical objects */
		struct drm_i915_gem_phys_object *phys_objs[I915_MAX_PHYS_OBJECT];

		/* accounting, useful for userland debugging */
		size_t gtt_total;
		size_t mappable_gtt_total;
		size_t object_memory;
		u32 object_count;
	} mm;

	/* Old dri1 support infrastructure, beware the dragons ya fools entering
	 * here! */
	struct {
		unsigned allow_batchbuffer : 1;
		u32 __iomem *gfx_hws_cpu_addr;
	} dri1;

	/* Kernel Modesetting */

	struct sdvo_device_mapping sdvo_mappings[2];
	/* indicate whether the LVDS_BORDER should be enabled or not */
	unsigned int lvds_border_bits;
	/* Panel fitter placement and size for Ironlake+ */
	u32 pch_pf_pos, pch_pf_size;

	struct drm_crtc *plane_to_crtc_mapping[3];
	struct drm_crtc *pipe_to_crtc_mapping[3];
	wait_queue_head_t pending_flip_queue;

	struct intel_pch_pll pch_plls[I915_NUM_PLLS];

	/* Reclocking support */
	bool render_reclock_avail;
	bool lvds_downclock_avail;
	/* indicates the reduced downclock for LVDS*/
	int lvds_downclock;
	u16 orig_clock;
	int child_dev_num;
	struct child_device_config *child_dev;
	struct drm_connector *int_lvds_connector;
	struct drm_connector *int_edp_connector;
	struct drm_connector *int_mipi_connector;

	bool mchbar_need_disable;

	/* gen6+ rps state */
	struct {
		struct work_struct work;
		struct delayed_work rps_timer_work;
		u32 pm_iir;
		/* lock - irqsave spinlock that protectects the work_struct and
		 * pm_iir. */
		spinlock_t lock;

		/* The below variables an all the rps hw state are protected by
		 * dev->struct mutext. */
		u8 cur_delay;
		u8 min_delay;
		u8 max_delay;
		u8 rpe_delay;
		u8 requested_delay; /* To track the actual requested delay */
		u8 lowest_delay; /* lowest possible delay on the platform */

		u8 rp_up_masked;
		u8 rp_down_masked;

		u32 cz_freq;
		u32 ei_interrupt_count;

		u32 cz_ts_up_EI;
		u32 render_up_EI_C0;
		u32 media_up_EI_C0;
		u32 cz_ts_down_EI;
		u32 render_down_EI_C0;
		u32 media_down_EI_C0;

		struct mutex rps_mutex;
	} rps;

	/* Runtime power management related */
	struct {
		/* To track (num of get calls - num of put calls)
		 * made by procfs
		 */
		atomic_t procfs_count;
		/* To make sure ring get/put are in pair */
		bool ring_active;
		struct proc_dir_entry *i915_proc_file;
		struct proc_dir_entry *local_proc_file;
	} rpm;

	u8 cur_delay;
	u8 min_delay;
	u8 max_delay;
	u8 fmax;
	u8 fstart;

#ifdef CONFIG_DEBUG_FS
	/* Variables declaration for DPST */
	struct {
		u32 bin_data[DPST_BIN_COUNT];
		u32 luma_data[DPST_LUMA_COUNT];
		u32 num_interrupt;
	} dpst;
#endif
	/* Adding this to fallback to normal Turbo logic */
	bool use_RC0_residency_for_turbo;

	struct {
		atomic_t up_threshold;
		atomic_t down_threshold;
	} turbodebug;

	int max_freq_enable_count;

	u64 last_count1;
	unsigned long last_time1;
	unsigned long chipset_power;
	u64 last_count2;
	struct timespec last_time2;
	unsigned long gfx_power;
	int c_m;
	int r_t;
	u8 corr;

	enum no_fbc_reason no_fbc_reason;

	struct drm_mm_node *compressed_fb;
	struct drm_mm_node *compressed_llb;

	struct drm_mm_node *vlv_pctx;

	unsigned long last_gpu_reset;

	/* list of fbdev register on this device */
	struct intel_fbdev *fbdev;

	struct backlight_device *backlight;

	struct drm_property *broadcast_rgb_property;
	struct drm_property *force_audio_property;
	struct drm_property *force_pfit_property;

	struct work_struct parity_error_work;
	bool hw_contexts_disabled;
	uint32_t hw_context_size;
	bool need_pcbr_setup;
#ifdef CONFIG_DRM_VXD_BYT
	struct drm_psb_private *vxd_priv;
	int (*vxd_driver_open)(struct drm_device *dev, struct drm_file *file);
	void (*vxd_lastclose)(struct drm_device *dev);
	long (*vxd_ioctl)(struct file *filp,
		unsigned int cmd, unsigned long arg);
	int (*vxd_release)(struct inode *inode, struct file *filp);
	int (*psb_mmap)(struct file *filp, struct vm_area_struct *vma);
	int (*psb_msvdx_interrupt)(void *pvData);
#endif
	/* Added for HDMI Audio */
	had_event_call_back had_event_callbacks;
	struct snd_intel_had_interface *had_interface;
	void *had_pvt_data;
	int tmds_clock_speed;
	int hdmi_audio_interrupt_mask;
	struct work_struct hdmi_audio_wq;

	/* Added for DPST */
	struct task_struct *dpst_task;
	u32 dpst_signal;
	u32 dpst_backlight_factor;
	u32 blc_data;
	u32 blc_user;
	bool is_dpst_enabled;

	uint32_t watchdog_threshold[I915_NUM_RINGS];

	int shut_down_state;
	bool is_resuming;
	bool is_turbo_enabled;
} drm_i915_private_t;

/* Iterate over initialised rings */
#define for_each_ring(ring__, dev_priv__, i__) \
	for ((i__) = 0; (i__) < I915_NUM_RINGS; (i__)++) \
		if (((ring__) = &(dev_priv__)->ring[(i__)]), intel_ring_initialized((ring__)))

enum hdmi_force_audio {
	HDMI_AUDIO_OFF_DVI = -2,	/* no aux data for HDMI-DVI converter */
	HDMI_AUDIO_OFF,			/* force turn off HDMI audio */
	HDMI_AUDIO_AUTO,		/* trust EDID */
	HDMI_AUDIO_ON,			/* force turn on HDMI audio */
};

enum panel_fitter {
	PFIT_OFF,
	AUTOSCALE,
	PILLARBOX,
	LETTERBOX,
};

enum i915_cache_level {
	I915_CACHE_NONE = 0,
	I915_CACHE_LLC,
	I915_CACHE_LLC_MLC, /* gen6+, in docs at least! */
	I915_CACHE_LLC_ELLC, /* some HSW skus */
	I915_CACHE_ELLC,    /* some HSW skus */
};

struct drm_i915_gem_object_ops {
	int (*get_pages)(struct drm_i915_gem_object *,
			 struct page **pages,
			 gfp_t gfpmask,
			 u32 *offset);
	int (*put_pages)(struct drm_i915_gem_object *);
	void(*release)(struct drm_i915_gem_object *);
	bool(*is_vmap_obj)(void);
};

struct drm_i915_gem_object {
	struct drm_gem_object base;

	/** Current space allocated to this object in the GTT, if any. */
	struct drm_mm_node *gtt_space;
	struct list_head gtt_list;

	/** This object's place on the active/inactive lists */
	struct list_head ring_list;
	struct list_head mm_list;
	/** This object's place in the batchbuffer or on the eviction list */
	struct list_head exec_list;

	/**
	 * This is set if the object is on the active lists (has pending
	 * rendering and so a non-zero seqno), and is not set if it i s on
	 * inactive (ready to be unbound) list.
	 */
	unsigned int active:1;

	/**
	 * This is set if the object has been written to since last bound
	 * to the GTT
	 */
	unsigned int dirty:1;

	/**
	 * Fence register bits (if any) for this object.  Will be set
	 * as needed when mapped into the GTT.
	 * Protected by dev->struct_mutex.
	 */
	signed int fence_reg:I915_MAX_NUM_FENCE_BITS;

	/**
	 * Advice: are the backing pages purgeable?
	 */
	unsigned int madv:2;

	/**
	 * Current tiling mode for the object.
	 */
	unsigned int tiling_mode:2;
	/**
	 * Whether the tiling parameters for the currently associated fence
	 * register have changed. Note that for the purposes of tracking
	 * tiling changes we also treat the unfenced register, the register
	 * slot that the object occupies whilst it executes a fenced
	 * command (such as BLT on gen2/3), as a "fence".
	 */
	unsigned int fence_dirty:1;

	/** How many users have pinned this object in GTT space. The following
	 * users can each hold at most one reference: pwrite/pread, pin_ioctl
	 * (via user_pin_count), execbuffer (objects are not allowed multiple
	 * times for the same batchbuffer), and the framebuffer code. When
	 * switching/pageflipping, the framebuffer code has at most two buffers
	 * pinned per crtc.
	 *
	 * In the worst case this is 1 + 1 + 1 + 2*2 = 7. That would fit into 3
	 * bits with absolutely no headroom. So use 4 bits. */
	/**
	 * But somehow VLV manages to exceed this counter, especially
	 * in Widi case, so time being use 5 bits to provide aditional headroom.
	 * Todo, understand that why 4 bits are proving insufficient
	 */
	unsigned int pin_count:5;
#define DRM_I915_GEM_OBJECT_MAX_PIN_COUNT 0x1f

	/**
	 * Is the object at the current location in the gtt mappable and
	 * fenceable? Used to avoid costly recalculations.
	 */
	unsigned int map_and_fenceable:1;

	/**
	 * Whether the current gtt mapping needs to be mappable (and isn't just
	 * mappable by accident). Track pin and fault separate for a more
	 * accurate mappable working set.
	 */
	unsigned int fault_mappable:1;
	unsigned int pin_mappable:1;

	/*
	 * Is the GPU currently using a fence to access this buffer,
	 */
	unsigned int pending_fenced_gpu_access:1;
	unsigned int fenced_gpu_access:1;

	unsigned int cache_level:2;

	unsigned int has_aliasing_ppgtt_mapping:1;
	unsigned int has_global_gtt_mapping:1;

	/*
	 * Is the object to be mapped as read-only to the GPU
	 * Only honoured if hardware has relevant pte bit
	 */
	unsigned long gt_ro;
	unsigned long gt_old_ro;

	struct page **pages;

	/**
	 * DMAR support
	 */
	struct scatterlist *sg_list;
	int num_sg;

	/* prime dma-buf support */
	struct sg_table *sg_table;
	void *dma_buf_vmapping;
	int vmapping_count;

	/**
	 * Used for performing relocations during execbuffer insertion.
	 */
	struct hlist_node exec_node;
	unsigned long exec_handle;
	struct drm_i915_gem_exec_object2 *exec_entry;

	/**
	 * Current offset of the object in GTT space.
	 *
	 * This is the same as gtt_space->start
	 */
	uint32_t gtt_offset;

	struct intel_ring_buffer *ring;

	/** Breadcrumb of last rendering to the buffer. */
	uint32_t last_read_seqno;
	uint32_t last_write_seqno;
	/** Breadcrumb of last fenced GPU access to the buffer. */
	uint32_t last_fenced_seqno;

	/** Current tiling stride for the object, if it's tiled. */
	uint32_t stride;

	/** Record of address bit 17 of each page at last unbind. */
	unsigned long *bit_17;

	/** User space pin count and filp owning the pin */
	uint32_t user_pin_count;
	struct drm_file *pin_filp;

	/** for phy allocated objects */
	struct drm_i915_gem_phys_object *phys_obj;

	/**
	 * Number of crtcs where this object is currently the fb, but
	 * will be page flipped away on the next vblank.  When it
	 * reaches 0, dev_priv->pending_flip_queue will be woken up.
	 */
	atomic_t pending_flip;

	/** Object datatype */
	uint32_t datatype;
};

struct i915_gem_vmap_object {
	struct drm_i915_gem_object gem;
	uintptr_t user_ptr;
	size_t user_size;
	int read_only;
	struct mm_struct *mm;
#if defined(CONFIG_MMU_NOTIFIER)
	struct mmu_notifier *mn;
#endif
};

union drm_i915_gem_objects {
	struct drm_i915_gem_object base;
	struct i915_gem_vmap_object vmap;
};

#define to_intel_bo(x) container_of(x, struct drm_i915_gem_object, base)

/**
 * Request queue structure.
 *
 * The request queue allows us to note sequence numbers that have been emitted
 * and may be associated with active buffers to be retired.
 *
 * By keeping this list, we can avoid having to do questionable
 * sequence-number comparisons on buffer last_rendering_seqnos, and associate
 * an emission time with seqnos for tracking how far ahead of the GPU we are.
 */
struct drm_i915_gem_request {
	/** On Which ring this request was generated */
	struct intel_ring_buffer *ring;

	/** GEM sequence number associated with this request. */
	uint32_t seqno;

	/** Postion in the ringbuffer of the end of the request */
	u32 tail;

	/** Time at which this request was emitted, in jiffies. */
	unsigned long emitted_jiffies;

	/** global list entry for this request */
	struct list_head list;

	struct drm_i915_file_private *file_priv;
	/** file_priv list entry for this request */
	struct list_head client_list;
};

struct drm_i915_file_private {
	struct {
		struct spinlock lock;
		struct list_head request_list;
	} mm;
	struct idr context_idr;
#ifdef CONFIG_DRM_VXD_BYT
	struct psb_fpriv *pPriv;
#endif
	struct {
		int max_freq;
		int rc6_disable;
	} perfmon_override_counter;
};

#define INTEL_INFO(dev)	(((struct drm_i915_private *) (dev)->dev_private)->info)

#define IS_I830(dev)		((dev)->pci_device == 0x3577)
#define IS_845G(dev)		((dev)->pci_device == 0x2562)
#define IS_I85X(dev)		(INTEL_INFO(dev)->is_i85x)
#define IS_I865G(dev)		((dev)->pci_device == 0x2572)
#define IS_I915G(dev)		(INTEL_INFO(dev)->is_i915g)
#define IS_I915GM(dev)		((dev)->pci_device == 0x2592)
#define IS_I945G(dev)		((dev)->pci_device == 0x2772)
#define IS_I945GM(dev)		(INTEL_INFO(dev)->is_i945gm)
#define IS_BROADWATER(dev)	(INTEL_INFO(dev)->is_broadwater)
#define IS_CRESTLINE(dev)	(INTEL_INFO(dev)->is_crestline)
#define IS_GM45(dev)		((dev)->pci_device == 0x2A42)
#define IS_G4X(dev)		(INTEL_INFO(dev)->is_g4x)
#define IS_PINEVIEW_G(dev)	((dev)->pci_device == 0xa001)
#define IS_PINEVIEW_M(dev)	((dev)->pci_device == 0xa011)
#define IS_PINEVIEW(dev)	(INTEL_INFO(dev)->is_pineview)
#define IS_G33(dev)		(INTEL_INFO(dev)->is_g33)
#define IS_IRONLAKE_D(dev)	((dev)->pci_device == 0x0042)
#define IS_IRONLAKE_M(dev)	((dev)->pci_device == 0x0046)
#define IS_IVYBRIDGE(dev)	(INTEL_INFO(dev)->is_ivybridge)
#define IS_VALLEYVIEW(dev)	(INTEL_INFO(dev)->is_valleyview)
#define IS_VALLEYVIEWP_M(dev)((dev)->pci_device == 0x0F31)
#define IS_HASWELL(dev)	(INTEL_INFO(dev)->is_haswell)
#define IS_MOBILE(dev)		(INTEL_INFO(dev)->is_mobile)

/*
 * The genX designation typically refers to the render engine, so render
 * capability related checks should use IS_GEN, while display and other checks
 * have their own (e.g. HAS_PCH_SPLIT for ILK+ display, IS_foo for particular
 * chips, etc.).
 */
#define IS_GEN2(dev)	(INTEL_INFO(dev)->gen == 2)
#define IS_GEN3(dev)	(INTEL_INFO(dev)->gen == 3)
#define IS_GEN4(dev)	(INTEL_INFO(dev)->gen == 4)
#define IS_GEN5(dev)	(INTEL_INFO(dev)->gen == 5)
#define IS_GEN6(dev)	(INTEL_INFO(dev)->gen == 6)
#define IS_GEN7(dev)	(INTEL_INFO(dev)->gen == 7)

#define HAS_BSD(dev)            (INTEL_INFO(dev)->has_bsd_ring)
#define HAS_BLT(dev)            (INTEL_INFO(dev)->has_blt_ring)
#define HAS_LLC(dev)            (INTEL_INFO(dev)->has_llc)
#define I915_NEED_GFX_HWS(dev)	(INTEL_INFO(dev)->need_gfx_hws)

#define HAS_HW_CONTEXTS(dev)	(INTEL_INFO(dev)->gen >= 6 && !IS_VALLEYVIEW(dev))
#define HAS_ALIASING_PPGTT(dev)	(INTEL_INFO(dev)->gen >=6 && !IS_VALLEYVIEW(dev))

#define HAS_OVERLAY(dev)		(INTEL_INFO(dev)->has_overlay)
#define OVERLAY_NEEDS_PHYSICAL(dev)	(INTEL_INFO(dev)->overlay_needs_physical)

/* With the 945 and later, Y tiling got adjusted so that it was 32 128-byte
 * rows, which changed the alignment requirements and fence programming.
 */
#define HAS_128_BYTE_Y_TILING(dev) (!IS_GEN2(dev) && !(IS_I915G(dev) || \
						      IS_I915GM(dev)))
#define SUPPORTS_DIGITAL_OUTPUTS(dev)	(!IS_GEN2(dev) && !IS_PINEVIEW(dev))
#define SUPPORTS_INTEGRATED_HDMI(dev)	(IS_G4X(dev) || IS_GEN5(dev))
#define SUPPORTS_INTEGRATED_DP(dev)	(IS_G4X(dev) || IS_GEN5(dev))
#define SUPPORTS_EDP(dev)		(IS_IRONLAKE_M(dev))
#define SUPPORTS_TV(dev)		(INTEL_INFO(dev)->supports_tv)
#define I915_HAS_HOTPLUG(dev)		 (INTEL_INFO(dev)->has_hotplug)
/* dsparb controlled by hw only */
#define DSPARB_HWCONTROL(dev) (IS_G4X(dev) || IS_IRONLAKE(dev))

#define HAS_FW_BLC(dev) (INTEL_INFO(dev)->gen > 2)
#define HAS_PIPE_CXSR(dev) (INTEL_INFO(dev)->has_pipe_cxsr)
#define I915_HAS_FBC(dev) (INTEL_INFO(dev)->has_fbc)

#define HAS_PIPE_CONTROL(dev) (INTEL_INFO(dev)->gen >= 5)

#define INTEL_PCH_TYPE(dev) (((struct drm_i915_private *)(dev)->dev_private)->pch_type)
#define HAS_PCH_LPT(dev) (INTEL_PCH_TYPE(dev) == PCH_LPT)
#define HAS_PCH_CPT(dev) (INTEL_PCH_TYPE(dev) == PCH_CPT)
#define HAS_PCH_IBX(dev) (INTEL_PCH_TYPE(dev) == PCH_IBX)
#define HAS_PCH_SPLIT(dev) (INTEL_PCH_TYPE(dev) != PCH_NONE)

#define HAS_FORCE_WAKE(dev) (INTEL_INFO(dev)->has_force_wake)

#define HAS_L3_GPU_CACHE(dev) (IS_IVYBRIDGE(dev) || IS_HASWELL(dev))

#include "i915_trace.h"

/**
 * RC6 is a special power stage which allows the GPU to enter an very
 * low-voltage mode when idle, using down to 0V while at this stage.  This
 * stage is entered automatically when the GPU is idle when RC6 support is
 * enabled, and as soon as new workload arises GPU wakes up automatically as well.
 *
 * There are different RC6 modes available in Intel GPU, which differentiate
 * among each other with the latency required to enter and leave RC6 and
 * voltage consumed by the GPU in different states.
 *
 * The combination of the following flags define which states GPU is allowed
 * to enter, while RC6 is the normal RC6 state, RC6p is the deep RC6, and
 * RC6pp is deepest RC6. Their support by hardware varies according to the
 * GPU, BIOS, chipset and platform. RC6 is usually the safest one and the one
 * which brings the most power savings; deeper states save more power, but
 * require higher latency to switch to and wake up.
 */
#define INTEL_RC6_ENABLE			(1<<0)
#define INTEL_RC6p_ENABLE			(1<<1)
#define INTEL_RC6pp_ENABLE			(1<<2)

extern struct drm_ioctl_desc i915_ioctls[];
extern int i915_max_ioctl;
extern unsigned int i915_fbpercrtc __always_unused;
extern int i915_panel_ignore_lid __read_mostly;
extern unsigned int i915_powersave __read_mostly;
extern int i915_semaphores __read_mostly;
extern unsigned int i915_lvds_downclock __read_mostly;
extern int i915_lvds_channel_mode __read_mostly;
extern int i915_panel_use_ssc __read_mostly;
extern int i915_vbt_sdvo_panel_type __read_mostly;
extern int i915_mipi_panel_id __read_mostly;
extern int i915_enable_rc6 __read_mostly;
extern int i915_rotation __read_mostly;
extern int i915_enable_fbc __read_mostly;
extern bool i915_enable_hangcheck __read_mostly;
extern unsigned int i915_hangcheck_period __read_mostly;
extern unsigned int i915_ring_reset_min_alive_period __read_mostly;
extern unsigned int i915_gpu_reset_min_alive_period __read_mostly;
extern int i915_enable_watchdog __read_mostly;
extern int i915_enable_ppgtt __read_mostly;
extern int i915_enable_turbo __read_mostly;
extern int i915_psr_support __read_mostly;
extern struct drm_display_mode rot_mode;

extern int i915_suspend(struct drm_device *dev, pm_message_t state);
extern int i915_resume(struct drm_device *dev);
extern int i915_master_create(struct drm_device *dev, struct drm_master *master);
extern void i915_master_destroy(struct drm_device *dev, struct drm_master *master);

				/* i915_dma.c */
void i915_update_dri1_breadcrumb(struct drm_device *dev);
extern void i915_kernel_lost_context(struct drm_device * dev);
extern int i915_driver_load(struct drm_device *, unsigned long flags);
extern int i915_driver_unload(struct drm_device *);
extern int i915_driver_open(struct drm_device *dev, struct drm_file *file_priv);
extern void i915_driver_lastclose(struct drm_device * dev);
extern void i915_driver_preclose(struct drm_device *dev,
				 struct drm_file *file_priv);
extern void i915_driver_postclose(struct drm_device *dev,
				  struct drm_file *file_priv);
extern int i915_driver_device_is_agp(struct drm_device * dev);
#ifdef CONFIG_COMPAT
extern long i915_compat_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg);
#endif
extern int i915_emit_box(struct drm_device *dev,
			 struct drm_clip_rect *box,
			 int DR1, int DR4);
extern int intel_gpu_reset(struct drm_device *dev);
extern int i915_handle_hung_ring(struct drm_device *dev, uint32_t ringid);
extern int i915_reset(struct drm_device *dev);
extern unsigned long i915_chipset_val(struct drm_i915_private *dev_priv);
extern unsigned long i915_mch_val(struct drm_i915_private *dev_priv);
extern unsigned long i915_gfx_val(struct drm_i915_private *dev_priv);
extern void i915_update_gfx_val(struct drm_i915_private *dev_priv);


/* i915_irq.c */
void i915_hangcheck_sample(unsigned long data);
void i915_handle_error(struct drm_device *dev, struct intel_hangcheck *hc,
			int watchdog);

extern void intel_irq_init(struct drm_device *dev);
extern void intel_gt_init(struct drm_device *dev);

void i915_error_state_free(struct kref *error_ref);

void
i915_enable_pipestat(drm_i915_private_t *dev_priv, int pipe, u32 mask);

void
i915_disable_pipestat(drm_i915_private_t *dev_priv, int pipe, u32 mask);

void intel_enable_asle(struct drm_device *dev);

#ifdef CONFIG_DEBUG_FS
extern void i915_destroy_error_state(struct drm_device *dev);
#else
#define i915_destroy_error_state(x)
#endif


/* i915_dpst.c */
int i915_dpst_context(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_reset_histogram(struct drm_device *dev);
int i915_dpst_enable_hist_interrupt(struct drm_device *dev, bool enable);
u32 i915_dpst_get_brightness(struct drm_device *dev);
void i915_dpst_set_brightness(struct drm_device *dev, u32 brightness_val);

/* i915_gem.c */
int i915_gem_init_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_create_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file_priv);
int i915_gem_pread_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv);
int i915_gem_pwrite_ioctl(struct drm_device *dev, void *data,
			  struct drm_file *file_priv);
int i915_gem_mmap_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_mmap_gtt_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_set_domain_ioctl(struct drm_device *dev, void *data,
			      struct drm_file *file_priv);
int i915_gem_sw_finish_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file_priv);
int i915_gem_execbuffer(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_execbuffer2(struct drm_device *dev, void *data,
			 struct drm_file *file_priv);
int i915_gem_pin_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv);
int i915_gem_unpin_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv);
int i915_gem_busy_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_get_cacheing_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file);
int i915_gem_set_cacheing_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file);
int i915_gem_throttle_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file_priv);
int i915_gem_madvise_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv);
int i915_gem_entervt_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv);
int i915_gem_leavevt_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file_priv);
int i915_gem_vmap_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
int i915_gem_set_tiling(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_get_tiling(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int i915_gem_get_aperture_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv);
int i915_gem_wait_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
void i915_gem_load(struct drm_device *dev);
int i915_gem_init_object(struct drm_gem_object *obj);
struct drm_i915_gem_object *i915_gem_alloc_object(struct drm_device *dev,
						  size_t size);
void i915_gem_object_init(struct drm_device *dev,
				struct drm_i915_gem_object *obj,
				const struct drm_i915_gem_object_ops *ops);
void i915_gem_free_object(struct drm_gem_object *obj);
int __must_check i915_gem_object_pin(struct drm_i915_gem_object *obj,
				     uint32_t alignment,
				     bool map_and_fenceable);
void i915_gem_object_unpin(struct drm_i915_gem_object *obj);
int __must_check i915_gem_object_unbind(struct drm_i915_gem_object *obj);
void i915_gem_release_mmap(struct drm_i915_gem_object *obj);
void i915_gem_lastclose(struct drm_device *dev);

int i915_gem_object_get_pages(struct drm_i915_gem_object *obj,
			  gfp_t gfpmask);
int i915_gem_object_get_pages_gtt(struct drm_i915_gem_object *obj,
				  struct page **pages,
				  gfp_t gfpmask,
				  u32 *offset);
int __must_check i915_mutex_lock_interruptible(struct drm_device *dev);
int i915_gem_object_sync(struct drm_i915_gem_object *obj,
			 struct intel_ring_buffer *to, bool add_request);
void i915_gem_object_move_to_active(struct drm_i915_gem_object *obj,
				    struct intel_ring_buffer *ring,
				    u32 seqno);

int i915_gem_dumb_create(struct drm_file *file_priv,
			 struct drm_device *dev,
			 struct drm_mode_create_dumb *args);
int i915_gem_mmap_gtt(struct drm_file *file_priv, struct drm_device *dev,
		      uint32_t handle, uint64_t *offset);
int i915_gem_dumb_destroy(struct drm_file *file_priv, struct drm_device *dev,
			  uint32_t handle);
/**
 * Returns true if seq1 is later than seq2.
 */
static inline bool
i915_seqno_passed(uint32_t seq1, uint32_t seq2)
{
	return (int32_t)(seq1 - seq2) >= 0;
}

u32 i915_gem_next_request_seqno(struct intel_ring_buffer *ring);

int __must_check i915_gem_object_get_fence(struct drm_i915_gem_object *obj);
int __must_check i915_gem_object_put_fence(struct drm_i915_gem_object *obj);

static inline bool
i915_gem_object_pin_fence(struct drm_i915_gem_object *obj)
{
	if (obj->fence_reg != I915_FENCE_REG_NONE) {
		struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
		dev_priv->fence_regs[obj->fence_reg].pin_count++;
		return true;
	} else
		return false;
}

static inline void
i915_gem_object_unpin_fence(struct drm_i915_gem_object *obj)
{
	if (obj->fence_reg != I915_FENCE_REG_NONE) {
		struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
		dev_priv->fence_regs[obj->fence_reg].pin_count--;
	}
}

static inline bool
i915_gem_is_vmap_object(struct drm_i915_gem_object *obj)
{
	const struct drm_i915_gem_object_ops *ops = obj->base.driver_private;
	if (ops == NULL)
		return 0;
	return ops->is_vmap_obj();
}

void i915_gem_retire_requests(struct drm_device *dev);
void i915_gem_retire_requests_ring(struct intel_ring_buffer *ring);
int __must_check i915_gem_check_wedge(struct drm_i915_private *dev_priv,
				      bool interruptible,
				      struct intel_ring_buffer *ring);

void i915_gem_reset(struct drm_device *dev);
void i915_gem_clflush_object(struct drm_i915_gem_object *obj);
int __must_check i915_gem_object_set_domain(struct drm_i915_gem_object *obj,
					    uint32_t read_domains,
					    uint32_t write_domain);
int __must_check i915_gem_object_finish_gpu(struct drm_i915_gem_object *obj);
int __must_check i915_gem_init(struct drm_device *dev);
int __must_check i915_gem_init_hw(struct drm_device *dev);
void i915_gem_l3_remap(struct drm_device *dev);
void i915_gem_init_swizzling(struct drm_device *dev);
void i915_gem_init_ppgtt(struct drm_device *dev);
void i915_gem_cleanup_ringbuffer(struct drm_device *dev);
int __must_check i915_gpu_idle(struct drm_device *dev);
int __must_check i915_gem_idle(struct drm_device *dev);
int i915_add_request(struct intel_ring_buffer *ring,
		     struct drm_file *file,
		     struct drm_i915_gem_request *request);
int i915_add_request_noflush(struct intel_ring_buffer *ring);
int __must_check i915_wait_seqno(struct intel_ring_buffer *ring,
				 uint32_t seqno);
int i915_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
int __must_check
i915_gem_object_set_to_gtt_domain(struct drm_i915_gem_object *obj,
				  bool write);
int __must_check
i915_gem_object_set_to_cpu_domain(struct drm_i915_gem_object *obj, bool write);
int __must_check
i915_gem_object_pin_to_display_plane(struct drm_i915_gem_object *obj,
				     u32 alignment,
				     struct intel_ring_buffer *pipelined);
int i915_gem_attach_phys_object(struct drm_device *dev,
				struct drm_i915_gem_object *obj,
				int id,
				int align);
void i915_gem_detach_phys_object(struct drm_device *dev,
				 struct drm_i915_gem_object *obj);
void i915_gem_free_all_phys_object(struct drm_device *dev);
void i915_gem_release(struct drm_device *dev, struct drm_file *file);
int i915_gem_access_datatype(struct drm_device *dev, void *data,
		   struct drm_file *file);
uint32_t
i915_gem_get_unfenced_gtt_alignment(struct drm_device *dev,
				    uint32_t size,
				    int tiling_mode);

int i915_gem_object_set_cache_level(struct drm_i915_gem_object *obj,
				    enum i915_cache_level cache_level);

struct drm_gem_object *i915_gem_prime_import(struct drm_device *dev,
				struct dma_buf *dma_buf);

struct dma_buf *i915_gem_prime_export(struct drm_device *dev,
				struct drm_gem_object *gem_obj, int flags);

/* i915_gem_context.c */
void i915_gem_context_init(struct drm_device *dev);
void i915_gem_context_fini(struct drm_device *dev);
void i915_gem_context_close(struct drm_device *dev, struct drm_file *file);
int i915_switch_context(struct intel_ring_buffer *ring,
			struct drm_file *file, int to_id);
int i915_gem_context_create_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file);
int i915_gem_context_destroy_ioctl(struct drm_device *dev, void *data,
				   struct drm_file *file);

/* i915_gem_gtt.c */
int __must_check i915_gem_init_aliasing_ppgtt(struct drm_device *dev);
void i915_gem_cleanup_aliasing_ppgtt(struct drm_device *dev);
void i915_ppgtt_bind_object(struct i915_hw_ppgtt *ppgtt,
			    struct drm_i915_gem_object *obj,
			    enum i915_cache_level cache_level);
void i915_ppgtt_unbind_object(struct i915_hw_ppgtt *ppgtt,
			      struct drm_i915_gem_object *obj);

void i915_gem_restore_gtt_mappings(struct drm_device *dev);
int __must_check i915_gem_gtt_prepare_object(struct drm_i915_gem_object *obj);
void i915_gem_gtt_bind_object(struct drm_i915_gem_object *obj,
				enum i915_cache_level cache_level);
void i915_gem_gtt_unbind_object(struct drm_i915_gem_object *obj);
void i915_gem_gtt_finish_object(struct drm_i915_gem_object *obj);
void i915_gem_init_global_gtt(struct drm_device *dev,
			      unsigned long start,
			      unsigned long mappable_end,
			      unsigned long end);

/* i915_gem_evict.c */
int __must_check i915_gem_evict_something(struct drm_device *dev, int min_size,
					  unsigned alignment,
					  unsigned cache_level,
					  bool mappable);
int i915_gem_evict_everything(struct drm_device *dev, bool purgeable_only);

/* i915_gem_stolen.c */
int i915_gem_init_stolen(struct drm_device *dev);
void i915_gem_cleanup_stolen(struct drm_device *dev);

/* i915_gem_tiling.c */
void i915_gem_detect_bit_6_swizzle(struct drm_device *dev);
void i915_gem_object_do_bit_17_swizzle(struct drm_i915_gem_object *obj);
void i915_gem_object_save_bit_17_swizzle(struct drm_i915_gem_object *obj);

/* i915_gem_debug.c */
void i915_gem_dump_object(struct drm_i915_gem_object *obj, int len,
			  const char *where, uint32_t mark);
#if WATCH_LISTS
int i915_verify_lists(struct drm_device *dev);
#else
#define i915_verify_lists(dev) 0
#endif
void i915_gem_object_check_coherency(struct drm_i915_gem_object *obj,
				     int handle);
void i915_gem_dump_object(struct drm_i915_gem_object *obj, int len,
			  const char *where, uint32_t mark);

/* i915_debugfs.c */
int i915_debugfs_init(struct drm_minor *minor);
void i915_debugfs_cleanup(struct drm_minor *minor);

/* i915_suspend.c */
extern int i915_save_state(struct drm_device *dev);
extern int i915_restore_state(struct drm_device *dev);
extern void i915_pm_init(struct drm_device *dev);
extern void i915_pm_deinit(struct drm_device *dev);

/* i915_sysfs.c */
void i915_setup_sysfs(struct drm_device *dev_priv);
void i915_teardown_sysfs(struct drm_device *dev_priv);

/* intel_i2c.c */
extern int intel_setup_gmbus(struct drm_device *dev);
extern void intel_teardown_gmbus(struct drm_device *dev);
extern inline bool intel_gmbus_is_port_valid(unsigned port)
{
	return (port >= GMBUS_PORT_SSC && port <= GMBUS_PORT_DPD);
}

void intel_set_gmbus_frequency(struct drm_i915_private *dev_priv);
extern struct i2c_adapter *intel_gmbus_get_adapter(
		struct drm_i915_private *dev_priv, unsigned port);
extern void intel_gmbus_set_speed(struct i2c_adapter *adapter, int speed);
extern void intel_gmbus_force_bit(struct i2c_adapter *adapter, bool force_bit);
extern inline bool intel_gmbus_is_forced_bit(struct i2c_adapter *adapter)
{
	return container_of(adapter, struct intel_gmbus, adapter)->force_bit;
}
extern void intel_i2c_reset(struct drm_device *dev);

/* intel_opregion.c */
extern int intel_opregion_setup(struct drm_device *dev);
#ifdef CONFIG_ACPI
extern void intel_opregion_init(struct drm_device *dev);
extern void intel_opregion_fini(struct drm_device *dev);
extern void intel_opregion_asle_intr(struct drm_device *dev);
extern void intel_opregion_gse_intr(struct drm_device *dev);
extern void intel_opregion_enable_asle(struct drm_device *dev);
#else
static inline void intel_opregion_init(struct drm_device *dev) { return; }
static inline void intel_opregion_fini(struct drm_device *dev) { return; }
static inline void intel_opregion_asle_intr(struct drm_device *dev) { return; }
static inline void intel_opregion_gse_intr(struct drm_device *dev) { return; }
static inline void intel_opregion_enable_asle(struct drm_device *dev) { return; }
#endif

/* intel_acpi.c */
#ifdef CONFIG_ACPI
extern void intel_register_dsm_handler(void);
extern void intel_unregister_dsm_handler(void);
#else
static inline void intel_register_dsm_handler(void) { return; }
static inline void intel_unregister_dsm_handler(void) { return; }
#endif /* CONFIG_ACPI */

/* modesetting */
extern void intel_modeset_init_hw(struct drm_device *dev);
extern void intel_modeset_init(struct drm_device *dev);
extern void intel_modeset_gem_init(struct drm_device *dev);
extern void intel_modeset_cleanup(struct drm_device *dev);
extern int intel_modeset_vga_set_state(struct drm_device *dev, bool state);
extern bool intel_fbc_enabled(struct drm_device *dev);
extern void intel_disable_fbc(struct drm_device *dev);
extern bool ironlake_set_drps(struct drm_device *dev, u8 val);
extern void ironlake_init_pch_refclk(struct drm_device *dev);
extern void gen6_set_rps(struct drm_device *dev, u8 val);
extern void valleyview_set_rps(struct drm_device *dev, u8 val);
extern bool valleyview_update_cur_delay(struct drm_device *dev);
extern void intel_detect_pch(struct drm_device *dev);
extern int intel_trans_dp_port_sel(struct drm_crtc *crtc);
extern int intel_enable_rc6(const struct drm_device *dev);
extern int i915_rotation_ffrd(const struct drm_device *dev,
			const struct drm_crtc *crtc);
extern void valleyview_enable_rps(struct drm_device *dev);
extern void valleyview_disable_rps(struct drm_device *dev);

extern bool i915_semaphore_is_enabled(struct drm_device *dev);
int i915_reg_read_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
int i915_set_plane_zorder(struct drm_device *dev, void *data,
			  struct drm_file *file);
int i915_enable_plane_reserved_reg_bit_2(struct drm_device *dev, void *data,
					 struct drm_file *file);
int i915_set_plane_180_rotation(struct drm_device *dev, void *data,
			struct drm_file *file);
int i915_disp_screen_control(struct drm_device *dev, void *data,
			struct drm_file *file);
int i915_set_plane_alpha(struct drm_device *dev, void *data,
			  struct drm_file *file);

/* overlay */
#ifdef CONFIG_DEBUG_FS
extern struct intel_overlay_error_state *intel_overlay_capture_error_state(struct drm_device *dev);
extern void intel_overlay_print_error_state(struct seq_file *m, struct intel_overlay_error_state *error);

extern struct intel_display_error_state *intel_display_capture_error_state(struct drm_device *dev);
extern void intel_display_print_error_state(struct seq_file *m,
					    struct drm_device *dev,
					    struct intel_display_error_state *error);
#endif

/* On SNB platform, before reading ring registers forcewake bit
 * must be set to prevent GT core from power down and stale values being
 * returned.
 */
void gen6_gt_force_wake_get(struct drm_i915_private *dev_priv, int fw_engine);
void gen6_gt_force_wake_put(struct drm_i915_private *dev_priv, int fw_engine);
int __gen6_gt_wait_for_fifo(struct drm_i915_private *dev_priv);
void gen6_gt_force_wake_restore(struct drm_i915_private *dev_priv);

int intel_fuse_read32(struct drm_i915_private *dev_priv, u32 reg, u32 *val);
u32 intel_dpio_read(struct drm_i915_private *dev_priv, int reg);
void intel_dpio_write(struct drm_i915_private *dev_priv, int reg, u32 val);

void intel_iosf_rw(struct drm_i915_private *dev_priv,
			u8 opcode, u32 port, u32 reg, u32 *val);

void vlv_force_wake_get(struct drm_i915_private *dev_priv, int fw_engine);
void vlv_force_wake_put(struct drm_i915_private *dev_priv, int fw_engine);
void vlv_force_wake_restore(struct drm_i915_private *dev_priv, int fw_engine);


#define FORCEWAKE_VLV_RENDER_RANGE_OFFSET(MmioOffset) \
			((MmioOffset >= 0x2000 && MmioOffset < 0x4000) ||\
			 (MmioOffset >= 0x5000 && MmioOffset < 0x8000) ||\
			 (MmioOffset >= 0xB000 && MmioOffset < 0x12000) ||\
			 (MmioOffset >= 0x2E000 && MmioOffset < 0x30000))

#define FORCEWAKE_VLV_MEDIA_RANGE_OFFSET(MmioOffset)\
			((MmioOffset >= 0x12000 && MmioOffset < 0x14000) ||\
			 (MmioOffset >= 0x22000 && MmioOffset < 0x24000) ||\
			 (MmioOffset >= 0x30000 && MmioOffset < 0x40000))

#define FORCEWAKE_RENDER	(1 << 0)
#define FORCEWAKE_MEDIA		(1 << 1)
#define FORCEWAKE_ALL		(FORCEWAKE_RENDER | FORCEWAKE_MEDIA)

#define KM_MEDIA_ENGINE_TIMEOUT_VALUE_IN_MS 60
#define KM_BSD_ENGINE_TIMEOUT_VALUE_IN_MS   60
#define KM_TIMER_MILLISECOND 1000

/* Timestamp timer resolution = 0.080 uSec, or 12500000 counts per second*/
#define KM_TIMESTAMP_CNTS_PER_SEC_80NS          12500000

/* Timestamp timer resolution = 0.640 uSec, or  1562500 counts per second*/
#define KM_TIMESTAMP_CNTS_PER_SEC_640NS          1562500

#define KM_TIMER_MHZ 1000000
#define KM_CD_CLK_FREQ (450 * KM_TIMER_MHZ)


#define __i915_read(x, y) \
	u##x i915_read##x(struct drm_i915_private *dev_priv, u32 reg, bool trace);

__i915_read(8, b)
__i915_read(16, w)
__i915_read(32, l)
__i915_read(64, q)
#undef __i915_read

#define __i915_write(x, y) \
	void i915_write##x(struct drm_i915_private *dev_priv, u32 reg, u##x val, bool trace);

__i915_write(8, b)
__i915_write(16, w)
__i915_write(32, l)
__i915_write(64, q)
#undef __i915_write

#define __i915_write_bits(x, y) \
	void i915_write_bits##x(struct drm_i915_private *dev_priv, \
		u32 reg, u##x val, u##x mask, bool trace);

__i915_write_bits(8, b)
__i915_write_bits(16, w)
__i915_write_bits(32, l)
__i915_write_bits(64, q)
#undef __i915_write_bits

#define I915_READ8(reg)		i915_read8(dev_priv, (reg), true)
#define I915_WRITE8(reg, val)	i915_write8(dev_priv, (reg), (val), true)

#define I915_READ16(reg)	i915_read16(dev_priv, (reg), true)
#define I915_WRITE16(reg, val)	i915_write16(dev_priv, (reg), (val), true)
#define I915_READ16_NOTRACE(reg)	i915_read16(dev_priv, (reg), false)
#define I915_WRITE16_NOTRACE(reg, val)	i915_write16(dev_priv, (reg), (val), false)

#define I915_READ(reg)		i915_read32(dev_priv, (reg), true)
#define I915_WRITE(reg, val)	i915_write32(dev_priv, (reg), (val), true)
#define I915_WRITE_BITS(reg, val, mask)	i915_write_bits32(dev_priv, \
				(reg), (val), (mask), true)
#define I915_READ_NOTRACE(reg)		i915_read32(dev_priv, (reg), false)
#define I915_WRITE_NOTRACE(reg, val)	i915_write32(dev_priv, (reg), (val), false)

#define I915_WRITE64(reg, val)	i915_write64(dev_priv, (reg), (val), true)
#define I915_READ64(reg)	i915_read64(dev_priv, (reg), true)

#define POSTING_READ(reg)	(void)I915_READ_NOTRACE(reg)
#define POSTING_READ16(reg)	(void)I915_READ16_NOTRACE(reg)

/* Runtime power management related */
#define RPM_AUTOSUSPEND		0x1
#define RPM_SYNC		0x2
#define RPM_SYNC_STRICT		0X4
#define RPM_NOIDLE		0X8
#define RPM_NORESUME		0x10

int i915_rpm_init(struct drm_device *dev);
int i915_rpm_deinit(struct drm_device *dev);

int i915_rpm_get(struct drm_device *drm_dev, u32 flags);
int i915_rpm_put(struct drm_device *drm_dev, u32 flags);

int i915_rpm_get_ring(struct intel_ring_buffer *ring);
int i915_rpm_put_ring(struct intel_ring_buffer *ring);

int i915_rpm_get_callback(struct drm_device *dev);
int i915_rpm_put_callback(struct drm_device *dev);

int i915_rpm_get_ioctl(struct drm_device *dev);
int i915_rpm_put_ioctl(struct drm_device *dev);

int i915_rpm_get_disp(struct drm_device *dev);
int i915_rpm_put_disp(struct drm_device *dev);

/* Runtime Power Management related */
int i915_rpm_get_procfs(struct inode *inode, struct file *file);
int i915_rpm_put_procfs(struct inode *inode, struct file *file);

#ifdef CONFIG_DRM_VXD_BYT
int i915_rpm_get_vxd(struct drm_device *dev);
int i915_rpm_put_vxd(struct drm_device *dev);
#endif

bool i915_rpm_access_check(struct drm_device *dev);
bool i915_is_device_active(struct drm_device *dev);
bool i915_is_device_resuming(struct drm_device *dev);
bool i915_is_device_suspended(struct drm_device *dev);
bool i915_is_device_suspending(struct drm_device *dev);

u32 intel_dpio_read32_tmp(struct drm_i915_private *dev_priv, u32 reg);
int intel_dpio_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_dpio_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_dpio_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
int intel_punit_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_punit_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_punit_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
int intel_gpio_nc_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_gpio_nc_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_gpio_nc_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
int intel_cck_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_cck_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_cck_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
int intel_ccu_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_ccu_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_ccu_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
int intel_gps_core_read32(struct drm_i915_private *dev_priv,
					u32 reg,  u32 *val);
int intel_gps_core_write32(struct drm_i915_private *dev_priv,
					u32 reg, u32 val);
int intel_gps_core_write32_bits(struct drm_i915_private *dev_priv,
					u32 reg, u32 val, u32 mask);
int intel_fuse_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);

int intel_pmc_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_pmc_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_pmc_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);
void i915_init_watchdog(struct drm_device *dev);

int intel_flisdsi_read32(struct drm_i915_private *dev_priv, u32 reg,  u32 *val);
int intel_flisdsi_write32(struct drm_i915_private *dev_priv, u32 reg, u32 val);
int intel_flisdsi_write32_bits(struct drm_i915_private *dev_priv, \
					u32 reg, u32 val, u32 mask);

#define intel_dpio_read(dev_priv, reg)	intel_dpio_read32_tmp(dev_priv, reg)
#define intel_dpio_write(dev_priv, reg, val) \
			intel_dpio_write32(dev_priv, reg, val)
#define intel_dpio_write_bits(dev_priv, reg, val, mask) \
			intel_dpio_write32_bits(dev_priv, reg, val, mask)
#define intel_punit_read(dev_priv, reg, val) \
			intel_punit_read32(dev_priv, reg, val)
#define intel_punit_write(dev_priv, reg, val) \
			intel_punit_write32(dev_priv, reg, val)
#define intel_punit_write_bits(dev_priv, reg, val, mask) \
			intel_punit_write32_bits(dev_priv, reg, val, mask)
#define intel_gpio_nc_read(dev_priv, reg, val) \
			intel_gpio_nc_read32(dev_priv, reg, val)
#define intel_gpio_nc_write(dev_priv, reg, val) \
			intel_gpio_nc_write32(dev_priv, reg, val)
#define intel_gpio_nc_write_bits(dev_priv, reg, val, mask) \
			intel_punit_write32_bits(dev_priv, reg, val, mask)
#define intel_cck_read(dev_priv, reg, val) \
			intel_cck_read32(dev_priv, reg, val)
#define intel_cck_write(dev_priv, reg, val) \
			intel_cck_write32(dev_priv, reg, val)
#define intel_cck_write_bits(dev_priv, reg, val, mask) \
			intel_cck_write32_bits(dev_priv, reg, val, mask)
#define intel_ccu_read(dev_priv, reg, val) \
			intel_ccu_read32(dev_priv, reg, val)
#define intel_ccu_write(dev_priv, reg, val) \
			intel_ccu_write32(dev_priv, reg, val)
#define intel_ccu_write_bits(dev_priv, reg, val, mask) \
			intel_ccu_write32_bits(dev_priv, reg, val, mask)
#define intel_pmc_read(dev_priv, reg, val) \
			intel_pmc_read32(dev_priv, reg, val)
#define intel_pmc_write(dev_priv, reg, val) \
			intel_pmc_write32(dev_priv, reg, val)
#define intel_pmc_write_bits(dev_priv, reg, val, mask) \
			intel_pmc_write32_bits(dev_priv, reg, val, mask)
#define intel_flisdsi_read(dev_priv, reg, val) \
			intel_flisdsi_read32(dev_priv, reg, val)
#define intel_flisdsi_write(dev_priv, reg, val) \
			intel_flisdsi_write32(dev_priv, reg, val)
#define intel_flisdsi_write_bits(dev_priv, reg, val, mask) \
			intel_flisdsi_write32_bits(dev_priv, reg, val, mask)

/* i915_perfmon.c */
int i915_perfmon_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
void i915_perfmon_init(struct drm_file *file);
void i915_perfmon_close(struct drm_device *dev, struct drm_file *file);
#endif
