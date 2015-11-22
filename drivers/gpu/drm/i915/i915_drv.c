/* i915_drv.c -- i830,i845,i855,i865,i915 driver -*- linux-c -*-
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

#include <linux/device.h>
#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_trace.h"
#include "intel_drv.h"

#include <linux/console.h>
#include <linux/module.h>
#include <drm/drm_crtc_helper.h>
/*Added for HDMI Audio */
#include "hdmi_audio_if.h"

static int i915_modeset __read_mostly = -1;
module_param_named(modeset, i915_modeset, int, 0400);
MODULE_PARM_DESC(modeset,
		"Use kernel modesetting [KMS] (0=DRM_I915_KMS from .config, "
		"1=on, -1=force vga console preference [default])");

unsigned int i915_fbpercrtc __always_unused = 0;
module_param_named(fbpercrtc, i915_fbpercrtc, int, 0400);

int i915_panel_ignore_lid __read_mostly = 1;
module_param_named(panel_ignore_lid, i915_panel_ignore_lid, int, 0600);
MODULE_PARM_DESC(panel_ignore_lid,
		"Override lid status (0=autodetect, 1=autodetect disabled [default], "
		"-1=force lid closed, -2=force lid open)");

unsigned int i915_powersave __read_mostly = 1;
module_param_named(powersave, i915_powersave, int, 0600);
MODULE_PARM_DESC(powersave,
		"Enable powersavings, fbc, downclocking, etc. (default: true)");

int i915_semaphores __read_mostly = -1;
module_param_named(semaphores, i915_semaphores, int, 0600);
MODULE_PARM_DESC(semaphores,
		"Use semaphores for inter-ring sync (default: -1 (use per-chip defaults))");

int i915_enable_rc6 __read_mostly = -1;
module_param_named(i915_enable_rc6, i915_enable_rc6, int, 0400);
MODULE_PARM_DESC(i915_enable_rc6,
		"Enable power-saving render C-state 6. "
		"Different stages can be selected via bitmask values "
		"(0 = disable; 1 = enable rc6; 2 = enable deep rc6; 4 = enable deepest rc6). "
		"For example, 3 would enable rc6 and deep rc6, and 7 would enable everything. "
		"default: -1 (use per-chip default)");

int i915_enable_fbc __read_mostly = -1;
module_param_named(i915_enable_fbc, i915_enable_fbc, int, 0600);
MODULE_PARM_DESC(i915_enable_fbc,
		"Enable frame buffer compression for power savings "
		"(default: -1 (use per-chip default))");

unsigned int i915_lvds_downclock __read_mostly = 0;
module_param_named(lvds_downclock, i915_lvds_downclock, int, 0400);
MODULE_PARM_DESC(lvds_downclock,
		"Use panel (LVDS/eDP) downclocking for power savings "
		"(default: false)");

int i915_lvds_channel_mode __read_mostly;
module_param_named(lvds_channel_mode, i915_lvds_channel_mode, int, 0600);
MODULE_PARM_DESC(lvds_channel_mode,
		 "Specify LVDS channel mode "
		 "(0=probe BIOS [default], 1=single-channel, 2=dual-channel)");

int i915_panel_use_ssc __read_mostly = -1;
module_param_named(lvds_use_ssc, i915_panel_use_ssc, int, 0600);
MODULE_PARM_DESC(lvds_use_ssc,
		"Use Spread Spectrum Clock with panels [LVDS/eDP] "
		"(default: auto from VBT)");

int i915_vbt_sdvo_panel_type __read_mostly = -1;
module_param_named(vbt_sdvo_panel_type, i915_vbt_sdvo_panel_type, int, 0600);
MODULE_PARM_DESC(vbt_sdvo_panel_type,
		"Override/Ignore selection of SDVO panel mode in the VBT "
		"(-2=ignore, -1=auto [default], index in VBT BIOS table)");

int i915_mipi_panel_id __read_mostly = -1;
module_param_named(mipi_panel_id, i915_mipi_panel_id, int, 0600);
MODULE_PARM_DESC(mipi_panel_id,
		"MIPI Panel selection in case MIPI block is not present in VBT "
		"(-1=auto [default], mipi panel id)");

#if defined(CONFIG_TF103CE) || defined(CONFIG_TF303CL)
int hdmi_connect_state __read_mostly = 0;
module_param_named(hdmi_state, hdmi_connect_state, int, 0644);
MODULE_PARM_DESC(hdmi_state,
		 "Show hdmi status");

int i915_pre_emp_vswing_setting __read_mostly = 0;
module_param_named(pre_emp_vswing_setting, i915_pre_emp_vswing_setting, int, 0600);
MODULE_PARM_DESC(pre_emp_vswing_setting,
		 "Need to get HDMI pre-emp, vswing settings from VBT."
		 "(definitions: 0 = 1000MV_2DB ,1 = 1000MV_0DB, 2 = 800MV_0DB, 3 = 600MV_2DB, 4 = 600MV_0DB)");
#endif

static bool i915_try_reset __read_mostly = true;
module_param_named(reset, i915_try_reset, bool, 0600);
MODULE_PARM_DESC(reset, "Attempt GPU resets (default: true)");

bool i915_enable_hangcheck __read_mostly = true;
module_param_named(enable_hangcheck, i915_enable_hangcheck, bool, 0644);
MODULE_PARM_DESC(enable_hangcheck,
		"Periodically check GPU activity for detecting hangs. "
		"WARNING: Disabling this can cause system wide hangs. "
		"(default: true)");

unsigned int i915_hangcheck_period __read_mostly = 1000;

int hangcheck_period_set(const char *val, const struct kernel_param *kp)
{
	/* Custom set function so we can validate the range*/
	unsigned long num;
	int ret;

	ret = kstrtoul(val, 0, &num);

	if (ret)
		return ret;

	/* Enforce minimum delay in ms */
	if ((num >= MINIMUM_HANGCHECK_PERIOD)
	&& (num <= MAXIMUM_HANGCHECK_PERIOD)) {
		i915_hangcheck_period = num;
		return 0;
	}

	return -EINVAL;
}

static const struct kernel_param_ops hangcheck_ops = {
	.set = hangcheck_period_set,
	.get = param_get_uint,
};

module_param_cb(i915_hangcheck_period, &hangcheck_ops,
		&i915_hangcheck_period, 0644);
MODULE_PARM_DESC(i915_hangcheck_period,
		"The hangcheck timer period in milliseconds. "
		"The actual time to detect a hang may be 3 - 4 times "
		"this value (default = 1000ms)");

unsigned int i915_ring_reset_min_alive_period __read_mostly;
module_param_named(i915_ring_reset_min_alive_period,
		i915_ring_reset_min_alive_period, int, 0644);
MODULE_PARM_DESC(i915_ring_reset_min_alive_period,
		"Catch excessive ring resets. Each ring maintains a timestamp of "
		"the last time it was reset. If it hangs again within this period "
		"then switch to full GPU reset to try and clear the hang."
		"default=0 seconds (disabled)");

unsigned int i915_gpu_reset_min_alive_period __read_mostly;
module_param_named(i915_gpu_reset_min_alive_period,
		i915_gpu_reset_min_alive_period, int, 0644);
MODULE_PARM_DESC(i915_gpu_reset_min_alive_period,
		"Catch excessive GPU resets. If the GPU hangs again within this period "
		"following the previous GPU reset then declare it wedged and "
		"prevent further resets. "
		"default=0 seconds (disabled)");

int i915_enable_watchdog __read_mostly = 1;
module_param_named(i915_enable_watchdog, i915_enable_watchdog, int, 0644);
MODULE_PARM_DESC(i915_enable_watchdog,
		"Enable watchdog timers (default: true)");

int i915_enable_ppgtt __read_mostly = -1;
module_param_named(i915_enable_ppgtt, i915_enable_ppgtt, int, 0600);
MODULE_PARM_DESC(i915_enable_ppgtt,
		"Enable PPGTT (default: true)");

int i915_enable_psr __read_mostly = 0;
module_param_named(enable_psr, i915_enable_psr, int, 0600);
MODULE_PARM_DESC(enable_psr, "Enable PSR (default: false)");

unsigned int i915_preliminary_hw_support __read_mostly = IS_ENABLED(CONFIG_DRM_I915_PRELIMINARY_HW_SUPPORT);
module_param_named(preliminary_hw_support, i915_preliminary_hw_support, int, 0600);
MODULE_PARM_DESC(preliminary_hw_support,
		"Enable preliminary hardware support.");

int i915_disable_power_well __read_mostly = 1;
module_param_named(disable_power_well, i915_disable_power_well, int, 0600);
MODULE_PARM_DESC(disable_power_well,
		 "Disable the power well when possible (default: true)");

int i915_enable_ips __read_mostly = 1;
module_param_named(enable_ips, i915_enable_ips, int, 0600);
MODULE_PARM_DESC(enable_ips, "Enable IPS (default: true)");

bool i915_fastboot __read_mostly = 0;
module_param_named(fastboot, i915_fastboot, bool, 0600);
MODULE_PARM_DESC(fastboot, "Try to skip unnecessary mode sets at boot time "
		 "(default: false)");

int i915_enable_pc8 __read_mostly = 1;
module_param_named(enable_pc8, i915_enable_pc8, int, 0600);
MODULE_PARM_DESC(enable_pc8, "Enable support for low power package C states (PC8+) (default: true)");

int i915_pc8_timeout __read_mostly = 5000;
module_param_named(pc8_timeout, i915_pc8_timeout, int, 0600);
MODULE_PARM_DESC(pc8_timeout, "Number of msecs of idleness required to enter PC8+ (default: 5000)");

bool i915_prefault_disable __read_mostly;
module_param_named(prefault_disable, i915_prefault_disable, bool, 0600);
MODULE_PARM_DESC(prefault_disable,
		"Disable page prefaulting for pread/pwrite/reloc (default:false). For developers only.");

int i915_psr_support __read_mostly = 1;
module_param_named(psr_support, i915_psr_support, int, 0400);
MODULE_PARM_DESC(psr_support,
		"Specify PSR support parameter "
		"1 = supported [default], 0 = not supported");

int i915_enable_turbo __read_mostly = 1;
module_param_named(i915_enable_turbo, i915_enable_turbo, int, 0600);
MODULE_PARM_DESC(i915_enable_turbo,
		"Enable VLV Turbo (default: true)");

int i915_enable_kernel_batch_copy __read_mostly = -1;
module_param_named(i915_enable_kernel_batch_copy,
		i915_enable_kernel_batch_copy, int, 0600);
MODULE_PARM_DESC(i915_enable_kernel_batch_copy,
		"i915 submits a kernel managed copy of the user passed batch buffer. "
		"(default: -1)");

int i915_enable_cmd_parser __read_mostly = 1;
module_param_named(i915_enable_cmd_parser, i915_enable_cmd_parser, int, 0600);
MODULE_PARM_DESC(i915_enable_cmd_parser,
		"Enable command parsing (default: true)");

int i915_drrs_interval __read_mostly = 180;
module_param_named(drrs_interval, i915_drrs_interval, int, 0600);
MODULE_PARM_DESC(drrs_interval,
	"DRRS idleness detection interval (default: 180 ms)."
	"If this field is set to 0, then seamless DRRS feature "
	"based on idleness detection is disabled."
	"The interval is to be set in milliseconds.");

static struct drm_driver driver;
extern int intel_agp_enabled;

#define INTEL_VGA_DEVICE(id, info) {		\
	.class = PCI_BASE_CLASS_DISPLAY << 16,	\
	.class_mask = 0xff0000,			\
	.vendor = 0x8086,			\
	.device = id,				\
	.subvendor = PCI_ANY_ID,		\
	.subdevice = PCI_ANY_ID,		\
	.driver_data = (unsigned long) info }

#define INTEL_QUANTA_VGA_DEVICE(info) {		\
	.class = PCI_BASE_CLASS_DISPLAY << 16,	\
	.class_mask = 0xff0000,			\
	.vendor = 0x8086,			\
	.device = 0x16a,			\
	.subvendor = 0x152d,			\
	.subdevice = 0x8990,			\
	.driver_data = (unsigned long) info }


static const struct intel_device_info intel_i830_info = {
	.gen = 2, .is_mobile = 1, .cursor_needs_physical = 1, .num_pipes = 2,
	.has_overlay = 1, .overlay_needs_physical = 1,
};

static const struct intel_device_info intel_845g_info = {
	.gen = 2, .num_pipes = 1,
	.has_overlay = 1, .overlay_needs_physical = 1,
};

static const struct intel_device_info intel_i85x_info = {
	.gen = 2, .is_i85x = 1, .is_mobile = 1, .num_pipes = 2,
	.cursor_needs_physical = 1,
	.has_overlay = 1, .overlay_needs_physical = 1,
};

static const struct intel_device_info intel_i865g_info = {
	.gen = 2, .num_pipes = 1,
	.has_overlay = 1, .overlay_needs_physical = 1,
};

static const struct intel_device_info intel_i915g_info = {
	.gen = 3, .is_i915g = 1, .cursor_needs_physical = 1, .num_pipes = 2,
	.has_overlay = 1, .overlay_needs_physical = 1,
};
static const struct intel_device_info intel_i915gm_info = {
	.gen = 3, .is_mobile = 1, .num_pipes = 2,
	.cursor_needs_physical = 1,
	.has_overlay = 1, .overlay_needs_physical = 1,
	.supports_tv = 1,
};
static const struct intel_device_info intel_i945g_info = {
	.gen = 3, .has_hotplug = 1, .cursor_needs_physical = 1, .num_pipes = 2,
	.has_overlay = 1, .overlay_needs_physical = 1,
};
static const struct intel_device_info intel_i945gm_info = {
	.gen = 3, .is_i945gm = 1, .is_mobile = 1, .num_pipes = 2,
	.has_hotplug = 1, .cursor_needs_physical = 1,
	.has_overlay = 1, .overlay_needs_physical = 1,
	.supports_tv = 1,
};

static const struct intel_device_info intel_i965g_info = {
	.gen = 4, .is_broadwater = 1, .num_pipes = 2,
	.has_hotplug = 1,
	.has_overlay = 1,
};

static const struct intel_device_info intel_i965gm_info = {
	.gen = 4, .is_crestline = 1, .num_pipes = 2,
	.is_mobile = 1, .has_fbc = 1, .has_hotplug = 1,
	.has_overlay = 1,
	.supports_tv = 1,
};

static const struct intel_device_info intel_g33_info = {
	.gen = 3, .is_g33 = 1, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_overlay = 1,
};

static const struct intel_device_info intel_g45_info = {
	.gen = 4, .is_g4x = 1, .need_gfx_hws = 1, .num_pipes = 2,
	.has_pipe_cxsr = 1, .has_hotplug = 1,
	.has_bsd_ring = 1,
};

static const struct intel_device_info intel_gm45_info = {
	.gen = 4, .is_g4x = 1, .num_pipes = 2,
	.is_mobile = 1, .need_gfx_hws = 1, .has_fbc = 1,
	.has_pipe_cxsr = 1, .has_hotplug = 1,
	.supports_tv = 1,
	.has_bsd_ring = 1,
};

static const struct intel_device_info intel_pineview_info = {
	.gen = 3, .is_g33 = 1, .is_pineview = 1, .is_mobile = 1, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_overlay = 1,
};

static const struct intel_device_info intel_ironlake_d_info = {
	.gen = 5, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_bsd_ring = 1,
};

static const struct intel_device_info intel_ironlake_m_info = {
	.gen = 5, .is_mobile = 1, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_fbc = 1,
	.has_bsd_ring = 1,
};

static const struct intel_device_info intel_sandybridge_d_info = {
	.gen = 6, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_bsd_ring = 1,
	.has_blt_ring = 1,
	.has_llc = 1,
	.has_force_wake = 1,
};

static const struct intel_device_info intel_sandybridge_m_info = {
	.gen = 6, .is_mobile = 1, .num_pipes = 2,
	.need_gfx_hws = 1, .has_hotplug = 1,
	.has_fbc = 1,
	.has_bsd_ring = 1,
	.has_blt_ring = 1,
	.has_llc = 1,
	.has_force_wake = 1,
};

#define GEN7_FEATURES  \
	.gen = 7, .num_pipes = 3, \
	.need_gfx_hws = 1, .has_hotplug = 1, \
	.has_bsd_ring = 1, \
	.has_blt_ring = 1, \
	.has_llc = 1, \
	.has_force_wake = 1

static const struct intel_device_info intel_ivybridge_d_info = {
	GEN7_FEATURES,
	.is_ivybridge = 1,
};

static const struct intel_device_info intel_ivybridge_m_info = {
	GEN7_FEATURES,
	.is_ivybridge = 1,
	.is_mobile = 1,
	.has_fbc = 1,
};

static const struct intel_device_info intel_ivybridge_q_info = {
	GEN7_FEATURES,
	.is_ivybridge = 1,
	.num_pipes = 0, /* legal, last one wins */
};

static const struct intel_device_info intel_valleyview_m_info = {
	GEN7_FEATURES,
	.is_mobile = 1,
	.num_pipes = 2,
	.is_valleyview = 1,
	.is_valleyview_c0 = 0,
	.has_dpst = 1,
	.display_mmio_offset = VLV_DISPLAY_BASE,
	.has_llc = 0, /* legal, last one wins */
};

static const struct intel_device_info intel_valleyview_d_info = {
	GEN7_FEATURES,
	.num_pipes = 2,
	.is_valleyview = 1,
	.is_valleyview_c0 = 0,
	.display_mmio_offset = VLV_DISPLAY_BASE,
	.has_llc = 0, /* legal, last one wins */
};

static const struct intel_device_info intel_haswell_d_info = {
	GEN7_FEATURES,
	.is_haswell = 1,
	.has_ddi = 1,
	.has_fpga_dbg = 1,
	.has_vebox_ring = 1,
};

static const struct intel_device_info intel_haswell_m_info = {
	GEN7_FEATURES,
	.is_haswell = 1,
	.is_mobile = 1,
	.has_ddi = 1,
	.has_fpga_dbg = 1,
	.has_fbc = 1,
	.has_vebox_ring = 1,
};

static const struct pci_device_id pciidlist[] = {		/* aka */
	INTEL_VGA_DEVICE(0x3577, &intel_i830_info),		/* I830_M */
	INTEL_VGA_DEVICE(0x2562, &intel_845g_info),		/* 845_G */
	INTEL_VGA_DEVICE(0x3582, &intel_i85x_info),		/* I855_GM */
	INTEL_VGA_DEVICE(0x358e, &intel_i85x_info),
	INTEL_VGA_DEVICE(0x2572, &intel_i865g_info),		/* I865_G */
	INTEL_VGA_DEVICE(0x2582, &intel_i915g_info),		/* I915_G */
	INTEL_VGA_DEVICE(0x258a, &intel_i915g_info),		/* E7221_G */
	INTEL_VGA_DEVICE(0x2592, &intel_i915gm_info),		/* I915_GM */
	INTEL_VGA_DEVICE(0x2772, &intel_i945g_info),		/* I945_G */
	INTEL_VGA_DEVICE(0x27a2, &intel_i945gm_info),		/* I945_GM */
	INTEL_VGA_DEVICE(0x27ae, &intel_i945gm_info),		/* I945_GME */
	INTEL_VGA_DEVICE(0x2972, &intel_i965g_info),		/* I946_GZ */
	INTEL_VGA_DEVICE(0x2982, &intel_i965g_info),		/* G35_G */
	INTEL_VGA_DEVICE(0x2992, &intel_i965g_info),		/* I965_Q */
	INTEL_VGA_DEVICE(0x29a2, &intel_i965g_info),		/* I965_G */
	INTEL_VGA_DEVICE(0x29b2, &intel_g33_info),		/* Q35_G */
	INTEL_VGA_DEVICE(0x29c2, &intel_g33_info),		/* G33_G */
	INTEL_VGA_DEVICE(0x29d2, &intel_g33_info),		/* Q33_G */
	INTEL_VGA_DEVICE(0x2a02, &intel_i965gm_info),		/* I965_GM */
	INTEL_VGA_DEVICE(0x2a12, &intel_i965gm_info),		/* I965_GME */
	INTEL_VGA_DEVICE(0x2a42, &intel_gm45_info),		/* GM45_G */
	INTEL_VGA_DEVICE(0x2e02, &intel_g45_info),		/* IGD_E_G */
	INTEL_VGA_DEVICE(0x2e12, &intel_g45_info),		/* Q45_G */
	INTEL_VGA_DEVICE(0x2e22, &intel_g45_info),		/* G45_G */
	INTEL_VGA_DEVICE(0x2e32, &intel_g45_info),		/* G41_G */
	INTEL_VGA_DEVICE(0x2e42, &intel_g45_info),		/* B43_G */
	INTEL_VGA_DEVICE(0x2e92, &intel_g45_info),		/* B43_G.1 */
	INTEL_VGA_DEVICE(0xa001, &intel_pineview_info),
	INTEL_VGA_DEVICE(0xa011, &intel_pineview_info),
	INTEL_VGA_DEVICE(0x0042, &intel_ironlake_d_info),
	INTEL_VGA_DEVICE(0x0046, &intel_ironlake_m_info),
	INTEL_VGA_DEVICE(0x0102, &intel_sandybridge_d_info),
	INTEL_VGA_DEVICE(0x0112, &intel_sandybridge_d_info),
	INTEL_VGA_DEVICE(0x0122, &intel_sandybridge_d_info),
	INTEL_VGA_DEVICE(0x0106, &intel_sandybridge_m_info),
	INTEL_VGA_DEVICE(0x0116, &intel_sandybridge_m_info),
	INTEL_VGA_DEVICE(0x0126, &intel_sandybridge_m_info),
	INTEL_VGA_DEVICE(0x010A, &intel_sandybridge_d_info),
	INTEL_VGA_DEVICE(0x0156, &intel_ivybridge_m_info), /* GT1 mobile */
	INTEL_VGA_DEVICE(0x0166, &intel_ivybridge_m_info), /* GT2 mobile */
	INTEL_VGA_DEVICE(0x0152, &intel_ivybridge_d_info), /* GT1 desktop */
	INTEL_VGA_DEVICE(0x0162, &intel_ivybridge_d_info), /* GT2 desktop */
	INTEL_VGA_DEVICE(0x015a, &intel_ivybridge_d_info), /* GT1 server */
	INTEL_QUANTA_VGA_DEVICE(&intel_ivybridge_q_info), /* Quanta transcode */
	INTEL_VGA_DEVICE(0x016a, &intel_ivybridge_d_info), /* GT2 server */
	INTEL_VGA_DEVICE(0x0402, &intel_haswell_d_info), /* GT1 desktop */
	INTEL_VGA_DEVICE(0x0412, &intel_haswell_d_info), /* GT2 desktop */
	INTEL_VGA_DEVICE(0x0422, &intel_haswell_d_info), /* GT3 desktop */
	INTEL_VGA_DEVICE(0x040a, &intel_haswell_d_info), /* GT1 server */
	INTEL_VGA_DEVICE(0x041a, &intel_haswell_d_info), /* GT2 server */
	INTEL_VGA_DEVICE(0x042a, &intel_haswell_d_info), /* GT3 server */
	INTEL_VGA_DEVICE(0x0406, &intel_haswell_m_info), /* GT1 mobile */
	INTEL_VGA_DEVICE(0x0416, &intel_haswell_m_info), /* GT2 mobile */
	INTEL_VGA_DEVICE(0x0426, &intel_haswell_m_info), /* GT2 mobile */
	INTEL_VGA_DEVICE(0x040B, &intel_haswell_d_info), /* GT1 reserved */
	INTEL_VGA_DEVICE(0x041B, &intel_haswell_d_info), /* GT2 reserved */
	INTEL_VGA_DEVICE(0x042B, &intel_haswell_d_info), /* GT3 reserved */
	INTEL_VGA_DEVICE(0x040E, &intel_haswell_d_info), /* GT1 reserved */
	INTEL_VGA_DEVICE(0x041E, &intel_haswell_d_info), /* GT2 reserved */
	INTEL_VGA_DEVICE(0x042E, &intel_haswell_d_info), /* GT3 reserved */
	INTEL_VGA_DEVICE(0x0C02, &intel_haswell_d_info), /* SDV GT1 desktop */
	INTEL_VGA_DEVICE(0x0C12, &intel_haswell_d_info), /* SDV GT2 desktop */
	INTEL_VGA_DEVICE(0x0C22, &intel_haswell_d_info), /* SDV GT3 desktop */
	INTEL_VGA_DEVICE(0x0C0A, &intel_haswell_d_info), /* SDV GT1 server */
	INTEL_VGA_DEVICE(0x0C1A, &intel_haswell_d_info), /* SDV GT2 server */
	INTEL_VGA_DEVICE(0x0C2A, &intel_haswell_d_info), /* SDV GT3 server */
	INTEL_VGA_DEVICE(0x0C06, &intel_haswell_m_info), /* SDV GT1 mobile */
	INTEL_VGA_DEVICE(0x0C16, &intel_haswell_m_info), /* SDV GT2 mobile */
	INTEL_VGA_DEVICE(0x0C26, &intel_haswell_m_info), /* SDV GT3 mobile */
	INTEL_VGA_DEVICE(0x0C0B, &intel_haswell_d_info), /* SDV GT1 reserved */
	INTEL_VGA_DEVICE(0x0C1B, &intel_haswell_d_info), /* SDV GT2 reserved */
	INTEL_VGA_DEVICE(0x0C2B, &intel_haswell_d_info), /* SDV GT3 reserved */
	INTEL_VGA_DEVICE(0x0C0E, &intel_haswell_d_info), /* SDV GT1 reserved */
	INTEL_VGA_DEVICE(0x0C1E, &intel_haswell_d_info), /* SDV GT2 reserved */
	INTEL_VGA_DEVICE(0x0C2E, &intel_haswell_d_info), /* SDV GT3 reserved */
	INTEL_VGA_DEVICE(0x0A02, &intel_haswell_d_info), /* ULT GT1 desktop */
	INTEL_VGA_DEVICE(0x0A12, &intel_haswell_d_info), /* ULT GT2 desktop */
	INTEL_VGA_DEVICE(0x0A22, &intel_haswell_d_info), /* ULT GT3 desktop */
	INTEL_VGA_DEVICE(0x0A0A, &intel_haswell_d_info), /* ULT GT1 server */
	INTEL_VGA_DEVICE(0x0A1A, &intel_haswell_d_info), /* ULT GT2 server */
	INTEL_VGA_DEVICE(0x0A2A, &intel_haswell_d_info), /* ULT GT3 server */
	INTEL_VGA_DEVICE(0x0A06, &intel_haswell_m_info), /* ULT GT1 mobile */
	INTEL_VGA_DEVICE(0x0A16, &intel_haswell_m_info), /* ULT GT2 mobile */
	INTEL_VGA_DEVICE(0x0A26, &intel_haswell_m_info), /* ULT GT3 mobile */
	INTEL_VGA_DEVICE(0x0A0B, &intel_haswell_d_info), /* ULT GT1 reserved */
	INTEL_VGA_DEVICE(0x0A1B, &intel_haswell_d_info), /* ULT GT2 reserved */
	INTEL_VGA_DEVICE(0x0A2B, &intel_haswell_d_info), /* ULT GT3 reserved */
	INTEL_VGA_DEVICE(0x0A0E, &intel_haswell_m_info), /* ULT GT1 reserved */
	INTEL_VGA_DEVICE(0x0A1E, &intel_haswell_m_info), /* ULT GT2 reserved */
	INTEL_VGA_DEVICE(0x0A2E, &intel_haswell_m_info), /* ULT GT3 reserved */
	INTEL_VGA_DEVICE(0x0D02, &intel_haswell_d_info), /* CRW GT1 desktop */
	INTEL_VGA_DEVICE(0x0D12, &intel_haswell_d_info), /* CRW GT2 desktop */
	INTEL_VGA_DEVICE(0x0D22, &intel_haswell_d_info), /* CRW GT3 desktop */
	INTEL_VGA_DEVICE(0x0D0A, &intel_haswell_d_info), /* CRW GT1 server */
	INTEL_VGA_DEVICE(0x0D1A, &intel_haswell_d_info), /* CRW GT2 server */
	INTEL_VGA_DEVICE(0x0D2A, &intel_haswell_d_info), /* CRW GT3 server */
	INTEL_VGA_DEVICE(0x0D06, &intel_haswell_m_info), /* CRW GT1 mobile */
	INTEL_VGA_DEVICE(0x0D16, &intel_haswell_m_info), /* CRW GT2 mobile */
	INTEL_VGA_DEVICE(0x0D26, &intel_haswell_m_info), /* CRW GT3 mobile */
	INTEL_VGA_DEVICE(0x0D0B, &intel_haswell_d_info), /* CRW GT1 reserved */
	INTEL_VGA_DEVICE(0x0D1B, &intel_haswell_d_info), /* CRW GT2 reserved */
	INTEL_VGA_DEVICE(0x0D2B, &intel_haswell_d_info), /* CRW GT3 reserved */
	INTEL_VGA_DEVICE(0x0D0E, &intel_haswell_d_info), /* CRW GT1 reserved */
	INTEL_VGA_DEVICE(0x0D1E, &intel_haswell_d_info), /* CRW GT2 reserved */
	INTEL_VGA_DEVICE(0x0D2E, &intel_haswell_d_info), /* CRW GT3 reserved */
	INTEL_VGA_DEVICE(0x0f30, &intel_valleyview_m_info),
	INTEL_VGA_DEVICE(0x0f31, &intel_valleyview_m_info),
	INTEL_VGA_DEVICE(0x0f32, &intel_valleyview_m_info),
	INTEL_VGA_DEVICE(0x0f33, &intel_valleyview_m_info),
	INTEL_VGA_DEVICE(0x0157, &intel_valleyview_m_info),
	INTEL_VGA_DEVICE(0x0155, &intel_valleyview_d_info),
	{0, 0, 0}
};

#if defined(CONFIG_DRM_I915_KMS)
MODULE_DEVICE_TABLE(pci, pciidlist);
#endif

void intel_detect_pch(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct pci_dev *pch;

	/* In all current cases, num_pipes is equivalent to the PCH_NOP setting
	 * (which really amounts to a PCH but no South Display).
	 */
	if (INTEL_INFO(dev)->num_pipes == 0) {
		dev_priv->pch_type = PCH_NOP;
		return;
	}

	/*
	 * The reason to probe ISA bridge instead of Dev31:Fun0 is to
	 * make graphics device passthrough work easy for VMM, that only
	 * need to expose ISA bridge to let driver know the real hardware
	 * underneath. This is a requirement from virtualization team.
	 *
	 * In some virtualized environments (e.g. XEN), there is irrelevant
	 * ISA bridge in the system. To work reliably, we should scan trhough
	 * all the ISA bridge devices and check for the first match, instead
	 * of only checking the first one.
	 */
	pch = pci_get_class(PCI_CLASS_BRIDGE_ISA << 8, NULL);
	while (pch) {
		struct pci_dev *curr = pch;
		if (pch->vendor == PCI_VENDOR_ID_INTEL) {
			unsigned short id;
			id = pch->device & INTEL_PCH_DEVICE_ID_MASK;
			dev_priv->pch_id = id;

			if (id == INTEL_PCH_IBX_DEVICE_ID_TYPE) {
				dev_priv->pch_type = PCH_IBX;
				DRM_DEBUG_KMS("Found Ibex Peak PCH\n");
				WARN_ON(!IS_GEN5(dev));
			} else if (id == INTEL_PCH_CPT_DEVICE_ID_TYPE) {
				dev_priv->pch_type = PCH_CPT;
				DRM_DEBUG_KMS("Found CougarPoint PCH\n");
				WARN_ON(!(IS_GEN6(dev) || IS_IVYBRIDGE(dev)));
			} else if (id == INTEL_PCH_PPT_DEVICE_ID_TYPE) {
				/* PantherPoint is CPT compatible */
				dev_priv->pch_type = PCH_CPT;
				DRM_DEBUG_KMS("Found PatherPoint PCH\n");
				WARN_ON(!(IS_GEN6(dev) || IS_IVYBRIDGE(dev)));
			} else if (id == INTEL_PCH_LPT_DEVICE_ID_TYPE) {
				dev_priv->pch_type = PCH_LPT;
				DRM_DEBUG_KMS("Found LynxPoint PCH\n");
				WARN_ON(!IS_HASWELL(dev));
				WARN_ON(IS_ULT(dev));
			} else if (id == INTEL_PCH_LPT_LP_DEVICE_ID_TYPE) {
				dev_priv->pch_type = PCH_LPT;
				DRM_DEBUG_KMS("Found LynxPoint LP PCH\n");
				WARN_ON(!IS_HASWELL(dev));
				WARN_ON(!IS_ULT(dev));
			} else {
				goto check_next;
			}
			pci_dev_put(pch);
			break;
		}
check_next:
		pch = pci_get_class(PCI_CLASS_BRIDGE_ISA << 8, curr);
		pci_dev_put(curr);
	}
	if (!pch)
		DRM_DEBUG_KMS("No PCH found?\n");
}

bool i915_semaphore_is_enabled(struct drm_device *dev)
{
	if (INTEL_INFO(dev)->gen < 6)
		return 0;

	if (i915_semaphores >= 0)
		return i915_semaphores;

#ifdef CONFIG_INTEL_IOMMU
	/* Enable semaphores on SNB when IO remapping is off */
	if (INTEL_INFO(dev)->gen == 6 && intel_iommu_gfx_mapped)
		return false;
#endif

	return 1;
}

static int i915_drm_freeze(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	if (!dev_priv->pm.funcs.drm_freeze)
		return -ENODEV;

	return dev_priv->pm.funcs.drm_freeze(dev);
}

int i915_suspend(struct drm_device *dev, pm_message_t state)
{
	int error;

	if (!dev || !dev->dev_private) {
		DRM_ERROR("dev: %p\n", dev);
		DRM_ERROR("DRM not initialized, aborting suspend.\n");
		return -ENODEV;
	}

	if (state.event == PM_EVENT_PRETHAW)
		return 0;


	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	error = i915_drm_freeze(dev);
	if (error)
		return error;

	if (state.event == PM_EVENT_SUSPEND) {
		/* Shut down the device */
		pci_disable_device(dev->pdev);
		pci_set_power_state(dev->pdev, PCI_D3hot);
	}

	return 0;
}

void intel_console_resume(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private,
			     console_resume_work);
	struct drm_device *dev = dev_priv->dev;

	console_lock();
	intel_fbdev_set_suspend(dev, FBINFO_STATE_RUNNING);
	console_unlock();
}

static int i915_drm_thaw(struct drm_device *dev, bool restore_gtt)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int error;

	intel_uncore_sanitize(dev);

	if (restore_gtt && drm_core_check_feature(dev, DRIVER_MODESET)) {
		mutex_lock(&dev->struct_mutex);
		i915_gem_restore_gtt_mappings(dev);
		mutex_unlock(&dev->struct_mutex);
	}

	if (!dev_priv->pm.funcs.drm_thaw)
		return -ENODEV;

	error = dev_priv->pm.funcs.drm_thaw(dev);

	return error;
}

int i915_resume_common(struct drm_device *dev, bool restore_gtt)
{
	int ret;

	if (dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	if (pci_enable_device(dev->pdev))
		return -EIO;

	pci_set_master(dev->pdev);

	ret = i915_drm_thaw(dev, restore_gtt);
	if (ret)
		return ret;

	drm_kms_helper_poll_enable(dev);

	return 0;
}

int i915_resume(struct drm_device *dev)
{
	return i915_resume_common(dev, true);
}

/**
 * i915_reset - reset chip after a hang
 * @dev: drm device to reset
 *
 * Reset the chip.  Useful if a hang is detected. Returns zero on successful
 * reset or otherwise an error code.
 *
 * Procedure is fairly simple:
 *   - reset the chip using the reset reg
 *   - re-init context state
 *   - re-init hardware status page
 *   - re-init ring buffer
 *   - re-init interrupt state
 *   - re-init display
 */
int i915_reset(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	bool simulated;
	int ret = 0;

	if (!i915_try_reset)
		return 0;

	/* Wake up wells for handling reset if rc6 is enabled*/
	if (IS_VALLEYVIEW(dev) && dev_priv->rc6.enabled)
		vlv_force_wake_get(dev_priv, FORCEWAKE_ALL);

	dev_priv->rps.state = dev_priv->rps.enabled;
	dev_priv->rc6.state = dev_priv->rc6.enabled;

	drm_halt(dev);

	/* Wait for DRM to go idle.
	* We will try the reset even if this fails because the alternative
	* is a terminal wedge */
	ret = drm_wait_idle(dev, 5000);

	if (ret != 0)
		DRM_ERROR("Failed to halt DRM. Attempting reset anyway...\n");

	mutex_lock(&dev->struct_mutex);

	DRM_ERROR("Reset GPU (GPU Hang)\n");

	i915_gem_reset(dev);

	simulated = dev_priv->gpu_error.stop_rings != 0;

	if (!simulated && i915_gpu_reset_min_alive_period > 0 &&
		(get_seconds() - dev_priv->gpu_error.last_reset)
			< i915_gpu_reset_min_alive_period) {
		DRM_ERROR("GPU hanging too fast, declaring wedged!\n");
		ret = -ENODEV;
	} else {
		ret = intel_gpu_reset(dev);

		/* Also reset the gpu hangman. */
		if (simulated) {
			DRM_INFO("Simulated gpu hang, resetting stop_rings\n");
			dev_priv->gpu_error.stop_rings = 0;
			if (ret == -ENODEV) {
				DRM_ERROR("Reset not implemented, but ignoring "
					  "error for simulated gpu hangs\n");
				ret = 0;
			}
		} else
			dev_priv->gpu_error.last_reset = get_seconds();
	}
	if (ret) {
		DRM_ERROR("Failed to reset chip.\n");
		mutex_unlock(&dev->struct_mutex);
		goto done;
	}

	/* Ok, now get things going again... */

	/*
	 * Everything depends on having the GTT running, so we need to start
	 * there.  Fortunately we don't need to do this unless we reset the
	 * chip at a PCI level.
	 *
	 * Next we need to restore the context, but we don't use those
	 * yet either...
	 *
	 * Ring buffer needs to be re-initialized in the KMS case, or if X
	 * was running at the time of the reset (i.e. we weren't VT
	 * switched away).
	 */
	if (drm_core_check_feature(dev, DRIVER_MODESET) ||
			!dev_priv->ums.mm_suspended) {
		struct intel_ring_buffer *ring;
		int i;

		dev_priv->ums.mm_suspended = 0;

		i915_gem_init_swizzling(dev);

		/* Release the reset condition so that i915_gem_context_restore
		 * can submit a request to the ring to set the context */
		smp_mb__before_atomic_inc();
		atomic_inc(&dev_priv->gpu_error.reset_counter);

		for_each_ring(ring, dev_priv, i) {
			ring->init(ring);
			i915_gem_context_restore(dev, ring);
		}

		if (dev_priv->mm.aliasing_ppgtt) {
			ret = dev_priv->mm.aliasing_ppgtt->enable(dev);

			if (ret) {
				DRM_ERROR("Enable PPGTT returns %d\n", ret);
				i915_gem_cleanup_aliasing_ppgtt(dev);
			}
		}

		/*
		 * It would make sense to re-init all the other hw state, at
		 * least the rps/rc6/emon init done within modeset_init_hw. For
		 * some unknown reason, this blows up my ilk, so don't.
		 */

		/* Do any cleanup required for pending vblanks / flips */
		drm_clean_pending_vblanks(dev);
		intel_display_handle_reset(dev);

		drm_irq_uninstall_locked(dev, 1);
		drm_irq_install_locked(dev, 1);

		mutex_unlock(&dev->struct_mutex);

		/* rps/rc6 re-init is necessary to restore state lost
		 * after the reset and the re-install of drm irq. */
		if (INTEL_INFO(dev)->gen > 5) {
			mutex_lock(&dev->struct_mutex);
			/* Cancel delayed work for media promotion timer */
			if (IS_VALLEYVIEW(dev) && dev_priv->rc6.enabled)
				cancel_delayed_work_sync(
					&dev_priv->rps.vlv_media_timeout_work);
			intel_enable_gt_powersave(dev);
			mutex_unlock(&dev->struct_mutex);
		}

		intel_hpd_init(dev);

	} else {
		mutex_unlock(&dev->struct_mutex);
	}

done:
	/* Allow DRM to continue processing IOCTLs */
	drm_continue(dev);

	return ret;
}

static int i915_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct intel_device_info *intel_info =
		(struct intel_device_info *) ent->driver_data;

	/* Only bind to function 0 of the device. Early generations
	 * used function 1 as a placeholder for multi-head. This causes
	 * us confusion instead, especially on the systems where both
	 * functions have the same PCI-ID!
	 */
	if (PCI_FUNC(pdev->devfn))
		return -ENODEV;

	/* We've managed to ship a kms-enabled ddx that shipped with an XvMC
	 * implementation for gen3 (and only gen3) that used legacy drm maps
	 * (gasp!) to share buffers between X and the client. Hence we need to
	 * keep around the fake agp stuff for gen3, even when kms is enabled. */
	if (intel_info->gen != 3) {
		driver.driver_features &=
			~(DRIVER_USE_AGP | DRIVER_REQUIRE_AGP);
	} else if (!intel_agp_enabled) {
		DRM_ERROR("drm/i915 can't work without intel_agp module!\n");
		return -ENODEV;
	}

	return drm_get_pci_dev(pdev, ent, &driver);
}

static void
i915_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);

	drm_put_dev(dev);
}

#ifdef CONFIG_DRM_VXD_BYT

#define DRM_PSB_FILE_PAGE_OFFSET ((0x100000000ULL >> PAGE_SHIFT) * 18)
#define VXD_TTM_MMAP_OFFSET_START DRM_PSB_FILE_PAGE_OFFSET
#define VXD_TTM_MMAP_OFFSET_END (DRM_PSB_FILE_PAGE_OFFSET + 0x10000000)
#define DRM_COMMAND_VXD_BASE 0x90

static int i915_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = file_priv->minor->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (vma->vm_pgoff < VXD_TTM_MMAP_OFFSET_START ||
	    vma->vm_pgoff > VXD_TTM_MMAP_OFFSET_END) {
		return drm_gem_mmap(filp, vma);
	} else {
		if (dev_priv->psb_mmap)
			return dev_priv->psb_mmap(filp, vma);
		else
			return 0;
	}
}
#endif

static int i915_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
#ifdef CONFIG_PM_RUNTIME
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = file_priv->minor->dev;

	i915_rpm_get_callback(dev);
#endif
#ifdef CONFIG_DRM_VXD_BYT
	{
		struct drm_i915_private *dev_priv = dev->dev_private;

		if (dev_priv->vxd_release)
			ret = dev_priv->vxd_release(inode, filp);
	}
#endif
	ret |= drm_release(inode, filp);
#ifdef CONFIG_PM_RUNTIME
	i915_rpm_put_callback(dev);
#endif
	return ret;
}

static long i915_ioctl(struct file *filp,
	      unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_PM_RUNTIME
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev = file_priv->minor->dev;
#endif

#ifdef CONFIG_DRM_VXD_BYT
	unsigned int nr = DRM_IOCTL_NR(cmd);
	struct drm_i915_private *dev_priv = dev->dev_private;

	if ((nr >= DRM_COMMAND_VXD_BASE) &&
		(nr < DRM_COMMAND_VXD_BASE + 0x10)) {
		BUG_ON(!dev_priv->vxd_ioctl);
		return dev_priv->vxd_ioctl(filp, cmd, arg);
	} else
#endif
	{
		int ret;
#ifdef CONFIG_PM_RUNTIME
		i915_rpm_get_ioctl(dev);
#endif
		ret = drm_ioctl(filp, cmd, arg);
#ifdef CONFIG_PM_RUNTIME
		i915_rpm_put_ioctl(dev);
#endif
		return ret;
	}
}

static int i915_suspend_common(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);
	int error;

	if (!drm_dev || !drm_dev->dev_private) {
		dev_err(dev, "DRM not initialized, aborting suspend.\n");
		return -ENODEV;
	}

	if (drm_dev->switch_power_state == DRM_SWITCH_POWER_OFF)
		return 0;

	error = i915_drm_freeze(drm_dev);
	if (error)
		return error;

	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);

	return 0;
}

static int i915_pm_suspend(struct device *dev)
{
	int ret;
	char *envp[] = { "GSTATE=3", NULL };
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	DRM_DEBUG_PM("PM Suspend called\n");
	ret = i915_suspend_common(dev);
	if (!ret)
		kobject_uevent_env(&drm_dev->primary->kdev.kobj,
			KOBJ_CHANGE, envp);
	DRM_DEBUG_PM("PM Suspend finished\n");

	return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int i915_rpm_suspend(struct device *dev)
{
	int ret;
	char *envp[] = { "GSTATE=3", NULL };
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	DRM_DEBUG_PM("Runtime PM Suspend called\n");
	ret = i915_suspend_common(dev);
	if (!ret)
		kobject_uevent_env(&drm_dev->primary->kdev.kobj,
			KOBJ_CHANGE, envp);
	DRM_DEBUG_PM("Runtime PM Suspend finished\n");

	return ret;
}
#endif

static int i915_pm_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);
	char *envp[] = { "GSTATE=0", NULL };
	u32 ret;

	DRM_DEBUG_PM("PM Resume called\n");
	ret = i915_resume_common(drm_dev, false);
	if (!ret)
		kobject_uevent_env(&drm_dev->primary->kdev.kobj,
			KOBJ_CHANGE, envp);
	DRM_DEBUG_PM("PM Resume finished\n");

	return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int i915_rpm_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);
	char *envp[] = { "GSTATE=0", NULL };
	int ret;

	DRM_DEBUG_PM("Runtime PM Resume called\n");
	ret = i915_resume_common(drm_dev, false);
	if (!ret)
		kobject_uevent_env(&drm_dev->primary->kdev.kobj,
			KOBJ_CHANGE, envp);
	DRM_DEBUG_PM("Runtime PM Resume finished\n");

	return ret;
}
#endif

static int i915_pm_freeze(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	if (!drm_dev || !drm_dev->dev_private) {
		dev_err(dev, "DRM not initialized, aborting suspend.\n");
		return -ENODEV;
	}

	return i915_drm_freeze(drm_dev);
}

static int i915_pm_thaw(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	return i915_drm_thaw(drm_dev, true);
}

static int i915_pm_poweroff(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *drm_dev = pci_get_drvdata(pdev);

	return i915_drm_freeze(drm_dev);
}

static void i915_pm_shutdown(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_device *drm_dev = pci_get_drvdata(pdev);
	struct drm_i915_private *dev_priv = drm_dev->dev_private;
	struct drm_crtc *crtc;

	/* make sure drm stops processing new ioctls */
	drm_halt(drm_dev);

	/* wait for drm to go idle */
	if (drm_wait_idle(drm_dev, 5000))
		DRM_ERROR("Failed to halt DRM. going for shutdown anyway...\n");

	/* Not doing drm_continue as we are going for shutdown anyway */

	/* even after drm_halt exceptions can still occur, which will
	 * call i915_gem_fault. using this flag to avoid this
	 */
	mutex_lock(&drm_dev->struct_mutex);
	/* take struct_mutex to avoid sync issue with i915_gem_fault */
	dev_priv->pm.shutdown_in_progress = true;
	mutex_unlock(&drm_dev->struct_mutex);

	if (i915_is_device_suspended(drm_dev)) {
		/* Device already in suspend state */
		return;
	}

	/* display might still be active, which might cause issue
	 * as we power gate display power island during suspend
	 */
	mutex_lock(&drm_dev->mode_config.mutex);

	/* If KMS is active, we do the leavevt stuff here */
	if (drm_core_check_feature(drm_dev, DRIVER_MODESET)) {
		dev_priv->enable_hotplug_processing = false;
		/* Disable CRTCs */
		list_for_each_entry(crtc, &drm_dev->mode_config.crtc_list, head)
			dev_priv->display.crtc_disable(crtc);
	}

	mutex_unlock(&drm_dev->mode_config.mutex);

	/* device suspend work */
	i915_suspend_common(dev);
}

static const struct dev_pm_ops i915_pm_ops = {
	.suspend = i915_pm_suspend,
	.resume = i915_pm_resume,
	.freeze = i915_pm_freeze,
	.thaw = i915_pm_thaw,
	.poweroff = i915_pm_poweroff,
	.restore = i915_pm_resume,
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = i915_rpm_suspend,
	.runtime_resume = i915_rpm_resume,
#endif
};

static const struct vm_operations_struct i915_gem_vm_ops = {
	.fault = i915_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations i915_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = i915_release,
	.unlocked_ioctl = i915_ioctl,
#ifdef CONFIG_DRM_VXD_BYT
	.mmap = i915_mmap,
#else
	.mmap = drm_gem_mmap,
#endif
	.poll = drm_poll,
	.fasync = drm_fasync,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = i915_compat_ioctl,
#endif
	.llseek = noop_llseek,
};

static struct drm_driver driver = {
	/* Don't use MTRRs here; the Xserver or userspace app should
	 * deal with them for Intel hardware.
	 */
	.driver_features =
	    DRIVER_USE_AGP | DRIVER_REQUIRE_AGP | /* DRIVER_USE_MTRR |*/
	    DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED | DRIVER_GEM | DRIVER_PRIME,
	.load = i915_driver_load,
	.unload = i915_driver_unload,
	.open = i915_driver_open,
	.lastclose = i915_driver_lastclose,
	.preclose = i915_driver_preclose,
	.postclose = i915_driver_postclose,

	/* Used in place of i915_pm_ops for non-DRIVER_MODESET */
	.suspend = i915_suspend,
	.resume = i915_resume,

	.device_is_agp = i915_driver_device_is_agp,
	.master_create = i915_master_create,
	.master_destroy = i915_master_destroy,
#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = i915_debugfs_init,
	.debugfs_cleanup = i915_debugfs_cleanup,
#endif
	.gem_init_object = i915_gem_init_object,
	.gem_free_object = i915_gem_free_object,
	.gem_vm_ops = &i915_gem_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = i915_gem_prime_export,
	.gem_prime_import = i915_gem_prime_import,

	.dumb_create = i915_gem_dumb_create,
	.dumb_map_offset = i915_gem_mmap_gtt,
	.dumb_destroy = i915_gem_dumb_destroy,
	.ioctls = i915_ioctls,
	.fops = &i915_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static struct pci_driver i915_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pciidlist,
	.probe = i915_pci_probe,
	.remove = i915_pci_remove,
	.driver.pm = &i915_pm_ops,
	.shutdown = i915_pm_shutdown,
};

static int __init i915_init(void)
{
	driver.num_ioctls = i915_max_ioctl;

	/*
	 * If CONFIG_DRM_I915_KMS is set, default to KMS unless
	 * explicitly disabled with the module pararmeter.
	 *
	 * Otherwise, just follow the parameter (defaulting to off).
	 *
	 * Allow optional vga_text_mode_force boot option to override
	 * the default behavior.
	 */
#if defined(CONFIG_DRM_I915_KMS)
	if (i915_modeset != 0)
		driver.driver_features |= DRIVER_MODESET;
#endif
	if (i915_modeset == 1)
		driver.driver_features |= DRIVER_MODESET;

#ifdef CONFIG_VGA_CONSOLE
	if (vgacon_text_force() && i915_modeset == -1)
		driver.driver_features &= ~DRIVER_MODESET;
#endif

	if (!(driver.driver_features & DRIVER_MODESET))
		driver.get_vblank_timestamp = NULL;

	return drm_pci_init(&driver, &i915_pci_driver);
}

static void __exit i915_exit(void)
{
	drm_pci_exit(&driver, &i915_pci_driver);
}

device_initcall_sync(i915_init);
module_exit(i915_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");

void i915_init_watchdog(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;

	/* Based on pre-defined time out value (60ms or 30ms) calculate
	* timer count thresholds needed based on core frequency.
	*
	* For RCS.
	* The timestamp resolution changed in Gen7 and beyond to 80ns
	* for all pipes. Before that it was 640ns.*/

	int freq;

	if (INTEL_INFO(dev)->gen >= 7)
		freq = KM_TIMESTAMP_CNTS_PER_SEC_80NS;
	else
		freq = KM_TIMESTAMP_CNTS_PER_SEC_640NS;

	dev_priv->watchdog_threshold[RCS] =
		((KM_MEDIA_ENGINE_TIMEOUT_VALUE_IN_MS) *
		(freq / KM_TIMER_MILLISECOND));

	dev_priv->watchdog_threshold[VCS] =
		((KM_BSD_ENGINE_TIMEOUT_VALUE_IN_MS) *
		(freq / KM_TIMER_MILLISECOND));

	DRM_DEBUG_TDR("RCS Thresh 0x%08x  VCS Thresh 0x%08x\n",
		dev_priv->watchdog_threshold[RCS],
		dev_priv->watchdog_threshold[VCS]);
}

