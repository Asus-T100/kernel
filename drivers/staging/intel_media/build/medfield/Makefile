# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/intel_media
GFX_INCDIR=drivers/staging/mrst/pvr
DISPLAY_INCDIR=drivers/staging/mrst/drv

GRAPHICS_DIR := $(INCDIR)/graphics

# Get definitions for PVRINCS-y and PVRDEFS-y
include $(src)/../pvrdefs.mk

include_dirs := \
		$(PVRINCS-y) \
		-I$(INCDIR)/ \
		-I$(DISPLAY_INCDIR) \
		-I$(DISPLAY_INCDIR)/otm_hdmi/os/android/include \
		-I$(DISPLAY_INCDIR)/otm_hdmi/pil/include \
		-I$(INCDIR)/video/decode \
		-I$(INCDIR)/video/encode \
		-I$(INCDIR)/video/common \
		-I$(INCDIR)/common \
		-Iinclude/linux \
		-Iinclude/drm


ifeq ($(CONFIG_GPU_BURST),y)
GPU_BURST_CONFIGURED := 1
else ifeq ($(CONFIG_GPU_BURST),m)
GPU_BURST_CONFIGURED := 1
else
GPU_BURST_CONFIGURED := 0
endif

ifeq ($(GPU_BURST_CONFIGURED),1)

gfx-y += $(SYSCONFIGDIR)/gburst_interface.o
gfx-y += $(SGXDIR)/gburst_hw.o

endif

obj-$(CONFIG_GPU_BURST) += ../../graphics/gburst/

ccflags-y += $(include_dirs)

ccflags-y += $(PVRDEFS-y)

ENVDIR = ../../../mrst/pvr/services4/srvkm/env/linux
COMMONDIR = ../../../mrst/pvr/services4/srvkm/common
BRIDGEDDIR = ../../../mrst/pvr/services4/srvkm/bridged
SGXDIR = ../../../mrst/pvr/services4/srvkm/devices/sgx
FBDEVDIR = ../../../mrst/pvr/services4/3rdparty/linux_framebuffer_drm
DISPLAYDIR = ../../../mrst/drv
SYSCONFIGDIR = ../../../mrst/pvr/services4/system/intel_drm
VIDEO_COMMON_DIR = ../../video/common
DECODE_DIR = ../../video/decode
ENCODE_DIR = ../../video/encode
MEDIA_COMMON_DIR = ../../common

gfx-y += $(ENVDIR)/osfunc.o \
		$(ENVDIR)/mutils.o \
		$(ENVDIR)/mmap.o \
		$(ENVDIR)/module.o \
		$(ENVDIR)/pdump.o \
		$(ENVDIR)/proc.o \
		$(ENVDIR)/pvr_bridge_k.o \
		$(ENVDIR)/pvr_debug.o \
		$(ENVDIR)/mm.o \
		$(ENVDIR)/mutex.o \
		$(ENVDIR)/event.o \
		$(ENVDIR)/osperproc.o \
		$(ENVDIR)/pvr_drm.o

gfx-y += $(COMMONDIR)/buffer_manager.o \
		$(COMMONDIR)/devicemem.o \
		$(COMMONDIR)/deviceclass.o \
		$(COMMONDIR)/handle.o \
		$(COMMONDIR)/hash.o \
		$(COMMONDIR)/lists.o \
		$(COMMONDIR)/mem.o \
		$(COMMONDIR)/mem_debug.o \
		$(COMMONDIR)/metrics.o \
		$(COMMONDIR)/osfunc_common.o \
		$(COMMONDIR)/pdump_common.o \
		$(COMMONDIR)/perproc.o \
		$(COMMONDIR)/power.o \
		$(COMMONDIR)/pvrsrv.o \
		$(COMMONDIR)/queue.o \
		$(COMMONDIR)/ra.o \
		$(COMMONDIR)/refcount.o \
		$(COMMONDIR)/resman.o

gfx-y += $(BRIDGEDDIR)/bridged_support.o \
		$(BRIDGEDDIR)/bridged_pvr_bridge.o \
		$(BRIDGEDDIR)/sgx/bridged_sgx_bridge.o

gfx-y += $(SYSCONFIGDIR)/sysconfig.o \
		$(SYSCONFIGDIR)/sysutils.o \
		$(SYSCONFIGDIR)/sys_pvr_drm_export.o

gfx-y += $(SGXDIR)/sgxinit.o \
		$(SGXDIR)/sgxpower.o \
		$(SGXDIR)/sgxreset.o \
		$(SGXDIR)/sgxutils.o \
		$(SGXDIR)/sgxkick.o \
		$(SGXDIR)/sgxtransfer.o \
		$(SGXDIR)/mmu.o \
		$(SGXDIR)/pb.o

gfx-y += $(FBDEVDIR)/drmlfb_displayclass.o \
		$(FBDEVDIR)/drmlfb_linux.o

gfx-y += $(MEDIA_COMMON_DIR)/psb_drv.o \
		$(MEDIA_COMMON_DIR)/psb_reset.o \
		$(MEDIA_COMMON_DIR)/psb_powermgmt.o \
		$(MEDIA_COMMON_DIR)/psb_irq.o \
		$(MEDIA_COMMON_DIR)/psb_gtt.o

gfx-y += $(DISPLAYDIR)/psb_bl.o \
		$(DISPLAYDIR)/psb_dpst.o \
		$(DISPLAYDIR)/psb_fb.o \
		$(DISPLAYDIR)/psb_intel_display.o \
		$(DISPLAYDIR)/mdfld_csc.o \
		$(DISPLAYDIR)/psb_socket.o \
		$(DISPLAYDIR)/psb_pvr_glue.o \
		$(DISPLAYDIR)/psb_umevents.o \
		$(DISPLAYDIR)/mdfld_dsi_dbi.o \
		$(DISPLAYDIR)/mdfld_dsi_dpi.o \
		$(DISPLAYDIR)/mdfld_dsi_output.o \
		$(DISPLAYDIR)/mdfld_output.o \
		$(DISPLAYDIR)/auo_sc1_vid.o \
		$(DISPLAYDIR)/auo_sc1_cmd.o \
		$(DISPLAYDIR)/gi_sony_vid.o \
		$(DISPLAYDIR)/gi_sony_cmd.o \
		$(DISPLAYDIR)/tmd_6x10_vid.o \
		$(DISPLAYDIR)/h8c7_vid.o \
		$(DISPLAYDIR)/h8c7_cmd.o \
		$(DISPLAYDIR)/mdfld_dsi_pkg_sender.o \
		$(DISPLAYDIR)/mdfld_dsi_esd.o \
		$(DISPLAYDIR)/mdfld_dsi_dbi_dsr.o \
		$(DISPLAYDIR)/tc35876x_vid.o

gfx-$(CONFIG_MDFD_HDMI) += $(DISPLAYDIR)/psb_hotplug.o \
		$(DISPLAYDIR)/psb_intel_hdmi.o \

gfx-$(CONFIG_SND_INTELMID_HDMI_AUDIO) += $(DISPLAYDIR)/mdfld_hdmi_audio.o

gfx-y +=  $(VIDEO_COMMON_DIR)/psb_ttm_glue.o

gfx-$(CONFIG_MDFD_VIDEO_DECODE) += $(ENCODE_DIR)/pnw_topaz.o \
		$(ENCODE_DIR)/pnw_topazinit.o \
		$(VIDEO_COMMON_DIR)/psb_cmdbuf.o \
		$(VIDEO_COMMON_DIR)/psb_buffer.o \
		$(VIDEO_COMMON_DIR)/psb_fence.o \
		$(VIDEO_COMMON_DIR)/psb_mmu.o \
		$(DECODE_DIR)/psb_msvdx.o \
		$(DECODE_DIR)/psb_msvdxinit.o \
		$(DECODE_DIR)/psb_msvdx_fw.o \
		$(VIDEO_COMMON_DIR)/psb_ttm_glue.o \
		$(VIDEO_COMMON_DIR)/psb_ttm_fence.o \
		$(VIDEO_COMMON_DIR)/psb_ttm_fence_user.o \
		$(VIDEO_COMMON_DIR)/psb_ttm_placement_user.o

gfx-$(CONFIG_DRM_MRFLD) += $(DECODE_DIR)/psb_msvdx_ec.o

gfx-$(CONFIG_MDFLD_DSI_DPU) += $(DISPLAYDIR)/mdfld_dsi_dbi_dpu.o

gfx-$(CONFIG_MDFD_GL3) += $(MEDIA_COMMON_DIR)/mdfld_gl3.o

obj-$(CONFIG_DRM_MDFLD) += gfx.o
obj-$(CONFIG_DRM_CTP) += gfx.o

obj-$(CONFIG_DRM_MDFLD) += $(DISPLAYDIR)/otm_hdmi/
obj-$(CONFIG_DRM_CTP) += $(DISPLAYDIR)/otm_hdmi/
