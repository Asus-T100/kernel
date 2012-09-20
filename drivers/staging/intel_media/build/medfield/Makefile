# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/intel_media
GFX_INCDIR=drivers/staging/mrst/pvr
DISPLAY_INCDIR=drivers/staging/mrst/drv

include_dirs := \
		-I$(GFX_INCDIR)/include4 \
		-I$(GFX_INCDIR)/services4/include \
		-I$(GFX_INCDIR)/services4/include/env/linux \
		-I$(GFX_INCDIR)/services4/srvkm/env/linux \
		-I$(GFX_INCDIR)/services4/srvkm/include \
		-I$(GFX_INCDIR)/services4/srvkm/bridged \
		-I$(GFX_INCDIR)/services4/system/include \
		-I$(GFX_INCDIR)/services4/srvkm/hwdefs \
		-I$(GFX_INCDIR)/services4/srvkm/bridged/sgx \
		-I$(GFX_INCDIR)/services4/srvkm/devices/sgx \
		-I$(GFX_INCDIR)/services4/srvkm/common \
		-I$(GFX_INCDIR)/services4/3rdparty/linux_framebuffer_drm \
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

ccflags-y += $(include_dirs)
ccflags-y += $(ANDROID_TOOLCHAIN_FLAGS) -I$(GFX_INCDIR)/services4/system/intel_drm -DANDROID -D_linux_ -D__KERNEL__

ifeq ($(pvrdbg),1)
ccflags-y += -DPVRDEBUG
endif
ifeq ($(CONFIG_DRM_MDFLD),y)
ccflags-y += -DMEDFIELD
IS_SGX540=y
endif
ifeq ($(CONFIG_DRM_CTP),y)
ccflags-y += -DCLOVERTRAIL_PHONE
ifeq ($(CONFIG_DRM_CTP_PR1),y)
IS_SGX544=y
else
IS_SGX545=y
endif
endif

ccflags-$(IS_SGX540) += -DSGX540 -DSUPPORT_SGX540 -DSGX_CORE_REV=121
ccflags-$(IS_SGX544) += -DSGX544 -DSGX_CORE_REV=115 \
		-DSGX_FEATURE_MP=1 -DSGX_FEATURE_MP_CORE_COUNT=2 -DSGX_FEATURE_SYSTEM_CACHE=1
ccflags-$(IS_SGX545) += -DSGX545 -DSUPPORT_SGX545 -DSGX_CORE_REV=1014

ccflags-y += \
		-DLINUX \
		-DPVRSRV_MODNAME="\"pvrsrvkm\"" \
		-DPVR_BUILD_DIR="\"pc_i686_medfield_linux\"" \
		-DSERVICES4 \
		-D_XOPEN_SOURCE=600 \
		-DPVR2D_VALIDATE_INPUT_PARAMS \
		-DDISPLAY_CONTROLLER=mrstlfb \
		-UDEBUG_LOG_PATH_TRUNCATE \
		-DSUPPORT_LIBDRM_LITE \
		-DOPK_FALLBACK="" \
		-DSUPPORT_ANDROID_PLATFORM \
		-DPROFILE_COMM \
		-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE\
		-DSUPPORT_ANDROID_PLATFORM \
		-DSUPPORT_GRAPHICS_HAL \
		-DPVR_SECURE_FD_EXPORT \
		-DSUPPORT_SRVINIT \
		-DSUPPORT_SGX \
		-DSUPPORT_PERCONTEXT_PB \
		-DSUPPORT_LINUX_X86_WRITECOMBINE \
		-DTRANSFER_QUEUE \
		-DSUPPORT_DRI_DRM \
		-DSUPPORT_DRI_DRM_EXT \
		-DSUPPORT_DRI_DRM_NO_DROPMASTER \
		-DSYS_USING_INTERRUPTS \
		-DSUPPORT_HW_RECOVERY \
		-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
		-DPVR_SECURE_HANDLES \
		-DLDM_PCI \
		-DUSE_PTHREADS \
		-DSUPPORT_SGX_EVENT_OBJECT \
		-DSUPPORT_SGX_HWPERF \
		-DSUPPORT_LINUX_X86_PAT \
		-DPVR_PROC_USE_SEQ_FILE \
		-DSUPPORT_CPU_CACHED_BUFFERS \
		-DDRM_PVR_RESERVED_INTEL_ORDER \
		-DDRM_PVR_USE_INTEL_FB \
		-DSUPPORT_MEMINFO_IDS \
		-DSUPPORT_SGX_NEW_STATUS_VALS \
		-DSUPPORT_LARGE_GENERAL_HEAP \
		-DSUPPORT_DC_CMDCOMPLETE_WHEN_NO_LONGER_DISPLAYED\
		-DSUPPORT_CUSTOM_SWAP_OPERATIONS \
		-DPVRSRV_NEED_PVR_DPF \
		-DPVRSRV_NEW_PVR_DPF  \
		-DPVRSRV_NEED_PVR_ASSERT \
		-DPVR_LINUX_MEM_AREA_POOL_ALLOW_SHRINK

#		-DSUPPORT_PDUMP_MULTI_PROCESS \

ifneq ($(IS_SGX545),y)
ccflags-y += -DSUPPORT_SGX_LOW_LATENCY_SCHEDULING
endif

ccflags-$(CONFIG_DRM_MID_RELEASE) += -DBUILD="\"release\"" -DPVR_BUILD_TYPE="\"release\"" -DRELEASE
ccflags-$(CONFIG_DRM_MID_DEBUG) += -DBUILD="\"debug\"" -DPVR_BUILD_TYPE="\"debug\"" -DDEBUG -DDEBUG_LINUX_MEM_AREAS -DDEBUG_LINUX_MEMORY_ALLOCATIONS -DDEBUG_LINUX_MMAP_AREAS -DDEBUG_BRIDGE_KM
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI
ccflags-$(CONFIG_MDFD_GL3) += -DSUPPORT_EXTERNAL_SYSTEM_CACHE -DCONFIG_MDFD_GL3

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
		$(MEDIA_COMMON_DIR)/psb_gtt.o \

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
		$(DISPLAYDIR)/mdfld_dsi_dbi_dsr.o

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
