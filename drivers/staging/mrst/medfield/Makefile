# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrst

include_dirs := \
	-I$(INCDIR)/pvr/include4 \
        -I$(INCDIR)/pvr/services4/include \
        -I$(INCDIR)/pvr/services4/include/env/linux \
        -I$(INCDIR)/pvr/services4/srvkm/env/linux \
        -I$(INCDIR)/pvr/services4/srvkm/include \
        -I$(INCDIR)/pvr/services4/srvkm/bridged \
	-I$(INCDIR)/pvr/services4/system/include \
        -I$(INCDIR)/pvr/services4/srvkm/hwdefs \
        -I$(INCDIR)/pvr/services4/srvkm/bridged/sgx \
        -I$(INCDIR)/pvr/services4/srvkm/devices/sgx \
        -I$(INCDIR)/pvr/services4/srvkm/common \
        -I$(INCDIR)/pvr/services4/3rdparty/linux_framebuffer_drm \
        -I$(INCDIR)/ \
        -I$(INCDIR)/drv \
        -I$(INCDIR)/bc_video \
        -I$(INCDIR)/imgv \
        -I$(INCDIR)/bc_video \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += $(include_dirs)
ccflags-y += $(ANDROID_TOOLCHAIN_FLAGS) -I$(INCDIR)/pvr/services4/system/intel_drm -DANDROID -D_linux_ -D__KERNEL__

ifeq ($(pvrdbg),1)
ccflags-y += -DPVRDEBUG
endif
ifeq ($(CONFIG_DRM_MDFLD),y)
ccflags-y += -DMEDFIELD
IS_SGX540=y
endif
ifeq ($(CONFIG_DRM_CTP),y)
ccflags-y += -DCLOVERTRAIL_PHONE
IS_SGX545=y
endif

ccflags-$(IS_SGX540) += -DSGX540 -DSUPPORT_SGX540 -DSGX_CORE_REV=121
ccflags-$(IS_SGX544) += -DSGX544 -DSGX_CORE_REV=104 \
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
    -DSUPPORT_SGX_NEW_STATUS_VALS

#    -DSUPPORT_PDUMP_MULTI_PROCESS \

ifneq ($(IS_SGX545),y)
ccflags-y += -DSUPPORT_SGX_LOW_LATENCY_SCHEDULING
endif

ccflags-$(CONFIG_DRM_MID_RELEASE) += -DBUILD="\"release\"" -DPVR_BUILD_TYPE="\"release\"" -DRELEASE
ccflags-$(CONFIG_DRM_MID_DEBUG) += -DBUILD="\"debug\"" -DPVR_BUILD_TYPE="\"debug\"" -DDEBUG -DDEBUG_LINUX_MEM_AREAS -DDEBUG_LINUX_MEMORY_ALLOCATIONS -DDEBUG_LINUX_MMAP_AREAS -DDEBUG_BRIDGE_KM
ccflags-$(CONFIG_PCI_MSI) += -DCONFIG_PCI_MSI
ccflags-$(CONFIG_MDFD_GL3) += -DSUPPORT_EXTERNAL_SYSTEM_CACHE -DCONFIG_MDFD_GL3

	
ENVDIR = ../pvr/services4/srvkm/env/linux
COMMONDIR = ../pvr/services4/srvkm/common
BRIDGEDDIR = ../pvr/services4/srvkm/bridged
SGXDIR = ../pvr/services4/srvkm/devices/sgx
FBDEVDIR = ../pvr/services4/3rdparty/linux_framebuffer_drm
DRMDRVDIR = ../drv
SYSCONFIGDIR = ../pvr/services4/system/intel_drm
IMGVDIR = ../imgv
BUFFER_CLASS_DIR = ../bc_video

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
	  $(COMMONDIR)/metrics.o \
	  $(COMMONDIR)/pvrsrv.o \
	  $(COMMONDIR)/queue.o \
	  $(COMMONDIR)/ra.o \
	  $(COMMONDIR)/resman.o \
	  $(COMMONDIR)/power.o \
	  $(COMMONDIR)/mem.o \
	  $(COMMONDIR)/pdump_common.o \
	  $(COMMONDIR)/perproc.o \
	  $(COMMONDIR)/lists.o \
	  $(COMMONDIR)/mem_debug.o \
	  $(COMMONDIR)/osfunc_common.o

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

gfx-y += $(DRMDRVDIR)/psb_bl.o \
	  $(DRMDRVDIR)/psb_dpst.o \
	  $(DRMDRVDIR)/psb_drv.o \
	  $(DRMDRVDIR)/psb_fb.o \
	  $(DRMDRVDIR)/psb_gtt.o \
	  $(DRMDRVDIR)/psb_hotplug.o \
	  $(DRMDRVDIR)/psb_intel_bios.o \
	  $(DRMDRVDIR)/psb_intel_opregion.o \
	  $(DRMDRVDIR)/psb_intel_display.o \
	  $(DRMDRVDIR)/psb_intel_i2c.o \
	  $(DRMDRVDIR)/psb_intel_lvds.o \
	  $(DRMDRVDIR)/psb_intel_modes.o \
	  $(DRMDRVDIR)/psb_intel_sdvo.o \
	  $(DRMDRVDIR)/mdfld_hdmi_audio.o \
	  $(DRMDRVDIR)/mdfld_msic.o \
	  $(DRMDRVDIR)/mdfld_csc.o \
	  $(DRMDRVDIR)/mdfld_intel_hdcp.o \
	  $(DRMDRVDIR)/psb_intel_hdmi.o \
	  $(DRMDRVDIR)/psb_intel_hdmi_i2c.o \
	  $(DRMDRVDIR)/psb_reset.o \
	  $(DRMDRVDIR)/psb_schedule.o \
	  $(DRMDRVDIR)/psb_sgx.o \
	  $(DRMDRVDIR)/psb_socket.o \
	  $(DRMDRVDIR)/psb_pvr_glue.o \
	  $(DRMDRVDIR)/psb_umevents.o \
	  $(DRMDRVDIR)/psb_intel_dsi.o \
	  $(DRMDRVDIR)/psb_intel_dsi_aava.o \
	  $(DRMDRVDIR)/mdfld_dsi_dbi.o \
	  $(DRMDRVDIR)/mdfld_dsi_dpi.o \
	  $(DRMDRVDIR)/mdfld_dsi_output.o \
	  $(DRMDRVDIR)/mdfld_output.o \
	  $(DRMDRVDIR)/tpo_cmd.o \
	  $(DRMDRVDIR)/tpo_vid.o \
	  $(DRMDRVDIR)/tmd_vid.o \
	  $(DRMDRVDIR)/tmd_6x10_vid.o \
	  $(DRMDRVDIR)/pyr_cmd.o \
	  $(DRMDRVDIR)/mdfld_dsi_pkg_sender.o \
	  $(DRMDRVDIR)/psb_powermgmt.o \
	  $(DRMDRVDIR)/psb_irq.o

gfx-y +=  $(IMGVDIR)/psb_ttm_glue.o 

gfx-$(CONFIG_MDFD_VIDEO_DECODE) += $(IMGVDIR)/lnc_topaz.o \
	  $(IMGVDIR)/topaz_power.o \
	  $(IMGVDIR)/lnc_topazinit.o \
	  $(IMGVDIR)/pnw_topaz.o \
	  $(IMGVDIR)/pnw_topazinit.o \
	  $(IMGVDIR)/psb_buffer.o \
	  $(IMGVDIR)/psb_fence.o \
	  $(IMGVDIR)/psb_mmu.o \
	  $(IMGVDIR)/psb_msvdx.o \
	  $(IMGVDIR)/msvdx_power.o \
	  $(IMGVDIR)/psb_msvdxinit.o \
	  $(IMGVDIR)/psb_ttm_glue.o \
	  $(IMGVDIR)/psb_ttm_fence.o \
	  $(IMGVDIR)/psb_ttm_fence_user.o \
	  $(IMGVDIR)/psb_ttm_placement_user.o 
		
gfx-$(CONFIG_MDFD_VIDEO_DECODE) += $(BUFFER_CLASS_DIR)/bufferclass_video.o \
	$(BUFFER_CLASS_DIR)/bufferclass_video_linux.o

gfx-$(CONFIG_MDFLD_DSI_DPU) += $(DRMDRVDIR)/mdfld_dsi_dbi_dpu.o

gfx-$(CONFIG_MDFD_GL3) += $(DRMDRVDIR)/mdfld_gl3.o

obj-$(CONFIG_DRM_MDFLD) += gfx.o
obj-$(CONFIG_DRM_CTP) += gfx.o
