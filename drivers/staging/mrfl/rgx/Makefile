# Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

RGXDIR=drivers/staging/mrfl/rgx
DISPINC=drivers/staging/mrfl/drv

ccflags-y += \
	-I$(RGXDIR)/include \
	-I$(RGXDIR)/generated/rgxcmp_bridge \
	-I$(RGXDIR)/generated/dmm_bridge \
	-I$(RGXDIR)/generated/pdumpcmm_bridge \
	-I$(RGXDIR)/generated/dc_bridge \
	-I$(RGXDIR)/generated/cachegeneric_bridge \
	-I$(RGXDIR)/generated/hostportio_bridge \
	-I$(RGXDIR)/generated/rgxtq_bridge \
	-I$(RGXDIR)/generated/mm_bridge \
	-I$(RGXDIR)/generated/cmm_bridge \
	-I$(RGXDIR)/generated/rgxpdump_bridge \
	-I$(RGXDIR)/generated/smm_bridge \
	-I$(RGXDIR)/generated/pdump_bridge \
	-I$(RGXDIR)/generated/syncsexport_bridge \
	-I$(RGXDIR)/generated/sync_bridge \
	-I$(RGXDIR)/generated/debugmisc_bridge \
	-I$(RGXDIR)/generated/syncexport_bridge \
	-I$(RGXDIR)/generated/dsync_bridge \
	-I$(RGXDIR)/generated/rgxinit_bridge \
	-I$(RGXDIR)/generated/pdumpmm_bridge \
	-I$(RGXDIR)/generated/rgxta3d_bridge \
	-I$(RGXDIR)/generated/rgxccb_bridge \
	-I$(RGXDIR)/generated/srvcore_bridge \
	-I$(RGXDIR)/generated/breakpoint_bridge \
	-I$(RGXDIR)/generated/pvrtl_bridge \
	-I$(RGXDIR)/generated/rgxhwperf_bridge \
	-I$(RGXDIR)/tools/intern/debug/include \
	-I$(RGXDIR)/tools/intern/debug/dbgdriv/common \
	-I$(RGXDIR)/hwdefs \
	-I$(RGXDIR)/hwdefs/km \
	-I$(RGXDIR)/hwdefs/km/cores \
	-I$(RGXDIR)/services/include \
	-I$(RGXDIR)/services/include/shared \
	-I$(RGXDIR)/services/include/env/linux \
	-I$(RGXDIR)/services/3rdparty/intel_drm \
	-I$(RGXDIR)/services/3rdparty/bufferclass_video \
	-I$(RGXDIR)/services/system/include \
	-I$(RGXDIR)/services/shared/devices/rgx \
	-I$(RGXDIR)/services/shared/include \
	-I$(RGXDIR)/services/server/devices/rgx \
	-I$(RGXDIR)/services/server/env/linux \
	-I$(RGXDIR)/services/server/include \
	-I$(RGXDIR)/services/system/rgx_nohw \
	-I$(RGXDIR)/../interface \
	-I$(DISPINC)/ospm \
	-I$(DISPINC)/ \
	-Iinclude/linux \
	-Iinclude/drm \
	-Iarch/arm/include


ccflags-y += \
	-DANDROID \
	-DLINUX -D__linux__ -D__KERNEL__\
	-DPVRSRV_MODNAME="\"pvrsrvkm\"" \
	-DPVR_BUILD_DIR="\"pc_android\"" \
	-DPVR_LDM_DRIVER_REGISTRATION_NAME="\"pvrsrvkm\"" \
	-DSUPPORT_RGX=1 \
	-DPVR_SECURE_HANDLES \
	-DRGX_BVNC_CORE_KM_HEADER="\"cores/rgxcore_km_1.14.4.4.h\"" \
	-DDISPLAY_CONTROLLER=pvrlfb \
	-DSUPPORT_RENDER_TARGET_ARRAYS \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DSUPPORT_LINUX_X86_PAT \
	-DSYS_USING_INTERRUPTS \
	-DLDM_DEVICE_CLASS \
	-DSUPPORT_DRM \
	-DSUPPORT_DRM_EXT \
	-DBC_DISCONTIG_BUFFERS \
	-DCACHEFLUSH_TYPE=CACHEFLUSH_X86 \
	-DSUPPORT_INSECURE_EXPORT \
	-DSUPPORT_SECURE_EXPORT \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DRGX_FEATURE_BYPASS_SLC_COMBINER \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_RGXFW_LOG \
	-DLMA \
	-DTC_MEMORY_CONFIG=TC_MEMORY_LOCAL

# Skip the following dependencies on Merrifield Virtual Platform
ifneq ($(CONFIG_BOARD_MRFLD_VP),y)
ccflags-y += \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES
endif

# for PDUMP
ccflags-$(CONFIG_PDUMP) += -DPDUMP -DSUPPORT_DBGDRV_EVENT_OBJECTS

# for no-hardware
#ccflags-y += -DNO_HARDWARE

# Either BUILD is debug or release. It can NOT be both.
ccflags-$(CONFIG_DRM_HANDSET_RELEASE) += -DBUILD="\"release\"" -DPVR_BUILD_TYPE="\"release\"" -DRELEASE
ccflags-$(CONFIG_DRM_HANDSET_DEBUG) += -DBUILD="\"debug\"" -DPVR_BUILD_TYPE="\"debug\"" -DDEBUG

ccflags-$(CONFIG_DRM_HANDSET_DEBUG) += -DDEBUG_LINUX_MEM_AREAS -DDEBUG_LINUX_MEMORY_ALLOCATIONS -DDEBUG_LINUX_MMAP_AREAS
ccflags-$(CONFIG_DRM_HANDSET_DEBUG) += -DRGXFW_ALIGNCHECKS

# The following is an option, it can be enabled for more debug information
# ccflags-$(CONFIG_DRM_HANDSET_DEBUG) += -DDEBUG_BRIDGE_KM -DDEBUG_TRACE_BRIDGE_KM -DDEBUG_BRIDGE_KM_DISPATCH_TABLE

# pvrsrvkm
merrifield_gfx-y += \
	services/server/common/pmr.o \
	services/server/common/devicemem_server.o \
	services/server/common/devicemem_heapcfg.o \
	services/server/common/dc_server.o \
	services/server/common/connection_server.o \
	services/server/common/pvrsrv.o \
	services/server/common/sync_server.o \
	services/server/common/resman.o \
	services/server/common/physmem_lma.o \
	services/server/common/power.o \
	services/server/common/physheap.o \
	services/server/common/cache_generic.o \
	services/server/common/scp.o \
	services/server/common/mmu_common.o \
	services/server/common/srvcore.o \
	services/server/common/lists.o \
	services/server/common/handle.o \
	services/server/common/physmem.o \
	services/server/env/linux/allocmem.o \
	services/server/env/linux/mm.o \
	services/server/env/linux/physmem_osmem_linux.o \
	services/server/env/linux/osfunc_x86.o \
	services/server/env/linux/mutils.o \
	services/server/env/linux/proc.o \
	services/server/env/linux/mutex.o \
	services/server/env/linux/osfunc.o \
	services/server/env/linux/event.o \
	services/server/env/linux/pvr_bridge_k.o \
	services/server/env/linux/pvr_debug.o \
	services/server/env/linux/osconnection_server.o \
	services/server/env/linux/devicemem_mmap_stub.o \
	services/server/env/linux/module.o \
	services/server/env/linux/mmap.o \
	services/server/env/linux/lock.o \
	services/server/env/linux/ossecure_export.o \
	services/server/env/linux/pdump.o \
	services/shared/common/dllist.o \
	services/shared/common/devicemem.o \
	services/shared/common/hash.o \
	services/shared/common/devicemem_utils.o \
	services/shared/common/sync.o \
	services/shared/common/ra.o \
	services/shared/devices/rgx/rgx_compat_bvnc.o \
	services/server/common/tlintern.o \
	services/server/common/tlstream.o \
	services/server/common/tltest.o \
	services/server/common/tlserver.o

#	services/server/env/linux/physmem_ion.o

# for PDUMP
merrifield_gfx-$(CONFIG_PDUMP) += \
	services/server/common/pdump_common.o \
	services/server/common/pdump_mmu.o \
	services/server/common/pdump_physmem.o \
	tools/intern/debug/dbgdriv/linux/main.o \
	tools/intern/debug/dbgdriv/linux/hostfunc.o \
	tools/intern/debug/dbgdriv/common/ioctl.o \
	tools/intern/debug/dbgdriv/common/dbgdriv.o \
	tools/intern/debug/dbgdriv/common/hotkey.o \
	tools/intern/debug/dbgdriv/common/handle.o \
	services/server/devices/rgx/rgxpdump.o \
	services/shared/common/devicemem_pdump.o

# For SUPPORT_RGX
merrifield_gfx-y += \
	services/server/devices/rgx/rgxinit.o \
	services/server/devices/rgx/rgxmmuinit.o \
	services/server/devices/rgx/rgxtransfer.o \
	services/server/devices/rgx/rgxfwutils.o \
	services/server/devices/rgx/rgxcompute.o \
	services/server/devices/rgx/rgxutils.o \
	services/server/devices/rgx/rgxpower.o \
	services/server/devices/rgx/rgxta3d.o \
	services/server/devices/rgx/debugmisc_server.o \
	services/server/devices/rgx/rgxmem.o \
	services/server/devices/rgx/rgxccb.o \
	services/server/devices/rgx/rgxdebug.o \
	services/server/devices/rgx/rgxbreakpoint.o

merrifield_gfx-y += \
	generated/cmm_bridge/server_cmm_bridge.o \
	generated/dc_bridge/server_dc_bridge.o \
	generated/pdump_bridge/server_pdump_bridge.o \
	generated/syncexport_bridge/server_syncexport_bridge.o \
	generated/srvcore_bridge/server_srvcore_bridge.o \
	generated/dsync_bridge/client_sync_bridge.o \
	generated/rgxinit_bridge/server_rgxinit_bridge.o \
	generated/breakpoint_bridge/server_breakpoint_bridge.o \
	generated/pdumpmm_bridge/server_pdumpmm_bridge.o \
	generated/rgxcmp_bridge/server_rgxcmp_bridge.o \
	generated/debugmisc_bridge/server_debugmisc_bridge.o \
	generated/rgxccb_bridge/server_rgxccb_bridge.o \
	generated/sync_bridge/server_sync_bridge.o \
	generated/rgxtq_bridge/server_rgxtq_bridge.o \
	generated/dmm_bridge/client_mm_bridge.o \
	generated/dpdumpmm_bridge/client_pdumpmm_bridge.o \
	generated/rgxta3d_bridge/server_rgxta3d_bridge.o \
	generated/cachegeneric_bridge/server_cachegeneric_bridge.o \
	generated/pdumpcmm_bridge/server_pdumpcmm_bridge.o \
	generated/rgxpdump_bridge/server_rgxpdump_bridge.o \
	generated/pvrtl_bridge/server_pvrtl_bridge.o \
	generated/rgxhwperf_bridge/server_rgxhwperf_bridge.o

# for SUPPORT_SECURE_EXPORT
merrifield_gfx-y += \
	generated/mm_bridge/server_mm_bridge.o \
	generated/smm_bridge/server_smm_bridge.o \
	generated/syncsexport_bridge/server_syncsexport_bridge.o
ccflags-y += \
	-I$(RGXDIR)/generated/smm_bridge

# buffer class video
merrifield_gfx-y += \
	services/3rdparty/bufferclass_video/bufferclass_video_linux.o \
	services/3rdparty/bufferclass_video/bufferclass_video.o

# display class
merrifield_gfx-y += \
	services/3rdparty/intel_drm/sysconfig.o \
	services/3rdparty/intel_drm/pvr_drm.o \
	services/3rdparty/intel_drm/msvdx_power.o \
	services/3rdparty/intel_drm/topaz_power.o \
	services/3rdparty/intel_drm/dc_mrfld.o \
	services/system/common/env/linux/pci_support.o

obj-$(CONFIG_DRM_MRFLD) += merrifield_gfx.o

