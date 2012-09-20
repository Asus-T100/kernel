# Copyright	2010 Imagination Technologies Limited. All rights reserved.
#
# No part of this software, either material or conceptual may be
# copied or distributed, transmitted, transcribed, stored in a
# retrieval system or translated into any human or computer
# language in any form by any means, electronic, mechanical,
# manual or other-wise, or disclosed to third parties without
# the express written permission of: Imagination Technologies
# Limited, HomePark Industrial Estate, Kings Langley,
# Hertfordshire, WD4 8LZ, UK
#

pvrsrvkm-y += \
	services/server/env/linux/event.o \
	services/server/env/linux/mm.o \
	services/server/env/linux/mmap.o \
	services/server/env/linux/module.o \
	services/server/env/linux/mutex.o \
	services/server/env/linux/mutils.o \
	services/server/env/linux/devicemem_mmap_stub.o \
	services/server/env/linux/osfunc.o \
	services/server/env/linux/allocmem.o \
	services/server/env/linux/osconnection_server.o \
	services/server/env/linux/pdump.o \
	services/server/env/linux/physmem_osmem_linux.o \
	services/server/env/linux/proc.o \
	services/server/env/linux/pvr_bridge_k.o \
	services/server/env/linux/pvr_debug.o \
	services/server/common/devicemem_heapcfg.o \
	services/shared/common/devicemem.o \
	services/shared/common/devicemem_utils.o \
	services/shared/common/hash.o \
	services/shared/common/ra.o \
	services/shared/common/sync.o \
	services/shared/common/dllist.o \
	services/server/common/devicemem_server.o \
	services/server/common/dc_server.o \
	services/server/common/handle.o \
	services/server/common/lists.o \
	services/server/common/mem_debug.o \
	services/server/common/mmu_common.o \
	services/server/common/connection_server.o \
	services/server/common/physheap.o \
	services/server/common/physmem.o \
	services/server/common/physmem_lma.o \
	services/server/common/pmr.o \
	services/server/common/power.o \
	services/server/common/pvrsrv.o \
	services/server/common/scp.o \
	services/server/common/resman.o \
	services/server/common/srvcore.o \
	services/server/common/sync_server.o

pvrsrvkm-$(CONFIG_X86) += services/server/env/linux/osfunc_x86.o

ifneq ($(W),1)
CFLAGS_mm.o := -Werror
CFLAGS_mmap.o := -Werror
CFLAGS_module.o := -Werror
CFLAGS_mutex.o := -Werror
CFLAGS_mutils.o := -Werror
CFLAGS_devicemem_mmap_stub.o := -Werror
CFLAGS_osfunc_x86.o := -Werror
CFLAGS_osconnection_server.o := -Werror
CFLAGS_pdump.o := -Werror
CFLAGS_proc.o := -Werror
CFLAGS_pvr_bridge_k.o := -Werror
CFLAGS_devicemem_heapcfg.o := -Werror
CFLAGS_devicemem.o := -Werror
CFLAGS_devicemem_utils.o = -Werror
CFLAGS_devicemem_pdump = -Werror
CFLAGS_hash.o := -Werror
CFLAGS_ra.o := -Werror
CFLAGS_sync.o := -Werror
CFLAGS_devicemem_server.o := -Werror
CFLAGS_dc_server.o := -Werror
CFLAGS_handle.o := -Werror
CFLAGS_lists.o := -Werror
CFLAGS_mem_debug.o := -Werror
CFLAGS_mmu_common.o := -Werror
CFLAGS_connection_server.o := -Werror
CFLAGS_physmem.o := -Werror
CFLAGS_physmem_lma.o := -Werror
CFLAGS_physmem_pmmif.o := -Werror
CFLAGS_pmr.o := -Werror
CFLAGS_power.o := -Werror
CFLAGS_pvrsrv.o := -Werror
CFLAGS_scp.o := -Werror
CFLAGS_resman.o := -Werror
#CFLAGS_srvcore.o := -Werror
CFLAGS_sync_server.o := -Werror
endif

ifeq ($(PDUMP),1)
pvrsrvkm-y += \
	services/server/common/pdump_common.o \
	services/server/common/pdump_mmu.o \
	services/server/common/pdump_physmem.o \
	services/shared/common/devicemem_pdump.o

ifneq ($(W),1)
CFLAGS_pdump_common.o := -Werror
CFLAGS_pdump_mmu.o := -Werror
CFLAGS_pdump_physmem.o := -Werror
CFLAGS_devicemem_pdump.o := -Werror
endif
endif


# For SUPPORT_RGX

pvrsrvkm-y += \
	services/server/devices/rgx/rgxinit.o \
	services/server/devices/rgx/rgxdebug.o \
	services/server/devices/rgx/rgxmem.o \
	services/server/devices/rgx/rgxta3d.o \
	services/server/devices/rgx/rgxcompute.o \
	services/server/devices/rgx/rgxccb.o \
	services/server/devices/rgx/rgxmmuinit.o \
	services/server/devices/rgx/rgxpower.o \
	services/server/devices/rgx/rgxtransfer.o \
	services/server/devices/rgx/rgxutils.o \
	services/server/devices/rgx/rgxfwutils.o \
	services/server/devices/rgx/hostportio_server.o \
	services/server/devices/rgx/rgxbreakpoint.o \
	services/server/devices/rgx/debugmisc_server.o

ifneq ($(W),1)
CFLAGS_rgxinit.o := -Werror
CFLAGS_rgxdebug.o := -Werror
CFLAGS_rgxmem.o := -Werror
CFLAGS_rgxta3d.o := -Werror
CFLAGS_rgxcompute.o := -Werror
CFLAGS_rgxccb.o := -Werror
CFLAGS_rgxmmuinit.o := -Werror
CFLAGS_rgxpower.o := -Werror
CFLAGS_rgxsharedpb.o := -Werror
CFLAGS_rgxtransfer.o := -Werror
CFLAGS_rgxutils.o := -Werror
CFLAGS_rgxfwutils.o := -Werror
CFLAGS_hostportio_server.o := -Werror
CFLAGS_rgxbreakpoint.o := -Werror
CFLAGS_debugmisc_server.o := -Werror
endif

ifeq ($(PDUMP),1)
pvrsrvkm-y += \
	services/server/devices/rgx/rgxpdump.o

ifneq ($(W),1)
CFLAGS_rgxpdump.o := -Werror
endif
endif

# Bridge headers and source files
ccflags-y += \
	-I$(TOP)/generated/dc_bridge \
	-I$(TOP)/generated/dmm_bridge \
	-I$(TOP)/generated/mm_bridge \
	-I$(TOP)/generated/cmm_bridge \
	-I$(TOP)/generated/pdumpmm_bridge \
	-I$(TOP)/generated/pdumpcmm_bridge \
	-I$(TOP)/generated/pdump_bridge \
	-I$(TOP)/generated/rgxtq_bridge \
	-I$(TOP)/generated/rgxinit_bridge \
	-I$(TOP)/generated/rgxta3d_bridge \
	-I$(TOP)/generated/rgxcmp_bridge \
	-I$(TOP)/generated/rgxccb_bridge \
	-I$(TOP)/generated/srvcore_bridge \
	-I$(TOP)/generated/dsync_bridge \
	-I$(TOP)/generated/sync_bridge \
	-I$(TOP)/generated/hostportio_bridge \
	-I$(TOP)/generated/breakpoint_bridge \
	-I$(TOP)/generated/debugmisc_bridge \
	-I$(TOP)/generated/rgxpdump_bridge \
	-I$(OUT)/target/intermediates/bc_bridge \
	-I$(OUT)/target/intermediates/dc_bridge \
	-I$(OUT)/target/intermediates/dmm_bridge \
	-I$(OUT)/target/intermediates/mm_bridge \
	-I$(OUT)/target/intermediates/cmm_bridge \
	-I$(OUT)/target/intermediates/pdumpmm_bridge \
	-I$(OUT)/target/intermediates/dpdumpmm_bridge \
	-I$(OUT)/target/intermediates/pdumpcmm_bridge \
	-I$(OUT)/target/intermediates/pdump_bridge \
	-I$(OUT)/target/intermediates/rgxtq_bridge \
	-I$(OUT)/target/intermediates/rgxinit_bridge \
	-I$(OUT)/target/intermediates/rgxta3d_bridge \
	-I$(OUT)/target/intermediates/rgxcmp_bridge \
	-I$(OUT)/target/intermediates/rgxccb_bridge \
	-I$(OUT)/target/intermediates/srvcore_bridge \
	-I$(OUT)/target/intermediates/dsync_bridge \
	-I$(OUT)/target/intermediates/sync_bridge \
	-I$(OUT)/target/intermediates/hostportio_bridge \
	-I$(OUT)/target/intermediates/breakpoint_bridge \
	-I$(OUT)/target/intermediates/debugmisc_bridge \
	-I$(OUT)/target/intermediates/rgxpdump_bridge

ifeq ($(CACHEFLUSH_TYPE),CACHEFLUSH_GENERIC)
ccflags-y += \
	-I$(TOP)/generated/cachegeneric_bridge \
	-I$(OUT)/target/intermediates/cachegeneric_bridge
endif

ifeq ($(SUPPORT_SECURE_EXPORT),1)
ccflags-y += \
	-I$(TOP)/generated/smm_bridge \
	-I$(OUT)/target/intermediates/smm_bridge \
	-I$(TOP)/generated/syncsexport_bridge \
	-I$(OUT)/target/intermediates/syncsexport_bridge
else
ccflags-y += \
	-I$(TOP)/generated/syncexport_bridge \
	-I$(OUT)/target/intermediates/syncexport_bridge
endif

ifeq ($(SUPPORT_PMMIF),1)
ccflags-y += \
	-I$(TOP)/generated/pmmif_bridge \
	-I$(OUT)/target/intermediates/pmmif_bridge
endif

pvrsrvkm-y += \
	generated/dc_bridge/server_dc_bridge.o \
	generated/mm_bridge/server_mm_bridge.o \
	generated/dmm_bridge/client_mm_bridge.o \
	generated/pdumpmm_bridge/server_pdumpmm_bridge.o \
	generated/dpdumpmm_bridge/client_pdumpmm_bridge.o \
	generated/cmm_bridge/server_cmm_bridge.o \
	generated/pdumpcmm_bridge/server_pdumpcmm_bridge.o \
	generated/pdump_bridge/server_pdump_bridge.o \
	generated/rgxtq_bridge/server_rgxtq_bridge.o \
	generated/rgxinit_bridge/server_rgxinit_bridge.o \
	generated/rgxta3d_bridge/server_rgxta3d_bridge.o \
	generated/rgxcmp_bridge/server_rgxcmp_bridge.o \
	generated/rgxccb_bridge/server_rgxccb_bridge.o \
	generated/srvcore_bridge/server_srvcore_bridge.o \
	generated/sync_bridge/server_sync_bridge.o \
	generated/dsync_bridge/client_sync_bridge.o \
	generated/hostportio_bridge/server_hostportio_bridge.o \
	generated/breakpoint_bridge/server_breakpoint_bridge.o \
	generated/debugmisc_bridge/server_debugmisc_bridge.o \
	generated/rgxpdump_bridge/server_rgxpdump_bridge.o

ifeq ($(CACHEFLUSH_TYPE),CACHEFLUSH_GENERIC)
pvrsrvkm-y += \
	services/server/common/cache_generic.o \
	generated/cachegeneric_bridge/server_cachegeneric_bridge.o
endif

ifeq ($(SUPPORT_SECURE_EXPORT),1)
pvrsrvkm-y += \
	services/server/env/linux/ossecure_export.o \
	generated/smm_bridge/server_smm_bridge.o \
	generated/syncsexport_bridge/server_syncsexport_bridge.o
else
pvrsrvkm-y += \
	generated/syncexport_bridge/server_syncexport_bridge.o
endif

ifeq ($(SUPPORT_PMMIF),1)
pvrsrvkm-y += \
	services/server/common/physmem_pmmif.o \
	generated/pmmif_bridge/server_pmmif_bridge.o
endif

ifneq ($(W),1)
CFLAGS_server_bc_bridge.o := -Werror
CFLAGS_server_dc_bridge.o := -Werror
CFLAGS_server_mm_bridge.o := -Werror
CFLAGS_server_cmm_bridge.o := -Werror
CFLAGS_client_mm_bridge.o := -Werror
CFLAGS_client_pdumpmm_bridge.o := -Werror
CFLAGS_server_pdump_bridge.o := -Werror
CFLAGS_server_sync_bridge.o := -Werror
CFLAGS_server_rgxtq_bridge.o := -Werror
CFLAGS_server_rgxinit_bridge.o := -Werror
CFLAGS_server_rgxta3d_bridge.o := -Werror
CFLAGS_server_rgxcmp_bridge.o := -Werror
CFLAGS_server_rgxccb_bridge.o := -Werror
CFLAGS_server_srvcore_bridge.o := -Werror
CFLAGS_server_hostportio_bridge.o := -Werror
CFLAGS_server_breakpoint_bridge.o := -Werror
CFLAGS_server_debugmisc_bridge.o := -Werror
CFLAGS_server_rgxpdump_bridge.o := -Werror
CFLAGS_server_pdumpmm_bridge.o := -Werror
CFLAGS_client_pdumpmm_bridge.o := -Werror
CFLAGS_server_pdumpcmm_bridge.o := -Werror
ifeq ($(CACHEFLUSH_TYPE),CACHEFLUSH_GENERIC)
CFLAGS_cache_generic.o := -Werror
CFLAGS_server_cachegeneric_bridge.o := -Werror
endif
ifeq ($(SUPPORT_SECURE_EXPORT),1)
CFLAGS_ossecure_export.o := -Werror
CFLAGS_server_smm_bridge.o := -Werror
CFLAGS_server_syncsexport_bridge.o := -Werror
else
CFLAGS_server_syncexport_bridge.o := -Werror
endif
ifeq ($(SUPPORT_PMMIF),1)
CFLAGS_physmem_pmmif.o := -Werror
CFLAGS_server_pmmif_bridge.o = -Werror
endif
endif

ifeq ($(SUPPORT_DRI_DRM),1)
pvrsrvkm-y += \
 services/server/env/linux/pvr_drm.o \
 services/server/env/linux/pvr_drm_mem.o

ccflags-y += \
 -I$(KERNELDIR)/include/drm \
 -I$(TOP)/services/include/env/linux
endif # SUPPORT_DRI_DRM

include $(TOP)/services/system/$(PVR_SYSTEM)/Kbuild.mk
