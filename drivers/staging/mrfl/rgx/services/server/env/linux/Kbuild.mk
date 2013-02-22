########################################################################### ###
#@File
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
# 
# The contents of this file are subject to the MIT license as set out below.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
# 
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
# 
# This License is also included in this distribution in the file called
# "MIT-COPYING".
# 
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

pvrsrvkm-y += \
 services/server/env/linux/event.o \
 services/server/env/linux/lock.o \
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
 services/server/common/sync_server.o \
 services/server/common/tlintern.o \
 services/server/common/tlserver.o \
 services/server/common/tlstream.o \
 services/server/common/tltest.o 
  

pvrsrvkm-$(CONFIG_X86) += services/server/env/linux/osfunc_x86.o
pvrsrvkm-$(CONFIG_ARM) += services/server/env/linux/osfunc_arm.o

ifneq ($(W),1)
CFLAGS_lock.o := -Werror
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
CFLAGS_devicemem_pdump.o = -Werror
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
 services/server/devices/rgx/rgxbreakpoint.o \
 services/server/devices/rgx/debugmisc_server.o \
 services/shared/devices/rgx/rgx_compat_bvnc.o

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
CFLAGS_rgxbreakpoint.o := -Werror
CFLAGS_debugmisc_server.o := -Werror
endif

ifeq ($(PDUMP),1)
pvrsrvkm-y += services/server/devices/rgx/rgxpdump.o
ifneq ($(W),1)
CFLAGS_rgxpdump.o := -Werror
endif
endif

# Bridge headers and source files
ccflags-y += \
 -I$(bridge_base)/dc_bridge \
 -I$(bridge_base)/dmm_bridge \
 -I$(bridge_base)/mm_bridge \
 -I$(bridge_base)/cmm_bridge \
 -I$(bridge_base)/pdumpmm_bridge \
 -I$(bridge_base)/pdumpcmm_bridge \
 -I$(bridge_base)/pdump_bridge \
 -I$(bridge_base)/rgxtq_bridge \
 -I$(bridge_base)/rgxinit_bridge \
 -I$(bridge_base)/rgxta3d_bridge \
 -I$(bridge_base)/rgxcmp_bridge \
 -I$(bridge_base)/rgxccb_bridge \
 -I$(bridge_base)/srvcore_bridge \
 -I$(bridge_base)/dsync_bridge \
 -I$(bridge_base)/sync_bridge \
 -I$(bridge_base)/breakpoint_bridge \
 -I$(bridge_base)/debugmisc_bridge \
 -I$(bridge_base)/rgxpdump_bridge \
 -I$(bridge_base)/pvrtl_bridge \
 -I$(bridge_base)/rgxhwperf_bridge

# Compatibility BVNC
ccflags-y += \
 -I$(TOP)/services/shared/devices/rgx

# Errata files
ccflags-y += \
 -I$(TOP)/hwdefs

ifeq ($(CACHEFLUSH_TYPE),CACHEFLUSH_GENERIC)
ccflags-y += -I$(bridge_base)/cachegeneric_bridge
endif

ifeq ($(SUPPORT_SECURE_EXPORT),1)
ccflags-y += \
 -I$(bridge_base)/syncsexport_bridge \
 -I$(bridge_base)/smm_bridge
endif
ifeq ($(SUPPORT_INSECURE_EXPORT),1)
ccflags-y += \
 -I$(bridge_base)/syncexport_bridge
endif

ifeq ($(SUPPORT_PMMIF),1)
ccflags-y += -I$(bridge_base)/pmmif_bridge
endif

ifeq ($(SUPPORT_ION),1)
ccflags-y += -I$(TOP)/services/include/env/linux
ccflags-y += -I$(bridge_base)/ion_bridge
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
 generated/breakpoint_bridge/server_breakpoint_bridge.o \
 generated/debugmisc_bridge/server_debugmisc_bridge.o \
 generated/rgxpdump_bridge/server_rgxpdump_bridge.o \
 generated/pvrtl_bridge/server_pvrtl_bridge.o \
 generated/rgxhwperf_bridge/server_rgxhwperf_bridge.o

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
endif
ifeq ($(SUPPORT_INSECURE_EXPORT),1)
pvrsrvkm-y += \
 generated/syncexport_bridge/server_syncexport_bridge.o
endif

ifeq ($(SUPPORT_PMMIF),1)
pvrsrvkm-y += \
 services/server/common/physmem_pmmif.o \
 generated/pmmif_bridge/server_pmmif_bridge.o
endif

ifeq ($(SUPPORT_ION),1)
pvrsrvkm-y += \
 services/server/env/linux/physmem_ion.o \
 generated/ion_bridge/server_ion_bridge.o
endif

ifneq ($(W),1)
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
endif
ifeq ($(SUPPORT_INSECURE_EXPORT),1)
CFLAGS_server_syncexport_bridge.o := -Werror
endif
ifeq ($(SUPPORT_PMMIF),1)
CFLAGS_physmem_pmmif.o := -Werror
CFLAGS_server_pmmif_bridge.o = -Werror
endif
CFLAGS_server_pvrtl_bridge.o := -Werror
CFLAGS_server_rgxhwperf_bridge.o := -Werror
ifeq ($(SUPPORT_ION),1)
CFLAGS_physmem_ion.o := -Werror
CFLAGS_server_ion_bridge.o = -Werror
endif
endif

ifeq ($(SUPPORT_DRM),1)
pvrsrvkm-y += \
 services/server/env/linux/pvr_drm.o \
 services/server/env/linux/pvr_drm_mem.o

ccflags-y += \
 -I$(KERNELDIR)/include/drm \
 -I$(TOP)/services/include/env/linux
endif # SUPPORT_DRM

include $(TOP)/services/system/$(PVR_SYSTEM)/Kbuild.mk
