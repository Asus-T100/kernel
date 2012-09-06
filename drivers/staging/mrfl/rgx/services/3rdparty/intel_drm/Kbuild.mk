# Copyright	2011 Imagination Technologies Limited. All rights reserved.
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

ccflags-y += \
 -I$(TOP)/../interface \
 -I$(TOP)/services/3rdparty/intel_drm \
 -DDC_NOHW_DISCONTIG_BUFFERS -DDC_NOHW_GET_BUFFER_DIMENSIONS

dcnohw-y += \
	services/3rdparty/intel_drm/dc_mrfld_displayclass.o \
	services/3rdparty/intel_drm/pvr_drm.o \
	services/3rdparty/intel_drm/sysconfig.o \
	services/3rdparty/intel_drm/topaz_power.o \
	services/3rdparty/intel_drm/msvdx_power.o

ifeq ($(PDUMP),1)
ccflags-y += -I$(TOP)/tools/intern/debug
endif