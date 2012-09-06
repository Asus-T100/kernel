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

ccflags-y += \
	-I$(KERNELDIR)/include/drm \
	-I$(TOP)/../interface \
	-I$(TOP)/services/3rdparty/bufferclass_video \
	-I$(TOP)/codegen/pixfmts \
  -DBC_DISCONTIG_BUFFERS


bc_video-y += \
	services/3rdparty/bufferclass_video/bufferclass_video.o \
	services/3rdparty/bufferclass_video/bufferclass_video_linux.o
