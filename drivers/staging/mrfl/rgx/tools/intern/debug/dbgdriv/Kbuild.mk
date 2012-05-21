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
	-I$(TOP)/tools/intern/debug/dbgdriv/common \
	-I$(TOP)/tools/intern/debug/include

dbgdrv-y += \
	tools/intern/debug/dbgdriv/common/dbgdriv.o \
	tools/intern/debug/dbgdriv/common/ioctl.o \
	tools/intern/debug/dbgdriv/common/handle.o \
	tools/intern/debug/dbgdriv/common/hotkey.o \
	tools/intern/debug/dbgdriv/linux/main.o \
	tools/intern/debug/dbgdriv/linux/hostfunc.o
