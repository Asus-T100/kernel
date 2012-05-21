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

pvrsrvkm-y += services/system/$(PVR_SYSTEM)/sysconfig.o \
				services/system/common/env/linux/pci_support.o

ifneq ($(W),1)
CFLAGS_sysconfig.o := -Werror
CFLAGS_pci_support.o := -Werror
endif
