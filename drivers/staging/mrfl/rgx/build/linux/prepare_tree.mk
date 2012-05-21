########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

.PHONY: prepare_tree

prepare_tree:

INTERNAL_INCLUDED_PREPARE_HEADERS :=
-include build/linux/prepare_headers.mk
ifneq ($(INTERNAL_INCLUDED_PREPARE_HEADERS),true)
missing_headers := $(strip $(shell test ! -e include/pvrversion.h && echo true))
ifdef missing_headers
$(info )
$(info ** include/pvrversion.h is missing, and cannot be rebuilt.)
$(info ** Cannot continue.)
$(info )
$(error Missing headers)
endif
endif
