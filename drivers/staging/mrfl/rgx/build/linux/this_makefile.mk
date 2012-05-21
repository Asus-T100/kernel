########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

# Find out the path of the Linux.mk makefile currently being processed, and
# set paths used by the build rules

# This magic is used so we can use this_makefile.mk twice: first when reading
# in each Linux.mk, and then again when generating rules. There we set
# $(THIS_MAKEFILE), and $(REMAINING_MAKEFILES) should be empty
ifneq ($(strip $(REMAINING_MAKEFILES)),)

# Absolute path to the Linux.mk being processed
THIS_MAKEFILE := $(firstword $(REMAINING_MAKEFILES))

# The list of makefiles left to process
REMAINING_MAKEFILES := $(wordlist 2,$(words $(REMAINING_MAKEFILES)),$(REMAINING_MAKEFILES))

else

# When generating rules, we should have read in every Linux.mk
$(if $(INTERNAL_INCLUDED_ALL_MAKEFILES),,$(error No makefiles left in $$(REMAINING_MAKEFILES), but $$(INTERNAL_INCLUDED_ALL_MAKEFILES) is not set))

endif

# Path to the directory containing Linux.mk
THIS_DIR := $(patsubst %/,%,$(dir $(THIS_MAKEFILE)))
ifeq ($(strip $(THIS_DIR)),)
$(error Empty $$(THIS_DIR) for makefile "$(THIS_MAKEFILE)")
endif

modules :=
