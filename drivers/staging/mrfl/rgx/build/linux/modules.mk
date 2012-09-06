########################################################################### ###
#@Title         Module processing
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

# Bits for processing $(modules) after reading in each Linux.mk

#$(info ---- $(modules) ----)
$(call must-be-nonempty,modules)

$(foreach _m,$(modules),$(if $(filter $(_m),$(ALL_MODULES)),$(error In makefile $(THIS_MAKEFILE): Duplicate module $(_m) (first seen in $(INTERNAL_MAKEFILE_FOR_MODULE_$(_m))) listed in $$(modules)),$(eval $(call register-module,$(_m)))))

ALL_MODULES += $(modules)
