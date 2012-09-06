########################################################################### ###
#@File          xorg_test.mk
#@Title         Determine whether X.Org support is required
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@Description   Determine whether X.Org related components should be built
#               by the Linux build system.
#@License       Strictly Confidential.
### ###########################################################################

# FIXME: Will go away when SUPPORT_DRI_DRM is untangled from
#        the old meaning of SUPPORT_XORG=1.

ifeq ($(filter xorg,$(EXCLUDED_APIS)),)
ifneq ($(wildcard ../common/apis/xorg.mk),)
want_xorg := 1
endif
endif
