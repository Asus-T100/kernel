########################################################################### ###
#@File          xorg.mk
#@Title         X.Org tunables
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@Description   Linux build system X.Org tunables.
#@License       Strictly Confidential.
### ###########################################################################
PVR_SECURE_DRM_AUTH_EXPORT := 1


ifeq ($(PVR_REMOTE),1)
ifeq ($(filter remote%,$(XORG_PVR_VIDEO)),)
else
endif
else
endif

