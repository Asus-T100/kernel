########################################################################### ###
#@File          dridrm.mk
#@Title         DRI/DRM tunables
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@Description   DRI/DRM tuables for the Linux build system.
#@License       Strictly Confidential.
### ###########################################################################

$(eval $(call TunableBothConfigC,SUPPORT_DRI_DRM,))
$(eval $(call TunableKernelConfigC,SUPPORT_DRI_DRM_PLUGIN,))


$(eval $(call TunableBothConfigMake,SUPPORT_DRI_DRM,))

ifeq ($(SUPPORT_DRI_DRM),1)
ifeq ($(SUPPORT_DRI_DRM_NO_LIBDRM),1)
endif
$(eval $(call TunableKernelConfigC,PVR_SECURE_DRM_AUTH_EXPORT,))
$(eval $(call TunableBothConfigC,SUPPORT_DRM_DC_MODULE,))
endif

$(eval $(call TunableKernelConfigC,PVR_DISPLAY_CONTROLLER_DRM_IOCTL,))

$(eval $(call TunableKernelConfigC,PVR_DRI_DRM_PLATFORM_DEV,))

