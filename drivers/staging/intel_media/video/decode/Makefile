# Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl
MEDIA_INCDIR=drivers/staging/intel_media
ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/rgx/include \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/../intel_media/video/common \
	-I$(INCDIR)/../intel_media/video/decode \
	-I$(INCDIR)/../../../include/linux \
	-I$(INCDIR)/../../../include/drm \
	-I$(INCDIR)/../../../include/drm/ttm

ccflags-y += -DANDROID -D_linux_ -DLINUX -D__KERNEL__ -DDISABLE_ENCODE -DPSB_MSVDX_FW_LOADED_BY_HOST -DMERRIFIELD -DCONFIG_VIDEO_MRFLD

#VIDEO_COMMON_DIR = $(TOP_DIR)/driver/staging/intel_media/video/common
#DECODE_DIR = $(TOP_DIR)/driver/staging/intel_media/video/decode

obj-y += \
	psb_msvdx.o \
	psb_msvdxinit.o \
	psb_msvdx_fw.o

