 # Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/imgv \
	-I$(INCDIR)/vsp \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += -DANDROID -D_linux_ -D__KERNEL__

obj-y += lnc_topaz.o \
	lnc_topazinit.o \
	pnw_topaz.o \
	pnw_topazinit.o \
	psb_buffer.o \
	psb_fence.o \
	psb_mmu.o \
	psb_msvdx.o \
	psb_msvdxinit.o \
	psb_ttm_glue.o \
	psb_ttm_fence.o \
	psb_ttm_fence_user.o \
	psb_ttm_placement_user.o

