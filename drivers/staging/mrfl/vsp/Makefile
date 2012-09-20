 # Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

INCDIR=drivers/staging/mrfl

ccflags-y += \
	-I$(INCDIR)/ \
	-I$(INCDIR)/interface \
	-I$(INCDIR)/drv \
	-I$(INCDIR)/imgv \
	-I$(INCDIR)/vsp \
	-I$(INCDIR)/rgx \
	-Iinclude/linux \
	-Iinclude/drm

ccflags-y += -DANDROID -D_linux_ -D__KERNEL__

obj-y += vsp_init.o \
	vsp.o
