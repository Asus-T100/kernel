#
# Generic framework for controlling remote processors
#

obj-$(CONFIG_REMOTEPROC)		+= remoteproc.o
remoteproc-y				:= remoteproc_core.o
remoteproc-y				+= remoteproc_debugfs.o
remoteproc-y				+= remoteproc_virtio.o

obj-$(CONFIG_INTEL_MID_REMOTEPROC)	+= intel_mid_rproc_scu.o intel_mid_rproc_core.o
obj-$(CONFIG_OMAP_REMOTEPROC)		+= omap_remoteproc.o
