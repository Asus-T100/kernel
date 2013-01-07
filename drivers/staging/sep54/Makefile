obj-$(CONFIG_DX_SEP54) += sep_driver.o
sep_driver-objs := dx_driver.o sep_init.o crypto_ctx_mgr.o sep_sysfs.o \
                   desc_mgr.o lli_mgr.o crypto_api.o sep_request_mgr.o \
                   sep_power.o sepapp.o

ccflags-y += -DSEP_SUPPORT_SHA=256

