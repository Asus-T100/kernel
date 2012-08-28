#
# Makefile for modem control device driver.
#

obj-$(CONFIG_MDM_CTRL) += modem_control.o
modem_control-objs := mdm_util.o mdm_ctrl.o

ccflags-$(CONFIG_MDM_IMC_EARLY_BOOT) += -DMDM_IMC_EARLY_BOOT

ifeq ($(CONFIG_MDM_IMC_6260), y)
  CFLAGS_mdm_ctrl.o += -DMDM_IMC_EARLY_BOOT
  modem_control-objs += imc_6x6x.o
endif

ifeq ($(CONFIG_MDM_IMC_7160), y)
  modem_control-objs += imc_7x6x.o
endif

