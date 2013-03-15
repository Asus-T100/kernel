#
# Makefile for modem control device driver.
#

obj-$(CONFIG_MDM_CTRL) += modem_control.o
modem_control-objs := mdm_util.o mdm_ctrl.o imc_6x6x.o imc_7x6x.o

# Workaround for CTPSCALELT as no board information is available yet
ccflags-$(CONFIG_MDM_CTRL_CTPLT) += -DMDM_CTRL_CTPLT
