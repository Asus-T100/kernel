CFLAGS_pmu.o        := -Werror

obj-$(CONFIG_INTEL_MID_MDFLD_POWER)		+= pmu.o
ifneq ($(strip $(CONFIG_BOARD_REDRIDGE)),)
obj-$(CONFIG_BOARD_REDRIDGE)   += board-redridge.o
else
obj-$(CONFIG_X86_MDFLD)		+= blackbay_pr2.o
endif
