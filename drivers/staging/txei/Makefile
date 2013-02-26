#
# Intel Management Engine Interface (Intel TXEI) Linux driver
# Copyright (c) 2010-2011, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
obj-$(CONFIG_INTEL_TXEI) += txei.o
txei-objs := init.o
txei-objs += interrupt.o
txei-objs += iorw.o
txei-objs += main.o
txei-objs += hw-txe.o
txei-objs += pci-txe.o
txei-objs += txei_mm.o
# Still needs debugging
#obj-$(CONFIG_DEBUG_FS) += txei_debugfs.o
