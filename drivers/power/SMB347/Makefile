#
#  Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
#  Written by chris chang chris1_chang@asus.com
#

ifeq ($(TARGET_BUILD_VARIANT),eng)
        KBUILD_CFLAGS += -DASUS_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DASUSU_SER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
        KBUILD_CFLAGS += -DASUS_USERDEBUG_BUILD
else
        KBUILD_CFLAGS += -DASUS_OTHER_BUILD
endif

obj-$(CONFIG_ASUS_CHARGER_SMB347)	+= battery_smb347.o


