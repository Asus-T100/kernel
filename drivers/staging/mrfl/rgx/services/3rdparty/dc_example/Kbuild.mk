########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

ccflags-y += \
 -I$(TOP)/services/3rdparty/dc_example

dc_example-y += \
	services/3rdparty/dc_example/dc_example.o

ifneq ($(W),1)
CFLAGS_dc_example.o := -Werror
endif
