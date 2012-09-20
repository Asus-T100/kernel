########################################################################### ###
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Strictly Confidential.
### ###########################################################################

# from-one-* recipes make a thing from one source file, so they use $<. Others
# use $(MODULE_something) instead of $^

# We expect that MODULE_*FLAGS contains all the flags we need, including the
# flags for all modules (like $(ALL_CFLAGS) and $(ALL_HOST_CFLAGS)), and
# excluding flags for include search dirs or for linking libraries. The
# exceptions are ALL_EXE_LDFLAGS and ALL_LIB_LDFLAGS, since they depend on the
# type of thing being linked, so they appear in the commands below

define host-o-from-one-c
$(if $(V),,@echo "  HOST_CC " $(call relative-to-top,$<))
$(HOST_CC) -MD -c $(MODULE_HOST_CFLAGS) $(MODULE_INCLUDE_FLAGS) \
	-include $(RGX_BVNC_H) \
	-include $(CONFIG_H) $< -o $@
endef

define target-o-from-one-c
$(if $(V),,@echo "  CC      " $(call relative-to-top,$<))
$(CC) -MD -c $(MODULE_CFLAGS) $(MODULE_INCLUDE_FLAGS) \
	 -include $(RGX_BVNC_H) \
	 -include $(CONFIG_H) $< -o $@
endef

# We use $(CC) to compile C++ files, and expect it to detect that it's
# compiling C++
define host-o-from-one-cxx
$(if $(V),,@echo "  HOST_CC " $(call relative-to-top,$<))
$(HOST_CC) -MD -c $(MODULE_HOST_CXXFLAGS) $(MODULE_INCLUDE_FLAGS) \
	 -include $(RGX_BVNC_H) \
	 -include $(CONFIG_H) $< -o $@
endef

define target-o-from-one-cxx
$(if $(V),,@echo "  CC      " $(call relative-to-top,$<))
$(CC) -MD -c $(MODULE_CXXFLAGS) $(MODULE_INCLUDE_FLAGS) \
	 -include $(RGX_BVNC_H) \
	 -include $(CONFIG_H) $< -o $@
endef

define host-executable-from-o
$(if $(V),,@echo "  HOST_LD " $(call relative-to-top,$@))
$(HOST_CC) $(MODULE_HOST_LDFLAGS) \
	-o $@ $(sort $(MODULE_ALL_OBJECTS)) $(MODULE_LIBRARY_DIR_FLAGS) \
	$(MODULE_LIBRARY_FLAGS)
endef

define host-executable-cxx-from-o
$(if $(V),,@echo "  HOST_LD " $(call relative-to-top,$@))
$(HOST_CXX) $(MODULE_HOST_LDFLAGS) \
	-o $@ $(sort $(MODULE_ALL_OBJECTS)) $(MODULE_LIBRARY_DIR_FLAGS) \
	$(MODULE_LIBRARY_FLAGS)
endef

define target-executable-from-o
$(if $(V),,@echo "  LD      " $(call relative-to-top,$@))
$(CC) \
	$(SYS_EXE_LDFLAGS) $(MODULE_LDFLAGS) -o $@ \
	$(SYS_EXE_CRTBEGIN) $(sort $(MODULE_ALL_OBJECTS)) $(SYS_EXE_CRTEND) \
	$(MODULE_LIBRARY_DIR_FLAGS) $(MODULE_LIBRARY_FLAGS) $(LIBGCC)
endef

define target-executable-cxx-from-o
$(if $(V),,@echo "  LD      " $(call relative-to-top,$@))
$(CXX) \
	$(SYS_EXE_LDFLAGS) $(MODULE_LDFLAGS) -o $@ \
	$(SYS_EXE_CRTBEGIN) $(sort $(MODULE_ALL_OBJECTS)) $(SYS_EXE_CRTEND) \
	$(MODULE_LIBRARY_DIR_FLAGS) $(MODULE_LIBRARY_FLAGS) $(LIBGCC)
endef

define target-shared-library-from-o
$(if $(V),,@echo "  LD      " $(call relative-to-top,$@))
$(CC) -shared -Wl,-Bsymbolic \
	$(SYS_LIB_LDFLAGS) $(MODULE_LDFLAGS) -o $@ \
	$(SYS_LIB_CRTBEGIN) $(sort $(MODULE_ALL_OBJECTS)) $(SYS_LIB_CRTEND) \
	$(MODULE_LIBRARY_DIR_FLAGS) $(MODULE_LIBRARY_FLAGS) $(LIBGCC)
endef

define host-shared-library-from-o
$(if $(V),,@echo "  HOST_LD " $(call relative-to-top,$@))
$(HOST_CC) -shared -Wl,-Bsymbolic \
	$(MODULE_HOST_LDFLAGS) -o $@ \
	$(sort $(MODULE_ALL_OBJECTS)) \
	$(MODULE_LIBRARY_DIR_FLAGS) $(MODULE_LIBRARY_FLAGS)
endef

# If there were any C++ source files in a shared library, we use this recipe,
# which runs the C++ compiler to link the final library
define target-shared-library-cxx-from-o
$(if $(V),,@echo "  LD      " $(call relative-to-top,$@))
$(CXX) -shared -Wl,-Bsymbolic \
	$(SYS_LIB_LDFLAGS) $(MODULE_LDFLAGS) -o $@ \
	$(SYS_LIB_CRTBEGIN) $(sort $(MODULE_ALL_OBJECTS)) $(SYS_LIB_CRTEND) \
	$(MODULE_LIBRARY_DIR_FLAGS) $(MODULE_LIBRARY_FLAGS) $(LIBGCC)
endef

define host-copy-debug-information
$(HOST_OBJCOPY) --only-keep-debug $@ $(basename $@).dbg
endef

define host-strip-debug-information
$(HOST_STRIP) --strip-unneeded $@
endef

define host-add-debuglink
$(if $(V),,@echo "  DBGLINK " $(call relative-to-top,$(basename $@).dbg))
$(HOST_OBJCOPY) --add-gnu-debuglink=$(basename $@).dbg $@
endef

define target-copy-debug-information
$(OBJCOPY) --only-keep-debug $@ $(basename $@).dbg
endef

define target-strip-debug-information
$(STRIP) --strip-unneeded $@
endef

define target-add-debuglink
$(if $(V),,@echo "  DBGLINK " $(call relative-to-top,$(basename $@).dbg))
$(OBJCOPY) --add-gnu-debuglink=$(basename $@).dbg $@
endef

define host-static-library-from-o
$(if $(V),,@echo "  HOST_AR " $(call relative-to-top,$@))
$(HOST_AR) cru $@ $(sort $(MODULE_ALL_OBJECTS))
endef

define target-static-library-from-o
$(if $(V),,@echo "  AR      " $(call relative-to-top,$@))
$(AR) cru $@ $(sort $(MODULE_ALL_OBJECTS))
endef

define tab-c-from-y
$(if $(V),,@echo "  BISON   " $(call relative-to-top,$<))
$(BISON) $(MODULE_BISON_FLAGS) -o $@ -d $<
endef

define l-c-from-l
$(if $(V),,@echo "  FLEX    " $(call relative-to-top,$<))
$(FLEX) $(MODULE_FLEX_FLAGS) -o$@ $<
endef

define clean-dirs
$(if $(V),,@echo "  RM      " $(call relative-to-top,$(MODULE_DIRS_TO_REMOVE)))
$(RM) -rf $(MODULE_DIRS_TO_REMOVE)
endef

define make-directory
$(MKDIR) -p $@
endef

define check-exports
endef

# Programs used in recipes

BISON ?= bison
CC ?= gcc
CXX ?= g++
HOST_CC ?= gcc
HOST_CXX ?= g++
JAR ?= jar
JAVA ?= java
JAVAC ?= javac
ZIP ?= zip

ifeq ($(USE_CCACHE),1)
CCACHE ?= ccache
endif

# META tools
ifeq ($(INTERNAL_CLOBBER_ONLY),)

ifeq ($(strip $(METAG_INST_ROOT)),)
# META_CC and LDLK point to the META compiler and post-linker. These are set
# according to METAG_INST_ROOT and optionally MECC_INST_ROOT. If
# METAG_INST_ROOT isn't set, error out.
$(warning METAG_INST_ROOT is not set to indicate the location of the META)
$(warning toolchain, which is required to build the firmware.)
$(error No META toolchain)
endif

endif # INTERNAL_CLOBBER_ONLY

LDLK := $(METAG_INST_ROOT)/metag-local/bin/ldlk
# define the rgxfw compiler vars depending on whether we use mecc or not
ifeq ($(MECC_INST_ROOT),)
META_CC := $(METAG_INST_ROOT)/metag-local/bin/gcc
else
META_CC := $(MECC_INST_ROOT)/metag-local/bin/mecc
endif

override AR			:= $(if $(V),,@)$(CROSS_COMPILE)ar
override BISON		:= $(if $(V),,@)$(BISON)
override BZIP2		:= $(if $(V),,@)bzip2 -9
override CAT		:= $(if $(V),,@)cat
override CC			:= $(if $(V),,@)$(CCACHE) $(CROSS_COMPILE)$(CC)
override CC_CHECK	:= $(if $(V),,@)$(MAKE_TOP)/tools/cc-check.sh
override CXX		:= $(if $(V),,@)$(CCACHE) $(CROSS_COMPILE)$(CXX)
override CHMOD		:= $(if $(V),,@)chmod
override CP			:= $(if $(V),,@)cp
override ECHO		:= $(if $(V),,@)echo
override FLEX		:= $(if $(V),,@)flex
override GAWK		:= $(if $(V),,@)gawk
override GREP		:= $(if $(V),,@)grep
override HOST_AR	:= $(if $(V),,@)ar
override HOST_CC	:= $(if $(V),,@)$(CCACHE) $(HOST_CC)
override HOST_CXX	:= $(if $(V),,@)$(CCACHE) $(HOST_CXX)
override HOST_OBJCOPY := $(if $(V),,@)objcopy
override HOST_STRIP := $(if $(V),,@)strip
override INSTALL	:= $(if $(V),,@)install
override JAR		:= $(if $(V),,@)$(JAR)
override JAVA		:= $(if $(V),,@)$(JAVA)
override JAVAC		:= $(if $(V),,@)$(JAVAC)
override M4			:= $(if $(V),,@)m4
override META_CC	:= $(if $(V),,@)$(CCACHE) $(META_CC)
override LDLK 		:= $(if $(V),,@)$(LDLK)
override MKDIR		:= $(if $(V),,@)mkdir
override MV			:= $(if $(V),,@)mv
override OBJCOPY	:= $(if $(V),,@)$(CROSS_COMPILE)objcopy
override PERL		:= $(if $(V),,@)perl
override PYTHON		:= $(if $(V),,@)python
override RANLIB		:= $(if $(V),,@)$(CROSS_COMPILE)ranlib
override RM			:= $(if $(V),,@)rm -f
override ROGUEASM	:= $(if $(V),,@)$(HOST_OUT)/rogueasm
override SED		:= $(if $(V),,@)sed
override STRIP		:= $(if $(V),,@)$(CROSS_COMPILE)strip
override TAR		:= $(if $(V),,@)tar
override TOUCH  	:= $(if $(V),,@)touch
override TEST		:= $(if $(V),,@)test
override VHD2INC	:= $(if $(V),,@)$(HOST_OUT)/vhd2inc
override ZIP		:= $(if $(V),,@)$(ZIP)
