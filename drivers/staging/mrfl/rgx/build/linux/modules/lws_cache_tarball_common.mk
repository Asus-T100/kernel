########################################################################### ###
#@File          
#@Title         Stuff used by all the lws_cache_tarball modules.
#@Copyright     Copyright (c) Imagination Technologies Ltd. All Rights Reserved
#@License       Dual MIT/GPLv2
# 
# The contents of this file are subject to the MIT license as set out below.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# Alternatively, the contents of this file may be used under the terms of
# the GNU General Public License Version 2 ("GPL") in which case the provisions
# of GPL are applicable instead of those above.
# 
# If you wish to allow use of your version of this file only under the terms of
# GPL, and not to allow others to use your version of this file under the terms
# of the MIT license, indicate your decision by deleting the provisions above
# and replace them with the notice and other provisions required by GPL as set
# out in the file called "GPL-COPYING" included in this distribution. If you do
# not delete the provisions above, a recipient may use your version of this file
# under the terms of either the MIT license or GPL.
# 
# This License is also included in this distribution in the file called
# "MIT-COPYING".
# 
# EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
# PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
### ###########################################################################

# Stuff that is set by the relevant tarball.mk file.
MODULE_TARBALL         := $($(THIS_MODULE)_tarball)
MODULE_PATCHES         := $($(THIS_MODULE)_patches)
MODULE_TARGETS         := $(MODULE_OUT)/.$($(THIS_MODULE))
MODULE_DEPENDS         := $(foreach _d,$($(THIS_MODULE)_depends),$(MODULE_OUT)/.$($(_d)))
MODULE_BYPASS_CACHE    := $(if $($(THIS_MODULE)_bypass_cache),true,)
MODULE_EXTRACT_DIR     := $(if $($(THIS_MODULE)_extract_dir),$($(THIS_MODULE)_extract_dir),$($(THIS_MODULE)))

MODULE_CONFIGURE_FLAGS := $($(THIS_MODULE)_configure_flags)
MODULE_AUTORECONF      := $(if $($(THIS_MODULE)_autoreconf),true,)
MODULE_INTLTOOLIZE     := $(if $($(THIS_MODULE)_intltoolize),true,)

MODULE_SOURCE_NAMES    := $(patsubst $(THIS_DIR)/%,%,$(MODULE_SOURCES))

# Name of the cached tarball we are going to generate.
# Concatenate the following..
#
#  - List of flags passed to `./configure'  (We add to these in the module-specific file, 
#                                            but usually unconditionally)
#  - Value of ALL_LWS_CFLAGS (effectively debug vs release)
#  - Value of ALL_LWS_CXXFLAGS
#  - CROSS_TRIPLE (As we haven't yet addeds cross-compile info to configure flags)
#  - Names of the header files we have to extract and use elsewhere.
#  - Contents of source tarball
#  - Contents of DDK patches to sources
#
# ..and pipe it through md5sum. The resulting hash is used to
# generate the filename for the cache. If any of the above things
# change, the cache will be regenerated, ensuring coherency.
#
# FIXME: Ideally we'd hash MODULE_LDFLAGS here too, but we can't because
#        it refer to paths $(MODULE_OUT)$(LWS_PREFIX) that depend on the 
#	 location of OUT.
#        Also, MODULE_LDFLAGS is still very module-dependent so can't be 
#        used in common code.
LWS_CACHE_FILE_NAME := $(LWS_BUILD_CACHES)/$(THIS_MODULE)-$(shell \
	echo $(MODULE_CONFIGURE_FLAGS) $(ALL_LWS_CFLAGS) $(ALL_LWS_CXXFLAGS) $(CROSS_TRIPLE) $(MODULE_SOURCE_NAMES)| \
	cat - $(MODULE_TARBALL) $(MODULE_PATCHES) | \
	md5sum -b - | \
	cut -d' ' -f1).tar.bz2

# Marking LWS_CACHE_FILE_NAME as phony will cause it to always
# be rebuilt. This acts as an lws cache bypass and is useful
# for development.
ifeq ($(MODULE_BYPASS_CACHE),true)
.PHONY: $(LWS_CACHE_FILE_NAME)
endif

# Patch and make available any .pc files
define lws-export-pc
$(MKDIR) -p $(PKG_CONFIG_TARGET_PATH)
$(if $(V),,@)for PC in $(MODULE_OUT)$(LWS_PREFIX)/lib/pkgconfig/*.pc $(MODULE_OUT)$(LWS_PREFIX)/share/pkgconfig/*.pc; do \
		[ ! -f $$PC ] && continue; \
		[ -f $(PKG_CONFIG_TARGET_PATH)/`basename $$PC` ] && continue; \
		sed -e '/^\(appdefaultdir\)\|\(dridriverdir\)=.*/ !s,$(LWS_PREFIX),$(abspath $(MODULE_OUT)/$(LWS_PREFIX)),' $$PC > $(PKG_CONFIG_TARGET_PATH)/`basename $$PC`; \
	done
endef

.PHONY: $(THIS_MODULE)
$(THIS_MODULE): $(MODULE_TARGETS)

$(MODULE_TARGETS): LWS_CACHE_FILE_NAME := $(LWS_CACHE_FILE_NAME)
$(MODULE_TARGETS): $(PKG_CONFIG_ENV_VAR) := $(PKG_CONFIG_TARGET_PATH)
$(MODULE_TARGETS): MODULE_OUT := $(MODULE_OUT)
$(MODULE_TARGETS): MODULE_DESTDIR := $(abspath $(MODULE_INTERMEDIATES_DIR))/.build
$(MODULE_TARGETS): $(LWS_CACHE_FILE_NAME) | $(MODULE_DEPENDS) $(MODULE_OUT)
	@: $(if $(filter j3.81,$(findstring j,$(MAKEFLAGS))$(MAKE_VERSION)),\
		$(error This makefile triggers bug #15919 in GNU Make 3.81 \
				when parallel make (-j) is used. Please upgrade \
				to GNU Make >= 3.82, or switch off parallel make))
	$(TAR) -C $(MODULE_OUT) -jxf $(LWS_CACHE_FILE_NAME)
	$(lws-export-pc)
	$(TOUCH) $@

# Tools possibly used in other build steps
define lws-extract-tarball
$(RM) -r $(MODULE_INTERMEDIATES_DIR)/source
$(TAR) -C $(MODULE_INTERMEDIATES_DIR) -xf $(MODULE_TARBALL)
$(MV) $(MODULE_INTERMEDIATES_DIR)/$(MODULE_EXTRACT_DIR) $(MODULE_INTERMEDIATES_DIR)/source
endef

define lws-patch-source
$(if $(V),,@)for patch in $(MODULE_PATCHES); do \
	patch -d $(MODULE_INTERMEDIATES_DIR)/source -sNp2 -i $$patch; \
done
endef

define lws-make
$(if $(V),,@)+MAKEFLAGS="$(filter-out -Rr -Rrk TOP=$(TOP),$(MAKEFLAGS))" \
	$(MAKE) -C $(MODULE_INTERMEDIATES_DIR) LDFLAGS="$(MODULE_LDFLAGS)"
endef

define lws-install
$(lws-make) DESTDIR="$(MODULE_DESTDIR)" install
$(if $(V),,@)( find $(MODULE_DESTDIR) -name '*.la' -exec rm -f '{}' ';' || /bin/true )
endef

# Run autoreconf step
ifeq ($(MODULE_AUTORECONF),true)
define lws-autoreconf
$(if $(V),,@)pwd=$$PWD && \
	cd $(MODULE_INTERMEDIATES_DIR)/source && \
		autoreconf --force --verbose --install && \
	cd $$pwd
endef

.SECONDARY: $(MODULE_INTERMEDIATES_DIR)/.autoreconf
$(MODULE_INTERMEDIATES_DIR)/.autoreconf: MODULE_INTERMEDIATES_DIR := $(MODULE_INTERMEDIATES_DIR)
$(MODULE_INTERMEDIATES_DIR)/.autoreconf: | $(MODULE_DEPENDS) $(MODULE_OUT)$(LWS_ACLOCAL)
ifeq ($(MODULE_INTLTOOLIZE),true)
$(MODULE_INTERMEDIATES_DIR)/.autoreconf: $(MODULE_INTERMEDIATES_DIR)/.intltoolize
else
$(MODULE_INTERMEDIATES_DIR)/.autoreconf: $(MODULE_INTERMEDIATES_DIR)/.extract
endif
	$(lws-autoreconf)
	$(TOUCH) $@
endif

# Run intltoolize step
ifeq ($(MODULE_INTLTOOLIZE),true)
define lws-intltoolize
$(if $(V),,@)pwd=$$PWD && \
	cd $(MODULE_INTERMEDIATES_DIR)/source && \
		intltoolize && \
	cd $$pwd
endef

.SECONDARY: $(MODULE_INTERMEDIATES_DIR)/.intltoolize
$(MODULE_INTERMEDIATES_DIR)/.intltoolize: MODULE_INTERMEDIATES_DIR := $(MODULE_INTERMEDIATES_DIR)
$(MODULE_INTERMEDIATES_DIR)/.intltoolize: | $(MODULE_DEPENDS)
$(MODULE_INTERMEDIATES_DIR)/.intltoolize: $(MODULE_INTERMEDIATES_DIR)/.extract
	$(lws-intltoolize)
	$(TOUCH) $@
endif

