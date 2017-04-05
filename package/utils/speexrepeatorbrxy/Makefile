#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=speexrepeatorbrxy
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/speexrepeatorbrxy
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=speexrepeatorbrxy
  DEPENDS:=+libspeex +libpthread +alsa-lib
endef


define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef


define Build/Configure
endef
TARGET_LDFLAGS :=-lspeex -lasound -lpthread
define Build/Compile
	$(MAKE)	$(PKG_JOBS) -C $(PKG_BUILD_DIR) CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/speexrepeatorbrxy/install
	$(INSTALL_DIR) $(1)/usr
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/speexrepeatorbrxy $(1)/usr/
	
endef

$(eval $(call BuildPackage,speexrepeatorbrxy))