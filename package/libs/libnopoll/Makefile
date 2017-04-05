#
# Copyright (C) 2006-2014 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=nopoll
PKG_VERSION:=0.4.3.b311
#PKG_VERSION:=0.4.2.b297

PKG_RELEASE:=2


PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=http://www.aspl.es/nopoll/downloads/
PKG_MD5SUM:=ff4f9351be03af547bf7ddc07d49a7251cc62795
#PKG_MD5SUM:=


#PKG_SOURCE_PROTO:=git
#PKG_SOURCE_URL:=https://git.oschina.net/hifiwifi/nopoll-0.2.9.b207-mleaf.git	
#PKG_SOURCE_VERSION:=770016ef79c8035c0372a11c4d873cc3425f8441
#PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.gz
#PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)


PKG_LICENSE:=nopoll
PKG_LICENSE_FILES:=LICENSE
PKG_FIXUP:=autoreconf
PKG_INSTALL:=1
PKG_MAINTAINER:=Felix Fietkau <nbd@openwrt.org>

include $(INCLUDE_DIR)/package.mk

TARGET_CFLAGS += $(FPIC)

define Package/libnopoll
  SECTION:=libs
  CATEGORY:=Libraries
  DEPENDS:=+libcrypto +libopenssl +libpthread
  TITLE:=OpenSource WebSocket written in ansi C
  URL:=http://www.aspl.es/nopoll/
endef

define Package/libnopoll/description
 noPoll is a OpenSource WebSocket implementation (RFC 6455), written in ansi C, that allows building pure WebSocket solutions or to provide WebSocket support to existing TCP oriented applications.
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/nopoll/* $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libnopoll.{a,so*} $(1)/usr/lib/
endef

define Package/libnopoll/install
	$(INSTALL_DIR) $(1)/usr/include
	$(CP) $(PKG_INSTALL_DIR)/usr/include/nopoll/* $(1)/usr/include/
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/libnopoll.so.* $(1)/usr/lib/
	$(INSTALL_DIR) $(1)/usr/lib/pkgconfig
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/pkgconfig/nopoll.pc \
	$(1)/usr/lib/pkgconfig/
endef



$(eval $(call BuildPackage,libnopoll))
