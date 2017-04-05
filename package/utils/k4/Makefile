#
# Copyright (C) 2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=k4
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/k4
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=k4
  DEPENDS:=+libnopoll +libopenssl +libspeex +libcurl +zlib +libmad +libstdcpp +alsa-lib +libuuid +libstdcpp +libjson-c +zlib +librt +libpthread +libuci
endef


define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef


define Build/Configure
endef
TARGET_LDFLAGS := -lnopoll -luci -lspeex -lcrypto -lssl -lmad -luuid -lasound -ltxdevicesdk -lz -ljson-c -lstdc++ -lpthread -lrt -lcurl -lm
define Build/Compile
	$(MAKE)	$(PKG_JOBS) -C $(PKG_BUILD_DIR) CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS) -Wall" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/k4/install
	$(INSTALL_DIR) $(1)/usr
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/k4 $(1)/usr/
	
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_BIN) files/libtxdevicesdk.so $(1)/usr/lib/libtxdevicesdk.so
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/1700002626.pem $(1)/usr/share/brxydata/1700002626.pem
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/public.pem $(1)/usr/share/brxydata/public.pem
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/licence.sign.file.txt $(1)/usr/share/brxydata/licence.sign.file.txt
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/brxy $(1)/usr/share/brxydata/brxy
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/card $(1)/usr/share/brxydata/card
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata/filesback
	$(INSTALL_BIN) files/brxy.back $(1)/usr/share/brxydata/filesback/brxy
	
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) files/websocket $(1)/etc/init.d/websocket
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/dts2bwatchdog.sh $(1)/usr/share/brxydata/dts2bwatchdog.sh
	
	$(INSTALL_DIR) $(1)/etc/crontabs
	$(INSTALL_BIN) files/root $(1)/etc/crontabs/root
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata/filesback
	$(INSTALL_BIN) files/card.back $(1)/usr/share/brxydata/filesback/card
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/Guid_file.txt $(1)/usr/share/brxydata/Guid_file.txt
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata/music
	$(INSTALL_BIN) files/music/* $(1)/usr/share/brxydata/music/
	
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/brxy-inner-ca-cert.pem $(1)/usr/share/brxydata/brxy-inner-ca-cert.pem
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/dts2b-cert.pem $(1)/usr/share/brxydata/dts2b-cert.pem
	$(INSTALL_DIR) $(1)/usr/share/brxydata
	$(INSTALL_BIN) files/dts2b-key-nopwd.pem $(1)/usr/share/brxydata/dts2b-key-nopwd.pem
	
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) files/brxyaps $(1)/usr/sbin/brxyaps
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) files/brxycheckwifi $(1)/usr/sbin/brxycheckwifi
	
endef

$(eval $(call BuildPackage,k4))
