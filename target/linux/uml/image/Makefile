# 
# Copyright (C) 2006-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

define Image/Prepare
	cp $(LINUX_DIR)/linux $(KDIR)/vmlinux.elf
endef

define Image/Build/squashfs
	dd if=/dev/zero of=$(KDIR)/root.squashfs bs=1024k count=0 seek=$(CONFIG_TARGET_ROOTFS_PARTSIZE)
endef

define Image/Build
	$(call Image/Build/$(1))
	cp $(KDIR)/root.$(1) $(BIN_DIR)/$(IMG_PREFIX)-$(1).img
	cp $(KDIR)/vmlinux.elf $(BIN_DIR)/$(IMG_PREFIX)-vmlinux
endef

$(eval $(call BuildImage))
