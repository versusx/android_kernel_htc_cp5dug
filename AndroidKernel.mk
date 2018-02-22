
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

KERNEL_ENABLE_EXFAT ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_FS" | sed 's/^\s*CONFIG_EXFAT_FS\s*=\s*//' )
KERNEL_EXFAT_PATH ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_PATH" | sed 's/^\s*CONFIG_EXFAT_PATH\s*=\s*\"//' | sed 's/\".*//' )
KERNEL_EXFAT_VERSION ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_VERSION" | sed 's/^\s*CONFIG_EXFAT_VERSION\s*=\s*\"//' | sed 's/\".*//' )
BUILD_PATH ?= $(shell pwd)

ifeq ($(USES_UNCOMPRESSED_KERNEL),true)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/Image
else
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
endif

$(KERNEL_OUT):hboot
	@echo "==== Start Kernel Compiling ... ===="

$(KERNEL_CONFIG): kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)

$(TARGET_PREBUILT_KERNEL) : $(KERNEL_OUT) $(KERNEL_CONFIG)
ifneq ($(TARGET_BUILD_PREBUILT),true)
ifeq ($(KERNEL_ENABLE_EXFAT), m)
	cp vendor/tuxera/exfat/tuxera_update_htc.sh kernel/
	cp vendor/tuxera/exfat/update_tuxera.sh kernel/
	cp vendor/tuxera/exfat/build_exfat.sh kernel/
	cp -rf vendor/tuxera/exfat/texfat kernel/fs/
	cp -rf vendor/tuxera/exfat/$(KERNEL_EXFAT_PATH) kernel/fs/
	mkdir -p $(KERNEL_OUT)/fs/$(KERNEL_EXFAT_PATH)
	# Update exFAT module after vmlinux but before modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- vmlinux
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT)
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT)
endif
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	$(warning "User-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -r
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -r
endif
else
	$(warning "NonUser-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -u
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -u
endif
endif

endif
endif
endif
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- zImage -j4
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- modules
	@-mkdir -p $(KERNEL_MODULES_OUT)
ifneq ($(TARGET_BUILD_PREBUILT),true)
ifeq ($(KERNEL_ENABLE_EXFAT), m)
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
	# Build exfat modules for DEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects/texfat.ko $(KERNEL_MODULES_OUT)/
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	# Build exfat modules for NonDebug-USER
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-user INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user/texfat.ko $(KERNEL_MODULES_OUT)/
else
	# Build exfat modules for NonDebug-USERDEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-userdebug INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug/texfat.ko $(KERNEL_MODULES_OUT)/
endif
endif
endif
endif
	@-find $(KERNEL_OUT) -name *.ko | xargs -I{} cp {} $(KERNEL_MODULES_OUT)
ifneq ($(TARGET_BUILD_PREBUILT),true)
ifeq ($(KERNEL_ENABLE_EXFAT), m)
	rm kernel/tuxera_update_htc.sh
	rm kernel/update_tuxera.sh
	rm kernel/build_exfat.sh
	rm -rf kernel/fs/texfat*
endif
endif

kernelheader:
	mkdir -p $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install
