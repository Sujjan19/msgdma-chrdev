DRV_NAME		= msgdma

# Allow override from command line, provide defaults if not set
CROSS_COMPILE ?= aarch64-none-linux-gnu-
ARCH ?= arm64
KBUILD_DIR ?= /home/qnu/gsrd.dk_si_agi027fb/linux-socfpga
ROOTFS_DIR ?= /home/qnu/gsrd.dk_si_agi027fb/sd_card/rootfs
KERNEL_VERSION ?= 6.6.51-g0447da78ed3c-dirty

obj-m += $(DRV_NAME).o

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KBUILD_DIR) M=$(PWD) modules

install:
	sudo mkdir -p $(ROOTFS_DIR)/lib/modules/$(KERNEL_VERSION)/extra
	sudo cp $(DRV_NAME).ko /home/qnu/gsrd.dk_si_agi027fb/sd_card/rootfs/lib/modules/6.6.51-g0447da78ed3c-dirty/extra
	sudo depmod -b $(ROOTFS_DIR) $(KERNEL_VERSION)

clean:
	$(MAKE) -C $(KBUILD_DIR) M=$(PWD) clean
