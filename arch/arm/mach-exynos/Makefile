# arch/arm/mach-exynos/Makefile
#
# Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
#		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core

obj-$(CONFIG_ARCH_EXYNOS)	+= common.o
obj-$(CONFIG_ARCH_EXYNOS4)	+= clock-exynos4.o
obj-$(CONFIG_ARCH_EXYNOS5)	+= clock-exynos5.o
obj-$(CONFIG_CPU_EXYNOS4210)	+= clock-exynos4210.o
obj-$(CONFIG_SOC_EXYNOS4212)	+= clock-exynos4212.o

obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM_GENERIC_DOMAINS) += pm_domains.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o

obj-$(CONFIG_ARCH_EXYNOS)	+= pmu.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o

obj-$(CONFIG_EXYNOS4_MCT)	+= mct.o

obj-$(CONFIG_ARM_TRUSTZONE)	+= smc_readsfr.o
plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_smc_readsfr.o :=-Wa,-march=armv7-a$(plus_sec)
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_ARM_TRUSTZONE)	+= smc.o
plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_smc.o :=-Wa,-march=armv7-a$(plus_sec)

obj-$(CONFIG_BUSFREQ)           += busfreq.o
obj-$(CONFIG_BUSFREQ_OPP)       += dev.o
ifeq ($(CONFIG_BUSFREQ_OPP),y)
obj-$(CONFIG_ARCH_EXYNOS4)      += ppmu.o busfreq_opp_exynos4.o busfreq_opp_4x12.o
endif

# machine support

obj-$(CONFIG_MACH_SMDKC210)		+= mach-smdkv310.o
obj-$(CONFIG_MACH_SMDKV310)		+= mach-smdkv310.o
obj-$(CONFIG_MACH_ARMLEX4210)		+= mach-armlex4210.o
obj-$(CONFIG_MACH_UNIVERSAL_C210)	+= mach-universal_c210.o
obj-$(CONFIG_MACH_NURI)			+= mach-nuri.o
obj-$(CONFIG_MACH_ORIGEN)		+= mach-origen.o

obj-$(CONFIG_MACH_SMDK4212)		+= mach-smdk4x12.o
obj-$(CONFIG_MACH_SMDK4412)		+= mach-smdk4x12.o
obj-$(CONFIG_MACH_MIXTILE4X12)		+= mach-mixtile4x12.o mixtile4x12-gpio.o
obj-$(CONFIG_MACH_TINY4412)		+= mach-tiny4412.o tiny4412-lcds.o

obj-$(CONFIG_MACH_EXYNOS4_DT)		+= mach-exynos4-dt.o
obj-$(CONFIG_MACH_EXYNOS5_DT)		+= mach-exynos5-dt.o

# device support

obj-y					+= dev-uart.o
obj-$(CONFIG_ARCH_EXYNOS4)		+= dev-audio.o
obj-$(CONFIG_EXYNOS4_DEV_AHCI)		+= dev-ahci.o
obj-$(CONFIG_EXYNOS4_DEV_DWMCI)		+= dev-dwmci.o
obj-$(CONFIG_EXYNOS_DEV_DMA)		+= dma.o
obj-$(CONFIG_EXYNOS4_DEV_USB_OHCI)	+= dev-ohci.o
obj-$(CONFIG_EXYNOS_DEV_DRM)		+= dev-drm.o
obj-$(CONFIG_EXYNOS_DEV_SYSMMU)		+= dev-sysmmu.o
obj-$(CONFIG_ION_EXYNOS)		+= dev-ion.o
obj-$(CONFIG_EXYNOS4_DEV_FIMC_IS)       += dev-fimc-is.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMC_IS)     += setup-fimc-is.o
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_LITE)     += dev-fimc-lite.o
obj-$(CONFIG_EXYNOS_THERMAL)     	+= dev-tmu.o

obj-$(CONFIG_ARCH_EXYNOS)		+= setup-i2c0.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMC)	+= setup-fimc.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMD0)	+= setup-fimd0.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C1)	+= setup-i2c1.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C2)	+= setup-i2c2.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C3)	+= setup-i2c3.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C4)	+= setup-i2c4.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C5)	+= setup-i2c5.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C6)	+= setup-i2c6.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C7)	+= setup-i2c7.o
obj-$(CONFIG_EXYNOS4_SETUP_KEYPAD)	+= setup-keypad.o
obj-$(CONFIG_EXYNOS4_SETUP_SDHCI_GPIO)	+= setup-sdhci-gpio.o
obj-$(CONFIG_EXYNOS4_SETUP_USB_PHY)	+= setup-usb-phy.o
obj-$(CONFIG_EXYNOS_SETUP_SPI)		+= setup-spi.o
obj-$(CONFIG_EXYNOS4_SETUP_HDMI)	+= setup-hdmi.o
obj-$(CONFIG_EXYNOS_ASV)		+= asv.o asv-4x12.o
obj-$(CONFIG_EXYNOS_DVFS_NR_POLICY_OLD) += dynamic-dvfs-nr_running-hotplug.o
obj-$(CONFIG_EXYNOS_DVFS_NR_RUNNING_POLICY) += dynamic-dvfs-nr-bassed-hotplug.o
obj-$(CONFIG_EXYNOS_DVFS_NR_RUNNING_QUICK_POLICY) += dynamic-dvfs-nr-bassed-hotplug.o
