/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2012-2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * Configuration settings for the Allwinner sunxi series of boards.
 */

#ifndef _SUNXI_COMMON_CONFIG_H
#define _SUNXI_COMMON_CONFIG_H

#include <asm/arch/cpu.h>
#include <linux/stringify.h>

#ifdef CONFIG_ARM64
#define CONFIG_SYS_BOOTM_LEN		(32 << 20)
#endif

/* Serial & console */
#define CONFIG_SYS_NS16550_SERIAL
/* ns16550 reg in the low bits of cpu reg */
#define CONFIG_SYS_NS16550_CLK		24000000
#ifndef CONFIG_DM_SERIAL
# define CONFIG_SYS_NS16550_REG_SIZE	-4
# define CONFIG_SYS_NS16550_COM1		SUNXI_UART0_BASE
# define CONFIG_SYS_NS16550_COM2		SUNXI_UART1_BASE
# define CONFIG_SYS_NS16550_COM3		SUNXI_UART2_BASE
# define CONFIG_SYS_NS16550_COM4		SUNXI_UART3_BASE
# define CONFIG_SYS_NS16550_COM5		SUNXI_R_UART_BASE
#endif

/* CPU */
#define COUNTER_FREQUENCY		24000000
#define CONFIG_SUNXI_SRAM_ADDRESS              (0x00020000L)

/*
 * The DRAM Base differs between some models. We cannot use macros for the
 * CONFIG_FOO defines which contain the DRAM base address since they end
 * up unexpanded in include/autoconf.mk .
 *
 * So we have to have this #ifdef #else #endif block for these.
 */
#ifdef CONFIG_MACH_SUN9I
#define SDRAM_OFFSET(x) 0x2##x
#define CONFIG_SYS_SDRAM_BASE		0x20000000
/* Note SPL_STACK_R_ADDR is set through Kconfig, we include it here
 * since it needs to fit in with the other values. By also #defining it
 * we get warnings if the Kconfig value mismatches. */
#define CONFIG_SPL_BSS_START_ADDR	0x2ff80000
#else
#define SDRAM_OFFSET(x) 0x4##x
#define CONFIG_SYS_SDRAM_BASE		0x40000000
/* V3s do not have enough memory to place code at 0x4a000000 */
/* Note SPL_STACK_R_ADDR is set through Kconfig, we include it here
 * since it needs to fit in with the other values. By also #defining it
 * we get warnings if the Kconfig value mismatches. */
#define CONFIG_SPL_BSS_START_ADDR	0x4ff80000
#endif

#define CONFIG_SPL_BSS_MAX_SIZE		0x00080000 /* 512 KiB */

/*
 * The A80's A1 sram starts at 0x00010000 rather then at 0x00000000 and is
 * slightly bigger. Note that it is possible to map the first 32 KiB of the
 * A1 at 0x00000000 like with older SoCs by writing 0x16aa0001 to the
 * undocumented 0x008000e0 SYS_CTRL register. Where the 16aa is a key and
 * the 1 actually activates the mapping of the first 32 KiB to 0x00000000.
 * A64 and H5 also has SRAM A1 at 0x00010000, but no magic remap register
 * is known yet.
 * H6 has SRAM A1 at 0x00020000.
 */
#define CONFIG_SYS_INIT_RAM_ADDR	CONFIG_SUNXI_SRAM_ADDRESS
/* FIXME: this may be larger on some SoCs */
#define CONFIG_SYS_INIT_RAM_SIZE	0x8000 /* 32 KiB */

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define PHYS_SDRAM_0			CONFIG_SYS_SDRAM_BASE
//#define PHYS_SDRAM_0_SIZE		0x80000000 /* 2 GiB */
#define PHYS_SDRAM_0_SIZE		0x8000000 /* 128 MiB */

#ifdef CONFIG_AHCI
#define CONFIG_SYS_64BIT_LBA
#endif

#ifdef CONFIG_NAND_SUNXI
#define CONFIG_SYS_NAND_MAX_ECCPOS 1664
#define CONFIG_SYS_MAX_NAND_DEVICE 8
#endif

/* mmc config */
#ifdef CONFIG_MMC
#define CONFIG_MMC_SUNXI_SLOT		0
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)

#ifdef CONFIG_ARM64
/*
 * This is actually (CONFIG_ENV_OFFSET -
 * (CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)), but the value will be used
 * directly in a makefile, without the preprocessor expansion.
 */
#define CONFIG_BOARD_SIZE_LIMIT		0x7e000
#endif

#define CONFIG_SYS_MMC_MAX_DEVICE	4
#endif

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_CBSIZE	1024	/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE	1024	/* Print Buffer Size */

/* standalone support */
#define CONFIG_STANDALONE_LOAD_ADDR	CONFIG_SYS_LOAD_ADDR

/* FLASH and environment organization */

#define CONFIG_SYS_MONITOR_LEN		(768 << 10)	/* 768 KiB */

#define CONFIG_SPL_BOARD_LOAD_IMAGE

/*
 * We cannot use expressions here, because expressions won't be evaluated in
 * autoconf.mk.
 */
#if CONFIG_SUNXI_SRAM_ADDRESS == 0x10000
#define CONFIG_SPL_MAX_SIZE		0x7fa0		/* 32 KiB */
#ifdef CONFIG_ARM64
/* end of SRAM A2 for now, as SRAM A1 is pretty tight for an ARM64 build */
#define LOW_LEVEL_SRAM_STACK		0x00054000
#else
#define LOW_LEVEL_SRAM_STACK		0x00018000
#endif /* !CONFIG_ARM64 */
#elif CONFIG_SUNXI_SRAM_ADDRESS == 0x20000
#ifdef CONFIG_MACH_SUN50I_H616
#define CONFIG_SPL_MAX_SIZE		0xbfa0		/* 48 KiB */
#define LOW_LEVEL_SRAM_STACK		0x58000
#else
#define CONFIG_SPL_MAX_SIZE		0x7fa0		/* 32 KiB */
/* end of SRAM A2 on H6 for now */
#define LOW_LEVEL_SRAM_STACK		0x00118000
#endif
#else
#define CONFIG_SPL_MAX_SIZE		0x5fa0		/* 24KB on sun4i/sun7i */
#define LOW_LEVEL_SRAM_STACK		0x00008000	/* End of sram */
#endif

#define CONFIG_SPL_STACK		LOW_LEVEL_SRAM_STACK

#ifndef CONFIG_MACH_SUN50I_H616
#define CONFIG_SPL_PAD_TO		32768		/* decimal for 'dd' */
#endif

/* Ethernet support */

#ifdef CONFIG_USB_EHCI_HCD
#define CONFIG_USB_OHCI_NEW
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS 1
#endif

#ifndef CONFIG_SPL_BUILD

#ifdef CONFIG_ARM64
/*
 * Boards seem to come with at least 512MB of DRAM.
 * The kernel should go at 512K, which is the default text offset (that will
 * be adjusted at runtime if needed).
 * There is no compression for arm64 kernels (yet), so leave some space
 * for really big kernels, say 256MB for now.
 * Scripts, PXE and DTBs should go afterwards, leaving the rest for the initrd.
 */
#define BOOTM_SIZE        __stringify(0xa000000)
#define KERNEL_ADDR_R     __stringify(SDRAM_OFFSET(0080000))
#define KERNEL_COMP_ADDR_R __stringify(SDRAM_OFFSET(4000000))
#define KERNEL_COMP_SIZE  __stringify(0xb000000)
#define FDT_ADDR_R        __stringify(SDRAM_OFFSET(FA00000))
#define SCRIPT_ADDR_R     __stringify(SDRAM_OFFSET(FC00000))
#define PXEFILE_ADDR_R    __stringify(SDRAM_OFFSET(FD00000))
#define FDTOVERLAY_ADDR_R __stringify(SDRAM_OFFSET(FE00000))
#define RAMDISK_ADDR_R    __stringify(SDRAM_OFFSET(FF00000))

#else
/*
 * 160M RAM (256M minimum minus 64MB heap + 32MB for u-boot, stack, fb, etc.
 * 32M uncompressed kernel, 16M compressed kernel, 1M fdt,
 * 1M script, 1M pxe, 1M dt overlay and the ramdisk at the end.
 */
#ifndef CONFIG_MACH_SUN8I_V3S
#define BOOTM_SIZE        __stringify(0xa000000)
#define KERNEL_ADDR_R     __stringify(SDRAM_OFFSET(2000000))
#define FDT_ADDR_R        __stringify(SDRAM_OFFSET(3000000))
#define SCRIPT_ADDR_R     __stringify(SDRAM_OFFSET(3100000))
#define PXEFILE_ADDR_R    __stringify(SDRAM_OFFSET(3200000))
#define FDTOVERLAY_ADDR_R __stringify(SDRAM_OFFSET(3300000))
#define RAMDISK_ADDR_R    __stringify(SDRAM_OFFSET(3400000))
#else
/*
 * 64M RAM minus 2MB heap + 16MB for u-boot, stack, fb, etc.
 * 16M uncompressed kernel, 8M compressed kernel, 1M fdt,
 * 1M script, 1M pxe, 1M dt overlay and the ramdisk at the end.
 */
#define BOOTM_SIZE        __stringify(0x2e00000)
#define KERNEL_ADDR_R     __stringify(SDRAM_OFFSET(1000000))
#define FDT_ADDR_R        __stringify(SDRAM_OFFSET(1800000))
#define SCRIPT_ADDR_R     __stringify(SDRAM_OFFSET(1900000))
#define PXEFILE_ADDR_R    __stringify(SDRAM_OFFSET(1A00000))
#define FDTOVERLAY_ADDR_R __stringify(SDRAM_OFFSET(1B00000))
#define RAMDISK_ADDR_R    __stringify(SDRAM_OFFSET(1C00000))
#endif
#endif

#define MEM_LAYOUT_ENV_SETTINGS \
	"bootm_size=" BOOTM_SIZE "\0" \
	"kernel_addr_r=" KERNEL_ADDR_R "\0" \
	"fdt_addr_r=" FDT_ADDR_R "\0" \
	"scriptaddr=" SCRIPT_ADDR_R "\0" \
	"pxefile_addr_r=" PXEFILE_ADDR_R "\0" \
	"fdtoverlay_addr_r=" FDTOVERLAY_ADDR_R "\0" \
	"ramdisk_addr_r=" RAMDISK_ADDR_R "\0"

#ifdef CONFIG_ARM64

#define MEM_LAYOUT_ENV_EXTRA_SETTINGS \
	"kernel_comp_addr_r=" KERNEL_COMP_ADDR_R "\0" \
	"kernel_comp_size=" KERNEL_COMP_SIZE "\0"

#else

#define MEM_LAYOUT_ENV_EXTRA_SETTINGS ""

#endif

#define DFU_ALT_INFO_RAM \
	"dfu_alt_info_ram=" \
	"kernel ram " KERNEL_ADDR_R " 0x1000000;" \
	"fdt ram " FDT_ADDR_R " 0x100000;" \
	"ramdisk ram " RAMDISK_ADDR_R " 0x4000000\0"

#ifdef CONFIG_MMC
#if CONFIG_MMC_SUNXI_SLOT_EXTRA != -1
#define BOOTENV_DEV_MMC_AUTO(devtypeu, devtypel, instance)		\
	BOOTENV_DEV_MMC(MMC, mmc, 0)					\
	BOOTENV_DEV_MMC(MMC, mmc, 1)					\
	"bootcmd_mmc_auto="						\
		"if test ${mmc_bootdev} -eq 1; then "			\
			"run bootcmd_mmc1; "				\
			"run bootcmd_mmc0; "				\
		"elif test ${mmc_bootdev} -eq 0; then "			\
			"run bootcmd_mmc0; "				\
			"run bootcmd_mmc1; "				\
		"fi\0"

#define BOOTENV_DEV_NAME_MMC_AUTO(devtypeu, devtypel, instance) \
	"mmc_auto "

#define BOOT_TARGET_DEVICES_MMC(func) func(MMC_AUTO, mmc_auto, na)
#else
#define BOOT_TARGET_DEVICES_MMC(func) func(MMC, mmc, 0)
#endif
#else
#define BOOT_TARGET_DEVICES_MMC(func)
#endif

#ifdef CONFIG_AHCI
#define BOOT_TARGET_DEVICES_SCSI(func) func(SCSI, scsi, 0)
#else
#define BOOT_TARGET_DEVICES_SCSI(func)
#endif

#ifdef CONFIG_USB_STORAGE
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#ifdef CONFIG_CMD_PXE
#define BOOT_TARGET_DEVICES_PXE(func) func(PXE, pxe, na)
#else
#define BOOT_TARGET_DEVICES_PXE(func)
#endif

#ifdef CONFIG_CMD_DHCP
#define BOOT_TARGET_DEVICES_DHCP(func) func(DHCP, dhcp, na)
#else
#define BOOT_TARGET_DEVICES_DHCP(func)
#endif

/* FEL boot support, auto-execute boot.scr if a script address was provided */
#define BOOTENV_DEV_FEL(devtypeu, devtypel, instance) \
	"bootcmd_fel=" \
		"if test -n ${fel_booted} && test -n ${fel_scriptaddr}; then " \
			"echo '(FEL boot)'; " \
			"source ${fel_scriptaddr}; " \
		"fi\0"
#define BOOTENV_DEV_NAME_FEL(devtypeu, devtypel, instance) \
	"fel "

#define BOOT_TARGET_DEVICES(func) \
	func(FEL, fel, na) \
	BOOT_TARGET_DEVICES_MMC(func) \
	BOOT_TARGET_DEVICES_SCSI(func) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_DEVICES_PXE(func) \
	BOOT_TARGET_DEVICES_DHCP(func)

#ifdef CONFIG_OLD_SUNXI_KERNEL_COMPAT
#define BOOTCMD_SUNXI_COMPAT \
	"bootcmd_sunxi_compat=" \
		"setenv root /dev/mmcblk0p3 rootwait; " \
		"if ext2load mmc 0 0x44000000 uEnv.txt; then " \
			"echo Loaded environment from uEnv.txt; " \
			"env import -t 0x44000000 ${filesize}; " \
		"fi; " \
		"setenv bootargs console=${console} root=${root} ${extraargs}; " \
		"ext2load mmc 0 0x43000000 script.bin && " \
		"ext2load mmc 0 0x48000000 uImage && " \
		"bootm 0x48000000\0"
#else
#define BOOTCMD_SUNXI_COMPAT
#endif

#include <config_distro_bootcmd.h>

#ifdef CONFIG_USB_KEYBOARD
#define CONSOLE_STDIN_SETTINGS \
	"stdin=serial,usbkbd\0"
#else
#define CONSOLE_STDIN_SETTINGS \
	"stdin=serial\0"
#endif

#ifdef CONFIG_DM_VIDEO
#define CONSOLE_STDOUT_SETTINGS \
	"stdout=serial,vidconsole\0" \
	"stderr=serial,vidconsole\0"
#else
#define CONSOLE_STDOUT_SETTINGS \
	"stdout=serial\0" \
	"stderr=serial\0"
#endif

#ifdef CONFIG_MTDIDS_DEFAULT
#define SUNXI_MTDIDS_DEFAULT \
	"mtdids=" CONFIG_MTDIDS_DEFAULT "\0"
#else
#define SUNXI_MTDIDS_DEFAULT
#endif

#ifdef CONFIG_MTDPARTS_DEFAULT
#define SUNXI_MTDPARTS_DEFAULT \
	"mtdparts=" CONFIG_MTDPARTS_DEFAULT "\0"
#else
#define SUNXI_MTDPARTS_DEFAULT
#endif

#define PARTS_DEFAULT \
	"name=loader1,start=8k,size=32k,uuid=${uuid_gpt_loader1};" \
	"name=loader2,size=984k,uuid=${uuid_gpt_loader2};" \
	"name=esp,size=128M,bootable,uuid=${uuid_gpt_esp};" \
	"name=system,size=-,uuid=${uuid_gpt_system};"

#define UUID_GPT_ESP "c12a7328-f81f-11d2-ba4b-00a0c93ec93b"

#ifdef CONFIG_ARM64
#define UUID_GPT_SYSTEM "b921b045-1df0-41c3-af44-4c6f280d3fae"
#else
#define UUID_GPT_SYSTEM "69dad710-2ce4-4e3c-b16c-21a1d49abed3"
#endif

#define CONSOLE_ENV_SETTINGS \
	CONSOLE_STDIN_SETTINGS \
	CONSOLE_STDOUT_SETTINGS

#ifdef CONFIG_ARM64
#define FDTFILE "allwinner/" CONFIG_DEFAULT_DEVICE_TREE ".dtb"
#else
#define FDTFILE CONFIG_DEFAULT_DEVICE_TREE ".dtb"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONSOLE_ENV_SETTINGS \
	MEM_LAYOUT_ENV_SETTINGS \
	MEM_LAYOUT_ENV_EXTRA_SETTINGS \
	DFU_ALT_INFO_RAM \
	"fdtfile=" FDTFILE "\0" \
	"console=ttyS0,115200\0" \
	SUNXI_MTDIDS_DEFAULT \
	SUNXI_MTDPARTS_DEFAULT \
	BOOTCMD_SUNXI_COMPAT \
	BOOTENV
/*
    "uuid_gpt_esp=" UUID_GPT_ESP "\0" \
	"uuid_gpt_system=" UUID_GPT_SYSTEM "\0" \
	"partitions=" PARTS_DEFAULT "\0"
*/

#else /* ifndef CONFIG_SPL_BUILD */
#define CONFIG_EXTRA_ENV_SETTINGS
#endif

#endif /* _SUNXI_COMMON_CONFIG_H */
