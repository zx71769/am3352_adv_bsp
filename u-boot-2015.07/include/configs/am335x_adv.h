/*
 * am335x_adv.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *	======== For Advantech crop. ICG custom board ========
 */

#ifndef __CONFIG_AM335X_EVM_H
#define __CONFIG_AM335X_EVM_H

#include <configs/ti_am335x_common.h>

#ifndef CONFIG_SPL_BUILD
#ifndef CONFIG_FIT
# define CONFIG_FIT
#endif
# define CONFIG_TIMESTAMP
# define CONFIG_LZO
#endif

#define CONFIG_SYS_BOOTM_LEN		(16 << 20)

#define MACH_TYPE_TIAM335EVM		3589	/* Until the next sync */
#define CONFIG_MACH_TYPE		MACH_TYPE_TIAM335EVM
#define CONFIG_BOARD_LATE_INIT

/* Clock Defines */
#define V_OSCK				24000000  /* Clock output from T2 */
#define V_SCLK				(V_OSCK)

/* The ld-script location */
#define CONFIG_SYS_LDSCRIPT		"board/advantech/am335x/u-boot.lds"

/* Always 128 KiB env size */
#define CONFIG_ENV_SIZE			(128 << 10)

/* Enhance our eMMC support / experience. */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION

#ifdef CONFIG_NAND
#define NANDARGS \
	"mtdids=" MTDIDS_DEFAULT "\0" \
	"mtdparts=" MTDPARTS_DEFAULT "\0" \
	"nandargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${nandroot} " \
		"rootfstype=${nandrootfstype}\0" \
	"nandroot=ubi0:rootfs rw ubi.mtd=NAND.file-system,2048\0" \
	"nandrootfstype=ubifs rootwait=1\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		"nand read ${fdtaddr} NAND.u-boot-spl-os; " \
		"nand read ${loadaddr} NAND.kernel; " \
		"bootz ${loadaddr} - ${fdtaddr}\0"
/* UBIFS */
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_RBTREE
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_LZO
#else
#define NANDARGS ""
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#ifndef CONFIG_SPL_BUILD
#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	DEFAULT_MMC_TI_ARGS \
	"boot_fdt=try\0" \
	"bootpart=0:2\0" \
	"bootdir=/boot\0" \
	"bootfile=zImage\0" \
	"fdtfile=undefined\0" \
	"console=ttyS0,115200n8\0" \
	"partitions=" \
		"uuid_disk=${uuid_gpt_disk};" \
		"name=rootfs,start=2MiB,size=-,uuid=${uuid_gpt_rootfs}\0" \
	"optargs=\0" \
	"ramroot=/dev/ram0 rw\0" \
	"ramrootfstype=ext2\0" \
	"spiroot=/dev/mtdblock4 rw\0" \
	"spirootfstype=jffs2\0" \
	"spisrcaddr=0xe0000\0" \
	"spiimgsize=0x362000\0" \
	"spibusno=0\0" \
	"spiargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${spiroot} " \
		"rootfstype=${spirootfstype}\0" \
	"ramargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${ramroot} " \
		"rootfstype=${ramrootfstype}\0" \
	"loadramdisk=load mmc ${mmcdev} ${rdaddr} ramdisk.gz\0" \
	"loadimage=load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}\0" \
	"loadfdt=load mmc ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile}\0" \
	"mmcloados=run args_mmc; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdtaddr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"mmcboot=mmc dev ${mmcdev}; " \
		"if mmc rescan; then " \
			"echo SD/MMC found on device ${mmcdev};" \
			"run envboot; " \
			"if run loadimage; then " \
				"run mmcloados;" \
			"fi;" \
		"fi;\0" \
	"spiboot=echo Booting from spi ...; " \
		"run spiargs; " \
		"sf probe ${spibusno}:0; " \
		"sf read ${loadaddr} ${spisrcaddr} ${spiimgsize}; " \
		"bootz ${loadaddr}\0" \
	"ramboot=echo Booting from ramdisk ...; " \
		"run ramargs; " \
		"bootz ${loadaddr} ${rdaddr} ${fdtaddr}\0" \
	"findfdt=setenv fdtfile am335x-adv.dtb;\0"\
	"init_console=setenv console ttyO0,115200n8;\0" \
	NANDARGS \
	NETARGS \
	DFUARGS
#endif

#define CONFIG_BOOTCOMMAND \
	"run findfdt; " \
	"run init_console; " \
	"run mmcboot;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"run mmcboot;" \
	"run nandboot;"

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1		0x44e09000	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		0x48022000	/* UART1 */
#define CONFIG_SYS_NS16550_COM3		0x48024000	/* UART2 */
#define CONFIG_SYS_NS16550_COM4		0x481a6000	/* UART3 */
#define CONFIG_SYS_NS16550_COM5		0x481a8000	/* UART4 */
#define CONFIG_SYS_NS16550_COM6		0x481aa000	/* UART5 */
#define CONFIG_BAUDRATE			115200

/* PMIC support */
#define CONFIG_POWER_TPS65217
#define CONFIG_POWER_TPS65910

/* SPL */
#define CONFIG_SPL_POWER_SUPPORT
#define CONFIG_SPL_YMODEM_SUPPORT

/* Bootcount using the RTC block */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_AM33XX
#define CONFIG_SYS_BOOTCOUNT_BE

#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/am33xx/u-boot-spl.lds"

/* NAND: device related configs */
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
					 CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128*1024)
/* NAND: driver related configs */
#define CONFIG_NAND_OMAP_GPMC
#define CONFIG_NAND_OMAP_GPMC_PREFETCH
#define CONFIG_NAND_OMAP_ELM
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	NAND_LARGE_BADBLOCK_POS
#define CONFIG_SYS_NAND_ECCPOS\
					{ 2, 3, 4, 5, 6, 7, 8, 9, \
					 10, 11, 12, 13, 14, 15, 16, 17, \
					 18, 19, 20, 21, 22, 23, 24, 25, \
					 26, 27, 28, 29, 30, 31, 32, 33, \
					 34, 35, 36, 37, 38, 39, 40, 41, \
					 42, 43, 44, 45, 46, 47, 48, 49, \
					 50, 51, 52, 53, 54, 55, 56, 57, }

#define CONFIG_SYS_NAND_ECCSIZE		512
#define CONFIG_SYS_NAND_ECCBYTES	14
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_NAND_OMAP_ECCSCHEME	OMAP_ECC_BCH8_CODE_HW
#define MTDIDS_DEFAULT			"nand0=nand.0"
#define MTDPARTS_DEFAULT		"mtdparts=nand.0:" \
					"128k(NAND.SPL)," \
					"128k(NAND.SPL.backup1)," \
					"128k(NAND.SPL.backup2)," \
					"128k(NAND.SPL.backup3)," \
					"256k(NAND.u-boot-spl-os)," \
					"1m(NAND.u-boot)," \
					"128k(NAND.u-boot-env)," \
					"128k(NAND.u-boot-env.backup1)," \
					"8m(NAND.kernel)," \
					"-(NAND.file-system)"
#define CONFIG_SYS_NAND_U_BOOT_OFFS			0x000c0000
/* Advantech U-Boot Nand */
#define CONFIG_SYS_ADV_NAND_U_BOOT_OFFS		0x000c0000
#define CONFIG_SYS_ADV_NAND_U_BOOT_SIZE		0x000100000
/* NAND: SPL related configs */
#ifdef CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_AM33XX_BCH
#endif
#ifdef CONFIG_SPL_OS_BOOT
#define CONFIG_CMD_SPL_NAND_OFS	0x00080000 /* os parameters */
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS	0x00200000 /* kernel offset */
#define CONFIG_CMD_SPL_WRITE_SIZE	0x2000
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_USBETH_SUPPORT)
/* Remove other SPL modes. */
#undef CONFIG_SPL_YMODEM_SUPPORT
#undef CONFIG_SPL_NAND_SUPPORT
#undef CONFIG_SPL_MMC_SUPPORT
#define CONFIG_ENV_IS_NOWHERE
#undef CONFIG_ENV_IS_IN_NAND
/* disable EFI partitions and partition UUID support */
#undef CONFIG_PARTITION_UUIDS
#undef CONFIG_EFI_PARTITION
/* General network SPL  */
#define CONFIG_SPL_NET_SUPPORT
#define CONFIG_SPL_ENV_SUPPORT
#define CONFIG_SPL_NET_VCI_STRING	"AM335x U-Boot SPL"
#endif

/* USB Device Firmware Update support */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_DFU_FUNCTION
#define CONFIG_DFU_MMC
#define CONFIG_CMD_DFU
#define DFU_ALT_INFO_MMC \
	"dfu_alt_info_mmc=" \
	"boot part 0 1;" \
	"rootfs part 0 2;" \
	"MLO fat 0 1;" \
	"MLO.raw raw 0x100 0x100;" \
	"u-boot.img.raw raw 0x300 0x400;" \
	"spl-os-args.raw raw 0x80 0x80;" \
	"spl-os-image.raw raw 0x900 0x2000;" \
	"spl-os-args fat 0 1;" \
	"spl-os-image fat 0 1;" \
	"u-boot.img fat 0 1;" \
	"uEnv.txt fat 0 1\0"
#define CONFIG_DFU_NAND
#define DFU_ALT_INFO_NAND \
	"dfu_alt_info_nand=" \
	"SPL part 0 1;" \
	"SPL.backup1 part 0 2;" \
	"SPL.backup2 part 0 3;" \
	"SPL.backup3 part 0 4;" \
	"u-boot part 0 5;" \
	"u-boot-spl-os part 0 6;" \
	"kernel part 0 8;" \
	"rootfs part 0 9\0"
#define CONFIG_DFU_RAM
#define DFU_ALT_INFO_RAM \
	"dfu_alt_info_ram=" \
	"kernel ram 0x80200000 0xD80000;" \
	"fdt ram 0x80F80000 0x80000;" \
	"ramdisk ram 0x81000000 0x4000000\0"
#define DFUARGS \
	"dfu_alt_info_emmc=rawemmc raw 0 3751936\0" \
	DFU_ALT_INFO_MMC \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_NAND
#endif

/*
 * Default to using SPI for environment, etc.
 * 0x000000 - 0x020000 : SPL (128KiB)
 * 0x020000 - 0x0A0000 : U-Boot (512KiB)
 * 0x0A0000 - 0x0BFFFF : First copy of U-Boot Environment (128KiB)
 * 0x0C0000 - 0x0DFFFF : Second copy of U-Boot Environment (128KiB)
 * 0x0E0000 - 0x442000 : Linux Kernel
 * 0x442000 - 0x800000 : Userland
 */
#if defined(CONFIG_SPI_BOOT)
/* SPL related */
#undef CONFIG_SPL_OS_BOOT		/* Not supported by existing map */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SECT_SIZE		(4 << 10) /* 4 KB sectors */
#define CONFIG_ENV_OFFSET		(768 << 10) /* 768 KiB in */
#define CONFIG_ENV_OFFSET_REDUND	(896 << 10) /* 896 KiB in */
#define MTDIDS_DEFAULT			"nor0=m25p80-flash.0"
#define MTDPARTS_DEFAULT		"mtdparts=m25p80-flash.0:128k(SPL)," \
					"512k(u-boot),128k(u-boot-env1)," \
					"128k(u-boot-env2),3464k(kernel)," \
					"-(rootfs)"
#elif defined(CONFIG_EMMC_BOOT)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SPL_ENV_SUPPORT
#define CONFIG_SYS_MMC_ENV_DEV		1
#define CONFIG_SYS_MMC_ENV_PART		2
#define CONFIG_ENV_OFFSET		0x0
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#elif defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_ENV_OFFSET		0x001c0000
#define CONFIG_ENV_OFFSET_REDUND	0x001e0000
#define CONFIG_SYS_ENV_SECT_SIZE	CONFIG_SYS_NAND_BLOCK_SIZE
#elif !defined(CONFIG_ENV_IS_NOWHERE)
/* Not NAND, SPI, NOR or eMMC env, so put ENV in a file on FAT */
#define CONFIG_ENV_IS_IN_FAT
#define FAT_ENV_INTERFACE		"mmc"
#define FAT_ENV_DEVICE_AND_PART		"0:1"
#define FAT_ENV_FILE			"uboot.env"
#endif

/* SPI flash. */
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SF_DEFAULT_SPEED		24000000

/* Network. */
#define CONFIG_PHY_GIGE
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

#ifdef CONFIG_DRIVER_TI_CPSW
#define CONFIG_CLOCK_SYNTHESIZER
#define CLK_SYNTHESIZER_I2C_ADDR 0x65
#endif

#endif	/* ! __CONFIG_AM335X_EVM_H */
