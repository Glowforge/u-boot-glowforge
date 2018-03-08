/*
 * Copyright (C) 2015-2018 Glowforge, Inc.
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Adapted from the configuration settings for the Freescale i.MX6Q SabreSD board.
 * All peripherals other than the serial port (video, ethernet, USB, I2C, etc.) are disabled.
 * Serial console available on UART1.
 *
 * SPDX-License-Identifier:  GPL-2.0+
 */

#ifndef __GLOWFORGE_CONFIG_H
#define __GLOWFORGE_CONFIG_H

/* Reserve top 1MB for kernel ramoops persistent store */
#define PSTORE_SIZE 0x100000

#ifdef PSTORE_SIZE
#define CONFIG_SYS_MEM_TOP_HIDE PSTORE_SIZE
#define PSTORE_ARGS \
  " ramoops.mem_address=0x${pstore_addr}" \
  " ramoops.mem_size=0x${pstore_size}" \
  " ramoops.record_size=0x2000" \
  " ramoops.console_size=0x2000" \
  " ramoops.ecc=1" \
  " ramoops.dump_oops=1"
#else
#define PSTORE_ARGS ""
#endif

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_BOOTDELAY 0

#define PHYS_SDRAM_SIZE       (512u * 1024 * 1024)
#define CONFIG_MXC_UART_BASE  UART1_BASE
#define CONFIG_CONSOLE_DEV    "ttymxc0"

/* Do not define CONFIG_HW_WATCHDOG; */
/* we want the machine to reset if it winds up in the bootloader prompt. */
/* `run wdr` can be typed at the bootloader prompt to reset the timer. */
#define CONFIG_IMX_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 60000

#define CONFIG_MXC_SPI

/*** This section adapted from mx6sabre_common.h ***/
#include "mx6_common.h"

#define CONFIG_IMX6_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN (9 * SZ_1M)
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR         0

/* support both SD card and eMMC */
#define CONFIG_SYS_FSL_USDHC_NUM          2

/* eMMC is mmc device 1 on usdhc 3. */
/* There are 3 hardware paritions, each can contain partitions:  */
/* 0: /dev/mmcblk2 (user partition)  */
/*   0: unnamed "partition" (u-boot) */
/*   1: /dev/mmcblk2p1 (rootfs 1)    */
/*   2: /dev/mmcblk2p2 (rootfs 2)    */
/*   3: /dev/mmcblk2p3 (data)        */
/* 1: /dev/mmcblk2boot0 (recovery)   */
/*   0: unnamed "partition" (u-boot) */
/*   1: /dev/mmcblk2boot0p1 (rootfs) */
/* 2: /dev/mmcblk2boot1              */
/* By default we boot into the recovery OS */
#define EMMC_MMCDEV    1
#define EMMC_MMCROOT   "/dev/mmcblk2boot0p1 rootfstype=squashfs"
#define EMMC_MMCHWPART 1
#define EMMC_MMCPART   1

/* sd is mmc device 0 on usdhc 2.    */
/* There are no hardware paritions   */
/* 0: /dev/mmcblk1                   */
/*   0: unnamed "partition" (u-boot) */
/*   1: /dev/mmcblk1p1 (rootfs 1)    */
/*   2: /dev/mmcblk1p2 (rootfs 2)    */
/*   3: /dev/mmcblk1p3 (data)        */
/* By default we boot into rootfs 1  */
#define SD_MMCDEV      0
#define SD_MMCROOT     "/dev/mmcblk1p1"
#define SD_MMCHWPART   0
#define SD_MMCPART     1

#if CONFIG_EMMC
/* Configure for booting from eMMC */
#define CONFIG_MMCDEV          EMMC_MMCDEV
#define CONFIG_MMCROOT         EMMC_MMCROOT
#define CONFIG_MMCHWPART       EMMC_MMCHWPART
#define CONFIG_MMCPART         EMMC_MMCPART
#define CONFIG_RECOVERY        "yes"
#else
/* Configure for booting from micro SD */
#define CONFIG_MMCDEV          SD_MMCDEV
#define CONFIG_MMCROOT         SD_MMCROOT
#define CONFIG_MMCHWPART       SD_MMCHWPART
#define CONFIG_MMCPART         SD_MMCPART
#define CONFIG_RECOVERY        "no"
#endif

#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION CONFIG_MMCPART
#define CONFIG_SYS_MMC_ENV_DEV             CONFIG_MMCDEV

#define CONFIG_SUPPORT_EMMC_BOOT

/* Command definition */

/* Enable "mtest" command for memory testing; use alternate test for better coverage. */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_START      CONFIG_SYS_SDRAM_BASE  /* Start memory location */
#define CONFIG_SYS_MEMTEST_END        0x26e00000             /* End memory location (assuming 512MB total) */
#define CONFIG_SYS_MEMTEST_SCRATCH    0x26e00004             /* 4-byte long location to allow writing anti-pattern to bus */

/* Environment organization */
#define CONFIG_ENV_SIZE      (8 * 1024)   /* 8 KB */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET    (1024 * 512) /* Located at 512 KB, block 1024 (0x400) */

#define CONFIG_ENV_OFFSET_REDUND \
    (CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)

#define CONFIG_RECOVERY_FDT_ADDR   "0x600"  /* Located at .75 MB, block 1536 */
#define CONFIG_RECOVERY_FDT_SIZE   "0x80"   /* 64 KB, 128 blocks */
#define CONFIG_RECOVERY_IMAGE_ADDR "0x800"  /* Located at 1 MB, block 2048 */
#define CONFIG_RECOVERY_IMAGE_SIZE "0x2800" /* 5 MB, 10240 blocks */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
  "script=boot.scr\0" \
  "bootenv=uEnv.txt\0" \
  "bootdir=/boot\0" \
  "image=zImage\0" \
  "fdt_addr=0x18000000\0" \
  "fdt_dev_addr=" CONFIG_RECOVERY_FDT_ADDR "\0" \
  "fdt_size=" CONFIG_RECOVERY_FDT_SIZE "\0" \
  "image_dev_addr=" CONFIG_RECOVERY_IMAGE_ADDR "\0" \
  "image_size=" CONFIG_RECOVERY_IMAGE_SIZE "\0" \
  "boot_fdt=try\0" \
  "boot_recovery=" CONFIG_RECOVERY "\0" \
  "console=" CONFIG_CONSOLE_DEV "\0" \
  "bootm_size=0x10000000\0" \
  "fdt_high=0xffffffff\0"    \
  "initrd_high=0xffffffff\0" \
  "mmcdev=" __stringify(CONFIG_MMCDEV) "\0" \
  "mmchwpart=" __stringify(CONFIG_MMCHWPART) "\0" \
  "mmcpart=" __stringify(CONFIG_MMCPART) "\0" \
  "mmcroot=" CONFIG_MMCROOT "\0" \
  "mmcargs=setenv bootargs console=${console},${baudrate} consoleblank=0 " \
    "root=${mmcroot} rw quiet" PSTORE_ARGS "\0" \
  "loadbootscript=" \
    "load mmc ${mmcdev}.${mmchwpart}:${mmcpart} ${loadaddr} ${bootdir}/${script};\0" \
  "bootscript=echo Running bootscript from mmc ...; " \
    "source\0" \
  "loadbootenv=load mmc ${mmcdev}.${mmchwpart}:${mmcpart} ${loadaddr} ${bootdir}/${bootenv};\0" \
  "importbootenv=echo Importing environment from mmc${mmcdev}.${mmchwpart} ...; " \
                "env import -t ${loadaddr} ${filesize};\0" \
  "loadimage=load mmc ${mmcdev}.${mmchwpart}:${mmcpart} ${loadaddr} ${bootdir}/${image}\0" \
  "loadimageaddr=mmc read ${loadaddr} ${image_dev_addr} ${image_size}\0" \
  "loadfdt=load mmc ${mmcdev}.${mmchwpart}:${mmcpart} ${fdt_addr} ${bootdir}/${fdt}\0" \
  "loadfdtaddr=mmc read ${fdt_addr} ${fdt_dev_addr} ${fdt_size}\0" \
  "mmcboot=echo Booting from mmc${mmcdev}.${mmchwpart} ...; " \
    "run mmcargs; " \
    "if test ${boot_recovery} = yes && run loadfdtaddr; then " \
      "bootz ${loadaddr} - ${fdt_addr}; " \
    "elif test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
      "setenv fdt_file_1 imx6dl-glowforge-v${board_rev}.dtb; " \
      "setenv fdt_file_2 imx6dl-glowforge-v${board_rev_major}.dtb; " \
      "for fdt in ${fdt_file} ${fdt_file_1} ${fdt_file_2}; do " \
        "if run loadfdt; then " \
          "bootz ${loadaddr} - ${fdt_addr}; " \
        "fi; " \
      "done; " \
      "echo WARN: Cannot load the device tree; " \
    "else " \
      "bootz; " \
    "fi;\0" \
   "wdr=mw.w 20bc002 5555; mw.w 20bc002 aaaa;\0"

#define CONFIG_BOOTCOMMAND \
  "mmc dev ${mmcdev} ${mmchwpart}; " \
  "if mmc rescan; then " \
    "if run loadbootenv; then " \
      "run importbootenv; " \
    "fi; " \
    "if test ${boot_recovery} = yes && run loadimageaddr; then " \
      "run mmcboot; " \
    "elif run loadimage; then " \
      "run mmcboot; " \
    "fi; " \
  "fi"

#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
       (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
       (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/*** End of section adapted from mx6sabre_common.h ***/

/*** Post-configuration to disable unneeded features ***/
#undef CONFIG_CMD_FAT
#undef CONFIG_FS_FAT
#undef CONFIG_EXT4_WRITE
#undef CONFIG_CMD_EXT4_WRITE
#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_VXWORKS
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#endif                         /* __GLOWFORGE_CONFIG_H */

