/*
 * Copyright (C) 2015-2018 Glowforge, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <spi.h>
#include <fsl_esdhc.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx6-ddr.h>
#ifdef CONFIG_IMX_WATCHDOG
extern void hw_watchdog_init(void);
extern void hw_watchdog_reset(void);
#endif

#define PRINT_BOOT_STATS 1

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define SPI_CS_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
          PAD_CTL_SRE_FAST)

#define GPIO_INTAKE_PWM     IMX_GPIO_NR(4, 29)
#define GPIO_EXHAUST_PWM    IMX_GPIO_NR(2, 10)
#define GPIO_AIR_ASSIST_PWM IMX_GPIO_NR(2,  9)
#define GPIO_LASER_PWM      IMX_GPIO_NR(4, 30)
#define GPIO_WATER_ON       IMX_GPIO_NR(6,  8)
#define GPIO_12V_ON         IMX_GPIO_NR(5,  8)
#define GPIO_LASER_ON       IMX_GPIO_NR(2, 30)

#define GPIO_PIC_SPI_CS     IMX_GPIO_NR(4, 16)

struct input_gpios {
  int door1;
  int door2;
  int door_active_level;
  int button;
  int button_active_level;
};

static const struct input_gpios v13_input_gpios = {
  .door1  = IMX_GPIO_NR(4, 14),
  .door2  = IMX_GPIO_NR(1,  1),
  .door_active_level = 0,       /* active low */
  .button = IMX_GPIO_NR(4,  9),
  .button_active_level = 0      /* active low */
};

static const struct input_gpios v12_input_gpios = {
  .door1  = IMX_GPIO_NR(5,  5),
  .door2  = IMX_GPIO_NR(5,  6),
  .door_active_level = 1,       /* active high */
  .button = IMX_GPIO_NR(4,  5),
  .button_active_level = 0      /* active low */
};

static const struct input_gpios *ingpios = NULL;
static char board_rev_str[8] = {0};
static char board_rev_major_str[8] = {0};
static uint32_t boot_fuses = 0;
static char boot_fuses_str[5] = {0};

/* Boot statistics are stored in SRC_GPR[5-8] and persisted across resets. */
/* SRC_GPR[1-4] are used during low power sleep/wake, but these are free. */
struct boot_stats {
  u32 current_boot_flags;
  u32 num_boot_attempts;
  u32 num_software_resets;
  u32 num_wdog_timeouts;
};
static __iomem struct boot_stats *bootstats = (struct boot_stats *)(SRC_BASE_ADDR+0x30);

#define CURRENT_BOOT_POR          (1 << 0)
#define CURRENT_BOOT_SW_RESET     (1 << 1)
#define CURRENT_BOOT_WDOG_TIMEOUT (1 << 2)
#define CURRENT_BOOT_RECOVERY     (1 << 3)


void board_gpio_init(void)
{
  /* enable 12V, drive all fan PWMs low, turn on pump */
  gpio_direction_output(GPIO_INTAKE_PWM, 0);
  gpio_direction_output(GPIO_EXHAUST_PWM, 0);
  gpio_direction_output(GPIO_AIR_ASSIST_PWM, 0);
  gpio_direction_output(GPIO_LASER_PWM, 0);
  gpio_direction_output(GPIO_WATER_ON, 1);
  gpio_direction_output(GPIO_12V_ON, 1);
  gpio_direction_output(GPIO_LASER_ON, 0);
  gpio_direction_output(GPIO_PIC_SPI_CS, 1);
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* micro-SD card on SDHC2 (4 bit) */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD (Glowforge-specific) */
};

/* eMMC on SDHC3 (8 bit) */
static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t ecspi2_pads[] = {
  MX6_PAD_EIM_OE__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_EIM_CS1__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_EIM_CS0__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_DI0_DISP_CLK__GPIO4_IO16 | MUX_PAD_CTRL(SPI_CS_PAD_CTRL), /* /CS */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_spi(void)
{
  imx_iomux_v3_setup_multiple_pads(ecspi2_pads, ARRAY_SIZE(ecspi2_pads));
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
  return (bus == 1 && cs == 0) ? GPIO_PIC_SPI_CS : -1;
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC3 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    eMMC (on sdhc3)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD2
	 * 0x2                  SD3 (eMMC)
	 */

	switch (reg & 0x3) {
	case 0x1:
		imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	default:
		printf("Warning: BOOT_CFG[11:12] has unrecognized %u value!\n",
				reg & 0x3);
		return -EINVAL;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif


/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}


int board_early_init_f(void)
{
  board_gpio_init();
  setup_iomux_uart();
  setup_spi();
  return 0;
}


int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	return 0;
}


static void read_board_rev(void)
{
  /* Board revision stored in OCOTP_GP1. */
  /* If not present, check if a serial number has been programmed. */
  /* If so, assume v12. If not, assume v13. */
  struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;
  struct fuse_bank *bank = &ocotp->bank[4];
  struct fuse_bank4_regs *fuse = (struct fuse_bank4_regs *)bank->fuse_regs;
  u32 board_rev = readl(&fuse->gp1);

  if (board_rev == 0) {
    u32 serial_number = readl(&fuse->mac_addr_low);
    board_rev = (serial_number == 0) ? 0xD0 : 0xC0;
  }

  u32 board_rev_major = (board_rev >> 4) & 0xFF;
  u32 board_rev_minor = board_rev & 0x0F;
  snprintf(board_rev_str, sizeof(board_rev_str), "%d%d", board_rev_major, board_rev_minor);
  snprintf(board_rev_major_str, sizeof(board_rev_major_str), "%d", board_rev_major);

  if (board_rev_major >= 13) {
    ingpios = &v13_input_gpios;
  } else {
    ingpios = &v12_input_gpios;
  }
}


static void read_boot_fuses(void)
{
  /* Boot fuses stored in OCOTP_CFG4. */
  struct ocotp_regs *ocotp = (struct ocotp_regs *)OCOTP_BASE_ADDR;
  struct fuse_bank *bank = &ocotp->bank[0];
  struct fuse_bank0_regs *fuse = (struct fuse_bank0_regs *)bank->fuse_regs;
  boot_fuses = readl(&fuse->cfg4);
  snprintf(boot_fuses_str, sizeof(boot_fuses_str), "%x", boot_fuses);
}


static int doors_closed(void)
{
  int door1 = !!gpio_get_value(ingpios->door1);
  int door2 = !!gpio_get_value(ingpios->door2);
  return (door1 == ingpios->door_active_level &&
          door2 == ingpios->door_active_level);
}


static int button_pressed(void)
{
  int button = !!gpio_get_value(ingpios->button);
  return (button == ingpios->button_active_level);
}


static void set_button_led_color(u16 r, u16 g, u16 b)
{
  struct spi_slave *spi = spi_setup_slave(1, 0, 3000000, SPI_MODE_0);
  if (spi) {
    if (spi_claim_bus(spi) == 0) {
      /* works best if data is sent a byte at a time */
      const u8 data[] = {
        0x98, (r & 0xFF), (r >> 8) & 0x3,
        0x9A, (g & 0xFF), (g >> 8) & 0x3,
        0x9C, (b & 0xFF), (b >> 8) & 0x3
      };
      int i;
      for (i = 0; i < ARRAY_SIZE(data); i++) {
        u8 byte = data[i];
        spi_xfer(spi, 8, &byte, &byte, SPI_XFER_ONCE);
      }
      spi_release_bus(spi);
    }
    spi_free_slave(spi);
  }
}


#define BUTTON_HOLD_TIME_MAX_MS       5000
#define BUTTON_HOLD_CHECK_INTERVAL_MS 100
#define BUTTON_COLOR_RECOVERY         0x1ff, 0x000, 0x3ff
#define BUTTON_COLOR_OFF              0x000, 0x000, 0x000
static void hang_in_uboot(void)
{
  int btn_count = 0;
  int btn_released = 0;

  /* Hang until the button is held, then boot normally */
  printf("*** We are hanging here...\n");
  while (1) {
#ifdef CONFIG_IMX_WATCHDOG
    hw_watchdog_reset();
#endif
    if (button_pressed()) {
      /* If button has been held long enough, boot normally. */
      btn_count++;
      if (btn_released && btn_count == 3) {
        /* give engineer a chance to enter the bootloader */
        setenv_hex("bootdelay", 3);
        return;
      }
    } else {
      btn_released = 1;
      btn_count = 0;
    }
    mdelay(2000);
  }
}


static int need_to_enter_recovery(void)
{
  /* On POR, if the door is open and button is held, enter recovery. */
  /* Don't check the button on reboots, in the rare chance that the machine */
  /* reboots on purpose while a user is holding the button... */
  if (bootstats->current_boot_flags & CURRENT_BOOT_POR) {
    if (!doors_closed() && button_pressed()) {
      int hold_time_ms = 0;
      printf("*** Recovery boot requested by user; release button to enter\n");
      /* Set the button LED to purple */
      set_button_led_color(BUTTON_COLOR_RECOVERY);
      /* To prevent recovery boot if something is holding down the button, */
      /* require the button to be released within a few seconds */
      while (hold_time_ms < BUTTON_HOLD_TIME_MAX_MS) {
        /* If button is released, enter recovery mode. */
        if (!button_pressed()) { return 1; }
        mdelay(BUTTON_HOLD_CHECK_INTERVAL_MS);
        hold_time_ms += BUTTON_HOLD_CHECK_INTERVAL_MS;
      }
      /* Button was held down too long, ignore it. */
      printf("*** Button held too long, booting normally\n");
      set_button_led_color(BUTTON_COLOR_OFF);
      return 0;
    }
  }
  /* Otherwise, check if WDOG1 has timed out */
#ifdef CONFIG_IMX_WATCHDOG
  if (bootstats->current_boot_flags & CURRENT_BOOT_WDOG_TIMEOUT) {
    printf("*** Reboot caused by watchdog timeout\n");
    return 1;
  }
#endif
  return 0;
}


/* MSB 0bXXX10XXX -> usdhc3 */
/* LSB 0b011XXXXX -> eMMC */
#define EMMC_BOOT_FUSES 0x1060
#define EMMC_RECOVERY_PARTITION_STR __stringify(EMMC_MMCDEV) "." \
                                    __stringify(EMMC_MMCHWPART) ":" \
                                    __stringify(EMMC_MMCPART)
static void enter_recovery_mode(void)
{
  bootstats->current_boot_flags |= CURRENT_BOOT_RECOVERY;

  /* Set the button LED to purple */
  set_button_led_color(BUTTON_COLOR_RECOVERY);

  /* If fused for eMMC boot, and boot parition exists, boot into the recovery partition */
  /* If fused for SD boot, there is no recovery partition: */
  /* hang here and then boot normal OS if requested */

  /* get_device_and_partition returns partition number, or -1 if not found */
  if (((boot_fuses & EMMC_BOOT_FUSES) == EMMC_BOOT_FUSES) &&
      (get_device_and_partition("mmc", EMMC_RECOVERY_PARTITION_STR, NULL, NULL, 0) == 1)) {
    printf("*** Recovery partition detected\n");
    setenv("mmcroot", EMMC_MMCROOT);
    setenv("mmcdev", __stringify(EMMC_MMCDEV));
    setenv("mmchwpart", __stringify(EMMC_MMCHWPART));
    setenv("mmcpart", __stringify(EMMC_MMCPART));
    setenv("boot_recovery", "yes");
  } else {
    hang_in_uboot();
  }

  /* We are proceeding to boot, turn off purple LED */
  printf("*** Initiating recovery boot\n");
}


static void update_boot_stats(void)
{
  bootstats->num_boot_attempts++;
  bootstats->current_boot_flags = 0;
  u32 reset_cause = get_imx_reset_cause();
  if (reset_cause & 0x01) {
    bootstats->current_boot_flags |= CURRENT_BOOT_POR;
  }
#ifdef CONFIG_IMX_WATCHDOG
  if (reset_cause & 0x10) {
    struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;
    u32 wrsr = readw(&wdog->wrsr);
    if (wrsr & 0x2) {
      /* watchdog timeout: TOUT bit set */
      bootstats->current_boot_flags |= CURRENT_BOOT_WDOG_TIMEOUT;
      bootstats->num_wdog_timeouts++;
    }
    if (wrsr & 0x1) {
      /* software reset: SFTW bit set */
      bootstats->current_boot_flags |= CURRENT_BOOT_SW_RESET;
      bootstats->num_software_resets++;
    }
  }
#endif
}


int board_late_init(void)
{
  if (getenv_yesno("env_saved") == -1) {
    setenv("env_saved", "yes");
    saveenv();
  }

  update_boot_stats();
  read_board_rev();
  read_boot_fuses();

#ifdef PSTORE_SIZE
  setenv_hex("pstore_addr", gd->ram_top);
  setenv_hex("pstore_size", PSTORE_SIZE);
#endif

  setenv("board_rev", board_rev_str);
  setenv("board_rev_major", board_rev_major_str);
  setenv("boot_fuses", boot_fuses_str);
#ifdef CONFIG_IMX_WATCHDOG
  hw_watchdog_init();
#endif

  if (need_to_enter_recovery()) {
    enter_recovery_mode();
  }

#if PRINT_BOOT_STATS
  printf("Current boot flags: %08x\n", bootstats->current_boot_flags);
  printf("Boot attempts:      %d\n", bootstats->num_boot_attempts);
  printf("Software resets:    %d\n", bootstats->num_software_resets);
  printf("Watchdog timeouts:  %d\n", bootstats->num_wdog_timeouts);
#endif

  return 0;
}


int checkboard(void)
{
  read_board_rev();
  printf("Board: Glowforge Control Board v%s\n", board_rev_str);
  return 0;
}

