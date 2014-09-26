/*
 * linux/arch/arm/mach-exynos4/mach-mixtile4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/lcd.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/serial_core.h>
#include <linux/lcd.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/input/pixcir_ts.h>
#include <linux/gpio_event.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/platform_data/exynos_thermal.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/dm9000.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/pm.h>
#include <linux/s3c_adc_battery.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>

#include <plat/regs-serial.h>
#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/keypad.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/fb.h>
#include <plat/regs-fb-v4.h>
#include <plat/regs-adc.h>
#include <plat/iic.h>
#include <plat/adc.h>
#include <plat/adc-core.h>
#include <plat/sdhci.h>
#include <plat/regs-fb-v4.h>
#include <plat/ehci.h>
#include <plat/fimc-core.h>
#include <plat/mfc.h>
#include <plat/pm.h>
#include <plat/hdmi.h>
#include <plat/camport.h>
#include <plat/fimg2d.h>
#include <plat/regs-srom.h>
#include <plat/sysmmu.h>
#include <plat/tv-core.h>
#include <plat/watchdog-reset.h>

#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>
#include <media/v4l2-mediabus.h>
#include <media/s5p_fimc.h>
#include <media/mixtile_camera.h>

#include <video/platform_lcd.h>

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#include <mach/dwmci.h>
#endif

#include <mach/map.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>
#include <mach/dev.h>
#include <mach/ppmu.h>
#include <mach/ohci.h>
#include <mach/sysmmu.h>
#include <mach/s3cfb.h>

#include <../../../drivers/staging/android/timed_gpio.h>

#include <mach/gpio-mixtile4x12.h>
#include "mixtile4x12.h"

#include "common.h"

int dev_ver=0;
EXPORT_SYMBOL(dev_ver);

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

extern void mixtile4x12_config_gpio_table(void);
#define REG_INFORM4            (S5P_INFORM4)

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK4X12_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK4X12_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK4X12_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg mixtile4x12_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK4X12_UCON_DEFAULT,
		.ulcon		= SMDK4X12_ULCON_DEFAULT,
		.ufcon		= SMDK4X12_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_VIDEO_FIMC
#define WRITEBACK_ENABLED
#ifdef WRITEBACK_ENABLED
static struct i2c_board_info writeback_i2c_info = {
  I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
  .id    = CAMERA_WB,
  .fmt    = ITU_601_YCBCR422_8BIT,
  .order422  = CAM_ORDER422_8BIT_CBYCRY,
  .i2c_busnum  = 0,
  .info    = &writeback_i2c_info,
  .pixelformat  = V4L2_PIX_FMT_YUV444,
  .line_length  = 800,
  .width    = 480,
  .height    = 800,
  .window    = {
    .left  = 0,
    .top  = 0,
    .width  = 480,
    .height  = 800,
  },

  .initialized  = 0,
};
#endif /* WRITEBACK_ENABLED */

int mixtile4x12_cam_f_power(int onoff)
{
  if(onoff)
  {
    printk("CAM_FRONT power on\n");
		gpio_direction_output(GPIO_CAMERA_POWER, 1);
	  msleep(10);
		gpio_direction_output(GPIO_CAMERA_F_PWDN, 0);
	  msleep(10);
		gpio_direction_output(GPIO_CAMERA_F_RST, 1);
	  msleep(10);
 	}
	else
	{
    printk("CAM_FRONT power off\n");
		gpio_direction_output(GPIO_CAMERA_F_PWDN, 1);
		gpio_direction_output(GPIO_CAMERA_F_RST, 0);
		gpio_direction_output(GPIO_CAMERA_POWER, 0);
	}
	return 0;
}
EXPORT_SYMBOL(mixtile4x12_cam_f_power);

int mixtile4x12_cam_b_power(int onoff)
{
  if(onoff)
  {
    printk("CAM_BACK power on\n");
    gpio_direction_output(GPIO_CAMERA_POWER, 1);
    msleep(10);
    gpio_direction_output(GPIO_CAMERA_B_PWDN, 0);
    msleep(10);
    gpio_direction_output(GPIO_CAMERA_B_RST, 1);
  }
  else
  {
    printk("CAM_BACK power off\n");
		gpio_direction_output(GPIO_CAMERA_B_PWDN, 1);
		gpio_direction_output(GPIO_CAMERA_B_RST, 0);
		gpio_direction_output(GPIO_CAMERA_POWER, 0);
	}
	return 0;
}
EXPORT_SYMBOL(mixtile4x12_cam_b_power);

static int tvp5151_power(int onoff)
{
  if(onoff)
  {
    printk("TVP5151 power on\n");
    gpio_direction_output(GPIO_TVP5151_PWDN, 0);
    msleep(50);
		gpio_direction_output(GPIO_CAMERA_POWER, 1);
		gpio_direction_output(GPIO_TVP5151_PWDN, 1);
		 msleep(50);
		gpio_direction_output(GPIO_TVP5151_RST, 0);
    msleep(10);
		gpio_direction_output(GPIO_TVP5151_RST, 1);
	  msleep(10);
 	}
	else
	{
    printk("TVP5151 power off\n");
		gpio_direction_output(GPIO_TVP5151_PWDN, 0);
		gpio_direction_output(GPIO_TVP5151_RST, 1);
		gpio_direction_output(GPIO_CAMERA_POWER, 0);
	}
	return 0;
}
EXPORT_SYMBOL(tvp5151_power);

#ifdef CONFIG_VIDEO_GC2015
#define GC2015_WIDTH  800
#define GC2015_HEIGHT 600
static struct mixtile_camera_data gc2015_plat = {
	.default_width = GC2015_WIDTH,
	.default_height = GC2015_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info gc2015_i2c_info = {
	I2C_BOARD_INFO("GC2015", 0x30),
	.platform_data = &gc2015_plat,
};

static struct s3c_platform_camera gc2015 = {
	.id		= CAMERA_PAR_A,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 1,
	.cam_power	= mixtile4x12_cam_f_power,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &gc2015_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= GC2015_WIDTH,
	.width		= GC2015_WIDTH,
	.height		= GC2015_HEIGHT,
	.window		= {
		.left	= 0,
		.top	= 0,
  	.width		= GC2015_WIDTH,
  	.height		= GC2015_HEIGHT,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 1,
};
#endif /* CONFIG_VIDEO_GC2015 */

#ifdef CONFIG_VIDEO_TVP5150
#define TVP5151_WIDTH  720
#define TVP5151_HEIGHT 288
static struct mixtile_camera_data tvp5151_plat = {
	.default_width = TVP5151_WIDTH,
	.default_height = TVP5151_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 27000000,
	.is_mipi = 0,
};

static struct i2c_board_info tvp5151_i2c_info = {
	I2C_BOARD_INFO("tvp5150", 0x5D),
	.platform_data = &tvp5151_plat,
};

static struct s3c_platform_camera tvp5151 = {
  .id             = CAMERA_PAR_A,
  .clk_name       = "sclk_cam0",
  .i2c_busnum     = 1,
  .type           = CAM_TYPE_ITU,
  .fmt            = ITU_656_YCBCR422_8BIT,
  .order422       = CAM_ORDER422_8BIT_CBYCRY,
  .cam_power      = tvp5151_power,
  .info           = &tvp5151_i2c_info,
  .pixelformat    = V4L2_PIX_FMT_YUYV,
	.srclk_name	    = "xusbxti",
  .clk_rate       = 27000000,
//  .line_length    = 1920,
  .width          = TVP5151_WIDTH,
  .height         = TVP5151_HEIGHT,
  .window         = {
    .left   = 0,
    .top    = 0,
    .width  = TVP5151_WIDTH,
    .height = TVP5151_HEIGHT,
  },
  
  /* Polarity */
  .inv_pclk       = 0,
  .inv_vsync      = 0,
  .inv_href       = 1,
  .inv_hsync      = 0,
  .initialized    = 0,
};
#endif /* CONFIG_VIDEO_TVP5150 */

#ifdef CONFIG_VIDEO_MT9P111
#define MT9P111_WIDTH  800
#define MT9P111_HEIGHT 600
static struct mixtile_camera_data mt9p111_plat = {
	.default_width = MT9P111_WIDTH,
	.default_height = MT9P111_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 48000000,
	.is_mipi = 0,
};

static struct i2c_board_info mt9p111_i2c_info = {
	I2C_BOARD_INFO("MT9P111", 0x3C),
	.platform_data = &mt9p111_plat,
};

static struct s3c_platform_camera mt9p111 = {
	.id		= CAMERA_PAR_B,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 1,
	.cam_power	= mixtile4x12_cam_b_power,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &mt9p111_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= MT9P111_WIDTH,
	.width		= MT9P111_WIDTH,
	.height		= MT9P111_HEIGHT,
	.window		= {
		.left	= 0,
		.top	= 0,
  	.width		= MT9P111_WIDTH,
  	.height		= MT9P111_HEIGHT,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 1,
};
#endif /* CONFIG_VIDEO_MT9P111 */

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
  .default_cam  = CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
  .default_cam  = CAMERA_PAR_B,
#endif
#ifdef WRITEBACK_ENABLED
  .default_cam  = CAMERA_WB,
#endif
  .camera    = {
#ifdef CONFIG_VIDEO_GC2015
		&gc2015,                //cam_id = 0;
#endif
#ifdef CONFIG_VIDEO_MT9P111
		&mt9p111,
#endif
#ifdef CONFIG_VIDEO_TVP5150
    &tvp5151,
#endif
#ifdef WRITEBACK_ENABLED
    &writeback,
#endif
  },
  .hw_ver    = 0x51,
};

static struct s3c_platform_fimc fimc_plat_htpc = {
#ifdef CONFIG_ITU_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#ifdef CONFIG_VIDEO_GC2015
		&gc2015,                //cam_id = 0;
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};
#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_LCD_MIXTILE
#ifdef CONFIG_FB_LCD_1024X768
  #define MIXTILE_LCD_WIDTH 1024
  #define MIXTILE_LCD_HEIGHT 768
#endif

#ifdef CONFIG_FB_LCD_1280X720
  #define MIXTILE_LCD_WIDTH 1280
  #define MIXTILE_LCD_HEIGHT 720
#endif

#ifdef CONFIG_FB_S3C_LG97_IPAD1
struct s3cfb_lcd mixtile_lcd = {
  .width = MIXTILE_LCD_WIDTH,
  .height = MIXTILE_LCD_HEIGHT,
  .bpp = 32,
  .freq = 60,

  .timing = {
    .h_fp = 48,
    .h_bp = 32,
    .h_sw = 80,
    .v_fp = 3,
    .v_fpe = 1,
    .v_bp = 15,
    .v_bpe = 1,
    .v_sw = 4,
  },

  .polarity = {
    .rise_vclk = 1,
    .inv_hsync = 1,
    .inv_vsync = 1,
    .inv_vden = 0,
  },
};
#endif /* CONFIG_FB_S3C_LG97_IPAD1 */

#ifdef CONFIG_FB_S3C_VGA
struct s3cfb_lcd mixtile_lcd = {
  .width = MIXTILE_LCD_WIDTH,
  .height = MIXTILE_LCD_HEIGHT,
  .bpp = 32,
  .freq = 60,

  .timing = {
    .h_fp = 110,
    .h_bp = 220,
    .h_sw = 40,
    .v_fp = 5,
    .v_fpe = 1,
    .v_bp = 20,
    .v_bpe = 1,
    .v_sw = 5,
  },

  .polarity = {
    .rise_vclk = 1,
    .inv_hsync = 1,
    .inv_vsync = 1,
    .inv_vden = 0,
  },
};
#endif /* CONFIG_FB_S3C_LG97_IPAD1 */

struct s3cfb_lcd mixtile_lcd_1080p = {
  .width = 1920,
  .height = 1080,
  .bpp = 32,
  .freq = 60,

  .timing = {
    .h_fp = 1,
    .h_bp = 1,
    .h_sw = 1,
    .v_fp = 1,
    .v_fpe = 1,
    .v_bp = 1,
    .v_bpe = 1,
    .v_sw = 1,
  },

  .polarity = {
    .rise_vclk = 0,
    .inv_hsync = 1,
    .inv_vsync = 1,
    .inv_vden = 0,
  },
};

struct s3cfb_lcd mixtile_lcd_720p = {
  .width = 1280,
  .height = 720,
  .bpp = 32,
  .freq = 60,

  .timing = {
    .h_fp = 1,
    .h_bp = 1,
    .h_sw = 1,
    .v_fp = 1,
    .v_fpe = 1,
    .v_bp = 1,
    .v_bpe = 1,
    .v_sw = 1,
  },

  .polarity = {
    .rise_vclk = 0,
    .inv_hsync = 1,
    .inv_vsync = 1,
    .inv_vden = 0,
  },
};

#endif /* CONFIG_FB_LCD_MIXTILE */
#endif /* CONFIG_FB_S5P */

#ifdef CONFIG_SWITCH_HEADPHONE
#include <linux/switch.h>
static struct gpio_switch_platform_data hp_switch_data = {
	.name = "h2w",
	.gpio = GPIO_HEADPHONE,
	.name_on = "PULL IN",
	.name_off = "PULL OUT",
};
			
struct platform_device hp_switch_device = {
	.name	= "switch-hp",
	.dev = {
	   .platform_data    = &hp_switch_data,
	   .parent = &s3c_device_adc.dev,
	 },
};
#endif /* CONFIG_SWITCH_HEADPHONE */

static int exynos4_notifier_call(struct notifier_block *this,  unsigned long code, void *_cmd)
{
  int mode = 0;

  if ((code == SYS_RESTART) && _cmd)
    if (!strcmp((char *)_cmd, "recovery"))
      mode = 0xf;

  __raw_writel(mode, REG_INFORM4);

  return NOTIFY_DONE;
}

static struct notifier_block exynos4_reboot_notifier = {
  .notifier_call = exynos4_notifier_call,
};

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata mixtile4x12_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata mixtile4x12_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata mixtile4x12_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata mixtile4x12_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
};
#endif

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	default:
		break;
	}

	gpio = EXYNOS4_GPK0(2);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};
#endif


// static struct s5p_platform_mipi_csis mipi_csis_platdata = {
// #ifdef CONFIG_VIDEO_S5K6A3
// 	.clk_rate	= 160000000UL,
// 	.lanes		= 1,
// 	.alignment	= 24,
// 	.hs_settle	= 12,
// 	.phy_enable	= s5p_csis_phy_enable,
// #endif
// #ifdef CONFIG_VIDEO_M5MOLS
// 	.clk_rate	= 166000000UL,
// 	.lanes		= 2,
// 	.alignment	= 32,
// 	.hs_settle	= 12,
// 	.phy_enable	= s5p_csis_phy_enable,
// #endif
// };
// #define GPIO_CAM_LEVEL_EN(n)	EXYNOS4_GPX1(2)
// #define GPIO_CAM_8M_ISP_INT	EXYNOS4_GPX3(3)	/* XEINT_27 */
// #define GPIO_CAM_MEGA_nRST	EXYNOS4_GPX1(2) 
// static int m5mols_set_power(struct device *dev, int on)
// {
// 	gpio_set_value(EXYNOS4_GPX1(2), !on);
// 	gpio_set_value(EXYNOS4_GPX1(2), !!on);
// 	return 0;
// }
// static struct m5mols_platform_data m5mols_platdata = {
// 	.gpio_reset	= GPIO_CAM_MEGA_nRST,
// 	.reset_polarity	= 0,
// 	.set_power	= m5mols_set_power,
// };
// static struct i2c_board_info m5mols_board_info = {
// 	I2C_BOARD_INFO("M5MOLS", 0x1F),
// 	.platform_data = &m5mols_platdata,
// };
// 
// #ifdef CONFIG_VIDEO_S5K6A3
// static struct i2c_board_info s5k6a3_sensor_info = {
//         .type = "S5K6A3",
// };
// #endif
// 
// #ifdef CONFIG_VIDEO_S5K6A3
// static int smdk4x12_cam1_reset(int dummy) 
// {
//         int err;
// 
//         /* Camera B */
// 
//         err = gpio_request(EXYNOS4_GPX1(0), "GPX1");
//         if (err)
//                 printk(KERN_ERR "#### failed to request GPX1_0 ####\n");
// 
//         s3c_gpio_setpull(EXYNOS4_GPX1(0), S3C_GPIO_PULL_NONE);
//         gpio_direction_output(EXYNOS4_GPX1(0), 0);
//         gpio_direction_output(EXYNOS4_GPX1(0), 1);
//         gpio_free(EXYNOS4_GPX1(0));
// 
//         return 0;
// }
// 
// static struct s3c_platform_camera s5k6a3 = {
//         .id             = CAMERA_CSI_D,
//         .clk_name       = "sclk_cam1",
//         .cam_power      = smdk4x12_cam1_reset,
//         .type           = CAM_TYPE_MIPI,
//         .fmt            = MIPI_CSI_RAW10,
//         .order422       = CAM_ORDER422_8BIT_YCBYCR,
//         .pixelformat    = V4L2_PIX_FMT_UYVY,
//         .line_length    = 1920,
//         .width          = 1920,
//         .height         = 1080,
//         .window         = {
//                 .left   = 0,
//                 .top    = 0,
//                 .width  = 1920,
//                 .height = 1080,
//         },
//         .srclk_name     = "xusbxti",
//         .clk_rate       = 24000000,
//         .mipi_lanes     = 1,
//         .mipi_settle    = 12,
//         .mipi_align     = 24,
// 
//         .initialized    = 0,
//         .flite_id       = FLITE_IDX_B,
//         .use_isp        = true,
//         .sensor_index   = 102,
// 	.type  		= CAM_TYPE_MIPI,
//         .use_isp 	= true,
//         .inv_pclk 	= 0,
//         .inv_vsync 	= 0,
//         .inv_href 	= 0,
//         .inv_hsync 	= 0,
// };
// 
// 
// static struct s3c_platform_fimc fimc_plat = {
// 	.default_cam    = CAMERA_CSI_D,
// 	.camera         = {
// 			&s5k6a3,
// 	},
// };
// #endif
// static struct s5p_fimc_isp_info smdk4x12_camera_sensors[] = {
// #ifdef CONFIG_VIDEO_S5K6A3
// 	{
//                 .board_info     = &s5k6a3_sensor_info,
//                 .clk_frequency  = 24000000UL,
//                 .bus_type       = FIMC_MIPI_CSI2,
// 		.i2c_bus_num    = 1,
//                 .mux_id         = 1, /* A-Port : 0, B-Port : 1 */
//                 .flite_id       = FLITE_IDX_B,
//                 .cam_power      = smdk4x12_cam1_reset,
// 		.flags          = 0,
//                 .csi_data_align = 24,
//                 .use_isp        = true,
//         },
// #endif
// #ifdef CONFIG_VIDEO_M5MOLS
// 	{
// 		.mux_id		= 0,
// 		.flags		= V4L2_MBUS_PCLK_SAMPLE_FALLING |
// 				  V4L2_MBUS_VSYNC_ACTIVE_LOW,
// 		.bus_type	= FIMC_MIPI_CSI2,
// 		.board_info	= &m5mols_board_info,
// 		.i2c_bus_num	= 4,
// 		.clk_frequency	= 24000000UL,
// 		.csi_data_align	= 32,
// 	},
// #endif
// };
// static struct s5p_platform_fimc fimc_md_platdata = {
// 	.isp_info	= smdk4x12_camera_sensors,
// 	.num_clients	= ARRAY_SIZE(smdk4x12_camera_sensors),
// #ifdef CONFIG_VIDEO_S5K6A3
// 	.fimc_plat	= &fimc_plat,
// #endif
// };
// 
// static struct gpio smdk4x12_camera_gpios[] = {
// 	{ GPIO_CAM_8M_ISP_INT,	GPIOF_IN,            "8M_ISP_INT"  },
// 	{ GPIO_CAM_MEGA_nRST,	GPIOF_OUT_INIT_LOW,  "CAM_8M_NRST" },
// };
// static void __init smdk4x12_camera_init(void)
// {
// 	s3c_set_platdata(&mipi_csis_platdata, sizeof(mipi_csis_platdata),
// 			 &s5p_device_mipi_csis0);
// 	s3c_set_platdata(&mipi_csis_platdata, sizeof(mipi_csis_platdata),
//                          &s5p_device_mipi_csis1);
// 	s3c_set_platdata(&fimc_md_platdata,  sizeof(fimc_md_platdata),
// 			 &s5p_device_fimc_md);
// 	
// 
// 	if (gpio_request_array(smdk4x12_camera_gpios,
// 			       ARRAY_SIZE(smdk4x12_camera_gpios))) {
// 		pr_err("%s: GPIO request failed\n", __func__);
// 		return;
// 	}
// 
// 	if (!s3c_gpio_cfgpin(GPIO_CAM_8M_ISP_INT, S3C_GPIO_SFN(0xf)))
// 	{
//         	s3c_gpio_setpull(GPIO_CAM_8M_ISP_INT, S3C_GPIO_PULL_NONE);
// 		m5mols_board_info.irq = gpio_to_irq(GPIO_CAM_8M_ISP_INT);
// 	}
// 	else
// 		pr_err("Failed to configure 8M_ISP_INT GPIO\n");
// 
// 	/* Free GPIOs controlled directly by the sensor drivers. */
// 	gpio_free(GPIO_CAM_MEGA_nRST);
// 	gpio_free(GPIO_CAM_8M_ISP_INT);
// 
// 	if (exynos4_fimc_setup_gpio(S5P_CAMPORT_A))
// 		pr_err("Camera port A setup failed\n");
// }

/* USB OTG */
static struct s3c_hsotg_plat mixtile4x12_hsotg_pdata;

static struct platform_device exynos4_bus_devfreq = {
	.name		= "exynos4412-busfreq",
	.id			= 1,
};

/* USB EHCI */
static struct s5p_ehci_platdata mixtile4x12_ehci_pdata;

static void __init mixtile4x12_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &mixtile4x12_ehci_pdata;
	int err;

	s5p_ehci_set_platdata(pdata);

#define GPIO_USBH_RESET		EXYNOS4X12_GPM2(4)
	err = gpio_request_one(GPIO_USBH_RESET,
			GPIOF_OUT_INIT_HIGH, "USBH_RESET");
	if (err)
		pr_err("failed to request GPM2_4 for USB reset control\n");

	s3c_gpio_setpull(GPIO_USBH_RESET, S3C_GPIO_PULL_UP);
	gpio_set_value(GPIO_USBH_RESET, 0);
	mdelay(1);
	gpio_set_value(GPIO_USBH_RESET, 1);
	gpio_free(GPIO_USBH_RESET);
}

/* USB OHCI */
static struct exynos4_ohci_platdata mixtile4x12_ohci_pdata;

static void __init mixtile4x12_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &mixtile4x12_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

/* USB POWER */
void usb_hub_power(int onoff)
{
    gpio_direction_output(GPIO_USBHUB_POWER, onoff);
}

void usb_host_power(int onoff)
{
  gpio_direction_output(GPIO_USB5V_POWER, onoff);
}

#if defined(CONFIG_RTL8188CUS_MODULE) || defined(CONFIG_RTL8188EU_MODULE) 
void usb_wifi_power(int onoff)
{
  gpio_direction_output(GPIO_WIFI_POWER, onoff);
  printk("%s %s\n", __func__, (onoff? "on":"off"));
}
EXPORT_SYMBOL(usb_wifi_power);
#endif

void flash_led_power(int onoff, int mode)
{
  printk("flash_led_power %d %d\n", onoff, mode);
  gpio_direction_output(GPIO_FLASH_EN, onoff);
//  gpio_direction_output(GPIO_FLASH_MODE, 0);
  if(onoff == 0)
    s3c_gpio_cfgpin(GPIO_FLASH_EN, S3C_GPIO_SFN(2));
};

#ifdef CONFIG_LEDS_PWM
static struct led_pwm mixtile4x12_pwm_leds[] = {
	{
		.name		= "flashlight",
		.pwm_id		= 2,
		.max_brightness	= 255,
		.pwm_period_ns	= 700000,
	},
};

static struct led_pwm_platform_data mixtile4x12_pwm_data = {
	.num_leds	= ARRAY_SIZE(mixtile4x12_pwm_leds),
	.leds		= mixtile4x12_pwm_leds,
};

static struct platform_device mixtile4x12_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &mixtile4x12_pwm_data,
	},
};

void mixtile4x12_leds_pwm_init(void)
{
	platform_device_register(&s3c_device_timer[mixtile4x12_pwm_leds[0].pwm_id]);
};
#endif /* CONFIG_LEDS_PWM */

#ifdef GPIO_MOTOR_POWER
static struct timed_gpio timed_gpios[] = {
  {
    .name = "vibrator",
    .gpio = GPIO_MOTOR_POWER,
    .max_timeout = 15000,
    .active_low = 0,
  },
};

static struct timed_gpio_platform_data timed_gpio_data = {
  .num_gpios	= ARRAY_SIZE(timed_gpios),
  .gpios		= timed_gpios,
};

static struct platform_device mixtile4x12_timed_gpios = {
  .name   = "timed-gpio",
  .id     = -1,
  .dev    = {
    .platform_data = &timed_gpio_data,
  },
};
#endif /* GPIO_MOTOR_POWER */


/* I2C module and id for HDMIPHY */
static struct i2c_board_info mixtile4x12_i2c_hdmiphy[] __initdata = {
        { I2C_BOARD_INFO("hdmiphy-exynos4412", 0x38), }
};

static void s5p_tv_setup(void)
{
        /* direct HPD to External Interrupt */
        WARN_ON(gpio_request_one(EXYNOS4_GPX3(7), GPIOF_IN, "hpd-plug"));
        s3c_gpio_cfgpin(EXYNOS4_GPX3(7), S3C_GPIO_SFN(0xf));
        s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4_GPX3(7));
}

static struct i2c_board_info mixtile4x12_i2c_devs0[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_GT81XX
	{	I2C_BOARD_INFO("Goodix-TS", 0x55),},
#endif
};

static struct i2c_board_info mixtile4x12_i2c_devs1[] __initdata = {
#ifdef CONFIG_VIDEO_TVOUT
  {
    I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
  },
#endif
#ifdef CONFIG_SND_SOC_WM8960
	{	I2C_BOARD_INFO("wm8960", 0x1A), },
#endif
};

static struct i2c_board_info mixtile4x12_i2c_devs2[] __initdata = {
};

static struct i2c_board_info mixtile4x12_i2c_devs3[] __initdata = {
#ifdef CONFIG_SENSORS_MMA8452
	{ I2C_BOARD_INFO("mma8452", 0x1D), },
#endif
#ifdef CONFIG_SENSORS_AFA750
	{ I2C_BOARD_INFO("afa750", 0x7A>>1), },
#endif
#ifdef CONFIG_GYRO_L3G4200D
	{ I2C_BOARD_INFO("l3g4200d", 0x69), },
#endif
#ifdef CONFIG_COMPASS_MMC3280
	{ I2C_BOARD_INFO("mmc3280", 0x30), },
#endif
#ifdef CONFIG_LIGHT_BH1750
	{ I2C_BOARD_INFO("bh1750", 0x23), },
#endif
#ifdef CONFIG_SENSORS_APDS9900
	{ I2C_BOARD_INFO("apds9900", 0x39),},
#endif
};

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button mixtile_buttons[] = {
	{
		.gpio			= GPIO_POWER_KEY,
		.code			= KEY_POWER,
		.desc			= "Power",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_VOLUMEUP_KEY,
		.code			= KEY_VOLUMEUP,
		.desc			= "VolumeUp",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_VOLUMEDOWN_KEY,
		.code			= KEY_VOLUMEDOWN,
		.desc			= "VolumeDown",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_MEDIA_KEY,
		.code			= KEY_MEDIA,
		.desc			= "HeadsetHook",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_HOME_KEY,
		.code			= KEY_HOMEPAGE,
		.desc			= "Home",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_MENU_KEY,
		.code			= KEY_MENU,
		.desc			= "MENU",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
	{
		.gpio			= GPIO_BACK_KEY,
		.code			= KEY_ESC,
		.desc			= "ESC",
		.active_low		= 1,
		.debounce_interval	= 5,
		.type     = EV_KEY,
	},
};

static struct gpio_keys_platform_data mixtile_buttons_data  = {
	.buttons	= mixtile_buttons,
	.nbuttons	= ARRAY_SIZE(mixtile_buttons),
};

static struct platform_device mixtile4x12_buttons_device  = {
	.name		= "gpio-keys",
	.id		= 0,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &mixtile_buttons_data,
	}
};
#endif /* CONFIG_KEYBOARD_GPIO */

#ifdef CONFIG_BATTERY_MIXTILE
static struct s3c_adc_bat_pdata mixtile4x12_bat_cfg = {
  .volt_channel = 0,
};

static struct platform_device mixtile4x12_device_battery = {
	.name	= "mixtile_adc_bat",
	.id	= -1,
	.dev = {
	   .parent = &s3c_device_adc.dev,
	   .platform_data = &mixtile4x12_bat_cfg,
	 },

};
#endif

#ifdef CONFIG_DM9000
#define DM9000_PA (EXYNOS4_PA_SROM_BANK(1) + 0x300)
static struct resource mixtile4x12_dm9000_resources[] = {
	[0] = {
		.start	= DM9000_PA,
		.end	= DM9000_PA + 3,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= DM9000_PA + 8,          //16 bit +8; 8 bit +4
		.end	= DM9000_PA + 8 + 3,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= GPIO_DM9000_IRQ,
		.end	= GPIO_DM9000_IRQ,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct dm9000_plat_data mixtile4x12_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

struct platform_device mixtile4x12_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(mixtile4x12_dm9000_resources),
	.resource	= mixtile4x12_dm9000_resources,
	.dev		= {
		.platform_data	= &mixtile4x12_dm9000_platdata,
	},
};

static void __init mixtile4x12_dm9000_init(void)
{
	u32 tmp;
	//电平转换芯片使能
//	gpio_request(GPIO_DM9000_CS0, "GPIO_DM9000_CS0");
//	s3c_gpio_cfgpin(GPIO_DM9000_CS0, S3C_GPIO_SFN(1));
//	gpio_free(GPIO_DM9000_CS0);
//	gpio_direction_output(GPIO_DM9000_CS0, 0);//CS0 -> 0
	//DM9000 时序
	tmp = (2<<4)|(2<<8)|(2<<12)|(3<<16)|(2<<24)|(2<<28);
	__raw_writel(tmp, S5P_SROM_BC1);
	//SROM_BW 配置
	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	
	tmp |= (1 << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);


  //电源
  //gpio_direction_output(GPIO_DM9000_POWER, 1);		     
}
#endif /* CONFIG_DM9000 */

#ifdef CONFIG_AX88796C

#define AX88796_PA (EXYNOS4_PA_SROM_BANK(1))

static void __init mixtile4x12_ax88796c_init(void)
{
	u32 tmp;
	//AX88796 时序
	tmp = (2<<4)|(2<<8)|(2<<12)|(3<<16)|(2<<24)|(2<<28);
	__raw_writel(tmp, S5P_SROM_BC1);
	//SROM_BW 配置
	tmp = __raw_readl(S5P_SROM_BW);

	tmp &= ~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	
	tmp |= (0xb << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);

//  s3c_gpio_setpull(EXYNOS4_GPX3(3), S3C_GPIO_PULL_DOWN);
  s3c_gpio_cfgpin(EXYNOS4_GPX2(5), S3C_GPIO_SFN(0xF));    
    
}

static struct resource mixtile4x12_asix_resource[] = {
	[0] = {
		.start = AX88796_PA,
		.end   = AX88796_PA + 0xFFFFF,//(0x20 * 0x20) -1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = IRQ_EINT(21),
		.end   = IRQ_EINT(21),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL
	}
};

static struct platform_device mixtile4x12_device_asix = {
	.name		= "ax88796c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(mixtile4x12_asix_resource),
	.resource	= mixtile4x12_asix_resource,
};

#endif /* CONFIG_AX88796 */

static void gps_gpio_init(void)
{
	struct device *gps_dev;

	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev)) {
		pr_err("Failed to create device(gps)!\n");
		goto err;
	}
	gpio_direction_output(GPIO_GPS_POWER, 0);
	gpio_direction_output(GPIO_GPS_EN, 0);

	s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
//	s3c_gpio_setpull(GPIO_GPS_RTS, S3C_GPIO_PULL_UP);
	
	gpio_export(GPIO_GPS_EN, 1);
	gpio_export(GPIO_GPS_POWER, 1);
	gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_EN);
	gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_EN);
	gpio_export_link(gps_dev, "GPS_ENABLE", GPIO_GPS_POWER);

 err:
	return;
}

// static struct platform_device *smdk4412_devices[] __initdata = {
// };

/* LCD Backlight data */
static struct samsung_bl_gpio_info mixtile4x12_bl_gpio_info = {
	.no = GPIO_LCD_PWM,
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data mixtile4x12_bl_data = {
	.pwm_id = 0,
	.pwm_period_ns  = 52631,
	.max_brightness = 255,
	.dft_brightness = 255,
	.lth_brightness = 0,
};

// static struct s3c_fb_pd_win smdk4x12_fb_win0 = {
// 	.xres			= 480,
// 	.yres			= 800,
//         .virtual_x              = 480,
//         .virtual_y              = 800 * CONFIG_FB_S3C_NR_BUFFERS,
//         .max_bpp                = 32,
//         .default_bpp            = 24,
// 	.width			= 66,
// 	.height 		= 109,
// };
// 
// static struct s3c_fb_pd_win smdk4x12_fb_win1 = {
// 	.xres			= 480,
// 	.yres			= 800,
//         .virtual_x              = 480,
//         .virtual_y              = 800 * CONFIG_FB_S3C_NR_BUFFERS,
//         .max_bpp                = 32,
//         .default_bpp            = 24,
// 	.width			= 66,
// 	.height 		= 109,
// };
// 
// static struct s3c_fb_pd_win smdk4x12_fb_win2 = {
// 	.xres			= 480,
// 	.yres			= 800,
//         .virtual_x              = 480,
//         .virtual_y              = 800 * CONFIG_FB_S3C_NR_BUFFERS,
//         .max_bpp                = 32,
//         .default_bpp            = 24,
// 	.width			= 66,
// 	.height 		= 109,
// };
// 
// static struct s3c_fb_pd_win smdk4x12_fb_win3 = {
// 	.xres			= 480,
// 	.yres			= 800,
//         .virtual_x              = 480,
//         .virtual_y              = 800 * CONFIG_FB_S3C_NR_BUFFERS,
//         .max_bpp                = 32,
//         .default_bpp            = 24,
// 	.width			= 66,
// 	.height 		= 109,
// };
// 
// static struct s3c_fb_pd_win smdk4x12_fb_win4 = {
// 	.xres			= 480,
// 	.yres			= 800,
//         .virtual_x              = 480,
//         .virtual_y              = 800 * CONFIG_FB_S3C_NR_BUFFERS,
//         .max_bpp                = 32,
//         .default_bpp            = 24,
// 	.width			= 66,
// 	.height 		= 109,
// };
// 
// static struct fb_videomode smdk4x12_lcd_timing = {
// 	.left_margin	= 9,
// 	.right_margin	= 9,
// 	.upper_margin	= 5,
// 	.lower_margin	= 5,
// 	.hsync_len	= 2,
// 	.vsync_len	= 2,
// 	.xres		= 480,
// 	.yres		= 800,
// };
// 
// static struct s3c_fb_platdata smdk4x12_lcd0_pdata __initdata = {
//         .win[0]         = &smdk4x12_fb_win0,
//         .win[1]         = &smdk4x12_fb_win1,
//         .win[2]         = &smdk4x12_fb_win2,
//         .win[3]         = &smdk4x12_fb_win3,
//         .win[4]         = &smdk4x12_fb_win4,
// 	.vtiming	= &smdk4x12_lcd_timing,
//         .vidcon0        = VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
//         .vidcon1        = VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
//         .setup_gpio     = exynos4_fimd0_gpio_setup_24bpp,
// };

// #ifdef CONFIG_S3C64XX_DEV_SPI0
// static struct s3c64xx_spi_csinfo spi0_csi[] = {
// 	[0] = {
// 		.line = EXYNOS4_GPB(1),
// 		.fb_delay = 0x0,
// 	},
// };
// 
// static struct spi_board_info spi0_board_info[] __initdata = {
// 	{
// 		.modalias = "spidev",
// 		.platform_data = NULL,
// 		.max_speed_hz = 10*1000*1000,
// 		.bus_num = 0,
// 		.chip_select = 0,
// 		.mode = SPI_MODE_0,
// 		.controller_data = &spi0_csi[0],
// 	}
// };
// #endif
// 
// #ifdef CONFIG_S3C64XX_DEV_SPI1
// static struct s3c64xx_spi_csinfo spi1_csi[] = {
// 	[0] = {
// 		.line = EXYNOS4_GPB(5),
// 		.fb_delay = 0x0,
// 	},
// };
// 
// static struct spi_board_info spi1_board_info[] __initdata = {
// 	{
// 		.modalias = "spidev",
// 		.platform_data = NULL,
// 		.max_speed_hz = 10*1000*1000,
// 		.bus_num = 1,
// 		.chip_select = 0,
// 		.mode = SPI_MODE_0,
// 		.controller_data = &spi1_csi[0],
// 	}
// };
// #endif
// 
// #ifdef CONFIG_S3C64XX_DEV_SPI2
// static struct s3c64xx_spi_csinfo spi2_csi[] = {
// 	[0] = {
// 		.line = EXYNOS4_GPC1(2),
// 		.fb_delay = 0x0,
// 	},
// };
// 
// static struct spi_board_info spi2_board_info[] __initdata = {
// 	{
// 		.modalias = "spidev",
// 		.platform_data = NULL,
// 		.max_speed_hz = 10*1000*1000,
// 		.bus_num = 2,
// 		.chip_select = 0,
// 		.mode = SPI_MODE_0,
// 		.controller_data = &spi2_csi[0],
// 	}
// };
// #endif
// 
// #ifdef CONFIG_LCD_LMS501KF03
// static int lcd_power_on(struct lcd_device *ld, int enable)
// {
//         return 1;
// }
// 
// static int reset_lcd(struct lcd_device *ld)
// {
//         int err = 0;
//         err = gpio_request_one(EXYNOS4X12_GPM3(6),
//                                GPIOF_OUT_INIT_HIGH, "GPM3");
// 	if (err) {
// 		pr_err("failed to request GPM3 for lcd reset control\n");
//                 return err;
// 	}
//         gpio_set_value(EXYNOS4X12_GPM3(6), 0);
//         mdelay(1);
//         gpio_set_value(EXYNOS4X12_GPM3(6), 1);
// 	gpio_free(EXYNOS4X12_GPM3(6));
// 
//         return 1;
// }
// static struct lcd_platform_data lms501kf03_platform_data = {
//         .reset                  = reset_lcd,
//         .power_on               = lcd_power_on,
//         .lcd_enabled            = 0,
//         .reset_delay            = 100,  /* 100ms */
// };
// #define         LCD_BUS_NUM     3
// static struct spi_board_info spi_board_info[] __initdata = {
//         {
//                 .modalias               = "lms501kf03",
//                 .platform_data          = (void *)&lms501kf03_platform_data,
//                 .max_speed_hz           = 1200000,
//                 .bus_num                = LCD_BUS_NUM,
//                 .chip_select            = 0,
//                 .mode                   = SPI_MODE_3,
//                 .controller_data        = (void *)EXYNOS4_GPB(5),
//         }
// };
// 
// static struct spi_gpio_platform_data lms501kf03_spi_gpio_data = {
//         .sck    = EXYNOS4_GPB(4), /* DISPLAY_CLK */
//         .mosi   = EXYNOS4_GPB(7), /* DISPLAY_SI */
//         .miso   = SPI_GPIO_NO_MISO,
//         .num_chipselect = 1,
// };
// 
// static struct platform_device s3c_device_spi_gpio = {
//         .name   = "spi_gpio",
//         .id     = LCD_BUS_NUM,
//         .dev    = {
//                 .parent         = &s5p_device_fimd0.dev,
//                 .platform_data  = &lms501kf03_spi_gpio_data,
//         },
// };
// #endif
// 

/*static struct gpio_event_direct_entry mixtile4x12_keypad_key_map[] = {
	{
		.gpio   = EXYNOS4_GPX0(0),
		.code   = KEY_POWER,
	}
};

static struct gpio_event_input_info mixtile4x12_keypad_key_info = {
	.info.func              = gpio_event_input_func,
	.info.no_suspend        = true,
	.debounce_time.tv64     = 20 * NSEC_PER_MSEC,
	.type                   = EV_KEY,
	.keymap                 = mixtile4x12_keypad_key_map,
	.keymap_size            = ARRAY_SIZE(mixtile4x12_keypad_key_map)
};

static struct gpio_event_info *mixtile4x12_input_info[] = {
	&mixtile4x12_keypad_key_info.info,
};

static struct gpio_event_platform_data mixtile4x12_input_data = {
	.names  = {
		"mixtile4x12-keypad",
		NULL,
	},
	.info           = mixtile4x12_input_info,
	.info_count     = ARRAY_SIZE(mixtile4x12_input_info),
};

static struct platform_device mixtile4x12_input_device = {
	.name   = GPIO_EVENT_DEV_NAME,
	.id     = 0,
	.dev    = {
		.platform_data = &mixtile4x12_input_data,
	},
}*/;

// #ifdef CONFIG_SAMSUNG_DEV_KEYPAD
// static uint32_t mixtile4x12_keymap[] __initdata = {
// 	/* KEY(row, col, keycode) */
// 	KEY(1, 3, KEY_1), KEY(1, 4, KEY_2), KEY(1, 5, KEY_3),
// 	KEY(1, 6, KEY_4), KEY(1, 7, KEY_5),
// 	KEY(2, 5, KEY_D), KEY(2, 6, KEY_A), KEY(2, 7, KEY_B),
// 	KEY(0, 7, KEY_E), KEY(0, 5, KEY_C),
// 	KEY(0, 6, KEY_DOWN), KEY(0, 3, KEY_MENU), KEY(0, 4, KEY_RIGHT)
// };
// 
// static struct matrix_keymap_data mixtile4x12_keymap_data __initdata = {
//         .keymap         = mixtile4x12_keymap,
//         .keymap_size    = ARRAY_SIZE(mixtile4x12_keymap),
// };
// 
// static struct samsung_keypad_platdata mixtile4x12_keypad_data __initdata = {
//         .keymap_data    = &mixtile4x12_keymap_data,
//         .rows           = 3,
//         .cols           = 8,
// };
// #endif

static struct platform_device samsung_audio = {
	.name   = "SOC-AUDIO-SAMSUNG",
	.id     = -1,
};

#ifdef CONFIG_BUSFREQ_OPP
/* BUSFREQ to control memory/bus*/
static struct device_domain busfreq;

static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};
#endif

static struct platform_device *mixtile4x12_devices[] __initdata = {

#ifdef GPIO_MOTOR_POWER
	&mixtile4x12_timed_gpios,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&mixtile4x12_buttons_device,
#endif

	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_rtc,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s3c_device_adc,
#ifdef CONFIG_SWITCH_HEADPHONE
	&hp_switch_device,
#endif

	&s5p_device_ehci,
	&exynos4_device_ohci,
	&s5p_device_usbswitch,
	
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos_device_dwmci,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
	
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	&exynos_device_flite0,
	&exynos_device_flite1,
#endif
// 	&s5p_device_mipi_csis0,
// 	&s5p_device_mipi_csis1,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&mali_gpu_device,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_jpeg,

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	&exynos4_device_fimc_is,
#endif
// #ifdef CONFIG_LCD_LMS501KF03
//         &s3c_device_spi_gpio,
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI0
// 	&s3c64xx_device_spi0,
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI1
// 	&s3c64xx_device_spi1,
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI2
// 	&s3c64xx_device_spi2,
// #endif
#ifdef CONFIG_ION_EXYNOS
        &exynos_device_ion,
#endif
	&s5p_device_i2c_hdmiphy,
	&s5p_device_hdmi,
	&s5p_device_mixer,
	&exynos4_bus_devfreq,
	&samsung_asoc_dma,
	&samsung_asoc_idma,
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos4_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos4_device_pcm0,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
        &exynos4_device_spdif,
#endif
	&samsung_audio,
#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos_device_tmu,
#endif

#if defined CONFIG_SND_SAMSUNG_ALP
        &exynos_device_srp,
#endif
#ifdef CONFIG_BUSFREQ_OPP
	&exynos4_busfreq,
#endif
#ifdef CONFIG_BATTERY_MIXTILE
	&mixtile4x12_device_battery,
#endif
#ifdef CONFIG_DM9000
	&mixtile4x12_dm9000,
#endif
#ifdef CONFIG_AX88796C
	&mixtile4x12_device_asix,
#endif
#ifdef CONFIG_LEDS_PWM
	&mixtile4x12_leds_pwm,
#endif
};

static void __init mixtile4x12_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(mixtile4x12_uartcfgs, ARRAY_SIZE(mixtile4x12_uartcfgs));
}

static void __init mixtile4x12_reserve(void)
{
	// HACK: This reserved memory will be used for FIMC-IS
	s5p_mfc_reserve_mem(0x58000000, 32<< 20, 0x43000000, 0 << 20);
}

static void mixtile4x12_pmu_wdt_init(void)
{
	unsigned int value;

	if (soc_is_exynos4212() || soc_is_exynos4412()) {
		value = __raw_readl(S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_AUTOMATIC_WDT_RESET_DISABLE);
		value = __raw_readl(S5P_MASK_WDT_RESET_REQUEST);
		value &= ~S5P_SYS_WDTRESET;
		__raw_writel(value, S5P_MASK_WDT_RESET_REQUEST);
	}
}

static void mixtile4x12_rtc_wake_init(void)
{
#ifdef CONFIG_PM
	gic_arch_extn.irq_set_wake = s3c_irq_wake;
#endif
}

struct s3c2410_platform_i2c default_i2c1_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 100*1000,
	.sda_delay	= 100,
	.bus_num = 1,
};

struct s3c2410_platform_i2c default_i2c3_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 200*1000,
	.sda_delay	= 100,
	.bus_num = 3,
};

// static struct s3c2410_platform_i2c universal_i2c4_platdata __initdata = {
// 	.frequency	= 300 * 1000,
// 	.sda_delay	= 200,
// };

// #ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
// static void __set_flite_camera_config(struct exynos_platform_flite *data,
//                                         u32 active_index, u32 max_cam)
// {       
//         data->active_cam_index = active_index;
//         data->num_clients = max_cam;
// }
// 
// static void __init smdk4x12_set_camera_flite_platdata(void)
// {
//         int flite0_cam_index = 0;
//         int flite1_cam_index = 0;
// #ifdef CONFIG_VIDEO_S5K6A3
// #ifdef CONFIG_S5K6A3_CSI_C
//         exynos_flite0_default_data.cam[flite0_cam_index++] = &s5k6a3;
// #endif
// #ifdef CONFIG_S5K6A3_CSI_D
//         exynos_flite1_default_data.cam[flite1_cam_index++] = &s5k6a3;
// #endif
// #endif
//         __set_flite_camera_config(&exynos_flite0_default_data, 0, flite0_cam_index);
//         __set_flite_camera_config(&exynos_flite1_default_data, 0, flite1_cam_index);
// }
// #endif

// #ifdef CONFIG_USB_EXYNOS_SWITCH
// static struct s5p_usbswitch_platdata mixtile4x12_usbswitch_pdata;
// 
// static void __init mixtile4x12_usbswitch_init(void)
// {
// 	struct s5p_usbswitch_platdata *pdata = &mixtile4x12_usbswitch_pdata;
// 	int err;
// 
// 	pdata->gpio_host_detect = EXYNOS4_GPX3(5); /* low active */
// 	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN,
// 							"HOST_DETECT");
// 	if (err) {
// 		pr_err("failed to request gpio_host_detect\n");
// 		return;
// 	}
// 
// 	s3c_gpio_cfgpin(pdata->gpio_host_detect, S3C_GPIO_SFN(0xF));
// 	s3c_gpio_setpull(pdata->gpio_host_detect, S3C_GPIO_PULL_NONE);
// 	gpio_free(pdata->gpio_host_detect);
// 
// 	pdata->gpio_device_detect = EXYNOS4_GPX3(4); /* high active */
// 	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN,
// 							"DEVICE_DETECT");
// 	if (err) {
// 		pr_err("failed to request gpio_host_detect for\n");
// 		return;
// 	}
// 
// 	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_SFN(0xF));
// 	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
// 	gpio_free(pdata->gpio_device_detect);
// 
// 	pdata->gpio_host_vbus = EXYNOS4_GPL2(0);
// 	err = gpio_request_one(pdata->gpio_host_vbus, GPIOF_OUT_INIT_LOW,
// 							"HOST_VBUS_CONTROL");
// 	if (err) {
// 		pr_err("failed to request gpio_host_vbus\n");
// 		return;
// 	}
// 
// 	s3c_gpio_setpull(pdata->gpio_host_vbus, S3C_GPIO_PULL_NONE);
// 	gpio_free(pdata->gpio_host_vbus);
// 
// 	s5p_usbswitch_set_platdata(pdata);
// }
// #endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.ip_ver         = IP_VER_G2D_4P,
	.hw_ver         = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname        = "sclk_fimg2d",
	.gate_clkname   = "fimg2d",
	.clkrate        = 200 * MHZ,
};
#endif

static int __init exynos4_setup_clock(struct device *dev,
						const char *clock,
						const char *parent,
						unsigned long clk_rate)
{
	struct clk *clk_parent;
	struct clk *sclk;

	sclk = clk_get(dev, clock);
	if (IS_ERR(sclk)) {
		pr_err("Unable to get clock:%s.\n", clock);
		return PTR_ERR(sclk);
	}

	clk_parent = clk_get(NULL, parent);
	if (IS_ERR(clk_parent)) {
		clk_put(sclk);
		pr_err("Unable to get parent clock:%s of clock:%s.\n",
				parent, sclk->name);
		return PTR_ERR(clk_parent);
	}

	if (clk_set_parent(sclk, clk_parent)) {
		pr_err("Unable to set parent %s of clock %s.\n", parent, clock);
		clk_put(sclk);
		clk_put(clk_parent);
		return PTR_ERR(sclk);
	}

	if (clk_rate)
		if (clk_set_rate(sclk, clk_rate)) {
			pr_err("%s rate change failed: %lu\n", sclk->name,
				clk_rate);
			clk_put(sclk);
			clk_put(clk_parent);
			return PTR_ERR(sclk);
		}

	clk_put(sclk);
	clk_put(clk_parent);

	return 0;
}

static void initialize_prime_clocks(void)
{
	exynos4_setup_clock(&s5p_device_fimd0.dev, "sclk_fimd",
                                        "mout_mpll_user", 176 * MHZ);

	exynos4_setup_clock(&s5p_device_fimc0.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc1.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc2.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc3.dev, "sclk_fimc",
					"mout_mpll_user", 176 * MHZ);

// 	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
// 					"mout_mpll_user", 176 * MHZ);
// 	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
// 					"mout_mpll_user", 176 * MHZ);

	exynos4_setup_clock(NULL, "mout_mfc0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_mfc",
					"mout_mfc0", 220 * MHZ);

	exynos4_setup_clock(NULL, "mout_jpeg0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_jpeg",
					"mout_jpeg0", 176 * MHZ);
#ifdef CONFIG_S3C_DEV_HSMMC
	exynos4_setup_clock(&s3c_device_hsmmc0.dev, "dout_mmc0",
					"mout_mpll_user", 100 * MHZ);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1	
	exynos4_setup_clock(&s3c_device_hsmmc1.dev, "dout_mmc1",
					"mout_mpll_user", 100 * MHZ);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	exynos4_setup_clock(&s3c_device_hsmmc2.dev, "dout_mmc2",
					"mout_mpll_user", 100 * MHZ);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	exynos4_setup_clock(&s3c_device_hsmmc3.dev, "dout_mmc3",
					"mout_mpll_user", 100 * MHZ);
#endif
	
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#ifdef CONFIG_SND_SAMSUNG_I2S_MASTER
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_epll", 400 * MHZ);
#else
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_mpll_user", 440 * MHZ);
#endif
#endif
}

static void initialize_non_prime_clocks(void)
{
	exynos4_setup_clock(&s5p_device_fimd0.dev, "sclk_fimd",
                                        "mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(&s5p_device_fimc0.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc1.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc2.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);
	exynos4_setup_clock(&s5p_device_fimc3.dev, "sclk_fimc",
					"mout_mpll_user", 160 * MHZ);

// 	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
// 					"mout_mpll_user", 160 * MHZ);
// 	exynos4_setup_clock(&s5p_device_mipi_csis0.dev, "sclk_csis",
// 					"mout_mpll_user", 160 * MHZ);

	exynos4_setup_clock(NULL, "mout_mfc0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_mfc",
					"mout_mfc0", 200 * MHZ);

	exynos4_setup_clock(NULL, "mout_jpeg0", "mout_mpll", 0);
	exynos4_setup_clock(&s5p_device_mfc.dev, "sclk_jpeg",
					"mout_jpeg0", 160 * MHZ);

	exynos4_setup_clock(&s3c_device_hsmmc2.dev, "dout_mmc2",
					"mout_mpll_user", 100 * MHZ);
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos4_setup_clock(&exynos_device_dwmci.dev, "dout_mmc4",
					"mout_mpll_user", 400 * MHZ);
#endif
}

static void mixtile4x12_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");

	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");
};

void tp_power(int onoff)
{
printk("%s %d\n", __func__, onoff);
    gpio_direction_output(GPIO_TP_POWER, onoff);
#ifdef CONFIG_TOUCHSCREEN_GT81XX
  if(onoff)
  {
    gpio_direction_output(GPIO_TP_RST, 0);
    msleep(5);
    gpio_direction_output(GPIO_TP_RST, 1);
    msleep(5);
  }
  else
    gpio_direction_output(GPIO_TP_RST, 0);
#endif /* CONFIG_TOUCHSCREEN_GT81XX */
}
EXPORT_SYMBOL(tp_power);

#define REBOOT_MODE_PREFIX	0x12345670
#define REBOOT_MODE_NONE	0
#define REBOOT_MODE_DOWNLOAD	1
#define REBOOT_MODE_UPLOAD	2
#define REBOOT_MODE_CHARGING	3
#define REBOOT_MODE_RECOVERY	4
#define REBOOT_MODE_ARM11_FOTA	5

#define REBOOT_SET_PREFIX	0xabc00000
#define REBOOT_SET_DEBUG	0x000d0000
#define REBOOT_SET_SWSEL	0x000e0000
#define REBOOT_SET_SUD		0x000f0000
static void mixtile4x12_reboot(char str, const char *cmd)
{
 	printk("mixtile4x12_reboot %s\n", cmd ? cmd : "(null)");
	local_irq_disable();
	writel(0x12345678, S5P_INFORM2);	/* Don't enter lpm mode */

	if (!cmd) {
		writel(REBOOT_MODE_PREFIX | REBOOT_MODE_NONE, S5P_INFORM2);
	} else {
		if (!strcmp(cmd, "recovery"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_RECOVERY, S5P_INFORM2);
		else if (!strcmp(cmd, "fota"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_ARM11_FOTA, S5P_INFORM2);
		else if (!strcmp(cmd, "download"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_DOWNLOAD, S5P_INFORM2);
		else if (!strcmp(cmd, "upload"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_UPLOAD, S5P_INFORM2);
		else if (!strcmp(cmd, "chargeing"))
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_CHARGING, S5P_INFORM2);
		else
			writel(REBOOT_MODE_PREFIX | REBOOT_MODE_NONE, S5P_INFORM2);
	}

	flush_cache_all();
	outer_flush_all();
	//arch_reset(0, 0);
	arch_wdt_reset();

	pr_emerg("%s: waiting for reboot\n", __func__);
	while (1)
	  msleep(1000);
}

static void mixtile4x12_power_off(void)
{
  int state1, state2;
  state1 = gpio_get_value(GPIO_CHARGE_STATE1);
  state2 = gpio_get_value(GPIO_CHARGE_STATE2);

  if((state1==0) || (state2==0))
    mixtile4x12_reboot('h', "charging");
  local_irq_disable();
 /* PS_HOLD low*/
 printk("mixtile4x12_power_off %d %d\n", state1, state2);
 while(1)
 {
   //writel(readl(EXYNOS4_PS_HOLD_CONTROL) & (~(0x1<<8)), EXYNOS4_PS_HOLD_CONTROL);
   msleep(1000);
 }
}

static int mixtile4x12_get_ver(int channel)
{
  return 0;
}

// Machine initialization entry point
static void __init mixtile4x12_machine_init(void)
{
	dev_ver = mixtile4x12_get_ver(3);
	mixtile4x12_config_gpio_table();

	pm_power_off = mixtile4x12_power_off;
// 	arm_pm_restart = mixtile4x12_reboot;
	
// #ifdef CONFIG_S3C64XX_DEV_SPI0
// 	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI1
// 	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI2
// 	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
// #endif

	s3c_adc_set_platdata(NULL);
	s3c_adc_setname("samsung-adc-v4");
	
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, mixtile4x12_i2c_devs0,
				ARRAY_SIZE(mixtile4x12_i2c_devs0));

	s3c_i2c1_set_platdata(&default_i2c1_data);
	i2c_register_board_info(1, mixtile4x12_i2c_devs1,
				ARRAY_SIZE(mixtile4x12_i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
        i2c_register_board_info(2, mixtile4x12_i2c_devs2, 
				ARRAY_SIZE(mixtile4x12_i2c_devs2));

	s3c_i2c3_set_platdata(&default_i2c3_data);
	i2c_register_board_info(3, mixtile4x12_i2c_devs3,
				ARRAY_SIZE(mixtile4x12_i2c_devs3));

	s3c_i2c4_set_platdata(NULL);
	s3c_i2c5_set_platdata(NULL);

	mixtile4x12_rtc_wake_init();
	mixtile4x12_pmu_wdt_init();

	s3c_hsotg_set_platdata(&mixtile4x12_hsotg_pdata);
	
//	s5p_fimd0_set_platdata(&smdk4x12_lcd0_pdata);

#ifdef CONFIG_FB_S5P
	s3cfb_set_platdata(NULL);
#endif

	mixtile4x12_ehci_init();
	mixtile4x12_ohci_init();

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&mixtile4x12_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&mixtile4x12_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&mixtile4x12_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&mixtile4x12_hsmmc3_pdata);
#endif

#ifdef CONFIG_ION_EXYNOS
        exynos_ion_set_platdata();
#endif
	s5p_tv_setup();
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(mixtile4x12_i2c_hdmiphy, NULL, 0);

#ifdef CONFIG_VIDEO_EXYNOS_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
// #if defined(CONFIG_VIDEO_M5MOLS) || defined(CONFIG_VIDEO_S5K6A3)
// 	smdk4x12_camera_init();
// #endif
	
// fix: setup camera
// #ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
//         smdk4x12_set_camera_flite_platdata();
//         s3c_set_platdata(&exynos_flite0_default_data,
//                         sizeof(exynos_flite0_default_data), &exynos_device_flite0);
//         s3c_set_platdata(&exynos_flite1_default_data,
//                         sizeof(exynos_flite1_default_data), &exynos_device_flite1);
// #endif

// #ifdef CONFIG_S3C64XX_DEV_SPI0
// 	s3c64xx_spi0_set_platdata(NULL, 0, 1);
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI1
// 	s3c64xx_spi1_set_platdata(NULL, 0, 1);
// #endif
// #ifdef CONFIG_S3C64XX_DEV_SPI2
// 	s3c64xx_spi2_set_platdata(NULL, 0, 1);
// #endif

	samsung_bl_set(&mixtile4x12_bl_gpio_info, &mixtile4x12_bl_data);

#ifdef CONFIG_AX88796C
	mixtile4x12_ax88796c_init();
#endif
	
#ifdef CONFIG_DM9000
	mixtile4x12_dm9000_init();
#endif
	
#ifdef CONFIG_VIDEO_FIMC
//#if defined(CONFIG_ITU_B)
	mixtile4x12_cam_b_power(1);
	msleep(50);
	mixtile4x12_cam_b_power(0);
//#endif
//#if defined(CONFIG_ITU_A)
	mixtile4x12_cam_f_power(1);
	msleep(50);
	mixtile4x12_cam_f_power(0);
//#endif
//#ifdef CONFIG_VIDEO_TVP5151
	tvp5151_power(1);
	msleep(50);
	tvp5151_power(0);
//#endif
#endif

#if defined(CONFIG_RTL8188CUS_MODULE) || defined(CONFIG_RTL8188EU_MODULE) 
  usb_wifi_power(0);
#endif
  if(dev_ver==0)
  {
    usb_hub_power(1);
    msleep(200);
    usb_host_power(1);
    msleep(200);
  }
	mixtile4x12_switch_init();
	gps_gpio_init();
  
	platform_add_devices(mixtile4x12_devices, ARRAY_SIZE(mixtile4x12_devices));
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
        exynos4_fimc_is_set_platdata(NULL);
#endif

	if (soc_is_exynos4412()) {
		if ((samsung_rev() >= EXYNOS4412_REV_2_0))
			initialize_prime_clocks();
		else
			initialize_non_prime_clocks();
	}
#ifdef CONFIG_BUSFREQ_OPP
        dev_add(&busfreq, &exynos4_busfreq.dev);
        ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos4_busfreq.dev);
        ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos4_busfreq.dev);
        ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos4_busfreq.dev);
#endif
	set_tmu_platdata();
	register_reboot_notifier(&exynos4_reboot_notifier);
	
#ifdef CONFIG_LEDS_PWM
  mixtile4x12_leds_pwm_init();
#endif
}

MACHINE_START(SMDK4412, "SMDK4X12")
	.atag_offset	= 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= mixtile4x12_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= mixtile4x12_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &mixtile4x12_reserve,
MACHINE_END
