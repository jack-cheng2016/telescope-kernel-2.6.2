/*
 * linux/arch/arm/mach-omap2/board-omap3telescope.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/spi/spi.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/mcspi.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#include <media/soc_camera.h>


#ifdef CONFIG_PM
static struct omap_opp * _omap35x_mpu_rate_table        = omap35x_mpu_rate_table;
static struct omap_opp * _omap37x_mpu_rate_table        = omap37x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table        = omap35x_dsp_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table        = omap37x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table         = omap35x_l3_rate_table;
static struct omap_opp * _omap37x_l3_rate_table         = omap37x_l3_rate_table;
#else   /* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table        = NULL;
static struct omap_opp * _omap37x_mpu_rate_table        = NULL;
static struct omap_opp * _omap35x_dsp_rate_table        = NULL;
static struct omap_opp * _omap37x_dsp_rate_table        = NULL;
static struct omap_opp * _omap35x_l3_rate_table         = NULL;
static struct omap_opp * _omap37x_l3_rate_table         = NULL;
#endif  /* CONFIG_PM */

#if defined(CONFIG_VIDEO_MT9V113) || defined(CONFIG_VIDEO_MT9V113_MODULE)
#include <media/v4l2-int-device.h>
#include <media/mt9v113.h>
extern struct mt9v113_platform_data mt9v113_pdata;
#endif

#if defined(CONFIG_VIDEO_MT9T112) || defined(CONFIG_VIDEO_MT9T112_MODULE)
#include <media/v4l2-int-device.h>
#include <media/mt9t112.h>
extern struct mt9t112_platform_data mt9t112_pdata;
#endif

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
#include <media/v4l2-int-device.h>
#include <media/mt9p031.h>
extern struct mt9p031_platform_data mt9p031_pdata;
#endif

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

char expansionboard_name[16];

#if defined(CONFIG_ENC28J60) || defined(CONFIG_ENC28J60_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3TELESCOPE_GPIO_ENC28J60_IRQ 157

static struct omap2_mcspi_device_config enc28j60_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3telescope_zippy_spi_board_info[] __initdata = {
	{
		.modalias		= "enc28j60",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 20000000,
		.controller_data	= &enc28j60_spi_chip_info,
	},
};

static void __init omap3telescope_enc28j60_init(void)
{
	if ((gpio_request(OMAP3TELESCOPE_GPIO_ENC28J60_IRQ, "ENC28J60_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3TELESCOPE_GPIO_ENC28J60_IRQ) == 0)) {
		gpio_export(OMAP3TELESCOPE_GPIO_ENC28J60_IRQ, 0);
		omap3telescope_zippy_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3TELESCOPE_GPIO_ENC28J60_IRQ);
		set_irq_type(omap3telescope_zippy_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for ENC28J60_IRQ\n");
		return;
	}

	spi_register_board_info(omap3telescope_zippy_spi_board_info,
			ARRAY_SIZE(omap3telescope_zippy_spi_board_info));
}

#else
static inline void __init omap3telescope_enc28j60_init(void) { return; }
#endif

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define OMAP3TELESCOPE_GPIO_KS8851_IRQ 157

static struct omap2_mcspi_device_config ks8851_spi_chip_info = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3telescope_zippy2_spi_board_info[] __initdata = {
	{
		.modalias		= "ks8851",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 36000000,
		.controller_data	= &ks8851_spi_chip_info,
	},
};

static void __init omap3telescope_ks8851_init(void)
{
	if ((gpio_request(OMAP3TELESCOPE_GPIO_KS8851_IRQ, "KS8851_IRQ") == 0) &&
	    (gpio_direction_input(OMAP3TELESCOPE_GPIO_KS8851_IRQ) == 0)) {
		gpio_export(OMAP3TELESCOPE_GPIO_KS8851_IRQ, 0);
		omap3telescope_zippy2_spi_board_info[0].irq	= OMAP_GPIO_IRQ(OMAP3TELESCOPE_GPIO_KS8851_IRQ);
		set_irq_type(omap3telescope_zippy2_spi_board_info[0].irq, IRQ_TYPE_EDGE_FALLING);
	} else {
		printk(KERN_ERR "could not obtain gpio for KS8851_IRQ\n");
		return;
	}
	
	spi_register_board_info(omap3telescope_zippy2_spi_board_info,
							ARRAY_SIZE(omap3telescope_zippy2_spi_board_info));
}

#else
static inline void __init omap3telescope_ks8851_init(void) { return; }
#endif

static struct mtd_partition omap3telescope_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3telescope_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= omap3telescope_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3telescope_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource omap3telescope_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3telescope_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3telescope_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3telescope_nand_resource,
};

/* DSS */

static int telescope_enable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void telescope_disable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device telescope_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = 170,
	.platform_enable = telescope_enable_dvi,
	.platform_disable = telescope_disable_dvi,
};

static int telescope_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void telescope_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_device telescope_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable = telescope_panel_enable_tv,
	.platform_disable = telescope_panel_disable_tv,
};

static struct omap_dss_device *telescope_dss_devices[] = {
	&telescope_dvi_device,
	&telescope_tv_device,
};

static struct omap_dss_board_info telescope_dss_data = {
	.num_devices = ARRAY_SIZE(telescope_dss_devices),
	.devices = telescope_dss_devices,
	.default_device = &telescope_dvi_device,
};

static struct platform_device telescope_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &telescope_dss_data,
	},
};

static struct regulator_consumer_supply telescope_vdac_supply = {
	.supply		= "vdda_dac",
	.dev		= &telescope_dss_device.dev,
};

static struct regulator_consumer_supply telescope_vdvi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &telescope_dss_device.dev,
};

static void __init telescope_display_init(void)
{
	int r;

	r = gpio_request(telescope_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(telescope_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_wp	= 29,
	},
	{
		.mmc		= 2,
		.wires		= 4,
		.transceiver	= true,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply telescope_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply telescope_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct gpio_led gpio_leds[];

static int telescope_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	if (system_rev >= 0x20 && system_rev <= 0x34301000) {
		omap_mux_init_gpio(23, OMAP_PIN_INPUT);
		mmc[0].gpio_wp = 23;
	} else {
		omap_mux_init_gpio(29, OMAP_PIN_INPUT);
	}
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	telescope_vmmc1_supply.dev = mmc[0].dev;
	telescope_vsim_supply.dev = mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */

	if (cpu_is_omap3630()) {
		/* DVI reset GPIO is different between revisions */	
		telescope_dvi_device.reset_gpio = 129;

		/* Power on DVI, Serial and PWR led */ 
 		gpio_request(gpio + 1, "nDVI_PWR_EN");
		gpio_direction_output(gpio + 1, 0);	

		/* Power on camera interface */
		gpio_request(gpio + 2, "CAM_EN");
		gpio_direction_output(gpio + 2, 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	}
	else {
		/* DVI reset GPIO is different between revisions */ 
		telescope_dvi_device.reset_gpio = 170;
				
		gpio_request(gpio + 1, "EHCI_nOC");
		gpio_direction_input(gpio + 1);

		/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
		gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);
	}


	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data telescope_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= telescope_twl_gpio_setup,
};

#ifdef use_m9tm001

static struct i2c_board_info mt9m001_camera_i2c[] = {
	{
		I2C_BOARD_INFO("mt9m001", 0x5d),
	},
};

static struct soc_camera_link iclink[] = {
	{
		.bus_id			= 2, /* Must match with the camera ID */
		.board_info		= &mt9m001_camera_i2c[0],
		.i2c_adapter_id		= 2,
		.query_bus_param	= NULL,//pcm990_camera_query_bus_param,
		.set_bus_param		= NULL,//pcm990_camera_set_bus_param,
		.free_bus		= NULL,//pcm990_camera_free_bus,
		.module_name		= "mt9m001",
	},
};
#endif




static struct platform_device telescope_cam_device = {
	.name		= "telescope_cam",
	.id		= -1,
#ifdef use_m9tm001	
	.dev	= {
			.platform_data = &iclink[0],
		},	
#endif		
};

static struct regulator_consumer_supply telescope_vaux3_supply = {
	.supply		= "cam_1v8",
	.dev		= &telescope_cam_device.dev,
};

static struct regulator_consumer_supply telescope_vaux4_supply = {
	.supply		= "cam_2v8",
	.dev		= &telescope_cam_device.dev,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data telescope_vaux3 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vaux3_supply,
};

/* VAUX4 for CAM_2V8 */
static struct regulator_init_data telescope_vaux4 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vaux4_supply,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data telescope_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data telescope_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data telescope_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data telescope_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &telescope_vdvi_supply,
};

static struct twl4030_usb_data telescope_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data telescope_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data telescope_codec_data = {
	.audio_mclk = 26000000,
	.audio = &telescope_audio_data,
};

static struct twl4030_madc_platform_data telescope_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data telescope_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &telescope_usb_data,
	.gpio		= &telescope_gpio_data,
	.codec		= &telescope_codec_data,
	.madc		= &telescope_madc_data,
	.vmmc1		= &telescope_vmmc1,
	.vsim		= &telescope_vsim,
	.vdac		= &telescope_vdac,
	.vpll2		= &telescope_vpll2,
	.vaux3		= &telescope_vaux3,
	.vaux4		= &telescope_vaux4,
};

static struct i2c_board_info __initdata telescope_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &telescope_twldata,
	},
};

	
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
#include <linux/i2c/at24.h>

static struct at24_platform_data m24c01 = {
	        .byte_len       = SZ_1K / 8,
	        .page_size      = 16,
};

#if defined(CONFIG_RTC_DRV_DS1307) || \
	defined(CONFIG_RTC_DRV_DS1307_MODULE)

static struct i2c_board_info __initdata telescope_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data	= &m24c01,
	},
};
#else
static struct i2c_board_info __initdata telescope_zippy_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data  = &m24c01,
	},
};
#endif
#else
static struct i2c_board_info __initdata telescope_zippy_i2c2_boardinfo[] = {};
#endif

static struct i2c_board_info __initdata telescope_i2c2_boardinfo[] = {
#if defined(CONFIG_VIDEO_MT9V113) || defined(CONFIG_VIDEO_MT9V113_MODULE)
	{
		I2C_BOARD_INFO("mt9v113", MT9V113_I2C_ADDR),
		.platform_data	= &mt9v113_pdata,
	},
#endif
#if defined(CONFIG_VIDEO_MT9T112) || defined(CONFIG_VIDEO_MT9T112_MODULE)
	{
		I2C_BOARD_INFO("mt9t112", MT9T112_I2C_ADDR),
		.platform_data	= &mt9t112_pdata,
	},
#endif
#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)		
	{
		I2C_BOARD_INFO("mt9p031", MT9P031_I2C_ADDR),
		.platform_data	= &mt9p031_pdata,
	},	
#endif	

//#ifdef use_m9tm001
#if defined(CONFIG_SOC_CAMERA_MT9M001) || defined(CONFIG_SOC_CAMERA_MT9M001_MODULE)		
	{
		I2C_BOARD_INFO("mt9m001", 0x5d),//mt9m001 I2C address is 0x5d
		//.platform_data	= &mt9p031_pdata,//&iclink[0],
	},		
#endif
//#endif

};

static int __init omap3_telescope_i2c_init(void)
{
	//printk("iclink[0]:%p\n",(unsigned int*)&iclink[0]);
	
	omap_register_i2c_bus(1, 2600, telescope_i2c1_boardinfo,ARRAY_SIZE(telescope_i2c1_boardinfo));
	
	if(!strcmp(expansionboard_name, "zippy") || !strcmp(expansionboard_name, "zippy2")) 
	{
		printk(KERN_INFO "Telescope expansionboard: registering i2c2 bus for zippy/zippy2\n");
		printk("Telescope expansionboard: registering i2c2 bus for zippy/zippy2\n");
		omap_register_i2c_bus(2, 400,  telescope_zippy_i2c2_boardinfo,	ARRAY_SIZE(telescope_zippy_i2c2_boardinfo));
	} 
	else
	{
		printk("Telescope expansionboard: registering i2c2 bus ...\n");
		omap_register_i2c_bus(2, 400,  telescope_i2c2_boardinfo,ARRAY_SIZE(telescope_i2c2_boardinfo));
	}
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "telescopeboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "telescopeboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "telescopeboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static struct spi_board_info telescopefpga_mcspi_board_info[] = {
	// spi 4.0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, //48 Mbps
		.bus_num	= 4,
		.chip_select	= 0,	
		.mode = SPI_MODE_1,
	},
};

static void __init telescopefpga_init_spi(void)
{
	/* hook the spi ports to the spidev driver */
	spi_register_board_info(telescopefpga_mcspi_board_info,
		ARRAY_SIZE(telescopefpga_mcspi_board_info));
}

static void __init omap3_telescope_init_irq(void)
{
        if (cpu_is_omap3630())
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap37x_mpu_rate_table,
                                        _omap37x_dsp_rate_table,
                                        _omap37x_l3_rate_table);
        }
        else
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        mt46h32m32lf6_sdrc_params,
                                        _omap35x_mpu_rate_table,
                                        _omap35x_dsp_rate_table,
                                        _omap35x_l3_rate_table);
        }
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
}

static struct platform_device *omap3_telescope_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&telescope_dss_device,
	&telescope_cam_device,
};

static void __init omap3telescope_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3telescope_nand_data.cs = nandcs;
		omap3telescope_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3telescope_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3telescope_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 17,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* Camera - Parallel Data */
	OMAP3_MUX(CAM_D0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	//OMAP3_MUX(CAM_D8, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	//OMAP3_MUX(CAM_D9, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	//OMAP3_MUX(CAM_D10, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	//OMAP3_MUX(CAM_D11, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* Camera - HS/VS signals */
	OMAP3_MUX(CAM_HS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_VS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* Camera - Reset GPIO 98 */
	OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* Camera - XCLK */
	OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;
	strncpy(expansionboard_name, str, 16);
	printk(KERN_INFO "Telescope expansionboard: %s\n", expansionboard_name);
	return 0;
}

static void __init omap3_telescope_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_telescope_i2c_init();

	if (cpu_is_omap3630()) {
		gpio_buttons[0].gpio = 4;
	}

	platform_add_devices(omap3_telescope_devices,ARRAY_SIZE(omap3_telescope_devices));

	//platform_device_register(&telescope_cam_device);
	
	omap_serial_init();

	if(!strcmp(expansionboard_name, "zippy"))
	{
		printk(KERN_INFO "Telescope expansionboard: initializing enc28j60\n");
		omap3telescope_enc28j60_init();
		printk(KERN_INFO "Telescope expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}

	if(!strcmp(expansionboard_name, "zippy2"))
	{
		printk(KERN_INFO "Telescope expansionboard: initializing ks_8851\n");
		omap3telescope_ks8851_init();
		printk(KERN_INFO "Telescope expansionboard: assigning GPIO 141 and 162 to MMC1\n");
		mmc[1].gpio_wp = 141;
		mmc[1].gpio_cd = 162;
	}

	if(!strcmp(expansionboard_name, "trainer"))
	{
		printk(KERN_INFO "Telescope expansionboard: exporting GPIOs 130-141,162 to userspace\n");
		gpio_request(130, "sysfs");
		gpio_export(130, 1);
		gpio_request(131, "sysfs");
		gpio_export(131, 1);
		gpio_request(132, "sysfs");
		gpio_export(132, 1);
		gpio_request(133, "sysfs");
		gpio_export(133, 1);
		gpio_request(134, "sysfs");
		gpio_export(134, 1);
		gpio_request(135, "sysfs");
		gpio_export(135, 1);
		gpio_request(136, "sysfs");
		gpio_export(136, 1);
		gpio_request(137, "sysfs");
		gpio_export(137, 1);
		gpio_request(138, "sysfs");
		gpio_export(138, 1);
		gpio_request(139, "sysfs");
		gpio_export(139, 1);
		gpio_request(140, "sysfs");
		gpio_export(140, 1);
		gpio_request(141, "sysfs");
		gpio_export(141, 1);
		gpio_request(162, "sysfs");
		gpio_export(162, 1);
	}

	if(!strcmp(expansionboard_name, "telescopefpga"))
	{  
		printk(KERN_INFO "Telescope expansionboard: Using McSPI for SPI\n");
		telescopefpga_init_spi();
	}

	usb_musb_init();
	usb_ehci_init(&ehci_pdata);
	omap3telescope_flash_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	telescope_display_init();
}
static void __init omap3_telescope_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

early_param("buddy", expansionboard_setup);

MACHINE_START(OMAP3_TELESCOPE, "OMAP3 Telescope Board")
	/* Maintainer: Syed Mohammed Khasim - http://telescopeboard.org */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_telescope_map_io,
	.init_irq	= omap3_telescope_init_irq,
	.init_machine	= omap3_telescope_init,
	.timer		= &omap_timer,
MACHINE_END
