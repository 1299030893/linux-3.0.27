/*
 * Code for TI8148 EVM.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 /*
* ChangeLog:
 * Copyright 2012 Advantech Corporation
 * Copyright 2013 Sony Corporation , Sony Confidential
* 2013/01/31 Support Sony CBK WA-100 platform added by hbchen
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/phy.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/qt602240_ts.h>
#include <linux/i2c/pcf857x.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps65910.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/asp.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/hdmi_lib.h>
#include <mach/board-ti814x.h>

#include "board-flash.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"
#include "cm81xx.h"


#define ACL_CORSICA1	1    
#define ACL_MMC2	1    

#define GPIO_TSC               31

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#if 1 //def ACL_CORSICA1 // hbchen 
	/* PIN mux for McASP 0 ======================= */
	// TI814X_CONTROL_PADCONF_##mode0##_OFFSET
	TI814X_MUX(UART0_DSRN, OMAP_MUX_MODE6),
	// (1 << (mux_val & 0x7))
	//TI814X_MUX(XREF_CLK0, OMAP_MUX_MODE2),
	TI814X_MUX(XREF_CLK0, OMAP_MUX_MODE7), // hbchen
	TI814X_MUX(USB_DRVVBUS, OMAP_MUX_MODE0), // hbchen
	/* PIN mux for EMAC 1 ======================== */
	// pin 252, 253, 232, 255, 256, 257, 258
	// 232 RMII_REFCLK 
	//TI814X_MUX(RMII_REFCLK, OMAP_MUX_MODE0),
	// 252 GMII0_TXD2
	TI814X_MUX(GMII0_TXD2, OMAP_MUX_MODE1),
	// 253 GMII0_TXD3
	TI814X_MUX(GMII0_TXD3, OMAP_MUX_MODE1),
	// 254 GMII0_TXD4
	TI814X_MUX(GMII0_TXD4, OMAP_MUX_MODE1),
	// 255 GMII0_TXD5
	TI814X_MUX(GMII0_TXD5, OMAP_MUX_MODE1),
	// 256 GMII0_TXD6
	TI814X_MUX(GMII0_TXD6, OMAP_MUX_MODE1),
	// 257 GMII0_TXD7
	TI814X_MUX(GMII0_TXD7, OMAP_MUX_MODE1),
	// 258 GMII0_TXEN
	TI814X_MUX(GMII0_TXEN, OMAP_MUX_MODE1),
	
	/* PIN mux for RTC I2C1(0~3) ======================== */
	TI814X_MUX(I2C1_SCL, OMAP_MUX_MODE0),
	TI814X_MUX(I2C1_SDA, OMAP_MUX_MODE0),
	TI814X_MUX(VIN0_FLD0_MUX1,OMAP_MUX_MODE1),//set this pin to FLD0
	TI814X_MUX(VIN0_FLD0_MUX0,OMAP_MUX_MODE7),//set this pin to other function
	TI814X_MUX(VIN0_DE0_MUX0,OMAP_MUX_MODE7),//set this pin to other function
	TI814X_MUX(VIN0_DE0_MUX1,OMAP_MUX_MODE1),//set this pin to DE
	/* PIN mux for XXX ====================== */

	/* PIN mux for MMC 2 ======================== */
#if 0 // 
	/* PIN mux for GPIO ======================== */
	// 51 MCASP4_CLKX GP0[21]
	TI814X_MUX(MCASP4_CLKX, OMAP_MUX_MODE7),
	// 53 MCASP4_AXR0 GP0[23]
	TI814X_MUX(MCASP4_AXR0, OMAP_MUX_MODE7),
	// 54 MCASP4_AXR1 GP0[24]
	TI814X_MUX(MCASP4_AXR1, OMAP_MUX_MODE7),
	// 55 MCASP5_CLKX GP0[25]
	TI814X_MUX(MCASP5_CLKX, OMAP_MUX_MODE7),
	// 56 MCASP5_FSX GP0[26]
	TI814X_MUX(MCASP5_FSX, OMAP_MUX_MODE7),
	// 57 MCASP5_AXR0 GP0[27]
	TI814X_MUX(MCASP5_AXR0, OMAP_MUX_MODE7),
	// 58 MCASP5_AXR1 GP0[28]
	TI814X_MUX(MCASP5_AXR1, OMAP_MUX_MODE7),

	// 208 VOUT1_B_CB_C3 GP3[0]
	TI814X_MUX(VOUT1_B_CB_C3, OMAP_MUX_MODE7),
	// 209 VOUT1_B_CB_C4 GP3[1]
	TI814X_MUX(VOUT1_B_CB_C4, OMAP_MUX_MODE7),
	// 210 VOUT1_B_CB_C5 GP3[2]
	TI814X_MUX(VOUT1_B_CB_C5, OMAP_MUX_MODE7),
	// 211 VOUT1_B_CB_C6 GP3[3]
	TI814X_MUX(VOUT1_B_CB_C6, OMAP_MUX_MODE7),
	/* PIN mux for xxxx ======================== */	
#endif

#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux     NULL
#endif

static struct omap2_hsmmc_info mmc[] = {
#ifdef ACL_MMC2
	{
		.mmc		= 2, // hbchen , on board emmc
		.order		= 0,
		.caps		= MMC_CAP_4_BIT_DATA,
		.transceiver	= false,
		.nonremovable	= true,
		.gpio_cd	= -EINVAL, /* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
#endif
	{
		.mmc		= 1, // hbchen , sd card
		.order		= 1, // hbchen add element
		.caps		= MMC_CAP_4_BIT_DATA,
#if 0 //def ACL_CORSICA1 
		.gpio_cd	= -EINVAL, // hbchen
#else
		.gpio_cd	= GPIO_TO_PIN(1, 6), /* Dedicated pins for CD and WP */
#endif		
		//.gpio_wp	= -EINVAL,
		.gpio_wp	= GPIO_TO_PIN(1, 3),
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

#define GPIO_LCD_PWR_DOWN	0

static int setup_gpio_ioexp(struct i2c_client *client, int gpio_base,
	 unsigned ngpio, void *context) {
	int ret = 0;
	int gpio = gpio_base + GPIO_LCD_PWR_DOWN;

	ret = gpio_request(gpio, "lcd_power");
	if (ret) {
		printk(KERN_ERR "%s: failed to request GPIO for LCD Power"
			": %d\n", __func__, ret);
		return ret;
	}

	gpio_export(gpio, true);
	gpio_direction_output(gpio, 0);

	return 0;
}

/* IO expander data */
static struct pcf857x_platform_data io_expander_data = {
	.gpio_base	= 4 * 32,
	.setup		= setup_gpio_ioexp,
};
static struct i2c_board_info __initdata ti814x_i2c_boardinfo1[] = {
#ifdef ACL_CORSICA1// hbchen 
	{
		I2C_BOARD_INFO("s35390a", 0x30), // RTC
	},
#if 0 // (ACL_NETRA) // hbchen 
 	{
 		I2C_BOARD_INFO("tvp5158_audio", 0x58),
 	},
#endif
#else
	{
		I2C_BOARD_INFO("pcf8575_1", 0x21),
	},
#endif	

};

#define VPS_VC_IO_EXP_RESET_DEV_MASK        (0x0Fu)
#define VPS_VC_IO_EXP_SEL_VIN0_S1_MASK      (0x04u)
#define VPS_VC_IO_EXP_THS7368_DISABLE_MASK  (0x10u)
#define VPS_VC_IO_EXP_THS7368_BYPASS_MASK   (0x20u)
#define VPS_VC_IO_EXP_THS7368_FILTER1_MASK  (0x40u)
#define VPS_VC_IO_EXP_THS7368_FILTER2_MASK  (0x80u)
#define VPS_VC_IO_EXP_THS7368_FILTER_SHIFT  (0x06u)
#define pcf8575_IR_REMOTE_OFF (0x40)


static const struct i2c_device_id pcf8575_video_id[] = {
	{ "pcf8575_1", 0 },
	{ }
};
static struct i2c_client *pcf8575_client;
static unsigned char pcf8575_port[2] = {0x4F, 0x7F};
int vps_ti814x_select_video_decoder(int vid_decoder_id);

static int pcf8575_video_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	pcf8575_client = client;
	vps_ti814x_select_video_decoder(0);
	return 0;
}

static int __devexit pcf8575_video_remove(struct i2c_client *client)
{
	pcf8575_client = NULL;
	return 0;
}

static struct i2c_driver pcf8575_driver = {
	.driver = {
		.name   = "pcf8575_1",
	},
	.probe          = pcf8575_video_probe,
	.remove         = pcf8575_video_remove,
	.id_table       = pcf8575_video_id,
};
static const struct i2c_device_id pcf8575_cir_id[] = {
	{ "IO Expander", 0 },
	{ }
};
static struct i2c_client *pcf8575_cir_client;
static unsigned char pcf8575_cir_port[2] = {0, 0xbf};
static int pcf8575_cir_enable(void);
static int pcf8575_cir_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	pcf8575_cir_client = client;
	pcf8575_cir_enable();
	return 0;
}

static int __devexit pcf8575_cir_remove(struct i2c_client *client)
{
	pcf8575_cir_client = NULL;
	return 0;
}
static struct i2c_driver pcf8575_cir_driver = {
	.driver = {
		.name	= "IO Expander",
	},
	.probe		= pcf8575_cir_probe,
	.remove		= pcf8575_cir_remove,
	.id_table		= pcf8575_cir_id,
};
int ti814x_pcf8575_cir_init(void)
{
	i2c_add_driver(&pcf8575_cir_driver);
	return 0;
}

int ti814x_pcf8575_cir_exit(void)
{
	i2c_del_driver(&pcf8575_cir_driver);
	return 0;
}
int ti814x_pcf8575_init(void)
{
	i2c_add_driver(&pcf8575_driver);
	return 0;
}
EXPORT_SYMBOL(ti814x_pcf8575_init);

int ti814x_pcf8575_exit(void)
{
	i2c_del_driver(&pcf8575_driver);
	return 0;
}
EXPORT_SYMBOL(ti814x_pcf8575_exit);
#define VPS_VC_IO_EXP_RESET_DEV_MASK        (0x0Fu)
#define VPS_VC_IO_EXP_SEL_VIN0_S1_MASK      (0x04u)
#define VPS_VC_IO_EXP_THS7368_DISABLE_MASK  (0x10u)
#define VPS_VC_IO_EXP_THS7368_BYPASS_MASK   (0x20u)
#define VPS_VC_IO_EXP_THS7368_FILTER1_MASK  (0x40u)
#define VPS_VC_IO_EXP_THS7368_FILTER2_MASK  (0x80u)
#define VPS_VC_IO_EXP_THS7368_FILTER_SHIFT  (0x06u)
static void cir_pin_mux(void)
{
	char mux_name[100];
	sprintf(mux_name, "uart0_rin.uart1_rxd_mux0");
	omap_mux_init_signal(mux_name, OMAP_MUX_MODE0 |
			TI814X_PULL_DIS | TI814X_INPUT_EN);
	return;
}

int pcf8575_cir_enable(void)
{
	int ret = 0;
	struct i2c_msg msg = {
		.addr = pcf8575_cir_client->addr,
		.flags = 1,
		.len = 2,
	};
	msg.buf = pcf8575_cir_port;
	ret = i2c_transfer(pcf8575_cir_client->adapter, &msg, 1);
	msg.flags = 0;
	if (ret < 0)
		printk(KERN_ERR "I2C: Read failed at %s %d with error code: %d\n",
			__func__, __LINE__, ret);
	pcf8575_cir_port[0] = msg.buf[0];
	pcf8575_cir_port[1] = (msg.buf[1] & ~(pcf8575_IR_REMOTE_OFF));
	ret = i2c_transfer(pcf8575_cir_client->adapter, &msg, 1);
	cir_pin_mux();
	if (ret < 0)
		printk(KERN_ERR "I2C: Transfer failed at %s %d with error code: %d\n",
			__func__, __LINE__, ret);
	return ret;

}
int vps_ti814x_select_video_decoder(int vid_decoder_id)
{
	int ret = 0;
	struct i2c_msg msg = {
			.addr = pcf8575_client->addr,
			.flags = 0,
			.len = 2,
		};
	msg.buf = pcf8575_port;
	if (VPS_SEL_TVP7002_DECODER == vid_decoder_id)
		pcf8575_port[1] &= ~VPS_VC_IO_EXP_SEL_VIN0_S1_MASK;
	else
		pcf8575_port[1] |= VPS_VC_IO_EXP_SEL_VIN0_S1_MASK;
	ret = (i2c_transfer(pcf8575_client->adapter, &msg, 1));
	if (ret < 0)
		printk(KERN_ERR "I2C: Transfer failed at %s %d with error code: %d\n",
			__func__, __LINE__, ret);
	return ret;
}
EXPORT_SYMBOL(vps_ti814x_select_video_decoder);

#define I2C_RETRY_COUNT 10u
int vps_ti814x_set_tvp7002_filter(enum fvid2_standard standard)
{
	int filter_sel;
	int ret;
	struct i2c_msg msg = {
			.addr = pcf8575_client->addr,
			.flags = 0,
			.len = 2,
		};

	pcf8575_port[0] &= ~(VPS_VC_IO_EXP_THS7368_DISABLE_MASK
		| VPS_VC_IO_EXP_THS7368_BYPASS_MASK
		| VPS_VC_IO_EXP_THS7368_FILTER1_MASK
		| VPS_VC_IO_EXP_THS7368_FILTER2_MASK);
	switch (standard) {
	case FVID2_STD_1080P_60:
	case FVID2_STD_1080P_50:
	case FVID2_STD_SXGA_60:
	case FVID2_STD_SXGA_75:
	case FVID2_STD_SXGAP_60:
	case FVID2_STD_SXGAP_75:
	case FVID2_STD_UXGA_60:
		filter_sel = 0x03u;  /* Filter2: 1, Filter1: 1 */
		break;
	case FVID2_STD_1080I_60:
	case FVID2_STD_1080I_50:
	case FVID2_STD_1080P_24:
	case FVID2_STD_1080P_30:
	case FVID2_STD_720P_60:
	case FVID2_STD_720P_50:
	case FVID2_STD_SVGA_60:
	case FVID2_STD_SVGA_72:
	case FVID2_STD_SVGA_75:
	case FVID2_STD_SVGA_85:
	case FVID2_STD_XGA_60:
	case FVID2_STD_XGA_70:
	case FVID2_STD_XGA_75:
	case FVID2_STD_XGA_85:
	case FVID2_STD_WXGA_60:
	case FVID2_STD_WXGA_75:
	case FVID2_STD_WXGA_85:
		filter_sel = 0x01u;  /* Filter2: 0, Filter1: 1 */
		break;
	case FVID2_STD_480P:
	case FVID2_STD_576P:
	case FVID2_STD_VGA_60:
	case FVID2_STD_VGA_72:
	case FVID2_STD_VGA_75:
	case FVID2_STD_VGA_85:
		filter_sel = 0x02u;  /* Filter2: 1, Filter1: 0 */
		break;
	case FVID2_STD_NTSC:
	case FVID2_STD_PAL:
	case FVID2_STD_480I:
	case FVID2_STD_576I:
	case FVID2_STD_D1:
		filter_sel = 0x00u;  /* Filter2: 0, Filter1: 0 */
		break;

	default:
		filter_sel = 0x01u;  /* Filter2: 0, Filter1: 1 */
		break;
	}
	pcf8575_port[0] |=
		(filter_sel << VPS_VC_IO_EXP_THS7368_FILTER_SHIFT);
	msg.buf = pcf8575_port;
	ret =  (i2c_transfer(pcf8575_client->adapter, &msg, 1));
	if (ret < 0) {
		printk(KERN_ERR "I2C: Transfer failed at %s %d with error code: %d\n",
			__func__, __LINE__, ret);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(vps_ti814x_set_tvp7002_filter);
/* Touchscreen platform data */
static struct qt602240_platform_data ts_platform_data = {
	.x_line		= 18,
	.y_line		= 12,
	.x_size		= 800,
	.y_size		= 480,
	.blen		= 0x01,
	.threshold	= 30,
	.voltage	= 2800000,
	.orient		= QT602240_HORIZONTAL_FLIP,
};

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
};

static struct regulator_consumer_supply ti8148evm_mpu_supply =
	REGULATOR_SUPPLY("mpu", NULL);

/*
 * DM814x/AM387x (TI814x) devices have restriction that none of the supply to
 * the device should be turned of.
 *
 * NOTE: To prevent turning off regulators not explicitly consumed by drivers
 * depending on it, ensure following:
 *	1) Set always_on = 1 for them OR
 *	2) Avoid calling regulator_has_full_constraints()
 *
 * With just (2), there will be a warning about incomplete constraints.
 * E.g., "regulator_init_complete: incomplete constraints, leaving LDO8 on"
 *
 * In either cases, the supply won't be disabled.
 *
 * We are taking approach (1).
 */
static struct regulator_init_data tps65911_reg_data[] = {
	/* VRTC */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VIO -VDDA 1.8V */
	{
		.constraints = {
			.min_uV = 1500000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VDD1 - MPU */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
		.num_consumer_supplies	= 1,
		.consumer_supplies	= &ti8148evm_mpu_supply,
	},

	/* VDD2 - DSP */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1500000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* VDDCtrl - CORE */
	{
		.constraints = {
			.min_uV = 600000,
			.max_uV = 1400000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO1 - VDAC */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO2 - HDMI */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO3 - GPIO 3.3V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO4 - PLL 1.8V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO5 - SPARE */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO6 - CDC */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.always_on = 1,
		},
	},

	/* LDO7 - SPARE */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},

	/* LDO8 - USB 1.8V */
	{
		.constraints = {
			.min_uV = 1100000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
			.always_on = 1,
		},
	},
};

static struct tps65910_board __refdata tps65911_pdata = {
	.irq				= 0,	/* No support currently */
	.gpio_base			= 0,	/* No support currently */
	.tps65910_pmic_init_data	= tps65911_reg_data,
};

static struct i2c_board_info __initdata ti814x_i2c_boardinfo[] = {
#if 0 //def ACL_CORSICA1
	// need add any item for app to contact with fpga ?? hbchen 
	// advantech fpga i2c[0]
 	{
		I2C_BOARD_INFO("corsica_fpga", 0x2f), // fpga
	},
#else
	{
		I2C_BOARD_INFO("eeprom", 0x50),
		.platform_data	= &eeprom_info,
	},
	{
		I2C_BOARD_INFO("cpld", 0x23),
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("IO Expander", 0x20),
	},
	{
		I2C_BOARD_INFO("tlc59108", 0x40),
	},
	{
		I2C_BOARD_INFO("pcf8575", 0x21),
		.platform_data = &io_expander_data,
	},
	{
		I2C_BOARD_INFO("qt602240_ts", 0x4A),
		.platform_data = &ts_platform_data,
	},
	{
		I2C_BOARD_INFO("tps65911", 0x2D),
		.platform_data = &tps65911_pdata,
	},
#endif	
};

static void __init ti814x_tsc_init(void)
{
	int error;

	omap_mux_init_signal("mlb_clk.gpio0_31", TI814X_PULL_DIS | (1 << 18));

	error = gpio_request(GPIO_TSC, "ts_irq");
	if (error < 0) {
		printk(KERN_ERR "%s: failed to request GPIO for TSC IRQ"
			": %d\n", __func__, error);
		return;
	}

	gpio_direction_input(GPIO_TSC);
	ti814x_i2c_boardinfo[6].irq = gpio_to_irq(GPIO_TSC);

	gpio_export(31, true);
}

//#ifdef ACL_CORSICA1	// hbchen
#if 1 // hbchen
#define Wifi_SW		21
#define WPS_SW		23
#define AP_LED		24
#define S_LED_R		25 // status led
#define S_LED_YG	26
#define REC_LED		27
#define POW_LED		28
#define SD_POW		34
#define SD_LED		36
#define FPGA_INT_0	96 // pinctl 208 GP3[0]
#define FPGA_INT_1	97
#define FPGA_INT_2	98
#define FPGA_INT_3	99

static void __init corsica_pinmux_gpio_init(void)
{
	int error;

	// =========================
	// EMAC1 PinMux
	omap_writel(0x00000106, 0x48140650);
	// =========================
	// xxxx PinMux
	// =========================
	// GPIO PinMux
	omap_writel(0x00060040, 0x48140928); // 75 GP1[3]
	
	omap_writel(0x00040080, 0x481408c8); // 51 GP0[21]
	omap_writel(0x000c0080, 0x481408d0); // 53 GP0[23]
	omap_writel(0x000c0080, 0x481408d4); // 54 GP0[24]
	omap_writel(0x000c0080, 0x481408d8); // 55 GP0[25]
	omap_writel(0x000c0080, 0x481408dc); // 56 GP0[26]
	omap_writel(0x000c0080, 0x481408e0); // 57 GP0[27]
	omap_writel(0x000c0080, 0x481408e4); // 58 GP0[28]
	omap_writel(0x000c0080, 0x48140924); // 74 GP1[2] SD POW
	omap_writel(0x000c0080, 0x4814092c); // 76 GP1[4] SD LED
	omap_writel(0x000c0080, 0x481408e4); // 208 GP3[0]
	omap_writel(0x000c0080, 0x481408e4); // 209 GP3[1]
	omap_writel(0x000c0080, 0x481408e4); // 210 GP3[2]
	omap_writel(0x000c0080, 0x481408e4); // 211 GP3[3]
	// =========================
	error = gpio_request(Wifi_SW, "Wifi_SW");
	error |= gpio_request(WPS_SW, "WPS_SW");
	error |= gpio_request(AP_LED, "AP_LED");
	error |= gpio_request(S_LED_R, "S_LED_R");
	error |= gpio_request(S_LED_YG, "S_LED_YG");
	error |= gpio_request(REC_LED, "REC_LED");
	error |= gpio_request(SD_LED, "SD_LED");
	error |= gpio_request(SD_POW, "SD_POW");
	error |= gpio_request(POW_LED, "POW_LED");
	error |= gpio_request(FPGA_INT_0, "FPGA_INT_0");
	error |= gpio_request(FPGA_INT_1, "FPGA_INT_1");
	error |= gpio_request(FPGA_INT_2, "FPGA_INT_2");
	error |= gpio_request(FPGA_INT_3, "FPGA_INT_3");
	if (error < 0) {
		printk(KERN_ERR "%s: failed to request Corsica GPIO "
			": %d\n", __func__, error);
		return;
	}

	gpio_direction_output(AP_LED,1);
	gpio_direction_output(S_LED_R,1);	
	gpio_direction_output(S_LED_YG,1);	
	gpio_direction_output(REC_LED,1);	
	gpio_direction_output(POW_LED,1);	
	gpio_direction_output(SD_POW,1);// SD pow output
	gpio_direction_output(SD_LED,1);// SD LED output
	gpio_export(AP_LED, true);
	gpio_export(S_LED_R, true);
	gpio_export(S_LED_YG, true);
	gpio_export(REC_LED, true);
	gpio_export(POW_LED, true);
	gpio_export(SD_POW, true);
	gpio_export(SD_LED, true);

	gpio_set_value(POW_LED, 0);	
	gpio_set_value(SD_POW, 0);	// Initial power ON

	gpio_direction_input(Wifi_SW);
	gpio_direction_input(WPS_SW);
	gpio_direction_input(FPGA_INT_0);
	gpio_direction_input(FPGA_INT_1);
	gpio_direction_input(FPGA_INT_2);
	gpio_direction_input(FPGA_INT_3);

	gpio_export(Wifi_SW, true);
	gpio_export(WPS_SW, true);
	gpio_export(FPGA_INT_0, true);
	gpio_export(FPGA_INT_1, true);
	gpio_export(FPGA_INT_2, true);
	gpio_export(FPGA_INT_3, true);
	
//	gpio_free(AP_LED); // Release this pin out of gpio driver.
	gpio_free(SD_LED);
}
#endif


#if 0 //def ACL_CORSICA1	 // hbchen 
// Leo , fpga driver for shutdown
static struct i2c_client *fpga_client;
static int fpga_probe(struct i2c_client *client,
			const struct i2c_device_id *id){
	if (client->addr == 0x2f){
		fpga_client = client;
		u8 data[2]={0x0a,0x01}; // fpga spi flash miso -> dm8168 spi miso
		struct i2c_msg msg = {
			.addr = fpga_client->addr,
			.flags = 0,
			.len = 2,
			.buf = data,
		};
		i2c_transfer(fpga_client->adapter, &msg, 1);
	}
	else{
		BUG();
		printk(KERN_ERR "******\n Probe FPGA Driver error !!!\n\n");
	}
	printk(KERN_ERR "******\n Probe FPGA Driver !!!\n\n");
	return 0;
}
static const struct i2c_device_id fpga_reg_ids[] = {
		{ "corsica_fpga", 0, },
		{ },
};
static struct i2c_driver corsica_fpga_driver = {
	.driver.name    = "corsica_fpga",
	.id_table       = fpga_reg_ids,
	.probe          = fpga_probe,
};	
#endif


static void __init ti814x_evm_i2c_init(void)
{
	/* There are 4 instances of I2C in TI814X but currently only one
	 * instance is being used on the TI8148 EVM
	 */	
#ifdef ACL_CORSICA1	 // hbchen , only for rtc
	omap_register_i2c_bus(1, 100, ti814x_i2c_boardinfo,
				ARRAY_SIZE(ti814x_i2c_boardinfo));	
	omap_register_i2c_bus(2, 100, ti814x_i2c_boardinfo1,
				ARRAY_SIZE(ti814x_i2c_boardinfo1));
#else // orig
	omap_register_i2c_bus(1, 400, ti814x_i2c_boardinfo,
				ARRAY_SIZE(ti814x_i2c_boardinfo));	
	omap_register_i2c_bus(3, 400, ti814x_i2c_boardinfo1,
				ARRAY_SIZE(ti814x_i2c_boardinfo1));
#endif
}

static u8 ti8148_iis_serializer_direction[] = {
#ifdef CONFIG_SND_SOC_ACLVFE_AUDIO // merge from 1.09 8662
	RX_MODE,	TX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
#else		
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
#endif	
};

static struct snd_platform_data ti8148_evm_snd_data = {
	//.tx_dma_offset	= 0x46800000, // mcasp 2
	//.rx_dma_offset	= 0x46800000,
	.tx_dma_offset	= 0x46400000, // hbchen try mcbsp 1
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(ti8148_iis_serializer_direction),
#ifdef ACL_CORSICA1	// hbchen
	.tdm_slots	= 2, // for 4ch audio, tdm slots should be 4 ?? hbchen
#else
	.tdm_slots	= 2,
#endif	
	.serial_dir	= ti8148_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

/* NOR Flash partitions */
static struct mtd_partition ti814x_evm_norflash_partitions[] = {
	/* bootloader (U-Boot, etc) in first 5 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 2 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name		= "env",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 25 * SZ_2M,
		.mask_flags	= 0
	},
	/* reserved */
	{
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

/* NAND flash information */
static struct mtd_partition ti814x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "U-Boot-min",
		.offset         = 0,    /* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,/* Offset = 0x0 + 128K */
		.size           = 18 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
        .name           = "U-Boot Logo",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
        .size           = 24 * SZ_128K,
    },	
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x580000 */
		.size           = 34 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x9C0000 */
		.size           = 1601 * SZ_128K,
	},
	{
		.name           = "Reserved",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0xD1E0000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* SPI fLash information */
struct mtd_partition ti8148_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
#if 1
	{
		.name		= "UBOOT SPI FLASH",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= MTDPART_SIZ_FULL,
	},
#else
	{
		.name		= "U-Boot-min",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 32 * SZ_4K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND, /* 0x0 + (32*4K) */
		.size		= 64 * SZ_4K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND, /* 0x40000 + (32*4K) */
		.size		= 2 * SZ_4K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND, /* 0x42000 + (32*4K) */
		.size		= 640 * SZ_4K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND, /* 0x2C2000 + (32*4K) */
		.size		= MTDPART_SIZ_FULL, /* size ~= 1.1 MiB */
	}
#endif		
};

const struct flash_platform_data ti8148_spi_flash = {
	.type		= "sst25vf040b", // w25x32
	.name		= "spi_flash",
	.parts		= ti8148_spi_partitions,
	.nr_parts	= ARRAY_SIZE(ti8148_spi_partitions),
};

#if 1 // def ACL_CORSICA1// hbchen 
struct mtd_partition ti814x_fpga_spi_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "FPGA RTL Code",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= MTDPART_SIZ_FULL,
	},
};

const struct flash_platform_data ti814x_fpga_spi_flash = {
	.type		= "m25p16",
	.name		= "spi_flash",
	.parts		= ti814x_fpga_spi_partitions,
	.nr_parts	= ARRAY_SIZE(ti814x_fpga_spi_partitions),
};
#endif

struct spi_board_info __initdata ti8148_spi_slave_info[] = {
	{
		.modalias	= "m25p80",
		.platform_data	= &ti8148_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 50000000,//75000000, // hbchen merge from s--
		.bus_num	= 1,
		.chip_select	= 0,
	},
#if 1 // def ACL_CORSICA1// hbchen 
	{
		.modalias	= "m25p80",
		.platform_data	= &ti814x_fpga_spi_flash,
		.irq		= -1,
		.max_speed_hz	= 7500000,
		.bus_num	= 2,
		.chip_select	= 0,
	},
#endif
};

void __init ti8148_spi_init(void)
{
	spi_register_board_info(ti8148_spi_slave_info,
				ARRAY_SIZE(ti8148_spi_slave_info));
}

/********** LED definition **************/

static struct gpio_led gpio_leds[] = {
	{
		.name			= "corsica::usr0",
		.default_trigger	= "mmc1", // "heartbeat", // "mmc1"
		.gpio			= SD_LED,
		.active_low		= false,
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


static struct platform_device *corsica_lowspeed_devices[] __initdata = {
	&leds_gpio,
};



/****************************************/

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power		= 500,
	.instances	= 1,
};

static void __init ti8148_evm_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
}

#ifdef CONFIG_SND_SOC_ACLVFE_AUDIO // merge from 1.09 8662 , hbchen

static struct platform_device aclvfe_audio_device = {
	.name	= "aclvfe-dummy-codec",
	.id	= -1,
};

static struct platform_device *aclvfe_devices[] __initdata = {
	&aclvfe_audio_device,
};
#endif

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_hdmi_platform_data ti8148_snd_hdmi_pdata = {
	.dma_addr = TI81xx_HDMI_WP + HDMI_WP_AUDIO_DATA,
	.channel = 53,
	.data_type = 4,
	.acnt = 4,
	.fifo_level = 0x20,
};

static struct platform_device ti8148_hdmi_audio_device = {
	.name   = "hdmi-dai",
	.id     = -1,
    .num_resources = 0,
	.dev = {
		.platform_data = &ti8148_snd_hdmi_pdata,
	}
};

static struct platform_device ti8148_hdmi_codec_device = {
	.name   = "hdmi-dummy-codec",
	.id     = -1,
};

static struct platform_device *ti8148_devices[] __initdata = {
	&ti8148_hdmi_audio_device,
	&ti8148_hdmi_codec_device,
};

/*
 * HDMI Audio Auto CTS MCLK configuration.
 * sysclk20, sysclk21, sysclk21 and CLKS(external)
 * setting sysclk20 as the parent of hdmi_i2s_ck
 * ToDo:
 */
void __init ti8148_hdmi_clk_init(void)
{
	int ret = 0;
	struct clk *parent, *child;

	/* modify the clk name to choose diff clk*/
	parent = clk_get(NULL, "sysclk20_ck");
	if (IS_ERR(parent))
		pr_err("Unable to get [sysclk20_ck] clk\n");

	child = clk_get(NULL, "hdmi_i2s_ck");
	if (IS_ERR(child))
		pr_err("Unable to get [hdmi_i2s_ck] clk\n");

	ret = clk_set_parent(child, parent);
	if (ret < 0)
		pr_err("Unable to set parent clk [hdmi_i2s_ck]\n");

	clk_put(child);
	clk_put(parent);
	pr_debug("{{HDMI Audio MCLK setup completed}}\n");
}

#endif

#ifdef CONFIG_SND_SOC_TVP5158_AUDIO
static struct platform_device tvp5158_audio_device = {
	.name	= "tvp5158-audio",
	.id	= -1,
};
#endif

#define LSI_PHY_ID		0x0282F014
#define LSI_PHY_MASK		0xffffffff
#define PHY_CONFIG_REG		22

static int ti8148_evm_lsi_phy_fixup(struct phy_device *phydev)
{
	unsigned int val;

	/* This enables TX_CLK-ing in case of 10/100MBps operation */
	val = phy_read(phydev, PHY_CONFIG_REG);
	phy_write(phydev, PHY_CONFIG_REG, (val | BIT(5)));

	return 0;
}

static void __init ti8148_evm_init(void)
{
	int bw; /* bus-width */
	//u32 reg=0;

	ti814x_mux_init(board_mux);
	omap_serial_init();
	corsica_pinmux_gpio_init(); // hbchen
#ifndef ACL_CORSICA1// hbchen 
	ti814x_tsc_init();
#endif
	ti814x_evm_i2c_init();

#ifdef CONFIG_SND_SOC_TVP5158_AUDIO
	platform_device_register(&tvp5158_audio_device);
#endif

	ti81xx_register_mcasp(0, &ti8148_evm_snd_data);
	
#if 0//def ACL_CORSICA1// hbchen 
	i2c_add_driver(&corsica_fpga_driver);
#endif

	omap2_hsmmc_init(mmc);

	/* nand initialisation */
#if 0 // hbchen
	if (cpu_is_ti814x()) {
		u32 *control_status = TI81XX_CTRL_REGADDR(0x40);
		if (*control_status & (1<<16))
			bw = 2; /*16-bit nand if BTMODE BW pin on board is ON*/
		else
			bw = 0; /*8-bit nand if BTMODE BW pin on board is OFF*/
			
    #ifdef CONFIG_MACH_TI814XDVR
        bw = 0; /* use 8-bit always in 814xDVR board */
    #endif

		board_nand_init(ti814x_nand_partitions,
			ARRAY_SIZE(ti814x_nand_partitions), 0, bw);
	} else
		board_nand_init(ti814x_nand_partitions,
		ARRAY_SIZE(ti814x_nand_partitions), 0, NAND_BUSWIDTH_16);
#endif
	/* initialize usb */
	usb_musb_init(&musb_board_data);

	ti8148_spi_init(); // two spi flash in corsica, one is fpga , another is bootloader spi
#if 1 // hbchen	
	platform_add_devices(corsica_lowspeed_devices,
			ARRAY_SIZE(corsica_lowspeed_devices));
#ifdef CONFIG_SND_SOC_TI81XX_HDMI
	/*setup the clokc for HDMI MCLK*/
	ti8148_hdmi_clk_init();
	__raw_writel(0x0, DSS_HDMI_RESET);
	platform_add_devices(ti8148_devices, ARRAY_SIZE(ti8148_devices));
#endif
#endif
#ifdef CONFIG_SND_SOC_ACLVFE_AUDIO
	platform_add_devices(aclvfe_devices, ARRAY_SIZE(aclvfe_devices));
#endif

	regulator_use_dummy_regulator();
#ifndef ACL_CORSICA1// hbchen 
#if 0 //   hbchen 
	board_nor_init(ti814x_evm_norflash_partitions,
		ARRAY_SIZE(ti814x_evm_norflash_partitions), 0);
#endif
#endif	
	/* LSI Gigabit Phy fixup */
#if 0 // hbchen
	phy_register_fixup_for_uid(LSI_PHY_ID, LSI_PHY_MASK,
				   ti8148_evm_lsi_phy_fixup);
#endif
#if 0 // hbchen
#ifndef CONFIG_SND_TI81XX_ACLBRD 		
	ti814x_pcf8575_cir_init();
#endif
#endif
	//reg = omap_readl(0x48140928);
	//pr_err("0x48140928= 0x%x\n", reg);	
	//omap_writel(0x00060040, 0x48140928);// temp hbchen
}

static void __init ti8148_evm_map_io(void)
{
	omap2_set_globals_ti814x();
	ti81xx_map_common_io();
}

MACHINE_START(TI8148EVM, "ti8148evm")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= ti8148_evm_map_io,
	.reserve         = ti81xx_reserve,
	.init_irq	= ti8148_evm_init_irq,
	.init_machine	= ti8148_evm_init,
	.timer		= &omap_timer,
MACHINE_END
