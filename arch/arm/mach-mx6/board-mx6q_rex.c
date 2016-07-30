/*
 * Copyright (C) 2014 Voipac. All Rights Reserved.
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/tsc2007.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_rex.h"
#include "board-mx6dl_rex.h"

#define REX_USR_DEF_RED_LED	IMX_GPIO_NR(1, 2)
#define REX_ENET_RST		IMX_GPIO_NR(1, 25)
#define REX_ENET_IRQ1		IMX_GPIO_NR(1, 26)
#define REX_ENET_IRQ2		IMX_GPIO_NR(1, 28)

#define REX_SD3_CD		IMX_GPIO_NR(2, 0)
#define REX_SD3_WP		IMX_GPIO_NR(2, 1)
#define REX_SD2_CD		IMX_GPIO_NR(2, 2)
#define REX_SD2_WP		IMX_GPIO_NR(2, 3)

#define REX_USB_OTG_PWR_EN	IMX_GPIO_NR(3, 22)
#define REX_USB_H1_PWR_EN	IMX_GPIO_NR(3, 31)

#define REX_ECSPI3_CS0		IMX_GPIO_NR(4, 26)

#define REX_ECSPI2_CS0		IMX_GPIO_NR(5, 12)
#define REX_PCIE_WAKE_B		IMX_GPIO_NR(5, 20)

#define REX_TSC2007_INT		IMX_GPIO_NR(6, 9)
#define REX_CPU_INT		IMX_GPIO_NR(6, 16)

static struct clk *sata_clk;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_rex_sd2_data __initconst = {
	.cd_gpio = REX_SD2_CD,
	.wp_gpio = REX_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_rex_sd3_data __initconst = {
	.cd_gpio = REX_SD3_CD,
	.wp_gpio = REX_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT, //.cd_type = ESDHC_CD_CONTROLLER,
	.always_present = 1,           //.runtime_pm = 1,
};

static const struct anatop_thermal_platform_data
	mx6q_rex_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_rex_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
}

static int mx6q_rex_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1c00);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* min tx data delay */
	phy_write(phydev, 0x0b, 0x8106);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_rex_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = -1,
};

static int mx6q_rex_spi1_cs[] = {
	REX_ECSPI2_CS0,
};

static const struct spi_imx_master mx6q_rex_spi1_data __initconst = {
	.chipselect     = mx6q_rex_spi1_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_rex_spi1_cs),
};

static int mx6q_rex_spi2_cs[] = {
	REX_ECSPI3_CS0,
};

static const struct spi_imx_master mx6q_rex_spi2_data __initconst = {
	.chipselect     = mx6q_rex_spi2_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_rex_spi2_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_rex_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_rex__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_rex_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_rex_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_rex_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = &imx6_rex__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_rex_spi_nor_device,
				ARRAY_SIZE(imx6_rex_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_rex_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_rex_audio_device = {
	.name = "imx-sgtl5000",
};

static int mx6_rex_sgtl5000_clk_enable(int enable)
{
	struct clk *clko;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}

	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static struct mxc_audio_platform_data mx6_rex_audio_data;

static int mx6_rex_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_rex_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct mxc_audio_platform_data mx6_rex_audio_data = {
	.ssi_num      = 1,
	.src_port     = 2,
	.ext_port     = 3,
	.hp_gpio      = -1,
	.mic_gpio     = -1,
	.init         = mx6_rex_sgtl5000_init,
	.clock_enable = mx6_rex_sgtl5000_clk_enable,
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_rex_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_rex_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_rex_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_rex_vdda_reg_initdata = {
	.constraints = {
		.name = "SGTL5000_VDDA",
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_rex_consumer_vdda,
};

static struct regulator_init_data sgtl5000_rex_vddio_reg_initdata = {
	.constraints = {
		.name = "SGTL5000_VDDIO",
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_rex_consumer_vddio,
};

static struct regulator_init_data sgtl5000_rex_vddd_reg_initdata = {
	.constraints = {
		.name = "SGTL5000_VDDD",
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_rex_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_rex_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_rex_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_rex_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_rex_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_rex_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &sgtl5000_rex_vddd_reg_initdata,
};

static struct platform_device sgtl5000_rex_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_rex_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_rex_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_rex_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_rex_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_rex_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_rex_audio_device,
			    &mx6_rex_audio_data);
	imx6q_add_imx_ssi(1, &mx6_rex_ssi_pdata);

#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_rex_vdda_reg_devices);
	platform_device_register(&sgtl5000_rex_vddio_reg_devices);
	platform_device_register(&sgtl5000_rex_vddd_reg_devices);
#endif

	return 0;
}



static int tsc2007_init_platform_hw(void)
{
	gpio_request(REX_TSC2007_INT, "tsc2007_irq");
	gpio_direction_input(REX_TSC2007_INT);
	gpio_free(REX_TSC2007_INT);

	return 0;
}

static void tsc2007_exit_platform_hw(void)
{
	;
}

static struct tsc2007_platform_data tsc2007_data = {
        .model            = 2007,
        .x_plate_ohms     = 500,
	.init_platform_hw = tsc2007_init_platform_hw,
	.exit_platform_hw = tsc2007_exit_platform_hw,
	.swap_xy          = 1,
	.mirror_x         = 1,
	.mirror_y         = 0,
};

static struct imxi2c_platform_data mx6q_rex_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq = gpio_to_irq(REX_TSC2007_INT),
		.platform_data  = &tsc2007_data,
	},
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
		.platform_data = (void *)0,	/* lvds port0 */
	},
};

static void imx6q_rex_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(REX_USB_OTG_PWR_EN, 1);
	else
		gpio_set_value(REX_USB_OTG_PWR_EN, 0);
}

static void imx6q_rex_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(REX_USB_H1_PWR_EN, 1);
	else
		gpio_set_value(REX_USB_H1_PWR_EN, 0);
}

static void __init imx6q_rex_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(REX_USB_OTG_PWR_EN, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO REX_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(REX_USB_OTG_PWR_EN, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(REX_USB_H1_PWR_EN, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO REX_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(REX_USB_H1_PWR_EN, 1);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_rex_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_rex_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_rex_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_rex_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_rex_sata_data = {
	.init = mx6q_rex_sata_init,
	.exit = mx6q_rex_sata_exit,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data rex_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = true,
	.late_init = false,
	},
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = true,
	.late_init = false,
	},
	{
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x rex board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_rex_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_rex_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_rex_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_rex_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_rex_i2c2_pads,
			ARRAY_SIZE(mx6dl_rex_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_rex_i2c2_pads,
			ARRAY_SIZE(mx6q_rex_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SIN0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 0,
		},
	},
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = 0,
};

static void rex_suspend_enter(void)
{
	/* suspend preparation */
}

static void rex_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_rex_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = rex_suspend_enter,
	.suspend_exit = rex_suspend_exit,
};

static struct regulator_consumer_supply rex_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data rex_vmmc_init = {
	.constraints = {
		.name = "SD_VMMC",
		.always_on = 1,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(rex_vmmc_consumers),
	.consumer_supplies = rex_vmmc_consumers,
};

static struct fixed_voltage_config rex_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &rex_vmmc_init,
};

static struct platform_device rex_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &rex_vmmc_reg_config,
	},
};

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= trigger,				\
}

static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(REX_USR_DEF_RED_LED, "usr-def-red-led", 0, 1, "heartbeat"),
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button rex_buttons[] = {
	GPIO_BUTTON(REX_CPU_INT, KEY_POWER, 1, "power-key", 1, 1)
};

static struct gpio_keys_platform_data rex_button_data = {
	.buttons	= rex_buttons,
	.nbuttons	= 1,//ARRAY_SIZE(new_rex_buttons),
};

static struct platform_device rex_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_add_data(&rex_button_device,
				&rex_button_data,
				sizeof(rex_button_data));

	platform_device_register(&rex_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_rex_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data rex_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = rex_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(rex_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_rex_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= -EINVAL,
	.pcie_wake_up	= REX_PCIE_WAKE_B,
	.pcie_dis	= -EINVAL,
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
	.pcie_power_always_on = 1,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

/*!
 * Board specific initialization.
 */
static void __init mx6_rex_board_init(void)
{
	int i;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_rex_pads,
			ARRAY_SIZE(mx6q_rex_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_rex_pads,
			ARRAY_SIZE(mx6dl_rex_pads));
	}

	gp_reg_id = rex_dvfscore_data.reg_id;
	soc_reg_id = rex_dvfscore_data.soc_id;
	mx6q_rex_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 1;
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(rex_fb_data); i++)
			imx6q_add_ipuv3fb(i, &rex_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(rex_fb_data); i++)
			imx6q_add_ipuv3fb(i, &rex_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_rex_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_rex_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_rex_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6q_rex_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(1, &mx6q_rex_spi1_data);
	imx6q_add_ecspi(2, &mx6q_rex_spi2_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_rex_anatop_thermal_data);

	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_rex_pm_data);

	/*
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_rex_sd3_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_rex_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_rex_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_rex_sata_data);
#else
		mx6q_rex_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&rex_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

#ifdef CONFIG_HAVE_EPIT
	imx6q_add_mxc_epit(0);
	imx6q_add_mxc_epit(1);
#endif

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm_backlight(0, &mx6_rex_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&rex_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_rex_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_rex_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_rex_timer = {
	.init   = mx6_rex_timer_init,
};

static void __init mx6_rex_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(rex_fb_data);
	if (fb_array_size > 0 && rex_fb_data[0].res_base[0] &&
	    rex_fb_data[0].res_size[0]) {
		if (rex_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    rex_fb_data[0].res_base[0]);
		memblock_reserve(rex_fb_data[0].res_base[0],
				 rex_fb_data[0].res_size[0]);
		memblock_remove(rex_fb_data[0].res_base[0],
				rex_fb_data[0].res_size[0]);
		rex_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (rex_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(rex_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, rex_fb_data[i].res_size[0]);
			rex_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_REX data structure.
 */
MACHINE_START(MX6Q_REX, "Fedevel i.MX 6Quad/DualLite Rex Board")
	/* Maintainer: Voipac */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_rex_board_init,
	.timer = &mx6_rex_timer,
	.reserve = mx6_rex_reserve,
MACHINE_END
