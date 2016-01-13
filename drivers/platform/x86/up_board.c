/*
 * UP Board header pin GPIO driver.
 *
 * Copyright (c) 2016, Emutex Ltd.  All rights reserved.
 *
 * Author: Dan O'Donovan <dan@emutex.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/pca953x.h>

#define UP_BOARD_GPIO_EXP_I2C_BUS 5
#define UP_BOARD_GPIO_EXP0_BASE   32
#define UP_BOARD_GPIO_EXP1_BASE   48

#define UP_BOARD_SPIDEV_BUS_NUM 2
#define UP_BOARD_SPIDEV_MAX_CLK 25000000

struct up_board_info {
	struct pinctrl_map *pinmux_maps;
	unsigned num_pinmux_maps;
	struct i2c_board_info *i2c_devices;
	unsigned num_i2c_devices;
};

static bool spidev0 = true;
module_param(spidev0, bool, S_IRUGO);
MODULE_PARM_DESC(spidev0, "register a spidev device on SPI bus 2-0");

static unsigned long i2c_pullup_conf[] = {
	PIN_CONF_PACKED(PIN_CONFIG_BIAS_PULL_UP, 5000),
};

/* Maps pin functions on UP Board I/O pin header to specific CHT SoC devices */
static struct pinctrl_map up_pinmux_maps_v0_1[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:00", "INT33FF:00", NULL, "uart1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:01", "INT33FF:00", NULL, "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:00", "INT33FF:00", NULL, "i2c0"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:01", "INT33FF:00", NULL, "i2c1"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622A8:00", "INT33FF:00", NULL, "lpe"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:00", "INT33FF:03", NULL, "pwm0"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:01", "INT33FF:03", NULL, "pwm1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228E:01", "INT33FF:03", NULL, "spi2"),
};

static struct pinctrl_map up_pinmux_maps_v0_2[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:00", "up-pinctrl", NULL, "uart1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:01", "up-pinctrl", NULL, "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:00", "up-pinctrl", NULL, "i2c0"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:01", "up-pinctrl", NULL, "i2c1"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622A8:00", "up-pinctrl", NULL, "i2s0"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:00", "up-pinctrl", NULL, "pwm0"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:01", "up-pinctrl", NULL, "pwm1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228E:01", "up-pinctrl", NULL, "spi2"),

	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:00", "INT33FF:00", NULL, "uart1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:01", "INT33FF:00", NULL, "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:00", "INT33FF:00", NULL, "i2c0"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:01", "INT33FF:00", NULL, "i2c1"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622A8:00", "INT33FF:00", NULL, "lpe"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:00", "INT33FF:03", NULL, "pwm0"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:01", "INT33FF:03", NULL, "pwm1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228E:01", "INT33FF:03", NULL, "spi2"),

	/* Enable the I2C5 bus with internal pull-ups to reach the
	 * PCA9555 GPIO expanders on the v0.2 board version
	 */
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:05", "INT33FF:00", NULL, "i2c5"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("808622C1:05", "INT33FF:00", "I2C5_SCL",
				    i2c_pullup_conf),
	PIN_MAP_CONFIGS_PIN_DEFAULT("808622C1:05", "INT33FF:00", "I2C5_SDA",
				    i2c_pullup_conf),
};

static struct platform_device *up_pinctrl_dev;

static struct pca953x_platform_data gpio_exp0_pdata = {
	.gpio_base = UP_BOARD_GPIO_EXP0_BASE,
};

static struct pca953x_platform_data gpio_exp1_pdata = {
	.gpio_base = UP_BOARD_GPIO_EXP1_BASE,
};

static struct i2c_board_info up_i2c_devices_v0_2[] __initdata = {
	{
		I2C_BOARD_INFO("pca9555", 0x24),
		.platform_data = &gpio_exp0_pdata,
	},
	{
		I2C_BOARD_INFO("pca9555", 0x23),
		.platform_data = &gpio_exp1_pdata,
	},
};

static struct spi_board_info up_spidev0_info __initdata = {
	.modalias	= "spidev",
	.bus_num	= UP_BOARD_SPIDEV_BUS_NUM,
	.chip_select	= 0,
	.max_speed_hz   = UP_BOARD_SPIDEV_MAX_CLK,
};

static struct up_board_info up_board_info_v0_1 = {
	.pinmux_maps = up_pinmux_maps_v0_1,
	.num_pinmux_maps = ARRAY_SIZE(up_pinmux_maps_v0_1),
};

static struct up_board_info up_board_info_v0_2 = {
	.pinmux_maps = up_pinmux_maps_v0_2,
	.num_pinmux_maps = ARRAY_SIZE(up_pinmux_maps_v0_2),
	.i2c_devices = up_i2c_devices_v0_2,
	.num_i2c_devices = ARRAY_SIZE(up_i2c_devices_v0_2),
};

static const struct dmi_system_id up_board_id_table[] = {
	{
		/* TODO - remove when new BIOS is available with
		 * correct board version numbering
		 */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_MATCH(DMI_BOARD_VERSION, "V1.0"),
		},
		.driver_data = (void *)&up_board_info_v0_2
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_MATCH(DMI_BOARD_VERSION, "V0.2"),
		},
		.driver_data = (void *)&up_board_info_v0_2
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_MATCH(DMI_BOARD_VERSION, "V0.1"),
		},
		.driver_data = (void *)&up_board_info_v0_1
	},
	{ }
};

static int __init
up_board_init(void)
{
	const struct dmi_system_id *system_id;
	struct up_board_info *board_info;
	int ret;

	system_id = dmi_first_match(up_board_id_table);
	if (!system_id)
		return -ENXIO;

	board_info = system_id->driver_data;

	if (board_info->pinmux_maps) {
		ret = pinctrl_register_mappings(board_info->pinmux_maps,
						board_info->num_pinmux_maps);
		if (ret) {
			pr_err("Failed to register UP Board pinctrl mapping");
			goto fail_pinctrl_reg;
		}
	}

	if (board_info->i2c_devices) {
		ret = i2c_register_board_info(UP_BOARD_GPIO_EXP_I2C_BUS,
					      board_info->i2c_devices,
					      board_info->num_i2c_devices);
		if (ret) {
			pr_err("Failed to register UP Board i2c devices");
			goto fail_i2c_register;
		}
	}

	if (spidev0) {
		ret = spi_register_board_info(&up_spidev0_info, 1);
		if (ret) {
			pr_err("Failed to register UP Board spidev0 device");
			goto fail_spidev0_register;
		}
	}

	/* Create a virtual device to manage the UP Board GPIO pin header */
	up_pinctrl_dev = platform_device_alloc("up-pinctrl", -1);
	if (!up_pinctrl_dev) {
		ret = -ENOMEM;
		goto fail_pinctrl_dev_alloc;
	}
	ret = platform_device_add(up_pinctrl_dev);
	if (ret)
		goto fail_pinctrl_dev_add;

	return 0;

fail_pinctrl_dev_add:
	platform_device_put(up_pinctrl_dev);
fail_pinctrl_dev_alloc:
fail_spidev0_register:
fail_i2c_register:
fail_pinctrl_reg:
	return ret;
}

static void __exit
up_board_exit(void)
{
	platform_device_unregister(up_pinctrl_dev);
}

module_init(up_board_init);
module_exit(up_board_exit);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("Platform driver for UP Board");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("dmi:*:svnAAEON*:rnUP-CHT01:*");
