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
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>

#define UP_BOARD_GPIO_EXP_I2C_BUS 5
#define UP_BOARD_GPIO_EXP0_BASE   32
#define UP_BOARD_GPIO_EXP1_BASE   48

static unsigned long i2c_pullup_conf[] = {
	PIN_CONF_PACKED(PIN_CONFIG_BIAS_PULL_UP, 5000),
};

/* Maps pin functions on UP Board I/O pin header to specific CHT SoC devices */
static struct pinctrl_map pinmux_map[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:00", "INT33FF:00", NULL, "uart1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228A:01", "INT33FF:00", NULL, "uart2"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:00", "INT33FF:00", NULL, "i2c0"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:01", "INT33FF:00", NULL, "i2c1"),
	PIN_MAP_MUX_GROUP_DEFAULT("808622A8:00", "INT33FF:00", NULL, "lpe"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862288:00", "INT33FF:03", NULL, "pwm0"),
	PIN_MAP_MUX_GROUP_DEFAULT("80862289:00", "INT33FF:03", NULL, "pwm1"),
	PIN_MAP_MUX_GROUP_DEFAULT("8086228E:01", "INT33FF:03", NULL, "spi2"),

	/* Enable the I2C5 bus with internal pull-ups to reach the
	 * PCA9555 GPIO expanders on the EVT2 board
	 */
	PIN_MAP_MUX_GROUP_DEFAULT("808622C1:05", "INT33FF:00", NULL, "i2c5"),
	PIN_MAP_CONFIGS_PIN_DEFAULT("808622C1:05", "INT33FF:00", "I2C5_SCL",
				    i2c_pullup_conf),
	PIN_MAP_CONFIGS_PIN_DEFAULT("808622C1:05", "INT33FF:00", "I2C5_SDA",
				    i2c_pullup_conf),
};

static struct platform_device *up_gpio_dev;

static struct pca953x_platform_data gpio_exp0_pdata = {
	.gpio_base = UP_BOARD_GPIO_EXP0_BASE,
};

static struct pca953x_platform_data gpio_exp1_pdata = {
	.gpio_base = UP_BOARD_GPIO_EXP1_BASE,
};

static struct i2c_board_info gpio_exp_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("pca9555", 0x24),
		.platform_data = &gpio_exp0_pdata,
	},
	{
		I2C_BOARD_INFO("pca9555", 0x23),
		.platform_data = &gpio_exp1_pdata,
	},
};

static int __init
up_board_init(void)
{
	int ret;

	pr_info("Loading UP Board platform driver");

	/* Register default mappings to enable correct I/O pin functions */
	ret = pinctrl_register_mappings(pinmux_map, ARRAY_SIZE(pinmux_map));
	if (ret) {
		pr_err("Failed to register pinctrl mapping");
		goto fail_pinctrl_reg;
	}

	/* Register the I2C GPIO expanders on the EVT2 board
	 *
	 * TODO - this code should be in an EVT2-specific block
	 * Refactor this once the DMI board name has been updated for EVT2
	 * so that we can detect which board we're running on
	 */
	i2c_register_board_info(UP_BOARD_GPIO_EXP_I2C_BUS,
				gpio_exp_board_info,
				ARRAY_SIZE(gpio_exp_board_info));

	/* Create a virtual device to manage the UP Board GPIO pin header */
	up_gpio_dev = platform_device_alloc("up-gpio", -1);
	if (!up_gpio_dev) {
		ret = -ENOMEM;
		goto fail_gpio_dev_alloc;
	}
	ret = platform_device_add(up_gpio_dev);
	if (ret)
		goto fail_gpio_dev_add;

	return 0;

fail_gpio_dev_add:
	platform_device_put(up_gpio_dev);
fail_gpio_dev_alloc:
fail_pinctrl_reg:
	return ret;
}

static void __exit
up_board_exit(void)
{
	pr_info("Unloading UP Board platform driver");
	platform_device_unregister(up_gpio_dev);
}

module_init(up_board_init);
module_exit(up_board_exit);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("Platform driver for UP Board");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("dmi:*:svnAAEON*:rnUP-CHT01:*");
