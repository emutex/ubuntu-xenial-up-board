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
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

/*
 * The UP Board features an external 40-pin header for I/O functions including
 * GPIO, I2C, UART, SPI, PWM and I2S, similar in layout to the Raspberry Pi 2.
 * At the heart of the UP Board is an Intel X5-Z8300 "Cherry Trail" SoC, which
 * provides the I/O functions for these pins.  However, the I/O from the SoC is
 * 1.8V and not capable of sourcing or sinking significant current (i.e. not
 * LV-TTL or LV-CMOS compatible) so additional buffers are used between the SoC
 * and the I/O pin header to address that.  These buffers require software
 * configuration based on the pin function or GPIO direction selected by the
 * user.
 *
 * The purpose of this driver is to manage the complexity of the buffer
 * configuration so that application code can transparently access the I/O
 * functions on the external pins through standard kernel interfaces.  It
 * instantiates gpio and pinctrl devices, and effectively acts as a "proxy"
 * between application code and the underlying Cherry Trail GPIO driver.
 */

/* Information/references to Cherry Trail GPIO chip driver */
struct up_gpio_soc_gpiochip_info {
	char *name;
	struct gpio_chip *chip;
};

/* Information/references to Cherry Trail GPIO pins */
struct up_gpio_soc_gpio_info {
	struct up_gpio_soc_gpiochip_info *ci;
	struct gpio_desc *desc;
	unsigned offset;
	int irq;
};

/* Information for a single I/O pin on the UP board */
struct up_gpio_pin_info {
	struct up_gpio_soc_gpio_info soc_gpio;
	int irq;
};

/* The Cherry Trail SoC has 4 independent GPIO controllers */
static struct up_gpio_soc_gpiochip_info chip_cht_SW = { .name = "INT33FF:00" };
static struct up_gpio_soc_gpiochip_info chip_cht_N  = { .name = "INT33FF:01" };
static struct up_gpio_soc_gpiochip_info chip_cht_E  = { .name = "INT33FF:02" };
static struct up_gpio_soc_gpiochip_info chip_cht_SE = { .name = "INT33FF:03" };

#define GPIO_PIN(c, o)			\
	{				\
		.soc_gpio.ci = (c),	\
		.soc_gpio.offset = (o),	\
	}

#define N_GPIO 28

/*
 * Table of GPIO pins on the 40-pin I/O header of the UP Board
 * The layout and numbering is designed to emulate the Raspberry Pi 2
 */
static struct up_gpio_pin_info up_gpio_pins_v1[N_GPIO] = {
	GPIO_PIN(&chip_cht_SW, 33), /*  0 */
	GPIO_PIN(&chip_cht_SW, 37), /*  1 */
	GPIO_PIN(&chip_cht_SW, 32), /*  2 */
	GPIO_PIN(&chip_cht_SW, 35), /*  3 */
	GPIO_PIN(&chip_cht_SE,  7), /*  4 */
	GPIO_PIN(&chip_cht_SE,  4), /*  5 */
	GPIO_PIN(&chip_cht_SE,  3), /*  6 */
	GPIO_PIN(&chip_cht_N,  41), /*  7 */
	GPIO_PIN(&chip_cht_N,  36), /*  8 */
	GPIO_PIN(&chip_cht_E,  17), /*  9 */
	GPIO_PIN(&chip_cht_E,  23), /* 10 */
	GPIO_PIN(&chip_cht_E,  14), /* 11 */
	GPIO_PIN(&chip_cht_SE,  1), /* 12 */
	GPIO_PIN(&chip_cht_SE,  5), /* 13 */
	GPIO_PIN(&chip_cht_SW, 13), /* 14 */
	GPIO_PIN(&chip_cht_SW,  9), /* 15 */
	GPIO_PIN(&chip_cht_N,   6), /* 16 */
	GPIO_PIN(&chip_cht_SE,  6), /* 17 */
	GPIO_PIN(&chip_cht_SW, 17), /* 18 */
	GPIO_PIN(&chip_cht_SW, 21), /* 19 */
	GPIO_PIN(&chip_cht_SW, 19), /* 20 */
	GPIO_PIN(&chip_cht_SW, 16), /* 21 */
	GPIO_PIN(&chip_cht_N,   3), /* 22 */
	GPIO_PIN(&chip_cht_N,   2), /* 23 */
	GPIO_PIN(&chip_cht_N,   1), /* 24 */
	GPIO_PIN(&chip_cht_SW, 14), /* 25 */
	GPIO_PIN(&chip_cht_N,   7), /* 26 */
	GPIO_PIN(&chip_cht_SW, 10), /* 27 */
};

struct up_gpio {
	struct gpio_chip chip;
	struct up_gpio_pin_info *pin_info;
};

static inline struct up_gpio *to_up_gpio(struct gpio_chip *gc)
{
	return container_of(gc, struct up_gpio, chip);
}

static int up_gpiochip_match(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static int up_gpio_map_pins(struct up_gpio_pin_info *pin_info)
{
	unsigned i;

	/* Find the Cherry Trail GPIO descriptors corresponding
	 * with each GPIO pin on the UP Board I/O header
	 */
	for (i = 0; i < N_GPIO; i++) {
		struct up_gpio_pin_info *pin = &pin_info[i];
		struct up_gpio_soc_gpiochip_info *ci = pin->soc_gpio.ci;

		if (!ci->chip) {
			ci->chip = gpiochip_find(ci->name, up_gpiochip_match);
			if (!ci->chip)
				return -EPROBE_DEFER;
		}
		pin->soc_gpio.desc = gpio_to_desc(ci->chip->base
						  + pin->soc_gpio.offset);
		if (!pin->soc_gpio.desc)
			return -EINVAL;
	}

	return 0;
}

static irqreturn_t up_gpio_irq_handler(int irq, void *data)
{
	struct up_gpio_pin_info *pin = (struct up_gpio_pin_info *)data;

	generic_handle_irq(pin->irq);
	return IRQ_HANDLED;
}

static unsigned int up_gpio_irq_startup(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct up_gpio *up_gpio = to_up_gpio(gc);
	unsigned offset = irqd_to_hwirq(data);
	struct up_gpio_pin_info *pin = &up_gpio->pin_info[offset];

	return request_irq(pin->soc_gpio.irq, up_gpio_irq_handler,
			   IRQF_ONESHOT, "up-gpio", pin);
}

static void up_gpio_irq_shutdown(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct up_gpio *up_gpio = to_up_gpio(gc);
	unsigned offset = irqd_to_hwirq(data);
	struct up_gpio_pin_info *pin = &up_gpio->pin_info[offset];

	free_irq(pin->soc_gpio.irq, pin);
}

static struct irq_chip up_gpio_irqchip = {
	.name = "up-gpio",
	.irq_startup = up_gpio_irq_startup,
	.irq_shutdown = up_gpio_irq_shutdown,
	.irq_enable = irq_chip_enable_parent,
	.irq_disable = irq_chip_disable_parent,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_ack = irq_chip_ack_parent,
	.irq_set_type = irq_chip_set_type_parent,
};

static int up_gpio_dir_in(struct gpio_chip *chip, unsigned offset)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	return gpiod_direction_input(desc);
}

static int up_gpio_dir_out(struct gpio_chip *chip, unsigned offset,
				int value)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	return gpiod_direction_output(desc, value);
}

static int up_gpio_get_dir(struct gpio_chip *chip, unsigned offset)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	return gpiod_get_direction(desc);
}

static int up_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	return gpio_request(desc_to_gpio(desc), chip->label);
}

static void up_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	gpio_free(desc_to_gpio(desc));
}

static int up_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	return gpiod_get_value(desc);
}

static void up_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct up_gpio *up_gpio = to_up_gpio(chip);
	struct gpio_desc *desc = up_gpio->pin_info[offset].soc_gpio.desc;

	gpiod_set_value(desc, value);
}

static struct gpio_chip up_gpio_chip = {
	.owner			= THIS_MODULE,
	.ngpio			= N_GPIO,
	.request		= up_gpio_request,
	.free			= up_gpio_free,
	.get_direction		= up_gpio_get_dir,
	.direction_input	= up_gpio_dir_in,
	.direction_output	= up_gpio_dir_out,
	.get			= up_gpio_get,
	.set			= up_gpio_set,
};

static const struct dmi_system_id up_board_id_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
		},
		.driver_data = (void *)&up_gpio_pins_v1
	},
	{}
};

static int up_gpio_probe(struct platform_device *pdev)
{
	struct up_gpio *up_gpio;
	struct up_gpio_pin_info *pin_info;
	const struct dmi_system_id *system_id;
	unsigned offset;
	int ret;

	system_id = dmi_first_match(up_board_id_table);
	if (!system_id)
		return -ENXIO;
	pin_info = system_id->driver_data;

	ret = up_gpio_map_pins(pin_info);
	if (ret)
		return ret;

	up_gpio = devm_kzalloc(&pdev->dev, sizeof(*up_gpio), GFP_KERNEL);
	if (!up_gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, up_gpio);
	up_gpio->pin_info = pin_info;
	up_gpio->chip = up_gpio_chip;
	up_gpio->chip.label = dev_name(&pdev->dev);
	up_gpio->chip.dev = &pdev->dev;

	ret = gpiochip_add(&up_gpio->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to add %s chip\n",
			up_gpio->chip.label);
		return ret;
	}

	ret = gpiochip_irqchip_add(&up_gpio->chip, &up_gpio_irqchip, 0,
				   handle_simple_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&pdev->dev, "failed to add IRQ chip\n");
		goto fail_irqchip_add;
	}

	for (offset = 0; offset < up_gpio->chip.ngpio; offset++) {
		struct up_gpio_pin_info *pin = &up_gpio->pin_info[offset];
		struct irq_data *irq_data;

		pin->irq = irq_find_mapping(up_gpio->chip.irqdomain, offset);
		pin->soc_gpio.irq = gpiod_to_irq(pin->soc_gpio.desc);
		irq_set_parent(pin->irq, pin->soc_gpio.irq);
		irq_data = irq_get_irq_data(pin->irq);
		irq_data->parent_data = irq_get_irq_data(pin->soc_gpio.irq);
	}

	return 0;

fail_irqchip_add:
	gpiochip_remove(&up_gpio->chip);

	return ret;
}

static int up_gpio_remove(struct platform_device *pdev)
{
	struct up_gpio *up_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&up_gpio->chip);

	return 0;
}

static struct platform_driver up_gpio_driver = {
	.driver.name	= "up-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= up_gpio_probe,
	.remove		= up_gpio_remove,
};

static int __init up_gpio_init(void)
{
	return platform_driver_register(&up_gpio_driver);
}
subsys_initcall(up_gpio_init);

static void __exit up_gpio_exit(void)
{
	platform_driver_unregister(&up_gpio_driver);
}
module_exit(up_gpio_exit);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("GPIO driver for UP Board I/O pin header");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:up-gpio");
