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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

/*
 * The UP Board features an external 40-pin header for I/O functions including
 * GPIO, I2C, UART, SPI, PWM and I2S, similar in layout to the Raspberry Pi 2.
 * At the heart of the UP Board is an Intel X5-Z8300 "Cherry Trail" SoC, which
 * provides the I/O functions for these pins at 1.8V logic levels.
 *
 * Additional buffers and mux switches are used between the SoC and the I/O pin
 * header to convert between the 1.8V SoC I/O and the 3.3V levels required at
 * the pin header, with sufficient current source/sink capability for
 * LV-TTL/LV-CMOS compatibility.  These buffers and mux switches require
 * run-time configuration based on the pin function or GPIO direction selected
 * by the user.
 *
 * The purpose of this driver is to manage the complexity of the buffer
 * configuration so that application code can transparently access the I/O
 * functions on the external pins through standard kernel interfaces.  It
 * instantiates a gpio and pinctrl device, and effectively acts as a "shim"
 * between application code and the underlying Cherry Trail GPIO driver.
 */

/* References to Cherry Trail GPIO chip driver */
struct up_soc_gpiochip_info {
	char *name;
	struct gpio_chip *chip;
};

/* References to Cherry Trail GPIO pins */
struct up_soc_gpio_info {
	struct up_soc_gpiochip_info *ci;
	struct gpio_desc *desc;
	unsigned offset;
	int irq;
};

/* Information for a single I/O pin on the UP board */
struct up_pin_info {
	struct up_soc_gpio_info soc_gpio;
	int irq;
	int dir_gpio;
	int func_dir;
	int mux_gpio;
	int func_mux;
	bool func_sel;
};

/* Context variables for this driver */
struct up_pctrl {
	struct gpio_chip chip;
	struct up_pin_info *pin_info;
	struct pinctrl_desc pctldesc;
	struct pinctrl_dev *pctldev;
};

/* Pin group information */
struct up_pingroup {
	const char *name;
	const unsigned *pins;
	size_t npins;
};

/* Pin function information */
struct up_function {
	const char *name;
	const char * const *groups;
	size_t ngroups;
};

/* The Cherry Trail SoC has 4 independent GPIO controllers */
static struct up_soc_gpiochip_info chip_cht_SW = { .name = "INT33FF:00" };
static struct up_soc_gpiochip_info chip_cht_N  = { .name = "INT33FF:01" };
static struct up_soc_gpiochip_info chip_cht_E  = { .name = "INT33FF:02" };
static struct up_soc_gpiochip_info chip_cht_SE = { .name = "INT33FF:03" };

#define GPIO_PIN(c, o, d, f, m, x)	\
	{				\
		.soc_gpio.ci = (c),	\
		.soc_gpio.offset = (o),	\
		.dir_gpio = (d),	\
		.func_dir = (f),	\
		.mux_gpio = (m),	\
		.func_mux = (x),	\
		.func_sel = false,	\
	}

#define PIN_GROUP(n, p)				\
	{					\
		.name = (n),			\
		.pins = (p),			\
		.npins = ARRAY_SIZE((p)),	\
	}

#define FUNCTION(n, g)				\
	{					\
		.name = (n),			\
		.groups = (g),			\
		.ngroups = ARRAY_SIZE((g)),	\
	}

#define GPIO_PINRANGE(start, end)		\
	{					\
		.base = (start),		\
		.npins = (end) - (start) + 1,	\
	}

#define N_GPIO 28

#define DIR_NONE -1
#define DIR_OUT  1
#define DIR_IN   0

#define MUX_NONE -1
#define MUX_GPIO 0
#define MUX_FUNC 1

/*
 * Table of I/O pins on the 40-pin header of the UP Board (version-specific)
 */
/* UP Board v0.1 uses auto-sensing level shifters on each pin */
static struct up_pin_info up_pins_v0_1[N_GPIO] = {
	GPIO_PIN(&chip_cht_SW, 33, -1, DIR_NONE, -1, -1), /*  0 */
	GPIO_PIN(&chip_cht_SW, 37, -1, DIR_NONE, -1, -1), /*  1 */
	GPIO_PIN(&chip_cht_SW, 32, -1, DIR_NONE, -1, -1), /*  2 */
	GPIO_PIN(&chip_cht_SW, 35, -1, DIR_NONE, -1, -1), /*  3 */
	GPIO_PIN(&chip_cht_SE,  7, -1, DIR_NONE, -1, -1), /*  4 */
	GPIO_PIN(&chip_cht_SE,  4, -1, DIR_NONE, -1, -1), /*  5 */
	GPIO_PIN(&chip_cht_SE,  3, -1, DIR_NONE, -1, -1), /*  6 */
	GPIO_PIN(&chip_cht_N,  41, -1, DIR_NONE, -1, -1), /*  7 */
	GPIO_PIN(&chip_cht_N,  36, -1, DIR_NONE, -1, -1), /*  8 */
	GPIO_PIN(&chip_cht_E,  17, -1, DIR_NONE, -1, -1), /*  9 */
	GPIO_PIN(&chip_cht_E,  23, -1, DIR_NONE, -1, -1), /* 10 */
	GPIO_PIN(&chip_cht_E,  14, -1, DIR_NONE, -1, -1), /* 11 */
	GPIO_PIN(&chip_cht_SE,  1, -1, DIR_NONE, -1, -1), /* 12 */
	GPIO_PIN(&chip_cht_SE,  5, -1, DIR_NONE, -1, -1), /* 13 */
	GPIO_PIN(&chip_cht_SW, 13, -1, DIR_NONE, -1, -1), /* 14 */
	GPIO_PIN(&chip_cht_SW,  9, -1, DIR_NONE, -1, -1), /* 15 */
	GPIO_PIN(&chip_cht_N,   6, -1, DIR_NONE, -1, -1), /* 16 */
	GPIO_PIN(&chip_cht_SE,  6, -1, DIR_NONE, -1, -1), /* 17 */
	GPIO_PIN(&chip_cht_SW, 17, -1, DIR_NONE, -1, -1), /* 18 */
	GPIO_PIN(&chip_cht_SW, 21, -1, DIR_NONE, -1, -1), /* 19 */
	GPIO_PIN(&chip_cht_SW, 19, -1, DIR_NONE, -1, -1), /* 20 */
	GPIO_PIN(&chip_cht_SW, 16, -1, DIR_NONE, -1, -1), /* 21 */
	GPIO_PIN(&chip_cht_N,   3, -1, DIR_NONE, -1, -1), /* 22 */
	GPIO_PIN(&chip_cht_N,   2, -1, DIR_NONE, -1, -1), /* 23 */
	GPIO_PIN(&chip_cht_N,   1, -1, DIR_NONE, -1, -1), /* 24 */
	GPIO_PIN(&chip_cht_SW, 14, -1, DIR_NONE, -1, -1), /* 25 */
	GPIO_PIN(&chip_cht_N,   7, -1, DIR_NONE, -1, -1), /* 26 */
	GPIO_PIN(&chip_cht_SW, 10, -1, DIR_NONE, -1, -1), /* 27 */
};

/* UP Board v0.2 uses GPIO-controlled buffers and mux switches
 * to provide better current source/sink capability on the I/O pins
 */
static struct up_pin_info up_pins_v0_2[N_GPIO] = {
	GPIO_PIN(&chip_cht_SW, 33, 57, DIR_OUT,  53, MUX_FUNC), /*  0 */
	GPIO_PIN(&chip_cht_SW, 37, 59, DIR_OUT,  53, MUX_FUNC), /*  1 */
	GPIO_PIN(&chip_cht_SW, 32, 56, DIR_OUT,  54, MUX_FUNC), /*  2 */
	GPIO_PIN(&chip_cht_SW, 35, 58, DIR_OUT,  54, MUX_FUNC), /*  3 */
	GPIO_PIN(&chip_cht_E,  18, 32, DIR_NONE, -1, -1),       /*  4 */
	GPIO_PIN(&chip_cht_E,  21, 36, DIR_NONE, -1, -1),       /*  5 */
	GPIO_PIN(&chip_cht_E,  12, 37, DIR_NONE, -1, -1),       /*  6 */
	GPIO_PIN(&chip_cht_SE, 48, 47, DIR_NONE, -1, -1),       /*  7 */
	GPIO_PIN(&chip_cht_SE,  7, 46, DIR_OUT,  -1, -1),       /*  8 */
	GPIO_PIN(&chip_cht_SE,  3, 45, DIR_IN,   -1, -1),       /*  9 */
	GPIO_PIN(&chip_cht_SE,  6, 44, DIR_OUT,  -1, -1),       /* 10 */
	GPIO_PIN(&chip_cht_SE,  4, 43, DIR_OUT,  -1, -1),       /* 11 */
	GPIO_PIN(&chip_cht_SE,  5, 48, DIR_OUT,  -1, -1),       /* 12 */
	GPIO_PIN(&chip_cht_SE,  1, 49, DIR_OUT,  -1, -1),       /* 13 */
	GPIO_PIN(&chip_cht_SW, 13, 50, DIR_OUT,  -1, -1),       /* 14 */
	GPIO_PIN(&chip_cht_SW,  9, 51, DIR_IN,   -1, -1),       /* 15 */
	GPIO_PIN(&chip_cht_N,   6, 42, DIR_NONE, -1, -1),       /* 16 */
	GPIO_PIN(&chip_cht_E,  15, 33, DIR_NONE, -1, -1),       /* 17 */
	GPIO_PIN(&chip_cht_SW, 17, 63, DIR_OUT,  55, MUX_GPIO), /* 18 */
	GPIO_PIN(&chip_cht_SW, 21, 62, DIR_OUT,  55, MUX_GPIO), /* 19 */
	GPIO_PIN(&chip_cht_SW, 19, 61, DIR_IN,   55, MUX_GPIO), /* 20 */
	GPIO_PIN(&chip_cht_SW, 16, 60, DIR_OUT,  55, MUX_GPIO), /* 21 */
	GPIO_PIN(&chip_cht_N,   3, 35, DIR_NONE, -1, -1),       /* 22 */
	GPIO_PIN(&chip_cht_N,   2, 39, DIR_NONE, -1, -1),       /* 23 */
	GPIO_PIN(&chip_cht_N,   1, 40, DIR_NONE, -1, -1),       /* 24 */
	GPIO_PIN(&chip_cht_SW, 14, 41, DIR_OUT,  -1, -1),       /* 25 */
	GPIO_PIN(&chip_cht_N,   7, 38, DIR_NONE, -1, -1),       /* 26 */
	GPIO_PIN(&chip_cht_SW, 10, 34, DIR_IN,   -1, -1),       /* 27 */
};

/* The layout and numbering is designed to emulate the Raspberry Pi 2 */
static const struct pinctrl_pin_desc up_pins[] = {
	PINCTRL_PIN(0,  "I2C0_SDA"),
	PINCTRL_PIN(1,  "I2C0_SCL"),
	PINCTRL_PIN(2,  "I2C1_SDA"),
	PINCTRL_PIN(3,  "I2C1_SCL"),
	PINCTRL_PIN(4,  "GPIO4"),
	PINCTRL_PIN(5,  "GPIO5"),
	PINCTRL_PIN(6,  "GPIO6"),
	PINCTRL_PIN(7,  "SPI_CS1"),
	PINCTRL_PIN(8,  "SPI_CS0"),
	PINCTRL_PIN(9,  "SPI_MISO"),
	PINCTRL_PIN(10, "SPI_MOSI"),
	PINCTRL_PIN(11, "SPI_CLK"),
	PINCTRL_PIN(12, "PWM0"),
	PINCTRL_PIN(13, "PWM1"),
	PINCTRL_PIN(14, "UART1_TX"),
	PINCTRL_PIN(15, "UART1_RX"),
	PINCTRL_PIN(16, "GPIO16"),
	PINCTRL_PIN(17, "GPIO17"),
	PINCTRL_PIN(18, "I2S0_CLK"),
	PINCTRL_PIN(19, "I2S0_FRM"),
	PINCTRL_PIN(20, "I2S0_DIN"),
	PINCTRL_PIN(21, "I2S0_DOUT"),
	PINCTRL_PIN(22, "GPIO22"),
	PINCTRL_PIN(23, "GPIO23"),
	PINCTRL_PIN(24, "GPIO24"),
	PINCTRL_PIN(25, "UART2_TX"),
	PINCTRL_PIN(26, "GPIO26"),
	PINCTRL_PIN(27, "UART2_RX"),
};

static const unsigned uart1_pins[] = { 14, 15 };
static const unsigned uart2_pins[] = { 25, 27 };
static const unsigned i2c0_pins[]  = { 0, 1 };
static const unsigned i2c1_pins[]  = { 2, 3 };
static const unsigned spi2_pins[]  = { 8, 9, 10, 11 };
static const unsigned i2s0_pins[]  = { 18, 19, 20, 21 };
static const unsigned pwm0_pins[]  = { 12 };
static const unsigned pwm1_pins[]  = { 13 };

static const struct up_pingroup pin_groups[] = {
	PIN_GROUP("uart1_grp", uart1_pins),
	PIN_GROUP("uart2_grp", uart2_pins),
	PIN_GROUP("i2c0_grp", i2c0_pins),
	PIN_GROUP("i2c1_grp", i2c1_pins),
	PIN_GROUP("spi2_grp", spi2_pins),
	PIN_GROUP("i2s0_grp", i2s0_pins),
	PIN_GROUP("pwm0_grp", pwm0_pins),
	PIN_GROUP("pwm1_grp", pwm1_pins),
};

static const char * const uart1_groups[] = { "uart1_grp" };
static const char * const uart2_groups[] = { "uart2_grp" };
static const char * const i2c0_groups[]  = { "i2c0_grp" };
static const char * const i2c1_groups[]  = { "i2c1_grp" };
static const char * const spi2_groups[]  = { "spi2_grp" };
static const char * const i2s0_groups[]  = { "i2s0_grp" };
static const char * const pwm0_groups[]  = { "pwm0_grp" };
static const char * const pwm1_groups[]  = { "pwm1_grp" };

static const struct up_function pin_functions[] = {
	FUNCTION("uart1", uart1_groups),
	FUNCTION("uart2", uart2_groups),
	FUNCTION("i2c0",  i2c0_groups),
	FUNCTION("i2c1",  i2c1_groups),
	FUNCTION("spi2",  spi2_groups),
	FUNCTION("i2s0",  i2s0_groups),
	FUNCTION("pwm0",  pwm0_groups),
	FUNCTION("pwm1",  pwm1_groups),
};

static inline struct up_pctrl *gc_to_up_pctrl(struct gpio_chip *gc)
{
	return container_of(gc, struct up_pctrl, chip);
}

static int up_gpiochip_match(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static int up_gpio_map_pins(struct platform_device *pdev,
			    struct up_pin_info *pin_info)
{
	unsigned i;
	int ret;

	/* Find the Cherry Trail GPIO descriptors corresponding
	 * with each GPIO pin on the UP Board I/O header
	 */
	for (i = 0; i < N_GPIO; i++) {
		struct up_pin_info *pin = &pin_info[i];
		struct up_soc_gpiochip_info *ci = pin->soc_gpio.ci;

		if (!ci->chip) {
			ci->chip = gpiochip_find(ci->name, up_gpiochip_match);
			if (!ci->chip)
				return -EPROBE_DEFER;
		}
		pin->soc_gpio.desc = gpio_to_desc(ci->chip->base
						  + pin->soc_gpio.offset);
		if (!pin->soc_gpio.desc)
			return -EINVAL;

		/* Reserve GPIOs for pin direction and mux control, if any */
		if (gpio_is_valid(pin->dir_gpio)) {
			ret = devm_gpio_request_one(&pdev->dev, pin->dir_gpio,
						    GPIOF_OUT_INIT_HIGH,
						    dev_name(&pdev->dev));
			if (ret)
				return ret;
		}
		if (gpio_is_valid(pin->mux_gpio)) {
			ret = devm_gpio_request_one(&pdev->dev, pin->mux_gpio,
						    GPIOF_OUT_INIT_HIGH,
						    dev_name(&pdev->dev));
			/* mux may be shared by multiple pins - ignore EBUSY */
			if (ret && ret != -EBUSY)
				return ret;
		}
	}

	return 0;
}

static irqreturn_t up_gpio_irq_handler(int irq, void *data)
{
	struct up_pin_info *pin = (struct up_pin_info *)data;

	generic_handle_irq(pin->irq);
	return IRQ_HANDLED;
}

static unsigned int up_gpio_irq_startup(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	unsigned offset = irqd_to_hwirq(data);
	struct up_pin_info *pin = &up_pctrl->pin_info[offset];

	return request_irq(pin->soc_gpio.irq, up_gpio_irq_handler,
			   IRQF_ONESHOT, dev_name(gc->dev), pin);
}

static void up_gpio_irq_shutdown(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	unsigned offset = irqd_to_hwirq(data);
	struct up_pin_info *pin = &up_pctrl->pin_info[offset];

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

static int up_gpio_dir_in(struct gpio_chip *gc, unsigned offset)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;
	int ret;

	ret = gpiod_direction_input(desc);
	if (ret)
		return ret;

	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int up_gpio_dir_out(struct gpio_chip *gc, unsigned offset, int value)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;
	int ret;

	ret = pinctrl_gpio_direction_output(gc->base + offset);
	if (ret)
		return ret;

	return gpiod_direction_output(desc, value);
}

static int up_gpio_get_dir(struct gpio_chip *gc, unsigned offset)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;

	return gpiod_get_direction(desc);
}

static int up_gpio_request(struct gpio_chip *gc, unsigned offset)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;

	pinctrl_request_gpio(gc->base + offset);
	return gpio_request(desc_to_gpio(desc), gc->label);
}

static void up_gpio_free(struct gpio_chip *gc, unsigned offset)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;

	pinctrl_free_gpio(gc->base + offset);
	gpio_free(desc_to_gpio(desc));
}

static int up_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;

	return gpiod_get_value(desc);
}

static void up_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct up_pctrl *up_pctrl = gc_to_up_pctrl(gc);
	struct gpio_desc *desc = up_pctrl->pin_info[offset].soc_gpio.desc;

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

static int up_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(pin_groups);
}

static const char *up_get_group_name(struct pinctrl_dev *pctldev,
				      unsigned group)
{
	return pin_groups[group].name;
}

static int up_get_group_pins(struct pinctrl_dev *pctldev, unsigned group,
			      const unsigned **pins, unsigned *npins)
{
	*pins = pin_groups[group].pins;
	*npins = pin_groups[group].npins;
	return 0;
}

static const struct pinctrl_ops up_pinctrl_ops = {
	.get_groups_count = up_get_groups_count,
	.get_group_name = up_get_group_name,
	.get_group_pins = up_get_group_pins,
};

static int up_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(pin_functions);
}

static const char *up_get_function_name(struct pinctrl_dev *pctldev,
					unsigned function)
{
	return pin_functions[function].name;
}

static int up_get_function_groups(struct pinctrl_dev *pctldev,
				  unsigned function,
				  const char * const **groups,
				  unsigned * const ngroups)
{
	*groups = pin_functions[function].groups;
	*ngroups = pin_functions[function].ngroups;
	return 0;
}

static int up_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned function,
			     unsigned group)
{
	struct up_pctrl *up_pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct up_pingroup *grp = &pin_groups[group];
	int i;

	for (i = 0; i < grp->npins; i++) {
		int offset = grp->pins[i];
		struct up_pin_info *pin = &up_pctrl->pin_info[offset];

		if (gpio_is_valid(pin->dir_gpio) && (pin->func_dir != DIR_NONE))
			gpio_set_value_cansleep(pin->dir_gpio, pin->func_dir);
		if (gpio_is_valid(pin->mux_gpio))
			gpio_set_value_cansleep(pin->mux_gpio, pin->func_mux);
		pin->func_sel = true;
	}

	return 0;
}

static int up_gpio_set_direction(struct pinctrl_dev *pctldev,
				 struct pinctrl_gpio_range *range,
				 unsigned offset, bool input)
{
	struct up_pctrl *up_pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct up_pin_info *pin = &up_pctrl->pin_info[offset];

	if (gpio_is_valid(pin->dir_gpio))
		gpio_set_value_cansleep(pin->dir_gpio,
					input ? DIR_IN : DIR_OUT);

	return 0;
}

static int up_gpio_request_enable(struct pinctrl_dev *pctldev,
				  struct pinctrl_gpio_range *range,
				  unsigned offset)
{
	struct up_pctrl *up_pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct up_pin_info *pin = &up_pctrl->pin_info[offset];

	if (gpio_is_valid(pin->mux_gpio))
		gpio_set_value_cansleep(pin->mux_gpio, MUX_GPIO);
	if (gpio_is_valid(pin->dir_gpio))
		gpio_set_value_cansleep(pin->dir_gpio,
					gpiod_get_direction(pin->soc_gpio.desc)
					? DIR_IN : DIR_OUT);

	return 0;
}

static void up_gpio_disable_free(struct pinctrl_dev *pctldev,
				 struct pinctrl_gpio_range *range,
				 unsigned offset)
{
	struct up_pctrl *up_pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct up_pin_info *pin = &up_pctrl->pin_info[offset];

	if (gpio_is_valid(pin->dir_gpio) && pin->func_sel)
		gpio_set_value_cansleep(pin->dir_gpio, pin->func_dir);
	if (gpio_is_valid(pin->mux_gpio))
		gpio_set_value_cansleep(pin->mux_gpio, pin->func_mux);
}

static const struct pinmux_ops up_pinmux_ops = {
	.get_functions_count = up_get_functions_count,
	.get_function_name = up_get_function_name,
	.get_function_groups = up_get_function_groups,
	.set_mux = up_pinmux_set_mux,
	.gpio_request_enable = up_gpio_request_enable,
	.gpio_disable_free = up_gpio_disable_free,
	.gpio_set_direction = up_gpio_set_direction,
};

static int up_config_get(struct pinctrl_dev *pctldev, unsigned pin,
			  unsigned long *config)
{
	return -ENOTSUPP;
}

static int up_config_set(struct pinctrl_dev *pctldev, unsigned pin,
			  unsigned long *configs, unsigned nconfigs)
{
	return 0;
}

static const struct pinconf_ops up_pinconf_ops = {
	.is_generic = true,
	.pin_config_set = up_config_set,
	.pin_config_get = up_config_get,
};

static struct pinctrl_desc up_pinctrl_desc = {
	.pins = up_pins,
	.npins = ARRAY_SIZE(up_pins),
	.pctlops = &up_pinctrl_ops,
	.pmxops = &up_pinmux_ops,
	.confops = &up_pinconf_ops,
	.owner = THIS_MODULE,
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
		.driver_data = (void *)&up_pins_v0_2
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_MATCH(DMI_BOARD_VERSION, "V0.2"),
		},
		.driver_data = (void *)&up_pins_v0_2
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_MATCH(DMI_BOARD_NAME, "UP-CHT01"),
			DMI_MATCH(DMI_BOARD_VERSION, "V0.1"),
		},
		.driver_data = (void *)&up_pins_v0_1
	},
	{}
};

static int up_pinctrl_probe(struct platform_device *pdev)
{
	struct up_pctrl *up_pctrl;
	struct up_pin_info *pin_info;
	const struct dmi_system_id *system_id;
	unsigned offset;
	int ret;

	system_id = dmi_first_match(up_board_id_table);
	if (!system_id)
		return -ENXIO;

	pin_info = system_id->driver_data;

	ret = up_gpio_map_pins(pdev, pin_info);
	if (ret)
		return ret;

	up_pctrl = devm_kzalloc(&pdev->dev, sizeof(*up_pctrl), GFP_KERNEL);
	if (!up_pctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, up_pctrl);

	up_pctrl->pctldesc = up_pinctrl_desc;
	up_pctrl->pctldesc.name = dev_name(&pdev->dev);
	up_pctrl->pctldev = pinctrl_register(&up_pctrl->pctldesc,
					     &pdev->dev, up_pctrl);
	if (IS_ERR(up_pctrl->pctldev)) {
		dev_err(&pdev->dev, "failed to register pinctrl driver\n");
		return PTR_ERR(up_pctrl->pctldev);
	}

	up_pctrl->pin_info = pin_info;
	up_pctrl->chip = up_gpio_chip;
	up_pctrl->chip.label = dev_name(&pdev->dev);
	up_pctrl->chip.dev = &pdev->dev;

	ret = gpiochip_add(&up_pctrl->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to add %s chip\n",
			up_pctrl->chip.label);
		return ret;
	}

	ret = gpiochip_add_pin_range(&up_pctrl->chip, dev_name(&pdev->dev),
				     0, 0, N_GPIO);
	if (ret) {
		dev_err(&pdev->dev, "failed to add GPIO pin range\n");
		goto fail_add_pin_range;
	}

	ret = gpiochip_irqchip_add(&up_pctrl->chip, &up_gpio_irqchip, 0,
				   handle_simple_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&pdev->dev, "failed to add IRQ chip\n");
		goto fail_irqchip_add;
	}

	for (offset = 0; offset < up_pctrl->chip.ngpio; offset++) {
		struct up_pin_info *pin = &up_pctrl->pin_info[offset];
		struct irq_data *irq_data;

		pin->irq = irq_find_mapping(up_pctrl->chip.irqdomain, offset);
		pin->soc_gpio.irq = gpiod_to_irq(pin->soc_gpio.desc);
		irq_set_parent(pin->irq, pin->soc_gpio.irq);
		irq_data = irq_get_irq_data(pin->irq);
		irq_data->parent_data = irq_get_irq_data(pin->soc_gpio.irq);
	}

	return 0;

fail_irqchip_add:
fail_add_pin_range:
	gpiochip_remove(&up_pctrl->chip);

	return ret;
}

static int up_pinctrl_remove(struct platform_device *pdev)
{
	struct up_pctrl *up_pctrl = platform_get_drvdata(pdev);

	gpiochip_remove(&up_pctrl->chip);
	pinctrl_unregister(up_pctrl->pctldev);

	return 0;
}

static struct platform_driver up_pinctrl_driver = {
	.driver.name	= "up-pinctrl",
	.driver.owner	= THIS_MODULE,
	.probe		= up_pinctrl_probe,
	.remove		= up_pinctrl_remove,
};

static int __init up_pinctrl_init(void)
{
	return platform_driver_register(&up_pinctrl_driver);
}
subsys_initcall(up_pinctrl_init);

static void __exit up_pinctrl_exit(void)
{
	platform_driver_unregister(&up_pinctrl_driver);
}
module_exit(up_pinctrl_exit);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("Pin Control driver for UP Board I/O pin header");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:up-pinctrl");
