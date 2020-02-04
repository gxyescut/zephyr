/*
 * Copyright (c) 2020, Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT quicklogic_eos_s3_gpio

#include <errno.h>
#include <drivers/gpio.h>
#include <soc.h>
#include <eoss3_hal_gpio.h>
#include <eoss3_hal_pads.h>
#include <eoss3_hal_pad_config.h>

#include "gpio_utils.h"

#define MAX_GPIOS_PER_PORT	8
#define PORTS_NUM		2
#define MAX_GPIOS		(MAX_GPIOS_PER_PORT * PORTS_NUM)

struct gpio_eos_s3_config {
};

struct gpio_eos_s3_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* Runtime configuration */
	uint8_t gpio_irq_num[MAX_GPIOS];
};

/*
 * The PAD (pin) value can be greater than 31 in EOS-S3,
 * because of that it is assumed that instead of the PAD number,
 * the GPIO number is passed as the `gpio_pin_t` argument in `pin_configure`
 * call to avoid errors from the high level API which uses BIT()
 * macro to point the bit corresponding to target pin.
 *
 * Additionally, there are only 8 GPIOs in EOS-S3 and there are always 2 PADs
 * which can be configured to each GPIO. Since each GPIO has two possibilities
 * of connecting to the one of PADs, there are two mockup ports
 * defined to distinguish these configurations. Because each port has
 * the same IRQn we cannot define separate devices in the device tree.
 * In this case to configure either "port0" or "port1" it is assmued that GPIOs
 * in range of 0 to 7 point to "port0" and GPIOs indexed from 8 to 15 point
 * to "port1".
 */

static int find_pad(GPIOCfgTypeDef *cfg, int port_num)
{
	/* Find out which PAD will be used for given GPIO */
	if (port_num == 0) {
		switch (cfg->ucGpioNum) {
		case GPIO_0:
			cfg->xPadConf->ucPin = PAD_6;
			cfg->xPadConf->ucFunc = PAD6_FUNC_SEL_GPIO_0;
			break;
		case GPIO_1:
			cfg->xPadConf->ucPin = PAD_9;
			cfg->xPadConf->ucFunc = PAD9_FUNC_SEL_GPIO_1;
			break;
		case GPIO_2:
			cfg->xPadConf->ucPin = PAD_11;
			cfg->xPadConf->ucFunc = PAD11_FUNC_SEL_GPIO_2;
			break;
		case GPIO_3:
			cfg->xPadConf->ucPin = PAD_14;
			cfg->xPadConf->ucFunc = PAD14_FUNC_SEL_GPIO_3;
			break;
		case GPIO_4:
			cfg->xPadConf->ucPin = PAD_18;
			cfg->xPadConf->ucFunc = PAD18_FUNC_SEL_GPIO_4;
			break;
		case GPIO_5:
			cfg->xPadConf->ucPin = PAD_21;
			cfg->xPadConf->ucFunc = PAD21_FUNC_SEL_GPIO_5;
			break;
		case GPIO_6:
			cfg->xPadConf->ucPin = PAD_22;
			cfg->xPadConf->ucFunc = PAD22_FUNC_SEL_GPIO_6;
			break;
		case GPIO_7:
			cfg->xPadConf->ucPin = PAD_23;
			cfg->xPadConf->ucFunc = PAD23_FUNC_SEL_GPIO_7;
			break;
		default:
			return -EINVAL;
		}
	} else if (port_num == 1) {
		switch (cfg->ucGpioNum) {
		case GPIO_0:
			cfg->xPadConf->ucPin = PAD_24;
			cfg->xPadConf->ucFunc = PAD24_FUNC_SEL_GPIO_0;
			break;
		case GPIO_1:
			cfg->xPadConf->ucPin = PAD_26;
			cfg->xPadConf->ucFunc = PAD26_FUNC_SEL_GPIO_1;
			break;
		case GPIO_2:
			cfg->xPadConf->ucPin = PAD_28;
			cfg->xPadConf->ucFunc = PAD28_FUNC_SEL_GPIO_2;
			break;
		case GPIO_3:
			cfg->xPadConf->ucPin = PAD_30;
			cfg->xPadConf->ucFunc = PAD30_FUNC_SEL_GPIO_3;
			break;
		case GPIO_4:
			cfg->xPadConf->ucPin = PAD_31;
			cfg->xPadConf->ucFunc = PAD31_FUNC_SEL_GPIO_4;
			break;
		case GPIO_5:
			cfg->xPadConf->ucPin = PAD_36;
			cfg->xPadConf->ucFunc = PAD36_FUNC_SEL_GPIO_5;
			break;
		case GPIO_6:
			cfg->xPadConf->ucPin = PAD_38;
			cfg->xPadConf->ucFunc = PAD38_FUNC_SEL_GPIO_6;
			break;
		case GPIO_7:
			cfg->xPadConf->ucPin = PAD_45;
			cfg->xPadConf->ucFunc = PAD45_FUNC_SEL_GPIO_7;
			break;
		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

/*
 * NOTE:
 * Pin number defined in the device tree should be actually a GPIO number
 * to avoid overflow in high level GPIO API.
 * `gpio_num_extended` in range of 0 to 7 matches to `port_num == 0` PADs from
 * `find_pad` function and `gpio_num_extended` in range of 8 to 15
 * are associated with `port_num == 1` consequently.
 */
static int gpio_eos_s3_configure(const struct device *dev,
				 gpio_pin_t gpio_num_extended,
				 gpio_flags_t flags)
{
	struct gpio_eos_s3_data *data = dev->data;
	int ret;
	GPIOCfgTypeDef gpio_cfg;
	PadConfig pad_conf;
	int port_num = 0;

	gpio_cfg.xPadConf = &pad_conf;

	/* Check for an invalid gpio number */
	if (gpio_num_extended >= MAX_GPIOS) {
		return -EINVAL;
	}

	/* Determine "port" number */
	if ((int)(gpio_num_extended / MAX_GPIOS_PER_PORT) == 1) {
		port_num = 1;
	}

	/* Get actual GPIO number */
	uint8_t gpio_num = gpio_num_extended % 8;

	/* Check for an invalid pin configuration */
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0))
		return -ENOTSUP;

	gpio_cfg.ucGpioNum = gpio_num;
	ret = find_pad(&gpio_cfg, port_num);

	if (ret)
		return ret;

	/* Configure PAD */
	if (flags & GPIO_PULL_UP)
		gpio_cfg.xPadConf->ucPull = PAD_PULLUP;
	else if (flags & GPIO_PULL_DOWN)
		gpio_cfg.xPadConf->ucPull = PAD_PULLDOWN;
	else
		gpio_cfg.xPadConf->ucPull = PAD_NOPULL;

	if ((flags & GPIO_DIR_MASK) == GPIO_INPUT) {
		gpio_cfg.xPadConf->ucMode = PAD_MODE_INPUT_EN;
		gpio_cfg.xPadConf->ucSmtTrg = PAD_SMT_TRIG_EN;
		HAL_PAD_Config(gpio_cfg.xPadConf);
	} else {
		gpio_cfg.xPadConf->ucMode = PAD_MODE_OUTPUT_EN;
		HAL_PAD_Config(gpio_cfg.xPadConf);
	}

	return 0;
}

static int gpio_eos_s3_port_get_raw(const struct device *dev,
				    uint32_t *value)
{
	ARG_UNUSED(dev);
	uint8_t pin_value = 0;
	*value = 0;

	for (uint8_t gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
		HAL_GPIO_Read(gpion, &pin_value);
		*value |= (pin_value == 1) ? 1 << gpion : 0;
	}

	return 0;
}

static int gpio_eos_s3_port_set_masked_raw(const struct device *dev,
					   uint32_t mask,
					   uint32_t value)
{
	ARG_UNUSED(dev);
	uint32_t target_value;
	uint32_t output_states = MISC_CTRL->IO_OUTPUT;

	target_value = ((output_states & ~mask) | (value & mask));
	for (uint8_t gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
		if (target_value & 1) {
			HAL_GPIO_Write(gpion, 1);
		} else {
			HAL_GPIO_Write(gpion, 0);
		}
		target_value = target_value >> 1;
	}

	return 0;
}

static int gpio_eos_s3_port_set_bits_raw(const struct device *dev,
					 uint32_t value)
{
	ARG_UNUSED(dev);

	for (uint8_t gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
		if (value & 1) {
			HAL_GPIO_Write(gpion, 1);
			break;
		}
		value = value >> 1;
	}

	return 0;
}

static int gpio_eos_s3_port_clear_bits_raw(const struct device *dev,
					   uint32_t value)
{
	ARG_UNUSED(dev);

	for (uint8_t gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
		if (value & 1) {
			HAL_GPIO_Write(gpion, 0);
		}
		value = value >> 1;
	}

	return 0;
}

static int gpio_eos_s3_port_toggle_bits(const struct device *dev,
					uint32_t value)
{
	ARG_UNUSED(dev);
	uint32_t target_value;
	uint32_t output_states = MISC_CTRL->IO_OUTPUT;

	target_value = output_states ^ value;
	for (uint8_t gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
		if (target_value & 1) {
			HAL_GPIO_Write(gpion, 1);
		} else {
			HAL_GPIO_Write(gpion, 0);
		}
		target_value = target_value >> 1;
	}

	return 0;
}

static int gpio_eos_s3_manage_callback(const struct device *dev,
				       struct gpio_callback *callback, bool set)
{
	struct gpio_eos_s3_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_eos_s3_pin_interrupt_configure(const struct device *dev,
					       gpio_pin_t gpio_num_extended,
					       enum gpio_int_mode mode,
					       enum gpio_int_trig trig)
{
	struct gpio_eos_s3_data *data = dev->data;
	GPIOCfgTypeDef gpio_cfg;
	PadConfig pad_conf;
	int ret;
	int port_num = 0;

	gpio_cfg.xPadConf = &pad_conf;

	/* Check for an invalid gpio number */
	if (gpio_num_extended >= MAX_GPIOS) {
		return -EINVAL;
	}

	/* Determine "port" number */
	if ((int)(gpio_num_extended / MAX_GPIOS_PER_PORT) == 1) {
		port_num = 1;
	}

	/* Get actual GPIO number */
	uint8_t gpio_num = gpio_num_extended % 8;

	gpio_cfg.ucGpioNum = gpio_num;
	ret = find_pad(&gpio_cfg, port_num);

	if (ret)
		return ret;

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* Reset IRQ configuration */
		int irq_num = HAL_GPIO_IntrCfg(&gpio_cfg);

		if (irq_num < 0)
			return -EINVAL;

		data->gpio_irq_num[gpio_cfg.ucGpioNum] = BIT((uint8_t)irq_num);
		/* Disable IRQ */
		INTR_CTRL->GPIO_INTR_EN_M4 &= ~data->gpio_irq_num[gpio_num];
	} else {
		/* Prepare configuration */
		if (mode == GPIO_INT_MODE_LEVEL) {
			gpio_cfg.intr_type = LEVEL_TRIGGERED;
			if (trig == GPIO_INT_TRIG_LOW) {
				gpio_cfg.pol_type = FALL_LOW;
			} else {
				gpio_cfg.pol_type = RISE_HIGH;
			}
		} else {
			gpio_cfg.intr_type = EDGE_TRIGGERED;
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				gpio_cfg.pol_type = FALL_LOW;
				break;
			case GPIO_INT_TRIG_HIGH:
				gpio_cfg.pol_type = RISE_HIGH;
				break;
			case GPIO_INT_TRIG_BOTH:
				return -ENOTSUP;
			}
		}

		/* Set IRQ configuration */
		int irq_num = HAL_GPIO_IntrCfg(&gpio_cfg);

		if (irq_num < 0)
			return -EINVAL;

		data->gpio_irq_num[gpio_cfg.ucGpioNum] = BIT((uint8_t)irq_num);

		/* Enable IRQ */
		INTR_CTRL->GPIO_INTR_EN_M4 |= data->gpio_irq_num[gpio_num];
	}

	return 0;
}

static void gpio_eos_s3_isr(const struct device *dev)
{
	struct gpio_eos_s3_data *data = dev->data;
	uint8_t intr_status = INTR_CTRL->GPIO_INTR;

	for (uint8_t i = 0; i < MAX_GPIOS_PER_PORT; i++) {
		uint8_t intr_num = BIT(i);

		if (intr_status & intr_num) {
			for (int gpion = 0; gpion < MAX_GPIOS_PER_PORT; gpion++) {
				if (intr_num == data->gpio_irq_num[gpion]) {
					gpio_fire_callbacks(&data->callbacks,
							    dev, BIT(gpion));
				}
			}
			INTR_CTRL->GPIO_INTR |= intr_num;
		}
	}
}

static const struct gpio_driver_api gpio_eos_s3_driver_api = {
	.pin_configure = gpio_eos_s3_configure,
	.port_get_raw = gpio_eos_s3_port_get_raw,
	.port_set_masked_raw = gpio_eos_s3_port_set_masked_raw,
	.port_set_bits_raw = gpio_eos_s3_port_set_bits_raw,
	.port_clear_bits_raw = gpio_eos_s3_port_clear_bits_raw,
	.port_toggle_bits = gpio_eos_s3_port_toggle_bits,
	.pin_interrupt_configure = gpio_eos_s3_pin_interrupt_configure,
	.manage_callback = gpio_eos_s3_manage_callback,
};

static int gpio_eos_s3_init(const struct device *dev);
const struct gpio_eos_s3_config gpio_eos_s3_config;
static struct gpio_eos_s3_data gpio_eos_s3_data;

DEVICE_AND_API_INIT(gpio_eos_s3, DT_INST_LABEL(0),
		    gpio_eos_s3_init,
		    &gpio_eos_s3_data,
		    &gpio_eos_s3_config,
		    POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_eos_s3_driver_api);

static int gpio_eos_s3_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    gpio_eos_s3_isr,
		    DEVICE_GET(gpio_eos_s3), 0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}
