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

#define MAX_GPIOS		8U
#define GPIOS_MASK		(BIT(MAX_GPIOS) - 1)

struct gpio_eos_s3_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* Pin configuration to determine whether use primary(0)
	 * or secondary(1) pin for a target GPIO
	 */
	uint8_t pin_cfgs[MAX_GPIOS];
};

struct gpio_eos_s3_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* runtime configuration */
	uint8_t gpio_irq_num[MAX_GPIOS];
};

/* connection table to configure GPIOs with pads */
static const PadConfig pad_configs[] = {
	{.ucPin = PAD_6, .ucFunc = PAD6_FUNC_SEL_GPIO_0},
	{.ucPin = PAD_9, .ucFunc = PAD9_FUNC_SEL_GPIO_1},
	{.ucPin = PAD_11, .ucFunc = PAD11_FUNC_SEL_GPIO_2},
	{.ucPin = PAD_14, .ucFunc = PAD14_FUNC_SEL_GPIO_3},
	{.ucPin = PAD_18, .ucFunc = PAD18_FUNC_SEL_GPIO_4},
	{.ucPin = PAD_21, .ucFunc = PAD21_FUNC_SEL_GPIO_5},
	{.ucPin = PAD_22, .ucFunc = PAD22_FUNC_SEL_GPIO_6},
	{.ucPin = PAD_23, .ucFunc = PAD23_FUNC_SEL_GPIO_7},
	{.ucPin = PAD_24, .ucFunc = PAD24_FUNC_SEL_GPIO_0},
	{.ucPin = PAD_26, .ucFunc = PAD26_FUNC_SEL_GPIO_1},
	{.ucPin = PAD_28, .ucFunc = PAD28_FUNC_SEL_GPIO_2},
	{.ucPin = PAD_30, .ucFunc = PAD30_FUNC_SEL_GPIO_3},
	{.ucPin = PAD_31, .ucFunc = PAD31_FUNC_SEL_GPIO_4},
	{.ucPin = PAD_36, .ucFunc = PAD36_FUNC_SEL_GPIO_5},
	{.ucPin = PAD_38, .ucFunc = PAD38_FUNC_SEL_GPIO_6},
	{.ucPin = PAD_45, .ucFunc = PAD45_FUNC_SEL_GPIO_7},
};

static int gpio_eos_s3_configure(const struct device *dev,
				 gpio_pin_t gpio_num,
				 gpio_flags_t flags)
{
	const struct gpio_eos_s3_config *config = dev->config;
	uint8_t pin_cfg = config->pin_cfgs[gpio_num];
	GPIOCfgTypeDef gpio_cfg;
	PadConfig pad_conf;

	/* pin_cfg should be either 0 (primary) or 1 (secondary) */
	if (pin_cfg > 1) {
		return -EINVAL;
	}

	if (flags & GPIO_SINGLE_ENDED) {
		return -ENOTSUP;
	}

	gpio_cfg.ucGpioNum = gpio_num;
	pad_conf = pad_configs[(8 * pin_cfg) + gpio_num];
	gpio_cfg.xPadConf = &pad_conf;

	/* Configure PAD */
	if (flags & GPIO_PULL_UP) {
		gpio_cfg.xPadConf->ucPull = PAD_PULLUP;
	} else if (flags & GPIO_PULL_DOWN) {
		gpio_cfg.xPadConf->ucPull = PAD_PULLDOWN;
	} else {
		gpio_cfg.xPadConf->ucPull = PAD_NOPULL;
	}

	if ((flags & GPIO_DIR_MASK) == GPIO_INPUT) {
		gpio_cfg.xPadConf->ucMode = PAD_MODE_INPUT_EN;
		gpio_cfg.xPadConf->ucSmtTrg = PAD_SMT_TRIG_EN;
		HAL_PAD_Config(gpio_cfg.xPadConf);
	} else {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			MISC_CTRL->IO_OUTPUT |= BIT(gpio_num);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			MISC_CTRL->IO_OUTPUT &= ~BIT(gpio_num);
		}
		gpio_cfg.xPadConf->ucMode = PAD_MODE_OUTPUT_EN;
		HAL_PAD_Config(gpio_cfg.xPadConf);
	}

	return 0;
}

static int gpio_eos_s3_port_get_raw(const struct device *dev,
				    uint32_t *value)
{
	ARG_UNUSED(dev);

	*value = (MISC_CTRL->IO_INPUT & GPIOS_MASK);

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
	MISC_CTRL->IO_OUTPUT = (target_value & GPIOS_MASK);

	return 0;
}

static int gpio_eos_s3_port_set_bits_raw(const struct device *dev,
					 uint32_t mask)
{
	ARG_UNUSED(dev);

	MISC_CTRL->IO_OUTPUT |= (mask & GPIOS_MASK);

	return 0;
}

static int gpio_eos_s3_port_clear_bits_raw(const struct device *dev,
					   uint32_t mask)
{
	ARG_UNUSED(dev);

	MISC_CTRL->IO_OUTPUT &= ~(mask & GPIOS_MASK);

	return 0;
}

static int gpio_eos_s3_port_toggle_bits(const struct device *dev,
					uint32_t mask)
{
	ARG_UNUSED(dev);
	uint32_t target_value;
	uint32_t output_states = MISC_CTRL->IO_OUTPUT;

	target_value = output_states ^ mask;
	MISC_CTRL->IO_OUTPUT = (target_value & GPIOS_MASK);

	return 0;
}

static int gpio_eos_s3_manage_callback(const struct device *dev,
				       struct gpio_callback *callback, bool set)
{
	struct gpio_eos_s3_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_eos_s3_pin_interrupt_configure(const struct device *dev,
					       gpio_pin_t gpio_num,
					       enum gpio_int_mode mode,
					       enum gpio_int_trig trig)
{
	struct gpio_eos_s3_data *data = dev->data;
	const struct gpio_eos_s3_config *config = dev->config;
	uint8_t pin_cfg = config->pin_cfgs[gpio_num];
	GPIOCfgTypeDef gpio_cfg;
	PadConfig pad_conf;

	/* pin_cfg should be either 0 (primary) or 1 (secondary) */
	if (pin_cfg > 1) {
		return -EINVAL;
	}

	gpio_cfg.ucGpioNum = gpio_num;
	pad_conf = pad_configs[(8 * pin_cfg) + gpio_num];
	gpio_cfg.xPadConf = &pad_conf;

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* Reset IRQ configuration */
		int irq_num = HAL_GPIO_IntrCfg(&gpio_cfg);

		if (irq_num < 0) {
			return -EINVAL;
		}

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

		if (irq_num < 0) {
			return -EINVAL;
		}

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
	uint8_t intr_num = intr_status & GPIOS_MASK;

	INTR_CTRL->GPIO_INTR |= intr_num;

	for (int gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
		if (intr_num == data->gpio_irq_num[gpio_num]) {
			gpio_fire_callbacks(&data->callbacks,
					    dev, BIT(gpio_num));
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

static int gpio_eos_s3_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    gpio_eos_s3_isr,
		    DEVICE_DT_INST_GET(0),
		    0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static int gpio_eos_s3_init(const struct device *dev);

const struct gpio_eos_s3_config gpio_eos_s3_config = {
	.pin_cfgs = DT_INST_PROP(0, pin_cfgs),
};

static struct gpio_eos_s3_data gpio_eos_s3_data;

DEVICE_DT_INST_DEFINE(0,
		    gpio_eos_s3_init,
		    device_pm_control_nop,
		    &gpio_eos_s3_data,
		    &gpio_eos_s3_config,
		    POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_eos_s3_driver_api);
