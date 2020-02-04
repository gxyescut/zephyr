/*
 * Copyright (c) 2020, Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/gpio.h>
#include <soc.h>
#include <eoss3_hal_gpio.h>
#include <eoss3_hal_pads.h>
#include <eoss3_hal_pad_config.h>

#include "gpio_utils.h"

#define MAX_GPIOS	8

struct gpio_eos_s3_config {
};

struct gpio_eos_s3_data {
	/* port ISR callback routine address */
	sys_slist_t callbacks;

	/* Runtime configuration */
	u8_t gpio_irq_num[MAX_GPIOS];
	u8_t pad_num[MAX_GPIOS];
};

static int find_gpio(GPIOCfgTypeDef *cfg)
{
	switch (cfg->xPadConf->ucPin) {
	case PAD_6:
		cfg->ucGpioNum = GPIO_0;
		cfg->xPadConf->ucFunc = PAD6_FUNC_SEL_GPIO_0;
		break;
	case PAD_9:
		cfg->ucGpioNum = GPIO_1;
		cfg->xPadConf->ucFunc = PAD9_FUNC_SEL_GPIO_1;
		break;
	case PAD_11:
		cfg->ucGpioNum = GPIO_2;
		cfg->xPadConf->ucFunc = PAD11_FUNC_SEL_GPIO_2;
		break;
	case PAD_14:
		cfg->ucGpioNum = GPIO_3;
		cfg->xPadConf->ucFunc = PAD14_FUNC_SEL_GPIO_3;
		break;
	case PAD_18:
		cfg->ucGpioNum = GPIO_4;
		cfg->xPadConf->ucFunc = PAD18_FUNC_SEL_GPIO_4;
		break;
	case PAD_21:
		cfg->ucGpioNum = GPIO_5;
		cfg->xPadConf->ucFunc = PAD21_FUNC_SEL_GPIO_5;
		break;
	case PAD_22:
		cfg->ucGpioNum = GPIO_6;
		cfg->xPadConf->ucFunc = PAD22_FUNC_SEL_GPIO_6;
		break;
	case PAD_23:
		cfg->ucGpioNum = GPIO_7;
		cfg->xPadConf->ucFunc = PAD23_FUNC_SEL_GPIO_7;
		break;
	case PAD_24:
		cfg->ucGpioNum = GPIO_0;
		cfg->xPadConf->ucFunc = PAD24_FUNC_SEL_GPIO_0;
		break;
	case PAD_26:
		cfg->ucGpioNum = GPIO_1;
		cfg->xPadConf->ucFunc = PAD26_FUNC_SEL_GPIO_1;
		break;
	case PAD_28:
		cfg->ucGpioNum = GPIO_2;
		cfg->xPadConf->ucFunc = PAD28_FUNC_SEL_GPIO_2;
		break;
	case PAD_30:
		cfg->ucGpioNum = GPIO_3;
		cfg->xPadConf->ucFunc = PAD30_FUNC_SEL_GPIO_3;
		break;
	case PAD_31:
		cfg->ucGpioNum = GPIO_4;
		cfg->xPadConf->ucFunc = PAD31_FUNC_SEL_GPIO_4;
		break;
	case PAD_36:
		cfg->ucGpioNum = GPIO_5;
		cfg->xPadConf->ucFunc = PAD36_FUNC_SEL_GPIO_5;
		break;
	case PAD_38:
		cfg->ucGpioNum = GPIO_6;
		cfg->xPadConf->ucFunc = PAD38_FUNC_SEL_GPIO_6;
		break;
	case PAD_45:
		cfg->ucGpioNum = GPIO_7;
		cfg->xPadConf->ucFunc = PAD45_FUNC_SEL_GPIO_7;
		break;
	default:
		return -EINVAL;
	};

	return 0;
}

static int gpio_eos_s3_configure(struct device *dev, int access_op, u32_t pin,
				 int flags)
{
	struct gpio_eos_s3_data *data = dev->driver_data;
	int ret;
	GPIOCfgTypeDef gpio_cfg;
	PadConfig pad_conf;
	int irq_num;

	gpio_cfg.xPadConf = &pad_conf;

	/* Check for an invalid pin configuration */
	if ((flags & GPIO_INT) && (flags & GPIO_DIR_OUT))
		return -EINVAL;

	/* Accessing by port is not supported */
	if (access_op == GPIO_ACCESS_BY_PORT)
		return -ENOTSUP;

	gpio_cfg.xPadConf->ucPin = pin;
	ret = find_gpio(&gpio_cfg);

	if (ret)
		return ret;

	data->pad_num[gpio_cfg.ucGpioNum] = pin;

	if (flags & GPIO_PUD_PULL_UP)
		gpio_cfg.xPadConf->ucPull = PAD_PULLUP;
	else if (flags & GPIO_PUD_PULL_DOWN)
		gpio_cfg.xPadConf->ucPull = PAD_PULLDOWN;
	else
		gpio_cfg.xPadConf->ucPull = PAD_NOPULL;

	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {

		if (flags & GPIO_INT_ACTIVE_HIGH)
			gpio_cfg.pol_type = RISE_HIGH;
		else
			gpio_cfg.pol_type = FALL_LOW;

		if (flags & GPIO_INT_EDGE)
			gpio_cfg.intr_type = EDGE_TRIGGERED;
		else
			gpio_cfg.intr_type = LEVEL_TRIGGERED;

		gpio_cfg.xPadConf->ucMode = PAD_MODE_INPUT_EN;
		gpio_cfg.xPadConf->ucSmtTrg = PAD_SMT_TRIG_EN;

		HAL_PAD_Config(gpio_cfg.xPadConf);
		irq_num = HAL_GPIO_IntrCfg(&gpio_cfg);

		if (irq_num < 0)
			return irq_num;

		data->gpio_irq_num[gpio_cfg.ucGpioNum] = BIT((u8_t)irq_num);


	} else { /* GPIO_DIR_OUT */

		gpio_cfg.xPadConf->ucMode = PAD_MODE_OUTPUT_EN;
		HAL_PAD_Config(gpio_cfg.xPadConf);
	}

	return 0;
}

static int gpio_eos_s3_write(struct device *dev, int access_op,
			     u32_t pin, u32_t value)
{
	struct gpio_eos_s3_data *data = dev->driver_data;

	/* Accessing by port is not supported */
	if (access_op == GPIO_ACCESS_BY_PORT)
		return -ENOTSUP;

	for (u8_t gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
		if (pin == data->pad_num[gpio_num]) {
			HAL_GPIO_Write(gpio_num, (u8_t)value);
			return 0;
		}
	}

	return -EINVAL;
}

static int gpio_eos_s3_read(struct device *dev,
			   int access_op, u32_t pin, u32_t *value)
{
	struct gpio_eos_s3_data *data = dev->driver_data;

	/* Accessing by port is not supported */
	if (access_op == GPIO_ACCESS_BY_PORT)
		return -ENOTSUP;

	for (u8_t gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
		if (pin == data->pad_num[gpio_num]) {
			HAL_GPIO_Read(gpio_num, (u8_t *)value);
			return 0;
		}
	}

	return -EINVAL;
}

static int gpio_eos_s3_manage_callback(struct device *dev,
				       struct gpio_callback *callback, bool set)
{
	struct gpio_eos_s3_data *data = dev->driver_data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_eos_s3_enable_callback(struct device *dev,
				       int access_op, u32_t pin)
{
	struct gpio_eos_s3_data *data = dev->driver_data;

	if (access_op == GPIO_ACCESS_BY_PORT)
		return -ENOTSUP;

	for (u8_t gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
		if (pin == data->pad_num[gpio_num]) {
			INTR_CTRL->GPIO_INTR_EN_M4 |=
					data->gpio_irq_num[gpio_num];
			return 0;
		}
	}

	return -EINVAL;
}

static int gpio_eos_s3_disable_callback(struct device *dev,
				       int access_op, u32_t pin)
{
	struct gpio_eos_s3_data *data = dev->driver_data;

	if (access_op == GPIO_ACCESS_BY_PORT)
		return -ENOTSUP;

	for (u8_t gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
		if (pin == data->pad_num[gpio_num]) {
			INTR_CTRL->GPIO_INTR_EN_M4 &=
					~data->gpio_irq_num[gpio_num];
			return 0;
		}
	}

	return -EINVAL;
}

static void gpio_eos_s3_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct gpio_eos_s3_data *data = dev->driver_data;
	u8_t intr_status = INTR_CTRL->GPIO_INTR;
	u8_t pin;

	for (u8_t i = 0; i < MAX_GPIOS; i++) {
		u8_t enabled_intr = BIT(i);

		if (intr_status & enabled_intr) {
			for (u8_t gpio_num = 0; gpio_num < MAX_GPIOS; gpio_num++) {
				if (enabled_intr == data->gpio_irq_num[gpio_num]) {
					pin = data->pad_num[gpio_num];
					gpio_fire_callbacks(&data->callbacks,
							    dev, BIT(pin));
				}
			}
			INTR_CTRL->GPIO_INTR |= enabled_intr;
		}
	}
}

static const struct gpio_driver_api gpio_eos_s3_driver_api = {
	.config = gpio_eos_s3_configure,
	.write = gpio_eos_s3_write,
	.read = gpio_eos_s3_read,
	.manage_callback = gpio_eos_s3_manage_callback,
	.enable_callback = gpio_eos_s3_enable_callback,
	.disable_callback = gpio_eos_s3_disable_callback,
};

#ifdef CONFIG_GPIO_EOS_S3
static int gpio_eos_s3_init(struct device *dev);

static const struct gpio_eos_s3_config gpio_eos_s3_config;

static struct gpio_eos_s3_data gpio_eos_s3_data;

DEVICE_AND_API_INIT(gpio_eos_s3, DT_GPIO_EOS_S3_NAME,
		    gpio_eos_s3_init,
		    &gpio_eos_s3_data, &gpio_eos_s3_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_eos_s3_driver_api);

static int gpio_eos_s3_init(struct device *dev)
{
	IRQ_CONNECT(Gpio_IRQn, DT_GPIO_EOS_S3_IRQ_PRI,
		    gpio_eos_s3_isr, DEVICE_GET(gpio_eos_s3), 0);

	irq_enable(Gpio_IRQn);

	return 0;
}
#endif /* CONFIG_GPIO_EOS_S3 */
