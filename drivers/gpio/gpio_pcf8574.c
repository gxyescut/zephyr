/*
 * Copyright (c) 2021 Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_pcf8574

/**
 * @file Driver for PCF8574 I2C-based GPIO driver.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

#include "gpio_utils.h"

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(gpio_pcf8574);

struct pcf8574_irq_state {
	uint8_t rising;
	uint8_t falling;
};

/** Configuration data */
struct gpio_pcf8574_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;

	/** The master I2C device's name */
	const char * const i2c_master_dev_name;

	/** The slave address of the chip */
	uint16_t i2c_slave_addr;

	const struct gpio_dt_spec gpio_int;
	bool interrupt_enabled;
};

/** Runtime driver data */
struct gpio_pcf8574_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	const struct device *i2c_master;
	uint8_t reg_cache;
	struct k_sem lock;
	struct k_work work;
	struct pcf8574_irq_state irq_state;
	struct gpio_callback gpio_cb;
	const struct device *dev;

	/* user ISR cb */
	sys_slist_t cb;
};

/**
 * @brief Read pins state of PCF8574 I/O Port.
 *
 * Pin states are read from PCF8574 and saved to reg_cache.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
static int pcf8574_read_port(const struct device *dev)
{
	const struct gpio_pcf8574_config *config = dev->config;
	struct gpio_pcf8574_drv_data *drv_data = dev->data;
	uint8_t buf;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret |= i2c_read(drv_data->i2c_master, &buf, 1, config->i2c_slave_addr);

	if (ret) {
		LOG_ERR("PCF8574[0x%X]: error reading from port (%d)",
				config->i2c_slave_addr, ret);
	} else {
		drv_data->reg_cache = buf;
	}

	k_sem_give(&drv_data->lock);

	return ret;
}

/**
 * @brief Write pins state of PCF8574 I/O Port.
 *
 * Writes new states of all pins of PCF8574 and updates reg_cache value.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param value Pin states that will be set. Value of n-th bit will be its state.
 */
static int pcf8574_write_port(const struct device *dev, uint8_t value)
{
	const struct gpio_pcf8574_config *config = dev->config;
	struct gpio_pcf8574_drv_data *drv_data = dev->data;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret |= i2c_write(drv_data->i2c_master, &value, 1, config->i2c_slave_addr);

	if (ret != 0) {
		LOG_ERR("PCF8574[0x%X]: error writing to port (%d)",
				config->i2c_slave_addr, ret);
	} else {
		drv_data->reg_cache = value;
	}

	k_sem_give(&drv_data->lock);

	return ret;
}

/**
 * @brief Handles interrupt triggered by the interrupt pin of PCF8574 I/O Port.
 *
 * If nint_gpios is configured in device tree then this will be triggered each
 * time a gpio configured as an input changes state. The gpio input states are
 * read in this function which clears the interrupt.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
static void gpio_pcf8574_handle_interrupt(const struct device *dev)
{
	struct gpio_pcf8574_drv_data *drv_data = dev->data;
	struct pcf8574_irq_state *irq_state = &drv_data->irq_state;
	int ret = 0;
	uint8_t previous_state = drv_data->reg_cache;
	uint8_t current_state;
	uint8_t transitioned_pins;
	uint8_t interrupt_status = 0;

	/* Any interrupts enabled? */
	if (!irq_state->rising && !irq_state->falling) {
		return;
	}

	ret = pcf8574_read_port(dev);

	k_sem_take(&drv_data->lock, K_FOREVER);

	current_state = drv_data->reg_cache;

	/* Find out which input pins have changed state */
	transitioned_pins = previous_state ^ current_state;

	/* Mask gpio transactions with rising/falling edge interrupt config */
	interrupt_status = (irq_state->rising & transitioned_pins &
			  current_state);
	interrupt_status |= (irq_state->falling & transitioned_pins &
			   previous_state);

	k_sem_give(&drv_data->lock);

	if ((ret == 0) && (interrupt_status)) {
		gpio_fire_callbacks(&drv_data->cb, dev, interrupt_status);
	}
}

/**
 * @brief Work handler for PCF8574 interrupt
 *
 * @param work Work struct that contains pointer to interrupt handler function
 */
static void gpio_pcf8574_work_handler(struct k_work *work)
{
	struct gpio_pcf8574_drv_data *drv_data =
		CONTAINER_OF(work, struct gpio_pcf8574_drv_data, work);

	gpio_pcf8574_handle_interrupt(drv_data->dev);
}

/**
 * @brief ISR for intterupt pin of PCF8574
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param gpio_cb Pointer to callback function struct
 * @param pins Bitmask of pins that triggered interrupt
 */
static void gpio_pcf8574_init_cb(const struct device *dev,
				 struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct gpio_pcf8574_drv_data *drv_data =
		CONTAINER_OF(gpio_cb, struct gpio_pcf8574_drv_data, gpio_cb);

	ARG_UNUSED(pins);

	k_work_submit(&drv_data->work);
}

static int gpio_pcf8574_config(const struct device *dev,
			       gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_pcf8574_config *config = dev->config;
	struct gpio_pcf8574_drv_data *drv_data = dev->data;
	uint8_t reg_cache = drv_data->reg_cache;
	int ret = 0;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);
	/* Pin can only be configured as input or output */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		ret = -ENOTSUP;
	} else {
		/* For each pin, 0 == output LOW, 1 == output HIGH, 1 == input */
		if ((flags & GPIO_OUTPUT) != 0U) {
			if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
				reg_cache |= BIT(pin);
			} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
				reg_cache &= ~BIT(pin);
			}
		} else if ((flags & GPIO_INPUT) != 0U) {
			reg_cache |= BIT(pin);
		} else {
			ret = -ENOTSUP;
		}
	}

	k_sem_give(&drv_data->lock);

	if (!ret) {
		ret = pcf8574_write_port(dev, reg_cache);
	}

	if (ret) {
		LOG_ERR("PCF8574[0x%X]: error while configuring pin %d (%d)",
			config->i2c_slave_addr, pin, ret);
	}

	return ret;
}

static int gpio_pcf8574_port_get_raw(const struct device *dev,
				     uint32_t *value)
{
	struct gpio_pcf8574_drv_data * drv_data = dev->data;
	int ret = 0;

	ret = pcf8574_read_port(dev);

	if (ret) {
		return ret;
	}

	*value = drv_data->reg_cache;

	return ret;
}

static int gpio_pcf8574_port_set_masked_raw(const struct device *dev,
					      uint32_t mask, uint32_t value)
{
	struct gpio_pcf8574_drv_data * drv_data = dev->data;
	int ret = 0;
	uint8_t reg_cache = drv_data->reg_cache;

	reg_cache = (reg_cache & ~mask) | (mask & value);

	ret = pcf8574_write_port(dev, reg_cache);

	return ret;
}

static int gpio_pcf8574_port_set_bits_raw(const struct device *dev,
					  uint32_t mask)
{
	return gpio_pcf8574_port_set_masked_raw(dev, mask, mask);
}

static int gpio_pcf8574_port_clear_bits_raw(const struct device *dev,
					    uint32_t mask)
{
	return gpio_pcf8574_port_set_masked_raw(dev, mask, 0);
}

static int gpio_pcf8574_port_toggle_bits(const struct device *dev,
					 uint32_t mask)
{
	struct gpio_pcf8574_drv_data * drv_data = dev->data;
	uint16_t reg_cache;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	reg_cache = drv_data->reg_cache;
	reg_cache ^= mask;

	k_sem_give(&drv_data->lock);

	ret = pcf8574_write_port(dev, reg_cache);

	return ret;
}

static int gpio_pcf8574_pin_interrupt_configure(const struct device *dev,
						  gpio_pin_t pin,
						  enum gpio_int_mode mode,
						  enum gpio_int_trig trig)
{
    const struct gpio_pcf8574_config *cfg = dev->config;
	struct gpio_pcf8574_drv_data *drv_data = dev->data;
	struct pcf8574_irq_state *irq = &drv_data->irq_state;

	if (!cfg->interrupt_enabled) {
		return -ENOTSUP;
	}
	/* Device does not support level-triggered interrupts. */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	if (mode == GPIO_INT_MODE_DISABLED) {
		irq->falling &= ~BIT(pin);
		irq->rising &= ~BIT(pin);
	} else { /* GPIO_INT_MODE_EDGE */
		if (trig == GPIO_INT_TRIG_BOTH) {
			irq->falling |= BIT(pin);
			irq->rising |= BIT(pin);
		} else if (trig == GPIO_INT_TRIG_LOW) {
			irq->falling |= BIT(pin);
			irq->rising &= ~BIT(pin);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irq->falling &= ~BIT(pin);
			irq->rising |= BIT(pin);
		}
	}

	k_sem_give(&drv_data->lock);

	return 0;
}

static int gpio_pcf8574_manage_callback(const struct device *dev,
					struct gpio_callback *callback,
					bool set)
{
	struct gpio_pcf8574_drv_data *data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static const struct gpio_driver_api gpio_pcf8574_drv_api_funcs = {
	.pin_configure = gpio_pcf8574_config,
	.port_get_raw = gpio_pcf8574_port_get_raw,
	.port_set_masked_raw = gpio_pcf8574_port_set_masked_raw,
	.port_set_bits_raw = gpio_pcf8574_port_set_bits_raw,
	.port_clear_bits_raw = gpio_pcf8574_port_clear_bits_raw,
	.port_toggle_bits = gpio_pcf8574_port_toggle_bits,
	.pin_interrupt_configure = gpio_pcf8574_pin_interrupt_configure,
	.manage_callback = gpio_pcf8574_manage_callback,
};

static int gpio_pcf8574_init(const struct device *dev)
{
	const struct gpio_pcf8574_config * config = dev->config;
	struct gpio_pcf8574_drv_data * drv_data = dev->data;
	const struct device *i2c_master;
	int ret = 0;

	/* Find out the device struct of the I2C master */
	i2c_master = device_get_binding((char *)config->i2c_master_dev_name);
	if (!i2c_master) {
		return -EINVAL;
	}
	drv_data->i2c_master = i2c_master;

	k_sem_init(&drv_data->lock, 1, 1);

	/* Save pin states to reg_cache and clear interrupts if present */
	ret = pcf8574_read_port(dev);

	if (config->interrupt_enabled) {
		if (!device_is_ready(config->gpio_int.port)) {
			LOG_ERR("Cannot get pointer to gpio interrupt device");
			return -EINVAL;
		}

		drv_data->dev = dev;

		k_work_init(&drv_data->work, gpio_pcf8574_work_handler);

		ret = gpio_pin_configure_dt(&config->gpio_int, GPIO_INPUT);
		if (ret) {
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&config->gpio_int,
						GPIO_INT_EDGE_TO_ACTIVE);
		if (ret) {
			return ret;
		}

		gpio_init_callback(&drv_data->gpio_cb,
					gpio_pcf8574_init_cb,
					BIT(config->gpio_int.pin));

		ret = gpio_add_callback(config->gpio_int.port,
					&drv_data->gpio_cb);
	}

	return ret;
}

#define GPIO_PCF8574_DEVICE_INSTANCE(inst)					\
	static const struct gpio_pcf8574_config gpio_pcf8574_##inst##_cfg = {	\
		.common = {							\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),	\
		},								\
		.i2c_master_dev_name = DT_INST_BUS_LABEL(inst),			\
		.i2c_slave_addr = DT_INST_REG_ADDR(inst),			\
		.interrupt_enabled = DT_INST_NODE_HAS_PROP(inst, nint_gpios),	\
		.gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, nint_gpios, {0}),	\
	};									\
										\
	static struct gpio_pcf8574_drv_data gpio_pcf8574_##inst##_drvdata = {	\
		.reg_cache = 0x0						\
	};  									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
		gpio_pcf8574_init,						\
		NULL,								\
		&gpio_pcf8574_##inst##_drvdata,					\
		&gpio_pcf8574_##inst##_cfg,					\
		POST_KERNEL, CONFIG_GPIO_PCF8574_INIT_PRIORITY,			\
		&gpio_pcf8574_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PCF8574_DEVICE_INSTANCE)
