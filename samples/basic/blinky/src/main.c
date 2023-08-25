/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define IRQ_NODE  DT_ALIAS(bleirq)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(IRQ_NODE, gpios);

static struct gpio_callback	gpio_cb;

static void irq_isr(const struct device *unused1,
		       struct gpio_callback *unused2,
		       uint32_t unused3)
{
	printk(".\n");
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&irq)) {
		printk("Device not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&irq, GPIO_INPUT);
	if (ret < 0) {
		printk("IRQ PIN Config fail\n");
		return 0;
	}

	gpio_init_callback(&gpio_cb, irq_isr, BIT(irq.pin));
	ret = gpio_add_callback(irq.port, &gpio_cb);
	if (ret) {
		printk("Add callback fail\n");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("Interrupt config fail\n");
		return ret;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
