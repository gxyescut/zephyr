/*
 * Copyright (c) 2022 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <drivers/led.h>
#include <drivers/uart.h>
#include <sys/util.h>
#include <zephyr.h>

void main(void)
{
	const struct device *led_dev = DEVICE_DT_GET_ANY(fomu_sbled);
	const struct device *uart_dev = DEVICE_DT_GET_ANY(fomu_uart);

	char c = '\0';
	while(c != 'q') {
		while (uart_poll_in(uart_dev, &c));
		printk("Got letter %c\n", c);
	}

	for (int i = 0;; i++) {
		led_off(led_dev, 0);
		led_off(led_dev, 1);
		led_off(led_dev, 2);

		led_on(led_dev, i % 3);

		k_msleep(1000);
	}
}
