/*
 * Copyright (c) 2022 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT fomu_uart

#include <kernel.h>
#include <arch/cpu.h>
#include <init.h>
#include <irq.h>
#include <device.h>
#include <drivers/uart.h>
#include <zephyr/types.h>

#include "fomu/include/usb.h"

/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register. Waits for space if transmitter is full.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_fomu_poll_out(const struct device *dev, unsigned char c)
{
	while (!usb_can_putc())
		usb_poll();
	usb_putc(c);
	usb_poll();
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_fomu_poll_in(const struct device *dev, unsigned char *c)
{
	usb_poll();
	if (usb_can_getc())
	{
		*c = usb_getc();
		return 0;
	}
	else
	{
		return -1;
	}
}

static const struct uart_driver_api uart_fomu_driver_api = {
	.poll_in		= uart_fomu_poll_in,
	.poll_out		= uart_fomu_poll_out,
	.err_check		= NULL,
};

static int uart_fomu_init(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQ(0, irq), DT_INST_IRQ(0, priority),
			usb_isr, DEVICE_DT_INST_GET(0),
			0);

	irq_enable(DT_INST_IRQ(0, irq));

	usb_init();
	usb_connect();

	while (usb_can_getc())
		usb_poll();

	return 0;
}

DEVICE_DT_INST_DEFINE(0,
		uart_fomu_init,
		NULL,
		NULL, NULL,
		POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY,
		(void *)&uart_fomu_driver_api);
