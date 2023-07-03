/*
 * Copyright (c) 2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_PL011_AMBIQ_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_PL011_AMBIQ_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "uart_pl011_registers.h"

static inline void pl011_ambiq_enable_clk(const struct device *dev)
{
	get_uart(dev)->cr |=  PL011_CR_AMBIQ_CLKEN;
}

static inline int pl011_ambiq_clk_set(const struct device *dev, uint32_t clk)
{
	uint8_t clksel;

	switch (clk) {
	case 3000000:
		clksel = PL011_CR_AMBIQ_CLKSEL_3MHZ;
		break;
	case 6000000:
		clksel = PL011_CR_AMBIQ_CLKSEL_6MHZ;
		break;
	case 12000000:
		clksel = PL011_CR_AMBIQ_CLKSEL_12MHZ;
		break;
	case 24000000:
		clksel = PL011_CR_AMBIQ_CLKSEL_24MHZ;
		break;
	case 48000000:
		clksel = PL011_CR_AMBIQ_CLKSEL_48MHZ;
		break;
	default:
		return -EINVAL;
	}

	get_uart(dev)->cr |= FIELD_PREP(PL011_CR_AMBIQ_CLKSEL, clksel);
	return 0;
}

static inline int clk_enable_ambiq_uart(const struct device *dev, uint32_t clk)
{
	pl011_ambiq_enable_clk(dev);
	return pl011_ambiq_clk_set(dev, clk);
}

#define AMBIQ_UART_PWRCTRL_ID(uart) AM_HAL_PWRCTRL_PERIPH_UART ## uart
#define AMBIQ_UART_PWRCTRL_ENABLE(idx) \
	am_hal_pwrctrl_periph_enable(AMBIQ_UART_PWRCTRL_ID(idx));

#define QUIRK_AMBIQ_UART_DEFINE(n)						\
										\
	static int pwr_on_ambiq_uart_##n(void)					\
	{									\
		return AMBIQ_UART_PWRCTRL_ENABLE(				\
				DT_INST_PROP(n, ambiq_uart_instance));		\
	}

#define PL011_QUIRK_AMBIQ_UART_DEFINE(n)					\
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(n), ambiq_uart),		\
		    (QUIRK_AMBIQ_UART_DEFINE(n)), ())

#endif /* ZEPHYR_DRIVERS_SERIAL_UART_PL011_AMBIQ_H_ */
