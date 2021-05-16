/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>

/* WIC functions */
void eos_s3_wic_disable_irq(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR_EN_M4 &=
				~(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
	}
}

void eos_s3_wic_enable_irq(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR_EN_M4 |=
				(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
	}
}

void eos_s3_wic_clear_pending_irq(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR |=
				(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
	}
}

/* Whenever IRQ_CONNECT is used for EOS-S3 SoC, a proper IRQ wrapper
 * should be added here.
 */
extern void pl011_isr(void *arg);
extern void gpio_eos_s3_isr(void *arg);
extern void spi_dw_isr(void *arg);

void eos_s3_pl011_isr_wrapper(void *arg)
{
    eos_s3_wic_clear_pending_irq(Uart_IRQn);
    pl011_isr(arg);
}

void eos_s3_gpio_eos_s3_isr_wrapper(void *arg)
{
    /* GPIO controller has separate IRQ handling */
    gpio_eos_s3_isr(arg);
}

void eos_s3_spi_dw_isr_wrapper(void *arg)
{
    eos_s3_wic_clear_pending_irq(SpiMs_IRQn);
    spi_dw_isr(arg);
}
