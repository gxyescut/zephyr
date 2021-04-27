#include <soc.h>

/* EOS S3 has WIC (Wake-up Interrupt Controller) which is an independent
 * interrupt controller that also handles enabling, disabling and
 * clearing IRQs. To actually make IRQs work properly we need to
 * override NVIC functions and include WIC handling in them..
 */
void EOSS3_DisableIRQ(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] =
			(uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR_EN_M4 &=
				~(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
		__DSB();
		__ISB();
	}
}

void EOSS3_EnableIRQ(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		__COMPILER_BARRIER();
		NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] =
			(uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
		__COMPILER_BARRIER();
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR_EN_M4 |=
				(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
	}
}

void EOSS3_ClearPendingIRQ(IRQn_Type IRQn)
{
	if ((int32_t)(IRQn) >= 0) {
		NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] =
			(uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
		if ((int32_t)IRQn != WIC_GPIO_IRQ_BASE) {
			INTR_CTRL->OTHER_INTR |=
				(1 << (IRQn - WIC_OTHER_IRQ_BASE));
		}
	}
}

extern void pl011_isr(void *arg);
extern void gpio_eos_s3_isr(void *arg);
extern void spi_dw_isr(void *arg);

void eos_s3_pl011_isr_wrapper(void *arg)
{
    pl011_isr(arg);
    EOSS3_ClearPendingIRQ(Uart_IRQn);
}

void eos_s3_gpio_eos_s3_isr_wrapper(void *arg)
{
    gpio_eos_s3_isr(arg);
    EOSS3_ClearPendingIRQ(Gpio_IRQn);
}

void eos_s3_spi_dw_isr_wrapper(void *arg)
{
    spi_dw_isr(arg);
    EOSS3_ClearPendingIRQ(SpiMs_IRQn);
}
