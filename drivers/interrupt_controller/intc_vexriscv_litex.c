/*
 * Copyright (c) 2018 - 2022 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_eth0

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/zephyr.h>
#include <zephyr/types.h>

#define IRQ_MASK		DT_REG_ADDR_BY_NAME(DT_INST(0, vexriscv_intc0), irq_mask)
#define IRQ_PENDING		DT_REG_ADDR_BY_NAME(DT_INST(0, vexriscv_intc0), irq_pending)

static inline void vexriscv_litex_irq_setmask(uint32_t mask)
{
	__asm__ volatile ("csrw %0, %1" :: "i"(IRQ_MASK), "r"(mask));
}

static inline uint32_t vexriscv_litex_irq_getmask(void)
{
	uint32_t mask;

	__asm__ volatile ("csrr %0, %1" : "=r"(mask) : "i"(IRQ_MASK));
	return mask;
}

static inline uint32_t vexriscv_litex_irq_pending(void)
{
	uint32_t pending;

	__asm__ volatile ("csrr %0, %1" : "=r"(pending) : "i"(IRQ_PENDING));
	return pending;
}

static inline void vexriscv_litex_irq_setie(uint32_t ie)
{
	if (ie) {
		__asm__ volatile ("csrrs x0, mstatus, %0"
				:: "r"(MSTATUS_IEN));
	} else {
		__asm__ volatile ("csrrc x0, mstatus, %0"
				:: "r"(MSTATUS_IEN));
	}
}

static void vexriscv_litex_irq_handler(const void *device)
{
	struct _isr_table_entry *ite;
	uint32_t pending, mask, irqs;

	pending = vexriscv_litex_irq_pending();
	mask = vexriscv_litex_irq_getmask();
	irqs = pending & mask;

	for (int i = 0; i < CONFIG_NUM_IRQS; i++)
	{
		if (irqs & (1 << i)) {
			ite = &_sw_isr_table[i];
			ite->isr(ite->arg);
		}
	}
}

void arch_irq_enable(unsigned int irq)
{
	vexriscv_litex_irq_setmask(vexriscv_litex_irq_getmask() | (1 << irq));
}

void arch_irq_disable(unsigned int irq)
{
	vexriscv_litex_irq_setmask(vexriscv_litex_irq_getmask() & ~(1 << irq));
}

int arch_irq_is_enabled(unsigned int irq)
{
	return vexriscv_litex_irq_getmask() & (1 << irq);
}

static int vexriscv_litex_irq_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	__asm__ volatile ("csrrs x0, mie, %0"
			:: "r"((1 << RISCV_MACHINE_TIMER_IRQ)
				| (1 << RISCV_MACHINE_EXT_IRQ)));
	vexriscv_litex_irq_setie(1);
	IRQ_CONNECT(RISCV_MACHINE_EXT_IRQ, 0, vexriscv_litex_irq_handler,
			NULL, 0);

	return 0;
}

SYS_INIT(vexriscv_litex_irq_init, PRE_KERNEL_1,
		CONFIG_INTC_INIT_PRIORITY);
