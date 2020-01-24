/*
 * Copyright (c) 2018 - 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <irq.h>
#include <drivers/timer/system_timer.h>

#define TIMER_BASE	        DT_INST_0_LITEX_TIMER0_BASE_ADDRESS
#define TIMER_LOAD_ADDR		((TIMER_BASE) + 0x00)
#define TIMER_RELOAD_ADDR	((TIMER_BASE) + 0x10)
#define TIMER_EN_ADDR		((TIMER_BASE) + 0x20)
#define TIMER_EV_UPDATE_VALUE	((TIMER_BASE) + 0x24)
#define TIMER_EV_READ_ADDR	((TIMER_BASE) + 0x28)
#define TIMER_EV_PENDING_ADDR	((TIMER_BASE) + 0x3c)
#define TIMER_EV_ENABLE_ADDR	((TIMER_BASE) + 0x40)


#define TIMER_EV		0x1
#define TIMER_IRQ		DT_INST_0_LITEX_TIMER0_IRQ_0
#define TIMER_DISABLE		0x0
#define TIMER_ENABLE		0x1

static u32_t accumulated_cycle_count;

static void litex_timer_irq_handler(void *device)
{
	ARG_UNUSED(device);
	int key = irq_lock();

	litex_write8(TIMER_EV, TIMER_EV_PENDING_ADDR);
	accumulated_cycle_count += k_ticks_to_cyc_floor32(1);
	z_clock_announce(1);
	irq_unlock(key);
}

u32_t z_timer_cycle_get_32(void)
{
	static u32_t last_cycle_count = 0;
	u32_t cycles = 0u;
	u32_t total_cycles = 0u;
	do {
		int key = irq_lock();

		litex_write8(TIMER_ENABLE, TIMER_EV_UPDATE_VALUE);
		cycles = litex_read32(TIMER_EV_READ_ADDR);
		litex_write8(TIMER_DISABLE, TIMER_EV_UPDATE_VALUE);

		total_cycles = accumulated_cycle_count + (k_ticks_to_cyc_floor32(1) - cycles);
		irq_unlock(key);

	} while(total_cycles < last_cycle_count);
	last_cycle_count = total_cycles;

	return total_cycles;
}

/* tickless kernel is not supported */
u32_t z_clock_elapsed(void)
{
	return 0;
}

int z_clock_driver_init(struct device *device)
{
	ARG_UNUSED(device);
	IRQ_CONNECT(TIMER_IRQ, DT_INST_0_LITEX_TIMER0_IRQ_0_PRIORITY,
			litex_timer_irq_handler, NULL, 0);
	irq_enable(TIMER_IRQ);

	litex_write8(TIMER_DISABLE, TIMER_EN_ADDR);
	litex_write32(k_ticks_to_cyc_floor32(1), TIMER_RELOAD_ADDR);
	litex_write32(k_ticks_to_cyc_floor32(1), TIMER_LOAD_ADDR);
	litex_write8(TIMER_ENABLE, TIMER_EN_ADDR);
	litex_write8(litex_read8(TIMER_EV_PENDING_ADDR), TIMER_EV_PENDING_ADDR);
	litex_write8(TIMER_EV, TIMER_EV_ENABLE_ADDR);

	return 0;
}
