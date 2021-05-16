/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <sys/util.h>
#include <eoss3_dev.h>
#include <devicetree.h>
#include <arch/arm/aarch32/irq.h>

/* Available frequencies */
#define HSOSC_1MHZ	1024000
#define HSOSC_2MHZ	(2*HSOSC_1MHZ)
#define HSOSC_3MHZ	(3*HSOSC_1MHZ)
#define HSOSC_4MHZ	(4*HSOSC_1MHZ)
#define HSOSC_5MHZ	(5*HSOSC_1MHZ)
#define HSOSC_6MHZ	(6*HSOSC_1MHZ)
#define HSOSC_8MHZ	(8*HSOSC_1MHZ)
#define HSOSC_9MHZ	(9*HSOSC_1MHZ)
#define HSOSC_10MHZ	(10*HSOSC_1MHZ)
#define HSOSC_12MHZ	(12*HSOSC_1MHZ)
#define HSOSC_15MHZ	(15*HSOSC_1MHZ)
#define HSOSC_16MHZ	(16*HSOSC_1MHZ)
#define HSOSC_18MHZ	(18*HSOSC_1MHZ)
#define HSOSC_20MHZ	(20*HSOSC_1MHZ)
#define HSOSC_21MHZ	(21*HSOSC_1MHZ)
#define HSOSC_24MHZ	(24*HSOSC_1MHZ)
#define HSOSC_27MHZ	(27*HSOSC_1MHZ)
#define HSOSC_30MHZ	(30*HSOSC_1MHZ)
#define HSOSC_32MHZ	(32*HSOSC_1MHZ)
#define HSOSC_35MHZ	(35*HSOSC_1MHZ)
#define HSOSC_36MHZ	(36*HSOSC_1MHZ)
#define HSOSC_40MHZ	(40*HSOSC_1MHZ)
#define HSOSC_45MHZ	(45*HSOSC_1MHZ)
#define HSOSC_48MHZ	(48*HSOSC_1MHZ)
#define HSOSC_54MHZ	(54*HSOSC_1MHZ)
#define HSOSC_60MHZ	(60*HSOSC_1MHZ)
#define HSOSC_64MHZ	(64*HSOSC_1MHZ)
#define HSOSC_70MHZ	(70*HSOSC_1MHZ)
#define HSOSC_72MHZ	(72*HSOSC_1MHZ)
#define HSOSC_80MHZ	(80*HSOSC_1MHZ)

#define OSC_CLK_LOCKED()	(AIP->OSC_STA_0 & 0x1)
#define OSC_SET_FREQ_INC(FREQ)	(AIP->OSC_CTRL_1 = ((FREQ / 32768) - 3) & 0xFFF)
#define OSC_GET_FREQ_INC()	(((AIP->OSC_CTRL_1 & 0xFFF) + 3) * 32768)

#define EOS_S3_MAX_PAD_NR			45

void eos_s3_lock_enable(void);
void eos_s3_lock_disable(void);

int eos_s3_io_mux(uint32_t pad_nr, uint32_t pad_cfg);

/* Override default IRQ_CONNECT:
 * EOS S3 has WIC (Wake-up Interrupt Controller) which is an independent
 * interrupt controller that also handles enabling, disabling and
 * clearing IRQs. To handle ISRs properly we need to wrap original ISR
 * and append WIC functions.
 */
#undef IRQ_CONNECT
#define IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
    void eos_s3_##isr_p##_wrapper(void *arg); \
    Z_ISR_DECLARE(irq_p, 0, eos_s3_##isr_p##_wrapper, isr_param_p); \
    z_arm_irq_priority_set(irq_p, priority_p, flags_p);

#define WIC_GPIO_IRQ_BASE (5UL)
#define WIC_OTHER_IRQ_BASE (6UL)

/* WIC functions */
void eos_s3_wic_enable_irq(IRQn_Type IRQn);
void eos_s3_wic_disable_irq(IRQn_Type IRQn);
void eos_s3_wic_clear_pending_irq(IRQn_Type IRQn);

#endif /* _SOC__H_ */
