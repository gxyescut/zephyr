/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/pwm.h>
#include <zephyr/types.h>
#include <quick_feather_pwm_gen.h>
#include <board.h>
#include <soc.h>

#define PWM_GEN_ENABLE		0x7
#define PWM_GEN_DISABLE		0x0

#define NUMBER_OF_CHANNELS	3

struct pwm_gen_eos_s3_cfg {
	PWM_GEN_TypeDef *base;
	u32_t divider;
};

enum {
	PWM_GEN_EN,
	PWM_GEN_WIDTH,
	PWM_GEN_PERIOD,
};

static void pwm_gen_eos_s3_set(struct device *dev, u32_t nr,
			       u8_t reg, u32_t val)
{
	const struct pwm_gen_eos_s3_cfg *cfg = dev->config->config_info;

	switch (reg) {
	case PWM_GEN_EN:
		cfg->base->PWM_EN = val;
		break;
	case PWM_GEN_WIDTH:
		switch (nr) {
		case 0:
			cfg->base->PWM0_WIDTH = val;
			break;
		case 1:
			cfg->base->PWM1_WIDTH = val;
			break;
		case 2:
			cfg->base->PWM2_WIDTH = val;
			break;
		}
		break;
	case PWM_GEN_PERIOD:
		switch (nr) {
		case 0:
			cfg->base->PWM0_PERIOD = val;
			break;
		case 1:
			cfg->base->PWM1_PERIOD = val;
			break;
		case 2:
			cfg->base->PWM2_PERIOD = val;
			break;
		}
		break;
	}
}

int pwm_gen_eos_s3_init(struct device *dev)
{
	eos_s3_io_mux(PWM0_PAD, PWM0_PAD_CFG);
	eos_s3_io_mux(PWM1_PAD, PWM1_PAD_CFG);
	eos_s3_io_mux(PWM2_PAD, PWM2_PAD_CFG);

	pwm_gen_eos_s3_set(dev, 0, PWM_GEN_EN, PWM_GEN_ENABLE);

	return 0;
}

int pwm_gen_eos_s3_pin_set(struct device *dev, u32_t pwm, u32_t period_cycles,
			   u32_t pulse_cycles, pwm_flags_t flags)
{
	ARG_UNUSED(flags);

	if (pwm >= NUMBER_OF_CHANNELS) {
		return -EINVAL;
	}

	pwm_gen_eos_s3_set(dev, pwm, PWM_GEN_EN, PWM_GEN_DISABLE);
	pwm_gen_eos_s3_set(dev, pwm, PWM_GEN_WIDTH, pulse_cycles);
	pwm_gen_eos_s3_set(dev, pwm, PWM_GEN_PERIOD, period_cycles);
	pwm_gen_eos_s3_set(dev, pwm, PWM_GEN_EN, PWM_GEN_ENABLE);

	return 0;
}

int pwm_gen_eos_s3_get_cycles_per_sec(struct device *dev, u32_t pwm,
				      u64_t *cycles)
{
	const struct pwm_gen_eos_s3_cfg *cfg = dev->config->config_info;

	if (pwm >= NUMBER_OF_CHANNELS) {
		return -EINVAL;
	}

	/* All PWM channels have the same cycles values */
	*cycles = sys_clock_hw_cycles_per_sec() / cfg->divider;

	return 0;
}

static const struct pwm_driver_api pwm_gen_eos_s3_driver_api = {
	.pin_set             = pwm_gen_eos_s3_pin_set,
	.get_cycles_per_sec  = pwm_gen_eos_s3_get_cycles_per_sec,
};

static const struct pwm_gen_eos_s3_cfg pwm_gen_eos_s3_cfg_0 = {
	.base = (PWM_GEN_TypeDef *)DT_PWM_GEN_EOS_S3_BASE_ADDRESS,
	.divider = DT_PWM_GEN_EOS_S3_DIVIDER,
};

DEVICE_AND_API_INIT(pwm_gen_eos_s3, DT_PWM_GEN_EOS_S3_NAME,
		    &pwm_gen_eos_s3_init,
		    NULL, &pwm_gen_eos_s3_cfg_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &pwm_gen_eos_s3_driver_api);
