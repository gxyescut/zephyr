/*
 * Copyright 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_clock

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/clock_control_ambiq.h>
#include <am_mcu_apollo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control, CONFIG_CLOCK_CONTROL_LOG_LEVEL);


struct ambiq_clock_config {
	uint32_t clock_freq;
	const struct pinctrl_dev_config *pcfg;
};


static int ambiq_clock_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);

	int ret = 0;
	am_hal_mcuctrl_control_arg_t arg = {0};
	arg.b_arg_hfxtal_in_use    = true;
	arg.b_arg_apply_ext_source = false;
	arg.b_arg_force_update     = false;

	uint32_t clock_name = (uint32_t)sub_system;
	if (clock_name >= CLOCK_CONTROL_AMBIQ_TYPE_MAX) {
		return -EINVAL;
	}

	switch (clock_name) {
	case CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE:
		arg.ui32_arg_hfxtal_user_mask = 1 << AM_HAL_HFXTAL_BLE_CONTROLLER_EN;
		arg.b_arg_enable_HfXtalClockout = true;
		ret = am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, (void *)&arg);
		break;
	case CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL:
		ret = am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE, 0);
	default:
		break;
	}

	return ret;
}

static int ambiq_clock_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);

	int ret = 0;
	am_hal_mcuctrl_control_arg_t arg = {0};
	arg.b_arg_hfxtal_in_use    = true;
	arg.b_arg_apply_ext_source = false;
	arg.b_arg_force_update     = false;

	uint32_t clock_name = (uint32_t)sub_system;
	if (clock_name >= CLOCK_CONTROL_AMBIQ_TYPE_MAX) {
		return -EINVAL;
	}

	switch (clock_name) {
	case CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE:
		arg.ui32_arg_hfxtal_user_mask = 1 << AM_HAL_HFXTAL_BLE_CONTROLLER_EN;
		arg.b_arg_enable_HfXtalClockout = true;
		ret = am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE, (void *)&arg);
		break;
	case CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL:
		ret = am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE, 0);
		break;
	default:
		break;
	}

	return 0;
}

static inline int ambiq_clock_get_rate(const struct device *dev,
                       clock_control_subsys_t sub_system, uint32_t *rate)
{
	ARG_UNUSED(sub_system);

	const struct ambiq_clock_config *cfg = dev->config;
	*rate = cfg->clock_freq;

	return 0;
}

static inline int ambiq_clock_configure(const struct device *dev,
                       clock_control_subsys_t sub_system, void *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sub_system);
	ARG_UNUSED(data);

	return 0;
}

static int ambiq_clock_init(const struct device *dev)
{
	const struct ambiq_clock_config *cfg = dev->config;
	int ret = 0;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	return ret;
}

static const struct clock_control_driver_api ambiq_clock_driver_api = {
	.on = ambiq_clock_on,
	.off = ambiq_clock_off,
	.get_rate = ambiq_clock_get_rate,
	.configure = ambiq_clock_configure,
};


#define AMBIQ_CLOCK_INIT(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                \
	static const struct ambiq_clock_config ambiq_clock_config##n = {                          \
		.clock_freq = DT_INST_PROP(n, clock_frequency),                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)};                                           \
	DEVICE_DT_INST_DEFINE(n, ambiq_clock_init, NULL, NULL, &ambiq_clock_config##n,            \
			      POST_KERNEL, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &ambiq_clock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBIQ_CLOCK_INIT)
