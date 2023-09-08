/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_AMBIQ_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_AMBIQ_H_

#include <zephyr/device.h>
#include <zephyr/sys/onoff.h>
#include <zephyr/drivers/clock_control.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Clocks handled by the CLOCK peripheral.
 *
 * Enum shall be used as a sys argument in clock_control API.
 */
enum clock_control_ambiq_type {
	CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE,
	CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_USB,
	CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_ADC,
	CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_AUADC,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_DBGCTRL,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_CLKGEN_MISC,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_CLKGEN_CLKOUT,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_PDM,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_IIS,
	CLOCK_CONTROL_AMBIQ_TYPE_HCXTAL_IOM,
	CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL,
	CLOCK_CONTROL_AMBIQ_TYPE_MAX
};


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_CLOCK_CONTROL_AMBIQ_H_ */
