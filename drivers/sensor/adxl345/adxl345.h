/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_
#define ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/util.h>

struct adxl345_dev_data {
	unsigned int sample_number;
};

struct adxl345_dev_config {
	struct adxl345_dev_data *data;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_ */
