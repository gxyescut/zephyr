/*
 * Copyright (c) 2019 Antmicro <www.antmicro.com>
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

#define ADXL_I2C_ADDR 		(0x1d)
#define ADXL_DEVICE_ID_REG 	(0x00)
#define ADXL_RATE_REG 		(0x2c)
#define ADXL_POWER_CTL_REG 	(0x2d)
#define ADXL_DATA_FORMAT_REG	(0x31)
#define ADXL_X_AXIS_DATA_0_REG 	(0x32)
#define ADXL_FIFO_CTL_REG 	(0x38)
#define ADXL_FIFO_STATUS_REG 	(0x39)

#define ADXL_RANGE_2G		(0x0)
#define ADXL_RANGE_4G		(0x1)
#define ADXL_RANGE_8G		(0x2)
#define ADXL_RANGE_16G		(0x3)
#define ADXL_RATE_25HZ 		(0x8)
#define ADXL_ENABLE_MEASURE_BIT (1 << 3)
#define ADXL_FIFO_STREAM_MODE 	(1 << 7)
#define ADXL_FIFO_COUNT_MASK 	(0x3f)
#define ADXL_COMPLEMENT 	(0xfc00)

struct adxl345_dev_data {
	unsigned int sample_number;
	struct device *i2c_master;
	u8_t i2c_addr;
};

struct adxl345_sample {
	s16_t x;
	s16_t y;
	s16_t z;
};

struct adxl345_dev_config {
	struct adxl345_dev_data *data;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ADX345_ADX345_H_ */
