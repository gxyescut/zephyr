/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <stdlib.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "adxl345.h"

LOG_MODULE_REGISTER(ADXL345, CONFIG_SENSOR_LOG_LEVEL);

const int accel_data[] = {
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    -665, 228,  827,  -680, 339,  716,
    -680, 564,  812,  -679, 552,  818,  -665, 528,  751,
    -658, 432,  618,  -655, 445,  592,  -667, 484,  556,
    -684, 590,  510,  -674, 672,  475,  -660, 786,  390,
    -562, 1124, 128,  -526, 1140, 111,  -486, 1044, 33,
    -416, 652,  -134, -390, 534,  -143, -365, 381,  -117,
    -314, 60,   94,   -322, 7,    190,  -338, -95,  342,
    -360, -106, 842,  -351, -41,  965,  -352, 12,   960,
    -366, 42,   1124, -322, 56,   1178, -312, 15,   1338,
    -254, 10,   1532, -241, 5,    1590, -227, 60,   1565,
    -204, 282,  1560, -180, 262,  1524, -138, 385,  1522,
    -84,  596,  1626, -55,  639,  1604, -19,  771,  1511,
    16,   932,  1132, 15,   924,  1013, 1,    849,  812,
    -88,  628,  500,  -114, 609,  463,  -155, 559,  382,
    -234, 420,  278,  -254, 390,  272,  -327, 200,  336,
    -558, -556, 630,  -640, -607, 740,  -706, -430, 868,
    -778, 42,   1042, -763, 84,   973,  -735, 185,  931,
    -682, 252,  766,  -673, 230,  757,  -671, 218,  757,
    -656, 222,  714,  -659, 238,  746,  -640, 276,  731,
    -634, 214,  754,  -637, 207,  735,  -637, 194,  742,
    -634, 248,  716,  -631, 265,  697,  -628, 252,  797,
    -592, 204,  816,  -618, 218,  812,  -633, 231,  828,
    -640, 222,  736,  -634, 221,  787,
};

static void adxl345_accel_convert(struct sensor_value *val, unsigned int sample)
{
	val->val1 = accel_data[sample];
	val->val2 = 0;
}

static int adxl345_attr_set(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	return 0;
}

static int adxl345_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct adxl345_dev_data *data = dev->driver_data;
	return 0;
}

static int adxl345_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adxl345_dev_data *data = dev->driver_data;

	if (data->sample_number == 3*128) data->sample_number = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl345_accel_convert(val, data->sample_number);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl345_accel_convert(val, data->sample_number);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl345_accel_convert(val, data->sample_number);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl345_accel_convert(val++, data->sample_number);
		data->sample_number++;
		adxl345_accel_convert(val++, data->sample_number);
		data->sample_number++;
		adxl345_accel_convert(val, data->sample_number);
		data->sample_number++;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api adxl345_api_funcs = {
	.attr_set     = adxl345_attr_set,
	.sample_fetch = adxl345_sample_fetch,
	.channel_get  = adxl345_channel_get,
};

static int adxl345_probe(struct device *dev)
{
	struct adxl345_dev_data *data = dev->driver_data;
	data->sample_number = 0;
	return 0;
}

static int adxl345_init(struct device *dev)
{
	if (adxl345_probe(dev) < 0) {
		return -ENODEV;
	}

	return 0;
}

static struct adxl345_dev_data adxl345_data;

static const struct adxl345_dev_config adxl345_config = {
	.data = &adxl345_data
};

DEVICE_AND_API_INIT(adxl345, "accel-0", adxl345_init,
		    &adxl345_data, &adxl345_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &adxl345_api_funcs);
