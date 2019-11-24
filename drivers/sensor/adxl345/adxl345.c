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

static void adxl345_accel_convert(const struct sensor_value *val, unsigned int sample)
{
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

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl345_accel_convert(val, data->sample_number);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl345_accel_convert(val, data->sample_number);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl345_accel_convert(val, data->sample_number);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl345_accel_convert(val++, data->sample_number);
		adxl345_accel_convert(val++, data->sample_number);
		adxl345_accel_convert(val, data->sample_number);
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
