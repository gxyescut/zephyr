/*
 * Copyright (c) 2019 Antmicro <www.antmicro.com>
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

static s16_t bufx[32];
static s16_t bufy[32];
static s16_t bufz[32];

static int adxl345_read_reg(struct device *dev, u8_t reg)
{
	struct adxl345_dev_data *data = dev->driver_data;
	u8_t reg_val;
	int rc = i2c_write_read(data->i2c_master,
			data->i2c_addr,
			&reg, 1,
			&reg_val, 1);

	if(rc < 0) {
		printk("Register read failed with rc=%d\n", rc);
		return rc;
	}
	return reg_val;
}

static int adxl345_write_reg(struct device *dev, u8_t reg, u8_t val)
{
	struct adxl345_dev_data *data = dev->driver_data;
	int rc;
	u8_t write_buf[2];

	write_buf[0] = reg;
	write_buf[1] = val;
	rc = i2c_write(data->i2c_master,
			write_buf, 2,
			data->i2c_addr);
	if(rc < 0) {
		printk("Writing register failed\n");
		return rc;
	}
	return 0;
}

static int adxl345_read_sample(struct device *dev, struct adxl345_sample *sample)
{
	struct adxl345_dev_data *data = dev->driver_data;
	s16_t raw_x, raw_y, raw_z;
	u8_t axis_data[6];

        int rc = i2c_burst_read( data->i2c_master,
				 data->i2c_addr,
				 ADXL_X_AXIS_DATA_0_REG,
				 axis_data,
				 6);
	if (rc < 0) {
		printk("Samples read failed with rc=%d\n", rc);
		return rc;
	}

	raw_x = axis_data[0] | (axis_data[1] << 8);
	raw_y = axis_data[2] | (axis_data[3] << 8);
	raw_z = axis_data[4] | (axis_data[5] << 8);

	if(raw_x & (1<<9)) raw_x |= ADXL_COMPLEMENT;
	if(raw_y & (1<<9)) raw_y |= ADXL_COMPLEMENT;
	if(raw_z & (1<<9)) raw_z |= ADXL_COMPLEMENT;

	sample->x = (raw_x * 1000) / 32;
	sample->y = (raw_y * 1000) / 32;
	sample->z = (raw_z * 1000) / 32;
	return 0;
}

static void adxl345_accel_convert(struct sensor_value *val, unsigned int sample)
{
	val->val1 = sample;
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
	struct adxl345_sample sample;
	int samples_count;
	int rc;

	data->sample_number = 0;
	rc = adxl345_read_reg(dev, ADXL_FIFO_STATUS_REG);
	if(rc < 0) {
		printk("Failed to read FIFO status rc = %d\n", rc);
		return rc;
	}
	samples_count = rc;
	for(int s = 0; s < samples_count; s ++) {
		rc = adxl345_read_sample(dev, &sample);
		if(rc < 0) {
			printk("Failed to fetch sample rc=%d\n");
			return rc;
		}
		bufx[s] = sample.x;
		bufy[s] = sample.y;
		bufz[s] = sample.z;
	}
	return samples_count;
}

static int adxl345_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adxl345_dev_data *data = dev->driver_data;

	if (data->sample_number > 32) data->sample_number = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl345_accel_convert(val, bufx[data->sample_number]);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl345_accel_convert(val, bufy[data->sample_number]);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl345_accel_convert(val, bufz[data->sample_number]);
		data->sample_number++;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl345_accel_convert(val++, bufx[data->sample_number]);
		adxl345_accel_convert(val++, bufy[data->sample_number]);
		adxl345_accel_convert(val, bufz[data->sample_number]);
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
	u8_t id;
	int rc;
	struct adxl345_dev_data *data = dev->driver_data;

	data->sample_number = 0;
	data->i2c_master = device_get_binding("i2c0");
	data->i2c_addr = ADXL_I2C_ADDR;

	if(!data->i2c_master) {
		printk("Failed to get I2C master\n");
		return -EINVAL;
	}

	id = adxl345_read_reg(dev, ADXL_DEVICE_ID_REG);
	if (id < 0)
		printk("Read ID failed\n");
	else
		printk("Got id: 0x%x\n", id);

	rc = adxl345_write_reg(dev, ADXL_FIFO_CTL_REG,
			ADXL_FIFO_STREAM_MODE);
	if(rc < 0) printk("FIFO enable failed\n");

	rc = adxl345_write_reg(dev, ADXL_DATA_FORMAT_REG,
			ADXL_RANGE_16G);
	if(rc < 0) printk("FIFO enable failed\n");

	rc = adxl345_write_reg(dev, ADXL_RATE_REG,
			ADXL_RATE_25HZ);
	if(rc < 0) printk("Rate setting failed\n");

	rc = adxl345_write_reg(dev, ADXL_POWER_CTL_REG,
		       	ADXL_ENABLE_MEASURE_BIT);
	if(rc < 0) printk("Enable measure failed\n");

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
