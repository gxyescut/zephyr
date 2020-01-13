/*
 * Copyright (c) 2018 Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <em_cmu.h>
#include <em_i2c.h>
#include <em_gpio.h>
#include <soc.h>
#include <stdio.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_gecko);

#include "i2c-priv.h"

#define DEV_CFG(dev) \
	((struct i2c_gecko_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2c_gecko_data * const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((I2C_TypeDef *)(DEV_CFG(dev))->base)

struct i2c_gecko_config {
	I2C_TypeDef *base;
	CMU_Clock_TypeDef clock;
	I2C_Init_TypeDef i2cInit;
	u32_t bitrate;
	struct soc_gpio_pin pin_sda;
	struct soc_gpio_pin pin_scl;
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	u8_t loc_sda;
	u8_t loc_scl;
#else
	u8_t loc;
#endif
#if defined(CONFIG_I2C_SLAVE)
        void (*irq_config)(void);
#endif
};

struct i2c_gecko_data {
	struct k_sem device_sync_sem;
	u32_t dev_config;
#if defined(CONFIG_I2C_SLAVE)
        struct i2c_slave_config *slave_cfg;
#endif

};

void i2c_gecko_config_pins(struct device *dev,
			   const struct soc_gpio_pin *pin_sda,
			   const struct soc_gpio_pin *pin_scl)
{
	I2C_TypeDef *base = DEV_BASE(dev);
	struct i2c_gecko_config *config = DEV_CFG(dev);

	soc_gpio_configure(pin_scl);
	soc_gpio_configure(pin_sda);

#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	base->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
	base->ROUTELOC0 = (config->loc_sda << _I2C_ROUTELOC0_SDALOC_SHIFT) |
			  (config->loc_scl << _I2C_ROUTELOC0_SCLLOC_SHIFT);
#else
	base->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (config->loc << 8);
#endif
}

static int i2c_gecko_configure(struct device *dev, u32_t dev_config_raw)
{
	I2C_TypeDef *base = DEV_BASE(dev);
	struct i2c_gecko_config *config = DEV_CFG(dev);
	struct i2c_gecko_data *data = DEV_DATA(dev);
	u32_t baudrate;

	if (!(I2C_MODE_MASTER & dev_config_raw)) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		baudrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		baudrate = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	data->dev_config = dev_config_raw;
	config->i2cInit.freq = baudrate;

	I2C_Init(base, &config->i2cInit);

	return 0;
}

static int i2c_gecko_transfer(struct device *dev, struct i2c_msg *msgs,
			      u8_t num_msgs, u16_t addr)
{
	I2C_TypeDef *base = DEV_BASE(dev);
	struct i2c_gecko_data *data = DEV_DATA(dev);
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret = -EIO;
	u32_t timeout = 300000U;

	if (!num_msgs) {
		return 0;
	}

	seq.addr = addr << 1;

	do {
		seq.buf[0].data = msgs->buf;
		seq.buf[0].len	= msgs->len;

		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			seq.flags = I2C_FLAG_READ;
		} else {
			seq.flags = I2C_FLAG_WRITE;
			if (num_msgs > 1) {
				/* Next message */
				msgs++;
				num_msgs--;
				if ((msgs->flags & I2C_MSG_RW_MASK)
				    == I2C_MSG_READ) {
					seq.flags = I2C_FLAG_WRITE_READ;
				} else {
					seq.flags = I2C_FLAG_WRITE_WRITE;
				}
				seq.buf[1].data = msgs->buf;
				seq.buf[1].len	= msgs->len;
			}
		}

		if (data->dev_config & I2C_ADDR_10_BITS) {
			seq.flags |= I2C_FLAG_10BIT_ADDR;
		}

		/* Do a polled transfer */
		ret = I2C_TransferInit(base, &seq);
		while (ret == i2cTransferInProgress && timeout--) {
			ret = I2C_Transfer(base);
		}

		if (ret != i2cTransferDone) {
			goto finish;
		}

		/* Next message */
		msgs++;
		num_msgs--;
	} while (num_msgs);

finish:
	if (ret != i2cTransferDone) {
		ret = -EIO;
	}
	return ret;
}

static int i2c_gecko_init(struct device *dev)
{
	struct i2c_gecko_config *config = DEV_CFG(dev);
	u32_t bitrate_cfg;
	int error;

	CMU_ClockEnable(config->clock, true);

	i2c_gecko_config_pins(dev, &config->pin_sda, &config->pin_scl);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);

	error = i2c_gecko_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (error) {
		return error;
	}

	return 0;
}

#if defined(CONFIG_I2C_SLAVE)
static int i2c_gecko_slave_register(struct device *dev, struct i2c_slave_config *slave_config)
{
        struct i2c_gecko_config *config = DEV_CFG(dev);
	struct i2c_gecko_data *data = DEV_DATA(dev);


        if (!slave_config) {
                return -EINVAL;
        }

        I2C_Enable(config->base, false);
        config->i2cInit.master = false;
        I2C_Init(config->base, &config->i2cInit);

        I2C_SlaveAddressSet(config->base, slave_config->address);
        I2C_SlaveAddressMaskSet(config->base, 0x7F);
        (config->base)->CTRL |= I2C_CTRL_SLAVE;

        I2C_IntClear(config->base, _I2C_IF_MASK);
        I2C_IntEnable(config->base, I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_BUSERR | I2C_IEN_ARBLOST | I2C_IEN_SSTOP);
        config->irq_config();
        data->slave_cfg = slave_config;


        return 0;
}

static struct device DEVICE_NAME_GET(i2c_gecko_0);

ISR_DIRECT_DECLARE(i2c_0_gecko_isr)
{
        struct device *const dev = DEVICE_GET(i2c_gecko_0);
        struct i2c_gecko_data *data = DEV_DATA(dev);
        const struct i2c_slave_callbacks *slave_cb = data->slave_cfg->callbacks;
        static bool write_request;
        uint32_t pending;
        uint32_t rxData;
        static uint8_t val;
        pending = I2C0->IF;

        if(pending & (I2C_IF_BUSERR | I2C_IF_ARBLOST)) {

        } else {
                if(pending & I2C_IF_ADDR) {
                        rxData = I2C0->RXDATA;
                        I2C0->CMD = I2C_CMD_ACK;
                        I2C_IntClear(I2C0, I2C_IF_ADDR | I2C_IF_RXDATAV);

                        if(!(rxData & I2C_MSG_READ)) {
                                write_request = true;
                        }
                        printf("Addr got 0x%x \n", rxData);

                } else if(pending & I2C_IF_RXDATAV) {
                        rxData = I2C0->RXDATA;
                        I2C0->CMD = I2C_CMD_ACK;
                        I2C_IntClear(I2C0, I2C_IF_RXDATAV);
                        val = rxData;

                        printf("Val 0x%x \n", rxData);
                } else if(pending & I2C_IF_SSTOP) {
                        I2C_IntClear(I2C0, I2C_IF_SSTOP);
                        if (write_request) {
                                printf("Got write cmd \n");
                                slave_cb->write_received(data->slave_cfg, val);
                        }
                }
        }
}
#endif

static const struct i2c_driver_api i2c_gecko_driver_api = {
	.configure = i2c_gecko_configure,
	.transfer = i2c_gecko_transfer,
#if defined(CONFIG_I2C_SLAVE)
        .slave_register = i2c_gecko_slave_register,
#endif
};

#if defined(CONFIG_I2C_SLAVE)
static void i2c_gecko_config_func_0(struct device *dev)
{
        IRQ_DIRECT_CONNECT(DT_INST_0_SILABS_GECKO_I2C_IRQ_0,
                    DT_INST_0_SILABS_GECKO_I2C_IRQ_0_PRIORITY,
                    i2c_0_gecko_isr, 0);

        irq_enable(DT_INST_0_SILABS_GECKO_I2C_IRQ_0);
}
#endif

#ifdef DT_INST_0_SILABS_GECKO_I2C

#define PIN_I2C_0_SDA {DT_INST_0_SILABS_GECKO_I2C_LOCATION_SDA_1, \
		DT_INST_0_SILABS_GECKO_I2C_LOCATION_SDA_2, gpioModeWiredAnd, 1}
#define PIN_I2C_0_SCL {DT_INST_0_SILABS_GECKO_I2C_LOCATION_SCL_1, \
		DT_INST_0_SILABS_GECKO_I2C_LOCATION_SCL_2, gpioModeWiredAnd, 1}

static struct i2c_gecko_config i2c_gecko_config_0 = {
	.base = (I2C_TypeDef *)DT_INST_0_SILABS_GECKO_I2C_BASE_ADDRESS,
	.clock = cmuClock_I2C0,
	.i2cInit = I2C_INIT_DEFAULT,
	.pin_sda = PIN_I2C_0_SDA,
	.pin_scl = PIN_I2C_0_SCL,
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	.loc_sda = DT_INST_0_SILABS_GECKO_I2C_LOCATION_SDA_0,
	.loc_scl = DT_INST_0_SILABS_GECKO_I2C_LOCATION_SCL_0,
#else
#if DT_INST_0_SILABS_GECKO_I2C_LOCATION_SDA_0 \
	!= DT_INST_0_SILABS_GECKO_I2C_LOCATION_SCL_0
#error I2C_0 DTS location-* properties must have identical value
#endif
	.loc = DT_INST_0_SILABS_GECKO_I2C_LOCATION_SCL_0,
#endif
	.bitrate = DT_INST_0_SILABS_GECKO_I2C_CLOCK_FREQUENCY,
#if defined(CONFIG_I2C_SLAVE)
        .irq_config = i2c_gecko_config_func_0,
#endif
};

static struct i2c_gecko_data i2c_gecko_data_0;

DEVICE_AND_API_INIT(i2c_gecko_0, DT_INST_0_SILABS_GECKO_I2C_LABEL,
		    &i2c_gecko_init, &i2c_gecko_data_0, &i2c_gecko_config_0,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_gecko_driver_api);
#endif /* DT_INST_0_SILABS_GECKO_I2C */

#ifdef DT_INST_1_SILABS_GECKO_I2C

#define PIN_I2C_1_SDA {DT_INST_1_SILABS_GECKO_I2C_LOCATION_SDA_1, \
		DT_INST_1_SILABS_GECKO_I2C_LOCATION_SDA_2, gpioModeWiredAnd, 1}
#define PIN_I2C_1_SCL {DT_INST_1_SILABS_GECKO_I2C_LOCATION_SCL_1, \
		DT_INST_1_SILABS_GECKO_I2C_LOCATION_SCL_2, gpioModeWiredAnd, 1}

static struct i2c_gecko_config i2c_gecko_config_1 = {
	.base = (I2C_TypeDef *)DT_INST_1_SILABS_GECKO_I2C_BASE_ADDRESS,
	.clock = cmuClock_I2C1,
	.i2cInit = I2C_INIT_DEFAULT,
	.pin_sda = PIN_I2C_1_SDA,
	.pin_scl = PIN_I2C_1_SCL,
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	.loc_sda = DT_INST_1_SILABS_GECKO_I2C_LOCATION_SDA_0,
	.loc_scl = DT_INST_1_SILABS_GECKO_I2C_LOCATION_SCL_0,
#else
#if DT_INST_1_SILABS_GECKO_I2C_LOCATION_SDA_0 \
	!= DT_INST_1_SILABS_GECKO_I2C_LOCATION_SCL_0
#error I2C_1 DTS location-* properties must have identical value
#endif
	.loc = DT_INST_1_SILABS_GECKO_I2C_LOCATION_SCL_0,
#endif
	.bitrate = DT_INST_1_SILABS_GECKO_I2C_CLOCK_FREQUENCY,
};

static struct i2c_gecko_data i2c_gecko_data_1;

DEVICE_AND_API_INIT(i2c_gecko_1, DT_INST_1_SILABS_GECKO_I2C_LABEL,
		    &i2c_gecko_init, &i2c_gecko_data_1, &i2c_gecko_config_1,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2c_gecko_driver_api);
#endif /* DT_INST_1_SILABS_GECKO_I2C */
