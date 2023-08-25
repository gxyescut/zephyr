/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define IRQ_NODE  DT_ALIAS(bleirq)
#define SPI_NODE  DT_NODELABEL(spi2)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(IRQ_NODE, gpios);

static struct gpio_callback	gpio_cb;

bool g_spi_rx_sem = false;
static void irq_isr(const struct device *unused1,
		       struct gpio_callback *unused2,
		       uint32_t unused3)
{
	printk(".\n");
	g_spi_rx_sem = true;
}

void spi_rx_process(void)
{
	const struct device *const dev = DEVICE_DT_GET(SPI_NODE);
	struct spi_config config = {0};

	config.frequency = 1000000;
	config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8);

	uint8_t command[1] = { 0x04 };
	uint8_t response[2];

	struct spi_buf tx_buf[1] = {
		{.buf = command, .len = 1},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = response, .len = 2},
	};

	struct spi_buf_set tx_set = { .buffers = tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = rx_buf, .count = 1 };
	uint16_t length = 0;
	int ret;

	ret = spi_transceive(dev, &config, &tx_set, &rx_set);
	length = (uint16_t)((response[0]) | response[1] << 8);
	
	if (length == 0) {
		return;
	}

    uint8_t data[32];
	struct spi_buf rx_buf_2[1] = {
		{.buf = data, .len = length},
	};
	struct spi_buf_set rx_set_2 = { .buffers = rx_buf_2, .count = 1 };
	ret = spi_transceive(dev, &config, NULL, &rx_set_2);

    printk("RX data: ");
	for (int i = 0; i < length; i++)
	{
		printk("%02x ", data[i]);
	}
	printk("\n");
}

void test_spi_send(const struct device *dev)
{
	int ret;
	struct spi_config config = {0};

	config.frequency = 1000000;
	config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8);

	uint8_t command[1] = { 0x80 };
	uint8_t response[2];

	struct spi_buf tx_buf[1] = {
		{.buf = command, .len = 1},
	};
	struct spi_buf rx_buf[1] = {
		{.buf = response, .len = 2},
	};

	struct spi_buf_set tx_set = { .buffers = tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = rx_buf, .count = 1 };
	uint8_t count = 0;

    do {
	    ret = spi_transceive(dev, &config, &tx_set, &rx_set);
	    if ((response[0] != 0x68) || (response[1] != 0xA8) || ret)
		{
			count ++;
			if (count > 50)
			    break;
			k_sleep(K_MSEC(2));
		}
		else
		{
			break;
		}
	} while (1);

	if (count > 50) {
		printk("SPI send fail\n");
		return;
	}

	uint8_t txdata[] = { 0x01, 0x03, 0x0C, 0x00};

    printk("TX data: ");
	for (int i = 0; i < sizeof(txdata); i++)
	{
		printk("%02X ", txdata[i]);
	}
	printk("\n");

	struct spi_buf tx_buf_2[1] = {
		{.buf = txdata, .len = sizeof(txdata)},
	};

	struct spi_buf_set tx_set_2 = { .buffers = tx_buf_2, .count = 1 };
	ret = spi_transceive(dev, &config, &tx_set_2, NULL);

	if (ret) {
		printk("SPI send fail, ret: %d\n", ret);
	}

	while (g_spi_rx_sem == false);
	spi_rx_process();
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&irq)) {
		printk("Device not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&irq, GPIO_INPUT);
	if (ret < 0) {
		printk("IRQ PIN Config fail\n");
		return 0;
	}

	gpio_init_callback(&gpio_cb, irq_isr, BIT(irq.pin));
	ret = gpio_add_callback(irq.port, &gpio_cb);
	if (ret) {
		printk("Add callback fail\n");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("Interrupt config fail\n");
		return ret;
	}

    const struct device *const dev = DEVICE_DT_GET(SPI_NODE);

	if (!device_is_ready(dev)) {
		printk("Ddevice not ready");
		return -ENODEV;
	}

	test_spi_send(dev);

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
