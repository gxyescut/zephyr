/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT	quicklogic_eos_s3_spi

#include <sys/sys_io.h>
#include <device.h>
#include <drivers/spi.h>
#include <soc.h>
#include <soc_pinmap.h>
#include <eoss3_hal_spi.h>

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_eos_s3);

#include "spi_context.h"
#include "spi_eos_s3.h"

struct spi_eos_s3_data {
	struct spi_context ctx;
	SPI_Mode mode;
};

struct spi_eos_s3_config {
	SPI_TypeDef *base;
};

static int spi_eos_s3_configure(SPI_TypeDef *base,
				const struct spi_config *config,
				SPI_Mode mode)
{
	uint32_t ctrlr0;

	if (SPI_WORD_SIZE_GET(config->operation) != SPI_WORD_SIZE) {
		LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
		return -ENOTSUP;
	}

	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		LOG_ERR("CS active high not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only supports single mode");
		return -ENOTSUP;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	/* Disable SPI controller */
	base->SSIENR = SSIENR_SSI_DISABLE;

	/* Disable Slave Select */
	base->SER = 0;

	/* Find divisor to setup clock rate */
	uint32_t clock_div = SPI_CLK / config->frequency;

	/* Divisor must be an even number */
	if (clock_div % 2) {
		clock_div++;
	}

	/* This driver doesn't work correctly with clock rate higher than 2MHz.
	 * 10MHz / 6 ~= 1.6 MHz
	 */
	if (clock_div < 6) {
		return -EINVAL;
	}

	base->BAUDR = clock_div;

	ctrlr0 = SPI_DATASIZE_8BIT;
	ctrlr0 |= (config->operation & SPI_MODE_CPOL) ? SPI_POLARITY_HIGH
		: SPI_POLARITY_LOW;
	ctrlr0 |= (config->operation & SPI_MODE_CPHA) ? SPI_PHASE_1EDGE
		: SPI_PHASE_2EDGE;

	switch (mode) {
	case TXRX_MODE:
		ctrlr0 |= CTRLR0_TMOD_TX_RX;
		break;
	case TX_MODE:
		ctrlr0 |= CTRLR0_TMOD_TX;
		break;
	case RX_MODE:
		ctrlr0 |= CTRLR0_TMOD_RX;
		/* Set number of data frames to read */
		break;
	case EEPROM_READ_MODE:
		/* Not supported */
		break;
	}

	base->CTRLR0 = ctrlr0;

	/* Choose Slave Select line */
	base->SER = BIT(config->slave);

	/* Enable SPI controller */
	base->SSIENR = SSIENR_SSI_EN;

	return 0;
}

static void spi_eos_s3_set_nframes(SPI_TypeDef *base, uint16_t nfrm)
{
	/* Disable SPI controller */
	base->SSIENR = SSIENR_SSI_DISABLE;

	/* Program number of frames to receive */
	uint16_t ctrlr1 = nfrm - 1;

	base->CTRLR1 = ctrlr1;

	/* Enable SPI controller */
	base->SSIENR = SSIENR_SSI_EN;
}

/* Finish any ongoing writes and drop any remaining read data */
static void spi_eos_s3_finish(SPI_TypeDef *base)
{
	while (!(base->SR & SR_TFE)) {
	}

	while (base->SR & SR_RFNE) {
		(void)base->DR0;
	}
}

static void spi_eos_s3_tx(SPI_TypeDef *base,
			  uint8_t frame)
{
	uint8_t c = 0;
	/* Wait for not full TX FIFO */
	while (!(base->SR & SR_TFNF)) {
	}

	c = frame;
	base->DR0 = c;
}

static int spi_eos_s3_rx(SPI_TypeDef *base)
{
	if (!(base->SR & SR_RFNE)) {
		return NO_RX_DATA;
	}

	return base->DR0;
}

static int spi_eos_s3_init(const struct device *dev)
{
	return 0;
}

static inline int spi_eos_s3_next_tx(struct spi_eos_s3_data *data)
{
	int tx_frame;

	if (spi_context_tx_buf_on(&data->ctx)) {
		tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
	} else {
		tx_frame = NO_TX_DATA;
	}

	return tx_frame;
}

static int spi_eos_s3_shift_frames(SPI_TypeDef *base,
				   struct spi_eos_s3_data *data)
{
	int tx_frame;
	int rx_frame;

	/* Do not send data when continuous RX mode is selected */
	if (data->mode != RX_MODE) {
		tx_frame = spi_eos_s3_next_tx(data);
		/* Do not send redundant data */
		if (tx_frame != NO_TX_DATA) {
			spi_eos_s3_tx(base, (uint8_t)tx_frame);
			spi_context_update_tx(&data->ctx, 1, 1);
		}
	}

	rx_frame = spi_eos_s3_rx(base);

	if (rx_frame == NO_RX_DATA) {
		return 0;
	}

	if (spi_context_rx_buf_on(&data->ctx)) {
		UNALIGNED_PUT((uint8_t)rx_frame, (uint8_t *)data->ctx.rx_buf);
	}
	spi_context_update_rx(&data->ctx, 1, 1);

	return 0;
}

static bool spi_eos_s3_transfer_ongoing(struct spi_eos_s3_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static void spi_eos_s3_xfer(const struct device *dev)
{
	int ret;
	struct spi_eos_s3_data *data = DEV_DATA(dev);
	struct spi_context *ctx = &data->ctx;
	const struct spi_eos_s3_config *cfg = dev->config;
	SPI_TypeDef *base = cfg->base;

	if (data->mode == RX_MODE) {
		/* Set number of bytes for continuous read */
		spi_eos_s3_set_nframes(base, ctx->rx_len);
		/* Send dummy byte*/
		spi_eos_s3_tx(base, 0);
	}

	do {
		ret = spi_eos_s3_shift_frames(base, data);
	} while (!ret && spi_eos_s3_transfer_ongoing(data));

	spi_eos_s3_finish(base);
	spi_context_complete(ctx, 0);
}

/* API Functions */
static int spi_eos_s3_transceive(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	const struct spi_eos_s3_config *cfg = dev->config;
	struct spi_eos_s3_data *data = DEV_DATA(dev);
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	SPI_TypeDef *base = cfg->base;
	int ret = 0;

	data->ctx.config = config;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	/* Find out what mode is ordered */
	if (tx_count != 0 && rx_count != 0) {
		/* TXRX */
		data->mode = TXRX_MODE;
	} else if (tx_count != 0) {
		/* TX */
		data->mode = TX_MODE;
	} else {
		/* RX */
		data->mode = RX_MODE;
	}
	/* EEPROM READ not supported */

	ret = spi_eos_s3_configure(base, config, data->mode);

	if (ret) {
		spi_context_release(&data->ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
	spi_eos_s3_xfer(dev);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_eos_s3_transceive_async(const struct device *dev,
				       const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs,
				       struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_eos_s3_release(const struct device *dev,
			      const struct spi_config *config)
{
	const struct spi_eos_s3_config *eos_s3_cfg = dev->config;

	if (!(eos_s3_cfg->base->SR & SR_BUSY)) {
		return -EBUSY;
	}

	return 0;
}

/* Device Instantiation */
static struct spi_driver_api spi_eos_s3_driver_api = {
	.transceive = spi_eos_s3_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_eos_s3_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = spi_eos_s3_release,
};

static const struct spi_eos_s3_config spi_eos_s3_config = {
	.base = (SPI_TypeDef *)DT_INST_REG_ADDR(0),
};

static struct spi_eos_s3_data spi_eos_s3_dev_data = {
	SPI_CONTEXT_INIT_LOCK(spi_eos_s3_dev_data, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_eos_s3_dev_data, ctx),
};

DEVICE_AND_API_INIT(spi_eos_s3,
		    DT_INST_LABEL(0),
		    &spi_eos_s3_init,
		    &spi_eos_s3_dev_data,
		    &spi_eos_s3_config,
		    POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &spi_eos_s3_driver_api);
