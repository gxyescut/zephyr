/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_fastvdma

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <drivers/dma.h>
#include <soc.h>

#include "dma_fastvdma_litex.h"

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_fastvdma);

#define DEV_CFG(dev) \
    ((struct fastvdma_dev_cfg *)(dev)->config)

static void fastvdma_litex_irq_handler(const struct device *dev)
{
    const struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);

    uint8_t ev_int_status =
        litex_read(dev_cfg->ev_pending_addr, REG_SIZE);
    uint8_t int_status =
        litex_read(dev_cfg->irq_status_addr, REG_SIZE);

    /* clear events */
    litex_write(dev_cfg->irq_status_addr, REG_SIZE,
            int_status);
    litex_write(dev_cfg->ev_pending_addr, REG_SIZE,
            ev_int_status);

    /* interrupts are triggered only on transfer complete */
    if (ev_int_status & FASTVDMA_IRQ_WR_TRIG) {
        if(dev_cfg->cb) {
            dev_cfg->cb(dev, dev_cfg->user_data, 0, 0);
        }
    }
}

static int fastvdma_litex_configure(const struct device *dev,
            uint32_t channel, struct dma_config *cfg)
{
    struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);
    const struct dma_block_config *block_cfg = cfg->head_block;

    if (channel != 0) {
        LOG_ERR("FastVDMA does not provide support for multiple channels");
        return -EINVAL;
    }

    /**
     * Not checking source_data_size, dest_data_size and others. These
     * parameters are set in FastVDMA scala settings before integration
     * with FPGA design thus can't be modified by software.
     *
     * Channel direction is supported by driver but it must match gateware
     * implementation where FastVDMA streams are connected to peripherals
     * or memory, e.g. if FastVDMA is connected between memory and peripheral
     * there must be set MEMORY_TO_PERIPHERAL, other configurations will
     * not work properly.
     */

    switch(cfg->channel_direction) {
        case MEMORY_TO_MEMORY:
            litex_write(dev_cfg->read_base_addr, REG_SIZE,
                        block_cfg->source_address);
            litex_write(dev_cfg->write_base_addr, REG_SIZE,
                        block_cfg->dest_address);
            break;

        case MEMORY_TO_PERIPHERAL:
            litex_write(dev_cfg->read_base_addr, REG_SIZE,
                        block_cfg->source_address);
            litex_write(dev_cfg->write_base_addr, REG_SIZE, 0);
            break;

        case PERIPHERAL_TO_MEMORY:
            litex_write(dev_cfg->read_base_addr, REG_SIZE, 0);
            litex_write(dev_cfg->write_base_addr, REG_SIZE,
                        block_cfg->dest_address);
            break;

        case PERIPHERAL_TO_PERIPHERAL:
            litex_write(dev_cfg->read_base_addr, REG_SIZE, 0);
            litex_write(dev_cfg->write_base_addr, REG_SIZE, 0);
            break;

        default:
            LOG_ERR("channel_direction %d is not supported",
                    cfg->channel_direction);
            return -EINVAL;
    }

    dev_cfg->dir = cfg->channel_direction;

    uint8_t control = litex_read(dev_cfg->control_addr, REG_SIZE);

    /* enable/disable read loop mode */
    if (block_cfg->source_gather_en) {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control |
                    FASTVDMA_CTRL_RD_LOOP);
    } else {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control &
                    ~FASTVDMA_CTRL_RD_LOOP);
    }

    /* enable/disable write loop mode */
    if (block_cfg->dest_scatter_en) {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control |
                    FASTVDMA_CTRL_WR_LOOP);
    } else {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control &
                    ~FASTVDMA_CTRL_WR_LOOP);
    }

    /* disable reader sync */
    if (cfg->source_handshake) {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control | FASTVDMA_CTRL_RD_SYNC);
    }
    if (cfg->dest_handshake) {
        control = litex_read(dev_cfg->control_addr, REG_SIZE);
        litex_write(dev_cfg->control_addr, REG_SIZE, control | FASTVDMA_CTRL_WR_SYNC);
    }

    /**
     * source_gather_count should be equal to dest_scatter_count.
     * FastVDMA requires length of each line and a number of lines (count)
     * so line length is extracted from block_size.
     */
    if (block_cfg->source_gather_count != block_cfg->dest_scatter_count) {
        LOG_ERR("Source gather count should be equal to destination scatter count");
        return -EINVAL;
    }

    int length = block_cfg->block_size / block_cfg->dest_scatter_count;
    int count = block_cfg->dest_scatter_count;

    /**
     * Set FastVDMA length and count, these are always the same for read and
     * write.
     */
    litex_write(dev_cfg->read_length_addr, REG_SIZE, length);
    litex_write(dev_cfg->write_length_addr, REG_SIZE, length);
    litex_write(dev_cfg->read_count_addr, REG_SIZE, count);
    litex_write(dev_cfg->write_count_addr, REG_SIZE, count);

    /* Set gap between lines */
    litex_write(dev_cfg->write_stride_addr, REG_SIZE,
                block_cfg->dest_scatter_interval);
    litex_write(dev_cfg->read_stride_addr, REG_SIZE,
                block_cfg->source_gather_interval);

    /* Set callback */
    dev_cfg->cb = cfg->dma_callback;

    /* enable interrupt at complete */
    litex_write(dev_cfg->ev_enable_addr, REG_SIZE, EV_ENABLE);
    litex_write(dev_cfg->irq_mask_addr, REG_SIZE, FASTVDMA_IRQ_WR_EN);

    return 0;
}

static int fastvdma_litex_transfer_start(const struct device *dev, uint32_t channel)
{
    const struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);

    uint32_t control_reg = litex_read(dev_cfg->control_addr, REG_SIZE);

    litex_write(dev_cfg->control_addr, REG_SIZE, control_reg | FASTVDMA_CTRL_RD_EN |
                FASTVDMA_CTRL_WR_EN);

    return 0;
}

static int fastvdma_litex_transfer_stop(const struct device *dev, uint32_t channel)
{
    const struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);

    uint32_t control_reg = litex_read(dev_cfg->control_addr, REG_SIZE);

    litex_write(dev_cfg->control_addr, REG_SIZE, control_reg & ~FASTVDMA_CTRL_RD_EN &
                ~FASTVDMA_CTRL_WR_EN);

    return 0;
}

static int fastvdma_litex_get_status(const struct device *dev, uint32_t channel,
              struct dma_status *status)
{
    const struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);
    const struct dma_block_config *block_cfg = block_cfg;

    uint32_t status_reg = litex_read(dev_cfg->status_addr, REG_SIZE);

    /**
     * FastVDMA direction is dependent on FPGA design so driver won't know
     * its settings. There is also no register containing pending length
     * so we only set status->busy.
     */
    if (status_reg) {
        status->busy = true;
    } else {
        status->busy = false;
    }

    status->dir = dev_cfg->dir;

    return 0;
}

static int fastvdma_litex_init(const struct device *dev)
{
    const struct fastvdma_dev_cfg *dev_cfg = DEV_CFG(dev);

    dev_cfg->irq_config();

    return 0;
}

static const struct dma_driver_api fastvdma_litex_driver_api = {
    .config = fastvdma_litex_configure,
    .start = fastvdma_litex_transfer_start,
    .stop = fastvdma_litex_transfer_stop,
    .get_status = fastvdma_litex_get_status,
};

#define FASTVDMA_INIT(n)    \
    static void fastvdma_litex_irq_init_##n(void);  \
            \
    static const struct fastvdma_dev_cfg dev_cfg_##n = { \
        .control_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, control), \
        .status_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, status), \
        .irq_mask_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, irq_mask), \
        .irq_status_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, irq_status), \
        .read_base_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, read_base), \
        .read_length_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, read_length), \
        .read_count_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, read_count), \
        .read_stride_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, read_stride), \
        .write_base_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, write_base), \
        .write_length_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, write_length), \
        .write_count_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, write_count), \
        .write_stride_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, write_stride), \
        .ev_status_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, ev_status), \
        .ev_pending_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, ev_pending), \
        .ev_enable_addr = \
        (volatile uint32_t *) DT_INST_REG_ADDR_BY_NAME(n, ev_enable), \
        .irq_config = fastvdma_litex_irq_init_##n \
    };  \
\
    DEVICE_DT_INST_DEFINE(n,\
        &fastvdma_litex_init,\
        NULL,\
        NULL, &dev_cfg_##n, POST_KERNEL,\
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
        &fastvdma_litex_driver_api);\
\
    static void fastvdma_litex_irq_init_##n(void)\
    {\
        IRQ_CONNECT(DT_INST_IRQN(n),\
                DT_INST_IRQ(n, priority),\
                fastvdma_litex_irq_handler,\
                DEVICE_DT_INST_GET(n), 0);\
        irq_enable(DT_INST_IRQN(n));\
    }

DT_INST_FOREACH_STATUS_OKAY(FASTVDMA_INIT)
