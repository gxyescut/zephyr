/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spi_flash_litex.h"
#include <string.h>

static bool write_protection = 1;

static int wait_while_spi_busy(void)
{
	u32_t start = k_cycle_get_32();

	while (litex_read8(COMMAND_QUEUED)) {
		if (k_cycle_get_32() - start > TIMEOUT_SPI)
			return 1;
	}
	return 0;
}

static int run_command(int reg)
{
	int done;

	litex_write8(0x01, reg);
	done = !wait_while_spi_busy();
	litex_write8(0x00, reg);

	if (!done)
		return 1;

	return 0;
}

static int wait_while_flash_busy(void)
{
	u32_t start = k_cycle_get_32();

	do {
		if (run_command(READ_STATUS_CMD_CSR))
			return 1;

		if (k_cycle_get_32() - start > TIMEOUT_FLASH)
			return 1;

	} while (litex_read8(STATUS_CSR) & WIP);
	return 0;
}

static int spi_flash_litex_init(struct device *dev)
{
	if (wait_while_spi_busy())
		return 1;
	/* Setting EN_QUAD bit sets flash to quad mode and tries to read ID */
	/* register via QSPI then saves output to READ_ID_REG */
	litex_write8(0x01, EN_QUAD);

	if (wait_while_spi_busy())
		return 1;

	if (litex_read8(READ_ID_REG) != MANUFACTURER_ID)
		return 1;

	return 0;
}

static int spi_flash_litex_write_protection(struct device *dev, bool enable)
{
	write_protection = enable;
	return 0;
}

static int prepare_write(struct device *dev)
{
	if (write_protection)
		return 1;

	if (wait_while_flash_busy())
		return 1;

	if (run_command(WRITE_ENABLE_CSR))
		return 1;

	if (run_command(READ_STATUS_CMD_CSR))
		return 1;

	if (litex_read8(STATUS_CSR) & WEN_SET)
		return 0;

	return 1;
}

static int spi_flash_litex_write(struct device *dev, off_t addr,
			const void *data,
			size_t data_size)
{
	u32_t buf;
	u8_t flash_offset;
	u8_t miss;
	u8_t min;
	u8_t *buf8 = (u8_t *)&buf;
	u8_t *data8 = (u8_t *)data;

	if ((addr + data_size) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	while (data_size > 0) {
		buf = 0xFFFFFFFF;
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;

		if (prepare_write(dev))
			return 1;

		min = MIN(data_size, miss);
		memcpy(buf8 + flash_offset, data8, min);
		sys_write32(buf, MEMORY_MAP_BASE_ADDR + addr - flash_offset);
		data8 += min;
		addr += min;
		data_size -= min;
	}
	return 0;
}

static int spi_flash_litex_read(struct device *dev, off_t addr,
			void *data,
			size_t data_size)
{
	u32_t buf;
	u8_t flash_offset;
	u8_t miss;
	u8_t min;
	u8_t *buf8 = (u8_t *)&buf;
	u8_t *data8 = (u8_t *)data;

	if ((addr + data_size) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	while (data_size > 0) {
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;
		min = MIN(miss, data_size);

		if (wait_while_spi_busy())
			return 1;

		buf = sys_read32(MEMORY_MAP_BASE_ADDR + addr - flash_offset);
		memcpy(data8, buf8 + flash_offset, min);
		addr += min;
		data8 += min;
		data_size -= min;
	}
	return 0;
}

static int spi_flash_litex_erase(struct device *dev, off_t addr, size_t size)
{
	if ((addr + size) > MEMORY_SIZE || addr < 0)
		return -EFAULT;

	for (int i = 0; i < size; i += SUBSECTOR_SIZE) {
		if (prepare_write(dev))
			return 1;
		litex_write8(SUBSECTOR_ERASE, MANUAL_CMD);
		litex_write8((addr + i) >> 16, MANUAL_CMD_ADDR_0_7);
		litex_write8((addr + i) >> 8, MANUAL_CMD_ADDR_8_15);
		litex_write8((addr + i) >> 0, MANUAL_CMD_ADDR_16_23);
	}
	return 0;
}

int z_impl_flash_get_page_info_by_offs(struct device *dev, off_t offs,
				      struct flash_pages_info *info)
{
	/* Zephyr assumes flash is page erasable but micron N25Q128A */
	/* is subsector erasable so we need to return subsector values */
	/* This shouldn't cause problems as driver can manage writing */
	/* more than page size */

	info->start_offset = (offs / SUBSECTOR_SIZE) * SUBSECTOR_SIZE;
	info->size = SUBSECTOR_SIZE;
	info->index = offs / SUBSECTOR_SIZE;
	return 0;
}

static struct flash_driver_api spi_flash_litex_driver_api = {
	.read = spi_flash_litex_read,
	.write = spi_flash_litex_write,
	.erase = spi_flash_litex_erase,
	.write_block_size = WRITE_SIZE,
	.write_protection = spi_flash_litex_write_protection,
};

DEVICE_DECLARE(spi_flash_memory);

DEVICE_AND_API_INIT(spi_flash_memory,
	SPI_FLASH4X0_LABEL,
	spi_flash_litex_init,
	NULL,
	NULL,
	POST_KERNEL,
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	&spi_flash_litex_driver_api);
