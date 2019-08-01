/*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_FLASH_H_
#define ZEPHYR_DRIVERS_SPI_FLASH_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/spi.h>
#include <drivers/flash.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>


#ifdef __cplusplus
}
#endif

#define MEMORY_MAP_BASE_ADDR DT_INST_0_LITEX_SPIFLASH4X0_MEMORY_MAP_BASE_ADDRESS

#define SPI_CSR_BASE DT_INST_0_LITEX_SPIFLASH4X0_BASE_ADDRESS_0
#define SPI_FLASH4X0_LABEL DT_INST_0_LITEX_SPIFLASH4X0_LABEL

#define SPI_FLASH4X0_BASE SPI_CSR_BASE

#define SUBSECTOR_ERASE 0x20

#define MEMORY_SIZE DT_INST_0_LITEX_SPIFLASH4X0_MEMORY_MAP_SIZE
#define SUBSECTOR_SIZE 4096L
#define WRITE_SIZE 1

#define TIMEOUT_FLASH 0x5000000
#define TIMEOUT_SPI 0x500

#define WIP 0x1
#define WEN_SET 0x2

#define STATUS_CSR (SPI_CSR_BASE | 0x10)
#define READ_STATUS_CMD_CSR (SPI_CSR_BASE | 0x18)
#define WRITE_ENABLE_CSR (SPI_CSR_BASE | 0x1C)
#define MANUAL_CMD (SPI_CSR_BASE | 0x20)
#define MANUAL_CMD_ADDR_0_7 (SPI_CSR_BASE | 0x24)
#define MANUAL_CMD_ADDR_8_15 (SPI_CSR_BASE | 0x28)
#define MANUAL_CMD_ADDR_16_23 (SPI_CSR_BASE | 0x2C)
#define COMMAND_QUEUED (SPI_CSR_BASE | 0x30)
#define EN_QUAD (SPI_CSR_BASE | 0x34)
#define READ_ID_REG (SPI_CSR_BASE | 0x38)

#define MANUFACTURER_ID 0x20

#endif /* ZEPHYR_DRIVERS_SPI_FLASH_H_ */
