/*
 * Copyright (c) 2020 Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

static int video_booster_init(struct device *dev)
{
	struct device *bce_dev; /* Board Controller Enable Gpio Device */

	ARG_UNUSED(dev);

	return 0;
}

/* needs to be done after GPIO driver init */
SYS_INIT(video_booster_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
