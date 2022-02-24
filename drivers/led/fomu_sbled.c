/*
 * Copyright (c) 2022 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT fomu_sbled

#include <drivers/led.h>
#include <sys/util.h>
#include <zephyr.h>

#include "led_context.h"

enum led_registers {
	LEDDCR0 = 8,
	LEDDBR = 9,
	LEDDONR = 10,
	LEDDOFR = 11,
	LEDDBCRR = 5,
	LEDDBCFR = 6,
	LEDDPWRR = 1,
	LEDDPWRG = 2,
	LEDDPWRB = 3,
};

#define BREATHE_ENABLE (1 << 7)
#define BREATHE_EDGE_ON (0 << 6)
#define BREATHE_EDGE_BOTH (1 << 6)
#define BREATHE_MODE_MODULATE (1 << 5)
#define BREATHE_MODE_FIXED (0 << 5)

// Breathe rate is in 128 ms increments
#define BREATHE_RATE_MS(x) ((((x)+1) / 128 & 7) << 0)

// Blink on/off time is in 32 ms increments
#define BLINK_TIME_MS(x) (((x)) / 32)

#define LEDDEN (1 << 7)
#define FR250 (1 << 6)
#define OUTPUL (1 << 5)
#define OUTSKEW (1 << 4)
#define QUICK_STOP (1 << 3)
#define PWM_MODE_LFSR (1 << 2)
#define PWM_MODE_LINEAR (0 << 2)

#define SBLED_DAT_ADDR	DT_INST_REG_ADDR_BY_NAME(0, dat)
#define SBLED_ADDR_ADDR	DT_INST_REG_ADDR_BY_NAME(0, addr)
#define SBLED_CTRL_ADDR	DT_INST_REG_ADDR_BY_NAME(0, ctrl)
#define SBLED_RAW_ADDR	DT_INST_REG_ADDR_BY_NAME(0, raw)

static void rgb_write(uint8_t value, uint8_t addr) {
	litex_write8(addr, SBLED_ADDR_ADDR);
	litex_write8(value, SBLED_DAT_ADDR);
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
	// Note: the LEDD control registers have arbitrary names that
	// do not match up with the LEDD pin outputs.  Hence this strange
	// mapping.
	rgb_write(r, LEDDPWRR); // Blue
	rgb_write(g, LEDDPWRG); // Red
	rgb_write(b, LEDDPWRB); // Green
}

// Time to stay "on"
static void rgb_on_time(uint8_t ms) {
	rgb_write(BLINK_TIME_MS(ms), LEDDONR);
}

// Time to stay "off"
static void rgb_off_time(uint8_t ms) {
	rgb_write(BLINK_TIME_MS(ms), LEDDOFR);
}

// The amount of time to breathe in/out
static void rgb_in_time(uint8_t ms) {
	rgb_write(BREATHE_ENABLE| BREATHE_MODE_FIXED | BREATHE_RATE_MS(ms), LEDDBCRR);
}

static void rgb_out_time(uint8_t ms) {
	rgb_write(BREATHE_ENABLE | BREATHE_MODE_FIXED | BREATHE_RATE_MS(ms), LEDDBCFR);
}

static enum {
	INVALID = 0,
	IDLE,
	WRITING,
	ERROR,
	DONE,
} rgb_mode;

static void rgb_switch_mode(uint8_t mode,
		uint8_t on_time, uint8_t off_time,
		uint8_t in_time, uint8_t out_time,
		uint8_t r, uint8_t g, uint8_t b) {

	if (rgb_mode == mode)
		return;

	rgb_mode = mode;

	/* Toggle LEDD_EXE to force the mode to switch */
	litex_write8(0b110, SBLED_CTRL_ADDR);
	litex_write8(0b111, SBLED_CTRL_ADDR);

	rgb_on_time(on_time);
	rgb_off_time(off_time);

	rgb_write(BREATHE_ENABLE | BREATHE_EDGE_BOTH | BREATHE_MODE_MODULATE | BREATHE_RATE_MS(in_time), LEDDBCRR);
	rgb_write(BREATHE_ENABLE | BREATHE_MODE_MODULATE | BREATHE_RATE_MS(out_time), LEDDBCFR);

	rgb_set(r, g, b);
}

__attribute__ ((unused))
static void rgb_mode_idle(void) {
	rgb_switch_mode(IDLE, 12, 14, 2, 3, 0x00/4, 0xe1/4, 0x4a/4);
}

__attribute__ ((unused))
static void rgb_mode_writing(void) {
	rgb_switch_mode(WRITING, 1, 2, 1, 3, 0x00/4, 0x51/4, 0x7a/4);
}

__attribute__ ((unused))
static void rgb_mode_error(void) {
	rgb_switch_mode(ERROR, 3, 3, 2, 3, 0xf0/4, 0x01/4, 0x0a/4);
}

__attribute__ ((unused))
static void rgb_mode_done(void) {
	rgb_switch_mode(DONE, 8, 8, 2, 3, 0x14/4, 0x44/4, 0xff/4);
}

enum {
	R_CHAN = 0,
	G_CHAN = 1,
	B_CHAN = 2,
};

static inline int fomu_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	switch (led)
	{
	case R_CHAN:
		rgb_write(value, LEDDPWRG); // Red
		break;
	case G_CHAN:
		rgb_write(value, LEDDPWRB); // Green
		break;
	case B_CHAN:
		rgb_write(value, LEDDPWRR); // Blue
		break;
	default:
		return -1;
	}
	return 0;
}

static inline int fomu_led_blink(const struct device *dev, uint32_t led,
		uint32_t delay_on, uint32_t delay_off)
{
	rgb_on_time(delay_on);
	rgb_off_time(delay_off);

	return 0;
}

static inline int fomu_led_on(const struct device *dev, uint32_t led)
{
	return fomu_led_set_brightness(dev, led, 255);
}

static inline int fomu_led_off(const struct device *dev, uint32_t led)
{
	return fomu_led_set_brightness(dev, led, 0);
}

static int fomu_led_init(const struct device *dev)
{
	// Turn on the RGB block and current enable, as well as enabling led control
	litex_write8(0b111, SBLED_CTRL_ADDR);

	// Enable the LED driver, and set 250 Hz mode.
	// Also set quick stop, which we'll use to switch patterns quickly.
	rgb_write(LEDDEN | FR250 | QUICK_STOP, LEDDCR0);

	// Set clock register to 12 MHz / 64 kHz - 1
	rgb_write((12000000/64000)-1, LEDDBR);

	rgb_on_time(32);
	rgb_off_time(0);

	rgb_in_time(128);
	rgb_out_time(128);

	rgb_mode_idle();

	return 0;
}

static const struct led_driver_api fomu_led_api = {
	.blink = fomu_led_blink,
	.on = fomu_led_on,
	.off = fomu_led_off,
	.set_brightness = fomu_led_set_brightness,
};

DEVICE_DT_INST_DEFINE(0, &fomu_led_init, NULL, NULL,
	NULL, PRE_KERNEL_1, CONFIG_LED_INIT_PRIORITY,
	&fomu_led_api);
