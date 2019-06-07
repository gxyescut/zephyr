/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <version.h>
#include <logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(app);

extern void foo(void);

void timer_expired_handler(struct k_timer *timer)
{
	LOG_INF("Timer expired.");

	/* Call another module to present logging from multiple sources. */
	foo();
}

K_TIMER_DEFINE(log_timer, timer_expired_handler, NULL);

static int cmd_log_test_start(const struct shell *shell, size_t argc,
			      char **argv, u32_t period)
{
	ARG_UNUSED(argv);

	k_timer_start(&log_timer, period, period);
	shell_print(shell, "Log test started\n");

	return 0;
}

static int cmd_log_test_start_demo(const struct shell *shell, size_t argc,
				   char **argv)
{
	return cmd_log_test_start(shell, argc, argv, 200);
}

static int cmd_log_test_start_flood(const struct shell *shell, size_t argc,
				    char **argv)
{
	return cmd_log_test_start(shell, argc, argv, 10);
}

static int cmd_log_test_stop(const struct shell *shell, size_t argc,
			     char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_timer_stop(&log_timer);
	shell_print(shell, "Log test stopped");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_log_test_start,
	SHELL_CMD_ARG(demo, NULL,
		  "Start log timer which generates log message every 200ms.",
		  cmd_log_test_start_demo, 1, 0),
	SHELL_CMD_ARG(flood, NULL,
		  "Start log timer which generates log message every 10ms.",
		  cmd_log_test_start_flood, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_STATIC_SUBCMD_SET_CREATE(sub_log_test,
	SHELL_CMD_ARG(start, &sub_log_test_start, "Start log test", NULL, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "Stop log test.", cmd_log_test_stop, 1, 0),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(log_test, &sub_log_test, "Log test", NULL);

static int cmd_demo_ping(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "pong");

	return 0;
}

static int cmd_demo_params(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "argc = %d", argc);
	for (size_t cnt = 0; cnt < argc; cnt++) {
		shell_print(shell, "  argv[%d] = %s", cnt, argv[cnt]);
	}

	return 0;
}

static int cmd_version(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Zephyr version %s", KERNEL_VERSION_STRING);

	return 0;
}

int led_on = 0;

static int cmd_led_toggle(const struct shell *shell, size_t argc, char **argv)
{
	*((volatile uint32_t*)0xe0006808) = 0x7;

	*((volatile uint32_t*)0xe0006804) = 0x8; // address: LEDDCR0
	*((volatile uint32_t*)0xe0006800) = 0xc8;
	*((volatile uint32_t*)0xe0006804) = 0x9; // address: LEDDBR
	*((volatile uint32_t*)0xe0006800) = 0xba;

	*((volatile uint32_t*)0xe0006808) = 0x6;
	*((volatile uint32_t*)0xe0006808) = 0x7;

	// control breathe on/off
	*((volatile uint32_t*)0xe0006804) = 0x5; // address: LEDDBCRR
	*((volatile uint32_t*)0xe0006800) = 0x0;
	*((volatile uint32_t*)0xe0006804) = 0x6; // address: LEDDBCFR
	*((volatile uint32_t*)0xe0006800) = 0x0;

	// set brightness
	*((volatile uint32_t*)0xe0006804) = 0x2; // address: LEDDPWRG
	*((volatile uint32_t*)0xe0006800) = 0x0;
	*((volatile uint32_t*)0xe0006804) = 0x3; // address: LEDDPWRB - this one seems to be red
	*((volatile uint32_t*)0xe0006800) = 0x0;
	*((volatile uint32_t*)0xe0006804) = 0x1; // address: LEDDPWRR - it seems to be blue
	*((volatile uint32_t*)0xe0006800) = 0xff;

	if(led_on)
	{
		// led driver on/off time register
		*((volatile uint32_t*)0xe0006804) = 0xa; // address: LEDDONR
		*((volatile uint32_t*)0xe0006800) = 0x0;
		*((volatile uint32_t*)0xe0006804) = 0xb; // address: LEDDOFR
		*((volatile uint32_t*)0xe0006800) = 0xff;

		led_on = 0;
		shell_print(shell, "LED turned off");
	}
	else
	{
		// led driver on/off time register
		*((volatile uint32_t*)0xe0006804) = 0xa; // address: LEDDONR
		*((volatile uint32_t*)0xe0006800) = 0xff;
		*((volatile uint32_t*)0xe0006804) = 0xb; // address: LEDDOFR
		*((volatile uint32_t*)0xe0006800) = 0x0;

		led_on = 1;
		shell_print(shell, "LED turned on");
	}

	return 0;
}

static int cmd_led_breathe(const struct shell *shell, size_t argc, char **argv)
{
	*((volatile uint32_t*)0xe0006808) = 0x7;

	*((volatile uint32_t*)0xe0006804) = 0x8; // address: LEDDCR0
	*((volatile uint32_t*)0xe0006800) = 0xc8;
	*((volatile uint32_t*)0xe0006804) = 0x9; // address: LEDDBR
	*((volatile uint32_t*)0xe0006800) = 0xba;

	*((volatile uint32_t*)0xe0006808) = 0x6;
	*((volatile uint32_t*)0xe0006808) = 0x7;


	// led driver on/off time register
	*((volatile uint32_t*)0xe0006804) = 0xa; // address: LEDDONR
	*((volatile uint32_t*)0xe0006800) = 0x3;
	*((volatile uint32_t*)0xe0006804) = 0xb; // address: LEDDOFR
	*((volatile uint32_t*)0xe0006800) = 0x3;


	// control breathe on/off
	*((volatile uint32_t*)0xe0006804) = 0x5; // address: LEDDBCRR
	*((volatile uint32_t*)0xe0006800) = 0xe2;
	*((volatile uint32_t*)0xe0006804) = 0x6; // address: LEDDBCFR
	*((volatile uint32_t*)0xe0006800) = 0xa3;


	// set brightness
	*((volatile uint32_t*)0xe0006804) = 0x2; // address: LEDDPWRG
	*((volatile uint32_t*)0xe0006800) = 0x3c;
	*((volatile uint32_t*)0xe0006804) = 0x3; // address: LEDDPWRB
	*((volatile uint32_t*)0xe0006800) = 0x2;
	*((volatile uint32_t*)0xe0006804) = 0x1; // address: LEDDPWRR
	*((volatile uint32_t*)0xe0006800) = 0x0;

	led_on = 1;

	shell_print(shell, "LED breathing turned on");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo,
	SHELL_CMD(params, NULL, "Print params command.", cmd_demo_params),
	SHELL_CMD(ping, NULL, "Ping command.", cmd_demo_ping),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);

SHELL_CMD_ARG_REGISTER(version, NULL, "Show kernel version", cmd_version, 1, 0);

SHELL_CMD_ARG_REGISTER(led_toggle, NULL, "Toggles LED", cmd_led_toggle, 1, 0);
SHELL_CMD_ARG_REGISTER(led_breathe, NULL, "Switched LED into blink with breathe mode", cmd_led_breathe, 1, 0);

void main(void)
{

}
