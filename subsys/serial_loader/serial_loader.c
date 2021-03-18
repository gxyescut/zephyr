/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <sys/byteorder.h>

#ifdef CONFIG_SHELL_BACKEND_SERIAL_RX_POLL_PERIOD
#define RX_POLL_PERIOD K_MSEC(CONFIG_SHELL_BACKEND_SERIAL_RX_POLL_PERIOD)
#else
#define RX_POLL_PERIOD K_NO_WAIT
#endif

#define EOT 4
#define CR 13
#define LF 10
#define SPC 32
#define TAB 9

static int cmd_load(const struct shell *shell, size_t argc, char **argv)
{
	bool littleendian = false;
	char *arg;

	while (argc >= 2) {
		arg = argv[1] + (!strncmp(argv[1], "--", 2) && argv[1][2]);
		if (!strncmp(arg, "-e", 2))
			littleendian = true;
		else if (!strcmp(arg, "--")) {
			argv++;
			argc--;
			break;
		} else if (arg[0] == '-' && arg[1])
			shell_print(shell, "Unknown option \"%s\"", arg);
		else
			break;
		argv++;
		argc--;
	}

	const struct shell_uart *sh_uart =
		(struct shell_uart *)shell->iface->ctx;
	k_timer_stop(sh_uart->timer);

	volatile uint32_t *data = (uint32_t *)strtol(argv[1], NULL, 0);

	shell_print(shell, "Loading...");
	shell_print(shell, "Press ctrl+d to stop");

	uint32_t recv = 0;
	unsigned char recv_char;
	int i = 0;
	int sum = 0;
	char chunk[] = "00000000";

	while (1) {
		while (uart_poll_in(sh_uart->ctrl_blk->dev, &recv_char) < 0) {
		}

		if (recv_char == EOT) {
			break;
		}

		if (recv_char == CR || recv_char == LF || recv_char == SPC ||
		    recv_char == TAB) {
			continue;
		}

		chunk[i] = recv_char;
		i++;

		if (i == 8) {
			recv = (uint32_t)strtoul(chunk, NULL, 16);
			if (littleendian)
				recv = __bswap_32(recv);
			*data = recv;
			data++;
			sum++;
			i = 0;
		}
	}

	k_timer_start(sh_uart->timer, RX_POLL_PERIOD, RX_POLL_PERIOD);
	shell_print(shell, "Number of bytes read: %d", sum * 4);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_serial,
			       SHELL_CMD_ARG(load, NULL,
					     "Usage:\n"
					     "serial load [options] [address]\n"
					     "Options:\n"
					     "-e\tlittle-endian parse",
					     cmd_load, 2, 1),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(serial, &sub_serial, "serial commands", NULL);
