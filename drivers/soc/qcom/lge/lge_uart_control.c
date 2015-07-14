/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/string.h>

static unsigned int uart_console_mode;

unsigned int lge_get_uart_mode(void)
{
	return uart_console_mode;
}
EXPORT_SYMBOL(lge_get_uart_mode);

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 6)) {
		uart_console_mode = 1;
		pr_info("UART CONSOLE : enabled\n");
	} else {
		uart_console_mode = 0;
		pr_info("UART CONSOLE : disabled\n");
	}

	return 1;
}
__setup("lge.uart=", lge_uart_mode);
