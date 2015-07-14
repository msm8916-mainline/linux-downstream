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
#include <soc/qcom/lge/lge_earjack_debugger.h>


/* s_uart_console_status bits format
 * ------higher than bit4 are not used
 * bit5...: not used
 * ------bit4 indicates whenter uart console was ready(probed)
 * bit4: [UART_CONSOLE_READY]
 * ------current uart console status -----------------
 * bit3: [UART_CONSOLE_ENABLED]
 * ------configuration bit field -----------------
 * bit2: [UART_CONSOLE_ENABLE_ON_DEFAULT]
 * bit1; [UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER]
 * bit0: [UART_CONSOLE_ENABLE_ON_EARJACK]
 */
static unsigned int s_uart_console_status = 0;	/* disabling uart console */

unsigned int lge_uart_console_get_config(void)
{
	return (s_uart_console_status & UART_CONSOLE_MASK_CONFIG);
}

void lge_uart_console_set_config(unsigned int config)
{
	config &= UART_CONSOLE_MASK_CONFIG;
	s_uart_console_status |= config;
}

unsigned int lge_uart_console_get_enabled(void)
{
	return s_uart_console_status & UART_CONSOLE_MASK_ENABLED;
}

void lge_uart_console_set_enabled(int enabled)
{
	s_uart_console_status &= ~UART_CONSOLE_MASK_ENABLED;
	/* for caller conding convenience, regard no-zero as enabled also */
	s_uart_console_status |= (enabled ? UART_CONSOLE_ENABLED : 0);
}

unsigned int lge_uart_console_get_ready(void)
{
	return s_uart_console_status & UART_CONSOLE_MASK_READY;
}

void lge_uart_console_set_ready(unsigned int ready)
{
	s_uart_console_status &= ~UART_CONSOLE_MASK_READY;
	/* for caller side coding convenience, regard no-zero as ready also */
	s_uart_console_status |= (ready ? UART_CONSOLE_READY : 0);
}