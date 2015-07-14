/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#ifndef _LGE_EARJACK_DEBUGGER_H
#define _LGE_EARJACK_DEBUGGER_H

/* config */
#define UART_CONSOLE_ENABLE_ON_EARJACK		BIT(0)
#define UART_CONSOLE_ENABLE_ON_EARJACK_DEBUGGER	BIT(1)
#define UART_CONSOLE_ENABLE_ON_DEFAULT		BIT(2)
/* current status
 * ENABLED | DISABLED : logical enable/disable
 * READY : It means whether device is ready or not.
 *         So even if in ENABLED state, console output will
 *         not be emitted on NOT-ready state.
 */
#define UART_CONSOLE_ENABLED		BIT(3)
#define UART_CONSOLE_DISABLED		!(BIT(3))
#define UART_CONSOLE_READY		BIT(4)
/* filter */
# define UART_CONSOLE_MASK_ENABLE_ON	(BIT(0) | BIT(1) | BIT(2))
# define UART_CONSOLE_MASK_CONFIG	UART_CONSOLE_MASK_ENABLE_ON
# define UART_CONSOLE_MASK_ENABLED	BIT(3)
# define UART_CONSOLE_MASK_READY	BIT(4)

/* config =  UART_CONSOLE_ENABLE_ON_XXX [| UART_CONSOLE_ENABLE_ON_XXX]* */
extern unsigned int lge_uart_console_get_config(void);
extern void lge_uart_console_set_config(unsigned int config);

/* logical uart console status modifier
 * used as a flag to tell "I want to enable/disable uart console"
 * @RETURN or @PARAM::enabled
 * UART_CONSOLE_ENABLED  (non-zero): enabled
 * !UART_CONSOLE_ENABLED (zero): disabled
 */
extern unsigned int lge_uart_console_get_enabled(void);
extern void lge_uart_console_set_enabled(int enabled);
/* internal uart console device status tracker
 *
 * @RETURN or @PARAM::ready
 * UART_CONSOLE_READY (non-zero): device is ready
 * !UART_CONSOLE_READY (zero): device is not ready
 */
extern unsigned int lge_uart_console_get_ready(void);
extern void lge_uart_console_set_ready(unsigned int ready);

/* real device enabler (or disabler)
 * control uart console device to enable/disable
 * NOTE @PARAM::enable should be selected by uart console enable/disable policy
 * which can be known by lge_uart_console_should_enable_on_xxx.
 * @PARAM::enable
 * zero : disabled
 * non-zero : enable
 */
extern int msm_serial_set_uart_console(int enable);
#endif
