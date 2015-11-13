/*
 *  Siano core API module
 *
 *  This file contains implementation for the interface to sms core component
 *
 *  author: wood
 *
 *  Copyright (c), 2005-2008 Siano Mobile Silicon, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation;
 *
 *  Software distributed under the License is distributed on an "AS IS"
 *  basis, WITHOUT WARRANTY OF ANY KIND, either express or implied.
 *
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include "smscoreapi.h"
#include <linux/delay.h>
#include "smspower.h"



#include <linux/gpio.h>

#ifdef CONFIG_MEDIATEK_SOLUTION
#include <mach/mt_gpio.h>
#include <cust_gpio_boot.h>

#define  HOST_SMS_RSET_PIN (GPIO108 | 0x80000000)
#define  HOST_SMS_POWER_IO_PIN (GPIO20 | 0x80000000)
//#define  HOST_SMS_POWER_EMI_PIN (GPIO62 | 0x80000000)
#define  HOST_SMS_POWER_CORE_PIN (GPIO21 | 0x80000000)
#define GPIO_HIGH	GPIO_OUT_ONE
#define GPIO_LOW	GPIO_OUT_ZERO
#define gpio_request(pin,name) 0
#define gpio_free(pin)			0
#define gpio_direction_output(pin,level) \
	do { \
		mt_set_gpio_mode(pin,GPIO_MODE_00); \
		mt_set_gpio_dir(pin, GPIO_DIR_OUT); \
		mt_set_gpio_out(pin, level); \
	} while (0)
	
#define gpio_set_value(pin,lever) mt_set_gpio_out(pin,lever)

#elif defined(CONFIG_PLAT_RK)
#include <linux/interrupt.h>
#include "../../../spi/rk29_spim.h"
#define  HOST_SMS_RSET_PIN RK30_PIN0_PB1
#define  HOST_SMS_POWER_IO_PIN RK30_PIN0_PC7
#define  HOST_SMS_POWER_EMI_PIN RK30_PIN0_PC0
#define  HOST_SMS_POWER_CORE_PIN RK30_PIN0_PC1

#endif //CONFIG_ARCH_ ...

void sms_chip_poweron(void)
{
	sms_info("");		
#ifdef  HOST_SMS_RSET_PIN
	gpio_request(HOST_SMS_RSET_PIN , "SMSXXXX_RST");
#endif

#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_request(HOST_SMS_POWER_IO_PIN , "SMSXXXX_PWR_IO");
#endif
#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_request(HOST_SMS_POWER_EMI_PIN , "SMSXXXX_PWR_IO");
#endif
#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_request(HOST_SMS_POWER_CORE_PIN , "SMSXXXX_PWR_CORE");
#endif
	
#ifdef  HOST_SMS_RSET_PIN
	gpio_direction_output(HOST_SMS_RSET_PIN, GPIO_LOW);
	msleep(50);
#endif
#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_direction_output(HOST_SMS_POWER_IO_PIN, GPIO_LOW);
	msleep(10);
#endif
#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_direction_output(HOST_SMS_POWER_EMI_PIN, GPIO_LOW);
	msleep(10);
#endif
#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_direction_output(HOST_SMS_POWER_CORE_PIN, GPIO_LOW);
	msleep(200);
#endif

#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_set_value(HOST_SMS_POWER_CORE_PIN, GPIO_HIGH);
	msleep(10);
#endif

#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_set_value(HOST_SMS_POWER_EMI_PIN, GPIO_HIGH);
	msleep(50);
#endif
#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_set_value(HOST_SMS_POWER_IO_PIN, GPIO_HIGH);
	msleep(10);
#endif
#ifdef  HOST_SMS_RSET_PIN
	gpio_set_value(HOST_SMS_RSET_PIN, GPIO_HIGH);
#endif
	msleep(200);


#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_free(HOST_SMS_POWER_CORE_PIN);
#endif
#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_free(HOST_SMS_POWER_EMI_PIN);
#endif
#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_free(HOST_SMS_POWER_IO_PIN);
#endif
#ifdef  HOST_SMS_RSET_PIN
	gpio_free(HOST_SMS_RSET_PIN);
#endif
}


void sms_chip_poweroff(void)
{
	sms_info("");
#ifdef  HOST_SMS_RSET_PIN
	gpio_request(HOST_SMS_RSET_PIN , "SMSXXXX_RST");
#endif

#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_request(HOST_SMS_POWER_IO_PIN , "SMSXXXX_PWR_IO");
#endif
#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_request(HOST_SMS_POWER_EMI_PIN , "SMSXXXX_PWR_EMI");
#endif
#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_request(HOST_SMS_POWER_CORE_PIN , "SMSXXXX_PWR_CORE");
#endif
	
#ifdef  HOST_SMS_RSET_PIN
	gpio_direction_output(HOST_SMS_RSET_PIN, GPIO_LOW);
	msleep(50);
#endif	

#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_direction_output(HOST_SMS_POWER_IO_PIN, GPIO_LOW);
	msleep(10);
#endif

#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_direction_output(HOST_SMS_POWER_EMI_PIN, GPIO_LOW);
	msleep(10);
#endif
#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_direction_output(HOST_SMS_POWER_CORE_PIN, GPIO_LOW);
#endif

	msleep(50);


#ifdef  HOST_SMS_POWER_CORE_PIN
	gpio_free(HOST_SMS_POWER_CORE_PIN);
#endif
#ifdef  HOST_SMS_POWER_EMI_PIN
	gpio_free(HOST_SMS_POWER_EMI_PIN);
#endif
#ifdef  HOST_SMS_POWER_IO_PIN
	gpio_free(HOST_SMS_POWER_IO_PIN);
#endif
#ifdef  HOST_SMS_RSET_PIN
	gpio_free(HOST_SMS_RSET_PIN);
#endif
}
