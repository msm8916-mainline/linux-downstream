/* Lge_touch_platform.c
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include "lge_touch_platform.h"

#ifdef CONFIG_MTK_TOUCHPANEL
#include <cust_eint.h>
#include <linux/dma-mapping.h>
#include "tpd.h"

static u8* I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
#endif

#ifdef CONFIG_MTK_TOUCHPANEL
int ts_dma_allocation(void)
{
	I2CDMABuf_va = (u8*)dma_alloc_coherent (NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);

	if(!I2CDMABuf_va)
		return -ENOMEM;
	else
		return 0;
}

static int ts_i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i;

	for (i = 0 ; i < len ; i++)
		I2CDMABuf_va[i] = buf[i];

	if (len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	} else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int ts_i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i;
	int ret = 0;

	if(len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	} else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);

		if (ret < 0)
			return ret;

		for (i = 0 ; i < len ; i++)
			buf[i] = I2CDMABuf_va[i];
	}

	return ret;
}

int ts_i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count)
{
	int i;
	int ret = 0;

	for (i = 0 ; i < count ; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = ts_i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		else
			ret = ts_i2c_dma_write(client, msgs[i].buf, msgs[i].len);

		if (ret < 0)
			return ret;
	}

	return ret;
}
#endif

//GPIO Control
int ts_gpio_init(unsigned gpio, const char *label, enum gpio_type type)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	GPIO_MODE mode;

	switch(type) {
	case GPIO_RST_PIN :
		mode = GPIO_MODE_00;
		break;
	case GPIO_INT_PIN :
		mode = GPIO_CTP_EINT_PIN_M_EINT;
		break;
	default :
		mode = GPIO_MODE_DEFAULT;
		break;
	}

	mt_set_gpio_pull_enable(gpio, GPIO_PULL_ENABLE);
	return mt_set_gpio_mode(gpio, mode);
#else
	return gpio_request(gpio, label);
#endif
}

int ts_gpio_direction_output(unsigned gpio, int value)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	mt_set_gpio_dir(gpio, GPIO_DIR_OUT);

	return mt_set_gpio_out(gpio, value);
#else
	return gpio_direction_output(gpio, value);
#endif
}

int ts_gpio_direction_input(unsigned gpio)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	mt_set_gpio_dir(gpio, GPIO_DIR_IN);

	return mt_get_gpio_in(gpio);
#else
	return gpio_direction_input(gpio);
#endif
}

void ts_gpio_set_value(unsigned gpio, int value)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	int pull_status;

	if(value)
		pull_status = GPIO_PULL_UP;
	else
		pull_status = GPIO_PULL_DOWN;

	mt_set_gpio_pull_select(gpio, pull_status);
#else
	gpio_set_value(gpio, value);
#endif
}

int ts_gpio_get_value(unsigned gpio)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	return mt_get_gpio_in(gpio);
#else
	return gpio_get_value(gpio);
#endif
}

void ts_enable_irq(unsigned int irq_num)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	mt_eint_unmask(irq_num);
#else
	enable_irq(irq_num);
#endif
}

void ts_disable_irq(unsigned int irq_num)
{
#ifdef CONFIG_MTK_TOUCHPANEL
	mt_eint_mask(irq_num);
#else
	disable_irq(irq_num);
#endif
}

int ts_register_irq(struct lge_touch_data *ts, void* handler, void* threaded_handler)
{
	int ret = 0;

#ifdef CONFIG_MTK_TOUCHPANEL
	ts_gpio_set_value(GPIO_CTP_EINT_PIN, 1);
	msleep(50);

	/*Configure external interrupt settings for external interrupt pin*/
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM,
					CUST_EINT_TOUCH_PANEL_TYPE, handler, 1);
#else
	ret = request_irq(ts->client->irq,
		(irq_handler_t)handler,
		ts->pdata->role->irqflags | IRQF_ONESHOT,
		ts->client->name, ts);
#endif

	INIT_DELAYED_WORK(&ts->work_irq, threaded_handler);

	return ret;
}

