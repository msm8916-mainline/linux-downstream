/***************************************************************************
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
 *    File  	: lgtp_platform_api.c
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>

#include <linux/input/lgtp_common_driver.h>
#include <linux/input/lgtp_platform_api.h>
#include <linux/input/lgtp_model_config.h>
#include <linux/input/lgtp_device_s3320.h>

#define BUFFER_SIZE_TO_DIVIDE 	255


#if defined ( CONFIG_MTK_TOUCHPANEL )
#include <cust_eint.h>
#include <linux/dma-mapping.h>
#include "tpd.h"

static u8* I2CDMABuf_va = NULL;
static u32 I2CDMABuf_pa = NULL;
#endif

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
int nInt_Gpio_num;
int nRst_Gpio_num;
int nIrq_num;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/

/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
#ifdef CONFIG_MTK_TOUCHPANEL
static int ts_i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;

	for ( i = 0 ; i < len ; i++ )
		I2CDMABuf_va[i] = buf[i];

	if (len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int ts_i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i = 0;
	int ret = 0;

	if(len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		return i2c_master_recv(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 400;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);

		if ( ret < 0 )
			return ret;

		for ( i = 0 ; i < len ; i++ )
			buf[i] = I2CDMABuf_va[i];
	}

	return ret;
}

int ts_i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count)
{
	int i = 0;
	int ret = 0;

	for (i = 0 ; i < count ; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = ts_i2c_dma_read(client, msgs[i].buf, msgs[i].len);
		else
			ret = ts_i2c_dma_write(client, msgs[i].buf, msgs[i].len);

		if ( ret < 0 )
			return ret;
	}

	return 0;
}
#endif

#if defined(CONFIG_ARCH_MSM8916)
static int pinctrl_init(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR(ts->ts_pinctrl)) {
		if (PTR_ERR(ts->ts_pinctrl) == -EPROBE_DEFER) {
			LGTC_ERR("ts_pinctrl == -EPROBE_DEFER\n");
			return -EPROBE_DEFER;
		}
		LGTC_DBG("Target does not use pinctrl(ts->ts_pinctrl == NULL) \n");
		ts->ts_pinctrl = NULL;
	}

	if (ts->ts_pinctrl) {
		ts->ts_pinset_state_active = pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
		if (IS_ERR(ts->ts_pinset_state_active))
			LGTC_DBG("cannot get ts pinctrl active state\n");

		ts->ts_pinset_state_suspend = pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
		if (IS_ERR(ts->ts_pinset_state_suspend))
			LGTC_DBG("cannot get ts pinctrl active state\n");

		if (ts->ts_pinset_state_active) {
			DO_SAFE(pinctrl_select_state(ts->ts_pinctrl, ts->ts_pinset_state_active), error);
		} else {
			LGTC_DBG("pinctrl active == NULL \n");
		}
	}

	return NO_ERROR;
error:
	return ERROR;
}
#endif



/****************************************************************************
* Global Functions
****************************************************************************/
#ifdef CONFIG_MTK_TOUCHPANEL
int ts_dma_allocation(void)
{
	I2CDMABuf_va = (u8*)dma_alloc_coherent (NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);

	if(!I2CDMABuf_va)
		return -ENOMEM;
	else
		return 0;
}
#endif

int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf)
 {
	 struct i2c_msg msgs[] = {
		 {
			 .addr = client->addr,
			 .flags = 0,
			 .len = 1,
			 .buf = &reg,
		 },
		 {
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = len,
			 .buf = buf,
		 },
	 };

	 int ret = 0;

	#ifdef CONFIG_MTK_TOUCHPANEL
	 ret = ts_i2c_msg_transfer(client, msgs, 2);
	#else
	 ret = i2c_transfer(client->adapter, msgs, 2);
	#endif

	 if (ret < 0) {
		 if (printk_ratelimit())
			 LGTC_ERR("transfer error\n");
		 return -EIO;
	 } else
		 return 0;
 }
 
int touch_i2c_read_byte(struct i2c_client *client, u8 reg, int len, u8 *buf)
 {
	 struct i2c_msg* msgs = NULL;
	 int msg_count = ((len - 1) / BUFFER_SIZE_TO_DIVIDE) + 2;
	 int msg_rest_count = len % BUFFER_SIZE_TO_DIVIDE;
	 int i = 0;
	 int data_len = 0;
 
	 msgs = (struct i2c_msg*)kcalloc(msg_count, sizeof(struct i2c_msg), GFP_KERNEL);
	 if(msgs != NULL)
		 memset(msgs, 0x00, sizeof(struct i2c_msg));
	 else
		 return -EIO;
 
	 msgs[0].addr = client->addr;
	 msgs[0].flags = 0;
	 msgs[0].len = 1;
	 msgs[0].buf = &reg;
 
	 if (!msg_rest_count)
		 msg_rest_count = BUFFER_SIZE_TO_DIVIDE;
 
	 for (i = 0 ; i < (msg_count - 1) ; i++) {
		 if (i == (msg_count - 2))
			 data_len = msg_rest_count;
		 else
			 data_len = BUFFER_SIZE_TO_DIVIDE;
 
		 msgs[i + 1].addr = client->addr;
		 msgs[i + 1].flags = I2C_M_RD;
		 msgs[i + 1].len = data_len;
		 msgs[i + 1].buf = buf + BUFFER_SIZE_TO_DIVIDE * i;
	 }
 
	#ifdef CONFIG_MTK_TOUCHPANEL
	 if (ts_i2c_msg_transfer(client, msgs, msg_count) < 0) {
		 if (printk_ratelimit())
			 LGTC_ERR("transfer error\n");
 
		 return -EIO;
	 }
	#else
	 if (i2c_transfer(client->adapter, msgs, msg_count) == 2) {
		 if (printk_ratelimit())
			 LGTC_ERR("transfer error\n");
 
		 return -EIO;
	 }
	#endif
 
	 kfree(msgs);
	 return 0;
 }
 
int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf)
 {
	 unsigned char send_buf[len + 1];
 
	 struct i2c_msg msgs[] = {
		 {
			 .addr = client->addr,
			 .flags = client->flags,
			 .len = len+1,
			 .buf = send_buf,
		 },
	 };
 
	 int ret = 0;

	 send_buf[0] = (unsigned char)reg;
	 memcpy(&send_buf[1], buf, len);
 
	#ifdef CONFIG_MTK_TOUCHPANEL
	 ret = ts_i2c_msg_transfer(client, msgs, 1);
	#else
	 ret = i2c_transfer(client->adapter, msgs, 1);
	#endif

	 if (ret < 0) {
		 if (printk_ratelimit())
			 LGTC_ERR("transfer error\n");
		 return -EIO;
	 } else
		 return 0;
 }
 
int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data)
 {
	 unsigned char send_buf[2];
	 struct i2c_msg msgs[] = {
		 {
			 .addr = client->addr,
			 .flags = client->flags,
			 .len = 2,
			 .buf = send_buf,
		 },
	 };

	 int ret = 0;

	 send_buf[0] = (unsigned char)reg;

	#ifdef CONFIG_MTK_TOUCHPANEL
	 memcpy(&send_buf[1], &data, 1);
	 ret = ts_i2c_msg_transfer(client, msgs, 1);
	#else
	 send_buf[1] = (unsigned char)data;
	 ret = i2c_transfer(client->adapter, msgs, 1);
	#endif

	 if (ret < 0) {
		 if (printk_ratelimit())
			 LGTC_ERR("transfer error\n");
		 return -EIO;
	 } else
		 return 0;
 }


//==========================================================
// 함수명 : TouchReset
// Touch IC를 Hardware적으로 Reset시킨다.
// Reset 동작및 IC안정화에 필요한 Delay은 함수 내부에 구현되어야 한다.
// ( 함수가 Return되면, 바로 IC에 Read/Write동작을 수행함 )
//==========================================================
void TouchReset( void )
{
	LGTC_FUN();

	LGTC_DBG("Reset Asserted\n");

	#if defined ( CONFIG_MTK_TOUCHPANEL )

	mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
	msleep(10);
	mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
	msleep(150);

	#else

	gpio_set_value(nRst_Gpio_num, 0);
	msleep(10);
	gpio_set_value(nRst_Gpio_num, 1);
	msleep(150);

	#endif

	LGTC_DBG("Reset Released\n");
}

void TouchEnableIrq( void )
{
	#if defined ( CONFIG_MTK_TOUCHPANEL )
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	#else
	enable_irq(nIrq_num);
	#endif

	LGTC_LOG("Interrupt Enabled\n");
}

void TouchDisableIrq( void )
{
	#ifdef CONFIG_MTK_TOUCHPANEL
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	#else
	disable_irq(nIrq_num);
	#endif

	LGTC_LOG("Interrupt Disabled\n");
}

int TouchReadGpioLineInt( void )
{
	int gpioState = 0;

	#ifdef CONFIG_MTK_TOUCHPANEL
	gpioState = mt_get_gpio_in(GPIO_TOUCH_INT);
	#else
	gpioState = gpio_get_value(nInt_Gpio_num);
	#endif

	return gpioState;
}

//==========================================================
// 함수명 : InitialisePlatform
// Platform별로 필요한 초기화를 수행한다.
// 주로, 전원, 리셋, GPIO, Intrrupt, I2C등을 컨드롤하기 위해 사전에 필요한 작업을 한다.
// 시스템 부팅후 한번만 호출된다.
//==========================================================
int TouchInitialisePlatform ( struct lge_touch_data *pDriverData )
{
	int result = LGTC_SUCCESS;

	LGTC_FUN();

	result = TouchGetModelConfig(pDriverData);
	if( result == LGTC_FAIL ) {
		LGTC_ERR("Failed at TouchGetModelConfig\n");
		goto earlyReturn;
	}

	#if defined ( CONFIG_MTK_TOUCHPANEL )

	/* Reset PIN */
	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);
	//mt_set_gpio_pull_select(GPIO_TOUCH_RESET, GPIO_PULL_UP);
	//mt_set_gpio_pull_enable(GPIO_TOUCH_RESET, GPIO_PULL_ENABLE);
	//mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);

	/* Interrupt PIN*/
	mt_set_gpio_mode(GPIO_TOUCH_INT, GPIO_TOUCH_INT_M_EINT);
	mt_set_gpio_dir(GPIO_TOUCH_INT, GPIO_DIR_IN);
	//mt_set_gpio_pull_select(GPIO_TOUCH_INT, GPIO_PULL_UP);
	//mt_set_gpio_pull_enable(GPIO_TOUCH_INT, GPIO_PULL_ENABLE);

	#else

	#if defined(CONFIG_ARCH_MSM8916)
	pinctrl_init(pDriverData->client);
	#endif

	if (pDriverData->mConfig.reset_pin > 0) {
		gpio_request(pDriverData->mConfig.reset_pin, "touch_reset");
		gpio_direction_output(pDriverData->mConfig.reset_pin, 1);
	}

	if (pDriverData->mConfig.int_pin > 0) {
		gpio_request(pDriverData->mConfig.int_pin, "touch_int");
		gpio_direction_input(pDriverData->mConfig.int_pin);
	}

	#endif

earlyReturn:

	return result;
}



/* End Of File */

