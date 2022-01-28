/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define COMMS_DEBUG 0

#define IRQ_DEBUG 0

#if COMMS_DEBUG || IRQ_DEBUG
#define DEBUG
#endif

#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/rmi.h>
#include "rmi_driver.h"
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#ifdef CONFIG_BBK_DRIVER_INFO
#include <linux/bbk_drivers_info.h>
#endif

#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

int is_i2c_probe_done = 0;

static char *phys_proto_name = "i2c";
extern int fmea_notify(char *name,char *data);
	
//int qup_i2c_suspended;  //add  temporarily
extern int qup_i2c_suspended; // add for i2c timeout

struct rmi_i2c_data {
	struct mutex page_mutex;
	int page;
	int enabled;
	int irq;
	int irq_flags;
	struct rmi_phys_device *phys;
};

static void rmi_irq_work_func(struct work_struct *work)
{
	struct rmi_phys_device *phys = container_of(work, struct rmi_phys_device,
											irq_work);
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = phys->dev->platform_data;

	if (gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity) {
		phys->info.attn_count++;
		if (driver && driver->irq_handler && rmi_dev)
			driver->irq_handler(rmi_dev);
	}

}

extern struct rmi_driver_data *global_ddata;

static irqreturn_t rmi_i2c_irq_thread(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = phys->dev->platform_data;
	int count = 0;
	//defined in rmi_driver.h
	struct rmi_driver_data *local_data = global_ddata;

#if IRQ_DEBUG
	VIVO_TS_LOG_DBG("[%s]:ATTN gpio, value: %d.\n", __func__,
			gpio_get_value(pdata->attn_gpio));
#endif

	VIVO_TS_LOG_DBG("[%s]:Enter irq . ATTN gpio, value: %d.\n", __func__,
			gpio_get_value(pdata->attn_gpio));
	
	if(local_data->has_lcd_shutoff)
	{		
		wake_lock_timeout(&local_data->suspend_wakelock, 2*HZ);
		while(qup_i2c_suspended > 0 && count < 20)
		{
			msleep(5);
			count++;
		}
		if(count == 20)
		{
			VIVO_TS_LOG_ERR("[%s] The i2c bus still suspend after 100 times try\n", __func__);
			goto i2c_suspended;
		}
	}
	
	
//	schedule_work(&phys->irq_work);
#if 1 //wyl 
	if (gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity) {
		phys->info.attn_count++;
		if (driver && driver->irq_handler && rmi_dev)
			driver->irq_handler(rmi_dev);
	}
#endif
i2c_suspended:
	return IRQ_HANDLED;
}

/*
 * rmi_set_page - Set RMI page
 * @phys: The pointer to the rmi_phys_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_phys_device *phys, unsigned int page)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	char txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;
	int retry_times = 0;
	if(qup_i2c_suspended > 0){
		VIVO_TS_LOG_ERR("[%s]:iic bus is suspend. so not iic transfer. \n", __func__);
		return -1;
	}

#if COMMS_DEBUG
	VIVO_TS_LOG_DBG("[%s]:RMI4 I2C writes 3 bytes: %02x %02x\n", __func__,
		txbuf[0], txbuf[1]);
#endif
	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
Retry:
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		if (retry_times < I2C_RETRY_TIMES_LIMIT) {
			VIVO_TS_LOG_ERR("[%s]:i2c_master_send failed: %d. retry times %d", __func__, retval, retry_times);
			retry_times++;
			goto Retry;
		}else{
			VIVO_TS_LOG_ERR("[%s]:set page failed: %d.", __func__, retval);
		#ifdef CONFIG_FMEA
				fmea_notify("[SYNA]","rmi_set_page failed ,retry 3 times\n");	
		#endif
		return (retval < 0) ? retval : -EIO;
		}
	}
	data->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			       int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[len + 1];
	int retval;
	int retry_times = 0;
	
#if	COMMS_DEBUG
	char debug_buf[len*3 + 1];
	int i, n;
#endif
	if(qup_i2c_suspended > 0){
		VIVO_TS_LOG_ERR("[%s]:iic bus is suspend. so not iic transfer. \n", __func__);
		return -1;
	}

	txbuf[0] = addr & 0xff;
	memcpy(txbuf + 1, buf, len);

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

#if COMMS_DEBUG
	n = 0;
	for (i = 0; i < len; i++)
		n = snprintf(debug_buf+n, 4, "%02x ", buf[i]);
	VIVO_TS_LOG_ERR("[%s]:RMI4 I2C writes %d bytes at %#06x: %s\n", __func__,
		len, addr, debug_buf);
#endif

	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
Retry:	
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		if (retry_times < I2C_RETRY_TIMES_LIMIT) {
			VIVO_TS_LOG_ERR("[%s]:i2c_master_send failed: %d. retry times %d", __func__, retval, retry_times);
			retry_times++;
			goto Retry;
		} else {
			VIVO_TS_LOG_ERR("[%s]:write block failed: %d. retry times %d", __func__, retval, retry_times);
			#ifdef CONFIG_FMEA
				fmea_notify("[SYNA]","rmi_i2c_write_block failed ,retry 3 times\n");	
			#endif
			retval = (retval < 0) ? retval : -EIO;
			goto exit;
		}
	}
#if 0
	if (retval < 0)
		phys->info.tx_errs++;
	else
		retval--; /* don't count the address byte */
#endif

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int rmi_i2c_write(struct rmi_phys_device *phys, u16 addr, u8 data)
{
	int retval;
	if(global_ddata != NULL)
	{
		mutex_lock(&global_ddata->syna_i2c_reset_mutex);
	}
	retval = rmi_i2c_write_block(phys, addr, &data, 1);
	if(global_ddata != NULL)
	{
		mutex_unlock(&global_ddata->syna_i2c_reset_mutex);
	}
	return (retval < 0) ? retval : 0;
}

static int rmi_i2c_read_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			      int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[1] = {addr & 0xff};
	int retval;
	int retry_times = 0;
#if	COMMS_DEBUG
	char debug_buf[len*3 + 1];
	char *temp = debug_buf;
	int i, n;
#endif
	if(qup_i2c_suspended > 0){
		VIVO_TS_LOG_ERR("[%s]:iic bus is suspend. so not iic transfer. \n", __func__);
		return -1;
	}

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

#if COMMS_DEBUG
	VIVO_TS_LOG_ERR("[%s]:RMI4 I2C writes 1 bytes: %02x\n", __func__, txbuf[0]);
#endif
	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
Retry:
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		if (retry_times < I2C_RETRY_TIMES_LIMIT) {
			VIVO_TS_LOG_ERR("[%s]:send addr i2c_master_send failed: %d. retry times %d", __func__, retval, retry_times);
			retry_times++;
			goto Retry;
		} else {
			VIVO_TS_LOG_ERR("[%s]:read block failed at send addr: %d. retry times %d", __func__, retval, retry_times);
			#ifdef CONFIG_FMEA
				fmea_notify("[SYNA]","rmi_i2c_read_block failed ,retry 3 times\n");	
			#endif
			retval = (retval < 0) ? retval : -EIO;
			goto exit;
		}
		//retval = (retval < 0) ? retval : -EIO;
		//goto exit;
	}

	retval = i2c_master_recv(client, buf, len);
	if (retval != len) {
		if (retry_times < I2C_RETRY_TIMES_LIMIT) {
			VIVO_TS_LOG_ERR("[%s]:i2c_master_recv failed: %d. retry times %d", __func__, retval, retry_times);
			retry_times++;
			goto Retry;
		} else {
			VIVO_TS_LOG_ERR("[%s]:read block failed at i2c_master_recv: %d. retry times %d", __func__, retval, retry_times);
			#ifdef CONFIG_FMEA
				fmea_notify("[SYNA]","rmi_i2c_read_block failed ,retry 3 times\n");	
			#endif
			retval = (retval < 0) ? retval : -EIO;
			goto exit;
		}
	}

	phys->info.rx_count++;
	phys->info.rx_bytes += len;
	if (retval < 0)
		phys->info.rx_errs++;
#if COMMS_DEBUG
	else {
		n = 0;
		for (i = 0; i < len; i++) {
			n = sprintf(temp, " %02x", buf[i]);
			temp += n;
		}
		VIVO_TS_LOG_ERR("[%s]:RMI4 I2C read %d bytes at %#06x:%s\n", __func__,
			len, addr, debug_buf);
	}
#endif

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int rmi_i2c_read(struct rmi_phys_device *phys, u16 addr, u8 *buf)
{
	int retval;
	if(global_ddata != NULL)
	{
		mutex_lock(&global_ddata->syna_i2c_reset_mutex);
	}
	retval = rmi_i2c_read_block(phys, addr, buf, 1);
	if(global_ddata != NULL)
	{
		mutex_unlock(&global_ddata->syna_i2c_reset_mutex);
	}
	return (retval < 0) ? retval : 0;
}

static int acquire_attn_irq(struct rmi_i2c_data *data)
{
	return request_threaded_irq(data->irq, NULL, rmi_i2c_irq_thread,
			data->irq_flags, dev_name(data->phys->dev), data->phys);
}

static int enable_device(struct rmi_phys_device *phys)
{
	int retval = 0;

	struct rmi_i2c_data *data = phys->data;

	if (data->enabled)
		return 0;
	VIVO_TS_LOG_INF("[%s]:SYNA Enter \n",__func__);

	retval = acquire_attn_irq(data);
	if (retval)
		goto error_exit;

	data->enabled = true;
	VIVO_TS_LOG_DBG("[%s]:Physical device enabled.\n", __func__);
	return 0;

error_exit:
	VIVO_TS_LOG_ERR("[%s]:Failed to enable physical device. Code=%d.\n", __func__,
		retval);
	return retval;
}

static void disable_device(struct rmi_phys_device *phys)
{
	struct rmi_i2c_data *data = phys->data;
	VIVO_TS_LOG_INF("[%s]:SYNA Enter \n",__func__);
	if (!data->enabled)
		return;

	disable_irq(data->irq);
	free_irq(data->irq, data->phys);

	VIVO_TS_LOG_DBG("[%s]:Physical device disabled.\n", __func__);
	data->enabled = false;
}

#ifdef CONFIG_OF
static int rmi_parse_dt(struct device *dev, struct rmi_device_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;
	const char *str_val = NULL;

	/* reset, irq gpio info */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "synaptics,reset-gpio",
				0, &pdata->reset_gpio_flags);
	pdata->attn_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio",
				0, &pdata->irq_gpio_flags);
	pdata->ldo_gpio = of_get_named_gpio_flags(np, "synaptics,ldo-gpio",
				0, &pdata->ldo_gpio_flags);
	rc = of_property_read_u32(np, "synaptics,attn_polarity", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read attn_polarity value\n", __func__);
	else if (rc != -EINVAL)
		pdata->attn_polarity = (u8) temp_val;
		
		
	//chenyunzhe add BEG-----------------------------------------------	
	rc = of_property_read_u32(np, "ts-suspend-resume", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-suspend-resume value\n",__func__);
	else if (rc != -EINVAL)
		pdata->suspend_resume_methods = temp_val;
		
	rc = of_property_read_u32(np, "ts-fixed-key-type", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-fixed-key-type value\n", __func__);
	else if (rc != -EINVAL)
		pdata->fixed_key_type = temp_val;
		
	rc = of_property_read_u32(np, "lcd-dimension-x", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read lcd-dimension-x value\n", __func__);
	else if (rc != -EINVAL)
		pdata->lcd_dimension_x =  temp_val;
	
	rc = of_property_read_u32(np, "lcd-dimension-y", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read lcd-dimension-y value\n", __func__);
	else if (rc != -EINVAL)
		pdata->lcd_dimension_y =  temp_val;
		
	rc = of_property_read_u32(np, "ts-dimension-x", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dimension-x value\n", __func__);
	else if (rc != -EINVAL)
		pdata->ts_dimension_x = temp_val;
		
	rc = of_property_read_u32(np, "ts-dimension-y", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dimension-y value\n", __func__);
	else if (rc != -EINVAL)
		pdata->ts_dimension_y =  temp_val;
		
	rc = of_property_read_u32(np, "ts-dclick-dimension-x-min", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dclick-dimension-x-min value\n", __func__);
	else if (rc != -EINVAL)
		pdata->dclick_dimension_min_x =  temp_val;
		
	rc = of_property_read_u32(np, "ts-dclick-dimension-x-max", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dclick-dimension-x-max value\n", __func__);
	else if (rc != -EINVAL)
		pdata->dclick_dimension_max_x =  temp_val;
		
	rc = of_property_read_u32(np, "ts-dclick-dimension-y-min", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dclick-dimension-y-min value\n", __func__);
	else if (rc != -EINVAL)
		pdata->dclick_dimension_min_y =  temp_val;
	
	rc = of_property_read_u32(np, "ts-dclick-dimension-y-max", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dclick-dimension-y-max value\n", __func__);
	else if (rc != -EINVAL)
		pdata->dclick_dimension_max_y =  temp_val;
		
	rc = of_property_read_string(np,"ts-virt-key",&str_val);
	if (rc) {
		VIVO_TS_LOG_ERR("[%s]:Unable to read ts-dclick-dimension-y-max value\n", __func__);
	} else {
	    pdata->virtual_key_string =  str_val;
	}
		
	VIVO_TS_LOG_INF("[%s]:srm(%d) fkt(%d) lcd(%d %d) ts(%d %d) dclk(%d %d %d %d)\n",__func__,
	    pdata->suspend_resume_methods,pdata->fixed_key_type,pdata->lcd_dimension_x,pdata->lcd_dimension_y,
		pdata->ts_dimension_x,pdata->ts_dimension_y,pdata->dclick_dimension_min_x,pdata->dclick_dimension_max_x,
		pdata->dclick_dimension_min_y,pdata->dclick_dimension_max_y);
		
	VIVO_TS_LOG_INF("[%s]:virt-key: %s\n",__func__,pdata->virtual_key_string);
	//chenyunzhe add END-----------------------------------------------	
	return 0;
}
#else
static int rmi_parse_dt(struct device *dev, struct rmi_device_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int rmi_power_on(struct rmi_device_platform_data *data)
{
	int rc;
#ifdef CONFIG_OF
	int error;
	if (gpio_is_valid(data->ldo_gpio)) {
		/* configure touchscreen reset out gpio */
		rc = gpio_request(data->ldo_gpio, "rmi4_ldo");
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]:unable to request gpio [%d]\n", __func__,
						data->ldo_gpio);
			return rc;
		}
		
		rc = gpio_direction_output(data->ldo_gpio, 1);
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction out to 0 for gpio [%d]\n", __func__,
				data->rst_gpio);
		}
    } else {
		data->vcc_ana = regulator_get(&data->client->dev, "vdd_ana");
		if (IS_ERR(data->vcc_ana)) {
			rc = PTR_ERR(data->vcc_ana);
			VIVO_TS_LOG_ERR("[%s]:Regulator get failed vcc_ana rc=%d\n", __func__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vcc_ana) > 0) {
			rc = regulator_set_voltage(data->vcc_ana, 3300000,
								3300000);
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]:regulator vcc_ana failed rc=%d\n", __func__, rc);
				regulator_put(data->vcc_ana);
				return rc;
			}
		}
		rc = regulator_enable(data->vcc_ana);
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]:Regulator vcc_ana enable failed rc=%d\n", __func__, rc);
			return rc;
		}
	}

	if (gpio_is_valid(data->rst_gpio)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(data->rst_gpio, "rmi4_rst");
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to request gpio [%d]\n", __func__,
						data->rst_gpio);
			return error;
		}
		
		error = gpio_direction_output(data->rst_gpio, 1);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction out to 0 for gpio [%d]\n", __func__,
				data->rst_gpio);
			goto err_reset_gpio_req;
		}

	}
#else
	data->vcc_ana = regulator_get(&data->client->dev, "vdd_ana");
	if (IS_ERR(data->vcc_ana)) {
		rc = PTR_ERR(data->vcc_ana);
		VIVO_TS_LOG_ERR("[%s]:Regulator get failed vcc_ana rc=%d\n", __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_ana) > 0) {
		rc = regulator_set_voltage(data->vcc_ana, 2950000,
							2950000);
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]:regulator vcc_ana failed rc=%d\n",__func__, rc);
			regulator_put(data->vcc_ana);
			return rc;
		}
	}
	rc = regulator_enable(data->vcc_ana);
	if (rc) {
		VIVO_TS_LOG_ERR("[%s]:Regulator vcc_ana enable failed rc=%d\n",__func__, rc);
		return rc;
	}
#endif
	//zhouyulin add for gyro power on
	//add end
	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		VIVO_TS_LOG_ERR("[%s]:Regulator get failed vcc_i2c rc=%d\n", __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, 1800000,
							1800000);
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]:regulator vcc_i2c failed rc=%d\n", __func__, rc);
			regulator_put(data->vcc_i2c);
			return rc;
		}
	}
	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		VIVO_TS_LOG_ERR("[%s]:Regulator vcc_i2c enable failed rc=%d\n", __func__, rc);
		return rc;
	}

	err_reset_gpio_req:
	if (gpio_is_valid(data->rst_gpio)){
		gpio_free(data->rst_gpio);
	}
	return error;
}

static int rmi_config_gpio(struct rmi_device_platform_data *pdata)
{
	int error;

	if (gpio_is_valid(pdata->attn_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->attn_gpio, "rmi4_attn");
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to request gpio [%d]\n", __func__,
						pdata->attn_gpio);
			return error;
		}
		error = gpio_direction_input(pdata->attn_gpio);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction for gpio [%d]\n", __func__,
				pdata->attn_gpio);
			goto err_irq_gpio_req;
		}
	} else {
		VIVO_TS_LOG_ERR("[%s]:irq gpio not provided\n", __func__);
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->rst_gpio)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(pdata->rst_gpio, "rmi4_rst");
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to request gpio [%d]\n", __func__,
						pdata->rst_gpio);
			return error;
		}
		
		error = gpio_direction_output(pdata->rst_gpio, 1);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction out to 0 for gpio [%d]\n", __func__,
				pdata->rst_gpio);
			goto err_reset_gpio_req;
		}
		msleep(10);		
		
		error = gpio_direction_output(pdata->rst_gpio, 0);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction out to 0 for gpio [%d]\n", __func__,
				pdata->rst_gpio);
			goto err_reset_gpio_req;
		}

		msleep(20);

		error = gpio_direction_output(pdata->rst_gpio, 1);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:unable to set direction out to 1 for gpio [%d]\n", __func__,
				pdata->rst_gpio);
			goto err_reset_gpio_req;
		}
		
		msleep(200);
	}

	return 0;

err_reset_gpio_req:
	if (gpio_is_valid(pdata->rst_gpio))
		gpio_free(pdata->rst_gpio);
err_irq_gpio_req:
	if (gpio_is_valid(pdata->attn_gpio))
		gpio_free(pdata->attn_gpio);
	return error;

}

static int rmi_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct rmi_phys_device *rmi_phys;
	struct rmi_i2c_data *data;
	struct rmi_device_platform_data *pdata = NULL; 
	int error;

	VIVO_TS_LOG_INF("[%s]:~~~~~~~~~~~~~~~~~~~~~~~~~~probe\n", __func__);
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct rmi_device_platform_data), GFP_KERNEL);
		if (!pdata) {
			VIVO_TS_LOG_ERR("[%s]:Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		error = rmi_parse_dt(&client->dev, pdata);
		if (error)
			return error;

		client->dev.platform_data = pdata;
		pdata->driver_name = devm_kzalloc(&client->dev, 16, GFP_KERNEL);
		if (!pdata->driver_name) {
			VIVO_TS_LOG_ERR("[%s]:Failed to allocate driver name memory\n", __func__);
			return -ENOMEM;
		}
		strcpy(pdata->driver_name, "rmi_generic");
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		VIVO_TS_LOG_ERR("[%s]:no platform data\n", __func__);
		return -EINVAL;
	}
	VIVO_TS_LOG_ERR("[%s]:Probing %s at %#02x (IRQ %d).\n", __func__,
		pdata->sensor_name ? pdata->sensor_name : "-no name-",
		client->addr, pdata->attn_gpio);

	pdata->client = client;

	/* add for compatible the old platform */
	if (!client->dev.of_node) {
		/* old driver power and gpio config */
		if (pdata->power_config) {
			VIVO_TS_LOG_INF("[%s]:Configuring Power.\n", __func__);
			pdata->power_config();
			VIVO_TS_LOG_INF("[%s]:End Configuring Power.\n", __func__);
		}	

		if (pdata->gpio_config) {
			VIVO_TS_LOG_INF("[%s]:Configuring GPIOs.\n", __func__);
			error = pdata->gpio_config(pdata, true);
			if (error < 0) {
				VIVO_TS_LOG_ERR("[%s]:Failed to configure attn_gpio, code: %d.\n", __func__,
					error);
				return error;
			}
		
			VIVO_TS_LOG_ERR("[%s]:Done with GPIO configuration.\n", __func__);
		}
	} else {
		error = rmi_power_on(pdata);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:Failed to power on\n", __func__);
			return error;
		}
		msleep(20);
		error = rmi_config_gpio(pdata);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:Failed to config gpio\n", __func__);
			return error;
		}
	}

	error = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!error) {
		VIVO_TS_LOG_ERR("[%s]:i2c_check_functionality error %d.\n", __func__,
			error);
		return error;
	}

	rmi_phys = kzalloc(sizeof(struct rmi_phys_device), GFP_KERNEL);
	if (!rmi_phys)
		return -ENOMEM;

	INIT_WORK(&rmi_phys->irq_work, rmi_irq_work_func);

	data = kzalloc(sizeof(struct rmi_i2c_data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_phys;
	}

	data->enabled = true;	/* We plan to come up enabled. */
	data->irq = gpio_to_irq(pdata->attn_gpio);
#if 0
	if (pdata->level_triggered) {
		data->irq_flags = IRQF_ONESHOT |
			((pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH) ? 
			IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW);
	} else {
		data->irq_flags = 
			(pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH) ?
			IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	}
#else
	data->irq_flags = IRQF_ONESHOT | IRQF_TRIGGER_FALLING;
#endif
	data->phys = rmi_phys;

	rmi_phys->data = data;
	rmi_phys->dev = &client->dev;

	rmi_phys->write = rmi_i2c_write;
	rmi_phys->write_block = rmi_i2c_write_block;
	rmi_phys->read = rmi_i2c_read;
	rmi_phys->read_block = rmi_i2c_read_block;
	rmi_phys->enable_device = enable_device;
	rmi_phys->disable_device = disable_device;

	rmi_phys->info.proto = phys_proto_name;

	mutex_init(&data->page_mutex);

	/* Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	error = rmi_set_page(rmi_phys, 0);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]:Failed to set page select to 0.\n", __func__);
		goto err_data;
	}

	error = rmi_register_phys_device(rmi_phys);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]:failed to register physical driver at 0x%.2X.\n", __func__,
			client->addr);
		goto err_gpio;
	}
	i2c_set_clientdata(client, rmi_phys);

	if (pdata->attn_gpio > 0) {
		error = acquire_attn_irq(data);
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:request_threaded_irq failed %d\n", __func__,
				pdata->attn_gpio);
			goto err_unregister;
		}
	}
	error = enable_irq_wake(data->irq);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]:set_irq_wake failed for gpio %d, "
			"irq %d\n", __func__, pdata->attn_gpio, data->irq);
		goto err_unregister;
	}

#if defined(CONFIG_RMI4_DEV)
	error = gpio_export(pdata->attn_gpio, false);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]:WARNING: Failed to export ATTN gpio!\n", __func__);
		error = 0;
	} else {
		error = gpio_export_link(&(rmi_phys->rmi_dev->dev), "attn",
					pdata->attn_gpio);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]:WARNING: Failed to symlink ATTN gpio!\n", __func__);
			error = 0;
		} else {
			VIVO_TS_LOG_INF("[%s]:Exported ATTN GPIO %d.", __func__,
				pdata->attn_gpio);
		}
	}
#endif /* CONFIG_RMI4_DEV */

	VIVO_TS_LOG_ERR("[%s]:registered rmi i2c driver at %#04x.\n", __func__,
			client->addr);

	VIVO_TS_LOG_ERR("[%s]:SYNA i2c_driver finished!\n", __func__);
	is_i2c_probe_done++;
	return 0;

err_unregister:
	rmi_unregister_phys_device(rmi_phys);
err_gpio:
	if (pdata->gpio_config)
		pdata->gpio_config(pdata,false);
err_data:
	kfree(data);
err_phys:
	kfree(rmi_phys);
	return error;
}

static int rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_phys_device *phys = i2c_get_clientdata(client);
	struct rmi_device_platform_data *pd = client->dev.platform_data;

	disable_device(phys);
	rmi_unregister_phys_device(phys);
	kfree(phys->data);
	kfree(phys);

	if (pd->gpio_config)
		pd->gpio_config(pd,false);

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi", 0 },
	{ "rmi_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

#ifdef CONFIG_OF
static struct of_device_id rmi_match_table[] = {
	{ .compatible = "synaptics_3202",},
	{ },
};
#else
#define rmi_match_table NULL
#endif


static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_i2c",
		.of_match_table = rmi_match_table,
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= rmi_i2c_remove,
};

extern unsigned int is_atboot;
static int __init rmi_i2c_init(void)
{

    if(!vivo_touchscreen_test_ic_in_use("synaptics_3202"))
	{
	    return 0;
	}
	
#if 0
#ifdef CONFIG_BBK_DRIVER_INFO
	char *board_version = get_bbk_board_version();
	if (!strcmp(board_version, "A1")) 
	 	return 0;
#endif
#endif
	VIVO_TS_LOG_ERR("[%s]:~~~~~~~~~~~~~~~~~~~~called\n", __func__);
	if(is_atboot == 1)
	  return 0;
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __exit rmi_i2c_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

late_initcall(rmi_i2c_init);
module_exit(rmi_i2c_exit);

/* use macro to do driver register */
//module_i2c_driver(rmi_i2c_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
