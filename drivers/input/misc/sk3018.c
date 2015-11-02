/* drivers/input/misc/cm36671.c - cm36671 optical sensors driver
 *
 * Copyright (C) 2014 Capella Microsystems Inc.
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
#include <linux/delay.h>
#include "earlysuspend.h"
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <asm/setup.h>

#define DRIVER_NAME "hall_sensor"
#if 1
#define PS_DBG(format,...) do{ printk("[SK3810]");\
						printk(format, ## __VA_ARGS__);	\
			}while(0)
#else
#define PS_DBG(format,...) do{ 
			}while(0)
#endif

#define SK3018_GPIO     108 
#define SK3018_NAME   "APX8132AI"
struct sk3018_info {
	struct class *sk3018_class;
	struct platform_device *pdev;
	struct mutex lock;
	struct device *ha_dev;
	int hallsensor_opened;
	struct input_dev *hall_input_dev;
	struct workqueue_struct *lp_wq;
	struct wake_lock hs_wake_lock;
	
	int sk3018_gpio_pin;
	int enable;
	int data;
	int sk3018_irq;

	struct early_suspend early_suspend;
	//struct workqueue_struct *lp_wq;

} *halls;

/*
static ssize_t sk3018_enable_show(struct device *dev,
				struct device_attribute *attr,char *buf)
{
	struct sk3018_info *lpi = dev_get_drvdata(dev);
	int enable;
	enable=lpi->enable;
	return sprintf(buf, "%d\n", enable);
}

static ssize_t sk3018_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sk3018_info *lpi = dev_get_drvdata(dev);
	//int enable;
	int en = -1;
	
	sscanf(buf, "%d", &en);

	if (en != 0 && en != 1)
		return -EINVAL;
	
	if(lpi->enable){
		if(en)
			return 0;
	       else{
	       	enable_irq(lpi->sk3018_irq);
	       	lpi->enable=en;
	       	}
		}
	else{
		if(en){
			disable_irq(lpi->sk3018_irq);
			lpi->enable=en;
			}
		else
		printk("the hall sensor already disabled!\n");
		}
	return count;
}

static ssize_t sk3018_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int gpio_value;
	gpio_value=gpio_get_value(SK3018_GPIO);
	return sprintf(buf, "%d\n", gpio_value);
	//return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, sk3018_enable_show, sk3018_enable_store);
static DEVICE_ATTR(Hdata, S_IRUGO, sk3018_data_show, NULL);

static struct attribute *hall_attributes[] = {
	&dev_attr_Hdata.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group hall_attribute_group = {
	.attrs = hall_attributes
};
*/
static irqreturn_t sk3018_irq_do_work(int irq, void *data)
{
    struct sk3018_info *lpi = data;
    int gpio_value;
    	gpio_value = gpio_get_value(lpi->sk3018_gpio_pin);
	printk("yhj add here for sk3018 interrupt gpio_value=%d\n",gpio_value);
	if(gpio_value){
		 /*hall near*/
	input_event(lpi->hall_input_dev, EV_KEY, KEY_SHOP, 1);
	input_sync(lpi->hall_input_dev);
	input_event(lpi->hall_input_dev, EV_KEY, KEY_SHOP, 0);
	input_sync(lpi->hall_input_dev);
    }
	else{
	    /*hall far*/
	input_event(lpi->hall_input_dev, EV_KEY, KEY_SPORT, 1);
	input_sync(lpi->hall_input_dev);
	input_event(lpi->hall_input_dev, EV_KEY, KEY_SPORT, 0);
	input_sync(lpi->hall_input_dev);

    }

	return IRQ_HANDLED;
}

static irqreturn_t sk3018_irq_handler(int irq, void *data)
{
	struct sk3018_info *lpi = data;
	PS_DBG("[sk3018]IRQ Handled for hall sensor interrupt: %d\n",lpi->sk3018_irq);

	
	return IRQ_WAKE_THREAD;
}


static int hall_setup(struct sk3018_info *lpi)
{
	int ret;

	lpi->hall_input_dev = input_allocate_device();
	if (!lpi->hall_input_dev) {
		pr_err(
			"[sk3018]%s: could not allocate hall input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->hall_input_dev->name = "APX8132AI";
	
	set_bit(EV_KEY, lpi->hall_input_dev->evbit);
	set_bit(KEY_SHOP, lpi->hall_input_dev->keybit);
	set_bit(KEY_SPORT, lpi->hall_input_dev->keybit);

	input_set_capability(lpi->hall_input_dev, EV_KEY, KEY_POWER);

	ret = input_register_device(lpi->hall_input_dev);
	if (ret < 0) {
		pr_err(
			"[sk3018 error]%s: could not register hall input device\n",
			__func__);
		goto err_free_hall_input_device;
	}

	return ret;

err_free_hall_input_device:
	input_free_device(lpi->hall_input_dev);
	return ret;
}

static int sk3018_setup(struct sk3018_info *lpi)
{
	int ret = 0;
	msleep(5);
	
	ret = gpio_request(lpi->sk3018_gpio_pin, "gpio_sk3018_pin");
	if (ret < 0) {
		pr_err("[sk3018 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->sk3018_gpio_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->sk3018_gpio_pin);
	if (ret < 0) {
		pr_err(
			"[sk3018 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->sk3018_gpio_pin, ret);
		goto fail_free_gpio_pin;
	}
	
	lpi->sk3018_irq= gpio_to_irq(lpi->sk3018_gpio_pin);

	PS_DBG("[sk3018 ]sk3018_irq= %d the gpio is %d\n", lpi->sk3018_irq,lpi->sk3018_gpio_pin);
	
	ret = request_threaded_irq(lpi->sk3018_irq,
		sk3018_irq_handler, sk3018_irq_do_work, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
				   "APX8132AI", lpi);

	if (ret) {
		PS_DBG("[sk3018 error]Failed to request IRQ: %d\n", ret);
		}
	enable_irq_wake(lpi->sk3018_irq);
	//INIT_DELAYED_WORK(&lpi->delay_work, sk3018_poll);

	return ret;

fail_free_gpio_pin:
	gpio_free(lpi->sk3018_gpio_pin);
	return ret;
}

static void sk3018_early_suspend(struct early_suspend *h)
{
	/*
	if(lpi->enable) {
		enable=0;
		disable_irq(sk3018_irq);
		}*/
	return ;
}
static void sk3018_late_resume(struct early_suspend *h)
{
	/*if(enable!=1){
		enable=1;
		enable_irq(sk3018_irq);
		}
		*/
	return ;
}

static int sk3018_probe(struct platform_device *pdev)
{
	int ret=0 ;
	struct sk3018_info *lpi;
	struct device_node *np = pdev->dev.of_node;
	printk("////yhj add here sk3018 probe start  ...\n");
	if(!np) {
		printk(KERN_ERR "np error!\n");
		return -ENOMEM;
	}
	lpi = kzalloc(sizeof(struct sk3018_info), GFP_KERNEL);
	
	if (!lpi)
		return -ENOMEM;
	
	ret = of_get_named_gpio_flags(np, "sk3018,interrupt-gpio",
			0, NULL);
	if (ret < 0) {
		printk("Unable to read interrupt pin number\n");
		return ret;
	} else {
		lpi->sk3018_gpio_pin = ret;
		printk(KERN_ERR "of_get_gpio failed: gpio=%d \n", lpi->sk3018_gpio_pin);
	}
	
	/*
	lpi->sk3018_gpio_pin = of_get_gpio(np, 0);
	printk("////yhj add here for hall gpio=%d\n",lpi->sk3018_gpio_pin );
	if (lpi->sk3018_gpio_pin < 0 ) {
		printk(KERN_ERR "of_get_gpio failed: gpio=%d \n", lpi->sk3018_gpio_pin);
		ret = -EINVAL;
		goto error;
	}
	*/

	ret=hall_setup(lpi);
	if (ret < 0) {
		pr_err("[sk3018 error]%s: hall_setup error!!\n",
			__func__);
		return ret;
	}
/*
	lpi->lp_wq = create_singlethread_workqueue("sk3018_wq");
	if (!lpi->lp_wq) {
		pr_err("[SK3018 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "hall_sensor");
*/	
	ret=sk3018_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][sk3018 error]%s: sk3018_setup error!!\n",
			__func__);
		goto err_sys_init;
	}
/*
	ret = sysfs_create_group(&pdev->dev.kobj, &hall_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs can not create group\n");
		goto err_sys_init;
	}
*/
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = sk3018_early_suspend;
	lpi->early_suspend.resume = sk3018_late_resume;
	register_early_suspend(&lpi->early_suspend);
	
	printk("////yhj add here sk3018 probe success.\n");
	return ret;

//error:
//	kfree(lpi);
//err_create_singlethread_workqueue:
//	destroy_workqueue(lpi->lp_wq);
err_sys_init:
	free_irq(lpi->sk3018_irq, lpi);
	gpio_free(lpi->sk3018_gpio_pin);
	return ret;
	
}

static int sk3018_remove(struct platform_device *pdev)
{
	struct sk3018_info *lpi = platform_get_drvdata(pdev);
	int irq = gpio_to_irq(lpi->sk3018_gpio_pin);

	input_unregister_device(lpi->hall_input_dev);
	free_irq(irq, lpi);
	gpio_free(lpi->sk3018_gpio_pin);
	kfree(lpi);
	return 0;
}


static const struct of_device_id hall_sensor_of_match[] = {
	{ .compatible = "APX8132AI", },
	{ },
};


static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = SK3018_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hall_sensor_of_match,
		},
	.probe = sk3018_probe,
	.remove = sk3018_remove,
};

module_platform_driver(hall_sensor_driver);
MODULE_LICENSE("GPL v2");

MODULE_DESCRIPTION("APX8132AI Driver");

