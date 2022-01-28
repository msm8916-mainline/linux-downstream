/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/wakelock.h>
#define HALL_TAG "Hall"

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int enable;
	int invalid;
	struct work_struct work;
	struct work_struct switch_work;
	struct wake_lock wakelock;
};

static unsigned long switch_enable = -1;

static void gpio_irq_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	state = gpio_get_value(data->gpio);
	printk(KERN_ERR "[%s]:[%s] state is %d \n", HALL_TAG, __func__, state);
	switch_set_state(&data->sdev, state);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
	wake_lock_timeout(&(switch_data->wakelock),3*HZ);
	printk(KERN_ERR "[%s]:[%s] irq had triggered \n", HALL_TAG, __func__);
	switch_data->invalid = 0;
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static void gpio_switch_work(struct work_struct *switch_work)
{
	unsigned long switch_state = switch_enable;
	int ret = -1;
	struct gpio_switch_data	*switch_data =
		container_of(switch_work, struct gpio_switch_data, switch_work);

	if(switch_state == 1){
		if(switch_data->enable == 1){
			return;
		}
		
		ret = request_irq( switch_data->irq, gpio_irq_handler,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, switch_data->sdev.name, switch_data);
		if (ret < 0){
			printk(KERN_ERR "[%s]:[%s] request irq %d err %d\n", HALL_TAG, __func__, switch_data->irq, ret);
			return;
		}
				
		ret = enable_irq_wake(switch_data->irq);
		if (ret < 0){
			printk(KERN_ERR "[%s]:[%s] enable irq %d err %d\n", HALL_TAG, __func__, switch_data->irq, ret);
			free_irq(switch_data->irq,switch_data);
			return;
		}
		
		if(switch_data->invalid == 0)
		{
			schedule_work(&switch_data->work);
		}else {
			printk(KERN_ERR "[%s]:[%s] hall is invalid %d \n", HALL_TAG, __func__, switch_data->invalid);
		}

		switch_data->enable = 1;
		printk(KERN_ERR "[%s]:[%s] switch enable is %d \n", HALL_TAG, __func__, switch_data->enable);
	}else {
		if(switch_data->enable == 0 || switch_data->enable == -1){
			return;
		}

		disable_irq_wake(switch_data->irq);
		disable_irq(switch_data->irq);
		free_irq(switch_data->irq,switch_data);
		
		switch_set_state(&switch_data->sdev, 1);
		switch_data->enable = 0;
		printk(KERN_ERR "[%s]:[%s] switch enable is %d \n", HALL_TAG, __func__, switch_data->enable);	
	}
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t length)
{
	unsigned long val;
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct gpio_switch_data *switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);

    if(!attr || !dev || !buf)
		return -EINVAL;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
		
	switch_enable = val;
	schedule_work(&switch_data->switch_work);
	
	return length;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct gpio_switch_data *switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);

	return sprintf(buf, "enable=%d\n", switch_data->enable);
}

static ssize_t invalid_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t length)
{
	unsigned long val;
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct gpio_switch_data *switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);

    if(!attr || !dev || !buf)
		return -EINVAL;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
		
	switch_data->invalid = val == 1 ? 1 : 0;
	printk(KERN_ERR "[%s]:[gpio_invalid_work] switch invalid is %d \n", HALL_TAG, switch_data->invalid);
	switch_set_state(&switch_data->sdev, 1);

	return length;
}

static ssize_t invalid_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);
	struct gpio_switch_data *switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);

	return sprintf(buf, "invalid=%d\n", switch_data->invalid);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUGO, enable_show, enable_store);
static DEVICE_ATTR(invalid, S_IRUGO | S_IWUGO, invalid_show, invalid_store);

static int gpio_switch_probe(struct platform_device *pdev)
{

	struct gpio_switch_data *switch_data;
	struct regulator *hall_vdd;
	int ret = 0;
#ifdef CONFIG_OF
	enum of_gpio_flags flags;	
#else
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	if (!pdata)
		return -EBUSY;
#endif
	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

#ifdef CONFIG_OF
	switch_data->sdev.name = "hall";
	switch_data->enable = -1;
	switch_data->invalid = 0;
	switch_data->gpio = of_get_named_gpio_flags(pdev->dev.of_node,"switch-hall,gpios",0,&flags);
#else
	switch_data->sdev.name = pdata->name;
	switch_data->enable = -1;
	switch_data->invalid = 0;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
#endif

	switch_data->sdev.print_state = switch_gpio_print_state;
	wake_lock_init(&(switch_data->wakelock), WAKE_LOCK_SUSPEND, "hall_lock");
    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = device_create_file(switch_data->sdev.dev, &dev_attr_enable);
	if (ret < 0)
		goto err_switch_dev_register;
		
	ret = device_create_file(switch_data->sdev.dev, &dev_attr_invalid);
	if (ret < 0)
		goto err_switch_dev_register;
	
	hall_vdd = regulator_get(&pdev->dev, "hall");
	if (!IS_ERR(hall_vdd)) {
		ret = regulator_enable(hall_vdd);
		if(ret) {
			dev_err(&pdev->dev, "Hall regulator_enable error ");
		}
	} else {
		dev_err(&pdev->dev, "Hall regulator_get error ");
	}
		
	ret = gpio_request(switch_data->gpio, switch_data->sdev.name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_irq_work);
	INIT_WORK(&switch_data->switch_work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}
#if 0
	ret = request_irq( switch_data->irq, gpio_irq_handler,
			 IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, switch_data->sdev.name, switch_data);
	if (ret < 0)
		goto err_request_irq;
	ret = enable_irq_wake(switch_data->irq);
	if (ret < 0)
		goto err_request_irq;
#endif 
	/* Perform initial detection */
	//gpio_irq_work(&switch_data->work);
	// init the state is 1
	switch_set_state(&switch_data->sdev, 1);

	return 0;

err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hall_match_table[] = {
	{ .compatible = "switch-hall",},
	{},
};
#endif

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= gpio_switch_remove,
	.driver		= {
		.name	= "switch-hall",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = hall_match_table,
#endif
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
