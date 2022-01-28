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

#include <linux/wakelock.h>


struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	struct wake_lock wakelock;
};

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	state = gpio_get_value(data->gpio);
	printk(KERN_ERR "Gsensor.I %s read gpio_value = %d\n", __func__, state);
	switch_set_state(&data->sdev, state);
}

/*laizhilong add for step counter start*/
static struct gpio_switch_data *switch_dt=NULL;

void gpio_switch_setstate(int state)
{
     if(switch_dt)switch_set_state(&switch_dt->sdev, state);
}
EXPORT_SYMBOL_GPL(gpio_switch_setstate);
/*laizhilong add for step counter end*/


static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
	wake_lock_timeout(&(switch_data->wakelock),3*HZ);
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
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

/*laizhilong add start*/
#ifdef CONFIG_OF
static int gsensor_get_devtree_pdata(struct device *dev,
			    struct gpio_switch_data *pdata)
{
	struct device_node *node;
	int rc=-1;
	enum of_gpio_flags flags;

    if(!pdata || !dev)
		return -EINVAL;
    
	node=dev->of_node;
	if (node == NULL)
		return -ENODEV;

    dev_err(NULL, "%s\n", __func__);
	memset(pdata, 0, sizeof *pdata);

    if(of_property_read_bool(dev->of_node,"sensor,gpio-irq"))
    {
	rc = of_get_named_gpio_flags(dev->of_node,
				"sensor,gpio-irq", 0, &flags);
	if (rc < 0) {
		printk(KERN_ERR "%s read interrupt int pin error %d\n", __func__, rc);
	} 
	pdata->gpio= rc;
	}
	else
	{
        pdata->gpio= -2;
	}	

	pdata->sdev.name = of_get_property(node, "sensor-name", NULL);
	dev_err(NULL, "%s %s\n", __func__, pdata->sdev.name);
	if(pdata->sdev.name)
	     return 0;
	else
		 return -ENODATA;
}

static struct of_device_id gsensor_of_match[] = {
	{ .compatible = "vivo-gsensor", },
	{ },
};
MODULE_DEVICE_TABLE(of, gsensor_of_match);
#else
#define gsensor_of_match NULL
#endif
/*laizhilong add end*/

static int gpio_switch_probe(struct platform_device *pdev)
{
    #ifndef CONFIG_OF
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	#endif
	struct gpio_switch_data *switch_data;
	int ret = 0;

	dev_err(NULL, "%s \n", __func__);/*laizhilong debug*/
	#ifndef CONFIG_OF
	if (!pdata)
		return -EBUSY;
	#endif
	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

    /*laizhilong modified start*/
	#ifdef CONFIG_OF
	ret = gsensor_get_devtree_pdata(&pdev->dev , switch_data);
	if (ret)
		goto err_switch_dev_register;
	#else
	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	#endif
	switch_data->sdev.print_state = switch_gpio_print_state;
	
	wake_lock_init(&(switch_data->wakelock), WAKE_LOCK_SUSPEND, "gsensor_lock");
    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	#ifdef CONFIG_OF
    /*laizhilong modified for step counter start*/
    if(0>((int)switch_data->gpio))
    {
        switch_dt=switch_data;
        return 0;
    }
    /*laizhilong modified for step counter start*/
	#endif
	/*laizhilong modified end*/
	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;
	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}
	ret = request_irq( switch_data->irq, gpio_irq_handler,
			 IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;
	ret = enable_irq_wake(switch_data->irq);
	if (ret < 0)
		goto err_request_irq;
	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);
	dev_err(NULL, "%s haha\n", __func__);/*laizhilong debug*/
	return 0;

err_request_irq:
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

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= gpio_switch_remove,
	.driver		= {
		.name	= "switch-gsensor-int1",
		.owner	= THIS_MODULE,
		.of_match_table = gsensor_of_match,
	},
};

static int __init gpio_switch_init(void)
{dev_err(NULL, "%s \n", __func__);/*laizhilong debug*/
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
