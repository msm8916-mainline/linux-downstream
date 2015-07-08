/*******************************************************************************************
* Copyright 2014 xxx Corporation. All rights reserved.
*
* Unless you and xxx execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this software
* in any way with any other Broadcom software provided under a license other than
* the GPL, without xxx's express prior written consent.
* *******************************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/hall_sensor.h>


struct hall_dev
{
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	unsigned int timer_debounce;
	unsigned int irq;

	struct hall_platform_data data;
};

static void hall_gpio_work_func(struct work_struct *work)
{
	struct hall_dev *hall_dev = container_of(work, struct hall_dev, work);
	struct hall_platform_data *pdata = &hall_dev->data;
	struct input_dev *input = hall_dev->input;
	unsigned int type = pdata->input_type;
	int state = gpio_get_value_cansleep(pdata->int_gpio) ? 1 : 0;
	
	if (state == !!pdata->active_gpio_level) {
		input_event(input, type, pdata->active_code, 1);
		input_sync(input);
		input_event(input, type, pdata->active_code, 0);
		input_sync(input);
		pr_info("%s,state:%d,active_code:0x%x\n",__func__,state,pdata->active_code);
	} else {
		input_event(input, type, pdata->inactive_code, 1);
		input_sync(input);
		input_event(input, type, pdata->inactive_code, 0);
		input_sync(input);
		pr_info("%s,state:%d,inactive_code:0x%x\n",__func__,state,pdata->inactive_code);
	}
}

static void hall_gpio_timer(unsigned long _data)
{
	struct hall_dev *hall_dev = (struct hall_dev *)_data;
	
	schedule_work(&hall_dev->work);
}

static irqreturn_t hall_gpio_isr(int irq, void *dev_id)
{
	struct hall_dev *hall_dev = dev_id;
	pr_err("%s\n",__func__);

	BUG_ON(irq != hall_dev->irq);

	if (hall_dev->timer_debounce)
		mod_timer(&hall_dev->timer,	jiffies + msecs_to_jiffies(hall_dev->timer_debounce));
	else
		schedule_work(&hall_dev->work);

	return IRQ_HANDLED;
}


static int hall_parse_dt(struct device *dev, struct hall_platform_data *pdata)
{
	//int rc = 0;
	struct device_node *np = dev->of_node;
	unsigned int val = 0;

	pdata->name = of_get_property(np, "input-name", NULL);

	pdata->int_gpio = of_get_named_gpio(np, "hall,int-gpio", 0);
	if ((!gpio_is_valid(pdata->int_gpio))) {
		pr_err("%s: gpio_is_valid for %d failed\n",
			__func__, pdata->int_gpio);
		return -EINVAL;
	}

	/*	
	rc = gpio_tlmm_config(GPIO_CFG(pdata->int_gpio, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, pdata->int_gpio);
		return -EINVAL;
	}*/

	if (of_property_read_u32(np, "hall,input-type", &val) == 0)
		pdata->input_type = val;
	else
		pdata->input_type = EV_KEY;

	if (of_property_read_u32(np, "hall,active-lvl", &val) == 0)
		pdata->active_gpio_level = val;
	else
		pdata->active_gpio_level = 0;

	if (of_property_read_u32(np, "hall,active_code", &val) == 0)
		pdata->active_code = val;
	else
		pdata->active_code = 115; //vol+ for test

	if (of_property_read_u32(np, "hall,inactive_code", &val) == 0)
		pdata->inactive_code = val;
	else
		pdata->inactive_code = 114; //vol- for test

	if (of_property_read_u32(np, "hall,debounce-interval", &val) == 0)
		pdata->debounce_interval = val;
	else
		pdata->debounce_interval = 20;

	pr_err("%s int_gpio=%d,active_lvl=%d,active_code=%d,inactive_code=%d,debounce_interval=%d\n",
		__func__, pdata->int_gpio,pdata->active_gpio_level,pdata->active_code,pdata->inactive_code,pdata->debounce_interval);

	return 0;
}


static ssize_t hall_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int state;
	struct hall_platform_data *data = dev_get_drvdata(dev);
	if (NULL == data) return -ENODEV;

	state = gpio_get_value(data->int_gpio);
	
	if (!!state == data->active_gpio_level)
		return sprintf(buf, "hall_state: active\n");
	else
		return sprintf(buf, "hall_state: inactive\n");
}

static DEVICE_ATTR(state, S_IRUGO|S_IWUSR, hall_state_show, NULL);

static struct attribute *hall_attrs[] = {
	&dev_attr_state.attr,
	NULL
};
static struct attribute_group hall_attr_group = {
	.attrs = hall_attrs,
};

static int  hall_probe(struct platform_device *pdev)
{
	int err = -ENOMEM;
	struct hall_dev *hall_dev;
	struct input_dev *input_dev;
	struct hall_platform_data *platform_data;
	int irq;
	struct device *dev;

	printk("%s\n",__func__);

	if (pdev->dev.of_node) {
		platform_data = devm_kzalloc(&pdev->dev,
			sizeof(struct hall_platform_data), GFP_KERNEL);
		if (NULL == platform_data) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = hall_parse_dt(&pdev->dev, platform_data);
		if (err) {
			pr_err("hall_parse_dt failed.\n");
			return err;
		}
	} else {
		platform_data = pdev->dev.platform_data;
	}

	if (NULL == platform_data) {
		pr_err("%s fail.\n", __func__);
		return -ENODEV;
	}
	
	hall_dev = kzalloc(sizeof(struct hall_dev), GFP_KERNEL);
	if (NULL == hall_dev) {
		dev_err(&pdev->dev,
			"failed to allocate memory\n");
		return -ENOMEM;
	}

	hall_dev->data.name = platform_data->name;
	hall_dev->data.int_gpio = platform_data->int_gpio;
	hall_dev->data.input_type = platform_data->input_type;
	hall_dev->data.active_gpio_level = platform_data->active_gpio_level;
	hall_dev->data.active_code = platform_data->active_code;
	hall_dev->data.inactive_code = platform_data->inactive_code;
	hall_dev->data.debounce_interval = platform_data->debounce_interval;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto fail;

	hall_dev->input = input_dev;

	platform_set_drvdata(pdev, hall_dev);
	
	input_set_drvdata(input_dev, hall_dev);
	
	input_dev->name = hall_dev->data.name;
	input_dev->phys = "hall/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &pdev->dev;

	__set_bit(hall_dev->data.input_type, input_dev->evbit);
	__set_bit(hall_dev->data.active_code, input_dev->keybit);
	__set_bit(hall_dev->data.inactive_code, input_dev->keybit);

	if (gpio_is_valid(hall_dev->data.int_gpio)) {
		err = gpio_request(hall_dev->data.int_gpio, "hall-int");
		if (err < 0) {
			pr_err("Failed to request GPIO %d, error %d\n",hall_dev->data.int_gpio,err);
			goto fail_allocate;
		}
		
		err = gpio_direction_input(hall_dev->data.int_gpio);
		if (err < 0) {
			pr_err("Failed to configure direction for GPIO %d, error %d\n",
				hall_dev->data.int_gpio,err);
			goto fail_allocate;
		}

		if (hall_dev->data.debounce_interval) {
			err = gpio_set_debounce(hall_dev->data.int_gpio,
									hall_dev->data.debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (err < 0)
				hall_dev->timer_debounce = hall_dev->data.debounce_interval; 
		}

		irq = gpio_to_irq(hall_dev->data.int_gpio);
		if (irq < 0) {
			err = irq;
			pr_err("Unable to get irq number for GPIO %d, error %d\n",
					hall_dev->data.int_gpio, err);
			goto fail_allocate;
		}

		hall_dev->irq = irq;
		
		INIT_WORK(&hall_dev->work, hall_gpio_work_func);
		setup_timer(&hall_dev->timer,
				hall_gpio_timer, (unsigned long)hall_dev);

		err = request_any_context_irq(hall_dev->irq, hall_gpio_isr, 
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "hall_int", hall_dev);
		if (err < 0) {
			pr_err("Unable to claim irq %d; error %d\n",irq, err);
			goto fail_gpio;
		}
		
	} else {
		pr_err("gpio_is_valid %d failed\n",hall_dev->data.int_gpio);
		goto fail_allocate;
	}

	err = input_register_device(input_dev);
	if (err)
		goto fail_gpio;

	input_sync(input_dev);
	device_init_wakeup(&pdev->dev, 1);

	dev = get_device(&hall_dev->input->dev);
	dev_set_drvdata(dev, &hall_dev->data);
	
	err = sysfs_create_group(&dev->kobj, &hall_attr_group);
	if(err)	{
		printk("failed to create hall sysfs.\n");
		goto fail_gpio;
	}	

	irq_set_irq_wake(hall_dev->irq, 1);

	printk("end %s\n",__func__);
	return 0;

fail_gpio:
	if (hall_dev->timer_debounce)
		del_timer_sync(&hall_dev->timer);
	cancel_work_sync(&hall_dev->work);
	if (gpio_is_valid(hall_dev->data.int_gpio))
		gpio_free(hall_dev->data.int_gpio);
fail_allocate:
	input_free_device(input_dev);
fail:
	if (hall_dev) {
		kfree(hall_dev);
		hall_dev = NULL;
	}
	return err;
}

static int hall_remove(struct platform_device *pdev)
{
	struct hall_dev *hall_dev = platform_get_drvdata(pdev);
	
	device_init_wakeup(&pdev->dev, 0);
	
	free_irq(hall_dev->irq, hall_dev);
	if (hall_dev->timer_debounce)
		del_timer_sync(&hall_dev->timer);
	cancel_work_sync(&hall_dev->work);
	if (gpio_is_valid(hall_dev->data.int_gpio))
		gpio_free(hall_dev->data.int_gpio);
	input_unregister_device(hall_dev->input);
	
	kfree(hall_dev);

	return 0;
}

static struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,hall"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

static struct platform_driver hall_device_driver = {
	.probe		= hall_probe,
	.remove		= hall_remove,
	.driver		= {
		.name	= "hall",
		.owner	= THIS_MODULE,
		.of_match_table = msm_match_table,
	}
};

module_platform_driver(hall_device_driver);

MODULE_AUTHOR("Jane<jane.zhangjiajuan@gmail.com>");
MODULE_DESCRIPTION("Hall sensor input driver");
MODULE_SUPPORTED_DEVICE("hall");
MODULE_LICENSE("GPL");
MODULE_VERSION("v0.1");
