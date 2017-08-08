/*
 * Copyright LG Electronics (c) 2011
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/two-hall-ic.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#ifdef CONFIG_64BIT
#include <soc/qcom/lge/board_lge.h>
#else
#include <mach/board_lge.h>
#endif

#define COVER_DETECT_DELAY 100

struct pm8xxx_cradle {
	struct switch_dev sdev;
	struct switch_dev hani_sdev;
	struct delayed_work left_work;
	struct delayed_work right_work;
	struct device *dev;
	struct wake_lock wake_lock;
	const struct pm8xxx_cradle_platform_data *pdata;
	spinlock_t lock;
	int state;
	int hall_state;
	int left;
	int right;
};


static struct workqueue_struct *cradle_wq;
static struct pm8xxx_cradle *cradle;

bool cradle_boot_mode(void)
{
	enum lge_boot_mode_type boot_mode;
	boot_mode = lge_get_boot_mode();
	if (boot_mode == LGE_BOOT_MODE_QEM_56K || boot_mode == LGE_BOOT_MODE_QEM_130K)
	{
		printk("[HALL] boot_mode == 56K || 130K\n");
		return 1;
	}
	printk("[HALL] boot_mode ==%d\n", boot_mode);
	return 0;
}

static void boot_cradle_factory_func(void)
{
	int hall_state = 0;
	unsigned long flags;
	int tmp_left = 0;
	int tmp_right = 0;

	spin_lock_irqsave(&cradle->lock, flags);

	if (cradle->pdata->hallic_left_detect_pin)
		tmp_left = !gpio_get_value(cradle->pdata->hallic_left_detect_pin);
	if (cradle->pdata->hallic_right_detect_pin)
		tmp_right = !gpio_get_value(cradle->pdata->hallic_right_detect_pin);

	if (tmp_left == 1 && tmp_right == 1)
		hall_state = SMARTCOVER_FATORY_CLOSED;
	else if (tmp_left == 0 && tmp_right == 1)
		hall_state = SMARTCOVER_FATORY_RIGHT;
	else if (tmp_left == 1 && tmp_right == 0)
		hall_state = SMARTCOVER_FATORY_LEFT;
	else
		hall_state = SMARTCOVER_FATORY_OPENED;

	if (cradle->hall_state != hall_state) {
		cradle->hall_state = hall_state;
		spin_unlock_irqrestore(&cradle->lock, flags);
		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->hani_sdev, cradle->hall_state);
		printk("%s : [HALL] Cradle value is %d\n", __func__ , hall_state);
	}
	else {
		spin_unlock_irqrestore(&cradle->lock, flags);
		printk("%s : [HALL] Cradle value is %d (no change)\n", __func__ , hall_state);
	}
}

static void boot_cradle_det_func(void)
{
	int state;

	if (cradle->pdata->hallic_left_detect_pin)
		cradle->left = !gpio_get_value(cradle->pdata->hallic_left_detect_pin);


	if (cradle->pdata->hallic_right_detect_pin)
		cradle->right = !gpio_get_value(cradle->pdata->hallic_right_detect_pin);


	if(cradle->left == 1)
		state = SMARTCOVER_BOTTOM_CLOSED;
	else if(cradle->right == 1)
		state = SMARTCOVER_TOP_CLOSED;
	else if(cradle->left == 0 || cradle->right == 0)
		state = SMARTCOVER_OPENED;
	
	printk("%s : [HALL] boot cover value is %d\n", __func__ , state);

	cradle->state = state;
	cradle->hall_state = state;
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->sdev, cradle->state);
	switch_set_state(&cradle->hani_sdev, cradle->hall_state);
}

static void pm8xxx_cover_work_func(struct work_struct *work)
{
	int state = 0;
	int hall_state = 0;
	unsigned long flags;

	spin_lock_irqsave(&cradle->lock, flags);

	if (cradle->pdata->hallic_left_detect_pin)
		cradle->left = !gpio_get_value(cradle->pdata->hallic_left_detect_pin);

	if (cradle->pdata->hallic_right_detect_pin)
		cradle->right = !gpio_get_value(cradle->pdata->hallic_right_detect_pin);


	if (cradle->left == 1 && cradle->right == 1) {
		hall_state = SMARTCOVER_FATORY_CLOSED;
		state = cradle->state;
	}
	else if (cradle->left == 0 && cradle->right == 1) {
		hall_state = SMARTCOVER_FATORY_RIGHT;
		state = SMARTCOVER_TOP_CLOSED;
	}
	else if (cradle->left == 1 && cradle->right == 0) {
		hall_state = SMARTCOVER_FATORY_LEFT;
		state = SMARTCOVER_BOTTOM_CLOSED;
	}
	else {
		hall_state = SMARTCOVER_FATORY_OPENED;
		state = SMARTCOVER_OPENED;
	}

	if (cradle->hall_state != hall_state) {
		cradle->hall_state = hall_state;
		cradle->state = state;
		spin_unlock_irqrestore(&cradle->lock, flags);
		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->hani_sdev, cradle->hall_state);
		switch_set_state(&cradle->sdev, cradle->state);
		printk("%s : [HALL] cover value is %d\n", __func__ , state);
	}
	else {
		spin_unlock_irqrestore(&cradle->lock, flags);
		printk("%s : [HALL] cover value is %d (no change)\n", __func__ , state);
	}

}

static irqreturn_t pm8xxx_left_irq_handler(int irq, void *handle)
{
	struct pm8xxx_cradle *cradle_handle = handle;
	int v = 0;
	printk("left irq!!!!\n");
	v = 1 + 1*(!gpio_get_value(cradle->pdata->hallic_left_detect_pin));
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(COVER_DETECT_DELAY*v+5));
	queue_delayed_work(cradle_wq, &cradle_handle->left_work, msecs_to_jiffies(COVER_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}

static irqreturn_t pm8xxx_right_irq_handler(int irq, void *handle)
{
	struct pm8xxx_cradle *cradle_handle = handle;
	int v = 0;
	printk("right irq!!!!\n");
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(COVER_DETECT_DELAY*v+5));
	queue_delayed_work(cradle_wq, &cradle_handle->right_work, msecs_to_jiffies(COVER_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}

static ssize_t
cradle_left_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_cradle *cradle = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "left : %d\n", cradle->left);

	return len;
}

static ssize_t
cradle_right_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_cradle *cradle = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "right : %d\n", cradle->right);

	return len;
}

static ssize_t
cradle_sensing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_cradle *cradle = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "sensing(cradle state) : %d\n", cradle->state);

	return len;
}


static struct device_attribute cradle_sensing_attr = __ATTR(sensing, S_IRUGO, cradle_sensing_show, NULL);
static struct device_attribute cradle_left_attr   = __ATTR(left, S_IRUGO, cradle_left_show, NULL);
static struct device_attribute cradle_right_attr   = __ATTR(right, S_IRUGO, cradle_right_show, NULL);

static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:
		return sprintf(buf, "OPEN\n");
	case 1:
		return sprintf(buf, "BOTTOM_CLOSE\n");
	case 2:
		return sprintf(buf, "TOP_CLOSE\n");
	}
	return -EINVAL;
}



static void bu52061nvx_parse_dt(struct device *dev,
		struct pm8xxx_cradle_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if ((pdata->hallic_left_detect_pin = of_get_named_gpio_flags(np, "hallic-left-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_left_irq = gpio_to_irq(pdata->hallic_left_detect_pin);

	printk("[Hall IC] hallic_left_gpio: %d\n", pdata->hallic_left_detect_pin);

	if ((pdata->hallic_right_detect_pin = of_get_named_gpio_flags(np, "hallic-right-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_right_irq = gpio_to_irq(pdata->hallic_right_detect_pin);

	printk("[Hall IC] hallic_right_gpio: %d\n", pdata->hallic_right_detect_pin);

	pdata->irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
}


static int pm8xxx_cradle_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int hall_left_gpio_irq = 0, hall_right_gpio_irq = 0;

	struct pm8xxx_cradle_platform_data *pdata;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct pm8xxx_cradle_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;

		bu52061nvx_parse_dt(&pdev->dev, pdata);

	} else {
		pdata = pdev->dev.platform_data;
	}
	if (!pdata) {
		pr_err("%s: no pdata\n", __func__);
		return -ENOMEM;
	}

	cradle = kzalloc(sizeof(*cradle), GFP_KERNEL);
	if (!cradle)
		return -ENOMEM;

	cradle->pdata	= pdata;

	cradle->sdev.name = "smartcover";
	cradle->hani_sdev.name = "hallstate";
	cradle->sdev.print_name = cradle_print_name;
	cradle->left = 0;
	cradle->right = 0;
	cradle->state = 0;
	cradle->hall_state = 0;

	spin_lock_init(&cradle->lock);

	ret = switch_dev_register(&cradle->sdev);
	ret = switch_dev_register(&cradle->hani_sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	
	wake_lock_init(&cradle->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

	INIT_DELAYED_WORK(&cradle->left_work, pm8xxx_cover_work_func);
	INIT_DELAYED_WORK(&cradle->right_work, pm8xxx_cover_work_func);

	printk("%s : [HALL] init work_func\n", __func__);

	/* initialize irq of gpio_hall */
	if (cradle->pdata->hallic_left_detect_pin > 0) {
		hall_left_gpio_irq = gpio_to_irq(cradle->pdata->hallic_left_detect_pin);
		printk("%s : [HALL] hall_left_gpio_irq = [%d]\n", __func__, hall_left_gpio_irq);
		if (hall_left_gpio_irq < 0) {
			printk("Failed : LEFT GPIO TO IRQ \n");
			ret = hall_left_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(hall_left_gpio_irq, pm8xxx_left_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, cradle);
		if (ret > 0) {
			printk(KERN_ERR "%s: Can't allocate irq %d, ret %d\n", __func__, hall_left_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_left_gpio_irq) == 0)
			printk("%s : [HALL]left enable_irq_wake Enable(1)\n",__func__);
		else
			printk("%s : [HALL]left enable_irq_wake failed(1)\n",__func__);
	}

	if (cradle->pdata->hallic_right_detect_pin > 0) {
		hall_right_gpio_irq = gpio_to_irq(cradle->pdata->hallic_right_detect_pin);
		printk("%s : [HALL] hall_right_gpio_irq = [%d]\n", __func__, hall_right_gpio_irq);
		if (hall_right_gpio_irq < 0) {
			printk("Failed : RIGHT GPIO TO IRQ \n");
			ret = hall_right_gpio_irq;
			goto err_request_irq;
		}
		ret = request_irq(hall_right_gpio_irq, pm8xxx_right_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, cradle);
		if (ret > 0) {
			printk(KERN_ERR "%s: Can't allocate irq %d, ret %d\n", __func__, hall_right_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_right_gpio_irq) == 0)
			printk("%s : [HALL]right enable_irq_wake Enable(2)\n",__func__);
		else
			printk("%s : [HALL]right enable_irq_wake failed(2)\n",__func__);
	}
	//printk("%s : [HALL] pdata->irq_flags = [%d]\n", __func__,(int)pdata->irq_flags);

	printk("%s : [HALL]boot_cradle_det_func START\n",__func__);

	if (cradle_boot_mode()) {
		boot_cradle_factory_func();
	}else
		boot_cradle_det_func();

	ret = device_create_file(&pdev->dev, &cradle_sensing_attr);
	if (ret)
		goto err_request_irq;

	if (cradle->pdata->hallic_left_detect_pin > 0) {
		ret = device_create_file(&pdev->dev, &cradle_left_attr);
		if (ret)
			goto err_request_irq;
	}
	if (cradle->pdata->hallic_right_detect_pin > 0) {
		ret = device_create_file(&pdev->dev, &cradle_right_attr);
		if (ret)
			goto err_request_irq;
	}

	platform_set_drvdata(pdev, cradle);
	return 0;

err_request_irq:
	if (hall_left_gpio_irq)
		free_irq(hall_left_gpio_irq, 0);
	if (hall_right_gpio_irq)
		free_irq(hall_right_gpio_irq, 0);

err_switch_dev_register:
	switch_dev_unregister(&cradle->sdev);
	kfree(cradle);
	return ret;
}

static int pm8xxx_cradle_remove(struct platform_device *pdev)
{
	struct pm8xxx_cradle *cradle = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&cradle->left_work);
	cancel_delayed_work_sync(&cradle->right_work);
	switch_dev_unregister(&cradle->sdev);
	platform_set_drvdata(pdev, NULL);
	kfree(cradle);

	return 0;
}

static int pm8xxx_cradle_suspend(struct device *dev)
{
	return 0;
}

static int pm8xxx_cradle_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pm8xxx_cradle_pm_ops = {
	.suspend = pm8xxx_cradle_suspend,
	.resume = pm8xxx_cradle_resume,
};

#ifdef CONFIG_OF
static struct of_device_id bu52061nvx_match_table[] = {
	{ .compatible = "rohm,hall-bu52061nvx", },
	{ },
};
#endif

static struct platform_driver pm8xxx_cradle_driver = {
	.probe		= pm8xxx_cradle_probe,
	.remove		= pm8xxx_cradle_remove,
	.driver		= {
		.name    = HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bu52061nvx_match_table,
#endif
#ifdef CONFIG_PM
		.pm	= &pm8xxx_cradle_pm_ops,
#endif
	},
};

static int __init pm8xxx_cradle_init(void)
{
	cradle_wq = create_singlethread_workqueue("cradle_wq");
       printk(KERN_ERR "cradle init \n");
	if (!cradle_wq)
		return -ENOMEM;
	return platform_driver_register(&pm8xxx_cradle_driver);
}
module_init(pm8xxx_cradle_init);

static void __exit pm8xxx_cradle_exit(void)
{
	if (cradle_wq)
		destroy_workqueue(cradle_wq);
	platform_driver_unregister(&pm8xxx_cradle_driver);
}
module_exit(pm8xxx_cradle_exit);

MODULE_ALIAS("platform:" HALL_IC_DEV_NAME);
MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("pm8xxx cradle driver");
MODULE_LICENSE("GPL");
