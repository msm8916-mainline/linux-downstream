
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#if defined(CONFIG_TCT_8X16_COMMON)
extern  void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value);
#endif
#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,camera-led-flash-gpio"

struct led_gpio_flash_data {
	int brightness;
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct mutex lock;
	struct work_struct work;
	uint16_t gpio_flash_en;
};

static struct led_gpio_flash_data *p_data = NULL;

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void led_gpio_brightness_set_work(struct work_struct *work)
{
	int flash_en = 0;

	printk(KERN_DEBUG"%s:%d, brightness:%d\n", __func__, __LINE__, p_data->brightness);
	mutex_lock(&p_data->lock);
    if (p_data->brightness > LED_OFF) {
		flash_en = 1;
	} else {
		flash_en = 0;
	}

    gpio_request(p_data->gpio_flash_en, "FLASH_EN");
	gpio_direction_output(p_data->gpio_flash_en, flash_en);
	gpio_free(p_data->gpio_flash_en);
	mutex_unlock(&p_data->lock);
}

static void led_gpio_sgm_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	if (value != p_data->brightness)
	{
	    p_data->brightness = value;
#if defined(CONFIG_TCT_8X16_COMMON)
		led_gpio_brightness_set(led_cdev,value);
#else
	    schedule_work(&p_data->work);
#endif
	 printk("%s:##### brightness %d;\n", __func__,value);

	} else
	{
		printk(KERN_DEBUG"%s:%d, same brightness with last time\n", __func__, __LINE__);
	}
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev *led_cdev)
{
	return p_data->brightness;
}

int led_gpio_sgm_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *sub_node = NULL;

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	p_data = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (p_data == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

    // parse gpio flash-en
    if (!of_gpio_named_count(node, "qcom,flash-en"))
    {
		printk(KERN_DEBUG"%s:%d, gpio flash-en not set\n", __func__, __LINE__);
		goto error;
    }
    p_data->gpio_flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	printk(KERN_DEBUG"%s:%d, gpio flash-en:%u\n", __func__, __LINE__, p_data->gpio_flash_en);
	
	// parse of node "qcom,flash"
	sub_node = of_find_node_by_name(node, "qcom,flash");
	if (!sub_node)
	{
		pr_err("%s:%d, qcom,flash node not found\n", __func__, __LINE__);
		goto error;
	}
    rc = of_property_read_string(sub_node, "linux,default-trigger",
		&p_data->cdev_flash.default_trigger);
	if (rc < 0) {
		pr_err("%s: Failed to read flash default-trigger. rc = %d\n", __func__, rc);
		goto error;
	}
    rc = of_property_read_string(sub_node, "linux,name", &p_data->cdev_flash.name);
	if (rc < 0) {
		pr_err("%s: Failed to read flash name. rc = %d\n", __func__, rc);
		goto error;
	}
    p_data->cdev_flash.max_brightness = LED_FULL;
	p_data->cdev_flash.brightness_set = led_gpio_sgm_brightness_set;
	p_data->cdev_flash.brightness_get = led_gpio_brightness_get;
    rc = led_classdev_register(&pdev->dev, &p_data->cdev_flash);
	if (rc) {
		pr_err("%s: Failed to register led dev flash. rc = %d\n", __func__, rc);
		goto error;
	}
	
	// parse of node "qcom,torch"
	sub_node = of_find_node_by_name(node, "qcom,torch");
	if (!sub_node)
	{
		pr_err("%s:%d, qcom,torch node not found\n", __func__, __LINE__);
		goto error_cdev;
	}
    rc = of_property_read_string(sub_node, "linux,default-trigger",
		&p_data->cdev_torch.default_trigger);
	if (rc < 0) {
		pr_err("%s: Failed to read torch default-trigger. rc = %d\n", __func__, rc);
		goto error_cdev;
	}
    rc = of_property_read_string(sub_node, "linux,name", &p_data->cdev_torch.name);
	if (rc < 0) {
		pr_err("%s: Failed to read torch name. rc = %d\n", __func__, rc);
		goto error_cdev;
	}
    p_data->cdev_torch.max_brightness = LED_FULL;
	p_data->cdev_torch.brightness_set = led_gpio_sgm_brightness_set;
	p_data->cdev_torch.brightness_get = led_gpio_brightness_get;
    rc = led_classdev_register(&pdev->dev, &p_data->cdev_torch);
	if (rc) {
		pr_err("%s: Failed to register led dev torch. rc = %d\n", __func__, rc);
		goto error_cdev;
	}

    INIT_WORK(&p_data->work, led_gpio_brightness_set_work);
	mutex_init(&p_data->lock);
	platform_set_drvdata(pdev, p_data);
	
	return 0;
	
error_cdev:
	led_classdev_unregister(&p_data->cdev_flash);
error:
	devm_kfree(&pdev->dev, p_data);
	return rc;
}

int led_gpio_sgm_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);

	cancel_work_sync(&flash_led->work);
	mutex_destroy(&flash_led->lock);
	led_classdev_unregister(&flash_led->cdev_flash);
	led_classdev_unregister(&flash_led->cdev_torch);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_sgm_flash_probe,
	.remove = led_gpio_sgm_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init led_gpio_flash_init(void)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
