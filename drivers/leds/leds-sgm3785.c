
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>

#include <linux/delay.h>


/* #define CONFIG_GPIO_FLASH_DEBUG */
#undef CDBG
#ifdef CONFIG_GPIO_FLASH_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define LED_GPIO_FLASH_DRIVER_NAME	"sgm3785"
#define LED_TRIGGER_DEFAULT		"none"

struct led_gpio_flash_data {
	int flash_en;
	int torch_en;
	int brightness;
	int clk_prepared;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct pinctrl *pinctrl;

	struct clk *flash_clk;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;

	struct mutex lock;
	struct work_struct pwm_work;
};

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};


extern void pub_msm_tlmm_v4_gp_fn_gpio_32(uint pin_no, u32 func,	bool enable);

static int led_torch_control_pwm(struct led_gpio_flash_data *flash_led, enum led_brightness value)
{
	int brightness = value;

	gpio_direction_output(flash_led->flash_en, 0);

	pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
	if (0 == value)
		gpio_direction_output(flash_led->torch_en, 0);
	else
		gpio_direction_output(flash_led->torch_en, 1);

	msleep(5);

	if (brightness > LED_HALF) {
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(flash_led->torch_en, 1);
		flash_led->clk_prepared = 0;
	} else if(brightness > LED_OFF){
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,2,1);

		if (brightness > 85) {
			if(flash_led->clk_prepared)
				clk_disable_unprepare(flash_led->flash_clk);
			clk_set_rate(flash_led->flash_clk, 24000);
			clk_prepare_enable(flash_led->flash_clk);
			flash_led->clk_prepared =1;
		} else if (brightness > 42) {
			if(flash_led->clk_prepared)
				clk_disable_unprepare(flash_led->flash_clk);
			clk_set_rate(flash_led->flash_clk, 24001);
			clk_prepare_enable(flash_led->flash_clk);
			flash_led->clk_prepared =1;
		} else if (brightness > LED_OFF) {
			if(flash_led->clk_prepared)
				clk_disable_unprepare(flash_led->flash_clk);
			clk_set_rate(flash_led->flash_clk, 24002);
			clk_prepare_enable(flash_led->flash_clk);
			flash_led->clk_prepared =1;
		}
	}else {
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(flash_led->torch_en, 0);
		if(flash_led->clk_prepared)
			clk_disable_unprepare(flash_led->flash_clk);
		flash_led->clk_prepared = 0;
	}

	return 0;
}

static void led_flash_control(struct led_gpio_flash_data *flash_led, enum led_brightness value)
{
	int rc = 0;
	int flash_en = 0, torch_en = 0;

	if (value > 0) {
		torch_en = 1;
		flash_en = 1;
	} else if (0 == value) {
		flash_en = 0;
		torch_en = 0;
	}

	rc = gpio_direction_output(flash_led->torch_en, torch_en);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->torch_en, rc);
	}

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->flash_en, rc);
	}

}

static void led_torch_control_gpio(struct led_gpio_flash_data *flash_led, enum led_brightness value)
{
	int rc = 0;
	int flash_en = 0, torch_en = 0;

	if (value > 0) {
		flash_en = 0;
		torch_en = 1;
	} else if (0 == value) {
		flash_en = 0;
		torch_en = 0;
	}

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->flash_en, rc);
	}

	rc = gpio_direction_output(flash_led->torch_en, torch_en);
	if (rc) {
		printk("%s: Failed gpio(%d) rc=%d \n", __func__, flash_led->torch_en, rc);
	}

}

static void led_torch_pwm_work(struct work_struct *work)
{
	struct led_gpio_flash_data *flash_led =NULL;
	flash_led = container_of(work, struct led_gpio_flash_data, pwm_work);

	if (flash_led) {
		mutex_lock(&flash_led->lock);
		led_torch_control_pwm(flash_led, flash_led->cdev_torch.brightness);
		mutex_unlock(&flash_led->lock);
	} else
		printk("%s flash_led NULL \n", __func__);

}

static void led_gpio_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct led_gpio_flash_data *flash_led =NULL;

	if (0 == strncmp(led_cdev->name, "led-flash", strlen("led-flash"))) {

		flash_led = container_of(led_cdev, struct led_gpio_flash_data, cdev_flash);
		if (flash_led)
			led_flash_control(flash_led, value);
		else
			printk("%s line(%d) NULL flash_led \n", __func__, __LINE__);

	} else if (0 == strncmp(led_cdev->name, "led-torch", strlen("led-torch"))) {

		flash_led = container_of(led_cdev, struct led_gpio_flash_data, cdev_torch);

		if (flash_led) {

			if (904 == flash_led->torch_en) {
				led_torch_control_gpio(flash_led, value);
			} else if (934 == flash_led->torch_en) {
				schedule_work(&flash_led->pwm_work);
			} else
				printk("%s line(%d) error gpio_pin torch_en=%d \n", __func__, __LINE__, flash_led->torch_en);

		} else
			printk("%s line(%d) NULL torch_led \n", __func__, __LINE__);
	}

}

static enum led_brightness led_gpio_brightness_get(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int led_gpio_flash_config(struct platform_device *pdev, struct device_node *temp)
{
	int rc = 0;
	const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct led_classdev *cdev = NULL;

	flash_led = platform_get_drvdata(pdev);

	if (!flash_led) {
		printk(" %s NULL pointer at flash_led \n", __func__);
		return -1;
	}

	rc = of_property_read_string(temp, "linux,name", &temp_str);
	if (rc) {
		printk("%s: Failed to read linux name. rc = %d\n", __func__, rc);
		return -2;
	}

	if (0 == strncmp(temp_str, "led-torch", strlen("led-torch"))) {
		cdev = &flash_led->cdev_torch;
	} else if (0 == strncmp(temp_str, "led-flash", strlen("led-flash"))) {
		cdev = &flash_led->cdev_flash;
	}

	if (!cdev) {
		printk("%s NULL pointer at cdev \n", __func__);
		return -3;
	}

	cdev->name = temp_str;
	rc = of_property_read_string(temp, "linux,name", &cdev->name);
	if (rc) {
		printk("%s: Failed to read linux name. rc = %d\n", __func__, rc);
		return -4;
	}

	cdev->default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(temp, "linux,default-trigger", &temp_str);
	if (!rc)
		cdev->default_trigger = temp_str;

	cdev->max_brightness = LED_FULL;
	cdev->brightness_set = led_gpio_brightness_set;
	cdev->brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, cdev);
	if (rc) {
		printk("%s: Failed to register led dev. rc = %d\n", __func__, rc);
		led_classdev_unregister(cdev);
		return -5;
	}

	return 0;
}

static int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *temp = NULL;
	int num_leds = 0;

	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		printk("%s:%d Unable to allocate memory \n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!of_get_property(node, "clock-names", NULL)) {
		printk("%s:%d Unable to get clock-names \n", __func__, __LINE__);
		return -1;
	}

	flash_led->flash_clk = devm_clk_get(&pdev->dev, "flash_pwm_clk");
	if (IS_ERR(flash_led->flash_clk)) {
		rc = PTR_ERR(flash_led->flash_clk);
	if (rc != -EPROBE_DEFER)
		printk("%s:%d fail to get flash clk %d\n", __func__, __LINE__, rc);
		return -2;
	}

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		printk("%s:%d failed to get pinctrl\n", __func__, __LINE__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->pins_active = pinctrl_lookup_state(flash_led->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(flash_led->pins_active)) {
	    printk("%s:%d Failed to lookup pinctrl default state \n", __func__, __LINE__);
	    return PTR_ERR(flash_led->pins_active);
	}

	flash_led->pins_sleep = pinctrl_lookup_state(flash_led->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(flash_led->pins_sleep)) {
	    printk("%s:%d Failed to lookup pinctrl sleep state\n", __func__, __LINE__);
	    return PTR_ERR(flash_led->pins_sleep);
	}

	rc = pinctrl_select_state(flash_led->pinctrl, flash_led->pins_active);
	if (rc) {
	    printk("%s: Can not set %s pins rc=%d \n", __func__, PINCTRL_STATE_DEFAULT, rc);
	}

	flash_led->clk_prepared = 0;

	flash_led->flash_en = of_get_named_gpio(node, "leds,flash-en", 0);

	if (flash_led->flash_en < 0) {
		printk("%s Looking up %s property in node %s failed. rc =  %d \n", __func__,
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			printk("%s: Failed to request gpio %d,rc = %d\n", __func__, flash_led->flash_en, rc);
			goto error;
		}
	}

	flash_led->torch_en = of_get_named_gpio(node, "leds,torch-en", 0);
	if (flash_led->torch_en < 0) {
		printk("%s Looking up %s property in node %s failed. rc =  %d \n", __func__,
			"torch-en", node->full_name, flash_led->torch_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->torch_en, "TORCH_EN");
		if (rc) {
			printk("%s: Failed to request gpio %d, rc = %d\n",
				__func__, flash_led->torch_en, rc);
			goto error;
		}
	}

	gpio_direction_output(flash_led->torch_en, 0);
	gpio_direction_output(flash_led->flash_en, 0);
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (2 != num_leds)
		return -3;

	platform_set_drvdata(pdev, flash_led);

	if (2 == num_leds)
		for_each_child_of_node(node, temp)
			led_gpio_flash_config(pdev, temp);

	INIT_WORK(&flash_led->pwm_work, led_torch_pwm_work);
	mutex_init(&flash_led->lock);

	printk("%s:probe successfully!\n", __func__);
	return 0;

error:
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

static int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);

	cancel_work_sync(&flash_led->pwm_work);
	mutex_destroy(&flash_led->lock);

	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init led_gpio_flash_init(void)
{
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-msm-gpio-flash");
