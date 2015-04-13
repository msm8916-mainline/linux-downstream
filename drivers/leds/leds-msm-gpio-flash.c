
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
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#if defined(CONFIG_TCT_8X16_COMMON)
#include <linux/clk.h>
#endif
#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,leds-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

enum msm_flash_seq_type_t {
	GPIO_FLASH_EN,
	GPIO_FLASH_NOW,
};

struct msm_flash_ctrl_setting{
	enum msm_flash_seq_type_t seq_type;
	long config_flash_on_val;
	long config_torch_on_val;
	long config_flash_off_val;
};

struct led_gpio_flash_data {
	int flash_en;
	int flash_now;
	int brightness;
	struct led_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct msm_flash_ctrl_setting ctrl_setting[2];
};

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};
#if defined(CONFIG_TCT_8X16_COMMON)
struct clk *flash_clk;
struct pinctrl_state *pins_active;
struct pinctrl_state *pins_sleep;
int clk_is_prepare = 0;
int flash_now_bk = 0;
extern void pub_msm_tlmm_v4_gp_fn_gpio_32(uint pin_no, u32 func,	bool enable);
#endif
 void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
#if defined(CONFIG_TCT_8X16_COMMON)
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	int brightness = value;
	flash_led->flash_now = flash_now_bk;
	//printk("%s: ~~~~brightness %d; flash_led->flash_now =%d\n", __func__,brightness,flash_led->flash_now);
	if (brightness > LED_HALF) {
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(flash_led->flash_now, 1);
		clk_is_prepare = 0;
	}else if(brightness > LED_OFF){
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,2,1);
	    if (brightness > 85) {
		if(clk_is_prepare)
		clk_disable_unprepare(flash_clk);
		clk_set_rate(flash_clk, 24000);
		clk_prepare_enable(flash_clk);
		clk_is_prepare =1;
	} else if (brightness > 42) {
		if(clk_is_prepare)
		clk_disable_unprepare(flash_clk);
		clk_set_rate(flash_clk, 24001);
		clk_prepare_enable(flash_clk);
		clk_is_prepare =1;
	} else if (brightness > LED_OFF) {
		if(clk_is_prepare)
		clk_disable_unprepare(flash_clk);
		clk_set_rate(flash_clk, 24002);
		clk_prepare_enable(flash_clk);
		clk_is_prepare =1;
	}
	}else {
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(flash_led->flash_now, 0);
		if(clk_is_prepare)
		clk_disable_unprepare(flash_clk);
		clk_is_prepare = 0;
	}
	return;
#else
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);

	int brightness = value;
	int flash_en = 0, flash_now = 0;

	if (brightness > LED_HALF) {
		flash_en = 1;
		flash_now = 1;
	} else if (brightness > LED_OFF) {
		flash_en = 1;
		flash_now = 0;
	} else {
		flash_en = 0;
		flash_now = 0;
	}
	printk("%s: brightness %d; flash_en %d; flash_now %d  \n", __func__,brightness,
		       flash_en,flash_now);

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_en);
		goto err;
	}
	rc = gpio_direction_output(flash_led->flash_now, flash_now);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_now);
		goto err;
	}

	flash_led->brightness = brightness;
err:
	return;
#endif
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	return flash_led->brightness;
}

int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;

	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;
/* [PLATFORM]-Add-BEGIN by TCTNB.qijiang.yu, 2014/05/07,  change for camera led flash Alto4.5 */
#if defined(CONFIG_TCT_8X16_COMMON)
	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}
	pins_active = pinctrl_lookup_state(flash_led->pinctrl,
                PINCTRL_STATE_DEFAULT);
    if (IS_ERR_OR_NULL(pins_active)) {
        pr_err("ice40 Failed to lookup pinctrl default state\n");
        return PTR_ERR(pins_active);
    }

    pins_sleep = pinctrl_lookup_state(flash_led->pinctrl,
                PINCTRL_STATE_SLEEP);
    if (IS_ERR_OR_NULL(pins_sleep)) {
        pr_err("ice40 Failed to lookup pinctrl sleep state\n");
        return PTR_ERR(pins_sleep);
    }
    rc = pinctrl_select_state(flash_led->pinctrl, pins_active);
    if (rc) {
        pr_err("%s: Can not set %s pins\n", __func__, PINCTRL_STATE_DEFAULT);
    }
#endif
#if 0
	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"ocp8110_default");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);
#endif
/* [PLATFORM]-Add-END by TCTNB.qijiang.yu */
	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);

	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_en, rc);

			goto error;
		}
	}

	flash_led->flash_now = of_get_named_gpio(node, "qcom,flash-now", 0);
	if (flash_led->flash_now < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-now", node->full_name, flash_led->flash_now);
		goto error;
	} else {
#if !(defined(CONFIG_TCT_8X16_COMMON))
		rc = gpio_request(flash_led->flash_now, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_now, rc);
			goto error;
		}
#else
     	flash_now_bk = flash_led->flash_now;
		printk("flash_led->flash_now = %d \n",flash_led->flash_now);
#endif
	}

	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error;
	}

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
#if defined(CONFIG_TCT_8X16_COMMON)
	if (!of_get_property(node, "clock-names", NULL))
		return 0;
	flash_clk = devm_clk_get(&pdev->dev, "cam_sensor_flash");
	if (IS_ERR(flash_clk)) {
		rc = PTR_ERR(flash_clk);
	if (rc != -EPROBE_DEFER)
		pr_err("fail to get flash clk %d\n", rc);
	}
	if (flash_clk) {
		rc = 0;// clk_set_rate(flash_clk, 19200000);
	if (rc < 0) {
		pr_err("fail to setting flash clk %d\n", rc);
		}
	}

	if (flash_clk) {
		rc = 0;//clk_prepare_enable(flash_clk);
	if (rc < 0) {
		pr_err("fail to enable flash clk %d\n", rc);
		}
	}
#endif
	pr_err("%s:probe successfully!\n", __func__);
	return 0;

error:
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);

	led_classdev_unregister(&flash_led->cdev);
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
