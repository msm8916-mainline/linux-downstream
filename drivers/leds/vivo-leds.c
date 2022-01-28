/*
 * LEDs driver for GPIOs
 *
 * Copyright (C) 2014 VIVO Technologies inc.
 * ZhouYulin create <zhouyulin@vivo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>

#define MAX_BRIGHTNESS 255
#define DRV_NAME "led-driver"

struct vivo_led{
	struct pwm_device *led_pwm;
	int enable_gpio;
};
extern int get_vivo_led_pwm(struct vivo_led *vivo_led_pwm);
extern int lcm_flag;
struct vivo_led_config{
	struct led_classdev led_dev;
	struct vivo_led vivo_led_pwm;
	int enable;
	int pwm_crtl;
	int pwm_period;
	int pwm_duty;
	int pwm_channel;
	int blink;
	int engineering_value;
	bool share_pwm;
};

static struct led_classdev* vivo_led_dev;

extern int get_hw_subtype(void);
static void vivo_led_brightness_set (struct led_classdev *led_cdev,
                enum led_brightness value)
{
	struct vivo_led_config *led = container_of(led_cdev, struct vivo_led_config, led_dev);
	int ret;
	if(value > led_cdev->max_brightness)
		value = led_cdev->max_brightness;
	led_cdev->brightness = value;
	if(led->blink == 0) {
		led->pwm_period = 2000;

		if(get_hw_subtype() == 14 || 16 == get_hw_subtype())//pd1524
			value = 255;
		else
			value = 70;//The brightness of the indicator light
	}
	led->pwm_duty = value * led->pwm_period;     
	led->pwm_duty /= 255;
	if(led_cdev->brightness == 0){
		led->enable = 0;
		led->blink = 0;
	}else
		led->enable = 1;
	if(led->enable && lcm_flag){
		ret = pwm_config_us(led->vivo_led_pwm.led_pwm, led->pwm_duty, led->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
		ret = pwm_enable(led->vivo_led_pwm.led_pwm);
		if (ret){
			pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
			led->enable = 1;
		}
		ret = gpio_direction_output(led->vivo_led_pwm.enable_gpio,1);
		if (ret){
			pr_err("%s: set led-enable_gpio status 1 failed err=%d\n", __func__,
				ret);
		}
	}else{
		if(!(led->share_pwm)){
			pwm_disable(led->vivo_led_pwm.led_pwm);
		}
		ret = gpio_direction_output(led->vivo_led_pwm.enable_gpio,0);
		if (ret){
			pr_err("%s: set led-enable_gpio status 0 failed err=%d\n", __func__,
				ret);
		}
	}
}

void vivo_led_light(void)
{
	vivo_led_brightness_set(vivo_led_dev,vivo_led_dev->brightness);
}
EXPORT_SYMBOL_GPL(vivo_led_light);

static ssize_t vivo_led_blink(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct vivo_led_config *led = container_of(led_cdev, struct vivo_led_config, led_dev);
	unsigned long blinking;
	ssize_t ret = -EINVAL;
	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	led->blink = blinking;
	led_cdev->brightness = blinking ? 23 : 0;
	led->pwm_period = blinking ? 2000000 : 2000;
	vivo_led_brightness_set(led_cdev,led_cdev->brightness);
	return size;
}

static ssize_t vivo_led_show_blink(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct vivo_led_config *led = container_of(led_cdev, struct vivo_led_config, led_dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", led->blink);
}

static ssize_t led_store_engineering(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct vivo_led_config *led = container_of(led_cdev, struct vivo_led_config, led_dev);
	unsigned long blinking;
	int error;
	ssize_t ret = -EINVAL;
	ret = kstrtoul(buf, 10, &blinking);
	led->engineering_value = blinking;
	if (ret)
		return ret;
	if(blinking != 0){
		error = gpio_direction_output(led->vivo_led_pwm.enable_gpio,1);
		if (error){
			pr_err("%s: set led-enable_gpio status 1 failed err=%d\n", __func__,
				error);
		}
	}else{
		error = gpio_direction_output(led->vivo_led_pwm.enable_gpio,0);
		if (error){
			pr_err("%s: set led-enable_gpio status 0 failed err=%d\n", __func__,
				error);
		}
	}
	return size;
}

static ssize_t led_show_engineering(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct vivo_led_config *led = container_of(led_cdev, struct vivo_led_config, led_dev);
	return snprintf(buf, PAGE_SIZE, "%u\n", led->engineering_value);
}

static DEVICE_ATTR(blink, 0666, vivo_led_show_blink, vivo_led_blink);

static DEVICE_ATTR(engineering, 0666, led_show_engineering, led_store_engineering);

static int vivo_led_probe(struct platform_device *pdev)
{
	int rc = -1,error;
	struct vivo_led_config* drvdata;
	u32 tmp;
	
	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct vivo_led_config), GFP_KERNEL);
	
	pr_err("vivo_led_probe in\n");
	rc = of_property_read_string(pdev->dev.of_node,"vivo,name",&drvdata->led_dev.name);
	if(rc){
		pr_err(DRV_NAME " Property name could not be read: %d\n", rc);
		return -EINVAL;
	}
	error = of_property_read_u32(pdev->dev.of_node,"led,pwm_period",&drvdata->pwm_period);
	if(error){
		printk(DRV_NAME " Property pwm_period could not be read: %d\n", rc);
		//use default value
		drvdata->pwm_period = 2000;
	}
	error = of_property_read_u32(pdev->dev.of_node,"led,pwm_duty",&drvdata->pwm_duty);
	if(error){
		printk(DRV_NAME" Property pwm_duty could not be read: %d\n", rc);
		//use default value
		drvdata->pwm_duty = 200000;
	}
	drvdata->share_pwm=of_property_read_bool(pdev->dev.of_node,"vivo,led-share-pwm");
	if(!(drvdata->share_pwm))
	{	
		lcm_flag = 1;
		rc = of_property_read_u32(pdev->dev.of_node,"qcom,led-pwm-select",&tmp);
		if (rc) {
			pr_err("%s:Error, lpg channel\n",__func__);
			return -EINVAL;
		}
		drvdata->vivo_led_pwm.led_pwm = pwm_request(tmp, "led-pwm");
		
		if (drvdata->vivo_led_pwm.led_pwm == NULL || IS_ERR(drvdata->vivo_led_pwm.led_pwm)) {
			pr_err("%s: Error: lpg_chan pwm request failed",
					__func__);
		}

		//add for vivo indicator light
		drvdata->vivo_led_pwm.enable_gpio = of_get_named_gpio(pdev->dev.of_node,
			"vivo,led-en-gpio", 0);
		if (!gpio_is_valid(drvdata->vivo_led_pwm.enable_gpio)){
			pr_info("%s: vivo_led_enable gpio not specified\n", __func__);
			drvdata->enable = 0;
		}else{
			rc = gpio_request(drvdata->vivo_led_pwm.enable_gpio,
							"vivo_led_enable");
			drvdata->enable = 1;
			if (rc) {
				pr_err("%s: request vivo_led_enable gpio failed, rc=%d\n",
							   __func__,rc);
				drvdata->enable = 0;
			}
		}
	}else{
		if(get_vivo_led_pwm(&drvdata->vivo_led_pwm)){
			pr_err(DRV_NAME " PWM_request is failed\n");
			return -EINVAL;
		}
	}
	//add end
	
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;
	drvdata->led_dev.brightness_set = vivo_led_brightness_set;
	error = led_classdev_register(&pdev->dev, &drvdata->led_dev);
	if (error < 0) {
		pr_err(DRV_NAME "Register indicator led class failed: %d\n", error);
		//goto err_create;
	}
	//creat file nodes
	vivo_led_dev = &drvdata->led_dev;
	error = device_create_file(drvdata->led_dev.dev, &dev_attr_blink);
	if (error < 0) {
		pr_err(DRV_NAME "vivo led blink file device creation failed: %d\n", error);
		error = -ENODEV;
		//goto err_create;
	}
	error = device_create_file(drvdata->led_dev.dev, &dev_attr_engineering);
	if (error < 0) {
		pr_err(DRV_NAME "vivo led engineering file device creation failed: %d\n", error);
		error = -ENODEV;
		//goto err_create;
	}
	drvdata->blink = 0;
	drvdata->engineering_value = 0;
	return rc;
}

static int vivo_led_remove(struct platform_device *pdev)
{
	return 0;
}
static const struct of_device_id vivo_led_of_match[]  = {
	{ .compatible = "vivo,led", },
	{},
};

static struct platform_driver vivo_led_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vivo_led_of_match,
	},
	.probe = vivo_led_probe,
	.remove = vivo_led_remove,
};

static int vivo_led_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&vivo_led_driver);
	if(rc < 0)
		pr_err("platform register failed (%s)\n",__func__);
	return 0;
}

static void __exit vivo_led_exit(void)
{
	platform_driver_unregister(&vivo_led_driver);
}
module_init(vivo_led_init);
module_exit(vivo_led_exit);

MODULE_DESCRIPTION("VIVO led driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, vivo_led_of_match);
