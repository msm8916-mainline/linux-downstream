
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/printk.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/clk.h>

#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,camera-led-flash-gpio"

#define FLASH_TIME	660

enum msm_flash_seq_type_t {
	FLASH_EN,
	FLASH_NOW,
};

struct msm_flash_ctrl_seq {
	enum msm_flash_seq_type_t seq_type;
	uint8_t flash_on_val;
	uint8_t torch_on_val;
	uint8_t flash_off_val;
};

struct sgm3785_info {
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct workqueue_struct *queue;
	struct delayed_work flash_work;
	struct work_struct pwm_work;
	struct mutex lock;
	int flash_brightness;
	int torch_brightness;
	uint16_t gpio_flash_en;
	uint16_t gpio_torch_en;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct msm_flash_ctrl_seq ctrl_seq[2];
};

struct clk *flash_clk;
struct pinctrl_state *pins_active;
struct pinctrl_state *pins_sleep;
int clk_is_prepare = 0;
int flash_now_bk = 0;

extern void pub_msm_tlmm_v4_gp_fn_gpio_32(uint pin_no, u32 func,	bool enable);
static struct sgm3785_info *_sinfo;

static struct of_device_id sgm3785_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void sgm3785_flash_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct sgm3785_info *sinfo = _sinfo;

	sinfo->flash_brightness = value;

	 if(value > LED_OFF)
	{
		queue_delayed_work(sinfo->queue,&sinfo->flash_work,msecs_to_jiffies(0));
	}
	else 
	{
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(sinfo->gpio_torch_en, 0);
		gpio_direction_output(sinfo->gpio_flash_en, 0);
		if(clk_is_prepare)
			clk_disable_unprepare(flash_clk);
		clk_is_prepare = 0;
	}		
	
}

static enum led_brightness sgm3785_flash_brightness_get(struct led_classdev *led_cdev)
{
	struct sgm3785_info *sinfo = _sinfo;

	return sinfo->flash_brightness;
}

static void sgm3785_torch_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct sgm3785_info *sinfo = _sinfo;
	//pr_err("[Liu]%s:torchen=%d, enable=%d\n",__func__,sinfo->gpio_torch_en, value);
	//gpio_direction_output(sinfo->gpio_flash_en,!!value);
	sinfo->torch_brightness = value;

	if (value > LED_HALF) 
	{
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(sinfo->gpio_torch_en, 1);
		clk_is_prepare = 0;
	}
	else if(value > LED_OFF)
	{
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(sinfo->gpio_torch_en, 1);

		mdelay(6);
		
		schedule_work(&sinfo->pwm_work);

	}
	else 
	{
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
		gpio_direction_output(sinfo->gpio_torch_en, 0);
		if(clk_is_prepare)
			clk_disable_unprepare(flash_clk);
		clk_is_prepare = 0;
	}

	
}

static enum led_brightness sgm3785_torch_brightness_get(struct led_classdev *led_cdev)
{
	struct sgm3785_info *sinfo = _sinfo;

	return sinfo->torch_brightness;
}
void led_torch_control_pwm( enum led_brightness value)
{
		pub_msm_tlmm_v4_gp_fn_gpio_32(32,2,1);

	 if (value > 90) {
			if(clk_is_prepare)
				clk_disable_unprepare(flash_clk);

			clk_set_rate(flash_clk, 24003);
			clk_prepare_enable(flash_clk);
			clk_is_prepare =1;
			pr_err("%s,24003\n",__func__);
		} else if (value > 60) {
			if(clk_is_prepare)
				clk_disable_unprepare(flash_clk);

			clk_set_rate(flash_clk, 24002);
			clk_prepare_enable(flash_clk);
			clk_is_prepare =1;
			pr_err("%s,24002\n",__func__);
		} else if (value > 30) {
			if(clk_is_prepare)
				clk_disable_unprepare(flash_clk);

			clk_set_rate(flash_clk, 24001);
			clk_prepare_enable(flash_clk);
			clk_is_prepare =1;
			pr_err("%s,24001\n",__func__);
		} else if (value > LED_OFF) {
			if(clk_is_prepare)
				clk_disable_unprepare(flash_clk);

			clk_set_rate(flash_clk, 24000);
			clk_prepare_enable(flash_clk);
			clk_is_prepare =1;
			pr_err("%s,24000\n",__func__);
		}
		
}


static void sgm3785_flash_worker(struct work_struct *work)
{
	struct sgm3785_info *sinfo = _sinfo;

	mutex_lock(&sinfo->lock);
	led_torch_control_pwm(sinfo->flash_brightness);
	mutex_unlock(&sinfo->lock);
	gpio_direction_output(sinfo->gpio_flash_en, 1);
	msleep(FLASH_TIME);
	pub_msm_tlmm_v4_gp_fn_gpio_32(32,0,1);
	gpio_direction_output(sinfo->gpio_torch_en, 0);
	gpio_direction_output(sinfo->gpio_flash_en, 0);
	if(clk_is_prepare)
		clk_disable_unprepare(flash_clk);
	clk_is_prepare = 0;
}
static void led_torch_pwm_work(struct work_struct *work)
{
	
	struct sgm3785_info *sinfo = _sinfo;

	if (sinfo) {
		mutex_lock(&sinfo->lock);
		led_torch_control_pwm(sinfo->torch_brightness);
		mutex_unlock(&sinfo->lock);
	} else
		printk("%s flash_led NULL \n", __func__);

}


void load_pinctrl(struct sgm3785_info *sinfo,struct platform_device *pdev)
{
	int rc = 0;
	if (!sinfo) {
		return;
	}
	sinfo->pinctrl = devm_pinctrl_get(&pdev->dev); 
	if (IS_ERR(sinfo->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return;
	}
	pins_active = pinctrl_lookup_state(sinfo->pinctrl,
                PINCTRL_STATE_DEFAULT);
    if (IS_ERR_OR_NULL(pins_active)) {
        pr_err("ice40 Failed to lookup pinctrl default state\n");
        return;
    }
    pins_sleep = pinctrl_lookup_state(sinfo->pinctrl,
                PINCTRL_STATE_SLEEP);
    if (IS_ERR_OR_NULL(pins_sleep)) {
        pr_err("ice40 Failed to lookup pinctrl sleep state\n");
        return;
    }
    rc = pinctrl_select_state(sinfo->pinctrl, pins_active);
    if (rc) {
        pr_err("%s: Can not set %s pins\n", __func__, PINCTRL_STATE_DEFAULT);
    }
}

int sgm3785_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *sub_node;
	static struct sgm3785_info *sinfo;

	sinfo = devm_kzalloc(&pdev->dev, sizeof(struct sgm3785_info),
				 GFP_KERNEL);
	if (!sinfo) {
		pr_err("[Liu]%s: no mem for sinfo\n", __func__);
		return -ENOMEM;
	}

	load_pinctrl(sinfo,pdev);

	_sinfo = sinfo;
	sub_node = of_find_node_by_name(node, "qcom,flash");
	if (!sub_node) {
		pr_err("[Liu]%s: qcom,flash not found\n", __func__);
		goto error1;
	}
	rc = of_property_read_string(sub_node, "flash,name",
		&sinfo->cdev_flash.name);
	if (rc < 0) {
		pr_err("[Liu]%s: flash,name not found\n", __func__);
		goto error1;
	}
	if (!of_gpio_named_count(sub_node, "qcom,flash-en")) {
		pr_err("[Liu]%s: qcom,flash-en not found\n", __func__);
		goto error1;
	}
	sinfo->gpio_flash_en = of_get_named_gpio(sub_node, "qcom,flash-en", 0);
	gpio_request(sinfo->gpio_flash_en, "FLASH_EN");
	gpio_direction_output(sinfo->gpio_flash_en, 0);
	pr_info("[Liu]%s: gpio_flash_en=%d\n",__func__, sinfo->gpio_flash_en);
	rc = of_property_read_string(sub_node, "flash,default-trigger",
		&sinfo->cdev_flash.default_trigger);
	if (rc < 0) {
		pr_err("[Liu]%s: flash default_trigger not found\n", __func__);
		goto error1;
	}
	sinfo->cdev_flash.max_brightness = LED_FULL;
	sinfo->cdev_flash.brightness_set = sgm3785_flash_brightness_set;
	sinfo->cdev_flash.brightness_get = sgm3785_flash_brightness_get;
	rc = led_classdev_register(&pdev->dev, &sinfo->cdev_flash);
	if (rc) {
		pr_err("[Liu]%s: cdev_flash registed error\n", __func__);
		goto error1;
	}

	sub_node = of_find_node_by_name(node, "qcom,torch");
	if (!sub_node)
	{
		pr_err("[Liu]%s: qcom,torch not found\n", __func__);
		goto error2;
	}
	rc = of_property_read_string(sub_node, "torch,name", &sinfo->cdev_torch.name);
	if (rc < 0) {
		pr_err("[Liu]%s: torch,name not found\n", __func__);
		goto error2;
	}
	if (!of_gpio_named_count(sub_node, "qcom,torch-en"))
	{
		pr_err("[Liu]%s: qcom,torch-en not found\n", __func__);
		goto error2;
	}
	sinfo->gpio_torch_en = of_get_named_gpio(sub_node, "qcom,torch-en", 0);
	gpio_request(sinfo->gpio_torch_en, "TORCH_EN");
	gpio_direction_output(sinfo->gpio_torch_en, 0);
	pr_info("[Liu]%s: gpio_torch_en=%d\n",__func__, sinfo->gpio_torch_en);
	rc = of_property_read_string(sub_node, "torch,default-trigger",
		&sinfo->cdev_torch.default_trigger);
	if (rc < 0) {
		pr_err("[Liu]%s: torch default_trigger not found\n", __func__);
		goto error2;
	}
	sinfo->cdev_torch.max_brightness = LED_FULL;
	sinfo->cdev_torch.brightness_set = sgm3785_torch_brightness_set;
	sinfo->cdev_torch.brightness_get = sgm3785_torch_brightness_get;
	rc = led_classdev_register(&pdev->dev, &sinfo->cdev_torch);
	if (rc) {
		pr_err("[Liu]%s: cdev_torch registed error\n", __func__);
		goto error2;
	}
	sinfo->queue = alloc_ordered_workqueue("sgm3785", 0);
	if (!sinfo->queue) {
		rc = -ENOMEM;
		goto error3;
	}
	INIT_DELAYED_WORK(&sinfo->flash_work, sgm3785_flash_worker);
	INIT_WORK(&sinfo->pwm_work, led_torch_pwm_work);
	mutex_init(&sinfo->lock);
	platform_set_drvdata(pdev, sinfo);

	if (!of_get_property(node, "clock-names", NULL))
	{
		pr_err("[Liu]%s: clock-names not found\n", __func__);
		return 0;
	}
	flash_clk = devm_clk_get(&pdev->dev, "cam_sensor_flash");
	if (IS_ERR(flash_clk)) {
		rc = PTR_ERR(flash_clk);
	if (rc != -EPROBE_DEFER)
		pr_err("fail to get flash clk %d\n", rc);
	}
	if (flash_clk) {
//		clk_set_rate(flash_clk, 15000);
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

	return 0;

error3:
	led_classdev_unregister(&sinfo->cdev_torch);
error2:
	led_classdev_unregister(&sinfo->cdev_flash);
error1:
	devm_kfree(&pdev->dev, sinfo);
	return rc;
}

int sgm3785_remove(struct platform_device *pdev)
{
	struct sgm3785_info *sinfo = _sinfo;

	mutex_destroy(&sinfo->lock);
	destroy_workqueue(sinfo->queue);
	led_classdev_unregister(&sinfo->cdev_torch);
	led_classdev_unregister(&sinfo->cdev_flash);
	devm_kfree(&pdev->dev, sinfo);
	return 0;
}

static struct platform_driver sgm3785_driver = {
	.probe = sgm3785_probe,
	.remove = sgm3785_remove,
	.driver = {
		.name = LED_GPIO_FLASH_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sgm3785_of_match,
	}
};

static int __init sgm3785_init(void)
{
	return platform_driver_register(&sgm3785_driver);
}

static void __exit sgm3785_exit(void)
{
	return platform_driver_unregister(&sgm3785_driver);
}

subsys_initcall(sgm3785_init);
module_exit(sgm3785_exit);

MODULE_DESCRIPTION("SGM3785 LED driver");
MODULE_LICENSE("GPL v2");
