/* drivers/video/backlight/lm3639_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
//#include <mach/board.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include "msm_led_flash.h"
#include <mach/board_lge.h>
#define LM3639_FLED_EN
#include <linux/platform_data/lm3639_bl.h>

//#undef pr_debug
//#define pr_debug pr_err

#if defined(LM3639_FLED_EN)
#define I2C_BL_NAME                              "qcom,led-flash"
#else
#define I2C_BL_NAME                              "lm3639_bl"
#endif

#define MAX_BRIGHTNESS_LM3639                    0x7F
#define MIN_BRIGHTNESS_LM3639                    0x05
#define DEFAULT_BRIGHTNESS                       0x1F
#define POWER_OFF		0x00
#define BOTH_ON			0xFF
#define BL_ON			0xF0
#define FLASH_ON		0x0F

/* LGE_CHANGE  - To turn backlight on by setting default brightness while kernel booting */
#define BOOT_BRIGHTNESS 1

#if defined(LM3639_FLED_EN)
static struct msm_camera_i2c_client lm3639_flash_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
#endif
static struct i2c_client *lm3639_i2c_client;

static int store_level_used = 0;

static int init_complete = 0;
#if defined(LM3639_FLED_EN)
static struct msm_led_flash_ctrl_t fctrl;

static const struct i2c_device_id lm3639_bl_id[] = {
	{ I2C_BL_NAME, (kernel_ulong_t)&fctrl},
	{ },
};
#else
static const struct i2c_device_id lm3639_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};
#endif

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static int lm3639_read_reg(struct i2c_client *client, u8 reg, u8 *buf);
#endif

static int lm3639_write_reg(struct i2c_client *client, unsigned char reg, unsigned char val);


static int cur_main_lcd_level = DEFAULT_BRIGHTNESS;
static int saved_main_lcd_level = DEFAULT_BRIGHTNESS;
static int backlight_status = POWER_OFF;
static int lm3639_pwm_enable;
static struct lm3639_device *main_lm3639_dev;

#if defined(CONFIG_LGE_LCD_TUNING)
int lm3639_lcd_backlight_get_blmap(char *bl_map)
{
	if(main_lm3639_dev->blmap_size<=0)
		return -1;
	memcpy(bl_map, main_lm3639_dev->blmap, main_lm3639_dev->blmap_size);
	return 0;
}
int lm3639_lcd_backlight_set_blmap(int bl_size, char *bl_map)
{
	char *temp;
	if (bl_size!=main_lm3639_dev->blmap_size) {
		pr_err("bl size is update\n");

		temp = kzalloc(sizeof(char) * bl_size, GFP_KERNEL);
		if (!temp) {
			kfree(main_lm3639_dev->blmap);
			main_lm3639_dev->blmap = temp;
		}

		main_lm3639_dev->blmap_size = bl_size;
	}

	memcpy(main_lm3639_dev->blmap, bl_map, bl_size);
	return 0;
}
int lm3639_lcd_backlight_get_blmap_size(int *bl_size)
{
	if(main_lm3639_dev->blmap_size<=0)
		return -1;
	*bl_size = main_lm3639_dev->blmap_size;
	return 0;
}

EXPORT_SYMBOL(lm3639_lcd_backlight_get_blmap);
EXPORT_SYMBOL(lm3639_lcd_backlight_set_blmap);
EXPORT_SYMBOL(lm3639_lcd_backlight_get_blmap_size);
#endif


#ifdef CONFIG_LGE_WIRELESS_CHARGER
int wireless_backlight_state(void)
{
	return backlight_status;
}
EXPORT_SYMBOL(wireless_backlight_state);
#endif

static void lm3639_hw_reset(void)
{
	int gpio = main_lm3639_dev->bl_gpio;
	/* LGE_CHANGE - Fix GPIO Setting Warning*/
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, 1);
		gpio_set_value_cansleep(gpio, 1);
		mdelay(10);
	}
	else
		pr_err("%s: gpio is not valid !!\n", __func__);
}
//#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static int lm3639_read_reg(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if(ret < 0)
		pr_err("[LCD][DEBUG] error\n");

	*buf = ret;

	return 0;

}
//#endif
static int lm3639_write_reg(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg = {
		client->addr, 0, 2, buf
	};

	buf[0] = reg;
	buf[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		dev_err(&client->dev, "i2c write error reg: %d, val: %d\n", buf[0], buf[1]  );
	return 0;
}

static int exp_min_value = 150;
static int cal_value;
static unsigned char bl_ctrl;

static void lm3639_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3639_device *dev = i2c_get_clientdata(client);
	int min_brightness = dev->min_brightness;
	int max_brightness = dev->max_brightness;

	if (level == -BOOT_BRIGHTNESS)
		level = dev->default_brightness;

	cur_main_lcd_level = level;
	dev->bl_dev->props.brightness = cur_main_lcd_level;

	store_level_used = 0;

	mutex_lock(&dev->bl_mutex);
	if (level != 0) {
		if (level > 0 && level <= min_brightness)
			level = min_brightness;
		else if (level > max_brightness)
			level = max_brightness;
		if (dev->blmap) {
			if (level < dev->blmap_size) {
				cal_value = dev->blmap[level];
				lm3639_write_reg(client, 0x05,
						cal_value);
			} else
				dev_warn(&client->dev, "invalid index %d:%d\n",
						dev->blmap_size,
						level);
		} else {
			cal_value = level;
			lm3639_write_reg(client, 0x05, cal_value);
		}
	} else{
			lm3639_write_reg(client, 0x05, 0x00);
			bl_ctrl = 0;
			lm3639_read_reg(main_lm3639_dev->client, 0x0A, &bl_ctrl);
			bl_ctrl &= 0xE6;
			lm3639_write_reg(main_lm3639_dev->client, 0x0A, bl_ctrl);
		}

	mutex_unlock(&dev->bl_mutex);

	pr_info("[LCD] %s : level=%d, cal_value=%d \n",
				__func__, level, cal_value);
}

static void lm3639_set_main_current_level_no_mapping(
		struct i2c_client *client, int level)
{
	struct lm3639_device *dev;
	dev = (struct lm3639_device *)i2c_get_clientdata(client);

	if (level > MAX_BRIGHTNESS_LM3639)
		level = MAX_BRIGHTNESS_LM3639;
	else if (level < 0)
		level = 0;

	cur_main_lcd_level = level;
	dev->bl_dev->props.brightness = cur_main_lcd_level;

	store_level_used = 1;

	mutex_lock(&main_lm3639_dev->bl_mutex);
	if (level != 0) {
		lm3639_write_reg(client, 0x05, level);
	} else {
		lm3639_write_reg(client, 0x00, 0x00);
	}
	mutex_unlock(&main_lm3639_dev->bl_mutex);
}

void lm3639_backlight_on(int level)
{
	if ((backlight_status != BL_ON) && (backlight_status != BOTH_ON)){
		if(!init_complete)
			lm3639_hw_reset();
		bl_ctrl = 0;
		lm3639_read_reg(main_lm3639_dev->client, 0x0A, &bl_ctrl);

		bl_ctrl |= 0x11;

		lm3639_write_reg(main_lm3639_dev->client, 0x0A, bl_ctrl);

		pr_info("%s: ON!\n", __func__);
	}
	mdelay(1);

	lm3639_set_main_current_level(main_lm3639_dev->client, level);
	backlight_status |= BL_ON;

	return;
}

void lm3639_backlight_off(void)
{
	if (!(backlight_status & BL_ON))
		return;

	saved_main_lcd_level = cur_main_lcd_level;
	pr_err("%s\n", __func__);
	lm3639_set_main_current_level(main_lm3639_dev->client, 0);
	backlight_status &= ~BL_ON;

	return;
}

void lm3639_led_enable(void){

	int gpio = main_lm3639_dev->bl_gpio;
	pr_err(">> %s: START\n", __func__);

	mutex_lock(&main_lm3639_dev->bl_mutex);
	if (gpio_get_value(gpio) != 1) {
		gpio_direction_output(gpio, 1);
		if (backlight_status == POWER_OFF) {/* LGE_CHANGE, gangsu.baek@lge.com, If display turn off, set backlight brightness 0x00*/
			lm3639_write_reg(main_lm3639_dev->client, 0x05, 0x00);
		}
		mdelay(10);
		pr_err("%s: ON!\n", __func__);
	}

	backlight_status |= FLASH_ON;
	mutex_unlock(&main_lm3639_dev->bl_mutex);
	pr_err("<< %s: END\n", __func__);
}

void lm3639_led_disable(void){
	int gpio = 0;

	pr_debug(">> %s START", __func__);
	//pr_debug("main_lm3639_dev: %d", (int) main_lm3639_dev);
	if (!main_lm3639_dev) {
		pr_debug("<< %s END : FAIL! (line: %d)", __func__, __LINE__);
		return;
	}

	gpio = main_lm3639_dev->bl_gpio;

	pr_err("%s: Enter\n", __func__);

	mutex_lock(&main_lm3639_dev->bl_mutex);
	backlight_status &= ~FLASH_ON;

	mutex_unlock(&main_lm3639_dev->bl_mutex);
	pr_err("%s: Exit\n", __func__);

	pr_debug("<< %s END", __func__);
}

void lm3639_lcd_backlight_set_level(int level)
{
	if (level > MAX_BRIGHTNESS_LM3639)
		level = MAX_BRIGHTNESS_LM3639;

	if (lm3639_i2c_client != NULL) {
		if (level == 0) {
			lm3639_backlight_off();
		} else {
			lm3639_backlight_on(level);
		}
	} else {
		pr_err("%s(): No client\n", __func__);
	}
}
EXPORT_SYMBOL(lm3639_lcd_backlight_set_level);

static int bl_set_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	/* LGE_CHANGE - if it's trying to set same backlight value, skip it.*/
	if(bd->props.brightness == cur_main_lcd_level){
		pr_err("%s level is already set. skip it\n", __func__);
		return 0;
	}

	lm3639_set_main_current_level(client, bd->props.brightness);
	cur_main_lcd_level = bd->props.brightness;

	return 0;
}

static int bl_get_intensity(struct backlight_device *bd)
{
	unsigned char val = 0;
	val &= 0x1f;

	return (int)val;
}

static ssize_t lcd_backlight_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	if(store_level_used == 0)
		r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
				cal_value);
	else if(store_level_used == 1)
		r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
				cur_main_lcd_level);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int level;
	struct i2c_client *client = to_i2c_client(dev);

	if (!count)
		return -EINVAL;

	level = simple_strtoul(buf, NULL, 10);

	lm3639_set_main_current_level_no_mapping(client, level);
	pr_info("[LCD][DEBUG] write %d direct to "
			"backlight register\n", level);

	return count;
}

static int lm3639_bl_resume(struct i2c_client *client)
{
	lm3639_lcd_backlight_set_level(saved_main_lcd_level);
	return 0;
}

static int lm3639_bl_suspend(struct i2c_client *client, pm_message_t state)
{
	lm3639_lcd_backlight_set_level(saved_main_lcd_level);
	return 0;
}

static ssize_t lcd_backlight_show_on_off(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	pr_info("%s received (prev backlight_status: %s)\n",
			__func__, backlight_status ? "ON" : "OFF");

	return r;
}

static ssize_t lcd_backlight_store_on_off(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int on_off;
	struct i2c_client *client = to_i2c_client(dev);

	if (!count)
		return -EINVAL;

	pr_info("%s received (prev backlight_status: %s)\n",
			__func__, backlight_status ? "ON" : "OFF");

	on_off = simple_strtoul(buf, NULL, 10);

	pr_info("[LCD][DEBUG] %d", on_off);

	if (on_off == 1)
		lm3639_bl_resume(client);
	else if (on_off == 0)
		lm3639_bl_suspend(client, PMSG_SUSPEND);

	return count;

}
static ssize_t lcd_backlight_show_exp_min_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;

	r = snprintf(buf, PAGE_SIZE, "LCD Backlight  : %d\n", exp_min_value);

	return r;
}

static ssize_t lcd_backlight_store_exp_min_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;

	if (!count)
		return -EINVAL;

	value = simple_strtoul(buf, NULL, 10);
	exp_min_value = value;

	return count;
}

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static ssize_t lcd_backlight_show_pwm(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	u8 level,pwm_low,pwm_high,config;

	mutex_lock(&main_lm3639_dev->bl_mutex);
	lm3639_read_reg(main_lm3639_dev->client, 0x01, &config);
	mdelay(3);
	lm3639_read_reg(main_lm3639_dev->client, 0x03, &level);
	mdelay(3);
	lm3639_read_reg(main_lm3639_dev->client, 0x12, &pwm_low);
	mdelay(3);
	lm3639_read_reg(main_lm3639_dev->client, 0x13, &pwm_high);
	mdelay(3);
	mutex_unlock(&main_lm3639_dev->bl_mutex);

	r = snprintf(buf, PAGE_SIZE, "Show PWM level: %d pwm_low: %d "
			"pwm_high: %d config: %d\n", level, pwm_low,
			pwm_high, config);

	return r;
}
static ssize_t lcd_backlight_store_pwm(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}
#endif

DEVICE_ATTR(lm3639_level, 0644, lcd_backlight_show_level,
		lcd_backlight_store_level);
DEVICE_ATTR(lm3639_backlight_on_off, 0644, lcd_backlight_show_on_off,
		lcd_backlight_store_on_off);
DEVICE_ATTR(lm3639_exp_min_value, 0644, lcd_backlight_show_exp_min_value,
		lcd_backlight_store_exp_min_value);
#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
DEVICE_ATTR(lm3639_pwm, 0644, lcd_backlight_show_pwm, lcd_backlight_store_pwm);
#endif

#ifdef CONFIG_OF
static int lm3639_parse_dt(struct device *dev,
		struct backlight_platform_data *pdata)
{
	int rc = 0, i;
	u32 *array;
	struct device_node *np = dev->of_node;

	pdata->bl_gpio = of_get_named_gpio_flags(np, "lm3639,lcd_bl_en", 0, NULL);
	rc = of_property_read_u32(np, "lm3639,max_current",
			&pdata->max_current);
	rc = of_property_read_u32(np, "lm3639,min_brightness",
			&pdata->min_brightness);
	rc = of_property_read_u32(np, "lm3639,default_brightness",
			&pdata->default_brightness);
	rc = of_property_read_u32(np, "lm3639,max_brightness",
			&pdata->max_brightness);

	rc = of_property_read_u32(np, "lm3639,enable_pwm",
			&lm3639_pwm_enable);
	if(rc == -EINVAL)
		lm3639_pwm_enable = 1;

	rc = of_property_read_u32(np, "lm3639,blmap_size",
			&pdata->blmap_size);

	if (pdata->blmap_size) {
		array = kzalloc(sizeof(u32) * pdata->blmap_size, GFP_KERNEL);
		if (!array)
			return -ENOMEM;

		rc = of_property_read_u32_array(np, "lm3639,blmap", array, pdata->blmap_size);
		if (rc) {
			pr_err("%s:%d, uable to read backlight map\n",__func__, __LINE__);
			return -EINVAL;
		}
		pdata->blmap = kzalloc(sizeof(char) * pdata->blmap_size, GFP_KERNEL);

		if (!pdata->blmap)
			return -ENOMEM;

		for (i = 0; i < pdata->blmap_size; i++ )
			pdata->blmap[i] = (char)array[i];

		if (array)
			kfree(array);

	} else {
		pdata->blmap = NULL;
	}

	pr_err("%s max_current: %d, min: %d, "
			"default: %d, max: %d, pwm : %d , blmap_size : %d\n",
			__func__,
			pdata->max_current,
			pdata->min_brightness,
			pdata->default_brightness,
			pdata->max_brightness,
			lm3639_pwm_enable,
			pdata->blmap_size);

	return rc;
}
#endif

static struct backlight_ops lm3639_bl_ops = {
	.update_status = bl_set_intensity,
	.get_brightness = bl_get_intensity,
};

static int lm3639_probe(struct i2c_client *i2c_dev,
		const struct i2c_device_id *id)
{
	struct backlight_platform_data *pdata;
	struct lm3639_device *dev;
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	int err;

	pr_err("[LCD][DEBUG] %s: i2c probe start\n", __func__);

#ifdef CONFIG_OF
	if (&i2c_dev->dev.of_node) {
		pdata = devm_kzalloc(&i2c_dev->dev,
				sizeof(struct backlight_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = lm3639_parse_dt(&i2c_dev->dev, pdata);
		if (err != 0)
			return err;
	} else {
		pdata = i2c_dev->dev.platform_data;
	}
#else
	pdata = i2c_dev->dev.platform_data;
#endif
	if (pdata->bl_gpio && gpio_request(pdata->bl_gpio, "lm3639 en") != 0) {
		return -ENODEV;
	}

	lm3639_i2c_client = i2c_dev;

	dev = kzalloc(sizeof(struct lm3639_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev, "fail alloc for lm3639_device\n");
		return 0;
	}
	main_lm3639_dev = dev;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;

	props.max_brightness = MAX_BRIGHTNESS_LM3639;
	bl_dev = backlight_device_register(I2C_BL_NAME, &i2c_dev->dev,
			NULL, &lm3639_bl_ops, &props);
	bl_dev->props.max_brightness = MAX_BRIGHTNESS_LM3639;
	bl_dev->props.brightness = DEFAULT_BRIGHTNESS;
	bl_dev->props.power = FB_BLANK_UNBLANK;

	dev->bl_dev = bl_dev;
	dev->client = i2c_dev;

	dev->bl_gpio = pdata->bl_gpio;
	dev->max_current = pdata->max_current;
	dev->min_brightness = pdata->min_brightness;
	dev->default_brightness = pdata->default_brightness;
	dev->max_brightness = pdata->max_brightness;
	dev->blmap_size = pdata->blmap_size;

	if (dev->blmap_size) {
		dev->blmap = kzalloc(sizeof(char) * dev->blmap_size, GFP_KERNEL);
		if (!dev->blmap) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		memcpy(dev->blmap, pdata->blmap, dev->blmap_size);
	} else {
		dev->blmap = NULL;
	}

	if (gpio_get_value(dev->bl_gpio))
		backlight_status = BL_ON;
	else
		backlight_status = POWER_OFF;

	i2c_set_clientdata(i2c_dev, dev);

	mutex_init(&dev->bl_mutex);

	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3639_level);
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3639_backlight_on_off);
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3639_exp_min_value);
#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3639_pwm);
#endif
	init_complete = 1;
#if defined(LM3639_FLED_EN)
if (!id) {
		pr_err("lm3639_probe: id is NULL");
		id = lm3639_bl_id;
	}
	return msm_flash_i2c_probe(i2c_dev, id);
#endif
	return 0;
}

static int lm3639_remove(struct i2c_client *i2c_dev)
{
	struct lm3639_device *dev;

	device_remove_file(&i2c_dev->dev, &dev_attr_lm3639_level);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3639_backlight_on_off);
	dev = (struct lm3639_device *)i2c_get_clientdata(i2c_dev);
	backlight_device_unregister(dev->bl_dev);
	i2c_set_clientdata(i2c_dev, NULL);

	if (gpio_is_valid(main_lm3639_dev->bl_gpio))
		gpio_free(main_lm3639_dev->bl_gpio);
	init_complete = 0;
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id lm3639_match_table[] = {
#ifdef LM3639_FLED_EN
	{ .compatible = "qcom,led-flash",},
	{ },
#else
	{ .compatible = "backlight,lm3639",},
	{ },
#endif
};
#endif

static struct i2c_driver main_lm3639_driver = {
	.probe = lm3639_probe,
	.remove = lm3639_remove,
	.suspend = NULL,
	.resume = NULL,
	.id_table = lm3639_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3639_match_table,
#endif
	},
};

static int __init lcd_backlight_init(void)
{
	static int err;

	err = i2c_add_driver(&main_lm3639_driver);

	return err;
}

#if defined(LM3639_FLED_EN)
static struct msm_flash_fn_t lm3639_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3639_flash_i2c_client,
	.func_tbl = &lm3639_func_tbl,
};
#endif

module_init(lcd_backlight_init);
MODULE_DESCRIPTION("Texas Instruments Backlight+Flash LED driver for lm3639");
MODULE_AUTHOR("Hanyoung Kim <hanyoung.kim@lge.com>");
MODULE_LICENSE("GPL v2");
