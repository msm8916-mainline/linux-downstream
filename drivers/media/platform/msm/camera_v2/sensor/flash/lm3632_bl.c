/* drivers/video/backlight/lm3632_bl.c
 *
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
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>

/*#include <mach/board_lge.h>*/
/*#define LM3632_FLED_EN*/
#include <linux/platform_data/lm3632_bl.h>
#include <linux/regulator/consumer.h>
#if defined(LM3632_FLED_EN)
#include "msm_led_flash.h"
#endif

#if defined(CONFIG_BL_CURVE)
#include <linux/ctype.h>
#endif

/*#undef pr_debug
#define pr_debug pr_err*/

#if defined(LM3632_FLED_EN)
#define I2C_BL_NAME                              "qcom,led-flash"
#else
#define I2C_BL_NAME                              "lm3632_bl"
#endif

#define MAX_BRIGHTNESS_LM3632                    0xFF
#define MIN_BRIGHTNESS_LM3632                    0x05
#define DEFAULT_BRIGHTNESS                       0x1F
#define BL_ON        1
#define BL_OFF       0

#define BOOT_BRIGHTNESS 1

#if defined(LM3632_FLED_EN)
static struct msm_camera_i2c_client lm3632_flash_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
#endif
static struct i2c_client *lm3632_i2c_client;

static int store_level_used;

static int init_complete;
#if defined(LM3632_FLED_EN)
static struct msm_led_flash_ctrl_t fctrl;

static const struct i2c_device_id lm3632_bl_id[] = {
	{ I2C_BL_NAME, (kernel_ulong_t)&fctrl},
	{ },
};
#else
static const struct i2c_device_id lm3632_bl_id[] = {
	{ I2C_BL_NAME, 0 },
	{ },
};
#endif

static int cur_main_lcd_level = DEFAULT_BRIGHTNESS;
static int saved_main_lcd_level = DEFAULT_BRIGHTNESS;
static int backlight_status = BL_OFF;
static int lm3632_pwm_enable;
static struct lm3632_device *main_lm3632_dev;

static void lm3632_set_main_current_level(struct i2c_client *client, int level);

static int bl_set_intensity(struct backlight_device *bd)
{
	struct i2c_client *client = to_i2c_client(bd->dev.parent);

	/*                                                                  */
	if (bd->props.brightness == cur_main_lcd_level) {
		pr_err("%s level is already set. skip it\n", __func__);
		return 0;
	}

	lm3632_set_main_current_level(client, bd->props.brightness);
	cur_main_lcd_level = bd->props.brightness;

	return 0;
}

static int bl_get_intensity(struct backlight_device *bd)
{
	unsigned char val = 0;
	val &= 0x1f;

	return (int)val;
}

static const struct backlight_ops lm3632_bl_ops = {
	.update_status = bl_set_intensity,
	.get_brightness = bl_get_intensity,
};

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static int lm3632_read_reg(struct i2c_client *client, u8 reg, u8 *buf);
#endif

static int lm3632_write_reg
	(struct i2c_client *client, unsigned char reg, unsigned char val);

#ifdef CONFIG_LGE_WIRELESS_CHARGER
int wireless_backlight_state(void)
{
	return backlight_status;
}
EXPORT_SYMBOL(wireless_backlight_state);
#endif

static void lm3632_backlight_init(void)
{
	/* OVP = 32V , MAX BLED = 12.1mA, 2-lane */
	lm3632_write_reg(main_lm3632_dev->client, 0x02, 0x70);
	lm3632_write_reg(main_lm3632_dev->client, 0x04, 0x00);
	lm3632_write_reg(main_lm3632_dev->client, 0x05, 0x1F);

	lm3632_write_reg(main_lm3632_dev->client, 0x0A, 0x11);
}

static void lm3632_dsv_init(void)
{
	/* OVP = 32V , MAX BLED = 12.1mA, 2-lane */
	/*Enable Discharge and set GPIO mode for DSV	*/
	lm3632_write_reg(main_lm3632_dev->client, 0x0C, 0x01);
	/*Set +5.5V*/
	lm3632_write_reg(main_lm3632_dev->client, 0x0E, 0x1E);
	/*Set -5.5V*/
	lm3632_write_reg(main_lm3632_dev->client, 0x0F, 0x1E);
}

static void lm3632_ic_enable(void)
{
	int gpio = main_lm3632_dev->bl_gpio;
	/*                                      */
	if (gpio_is_valid(gpio)) {
		gpio_direction_output(gpio, 1);
		gpio_set_value_cansleep(gpio, 1);
		mdelay(10);
	} else
		pr_err("%s: gpio is not valid !!\n", __func__);
}

static int lm3632_poweron(void)
{
	static struct regulator *vreg_l6;
	int rc = 0;

	/* 1.8V */
	if (!vreg_l6) {
		vreg_l6 = regulator_get(&main_lm3632_dev->client->dev,
								"lm3632,vcc_i2c");
		if (IS_ERR(vreg_l6)) {
			pr_err("%s: regulator get of pm8909_l6 failed (%ld)\n",
					__func__, PTR_ERR(vreg_l6));
			rc = PTR_ERR(vreg_l6);
			vreg_l6 = NULL;
			return rc;
		}
	}

	rc = regulator_enable(vreg_l6);

	return rc;
}

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static int lm3632_read_reg(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		pr_err("[LCD][DEBUG] error\n");

	*buf = ret;

	return 0;

}
#endif
static int lm3632_write_reg(struct i2c_client *client,
				unsigned char reg, unsigned char val)
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
		dev_err(&client->dev, "i2c write error reg: %d, val: %d\n",
			buf[0], buf[1]);
	return 0;
}

static int exp_min_value = 150;
static int cal_value;

static void lm3632_set_main_current_level(struct i2c_client *client, int level)
{
	struct lm3632_device *dev = i2c_get_clientdata(client);
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
				lm3632_write_reg(client, 0x05,
						cal_value);
			} else
				dev_warn(&client->dev, "invalid index %d:%d\n",
						dev->blmap_size,
						level);
		} else {
			cal_value = level;
			lm3632_write_reg(client, 0x05, cal_value);
		}
	} else
		lm3632_write_reg(client, 0x05, 0x00);

	mutex_unlock(&dev->bl_mutex);

	pr_debug("%s : level=%d, cal_value=%d\n",
				__func__, level, cal_value);
}

static void lm3632_set_main_current_level_no_mapping(
		struct i2c_client *client, int level)
{
	struct lm3632_device *dev;
	dev = (struct lm3632_device *)i2c_get_clientdata(client);

	if (level > 255)
		level = 255;
	else if (level < 0)
		level = 0;

	cur_main_lcd_level = level;
	dev->bl_dev->props.brightness = cur_main_lcd_level;

	store_level_used = 1;

	mutex_lock(&main_lm3632_dev->bl_mutex);
	if (level != 0)
		lm3632_write_reg(client, 0x05, level);
	else
		lm3632_write_reg(client, 0x00, 0x00);

	mutex_unlock(&main_lm3632_dev->bl_mutex);
}

void lm3632_backlight_on(int level)
{
	if (backlight_status == BL_OFF) {
		if (!init_complete) {
			lm3632_ic_enable();
			lm3632_backlight_init();
		}
	}
	mdelay(1);

	if (backlight_status == BL_OFF)
		pr_info("mdss %s\n", __func__);

	lm3632_set_main_current_level(main_lm3632_dev->client, level);
	backlight_status = BL_ON;

	return;
}

void lm3632_backlight_off(void)
{

	if (backlight_status == BL_OFF)
		return;

	saved_main_lcd_level = cur_main_lcd_level;
	pr_info("mdss %s\n", __func__);
	lm3632_set_main_current_level(main_lm3632_dev->client, 0);
	backlight_status = BL_OFF;

	return;
}

void lm3632_lcd_backlight_set_level(int level)
{
	if (level > MAX_BRIGHTNESS_LM3632)
		level = MAX_BRIGHTNESS_LM3632;

	if (lm3632_i2c_client != NULL) {
		if (level == 0)
			lm3632_backlight_off();
		else
			lm3632_backlight_on(level);
	} else {
		pr_err("%s(): No client\n", __func__);
	}
}
EXPORT_SYMBOL(lm3632_lcd_backlight_set_level);

static ssize_t lcd_backlight_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r = 0;

	if (store_level_used == 0)
		r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
				cal_value);
	else if (store_level_used == 1)
		r = snprintf(buf, PAGE_SIZE, "LCD Backlight Level is : %d\n",
				cur_main_lcd_level);

	return r;
}

static ssize_t lcd_backlight_store_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long level;
	struct i2c_client *client = to_i2c_client(dev);
	int err = 0;

	if (!count)
		return -EINVAL;

	err = kstrtoul(buf, 10, &level);
	if (err != 0)
		return err;

	lm3632_set_main_current_level_no_mapping(client, level);

	return count;
}

#if defined(CONFIG_BL_CURVE)
static ssize_t lcd_backlight_show_blmap(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3632_device *lm3632_dev = i2c_get_clientdata(client);
	int i, j;

	if (!lm3632_dev->blmap_size)
		return 0;

	buf[0] = '{';

	for (i = 0, j = 2; i < lm3632_dev->blmap_size && j < PAGE_SIZE; ++i) {
		if (!(i % 15)) {
			buf[j] = '\n';
			++j;
		}

		sprintf(&buf[j], "%d, ", lm3632_dev->blmap[i]);
		if (lm3632_dev->blmap[i] < 10)
			j += 3;
		else if (lm3632_dev->blmap[i] < 100)
			j += 4;
		else
			j += 5;
	}

	buf[j] = '\n';
	++j;
	buf[j] = '}';
	++j;

	return j;
}

static ssize_t lcd_backlight_store_blmap(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3632_device *lm3632_dev = i2c_get_clientdata(client);
	static char *blmap;
	int i;
	int j;
	int value, ret;

	if (count < 1)
		return count;

	if (buf[0] != '{')
		return -EINVAL;

	blmap = kzalloc(sizeof(char) * lm3632_dev->blmap_size, GFP_KERNEL);

	for (i = 1, j = 0; i < count && j < lm3632_dev->blmap_size; ++i) {
		if (!isdigit(buf[i]))
			continue;

		ret = sscanf(&buf[i], "%d", &value);
		if (ret < 1)
			pr_err("read error\n");
		blmap[j] = (char)value;

		while (isdigit(buf[i]))
			++i;
		++j;
	}

	for (i = 0; i < lm3632_dev->blmap_size; i++)
		lm3632_dev->blmap[i] = blmap[i];

	kfree(blmap);

	return count;
}
#endif

static int lm3632_bl_resume(struct i2c_client *client)
{
	lm3632_lcd_backlight_set_level(saved_main_lcd_level);
	return 0;
}

static int lm3632_bl_suspend(struct i2c_client *client, pm_message_t state)
{
	lm3632_lcd_backlight_set_level(saved_main_lcd_level);
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
	unsigned long on_off;
	struct i2c_client *client = to_i2c_client(dev);
	int err = 0;

	if (!count)
		return -EINVAL;

	pr_info("%s received (prev backlight_status: %s)\n",
			__func__, backlight_status ? "ON" : "OFF");

	err = kstrtoul(buf, 10, &on_off);
	if (err != 0)
		return err;

	pr_info("[LCD][DEBUG] %ld", on_off);

	if (on_off == 1)
		lm3632_bl_resume(client);
	else if (on_off == 0)
		lm3632_bl_suspend(client, PMSG_SUSPEND);

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
	unsigned long value;
	int err = 0;

	if (!count)
		return -EINVAL;

	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;

	exp_min_value = value;

	return count;
}

#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
static ssize_t lcd_backlight_show_pwm(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	u8 level, pwm_low, pwm_high, config;

	mutex_lock(&main_lm3632_dev->bl_mutex);
	lm3632_read_reg(main_lm3632_dev->client, 0x01, &config);
	mdelay(3);
	lm3632_read_reg(main_lm3632_dev->client, 0x03, &level);
	mdelay(3);
	lm3632_read_reg(main_lm3632_dev->client, 0x12, &pwm_low);
	mdelay(3);
	lm3632_read_reg(main_lm3632_dev->client, 0x13, &pwm_high);
	mdelay(3);
	mutex_unlock(&main_lm3632_dev->bl_mutex);

	r = snprintf(buf, PAGE_SIZE,
				"Show PWM level: %d pwm_low: %d pwm_high: %d config: %d\n",
				level, pwm_low, pwm_high, config);

	return r;
}
static ssize_t lcd_backlight_store_pwm(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}
#endif

DEVICE_ATTR(lm3632_level, 0644, lcd_backlight_show_level,
		lcd_backlight_store_level);
DEVICE_ATTR(lm3632_backlight_on_off, 0644, lcd_backlight_show_on_off,
		lcd_backlight_store_on_off);
DEVICE_ATTR(lm3632_exp_min_value, 0644, lcd_backlight_show_exp_min_value,
		lcd_backlight_store_exp_min_value);
#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
DEVICE_ATTR(lm3632_pwm, 0644, lcd_backlight_show_pwm, lcd_backlight_store_pwm);
#endif

#if defined(CONFIG_BL_CURVE)
DEVICE_ATTR(bl_blmap, 0644, lcd_backlight_show_blmap, lcd_backlight_store_blmap);
#endif

#ifdef CONFIG_OF
static int lm3632_parse_dt(struct device *dev,
		struct backlight_platform_data *pdata)
{
	int rc = 0, i;
	u32 *array;
	struct device_node *np = dev->of_node;

	pdata->bl_gpio = of_get_named_gpio_flags
					(np, "lm3632,lcd_bl_en", 0, NULL);
	pdata->dsv_p_gpio = of_get_named_gpio_flags
					(np, "lm3632,dsv_p_en", 0, NULL);
	pdata->dsv_n_gpio = of_get_named_gpio_flags
					(np, "lm3632,dsv_n_en", 0, NULL);
	pdata->init_on_kernel = of_property_read_bool
					(np, "lm3632,init_on_kernel");

	rc = of_property_read_u32(np, "lm3632,max_current",
			&pdata->max_current);
	rc = of_property_read_u32(np, "lm3632,min_brightness",
			&pdata->min_brightness);
	rc = of_property_read_u32(np, "lm3632,default_brightness",
			&pdata->default_brightness);
	rc = of_property_read_u32(np, "lm3632,max_brightness",
			&pdata->max_brightness);

	rc = of_property_read_u32(np, "lm3632,enable_pwm",
			&lm3632_pwm_enable);
	if (rc == -EINVAL)
		lm3632_pwm_enable = 1;

	rc = of_property_read_u32(np, "lm3632,blmap_size",
			&pdata->blmap_size);

	if (pdata->blmap_size) {
		array = kzalloc(sizeof(u32) * pdata->blmap_size, GFP_KERNEL);
		if (!array)
			return -ENOMEM;

		rc = of_property_read_u32_array(np, "lm3632,blmap",
				array, pdata->blmap_size);
		if (rc) {
			pr_err("%s:%d, uable to read backlight map\n",
					__func__, __LINE__);
			return -EINVAL;
		}
		pdata->blmap =
			kzalloc(sizeof(char) * pdata->blmap_size, GFP_KERNEL);

		if (!pdata->blmap)
			return -ENOMEM;

		for (i = 0; i < pdata->blmap_size; i++)
			pdata->blmap[i] = (char)array[i];

		kfree(array);

	} else {
		pdata->blmap = NULL;
	}

	pr_err("%s bl_gpio : %d, dsv_p_gpio : %d, dsv_n_gpio : %d\n",
			__func__, pdata->bl_gpio, pdata->dsv_p_gpio,
			pdata->dsv_n_gpio);

	return rc;
}
#endif

static int lm3632_probe(struct i2c_client *i2c_dev,
		const struct i2c_device_id *id)
{
	struct backlight_platform_data *pdata;
	struct lm3632_device *dev;
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
		err = lm3632_parse_dt(&i2c_dev->dev, pdata);
		if (err != 0)
			return err;
	} else {
		pdata = i2c_dev->dev.platform_data;
	}
#else
	pdata = i2c_dev->dev.platform_data;
#endif
	if (pdata->bl_gpio &&
		gpio_request(pdata->bl_gpio, "lm3632 en") != 0) {
		return -ENODEV;
	}

	if (pdata->dsv_p_gpio &&
		gpio_request(pdata->dsv_p_gpio, "lm3632 dsv p") != 0) {
		return -ENODEV;
	}
	if (pdata->dsv_n_gpio &&
		gpio_request(pdata->dsv_n_gpio, "lm3632 dsv n") != 0) {
		return -ENODEV;
	}

	lm3632_i2c_client = i2c_dev;

	dev = kzalloc(sizeof(struct lm3632_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&i2c_dev->dev, "fail alloc for lm3632_device\n");
		return 0;
	}
	main_lm3632_dev = dev;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;

	props.max_brightness = MAX_BRIGHTNESS_LM3632;
	bl_dev = backlight_device_register(I2C_BL_NAME, &i2c_dev->dev,
			NULL, &lm3632_bl_ops, &props);
	bl_dev->props.max_brightness = MAX_BRIGHTNESS_LM3632;
	bl_dev->props.brightness = DEFAULT_BRIGHTNESS;
	bl_dev->props.power = FB_BLANK_UNBLANK;

	dev->bl_dev = bl_dev;
	dev->client = i2c_dev;

	dev->bl_gpio = pdata->bl_gpio;
	dev->dsv_p_gpio = pdata->dsv_p_gpio;
	dev->dsv_n_gpio = pdata->dsv_n_gpio;
	dev->max_current = pdata->max_current;
	dev->min_brightness = pdata->min_brightness;
	dev->default_brightness = pdata->default_brightness;
	dev->max_brightness = pdata->max_brightness;
	dev->blmap_size = pdata->blmap_size;

	if (dev->blmap_size) {
		dev->blmap =
			kzalloc(sizeof(char) * dev->blmap_size, GFP_KERNEL);
		if (!dev->blmap) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		memcpy(dev->blmap, pdata->blmap, dev->blmap_size);
	} else {
		dev->blmap = NULL;
	}

	i2c_set_clientdata(i2c_dev, dev);

	if (pdata->init_on_kernel) {
		lm3632_poweron();
		lm3632_ic_enable();
		lm3632_backlight_init();
		lm3632_dsv_init();
	}
	backlight_status = BL_OFF;

	mutex_init(&dev->bl_mutex);

	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3632_level);
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3632_backlight_on_off);
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3632_exp_min_value);
#if defined(CONFIG_BACKLIGHT_CABC_DEBUG_ENABLE)
	err = device_create_file(&i2c_dev->dev,
			&dev_attr_lm3632_pwm);
#endif
	init_complete = 1;
#if defined(CONFIG_BL_CURVE)
	err = device_create_file(&i2c_dev->dev, &dev_attr_bl_blmap);
#endif
#if defined(LM3632_FLED_EN)
if (!id) {
		pr_err("lm3632_probe: id is NULL");
		id = lm3632_bl_id;
	}
	return msm_flash_i2c_probe(i2c_dev, id);
#endif
	return 0;
}

static int lm3632_remove(struct i2c_client *i2c_dev)
{
	struct lm3632_device *dev;

	device_remove_file(&i2c_dev->dev, &dev_attr_lm3632_level);
	device_remove_file(&i2c_dev->dev, &dev_attr_lm3632_backlight_on_off);
	dev = (struct lm3632_device *)i2c_get_clientdata(i2c_dev);
	backlight_device_unregister(dev->bl_dev);

	i2c_set_clientdata(i2c_dev, NULL);

	if (gpio_is_valid(main_lm3632_dev->bl_gpio))
		gpio_free(main_lm3632_dev->bl_gpio);
	if (gpio_is_valid(main_lm3632_dev->dsv_n_gpio))
		gpio_free(main_lm3632_dev->dsv_n_gpio);
	if (gpio_is_valid(main_lm3632_dev->dsv_p_gpio))
		gpio_free(main_lm3632_dev->dsv_p_gpio);
	init_complete = 0;
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id lm3632_match_table[] = {
#ifdef LM3632_FLED_EN
	{ .compatible = "qcom,led-flash",},
	{ },
#else
	{ .compatible = "backlight,lm3632",},
	{ },
#endif
};
#endif

static struct i2c_driver main_lm3632_driver = {
	.probe = lm3632_probe,
	.remove = lm3632_remove,
	.suspend = NULL,
	.resume = NULL,
	.id_table = lm3632_bl_id,
	.driver = {
		.name = I2C_BL_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3632_match_table,
#endif
	},
};

static int __init lcd_backlight_init(void)
{
	static int err;

	err = i2c_add_driver(&main_lm3632_driver);

	return err;
}

#if defined(LM3632_FLED_EN)
static struct msm_flash_fn_t lm3632_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3632_flash_i2c_client,
	.func_tbl = &lm3632_func_tbl,
};
#endif

module_init(lcd_backlight_init);
MODULE_DESCRIPTION("Texas Instruments Backlight+Flash LED driver for lm3632");
MODULE_AUTHOR("Hanyoung Kim <hanyoung.kim@lge.com>");
MODULE_LICENSE("GPL v2");
