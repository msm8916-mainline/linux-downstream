/*
 * leds-sn31xx.c - RGB LED Driver
 *
 * Copyright (C) 2014 TCL Communication Technology (Ningbo) Co., LTD.
 * Feng Longfei <longfei.feng@tcl.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * now is only green led is used,so disalbe other led,but ic is support RGB
 * Datasheet:
 * ===================================================================================
 *                             EDIT HISTORY FOR MODULE
 *
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 *	when       who        what, where, why
 * ------------------------------------------------------------------------------------
 * 10/18/2014  FLF      |RR-813549, sn31xx extern led ic chip
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>


/* POWER SUPPLY VOLTAGE RANGE */
#define sn31xx_VDD_MIN_UV  2000000
#define sn31xx_VDD_MAX_UV  3300000
#define sn31xx_VIO_MIN_UV        1750000
#define sn31xx_VIO_MAX_UV       1950000

enum led_ids {
    LED1,
	LED2,
    LED_NUM,
};

enum led_colors {
    RED,
    GREEN,
    BLUE,
};

enum led_bits {
    sn31xx_OFF,
    sn31xx_BLINK,
    sn31xx_ON,
};

/*
 * State '0' : 'off'
 * State '1' : 'blink'
 * State '2' : 'on'.
 */
struct led_state {
    unsigned r:2;
    unsigned g:2;
    unsigned b:2;
};
struct sn31xx_led_platform_data{
	unsigned int	en_gpio;
	
};

struct sn31xx_led {
    struct sn31xx_led_platform_data    *pdata;
    struct i2c_client       *client;
    struct rw_semaphore     rwsem;
    struct work_struct      work;

    struct led_state		led[2];
    struct regulator		*vio;
	struct regulator		*vdd;
    int						power_enabled;
    unsigned int	en_gpio;/*[BUGFIX]-del   by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
    /*
     * Making led_classdev as array is not recommended, because array
     * members prevent using 'container_of' macro. So repetitive works
     * are needed.
     */
    struct led_classdev     cdev_led1g;
    /*
     * Advanced Configuration Function(ADF) mode:
     * In ADF mode, user can set registers of sn31xxGU directly,
     * therefore sn31xxGU doesn't enter reset state.
     */
    int						adf_on;
	struct pinctrl			*sn31xx_pinctrl;
	struct pinctrl_state	*gpio_state;
    enum led_ids            led_id;
    enum led_colors         color;
    enum led_bits           state;
};
static int sn31xx_power_init(struct sn31xx_led *data, bool on)
{
    int rc;

    if (!on) {
        if (regulator_count_voltages(data->vio) > 0)
              regulator_set_voltage(data->vio, 0,sn31xx_VIO_MAX_UV);
           regulator_put(data->vio);
	} else {
		data->vio = regulator_get(&data->client->dev, "vio");
			if (IS_ERR(data->vio)) {
				rc = PTR_ERR(data->vio);
				dev_err(&data->client->dev,
						"Regulator get failed vio rc=%d\n", rc);
				return rc;
			}
			if (regulator_count_voltages(data->vio) > 0) {
				rc = regulator_set_voltage(data->vio,
									sn31xx_VIO_MIN_UV, sn31xx_VIO_MAX_UV);
				if (rc) {
					dev_err(&data->client->dev,
							"Regulator set failed vio rc=%d\n", rc);
				return rc;
				}
			}
		}

	return 0;
}
static int sn31xx_power_set(struct sn31xx_led *data, bool on)
{
    int rc = 0;

    if (!on && data->power_enabled) {
		rc = regulator_disable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vio disable failed rc=%d\n", rc);
            return rc;
        }
		data->power_enabled = false;
        return rc;
    } else if (on && !data->power_enabled) {
        rc = regulator_enable(data->vio);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator vio enable failed rc=%d\n", rc);
            return rc;
        }
        data->power_enabled = true;

        /*
         * The max time for the power supply rise time is 50ms.
         * Use 80ms to make sure it meets the requirements.
         */
        msleep(80);
        return rc;
	} else {
        dev_warn(&data->client->dev,
                "Power on=%d. enabled=%d\n",
                on, data->power_enabled);
        return rc;
    }
}
/*--------------------------------------------------------------*/
/*  sn31xxGU core functions                    */
/*--------------------------------------------------------------*/

static int sn31xx_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret >= 0)
        return 0;

    dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
                        __func__, reg, val, ret);

    return ret;
}

#define sn31xx_SET_REGISTER(reg_addr, reg_name)                \
static ssize_t sn31xx_store_reg##reg_addr(struct device *dev,      \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn31xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    msleep(500);                                                \
    sn31xx_write_byte(led->client, reg_addr, (u8) val);        \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute sn31xx_reg##reg_addr##_attr = {     \
    .attr = {.name = reg_name, .mode = 0644},           \
    .store = sn31xx_store_reg##reg_addr,               \
};

sn31xx_SET_REGISTER(0x00, "0x00");
sn31xx_SET_REGISTER(0x01, "0x01");
sn31xx_SET_REGISTER(0x02, "0x02");
sn31xx_SET_REGISTER(0x03, "0x03");
sn31xx_SET_REGISTER(0x04, "0x04");
sn31xx_SET_REGISTER(0x07, "0x07");
sn31xx_SET_REGISTER(0x0a, "0x0a");
sn31xx_SET_REGISTER(0x10, "0x10");
sn31xx_SET_REGISTER(0x16, "0x16");
sn31xx_SET_REGISTER(0x1c, "0x1c");
sn31xx_SET_REGISTER(0x1d, "0x1d");
sn31xx_SET_REGISTER(0x2f, "0x2f");


static struct device_attribute *sn31xx_addr_attributes[] = {
    &sn31xx_reg0x00_attr,
    &sn31xx_reg0x01_attr,
    &sn31xx_reg0x02_attr,
    &sn31xx_reg0x03_attr,
    &sn31xx_reg0x04_attr,
    &sn31xx_reg0x07_attr,
    &sn31xx_reg0x0a_attr,
    &sn31xx_reg0x10_attr,
    &sn31xx_reg0x16_attr,
    &sn31xx_reg0x1c_attr,
    &sn31xx_reg0x1d_attr,
    &sn31xx_reg0x2f_attr,
};

static void sn31xx_enable_adv_conf(struct sn31xx_led *led)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(sn31xx_addr_attributes); i++) {
        ret = device_create_file(&led->client->dev,
                        sn31xx_addr_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn31xx_addr_attributes[i]->attr.name);
            goto failed_remove_files;
        }
    }
    led->adf_on = 1;

    return;

failed_remove_files:
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev,
                        sn31xx_addr_attributes[i]);
}

static void sn31xx_disable_adv_conf(struct sn31xx_led *led)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(sn31xx_addr_attributes); i++)
        device_remove_file(&led->client->dev,
                        sn31xx_addr_attributes[i]);
    led->adf_on = 0;
}

static ssize_t sn31xx_show_adv_conf(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct sn31xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
    ssize_t ret;

    down_read(&led->rwsem);
    if (led->adf_on)
        ret = sprintf(buf, "on\n");
    else
        ret = sprintf(buf, "off\n");
    up_read(&led->rwsem);

    return ret;
}

static ssize_t sn31xx_store_adv_conf(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct sn31xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
    if (!count)
        return -EINVAL;

    down_write(&led->rwsem);
    if (!led->adf_on && !strncmp(buf, "on", 2))
        sn31xx_enable_adv_conf(led);
    else if (led->adf_on && !strncmp(buf, "off", 3))
        sn31xx_disable_adv_conf(led);
    up_write(&led->rwsem);

    return count;
}

static struct device_attribute sn31xx_adv_conf_attr = {
    .attr = {
        .name = "advanced_configuration",
        .mode = 0644,
    },
    .show = sn31xx_show_adv_conf,
    .store = sn31xx_store_adv_conf,
};

#define sn31xx_CONTROL_ATTR(attr_name, name_str)           \
static ssize_t sn31xx_show_##attr_name(struct device *dev,     \
    struct device_attribute *attr, char *buf)           \
{                                   \
    struct sn31xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    ssize_t ret;                            \
    down_read(&led->rwsem);                     \
    ret = sprintf(buf, "0x%02x\n", led->attr_name);         \
    up_read(&led->rwsem);                       \
    return ret;                         \
}                                   \
static ssize_t sn31xx_store_##attr_name(struct device *dev,        \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct sn31xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    led->attr_name = val;                       \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute sn31xx_##attr_name##_attr = {       \
    .attr = {                           \
        .name = name_str,                   \
        .mode = 0644,                       \
    },                              \
    .show = sn31xx_show_##attr_name,               \
    .store = sn31xx_store_##attr_name,             \
};

static struct device_attribute *sn31xx_attributes[] = {
    &sn31xx_adv_conf_attr,
};
static void sn31xx_set_led1g_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value)  
{               
    struct sn31xx_led *led =       
        container_of(led_cdev, struct sn31xx_led, cdev_led1g);
 
	sn31xx_power_set(led,1);
	 pr_err("  gpio=%d\n",led->en_gpio);
	gpio_direction_output(led->en_gpio, 1);/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
    if (value == LED_OFF)
       {   
            led->state = sn31xx_OFF;
           sn31xx_write_byte(led->client,0x2f, 0x00); //reset
           sn31xx_write_byte(led->client,0x00, 0x01); //led2 constant on  ,light on
           sn31xx_power_set(led,0);
	      gpio_direction_output(led->en_gpio, 0);/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
    }
      else
      {
			msleep(100);
            sn31xx_write_byte(led->client,0x2f, 0x00); //reset
            sn31xx_write_byte(led->client,0x00, 0x20); //
            sn31xx_write_byte(led->client,0x03, 0x04); //
            sn31xx_write_byte(led->client,0x02, 0x00); // 
            sn31xx_write_byte(led->client,0x04, 0xff); //10ma
            sn31xx_write_byte(led->client,0x07, 0xff); //update
            led->state = sn31xx_ON;
      }
}                                   
static void sn31xx_set_led1g_blink(struct led_classdev *led_cdev,u8 bblink)
{                               
	struct sn31xx_led *led = container_of(led_cdev,
										struct sn31xx_led, cdev_led1g);

	sn31xx_power_set(led,1);
	    pr_err("  gpio=%d\n",led->en_gpio);
       gpio_direction_output(led->en_gpio, 1);/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
	msleep(100);
	if(bblink==1) {
		pr_err("blink on mode 1 \n");
        sn31xx_write_byte(led->client,0x2f, 0x00); //reset
		sn31xx_write_byte(led->client,0x02, 0x20); //reset
        sn31xx_write_byte(led->client,0x00, 0x20); //reset          

        sn31xx_write_byte(led->client,0x03,0x04); //i max 10ma 
        sn31xx_write_byte(led->client,0x04, 0xc8); 
        sn31xx_write_byte(led->client,0x07, 0xff); 

		sn31xx_write_byte(led->client,0x0a, 0x00); //T0 1S
        sn31xx_write_byte(led->client,0x10, 0x40);//T1,Tt2
        sn31xx_write_byte(led->client,0x16, 0x48);//T3,T4 
		sn31xx_write_byte(led->client,0x1C, 0x00);//UPDATE

        led->state = sn31xx_BLINK;
	} else if(bblink==0) {			//constant 
           pr_err("blink off\n");
           msleep(100);
           sn31xx_write_byte(led->client,0x2f, 0x00); //reset
           sn31xx_write_byte(led->client,0x00, 0x20); //
           sn31xx_write_byte(led->client,0x03, 0x04); //
           sn31xx_write_byte(led->client,0x02, 0x00); // 
           sn31xx_write_byte(led->client,0x04, 0xff); //10ma
           sn31xx_write_byte(led->client,0x07, 0xff); //update
          
           led->state = sn31xx_ON;
	} else if (bblink==2) {
		pr_err("blink on mode 2 \n");
		sn31xx_write_byte(led->client,0x2f, 0x00); //reset
		sn31xx_write_byte(led->client,0x02, 0x20); //led mode
	    sn31xx_write_byte(led->client,0x00, 0x20);
		sn31xx_write_byte(led->client,0x03,0x04); //i max 10ma 
		sn31xx_write_byte(led->client,0x04, 0xc8); 
		sn31xx_write_byte(led->client,0x07, 0xff); 

		sn31xx_write_byte(led->client,0x0a, 0x00); //T0 1S
		sn31xx_write_byte(led->client,0x10, 0x40);//T1,Tt2
		//sn31xx_write_byte(led->client,0x16, 0x66);//T3,T4 
		sn31xx_write_byte(led->client,0x16, 0x48);//T3,T4
		sn31xx_write_byte(led->client,0x1C, 0x00);//UPDATE
		led->state = sn31xx_BLINK;
	   
	}
}


static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
{

      struct led_classdev *led_cdev = dev_get_drvdata(dev);
      u8 bblink;
      pr_err("in %s,name=%s\n",__func__,led_cdev->name);
      if(*buf=='0')
        bblink=0;
      else   if(*buf=='2')
        bblink=2;
     else
	    bblink=1;

      sn31xx_set_led1g_blink(led_cdev,bblink);
      return count;
}


static DEVICE_ATTR(blink, S_IWUSR, NULL, store_blink);


/* TODO: HSB, fade, timeadj, script ... */

static int sn31xx_register_led_classdev(struct sn31xx_led *led)
{
    int ret;
    led->cdev_led1g.name = "led_G";
    led->cdev_led1g.brightness = LED_OFF;
    led->cdev_led1g.brightness_set = sn31xx_set_led1g_brightness;

    ret = led_classdev_register(&led->client->dev, &led->cdev_led1g);
    if (ret < 0) {
        dev_err(&led->client->dev, "couldn't register LED %s\n",
                            led->cdev_led1g.name);
        goto failed_unregister_led1_G;
    }
    return 0;

failed_unregister_led1_G:
    led_classdev_unregister(&led->cdev_led1g);

    return ret;
}

static int sn31xx_init(struct i2c_client *client)
{
     int ret=0;
 
	ret= sn31xx_write_byte(client,0x2f, 0x00); //reset
  
   

       return ret;
}
/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
static int sn31xx_parse_dt(struct device *dev, struct sn31xx_led *led)
{
	int r = 0;

   led->en_gpio = of_get_named_gpio_flags(dev->of_node,
			"sn31,en-gpio", 0, NULL);

	if ((!gpio_is_valid(led->en_gpio)))
		return -EINVAL;
	return r;
}
/*[BUGFIX]-End by TCTNB.XQJ*/

extern void qnnp_lbc_enable_led(u8 onoff);
static int sn31xx_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int ret, i;
    struct sn31xx_led_platform_data *pdata, platformdata;
    struct sn31xx_led *led = devm_kzalloc(&client->dev,
										sizeof(struct sn31xx_led), GFP_KERNEL);
    pdata = &platformdata;
    if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
        return -ENOMEM;
    }
    sn31xx_parse_dt(&client->dev, led);/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
	led->pdata=pdata;
	led->client = client;
  
	i2c_set_clientdata(client, led);
	
/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/	
    pr_err("sn31 gpio=%d\n",led->en_gpio);
	ret = gpio_request(led->en_gpio, "sn31_en");
	if (ret)
		return  -ENODEV;
/*[BUGFIX]-End by TCTNB.XQJ*/
	ret = sn31xx_power_init(led, 1);
    if (ret < 0)
        goto exit1;
    ret = sn31xx_power_set(led, 1);
    if (ret < 0)
        goto exit2;
    gpio_direction_output(led->en_gpio, 0);/*[BUGFIX]-Add by TCTNB.XQJ,PR-817507 2014/10/23,add  led ic sn31xx,gpio is used for led ic*/
    udelay(100);
    ret=   sn31xx_init(client);
    if (ret < 0)
        goto exit3;


	sn31xx_power_set(led, 0);
    	    
    init_rwsem(&led->rwsem);

    ret = sn31xx_register_led_classdev(led);
	
    if (ret < 0)
        goto  exit4;
	
    for (i = 0; i < ARRAY_SIZE(sn31xx_attributes); i++) {   
        ret = device_create_file(&led->client->dev,
                        sn31xx_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    sn31xx_attributes[i]->attr.name);
            goto exit4;
        }
    }

	ret= sysfs_create_file (&led->cdev_led1g.dev->kobj, &dev_attr_blink.attr);
	qnnp_lbc_enable_led(0);
    return 0;

exit4:
	i=0;
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev, sn31xx_attributes[i]);
exit3:
    sn31xx_power_set(led, 0);
exit2:
    sn31xx_power_init(led, 0);
exit1:
//    kfree(led);
    qnnp_lbc_enable_led(1);
    return ret;
}

static int sn31xx_remove(struct i2c_client *client)
{
    struct sn31xx_led *led = i2c_get_clientdata(client);
    int i;

	led_classdev_unregister(&led->cdev_led1g);
    if (led->adf_on)
        sn31xx_disable_adv_conf(led);
    for (i = 0; i < ARRAY_SIZE(sn31xx_attributes); i++)
        device_remove_file(&led->client->dev, sn31xx_attributes[i]);
    return 0;
}

static const struct i2c_device_id sn31xx_id[] = {
    { "sn31xx", 0 },
    { }
};

static struct of_device_id sn31xx_match_table[] = {
    { .compatible = "sn31,sn31xx", },
    { },
};

static struct i2c_driver sn31xx_i2c_driver = {
    .probe		= sn31xx_probe,
    .remove		= sn31xx_remove,
    .id_table	= sn31xx_id,
    .driver = {
				.name    = "sn31xx",
				.owner  = THIS_MODULE,
				.of_match_table = sn31xx_match_table,
	},
};
static int __init sn31xx_i2c_init(void)
{
	return i2c_add_driver(&sn31xx_i2c_driver);
}
late_initcall(sn31xx_i2c_init);
MODULE_AUTHOR("flf<longfei.feng@tcl.com>");
MODULE_DESCRIPTION("sn31xx LED driver");
MODULE_LICENSE("GPL v2");
