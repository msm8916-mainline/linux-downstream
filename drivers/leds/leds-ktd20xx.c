/*fangang.luo 2015.12.17 task:1176409 START*/
/*
 * leds-ktd2xx.c - RGB LED Driver
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
 * 01/21/2014  XQJ      |RR-901590, kdt2xxx external led ic chip
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
#define ktd20xx_VDD_MIN_UV  2000000
#define ktd20xx_VDD_MAX_UV  3300000
#define ktd20xx_VIO_MIN_UV        1750000
#define ktd20xx_VIO_MAX_UV       1950000

#define  ktd20xx_REG_CONTROL  0x04
#define  ktd20xx_REG_LED1CURRENT  0x06
#define  ktd20xx_REG_LED2CURRENT  0x07
#define  ktd20xx_REG_LED3CURRENT  0x08
#define  ktd20xx_REG_LED4CURRENT  0x09
#define LED_NUM  3

enum reg_control {
     ALWAYSOFF,
     ALWAYSON,
     PWM1,                     //now main use  pwm1
     PWM2,
};

enum led_ids {
    LEDR,
    LEDG,
    LEDKBD,
};


enum led_state {
    ktd20xx_OFF,
    ktd20xx_BLINK,
    ktd20xx_BLINKSHORT,
    ktd20xx_ON,
};

struct ktd20xx_led {
    struct ktd20xx_led_platform_data    *pdata;
    struct i2c_client       *client;
    struct rw_semaphore     rwsem;
    struct work_struct      work;

    struct regulator		*vio;
    struct regulator		*vdd;
    int						power_enabled;
    /*
     * Making led_classdev as array is not recommended, because array
     * members prevent using 'container_of' macro. So repetitive works
     * are needed.
     */
    struct led_classdev     cdev_ledg;
    struct led_classdev		cdev_ledr;
    struct led_classdev		cdev_kbd;
     unsigned int	en_gpio;/*[BUGFIX]-Add by TCTNB.ZXZ,PR-938243 2015/03/04add  led ic ktd2xx ,gpio 108, enable  red*/
	

    int						adf_on;
	struct pinctrl			*ktd20xx_pinctrl;
	struct pinctrl_state	*gpio_state;
    enum led_ids            led_id;
    enum led_state           state[3];
};
struct i2c_client *ktd20xx_client;



static int ktd20xx_power_init(struct ktd20xx_led *data, bool on)
{
    int rc;

    if (!on) {
        if (regulator_count_voltages(data->vio) > 0)
              regulator_set_voltage(data->vio, 0,ktd20xx_VIO_MAX_UV);
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
									ktd20xx_VIO_MIN_UV, ktd20xx_VIO_MAX_UV);
				if (rc) {
					dev_err(&data->client->dev,
							"Regulator set failed vio rc=%d\n", rc);
				return rc;
				}
			}
		}

	return 0;
}
static int ktd20xx_power_set(struct ktd20xx_led *data, bool on)
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
/* ktd20xx GU core functions                    */
/*--------------------------------------------------------------*/

static int ktd20xx_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
    int ret = i2c_smbus_write_byte_data(client, reg, val);
    if (ret >= 0)
        return 0;

    dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err=%d, addr=0x%04x\n",
                        __func__, reg, val, ret,client->addr);

    return ret;
}

#define ktd20xx_SET_REGISTER(reg_addr, reg_name)                \
static ssize_t ktd20xx_store_reg##reg_addr(struct device *dev,      \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct ktd20xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    unsigned long val;                      \
    int ret;                            \
    if (!count)                         \
        return -EINVAL;                     \
    ret = kstrtoul(buf, 16, &val);                  \
    if (ret)                            \
        return ret;                     \
    down_write(&led->rwsem);                    \
    msleep(500);                                                \
    ktd20xx_write_byte(led->client, reg_addr, (u8) val);        \
    up_write(&led->rwsem);                      \
    return count;                           \
}                                   \
static struct device_attribute ktd20xx_reg##reg_addr##_attr = {     \
    .attr = {.name = reg_name, .mode = 0644},           \
    .store = ktd20xx_store_reg##reg_addr,         \
};

ktd20xx_SET_REGISTER(0x00, "0x00");
ktd20xx_SET_REGISTER(0x01, "0x01");
ktd20xx_SET_REGISTER(0x02, "0x02");
ktd20xx_SET_REGISTER(0x03, "0x03");
ktd20xx_SET_REGISTER(0x04, "0x04");
ktd20xx_SET_REGISTER(0x05, "0x05");
ktd20xx_SET_REGISTER(0x06, "0x06");
ktd20xx_SET_REGISTER(0x07, "0x07");
ktd20xx_SET_REGISTER(0x08, "0x08");
ktd20xx_SET_REGISTER(0x09, "0x09");


static struct device_attribute *ktd20xx_addr_attributes[] = {
    &ktd20xx_reg0x00_attr,
    &ktd20xx_reg0x01_attr,
    &ktd20xx_reg0x02_attr,
    &ktd20xx_reg0x03_attr,
    &ktd20xx_reg0x04_attr,
    &ktd20xx_reg0x05_attr,
    &ktd20xx_reg0x06_attr,
    &ktd20xx_reg0x07_attr,
    &ktd20xx_reg0x08_attr,
    &ktd20xx_reg0x09_attr,
   
};

static void ktd20xx_enable_adv_conf(struct ktd20xx_led *led)
{
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(ktd20xx_addr_attributes); i++) {
        ret = device_create_file(&led->client->dev,
                        ktd20xx_addr_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    ktd20xx_addr_attributes[i]->attr.name);
            goto failed_remove_files;
        }
    }
    led->adf_on = 1;

    return;

failed_remove_files:
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev,
                        ktd20xx_addr_attributes[i]);
}

static void ktd20xx_disable_adv_conf(struct ktd20xx_led *led)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ktd20xx_addr_attributes); i++)
        device_remove_file(&led->client->dev,
                        ktd20xx_addr_attributes[i]);
    led->adf_on = 0;
}

static ssize_t ktd20xx_show_adv_conf(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct ktd20xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
    ssize_t ret;

    down_read(&led->rwsem);
    if (led->adf_on)
        ret = sprintf(buf, "on\n");
    else
        ret = sprintf(buf, "off\n");
    up_read(&led->rwsem);

    return ret;
}

static ssize_t ktd20xx_store_adv_conf(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct ktd20xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
    if (!count)
        return -EINVAL;

    down_write(&led->rwsem);
    if (!led->adf_on && !strncmp(buf, "on", 2))
        ktd20xx_enable_adv_conf(led);
    else if (led->adf_on && !strncmp(buf, "off", 3))
        ktd20xx_disable_adv_conf(led);
    up_write(&led->rwsem);

    return count;
}

static struct device_attribute ktd20xx_adv_conf_attr = {
    .attr = {
        .name = "advanced_configuration",
        .mode = 0644,
    },
    .show = ktd20xx_show_adv_conf,
    .store = ktd20xx_store_adv_conf,
};

#define ktd20xx_CONTROL_ATTR(attr_name, name_str)           \
static ssize_t ktd20xx_show_##attr_name(struct device *dev,     \
    struct device_attribute *attr, char *buf)           \
{                                   \
    struct ktd20xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
    ssize_t ret;                            \
    down_read(&led->rwsem);                     \
    ret = sprintf(buf, "0x%02x\n", led->attr_name);         \
    up_read(&led->rwsem);                       \
    return ret;                         \
}                                   \
static ssize_t ktd20xx_store_##attr_name(struct device *dev,        \
    struct device_attribute *attr, const char *buf, size_t count)   \
{                                   \
    struct ktd20xx_led *led = i2c_get_clientdata(to_i2c_client(dev));\
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
static struct device_attribute ktd20xx_##attr_name##_attr = {       \
    .attr = {                           \
        .name = name_str,                   \
        .mode = 0644,                       \
    },                              \
    .show = ktd20xx_show_##attr_name,               \
    .store = ktd20xx_store_##attr_name,             \
};

static struct device_attribute *ktd20xx_attributes[] = {
    &ktd20xx_adv_conf_attr,
};
#define KTD20XX_CONTROL_RGBS(name, id)				\
static void ktd20xx_set_##name##_brightness(struct led_classdev *led_cdev,\
					enum led_brightness value)	\
{									\
	struct ktd20xx_led *led =					\
		container_of(led_cdev, struct ktd20xx_led, cdev_##name);	\
	led->led_id = id;						\
	if (value == LED_OFF)						\
		led->state[id] = ktd20xx_OFF;				\
	else								\
		led->state[id] = ktd20xx_ON;					\
	schedule_work(&led->work);					\
}									\
static int ktd20xx_set_##name##_blink(struct led_classdev *led_cdev,	\
		const char *buf, size_t count)	\
{									\
    struct ktd20xx_led *led =					\
              container_of(led_cdev, struct ktd20xx_led, cdev_##name);	\
     if (count == 0 )                    		\
            return -EINVAL;						\
     led->led_id = id;						\
    if(*buf=='0')                             \
        led->state[id] = ktd20xx_ON;	          \
    else   if(*buf=='1')                        \
       led->state[id] = ktd20xx_BLINK;             \
    else if(*buf=='2')                              \
         led->state[id] = ktd20xx_BLINKSHORT; \
    else                                                  \
           led->state[id] = ktd20xx_OFF;                \
	schedule_work(&led->work);					\
	return 0;							\
}\

KTD20XX_CONTROL_RGBS(ledr, LEDR);
KTD20XX_CONTROL_RGBS(ledg, LEDG);
KTD20XX_CONTROL_RGBS(kbd, LEDKBD);

static void ktd20xx_turn_on(struct ktd20xx_led *led, enum led_ids id	)
{
       u8 regaddr=ktd20xx_REG_LED1CURRENT;//to stort every led state,enable or pwm
       u8 regvalue=ALWAYSOFF;
       u8 regtemp;
       u8 blinkflag1=0;
       u8 blinkflag2=0;
       int i=0;
       regtemp=0;
       regvalue=0;
       regaddr= ktd20xx_REG_LED1CURRENT;  

       if(id	==LEDR)
           gpio_direction_output(led->en_gpio, 1);/*[BUGFIX]-Add by TCTNB.XQJR-938243 2015/03/04add  led ic ktd2xx ,gpio 108, enable  red*/
   
       for(i=0;i<LED_NUM;i++)//FOR LED R LED G,LED B(kbd)
      	{
      	     
      	       if(led->state[i]==ktd20xx_OFF)
              {
                   regtemp=0x0;
                    i2c_smbus_write_byte_data(ktd20xx_client, regaddr+i, 0);//current about 5ma
              }
	       else  
              {
                   i2c_smbus_write_byte_data(ktd20xx_client, regaddr+i, 0x2f);//reg4 ,all led off,
                   if(i==LEDKBD)//include led3,and led4,
                  {
  /*[BUGFIX]-Add by TCTNB.XQJ,PR-958139, 2015/03/25,modify button backlight led current*/
                       i2c_smbus_write_byte_data(ktd20xx_client, 0x8, 0x9f);//use led3 and led4 current.20ma
                       i2c_smbus_write_byte_data(ktd20xx_client, 0x9, 0x9f);//use led3 and led4 current 20ma
/*[BUGFIX]-end by TCTNB.XQJ*/
                   }
	     	}

	     if(led->state[i]==ktd20xx_BLINK)
            {
                blinkflag1=ktd20xx_BLINK;
	        regtemp=0x02;
	         if(i==LEDKBD)//include led3,and led4,
                   regtemp=0x0a;//bit 1010
	     }
            else   if(led->state[i]==ktd20xx_BLINKSHORT)
            {
                  blinkflag2=ktd20xx_BLINKSHORT;
                  regtemp=0x02;
                 if(i==LEDKBD)//include led3,and led4,
                     regtemp=0x0a;//1010
	     }
	     else  if(led->state[i]==ktd20xx_ON)
	     {
                  regtemp=0x01;
                 if(i==LEDKBD)//include led3,and led4,
                     regtemp=0x05;//0101
	     }
             regtemp=regtemp<<(2*i);//see datasheet
             regvalue=regvalue |regtemp;
	     dev_err(&ktd20xx_client->dev,"led->state[%d]=%d,regtemp =0x%x,regvalue=0x%x\n",i,led->state[i],regtemp,regvalue);		
	     regtemp=0;
      	}
       dev_dbg(&ktd20xx_client->dev,"reg  4 =0x%x\n",regvalue);
       i2c_smbus_write_byte_data(ktd20xx_client, 0x04, regvalue);//SET reg4  oo off ,blink	  
 /*[BUGFIX]-Add by TCTNB.XQJ,PR-937884 2015/03/25,modify blink flash frequency and ,duty*/
      if(blinkflag2!=0)//for all leds,at onte time ,only support one led flash ,the short flash led is the first ,
      {
      	   dev_dbg(&ktd20xx_client->dev,"short blink\n");
           i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);// 1 *slower
           i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
           i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0x55);//rase time 480ms
	    i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x17);//dry flash period,about 3s
		
	    i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x28);//flash on precent   15%
      }
      else if(blinkflag1!=0)
     	{
     	       dev_dbg(&ktd20xx_client->dev,"normal blink\n");
     		i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);// 1 *slower
        	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	       i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0x55);//rase time 480ms
	       i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x27);//dry flash period,about 5s
	       i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x28);//flash on precent  15%
     	}
/*[BUGFIX]-end by TCTNB.XQJ*/
}

static ssize_t store_blink(struct device *dev, struct device_attribute *attr,
              const char *buf, size_t count)
{

      struct led_classdev *led_cdev = dev_get_drvdata(dev);

      dev_dbg(&ktd20xx_client->dev,"in %s,name=%s\n",__func__,led_cdev->name);
      if(strcmp(led_cdev->name,"led_G")==0)
            ktd20xx_set_ledg_blink(led_cdev,buf,count);
      else if(strcmp(led_cdev->name,"led_R")==0)
            ktd20xx_set_ledr_blink(led_cdev,buf,count);	
      else if(strcmp(led_cdev->name,"button-backlight")==0)
            ktd20xx_set_kbd_blink(led_cdev,buf,count);
       else
            dev_err(&ktd20xx_client->dev, "%scouldn't set blink \n", led_cdev->name);

   
      return count;
}
static DEVICE_ATTR(blink, S_IWUSR, NULL, store_blink);

static void ktd20xx_led_work(struct work_struct *work)
{
	struct ktd20xx_led *led = container_of(work, struct ktd20xx_led, work);

	ktd20xx_turn_on(led, led->led_id);

}

/* TODO: HSB, fade, timeadj, script ... */

static int ktd20xx_register_led_classdev(struct ktd20xx_led *led)
{
    int ret;
    INIT_WORK(&led->work, ktd20xx_led_work);
    led->cdev_ledg.name = "led_G";
    led->cdev_ledg.brightness = LED_OFF;
    led->cdev_ledg.brightness_set = ktd20xx_set_ledg_brightness;
  //  led->cdev_ledg.blink_set = ktd20xx_set_ledg_blink;


    ret = led_classdev_register(&led->client->dev, &led->cdev_ledg);
    if (ret < 0) {
        dev_err(&led->client->dev, "couldn't register LED %s\n",
                            led->cdev_ledg.name);
        goto failed_unregister_led_G;
    }
    ret= sysfs_create_file (&led->cdev_ledg.dev->kobj, &dev_attr_blink.attr);
	
    led->cdev_ledr.name = "led_R";
    led->cdev_ledr.brightness = LED_OFF;
    led->cdev_ledr.brightness_set = ktd20xx_set_ledr_brightness;
//   led->cdev_ledr.blink_set =  ktd20xx_set_ledr_blink;
   
    ret = led_classdev_register(&led->client->dev, &led->cdev_ledr);
    if (ret < 0) {
        dev_err(&led->client->dev, "couldn't register LED %s\n",
                            led->cdev_ledr.name);
        goto failed_unregister_led_R;
    }
  ret= sysfs_create_file (&led->cdev_ledr.dev->kobj, &dev_attr_blink.attr);
    led->cdev_kbd.name = "button-backlight";
    led->cdev_kbd.brightness = LED_OFF;
    led->cdev_kbd.brightness_set = ktd20xx_set_kbd_brightness;
// led->cdev_kbd.blink_set =  ktd20xx_set_kbd_blink;
 
    ret = led_classdev_register(&led->client->dev, &led->cdev_kbd);
    if (ret < 0) {
        dev_err(&led->client->dev, "couldn't register LED %s\n",
                            led->cdev_kbd.name);
        goto failed_unregister_KBD;
    }
  ret= sysfs_create_file (&led->cdev_kbd.dev->kobj, &dev_attr_blink.attr);	
    return 0;
failed_unregister_KBD:
    led_classdev_unregister(&led->cdev_kbd);
failed_unregister_led_R:
    led_classdev_unregister(&led->cdev_ledr);
failed_unregister_led_G:
    led_classdev_unregister(&led->cdev_ledg);

    return ret;
}

static int ktd20xx_init(struct i2c_client *client)
{
      int ret=0;
    	//ktd2xx_led_off(); //turn off led when first start ktd
      i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
      i2c_smbus_write_byte_data(ktd20xx_client, 0x05, 0x00);//rase time
      i2c_smbus_write_byte_data(ktd20xx_client, 0x01, 0x00);//dry flash period,about 5s
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0.125mA
	
	if(ret < 0){
		dev_err(&ktd20xx_client->dev,"can't find ktd2026 led control ic!");
	}
     else
       	dev_dbg(&ktd20xx_client->dev,"i2c  ok ktd20xx_client-->addr=0x%x,ret=0x%x\n",ktd20xx_client->addr,ret);
//   ret = i2c_smbus_read_byte_data(client, j);
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);//turn off leds	
	if(ret < 0){
		dev_err(&ktd20xx_client->dev,"can't find ktd2026 led control ic!");
	}
	else
       	dev_dbg(&ktd20xx_client->dev,"i2c  ok ,ret=0x%x\n",ret);
  

       return ret;
}
/*[BUGFIX]-Add by TCTNB.ZXZ,PR-938243 2015/03/04add  led ic ktd2xx ,gpio 108, enable  red*/
static int ktd20xx_parse_dt(struct device *dev, struct ktd20xx_led *led)
{
	int r = 0;

      led->en_gpio = of_get_named_gpio_flags(dev->of_node,
            "ktd20,en-gpio", 0, NULL);

      if ((!gpio_is_valid(led->en_gpio)))
          return -EINVAL;
	return r;
}
/*[BUGFIX]-End by TCTNB.XQJ*/

//extern void qnnp_lbc_enable_led(u8 onoff);
static int ktd20xx_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int ret, i;
    struct ktd20xx_led *led = devm_kzalloc(&client->dev,
										sizeof(struct ktd20xx_led), GFP_KERNEL);
    if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
        return -ENOMEM;
    }
    ktd20xx_parse_dt(&client->dev, led);

	ktd20xx_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!ktd20xx_client) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		
	}

	ktd20xx_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		
		
	}
      led->client = ktd20xx_client;
  
	i2c_set_clientdata(ktd20xx_client, led);

	
	
    dev_dbg(&ktd20xx_client->dev,"xqj led ktd probe \n");
    ret = gpio_request(led->en_gpio, "ktd_red");/*[BUGFIX]-Add by TCTNB.ZXZ,PR-938243 2015/03/04add  led ic ktd2xx ,gpio 108, enable  red*/
    if (ret)
       return  -ENODEV;

	ret = ktd20xx_power_init(led, 1);
    if (ret < 0)
        goto exit1;
    ret = ktd20xx_power_set(led, 1);
    if (ret < 0)
        goto exit2;
    udelay(600);
    ret=   ktd20xx_init(client);
    if (ret < 0)
        goto exit3;

    for(i=0;i<LED_NUM;i++)
	led->state[i]=0;
		
   ktd20xx_power_set(led, 1);
    	    
    init_rwsem(&led->rwsem);

    for (i = 0; i < ARRAY_SIZE(ktd20xx_attributes); i++) {   
        ret = device_create_file(&led->client->dev,
                        ktd20xx_attributes[i]);
        if (ret) {
            dev_err(&led->client->dev, "failed: sysfs file %s\n",
                    ktd20xx_attributes[i]->attr.name);
            goto exit4;
        }
    }

    ret = ktd20xx_register_led_classdev(led);
	
    if (ret < 0)
        goto  exit4;
	
//	
#if (!defined CONFIG_TCT_8X16_IDOL3 )&&(!defined CONFIG_TCT_8X16_IDOL347)/*[BUGFIX]-Mod by TCTNB.XQJ,PR-817507 2014/10/23,not need in idol3*/
	//qnnp_lbc_enable_led(0);
#endif
    return 0;

exit4:
	i=0;
    for (i--; i >= 0; i--)
        device_remove_file(&led->client->dev, ktd20xx_attributes[i]);
exit3:
	dev_dbg(&ktd20xx_client->dev,"init nok\n");
    ktd20xx_power_set(led, 0);
exit2:
    ktd20xx_power_init(led, 0);
exit1:
//    kfree(led);
#if (!defined CONFIG_TCT_8X16_IDOL3 )&&(!defined CONFIG_TCT_8X16_IDOL347) /*[BUGFIX]-Mod by TCTNB.XQJ,PR-817507 2014/10/23,not need in idol3*/
//    qnnp_lbc_enable_led(1);
#endif
    return ret;
}

static int ktd20xx_remove(struct i2c_client *client)
{
    struct ktd20xx_led *led = i2c_get_clientdata(client);
    int i;
     cancel_work_sync(&led->work);
	led_classdev_unregister(&led->cdev_ledg);
	led_classdev_unregister(&led->cdev_ledr);
	led_classdev_unregister(&led->cdev_kbd);
    if (led->adf_on)
        ktd20xx_disable_adv_conf(led);
    for (i = 0; i < ARRAY_SIZE(ktd20xx_attributes); i++)
        device_remove_file(&led->client->dev, ktd20xx_attributes[i]);
    return 0;
}
static void ktd20xx_shutdown(struct i2c_client *client)/*[BUGFIX]-Add by TCTNB.XQJ,PR-1019141 2015/06/18,add shutdown interface for close all leds*/
{
       if(ktd20xx_client !=NULL)
	    i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0);//SET reg4  oo off
}
static const struct i2c_device_id ktd20xx_id[] = {
    { "ktd20xx", 0 },
    { }
};

static struct of_device_id ktd20xx_match_table[] = {
    { .compatible = "ktd20,ktd20xx", },
    { },
};

static struct i2c_driver ktd20xx_i2c_driver = {
    .probe		= ktd20xx_probe,
    .remove		= ktd20xx_remove,
    .id_table	= ktd20xx_id,
    .driver = {
				.name    = "ktd20xx",
				.owner  = THIS_MODULE,
				.of_match_table = ktd20xx_match_table,
	},
    .shutdown   = ktd20xx_shutdown,  /*[BUGFIX]-Add by TCTNB.XQJ,PR-1019141 2015/06/18,add shutdown interface for close all leds*/
};
static int __init ktd20xx_i2c_init(void)
{
	return i2c_add_driver(&ktd20xx_i2c_driver);
}
late_initcall(ktd20xx_i2c_init);
MODULE_AUTHOR("xqj<Qijun.xu@tcl.com>");
MODULE_DESCRIPTION("ktd2xx LED driver");
MODULE_LICENSE("GPL v2");
/*fangang.luo 2015.12.17 task:1176409 END*/
