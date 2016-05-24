/**
* modify	2015.2.11 change led4 to button-backbright
* change 2015.3.3 datasheet update led, so ...change the driver
* leds map: D1(Green), D2(Red), D3(Blue), D4(touchkey).
*
* modify by zhou at 2015.4.22. add mix clolor for led and  add mix color for blink
* the blink time can control also.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h> 
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/workqueue.h>



#define KTD_I2C_NAME			"ktd20xx"
#define MAX_BRIGHTNESS		(LED_FULL)
//#define MIN_BRIGHTNESS		(3)

//RGB current limit to 3mA
#define RGB_BRIGHTNESS_LIMIT (24)
//touch key current limit to 15mA
#define TOUCHKEY_BRIGHTNESS_LIMIT (120)

//0.128s per value
#define MIN_TIME_PER_VALUE      (128)
#define MAX_BLINK_VALUE          (127)
// need mutiply 100
#define DUTE_PER_VALUE 			(39)
#define MAX_DUTE 				(255)
#define DEFAULT_BLINK_BRIGHTNESS (77)
#define DEFAULT_RAISE_AND_FALL_TIME (0x33)

//registers address
#define CTRL_RESET_REG 	(0)
#define FLASH_PERIOD_REG (1)
#define PWM1_REG	(2)
#define PWM2_REG	(3)
#define EN_REG	(4)
#define RAISE_FALL_REG (5)
#define D1_DATA_REG		(6)
#define D2_DATA_REG		(7)
#define D3_DATA_REG		(8)
#define D4_DATA_REG		(9)

//EN_REG 
#define ALWAYS_OFF (0)
#define ALWAYS_ON (1)
#define ENABLE_PWM1 (2)
#define ENABLE_PWM2 (3)
#define ENABLE_MASK (3)

//which led you should control
enum ktd20xx_led{
	KTD20XX_LED_D1 = 0,
	KTD20XX_LED_D2,
	KTD20XX_LED_D3,
	KTD20XX_LED_D4,
	KTD20XX_UNKNOW
};

enum ktd20xx_time_scale{
	KTD20XX_SCALE_NOMAL=0,
	KTD20XX_SCALE_2SLOWER,
	KTD20XX_SCALE_4SLOWER,
	KTD20XX_SCALE_8FASTER
};

struct time_limit{
	unsigned long low;
	unsigned long high;
};

struct ramp_time{
	u8 rise;
	u8 fall;
};

struct ramp_time_map{
	struct time_limit time;
	struct ramp_time ramp_time;
	enum ktd20xx_time_scale scale;
};
//500 is step
struct ramp_time_map ramp_data[]= {
	{//period[0, 500], rise 96ms, fall 96ms
		.time = {
			.low = 0,
			.high = 500,
		},
		.ramp_time = {
			.rise = 0x1,
			.fall = 0x1,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[500, 1000], rise 192ms, fall 192ms
		.time = {
			.low = 500,
			.high = 1000,
		},
		.ramp_time = {
			.rise = 0x2,
			.fall = 0x2,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[1000, 1500], rise 288ms, fall 288ms
		.time = {
			.low = 1000,
			.high = 1500,
		},
		.ramp_time = {
			.rise = 0x3,
			.fall = 0x3,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[1500, 2000], rise 348ms, fall 384ms
		.time = {
			.low = 1500,
			.high = 2000,
		},
		.ramp_time = {
			.rise = 0x4,
			.fall = 0x4,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[2000, 2500], rise 480ms, fall 480ms
		.time = {
			.low = 2000,
			.high = 2500,
		},
		.ramp_time = {
			.rise = 0x5,
			.fall = 0x5,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[2500, 3000], rise 576ms, fall 576ms
		.time = {
			.low = 2500,
			.high = 3000,
		},
		.ramp_time = {
			.rise = 0x6,
			.fall = 0x6,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[3000, 3500],  rise 672ms, fall 672ms
		.time = {
			.low = 3000,
			.high = 3500,
		},
		.ramp_time = {
			.rise = 0x7,
			.fall = 0x7,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[3500, 4000], rise 768ms, fall 768ms
		.time = {
			.low = 3500,
			.high = 4000,
		},
		.ramp_time = {
			.rise = 0x8,
			.fall = 0x8,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[4000, 4500], rise 864ms, fall 864ms
		.time = {
			.low = 4000,
			.high = 4500,
		},
		.ramp_time = {
			.rise = 0x9,
			.fall = 0x9,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[4500, 5000], rise 960ms, fall 960ms
		.time = {
			.low = 4500,
			.high = 5000,
		},
		.ramp_time = {
			.rise = 0xa,
			.fall = 0xa,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[5000, 5500], rise 1056ms, fall 1056ms
		.time = {
			.low = 5000,
			.high = 5500,
		},
		.ramp_time = {
			.rise = 0xb,
			.fall = 0xb,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[5500, 6000], rise 1152ms, fall 1152ms
		.time = {
			.low = 5500,
			.high = 6000,
		},
		.ramp_time = {
			.rise = 0xc,
			.fall = 0xc,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[6000, 6500], rise 1248ms, fall 1248ms
		.time = {
			.low = 6000,
			.high = 6500,
		},
		.ramp_time = {
			.rise = 0xd,
			.fall = 0xd,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[6500, 7000], rise 1344ms, fall 1344ms
		.time = {
			.low = 6500,
			.high = 7000,
		},
		.ramp_time = {
			.rise = 0xe,
			.fall = 0xe,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[7000, 7500], rise 1440ms, fall 1440ms
		.time = {
			.low = 7000,
			.high = 7500,
		},
		.ramp_time = {
			.rise = 0xf,
			.fall = 0xf,
		},
		.scale = KTD20XX_SCALE_NOMAL,
	},
	{//period[7500, 8000], rise 1536ms, fall 1536ms
		.time = {
			.low = 7500,
			.high = 8000,
		},
		.ramp_time = {
			.rise = 0x8,
			.fall = 0x8,
		},
		.scale = KTD20XX_SCALE_2SLOWER,
	},
	{//period[8000, 8500], rise 1728ms, fall 1728ms
		.time = {
			.low = 8000,
			.high = 8500,
		},
		.ramp_time = {
			.rise = 0x9,
			.fall = 0x9,
		},
		.scale = KTD20XX_SCALE_2SLOWER,
	},
	{//period[8500, 9000], rise 1920ms, fall 1920ms
		.time = {
			.low = 8500,
			.high = 9000,
		},
		.ramp_time = {
			.rise = 0xa,
			.fall = 0xa,
		},
		.scale = KTD20XX_SCALE_2SLOWER,
	},
	{//period[9000, 9500], rise 2112ms, fall 2112ms
		.time = {
			.low = 9000,
			.high = 9500,
		},
		.ramp_time = {
			.rise = 0xb,
			.fall = 0xb,
		},
		.scale = KTD20XX_SCALE_2SLOWER,
	},
	{//period[9500, 10000], rise 2304ms, fall 2304ms
		.time = {
			.low = 9500,
			.high = 10000,
		},
		.ramp_time = {
			.rise = 0xc,
			.fall = 0xc,
		},
		.scale = KTD20XX_SCALE_2SLOWER,
	},		

};

//led class device data
struct ktd_device_data{
	struct led_classdev led_dev;
	unsigned long brightness;
	unsigned long blink_brightness;
	struct work_struct work_brightness;
	struct work_struct work_blink;
};

// main device data
struct ktd_dev_data{
	int red_gpio_pin;
	int is_first_time;
	unsigned long delay_on;//ms
	unsigned long delay_off;//ms
	
	struct i2c_client *i2c_client;
	struct mutex  ktd20xx_lock;
	struct workqueue_struct  *work_queue;
	struct ktd_device_data d1;
	struct ktd_device_data d2;
	struct ktd_device_data d3;	
	struct ktd_device_data d4;
	uint8_t ktd_reg_enable;
};

static int ktd20xx_red_gpio_set(struct ktd_dev_data *pdata);


static int ktd20xx_hw_init(struct i2c_client *client){
	int ret;
	i2c_smbus_write_byte_data(client, CTRL_RESET_REG, 0x07);
	udelay(1000);
	ret = i2c_smbus_write_byte_data(client, EN_REG, ALWAYS_OFF);//turn off leds	
	if(ret < 0){
		dev_err(&client->dev, "can't find ktd2027 led control ic!\n");
		return -1;
	} 
	
	return 0;
}


static void ktd20xx_hw_shutdown(struct i2c_client *client){
	struct ktd_dev_data *pdata;
	pdata = i2c_get_clientdata(client);
	
	mutex_lock(&pdata->ktd20xx_lock);
	flush_workqueue(pdata->work_queue);
	i2c_smbus_write_byte_data(client, EN_REG, ALWAYS_OFF);	
	i2c_smbus_write_byte_data(client, CTRL_RESET_REG, 0x07);
	gpio_set_value(pdata->red_gpio_pin, 0);
	mutex_unlock(&pdata->ktd20xx_lock);
}


/*ktd20xx_led_on_off
* control led on or off
*which which led should be on or off
*/
static void ktd20xx_led_on_off(struct ktd_dev_data *pktd, enum ktd20xx_led which){
	struct i2c_client *client = pktd->i2c_client;
	unsigned long brightness;
	unsigned int reg_num;
	unsigned int enable_value;
	int i, tmp;

	switch(which){
	case KTD20XX_LED_D1:
		brightness = pktd->d1.brightness;
		reg_num = D1_DATA_REG;
		break;
	case KTD20XX_LED_D2:
		brightness = pktd->d2.brightness;
		reg_num = D2_DATA_REG;
		break;
	case KTD20XX_LED_D3:
		brightness = pktd->d3.brightness;
		reg_num = D3_DATA_REG;
		break;
	case KTD20XX_LED_D4:
		brightness = pktd->d4.brightness;
		reg_num = D4_DATA_REG;
		break;
	default:
		dev_err(&client->dev, "err param");
		return;
	}


	//limit led current. touchkey max current is 15mA, other is 5mA
	if(which == KTD20XX_LED_D4){
		brightness = brightness * TOUCHKEY_BRIGHTNESS_LIMIT / MAX_BRIGHTNESS;
	}else{
		brightness = brightness * RGB_BRIGHTNESS_LIMIT / MAX_BRIGHTNESS;
	}
	
	mutex_lock(&pktd->ktd20xx_lock);
//	enable_value = atomic_read(&pktd->ktd_reg_enable);

	//if this time some leds are blinking, set off all leds except button-backlight
	if (which != KTD20XX_LED_D4) {
		for (i=0; i<4; i++) {
			tmp = pktd->ktd_reg_enable >> i*2;
			if (((tmp & 0x3) == 0x2) || ((tmp & 0x3) == 0x3)) {
				pktd->ktd_reg_enable &= 0xc0;
				i2c_smbus_write_byte_data(client, EN_REG, pktd->ktd_reg_enable);
				break;
			}
		}
	}

	enable_value = pktd->ktd_reg_enable;
	dev_dbg(&pktd->i2c_client->dev, "%s:Before set, enable reg value:%x\n",
			__FUNCTION__, enable_value);
	if(brightness){
		i2c_smbus_write_byte_data(client, reg_num, brightness);
		//mode set---IC work when both SCL and SDA goes high
		i2c_smbus_write_byte_data(client, CTRL_RESET_REG, 00);
		enable_value |=ALWAYS_ON << which*2;
		i2c_smbus_write_byte_data(client, EN_REG, enable_value);
	} else {
		enable_value &= ~(ENABLE_MASK << which*2);
		i2c_smbus_write_byte_data(client, EN_REG, enable_value);
	}

	//all leds are closed then disable the chip
	if(!enable_value){
		i2c_smbus_write_byte_data(client, CTRL_RESET_REG, 0x08);
	}
	
	//atomic_set(&pktd->ktd_reg_enable, enable_value);
	pktd->ktd_reg_enable = enable_value;
	dev_dbg(&pktd->i2c_client->dev, "%s:After set, enable reg value:%x\n",
			__FUNCTION__, enable_value);
	mutex_unlock(&pktd->ktd20xx_lock);
}


/*ktd20xx_breath_leds
* blink control function
* which which led should bink
*/
static void ktd20xx_breath_leds(struct ktd_dev_data *pktd, enum ktd20xx_led which){
	struct i2c_client *client = pktd->i2c_client;
	unsigned long delay_on;
	unsigned long delay_off;
	unsigned long blink_brightness;
	unsigned int period;
	unsigned int dute_value;
	unsigned int enable_value;
	unsigned int reg_num;
	unsigned int select_ramp;
	unsigned int ramp_data_len = sizeof(ramp_data) / sizeof(struct ramp_time_map);
	u8 rise_fall_time;
	enum ktd20xx_time_scale scale;

	delay_on =  pktd->delay_on;
	delay_off = pktd->delay_off;

	period = delay_on + delay_off;
	//500 is step of ramp_data
	select_ramp = period / 500;
	if (select_ramp < ramp_data_len) {
		rise_fall_time = ramp_data[select_ramp].ramp_time.rise + (ramp_data[select_ramp].ramp_time.fall << 4);
		scale = ramp_data[select_ramp].scale;
	} else {//not found, use default value;
		rise_fall_time = DEFAULT_RAISE_AND_FALL_TIME;
		scale = KTD20XX_SCALE_NOMAL;
	}

	dute_value = delay_on * 100 / period;//get the on time of period, should change to percent

	period = period / MIN_TIME_PER_VALUE;
	if(period > MAX_BLINK_VALUE)
		period = MAX_BLINK_VALUE;

	dute_value = dute_value * 100;
	dute_value = dute_value / DUTE_PER_VALUE;
	if(dute_value > MAX_DUTE)
		dute_value = MAX_DUTE;
	
	
	switch(which){
	case KTD20XX_LED_D1:
		reg_num = D1_DATA_REG;
		blink_brightness = pktd->d1.blink_brightness;
		break;
	case KTD20XX_LED_D2:
		reg_num = D2_DATA_REG;
		blink_brightness = pktd->d2.blink_brightness;
		break;
	case KTD20XX_LED_D3:
		reg_num = D3_DATA_REG;
		blink_brightness = pktd->d3.blink_brightness;
		break;
	case KTD20XX_LED_D4:
		reg_num = D4_DATA_REG;
		blink_brightness = pktd->d4.blink_brightness;
		break;
	default:
		dev_err(&client->dev, "err param");
		return;
	}
	//limit led current. touchkey max current is 15mA, other is 5mA
	if(which == KTD20XX_LED_D4){
		blink_brightness = blink_brightness * TOUCHKEY_BRIGHTNESS_LIMIT / MAX_BRIGHTNESS;
	}else{
		blink_brightness = blink_brightness * RGB_BRIGHTNESS_LIMIT / MAX_BRIGHTNESS;
	}
	
	mutex_lock(&pktd->ktd20xx_lock);
//	enable_value = atomic_read(&pktd->ktd_reg_enable);
	enable_value = pktd->ktd_reg_enable;
	dev_dbg(&pktd->i2c_client->dev, "%s:Before set, enable reg value:%x\n",
			__FUNCTION__, enable_value);

#ifdef BLINK_BRIGHTNESS
	i2c_smbus_write_byte_data(client, reg_num, blink_brightness);
#else
	i2c_smbus_write_byte_data(client, reg_num, DEFAULT_BLINK_BRIGHTNESS);
#endif
	//rase and fall time
	i2c_smbus_write_byte_data(client, RAISE_FALL_REG, rise_fall_time);
	//dry flash period
	i2c_smbus_write_byte_data(client, FLASH_PERIOD_REG, period);

	//reset internal counter
	i2c_smbus_write_byte_data(client, PWM1_REG, dute_value);

	// mode set---IC work when both SCL and SDA goes high 2X slower
	i2c_smbus_write_byte_data(client, CTRL_RESET_REG, scale << 5);
	//allocate led to timer1
	enable_value &= ~(ENABLE_MASK << which*2);
	enable_value |= (ENABLE_PWM1 << which*2);
	i2c_smbus_write_byte_data(client, EN_REG, enable_value);
//	atomic_set(&pktd->ktd_reg_enable, enable_value);
	pktd->ktd_reg_enable = enable_value;
	dev_dbg(&pktd->i2c_client->dev, "%s:After set, enable reg value:%x\n",
			__FUNCTION__,enable_value);
	mutex_unlock(&pktd->ktd20xx_lock);
}

/*ktd20xx_parse_dt
* get the gpio of control red led
*/
static int ktd20xx_parse_dt(struct ktd_dev_data *pdata){	
	pdata->red_gpio_pin = of_get_named_gpio_flags(pdata->i2c_client->dev.of_node, "ktd,gpio", 0, NULL);
	if (!gpio_is_valid(pdata->red_gpio_pin)) {
		dev_err(&pdata->i2c_client->dev, "gpio reset pin %d is invalid.\n",	pdata->red_gpio_pin);
		return -EINVAL;
	}
	return 0;
}

static void ktd20xx_work_brightness_d1(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d1.work_brightness);
	ktd20xx_led_on_off(pdata, KTD20XX_LED_D1);
}

static void ktd20xx_work_brightness_d2(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d2.work_brightness);
	ktd20xx_red_gpio_set(pdata);
	ktd20xx_led_on_off(pdata, KTD20XX_LED_D2);
}


static void ktd20xx_work_brightness_d3(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d3.work_brightness);
	ktd20xx_led_on_off(pdata, KTD20XX_LED_D3);
}


static void ktd20xx_work_brightness_d4(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d4.work_brightness);
	ktd20xx_led_on_off(pdata, KTD20XX_LED_D4);
}


static void ktd20xx_work_blink_d1(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d1.work_blink);
	ktd20xx_breath_leds(pdata, KTD20XX_LED_D1);
}

static void ktd20xx_work_blink_d2(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d2.work_blink);
	ktd20xx_red_gpio_set(pdata);
	ktd20xx_breath_leds(pdata, KTD20XX_LED_D2);
}

static void ktd20xx_work_blink_d3(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d3.work_blink);
	ktd20xx_breath_leds(pdata, KTD20XX_LED_D3);
}

static void ktd20xx_work_blink_d4(struct work_struct *work){
	struct ktd_dev_data *pdata;
	pdata = container_of(work, struct ktd_dev_data, d4.work_blink);
	ktd20xx_breath_leds(pdata, KTD20XX_LED_D4);
}

// led class D1
static void ktd20xx_d1_brightness_set(struct led_classdev *led_cdev,
					  							enum led_brightness brightness){

	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d1.led_dev);
	pdata->d1.brightness = brightness;
	queue_work(pdata->work_queue, &pdata->d1.work_brightness);
}
static int ktd20xx_d1_blink_set	(struct led_classdev *led_cdev,
										 unsigned long *delay_on,
										 unsigned long *delay_off){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d1.led_dev);
	
	pdata->delay_on= *delay_on;
	pdata->delay_off = *delay_off;
	pdata->d1.blink_brightness = led_cdev->blink_brightness;
	queue_work(pdata->work_queue, &pdata->d1.work_blink);
	return 0;
}

// led class D2
static void ktd20xx_d2_brightness_set(struct led_classdev *led_cdev,
					  							enum led_brightness brightness){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d2.led_dev);

	pdata->d2.brightness = brightness;
	queue_work(pdata->work_queue, &pdata->d2.work_brightness);
}
static int ktd20xx_d2_blink_set	(struct led_classdev *led_cdev,
										 unsigned long *delay_on,
										 unsigned long *delay_off){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d2.led_dev);

	pdata->delay_on = *delay_on;
	pdata->delay_off = *delay_off;
	pdata->d2.blink_brightness = led_cdev->blink_brightness;
	queue_work(pdata->work_queue, &pdata->d2.work_blink);
	return 0;
}

// led class D3

static void ktd20xx_d3_brightness_set(struct led_classdev *led_cdev,
					  							enum led_brightness brightness){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d3.led_dev);

	pdata->d3.brightness = brightness;
	queue_work(pdata->work_queue, &pdata->d3.work_brightness);
	
}
static int ktd20xx_d3_blink_set	(struct led_classdev *led_cdev,
										 unsigned long *delay_on,
										 unsigned long *delay_off){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev, struct ktd_dev_data, d3.led_dev);
	
	pdata->delay_on = *delay_on;
	pdata->delay_off = *delay_off;
	pdata->d3.blink_brightness = led_cdev->blink_brightness;
	queue_work(pdata->work_queue, &pdata->d3.work_blink);
	return 0;
}

// led class D4
static void ktd20xx_d4_brightness_set(struct led_classdev *led_cdev,
					  							enum led_brightness brightness){
	struct ktd_dev_data *pdata;
	pdata = container_of(led_cdev,struct ktd_dev_data, d4.led_dev);
	
	pdata->d4.brightness = brightness;
//	if((brightness > LED_OFF) && (brightness < MIN_BRIGHTNESS))
//		pdata->d4.brightness = MIN_BRIGHTNESS;
	queue_work(pdata->work_queue, &pdata->d4.work_brightness);
	
}

static int ktd20xx_d4_blink_set	(struct led_classdev *led_cdev,
										 unsigned long *delay_on,
										 unsigned long *delay_off){
	
	struct ktd_dev_data *pdata;	
	pdata = container_of(led_cdev, struct ktd_dev_data, d4.led_dev);

	pdata->delay_on = *delay_on;
	pdata->delay_off = *delay_off;
	pdata->d4.blink_brightness = led_cdev->blink_brightness;
	queue_work(pdata->work_queue, &pdata->d4.work_blink);
	return 0;
}


static int	ktd20xx_d1_classdev_register(struct i2c_client *client){
	int err;
	struct ktd_dev_data *pdata = i2c_get_clientdata(client);
    pdata->d1.led_dev.name= "green";
	pdata->d1.led_dev.max_brightness = MAX_BRIGHTNESS;
	pdata->d1.led_dev.brightness_set = ktd20xx_d1_brightness_set;
	pdata->d1.led_dev.blink_set  = ktd20xx_d1_blink_set;
	
	err = led_classdev_register((struct device *)&client->dev,
								&pdata->d1.led_dev);
    if(err){
        dev_err(&client->dev, "ktd20xx register D1 class device failed\n");
		return -1;
	}
	return 0;
}

static int	ktd20xx_d2_classdev_register(struct i2c_client *client){
	int err;
	struct ktd_dev_data *pdata = i2c_get_clientdata(client);
    pdata->d2.led_dev.name= "red";
	pdata->d2.led_dev.max_brightness = MAX_BRIGHTNESS;
	pdata->d2.led_dev.brightness_set = ktd20xx_d2_brightness_set;
	pdata->d2.led_dev.blink_set  = ktd20xx_d2_blink_set;
	
	err = led_classdev_register((struct device *)&client->dev,
								&pdata->d2.led_dev);
    if(err){
        dev_err(&client->dev, "ktd20xx register D2 class device failed\n");
		return -1;
	}
	return 0;
}

static int	ktd20xx_d3_classdev_register(struct i2c_client *client){
	int err;
	struct ktd_dev_data *pdata = i2c_get_clientdata(client);
    pdata->d3.led_dev.name= "blue";
	pdata->d3.led_dev.max_brightness = MAX_BRIGHTNESS;
	pdata->d3.led_dev.brightness_set = ktd20xx_d3_brightness_set;
	pdata->d3.led_dev.blink_set  = ktd20xx_d3_blink_set;
	
	err = led_classdev_register((struct device *)&client->dev,
								&pdata->d3.led_dev);
    if(err){
        dev_err(&client->dev, "ktd20xx register D3 class device failed\n");
		return -1;
	}
	return 0;
}

static int	ktd20xx_d4_classdev_register(struct i2c_client *client){
	int err;
	struct ktd_dev_data *pdata = i2c_get_clientdata(client);
    pdata->d4.led_dev.name= "button-backlight";
	pdata->d4.led_dev.max_brightness = MAX_BRIGHTNESS;
	pdata->d4.led_dev.brightness_set = ktd20xx_d4_brightness_set;
	pdata->d4.led_dev.blink_set  = ktd20xx_d4_blink_set;
	
	err = led_classdev_register((struct device *)&client->dev,
								&pdata->d4.led_dev);
    if(err){
        dev_err(&client->dev, "ktd20xx register D4 class device failed\n");
		return -1;
	}
	return 0;
}

/* ktd20xx_red_gpio_init
*  pull up the gpio to disable usb charge control leds
*/
static int ktd20xx_red_gpio_init(struct ktd_dev_data *pdata){
	int ret;

	ret = gpio_request(pdata->red_gpio_pin, KTD_I2C_NAME);
	if (ret < 0) {
		dev_err(&pdata->i2c_client->dev,
			"%s: GPIO %d Request Fail (%d)\n",
				__func__, pdata->red_gpio_pin, ret);
		return ret;
	}

	return 0;
}

static int ktd20xx_red_gpio_set(struct ktd_dev_data *pdata){
	int ret;

	if (likely(pdata->is_first_time == 0)) {
		return 0;
	} 
	
	pdata->is_first_time = 0;
	ret = gpio_direction_output(pdata->red_gpio_pin, 1);
	if (ret < 0) {
		dev_err(&pdata->i2c_client->dev,
			"%s: Set GPIO %d as Input Fail (%d)\n", __func__,
					pdata->red_gpio_pin, ret);
		return ret;
	}
//	gpio_set_value(pdata->red_gpio_pin, !!value);
	return 0;
}

static int ktd20xx_probe(struct i2c_client *client, const struct i2c_device_id *id){
	int err = 0;
    struct ktd_dev_data *pdata;
	dev_info(&client->dev, "[%s]: Enter!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.\n", __func__);
		err = -ENODEV;
		return err;
	}
	
	pdata = kzalloc(sizeof(struct ktd_dev_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev,
				"%s: memory allocation failed.\n", __func__);
		err = -ENOMEM;
		goto exit0;
	}

	pdata->i2c_client = client;
	i2c_set_clientdata(client, pdata);
	
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		 err = ktd20xx_parse_dt(pdata);
		 if(err){
			goto exit1;
		 }
	}
#endif

	pdata->is_first_time = 1;
	err = ktd20xx_red_gpio_init(pdata);
	if(err){
		goto exit1;
	}

//	atomic_set(&pdata->ktd_reg_enable, 0);
	mutex_init(&pdata->ktd20xx_lock);

	/*init hw*/
	err = ktd20xx_hw_init(client);
	
	if(err){
		dev_err(&client->dev, "ktd2027 init failed!\n");
		goto exit2;
	}

	//for green led
	err = ktd20xx_d1_classdev_register(client);
	if(err){
		goto exit2;
	}

	//for red led
	err = ktd20xx_d2_classdev_register(client);
	if(err){
		goto exit3;
	}

	//for blue led
	err = ktd20xx_d3_classdev_register(client);
	if(err){
		goto exit4;
	}

	//for touchkey led
	err = ktd20xx_d4_classdev_register(client);
	if(err){
		goto exit5;
	}
	
//	pdata->work_queue = alloc_workqueue("ktd20xx_work_queue",
//				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	pdata->work_queue = create_singlethread_workqueue("ktd20xx_work_queue");

	INIT_WORK(&pdata->d1.work_brightness, ktd20xx_work_brightness_d1);
	INIT_WORK(&pdata->d2.work_brightness, ktd20xx_work_brightness_d2);
	INIT_WORK(&pdata->d3.work_brightness, ktd20xx_work_brightness_d3);
	INIT_WORK(&pdata->d4.work_brightness, ktd20xx_work_brightness_d4);
	INIT_WORK(&pdata->d1.work_blink, ktd20xx_work_blink_d1);
	INIT_WORK(&pdata->d2.work_blink, ktd20xx_work_blink_d2);
	INIT_WORK(&pdata->d3.work_blink, ktd20xx_work_blink_d3);
	INIT_WORK(&pdata->d4.work_blink, ktd20xx_work_blink_d4);
	dev_info(&client->dev, "ktd20xx device driver init success\n");

	return 0;

exit5:
	led_classdev_unregister(&pdata->d3.led_dev);
exit4:
	led_classdev_unregister(&pdata->d2.led_dev);
exit3:
	led_classdev_unregister(&pdata->d1.led_dev);
exit2:
	dev_err(&client->dev, "ktd20xx red/green/blue/touchkey led register classdev failed\n");
	gpio_free(pdata->red_gpio_pin);
exit1:
	kfree(pdata);
exit0:
	return err;
}

static int ktd20xx_remove(struct i2c_client *client){
	struct ktd_dev_data * pdata = i2c_get_clientdata(client);

	printk("[%s]: Enter!\n", __func__);

	cancel_work_sync(&pdata->d1.work_brightness);
	cancel_work_sync(&pdata->d2.work_brightness);
	cancel_work_sync(&pdata->d3.work_brightness);
	cancel_work_sync(&pdata->d4.work_brightness);
	cancel_work_sync(&pdata->d1.work_blink);
	cancel_work_sync(&pdata->d2.work_blink);
	cancel_work_sync(&pdata->d3.work_blink);
	cancel_work_sync(&pdata->d4.work_blink);
	
	destroy_workqueue(pdata->work_queue);
	led_classdev_unregister(&pdata->d1.led_dev);
	led_classdev_unregister(&pdata->d2.led_dev);
	led_classdev_unregister(&pdata->d3.led_dev);
	led_classdev_unregister(&pdata->d4.led_dev);

	gpio_free(pdata->red_gpio_pin);
	kfree(pdata);
		
	return 0;
}

static void ktd20xx_shutdown(struct i2c_client *client){
	ktd20xx_hw_shutdown(client);
}


static int ktd20xx_suspend(struct device *dev){

	return 0;
}
static int ktd20xx_resume(struct device *dev){

	return 0;
}

static const struct dev_pm_ops ktd20xx_pm_ops = {
	.suspend	= ktd20xx_suspend,
	.resume     = ktd20xx_resume,
};

static const struct i2c_device_id ktd20xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ },
};

#ifdef CONFIG_OF
static struct of_device_id ktd20xx_match_table[] = {
        { .compatible = "ktd,ktd20xx"},
        { },
};
#else
#define ktd20xx_match_table NULL
#endif


static struct i2c_driver ktd20xx_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd20xx_match_table,
#ifdef CONFIG_PM
		.pm = &ktd20xx_pm_ops,
#endif		
	},
	.probe = ktd20xx_probe,
	.remove = ktd20xx_remove,
	.shutdown = ktd20xx_shutdown,
	.id_table = ktd20xx_id,
};

static int __init ktd20xx_init(void)
{
	int err;
	printk("%s\n",__func__);
	err = i2c_add_driver(&ktd20xx_driver);
	if (err) {
		printk(KERN_WARNING "ktd20xx driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd20xx_driver.driver.name);
	}
	return err;
}

static void __exit ktd20xx_exit(void)
{
	printk("%s\n",__func__);
	i2c_del_driver(&ktd20xx_driver);
}

module_init(ktd20xx_init);
module_exit(ktd20xx_exit);

MODULE_AUTHOR("Hengguo Zhou<hengguo.zhou@ck-telecom.com>");
MODULE_DESCRIPTION("Breath LEDs driver for KTD20xx");
MODULE_ALIAS("platform:ktd20xx");
MODULE_LICENSE("GPL");


