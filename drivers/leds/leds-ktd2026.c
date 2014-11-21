#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/sched.h>

#define DRIVER_NAME "ktd2026"

//#define DRIVER_DEBUG 
#ifdef DRIVER_DEBUG 
	# define LOGD(fmt, args...) printk(KERN_ERR "[Shengwei] %s: " fmt, __FUNCTION__, ## args)
#else 
	# define LOGD(fmt, args...) 
#endif 


//#define DEBUG
#ifdef DEBUG
    #define DEBUG_OUT	1
#else
    #define DEBUG_OUT	0
#endif

#define ktd2026_out( level, fmt, ... )    printk(level  "[%s]%s():%d:" fmt "\n",DRIVER_NAME, __func__, __LINE__, __VA_ARGS__)
#define ktd2026_error_out(fmt, ...)      ktd2026_out(KERN_ERR, fmt, __VA_ARGS__)
#define ktd2026_info_out(fmt, ...)       ktd2026_out(KERN_INFO, fmt, __VA_ARGS__)
#define ktd2026_debug_out(fmt, ...)      do { if(DEBUG_OUT){ ktd2026_out( KERN_DEBUG , fmt, __VA_ARGS__);} } while (0)
#define memcln(ref, len)                    memset(ref, 0x00, len)

/* Register, mask, value defines START */
#define KTD2026_CTRL_REG		    (0x00)
#define KTD2026_CTRL_MASK		    (0x07)
#define KTD2026_CTRL_T1			    (0)
#define KTD2026_CTRL_T2			    (1)
#define KTD2026_CTRL_T3			    (2)
#define KTD2026_CTRL_T4			    (3)
#define KTD2026_CTRL_RESET_MASK		    (0x07)
#define KTD2026_CTRL_RESET_BIT		    (4)	//Bit cleared
#define KTD2026_CTRL_RESET_REG		    (5)	//Reset Registers
#define KTD2026_CTRL_RESET_DIGI		    (6)	//Reset Main Digital
#define KTD2026_CTRL_RESET_CHIP		    (7)	//Reset Chip
#define KTD2026_CTRL_ENABLE_MASK	    (0x18)
#define KTD2026_CTRL_ENABLE_SCL_SDA	    (0 << 3)	    //Shutdown at either SCL or SDA goes low
#define KTD2026_CTRL_ENABLE_SDA_TOGGLING    (1 << 3)	    //Shutdown at SCL gose low or SDA stops toggling
#define KTD2026_CTRL_ENABLE_SDA_DONTCARE    (2 << 3)	    //Shutdown at SCL goes low
#define KTD2026_CTRL_ENABLE_ALWAYS_ON	    (3 << 3)	    //Always on
#define KTD2026_CTRL_RISE_FALL_SCALING_MASK (0x60)
#define KTD2026_CTRL_RISE_FALL_SCALING_1x   (0 << 5)	    //1x normal
#define KTD2026_CTRL_RISE_FALL_SCALING_2x   (1 << 5)	    //2x slower
#define KTD2026_CTRL_RISE_FALL_SCALING_4x   (2 << 5)	    //4x slower
#define KTD2026_CTRL_RISE_FALL_SCALING_1_8x (3 << 5)	    //0.125x slower, 8x faster

#define KTD2026_FLASH_PERIOD_REG	    (0x01)
#define KTD2026_FLASH_PERIOD_MASK	    (0x7F)
#define KTD2026_FLASH_PERIOD_LINEAR_MASK    (0x80)
#define KTD2026_FLASH_PERIOD_LINEAR	    (1)

#define KTD2026_ON_TIMER_BLINK_1_REG	    (0x02)

#define KTD2026_ON_TIMER_BLINK_2_REG	    (0x03)

#define KTD2026_LED_ENABLE_CTRL_REG	    (0x04)
#define KTD2026_LED_ENABLE_CTRL_L1_MASK	    (0x03)
#define KTD2026_LED_ENABLE_CTRL_L2_MASK	    (0x0C)
#define KTD2026_LED_ENABLE_CTRL_L3_MASK	    (0x30)
#define KTD2026_LED_ENABLE_CTRL_L4_MASK	    (0xC0)
#define KTD2026_LED_ENABLE_CTRL_OFF	    (0)
#define KTD2026_LED_ENABLE_CTRL_ON	    (1)
#define KTD2026_LED_ENABLE_CTRL_BLINK_1	    (2)
#define KTD2026_LED_ENABLE_CTRL_BLINK_2	    (3)

#define KTD2026_RAMP_TIME_REG		    (0x05)
#define KTD2026_RAMP_TIME_RISE_MASK	    (0x0F)
#define KTD2026_RAMP_TIME_DOWN_MASK	    (0xF0)

#define KTD2026_CURRENT_SETTING_L1_REG	    (0x06)

#define KTD2026_CURRENT_SETTING_L2_REG	    (0x07)

#define KTD2026_CURRENT_SETTING_L3_REG	    (0x08)

#define KTD2026_CURRENT_SETTING_L4_REG	    (0x09)

#define KTD2026_CURRENT_SETTING_TH	    (0x60)	//set to 0 ~ 20ma (0 ~ 159 0x9F),n: 0~ 255  if(n <= TH*2) val = n/2, else val = n - TH;
#define KTD2026_CURRENT_SETTING_MAX	    (0xFF - KTD2026_CURRENT_SETTING_TH)

#define KTD2026_MAX_REGISTER_COUNT	    (0x0A)
#define KTD2026_MAX_LED_USED		    (2)

/* Register, mask, value defines END */

/* enums START */
enum KTD2026_LED_RGB_ENUM{
    KTD2026_LED_RGB_ENUM_R = 1,
    KTD2026_LED_RGB_ENUM_G,
    KTD2026_LED_RGB_ENUM_B,
    KTD2026_LED_RGB_ENUM_MAX
};
enum KTD2026_LED_CH_ENUM{
    KTD2026_LED_CH_ENUM_CH1 = 1,
    KTD2026_LED_CH_ENUM_CH2,
    KTD2026_LED_CH_ENUM_CH3,
    KTD2026_LED_CH_ENUM_MAX
};
enum KTD2026_LED_ACTION_ENUM{
    KTD2026_LED_ACTION_ENUM_OFF = 0,
    KTD2026_LED_ACTION_ENUM_ON,
    KTD2026_LED_ACTION_ENUM_BLINK,
    KTD2026_LED_ACTION_ENUM_MAX
};
enum STATUS_ENUM{
	NONE,
	CHARGING_RED,
	CHARGING_GREEN,
	CHARGING_RED_LOCK,
	CHARGING_GREEN_LOCK,
	MISSCALL,
	LOWBAT
};
/* enums END */


/* structs START */
struct ktd2026_data{
    struct led_classdev		cdev;
    struct i2c_client		*i2c_cli;
    struct work_struct		work;
    u8				num_leds;
    enum KTD2026_LED_RGB_ENUM	rgb;
    enum KTD2026_LED_CH_ENUM	ch;
};
/* structs EBD */

/* Gobal valable START */
u8		ktd2026_gReg[KTD2026_MAX_REGISTER_COUNT];
struct mutex	*ktd2026_gRWlock;
struct mutex	action_mutex;
/* Gobal valable END */

/* [Shalin,20141024] add led priority start*/
static unsigned int low_battery_blink = 0;    //1:on  0:off
static unsigned int missed_call_blink = 0;    //1:on  0:off
static enum STATUS_ENUM status = NONE;    //1:on  0:off

static unsigned int chg_in = 0;    //1:on  0:off
struct ktd2026_data *led_R, *led_G;
/*[Shalin,20141006] shalin add END*/

/* Function defines START */
    /* Releated to sysfs */
static void ktd2026_led_set(struct led_classdev *led_cdev, enum led_brightness value);					/* Set led brightness entry */
static int resetLight(void);
static enum led_brightness ktd2026_led_get(struct led_classdev *led_cdev);						/* Get led brightness entry */
static ssize_t ktd2026_blink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);	/* Set led blinking entry */
static ssize_t ktd2026_Missedcall_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);	/* [Shalin, 20141024] Set Missedcall led blinking entry */
static ssize_t ktd2026_shutdown_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);	/* [Shalin, 20140930] Set led shutdown entry */
static void ktd2026_led_work(struct work_struct *work);	    /* ktd2026_led_set will starting this worker function and set brightness */
void ktd2026_set_led_blinking(struct ktd2026_data *led);    /* Set led blinking */
void ktd2026_set_led_shutdown(struct ktd2026_data *led);  /*[Shalin,20140930] set control IC shutdown */
    /* Basic I2C R/W */
static int ktd2026_i2c_masked_write_reg_i2c(struct i2c_client *client, u8 reg, u8 mask, u8 val);    /* Write only some bit change */
static int __ktd2026_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val);			    /* write one byte */
static int __ktd2026_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val);			    /* Read one byte */
    /* LED actions */
static int ktd2026_reset_chip(struct i2c_client *client);
static int ktd2026_init_reg(struct i2c_client *client);
static int ktd2026_set_max_brightness(struct ktd2026_data *led);				/* Set max brightness */
static int ktd2026_set_brightness(struct ktd2026_data *led);					/* set brightness */
static int ktd2026_set_all_led_off(struct ktd2026_data *led);					/* Turn off all LED channels */
static int ktd2026_set_led_action_on(struct ktd2026_data *led);					/* Turn On LED */
static int ktd2026_set_led_action_off(struct ktd2026_data *led);				/* Turn Off LED */
static int ktd2026_set_led_action_blinking(struct ktd2026_data *led);				/* Turn LED into blinking mode */
static int ktd2026_set_led_action(struct ktd2026_data *led, enum KTD2026_LED_ACTION_ENUM act);	/* Set LED ON/OFF/Blinking */
    /* Tools */
u8 ktd2026_brightness_2_current(u8 brightness);							/* Current 0 ~ 159, brightness 0~255, using this func. to mapping b->c */
u8 ktd2026_brightness_red_current(u8 brightness);							/* [Shalin, 20141024] brightness 0 current 0, brightness 255 current 39 (5mA) */
u8 ktd2026_brightness_green_current(u8 brightness);							/* [Shalin, 20141024] brightness 0 current 0, brightness 255 current 79 (10mA)*/
    /* Driver basic */
static int ktd2026_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ktd2026_remove(struct i2c_client *client);
static void ktd2026_shutdown(struct i2c_client *client);
static int __init ktd2026_init(void);
static void __exit ktd2026_exit(void);
/* Function defines END */

/* sysfs START */
static DEVICE_ATTR(blink, 0664, NULL, ktd2026_blink_store);
static struct attribute *ktd2026_blink_attrs[] = {
    &dev_attr_blink.attr,
    NULL
};
static const struct attribute_group ktd2026_blink_attr_group = {
    .attrs = ktd2026_blink_attrs,
};
/* sysfs END */

/*[Shalin,20141024] sysfs START */
static DEVICE_ATTR(Missedcall, 0664, NULL, ktd2026_Missedcall_store);
static struct attribute *ktd2026_Missedcall_attrs[] = {
    &dev_attr_Missedcall.attr,
    NULL
};
static const struct attribute_group ktd2026_Missedcall_attr_group = {
    .attrs = ktd2026_Missedcall_attrs,
};
/*[Shalin,20141024] sysfs END */

/*[Shalin,20140930] sysfs START */
static DEVICE_ATTR(shutdown, 0664, NULL, ktd2026_shutdown_store);
static struct attribute *ktd2026_shutdown_attrs[] = {
    &dev_attr_shutdown.attr,
    NULL
};
static const struct attribute_group ktd2026_shutdown_attr_group = {
    .attrs = ktd2026_shutdown_attrs,
};
/*[Shalin,20140930] sysfs END */


/* Function implement START */

    /* Releated to sysfs START */
static void ktd2026_led_set(struct led_classdev *led_cdev, enum led_brightness value){
    struct ktd2026_data *led;

    ktd2026_debug_out("%s", "");

    led = container_of(led_cdev, struct ktd2026_data, cdev);
    if (value < LED_OFF) {
	ktd2026_error_out("ERROR: Invalid brightness value %d\n", value);
	return;
    }

    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);
    ktd2026_debug_out("Set CH %d brightness to %d", led->ch, value);

    if (value > led->cdev.max_brightness)
	value = led->cdev.max_brightness;

    led->cdev.brightness = value;
    schedule_work(&led->work);
    return;
}

static enum led_brightness ktd2026_led_get(struct led_classdev *led_cdev)
{
    struct ktd2026_data *led;
    led = container_of(led_cdev, struct ktd2026_data, cdev);
    ktd2026_debug_out("%s", "");
    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);
    led = container_of(led_cdev, struct ktd2026_data, cdev);
    return led->cdev.brightness;
}

static ssize_t ktd2026_blink_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{

    struct ktd2026_data *led;
    unsigned long blinking;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    ssize_t ret = -EINVAL;

    ktd2026_debug_out("%s", "");

    ret = kstrtoul(buf, 10, &blinking);
    if (ret){
	ktd2026_error_out("ERROR: unaccepable input \"%s\"", buf);
	return ret;
    }
    led = container_of(led_cdev, struct ktd2026_data, cdev);
    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);
    led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
LOGD("led->rgb = %d, led->ch = %d, blinking = %lu\n", led->rgb, led->ch, blinking);
    switch (led->ch) {
	case KTD2026_LED_CH_ENUM_CH1:
	    if(led->cdev.brightness > 0){
			LOGD("low_battery_blink = 1\n");
	        low_battery_blink = 1;
		}
	    else{
			LOGD("low_battery_blink = 0\n");
	        low_battery_blink = 0;
		}
		ktd2026_set_led_action(led_R, KTD2026_LED_ACTION_ENUM_BLINK);
	   // ktd2026_set_led_blinking(led);
	    break;
	case KTD2026_LED_CH_ENUM_CH2:
		if(missed_call_blink != 1)
	 		ktd2026_set_led_blinking(led_G);
		break;
	case KTD2026_LED_CH_ENUM_CH3:
	    //ktd2026_set_led_blinking(led);
	    break;
	default:
	    ktd2026_error_out("ERROR: Invalid LED Channel %d\n", led->ch);
	    return -EINVAL;
    }
    return count;
}

/* [Shalin,20141024] add led priority start*/
static ssize_t ktd2026_Missedcall_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{

    struct ktd2026_data *led;
    unsigned long blinking;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    ssize_t ret = -EINVAL;

    ktd2026_debug_out("%s", "");

    ret = kstrtoul(buf, 10, &blinking);
    if (ret){
	ktd2026_error_out("ERROR: unaccepable input \"%s\"", buf);
	return ret;
    }
    led = container_of(led_cdev, struct ktd2026_data, cdev);
    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);
    led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;
LOGD("rgb = %d ch = %d, cdev.brightness = %d\n", led->rgb, led->ch, led->cdev.brightness);
    switch (led->ch) {
	case KTD2026_LED_CH_ENUM_CH1:
		//LOGD("Red from Miss call\n");
//	    ktd2026_set_led_blinking(led);
	    break;
	case KTD2026_LED_CH_ENUM_CH2:
	    if(led->cdev.brightness > 0){
			LOGD("**** missed_call_blink = 1 ****");
	        missed_call_blink = 1;
	    //    ktd2026_set_led_blinking(led);
			ktd2026_set_led_action(led_G, KTD2026_LED_ACTION_ENUM_BLINK);
	    }
	    else{
			LOGD("**** missed_call_blink = 0, statud = NONE****");
	        missed_call_blink = 0;
			status = NONE;
			LOGD("chg_in = %d, CHARGING_RED = %d, CHARGING_GREEN = %d\n", chg_in, CHARGING_RED, CHARGING_GREEN);
	        if(chg_in == CHARGING_RED){  
				LOGD("Enable charging red led\n");
				ktd2026_set_led_action(led_R, KTD2026_LED_ACTION_ENUM_ON);
			}
			else if(chg_in == CHARGING_GREEN){
				LOGD("Enable charging green led\n");
				ktd2026_set_led_action(led_G, KTD2026_LED_ACTION_ENUM_ON);
			}
			else if(low_battery_blink){
				LOGD("Enable low batter event\n");
				ktd2026_set_led_action(led_R, KTD2026_LED_ACTION_ENUM_BLINK);
			}
			else
				ktd2026_set_all_led_off(led);
#if 0			
	        if(chg_in == 1){  
	            ktd2026_set_led_blinking(led);
	            
	            ktd2026_set_max_brightness(led_R);  // charging, red solid
	            ktd2026_set_led_action(led_R, KTD2026_LED_ACTION_ENUM_ON);
	        }
	        else
#endif				
	    }
	    break;
	case KTD2026_LED_CH_ENUM_CH3:
	    ktd2026_set_led_blinking(led);
	    break;
	default:
	    ktd2026_error_out("ERROR: Invalid LED Channel %d\n", led->ch);
	    return -EINVAL;
    }
    return count;
}
/* [Shalin,20141024] add led priority END*/

/* [Shalin,20140930] shutdown led control IC entry START */
static ssize_t ktd2026_shutdown_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{

    struct ktd2026_data *led;
    unsigned long shutdown;
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    ssize_t ret = -EINVAL;

    ktd2026_debug_out("%s", "");

    ret = kstrtoul(buf, 10, &shutdown);
    if (ret){
	ktd2026_error_out("ERROR: unaccepable input \"%s\"", buf);
	return ret;
    }
    led = container_of(led_cdev, struct ktd2026_data, cdev);
    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);
  
    
    if (shutdown > 0){
    	ktd2026_set_all_led_off(led);
    	ktd2026_debug_out("Shutdown LED control IC%s\n","");
    	__ktd2026_i2c_write_reg(led->i2c_cli, 0x00, 0x08);        //Set LED control IC into shutdown mode
    	
    }
    else{
    	ktd2026_debug_out("POWER ON LED control IC%s\n","");
	__ktd2026_i2c_write_reg(led->i2c_cli, 0x00, 0x18);        //Set T1, Always ON, 1X scaling    
    }
    return count;
}
/* [Shalin,20140930] shutdown led control IC entry END */

static void ktd2026_led_work(struct work_struct *work)
{
    struct ktd2026_data *led = container_of(work, struct ktd2026_data, work);

if(led->rgb == KTD2026_LED_RGB_ENUM_R)
	LOGD("Red, led->cdev.brightness = %d\n", led->cdev.brightness);
else
	LOGD("Green, led->cdev.brightness = %d\n", led->cdev.brightness);
	if(led->rgb == KTD2026_LED_RGB_ENUM_G){
		if(led->cdev.brightness == 254 || led->cdev.brightness == 0)
			led->rgb = KTD2026_LED_RGB_ENUM_G;
		else
			return;
	}
    ktd2026_debug_out("%s", "");
    ktd2026_debug_out("led = 0x%08x\n", (int)led);

//    ktd2026_set_brightness(led);
    if(led->cdev.brightness > 0)
		ktd2026_set_led_action_on(led);
    else
		ktd2026_set_led_action_off(led);
}

void ktd2026_set_led_blinking(struct ktd2026_data *led){
    ktd2026_debug_out("%s", "");
    ktd2026_debug_out("Get led = 0x%04x rgb = %d ch = %d", (unsigned int)led, led->rgb, led->ch);

    ktd2026_set_all_led_off(led);
    if(led->cdev.brightness > 0){
	ktd2026_set_led_action_off(led);
	ktd2026_set_max_brightness(led);
	ktd2026_set_led_action_blinking(led);
    }
}

    /* Releated to sysfs END */

    /* Basic I2C R/W START */
static int ktd2026_i2c_masked_write_reg_i2c(struct i2c_client *client, u8 reg, u8 mask, u8 val){
    int rc;
    u8 data;

    ktd2026_debug_out("Masked write Reg[0x%02x] mask = 0x%02x val = %d\n", reg, mask, val);

    rc = __ktd2026_i2c_read_reg(client, reg, &data);
    if(rc < 0){
	ktd2026_error_out("ERROR: Could not read Reg[0x%02x]\n", reg);
	return rc;
    }

    ktd2026_debug_out("Data = 0x%02x\n", data);
    data &= ~mask;
    ktd2026_debug_out("Data = 0x%02x\n", data);
    data |= val;
    ktd2026_debug_out("Data = 0x%02x\n", data);
    rc = __ktd2026_i2c_write_reg(client, reg, data);
    if(rc){
	ktd2026_error_out("ERROR: Could not write Reg[0x%02x] = 0x%02x (%d)\n", reg, val, val);
    }
    
    	/* [Shalin,20141023] shutdown led control IC when turn off leds start*/
//        if(data == 0)
//            __ktd2026_i2c_write_reg(client, KTD2026_CTRL_REG, 0x08);
//        else
    __ktd2026_i2c_write_reg(client, KTD2026_CTRL_REG, 0x18);
        /* [Shalin,20141023] shutdown led control IC when turn off leds END*/

    return rc < 0 ? rc : 0;
}

static int __ktd2026_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val){
    int rc;
    
    ktd2026_debug_out("Writing Reg[%d] = 0x%02x (%d)\n", reg, val, val);

    mutex_lock(ktd2026_gRWlock);
    rc = i2c_smbus_write_byte_data(client, reg, val);
    if(rc < 0){
	ktd2026_error_out("ERROR: writing Reg[0x%02x] = %d rc = %d\n", reg, val,  rc);
    }
    else{
	ktd2026_debug_out("Writing Reg[%d] = 0x%02x (%d) success\n", reg, val, val);
	ktd2026_gReg[reg] = val;
    }
    mutex_unlock(ktd2026_gRWlock);
    return rc;
}
static int __ktd2026_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val){
    int rc;

    /* Because this decive does not suppot i2c read, so using array to remember reg value, start with init reg */
    ktd2026_debug_out("Reading Reg[%d]\n", reg);

    mutex_lock(ktd2026_gRWlock);
    if(reg > KTD2026_MAX_REGISTER_COUNT)
	rc = -EINVAL;
    else
	rc = 0;
    *val = rc ? 0	: ktd2026_gReg[reg];
    mutex_unlock(ktd2026_gRWlock);

    return rc;
}
    /* Basic I2C R/W END */

    /* LED actions START */
static int ktd2026_reset_chip(struct i2c_client *client){
    int rc;
    rc = __ktd2026_i2c_write_reg(client, KTD2026_CTRL_REG, 0x07);
    usleep(700);
    return rc;
}

static int ktd2026_init_reg(struct i2c_client *client){

    int rc;

    rc = ktd2026_reset_chip(client);
    rc = __ktd2026_i2c_write_reg(client, 0x04,0x00);	    //Set all OFF
    //rc = __ktd2026_i2c_write_reg(client, 0x00,0x18);        //Set T1, Always ON, 1X scaling
    rc = __ktd2026_i2c_write_reg(client, 0x00,0x08);        //Set led control IC into shutdown mode
//    rc = __ktd2026_i2c_write_reg(client, 0x06,0x9f);	    //Set L1 20.00mA
//    rc = __ktd2026_i2c_write_reg(client, 0x07,0x9f);	    //Set L2 20.00mA
    rc = __ktd2026_i2c_write_reg(client, 0x06,0x27);	    //Set L1 5.00mA
    rc = __ktd2026_i2c_write_reg(client, 0x07,0x4f);	    //Set L2 10.00mA
    rc = __ktd2026_i2c_write_reg(client, 0x08,0x00);	    //Set L3 0.125mA
    rc = __ktd2026_i2c_write_reg(client, 0x09,0x00);	    //Set L4 0.125mA
    rc = __ktd2026_i2c_write_reg(client, 0x05,0x00);	    //Set Ramp time, Rice 0ms, fall 0ms
    rc = __ktd2026_i2c_write_reg(client, 0x01,0x0E);	    //Set flash period 2.048s
    rc = __ktd2026_i2c_write_reg(client, 0x02,0x08);	    //Set ON timer T1 8 (3.1%)
    rc = __ktd2026_i2c_write_reg(client, 0x03,0x00);	    //Set ON timer T2 0	  ( 0%)

    if(rc)
	ktd2026_error_out("ERROR: Unable init register %s\n", "");

    return rc;
}

static int ktd2026_set_max_brightness(struct ktd2026_data *led){
    led->cdev.brightness = led->cdev.max_brightness;
    ktd2026_debug_out("%s", "");
    return ktd2026_set_brightness(led);
}
static int ktd2026_set_brightness(struct ktd2026_data *led){
    int rc;
    u8 value, reg;

    ktd2026_debug_out("%s", "");

//    value = ktd2026_brightness_2_current(led->cdev.brightness);
//    ktd2026_debug_out("Set LED[%d] brightness %d, currnet %d\n",led->ch, led->cdev.brightness, value);

    switch (led->ch) {
	case KTD2026_LED_CH_ENUM_CH1:
	    reg = KTD2026_CURRENT_SETTING_L1_REG;
	    value = ktd2026_brightness_red_current(led->cdev.brightness);  //5mA
            ktd2026_debug_out("Set LED[%d] brightness %d, currnet %d\n",led->ch, led->cdev.brightness, value);
	    break;
	case KTD2026_LED_CH_ENUM_CH2:
	    reg = KTD2026_CURRENT_SETTING_L2_REG;
	    value = ktd2026_brightness_green_current(led->cdev.brightness);  //10mA
            ktd2026_debug_out("Set LED[%d] brightness %d, currnet %d\n",led->ch, led->cdev.brightness, value);
	    break;
	case KTD2026_LED_CH_ENUM_CH3:
	    reg = KTD2026_CURRENT_SETTING_L3_REG;
	    break;
	default:
	    return -EINVAL;
	    break;
    }
    rc = __ktd2026_i2c_write_reg(led->i2c_cli, reg, value);
    if(rc){
	ktd2026_error_out("ERROR: Unable to write Reg[0x%02x] = 0x%02x", reg, value);
	return rc;
    }
    else
    {
	ktd2026_debug_out("Write Reg[0x%02x] = 0x%02x", reg, value);
    }

    return 0;
}
static int ktd2026_set_all_led_off(struct ktd2026_data *led){
    int i = led->num_leds - 1, rc;
    struct ktd2026_data *led_array = dev_get_drvdata(&led->i2c_cli->dev), *leds;
    ktd2026_debug_out("%s", "");
    for(; i > -1 ; --i){
	leds = &led_array[i];
	ktd2026_debug_out("Get led = 0x%04x ch = %d rgb = %d\n", (unsigned int)leds, leds->ch, leds->rgb);
	rc = ktd2026_set_led_action_off(leds);
	if(rc < 0){
	    ktd2026_error_out("ERROR: fail to turn off led = 0x%04x ch = %d rgb = %d\n", (unsigned int)leds, leds->ch, leds->rgb);
	    return rc;
	}
	else{
	    ktd2026_debug_out("Turn off led = 0x%04x ch = %d rgb = %d\n", (unsigned int)leds, leds->ch, leds->rgb);
	}
    }
    return 0;
}
static int ktd2026_set_led_action_on(struct ktd2026_data *led){
    ktd2026_debug_out("%s", "");
    return ktd2026_set_led_action(led, KTD2026_LED_ACTION_ENUM_ON);
}
static int ktd2026_set_led_action_off(struct ktd2026_data *led){
    ktd2026_debug_out("%s", "");
    return ktd2026_set_led_action(led, KTD2026_LED_ACTION_ENUM_OFF);
}
static int ktd2026_set_led_action_blinking(struct ktd2026_data *led){
    ktd2026_debug_out("%s", "");
    return ktd2026_set_led_action(led, KTD2026_LED_ACTION_ENUM_BLINK);
}
static int resetLight(void)
{
	int rc;
    u8 mask, value, reg, shift;
    reg = KTD2026_LED_ENABLE_CTRL_REG;
	shift = 2;
	mask = KTD2026_LED_ENABLE_CTRL_L2_MASK;
	value = KTD2026_LED_ACTION_ENUM_OFF;  //red blinking	
  	rc = ktd2026_i2c_masked_write_reg_i2c(led_R->i2c_cli, reg, mask, value << shift);
    if(rc){
		ktd2026_error_out("ERROR: Unable to mask write Reg[0x%02x] mask = 0x%02x value = 0x%02x\n", reg, mask, value);
		return rc;
    }
	shift = 0;
    mask = KTD2026_LED_ENABLE_CTRL_L1_MASK;
   	value = KTD2026_LED_ACTION_ENUM_OFF;  //red blinking	
   	rc = ktd2026_i2c_masked_write_reg_i2c(led_R->i2c_cli, reg, mask, value << shift);
    if(rc){
		ktd2026_error_out("ERROR: Unable to mask write Reg[0x%02x] mask = 0x%02x value = 0x%02x\n", reg, mask, value);
		return rc;
    }
	return 0;
	
}
static void stateTransition(struct ktd2026_data *led, u8 *shift, u8 *mask, u8 *value)
{
	int rc;
	
	rc = resetLight();
    if(rc)
		LOGD("ERROR: Unable to reset light\n");

    if (low_battery_blink == 1){
		LOGD("*****low baater Start ******\n");
		*shift = 0;
		*mask = KTD2026_LED_ENABLE_CTRL_L1_MASK;
		*value = KTD2026_LED_ACTION_ENUM_BLINK;
		ktd2026_set_max_brightness(led);
		status = LOWBAT;
		LOGD("*****low baater End ******\n");
	}
	else if(missed_call_blink == 1){
		LOGD("*****Miss call Start ******\n");
		*shift = 2;
		*mask = KTD2026_LED_ENABLE_CTRL_L2_MASK;
		*value = KTD2026_LED_ACTION_ENUM_BLINK;
		ktd2026_set_max_brightness(led);
		status = MISSCALL;
		LOGD("*****Miss call End ******\n");
	}
	else if(chg_in != 0){
		if(chg_in == CHARGING_RED){
		LOGD("chg_in = %d, status = %d, CHARGING_RED = %d, CHARGING_GREEN = %d, CHARGING_RED_LOCK = %d, CHARGING_GREEN_LOCK = %d\n", chg_in, status, CHARGING_RED, CHARGING_GREEN, CHARGING_RED_LOCK, CHARGING_GREEN_LOCK);
		LOGD("*****Red charging Start ******\n");	
 			*shift = 0;
		    *mask = KTD2026_LED_ENABLE_CTRL_L1_MASK;
			ktd2026_set_max_brightness(led_R);
			status = CHARGING_RED_LOCK;
		LOGD("*****Red charging End ******\n");
		}else{
		LOGD("*****Green charging Start ******\n");	
		LOGD("chg_in = %d, status = %d, CHARGING_RED = %d, CHARGING_GREEN = %d, CHARGING_RED_LOCK = %d, CHARGING_GREEN_LOCK = %d\n", chg_in, status, CHARGING_RED, CHARGING_GREEN, CHARGING_RED_LOCK, CHARGING_GREEN_LOCK);
 			*shift = 2;
		    *mask = KTD2026_LED_ENABLE_CTRL_L2_MASK;
			ktd2026_set_max_brightness(led_G);
			status = CHARGING_GREEN_LOCK;
		LOGD("*****Green charging End ******\n");
		}
	    *value = KTD2026_LED_ACTION_ENUM_ON;  //red blinking		
	}
	
}
static int priorityCheck(struct ktd2026_data *led, u8 *shift, u8 *mask, u8 *value)
{
	int res = 0;
    if (low_battery_blink == 1){
		if(status != LOWBAT)
			stateTransition(led_R, shift, mask, value);
		else {
			LOGD("**** low_bat then return ****");
			res = 1;
		}
    }
    else if (missed_call_blink == 1){
		if(status != MISSCALL)
			stateTransition(led_G, shift, mask, value);
		else {
			LOGD("**** misscall then return ****");
			res = 1;
		}
    }
	else if(chg_in != 0){
		if(status != CHARGING_RED_LOCK && status != CHARGING_GREEN_LOCK)
			stateTransition(led, shift, mask, value);
		else if(chg_in == CHARGING_RED && status == CHARGING_GREEN_LOCK)
			stateTransition(led, shift, mask, value);
		else if(chg_in == CHARGING_GREEN && status == CHARGING_RED_LOCK)
			stateTransition(led, shift, mask, value);
		else{
			LOGD("**** charging then return ****");
			res = 1;
		}
	}
	return res;
}
static int ktd2026_set_led_action(struct ktd2026_data *led, enum KTD2026_LED_ACTION_ENUM act){
    int rc;
//    u8 mask, value, reg, shift, s_value;
    u8 mask, value, reg, shift;

    ktd2026_debug_out("%s", "");
mutex_lock(&action_mutex);
    reg = KTD2026_LED_ENABLE_CTRL_REG;
LOGD("\n");
    switch (led->ch) {
	case KTD2026_LED_CH_ENUM_CH1:
	    mask = KTD2026_LED_ENABLE_CTRL_L1_MASK;
LOGD(" red **********");
	    shift = 0;
	    break;
	case KTD2026_LED_CH_ENUM_CH2:
LOGD("green *********");
	    mask = KTD2026_LED_ENABLE_CTRL_L2_MASK;
	    shift = 2;
	    break;
	case KTD2026_LED_CH_ENUM_CH3:
	    mask = KTD2026_LED_ENABLE_CTRL_L3_MASK;
	    shift = 4;
	    break;
	default:
	    return -EINVAL;
	    break;
    }

    switch(act){
	case KTD2026_LED_ACTION_ENUM_OFF:
LOGD("close ********\n");
	    value = KTD2026_LED_ENABLE_CTRL_OFF;
   		if(led->ch == KTD2026_LED_CH_ENUM_CH1){
			status = NONE;
			low_battery_blink = 0;
			if(chg_in == CHARGING_RED){
LOGD("Discharging RED\n");			
				chg_in = 0;
			}
		}else{
			if(chg_in == CHARGING_GREEN){
LOGD("Discharging GREEN\n");			
				status = NONE;
				chg_in = 0;
			}
		}
//	    s_value = 0x08;
	    break;
	case KTD2026_LED_ACTION_ENUM_ON:
LOGD("enable ********\n");
	    value = KTD2026_LED_ENABLE_CTRL_ON;
		low_battery_blink = 0;
   		if(led->ch == KTD2026_LED_CH_ENUM_CH1)
			chg_in = CHARGING_RED;
		else
			chg_in = CHARGING_GREEN;

//	    s_value = 0x18;i
	    break;
	case KTD2026_LED_ACTION_ENUM_BLINK:
LOGD("blink ************\n");
	    value = KTD2026_LED_ENABLE_CTRL_BLINK_1;
   		if(led->ch == KTD2026_LED_CH_ENUM_CH1){
LOGD("low_battery_blink = 1; chg_in = 0\n");		
			low_battery_blink = 1;
			chg_in = 0;
		}
//	    s_value = 0x18;
	    break;
	default:
mutex_unlock(&action_mutex);
	    return -EINVAL;
	    break;
    }
LOGD("\n");
LOGD("\n");
LOGD("******before prioiry transition ***********\n"); 
LOGD("status = %d\n", status);
LOGD("low_battery_blink = %d\n", low_battery_blink);
LOGD("missed_call_blink = %d\n", missed_call_blink);
LOGD("chg_in = %d\n", chg_in);
LOGD("******before prioiry transition ***********\n");
LOGD("\n");
LOGD("\n");
	
	if(priorityCheck(led, &shift, &mask, &value)){
		mutex_unlock(&action_mutex);
		return 0;
	}
LOGD("shift = %d, mask = %d, value = %d\n", shift, mask, value);		
    /* [Shalin,20140930] shutdown led control IC when turn off leds start*/
//    rc = __ktd2026_i2c_write_reg(led->i2c_cli, KTD2026_CTRL_REG, s_value);
//     if(rc){
//	ktd2026_error_out("ERROR: Unable to write Reg[0x00] s_value = 0x%02x success\n", s_value);
//	return rc;
//    }
    /* [Shalin,20140930] shutdown led control IC when turn off led END*/
    rc = ktd2026_i2c_masked_write_reg_i2c(led->i2c_cli, reg, mask, value << shift);
    if(rc){
	ktd2026_error_out("ERROR: Unable to mask write Reg[0x%02x] mask = 0x%02x value = 0x%02x\n", reg, mask, value);
	return rc;
    }
    else{
	ktd2026_debug_out("masked write Reg[0x%02x] mask = 0x%02x value = 0x%02x", reg, mask, value);
    }
	rc = __ktd2026_i2c_read_reg(led->i2c_cli, reg, &value);

	rc = low_battery_blink + missed_call_blink + chg_in;
	if(value == 0 && rc == 0)
		__ktd2026_i2c_write_reg(led->i2c_cli, KTD2026_CTRL_REG, 0x08);
	else
		__ktd2026_i2c_write_reg(led->i2c_cli, KTD2026_CTRL_REG, 0x18);
	
mutex_unlock(&action_mutex);
    return rc;
}
    /* LED actions END */

    /* Tools START */
u8 ktd2026_brightness_2_current(u8 brightness){
    ktd2026_debug_out("%s", "");
    return brightness <= KTD2026_CURRENT_SETTING_TH * 2 ? brightness : brightness - KTD2026_CURRENT_SETTING_TH;
}

u8 ktd2026_brightness_red_current(u8 brightness){
    ktd2026_debug_out("%s", "");
    return brightness > 0 ? 39 : 0;  //5mA or 0mA
}

u8 ktd2026_brightness_green_current(u8 brightness){
    ktd2026_debug_out("%s", "");
    return brightness > 0 ? 79 : 0;  //10mA or 0mA
}
    /* Tools END */

    /* Driver basic START */
static int ktd2026_probe(struct i2c_client *client, const struct i2c_device_id *id){
    
    struct ktd2026_data *led, *led_array;
    struct mutex	*lock;
    int rc;

    ktd2026_debug_out("Slave addr = 0x%02x", client->addr);

    led_array = devm_kzalloc(&client->dev, (sizeof(struct ktd2026_data) * KTD2026_MAX_LED_USED), GFP_KERNEL);
    if (!led_array) {
	ktd2026_error_out("ERROR: Unable to allocate memory for %s\n", "led_array");
	return -ENOMEM;
    }

    lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
    if (!lock){
	ktd2026_error_out("ERROR: Unable to allocate memory for %s\n", "mutex");
	return -ENOMEM;
    }

    mutex_init(lock);
	mutex_init(&action_mutex);
    ktd2026_gRWlock = lock;

    rc = ktd2026_init_reg(client);
    if(rc){
	ktd2026_error_out("ERROR: Unable to init the registers %s\n", "");
	return -ENODEV;
    }
    else
	ktd2026_debug_out("Init reg successfuly%s\n", "");

    /* Register /sys/class/leds/red to Channel 1 */
    led = &led_array[0];
    ktd2026_info_out(" led = 0x%08x\n", (unsigned int)led);
    led->num_leds   = KTD2026_MAX_LED_USED;
    led->i2c_cli    = client;
    led->cdev.name  = "red";
    led->rgb	    = KTD2026_LED_RGB_ENUM_R;
    led->ch         = KTD2026_LED_CH_ENUM_CH1;
    led->cdev.brightness = 0;
    led->cdev.brightness_set    = ktd2026_led_set;
    led->cdev.brightness_get    = ktd2026_led_get;
    led_R = led;
    led_classdev_register(&client->dev, &led->cdev);
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_blink_attr_group);
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_Missedcall_attr_group); // [Shalin, 20141024] Set Missedcall led blinking entry
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_shutdown_attr_group);  //[Shalin,20140930] add for shutdown led control IC	
    INIT_WORK(&led->work, ktd2026_led_work);

    /* Register /sys/class/leds/green to Channel 2 */
    led = &led_array[1];
    ktd2026_info_out(" led = 0x%08x\n", (unsigned int)led);
    led->num_leds   = KTD2026_MAX_LED_USED;
    led->i2c_cli    = client;
    led->cdev.name  = "green";
    led->rgb	    = KTD2026_LED_RGB_ENUM_G;
    led->ch         = KTD2026_LED_CH_ENUM_CH2;
    led->cdev.brightness = 0;
    led->cdev.brightness_set    = ktd2026_led_set;
    led->cdev.brightness_get    = ktd2026_led_get;
    led_G = led;
    led_classdev_register(&client->dev, &led->cdev);
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_blink_attr_group);
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_Missedcall_attr_group); // [Shalin, 20141024] Set Missedcall led blinking entry
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_shutdown_attr_group);  //[Shalin,20140930] add for shutdown led control IC
    INIT_WORK(&led->work, ktd2026_led_work);
	
	__ktd2026_i2c_write_reg(client, KTD2026_CTRL_REG, 0x18);
    /* Register /sys/class/leds/blue to Channel 3 */
    /*	No blue led in our device
    led = &led_array[2];
    ktd2026_info_out(" led = 0x%08x\n", (unsigned int)led);
    led->num_leds   = KTD2026_MAX_LED_USED;
    led->i2c_cli    = client;
    led->cdev.name  = "blue";
    led->rgb        = KTD2026_LED_RGB_ENUM_B;
    led->ch	    = KTD2026_LED_CH_ENUM_CH3;
    led->cdev.brightness = 0;
    led->cdev.brightness_set    = ktd2026_led_set;
    led->cdev.brightness_get    = ktd2026_led_get;
    led_classdev_register(&client->dev, &led->cdev);
    rc = sysfs_create_group(&led->cdev.dev->kobj, &ktd2026_blink_attr_group);
    INIT_WORK(&led->work, ktd2026_led_work);
    */
    dev_set_drvdata(&client->dev, led_array);

    return 0;
}

static int ktd2026_remove(struct i2c_client *client){

    int i;//, rc;
    struct ktd2026_data *led_array = dev_get_drvdata(&client->dev), *leds;

    ktd2026_debug_out("%s", "");

    leds = led_array;
    i = leds->num_leds - 1;
    /* Remove LEDs sys entry */
    for(; i > -1 ; --i){
	leds = &led_array[i];
	ktd2026_debug_out("Get led = 0x%04x ch = %d rgb = %d\n", (unsigned int)leds, leds->ch, leds->rgb);
	led_classdev_unregister(&leds->cdev);
	sysfs_remove_group(&leds->cdev.dev->kobj, &ktd2026_blink_attr_group);
    }
    ktd2026_init_reg(client);
    kfree(led_array);
    kfree(ktd2026_gRWlock);
    kfree(ktd2026_gReg);
    return 0;
}

static void ktd2026_shutdown(struct i2c_client *client){


    struct ktd2026_data *led_array = dev_get_drvdata(&client->dev), *leds;
    leds = led_array;
    
    low_battery_blink = 0;
    missed_call_blink = 0;
    chg_in = 0;
    ktd2026_set_all_led_off(leds);
	
    ktd2026_debug_out("Shutdown LED control IC%s\n","");
    __ktd2026_i2c_write_reg(client, 0x00,0x08);        //Set LED control IC into shutdown mode

}


    /* Driver basic END */
/* Function implement END */


static struct of_device_id ktd2026_table[] = {
    { .compatible = "kinetic,ktd2026",},
    { },
};
static struct i2c_device_id ktd2026_idtable[] = {
    { DRIVER_NAME, 0 },
    { }
};
static struct i2c_driver ktd2026_driver = {
    .driver = {
	.of_match_table = ktd2026_table,
	
	.owner  = THIS_MODULE,
	.name   = DRIVER_NAME
    },
    .id_table	= ktd2026_idtable,
    .probe	= ktd2026_probe,
    .remove	= ktd2026_remove,
    .shutdown	= ktd2026_shutdown,
};

static int __init ktd2026_init(void)
{
    return i2c_add_driver(&ktd2026_driver);
}
static void __exit ktd2026_exit(void)
{
    i2c_del_driver(&ktd2026_driver);
}

module_init(ktd2026_init);
module_exit(ktd2026_exit);
//module_i2c_driver(ktd2026_driver);

MODULE_DEVICE_TABLE(i2c, ktd2026_idtable);
MODULE_AUTHOR("riven.chen@quantatw.com (http://www.quantatw.com/)");
MODULE_DESCRIPTION("KTD-2026 3 channel LED controler with PWM blinking driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
