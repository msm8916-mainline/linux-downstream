#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>

#define RSTR   0x00   //can be ID register and RESET register 0x54 or 0x55 
#define GCR    0x01   
#define STATUS 0x02
#define effecREG30 0x30
#define LED0_CTRL 0x31
#define LED1_CTRL 0x32
#define LED2_CTRL 0x33
#define PWM0_CTRL 0x34
#define PWM1_CTRL 0x35
#define PWM2_CTRL 0x36
#define LED0_T0   0x37
#define LED0_T1   0x38
#define LED0_T2   0x39
#define LED1_T0   0x3A
#define LED1_T1   0x3B
#define LED1_T2   0x3C
#define LED2_T0   0x3D
#define LED2_T1   0x3E
#define LED2_T2   0x3F
#define SELECT_I2C_ADDR   0x77


//#define Imax        0x01   //LED ,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x02   //LED,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x03   //LED,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time   0x02   //LED,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time    0x01   //LED,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time  0x00   //LED,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED,0x00=ÃÃÃÃÂŽÃ,0x01=1ÂŽÃ,0x02=2ÂŽÃ.....0x0f=15ÂŽÃ

static u8 MAX_BRIGHTNESS = 200;
static u8 Imax = 0x01;

struct aw2013_chip {
    struct i2c_client *client;
	struct device     *dev;
	struct led_classdev red;
	struct led_classdev green;
	struct led_classdev blue;
	struct work_struct work_red;
	struct work_struct work_green;
	struct work_struct work_blue;
	struct work_struct work_red_blink;
	struct work_struct work_green_blink;
	struct work_struct work_blue_blink;
	/*brightness*/
	int    red_brightness;
	int    green_brightness;
	int    blue_brightness;
	/*blink on off time*/
	unsigned long    red_ontime;
	unsigned long    red_offtime;
	unsigned long    green_ontime;
	unsigned long    green_offtime;
	unsigned long    blue_ontime;
	unsigned long    blue_offtime;
	/*status*/
	int    enable_status; 
	int    pwm;
	int    light;
	struct dentry	  *debug_root;
	struct mutex  read_write_lock;
	struct pinctrl *redledgpio_pinctrl;
	struct pinctrl_state *gpio_state_active;
	#if 0
	struct pinctrl_state *dtv_state_suspend;
	int    dtv_gpio20;
	#endif
	int    red_gpio22;
};
enum aw2013_mode{
    RED_MODE,
    GREEN_MODE,
    BLUE_MODE,
    BREATH_MODE,
};
static struct i2c_client *aw_i2c_client;//globle 
static int aw2013_blink_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off);
static void red_led_gpio22_output(struct aw2013_chip *chip);
static int first_redgpio_output = 0;
static struct aw2013_chip *the_chip;
static int __aw2013_read(struct aw2013_chip *chip, int reg,
				u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __aw2013_write(struct aw2013_chip *chip, int reg,
						u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}
#if 1
static int aw2013_write(int reg,u8 val)
{
    s32 ret ;
	ret = i2c_smbus_write_byte_data(aw_i2c_client, reg, val);
	if (ret < 0) {
		printk("aw2013_write error\n");
		return ret;
	}
	//pr_debug("wuboadd Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}
/*
static int aw2013_read(int reg)
{
    s32 ret ;
	ret = i2c_smbus_read_byte_data(aw_i2c_client, reg);
	if (ret < 0) {
		printk("aw2013_write error\n");
		return ret;
	}
	//pr_debug("wuboadd Writing 0x%02x=0x%02x\n", reg, val);
	return ret;
}*/

#endif
static void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  

#if 0
	aw2013_write(0x00, 0x55);				// Reset 
	aw2013_write(0x01, 0x01);		// enable LED Â²Â»ÃÂ¹ÃÃÃÃÂ¶Ã		

	aw2013_write(0x31, 0x70|Imax);	//config mode, IMAX = 5mA	
	aw2013_write(0x32, 0x70|Imax);	//config mode, IMAX = 5mA	
	aw2013_write(0x33, 0x70|Imax);	//config mode, IMAX = 5mA	

	aw2013_write(0x34, 0xff);	// LED0 level,
	aw2013_write(0x35, 0xff);	// LED1 level,
	aw2013_write(0x36, 0xff);	// LED2 level,
											
	aw2013_write(0x37, Rise_time<<4 | Hold_time);	//led0  ÃÃÃÃœÃÂ±ÂŒÃ€Â£Â¬Â±Â£Â³ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš							
	aw2013_write(0x38, Fall_time<<4 | Off_time);	       //led0 ÃÃÂœÂµÃÂ±ÂŒÃ€Â£Â¬Â¹ÃÂ±ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš
	aw2013_write(0x39, Delay_time<<4| Period_Num);   //led0  ÂºÃŽÃÃŒÃÃÂ³ÃÃÂ±ÂŒÃ€Â£Â¬ÂºÃŽÃÃŒÃÃÃÃÃÃšÂ¶Âš

	aw2013_write(0x3a, Rise_time<<4 | Hold_time);	//led1ÃÃÃÃœÃÂ±ÂŒÃ€Â£Â¬Â±Â£Â³ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš								
	aw2013_write(0x3b, Fall_time<<4 | Off_time);	       //led1 ÃÃÂœÂµÃÂ±ÂŒÃ€Â£Â¬Â¹ÃÂ±ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš
	aw2013_write(0x3c, Delay_time<<4| Period_Num);   //led1  ÂºÃŽÃÃŒÃÃÂ³ÃÃÂ±ÂŒÃ€Â£Â¬ÂºÃŽÃÃŒÃÃÃÃÃÃšÂ¶Âš

	aw2013_write(0x3d, Rise_time<<4 | Hold_time);	//led2  ÃÃÃÃœÃÂ±ÂŒÃ€Â£Â¬Â±Â£Â³ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš				
	aw2013_write(0x3e, Fall_time<<4 | Off_time);	       //led2 ÃÃÂœÂµÃÂ±ÂŒÃ€Â£Â¬Â¹ÃÂ±ÃÃÂ±ÂŒÃ€ÃÃšÂ¶Âš
	aw2013_write(0x3f, Delay_time<<4| Period_Num);    //ÂºÃŽÃÃŒÃÃÂ³ÃÃÂ±ÂŒÃ€Â£Â¬ÂºÃŽÃÃŒÃÃÃÃÃÃšÂ¶Âš

	aw2013_write(0x30, led2<<2|led1<<1|led0);	       //led on=0x01 ledoff=0x00	
	mdelay(2);//ÃÃšÃÃÃÂ±5usÃÃÃÃ	
#endif
}

static int aw2013_control(struct aw2013_chip *chip,
			  u8 brightness, enum aw2013_mode opmode)
{
    
    //u8 data0x30 =0;
    //__aw2013_read(chip,0x30,&data0x30);
	//printk("wubo aw2013_control mode:%d reg0x30:%x\n",opmode,data0x30);
    __aw2013_write(chip,0x30,0x07);
	//__aw2013_read(chip,0x30,&data0x30);
	//printk("wubo aw2013_control after write again reg0x30:%x\n",data0x30);
	__aw2013_write(chip,GCR,0x01);
	//__aw2013_read(chip,GCR,&data0x30);
	//printk("wubo aw2013_control after write again reg0x01:%x\n",data0x30);
    switch(opmode){
           case RED_MODE:
                //__aw2013_write(chip,LED0_CTRL,Imax);
				__aw2013_write(chip,LED1_CTRL,Imax);
                //__aw2013_write(chip,LED2_CTRL,Imax);
                __aw2013_write(chip,PWM1_CTRL,brightness);
				//udelay(5);
				//__aw2013_write(chip,0x30, 0x02);
				//udelay(5);
			    break;
		   case GREEN_MODE:
                //__aw2013_write(chip,LED0_CTRL,Imax);
				//__aw2013_write(chip,LED1_CTRL,Imax);
				__aw2013_write(chip,LED2_CTRL,Imax);
		   	    __aw2013_write(chip,PWM2_CTRL,brightness);
				//udelay(5);
				//__aw2013_write(chip,0x30, 0x04);
				//udelay(5);
				break;
		   case BLUE_MODE:
                __aw2013_write(chip,LED0_CTRL,Imax);
				//__aw2013_write(chip,LED1_CTRL,Imax);
				//__aw2013_write(chip,LED2_CTRL,Imax);
		   	    __aw2013_write(chip,PWM0_CTRL,brightness);
				//udelay(5);
				//__aw2013_write(chip,0x30, 0x01);
				//udelay(5);
		   	    break;
		   case BREATH_MODE:
		   	    break;
	}
	return 1;
}
static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_chip *chip = i2c_get_clientdata(client);

	mutex_destroy(&chip->read_write_lock);
	debugfs_remove_recursive(chip->debug_root);

	return 0;
}
static void aw2013_deferred_red_brightness_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_red);
    red_led_gpio22_output(chip);
	mutex_lock(&chip->read_write_lock);
	aw2013_control(chip, chip->red_brightness, RED_MODE);
	mutex_unlock(&chip->read_write_lock);
}
static void aw2013_deferred_red_blink_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_red_blink);
    red_led_gpio22_output(chip);
	mutex_lock(&chip->read_write_lock);
	aw2013_blink_set(&chip->red,&chip->red_ontime,&chip->red_offtime);
	mutex_unlock(&chip->read_write_lock);
}

static void aw2013_deferred_green_brightness_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_green);

	mutex_lock(&chip->read_write_lock);
	aw2013_control(chip, chip->green_brightness, GREEN_MODE);
	mutex_unlock(&chip->read_write_lock);
}

static void aw2013_deferred_green_blink_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_green_blink);

	mutex_lock(&chip->read_write_lock);
	aw2013_blink_set(&chip->green,&chip->green_ontime,&chip->green_offtime);
	mutex_unlock(&chip->read_write_lock);
}

static void aw2013_deferred_blue_brightness_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_blue);

	mutex_lock(&chip->read_write_lock);
	aw2013_control(chip, chip->blue_brightness, BLUE_MODE);
	mutex_unlock(&chip->read_write_lock);
}

static void aw2013_deferred_blue_blink_set(struct work_struct *work)
{
	struct aw2013_chip *chip =
	    container_of(work, struct aw2013_chip, work_blue_blink);

	mutex_lock(&chip->read_write_lock);
	aw2013_blink_set(&chip->blue,&chip->blue_ontime,&chip->blue_offtime);
	mutex_unlock(&chip->read_write_lock);
}

static void aw2013_red_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct aw2013_chip *chip =
	    container_of(cdev, struct aw2013_chip, red);
    chip->red_ontime  = 0;
	chip->red_offtime = 0;
	chip->red_brightness = brightness;
	schedule_work(&chip->work_red);
}

static void aw2013_green_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct aw2013_chip *chip =
	    container_of(cdev, struct aw2013_chip, green);
    chip->green_ontime  = 0;
	chip->green_offtime = 0;
	chip->green_brightness = brightness;
	schedule_work(&chip->work_green);
}

static void aw2013_blue_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct aw2013_chip *chip =
	    container_of(cdev, struct aw2013_chip, blue);
    chip->blue_ontime  = 0;
	chip->blue_offtime = 0;
	chip->blue_brightness = brightness;
	schedule_work(&chip->work_blue);
}

static int aw2013_blink_red_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
{
	struct aw2013_chip *chip =
	    container_of(led_cdev, struct aw2013_chip, red);
    
	chip->red_ontime  = *delay_on;
	chip->red_offtime = *delay_off;
	schedule_work(&chip->work_red_blink);
	return 1;
}

static int aw2013_blink_green_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
{
	struct aw2013_chip *chip =
	    container_of(led_cdev, struct aw2013_chip, green);
	
	chip->green_ontime  = *delay_on;
	chip->green_offtime = *delay_off;
	schedule_work(&chip->work_green_blink);
	return 1;
}

static int aw2013_blink_blue_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
{
	struct aw2013_chip *chip =
	    container_of(led_cdev, struct aw2013_chip, blue);

	chip->blue_ontime  = *delay_on;
	chip->blue_offtime = *delay_off;
	schedule_work(&chip->work_blue_blink);
	return 1;
}

static int aw2013_blink_set(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
{

    int delay_on_new = *delay_on;
	int delay_off_new = *delay_off;
    unsigned char risetime =0;
	unsigned char holdtime =0;
	unsigned char falltime =0;
	unsigned char offtime =0;
	//unsigned char delaytime =0;
	switch(delay_on_new)
	{
	 case 500:risetime = 0;holdtime = 2;falltime = 0;break;
	 case 1000:risetime = 1;holdtime = 3;falltime = 1;break;
	 case 2000:risetime = 1;holdtime = 4;falltime = 1;break;
	 default :risetime = 1;holdtime = 3;falltime = 1;break;
	}
	
	switch(delay_off_new)
	{
	 case 500:offtime =  2;break;
	 case 1000:offtime =  3;break;
	 case 2000:offtime =  4;break;
	 case 5000:offtime =  5;break;
	 default :offtime =  5;break;
	}

    //aw2013_read(0x30);
	//printk("wuboadd blink blink reg0x30:%x\n",aw2013_read(0x30));
    aw2013_write(0x30,0x07);
	//aw2013_read(0x30);
	//printk("wuboadd blink after write again reg0x30:%x\n",aw2013_read(0x30));
	aw2013_write(GCR, 0x01);
	//printk("wuboadd blink after write again reg0x01:%x\n",aw2013_read(0x01));
	//printk("wuboadd risetime = %d;holdtime = %d;falltime = %d;offtime = %d delaytime =  %d\n",
	//	risetime,holdtime,falltime,offtime,delaytime);
     if(!strncmp("red",led_cdev->name,3))
       {

           aw2013_write(0x32, 0x70|Imax);	//config mode, IMAX = 5mA	
           aw2013_write(0x35, MAX_BRIGHTNESS);	// LED1 level,
           //aw2013_write(0x37, Rise_time<<4 | Hold_time);	//led0  ÉÏÉýÊ±Œä£¬±£³ÖÊ±ŒäÉè¶š							
	       //aw2013_write(0x38, Fall_time<<4 | Off_time);	       //led0 ÏÂœµÊ±Œä£¬¹Ø±ÕÊ±ŒäÉè¶š
	       //aw2013_write(0x39, Delay_time<<4| Period_Num);   //led0  ºôÎüÑÓ³ÙÊ±Œä£¬ºôÎüÖÜÆÚÉè¶š
	       aw2013_write(0x3A, risetime<<4 | holdtime);	//led0  ÉÏÉýÊ±Œä£¬±£³ÖÊ±ŒäÉè¶š							
	       aw2013_write(0x3B, falltime<<4 | offtime);	       //led0 ÏÂœµÊ±Œä£¬¹Ø±ÕÊ±ŒäÉè¶š
	       aw2013_write(0x3C, Delay_time<<4| Period_Num);   //led0  ºôÎüÑÓ³ÙÊ±Œä£¬ºôÎüÖÜÆÚÉè¶š
           //aw2013_write(0x30, 0x02);
 
	   return 0; 
       
       }
	 if(!strncmp("green",led_cdev->name,5))
       {
           aw2013_write(0x33, 0x70|Imax);	//config mode, IMAX = 5mA	
           aw2013_write(0x36, MAX_BRIGHTNESS);	// LED2 level,
	       //aw2013_write(0x3d, Rise_time<<4 | Hold_time);   //led2  ÉÏÉýÊ±Œä£¬±£³ÖÊ±ŒäÉè¶š			   
		   //aw2013_write(0x3e, Fall_time<<4 | Off_time); 		  //led2 ÏÂœµÊ±Œä£¬¹Ø±ÕÊ±ŒäÉè¶š
		   //aw2013_write(0x3f, Delay_time<<4| Period_Num);	 //ºôÎüÑÓ³ÙÊ±Œä£¬ºôÎüÖÜÆÚÉè¶š
           aw2013_write(0x3D, risetime<<4 | holdtime);
	       aw2013_write(0x3E, falltime<<4 | offtime);
	       aw2013_write(0x3F, Delay_time<<4| Period_Num);
		   //aw2013_write(0x30, 0x04);     //led on=0x01 ledoff=0x00
		   return 0; 
       }
	 if(!strncmp("blue",led_cdev->name,4))
       {
           aw2013_write(0x31, 0x70|Imax);	//config mode, IMAX = 5mA	
	       aw2013_write(0x34, MAX_BRIGHTNESS);	// LED0 level,	
	       aw2013_write(0x37, risetime<<4 | holdtime);
	       aw2013_write(0x38, falltime<<4 | offtime);
	       aw2013_write(0x39, Delay_time<<4| Period_Num);
	       //aw2013_write(0x37, Rise_time<<4 | Hold_time);	//led0  ÉÏÉýÊ±Œä£¬±£³ÖÊ±ŒäÉè¶š							
	       //aw2013_write(0x38, Fall_time<<4 | Off_time);	       //led0 ÏÂœµÊ±Œä£¬¹Ø±ÕÊ±ŒäÉè¶š
	       //aw2013_write(0x39, Delay_time<<4| Period_Num);   //led0  ºôÎüÑÓ³ÙÊ±Œä£¬ºôÎüÖÜÆÚÉè¶š
	       //aw2013_write(0x30, 0x01);     //led on=0x01 ledoff=0x00
		   return 0; 
       }
      return 0; 
}
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct aw2013_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;


	for (addr = 0x00; addr <= 0x02; addr++) {
		rc = __aw2013_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = 0x30; addr <= 0x3F; addr++) {
		rc = __aw2013_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct aw2013_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pinctrl_init_regled_gpio22(struct aw2013_chip *chip)
{
	int retval;
	pr_info("entry pinctrl_init_regled_gpio22\n");
	chip->redledgpio_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->redledgpio_pinctrl)) {
		pr_info("wuboadd chip->redledgpio_pinctrl IS_ERR_OR_NULL \n");
		dev_dbg(chip->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(chip->redledgpio_pinctrl);
		chip->redledgpio_pinctrl = NULL;
		return retval;
	}
	#if 0
	chip->dtv_state_suspend
		= pinctrl_lookup_state(chip->redledgpio_pinctrl,
			"dtv-ldoen-gpio");
	if (IS_ERR_OR_NULL(chip->dtv_state_suspend)) {
		pr_info("wuboadd IS_ERR_OR_NULL(chip->dtv_state_suspend) \n");
		dev_dbg(chip->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(chip->dtv_state_suspend);
		//chip->redledgpio_pinctrl = NULL;
		//return retval;
	}
	#endif
	chip->gpio_state_active
		= pinctrl_lookup_state(chip->redledgpio_pinctrl,
			"redled_gpio");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		pr_info("wuboadd IS_ERR_OR_NULL(chip->gpio_state_active) \n");
		dev_dbg(chip->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(chip->gpio_state_active);
		chip->redledgpio_pinctrl = NULL;
		return retval;
	}
	return 0;
}
#if 0
static int dtv_ldoen_disable(struct aw2013_chip *chip)
{

	int err =0;
    
    chip->dtv_gpio20 = of_get_named_gpio(chip->dev->of_node, "dtvldoen,gpio20", 0);
	
	if(gpio_is_valid(chip->dtv_gpio20))
	{
	 pr_info("wuboadd chip->dtv_gpio20 :%d \n",chip->dtv_gpio20);
	}
	else
	{
	 pr_info("wuboadd add invalid chip->dtv_gpio20 \n");
	 return -1;
	}

    pr_info("wuboadd pinctrl init successed \n");
	err = pinctrl_select_state(chip->redledgpio_pinctrl,chip->dtv_state_suspend);
	   
    if (err) {
			dev_err(chip->dev,
				"can not set dtv_gpio20 low\n");
		return -1;
	}else
	{
		pr_info("wuboadd pinctrl dtv_gpio20 pinctrl_select_state successed \n");
	}

	if(gpio_is_valid(chip->dtv_gpio20)){
			
        if((err = gpio_request(chip->dtv_gpio20, "dtv-ldoen"))){
		    pr_info("dtv_gpio20 gpio_request failed\n");
	    }
		
	    if((err = gpio_direction_output(chip->dtv_gpio20, 0))){
			
		     gpio_free(chip->red_gpio22);
		     pr_info("dtv_gpio20 gpio_direction_output failed\n");
	    }
	}
	return 0;
}
#endif
static int red_gpio22_output_init(struct aw2013_chip *chip)
{
    int err = 0;
    chip->red_gpio22 = of_get_named_gpio(chip->dev->of_node, "aw2013,red_gpio20", 0);
	if(gpio_is_valid(chip->red_gpio22))
	{
	 pr_info("wuboadd chip->red_gpio22 :%d \n",chip->red_gpio22);
	}
	else
	{
	 pr_info("wuboadd add invalid chip->red_gpio22 \n");
	 return -1;
	}
	
	if(!pinctrl_init_regled_gpio22(chip))
	{
	      //	dtv_ldoen_disable(chip);
	   err = pinctrl_select_state(chip->redledgpio_pinctrl,chip->gpio_state_active);
	   

		if (err) 
		{
			dev_err(chip->dev,
				"can not set redled_gpio22 high\n");
		}else
		{
		pr_info("wuboadd pinctrl pinctrl_select_state successed \n");
		}

	    if(gpio_is_valid(chip->red_gpio22))
		{
			
            if ((err = gpio_request(chip->red_gpio22, "led_gpio22"))){
		    pr_info("led_gpio22 gpio_request failed\n");
	        }

	    /*if((err = gpio_direction_output(chip->red_gpio22, 1))){
		     gpio_free(chip->red_gpio22);
		     pr_info("led_gpio22 gpio_direction_output failed\n");
	        }*/
	   }
	}
	else		
	    return -1;

	return 0;
}

static void red_led_gpio22_output(struct aw2013_chip *chip)
{
    //static int first_output = 0;
	int err =0;
    if(first_redgpio_output ==0)
    {
	    if(gpio_is_valid(chip->red_gpio22)){
		
		if((err = gpio_direction_output(chip->red_gpio22, 1))){
		     gpio_free(chip->red_gpio22);
		     pr_info("led_gpio22 gpio_direction_output failed\n");
	       }
	   }
		first_redgpio_output = 1;
    }
}

	void smb1360_redled_gpio_output(void)
{
    int err =0;
    if(gpio_is_valid(the_chip->red_gpio22)){
		first_redgpio_output = 1;
		if((err = gpio_direction_output(the_chip->red_gpio22, 1))){
		first_redgpio_output = 0;
		gpio_free(the_chip->red_gpio22);
		pr_info("led_gpio22 gpio_direction_output failed\n");
		}
	}
}
EXPORT_SYMBOL(smb1360_redled_gpio_output);

static void parse_dt_data(struct aw2013_chip *chip)
{
    u32 max_brightness_tmp =0;
	u32 imax_tmp = 0;
	int ret = 0;
	
    ret = of_property_read_u32(chip->dev->of_node, "aw2013,maxbrightness",
						&max_brightness_tmp);
	if(ret<0)
	{
	    pr_info("read aw2013,maxbrightness failed\n");
	}
	else
	{   
	    MAX_BRIGHTNESS = (u8)max_brightness_tmp;
	    pr_info("read aw2013,maxbrightness %d\n",max_brightness_tmp);
	}
	
    ret = of_property_read_u32(chip->dev->of_node, "aw2013,Imax",&imax_tmp);
	
	if(ret<0)
	{
	    pr_info("read aw2013,Imax failed\n");
	}
    else
    {
        Imax = (u8)imax_tmp;
        pr_info("read aw2013,Imax %d\n",imax_tmp);
    }
}

static int aw2013_hw_init(struct aw2013_chip *chip)
{
    u8 chip_id=0;
	//return 0;
    if(!__aw2013_read(chip,RSTR,&chip_id))
	{
	    pr_info("aw2013_hw_init aw2013 chipid: %02x ",chip_id);
        //__aw2013_write(chip,RSTR,0x55);
		//udelay(20);
		__aw2013_write(chip,GCR, 0x01);
		__aw2013_write(chip,STATUS,0x00);
		__aw2013_write(chip,effecREG30,0x07);
		//__aw2013_write(chip,LED0_CTRL,0x02);
		//__aw2013_write(chip,LED1_CTRL,0x02);
		//__aw2013_write(chip,LED2_CTRL,0x02);
		//__aw2013_write(chip,PWM0_CTRL,0x8f);
		//__aw2013_write(chip,PWM1_CTRL,0x8f);
		//__aw2013_write(chip,PWM2_CTRL,0x8f);
		//mdelay(20);
		return chip_id;
	};
	return 0;
}

static int aw2013_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	struct aw2013_chip *chip;
	int err =0;
	pr_info("wuboadd aw2013 aw2013_probe\n");
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
    aw_i2c_client = client;
	chip->client = client;
	chip->dev = &client->dev;
	err = aw2013_hw_init(chip);
	if(!err)
	{
	pr_err("aw2013_hwinit failed return -1\n");
	return -1;
	}
	parse_dt_data(chip);
	mutex_init(&chip->read_write_lock);
	i2c_set_clientdata(client, chip);

	err = red_gpio22_output_init(chip);
    	if(err){pr_err("red_gpio22_not support\n");}

    INIT_WORK(&chip->work_red, aw2013_deferred_red_brightness_set);
    INIT_WORK(&chip->work_red_blink, aw2013_deferred_red_blink_set);
    chip->red.name= "red";
	chip->red.max_brightness = MAX_BRIGHTNESS;
	chip->red.brightness_set = aw2013_red_brightness_set;
	chip->red.blink_set  = aw2013_blink_red_set;
	
	err = led_classdev_register((struct device *)
				    &client->dev, &chip->red);
    if(err){
        pr_debug("wubo add red led register error \n");
	}
	INIT_WORK(&chip->work_green, aw2013_deferred_green_brightness_set);
	INIT_WORK(&chip->work_green_blink, aw2013_deferred_green_blink_set);
	chip->green.name= "green";
	chip->green.max_brightness = MAX_BRIGHTNESS;
	chip->green.brightness_set = aw2013_green_brightness_set;
	chip->green.blink_set  = aw2013_blink_green_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &chip->green);
    if(err){
        pr_debug("wubo add green led register error \n");
	}
	INIT_WORK(&chip->work_blue, aw2013_deferred_blue_brightness_set);
	INIT_WORK(&chip->work_blue_blink, aw2013_deferred_blue_blink_set);
	chip->blue.name= "blue";
	chip->blue.max_brightness = MAX_BRIGHTNESS;
	chip->blue.brightness_set = aw2013_blue_brightness_set;
	chip->blue.blink_set  = aw2013_blink_blue_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &chip->blue);
    if(err){
        pr_debug("wubo add blue led register error \n");
	}
	chip->debug_root = debugfs_create_dir("aw2013", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");
	
    if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				err);
    }
	chip->enable_status = 0;
	chip->pwm =0;
	chip->light =0 ;
	the_chip = chip;
/*
	err = aw2013_hw_init(chip);
	if(!err)
	{
	 pr_info("aw2013_hwinit failed\n");
	}
*/
	aw2013_breath_all(0,1,0);
	return 0;
}


static int aw2013_suspend(struct device *dev)
{
	u8 pwm0_ctrl,pwm1_ctrl,pwm2_ctrl;
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct aw2013_chip *chip = i2c_get_clientdata(client);
	printk("aw2013_suspend enter \n");
	err |= __aw2013_read(chip,PWM0_CTRL,&pwm0_ctrl);
	err |= __aw2013_read(chip,PWM1_CTRL,&pwm1_ctrl);
	err |= __aw2013_read(chip,PWM2_CTRL,&pwm2_ctrl);

	if(err != 0)
	{
		pr_err("suspend err !\n");
		return err;
	}

	if((pwm0_ctrl == 0)&&(pwm1_ctrl == 0)&&(pwm2_ctrl == 0))
	{
		err |=__aw2013_write(chip,effecREG30,0x00);
		err |=__aw2013_write(chip,GCR, 0x00);
	}

	if(err != 0)
	{
		pr_err("suspend err !\n");
		return err;
	}

	return 0;
	
}

static int aw2013_suspend_noirq(struct device *dev)
{

return 0;

}

static int aw2013_resume(struct device *dev)
{

return 0;

}

static const struct dev_pm_ops aw2013_pm_ops = {
	.resume		= aw2013_resume,
	.suspend_noirq	= aw2013_suspend_noirq,
	.suspend	= aw2013_suspend,
};

static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw,aw2013-rgb-leds",},
	{ },
};

static const struct i2c_device_id aw2013_id[] = {
	{"aw2013-rgb-leds", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);

static struct i2c_driver aw2013_driver = {
	.driver		= {
		.name		= "aw2013-rgb-leds",
		.owner		= THIS_MODULE,
		.of_match_table	= aw2013_match_table,
		.pm		= &aw2013_pm_ops,
	},
	.probe		= aw2013_probe,
	.remove		= aw2013_remove,
	.id_table	= aw2013_id,
};

module_i2c_driver(aw2013_driver);

MODULE_DESCRIPTION("AW2013 RGB LED");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:AW2013 RGB LED");
MODULE_AUTHOR("bo.wu@ck-telecom.com");
