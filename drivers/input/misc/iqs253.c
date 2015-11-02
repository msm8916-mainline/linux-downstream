/*
 * iqs253.c - Android SAR Sensor Driver for IQS253
 *
 * Copyright (C) 2013 Azoteq (Pty) Ltd
 * Author: Alwino van der Merwe <alwino.vandermerwe@azoteq.com>
 *
 * Based on mcs5000_ts.c, azoteqiqs440_ts.c, iqs263.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	Azoteq (Pty) Ltd does not take responsibility for the use of this driver
 *
 *	This driver is an example driver. It will need to be ported to
 *	the specific platform and for the specific case in which it is used.
 */
#include "iqs253.h"
#include "IQS253_Init.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

/*	Get time for the IQS253 Timeout	*/
#include <linux/time.h>

/****************************
 *	Pins used in this setup	*
 ****************************/
int iqs253_enable;   //lm 
int RDYCount;
bool eventReady;
/*	RDY Pin used on this specific setup	- needs to be changed
 *	for each case (hardware dependent)
 */
 int RDYLinePin;

/*	VDDHI used on this specific setup	- needs to be changed
 *	for each case (hardware dependent).
 *	The SAR sensor can also be powered from the voltage
 *	rail of the system, but this allows more freedom.
 */
/*	GPIO0_7 - Bank 0 Header P9-42 on BBB	*/
//#define PWR_LINE_PIN			7

/*	Numbet of Bytes to read from IQS253 continuously	*/
#define IQS253_BLOCK_SIZE		3

#define IQS253_SETUP_REG_SIZE	26

/* Boolean value used for the initial setup */
bool doInitialSetup;
bool initialATI;
/*	Indicate the a Reseed of the IQS253 will follow	*/
bool reseed;

u8 events;

/*	Boolean to keep track of Chip Reset Events	*/
bool showReset;
/*	Boolean to keep track of Prox   Events	*/
bool prox;
/*	Boolean to keep track of Touch Events	*/
bool touch;
/*int value to indicate there is a sarsensor IC or not*/
int prcOrRow;

/*	Counter to keep track of setup state	*/
u8 setupCounter;

/*	Global variable to keep the currentState in	*/
u8 currentState;	/*	Always start at state 0	*/

/*	Boolean to indicate event mode active or not	*/
bool eventMode;		/*	not activated by default	*/

/*	Each client has this additional data	*/
struct iqs253_sar_data {
	struct i2c_client *client;
	struct input_dev  *input_dev_sar;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	int pin_rdy;
	int firstBootUp;
	const struct iqs253_sar_platform_data *platform_data;
	

	struct delayed_work work;
};

/*	Struct to keep the timer SAR Timer information in	*/
static struct timer_list stuck_timer;

/*	Struct to keep a stuck timer in	*/
static struct timer_list stuck;
void stuck_time(unsigned long data);
/********************************************************
 *		The IQS253 specific setup is done here			*
 *		This setup will change for each application		*
 ********************************************************/

/*	Command for reseeding the Azoteq IQS253	*/
static int iqs253_reseed(struct iqs253_sar_data *data)
{
	int ret;
	ret = i2c_smbus_write_byte_data(data->client, PROX_SETTINGS0, RESEED);
	if (ret)
		return 1;
	return 0;
}

/*	Command for re-doing ATI on the Azoteq IQS253	*/
static int iqs253_reati(struct iqs253_sar_data *data)
{
	/*	Redo ATI	*/
	int ret;
	ret = i2c_smbus_write_byte_data(data->client, PROX_SETTINGS0, REDO_ATI);
	if (ret)
		return 1;
	return 0;
}
//liumiao
static ssize_t iqs253_set_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct iqs253_sar_data *data = i2c_get_clientdata(client);
	int ret;
	unsigned long en = 0;
	ret = kstrtoul(buf, 10, &en);
	if (ret)
		return ret;
	if(en){
		if(!iqs253_enable){
			iqs253_enable =1;
			printk("liumiao set enable to 1\n");
			enable_irq(client->irq);
			}
	}else{
		if(iqs253_enable){
			iqs253_enable =0;
			disable_irq_nosync(client->irq);
		}	
	}
	return count;
}

static ssize_t iqs253_get_judge(struct device *dev,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", prcOrRow);
}

static ssize_t iqs253_set_init(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	doInitialSetup = true;
	setupCounter = 0;
	setup_timer(&stuck, stuck_time, 0);
	mod_timer(&stuck,
	jiffies + msecs_to_jiffies(1000));
	return count;
}
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,NULL, iqs253_set_enable);
static DEVICE_ATTR(judge, S_IRUGO|S_IWUSR|S_IWGRP,iqs253_get_judge, NULL);
static DEVICE_ATTR(init, S_IRUGO|S_IWUSR|S_IWGRP,NULL, iqs253_set_init);
static struct attribute *iqs253_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_judge.attr,
	&dev_attr_init.attr,
	NULL
};

static struct attribute_group iqs253_attribute_group = {
	.attrs = iqs253_attributes
};
//liumiao
/************************************************************************
 *							IQS253 Initial Setup						*
 ************************************************************************/
/* Setup the Active Channels for Azoteq IQS253 */
static int iqs253_init_setup(struct iqs253_sar_data *data, u8 numOfRegs)
{
	/**
	 *	Setup IQS253 completely - start at ATI register and
	 *	run through all regsiters
	 */
	 //struct i2c_client *client = data->client;
	 int ret;
	
	ret = gpio_get_value(data->pin_rdy);
	 if(!ret){
		ret = i2c_smbus_write_i2c_block_data(data->client, ATI_TARGET, numOfRegs,
				iqs253_default_regs);
		if (ret){

			return 1;
			}
		if( !prcOrRow ){
			printk("iqs253 set prcOrRow to 1\n");
			prcOrRow = 1;
		}
		return 0;
	 }
	 else{
	 	printk("iqs253 %s the RDY is HIGH.Init failed!\n",__func__);
		return 1;}
	
}

/************************************************************************
 *						State Machine Helper Functions					*
 ************************************************************************/

/**
 *	Check for Events on the IQS253. ATI, Prox and Touch event. Also check
 *	the Show Reset Flag
 *	The function returns no value - instead Global flags are set to
 *	indicate the events that ocurred
 */
static int readEvents(struct iqs253_sar_data *data)
{
	u8 data_buffer[3];
	int ret;
	/*
	 *	Read the Bytes for Prox and Touch events -
	 *	save this state
	 */
	ret = gpio_get_value(data->pin_rdy);
	if(!ret){
	ret = i2c_smbus_read_i2c_block_data(data->client, DEFAULT_COMMS_POINTER_VAL,
		IQS253_BLOCK_SIZE, data_buffer);
	if(ret!= 3)
		return 1;
		printk("iqs253 the data is 0x%x 0x%x\n",data_buffer[1],data_buffer[2]);

	//printk("liumiao the CS is 0x%x 0x%x!!!the LTA is 0x%x 0x%x\n",data_buffer[7],data_buffer[8],data_buffer[9],data_buffer[10]);
	/*	We are interested in byte 0, byte 1 and byte 2	*/
	/*	These bytes will give us all of the info we need	*/
	if (data_buffer[0]&SHOW_RESET)
		showReset = true;
	else
		showReset = false;

	if (data_buffer[1]&CH_ALL_EVENT)
		prox = true;
	else
		prox = false;

	if (data_buffer[2]&CH_0_EVENT)
		touch = true;
	else
		touch = false;
	return 0;
	}
	 else{
	 	printk("iqs253%s the RDY is HIGH.Read event failed!\n",__func__);
		return 1;
	 }
}

/*************************************************************************/

/*	Platform data for the Azoteq IQS253 SAR driver	*/
struct iqs253_sar_platform_data {
	void (*cfg_pin)(void);
};

/**
 *	Because the IQS253 operates in event mode, implement a handshake
 *	function that will initiate comms with the IQS253 if we want to talk
 *	to it.
 */
void iqs253_event_mode_handshake(void)
{
	/********************************************************
	 *			Try and do an Event Mode Handshake			*
	 *******************************************************/
	/*
	 *	There might be another way to build in the delay.
	 *	Event mode handshake is done by manually pulling
	 *	the IQS253 RDY line low for ~10ms and then releasing
	 *	it. This will tell the IQS253 that the Master wants
	 *	to talk and it will then give a Communications window
	 *	which will call the interrupt request function.
	 *	From there the IQS253 can be read again
	 */
	/*	Pull RDY low	*/
	gpio_direction_output(RDYLinePin, 0);
	/*	Hold the RDY low for ~10ms	*/
	mdelay(HANDSHAKE_DELAY_HOLD);
	/*	Release RDY line	*/
	gpio_direction_input(RDYLinePin);
	/*	Delay before talking to IQS253	*/
	udelay(HANDSHAKE_DELAY_SET);
}

/**
 *	Function that gets called when an interrupt does not occur after a while
 */


/** Timer interrupt function	*/
void stuck_time(unsigned long data)
{
	del_timer(&stuck_timer);
	iqs253_event_mode_handshake();
}

void count_delay_work(struct work_struct *work) 
{
	//struct iqs253_sar_data *data = container_of((struct delayed_work *)work, struct iqs253_sar_data, work);

	if (RDYCount >= 4){
		printk("iqs253:the noise!!\n");
		eventReady = false;
		}
	else
		eventReady = true;
	
	/*if (eventReady) {
		if(prox){
			if( data->prox1 && !data->prox2 ){
				//printk(" iqs253 CH0 close\n");
				input_event(data->input_dev_sar,EV_KEY, KEY_CHAT, 1);
				input_sync(data->input_dev_sar);
				//input_event(data->input_dev_sar,EV_KEY, KEY_ALTERASE, 0);
				//input_sync(data->input_dev_sar);
				input_event(data->input_dev_sar,EV_KEY, KEY_CONNECT, 0);
				input_sync(data->input_dev_sar);
				//input_event(data->input_dev_sar,EV_KEY, KEY_FINANCE, 0);
				//input_sync(data->input_dev_sar);
				}
			else if( !data->prox1 && data->prox2 ){
				//printk(" iqs253 CH1 close\n");
				//input_event(data->input_dev_sar,EV_KEY, KEY_ALTERASE, 1);
				//input_sync(data->input_dev_sar);
				input_event(data->input_dev_sar,EV_KEY, KEY_CHAT, 0);
				input_sync(data->input_dev_sar);
				input_event(data->input_dev_sar,EV_KEY, KEY_CONNECT, 1);
				input_sync(data->input_dev_sar);
				//input_event(data->input_dev_sar,EV_KEY, KEY_FINANCE, 0);
				//input_sync(data->input_dev_sar);
				}
			else{
				//printk(" iqs253 CH0&CH1 close\n");
				input_event(data->input_dev_sar,EV_KEY, KEY_CONNECT, 1);
				input_sync(data->input_dev_sar);
				input_event(data->input_dev_sar,EV_KEY, KEY_CHAT, 1);
				input_sync(data->input_dev_sar);
				//input_event(data->input_dev_sar,EV_KEY, KEY_ALTERASE, 0);
				//input_sync(data->input_dev_sar);
				//input_event(data->input_dev_sar,EV_KEY, KEY_FINANCE, 0);
				//input_sync(data->input_dev_sar);
				}
				
		} else{
			//printk(" iqs253 away\n");
			//input_report_abs(data->input_dev_sar,EV_KEY, KEY_FINANCE, 1);
			//input_sync(data->input_dev_sar);
			input_event(data->input_dev_sar,EV_KEY, KEY_CONNECT, 0);
			input_sync(data->input_dev_sar);
			input_event(data->input_dev_sar,EV_KEY, KEY_CHAT, 0);
			input_sync(data->input_dev_sar);
			//input_event(data->input_dev_sar,EV_KEY, KEY_ALTERASE, 0);
			//input_sync(data->input_dev_sar);
		}
	}*/
	RDYCount = 0;
}
/**
 *	Interrupt event fires on the Falling edge of
 *	the RDY signal from the Azoteq IQS253
 */
static irqreturn_t iqs253_sar_interrupt(int irq, void *dev_id)
{
	struct iqs253_sar_data *data = dev_id;
	//struct i2c_client *client = data->client;
	int ret;

	/*	Do the initial setup for the IQS253
	 *	Because we have to wait for the RDY from the IQS253 and
	 *	we cannot change the way Android handles I2C comms
	 *	the setup will take a few comms cycles
	 */

	/*
	 *	Check for a reset - if reset was seen,
	 *	Then setup has to been done all over again
	 *
	 *	If we need to setup the IQS253, traverse through the
	 *	setup
	 */
	if (doInitialSetup) {
		switch (setupCounter) {
		/*	Setup ProxSettings	*/
		case 0:
			//printk(KERN_ALERT "IQS253:Initializing..\n");
			/*	Del timer that should have
			 *	made init window
			 */
			del_timer(&stuck);
			/*	Do event mode handshake	*/
			ret = iqs253_init_setup(data,IQS253_SETUP_REG_SIZE );
			if (ret){
				printk("iqs253 init error\n");
				break;}
			 printk(KERN_ALERT "IQS253:Initializing..\n");

			setupCounter++;
			break;
		case 1:
			 if (data->firstBootUp){
				ret = iqs253_reati(data);
				if (ret){
					printk("iqs253 reATI error\n");
					break;
				}
				//printk("iqs253 data->firstBootUp == 0\n");
				data->firstBootUp++;
				//disable_irq_nosync(client->irq);
				}else{
				ret = iqs253_reseed(data);
				if(ret){
					printk("iqs253 reseed error\n");
					break;}
			 }
			setupCounter = 0;
			doInitialSetup = false;
			/*
			 *	Setup timer to trigger event
			 *	for ATI
			 */
			printk(KERN_ALERT "IQS253:Re-ATI\n");
			break;
		default:
			setupCounter = 0;
			doInitialSetup = false;
			break;
		}
		goto out;
	}
	//del_timer(&stuck);

	if (reseed) {
		ret = iqs253_reseed(data);
		if(ret){
			printk("iqs253 reseed error\n");
			goto out;}
		reseed = false;
		/*	Reset State machine	*/
		/*	Reset touch and prox status	*/
		currentState = 0;
		touch = false;
		events = 0;

		goto out;
	}

	ret = readEvents(data);	/*	Check Events	*/

out:
	return IRQ_HANDLED;
}


/**
 *	The probe function is called when Android is looking
 *	for the IQS253 on the I2C bus (I2C-1)
 */
static int iqs253_sar_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct iqs253_sar_data *data;
	//struct input_dev *input_dev;
	int ret;
	struct device *dev;

	/*	Allocate memory	*/
	data = kzalloc(sizeof(struct iqs253_sar_data), GFP_KERNEL);
	//input_dev = input_allocate_device();
	if (!data ) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}
	//lm for dt
	dev = &client->dev;
	data->pin_rdy = of_get_named_gpio_flags(dev->of_node,
					"Azoteq,interrupt-gpio", 0, NULL);
	if (gpio_is_valid(data->pin_rdy)){
		RDYLinePin = data->pin_rdy;
		ret = gpio_request(data->pin_rdy,"iqs_interrupt");
		if (ret){
			dev_err(&client->dev, "Failed to register interrupt\n");
			return ret;
		}
		ret = gpio_direction_input(data->pin_rdy);
		if (ret){
			dev_err(&client->dev, "Failed to register interrupt\n");
			return ret;
		}
		/*	Request an interrup on the RDY line	*/
		client->irq = gpio_to_irq(data->pin_rdy);
	/*
	 *	Request the interrupt on a falling trigger
	 *	and only one trigger per falling edge
	 */
		ret = request_threaded_irq(client->irq, NULL, iqs253_sar_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, DEVICE_NAME, data);
		//disable_irq_nosync(client->irq);
	}
	/*	Save the stuctures to be used in the driver	*/
	data->client = client;

	data->platform_data = client->dev.platform_data;
	
	data->input_dev_sar = input_allocate_device();
	data->input_dev_sar->name = "Azoteq IQS253 SAR Sensor";
	data->input_dev_sar->id.bustype = BUS_I2C;
	//input_dev->dev.parent = &client->dev;
	//set_bit(EV_ABS, data->input_dev_sar->evbit);
	//input_set_abs_params(data->input_dev_sar,ABS_PRESSURE,0,10,0,0);
	//input_set_capability(data->input_dev_sar, EV_KEY, BTN_0);
	set_bit(EV_KEY,data->input_dev_sar->evbit);
	set_bit(KEY_CHAT, data->input_dev_sar->keybit);
	//set_bit(KEY_ALTERASE, data->input_dev_sar->keybit);
	set_bit(KEY_CONNECT, data->input_dev_sar->keybit);
	//set_bit(KEY_FINANCE, data->input_dev_sar->keybit);
	
	/*	Register the device */
	ret = input_register_device(data->input_dev_sar);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_mem;
	}

	/*	Now, assign default values to global variables	*/
	/*	Periodically check for chip reset	*/
	currentState = 0;	/*	Always start at state 0	*/
	setupCounter = 0;	/*	Start initial setup	*/
	iqs253_enable = 0;/*	Start initial setup	*/
	RDYCount = 0;
	doInitialSetup = true;
	initialATI = false;
	eventMode = false;
	touch = false; /*	Assume no touch at first	*/
	reseed = false;
	showReset = false;
	eventReady = false;
	prcOrRow = 0;
	data->firstBootUp = 0;
	//init delay work
	INIT_DELAYED_WORK(&data->work,count_delay_work); 

	/* Set i2c client data	*/
	i2c_set_clientdata(client, data);

	ret = sysfs_create_group(&client->dev.kobj, &iqs253_attribute_group);
		if (ret) 
			dev_err(&client->dev, "sysfs create failed: %d\n", ret);

	/*	Setup timer if we miss first setup window	*/
	setup_timer(&stuck, stuck_time, 0);
	mod_timer(&stuck,
		jiffies + msecs_to_jiffies(1000));

	/*	Only swith on VDDHI now	*/
	/*	Switch on VDDHI (Power)	*/
	//gpio_set_value(PWR_LINE_PIN, 1);

	return 0;

err_free_mem:
	input_free_device(data->input_dev_sar);
	kfree(data);
	return ret;
}

static int iqs253_sar_remove(struct i2c_client *client)
{
	struct iqs253_sar_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);

	input_unregister_device(data->input_dev_sar);
	kfree(data);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id iqs253_sar_id[] = {
		{ DEVICE_NAME, 0 },
	{ },
};


static const struct of_device_id iqs253_match_table[] = {
		{ .compatible ="Azoteq,iqs253", },
		{},
};

/*
 *	Standard stucture containing the driver
 *	information and procedures
 */
static struct i2c_driver iqs253_sar_driver = {
	.probe = iqs253_sar_probe,
	.remove = iqs253_sar_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = iqs253_match_table,
		},
	.id_table = iqs253_sar_id,
};

/**
 *	Gets called from 'board-omap3beagle.c'
 *	when the device I2C bus init is called
 */
static int __init iqs253_sar_init(void)
{
	/*	Add i2c driver to kernel	*/
	printk(KERN_ALERT "Installing IQS253 SAR Sensor Driver");
	return i2c_add_driver(&iqs253_sar_driver);
}

/*	Remove the driver */
static void __exit iqs253_sar_exit(void)
{
	/*	Boolean value used for the initial setup	*/
	setupCounter = 0;	/*	Start initial setup	*/
	doInitialSetup = true;
	initialATI = false;

	/*	Reset the IC - or set the proxsettings again	*/
	//gpio_set_value(PWR_LINE_PIN, 0);

	/*	Debug - Reset states	*/
	//gpio_set_value(53, 0);

	/*	Delete driver	*/
	i2c_del_driver(&iqs253_sar_driver);
	printk(KERN_ALERT "Delete: - %s\n",
			"IQS253 SAR Sensor driver Deleted! ");
}

module_init(iqs253_sar_init);
module_exit(iqs253_sar_exit);

/* Module information */
MODULE_AUTHOR("Alwino van der Merwe <alwino.vandermerwe@azoteq.com>");
MODULE_DESCRIPTION("SAR Sensor driver for Azoteq IQS253");
MODULE_LICENSE("GPL");
