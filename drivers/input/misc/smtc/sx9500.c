/*! \file sx9500.c
 * \brief  SX9500 Driver
 *
 * Driver for the SX9500 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Change log:(include other .c, .h releated to sx9500 driver, under folling directory)
 *		    kernel/include/linux/input/smtc/misc/
 *		    kernel/driver/input/misc/smtc/
 *  [140607]
 *	Fix unused function,		    [//Riven fix unused function]
 *	Fix missing earlysuspend feature    [//Riven fix earlysuspend]
 *
 *  [140611]
 *	Add sx9500-specifics.h into code    [//Riven Add platform data]
 *
 *  [140616]
 *	Add OF (Devices tree) support in
 *	order to get interrupt gpio irq	    [//Riven Add OF support]
 *
 *  [140617]
 *	Add sysfs entry for user program to pass PID into driver.
 *	So when driver get an interrupt will use this PID to pass
 *	SIGnal to user program then send QMI SAR command to 
 *	modem.				    [//Riven Add sysfs user_pid]
 *
 *  [140618]
 *	Add alarm function to send SIGnal   [//Riven Add alarm function]
 *	to user program
 *
 *  [140623]
 *	Re-enabling threaded irq	    [//Riven Enable threaded_irq]
 *
 *  [140626]
 *	Change HW default setting	    [//Riven change default HW setting]
 *	    1. Enable all IRQ  -> only closing and far away
 *	    2. Enable CS 3, 2  -> Enable CS 1, 0
 *
 *  [140818]
 *	Disable input event		    [//Riven disable input event]
 *
 *  [140901]
 *	Add status reg sysfs entry	    [//Riven Add sysfs read_status]
 *
 */
 /* Riven add SAR Driver */
//#define DEBUG
#define DRIVER_NAME "sx9500"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>	    //Riven Add platform data
/* Riven Add alarm function START */
#include <linux/signal.h>   //siginfo
#include <linux/sched.h>    //find_task_by_pid_type
#include <linux/rcupdate.h> //rcu_read_lock
/* Riven Add alarm function END */
/* Riven Add OF support START */
#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif
/* Riven Add OF support END */

#include <linux/input/smtc/misc/sx86xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx9500_i2c_reg.h>
#include <linux/input/smtc/misc/sx9500_platform_data.h>  /* platform data */

#define IDLE 0
#define ACTIVE 1



/* Riven Add platform data START */
    /* Folling data origin from sx9500-specifics.h */

/* IO Used for NIRQ */
#define GPIO_SX9500_NIRQ 114

//#define SX9500_NIRQ OMAP_GPIO_IRQ(GPIO_SX9500_NIRQ)
#include <linux/input/smtc/misc/sx9500_platform_data.h>
#include <linux/input/smtc/misc/sx9500_i2c_reg.h>

/* Riven Add OF support START - change function parm. to accept pdata->irq_gpio from sx9500,irq-gpio*/
#ifdef CONFIG_OF
static int sx9500_get_nirq_state(int nirq)
{
    return !gpio_get_value(nirq);
}
#else
static int sx9500_get_nirq_state(void)
{
    return !gpio_get_value(GPIO_SX9500_NIRQ);
}
#endif

#ifdef CONFIG_OF
static inline void __init sx9500_platform_init(int nirq)
{
    if ((gpio_request(nirq, "SX9500_NIRQ") == 0) &&
	    (gpio_direction_input(nirq) == 0)) {
	gpio_export(nirq, 0);
	printk(KERN_ERR "obtained gpio for SX9500_NIRQ\n");
    } else {
	printk(KERN_ERR "could not obtain gpio for SX9500_NIRQ\n");
	return;
    }
}
#else
static inline void __init sx9500_platform_init(void)
{
    if ((gpio_request(GPIO_SX9500_NIRQ, "SX9500_NIRQ") == 0) &&
	    (gpio_direction_input(GPIO_SX9500_NIRQ) == 0)) {
	gpio_export(GPIO_SX9500_NIRQ, 0);
	printk(KERN_ERR "obtained gpio for SX9500_NIRQ\n");
    } else {
	printk(KERN_ERR "could not obtain gpio for SX9500_NIRQ\n");	
	return;
    }
}
#endif
/* Riven Add OF support END */

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
    {
	.reg = SX9500_IRQ_ENABLE_REG,
	.val = 0x68,	//0xFF, Only get IRQ when close and farir //Riven change default HW setting
    },
    {
	.reg = SX9500_CPS_CTRL1_REG,
	.val = 0x43,	//0x43,
    },
    {
	.reg = SX9500_CPS_CTRL2_REG,
	.val = 0x77,	//0x77,
    },
    {
	.reg = SX9500_CPS_CTRL3_REG,
	.val = 0x01,	//0x01,
    },
    {
	.reg = SX9500_CPS_CTRL4_REG,
	.val = 0x80,    //0x20,
    },
    {
	.reg = SX9500_CPS_CTRL5_REG,
	.val = 0x0F,    //0x16,
    },
    {
	.reg = SX9500_CPS_CTRL6_REG,
	.val = 0x04,    //0x04,
    },
    {
	.reg = SX9500_CPS_CTRL7_REG,
	.val = 0x00,    //0x40,
    },
    {
	.reg = SX9500_CPS_CTRL8_REG,
	.val = 0x00,    //0x00,
    },
    {
	.reg = SX9500_CPS_CTRL0_REG,
	.val = 0x53,	//0x2C,	Enalbe CS0,1 //Riven change default HW setting
    },
};

static struct _buttonInfo psmtcButtons[] = {
    {
	.keycode = KEY_0,
	.mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG,
    },
    {
	.keycode = KEY_1,
	.mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG,
    },
    {
	.keycode = KEY_2,
	.mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG,
    },
    {
	.keycode = KEY_3,
	.mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG,
    },
};

static struct _totalButtonInformation smtcButtonInformation = {
    .buttons = psmtcButtons,
    .buttonSize = ARRAY_SIZE(psmtcButtons),
};

static sx9500_platform_data_t sx9500_config = {
    /* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
    .get_is_nirq_low = sx9500_get_nirq_state,
    /*  pointer to an initializer function. Here in case needed in the future */
    //.init_platform_hw = sx9500_init_ts,
    .init_platform_hw = NULL,
    /*  pointer to an exit function. Here in case needed in the future */
    //.exit_platform_hw = sx9500_exit_ts,
    .exit_platform_hw = NULL,
    
    .pi2c_reg = sx9500_i2c_reg_setup,
    .i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup),
    
    .pbuttonInformation = &smtcButtonInformation,
};
/* Riven Add platform data END */

/* Riven Add OF support START - getting OF data */
#ifdef CONFIG_OF
static int sx9500_parse_dt(struct device *dev, psx86XX_t pdata){
    struct device_node *np = dev->of_node;
    if(np == NULL){
	dev_err(dev, "No device OF node found\n");
	return -EINVAL;
    }

    if((pdata->irq_gpio = of_get_named_gpio(np, "sx9500,irq-gpio", 0)) < 0){
	dev_err(dev, "Must assign an gpio to get interrupt %d\n", pdata->irq_gpio);
	return -EINVAL;
    }
    else{
	dev_info(dev, "Get irq_gpio from OF node sx9500,irq-gpio = %d\n", pdata->irq_gpio);
    }

    return 0;
}
#else
static int sx9500_parse_dt(struct device *dev, struct mxt_platform_data *pdata){
    dev_err(dev, "Not support OF\n");
    return -ENODEV;
}
#endif
/* Riven Add OF support END */


/*! \struct sx9500
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9500
{
    pbuttonInformation_t pbuttonInformation;
    psx9500_platform_data_t hw; /* specific platform data settings */
} sx9500_t, *psx9500_t;


/*! \fn static int write_register(psx86XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx86XX_t this, u8 address, u8 value)
{
    struct i2c_client *i2c = 0;
    char buffer[2];
    int returnValue = 0;
    buffer[0] = address;
    buffer[1] = value;
    returnValue = -ENOMEM;
    if (this && this->bus) {
	i2c = this->bus;
	
	returnValue = i2c_master_send(i2c,buffer,2);
	dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
		address,value,returnValue);
    }
    return returnValue;
}

/*! \fn static int read_register(psx86XX_t this, u8 address, u8 *value) 
 * \brief Reads a register's value from the device
 * \param this Pointer to main parent struct 
 * \param address 8-Bit address to read from
 * \param value Pointer to 8-bit value to save register value to 
 * \return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int read_register(psx86XX_t this, u8 address, u8 *value)
{
    struct i2c_client *i2c = 0;
    s32 returnValue = 0;
    if (this && value && this->bus) {
	i2c = this->bus;
	returnValue = i2c_smbus_read_byte_data(i2c,address);
	dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
	if (returnValue >= 0) {
	    *value = returnValue;
	    return 0;
	} else {
	    return returnValue;
	}
    }
    return -ENOMEM;
}
/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct 
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
/* Riven fix unused function
static int write_registerEx(psx86XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
    struct i2c_client *i2c = 0;
    u8 tx[MAX_WRITE_ARRAY_SIZE];
    int ret = 0;
    
    if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
    {
	dev_dbg(this->pdev, "inside write_registerEx()\n");
	tx[0] = reg;
	dev_dbg(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
		(void *)i2c,tx[0]);
	for (ret = 0; ret < size; ret++)
	{
	    tx[ret+1] = data[ret];
	    dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
	}
	dev_dbg(this->pdev, "\n");
	
	ret = i2c_master_send(i2c, tx, size+1 );
	if (ret < 0)
	    dev_err(this->pdev, "I2C write error\n");
    }
    dev_dbg(this->pdev, "leaving write_registerEx()\n");
    
    
    return ret;
}
*/
/*! \brief Reads a group of registers from the device
 * \param this Pointer to main parent struct 
 * \param reg 8-Bit address to read from (base address)
 * \param data Pointer to 8-bit value array to save registers to 
 * \param size size of array
 * \return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
/* Riven fix unused function
static int read_registerEx(psx86XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
    struct i2c_client *i2c = 0;
    int ret = 0;
    u8 tx[] = {
	reg
    };
    if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
    {
	dev_dbg(this->pdev, "inside read_registerEx()\n");
	dev_dbg(this->pdev,
		"going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
		(void *)i2c,(void *)tx,tx[0]);
	ret = i2c_master_send(i2c,tx,1);
	if (ret >= 0) {
	    dev_dbg(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
		    (void *)i2c,(void *)data,size);
	    ret = i2c_master_recv(i2c, data, size);
	}
    }
    if (unlikely(ret < 0))
	dev_err(this->pdev, "I2C read error\n");
    dev_dbg(this->pdev, "leaving read_registerEx()\n");
    return ret;
}
*/
/*********************************************************************/
/*! \brief Perform a manual offset calibration
 * \param this Pointer to main parent struct 
 * \return Value return value from the write register
 */
static int manual_offset_calibration(psx86XX_t this)
{
    s32 returnValue = 0;
    returnValue = write_register(this,SX9500_IRQSTAT_REG,0xFF);
    return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
    u8 reg_value = 0;
    psx86XX_t this = dev_get_drvdata(dev);
    
    dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
    read_register(this,SX9500_IRQSTAT_REG,&reg_value);
    return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
    psx86XX_t this = dev_get_drvdata(dev);
    unsigned long val;
    if (strict_strtoul(buf, 0, &val))
	return -EINVAL;
    if (val) {
	dev_info( this->pdev, "Performing manual_offset_calibration()\n");
	manual_offset_calibration(this);
    }
    return count;
}

static DEVICE_ATTR(calibrate, 0664, manual_offset_calibration_show,
				    manual_offset_calibration_store);

/* Riven Add sysfs user_pid START */
static ssize_t sx9500_user_pid_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
    psx86XX_t this = dev_get_drvdata(dev);
    if(this->user_pid_enable){
	dev_dbg(this->pdev, "Reading user program PID\n");
	return sprintf(buf, "%d\n", this->user_pid);
    }
    else{
	dev_err(this->pdev, "Not get user_pid from sysfs yet\n");
	return sprintf(buf, "%d\n", -1);
    }
}
static ssize_t sx9500_user_pid_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
    psx86XX_t this = dev_get_drvdata(dev);
    int val;
    if (sscanf(buf, "%d", &val) <= 0)
	return -EINVAL;
    this->user_pid = val;
    if(val > 0 )
	this->user_pid_enable = 1;
    else
	this->user_pid_enable = 0;
    dev_info( this->pdev, "Set user program PID = %d\n", val);
    return count;
}
static DEVICE_ATTR(user_pid, 0664, sx9500_user_pid_show,
	                            sx9500_user_pid_store);
/* Riven Add sysfs user_pid END */

/* Riven Add sysfs read_status START */
static ssize_t sx9500_read_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    psx86XX_t this = dev_get_drvdata(dev);
    u8 data;
    int rc;

    dev_dbg(this->pdev, "Reading SX9500_TCHCMPSTAT_REG\n");
    rc = read_register(this, SX9500_TCHCMPSTAT_REG, &data);
    if(rc < 0){
	dev_err(this->pdev, "ERROR: reading SX9500_TCHCMPSTAT_REG(%d), rc = %d\n", SX9500_TCHCMPSTAT_REG, rc);
	return sprintf(buf, "%d\n", rc);
    }
    else{
	dev_info( this->pdev, "Get status reg[0x%02x] = 0x%02x\n", SX9500_TCHCMPSTAT_REG, data);
	return sprintf(buf, "0x%02x\n", data);
    }
}
static DEVICE_ATTR(read_status, 0444, sx9500_read_status_show, NULL);
/* Riven Add sysfs read_status END */

/* Riven Add sysfs read_reg write_reg_address write_reg_data START */
    /* read_reg*/
static ssize_t sx9500_read_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
    psx86XX_t this = dev_get_drvdata(dev);
    long reg;

    if (kstrtol(buf, 16, &reg)){
	this->gRead_reg_ok = 0;
	return -EINVAL;
    }
    this->gRead_reg_ok = 1;
    this->gRead_reg_address = (u8)reg;
    dev_dbg(this->pdev, "Read to read reg[0x%02x]\n", (u8)reg);
    return count;
}
static ssize_t sx9500_read_reg_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{

    psx86XX_t this = dev_get_drvdata(dev);
    int rc;
    u8 data;

    if(!this->gRead_reg_ok)
	return -EINVAL;

    rc = read_register(this, this->gRead_reg_address, &data);
    if(rc <0){
	dev_err(this->pdev, "ERROR: reading 0x%02x(%d), rc = %d\n", this->gRead_reg_address, this->gRead_reg_address, rc);
	return sprintf(buf, "%d\n", rc);
    }
    else{
	dev_info( this->pdev, "Get reg[0x%02x] = 0x%02x\n", this->gRead_reg_address, data);
	return sprintf(buf, "reg[0x%02x] = 0x%02x\n", this->gRead_reg_address, data);
    }
}
    /* write reg address */
static ssize_t sx9500_write_reg_address_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
    psx86XX_t this = dev_get_drvdata(dev);
    long reg;

    if (kstrtol(buf, 16, &reg)){
	this->gWrite_reg_ok = 0;
	return -EINVAL;
    }
    this->gWrite_reg_ok = 1;
    this->gWrite_reg_address = (u8)reg;
    dev_dbg(this->pdev, "Read to write reg[0x%02x]\n", (u8)reg);
    return count;
}
static ssize_t sx9500_write_reg_address_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
    psx86XX_t this = dev_get_drvdata(dev);
    if(!this->gWrite_reg_ok)
	return -EINVAL;
    return sprintf(buf, "0x%02x\n", this->gWrite_reg_address);
}

    /* write reg data */
static ssize_t sx9500_write_reg_data_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
    psx86XX_t this = dev_get_drvdata(dev);
    long data;
    int rc;

    if (kstrtol(buf, 16, &data))
	return -EINVAL;

    this->gWrite_reg_data = (u8)data;
    rc = write_register(this, this->gWrite_reg_address, this->gWrite_reg_data);
    if(rc < 0){
	dev_err(this->pdev, "ERROR: writing reg[0x%02x] = 0x%02x, rc = %d\n", this->gWrite_reg_address, this->gWrite_reg_data, rc);
	return rc;
    }
    else{
	dev_info( this->pdev, "Set reg[0x%02x] = 0x%02x\n", this->gWrite_reg_address, this->gWrite_reg_data);
	return count;
    }
    return count;
}
static ssize_t sx9500_write_reg_data_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
    psx86XX_t this = dev_get_drvdata(dev);
    return sprintf(buf, "0x%02x\n", this->gWrite_reg_data);
}


static DEVICE_ATTR(read_reg	    , 0644, sx9500_read_reg_show	    , sx9500_read_reg_store);
static DEVICE_ATTR(write_reg_data   , 0644, sx9500_write_reg_data_show	    , sx9500_write_reg_data_store);
static DEVICE_ATTR(write_reg_address, 0644, sx9500_write_reg_address_show   , sx9500_write_reg_address_store);
/* Riven Add sysfs read_reg write_reg_address write_reg_data END */

static struct attribute *sx9500_attributes[] = {
    &dev_attr_calibrate.attr,
    &dev_attr_user_pid.attr,	//Riven Add sysfs user_pid
    &dev_attr_read_status.attr,	//Riven Add sysfs read_status
    &dev_attr_read_reg.attr,
    &dev_attr_write_reg_data.attr,
    &dev_attr_write_reg_address.attr,
    NULL,
};
static struct attribute_group sx9500_attr_group = {
    .attrs = sx9500_attributes,
};
/*********************************************************************/





/*! \fn static int read_regStat(psx86XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx86XX_t this)
{
    u8 data = 0;
    if (this) {
	if (read_register(this,SX9500_IRQSTAT_REG,&data) == 0)
	    return (data & 0x00FF);
    }
    return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx86XX_t this)
{
    psx9500_t pDevice = 0;
    psx9500_platform_data_t pdata = 0;
    int i = 0;
    /* configure device */
    dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
    if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
    {
	while ( i < pdata->i2c_reg_num) {
	    /* Write all registers/values contained in i2c_reg */
	    dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
		    pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
    //      msleep(3);        
	    write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
	    i++;
	}
    } else {
	dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
    }
}
/*********************************************************************/




/*! \fn static int initialize(psx86XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx86XX_t this)
{
    if (this) {
	/* prepare reset by disabling any irq handling */
	this->irq_disabled = 1;
	disable_irq(this->irq);
	/* perform a reset */
	write_register(this,SX9500_SOFTRESET_REG,SX9500_SOFTRESET);
	/* wait until the reset has finished by monitoring NIRQ */
	dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
	/* just sleep for awhile instead of using a loop with reading irq status */
	msleep(300);
//	while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
/* Riven Add OF support */
#ifdef CONFIG_OF
	dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low(this->irq_gpio));
#else
	dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
#endif
	hw_init(this);
	msleep(100); /* make sure everything is running */
	manual_offset_calibration(this);
	
	/* re-enable interrupt handling */
	enable_irq(this->irq);
	this->irq_disabled = 0;
	
	/* make sure no interrupts are pending since enabling irq will only
	 * work on next falling edge */
	read_regStat(this);
/* Riven Add OF support */
#ifdef CONFIG_OF
	dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low(this->irq_gpio));
#else
	dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
#endif
	return 0;
    }
    return -ENOMEM;
}

/* Riven Add alarm function START */
#define SIG_0_TOUCHED   47
#define SIG_1_TOUCHED   48
#define SIG_2_TOUCHED   49
#define SIG_3_TOUCHED	50
#define SIG_0_RELEASED  51
#define SIG_1_RELEASED  52
#define SIG_2_RELEASED  53
#define SIG_3_RELEASED  54

static int sx9500_send_SIGNAL_to_user_program(psx86XX_t this,int pad, int touched)
{
    int ret, signal;
    struct siginfo info;
    struct task_struct *t;

    if(!this->user_pid_enable){
	dev_err(this->pdev, "Not get PID from sysfs yet!\n");
	return -EINVAL;
    }
    rcu_read_lock();
    if((t = find_task_by_vpid(this->user_pid)) == NULL){
	dev_err(this->pdev, "No such PID(%d)\n", this->user_pid);
	rcu_read_unlock();
	return -ENODEV;
    }
    rcu_read_unlock();

    memset(&info, 0, sizeof(struct siginfo));
    switch(pad){
	case 0:
	    if(touched)
		signal = SIG_0_TOUCHED;
	    else
		signal = SIG_0_RELEASED;
	    break;
	case 1:
	    if(touched)
		signal = SIG_1_TOUCHED;
	    else
		signal = SIG_1_RELEASED;
	    break;
	case 2:
	    if(touched)
		signal = SIG_2_TOUCHED;
	    else
		signal = SIG_3_RELEASED;
	    break;
	case 3:
	    if(touched)
		signal = SIG_3_TOUCHED;
	    else
		signal = SIG_3_RELEASED;
	    break;	    
	default:
	    return -EINVAL;
	    break;
    }
    info.si_signo = signal;
    info.si_code = SI_KERNEL;
    info.si_int = 7;	    //This value might not been pass to user SIGNAL handler. or change si_code to SI_QUEUE

    ret = send_sig_info(signal, &info, t);//send signal to user land 
    return ret;
}
/* Riven Add alarm function END */

/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
 */
static void touchProcess(psx86XX_t this)
{
    int counter = 0;
    u8 i = 0;
    int numberOfButtons = 0;
    psx9500_t pDevice = NULL;
    struct _buttonInfo *buttons = NULL;
//    struct input_dev *input = NULL;						    //Riven disable input event
    
    struct _buttonInfo *pCurrentButton  = NULL;
    
    
    if (this && (pDevice = this->pDevice))
    {
	dev_dbg(this->pdev, "Inside touchProcess()\n");
	read_register(this, SX9500_TCHCMPSTAT_REG, &i);
	
	buttons = pDevice->pbuttonInformation->buttons;
    //	input = pDevice->pbuttonInformation->input;				    //Riven disable input event
	numberOfButtons = pDevice->pbuttonInformation->buttonSize;
	
	if (unlikely( (buttons==NULL) /* || (input==NULL) */ )) {
	    dev_err(this->pdev, "ERROR!! buttons " /*or input*/ "NULL!!!\n");	    //Riven disable input event
	    return;
	}
	
	for (counter = 0; counter < numberOfButtons; counter++) {
	    pCurrentButton = &buttons[counter];
	    if (pCurrentButton==NULL) {
		dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n",
			counter);
		return; // ERRORR!!!!
	    }
	    switch (pCurrentButton->state) {
		case IDLE: /* Button is not being touched! */
		    if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
			/* User pressed button */
			sx9500_send_SIGNAL_to_user_program(this, counter, 1);	//Riven Add alarm function
			dev_info(this->pdev, "cap button %d touched\n", counter);
    //			input_report_key(input, pCurrentButton->keycode, 1);	    //Riven disable input event
			pCurrentButton->state = ACTIVE;
		    } else {
			dev_dbg(this->pdev, "Button %d already released.\n",counter);
		    }
		    break;
		case ACTIVE: /* Button is being touched! */ 
		    if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
			/* User released button */
			sx9500_send_SIGNAL_to_user_program(this, counter, 0);	//Riven Add alarm function
			dev_info(this->pdev, "cap button %d released\n",counter);
    //			input_report_key(input, pCurrentButton->keycode, 0);	    //Riven disable input event
			pCurrentButton->state = IDLE;
		    } else {
			dev_dbg(this->pdev, "Button %d still touched.\n",counter);
		    }
		    break;
		default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
		    break;
	    };
	}
    //	input_sync(input);							    //Riven disable input event
	
	dev_dbg(this->pdev, "Leaving touchProcess()\n");
    }
}
/*! \fn static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i = 0, rc;
    psx86XX_t this = 0;
    psx9500_t pDevice = 0;
    psx9500_platform_data_t pplatData = 0;
//    struct input_dev *input = NULL;						    //Riven disable input event
    
    dev_info(&client->dev, "sx9500_probe()\n");

#ifdef CONFIG_OF
    client->dev.platform_data = &sx9500_config;	/* Riven Add platform data */
#endif

    pplatData = client->dev.platform_data;
    if (!pplatData) {
	dev_err(&client->dev, "platform data is required!\n");	
	return -EINVAL;
    }
    
    if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_READ_WORD_DATA))
	return -EIO;
    
    
    this = kzalloc(sizeof(sx86XX_t), GFP_KERNEL); /* create memory for main struct */
    dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%p\n",this);
    
/* Riven Add OF support START */
#ifdef CONFIG_OF
    /* Check pdata exist or not */
    if(!this){
	dev_err(&client->dev, "Could not kzalloc fot sx86XX_t\n");
	return -ENOMEM;
    }
    /* Get irq_gpio from dt file. */
    if((rc = sx9500_parse_dt(&client->dev, this)) != 0){
	dev_err(&client->dev, "Parsing dt error %d\n", rc);
	kfree(this);
	return -EINVAL;
    }
    /* Registing gpio */
    sx9500_platform_init(this->irq_gpio);
#endif
/* Riven Add OF support END */

    if (this)
    {
	/* In case we need to reinitialize data 
	 * (e.q. if suspend reset device) */
	this->init = initialize;
	/* shortcut to read status of interrupt */
	this->refreshStatus = read_regStat;
	/* pointer to function from platform data to get pendown 
	 * (1->NIRQ=0, 0->NIRQ=1) */
	this->get_nirq_low = pplatData->get_is_nirq_low;
	/* save irq in case we need to reference it */
	this->irq = client->irq;
	/* do we need to create an irq timer after interrupt ? */
	this->useIrqTimer = 0;
	
	/* Setup function to call on corresponding reg irq source bit */
	if (MAX_NUM_STATUS_BITS>= 8)
	{
	    this->statusFunc[0] = 0; /* TXEN_STAT */
	    this->statusFunc[1] = 0; /* UNUSED */
	    this->statusFunc[2] = 0; /* UNUSED */
	    this->statusFunc[3] = 0; /* CONV_STAT */
	    this->statusFunc[4] = 0; /* COMP_STAT */
	    this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
	    this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
	    this->statusFunc[7] = 0; /* RESET_STAT */
	}
	
	/* setup i2c communication */
	this->bus = client;
	i2c_set_clientdata(client, this);
	
	/* record device struct */
	this->pdev = &client->dev;
	
	/* create memory for device specific struct */
	this->pDevice = pDevice = kzalloc(sizeof(sx9500_t), GFP_KERNEL);
	dev_dbg(&client->dev, "\t Initialized Device Specific Memory: 0x%p\n",pDevice);
	
	if (pDevice)
	{

	    this->gRead_reg_ok		= 0;
	    this->gRead_reg_address	= 0;
	    this->gWrite_reg_ok		= 0;
	    this->gWrite_reg_address	= 0;
	    this->gWrite_reg_data	= 0;
	    /* for accessing items in user data (e.g. calibrate) */
	    if((rc = sysfs_create_group(&client->dev.kobj, &sx9500_attr_group)) != 0){
		dev_err(&client->dev, "\t Initialized sysfs enter faild %d\n", rc);
	    }
	    
	    
	    /* Check if we hava a platform initialization function to call*/
	    if (pplatData->init_platform_hw)
		pplatData->init_platform_hw();
	    
	    /* Add Pointer to main platform data struct */
	    pDevice->hw = pplatData;
	    
	    /* Initialize the button information initialized with keycodes */
	    pDevice->pbuttonInformation = pplatData->pbuttonInformation;
	    
	    /* Create the input device */
    /*										    //Riven disable input event
	    input = input_allocate_device();
	    if (!input) {
		return -ENOMEM;
	    }
    */
	    /* Set all the keycodes */
	    //__set_bit(EV_KEY, input->evbit);					    //Riven disable input event
	    for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
    //		__set_bit(pDevice->pbuttonInformation->buttons[i].keycode, input->keybit);  //Riven disable input event
		pDevice->pbuttonInformation->buttons[i].state = IDLE;
	    }
	    /* save the input pointer and finish initialization */
    /*										    //Riven disable input event
    	    pDevice->pbuttonInformation->input = input;
    	    input->name = "SX9500 Cap Touch";
    	    input->id.bustype = BUS_I2C;
	    input->id.product = sx863x->product;
	    input->id.version = sx863x->version;
    	    if(input_register_device(input))
    		return -ENOMEM;
    */
	    
	    this->user_pid_enable = 0;	    //Riven Add sysfs user_pid
	    
	    
	    
	}
	sx86XX_init(this);
	return  0;
    }
    return -ENOMEM;
}

/*! \fn static int sx9500_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx86XX_remove()
 */
static int sx9500_remove(struct i2c_client *client)
{
    psx9500_platform_data_t pplatData =0;
    psx9500_t pDevice = 0;
    psx86XX_t this = i2c_get_clientdata(client);
    if (this && (pDevice = this->pDevice))
    {
    //	input_unregister_device(pDevice->pbuttonInformation->input);		    //Riven disable input event
	
	sysfs_remove_group(&client->dev.kobj, &sx9500_attr_group);
	pplatData = client->dev.platform_data;
	if (pplatData && pplatData->exit_platform_hw)
	    pplatData->exit_platform_hw();
	kfree(this->pDevice);
    }
    return sx86XX_remove(this);
}

/*====================================================*/
#if defined(USE_KERNEL_SUSPEND)	//Riven fix unused function
/***** Kernel Suspend *****/
static int sx9500_suspend(struct i2c_client *client, pm_message_t mesg)
{
    psx86XX_t this = i2c_get_clientdata(client);
    sx86XX_suspend(this);
    return 0;
}
/***** Kernel Resume *****/
static int sx9500_resume(struct i2c_client *client)
{
    psx86XX_t this = i2c_get_clientdata(client);
    sx86XX_resume(this);
    return 0;
}
#endif
/*====================================================*/
/* Riven Add OF support START*/
#ifdef CONFIG_OF
static struct of_device_id semtech_sx9500_table[] = {
    { .compatible = "semtech,sx9500",},
    { },
};
#endif
/* Riven Add OF support END*/
static struct i2c_device_id sx9500_idtable[] = {
    { DRIVER_NAME, 28 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sx9500_idtable);
static struct i2c_driver sx9500_driver = {
    .driver = {
#ifdef CONFIG_OF    /* Riven Add OF support */
	.of_match_table = semtech_sx9500_table,
#endif
	.owner  = THIS_MODULE,
	.name   = DRIVER_NAME
    },
    .id_table = sx9500_idtable,
    .probe	  = sx9500_probe,
    .remove	  = sx9500_remove,
#if defined(USE_KERNEL_SUSPEND)
    .suspend	= sx9500_suspend,
    .resume	= sx9500_resume,
#endif
};
static int __init sx9500_init(void)
{
    return i2c_add_driver(&sx9500_driver);
}
static void __exit sx9500_exit(void)
{
    i2c_del_driver(&sx9500_driver);
}

module_init(sx9500_init);
module_exit(sx9500_exit);
//module_i2c_driver(sx9500_driver);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9500 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
