/* drivers/input/touchscreen/NVTtouch_205.c
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 *
 * Revision : V2 (2014/10/22)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/wakelock.h>

#include <linux/input/mt.h>
//#include <mach/board.h>
//#include <mach/hardware.h>
//#include <mach/sci.h>
//#include <mach/sci_glb_regs.h>
#include "NVTtouch_205.h"
#include <linux/of_gpio.h>


#if BOOT_UPDATE_FIRMWARE
#include "NVT_firmware_205.h"
static struct workqueue_struct *nvt_fwu_wq;
#endif

struct nvt_ts_data *ts;

static struct workqueue_struct *nvt_wq;
struct nvt_platform_data *pdata;


struct sprd_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

static int sprd_3rdparty_gpio_tp_rst = 0;
static int sprd_3rdparty_gpio_tp_irq = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif 


/*******************************************************
 I2C Read & Write
*******************************************************/
static int CTP_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return ret;	
}

void CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;		
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return;
}


/*******************************************************
  IC Reset using GPIO trigger 
*******************************************************/
void nvt_hw_reset(void)
{	
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(20);
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 0);
	msleep(10);
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);	
}


/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_CTRL_DRIVER
//static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"
ssize_t nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	
	char *str;
	int ret=-1;
	int retries = 0;
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	if(copy_from_user(str, buff, count))
		return -EFAULT;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = str[1];
	msgs[0].buf   = &str[2];

	//---change sw_reset to hw_reset---
	if(str[0]==0x70)
	{
		if(str[2]==0x00 && str[3]==0x5A)
		{
			nvt_hw_reset();
			return 1;
		}
	}
	
	while(retries < 20)
	{
		ret = i2c_transfer(ts->client->adapter, msgs, 1);
		if(ret == 1)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}
	return ret;
}

ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	 
	char *str;
	int ret = -1;
	int retries = 0;
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	if(copy_from_user(str, buff, count))
		return -EFAULT;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = 1;
	msgs[0].buf   = &str[2];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = str[0];
	msgs[1].len   = str[1]-1;
	msgs[1].buf   = &str[3];

	while(retries < 20)
	{
		ret = i2c_transfer(ts->client->adapter, msgs, 2);
		if(ret == 2)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}

	// copy buff to user if i2c transfer 	
	if(retries < 20)
	{
		if(copy_to_user(buff, str, count))
			return -EFAULT;
	}
	return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if(dev == NULL)
		return -ENOMEM;

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if(dev)
		kfree(dev);
	
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};
static int nvt_flash_init(void)
{		
	//int ret=0;
 	proc_create(DEVICE_NAME,0666,NULL,&nvt_flash_fops);

	printk("============================================================\n");
	printk("NVT_flash driver loaded\n");
	printk("============================================================\n");
	return 0;
}
#endif



/*******************************************************
  Auto Update FW in Probe
*******************************************************/
#if BOOT_UPDATE_FIRMWARE
int Check_FW_Ver(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);
	dev_info(&ts->client->dev, "IC FW Ver = %d\n", I2C_Buf[1]);
	dev_info(&ts->client->dev, "Bin FW Ver = %d\n", BUFFER_DATA[0x7F00]);
	if(I2C_Buf[1]>BUFFER_DATA[0x7F00])
		return 1;
	else
		return 0;
}

int Check_CheckSum(void)
{
	uint8_t I2C_Buf[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr=0;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum, WR_Filechksum;

	WR_Filechksum = 0;

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x5A;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(1000);


	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0xE8;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 3);

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0xEA;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 2);

	addr = 0;
	for(i=0;i<(BUFFER_LENGTH)/128;i++)
	{
		for(j=0;j<16;j++)
		{
			unsigned char tmp=0;
			addrH = addr>>8;
			addrL = addr&0xFF;
			for(k=0;k<8;k++)
			{
				tmp+=BUFFER_DATA[i*128+j*8+k];
			}
			tmp = tmp+addrH+addrL+8;
			tmp = (255-tmp)+1;
			WR_Filechksum+=tmp;
			addr+=8;
		}
	}

	msleep(800);

	do
	{
		msleep(10);
		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x3F;
		I2C_Buf[2]=0xF8;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, I2C_Buf, 3);

		buf2[0]=0x00;
		buf2[1]=0x00;
		buf2[2]=0x00;
		buf2[3]=0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf2, 4);

		Retry_Counter++;
		msleep(10);

	}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

	//---------------------------------------------------------------------------------------

	if(buf2[1]==0xAA)
	{
		RD_Filechksum=(buf2[2]<<8)+buf2[3];
		if(RD_Filechksum==WR_Filechksum)
		{
			dev_info(&ts->client->dev, "%s : firmware checksum match.\n", __func__);
			return 1;	// checksum match
		}
		else
		{
			dev_info(&ts->client->dev, "%s : firmware checksum not match!!\n", __func__);
			return 0;	// checksum not match
		}
	}
	else
	{
		dev_info(&ts->client->dev, "%s : read firmware checksum timeout!!\n", __func__);
		return -1;	// read checksum failed
	}
}

void Update_Firmware(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int i = 0;
	int j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = ts->client;
	int ret;

	//-------------------------------
	// Step1 --> initial BootLoader
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(2);
  
	// Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);

	msleep(20);

	// Read status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "Program: init get status(0x%2X) error.", I2C_Buf[1]);
		return;
	}
	dev_info(&client->dev, "Program: init get status(0x%2X) success.", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 
 	//---------------------------------------------------------
	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0E;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xB4;
	I2C_Buf[6]=0x3D;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1)
	{
		msleep(1);
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)
			break;
	}

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0F;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xEF;
	I2C_Buf[6]=0x01;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);

	while(1)
	{
		msleep(1);
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
		if(I2C_Buf[1]==0xAA)
			break;
	}
	

	for (i = 0 ; i < BUFFER_LENGTH/4096 ; i++)	// 32K = 8 times
	{
		Row_Address = i * 4096; 															

		// Erase Flash	
		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x33;
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 					
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 4);
		msleep(15);	// Delay 15 ms 
			  
		// Read Erase status
		CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2); 
			  
		// check status        
		if (I2C_Buf[1] != 0xAA)
		{
			dev_info(&client->dev, "Program: erase(0x%02X) error.", I2C_Buf[1]);
			return;
		}			
	}

	dev_info(&client->dev, "Program: erase(0x%02X) success.", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to IC  
	//----------------------------------------
	dev_info(&client->dev, "Program: write begin, please wait...");

    for(j=0;j<BUFFER_LENGTH/128;j++)
	{
    	Flash_Address=(j)*128;

	   	for (i = 0 ; i < 16 ; i++, Flash_Address += 8)	// 128/8 = 16 times for One Row program
		{
    		// write bin data to IC
  			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x08;	//Flash write length (byte)
			I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
			I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
			I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
			I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
			I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
			I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
			I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
			I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8

			// Calculate a check sum by Host controller.
			// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
			//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
			//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
            	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                	      I2C_Buf[13]) + 1;
			
			// Load check sum to I2C Buffer
			I2C_Buf[5] = CheckSum[i];
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 14);
		}
		msleep(10);
		
		// Read status
		I2C_Buf[0] = 0x00;
		while(1)
		{
			CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
			if(I2C_Buf[1]==0xAA)
				break;
		}
    }

	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step4. Verify  
	//----------------------------------------
	dev_info(&client->dev, "Program: Verify begin, please wait...");
	ret=Check_CheckSum();
	if(ret==1)
		dev_info(&client->dev, "Program: Verify Pass!");
	else if(ret==0)
		dev_info(&client->dev, "Program: Verify NG!");
	else if(ret==-1)
		dev_info(&client->dev, "Program: Verify FW not return!");

	//---write i2c command to reset---			
	//I2C_Buf[0] = 0x00;
	//I2C_Buf[1] = 0x5A;
	//CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
	
	//---trigger rst-pin to reset---
	nvt_hw_reset();

	msleep(500);
	dev_info(&client->dev, "Program: END");
}

void Boot_Update_Firmware(struct work_struct *work)
{
	struct i2c_client *client = ts->client;
	int ret=0;
	
	ret = Check_CheckSum();

	nvt_hw_reset();
	msleep(500);

	if(ret==-1) // read firmware checksum failed
	{
		Update_Firmware();
	}
	else if(ret==0&&(Check_FW_Ver()==0))	// (fw checksum not match) && (bin fw version > ic fw version)
	{
		dev_info(&client->dev, "%s : firmware version not match.\n", __func__);
		Update_Firmware();
	}

}
#endif


#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
static char prox_enable=0;

static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	dev_info(&ts->client->dev, "%s\n", __func__);
	return sprintf(buf, "%d\n", prox_enable);
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	uint8_t buf1[2]={0};

	on_off = (on_off > 0) ? 1 : 0;

	if(prox_enable != on_off)
	{
		if(on_off==0)
		{
			//disable proximity
			dev_info(&ts->client->dev, "Disable touch proximity.\n");
			buf1[0]=0xA4;
			buf1[1]=0x00;
			CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf1, 2);
		}
		else
		{
			//enable proximity
			dev_info(&ts->client->dev, "Enable touch proximity.\n");
			buf1[0]=0xA4;
			buf1[1]=0x01;
			CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf1, 2);
		}

		prox_enable = on_off;
	}
	
	return size;
}

static DEVICE_ATTR(enable, 0777, proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group proximity_attr_group = {
	.attrs = proximity_attributes,
};

static void nvt_ts_proximity_report(uint8_t prox_status)
{
	switch(prox_status)
	{
		case 0x01:
			//Proximity is near
			dev_info(&ts->client->dev, "Proximity is near.\n");
			input_report_abs(ts->input_dev, ABS_DISTANCE, 0);
			input_sync(ts->input_dev);
			break;
		case 0x03:
			//Proximity is far
			dev_info(&ts->client->dev, "Proximity is far.\n");
			input_report_abs(ts->input_dev, ABS_DISTANCE, 1);
			input_sync(ts->input_dev);
			break;
	
		default:
			break;
	}
}
#endif //endif NT11004_PROXIMITY


#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

static struct wake_lock gestrue_wakelock;

static unsigned char bTouchIsAwake=1;
void nvt_ts_wakeup_gesture_report(unsigned char gesture_id)
{
	struct i2c_client *client = ts->client;
	unsigned int keycode=0;

	switch(gesture_id)
	{
		case GESTURE_WORD_C:
			dev_info(&client->dev, "Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			dev_info(&client->dev, "Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			dev_info(&client->dev, "Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			dev_info(&client->dev, "Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			dev_info(&client->dev, "Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			dev_info(&client->dev, "Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			dev_info(&client->dev, "Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			dev_info(&client->dev, "Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			dev_info(&client->dev, "Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			dev_info(&client->dev, "Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			dev_info(&client->dev, "Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			dev_info(&client->dev, "Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			dev_info(&client->dev, "Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			dev_info(&client->dev, "Wrong Gesture!! ID=%d\n", gesture_id);
			break;
	}

	if(keycode > 0)
	{
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
	msleep(250);
}
#endif


/*******************************************************
Description:
	Novatek touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static void nvt_ts_work_func(struct work_struct *work)
{
	//struct nvt_ts_data *ts = container_of(work, struct nvt_ts_data, work);
	struct i2c_client *client = ts->client;
	
	int ret = -1;
	uint8_t  point_data[ (TOUCH_MAX_FINGER_NUM*6)+2+1]={0};
	unsigned int position = 0;	
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned char input_w = 0;
	unsigned char input_id = 0;
	
	int i;
	int finger_cnt=0;
		
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data,  ts->max_touch_num*6+2+1);
	
	if(ret < 0)
	{
		dev_info(&client->dev, "%s: CTP_I2C_READ failed.\n", __func__ );
		goto XFER_ERROR;
	}

	input_id = (unsigned int)(point_data[1]>>3);

#if WAKEUP_GESTURE
	if(bTouchIsAwake == 0)
	{
		nvt_ts_wakeup_gesture_report(input_id);
		enable_irq(ts->client->irq);
		return;
	}
#endif

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(input_id == 30)
	{
		nvt_ts_proximity_report(point_data[1]&0x07);
	}
#endif

	
	for(i=0; i<ts->max_touch_num; i++)
	{
		position = 1 + 6*i;
		input_id = (unsigned int)(point_data[position+0]>>3);

		if((point_data[position]&0x07) == 0x03) // finger up (break)
		{
			continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
		else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)) //finger down (enter&moving)
		{	
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int)(point_data[position+4])+10;
			if(input_w > 255)
				input_w = 255;

			if((input_x < 0) || (input_y < 0))
				continue;
			if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
				continue;

			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_w);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id-1);

			input_mt_sync(ts->input_dev);

			finger_cnt++;
		}
	}	
	if(finger_cnt == 0)
	{
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		
		input_mt_sync(ts->input_dev);
	}
#if TOUCH_KEY_NUM > 0
	if(point_data[ts->max_touch_num*6+1]==0xF8)
	{
		for(i=0; i<TOUCH_KEY_NUM; i++)
		{
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[ts->max_touch_num*6+2]>>i)&(0x01)));	
		}
	}
	else
	{
		//In A area,not report
		if(!(((point_data[1]&0x07) == 0x01) || ((point_data[1]&0x07) == 0x02)))
		{
			for(i=0; i<TOUCH_KEY_NUM; i++)
			{
				input_report_key(ts->input_dev, touch_key_array[i], 0);	
			}
		}
	}	
#endif

	input_sync(ts->input_dev);

XFER_ERROR:
	enable_irq(ts->client->irq);
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int irq, void *dev_id)
{
	//struct nvt_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	
	#if WAKEUP_GESTURE
	if(bTouchIsAwake == 0)
	{
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
	#endif

	queue_work(nvt_wq, &ts->nvt_work);
	
	return IRQ_HANDLED;
}

static uint8_t nvt_ts_read_chipid(void)

{

	uint8_t buf[8] = {0};

	int retry=0;



	//---check NT11205 for 5 times till success---

	for(retry=5; retry>=0; retry--)

	{

		//---sw reset idle---

		buf[0]=0x00;

		buf[1]=0xA5;

		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

		msleep(100);



		//---write i2c index---

		buf[0]=0xFF;

		buf[1]=0xF0;

		buf[2]=0x00;

		CTP_I2C_WRITE(ts->client, 0x01, buf, 3);

		msleep(10);



		//---read hw chip id---

		buf[0]=0x00;

		buf[1]=0x00;

		CTP_I2C_READ(ts->client, 0x01, buf, 3);



		if(buf[1] == 5)

		{

			break;

		}

	}

	dev_info(&ts->client->dev, "ChipID = %d.\n", buf[1]);

	return buf[1];

}

static int nvt_parse_dt(struct device *dev,
                           struct nvt_platform_data *pdata)
{
    int rc;
    struct device_node *np = dev->of_node;

	printk("nvt205 in parse_dt\n");
    pdata->name = "nvt205";
    rc = of_property_read_string(np, "nvt205,name", &pdata->name);
    if (rc && (rc != -EINVAL))
    {
        dev_err(dev, "Unable to read name\n");
        return rc;
    }

	printk("nvt205in parse_dt, get reset GPIO\n");
    pdata->reset_gpio = of_get_named_gpio_flags(np, "nvt205,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        return pdata->reset_gpio;

	printk("nvt205 in parse_dt, get irq GPIO\n");
    pdata->irq_gpio = of_get_named_gpio_flags(np, "nvt205,irq-gpio",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        return pdata->irq_gpio;

	printk("nvt205 in parse_dt done, RST:%d, IRQ:%d\n", pdata->reset_gpio, pdata->irq_gpio);
    return 0;
}


/*******************************************************
Description:
	Novatek touchscreen 
 function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	//uint8_t buf[7] = {0x00,};
	struct nvt_platform_data *pdata;
	
	struct device_node *np = client->dev.of_node;
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	if (!np) {
		printk("nvttouch 205 node failed!\n");
		goto  err_check_functionality_failed;
	}
	//sprd_3rdparty_gpio_tp_irq = of_get_gpio(np, 1);
	//sprd_3rdparty_gpio_tp_rst = of_get_gpio(np, 0);


        if (client->dev.of_node)
	{
		pdata = devm_kzalloc(&client->dev,
		                     sizeof(struct nvt_platform_data ), GFP_KERNEL);
		if (!pdata)
		{
		    //dev_err(&client->dev, "Failed to allocate memory\n");
		    return -ENOMEM;
		}

		ret = nvt_parse_dt(&client->dev, pdata);
		if (ret)
		{
		    //dev_err(&client->dev, "DT parsing failed\n");
		    return ret;
		}
	}
	else
		pdata = client->dev.platform_data;

         if (!pdata)
         {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
         }


      sprd_3rdparty_gpio_tp_rst= pdata->reset_gpio;
     sprd_3rdparty_gpio_tp_irq= pdata->irq_gpio;
        /*
           printk("begin to parse dts\n");
          sprd_3rdparty_gpio_tp_irq= of_get_named_gpio_flags(np, "nvt205,irq-gpio", 0, NULL);
	  printk("sprd_3rdparty_gpio_tp_irq:%d\n",sprd_3rdparty_gpio_tp_irq);	  
           sprd_3rdparty_gpio_tp_rst = of_get_named_gpio_flags(np, "nvt205,reset-gpio",  0, NULL);
         printk("sprd_3rdparty_gpio_tp_irq:%d\n",sprd_3rdparty_gpio_tp_rst);	  
*/
	 printk("begin to request gpio\n");
	if(gpio_request(pdata->irq_gpio, "ts_rst") < 0 || \
		gpio_request(pdata->reset_gpio, "ts_irq") < 0)
	{
		printk("%s: NT11205 gpio resource is requested by some tp.\n", __func__);
		goto  err_check_functionality_failed;
	}
	
	 printk("begin to request gpio  ok \n");
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(20);
	gpio_direction_input(sprd_3rdparty_gpio_tp_irq);

         
	 printk("check i2c func\n");
	//---check i2c func.---
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_info(&client->dev, "i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

      /*
	//---check hardware i2c read for 30 times till success---
	for(retry=30; retry>=0; retry--)
	{
		ret = CTP_I2C_READ(client, I2C_HW_Address, buf, 5);
		
		if(ret <= 0)	// i2c read failed
		{
			dev_info(&client->dev, "i2c read test failed at retry=%d.\n", retry);
		}
		else	// i2c read succeed
		{
			dev_info(&client->dev, "nt11205 i2c hardware test succeed.\n");
			break;
		}

		if(retry == 0)
			goto err_i2c_failed;	
	}
*/

     
        	//---check NT11205---

	if(nvt_ts_read_chipid() == 5)
	{
	dev_info(&ts->client->dev, "This IC is NT11205.\n");
	}

	else
	{
	dev_info(&ts->client->dev, "This IC is not NT11205.\n");

	ret = -ENODEV;
          goto err_check_functionality_failed;
	}

          printk("create workqueue\n");
	//---create workqueue---
	nvt_wq = create_workqueue("nvt_wq");
	if(!nvt_wq)
	{
		printk("%s : nvt_wq create workqueue failed.\n", __func__);
		return -ENOMEM;	
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);

             printk("allocate input device\n");   
	//---allocate input device---
	ts->input_dev = input_allocate_device();

	   printk("allocate input device ----222\n");
	if(ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		dev_info(&client->dev, "allocate input device failed.\n");
		goto err_input_dev_alloc_failed;
	}
	
	  printk("allocate input device ----333\n");
	  
#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if(!nvt_fwu_wq)
	{
		printk("%s : nvt_fwu_wq create workqueue failed.\n", __func__);
		return -ENOMEM; 
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(4000));
#endif


	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

	#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
	#endif

	  printk("allocate input device ----444\n");
	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);    //pressure = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	input_set_abs_params(ts->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
#endif

#if TOUCH_KEY_NUM > 0
	for(retry = 0; retry < TOUCH_KEY_NUM; retry++)
	{
		input_set_capability(ts->input_dev, EV_KEY,touch_key_array[retry]);	
	}
#endif

	  printk("allocate input device ----555\n");

#if WAKEUP_GESTURE
	for(retry = 0; retry < (sizeof(gesture_key_array)/sizeof(gesture_key_array[0])); retry++)
	{
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);	
	}
	
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0205;
	ts->input_dev->id.product = 0x0001;


	  printk("allocate input device ----666\n");
	//---register input device---	
	ret = input_register_device(ts->input_dev);
	if(ret)
	{
		dev_info(&client->dev, "register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	/*create device group in sysfs as user interface */
	ret = sysfs_create_group(&ts->input_dev->dev.kobj, &proximity_attr_group);
	if (ret) {
		printk("%s : sysfs_create_group() error, ret = %d\n",__func__,  ret);
		ret = -EINVAL;
		goto err_input_register_device_failed;
	}
#endif

	
	printk("allocate input device ----777\n");

	client->irq=gpio_to_irq(sprd_3rdparty_gpio_tp_irq);
	if(client->irq)
	{
		ret = request_irq(client->irq, nvt_ts_irq_handler, IRQ_TYPE_EDGE_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT, client->name, ts);

		if (ret != 0)
		{
			dev_info(&client->dev, "request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		}
		else 
		{
			disable_irq(client->irq);
			dev_info(&client->dev, "request irq %d succeed.\n", client->irq);
		}
	}
	nvt_hw_reset();

	//---set device node---
	#if NVT_TOUCH_CTRL_DRIVER
	ret = nvt_flash_init();
	if (ret != 0)
	{
		dev_info(&client->dev, "nvt_flash_init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
	#endif

	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nvt_ts_early_suspend;
	ts->early_suspend.resume = nvt_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	dev_info(&client->dev, "%s finished.\n", __func__);
	enable_irq(client->irq);
	 printk("probe sucess\n");
	return 0;


err_init_NVT_ts:
	free_irq(client->irq,ts);	
err_int_request_failed:
err_input_register_device_failed:
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
//err_i2c_failed:
err_check_functionality_failed:	
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_ts_remove(struct i2c_client *client)
{
	//struct nvt_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	dev_notice(&client->dev, "removing driver...\n");

	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

static int nvt_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	uint8_t buf[4]={0};

printk("in nvt_ts_suspend\n");
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(prox_enable)
		return 0;
#endif

#if WAKEUP_GESTURE
	bTouchIsAwake = 0;
	dev_info(&ts->client->dev, "Enable touch wakeup gesture.\n");

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA6;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
	enable_irq_wake(client->irq);
	irq_set_irq_type(client->irq, IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
#else
	disable_irq(client->irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA5;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
#endif

	msleep(50);
	printk("nvt_ts_suspend ok\n");
	return 0;
}

static int nvt_ts_resume(struct i2c_client *client)
{

  printk("in nvt_ts_resume\n");
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(prox_enable)
		return 0;
#endif
	
#if WAKEUP_GESTURE
	irq_set_irq_type(client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
	nvt_hw_reset();
	bTouchIsAwake = 1;
#else
	nvt_hw_reset();
	enable_irq(client->irq);
#endif
         printk(" nvt_ts_resume ok\n");
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	//struct nvt_ts_data *ts;
	//ts = container_of(h, struct nvt_ts_data, early_suspend);
	
	nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void nvt_ts_late_resume(struct early_suspend *h)
{
	//struct nvt_ts_data *ts;
	//ts = container_of(h, struct nvt_ts_data, early_suspend);
	
	nvt_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvt_ts_id);

static const struct of_device_id nvt_of_match[] = {
       { .compatible = "sprd,NVTouch_205", },
       { }
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.suspend	= nvt_ts_suspend,
	.resume		= nvt_ts_resume,
	.id_table	= nvt_ts_id,
	.driver	= {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = nvt_of_match,//of_match_ptr(nvt_of_match),
	},
};


/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __init nvt_driver_init(void)
{
	int ret = -1;
	 printk("begin to init\n");
	ret = i2c_add_driver(&nvt_i2c_driver);
	if(ret) {
		printk("i2c_add_driver failed!\n");
		return ret;
	}
	 printk("exit init\n");
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);
	if(nvt_wq)
		destroy_workqueue(nvt_wq);

	#if BOOT_UPDATE_FIRMWARE
	if(nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
	#endif
}

late_initcall(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
