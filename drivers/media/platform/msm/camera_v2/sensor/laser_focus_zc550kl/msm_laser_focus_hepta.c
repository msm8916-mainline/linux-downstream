/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_laser_focus.h"
#include "msm_laser_focus_vl6180x_def.h"
#include "sysfs/Laser_forcus_sysfs.h"
#include "msm_cci.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_gpio.h>


//DEFINE_MSM_MUTEX(msm_laser_focus_mutex);

#undef CDBG
#define CDBG(fmt, args...) pr_info(fmt, ##args)

//static struct v4l2_file_operations msm_laser_focus_v4l2_subdev_fops;
static int32_t TOF_power_up(struct msm_laser_focus_ctrl_t *a_ctrl);
static int32_t TOF_power_down(struct msm_laser_focus_ctrl_t *a_ctrl);
static int32_t TOF_match_id(struct msm_laser_focus_ctrl_t *a_ctrl);
static int TOF_init(struct msm_laser_focus_ctrl_t *a_ctrl);
static int TOF_deinit(struct msm_laser_focus_ctrl_t *a_ctrl);

struct msm_laser_focus_ctrl_t *tof_t = NULL;
bool tof_camera_on_flag = false;
struct mutex tof_mutex;
int tof_check_status = 0;

static int tof_laser_focus_enforce_ctrl = 0;

/*For LaserFocus STATUS Controll+++*/
#define	STATUS_PROC_FILE				"driver/LaserFocus_Status"
#define	STATUS_PROC_FILE_FOR_CAMERA	"driver/LaserFocus_Status_For_Camera"
#define	DEVICE_TURN_ON_FILE			"driver/LaserFocus_on"
#define	DEVICE_GET_VALUE				"driver/LaserFocus_value"
#define	DEVICE_SET_CALIBRATION			"driver/LaserFocus_CalStart"
#define	DEVICE_DUMP_REGISTER_VALUE	"driver/LaserFocus_regiser_dump"
#define	LASER_FOCUS_ENFORCE			"driver/LaserFocus_enforce"
static struct proc_dir_entry *status_proc_file;
static struct proc_dir_entry *device_trun_on_file;
static struct proc_dir_entry *device_get_value_file;
static struct proc_dir_entry *device_set_calibration_file;
static struct proc_dir_entry *dump_laser_focus_register_file;
static int ATD_status;

/* Swap high and low of the data (e.g 0x1234 => 0x3412) */
static uint16_t swap_data(uint16_t register_data){
	return ((register_data >> 8) | ((register_data & 0xff) << 8)) ;
}

static int TOF_WrDWord(uint32_t register_addr, uint16_t i2c_write_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = tof_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				swap_data(i2c_write_data), MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, swap_data(i2c_write_data));
	return status;
}

static int TOF_RdDWord(uint32_t register_addr, uint16_t *i2c_read_data)
{
	int status;
	uint16_t buf;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = tof_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
		&buf, MSM_CAMERA_I2C_WORD_DATA);
	*i2c_read_data = swap_data(buf);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

static int TOF_device_UpscaleRegInit(void)
{
	int status = 0;

	 /* Initialize TOF */
        status = TOF_WrDWord(0x0C, 0xE180);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x0E, 0x10FF);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x20, 0xA4B0);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x22, 0xA001);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x24, 0xA041);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x26, 0x0080);
        if (status < 0){
                return status;
        }
	
	return status;
} 

static int TOF_WaitDeviceBooted(void)
{
	int status = 0;

	/* Power up device */
        status = TOF_WrDWord(0x1C, 0x0061);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0X14, 0X0200);
        if (status < 0){
                return status;
        }
        status = TOF_WrDWord(0x04, 0x0092);
        if (status < 0){
                return status;
        }

	/* wait hardware booting(least 500us) */
	msleep(1);
	
	return status;
}

static ssize_t ATD_TOF_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (tof_t->device_state == val)	{
		printk("%s Setting same commond (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	switch (val) {
	case MSM_LASER_FOCUS_DEVICE_OFF:
		mutex_lock(&tof_mutex);
		if(tof_camera_on_flag){
			CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
			mutex_unlock(&tof_mutex);
			break;
		}
		rc = TOF_deinit(tof_t);
		rc = TOF_power_down(tof_t);
		tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		mutex_unlock(&tof_mutex);
		break;
	case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		if(tof_camera_on_flag){
			CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
			break;
		}
		if (tof_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
			rc = TOF_deinit(tof_t);
			rc = TOF_power_down(tof_t);
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}
		tof_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
		rc = TOF_power_up(tof_t);
		rc = TOF_init(tof_t);
		rc = TOF_WaitDeviceBooted();
		if (rc < 0)	{
			printk("%s Device trun on fail !!\n", __func__);
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			//return -EIO;
			goto DEVICE_TURN_ON_ERROR;
		} else	{
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, tof_t->device_state);
			TOF_device_UpscaleRegInit();
		}
		break;
	case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
		if(tof_camera_on_flag){
			CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
			break;
		}
		if (tof_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
			rc = TOF_deinit(tof_t);
			rc = TOF_power_down(tof_t);
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}
		tof_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
		rc = TOF_power_up(tof_t);
		rc = TOF_init(tof_t);
		rc = TOF_WaitDeviceBooted();
		if (rc < 0)	{
			printk("%s Device trun on fail !!\n", __func__);
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			//return -EIO;
			goto DEVICE_TURN_ON_ERROR;
		} else	{
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, tof_t->device_state);
			TOF_device_UpscaleRegInit();
		}
		break;
	case MSM_LASER_FOCUS_DEVICE_INIT_CCI:
		tof_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
		rc = TOF_init(tof_t);
		rc = TOF_WaitDeviceBooted();
		if (rc < 0)	{
			printk("%s Device turn on fail !!\n", __func__);
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			return -EIO;
		} else	{
			tof_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			printk("%s Init Device (%d)\n", __func__, tof_t->device_state);
			TOF_device_UpscaleRegInit();
		}

		tof_camera_on_flag = true;
		
		break;
	case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
		mutex_lock(&tof_mutex);
		rc = TOF_deinit(tof_t);
		tof_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
		tof_camera_on_flag = false;
		mutex_unlock(&tof_mutex);
		
		break;
	default:
		printk("%s commond fail !!\n", __func__);
		break;
	}
	return len;
	
DEVICE_TURN_ON_ERROR:
	rc = TOF_deinit(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_deinit failed %d\n", __func__, __LINE__);
	}
	rc = TOF_power_down(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_power_down failed %d\n", __func__, __LINE__);
	}
	return -EIO;
}

static int ATD_TOF_device_enable_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", tof_t->device_state);
	return 0;
}

static int ATD_TOF_device_enable_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_TOF_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_TOF_device_enable_open,
	.write = ATD_TOF_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_TOF_device_read_range(void)
{
	uint16_t RawRange, Data_value = 0;
	int status;

	/* Trigger single measure */
        status = TOF_WrDWord(0x04, 0x0081);
        if (status < 0){
		CDBG("%s: write register(0x04) failed \n", __func__);
              return status;
        }

	/* Wait until data ready */
        do{
			TOF_RdDWord(0x00, &Data_value);
        }while((int)(Data_value&0x10)!=0x10);

	/* Read distance */
        TOF_RdDWord(0x08, &RawRange);
	if((RawRange&0x4000)==0){
		//printk("%s: read range: %d\n", __func__,(RawRange>>2)&0x1ff);
		RawRange = (RawRange>>2)&0x1ff;
	}
       else {
                pr_err("%s: read register(0x08) failed\n", __func__);
                return -1;
        }
	
	CDBG("%s: read register(0x08) : 0x%x \n", __func__, RawRange);

	return (int)RawRange;
}


static int ATD_TOF_device_get_range_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;

	mutex_lock(&tof_mutex);

	if (tof_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		tof_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, tof_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_unlock(&tof_mutex);
		return -EBUSY;
	}

	if(tof_laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d\n", tof_laser_focus_enforce_ctrl);
		mutex_unlock(&tof_mutex);
		return 0;
	}

	RawRange = ATD_TOF_device_read_range();

	if (RawRange < 0) {
		pr_err("%s: ead_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}
	
	printk("%s : Get range (%d)  Device (%d)\n", __func__, RawRange , tof_t->device_state);

	seq_printf(buf, "%d\n", RawRange);
	
	mutex_unlock(&tof_mutex);
	
	return 0;
}
 
static int ATD_TOF_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_TOF_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_TOF_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_TOF_device_calibration_offset(void)
{
	int i = 0, RawRange = 0, sum = 0;
	uint16_t distance;
	uint16_t offset;

	mutex_lock(&tof_mutex);

	if (tof_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		tof_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, tof_t->device_state);
		mutex_unlock(&tof_mutex);
		return -EBUSY;
	}

	/* Set calibratioon Setting */
	//VL6180x_WrByte(0x0054, 0xFD);
	//VL6180x_WrByte(0x00A3, 0x3C);
	//VL6180x_WrByte(0x00B7, 0x3C);

	/* Clean system offset */
	//VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);
	//VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i=0; i<STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{
		RawRange = (int)ATD_TOF_device_read_range();
		msleep(50);
		sum += RawRange;
	}
	distance = (uint16_t)(sum / STMVL6180_RUNTIMES_OFFSET_CAL);
	printk("The measure distance is %d mm\n", distance);

	offset = (VL6180_OFFSET_CAL_RANGE - distance*3)/3;


	//VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);

	/* Write calibration file */
	tof_t->laser_focus_offset_value = offset;
	if (Laser_Forcus_sysfs_write_offset(offset) == false){
		mutex_unlock(&tof_mutex);
		return -ENOENT;
	}

	mutex_unlock(&tof_mutex);
	
	return 0;
}

static int ATD_TOF_device_calibration_crosstalkoffset(void)
{
	int i = 0, RawRange = 0;
	int rtnRate  = 0;
	int crosstalk  = 0;
	int xtalk_sum  = 0;
	uint16_t XtalkCompRate;

	mutex_lock(&tof_mutex);

	if (tof_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		tof_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, tof_t->device_state);
		mutex_unlock(&tof_mutex);
		return -EBUSY;
	}

	/* Clean crosstalk offset */
	//VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i = 0; i < STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{
		//rtnRate = (int)VL6180x_RdByte(RESULT_RANGE_SIGNAL_RATE);
		//RawRange = (int)VL6180x_RdByte(RESULT_RANGE_RAW);

		crosstalk = rtnRate * (1000- (RawRange*3000 /VL6180_CROSSTALK_CAL_RANGE));
		xtalk_sum += crosstalk;

		msleep(30);
	}
	printk("Crosstalk compensation rate is %d\n", xtalk_sum);

	XtalkCompRate = (uint16_t)((xtalk_sum /STMVL6180_RUNTIMES_OFFSET_CAL)/1000);
	printk("Crosstalk compensation rate is %d\n", XtalkCompRate);

	//VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, XtalkCompRate);

	/* Write calibration file */
	tof_t->laser_focus_cross_talk_offset_value = XtalkCompRate;
	if (Laser_Forcus_sysfs_write_cross_talk_offset(XtalkCompRate) == false){
		mutex_unlock(&tof_mutex);
		return -ENOENT;
	}

	mutex_unlock(&tof_mutex);
	
	return 0;
}
 
static ssize_t ATD_TOF_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	printk("%s commond : %d\n", __func__, val);
	switch (val) {
	case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		ret = ATD_TOF_device_calibration_offset();
		if (ret < 0)
			return ret;
		break;
	case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		ret = ATD_TOF_device_calibration_crosstalkoffset();
		if (ret < 0)
			return ret;
		break;
	default:
		printk("%s commond fail(%d) !!\n", __func__, val);
		return -EINVAL;
		break;
	}
	return len;
}

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_TOF_device_get_range_open,
	.write = ATD_TOF_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_TOF_I2C_status_check(struct msm_laser_focus_ctrl_t *s_ctrl){
	int32_t rc;

	rc = TOF_power_up(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_power_up failed %d\n", __func__, __LINE__);
		return 0;
	}
	TOF_init(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_init failed %d\n", __func__, __LINE__);
		return 0;
	}
	
	rc = TOF_match_id(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_match_id failed %d\n", __func__, __LINE__);
		rc = TOF_deinit(tof_t);
		if (rc < 0) {
			//kfree(tof_t);
			pr_err("%s TOF_deinit failed %d\n", __func__, __LINE__);
		}
		rc = TOF_power_down(tof_t);
		if (rc < 0) {
			//kfree(tof_t);
			pr_err("%s TOF_power_down failed %d\n", __func__, __LINE__);
		}
		return 0;
	}
	rc = TOF_deinit(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_deinit failed %d\n", __func__, __LINE__);
		return 0;
	}
	rc = TOF_power_down(tof_t);
	if (rc < 0) {
		//kfree(tof_t);
		pr_err("%s TOF_power_down failed %d\n", __func__, __LINE__);
		return 0;
	}

	tof_check_status = 1;

	return 1;
}

static int ATD_TOF_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = ATD_TOF_I2C_status_check(tof_t);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int ATD_TOF_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_TOF_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_TOF_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int TOF_I2C_status_check_via_prob(struct msm_laser_focus_ctrl_t *s_ctrl){
	return tof_check_status;
}

static int TOF_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = TOF_I2C_status_check_via_prob(tof_t);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int TOF_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, TOF_I2C_status_check_proc_read, NULL);
}

static const struct file_operations I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = TOF_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_TOF_register_read(struct seq_file *buf, void *v)
{
	int status, i = 0;
	uint16_t register_value = 0;

	for (i = 0; i <0x100; i++)	{
		register_value = 0;
		status = 0;//VL6180x_RdDWord(i, register_value);
	}
	seq_printf(buf, "%d\n", 0);
	return 0;
}

static int dump_TOF_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_TOF_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_TOF_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int TOF_laser_focus_enforce_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int TOF_laser_focus_enforce_open(struct inode *inode, struct  file *file)
{
	return single_open(file, TOF_laser_focus_enforce_read, NULL);
}

static ssize_t TOF_laser_focus_enforce_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char messages[8];

	if (tof_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		tof_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, tof_t->device_state);
		return -EBUSY;
	}

	if (len > 8) {
		len = 8;
	}

	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}
	
	tof_laser_focus_enforce_ctrl = (int)simple_strtol(messages, NULL, 10);
	
	return len;
}

static const struct file_operations laser_focus_enforce_fops = {
	.owner = THIS_MODULE,
	.open = TOF_laser_focus_enforce_open,
	.write = TOF_laser_focus_enforce_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static void TOF_create_proc_file(void)
{
	status_proc_file = proc_create(STATUS_PROC_FILE, 0660, NULL, &ATD_I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}
	
	device_trun_on_file = proc_create(DEVICE_TURN_ON_FILE, 0660, NULL, &ATD_laser_focus_device_enable_fops);
	if (device_trun_on_file) {
		printk("%s device_trun_on_file sucessed!\n", __func__);
	} else {
		printk("%s device_trun_on_file failed!\n", __func__);
	}

	device_get_value_file = proc_create(DEVICE_GET_VALUE, 0664, NULL, &ATD_laser_focus_device_get_range_fos);
	if (device_get_value_file) {
		printk("%s device_get_value_file sucessed!\n", __func__);
	} else {
		printk("%s device_get_value_file failed!\n", __func__);
	}

	/* now calibration method now */
	if(FALSE)	{
		device_set_calibration_file = proc_create(DEVICE_SET_CALIBRATION, 0660, NULL, &ATD_laser_focus_device_calibration_fops);
		if (device_set_calibration_file) {
			printk("%s device_set_calibration_file sucessed!\n", __func__);
		} else {
			printk("%s device_set_calibration_file failed!\n", __func__);
		}

		dump_laser_focus_register_file = proc_create(DEVICE_DUMP_REGISTER_VALUE, 0660, NULL, &dump_laser_focus_register_fops);
		if (dump_laser_focus_register_file) {
			printk("%s dump_laser_focus_register_file sucessed!\n", __func__);
		} else {
			printk("%s dump_laser_focus_register_file failed!\n", __func__);
		}
 	}

	status_proc_file = proc_create(STATUS_PROC_FILE_FOR_CAMERA, 0660, NULL, &I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}

	status_proc_file = proc_create(LASER_FOCUS_ENFORCE, 0660, NULL, &laser_focus_enforce_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}
}

int TOF_match_id(struct msm_laser_focus_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);

	
	if (chipid != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static int32_t msm_TOF_vreg_control(struct msm_laser_focus_ctrl_t *a_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_laser_focus_vreg *vreg_cfg;

	vreg_cfg = &a_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(a_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}

static int VL6180x_GPIO_High(struct msm_laser_focus_ctrl_t *a_ctrl){
	int rc = 0;
	
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	CDBG("Enter\n");

	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	
	sensordata = a_ctrl->sensordata;
	power_info = &sensordata->power_info;

	if(power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL){
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if(rc < 0){
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VDIG],
		GPIO_OUT_HIGH
	);
	
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_GPIO_Low(struct msm_laser_focus_ctrl_t *a_ctrl){
	int rc = 0;
	
	struct msm_camera_sensor_board_info *sensordata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	
	CDBG("Enter\n");

	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	
	sensordata = a_ctrl->sensordata;
	power_info = &sensordata->power_info;

	if(power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL){
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->gpio_num[SENSOR_GPIO_VDIG],
		GPIO_OUT_LOW
	);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if(rc < 0){
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	
	CDBG("Exit\n");
	return rc;
}

static int32_t TOF_power_down(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->laser_focus_state != LASER_FOCUS_POWER_DOWN) {

		rc = msm_TOF_vreg_control(a_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}

	VL6180x_GPIO_Low(a_ctrl);
	
	CDBG("Exit\n");
	return rc;
}

static int TOF_init(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	// CCI Init 
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	CDBG("Exit\n");
	return rc;
}

static int TOF_deinit(struct msm_laser_focus_ctrl_t *a_ctrl) {
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
		
	}


	CDBG("Exit\n");
	return rc;
}

static int32_t msm_TOF_get_dt_data(struct device_node *of_node,
		struct msm_laser_focus_ctrl_t *fctrl)
{
	int i = 0;
	int32_t rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint32_t id_info[3];
	struct msm_laser_focus_vreg *vreg_cfg = NULL;
	
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->sensordata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = fctrl->sensordata;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&sensordata->sensor_name);
	CDBG("%s label %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_master = MASTER_0;
		rc = 0;
	}

	if (of_find_property(of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &fctrl->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(fctrl);
			pr_err("failed rc %d\n", rc);
			return rc;
		}
	}

	sensordata->slave_info =
		kzalloc(sizeof(struct msm_camera_slave_info),
			GFP_KERNEL);
	if (!sensordata->slave_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}

	rc = of_property_read_u32_array(of_node, "qcom,slave-id",
		id_info, 3);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	fctrl->sensordata->slave_info->sensor_slave_addr = id_info[0];
	fctrl->sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	fctrl->sensordata->slave_info->sensor_id = id_info[2];

		CDBG("%s:%d slave addr 0x%x sensor reg 0x%x id 0x%x\n",
		__func__, __LINE__,
		fctrl->sensordata->slave_info->sensor_slave_addr,
		fctrl->sensordata->slave_info->sensor_id_reg_addr,
		fctrl->sensordata->slave_info->sensor_id);

	/* Handle GPIO (CAM_1V2_EN) */
	power_info = &sensordata->power_info;
	
	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if(!power_info->gpio_conf){
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	gconf = power_info->gpio_conf;
	
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if(gpio_array_size){
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size, GFP_KERNEL);
		if(!gpio_array){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf);
			rc = -ENOMEM;
			goto ERROR;
		}
		
		for(i=0; i < gpio_array_size; i++){
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i, gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf);
			goto ERROR;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_req_tbl);
			goto ERROR;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf, gpio_array, gpio_array_size);
		if(rc < 0){
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(gconf->cam_gpio_set_tbl);
			goto ERROR;
		}
	}
	kfree(gpio_array);

	return rc;

ERROR:
		kfree(fctrl->sensordata->slave_info);
	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static int32_t TOF_power_up(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = msm_TOF_vreg_control(a_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	a_ctrl->laser_focus_state = LASER_FOCUS_POWER_UP;

	VL6180x_GPIO_High(a_ctrl);

	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
	//.s_power = msm_laser_focus_power,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = "qcom,ois1", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);

static int32_t TOF_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	//uint32_t id_info[3];
	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;
	//struct msm_laser_focus_ctrl_t *tof_t = NULL;
	//struct msm_laser_focus_vreg *vreg_cfg;
	CDBG("Probe Start\n");
	ATD_status = 0;
	
	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	tof_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),
		GFP_KERNEL);
	if (!tof_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	tof_t->pdev = pdev;

	rc = msm_TOF_get_dt_data(pdev->dev.of_node, tof_t);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	/* Assign name for sub device */
	snprintf(tof_t->msm_sd.sd.name, sizeof(tof_t->msm_sd.sd.name),
			"%s", tof_t->sensordata->sensor_name);

	tof_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;
	//tof_t->laser_focus_mutex = &msm_laser_focus_mutex;
	//tof_t->cam_name = pdev->id;

	/* Set device type as platform device */
	tof_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	tof_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == tof_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}
	if (!tof_t->i2c_client->i2c_func_tbl)
		tof_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	tof_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!tof_t->i2c_client->cci_client) {
		kfree(tof_t->vreg_cfg.cam_vreg);
		kfree(tof_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	//tof_t->i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	cci_client = tof_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = tof_t->cci_master;
	if (tof_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = tof_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	v4l2_subdev_init(&tof_t->msm_sd.sd,
		tof_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&tof_t->msm_sd.sd, tof_t);
	tof_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	tof_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(tof_t->msm_sd.sd.name,
		ARRAY_SIZE(tof_t->msm_sd.sd.name), "msm_laser_focus");
	media_entity_init(&tof_t->msm_sd.sd.entity, 0, NULL, 0);
	tof_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	tof_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	tof_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&tof_t->msm_sd);
	tof_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;

	/* Init data struct */
	tof_t->laser_focus_cross_talk_offset_value = 0;
	tof_t->laser_focus_offset_value = 0;
	tof_t->laser_focus_state = MSM_LASER_FOCUS_DEVICE_OFF;

	/* Check I2C status */
	if(ATD_TOF_I2C_status_check(tof_t) == 0)
		goto probe_failure;

	ATD_status = 1;
	TOF_create_proc_file();
	CDBG("Probe Success\n");
	return 0;
probe_failure:
	CDBG("%s Probe failed\n", __func__);
	return rc;
}

static struct platform_driver msm_laser_focus_platform_driver = {
	//.probe = TOF_platform_probe,
	.driver = {
		.name = "qcom,ois1",
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int __init TOF_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	mutex_init(&tof_mutex);
	rc = platform_driver_probe(&msm_laser_focus_platform_driver,
		TOF_platform_probe);
	//rc = platform_driver_register(&msm_laser_focus_platform_driver);
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return rc;
//	return i2c_add_driver(&msm_laser_focus_i2c_driver);
}
static void __exit TOF_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	//i2c_del_driver(&msm_sensor_driver_i2c);
	return;
}

module_init(TOF_init_module);
module_exit(TOF_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
