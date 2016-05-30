/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.c
 * FUNCTION     : Driver source for RPR0521,
 *              : Proximity Sensor(PS) and Ambient Light Sensor(ALS) IC.
 * AUTHOR       : Masafumi Seike
 * PROGRAMMED   : Sensor System Development Group, ROHM CO.,LTD.
 * MODIFICATION : Modified by ROHM, JUN/24/2014
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2014 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/async.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <mach/board_lge.h>

#define MODULE_TAG "<rpr0521>"
#include "bs_log.h"
#include "rohm_rpr0521_i2c.h"
#include "rohm_rpr0521_i2c_config.h"
#include "rohm_rpr0521_i2c_if.h"

/* ssoon.lee@lge.com */
#define PS_CALIBRATION_PATH "/sns/prox_calibration.dat"
#define PS_THRESHOLD_MAX (4095)
#define PS_THRESHOLD_MIN (0)
#define ALS_THRESHOLD_MAX (65535)
#define ALS_THRESHOLD_MIN (0)
#define DEFAULT_CROSS_TALK (9)
#define ALSCALC_OFFSET (1000)
#define ALS_MAX_VALUE (43000)
#define ALSCALC_GAIN_MASK (0xF << 2)
#define GAIN_THRESHOLD (1000)

/******************************* define *******************************/
/* structure of peculiarity to use by system */

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct sensor_dt_to_pdata_map {
	const char *dt_name;
	void *ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type type;
	int default_val;
};

typedef struct {
	u32	pulse;
	u32	mode;
	u32	measure_time;
	u32	led_current;
	u32	als_gain;
	u32	infrared_level;
	u32	ps_gain;
	u32	persistence;
	u32	near_offset;
	u32	far_offset;
    INIT_ARG init_data;
} PS_ALS_CONFIG;
#endif

typedef struct {
    struct i2c_client   *client;     /* structure pointer for i2c bus            */
    int                 use_irq;     /* flag of whether to use interrupt or not  */
    struct hrtimer      timer;       /* structure for timer handler              */
    struct work_struct  work;        /* structure for work queue                 */
    struct input_dev    *input_dev;  /* structure pointer for input device       */
    struct input_dev    *input_dev_als;  /* structure pointer for input device       */
    struct delayed_work input_work;  /* structure for work queue                 */
    int                 delay_time;  /* delay time to set from application       */
    int                 fopen_cnt;   /* open count */

	/* ssoon.lee@lge.com */
	unsigned char		enable;		/* ps sensor on/off status*/
	unsigned char		als_enable;		/* als sensor on/off status*/
    unsigned short 		ps_data;	/* data value of PS data from sensor        */
    unsigned short 		als_data0;	/* data value of ALS data0 from sensor      */
    unsigned short 		als_data1;	/* data value of ALS data1 from sensor      */
	unsigned short		cross_talk;		/* cross_talk data for calibration		*/
	unsigned short 		default_cross_talk;
	unsigned short		adjusted_psth_upper;
	unsigned short		adjusted_psth_low;
	unsigned short		alsth_upper;
	unsigned short		alsth_low;
	bool				ps_value;			/* near=true, far=false			*/
	int					lux_value;
	int 				boot_mode;
	bool				is_interrupt;
	bool				is_timer;
	bool				is_calibrated;
	PS_ALS_CONFIG		config;
	struct mutex		enable_lock;
	struct mutex		work_lock;
	struct wake_lock 	ps_wlock;
} PS_ALS_DATA;

/* logical functions */
static int					make_init_data(PS_ALS_DATA *ps_als);
static int 					write_calibration_data_to_fs(unsigned short cal_data);
static int 					read_calibration_data_from_fs(void);
static int                  get_from_device(DEVICE_VAL *calc_data, struct i2c_client *client);
//static int                  long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus);
//static int                  calculate_als_data(READ_DATA_BUF data, DEVICE_VAL dev_val, struct i2c_client *client);
//static int                  calculate_ps_data(READ_DATA_BUF data, DEVICE_VAL dev_val, struct i2c_client *client);
static int 					get_boot_mode(void);
static int 					lux_calculation(void);
static int 					get_alsgain( struct i2c_client *client, unsigned char *raw_gain, unsigned char *gain);
static int 					set_alsgain( struct i2c_client *client, unsigned char raw_gain, signed long lux);
static int 					update_ps_value(unsigned short new_ps_data);
static int 					update_als_value(unsigned short new_als_data0, unsigned short new_als_data1);
static void                 ps_als_work_func(struct work_struct *work);
static enum hrtimer_restart ps_als_timer_func(struct hrtimer *timer);
static irqreturn_t          ps_als_irq_handler(int irq, void *dev_id);
static int                  ps_als_iodev_open(struct inode *inode, struct file *file);
static int                  ps_als_iodev_release(struct inode *inode, struct file *file);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int                  ps_als_iodev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#else
static long                 ps_als_iodev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif
static int                  ps_als_ioctl_power_on_off(PS_ALS_DATA *ps_als, POWERON_ARG *power_data);
static int                  ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  ps_als_remove(struct i2c_client *client);
static int __devinit        ps_als_init(void);
static void __exit          ps_als_exit(void);
/* access functions */
static int ps_als_driver_init(INIT_ARG data, struct i2c_client *client);
static int ps_als_driver_shutdown(struct i2c_client *client);
static int ps_als_driver_reset(struct i2c_client *client);
static int ps_als_driver_power_on_off(POWERON_ARG data, struct i2c_client *client);
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client);
static int ps_als_driver_read_data(READ_DATA_BUF *data, struct i2c_client *client);
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client);
static int ps_als_driver_write_measurement_time(unsigned char data, struct i2c_client *client);
static int ps_als_driver_write_persistence(unsigned char data, struct i2c_client *client);
static int ps_als_driver_write_interrupt_mode(unsigned char mode, struct i2c_client *client);
static int ps_als_driver_write_ps_th_h(unsigned short data, struct i2c_client *client);
static int ps_als_driver_write_ps_th_l(unsigned short data, struct i2c_client *client);
static int ps_als_driver_write_als_th_up(unsigned short data, struct i2c_client *client);
static int ps_als_driver_write_als_th_low(unsigned short data, struct i2c_client *client);
static int ps_als_driver_write_alsps_control(unsigned char data, struct i2c_client *client);
static int ps_als_driver_read_alsps_control(unsigned char* data, struct i2c_client *client);

/**************************** variable declaration ****************************/
static const char              rpr0521_driver_ver[] = RPR0521_DRIVER_VER;
static struct workqueue_struct *rohm_workqueue;
static PS_ALS_DATA             *ps_als_ginfo;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id ps_als_id[] = {
    { RPR0521_I2C_NAME, 0 }, /* rohm rpr0521 driver */
    { }
};

/* represent an I2C device driver */
static struct i2c_driver rpr0521_driver = {
    .driver = {                      /* device driver model driver */
        .name = RPR0521_I2C_NAME,
    },
    .probe    = ps_als_probe,        /* callback for device binding */
    .remove   = ps_als_remove,       /* callback for device unbinding */
    .id_table = ps_als_id,           /* list of I2C devices supported by this driver */
};

/* structure of file operation */
static struct file_operations rpr0521_fops = {
    .owner   = THIS_MODULE,          /* module of local file system */
    .open    = ps_als_iodev_open,    /* called function when application call open */
    .release = ps_als_iodev_release, /* called function when application call close */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
    .ioctl   = ps_als_iodev_ioctl,   /* called function when application call ioctl */
#else
    .unlocked_ioctl = ps_als_iodev_ioctl,   /* called function when application call ioctl */
#endif
};

/* structure of misc device */
static struct miscdevice rpr0521_device = {
    .minor = MISC_DYNAMIC_MINOR,     /* the minor number being registered */
    .name  = "rpr0521_iodev",         /* registered name to kernel */
    .fops  = &rpr0521_fops,           /* registered file operation to kernel */
};

/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
    short ALS;
    short PS;
} MCTL_TABLE[MODE_CTL_FACTOR] = {
    {  0,   0},   /*  0 */
    {  0,  10},   /*  1 */
    {  0,  40},   /*  2 */
    {  0, 100},   /*  3 */
    {  0, 400},   /*  4 */
    {100,  50},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {100,   0},   /*  8 */
    {100, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    { 50,  50},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    unsigned char DATA0;
    unsigned char DATA1;
} GAIN_TABLE[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  0,   0},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  0,   0},   /*  6 */
    {  0,   0},   /*  7 */
    {  0,   0},   /*  8 */
    {  0,   0},   /*  9 */
    { 64,  64},   /* 10 */
    {  0,   0},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};

/************************* sysfs ************************/
static ssize_t rpr0521_show_cross_talk_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", read_calibration_data_from_fs());
}

static ssize_t rpr0521_show_run_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->is_calibrated==true? 1:0);
}

static ssize_t rpr0521_store_run_calibration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	READ_DATA_BUF read_data_buf;
	unsigned short temp = 0;
	unsigned short cross_talk = 0;
	unsigned short cross_talk_array[20] = {0,};
	int i, j, result = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->is_calibrated = false;

	if(val != 1){
		PINFO("run_calibration failed by invalid store value\n");
		return -1;
	}

	for(i=0 ; i<20 ; i++){
		result = ps_als_driver_read_data(&read_data_buf, ps_als_ginfo->client);
		if (result < 0) {
			PINFO("ERROR! read data\n");
			return result;
		}
		cross_talk_array[i] = read_data_buf.ps_data;
		mdelay(5);
	}

	// sorting
	for(i=0 ; i<19 ; i++){
		for(j=i+1 ; j<20 ; j++){
			if(cross_talk_array[i] > cross_talk_array[j]){
				temp = cross_talk_array[i];
				cross_talk_array[i] = cross_talk_array[j];
				cross_talk_array[j] = temp;
			}
		}
	}

	// assign cross_talk value to middle value of cross_talk_array
	for(i=5 ; i<15 ; i++)
	{
		cross_talk += cross_talk_array[i];
	}
	cross_talk = (unsigned short)(cross_talk/10);
	PINFO("Calibration success. cross-talk value is %d", cross_talk);

	write_calibration_data_to_fs(cross_talk);
	ps_als_ginfo->cross_talk = cross_talk;
	ps_als_ginfo->is_calibrated = true;

	return count;
}

static ssize_t rpr0521_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!strcmp(dev->kobj.name, "lge_proximity")){
		return sprintf(buf, "%d\n", ps_als_ginfo->enable);
	}
	else{	// case of dev->kobj.name is "lge_light"
		return sprintf(buf, "%d\n", ps_als_ginfo->als_enable);
	}
}

static ssize_t rpr0521_store_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool is_proximity = false;
	bool is_light = false;
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int result = 0;
	POWERON_ARG parg = {0,};
	PWR_ST		pwr_st = {0,};

	if(!strcmp(dev->kobj.name, "lge_proximity")){
		is_proximity = true;
	}
	else {
		is_light = true;
	}

	//mutex_lock(&ps_als_ginfo->enable_lock);
	/* read power state */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("ps_als_driver_read_power_state fail");
		goto unlock;
	}

	/* check the stop system is */
	if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
		result = ps_als_driver_init(ps_als_ginfo->config.init_data, ps_als_ginfo->client);
		if (result < 0) {
			PINFO("ps_als_driver_init fail. error value = 0x%x\n", result);
			goto unlock;
		}
	}

	parg.power_als = pwr_st.als_state;
	parg.power_ps = pwr_st.ps_state;
	parg.intr = MODE_PROXIMITY;

	if(val == 1){
		if(is_proximity){
			parg.power_ps = PS_ALS_ENABLE;
			PINFO("change proximity sensor power state to (%ld)", val);
			if(pwr_st.ps_state == CTL_STANDALONE){
				PINFO("proximity sensor is already enabled");
				goto unlock;
			}
		}
		else if(is_light){
			parg.power_als = PS_ALS_ENABLE;
			PINFO("change light sensor power state to (%ld)", val);
			if(pwr_st.als_state == CTL_STANDALONE){
				PINFO("light sensor is already enabled");
				goto unlock;
			}
		}
	}
	else if(val == 0){
		if(is_proximity){
			parg.power_ps = PS_ALS_DISABLE;
			PINFO("change proximity sensor power state to (%ld)", val);
			if(pwr_st.ps_state == CTL_STANDBY){
				PINFO("proximity sensor is already disabled");
				goto unlock;
			}
		}
		else if(is_light){
			parg.power_als = PS_ALS_DISABLE;
			PINFO("change light sensor power state to (%ld)", val);
			if(pwr_st.als_state == CTL_STANDBY){
				PINFO("light sensor is already disabled");
				goto unlock;
			}
		}
	}
	else {
		PINFO("store value is invalid = %ld", val);
		goto unlock;
	}

	result = ps_als_driver_power_on_off(parg, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("proximity sensor power on/off failed, err = %d", result);
		goto unlock;
	}

	if(val==1){
		if(is_proximity){
			ps_als_ginfo->cross_talk = read_calibration_data_from_fs();

			// initialize current state to far for the setting of initial threshold
			update_ps_value(PS_THRESHOLD_MIN);
			input_report_abs(ps_als_ginfo->input_dev, ABS_DISTANCE, (ps_als_ginfo->ps_value==true)? 0:1);
			input_sync(ps_als_ginfo->input_dev);
			ps_als_ginfo->enable = PS_ALS_ENABLE;
		}
		else if(is_light){
			/* start timer of 1 second */
			result = hrtimer_start(&ps_als_ginfo->timer, ktime_set(0, 125000000), HRTIMER_MODE_REL);
			if (result != 0) {
				PINFO("can't start timer\n");
				goto unlock;
			}
			ps_als_ginfo->als_enable = PS_ALS_ENABLE;
		}
	}
	else{
		if(is_proximity)
			ps_als_ginfo->enable = PS_ALS_DISABLE;
		else if(is_light)
			ps_als_ginfo->als_enable = PS_ALS_DISABLE;
	}

	//mutex_unlock(&ps_als_ginfo->enable_lock);
	if(is_proximity)
		PINFO("rpr0521 proximity sensor power state change success.");
	else if(is_light)
		PINFO("rpr0521 light sensor power state change success.");
	return count;

unlock:
	//mutex_unlock(&ps_als_ginfo->enable_lock);
	if(is_proximity)
		PINFO("rpr0521 proximity sensor power state change fail.");
	else if(is_light)
		PINFO("rpr0521 light sensor power state change fail.");
	return result;
}

static ssize_t rpr0521_show_pdata(struct device *dev, struct device_attribute *attr, char *buf)
{
	int           result;
	READ_DATA_BUF read_data_buf;
	PWR_ST        pwr_st;
	unsigned char read_intr;
	GENREAD_ARG   gene_data;

	read_data_buf.ps_data   = 0;
	read_data_buf.als_data0 = 0;
	read_data_buf.als_data1 = 0;

	/* clear interrupt flag */
	gene_data.adr_reg = REG_INTERRUPT;
	gene_data.addr    = (unsigned char *)&read_intr;
	gene_data.size    = sizeof(read_intr);
	result = ps_als_driver_general_read(gene_data, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("general read can't execute \n");
		PINFO("can't read interrupt register \n");
		return result;
	}

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("power_state error \n");
		return result;
	}

	/* check the state of sensor */
	if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
        return result;
    }
    result = ps_als_driver_read_data(&read_data_buf, ps_als_ginfo->client);
    if (result < 0) {
        PINFO("ERROR! read data\n");
        return result;
    }
	ps_als_ginfo->ps_data = read_data_buf.ps_data;

	return sprintf(buf, "%d\n", ps_als_ginfo->ps_data);
}

static ssize_t rpr0521_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ((ps_als_ginfo->ps_value)==true) ? 1 : 0);
}

static ssize_t rpr0521_show_near_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->config.near_offset);
}

static ssize_t rpr0521_store_near_offset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->config.near_offset = val;
	make_init_data(ps_als_ginfo);
	return count;
}

static ssize_t rpr0521_show_far_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->config.far_offset);
}

static ssize_t rpr0521_store_far_offset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->config.far_offset= val;
	make_init_data(ps_als_ginfo);
	return count;
}

static ssize_t rpr0521_show_ps_gain(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->config.ps_gain);
}

static ssize_t rpr0521_store_ps_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->config.ps_gain = val;
	make_init_data(ps_als_ginfo);
	return count;
}

static ssize_t rpr0521_show_led_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->config.led_current);
}

static ssize_t rpr0521_store_led_current(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->config.led_current = val;
	make_init_data(ps_als_ginfo);
	return count;
}

static ssize_t rpr0521_show_infrared_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ps_als_ginfo->config.infrared_level);
}

static ssize_t rpr0521_store_infrared_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	ps_als_ginfo->config.infrared_level = val;
	make_init_data(ps_als_ginfo);
	return count;
}

static ssize_t rpr0521_show_ch0data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int           result;
	READ_DATA_BUF read_data_buf;
	PWR_ST        pwr_st;
	unsigned char read_intr;
	GENREAD_ARG   gene_data;

	read_data_buf.ps_data   = 0;
	read_data_buf.als_data0 = 0;
	read_data_buf.als_data1 = 0;

	/* clear interrupt flag */
	gene_data.adr_reg = REG_INTERRUPT;
	gene_data.addr    = (unsigned char *)&read_intr;
	gene_data.size    = sizeof(read_intr);
	result = ps_als_driver_general_read(gene_data, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("general read can't execute \n");
		PINFO("can't read interrupt register \n");
		return result;
	}

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("power_state error \n");
		return result;
	}

	/* check the state of sensor */
	if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
        return result;
    }
    result = ps_als_driver_read_data(&read_data_buf, ps_als_ginfo->client);
    if (result < 0) {
        PINFO("ERROR! read data\n");
        return result;
    }
	ps_als_ginfo->als_data0 = read_data_buf.als_data0;

	return sprintf(buf, "%d\n", ps_als_ginfo->als_data0);
}

static ssize_t rpr0521_show_ch1data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int           result;
	READ_DATA_BUF read_data_buf;
	PWR_ST        pwr_st;
	unsigned char read_intr;
	GENREAD_ARG   gene_data;

	read_data_buf.ps_data   = 0;
	read_data_buf.als_data0 = 0;
	read_data_buf.als_data1 = 0;

	/* clear interrupt flag */
	gene_data.adr_reg = REG_INTERRUPT;
	gene_data.addr    = (unsigned char *)&read_intr;
	gene_data.size    = sizeof(read_intr);
	result = ps_als_driver_general_read(gene_data, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("general read can't execute \n");
		PINFO("can't read interrupt register \n");
		return result;
	}

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als_ginfo->client);
	if (result < 0) {
		PINFO("power_state error \n");
		return result;
	}

	/* check the state of sensor */
	if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
        return result;
    }
    result = ps_als_driver_read_data(&read_data_buf, ps_als_ginfo->client);
    if (result < 0) {
        PINFO("ERROR! read data\n");
        return result;
    }
	ps_als_ginfo->als_data1 = read_data_buf.als_data1;

	return sprintf(buf, "%d\n", ps_als_ginfo->als_data1);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_enable, rpr0521_store_enable);
static DEVICE_ATTR(pdata, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_pdata, NULL);
static DEVICE_ATTR(value, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_value, NULL);
static DEVICE_ATTR(prox_cal_data, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_cross_talk_value, NULL);
static DEVICE_ATTR(run_calibration, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_run_calibration, rpr0521_store_run_calibration);
static DEVICE_ATTR(near_offset, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_near_offset, rpr0521_store_near_offset);
static DEVICE_ATTR(far_offset, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_far_offset, rpr0521_store_far_offset);
static DEVICE_ATTR(ps_gain, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_ps_gain, rpr0521_store_ps_gain);
static DEVICE_ATTR(led_current, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_led_current, rpr0521_store_led_current);
static DEVICE_ATTR(infrared_level, S_IRUGO | S_IWUSR | S_IWGRP , rpr0521_show_infrared_level, rpr0521_store_infrared_level);
static DEVICE_ATTR(ch0data, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_ch0data, NULL);
static DEVICE_ATTR(ch1data, S_IRUGO | S_IWUSR | S_IWGRP ,rpr0521_show_ch1data, NULL);

static struct attribute *ps_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_pdata.attr,
	&dev_attr_value.attr,
	&dev_attr_prox_cal_data.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_near_offset.attr,
	&dev_attr_far_offset.attr,
	&dev_attr_ps_gain.attr,
	&dev_attr_led_current.attr,
	&dev_attr_infrared_level.attr,
	NULL
};

static struct attribute_group ps_attr_group = {
    .attrs = ps_attributes,
};

static struct attribute *als_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_ps_gain.attr,
	&dev_attr_led_current.attr,
	&dev_attr_infrared_level.attr,
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
	NULL
};

static struct attribute_group als_attr_group = {
    .attrs = als_attributes,
};

/************************************************************
 *                      logic function                      *
 ************************************************************/
/******************************************************************************
 * NAME       : sensor_parse_dt
 * FUNCTION   : parse device tree
 * REMARKS    : ssoon.lee@lge.com
 *****************************************************************************/
#ifdef CONFIG_OF
static int sensor_parse_dt(struct device *dev, PS_ALS_DATA *ps_als)
{
	struct device_node *np = dev->of_node;

	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"Rohm,pulse", &ps_als->config.pulse, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,mode", &ps_als->config.mode, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,measure_time", &ps_als->config.measure_time, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,led_current", &ps_als->config.led_current, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,als_gain", &ps_als->config.als_gain, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,infrared_level", &ps_als->config.infrared_level, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,ps_gain", &ps_als->config.ps_gain, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,persistence", &ps_als->config.persistence, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,near_offset", &ps_als->config.near_offset, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,far_offset", &ps_als->config.far_offset, DT_SUGGESTED, DT_U32, 0},
		{"Rohm,default_cross_talk", &ps_als->default_cross_talk, DT_SUGGESTED, DT_U32, 0},
		{NULL, NULL, 0, 0, 0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(np, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(np, itr->dt_name, (u32 *)itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) = of_property_read_bool(np, itr->dt_name);
			ret = 0;
			break;
		default:
			PINFO("%d is an unknown DT entry type", itr->type);
			ret = -EBADE;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				PINFO("Missing '%s' DT entry", itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	return err;
}
#endif

/******************************************************************************
 * NAME       : write_calibration_data_to_fs
 * FUNCTION   : write cross-talk value to file system
 * REMARKS    : ssoon.lee@lge.com
 *****************************************************************************/
static int write_calibration_data_to_fs(unsigned short cal_data)
{
	int fd;
	int ret = 0;
	char buf[50];
	mm_segment_t old_fs = get_fs();
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "%d", cal_data);

	set_fs(KERNEL_DS);
	fd = sys_open(PS_CALIBRATION_PATH, O_WRONLY | O_CREAT, 0664);

	if (fd >= 0) {
		sys_write(fd, buf, sizeof(buf));
		sys_fsync(fd); /*ensure calibration data write to file system*/
		sys_close(fd);
		sys_chmod(PS_CALIBRATION_PATH, 0664);
		set_fs(old_fs);
	} else {
		ret++;
		sys_close(fd);
		set_fs(old_fs);
		return ret;
	}
	return ret;
}

/****************************************************************************
 * NAME       : read_calibration_data_from_fs
 * FUNCTION   : read cross-talk value from file system
 * REMARKS    : ssoon.lee@lge.com
 *****************************************************************************/
static int read_calibration_data_from_fs()
{
	int fd;
	int ret = 0;
	int len = 0;
	char read_buf[50];
	mm_segment_t old_fs = get_fs();
	memset(read_buf, 0, sizeof(read_buf));
	set_fs(KERNEL_DS);

	fd = sys_open(PS_CALIBRATION_PATH, O_RDONLY, 0);
	if (fd >= 0) {
		len = sys_read(fd, read_buf, sizeof(read_buf));
		PINFO("Read calibration value from fs successfully. cross-talk is (%d)", (int)simple_strtoul(read_buf, NULL, 10));
		if (len <= 0) {
			ret = -1;
			PINFO("%s size is %d", PS_CALIBRATION_PATH, len);
			sys_close(fd);
			set_fs(old_fs);
			return ret;
		}

		ret = (int)simple_strtoul(read_buf, NULL, 10);

	} else {

#ifdef CONFIG_OF
		ret = ps_als_ginfo->default_cross_talk;
#else
		ret = DEFAULT_CROSS_TALK;
#endif
		PINFO("Fail to read calibration value from fs(error code:%d), use default cross_talk (%d)", fd, ret);
	}

	sys_close(fd);
	set_fs(old_fs);

	ps_als_ginfo->adjusted_psth_upper = ps_als_ginfo->config.init_data.psth_upper + ret;
	ps_als_ginfo->adjusted_psth_low = ps_als_ginfo->adjusted_psth_upper - ps_als_ginfo->config.init_data.psth_low;
	PINFO("adjusted_psth_upper = %d, adjusted_psth_low = %d", ps_als_ginfo->adjusted_psth_upper, ps_als_ginfo->adjusted_psth_low);

	return ret;
}

/******************************************************************************
 * NAME       : get_from_device
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static int get_from_device(DEVICE_VAL *dev_val, struct i2c_client *client)
{
#define LEDBIT_MASK    (3)
#define GAIN_VAL_MASK  (0xF)
	int           result;
    GENREAD_ARG   gene_read;
    unsigned char alsps_ctl = 0;
    unsigned char read_time = 0;

    /* initialize the returning value */
    dev_val->time        = 0;
    dev_val->gain        = 0;
    dev_val->led_current = 0;

    /* get measure time parameter */
    gene_read.adr_reg = REG_MODECONTROL;
    gene_read.addr    = &read_time;
    gene_read.size    = sizeof(read_time);
    result = ps_als_driver_general_read(gene_read, client);
    if (result < 0) {
        PINFO("ERROR! read data of itime.\n");
        return (result);
    }
    dev_val->time = read_time & 0xF;

    /* get gain parameter */
    gene_read.adr_reg = REG_ALSPSCONTROL;
    gene_read.addr    = &alsps_ctl;
    gene_read.size    = sizeof(alsps_ctl);
    result = ps_als_driver_general_read(gene_read, client);
    if (result < 0) {
        PINFO("ERROR! read data of gain.\n");
        return (result);
    }
    dev_val->led_current = alsps_ctl & LEDBIT_MASK;
    dev_val->gain        = (alsps_ctl >> 2) & GAIN_VAL_MASK;

    return (0);
#undef LEDBIT_MASK
#undef GAIN_VAL_MASK
}

static int get_boot_mode()
{
	enum lge_boot_mode_type lge_boot_mode;
	lge_boot_mode = lge_get_boot_mode();

	return lge_boot_mode;
}

static int lux_calculation()
{
    /* TODO : calculation lux value with alsdata0 and alsdata1
      you can use rpr0521_get_alsdata0 & rpr0521_get_alsdata1 function for read register*/
    unsigned long  base1, base2;
    unsigned long  cnt;
    signed long    keep_lux;
    int            lux;
    int            result;
    unsigned short data0, data1;
    signed long    moving_point;
    unsigned char  raw_gain, als_gain;

    if (ALSCALC_OFFSET == 0) {
        moving_point = 1;
    } else {
        moving_point = ALSCALC_OFFSET;
    }

#if 1
    /* get the data from IC */
	data0 = ps_als_ginfo->als_data0;
	data1 = ps_als_ginfo->als_data1;
	get_alsgain(ps_als_ginfo->client, &raw_gain, &als_gain);

	/* Check boot mode */
	if(ps_als_ginfo->boot_mode != LGE_BOOT_MODE_NORMAL) {
		lux = data0;

		if( als_gain == 64 ) {
			lux = data0 / als_gain;
		}

		return lux;
	}

	/* IF      D1/ D0 < 1.13  : case 0 */
    /* ELSE IF D1/ D0 < 1.424 : case 1 */
    /* ELSE IF D1/ D0 < 1.751 : case 2 */
    /* ELSE IF D1/ D0 < 3.015 : case 3 */
    /* ELSE                   : case 4 */
	base1 = data1 * moving_point;
	for(cnt=0;cnt < COEFFICIENT; cnt++) {
		base2 = data0 * judge_coef[cnt];
		if(base1 < base2) {
			break;
		}
	}

	if(cnt >= COEFFICIENT) {
		/* case 4 */
		lux = 0;
	} else {
		/* case 0,1,2,3 */
		/* lux = ((data0_conf * data0 - data1_conf * data1) / (1000 * gain)  */
		keep_lux = ((data0_coef[cnt] * data0) - (data1_coef[cnt] * data1));
		lux      = (int)(keep_lux / (moving_point * als_gain));
		if (lux < 0) {
			/* minimum process */
			lux = 0;
		} else if (lux > ALS_MAX_VALUE) {
			/* overflow process */
			lux = ALS_MAX_VALUE;
		}
	}

	ps_als_ginfo->lux_value = lux;
	PINFO("rpr0521_lux_calculation : keep_lux = %ld", keep_lux);
	PINFO("rpr0521_lux_calculation : lux_result = %d", lux);
	result = set_alsgain(ps_als_ginfo->client, raw_gain, lux);
#endif

	return (lux);
}

static int get_alsgain( struct i2c_client *client, unsigned char *raw_gain, unsigned char *gain)
{
    int result;
    unsigned char check_gain;
    unsigned char gain_value;

    result = ps_als_driver_read_alsps_control(raw_gain, client);
    if(result < 0) {
		return result;
	}

	check_gain = *raw_gain & ALSCALC_GAIN_MASK;
	if (check_gain == 0) {
		gain_value = 1;
	} else {
		gain_value = 64;
	}
	*gain = gain_value;

	return (result);
}


static int set_alsgain( struct i2c_client *client, unsigned char raw_gain, signed long lux)
{
    int result;
    unsigned char set_gain;
    unsigned char check_value;
    unsigned char set_value;

    if (lux < GAIN_THRESHOLD) {
        set_gain = ALSGAIN_X64X64;
    } else {
        set_gain = ALSGAIN_X1X1;
    }

    check_value = raw_gain & ALSCALC_GAIN_MASK;
    if(check_value != set_gain) {
        /* make setting value */
        set_value = (raw_gain & ~ALSCALC_GAIN_MASK) | set_gain;
        result    = ps_als_driver_write_alsps_control(set_value, client);
    }

    return (result);
}

static int ps_als_driver_write_alsps_control(unsigned char data, struct i2c_client *client)
{
    int result;

    /* write register to RPR0521 via i2c */
    result = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, data);

    return (result);
}

static int ps_als_driver_read_alsps_control(unsigned char* data, struct i2c_client *client)
{
    int result;

    /* write register to RPR0521 via i2c */
    result = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	*data = result;

    return (result);
}

/******************************************************************************
 * NAME       : update_ps_value
 * FUNCTION   : update_ps_value to near(1) or far(0)
 * REMARKS    : ssoon.lee@lge.com
 *****************************************************************************/
static int update_ps_value(unsigned short new_ps_data)
{
	ps_als_ginfo->ps_data = new_ps_data;

	if(new_ps_data > ps_als_ginfo->adjusted_psth_upper)	// current state is near
	{
		ps_als_ginfo->ps_value = true;
		PINFO("pdata(%d) > adjusted_near_offset(%d), proximity state is changed far to near",
				new_ps_data, ps_als_ginfo->adjusted_psth_upper);
		ps_als_driver_write_ps_th_h(PS_THRESHOLD_MAX, ps_als_ginfo->client);
		ps_als_driver_write_ps_th_l(ps_als_ginfo->adjusted_psth_low, ps_als_ginfo->client);

	}
	else if(new_ps_data < ps_als_ginfo->adjusted_psth_low)	// current state is far
	{
		ps_als_ginfo->ps_value = false;
		PINFO("pdata(%d) < adjusted_far_offset(%d), proximity state is changed near to far",
				new_ps_data, ps_als_ginfo->adjusted_psth_low);
		ps_als_driver_write_ps_th_h(ps_als_ginfo->adjusted_psth_upper, ps_als_ginfo->client);
		ps_als_driver_write_ps_th_l(PS_THRESHOLD_MIN, ps_als_ginfo->client);
	}
	else
	{
		PINFO("proximity state is not changed. pdata(%d)", new_ps_data);
	}
	return 0;
}

/******************************************************************************
 * NAME       : update_als_value
 * FUNCTION   : update als_data0, als_data1
 * REMARKS    : ssoon.lee@lge.com
 *****************************************************************************/
static int update_als_value(unsigned short new_als_data0, unsigned short new_als_data1)
{
	ps_als_ginfo->als_data0 = new_als_data0;
	ps_als_ginfo->als_data1 = new_als_data1;

	return 0;
}


/******************************************************************************
 * NAME       : ps_als_work_func
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static void ps_als_work_func(struct work_struct *work)
{
    int           result;
    long          get_timer;
    long          wait_sec;
    unsigned long wait_nsec;
    unsigned char read_intr;
    PWR_ST        pwr_st;
    READ_DATA_BUF read_data_buf;
    PS_ALS_DATA   *ps_als;
    DEVICE_VAL    dev_val;
    GENREAD_ARG   gene_data;

	mutex_lock(&ps_als_ginfo->work_lock);

	read_data_buf.ps_data   = 0;
    read_data_buf.als_data0 = 0;
    read_data_buf.als_data1 = 0;
    ps_als = container_of(work, PS_ALS_DATA, work);

    /* clear interrupt flag */
    gene_data.adr_reg = REG_INTERRUPT;
    gene_data.addr    = (unsigned char *)&read_intr;
    gene_data.size    = sizeof(read_intr);
    result = ps_als_driver_general_read(gene_data, ps_als->client);
    if (result < 0) {
        PINFO("general read can't execute \n");
        PINFO("can't read interrupt register \n");
		goto unlock;
    }

    /* read the state of sensor */
    result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
    if (result < 0) {
        PINFO("power_state error \n");
		goto unlock;
    }

	/* check the state of sensor */
    if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
		goto unlock;
    }
    result = ps_als_driver_read_data(&read_data_buf, ps_als->client);
    if (result < 0) {
        PINFO("ERROR! read data\n");
		goto unlock;
    }

	/* read value from device */
    result = get_from_device(&dev_val, ps_als->client);
    if (result < 0) {
        PINFO("ERROR! read data from device.\n");
		goto unlock;
    }

    if (pwr_st.als_state == CTL_STANDALONE && ps_als_ginfo->is_timer) {
		update_als_value(read_data_buf.als_data0, read_data_buf.als_data1);
		input_report_abs(ps_als->input_dev_als, ABS_MISC, lux_calculation());
		//input_report_abs(ps_als->input_dev_als, ABS_MISC, ps_als_ginfo->als_data0);
		input_sync(ps_als->input_dev_als);
		ps_als_ginfo->is_timer = false;

		/* the setting value from application */
		get_timer = ps_als->delay_time;
		/* 125ms(8Hz) at least */
		wait_sec  = (get_timer / SM_TIME_UNIT);
		wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
		result = hrtimer_start(&ps_als->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
		if (result != 0) {
			PINFO("can't start timer\n");
			goto unlock;
		}
	}
	if (pwr_st.ps_state == CTL_STANDALONE && ps_als_ginfo->is_interrupt) {
		update_ps_value(read_data_buf.ps_data);
		input_report_abs(ps_als->input_dev, ABS_DISTANCE, (ps_als_ginfo->ps_value==true)? 0:1);
		input_sync(ps_als->input_dev);
		ps_als_ginfo->is_interrupt = false;

        enable_irq(ps_als->client->irq);
	}

unlock :
	mutex_unlock(&ps_als_ginfo->work_lock);
	return ;
}

/******************************************************************************
 * NAME       : ps_als_timer_func
 * FUNCTION   : call work function (thread of timer)
 * REMARKS    :
 *****************************************************************************/
static enum hrtimer_restart ps_als_timer_func(struct hrtimer *timer)
{
    PS_ALS_DATA *ps_als;
    int         result;

	ps_als_ginfo->is_timer = true;
    ps_als = container_of(timer, PS_ALS_DATA, timer);
    result = queue_work(rohm_workqueue, &ps_als->work);
    if (result == 0) {
        PINFO("can't register que.\n");
        PINFO("result = 0x%x\n", result);
    }

    return (HRTIMER_NORESTART);
}

/******************************************************************************
 * NAME       : ps_als_irq_handler
 * FUNCTION   : interruption function (irq)
 * REMARKS    :
 *****************************************************************************/
static irqreturn_t ps_als_irq_handler(int irq, void *dev_id)
{
    PS_ALS_DATA *ps_als;
    int         result;

	ps_als_ginfo->is_interrupt = true;
    ps_als = dev_id;
    if (wake_lock_active(&ps_als->ps_wlock))
        wake_unlock(&ps_als->ps_wlock);
    wake_lock_timeout(&ps_als->ps_wlock, 1 * HZ);
    disable_irq_nosync(ps_als->client->irq);
    result = queue_work(rohm_workqueue, &ps_als->work);
    if (result == 0) {
        PINFO("can't register que.\n");
    }

    return (IRQ_HANDLED);
}

/******************************************************************************
 * NAME       : ps_als_iodev_open
 * FUNCTION   : initialize device and open process
 * REMARKS    :
 *****************************************************************************/
static int ps_als_iodev_open(struct inode *inode, struct file *file)
{
#define FOPEN_MAX_TIME (1)
    int         result;
    PWR_ST      pwr_st;
    PS_ALS_DATA *ps_als;

	mutex_lock(&ps_als_ginfo->enable_lock);

	ps_als = ps_als_ginfo;
    if (ps_als->fopen_cnt >= FOPEN_MAX_TIME) {
        PINFO("already call open function! open time = %d \n", ps_als->fopen_cnt);
		result = (-ENFILE);
		goto unlock;
    }
    ps_als->fopen_cnt++;
    /* read power state */
    pwr_st.als_state = 0;
    pwr_st.ps_state  = 0;
    result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
    if (result < 0) {
        PINFO("power_state error \n");
		goto unlock;
    }
    /* check the stop system is */
    if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {

		make_init_data(ps_als_ginfo);
		/* set initialization state */
        result = ps_als_driver_init(ps_als_ginfo->config.init_data, ps_als->client);
        if (result < 0) {
            PINFO("%s don't initialize to sensor\n", __func__);
            PINFO("error value = 0x%x\n", result);
			goto unlock;
        }
        file->private_data = ps_als;
    } else {
        PINFO("already open device !!\n");
    }

	PINFO("ps_als_iodev_open success");
	mutex_unlock(&ps_als_ginfo->enable_lock);
    return result;

unlock:
	PINFO("ps_als_iodev_open fail");
	mutex_unlock(&ps_als_ginfo->enable_lock);
	return result;

#undef FOPEN_MAX_TIME
}

static int make_init_data(PS_ALS_DATA *ps_als)
{
	/* set initialization data */
#ifdef CONFIG_OF
	/* construct register setting value from device tree */
	ps_als->config.init_data.mode_ctl = (ps_als->config.pulse << 5) |
                                              (ps_als->config.mode << 4) |
                                              (ps_als->config.measure_time);
	ps_als->config.init_data.psals_ctl = (ps_als->config.als_gain << 2) |
                                               (ps_als->config.led_current);
	ps_als->config.init_data.ps_ctl = (ps_als->config.infrared_level << 6) |
                                            (ps_als->config.ps_gain << 4) |
                                            (ps_als->config.persistence);
	ps_als->config.init_data.intr = (PS_THH_BOTH_OUTSIDE | MODE_PROXIMITY);
	ps_als->config.init_data.psth_upper = ps_als->config.near_offset;
	ps_als->config.init_data.psth_low = ps_als->config.far_offset;;
	ps_als->config.init_data.alsth_upper = PS_ALS_SET_ALS_TH;
	ps_als->config.init_data.alsth_low = PS_ALS_SET_ALS_TL;
	ps_als->boot_mode 					 = get_boot_mode();
#else
	ps_als->config.init_data.mode_ctl    = PS_ALS_SET_MODE_CONTROL;
	ps_als->config.init_data.psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
	ps_als->config.init_data.ps_ctl      = PS_ALS_SET_PS_CONTROL;
	ps_als->config.init_data.intr        = PS_ALS_SET_INTR;
	ps_als->config.init_data.psth_upper  = PS_ALS_SET_PS_TH;
	ps_als->config.init_data.psth_low    = PS_ALS_SET_PS_TL;
	ps_als->config.init_data.alsth_upper = PS_ALS_SET_ALS_TH;
	ps_als->config.init_data.alsth_low   = PS_ALS_SET_ALS_TL;
#endif
	PINFO("mode_ctl = %d", ps_als->config.init_data.mode_ctl);
	PINFO("psals_ctl = %d", ps_als->config.init_data.psals_ctl);
	PINFO("ps_ctl = %d", ps_als->config.init_data.ps_ctl);
	PINFO("intr = %d", ps_als->config.init_data.intr);
	PINFO("psth_upper = %d", ps_als->config.init_data.psth_upper);
	PINFO("psth_low = %d", ps_als->config.init_data.psth_low);
	PINFO("default_cross_talk = %d", ps_als->default_cross_talk);
	PINFO("boot_mode = %d", ps_als->boot_mode);
	return 0;
}

/******************************************************************************
 * NAME       : ps_als_iodev_release
 * FUNCTION   : stop device and close process
 * REMARKS    :
 *****************************************************************************/
static int ps_als_iodev_release(struct inode *inode, struct file *file)
{
    int         result;
    PS_ALS_DATA *ps_als;

    /* copy client data */
    ps_als = file->private_data;

    /* close driver */
    ps_als->fopen_cnt--;
    result = ps_als_driver_shutdown(ps_als->client);
    if (result != 0) {
        PINFO("can't close device\n");
    }

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_iodev_ioctl
 * FUNCTION   : set each parameter to device and read version and set timer
 * REMARKS    :
 *****************************************************************************/
#define GEN_READ_MAX (19)
unsigned char gen_data[GEN_READ_MAX];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
static int ps_als_iodev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long ps_als_iodev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
    void __user     *argp;
    POWERON_ALS_ARG pwr_als;
    POWERON_PS_ARG  pwr_ps;
    POWERON_ARG     power_data;
    int             result;
    unsigned long   time;
    unsigned char   intr;
    PS_ALS_DATA     *ps_als;
    unsigned short  ps_th_h;
    unsigned short  ps_th_l;
    unsigned short  als_th_up;
    unsigned short  als_th_low;
    GENREAD_ARG     gen_read;
    unsigned char   persistence;
    unsigned char   measure_time;

    argp   = (void __user *)arg;
    ps_als = file->private_data;
    switch (cmd) {
        case IOCTL_APP_SET_TIMER:
            if (copy_from_user(&time, argp, sizeof(time))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            if (time >= (PS_ALS_SET_MIN_DELAY_TIME)) {
                ps_als_ginfo->delay_time = time;
            } else {
                PINFO("parameter is too small. \n");
                return (-EFAULT);
            }
            break;
        case IOCTL_APP_SET_PWRSET_ALS:
            if (copy_from_user(&pwr_als, argp, sizeof(pwr_als))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            /* input power data */
            power_data.power_als = pwr_als.power_als;
            power_data.power_ps  = UNRELATEDNESS;
            power_data.intr      = pwr_als.intr;
            result = ps_als_ioctl_power_on_off(ps_als, &power_data);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_PWRSET_PS:
            if (copy_from_user(&pwr_ps, argp, sizeof(pwr_ps))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            /* input power data */
            power_data.power_als = UNRELATEDNESS;
            power_data.power_ps  = pwr_ps.power_ps;
            power_data.intr      = pwr_ps.intr;
            result = ps_als_ioctl_power_on_off(ps_als, &power_data);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_MEASUR_TIME:
            if (copy_from_user(&measure_time, argp, sizeof(measure_time))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_measurement_time(measure_time, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_PERSISTENCE:
            if (copy_from_user(&persistence, argp, sizeof(persistence))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_persistence(persistence, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_INTR_MODE:
            if (copy_from_user(&intr, argp, sizeof(intr))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_interrupt_mode(intr, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_PS_TH_HIGH:
            if (copy_from_user(&ps_th_h, argp, sizeof(ps_th_h))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_ps_th_h(ps_th_h, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_PS_TH_LOW:
            if (copy_from_user(&ps_th_l, argp, sizeof(ps_th_l))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_ps_th_l(ps_th_l, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_ALS_TH_UP:
            if (copy_from_user(&als_th_up, argp, sizeof(als_th_up))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_als_th_up(als_th_up, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_ALS_TH_LOW:
            if (copy_from_user(&als_th_low, argp, sizeof(als_th_low))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            result = ps_als_driver_write_als_th_low(als_th_low, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_SET_GENERAL:
            if (copy_from_user(&gen_read, argp, sizeof(gen_read))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            if (gen_read.size > GEN_READ_MAX) {
                PINFO("parameter NG!! size is the bigger than %d bytes.\n", GEN_READ_MAX);
                return (-EFAULT);
            }
            memset(&gen_data, 0, sizeof(gen_data));
            gen_read.addr = gen_data;
            result = ps_als_driver_general_read(gen_read, ps_als->client);
            if (result < 0) {
                PINFO("error value = 0x%x \n", result);
                return (result);
            }
            break;

        case IOCTL_APP_READ_GENERAL:
            if (copy_to_user(argp, &gen_data, sizeof(gen_data))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            break;

        case IOCTL_APP_READ_DRIVER_VER:
            if (copy_to_user(argp, &rpr0521_driver_ver, sizeof(rpr0521_driver_ver))) {
                PINFO("parameter NG !!\n");
                return (-EFAULT);
            }
            break;

        default:
            PINFO("non cmd parameter!\n");
            return (-EFAULT);
            break;
    }

    return (0);
}
#undef GEN_READ_MAX

/******************************************************************************
 * NAME       : ps_als_ioctl_power_on_off
 * FUNCTION   : execute power on/off of device
 * REMARKS    :
 *****************************************************************************/
static int ps_als_ioctl_power_on_off(PS_ALS_DATA *ps_als, POWERON_ARG *power_data)
{
    PWR_ST pwr_st;
    int    result;

    result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
    if (result < 0) {
        PINFO("power_state error \n");
        PINFO("can't excute device on/off \n");
        return (result);
    }
    /* make power on data */
    if (power_data->power_als == UNRELATEDNESS) {
        if (pwr_st.als_state == CTL_STANDALONE) {
            power_data->power_als = PS_ALS_ENABLE;
        } else {
            power_data->power_als = PS_ALS_DISABLE;
        }
    }
    if (power_data->power_ps == UNRELATEDNESS) {
        if (pwr_st.ps_state == CTL_STANDALONE) {
            power_data->power_ps = PS_ALS_ENABLE;
        } else {
            power_data->power_ps = PS_ALS_DISABLE;
        }
    }
    /* set power state */
    result = ps_als_driver_power_on_off(*power_data, ps_als->client);
    if (result < 0) {
        PINFO("power on failed\n");
        PINFO("error value = 0x%x\n", result);
        return (result);
    }
    /* clear to timer que */
    if ((power_data->power_ps == PS_ALS_DISABLE) && (power_data->power_als == PS_ALS_DISABLE)) {
        cancel_work_sync(&ps_als_ginfo->work);
    }

    /* set to start timer to que */
    if (ps_als->use_irq == IRQ_NON_USE) {
        if ((pwr_st.als_state == CTL_STANDBY) && (pwr_st.ps_state == CTL_STANDBY)) {
            if ((power_data->power_ps == PS_ALS_ENABLE) || (power_data->power_als == PS_ALS_ENABLE)) {
                /* start timer of 1 second */
                result = hrtimer_start(&ps_als_ginfo->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
                if (result != 0) {
                    PINFO("can't start timer\n");
                    return (result);
                }
            }
        }
    }

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (1)       // 0=near, 1=far

    PS_ALS_DATA *ps_als;
    int         result;
	int 		err=0;

	PINFO("RPR0521 proximity & ambient light sensor probing...");
    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        PINFO("need I2C_FUNC_I2C\n");
        result = -ENODEV;
        goto err_check_functionality_failed;
    }
    ps_als = kzalloc(sizeof(*ps_als), GFP_KERNEL);
    if (ps_als == NULL) {
        result = -ENOMEM;
        goto err_alloc_data_failed;
    }
    wake_lock_init(&ps_als->ps_wlock, WAKE_LOCK_SUSPEND, "proxi_wakelock");
    INIT_WORK(&ps_als->work, ps_als_work_func);
    ps_als->client = client;
    i2c_set_clientdata(client, ps_als);

#ifdef CONFIG_OF
	if(client->dev.of_node){
		err = sensor_parse_dt(&client->dev, ps_als);
		if(err)
			return err;
	}
#endif


/*********** start proximity sensor input_dev registration **********/
    ps_als->input_dev = input_allocate_device();

    if (!ps_als->input_dev) {
        result = -ENOMEM;
        dev_err(&ps_als->client->dev, "input device allocate failed\n");
        goto err_power_failed;
    }
    input_set_drvdata(ps_als->input_dev, ps_als);
    /* set event bit */
    set_bit(EV_ABS, ps_als->input_dev->evbit);
    set_bit(EV_SYN, ps_als->input_dev->evbit);

    /* initialization of abs event */
    input_set_abs_params(ps_als->input_dev, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

    /* set event name */
    ps_als->input_dev->dev.init_name = LGE_PROXIMITY_NAME;
	ps_als->input_dev->uniq = RPR0521_PROX_DRV_NAME;
	ps_als->input_dev->name = SENSOR_TYPE_PROXIMITY;

    /* register the device */
    result = input_register_device(ps_als->input_dev);
    if (result) {
        dev_err(&ps_als->client->dev,
                "unable to register input polled device %s: %d\n",
                ps_als->input_dev->name, result);
        goto err_inputdev;
    }

	err = sysfs_create_group(&ps_als->input_dev->dev.kobj, &ps_attr_group);
/*********** end proximity sensor input dev registration ********/

/*********** start light sensor input_dev registration **********/
	ps_als->input_dev_als = input_allocate_device();

    if (!ps_als->input_dev_als) {
        result = -ENOMEM;
		PINFO("input_allocate_device() fail");
        dev_err(&ps_als->client->dev, "input device allocate failed\n");
        goto err_power_failed;
    }
    input_set_drvdata(ps_als->input_dev_als, ps_als);
    /* set event bit */
    set_bit(EV_ABS, ps_als->input_dev_als->evbit);
    set_bit(EV_SYN, ps_als->input_dev_als->evbit);

    /* initialization of abs event */
    input_set_abs_params(ps_als->input_dev_als, ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);

    /* set event name */
    ps_als->input_dev_als->dev.init_name = LGE_LIGHT_NAME;
	ps_als->input_dev_als->uniq = RPR0521_LIGHT_DRV_NAME;
	ps_als->input_dev_als->name = SENSOR_TYPE_LIGHT;

    /* register the device */
    result = input_register_device(ps_als->input_dev_als);
    if (result) {
		PINFO("input_register_device() fail ");
        dev_err(&ps_als->client->dev,
                "unable to register input polled device %s: %d\n",
                ps_als->input_dev_als->name, result);
        goto err_inputdev;
    }

	err = sysfs_create_group(&ps_als->input_dev_als->dev.kobj, &als_attr_group);
/*********** end light sensor input dev registration ***************/


	make_init_data(ps_als);
	client->adapter->retries = 15;
	/* set initialization state */
	result = ps_als_driver_init(ps_als->config.init_data, ps_als->client);
	if (result < 0) {
		PINFO("%s don't initialize to sensor\n", __func__);
		PINFO("error value = 0x%x\n", result);
		goto err_inputdev;
	}

	/* reset for sensor */
	result = ps_als_driver_reset(ps_als->client);
	if (result != 0) {
		PINFO("don't reset result = 0x%x\n", result);
		goto err_inputdev;
	}
	/* check whether to use interrupt or not */
	ps_als->use_irq = IRQ_USE;
	if (client->irq) {
		/* interrupt process */
#if ((LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)) || (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)))
        result = request_irq(client->irq, ps_als_irq_handler, IRQF_TRIGGER_FALLING, client->name, ps_als);
#else
        result = request_threaded_irq(client->irq, NULL, ps_als_irq_handler, IRQF_TRIGGER_FALLING, client->name, ps_als);
#endif
        /* 1 : interrupt mode/ 0 : polling mode */
        if (result == 0) {
            ps_als->use_irq = IRQ_USE;
        } else {
            PINFO("request IRQ Failed==>result : %d\n", result);
            PINFO("client->irq        = 0x%x\n", client->irq);
            PINFO("ps_als_irq_handler = 0x%x\n", (int)ps_als_irq_handler);
            PINFO("interrupt flag     = 0x%x\n", IRQF_TRIGGER_FALLING);
            PINFO("interrupt name     = %s\n", client->name);
            PINFO("base address       = 0x%x\n", (int)ps_als);
        }
        enable_irq_wake(client->irq);

		/* timer process */
        hrtimer_init(&ps_als->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ps_als->timer.function = ps_als_timer_func;
    }
    result = misc_register(&rpr0521_device);
    if (result != 0) {
        PINFO("ps_als_probe register failed\n");
        goto err_inputdev;
    }

	/* ssoon.lee@lge.com */
    /* initialize static variable */
    ps_als->delay_time = 125;
    ps_als->fopen_cnt = 0;
	ps_als->cross_talk = 0;
	ps_als->adjusted_psth_upper = 0;
	ps_als->adjusted_psth_low = 0;
	ps_als->ps_value = false;
	ps_als->is_interrupt = false;
	ps_als->is_timer = false;
	ps_als_ginfo = ps_als;

	mutex_init(&ps_als_ginfo->enable_lock);
	mutex_init(&ps_als_ginfo->work_lock);

	PINFO("RPR0521 proximity & ambient light sensor is probed successfully!");
    return (result);
err_inputdev:
    input_free_device(ps_als->input_dev);
err_power_failed:
    wake_lock_destroy(&ps_als->ps_wlock);
    kfree(ps_als);
err_alloc_data_failed:
err_check_functionality_failed:

    return (result);

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

/******************************************************************************
 * NAME       : ps_als_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_remove(struct i2c_client *client)
{
    PS_ALS_DATA *ps_als;
    int         result;

    ps_als = i2c_get_clientdata(client);
    disable_irq_wake(client->irq);
    if (ps_als->use_irq == IRQ_USE) {
        free_irq(client->irq, ps_als);
    } else {
        result = hrtimer_cancel(&ps_als->timer);
        if (result != 0) {
            PINFO("can't finish timer.\n");
            return (result);
        }
    }
    wake_lock_destroy(&ps_als->ps_wlock);
    kfree(ps_als);
    result = misc_deregister(&rpr0521_device);
    if (result != 0) {
        PINFO( "misc driver don't register in list.\n");
    }

	mutex_destroy(&ps_als_ginfo->enable_lock);
	mutex_destroy(&ps_als_ginfo->work_lock);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
static int __devinit ps_als_init(void)
{
    rohm_workqueue = create_singlethread_workqueue("rohm_workqueue");
    if (!rohm_workqueue) {
        return (-ENOMEM);
    }

    return (i2c_add_driver(&rpr0521_driver));
}

/******************************************************************************
 * NAME       : ps_als_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
static void __exit ps_als_exit(void)
{
    i2c_del_driver(&rpr0521_driver);
    if (rohm_workqueue) {
        destroy_workqueue(rohm_workqueue);
    }

    return;
}

/************************************************************
 *                     access function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : ps_als_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_init(INIT_ARG data, struct i2c_client *client)
{
    struct init_func_write_data {
        unsigned char mode_ctl;
        unsigned char psals_ctl;
        unsigned char ps_ctl;
        unsigned char reserved0;
        unsigned char reserved1;
        unsigned char reserved2;
        unsigned char reserved3;
        unsigned char reserved4;
        unsigned char reserved5;
        unsigned char intr;
        unsigned char psth_hl;
        unsigned char psth_hh;
        unsigned char psth_ll;
        unsigned char psth_lh;
        unsigned char alsth_hl;
        unsigned char alsth_hh;
        unsigned char alsth_ll;
        unsigned char alsth_lh;
    } write_data;
    int result;

    /* execute software reset */
    result = ps_als_driver_reset(client);
    if (result != 0) {
        return (result);
    }

    /* not check parameters are psth_upper, psth_low, alsth_upper, alsth_low */
    /* check the PS orerating mode */
    if ((data.mode_ctl & 0xF) > MEASUREMENT_MAX) {
        return (-EINVAL);
    }
    if (0 != (data.mode_ctl & INIT_MODE_MASK)) {
        return (-EINVAL);
    }

    /* check the parameter of ps and als control */
    if (data.psals_ctl > REG_ALSPSCTL_MAX) {

        return (-EINVAL);
    }
    /* check the parameter of ps interrupt persistence */
    if ((data.ps_ctl & 0xF) > PERSISTENCE_MAX) {
        return (-EINVAL);
    }
    /* check the parameter of interrupt */
    if (data.intr > REG_INTERRUPT_MAX) {
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold high */
    if (data.psth_upper > REG_PSTH_MAX) {
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold low */
    if (data.psth_low > REG_PSTL_MAX) {
        return (-EINVAL);
    }
    write_data.mode_ctl  = data.mode_ctl;
    write_data.psals_ctl = data.psals_ctl;
    write_data.ps_ctl    = data.ps_ctl;
    write_data.reserved0 = 0;
    write_data.reserved1 = 0;
    write_data.reserved2 = 0;
    write_data.reserved3 = 0;
    write_data.reserved4 = 0;
    write_data.reserved5 = 0;
    write_data.intr      = data.intr;
    write_data.psth_hl   = CONVERT_TO_BE(data.psth_upper) & MASK_CHAR;
    write_data.psth_hh   = CONVERT_TO_BE(data.psth_upper) >> 8;
    write_data.psth_ll   = CONVERT_TO_BE(data.psth_low) & MASK_CHAR;
    write_data.psth_lh   = CONVERT_TO_BE(data.psth_low) >> 8;
    write_data.alsth_hl  = CONVERT_TO_BE(data.alsth_upper) & MASK_CHAR;
    write_data.alsth_hh  = CONVERT_TO_BE(data.alsth_upper) >> 8;
    write_data.alsth_ll  = CONVERT_TO_BE(data.alsth_low) & MASK_CHAR;
    write_data.alsth_lh  = CONVERT_TO_BE(data.alsth_low) >> 8;
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_shutdown
 * FUNCTION   : shutdown RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_shutdown(struct i2c_client *client)
{
    int result;

    /* set soft ware reset */
    result = ps_als_driver_reset(client);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_reset(struct i2c_client *client)
{
    int result;

    /* set soft ware reset */
    result = i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, (REG_SW_RESET | REG_INT_RESET));

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_power_on_off
 * FUNCTION   : power on/off PS and ALS
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_power_on_off(POWERON_ARG data, struct i2c_client *client)
{
    int           result;
    unsigned char power_set;

    /* check the parameter of als power , ps power , als interrupt and ps interrupt */
    if ((data.power_als > PS_ALS_ENABLE) || (data.power_ps > PS_ALS_ENABLE) || (data.intr > MODE_BOTH)) {
        return (-EINVAL);
    }

    /* read PS and ALS control register */
    result = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
    if (result < 0) {
        /* i2c communication error */
        return (result);
    }
    /* clear low 4bit by zero */
    power_set  = (unsigned char)(result & ~INIT_MODE_MASK);
    power_set |= (data.power_als << PWRON_ALS) + (data.power_ps << PWRON_PS);
    result = i2c_smbus_write_byte_data(client, REG_MODECONTROL, power_set);
    if (result < 0) {
        /* i2c communication error */
		PINFO("i2c_smbus_write_byte_data fail");
        return (result);
    }

    /* write interrupt register and clear interrupt polarity bit */
    result = ps_als_driver_write_interrupt_mode(data.intr, client);
    if (result < 0) {
		/* i2c communication error */
		PINFO("ps_als_driver_write_inerrupt_mode fail");
        return (result);
    }

	PINFO("ps_als_driver_power_on_off success.");

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_read_power_state
 * FUNCTION   : read the value of PS and ALS status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client)
{
    int result;

    /* read control state of ps and als */
    result = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
    if (result < 0) {
        pwr_st->als_state = CTL_STANDBY;
        pwr_st->ps_state  = CTL_STANDBY;
    } else {
        /* check power state of als from control state */
        if (result & PWRON_ALS_EN) {
            pwr_st->als_state = CTL_STANDALONE;
        } else {
            pwr_st->als_state = CTL_STANDBY;
        }

        /* check power state of ps from control state */
        if (result & PWRON_PS_EN) {
            pwr_st->ps_state = CTL_STANDALONE;
        } else {
            pwr_st->ps_state = CTL_STANDBY;
        }
    }

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_read_data
 * FUNCTION   : read the value of PS data and ALS data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_read_data(READ_DATA_BUF *data, struct i2c_client *client)
{
    int           result;
    READ_DATA_BUF multi;
    GENREAD_ARG   gene_read;

    /* read start address */
    gene_read.adr_reg = REG_PSDATA;
    gene_read.addr    = (char *)&multi;
    gene_read.size    = sizeof(multi);

    /* block read */
    result = ps_als_driver_general_read(gene_read, client);
    if (result > 0) {
        data->ps_data   = CONVERT_TO_BE(multi.ps_data);
        data->als_data0 = CONVERT_TO_BE(multi.als_data0);
        data->als_data1 = CONVERT_TO_BE(multi.als_data1);
        result          = 0;
    } else {
        data->ps_data   = 0;
        data->als_data0 = 0;
        data->als_data1 = 0;
    }

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_general_read
 * FUNCTION   : read general multi bytes
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client)
{
    int            result;
    struct i2c_msg msg[2];

    if (data.size == 0) {
        return (-EINVAL);
    }
    /* check the parameter of register */
    if ((data.adr_reg < REG_SYSTEMCONTROL) || (data.adr_reg > REG_ALSDATA0TL_MBS)) {
        return (-EINVAL);
    }

    /* write start address */
    msg[0].addr  = client->addr;
    msg[0].flags = 0;
    msg[0].len   = 1;
    msg[0].buf   = &data.adr_reg;

    /* read data */
    msg[1].addr  = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len   = data.size;
    msg[1].buf   = data.addr;

    /* block read */
    result = i2c_transfer(client->adapter, msg, 2);
    if (result < 0) {
        PINFO("transfer error \n");
    }

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_measurement_time
 * FUNCTION   : set als/ps measurement time
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_measurement_time(unsigned char data, struct i2c_client *client)
{
    int           result;
    unsigned char set_measure;

    /* check the parameter of hysteresis */
    if (data > MEASUREMENT_MAX) {
        return (-EINVAL);
    }
    /* read register to RPR0521 via i2c */
    result = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
    if (result < 0) {
        return (result);
    }

    set_measure = (unsigned char)(result & CLR_LOW4BIT) | data;

    /* write register to RPR0521 via i2c */
    result = i2c_smbus_write_byte_data(client, REG_MODECONTROL, set_measure);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_persistence
 * FUNCTION   : set als/ps persistence
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_persistence(unsigned char data, struct i2c_client *client)
{
    int result;

    /* check the parameter of hysteresis */
    if (data > PERSISTENCE_MAX) {
        return (-EINVAL);
    }

    /* write register to RPR0521 via i2c */
    result = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_interrupt_mode
 * FUNCTION   : set interrupt register of mode
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_interrupt_mode(unsigned char mode, struct i2c_client *client)
{
    int           result;
    unsigned char int_data;

    /* check the parameter of mode */
    if (mode > MODE_BOTH) {
        return (-EINVAL);
    }
    /* read register to RPR0521 via i2c */
    result = i2c_smbus_read_byte_data(client, REG_INTERRUPT);
    if (result < 0) {
        return (result);
    } else {
        int_data = (unsigned char)result;
    }
    int_data = (int_data & CLR_LOW2BIT) | mode;

    /* write register to RPR0521 via i2c */
    result = i2c_smbus_write_byte_data(client, REG_INTERRUPT, int_data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_ps_th_h
 * FUNCTION   : set ps interrupt standard of high level
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_ps_th_h(unsigned short data, struct i2c_client *client)
{
    int            result;
    unsigned short write_data;

    /* check the parameter of ps threshold high */
    if (data > REG_PSTH_MAX) {
        return (-EINVAL);
    }
    /* write register to RPR0521 via i2c */
    write_data = CONVERT_TO_BE(data);
    result = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_ps_th_l
 * FUNCTION   : set ps interrupt standard of low level
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_ps_th_l(unsigned short data, struct i2c_client *client)
{
    int            result;
    unsigned short write_data;

    /* check the parameter of ps threshold high */
    if (data > REG_PSTL_MAX) {
        return (-EINVAL);
    }
    /* write register to RPR0521 via i2c */
    write_data = CONVERT_TO_BE(data);
    result = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_als_th_up
 * FUNCTION   : set als interrupt standard of up
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_als_th_up(unsigned short data, struct i2c_client *client)
{
    int            result;
    unsigned short write_data;

    /* write register to RPR0521 via i2c */
    write_data = CONVERT_TO_BE(data);
    result = i2c_smbus_write_i2c_block_data(client, REG_ALSDATA0TH, sizeof(write_data), (unsigned char *)&write_data);

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_write_als_th_low
 * FUNCTION   : set als interrupt standard of low
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_write_als_th_low(unsigned short data, struct i2c_client *client)
{
    int            result;
    unsigned short write_data;

    /* write register to RPR0521 via i2c */
    write_data = CONVERT_TO_BE(data);
    result = i2c_smbus_write_i2c_block_data(client, REG_ALSDATA0TL, sizeof(write_data), (unsigned char *)&write_data);

    return (result);
}

MODULE_DESCRIPTION("ROHM Proximity Sensor & Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");

module_init(ps_als_init);
module_exit(ps_als_exit);
