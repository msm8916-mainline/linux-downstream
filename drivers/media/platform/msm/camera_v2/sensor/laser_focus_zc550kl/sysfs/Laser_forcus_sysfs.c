/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "Laser_forcus_sysfs.h"

int Laser_Forcus_sysfs_read_offset(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	

	fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[Laser_Focu] Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		pr_err("[Laser_Focu] Offset Calibration file strlen f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		pr_err("[Laser_Focu] Offset Calibration file is NEGATIVE. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		printk("[Laser_Focu] Read Offset Calibration value : %d\n", cal_val);
	}	

	return cal_val;
}

bool Laser_Forcus_sysfs_write_offset(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

#if 1
	pr_info("[[Laser_Focu]] g_ASUS_bootmode = %d\n",g_ASUS_bootmode);

	if(g_ASUS_bootmode == USER_MODE || g_ASUS_bootmode == CHARGER_FACTORY_MODE || g_ASUS_bootmode == FFBM_MODE)
	{
		fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
	else if(g_ASUS_bootmode == SHIPPING_MODE || g_ASUS_bootmode == CHARGER_SHIPPING_MODE)
	{
		fp = filp_open(SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
	else
	{
		fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
#else
	fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
#endif
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[Laser_Focu] Offset Calibration file open (%s) fail\n", SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[Laser_Focu] Offset Calibration file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[Laser_Focu] write Offset Calibration value : %s\n", buf);

	return true;
}

int Laser_Forcus_sysfs_read_cross_talk_offset(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	

	fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[Laser_Focu] Cross-talk Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		pr_err("[Laser_Focu] Cross-talk Offset Calibration file strlen f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		pr_err("[Laser_Focu] Cross-talk Offset Calibration file is NEGATIVE. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		printk("[Laser_Focu] Read Cross-talk Offset Calibration value : %d\n", cal_val);
	}	

	return cal_val;
}

bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

#if 1
	pr_info("[[Laser_Focu]] g_ASUS_bootmode = %d\n",g_ASUS_bootmode);
	
	if(g_ASUS_bootmode == USER_MODE || g_ASUS_bootmode == CHARGER_FACTORY_MODE || g_ASUS_bootmode == FFBM_MODE)
	{
		fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
	else if(g_ASUS_bootmode == SHIPPING_MODE || g_ASUS_bootmode == CHARGER_SHIPPING_MODE)
	{
		fp = filp_open(SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
	else
	{
		fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	}
#else
	fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
#endif
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[Laser_Focu] Cross-talk Offset Calibration file open (%s) fail\n", SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[Laser_Focu] Cross-talk Offset Calibration file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[Laser_Focu] write Cross-talk Offset Calibration value : %s\n", buf);

	return true;
}

