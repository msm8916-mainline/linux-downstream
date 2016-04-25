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

#ifndef __LINUX_LASER_FORCUS_SENSOR_SYSFS_H
#define __LINUX_LASER_FORCUS_SENSOR_SYSFS_H

#define LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE		"/factory/LaserFocus_Calibration10.txt"
#define SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE		"/mnt/sdcard/LaserFocus_Calibration10.txt"
#define LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE		"/factory/LaserFocus_Calibration40.txt"
#define SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE		"/mnt/sdcard/LaserFocus_Calibration40.txt"

int Laser_Forcus_sysfs_read_offset(void);
bool Laser_Forcus_sysfs_write_offset(int calvalue);
int Laser_Forcus_sysfs_read_cross_talk_offset(void);
bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue);

#endif

