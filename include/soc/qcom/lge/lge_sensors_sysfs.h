/*
 * LGE charging scenario Header file.
 *
 * Copyright (C) 2013 LG Electronics
 * sangwoo <sangwoo2.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LGE_SENSOR_SYSFS_H_
#define __LGE_SENSOR_SYSFS_H_

#define ACCEL              "accelerometer"
#define GYRO               "gyroscope"
#define PROX               "proximity"

#define CALIBRATION        "run_calibration"
#define SELFTEST           "selftest"
#define CALIBRATION_DATA   "calibration_data"

#define SENSOR_SYSFS_ACCEL "/sys/devices/virtual/input/lge_accelerometer"
#define SENSOR_SYSFS_GYRO  "/sys/devices/virtual/input/lge_gyroscope"
#define SENSOR_SYSFS_PROX  "/sys/devices/virtual/input/lge_proximity"

struct sensor_sysfs_array {
    const char *group;
    const char *user_node;
    const char *kernel_node;
    struct proc_dir_entry* parent;
};

const char *sensor_mandatory_paths[][2] = {
    { ACCEL,   CALIBRATION },
    { ACCEL,   SELFTEST },
    { GYRO,    CALIBRATION },
    { GYRO,    SELFTEST },
    { PROX,    CALIBRATION },
    { PROX,    CALIBRATION_DATA },
};

/* unused
const char *group_names[] = {
    ACCEL,
    GYRO,
    PROX,
};
*/

/* Set sysfs node for non-using DT */
const char *default_sns_sysfs_path[][3] = {
    { ACCEL,   CALIBRATION,      SENSOR_SYSFS_ACCEL"/run_fast_calibration" },
    { ACCEL,   SELFTEST,         SENSOR_SYSFS_ACCEL"/"SELFTEST },
    { GYRO,    CALIBRATION,      SENSOR_SYSFS_GYRO"/"CALIBRATION },
    { GYRO,    SELFTEST,         SENSOR_SYSFS_GYRO"/"SELFTEST },
    { PROX,    CALIBRATION,      SENSOR_SYSFS_PROX"/"CALIBRATION },
    { PROX,    CALIBRATION_DATA, SENSOR_SYSFS_PROX"/"CALIBRATION_DATA },
};
#endif
