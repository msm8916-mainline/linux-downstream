/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#ifndef __LINUX_SHOW_VL6180x_SENSOR_SHIPPING_FUNC_H
#define __LINUX_SHOW_VL6180x_SENSOR_SHIPPING_FUNC_H

#include "msm_laser_focus.h"
#include "laura_i2c.h"

#ifndef LOG_SAMPLE_RATE
/* Log sample rate  */
#define LOG_SAMPLE_RATE 50
#endif

#ifndef OUT_OF_RANGE
/* Out of range */
#define OUT_OF_RANGE 765
#endif

#ifndef TIMEOUT_VAL
/* Time out value: mm */
#define TIMEOUT_VAL 80
#endif

/* Read retry flag */
#define READ_RETRY_FLAG 0

/* Read output limit flag */
#define READ_OUTPUT_LIMIT_FLAG 1

/* MCPU power on fail */
#define MCPU_POWER_ON_FAIL 1

/* Laura calibration message lens */
#define CAL_MSG_LEN 6

/* MCPU status */
#define MCPU_ON 0
#define MCPU_OFF 1
#define MCPU_STANDBY 2

/* Swap high and low of the data (e.g 0x1234 => 0x3412) */
uint16_t swap_data(uint16_t register_data);
/* Mailbox: create calibration data */
int Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]);
/* Load calibration data */
int Laura_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t);
/* Read range */
uint16_t Laura_device_read_range(struct msm_laser_focus_ctrl_t *dev_t);
/* MCPU controller */
int Laura_MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode);
/* Initialize Laura tof configure */
int Laura_device_UpscaleRegInit(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config);
/* Wait device go to standby mode */
int Laura_WaitDeviceStandby(struct msm_laser_focus_ctrl_t *dev_t);
/* Configure i2c interface */
int Laura_Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t);
/* Power up initialization without applying calibration data */
int Laura_Power_Up_Init_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t);
/* Power up initialization which apply calibration data */
int Laura_Power_Up_Init_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t);
/* Go to standby mode when do not do measure */
int Laura_non_measures_go_standby(struct msm_laser_focus_ctrl_t *dev_t);
/* Verify firmware version */
bool Laura_FirmWare_Verify(struct msm_laser_focus_ctrl_t *dev_t);
//int laura_read_write_test(struct msm_laser_focus_ctrl_t * dev_t);

#endif
