// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright ?2012 Synaptics Incorporated. All rights reserved.
//
// The information in this file is confidential under the terms
// of a non-disclosure agreement with Synaptics and is provided
// AS IS.
//
// The information in this file shall remain the exclusive property
// of Synaptics and may be the subject of Synaptics?patents, in
// whole or part. Synaptics?intellectual property rights in the
// information in this file are not expressly or implicitly licensed
// or otherwise transferred to you as a result of such information
// being made available to you.
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// FullRawCapacitance Support 0D button
//
#define LGTP_MODULE "[S3320]"

#include <linux/file.h>		//for file access
#include <linux/syscalls.h> //for file access
#include <linux/uaccess.h>  //for file access

#include <linux/input/unified_driver_2/lgtp_common.h>
#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_model_config.h>
#include <linux/input/unified_driver_2/lgtp_device_s3320.h>

#include "TestLimits.h"


#define REPORT_TYPE_REG					(ts->analog_fc.dsc.data_base)
#define REPORT_LSB_REG					(REPORT_TYPE_REG+1)
#define REPORT_MSB_REG					(REPORT_TYPE_REG+2)
#define REPORT_DATA_REG					(REPORT_TYPE_REG+3)

#define ANALOG_COMMAND_REG				(ts->analog_fc.dsc.command_base)

#define GENERAL_CONTROL_REG				(ts->analog_fc.dsc.control_base)
#define NOISE_MITIGRATION_CONTROL_REG	(GENERAL_CONTROL_REG+21)
#define ANALOG_CONTROL_REG				(GENERAL_CONTROL_REG+41)

#define RX_CH_CNT_REG					(ts->analog_fc.dsc.query_base)
#define TX_CH_CNT_REG					(RX_CH_CNT_REG+1)

#define TRX_MAX 						32
#define CAP_FILE_PATH					"/mnt/sdcard/touch_self_test.txt"
#define DS5_BUFFER_SIZE 				6000
#define WAIT_TIME 						100

u8 f54_wlog_buf[DS5_BUFFER_SIZE] = {0};

int full_raw_data[TRX_MAX][TRX_MAX];
int RxChNum;
int TxChNum;
int rst_size;

#define WRITE_BUFFER(buf, fmt, args...) do { 			\
	rst_size += sprintf(buf + rst_size, fmt, ##args);	\
} while(0)

static int pow_func(int x, int y)
{
	int result = 1;
	int i = 0;

	for(i = 0; i < y; i++)
		result *= x;

	return result;
}

static int write_log(char *filename, char *data)
{
	int fd;
	int cap_file_exist = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	if(filename == NULL){
		fd = sys_open(CAP_FILE_PATH, O_WRONLY|O_CREAT|O_APPEND, 0666);
		TOUCH_DBG("write log in /mnt/sdcard/touch_self_test.txt\n");
	} else{
		fd = sys_open(filename, O_WRONLY|O_CREAT, 0666);
		TOUCH_DBG("write log in /sns/touch/cap_diff_test.txt\n");
	}

	TOUCH_DBG("[%s]write file open %s, fd : %d\n", __FUNCTION__, (fd >= 0)? "success": "fail", fd);

	if(fd >= 0) {
		sys_write(fd, data, strlen(data));
		sys_close(fd);

		if(filename != NULL)
			cap_file_exist = 1;
	}
	set_fs(old_fs);

	return cap_file_exist;
}

static int RunAutoScan(struct synaptics_ts_data *ts)
{
	int ret = 0;
	int cnt = 0;
	u8 data;

	//Set the GetReport bit to run the AutoScan
	data = 0x01;
	ret |= Touch_I2C_Write(ts->client, ANALOG_COMMAND_REG, &data, 1);

	do {
		msleep(WAIT_TIME);
		ret |= Touch_I2C_Read(ts->client, ANALOG_COMMAND_REG, &data, 1);

		if(ret) {
			TOUCH_ERR("%s : failed to access I2C\n", __func__);
			return TOUCH_FAIL;
		}

		cnt += 1;
	} while(data != 0x00 && cnt < 3);

	if (cnt >= 3) {
		TOUCH_ERR("%s : Timeout -- Change the Report Type\n", __func__);

		WRITE_BUFFER(f54_wlog_buf, "Timeout -- Not supported Report Type in FW\n");
		write_log(NULL, f54_wlog_buf);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int ReadChCount(struct synaptics_ts_data *ts)
{
	int ret = 0;
	u8 data[64] = {0};
	u8 rx_max_cnt = 0;
	u8 tx_max_cnt = 0;
	int i;

	RxChNum = TxChNum = 0;

	/*Read channel count*/
	ret = Touch_I2C_Read(ts->client, RX_CH_CNT_REG, &rx_max_cnt, 1);
	ret |= Touch_I2C_Read(ts->client, TX_CH_CNT_REG, &tx_max_cnt, 1);
	ret |= Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, ts->sensor_fc.function_page);
	ret |= Touch_I2C_Read(ts->client, ts->sensor_fc.dsc.data_base+1, data, rx_max_cnt);

	for (i = 0; i < (int)rx_max_cnt; i++) {
		if (data[i] != 0xFF)
			RxChNum++;
	}

	ret |= Touch_I2C_Read(ts->client, ts->sensor_fc.dsc.data_base+2, data, tx_max_cnt);

	for (i = 0; i < (int)tx_max_cnt; i++) {
		if (data[i] != 0xFF)
			TxChNum++;
	}

	TOUCH_DBG("rx ch cnt = %d, tx ch cnt = %d\n", RxChNum, TxChNum);

	ret |= Touch_I2C_Write_Byte(ts->client, PAGE_SELECT_REG, ts->analog_fc.function_page);

	if(ret) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int TestPreparation(struct synaptics_ts_data *ts)
{
	int ret = 0;
	u8 data = 0;

	ReadChCount(ts);

	/*Turn off CBC.*/
	ret = Touch_I2C_Read(ts->client, ANALOG_CONTROL_REG, &data, 1);
	data = data & 0xDF;
	ret |= Touch_I2C_Write(ts->client, ANALOG_CONTROL_REG, &data, 1);

	/* Turn off SignalClarity. ForceUpdate is required for the change to be effective */
	ret |= Touch_I2C_Read(ts->client, NOISE_MITIGRATION_CONTROL_REG, &data, 1);
	data = data | 0x01;
	ret |= Touch_I2C_Write(ts->client, NOISE_MITIGRATION_CONTROL_REG, &data, 1);

	/*Apply ForceUpdate.*/
	ret |= Touch_I2C_Read(ts->client, ANALOG_COMMAND_REG, &data, 1);
	data = data | 0x04;
	ret |= Touch_I2C_Write(ts->client, ANALOG_COMMAND_REG, &data, 1);
	msleep(WAIT_TIME);

	/*Apply ForceCal.*/
	ret |= Touch_I2C_Read(ts->client, ANALOG_COMMAND_REG, &data, 1);
	data = data | 0x02;
	ret |= Touch_I2C_Write(ts->client, ANALOG_COMMAND_REG, &data, 1);
	msleep(WAIT_TIME);

	if(ret) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int ReadTRexShort(struct synaptics_ts_data *ts)
{
	int ret = 0;
	unsigned char TRX_Short[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	int MaxArrayLength = RxChNum * TxChNum * 2;
	int i, j = 0;
	int mask = 0x01;
	int value;	// Hardcode for Waikiki Test and it support up to 54 Tx
	int result = TOUCH_SUCCESS;
	u8 buf[40] = {0};
	u8 cap_data[MaxArrayLength];

	TOUCH_DBG("Start : TRex Short Test\n");

	ret = Touch_I2C_Read(ts->client, REPORT_DATA_REG, cap_data, MaxArrayLength);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	WRITE_BUFFER(f54_wlog_buf, "TRex Short Test\n");

	for (i = 0; i < 7; i++) {
		int short_num = 0;

		value = cap_data[i];
		cap_data[i] = 0;

		for (j = 0; j < 8; j++) {
			if((value & mask) == 1) {
				cap_data[i] = cap_data[i] + (unsigned char)pow_func(2, (7 - j));
				short_num += sprintf(buf+short_num, "%d ", (i * 8 + (7 - j)));
			}

			value >>= 1;
		}

		WRITE_BUFFER(f54_wlog_buf, "TRex-TRex Short Test Data = %#x", cap_data[i]);

		if (short_num) {
			WRITE_BUFFER(f54_wlog_buf, " (Short TRx Number: ");
			WRITE_BUFFER(f54_wlog_buf, buf);
			WRITE_BUFFER(f54_wlog_buf, ")\n");
		}

		WRITE_BUFFER(f54_wlog_buf, "\n");
	}

	for(i = 0; i < 7; i++) {
		if(cap_data[i] != TRX_Short[i]) {
			result = TOUCH_FAIL;
			break;
		}
	}

	if (result == TOUCH_SUCCESS)
		WRITE_BUFFER(f54_wlog_buf, "\nTRex-TRex Short Test passed.\n\n");
	else
		WRITE_BUFFER(f54_wlog_buf, "\nTRex-TRex Short Test failed.\n\n");

	write_log(NULL, f54_wlog_buf);

	TOUCH_DBG("TRex Short Test Result = %s\n", result ? "FAIL" : "SUCESS");

	return result;
}

static int ReadHighResistance(struct synaptics_ts_data *ts)
{
	int ret = 0;
	int HighResistanceLowerLimit[3] = {-1000, -1000, -400};
	int HighResistanceUpperLimit[3] = {450, 450, 200};
	int maxRxpF, maxTxpF, minpF;
	int i = 0;
	int result = TOUCH_SUCCESS;
	u8 cap_data[6];
	short maxRx, maxTx, min;

	TOUCH_DBG("Start : High Resistance Test\n");

	ret = Touch_I2C_Read(ts->client, REPORT_DATA_REG, cap_data, 6);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	maxRx = ((short)cap_data[0] | (short)cap_data[1] << 8);
	maxTx = ((short)cap_data[2] | (short)cap_data[3] << 8);
	min = ((short)cap_data[4] | (short)cap_data[5] << 8);

	maxRxpF = maxRx;
	maxTxpF = maxTx;
	minpF = min;

	WRITE_BUFFER(f54_wlog_buf, "High Resistance Test\n");
	WRITE_BUFFER(f54_wlog_buf, "Max Rx Offset(pF) = %d\n", maxRxpF);
	WRITE_BUFFER(f54_wlog_buf, "Max Tx Offset(pF) = %d\n", maxTxpF);
	WRITE_BUFFER(f54_wlog_buf, "Min(pF) = %d\n", minpF);
	WRITE_BUFFER(f54_wlog_buf, "\n=====================================================\n");
	WRITE_BUFFER(f54_wlog_buf, "\tHigh Resistance Test\n");
	WRITE_BUFFER(f54_wlog_buf, "=====================================================\n");
	WRITE_BUFFER(f54_wlog_buf, " Parameters: ");
	WRITE_BUFFER(f54_wlog_buf, "%5d %5d %5d ", maxRxpF, maxTxpF, minpF);
	WRITE_BUFFER(f54_wlog_buf, "\n\n Limits(+) : ");

	for(i = 0; i < 3; i++)
		WRITE_BUFFER(f54_wlog_buf, "%5d ", HighResistanceUpperLimit[i]);

	WRITE_BUFFER(f54_wlog_buf, "\n Limits(-) : ");

	for(i = 0; i < 3; i++)
		WRITE_BUFFER(f54_wlog_buf, "%5d ", HighResistanceLowerLimit[i]);

	WRITE_BUFFER(f54_wlog_buf, "\n-----------------------------------------------------\n");

	if (maxRxpF > HighResistanceUpperLimit[0] || maxRxpF < HighResistanceLowerLimit[0])
		result = TOUCH_FAIL;
	if (maxTxpF > HighResistanceUpperLimit[1] || maxTxpF < HighResistanceLowerLimit[1])
		result = TOUCH_FAIL;
	if (minpF > HighResistanceUpperLimit[2] || minpF < HighResistanceLowerLimit[2])
		result = TOUCH_FAIL;

	if (result == TOUCH_FAIL)
		WRITE_BUFFER(f54_wlog_buf, "HighResistance Test failed.\n\n");
	else
		WRITE_BUFFER(f54_wlog_buf, "HighResistance Test passed.\n\n");

	write_log(NULL, f54_wlog_buf);

	TOUCH_DBG("High Resistance Test Result = %s\n", result ? "FAIL" : "SUCESS");

	return result;
}

// Construct data with Report Type #20 data
static int ReadRawData(struct synaptics_ts_data *ts)
{
	int ret = 0;
	int MaxArrayLength = RxChNum * TxChNum * 2;
	int i,j,k = 0;
	int result = TOUCH_SUCCESS;
	u8 cap_data[MaxArrayLength];

	TOUCH_DBG("Start : Raw Data Test\n");

	ret = Touch_I2C_Read(ts->client, REPORT_DATA_REG, cap_data, MaxArrayLength);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	WRITE_BUFFER(f54_wlog_buf, "Full Raw Capacitance Test\n");
	WRITE_BUFFER(f54_wlog_buf, "\nInfo: Tx = %d Rx = %d \n", TxChNum, RxChNum);
	WRITE_BUFFER(f54_wlog_buf, "Image Data : \n");
	WRITE_BUFFER(f54_wlog_buf, "==========================================================================================================\n         :");

	for (i = 0; i < RxChNum; i++)
		WRITE_BUFFER(f54_wlog_buf, "%5d ", i);

	WRITE_BUFFER(f54_wlog_buf, "\n----------------------------------------------------------------------------------------------------------\n");

	for (i = 0; i < TxChNum; i++) {
		WRITE_BUFFER(f54_wlog_buf, "   %5d : ", i);
		for (j = 0; j < RxChNum; j++) {
			full_raw_data[i][j] = ((short)cap_data[k] | (short)cap_data[k+1] << 8);

			WRITE_BUFFER(f54_wlog_buf, "%5d ", full_raw_data[i][j]);
			k = k + 2;
		}
		WRITE_BUFFER(f54_wlog_buf, "\n");
	}
	WRITE_BUFFER(f54_wlog_buf, "------------------------------------------------------------------------------------------------------------\n");

	//Compare 2D area
	for (j = 0; j < RxChNum; j++) {
		for (i = 0; i < TxChNum; i++) {
			if ((full_raw_data[i][j] < LowerImageLimit[i][j]) || (full_raw_data[i][j] > UpperImageLimit[i][j])) {
				WRITE_BUFFER(f54_wlog_buf, "FAIL, %d,%d,%d\n", LowerImageLimit[i][j], UpperImageLimit[i][j], full_raw_data[i][j]);
				result = TOUCH_FAIL;
				break;
			}
		}
	}

	if (result == TOUCH_SUCCESS)
		WRITE_BUFFER(f54_wlog_buf, "\nFull Raw Capacitance Image Test passed.\n\n");
	else
		WRITE_BUFFER(f54_wlog_buf, "\nFull Raw Capacitance Image Test failed.\n\n");

	write_log(NULL, f54_wlog_buf);

	TOUCH_DBG("Raw Data Test Result = %s\n", result ? "FAIL" : "SUCESS");

	return result;
}

static int TRexShortTest(struct synaptics_ts_data *ts)
{
	int ret = 0;
	u8 data;

	TestPreparation(ts);

	/*Assign report type for TRex Short Test*/
	data = 0x1A;
	ret = Touch_I2C_Write(ts->client, REPORT_TYPE_REG, &data, 1);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	if (RunAutoScan(ts))
		return TOUCH_FAIL;

	if (ReadTRexShort(ts))
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

static int HighResistanceTest(struct synaptics_ts_data *ts)
{
	int ret = 0;
	u8 data;

	TestPreparation(ts);

	/*Assign report type for High Resistance report*/
	data = 0x04;
	ret = Touch_I2C_Write(ts->client, REPORT_TYPE_REG, &data, 1);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	if (RunAutoScan(ts))
		return TOUCH_FAIL;

	if (ReadHighResistance(ts))
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

// The following funtion illustrates the steps in getting a full raw image report (report #20) by Function $54.
static int FullRawTest(struct synaptics_ts_data *ts, int mode)
{
	int ret = 0;
	u8 data;

	TestPreparation(ts);

	TOUCH_DBG("[Touch][%s] raw capacitance mode!\n", __FUNCTION__);

	data = 0x14;	//raw capacitance mode
	ret = Touch_I2C_Write(ts->client, REPORT_TYPE_REG, &data, 1);
	if(ret == TOUCH_FAIL) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	if (RunAutoScan(ts))
		return TOUCH_FAIL;

	if (ReadRawData(ts))		//rawdata store mode
		return TOUCH_FAIL;

	return TOUCH_SUCCESS;
}

// Main entry point for the application
int F54TestHandle(struct synaptics_ts_data *ts, int input, int mode, char *buf)
{
	int ret = 0;
	u8 data;

	rst_size = 0;
	memset(f54_wlog_buf, 0x00, sizeof(f54_wlog_buf));

	data = 0x00;
	ret = Touch_I2C_Write(ts->client, REPORT_LSB_REG, &data, 1);
	ret |= Touch_I2C_Write(ts->client, REPORT_MSB_REG, &data, 1);

	if(ret) {
		TOUCH_ERR("%s : failed to access I2C\n", __func__);
		return TOUCH_FAIL;
	}

	switch(input) {
	case F54_FULL_RAW_CAP:
		ret = FullRawTest(ts, mode);
		break;
	case F54_HIGH_RESISTANCE:
		ret = HighResistanceTest(ts);
		break;
	case F54_TRX_TO_TRX:
		ret = TRexShortTest(ts);
		break;
	default:
		break;
	}

	msleep(100);

	return ret;
}
