/***************************************************************************
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
 *    File  	: lgtp_device_s3320.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[S3320]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_s3320.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/

/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control		45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */

#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT		0x34
#define LPWG_CONTROL				0x51
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define DEVICE_CONTROL_REG 			(ts->common_fc.dsc.control_base)
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)
#define CUSTOMER_FAMILY_REG			(ts->common_fc.dsc.query_base+2)
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)

#define COMMON_PAGE					(ts->common_fc.function_page)
#define LPWG_PAGE					(ts->lpwg_fc.function_page)
#define FINGER_PAGE					(ts->finger_fc.function_page)
#define ANALOG_PAGE					(ts->analog_fc.function_page)
#define FLASH_PAGE					(ts->flash_fc.function_page)
#define SENSOR_PAGE					(ts->sensor_fc.function_page)

#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)
#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base+1)
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base+1)

/* TOUCHPAD_SENSORS */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base+3)
#define FINGER_REPORT_REG			(ts->finger_fc.dsc.control_base+7)
#define OBJECT_REPORT_ENABLE_REG	(ts->finger_fc.dsc.control_base+9)
#define WAKEUP_GESTURE_ENABLE_REG	(ts->finger_fc.dsc.control_base+12)

#define DYNAMIC_SENSING_CONTROL_REG	(ts->analog_fc.dsc.control_base+40)

#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)
#define FLASH_STATUS_REG			(ts->flash_fc.dsc.data_base+3)

#define LPWG_STATUS_REG				(ts->lpwg_fc.dsc.data_base)
#define LPWG_DATA_REG				(ts->lpwg_fc.dsc.data_base+1)
#define LPWG_OVER_TAPCOUNT			(ts->lpwg_fc.dsc.data_base+73)

#define LPWG_TAPCOUNT_REG			(ts->lpwg_fc.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+1)
#define LPWG_MAX_INTERTAP_REG		(ts->lpwg_fc.dsc.control_base+2)
#define LPWG_TOUCH_SLOP_REG			(ts->lpwg_fc.dsc.control_base+3)
#define LPWG_TAP_DISTANCE_REG		(ts->lpwg_fc.dsc.control_base+4)
#define LPWG_INTERRUPT_DELAY_REG	(ts->lpwg_fc.dsc.control_base+6)
#define LPWG_TAPCOUNT_REG2			(ts->lpwg_fc.dsc.control_base+7)
#define LPWG_MIN_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+8)
#define LPWG_MAX_INTERTAP_REG2		(ts->lpwg_fc.dsc.control_base+9)
#define LPWG_TOUCH_SLOP_REG2		(ts->lpwg_fc.dsc.control_base+10)
#define LPWG_TAP_DISTANCE_REG2		(ts->lpwg_fc.dsc.control_base+11)
#define LPWG_INTERRUPT_DELAY_REG2	(ts->lpwg_fc.dsc.control_base+13)

#if defined(ENABLE_TCI_DEBUG)
#define LPWG_TCI1_FAIL_COUNT_REG			(ts->lpwg_fc.dsc.data_base+49)
#define LPWG_TCI1_FAIL_INDEX_REG			(ts->lpwg_fc.dsc.data_base+50)
#define LPWG_TCI1_FAIL_BUFFER_REG			(ts->lpwg_fc.dsc.data_base+51)
#define LPWG_TCI2_FAIL_COUNT_REG			(ts->lpwg_fc.dsc.data_base+61)
#define LPWG_TCI2_FAIL_INDEX_REG			(ts->lpwg_fc.dsc.data_base+62)
#define LPWG_TCI2_FAIL_BUFFER_REG			(ts->lpwg_fc.dsc.data_base+63)
#endif

#if defined(ENABLE_SWIPE_MODE)
#define SWIPE_COOR_START_X_LSB_REG			(ts->lpwg_fc.dsc.data_base + 74)
#define SWIPE_COOR_START_X_MSB_REG			(ts->lpwg_fc.dsc.data_base + 75)
#define SWIPE_COOR_START_Y_LSB_REG			(ts->lpwg_fc.dsc.data_base + 76)
#define SWIPE_COOR_START_Y_MSB_REG			(ts->lpwg_fc.dsc.data_base + 77)
#define SWIPE_COOR_END_X_LSB_REG			(ts->lpwg_fc.dsc.data_base + 78)
#define SWIPE_COOR_END_X_MSB_REG			(ts->lpwg_fc.dsc.data_base + 79)
#define SWIPE_COOR_END_Y_LSB_REG			(ts->lpwg_fc.dsc.data_base + 80)
#define SWIPE_COOR_END_Y_MSB_REG			(ts->lpwg_fc.dsc.data_base + 81)

#define SWIPE_FAIL_REASON_REG				(ts->lpwg_fc.dsc.data_base + 82)
#define SWIPE_TIME_LSB_REG					(ts->lpwg_fc.dsc.data_base + 83)
#define SWIPE_TIME_MSB_REG					(ts->lpwg_fc.dsc.data_base + 84)

#define SWIPE_ENABLE_REG					(ts->lpwg_fc.dsc.control_base + 15)
#define SWIPE_DISTANCE_REG					(ts->lpwg_fc.dsc.control_base + 16)
#define SWIPE_RATIO_THRESHOLD_REG			(ts->lpwg_fc.dsc.control_base + 17)
#define SWIPE_RATIO_CHECK_PERIOD_REG		(ts->lpwg_fc.dsc.control_base + 18)
#define SWIPE_RATIO_CHK_MIN_DISTANCE_REG	(ts->lpwg_fc.dsc.control_base + 19)
#define MIN_SWIPE_TIME_THRES_LSB_REG		(ts->lpwg_fc.dsc.control_base + 20)
#define MIN_SWIPE_TIME_THRES_MSB_REG		(ts->lpwg_fc.dsc.control_base + 21)
#define MAX_SWIPE_TIME_THRES_LSB_REG		(ts->lpwg_fc.dsc.control_base + 22)
#define MAX_SWIPE_TIME_THRES_MSB_REG		(ts->lpwg_fc.dsc.control_base + 23)
#endif

#define MAX_PRESSURE			255
#define LPWG_BLOCK_SIZE			7
#define KNOCKON_DELAY			68	/* 700ms */
#define KNOCKCODE_DELAY			25  /* 250ms */

#if defined(ENABLE_NOISE_LOG)
#define IM_LSB			(ts->analog_fc.dsc.data_base + 4)
#define IM_MSB			(ts->analog_fc.dsc.data_base + 5)
#define CURR_NOISE_STATE	(ts->analog_fc.dsc.data_base + 8)
#define CID_IM			(ts->analog_fc.dsc.data_base + 10)
#define FREQ_SCAN_IM		(ts->analog_fc.dsc.data_base + 11)
#endif

/****************************************************************************
 * Macros
 ****************************************************************************/
/* Get user-finger-data from register.
 */
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_x : _width_y
#define GET_WIDTH_MINOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_y : _width_x

#define GET_ORIENTATION(_width_y, _width_x) \
	((_width_y - _width_x) > 0) ? 0 : 1
#define GET_PRESSURE(_pressure) \
		_pressure

#if defined(ENABLE_SWIPE_MODE)
#define GET_LOW_U8_FROM_U16(_u16_data) \
	((u8)((_u16_data) & 0x00FF))
#define GET_HIGH_U8_FROM_U16(_u16_data) \
	((u8)(((_u16_data) & 0xFF00) >> 8))
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/
#if defined ( TOUCH_MODEL_C70 )
#if defined ( CONFIG_MACH_MSM8916_C70W_KR )
static const char defaultFirmware[] = "synaptics/c70/PLG455-V1.23-PR1815058-DS5.2.12.0.1013_40047197.img";
#else
static const char defaultFirmware[] = "synaptics/c70/PLG455-V1.22-PR1815058-DS5.2.12.0.1013_40047196.img";
#endif
#elif defined ( TOUCH_MODEL_Y90 )
static const char defaultFirmware[] = "synaptics/y90/PLG465-V0.01-PR1741017-DS5.2.12.0.1013-40050181.img";
#elif defined ( TOUCH_MODEL_Y70 )
static const char defaultFirmware[] = "synaptics/y70/PLG456-V1.01-PR1741017_DS5.2.12.0.1013_40047181.img";
#elif defined ( TOUCH_MODEL_C90 )
static const char defaultFirmware[] = "synaptics/c90/PLG431-V1.26_PR1741017_DS5.2.12.1013_4005019A-C90-DDIC-2CUT.img";
#elif defined ( TOUCH_MODEL_C90NAS )
static const char defaultFirmware[] = "synaptics/c90nas_spr_us/PLG465-V1.11-PR1815155-DS5.2.12.0.13-4005018B.img";
#elif defined ( TOUCH_MODEL_P1B )
static const char defaultFirmware[] = "synaptics/p1b/PLG449-V1.01-PR1797768-DS5.2.13.0.1014_40052181.img";
#elif defined ( TOUCH_MODEL_P1C )
static const char defaultFirmware[] = "synaptics/p1c/PLG465-V1.11-PR1815155-DS5.2.12.0.13-4005018B.img";
#elif defined ( TOUCH_MODEL_YG )
static const char defaultFirmware[] = "synaptics/yg/PLG521-V1.01-PR1815155-DS5.2.12.1013_40050181.img";
#else
#error "Model should be defined"
#endif

struct synaptics_ts_data *ts =NULL;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
int enable_rmi_dev = 0;
#if defined(ENABLE_SWIPE_MODE)
int get_swipe_mode = 1;
int wakeup_by_swipe = 0;
extern int lockscreen_stat;
#endif
#if defined(ENABLE_NOISE_LOG)
static unsigned long cnt = 0, im_sum=0, cns_sum=0, cid_im_sum=0, freq_scan_im_sum=0, prev_count=0;
static u16 im_aver=0, cns_aver=0, cid_im_aver=0, freq_scan_im_aver=0;
#endif

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
//mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta
extern int F54TestHandle(struct synaptics_ts_data *ts, int input, int mode, char *buf);


/****************************************************************************
* Local Function Prototypes
****************************************************************************/

/****************************************************************************
* Device Specific Function Prototypes
****************************************************************************/
static int S3320_Initialize(struct i2c_client *client);
static void S3320_Reset(struct i2c_client *client);
static int S3320_Connect(void);
static int S3320_InitRegister(struct i2c_client *client);
static int S3320_InterruptHandler(struct i2c_client *client,TouchReadData *pData);
static int S3320_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo);
static int S3320_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo);
static int S3320_UpdateFirmware(struct i2c_client *client, char *pFilename);
static int S3320_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting);
static int S3320_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen);
static int S3320_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue);


/****************************************************************************
* Local Functions
****************************************************************************/
static int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Read(client, reg, data, size);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
	
}
 
static int synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Write(client, reg, data, size);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}
 
static int synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Write_Byte(client, reg, data);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}

static int read_page_description_table(struct i2c_client *client)
{
	int ret = 0;
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	TOUCH_FUNC();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->lpwg_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++)
	{
		ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page_num);
		if( ret == TOUCH_FAIL ) {
			return TOUCH_FAIL;
		}

		for (address = DESCRIPTION_TABLE_START; address > 10;
				address -= sizeof(struct function_descriptor)) {
			ret = Touch_I2C_Read(client, address, (unsigned char *)&buffer, sizeof(buffer));
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case LPWG_CONTROL:
				ts->lpwg_fc.dsc = buffer;
				ts->lpwg_fc.function_page = page_num;
				break;
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
			default:
				break;
			}
		}
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, 0x00);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	if( ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0 || ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0 ) {
		TOUCH_ERR("failed to read page description\n");
		return TOUCH_FAIL;
	}

	TOUCH_DBG("common[%dP:0x%02x] finger[%dP:0x%02x] lpwg[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->lpwg_fc.function_page, ts->lpwg_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	return TOUCH_SUCCESS;

}

static int check_firmware_status(struct i2c_client *client)
{
	u8 device_status = 0;
	u8 flash_status = 0;

	Touch_I2C_Read_Byte(client, FLASH_STATUS_REG, &flash_status);
	Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &device_status);

	if ((device_status & (DEVICE_STATUS_FLASH_PROG|DEVICE_CRC_ERROR_MASK)) || (flash_status & 0xFF)) {
		TOUCH_ERR("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n", (u32)flash_status, (u32)device_status);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

#if defined(ENABLE_SWIPE_MODE)
static int swipe_control(struct i2c_client *client, int mode)
{
	int ret = 0;
	u8 buf = 0;

	buf = mode ? 0x01 : 0x00;

	ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, SWIPE_ENABLE_REG, buf);

	if(ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int swipe_setParam(struct i2c_client *client)
{
	int ret = 0;

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

	ret |= Touch_I2C_Write_Byte(client, SWIPE_DISTANCE_REG, 7);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_CHK_MIN_DISTANCE_REG, 1);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_THRESHOLD_REG, 200);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_CHECK_PERIOD_REG, 30);
	ret |= Touch_I2C_Write_Byte(client, MIN_SWIPE_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, MIN_SWIPE_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, MAX_SWIPE_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0xF4));
	ret |= Touch_I2C_Write_Byte(client, MAX_SWIPE_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0x01));

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

	if(ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}
#endif

static int tci_control(struct i2c_client *client, int type, u8 value)
{
	int ret = 0;
	u8 buffer[3] = {0};

	switch (type)
	{
		case REPORT_MODE_CTRL:
			ret = Touch_I2C_Read(client, INTERRUPT_ENABLE_REG, buffer, 1);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG,
					value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			ret = Touch_I2C_Read(client, FINGER_REPORT_REG, buffer, 3);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
			ret = Touch_I2C_Write(client, FINGER_REPORT_REG, buffer, 3);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			TOUCH_DBG("report mode: %d\n", value);
			break;
		case TCI_ENABLE_CTRL:
			ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TCI_ENABLE_CTRL2:
			ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_COUNT_CTRL:
			ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_COUNT_CTRL2:
			ret = synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MIN_INTERTAP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MIN_INTERTAP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MAX_INTERTAP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MAX_INTERTAP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TOUCH_SLOP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TOUCH_SLOP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_DISTANCE_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_DISTANCE_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case INTERRUPT_DELAY_CTRL:
			buffer[0] = value ? ( (KNOCKON_DELAY << 1) | 0x1 ) : 0;
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG, buffer[0]);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case INTERRUPT_DELAY_CTRL2:
			buffer[0] = value ? ( (KNOCKCODE_DELAY << 1) | 0x1 ) : 0;
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2, buffer[0]);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			break;
	}

	return TOUCH_SUCCESS;
	
}

static int sleep_control(struct i2c_client *client, int mode, int recal)
{
	int ret = 0;
	u8 curr = 0;
	u8 next = 0;

	ret = Touch_I2C_Read(client, DEVICE_CONTROL_REG, &curr, 1);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	next = (curr & 0xF8);

	if ( mode == 1 ) {
		next = next | DEVICE_CONTROL_NOSLEEP;
	} else {
		next = next | DEVICE_CONTROL_SLEEP;
	}

	ret = Touch_I2C_Write_Byte(client, DEVICE_CONTROL_REG, next);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	TOUCH_DBG("%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return TOUCH_SUCCESS;
	
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	int ret = 0;
	u8 readValue = 0;

	TOUCH_FUNC();

	/* backup interrupt enable register */
	ret = Touch_I2C_Read_Byte(client, INTERRUPT_ENABLE_REG, &readValue);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	/* disable interrupt */
	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, 0x00);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	switch(newState)
	{
		case STATE_NORMAL:
			if( ts->currState == STATE_OFF ) {
				sleep_control(client, 1, 0);
				msleep(5);
			}
			tci_control(client, TCI_ENABLE_CTRL, 0);
			tci_control(client, TCI_ENABLE_CTRL2, 0);
			tci_control(client, REPORT_MODE_CTRL, 0);
			msleep(25);
			break;
			
		case STATE_KNOCK_ON_ONLY:
			if( ts->currState == STATE_NORMAL ) {
				msleep(70);
			} else if ( ts->currState == STATE_OFF ) {
				sleep_control(client, 1, 0);
				msleep(5);
				tci_control(client, REPORT_MODE_CTRL, 0);
			}
			
			tci_control(client, TAP_COUNT_CTRL, 2);
			tci_control(client, MIN_INTERTAP_CTRL, 0);
			tci_control(client, MAX_INTERTAP_CTRL, 70);
			tci_control(client, TOUCH_SLOP_CTRL, 100);
			tci_control(client, TAP_DISTANCE_CTRL, 10);
			tci_control(client, INTERRUPT_DELAY_CTRL, 0);
			
			tci_control(client, TCI_ENABLE_CTRL, 1);
			tci_control(client, TCI_ENABLE_CTRL2, 0);
			tci_control(client, REPORT_MODE_CTRL, 1);

#if defined(ENABLE_SWIPE_MODE)
			if(ts->lpwgSetting.coverState || !get_swipe_mode) {
				swipe_control(client, 0);
			} else {
				swipe_control(client, 1);
				swipe_setParam(client);
			}
#endif
			break;
			
		case STATE_KNOCK_ON_CODE:
			if( ts->currState == STATE_NORMAL ) {
				msleep(70);
			} else if ( ts->currState == STATE_OFF ) {
				sleep_control(client, 1, 0);
				msleep(5);
				tci_control(client, REPORT_MODE_CTRL, 0);
			}
			
			tci_control(client, TAP_COUNT_CTRL, 2);
			tci_control(client, MIN_INTERTAP_CTRL, 0);
			tci_control(client, MAX_INTERTAP_CTRL, 70);
			tci_control(client, TOUCH_SLOP_CTRL, 100);
			tci_control(client, TAP_DISTANCE_CTRL, 10);
			tci_control(client, INTERRUPT_DELAY_CTRL, (u8)ts->lpwgSetting.isFirstTwoTapSame);
			
			tci_control(client, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount);
			tci_control(client, MIN_INTERTAP_CTRL2, 0);
			tci_control(client, MAX_INTERTAP_CTRL2, 70);
			tci_control(client, TOUCH_SLOP_CTRL2, 100);
			tci_control(client, TAP_DISTANCE_CTRL2, 255);
			tci_control(client, INTERRUPT_DELAY_CTRL2, 1);

			tci_control(client, TCI_ENABLE_CTRL, 1);
			tci_control(client, TCI_ENABLE_CTRL2, 1);
			tci_control(client, REPORT_MODE_CTRL, 1);

#if defined(ENABLE_SWIPE_MODE)
			if(ts->lpwgSetting.coverState || !get_swipe_mode) {
				swipe_control(client, 0);
			} else {
				swipe_control(client, 1);
				swipe_setParam(client);
			}
#endif
			break;

		case STATE_OFF:
			if( ts->currState == STATE_NORMAL ) {
				msleep(70);
			}
			tci_control(client, TCI_ENABLE_CTRL, 0);
			tci_control(client, TCI_ENABLE_CTRL2, 0);
			tci_control(client, REPORT_MODE_CTRL, 1);
			msleep(5);
			sleep_control(client, 0, 0);
			break;
			
		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			break;
			
	}

	/* restore interrupt enable register */
	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, readValue);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;	

}

static int get_object_count(struct i2c_client *client)
{
	u8 object_num = 0;
	u8 buf[2] = {0};
	u16 object_attention_data = 0;
	u16 i = 0;
	int ret = 0;

	ret = Touch_I2C_Read(client, OBJECT_ATTENTION_REG, buf, 2);
	if( ret == TOUCH_FAIL ) {
		TOUCH_WARN("failed to read finger number\n");
	}
	
	object_attention_data = (((u16)((buf[1] << 8)  & 0xFF00)  | (u16)((buf[0]) & 0xFF)));

	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if (object_attention_data & (0x1 << i)) {
			object_num = i + 1;
		}
	}

	return object_num;
}

/****************************************************************************
* Device Specific Functions
****************************************************************************/
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert)
		rmidev_fhandler.exp_fn = rmidev_fn;
	else
		rmidev_fhandler.exp_fn = NULL;

	return TOUCH_SUCCESS;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(client, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int i = 0;
	u8 buffer[7] = {0};

	ret = Touch_I2C_Read(client, FINGER_REPORT_REG, buffer, 3);
	ret += sprintf(buf+ret, "report_mode [%s]\n",
		(buffer[2] & 0x3) == 0x2 ? "WAKEUP_ONLY" : "NORMAL");
	ret = Touch_I2C_Read(client, WAKEUP_GESTURE_ENABLE_REG, buffer, 1);
	ret += sprintf(buf+ret, "wakeup_gesture [%d]\n\n", buffer[0]);

	for (i = 0; i < 2; i++) {
		synaptics_ts_page_data_read(client, LPWG_PAGE,
			LPWG_TAPCOUNT_REG + (i * LPWG_BLOCK_SIZE), 7, buffer);
		ret += sprintf(buf+ret, "TCI - %d\n", i+1);
		ret += sprintf(buf+ret, "TCI [%s]\n",
			(buffer[0] & 0x1) == 1 ? "enabled" : "disabled");
		ret += sprintf(buf+ret, "Tap Count [%d]\n",
			(buffer[0] & 0xf8) >> 3);
		ret += sprintf(buf+ret, "Min InterTap [%d]\n", buffer[1]);
		ret += sprintf(buf+ret, "Max InterTap [%d]\n", buffer[2]);
		ret += sprintf(buf+ret, "Touch Slop [%d]\n", buffer[3]);
		ret += sprintf(buf+ret, "Tap Distance [%d]\n", buffer[4]);
		ret += sprintf(buf+ret, "Interrupt Delay [%d]\n\n", buffer[6]);
	}

	return ret;
	
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %d %d %d %d ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page,
			reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		TOUCH_DBG("page[%d] reg[%d] offset[%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DBG("Usage\n");
		TOUCH_DBG("Write page reg offset value\n");
		TOUCH_DBG("Read page reg offset\n");
	}
	return count;
	
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	u8 object_report_enable_reg;
	u8 temp[8];

	int ret = 0;
	int i;

	ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg,sizeof(object_report_enable_reg));
	if( ret < 0 ) {
		return ret;
	}

	for (i = 0; i < 8; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	ret = sprintf(buf,
		"\n======= read object_report_enable register =======\n");
	ret += sprintf(buf+ret,
		" Addr Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" 0x%02X %4d %4d %4d %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG, temp[7], temp[6],
		temp[5], temp[4], temp[3], temp[2], temp[1], temp[0],
		object_report_enable_reg);
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit3  : [U]nclassified Object -> %7s\n",
		temp[3] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit4  : [H]overing Finger -> %7s\n",
		temp[4] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit5  : [G]loved Finger -> %7s\n",
		temp[5] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit6  : [N]arrow Object Swipe -> %7s\n",
		temp[6] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit7  : Hand[E]dge  -> %7s\n",
		temp[7] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		"==================================================\n\n");
	
	return ret;
	
}

static ssize_t store_object_report(struct i2c_client *client,
	const char *buf, size_t count)
{
	int ret = 0;
	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		TOUCH_DBG("<writing object_report guide>\n");
		TOUCH_DBG("echo [select] [value] > object_report\n");
		TOUCH_DBG("select: [F]inger, [S]tylus, [P]alm,"
			" [U]nclassified Object, [H]overing Finger,"
			" [G]loved Finger, [N]arrow Object Swipe,"
			" Hand[E]dge\n");
		TOUCH_DBG("select length: 1~8, value: 0~1\n");
		TOUCH_DBG("ex) echo F 1 > object_report        "
			" (enable [F]inger)\n");
		TOUCH_DBG("ex) echo s 1 > object_report        "
			" (enable [S]tylus)\n");
		TOUCH_DBG("ex) echo P 0 > object_report        "
			" (disable [P]alm)\n");
		TOUCH_DBG("ex) echo u 0 > object_report        "
			" (disable [U]nclassified Object)\n");
		TOUCH_DBG("ex) echo HgNe 1 > object_report     "
			" (enable [H]overing Finger, [G]loved Finger,"
			" [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_DBG("ex) echo eNGh 1 > object_report     "
			" (enable Hand[E]dge, [N]arrow Object Swipe,"
			" [G]loved Finger, [H]overing Finger)\n");
		TOUCH_DBG("ex) echo uPsF 0 > object_report     "
			" (disable [U]nclassified Object, [P]alm,"
			" [S]tylus, [F]inger)\n");
		TOUCH_DBG("ex) echo HguP 0 > object_report     "
			" (disable [H]overing Finger, [G]loved Finger,"
			" [U]nclassified Object, [P]alm)\n");
		TOUCH_DBG("ex) echo HFnuPSfe 1 > object_report "
			" (enable all object)\n");
		TOUCH_DBG("ex) echo enghupsf 0 > object_report "
			" (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f': /* (F)inger */
				bit_select |= (0x01 << 0);
				break;
			case 'S': case 's': /* (S)tylus */
				bit_select |= (0x01 << 1);
				break;
			case 'P': case 'p': /* (P)alm */
				bit_select |= (0x01 << 2);
				break;
			case 'U': case 'u': /* (U)nclassified Object */
				bit_select |= (0x01 << 3);
				break;
			case 'H': case 'h': /* (H)overing Filter */
				bit_select |= (0x01 << 4);
				break;
			case 'G': case 'g': /* (G)loved Finger */
				bit_select |= (0x01 << 5);
				break;
			case 'N': case 'n': /* (N)arrow Ojbect Swipe */
				bit_select |= (0x01 << 6);
				break;
			case 'E': case 'e': /* Hand (E)dge */
				bit_select |= (0x01 << 7);
				break;
			default:
				break;
			}
		}

		ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg_old, sizeof(object_report_enable_reg_old));
		if( ret < 0 ) {
			return count;
		}

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		ret = Touch_I2C_Write_Byte(client, OBJECT_REPORT_ENABLE_REG, object_report_enable_reg_new);
		if( ret < 0 ) {
			return count;
		}
		
	}

	return count;
	
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", enable_rmi_dev);

	return ret;
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;
	int value = 0;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_DBG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	TOUCH_DBG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			ret = rmidev_fhandler.exp_fn->init(ts);
			if( ret < 0 ) {
				TOUCH_ERR("fail to enable_rmi_dev\n");
				return count;
			}
			
			rmidev_fhandler.initialized = true;
		}
	}
	else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}

	return count;

}

#if defined(ENABLE_SWIPE_MODE)
static ssize_t show_swipe_mode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", get_swipe_mode);

	return ret;
}

static ssize_t store_swipe_mode(struct i2c_client *client,
	const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	get_swipe_mode = value;

	return count;
}
#endif

#if defined(ENABLE_NOISE_LOG)
static ssize_t show_ts_noise(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "Test Count : %lu\n", cnt);
	ret += sprintf(buf+ret, "Current Noise State : %d\n", cns_aver);
	ret += sprintf(buf+ret, "Interference Metric : %d\n", im_aver);
	ret += sprintf(buf+ret, "CID IM : %d\n", cid_im_aver);
	ret += sprintf(buf+ret, "Freq Scan IM : %d\n", freq_scan_im_aver);

	TOUCH_LOG("Aver: CNS[%5d]   IM[%5d]   CID_IM[%5d]    FREQ_SCAN_IM[%5d] (cnt:%lu)\n",
		cns_aver, im_aver, cid_im_aver, freq_scan_im_aver, cnt);

	return ret;
}

static ssize_t store_ts_noise(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->noiseState.check_noise_menu == MENU_OUT) && (value == MENU_ENTER)) {
		ts->noiseState.check_noise_menu = MENU_ENTER;
	} else if ((ts->noiseState.check_noise_menu == MENU_ENTER) && (value == MENU_OUT)) {
		ts->noiseState.check_noise_menu = MENU_OUT;
	} else {
		TOUCH_LOG("Already entered Check Noise menu .\n");
		TOUCH_LOG("check_noise_menu:%d, value:%d \n",
			ts->noiseState.check_noise_menu, value);
		return count;
	}

	TOUCH_LOG("Check Noise = %s\n",
		(ts->noiseState.check_noise_menu == MENU_OUT) ? "MENU_OUT" : "MENU_ENTER");

	return count;
}

static ssize_t show_ts_noise_log_enable(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "%d\n", ts->noiseState.noise_log_flag);
	TOUCH_LOG("ts noise log flag = %s\n",
		(ts->noiseState.noise_log_flag == TS_NOISE_LOG_DISABLE) ? "TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");

	return ret;
}

static ssize_t store_ts_noise_log_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	if ((ts->noiseState.noise_log_flag == TS_NOISE_LOG_DISABLE) && (value == TS_NOISE_LOG_ENABLE)) {
		ts->noiseState.noise_log_flag = TS_NOISE_LOG_ENABLE;
	} else if ((ts->noiseState.noise_log_flag == TS_NOISE_LOG_ENABLE) && (value == TS_NOISE_LOG_DISABLE)) {
		ts->noiseState.noise_log_flag = TS_NOISE_LOG_DISABLE;
	} else {
		TOUCH_LOG("Already Enable TS Noise Log.\n");
		TOUCH_LOG("ts_noise_log_flag:%d, value:%d \n",
			ts->noiseState.noise_log_flag, value);
		return count;
	}

	TOUCH_LOG("ts noise log flag = %s\n",
		(ts->noiseState.noise_log_flag == TS_NOISE_LOG_DISABLE) ? "TS_NOISE_LOG_DISABLE" : "TS_NOISE_LOG_ENABLE");

	return count;
}
#endif

static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);
#if defined(ENABLE_SWIPE_MODE)
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR, show_swipe_mode, store_swipe_mode);
#endif
#if defined(ENABLE_NOISE_LOG)
static LGE_TOUCH_ATTR(ts_noise, S_IRUGO | S_IWUSR, show_ts_noise, store_ts_noise);
static LGE_TOUCH_ATTR(ts_noise_log_enable, S_IRUGO | S_IWUSR, show_ts_noise_log_enable, store_ts_noise_log_enable);
#endif


static struct attribute *S3320_attribute_list[] = {
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
#if defined(ENABLE_SWIPE_MODE)
	&lge_touch_attr_swipe_mode.attr,
#endif
#if defined(ENABLE_NOISE_LOG)
	&lge_touch_attr_ts_noise.attr,
	&lge_touch_attr_ts_noise_log_enable.attr,
#endif
	NULL,
};

static int S3320_Initialize(struct i2c_client *client)
{
	int ret = 0;
	
	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if( ts == NULL ) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;

	/* read initial page description */
	ret = read_page_description_table(client);
	if( ret == TOUCH_FAIL ) {
		if( check_firmware_status(client) == TOUCH_FAIL ) {
			int cnt = 0;

			do {
				ret = S3320_UpdateFirmware(client, NULL);
				cnt++;
			} while(ret == TOUCH_FAIL && cnt < 3);

			if( ret == TOUCH_FAIL ) {
				devm_kfree(&client->dev, ts);
				return TOUCH_FAIL;
			}
		}
	}

	return TOUCH_SUCCESS;
	
}

static void S3320_Reset(struct i2c_client *client)
{
#if defined(ENABLE_SWIPE_MODE)
	if(wakeup_by_swipe)
	{
		wakeup_by_swipe = 0;
		TOUCH_LOG("swipe mode!! no reset\n");
	}
	else
#endif
	{
		/* IMPLEMENT : Device Reset function */
		if( client != NULL ) {
			int ret = 0;
			synaptics_ts_page_data_write_byte(client, ANALOG_PAGE, DYNAMIC_SENSING_CONTROL_REG, SENSING_CONTROL_120HZ);
			if( ret == TOUCH_FAIL ) {
				TOUCH_WARN("failed to change sensing control to 120Hz\n");
			}
		}

		TouchResetCtrl(0);
		msleep(10);
		TouchResetCtrl(1);
		msleep(150);

		TOUCH_LOG("Device was reset\n");
	}
}


static int S3320_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

static int S3320_InitRegister(struct i2c_client *client)
{
	int ret = 0;
	u8 reg = 0;
	u8 data[2] = {0};

	reg = DEVICE_CONTROL_REG;
	data[0] = DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	reg = INTERRUPT_ENABLE_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	reg = INTERRUPT_ENABLE_REG;
	data[0] |= INTERRUPT_MASK_ABS0;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	reg = FINGER_REPORT_REG;
	data[0] = 0;
	ret = Touch_I2C_Write(client, reg, data, 2);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	reg = INTERRUPT_STATUS_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ts->currState = STATE_NORMAL;

	return TOUCH_SUCCESS;
	
}

#if defined(ENABLE_NOISE_LOG)
static int synaptics_ts_noise_log(struct i2c_client *client, int curr_count)
{
	u8 buffer[2] = {0}, cns = 0;
	u16 im = 0, cid_im = 0, freq_scan_im = 0;
	int ret = 0;

	if (prev_count == 0 && curr_count) {
		cnt = im_sum = cns_sum = cid_im_sum = freq_scan_im_sum = 0;
		im_aver = cns_aver = freq_scan_im_aver = 0;
	}

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, ANALOG_PAGE);

	ret |= Touch_I2C_Read(client, IM_LSB, &buffer[0], 1);
	ret |= Touch_I2C_Read(client, IM_MSB, &buffer[1], 1);
	im = (buffer[1] << 8) | buffer[0];
	im_sum += im;

	ret |= Touch_I2C_Read(client, CURR_NOISE_STATE, &cns, 1);
	cns_sum += cns;

	ret |= Touch_I2C_Read(client, CID_IM, buffer, 2);
	cid_im = (buffer[1] << 8) | buffer[0];
	cid_im_sum += cid_im;

	ret |= Touch_I2C_Read(client, FREQ_SCAN_IM, buffer, 2);
	freq_scan_im = (buffer[1] << 8) | buffer[0];
	freq_scan_im_sum += freq_scan_im;

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

	if(ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	cnt++;

	if (prev_count != curr_count)
		TOUCH_LOG("curr: CNS[%5d]   IM[%5d]   CID_IM[%5d]    FREQ_SCAN_IM[%5d]\n",
			cns, im, cid_im, freq_scan_im);

	if ( ( prev_count && curr_count == 0 )
		|| ( cns_sum >= ULONG_MAX || im_sum >= ULONG_MAX || cid_im_sum >= ULONG_MAX
		     || freq_scan_im_sum >= ULONG_MAX || cnt >= ULONG_MAX ) ) {
			im_aver = im_sum/cnt;
			cns_aver = cns_sum/cnt;
			freq_scan_im_aver = freq_scan_im_sum/cnt;

		TOUCH_LOG("Aver: CNS[%5d]   IM[%5d]   CID_IM[%5d]    FREQ_SCAN_IM[%5d] (cnt:%lu)\n",
				cns_aver, im_aver, cid_im_aver, freq_scan_im_aver, cnt);

			if(curr_count == 0)
				prev_count = 0;
	} else
		prev_count = curr_count;

	return TOUCH_SUCCESS;
}
#endif

static int S3320_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	int result = 0;
	u8 i = 0;
	u8 lpwgTapCount = 0;
	u8 readFingerCnt = 0;
	u8 regDevStatus = 0;
	u8 regIntStatus = 0;
	u8 regFingerData[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
	u8 buffer[12][4] = { {0} };
#if defined(ENABLE_TCI_DEBUG)
	u8 lpwgFailCount = 0;
	u8 lpwgFailIndex = 0;
	u8 FailBuffer[10] = {0};
#endif

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	result = Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &regDevStatus);
	if( result == TOUCH_FAIL ) {
		TOUCH_WARN("failed to read device status\n");
	}

	result = Touch_I2C_Read_Byte(client, INTERRUPT_STATUS_REG, &regIntStatus);
	if( result == TOUCH_FAIL ) {
		TOUCH_WARN("failed to read interrupt status\n");
		regIntStatus = (1U << 7);
	}

	if (regIntStatus == INTERRUPT_MASK_NONE) {
		return TOUCH_SUCCESS;
	} else if (regIntStatus & INTERRUPT_MASK_ABS0 ) {
		readFingerCnt = get_object_count(client);
		if( readFingerCnt == 0 ) {
			readFingerCnt = MAX_NUM_OF_FINGERS;
		}

		if (readFingerCnt > 0) {
			int ret = 0;
			ret = Touch_I2C_Read(client, FINGER_DATA_REG_START,			
				(u8 *)regFingerData, (NUM_OF_EACH_FINGER_DATA_REG * readFingerCnt));

			if( ret == TOUCH_SUCCESS ) {
				pData->type = DATA_FINGER;
				for (i = 0; i < readFingerCnt; i++) {
					if (regFingerData[i][0]) {
						pFingerData = &pData->fingerData[pData->count];
						pFingerData->id = i;
						pFingerData->type = regFingerData[i][REG_OBJECT];
						pFingerData->x = GET_X_POSITION( regFingerData[i][REG_X_MSB], regFingerData[i][REG_X_LSB] );
						pFingerData->y = GET_Y_POSITION( regFingerData[i][REG_Y_MSB], regFingerData[i][REG_Y_LSB] );
						pFingerData->width_major = GET_WIDTH_MAJOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
						pFingerData->width_minor = GET_WIDTH_MINOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
						pFingerData->orientation = GET_ORIENTATION( regFingerData[i][REG_WY], regFingerData[i][REG_WX] );
						pFingerData->pressure = GET_PRESSURE( regFingerData[i][REG_Z] );

						pData->count++;
					}
				}
			}
			else
			{
				pData->type = DATA_UNKNOWN;
				TOUCH_WARN("failed to read I2C\n");
			}
		}
		else
		{
			pData->type = DATA_FINGER;
		}

#if defined(ENABLE_NOISE_LOG)
		if ((ts->noiseState.noise_log_flag == TS_NOISE_LOG_ENABLE)
			|| (ts->noiseState.check_noise_menu == MENU_ENTER))
			synaptics_ts_noise_log(client, pData->count);
#endif

	} else if (regIntStatus & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;

		synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status);

		if (status & 0x1) { /* TCI-1 : Double-Tap */
			TOUCH_LOG( "Knock-on Detected\n" );
			pData->type = DATA_KNOCK_ON;

		#if defined(ENABLE_TCI_DEBUG)
			synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI1_FAIL_COUNT_REG, 1, &lpwgFailCount);

			if(lpwgFailCount) {
				synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI1_FAIL_INDEX_REG, 1, &lpwgFailIndex);
				synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI1_FAIL_BUFFER_REG, 10, FailBuffer);
				TOUCH_LOG( "Knock On Fail Index: %d\n", lpwgFailIndex);

				for(i=0; i< lpwgFailIndex; i++)
					TOUCH_LOG( "Knock On Fail Reason[%d]: %x\n", i, FailBuffer[i]);
			}
		#endif
		} else if (status & 0x2) {  /* TCI-2 : Multi-Tap */
			TOUCH_LOG( "Knock-code Detected\n" );
			pData->type = DATA_KNOCK_CODE;

			synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DATA_REG, 4*ts->lpwgSetting.tapCount, &buffer[0][0]);

			for (i = 0; i < ts->lpwgSetting.tapCount; i++) {
				pData->knockData[i].x
					= GET_X_POSITION(buffer[i][1], buffer[i][0]);
				pData->knockData[i].y
					= GET_Y_POSITION(buffer[i][3], buffer[i][2]);
			}
			pData->count = ts->lpwgSetting.tapCount;

		#if defined(ENABLE_TCI_DEBUG)
			synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI2_FAIL_COUNT_REG, 1, &lpwgFailCount);

			if(lpwgFailCount) {
				synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI2_FAIL_INDEX_REG, 1, &lpwgFailIndex);
				synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TCI2_FAIL_BUFFER_REG, 10, FailBuffer);
				TOUCH_LOG( "Knock Code Fail Index: %d\n", lpwgFailIndex);

				for(i=0; i< lpwgFailIndex; i++)
					TOUCH_LOG( "Knock Code Fail Reason[%d]: %x\n", i, FailBuffer[i]);
			}
		#endif

#if defined(ENABLE_SWIPE_MODE)
		} else if (status & 0x4) {
			u16 swipe_start_x = 0;
			u16 swipe_start_y = 0;
			u16 swipe_end_x = 0;
			u16 swipe_end_y = 0;
			u16 swipe_time = 0;
			u8 buf_lsb = 0;
			u8 buf_msb = 0;
			u8 fail_reason = 0;

			TOUCH_LOG( "Swipe mode Detected\n" );
			Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

			Touch_I2C_Read_Byte(client, SWIPE_FAIL_REASON_REG, &fail_reason);
			TOUCH_LOG( "Swipe Fail reason : 0x%x\n", fail_reason );

			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_MSB_REG, &buf_msb);
			swipe_start_x = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_MSB_REG, &buf_msb);
			swipe_start_y = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_MSB_REG, &buf_msb);
			swipe_end_x = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_MSB_REG, &buf_msb);
			swipe_end_y = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_TIME_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_TIME_MSB_REG, &buf_msb);
			swipe_time = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

			TOUCH_LOG("LPWG Swipe Gesture: start(%4d,%4d) end(%4d,%4d)\n", swipe_start_x, swipe_start_y, swipe_end_x, swipe_end_y);

			if(lockscreen_stat){
				wakeup_by_swipe = 1;
				swipe_control(client, 0);
			}

			pData->count = 1;
			pData->knockData[0].x = swipe_end_x;
			pData->knockData[0].y = swipe_end_y;
			pData->type = DATA_SWIPE;
#endif
		} else if (status == 0) {
			TOUCH_LOG( "OverTap Knock-code Detected\n" );
			pData->type = DATA_KNOCK_CODE;

			synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_OVER_TAPCOUNT, 1, &lpwgTapCount);
			TOUCH_LOG( "Set Tap cnt : %d, Detected Tap Count: %d\n", ts->lpwgSetting.tapCount, lpwgTapCount );

			pData->knockData[0].x = 1;
			pData->knockData[0].y = 1;
			pData->knockData[1].x = -1;
			pData->knockData[1].y = -1;
			pData->count = ts->lpwgSetting.tapCount;
		} else {
			TOUCH_ERR( "Unexpected LPWG Interrupt Status ( 0x%X )\n", status);
		}

	} else {
		TOUCH_ERR( "Unexpected Interrupt Status ( 0x%X )\n", regIntStatus );
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}


//=================================================================
// module maker : ELK(0), Suntel(1), Tovis(2), Innotek(3), JDI(4), LGD(5)
// 
//
//
//
//=================================================================
static int S3320_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	u8 readData[FW_VER_INFO_NUM] = {0};
	
	TOUCH_FUNC();

	ret = Touch_I2C_Read(client, FLASH_CONFIG_ID_REG, readData, FW_VER_INFO_NUM);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", ( readData[0] >> 4 ) & 0xF, readData[0] & 0xF, ( readData[1] >> 4 ) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", readData[2] & 0xF, readData[1] & 0xF, ( readData[2] >> 4 ) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", readData[3] & 0x7F, ( ( readData[3] & 0x80 ) == 0x80 ) ? "Official Release" : "Test Release");
	
	pFwInfo->moduleMakerID = ( readData[0] >> 4 ) & 0x0F;
	pFwInfo->moduleVersion = readData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = ( readData[3] >> 7 ) & 0x01;
	pFwInfo->version = readData[3] & 0x7F;

	return TOUCH_SUCCESS;
	
}

static int S3320_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	u8 *pReadData = NULL;

	TOUCH_FUNC();


	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);
	pReadData = &pBin[0x16d00];

	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", ( pReadData[0] >> 4 ) & 0xF, pReadData[0] & 0xF, ( pReadData[1] >> 4 ) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", pReadData[2] & 0xF, pReadData[1] & 0xF, ( pReadData[2] >> 4 ) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", pReadData[3] & 0x7F, ( ( pReadData[3] & 0x80 ) == 0x80 ) ? "Official Release" : "Test Release");
	
	pFwInfo->moduleMakerID = ( pReadData[0] >> 4 ) & 0x0F;
	pFwInfo->moduleVersion = pReadData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = ( pReadData[3] >> 7 ) & 0x01;
	pFwInfo->version = pReadData[3] & 0x7F;

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;
		
}


static int S3320_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;
	
	TOUCH_FUNC();
	
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	ret = synaptics_ts_page_data_write_byte(client, ANALOG_PAGE, DYNAMIC_SENSING_CONTROL_REG, SENSING_CONTROL_120HZ);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to change sensing control to 120Hz\n");
		return TOUCH_FAIL;
	}

	FirmwareUpgrade(ts, pFwFilename);

	/* read changed page description */
	ret = read_page_description_table(client);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = check_firmware_status(client);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
	
}

static int S3320_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;
	
	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

#if defined(ENABLE_SWIPE_MODE)
	if(ts->lpwgSetting.coverState) {
		swipe_control(client, 0);
	}
#endif

	if( ts->currState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}

	if( ( newState < STATE_NORMAL ) && ( newState > STATE_KNOCK_ON_CODE ) ) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	ret = lpwg_control(client, newState);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	if( ret == TOUCH_SUCCESS ) {
		ts->currState = newState;
	}

	switch( newState )
	{
		case STATE_NORMAL:
			TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		default:
			TOUCH_LOG("impossilbe state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}


static int S3320_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int ret = 0;
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	int full_raw_cap;
	int trx_to_trx;
	int high_resistance;

	TOUCH_FUNC();

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, ANALOG_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	full_raw_cap = F54TestHandle(ts, F54_FULL_RAW_CAP, 0, pBuf);
	high_resistance = F54TestHandle(ts, F54_HIGH_RESISTANCE, 0, pBuf);
	trx_to_trx = F54TestHandle(ts, F54_TRX_TO_TRX, 0, pBuf);

	*pRawStatus = full_raw_cap;

	if (high_resistance == TOUCH_SUCCESS && trx_to_trx == TOUCH_SUCCESS) {
		*pChannelStatus = TOUCH_SUCCESS;
	} else {
		*pChannelStatus = TOUCH_FAIL;
	}

//	dataLen += sprintf(pBuf, "%s", "========= Additional Information ( Begin ) =========\n");
//	dataLen += sprintf(pBuf+dataLen, "%s", "========= Additional Information ( End ) =========\n");

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
	
}

static int S3320_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;
	
	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}

TouchDeviceSpecificFunction S3320_Func = {

	.Initialize = S3320_Initialize,
	.Reset = S3320_Reset,
	.Connect = S3320_Connect,
	.InitRegister = S3320_InitRegister,
	.InterruptHandler = S3320_InterruptHandler,
	.ReadIcFirmwareInfo = S3320_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = S3320_GetBinFirmwareInfo,
	.UpdateFirmware = S3320_UpdateFirmware,
	.SetLpwgMode = S3320_SetLpwgMode,
	.DoSelfDiagnosis = S3320_DoSelfDiagnosis,
	.AccessRegister = S3320_AccessRegister,
	.device_attribute_list = S3320_attribute_list,
	
};


/* End Of File */


