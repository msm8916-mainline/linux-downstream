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
 *    File  	: lgtp_device_dummy.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[MIT200]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_mit200.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define MIT_EVENT_PKT_SZ					0x0F
#define MIT_INPUT_EVENT						0x10

/* Event types */
#define MIT_LOG_EVENT						0xD
#define MIT_LPWG_EVENT						0xE
#define MIT_ERROR_EVENT						0xF
#define MIT_TOUCH_KEY_EVENT					0x40

#define MIT_FW_VERSION						0xC2
#define MIT_FW_PRODUCT						0xE0

/* MIT 200 LPWG Registers */
#define MIT_LPWG_IDLE_REPORTRATE_REG     	0x60
#define MIT_LPWG_ACTIVE_REPORTRATE_REG   	0x61
#define MIT_LPWG_SENSITIVITY_REG         	0x62
#define MIT_LPWG_ACTIVE_AREA_REG         	0x63

#define MIT_LPWG_TCI_ENABLE_REG          	0x70
#define MIT_LPWG_TOUCH_SLOP_REG          	0x71
#define MIT_LPWG_TAP_MIN_DISTANCE_REG    	0x72
#define MIT_LPWG_TAP_MAX_DISTANCE_REG    	0x73
#define MIT_LPWG_MIN_INTERTAP_REG        	0x74
#define MIT_LPWG_MAX_INTERTAP_REG        	0x76
#define MIT_LPWG_TAP_COUNT_REG           	0x78
#define MIT_LPWG_INTERRUPT_DELAY_REG     	0x79

#define MIT_LPWG_TCI_ENABLE_REG2         	0x80
#define MIT_LPWG_TOUCH_SLOP_REG2         	0x81
#define MIT_LPWG_TAP_MIN_DISTANCE_REG2   	0x82
#define MIT_LPWG_TAP_MAX_DISTANCE_REG2   	0x83
#define MIT_LPWG_MIN_INTERTAP_REG2       	0x84
#define MIT_LPWG_MAX_INTERTAP_REG2       	0x86
#define MIT_LPWG_TAP_COUNT_REG2          	0x88
#define MIT_LPWG_INTERRUPT_DELAY_REG2    	0x89

#define MIT_LPWG_STORE_INFO_REG          	0x8F
#define MIT_LPWG_START_REG               	0x90
#define MIT_LPWG_PANEL_DEBUG_REG        	0x91
#define MIT_LPWG_FAIL_REASON_REG        	0x92

/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
#if defined( TOUCH_MODEL_C50)
static const char defaultFirmware[] = "melfas/mit200/c50n_global_com/L0M45P1CY5_01_02.fw";
#endif
#if defined( TOUCH_MODEL_Y50)
static const char defaultFirmware[] = "melfas/yk/L0M45P1CY5_30_21.fw";
#endif

struct melfas_ts_data *ts = NULL;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/


/****************************************************************************
* Device Specific Functions
****************************************************************************/
static int tci_control(struct i2c_client *client, int type, u16 value)
{
	u8 cmd[2] = {MIT_REGH_CMD, 0x00};
	u8 buf[2] = {value, 0x00};
	int dataLen = 1;
	int ret = 0;

	switch (type) {
	case IDLE_REPORTRATE_CTRL:
		cmd[1] = MIT_LPWG_ACTIVE_REPORTRATE_REG;
		break;
	case ACTIVE_REPORTRATE_CTRL:
		cmd[1] = MIT_LPWG_IDLE_REPORTRATE_REG;
		break;
	case SENSITIVITY_CTRL:
		cmd[1] = MIT_LPWG_SENSITIVITY_REG;
		break;
	case TCI_ENABLE_CTRL:
		cmd[1] = MIT_LPWG_TCI_ENABLE_REG;
		break;
	case TOUCH_SLOP_CTRL:
		cmd[1] = MIT_LPWG_TOUCH_SLOP_REG;
		break;
	case TAP_MIN_DISTANCE_CTRL:
		cmd[1] = MIT_LPWG_TAP_MIN_DISTANCE_REG;
		break;
	case TAP_MAX_DISTANCE_CTRL:
		cmd[1] = MIT_LPWG_TAP_MAX_DISTANCE_REG;
		break;
	case MIN_INTERTAP_CTRL:
		cmd[1] = MIT_LPWG_MIN_INTERTAP_REG;
		buf[0] = (value >> 8);
		buf[1] = (value & 0xFF);
		dataLen = 2;
		break;
	case MAX_INTERTAP_CTRL:
		cmd[1] = MIT_LPWG_MAX_INTERTAP_REG;
		buf[0] = (value >> 8);
		buf[1] = (value & 0xFF);
		dataLen = 2;
		break;
	case TAP_COUNT_CTRL:
		cmd[1] = MIT_LPWG_TAP_COUNT_REG;
		break;
	case INTERRUPT_DELAY_CTRL:
		cmd[1] = MIT_LPWG_INTERRUPT_DELAY_REG;
		buf[0] = ((value ? KNOCKON_DELAY : 0) >> 8);
		buf[1] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
		dataLen = 2;
		break;
	case TCI_ENABLE_CTRL2:
		cmd[1] = MIT_LPWG_TCI_ENABLE_REG2;
		break;
	case TOUCH_SLOP_CTRL2:
		cmd[1] = MIT_LPWG_TOUCH_SLOP_REG2;
		break;
	case TAP_MIN_DISTANCE_CTRL2:
		cmd[1] = MIT_LPWG_TAP_MIN_DISTANCE_REG2;
		break;
	case TAP_MAX_DISTANCE_CTRL2:
		cmd[1] = MIT_LPWG_TAP_MAX_DISTANCE_REG2;
		break;
	case MIN_INTERTAP_CTRL2:
		cmd[1] = MIT_LPWG_MIN_INTERTAP_REG2;
		buf[0] = (value >> 8);
		buf[1] = (value & 0xFF);
		dataLen = 2;
		break;
	case MAX_INTERTAP_CTRL2:
		cmd[1] = MIT_LPWG_MAX_INTERTAP_REG2;
		buf[0] = (value >> 8);
		buf[1] = (value & 0xFF);
		dataLen = 2;
		break;
	case TAP_COUNT_CTRL2:
		cmd[1] = MIT_LPWG_TAP_COUNT_REG2;
		break;
	case INTERRUPT_DELAY_CTRL2:
		cmd[1] = MIT_LPWG_INTERRUPT_DELAY_REG2;
		buf[0] = ((value ? KNOCKON_DELAY : 0) >> 8);
		buf[1] = ((value ? KNOCKON_DELAY : 0) & 0xFF);
		dataLen = 2;
		break;
	case LPWG_STORE_INFO_CTRL:
		cmd[1] = MIT_LPWG_STORE_INFO_REG;
		break;
	case LPWG_START_CTRL:
		cmd[1] = MIT_LPWG_START_REG;
		break;
	case LPWG_PANEL_DEBUG_CTRL:
		cmd[1] = MIT_LPWG_PANEL_DEBUG_REG;
		break;
	case LPWG_FAIL_REASON_CTRL:
		cmd[1] = MIT_LPWG_FAIL_REASON_REG;
		break;
	default:
		break;
	}

	ret = MIT200_I2C_Write(client, cmd, 2, buf, dataLen);
	if (ret < 0) {
		TOUCH_ERR("fail tci control\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}


static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	TOUCH_FUNC();

	switch (newState) {
	case STATE_NORMAL:
		tci_control(client, TCI_ENABLE_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);
		tci_control(client, LPWG_START_CTRL, 0);
		break;

	case STATE_KNOCK_ON_ONLY:
		tci_control(client, IDLE_REPORTRATE_CTRL, 20);
		tci_control(client, ACTIVE_REPORTRATE_CTRL, 40);
		tci_control(client, SENSITIVITY_CTRL, 30);
		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TOUCH_SLOP_CTRL, 10);
		tci_control(client, TAP_MIN_DISTANCE_CTRL, 0);
		tci_control(client, TAP_MAX_DISTANCE_CTRL, 10);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 700);
		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, INTERRUPT_DELAY_CTRL, 0);
		tci_control(client, TCI_ENABLE_CTRL2, 0);

		tci_control(client, LPWG_START_CTRL, 1);
		break;
		
	case STATE_KNOCK_ON_CODE:
		tci_control(client, IDLE_REPORTRATE_CTRL, 20);
		tci_control(client, ACTIVE_REPORTRATE_CTRL, 40);
		tci_control(client, SENSITIVITY_CTRL, 30);
		tci_control(client, TCI_ENABLE_CTRL, 1);
		tci_control(client, TOUCH_SLOP_CTRL, 10);
		tci_control(client, TAP_MIN_DISTANCE_CTRL, 0);
		tci_control(client, TAP_MAX_DISTANCE_CTRL, 10);
		tci_control(client, MIN_INTERTAP_CTRL, 0);
		tci_control(client, MAX_INTERTAP_CTRL, 700);
		tci_control(client, TAP_COUNT_CTRL, 2);
		tci_control(client, INTERRUPT_DELAY_CTRL, 0);

		tci_control(client, TCI_ENABLE_CTRL2, 1);
		tci_control(client, TOUCH_SLOP_CTRL2, 10);
		tci_control(client, TAP_MIN_DISTANCE_CTRL2, 0);
		tci_control(client, TAP_MAX_DISTANCE_CTRL2, 255);
		tci_control(client, MIN_INTERTAP_CTRL2, 0);
		tci_control(client, MAX_INTERTAP_CTRL2, 700);
		tci_control(client, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount);
		tci_control(client, INTERRUPT_DELAY_CTRL2, 0);

		tci_control(client, LPWG_START_CTRL, 1);
		break;

	case STATE_OFF:
		break;
		
	default:
		TOUCH_ERR("invalid touch state ( %d )\n", newState);
		break;
	}

	return TOUCH_SUCCESS;
}

static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret += sprintf(buf+ret, "%s\n", "Dummy Device");
	
	return ret;
}

static LGE_TOUCH_ATTR(device_name, S_IRUGO | S_IWUSR, show_device_name, NULL);

static struct attribute *MIT200_attribute_list[] = {
	&lge_touch_attr_device_name.attr,
	NULL,
};



static int MIT200_Initialize(struct i2c_client *client)
{
	#if defined( TOUCH_MODEL_C50)
	struct regulator *vdd = NULL;
	#endif
	
	int ret = 0;

	TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;

	#if defined( TOUCH_MODEL_C50)
	
	vdd = regulator_get(&client->dev, "vdd");
	if (IS_ERR(vdd)) {
		ret = PTR_ERR(vdd);
		TOUCH_ERR("failed to get regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = regulator_set_voltage(vdd, 1800000, 1800000);
	if (ret < 0) {
		TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = regulator_enable(vdd);
	if (ret < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	msleep(50);

	#endif

	return TOUCH_SUCCESS;
}

static void MIT200_Reset(struct i2c_client *client)
{
	#if defined(TOUCH_MODEL_Y50)
	DSI_change_mode(DSI_INCELL_GPIO_TIMER_MODE);
	#endif

	TouchResetCtrl(0);
	msleep(5);
	TouchResetCtrl(1);
	msleep(5);

	TOUCH_LOG("Device was reset\n");
}


//====================================================================
// Function : MIT200_QueryDeviceConnection
// Description
//   - Check if touch IC was connected
//   - will be called at module init ( to select proper device driver function )
//   - implement using "Maker ID Pin" or "Read special register of IC"
//   - In case of using "Read special register of IC", you should be implement it using "touch_i2c_read_for_query()"
//====================================================================
static int MIT200_Connect(void)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Device detection function */

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : MIT200_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int MIT200_InitRegister(struct i2c_client *client)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Register initialization after reset */

	return TOUCH_SUCCESS;
}

static void MIT200_ClearInterrupt(struct i2c_client *client)
{
	
}

static int MIT200_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	u8 buf[MIT_FINGER_EVENT_SZ * MAX_NUM_OF_FINGERS] = {0x00, };
	u8 cmd[2] = {0, };
	u8 event_type;
	u8 *tmp;
	u8 sz = 0;	
	u8 index = 0;
	u8 state = 0;
	int i = 0;

	cmd[0] = MIT_REGH_CMD;
	cmd[1] = MIT_EVENT_PKT_SZ;

	if (MIT200_I2C_Read(client, cmd, 2, &sz, 1) < 0) {
		TOUCH_ERR("I2C read fail : MIT_EVENT_PKT_SZ\n");
		return TOUCH_FAIL;
	}

	if (sz == 0) {
		TOUCH_WARN("mms_get_packet sz = 0 \n");
		return TOUCH_SUCCESS;
	}

	cmd[1] = MIT_INPUT_EVENT;
	if (MIT200_I2C_Read(client, cmd, 2, buf, sz) < 0) {
		TOUCH_ERR("I2C read fail : MIT_INPUT_EVENT\n");
		return TOUCH_FAIL;
	}

	event_type = buf[0] & 0xf;

	if (event_type >= 0x1 && event_type <= 0xa) {
		static TouchFingerData pFingerData[MAX_FINGER+1];		

		for (i = 0; i < sz; i += MIT_FINGER_EVENT_SZ) {
			tmp = buf + i;
			index = (tmp[0] & 0xf) - 1;
			state = (tmp[0] & 0x80) ? 1 : 0;

			if (index < 0 || index > MAX_NUM_OF_FINGERS) {
				TOUCH_ERR("invalid touch index (%d)\n", index);
				return TOUCH_FAIL;
			}

			pData->type = DATA_FINGER;

			if (tmp[0] & 0x10) {
				if (state) {
					TOUCH_LOG("Palm detected : %d \n", tmp[5]);
				} else {
					TOUCH_LOG("Palm released : %d \n", tmp[5]);
				}

				return TOUCH_SUCCESS;
			}

			if (state) {
				pFingerData[index].id = index + 1;
				pFingerData[index].x = tmp[2] | ((tmp[1] & 0x0f) << 8);
				pFingerData[index].y = tmp[3] | ((tmp[1] & 0xf0) << 4);
				pFingerData[index].width_major = tmp[4];
				pFingerData[index].width_minor = 0;
				pFingerData[index].orientation = 0;
				pFingerData[index].pressure = tmp[5];

			} else {
				memset(&pFingerData[index], 0x00, sizeof(TouchFingerData));
			}
		}

		pData->count = MAX_NUM_OF_FINGERS;
		memcpy(pData->fingerData, pFingerData, sizeof(TouchFingerData) * MAX_FINGER);
	} else if(event_type == MIT_LPWG_EVENT) {
		if (buf[1] == 1) {
			TOUCH_LOG( "Knock-on Detected\n" );
			pData->type = DATA_KNOCK_ON;			
		} else if (buf[1] == 0) {
			TOUCH_LOG( "Knock-code Detected\n" );
			pData->type = DATA_KNOCK_CODE;

			for (i = 2; i < sz; i += MIT_LPWG_EVENT_SZ) {
				tmp = buf + i;

				pData->knockData[index].x = tmp[1] | ((tmp[0] & 0xf) << 8);
				pData->knockData[index].y = tmp[2] | (((tmp[0] >> 4 ) & 0xf) << 8);

				TOUCH_DBG("LPWG data [%d, %d]\n", pData->knockData[index].x, pData->knockData[i].y);
				index++;
			}

			pData->count = ts->lpwgSetting.tapCount;
		} else {
			TOUCH_WARN("Unknown Packet Error : %02X %02X %02X %02X %02X \n", buf[0], buf[1], buf[2], buf[3], buf[4]);
		}
	}

	return TOUCH_SUCCESS;
}

static int MIT200_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	u8 cmd[2] = {0, };
	u8 version[2] = {0, };
	int ret = 0;

	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */
	cmd[0] = 0x00;
	cmd[1] = MIT_FW_VERSION;

	ret = MIT200_I2C_Read(client, cmd, 2, version, 2);
	if(ret) {
		return TOUCH_FAIL;
	}

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[0];
	pFwInfo->version = version[1];

	TOUCH_LOG("IC F/W Version = v%X.%02X ( %s )\n", version[0], version[1], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;
}

static int MIT200_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 version[2] = {0, };
	u8 *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if(ret) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	version[0] = fw->data[0xFFFA];
	version[1] = fw->data[0xFFFB];

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[0] ;
	pFwInfo->version = version[1];

	/* Free firmware image buffer */
	release_firmware(fw);

	TOUCH_LOG("BIN F/W Version = v%X.%02X ( %s )\n", version[0], version[1], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;		
}


static int MIT200_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	char *pFwFilename = NULL;

	TOUCH_FUNC();
	
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	MIT_FirmwareUpgrade(ts, pFwFilename);

	return TOUCH_SUCCESS;	
}

static int MIT200_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;
	
	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

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


static int MIT200_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	dataLen += sprintf(pBuf, "%s", "========= Additional Information =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name = Dummy\n");

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
	
}

static int MIT200_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
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

static void MIT200_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch( notify )
	{
		case NOTIFY_CALL:
			TOUCH_LOG("Call was notified ( data = %d )\n", data);
			break;

		case NOTIFY_Q_COVER:
			TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
			break;

		default:
			TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
			break;
	}

	return;

}

TouchDeviceSpecificFunction MIT200_Func = {

	.Initialize 			= MIT200_Initialize,
	.Reset 					= MIT200_Reset,
	.Connect 				= MIT200_Connect,
	.InitRegister 			= MIT200_InitRegister,
	.ClearInterrupt 		= MIT200_ClearInterrupt,
	.InterruptHandler 		= MIT200_InterruptHandler,
	.ReadIcFirmwareInfo 	= MIT200_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= MIT200_GetBinFirmwareInfo,
	.UpdateFirmware 		= MIT200_UpdateFirmware,
	.SetLpwgMode 			= MIT200_SetLpwgMode,
	.DoSelfDiagnosis 		= MIT200_DoSelfDiagnosis,
	.AccessRegister 		= MIT200_AccessRegister,
	.NotifyHandler 		= MIT200_NotifyHandler,
	.device_attribute_list 	= MIT200_attribute_list,

};


/* End Of File */


