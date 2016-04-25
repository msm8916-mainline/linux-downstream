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
 *    File  	: lgtp_common_driver.c
 *    Author(s)   : Branden You < branden.you@lge.com >
 *    Description : 
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/lgtp_common.h>

#include <linux/input/lgtp_common_driver.h>
#include <linux/input/lgtp_platform_api.h>
#include <linux/input/lgtp_model_config.h>
#include <linux/input/lgtp_device_s3320.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define LGE_TOUCH_NAME		"lge_touch"


/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/

struct touch_device_driver* touch_device_func;
struct workqueue_struct* touch_wq;

struct mutex* pMutexTouch;
struct wake_lock* pWakeLockTouch;
struct delayed_work* pWorkTouch;

extern struct touch_device_driver synaptics_ts_driver ;
extern int nIrq_num;


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/

#define VALID_LPWG_UEVENT_SIZE 			3
#define MAX_ATTRIBUTE_ARRAY_SIZE 		30

static char *lpwg_uevent[VALID_LPWG_UEVENT_SIZE][2] = {
	{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
	{"TOUCH_GESTURE_WAKEUP=SIGNATURE", NULL}
};

static struct bus_type touch_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = "lge_touch",
};

static struct device device_touch = {
	.id    = 0,
	.bus   = &touch_subsys,
};

#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
void send_uevent_lpwg(struct i2c_client *client, int type)
{
	kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, lpwg_uevent[type-1]);
}
#else
static void send_uevent_lpwg(struct i2c_client *client, int type)
{
	kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, lpwg_uevent[type-1]);

#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141013 : */
#else
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(3000));

	if (type > 0 && type <= VALID_LPWG_UEVENT_SIZE
		&& atomic_read(&ts->state.uevent_state) == UEVENT_IDLE) {
		atomic_set(&ts->state.uevent_state, UEVENT_BUSY);
		send_uevent(lpwg_uevent[type-1]);
	}
#endif
}
#endif

static int report_event(struct lge_touch_data *ts)
{
	TouchReadData *pReadData = &ts->readData;
	u32 reportedFinger = ts->reportData.finger;

	u32 newFinger = 0;
	u32 changedFinger = 0;
	u32 pressedFinger = 0;
	u32 releasedFinger = 0;
	int i = 0;

	for( i=0 ; i < pReadData->count ; i++ ) {
		input_mt_slot(ts->input_dev, pReadData->fingerData[i].id);
 
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, pReadData->fingerData[i].id);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, pReadData->fingerData[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pReadData->fingerData[i].y);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, pReadData->fingerData[i].pressure);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, pReadData->fingerData[i].width_major);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR, pReadData->fingerData[i].width_minor);
		input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, pReadData->fingerData[i].orientation);
	}

	for(i=0 ; i < pReadData->count ; i++) {
		newFinger |= 1 << pReadData->fingerData[i].id ;
	}

	changedFinger = reportedFinger ^ newFinger;
	pressedFinger = newFinger & changedFinger;
	releasedFinger = reportedFinger & changedFinger;

	for(i=0 ; i < MAX_FINGER ; i++) {
		if((pressedFinger >> i) & 0x1) {
			LGTC_LOG("[FINGER] PRESS<%d> x = %d, y = %d, z= %d\n", i,
				pReadData->fingerData[i].x,
				pReadData->fingerData[i].y,
				pReadData->fingerData[i].pressure);
		}

		if((releasedFinger >> i) & 0x1) {
			LGTC_LOG("[FINGER] RELEASE<%d>\n", i);
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}

	ts->reportData.finger = newFinger;

	input_sync(ts->input_dev);

	return NO_ERROR;
}

static void release_all_touch_event(struct lge_touch_data *ts)
{
	memset(&ts->readData, 0, sizeof(TouchReadData));
	ts->readData.type = T_DATA_FINGER;
	ts->readData.count = 0;
	report_event(ts);
}

static void WqTouchInit(struct work_struct *work_init)
{
	struct lge_touch_data *pDriverData = container_of(to_delayed_work(work_init),
		 struct lge_touch_data, work_init);

	mutex_lock(pMutexTouch);

	touch_device_func->init(pDriverData->client);

	mutex_unlock(pMutexTouch);
}

//==========================================================
// Interrupt Service Routine ( Triggered by HW Interrupt )
//==========================================================
#if defined ( CONFIG_MTK_TOUCHPANEL )
static void TouchIrqHandler(void)
{
	if(pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}
}
#else
static irqreturn_t TouchIrqHandler(int irq, void *dev_id)
{
	if(pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}

	return IRQ_HANDLED;
}
#endif


//==========================================================
// Triggered by ISR ( TouchIrqHandler() )
//==========================================================
static void WqTouchIrqHandler(struct work_struct *work_irq)
{
	 struct lge_touch_data *pDriverData = container_of(to_delayed_work(work_irq),
		 struct lge_touch_data, work_irq);
	 error_type ret = ERROR;

	wake_lock_timeout ( pWakeLockTouch, msecs_to_jiffies(3000) );

	mutex_lock(pMutexTouch);

	/* TBD : init buffer */
	memset(&pDriverData->readData, 0x0, sizeof(TouchReadData));
	pDriverData->readData.type = NUM_OF_DATA_TYPE;

	/* do TouchIC specific interrupt processing */
	ret = touch_device_func->isr(pDriverData->client, &pDriverData->readData);

	/* do processing according to the data from TouchIC */
	if( pDriverData->readData.type == T_DATA_FINGER ) {
		report_event(pDriverData);
	} else if( pDriverData->readData.type == T_DATA_KNOCK_ON ) {
		/* Send Proper Event to CFW */
		send_uevent_lpwg(pDriverData->client, LPWG_DOUBLE_TAP);
	} else if( pDriverData->readData.type == T_DATA_KNOCK_CODE ) {
		#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
		if( ret == IGNORE_EVENT ) {
			LGTC_WARN("Knock-code detected but need to wait if there is any more touch action\n");
		} else if( ret == IGNORE_EVENT_BUT_SAVE_IT ) {
			LGTC_WARN("Over-tap detected but but need to wait if there is any more touch action\n");
		} else {
			LGTC_WARN("Unexpected result from device ISR\n");
		}
		#else
		/* Send Proper Event to CFW */
		send_uevent_lpwg(pDriverData->client, LPWG_PASSWORD);
		#endif

	}

	mutex_unlock(pMutexTouch);
}


/****************************************************************************
* Global Functions
****************************************************************************/
void set_touch_handle(struct i2c_client *client, void *h_touch)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	ts->h_touch = h_touch;
}

void *get_touch_handle(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	return ts->h_touch;
}

static void WqfirmwareUpgrade(struct work_struct *work_upgrade)
{
	struct lge_touch_data *pDriverData = container_of(to_delayed_work(work_upgrade),
		struct lge_touch_data, work_upgrade);

	LGTC_FUN();

	wake_lock ( pWakeLockTouch );

	mutex_lock(pMutexTouch);

	TouchDisableIrq();

	if(pDriverData->fw_info.fw_force_upgrade == 1) {
		LGTC_DBG("Start FW force upgrade\n");

		touch_device_func->fw_upgrade(pDriverData->client);
	}
	else {
		if(pDriverData->fw_info.fw_type == OFFICIAL_VERSION
			&& strcmp(pDriverData->fw_info.fw_ic_version, pDriverData->fw_info.fw_bin_version)) {
			LGTC_DBG("Start FW upgrade\n");

			strncpy(pDriverData->fw_info.fw_path, pDriverData->mConfig.fw_image, sizeof(pDriverData->fw_info.fw_path));
			touch_device_func->fw_upgrade(pDriverData->client);
		}
	}

	mutex_unlock(pMutexTouch);

	wake_unlock ( pWakeLockTouch );

	queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
	TouchEnableIrq();
}


static int UpdateLpwgSetting ( LGTcLpwgSetting *pNewSetting, int cmdType, int *cmdParam )
{
	int result = LGTC_SUCCESS;

	E_TouchLpwgCmdType lpwgCmd = (E_TouchLpwgCmdType)cmdType;

	switch ( lpwgCmd ) {
	case T_LPWG_CMD_MODE:
		if( cmdParam[0] > 0x7 ) {
			LGTC_ERR ( "Invalide LPWG Setting ( mode = %d )\n", cmdParam[0] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->mode = cmdParam[0];
		}
		break;

	case T_LPWG_CMD_LCD_PIXEL_SIZE:
		/* consider if we need to check "LCD Pixel Size" */
		LGTC_DBG ( "LPWG Setting ( LCD Pixel Size : X = %d, Y = %d )\n", cmdParam[0], cmdParam[1] );
		pNewSetting->lcdPixelSizeX = cmdParam[0];
		pNewSetting->lcdPixelSizeY = cmdParam[1];
		break;

	case T_LPWG_CMD_ACTIVE_TOUCH_AREA:
		/* consider if we need to check "Active Touch Area" */
		LGTC_DBG ( "LPWG Setting ( Active Touch Area : X1 = %d, X2 = %d, Y1 = %d, Y2 = %d )\n",
			cmdParam[0], cmdParam[1], cmdParam[2], cmdParam[3] );
		pNewSetting->activeTouchAreaX1 = cmdParam[0];
		pNewSetting->activeTouchAreaX2 = cmdParam[1];
		pNewSetting->activeTouchAreaY1 = cmdParam[2];
		pNewSetting->activeTouchAreaY2 = cmdParam[3];
		break;

	case T_LPWG_CMD_TAP_COUNT:
		if( cmdParam[0] > LGTC_MAX_TAP_COUNT ) {
			LGTC_ERR ( "Invalide LPWG Setting ( tapCount = %d )\n", cmdParam[0] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->tapCount = cmdParam[0];
		}
		break;

	case T_LPWG_CMD_TAP_DISTANCE:
		LGTC_ERR ( "Invalide LPWG Command ( T_LPWG_CMD_TAP_DISTANCE )\n" );
		result = LGTC_FAIL;
		break;

	case T_LPWG_CMD_LCD_STATUS:
		if( cmdParam[0] > 1 ) {
			LGTC_ERR ( "Invalide LPWG Setting ( lcdState = %d )\n", cmdParam[0] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->lcdState = cmdParam[0];
		}
		break;

	case T_LPWG_CMD_PROXIMITY_STATUS:
		if( cmdParam[0] > 1 ) {
			LGTC_ERR ( "Invalide LPWG Setting ( proximityState = %d )\n", cmdParam[0] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->proximityState = cmdParam[0];
		}
		break;

	case T_LPWG_CMD_FIRST_TWO_TAP:
		if( cmdParam[0] > 1 ) {
			LGTC_ERR ( "Invalide LPWG Setting ( isFirstTwoTapSame = %d )\n", cmdParam[0] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->isFirstTwoTapSame = cmdParam[0];
		}
		break;

	case T_LPWG_CMD_UPDATE_ALL:
		if( ( cmdParam[0] > 0x7 ) || ( cmdParam[1] > 1 ) || ( cmdParam[2] > 1 ) || ( cmdParam[3] > 1 ) ) {
			LGTC_ERR ( "Invalide LPWG Setting ( mode = %d, LCD = %d, Proximity = %d, Cover = %d )\n",
				cmdParam[0], cmdParam[1], cmdParam[2], cmdParam[3] );
			result = LGTC_FAIL;
		} else {
			pNewSetting->mode = cmdParam[0];
			pNewSetting->lcdState = cmdParam[1];
			pNewSetting->proximityState = cmdParam[2];
			pNewSetting->coverState = cmdParam[3];
		}
		break;

	default:
		LGTC_ERR ( "Invalide LPWG Command ( Type = %d )\n", lpwgCmd );
		result = LGTC_FAIL;
		break;
	}

	return result;
}

/* Sysfs - platform_data
  *
  * show_platform_data : Print all values of platform_data.
  * store_platform_data : User can change only the 'role'.
  */
static ssize_t show_platform_data(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	TouchModelConfig *pConfig = &ts->mConfig;

	int ret = 0;

	ret = sprintf(buf, "====== Platform data ======\n");
	ret += sprintf(buf+ret, "int_pin[%d] reset_pin[%d]\n",
			pConfig->int_pin, pConfig->reset_pin);
	ret += sprintf(buf+ret, "caps:\n");
	ret += sprintf(buf+ret, "\t%25s = %d\n", "button_support",
			pConfig->button_support);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "number_of_button",
			pConfig->number_of_button);
	ret += sprintf(buf+ret, "\t%25s = %d, %d, %d, %d\n", "button_name",
			pConfig->button_name[0],
			pConfig->button_name[1],
			pConfig->button_name[2],
			pConfig->button_name[3]);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_x", pConfig->max_x);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_y", pConfig->max_y);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_pressure",
			pConfig->max_pressure);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_width",
			pConfig->max_width);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_orientation",
			pConfig->max_orientation);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_id",
			pConfig->max_id);
	ret += sprintf(buf+ret, "role:\n");
	ret += sprintf(buf+ret, "\t%25s = %d\n", "protocol_type",
			pConfig->protocol_type);
	ret += sprintf(buf+ret, "\t%25s = 0x%lx\n", "irqflags",
			pConfig->irqflags);
	ret += sprintf(buf+ret, "pwr:\n");
	ret += sprintf(buf+ret, "firmware:\n");
	ret += sprintf(buf+ret, "\t%25s = %s\n", "fw_image",
			pConfig->fw_image);

	return ret;
}

/* Sysfs -ic_rw
 *
 * show_ic_rw : User can read the register using 'reg' and 'value'.
 * Both 'reg' and 'value' are assigned by 'store_ic_rw'. Use 'assign' command.
 * store_ic_rw : User can write values to registers.
 *
 * reg, value : these variables are used to read the register.
 */
static int reg, value;

static ssize_t show_ic_rw(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	u32 ret = 0, tmp = 0;

	do {
		touch_device_func->ic_ctrl(ts->client,
				IC_CTRL_READ, reg++, &tmp);
		ret += sprintf(buf+ret, "%d\n", tmp);
	} while (--value > 0);

	LGTC_LOG("%s\n", buf);
	return ret;
}

static ssize_t store_ic_rw(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	unsigned char string[30] = {0};
	int temp[2] = {0};
	u32 ret = 0;

	sscanf(buf, "%s %d %d", string, &temp[0], &temp[1]);

	if ((strcmp(string, "write") && strcmp(string, "assign")))
		return count;

	reg = temp[0];
	value = temp[1];
	if (!strcmp(string, "write")) {
		u32 write_data = ((0xFF & reg) << 16) | (0xFF & value);
		touch_device_func->ic_ctrl(ts->client,
				IC_CTRL_WRITE, write_data, &ret);
	}

	LGTC_LOG("%s - reg[%d] value[%d] return[%d]\n", string, reg, value, ret);

	return count;
}

/* Sysfs - firmware_upgrade
 *
 * store_upgrade : upgrade the firmware.
 */
static ssize_t store_upgrade(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	char path[256] = {0};

	sscanf(buf, "%s", path);

	memset(ts->fw_info.fw_path, 0x00, sizeof(ts->fw_info.fw_path));
	memcpy(ts->fw_info.fw_path, path, sizeof(ts->fw_info.fw_path));
	ts->fw_info.fw_force_upgrade = 1;

	queue_delayed_work(touch_wq, &ts->work_upgrade, 0);

	return count;
}

/* Sysfs - firmware_upgrade
 *
 * store_upgrade : upgrade the firmware.
 */
static ssize_t store_rewrite_bin_fw(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	strncpy(ts->fw_info.fw_path, ts->mConfig.fw_image, sizeof(ts->fw_info.fw_path));
	ts->fw_info.fw_force_upgrade = 1;

	queue_delayed_work(touch_wq, &ts->work_upgrade, 0);

	return count;
}


//==========================================================
// LPGW data will be read using this funtion.
//==========================================================
static ssize_t show_lpwg_data(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *pDriverData = i2c_get_clientdata(client);

	int i = 0;
	int ret = 0;

	LGTC_FUN();

	mutex_lock(pMutexTouch);
	wake_lock ( pWakeLockTouch );

	/* We already get the data on ISR, so we can return the data immediately */
	for( i=0 ; i<pDriverData->readData.count ; i++ ) {
		ret += sprintf(buf+ret, "%d %d\n", pDriverData->readData.knockData[i].x, pDriverData->readData.knockData[i].y);
	}

	wake_unlock ( pWakeLockTouch );
	mutex_unlock(pMutexTouch);

	return ret;
}

//==========================================================
// We can get a feedback ( result ) about the event we sent before on ISR
//==========================================================
static ssize_t store_lpwg_data(struct i2c_client *client,
	const char *buf, size_t count)
{
	#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
	struct lge_touch_data *pDriverData = i2c_get_clientdata(client);
	#endif

	int reply = 0;

	LGTC_FUN();

	sscanf(buf, "%d", &reply);

	mutex_lock(pMutexTouch);
	wake_lock ( pWakeLockTouch );

	if( reply == 1 ) {
		LGTC_LOG("LPWG : Code is matched\n");
		/* Code is matched, do something for leaving "LPWG Mode" if you need. But normally "Resume" will be triggered soon. */
	} else {
		LGTC_LOG("LPWG : Code is NOT matched\n");
		#if defined ( USE_OVER_TAP_COUNT_DETECTION_TIMER )
		touch_device_func->lpwg(pDriverData->client, pDriverData->reportMode, &pDriverData->lpwgSetting);
		#endif
	}

	wake_unlock ( pWakeLockTouch );
	mutex_unlock(pMutexTouch);

	return count;
}


//==========================================================
// LPWG Command ( Setting ) will be got from this function
//==========================================================
static ssize_t store_lpwg_notify(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *pDriverData = i2c_get_clientdata(client);

	int result = LGTC_SUCCESS;
	LGTcLpwgSetting newLpwgSetting = {0};
	LGTcLpwgSetting *pCurrLpwgSetting = &pDriverData->lpwgSetting;
	E_TouchReportMode reportMode = NUM_OF_T_REPORT_MODE;

	int type = 0;
	int value[4] = {0};

	/* Get command and parameter from buffer */
	sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);
	LGTC_DBG("LPWG CMD : Type[%d] value[%d %d %d %d]\n", type, value[0], value[1], value[2], value[3]);

	mutex_lock(pMutexTouch);
	wake_lock ( pWakeLockTouch );

	/* Copy current setting to temporal variable ( to overwrite the updated value ) */
	memcpy(&newLpwgSetting, pCurrLpwgSetting, sizeof(LGTcLpwgSetting));

	/* Verify and Make a updated new LPWG Settings */
	result = UpdateLpwgSetting(&newLpwgSetting, type, value);
	if( result == LGTC_FAIL ) {
		LGTC_ERR("LPWG CMD : Invalid input command or paramerer. Fix it.\n");
		goto earlyReturn;
	}

	LGTC_LOG("LPWG CMD : T[%d] M[%d] L[%d] P[%d] C[%d]\n",
		type, newLpwgSetting.mode, newLpwgSetting.lcdState, newLpwgSetting.proximityState, newLpwgSetting.coverState);

	if( ( (E_TouchLpwgCmdType)type == T_LPWG_CMD_LCD_STATUS ) ||
		( (E_TouchLpwgCmdType)type == T_LPWG_CMD_PROXIMITY_STATUS ) ) {
		LGTC_DBG("Ignore it if LPWG command type is 6 or 7. We will process type 9.\n");
		goto earlyReturn;
	}

	if( newLpwgSetting.lcdState == 1 ) {
		reportMode = T_REPORT_NORMAL;
	} else {
		if( newLpwgSetting.mode == 0 ) {
			reportMode = T_REPORT_OFF;
		} else if( newLpwgSetting.mode == 1 ) {
			reportMode = T_REPORT_KNOCK_ON_ONLY;
		} else if( newLpwgSetting.mode == 2 ) {
			LGTC_WARN("Invalild Report Mode. Currently we don't have this case.\n");
#if 1 /* LGE_BSP_COMMON : branden.you@lge.com_20141017 : */
			reportMode = T_REPORT_KNOCK_ON_CODE;
#else
			reportMode = T_REPORT_KNOCK_CODE_ONLY;
#endif
		} else if( newLpwgSetting.mode == 3 ) {
			reportMode = T_REPORT_KNOCK_ON_CODE;
		} else {
			LGTC_ERR("Impossible Report Mode ( %d )\n", reportMode);
			goto earlyReturn;
		}
	}

	if( pDriverData->reportMode != reportMode ) {
		pDriverData->reportMode = reportMode;

		#if defined ( LPWG_ENABLE_ON_SUSPEND ) /* JDI In-Cell Only */

		if( pDriverData->isSuspend == LGTC_TRUE ) {
			touch_device_func->lpwg(pDriverData->client, reportMode, &newLpwgSetting);
		}

		#else

		touch_device_func->lpwg(pDriverData->client, reportMode, &newLpwgSetting);

		#endif
	}

	/* Copy new LPWG settings to current setting */
	memcpy(pCurrLpwgSetting, &newLpwgSetting, sizeof(LGTcLpwgSetting));

earlyReturn:

	wake_unlock ( pWakeLockTouch );
	mutex_unlock(pMutexTouch);

	return count;
}

static ssize_t show_sd_info(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int raw_status = 0;
	int ch_status = 0;

	mutex_lock(pMutexTouch);
	wake_lock ( pWakeLockTouch );

	TouchDisableIrq();
	ret = touch_device_func->sd(client, buf, &raw_status, &ch_status);
	TouchEnableIrq();

	wake_unlock ( pWakeLockTouch );
	mutex_unlock(pMutexTouch);

	ret = sprintf(buf, "========RESULT=======\n");
	ret += sprintf(buf+ret, "Channel Status : %s\n", (ch_status == LGTC_SUCCESS) ? "Pass" : "Fail");
	ret += sprintf(buf+ret, "Raw Data : %s\n", (raw_status == LGTC_SUCCESS) ? "Pass" : "Fail");

	return ret;
}

static ssize_t show_knock_on_type(struct i2c_client *client, char *buf)
{
	int ret = 0;

	#if defined ( KNOCK_ON_SUPPORT )
	ret = sprintf(buf, "%d\n", 1);
	#else
	ret = sprintf(buf, "%d\n", 0);
	#endif

	return ret;
}


static LGE_TOUCH_ATTR(platform_data, S_IRUGO | S_IWUSR, show_platform_data, NULL);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, show_ic_rw, store_ic_rw);
static LGE_TOUCH_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, store_upgrade);
static LGE_TOUCH_ATTR(rewrite_bin_fw, S_IRUGO | S_IWUSR, NULL, store_rewrite_bin_fw);
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd_info, NULL);
static LGE_TOUCH_ATTR(knock_on_type, S_IRUGO | S_IWUSR, show_knock_on_type, NULL);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_platform_data.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_fw_upgrade.attr,
	&lge_touch_attr_rewrite_bin_fw.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_lpwg_notify.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_knock_on_type.attr,
	NULL,
};

static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj,
	struct attribute *attr, char *buf)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
		struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts->client, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj,
	struct attribute *attr,
			      const char *buf, size_t count)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
		struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts->client, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops	= &lge_touch_sysfs_ops,
};



/* sysfs_register
 *
 * get_attribute_array_size
 * : attribute_list should has NULL value at the end of list.
 */

int get_attribute_array_size(struct attribute **list)
{
	int i = 0;

	while (list[i] != NULL && i < MAX_ATTRIBUTE_ARRAY_SIZE)
		i++;

	return i <= MAX_ATTRIBUTE_ARRAY_SIZE ? i : 0;
}

static int sysfs_register(struct lge_touch_data *ts,
	struct attribute **attribute_list)
{
	struct attribute **new_attribute_list;

	int ret = 0;
	int n1 = get_attribute_array_size(lge_touch_attribute_list);
	int n2 = attribute_list ? get_attribute_array_size(attribute_list) : 0;

	LGTC_FUN();

	new_attribute_list = devm_kzalloc(&ts->client->dev, (n1+n2+1) * sizeof(struct attribute *), GFP_KERNEL);
	if( new_attribute_list == NULL ) {
		LGTC_ERR("Fail to allocation memory\n");
		return -ENOMEM;
	}

	memcpy(new_attribute_list, lge_touch_attribute_list, n1 * sizeof(struct attribute *));

	if( attribute_list ) {
		memcpy(new_attribute_list + n1, attribute_list, n2 * sizeof(struct attribute *));
	}

	lge_touch_kobj_type.default_attrs = new_attribute_list;

	ret = subsys_system_register(&touch_subsys, NULL);
	if (ret < 0) {
		LGTC_ERR("Fail to register subsys ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = device_register(&device_touch);
	if (ret < 0) {
		LGTC_ERR("Fail to register device ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = kobject_init_and_add(&ts->lge_touch_kobj,
			&lge_touch_kobj_type,
			ts->input_dev->dev.kobj.parent, "%s", LGE_TOUCH_NAME);
	if( ret < 0 ) {
		LGTC_ERR("Fail to init and add kobject ( error = %d )\n", ret);
		return -ENODEV;
	}

	return NO_ERROR;
}

static int register_input_dev(struct lge_touch_data *pDriverData)
{
	int result = LGTC_SUCCESS;
	int idx = 0;

	LGTC_FUN();

	pDriverData->input_dev = input_allocate_device();
	if( pDriverData->input_dev == NULL ) {
		LGTC_ERR("Failed at input_allocate_device()\n");
		result = LGTC_FAIL;
		goto earlyReturn;
	}

	pDriverData->input_dev->name = "touch_dev";

	set_bit(EV_SYN, pDriverData->input_dev->evbit);
	set_bit(EV_ABS, pDriverData->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, pDriverData->input_dev->propbit);

	input_set_abs_params(pDriverData->input_dev, ABS_MT_POSITION_X, 0, pDriverData->mConfig.max_x, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_POSITION_Y, 0, pDriverData->mConfig.max_y, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_PRESSURE, 0, pDriverData->mConfig.max_pressure, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_WIDTH_MAJOR, 0, pDriverData->mConfig.max_width, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_WIDTH_MINOR, 0, pDriverData->mConfig.max_width, 0, 0);
	input_set_abs_params(pDriverData->input_dev, ABS_MT_ORIENTATION, 0, pDriverData->mConfig.max_orientation, 0, 0);

	if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_A) {
		input_set_abs_params(pDriverData->input_dev, ABS_MT_TRACKING_ID, 0, pDriverData->mConfig.max_id, 0, 0);
	} else {
		#if defined ( KERNEL_ABOVE_3_4_67 )
		result = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id, 0);
		#else
		result = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id);
		#endif
		if( result < 0 ) {
			LGTC_ERR("Failed at input_mt_init_slots()\n");
			input_free_device(pDriverData->input_dev);
			goto earlyReturn;
		}
	}

	if (pDriverData->mConfig.button_support) {
		set_bit(EV_KEY, pDriverData->input_dev->evbit);
		for( idx = 0; idx < pDriverData->mConfig.number_of_button; idx++ ) {
			set_bit(pDriverData->mConfig.button_name[idx], pDriverData->input_dev->keybit);
		}
	}

	result = input_register_device(pDriverData->input_dev);
	if( result < 0 ) {
		LGTC_ERR("Failed at input_register_device()\n");

		if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B) {
			input_mt_destroy_slots(pDriverData->input_dev);
		}

		input_free_device(pDriverData->input_dev);
		goto earlyReturn;
	}

	input_set_drvdata(pDriverData->input_dev, pDriverData);

earlyReturn:

	return result;
}


static void touch_suspend(struct device *dev)
{
	struct lge_touch_data *pDriverData =  dev_get_drvdata(dev);

	LGTC_FUN();

	pDriverData->isSuspend = LGTC_TRUE;

	cancel_delayed_work_sync(&pDriverData->work_init);

	mutex_lock(pMutexTouch);

	#if defined ( LPWG_ENABLE_ON_SUSPEND ) /* JDI In-Cell Only */

	msleep(100); /* JDI In-Cell Only : TBD : Featuring */
	touch_device_func->lpwg(pDriverData->client, pDriverData->reportMode, &pDriverData->lpwgSetting);
	release_all_touch_event(pDriverData);
	#endif

	touch_device_func->suspend(pDriverData->client);

	mutex_unlock(pMutexTouch);
}

static void touch_resume(struct device *dev)
{
	struct lge_touch_data *pDriverData =  dev_get_drvdata(dev);

	LGTC_FUN();

	mutex_lock(pMutexTouch);

	pDriverData->isSuspend = LGTC_FALSE;

	touch_device_func->resume(pDriverData->client);

	mutex_unlock(pMutexTouch);

	#if 1 /* TBD : decide here or do it on "LPWG Notify" */
	queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
	#endif
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct lge_touch_data *ts =
		container_of(h, struct lge_touch_data, early_suspend);

	LGTC_FUN();

	touch_suspend(&ts->client->dev);
}

static void touch_late_resume(struct early_suspend *h)
{
	struct lge_touch_data *ts =
		container_of(h, struct lge_touch_data, early_suspend);

	LGTC_FUN();

	touch_resume(&ts->client->dev);
}

#elif defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	struct lge_touch_data *pDriverData =
		container_of(self, struct lge_touch_data, fb_notif);

	if (evdata && evdata->data) {
		int *blank = (int *)evdata->data;

		if(event == FB_EVENT_BLANK) {
			if (*blank == FB_BLANK_UNBLANK) {
				touch_resume(&pDriverData->client->dev);
			}
			else if (*blank == FB_BLANK_POWERDOWN) {
				if(pDriverData->reportMode != T_REPORT_NORMAL)
					enable_irq_wake(pDriverData->client->irq);
				else
					disable_irq_wake(pDriverData->client->irq);

				touch_suspend(&pDriverData->client->dev);
			}
		}
		#if defined ( LPWG_ENABLE_ON_SUSPEND ) /* JDI In-Cell Only */
		else if(event == FB_EARLY_EVENT_BLANK) {
			if(*blank == FB_BLANK_UNBLANK) {
				mutex_lock(pMutexTouch);

				touch_device_func->lpwg(pDriverData->client,
					T_REPORT_NORMAL,
					&pDriverData->lpwgSetting);

				mutex_unlock(pMutexTouch);

				pDriverData->isSuspend = LGTC_FALSE;
			} else if (*blank == FB_BLANK_POWERDOWN) {
				cancel_delayed_work_sync(&pDriverData->work_init);
			}
		}
		#endif
	}

	return NO_ERROR;
}
#endif

static int touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int result = LGTC_SUCCESS;

	struct lge_touch_data *pDriverData;
	struct attribute **specific_attribute_list;

	LGTC_FUN();

	/* memory allocation for globally used driver data */
	pDriverData = devm_kzalloc(&client->dev, sizeof(struct lge_touch_data), GFP_KERNEL);
	if( pDriverData == NULL ) {
		LGTC_ERR("Fail to allocation memory\n");
		return -ENOMEM;
	}

	/* set driver data to i2c client data */
	pDriverData->client = client;
	i2c_set_clientdata(client, pDriverData);

	/* Initialise Platform */
	result = TouchInitialisePlatform(pDriverData);
	if( result == LGTC_FAIL ) {
		LGTC_ERR("Failed at TouchInitialisePlatform()\n");
		kfree(pDriverData);
		return result;
	}

	strncpy(pDriverData->fw_info.fw_path, pDriverData->mConfig.fw_image, sizeof(pDriverData->fw_info.fw_path));

	/* call IC specific probe function ( TBD : remove platform data dependancy ) */
	touch_device_func->probe(pDriverData->client, &pDriverData->fw_info, &specific_attribute_list);

	/* register as input device */
	register_input_dev(pDriverData);

	/* register sysfs ( core driver + TouchIC specific ) */
	sysfs_register(pDriverData, specific_attribute_list);

	/* set and register "suspend" and "resume" */
	#if defined ( CONFIG_HAS_EARLYSUSPEND )
	pDriverData->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	pDriverData->early_suspend.suspend = touch_early_suspend;
	pDriverData->early_suspend.resume = touch_late_resume;
	register_early_suspend(&pDriverData->early_suspend);
	#elif defined( CONFIG_FB )
	pDriverData->fb_notif.notifier_call = touch_fb_notifier_callback;
	fb_register_client(&pDriverData->fb_notif);
	#endif

	/* initialise system resource ( mutex, wakelock, work function ) */
	mutex_init(&pDriverData->thread_lock);

	INIT_DELAYED_WORK(&pDriverData->work_upgrade, WqfirmwareUpgrade);
	INIT_DELAYED_WORK(&pDriverData->work_irq, WqTouchIrqHandler);
	INIT_DELAYED_WORK(&pDriverData->work_init, WqTouchInit);

	wake_lock_init(&pDriverData->lpwg_wake_lock, WAKE_LOCK_SUSPEND, "touch_lpwg");

	/* store system resource to global variable */
	pMutexTouch = &pDriverData->thread_lock;
	pWakeLockTouch = &pDriverData->lpwg_wake_lock;
	pWorkTouch = &pDriverData->work_irq;

	pDriverData->lpwgSetting.mode = 0x3;
	pDriverData->lpwgSetting.lcdState = 1;
	pDriverData->reportMode = T_REPORT_NORMAL;
	pDriverData->isSuspend = LGTC_FALSE;

	/* Initialise TouchIC */
	queue_delayed_work(touch_wq, &pDriverData->work_init, 0);

	/* Enable Interrupt */
	#if defined ( CONFIG_MTK_TOUCHPANEL )
	/* register and enable */
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM,
		CUST_EINT_TOUCH_PANEL_TYPE, TouchIrqHandler, 1);
	#else
	result = request_irq(pDriverData->client->irq,
		(irq_handler_t)TouchIrqHandler,
		pDriverData->mConfig.irqflags | IRQF_ONESHOT,
		pDriverData->client->name, pDriverData);
	#endif

	nIrq_num = pDriverData->client->irq;

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);

	return result;
}

static int touch_remove(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	LGTC_FUN();

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

	kobject_del(&ts->lge_touch_kobj);

	wake_lock_destroy(&ts->lpwg_wake_lock);

	free_irq(ts->client->irq, ts);

	if (ts->mConfig.protocol_type == MT_PROTOCOL_B)
		input_mt_destroy_slots(ts->input_dev);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	devm_kfree(&ts->client->dev, ts);

	return NO_ERROR;
}

static int touch_pm_suspend(struct device *dev)
{
	LGTC_FUN();

	return NO_ERROR;
}

static int touch_pm_resume(struct device *dev)
{
	LGTC_FUN();

	return NO_ERROR;
}

static struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_pm_suspend,
	.resume = touch_pm_resume,
};

static struct i2c_device_id lge_ts_id[] = {
	{LGE_TOUCH_NAME, 0},
};

static struct i2c_driver lge_touch_driver = {
	.probe   	= touch_probe,
	.remove	 	= touch_remove,
	.id_table 	= lge_ts_id,
	.driver	 	= {
		.name   = LGE_TOUCH_NAME,
		.owner	= THIS_MODULE,
		.pm		= &touch_pm_ops,
	},
};

static struct of_device_id match_table[] = {
	{ .compatible = "synaptics,s3320",},
	{ },
};


#if defined ( CONFIG_MTK_TOUCHPANEL )
static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20)};
#endif

static int __init touch_init(void)
{
	LGTC_FUN();

	#if defined ( CONFIG_MTK_TOUCHPANEL )
	i2c_register_board_info(0, &i2c_tpd, 1);

	if(ts_dma_allocation()) {
		LGTC_ERR("Fail to DMA allocation\n");
		return -ENODEV;
	}
	#endif

	touch_device_func = &synaptics_ts_driver; /* 이원화 등을 위해 고려가 필요함 */

	touch_wq = create_singlethread_workqueue("touch_wq");
	if( touch_wq == NULL ) {
		LGTC_ERR("Fail to create workqueue\n");
		return -ENODEV;
	}

	lge_touch_driver.driver.of_match_table = match_table;
	if( i2c_add_driver ( &lge_touch_driver ) ) {
		LGTC_ERR("Fail to add i2c driver\n" );
		destroy_workqueue(touch_wq);
		return -ENODEV;
	}

	return NO_ERROR;
}

static void __exit touch_exit(void)
{
	LGTC_FUN();

	i2c_del_driver(&lge_touch_driver);

	touch_device_func = NULL;

	if (touch_wq) {
		destroy_workqueue(touch_wq);
	}
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("Branden You");
MODULE_DESCRIPTION("LGE Touch Platform Driver");
MODULE_LICENSE("GPL");

/* End Of File */

