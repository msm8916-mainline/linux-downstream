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
 *    File  : lgtp_common_driver.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[COMMON]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>

#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_model_config.h>

#if defined(TOUCH_TYPE_INCELL)
#include <linux/input/unified_driver_3/lgtp_common_notify.h>
#endif

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void Device_Touch_Release(struct device *dev);


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define LGE_TOUCH_NAME "lge_touch"


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
enum{
	UEVENT_KNOCK_ON = 0,
	UEVENT_KNOCK_CODE,
	UEVENT_SIGNATURE,
	UEVENT_SWIPE_UP,
	UEVENT_SWIPE_DOWN,
	NUM_OF_UEVENT
};

int lockscreen_stat = 0;
int swipe_check_call = 0;

#if defined(TOUCH_PLATFORM_QCT)
atomic_t pm_state;
#endif
/****************************************************************************
* Variables
****************************************************************************/
static TouchDeviceSpecificFunction *pDeviceSpecificFunc;

struct workqueue_struct *touch_wq;

struct mutex *pMutexTouch;
struct wake_lock *pWakeLockTouch;
struct delayed_work *pWorkTouch;

#define MAX_ATTRIBUTE_ARRAY_SIZE		30

static char *touch_uevent[NUM_OF_UEVENT][2] = {
	{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
	{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
	{"TOUCH_GESTURE_WAKEUP=SIGNATURE", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_UP", NULL},
	{"TOUCH_GESTURE_WAKEUP=SWIPE_DOWN", NULL}
};

static struct bus_type touch_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = "lge_touch",
};

static struct device device_touch = {
	.id    = 0,
	.bus   = &touch_subsys,
	.release = Device_Touch_Release,
};

#if defined(TOUCH_PLATFORM_MTK)
#if defined(TOUCH_DEVICE_MIT200)
static struct i2c_board_info i2c_tpd __initdata = {I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x34)};
#else
static struct i2c_board_info i2c_tpd __initdata = {I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20)};
#endif
#endif


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/

extern unsigned int touch_module;

/****************************************************************************
* Local Functions
****************************************************************************/
static void Device_Touch_Release(struct device *dev)
{
	return;
}
static void SetDriverState(TouchDriverData *pDriverData, TouchState newState)
{
	int ret = 0;

	if (newState == STATE_BOOT) {
		TOUCH_LOG("STATE = BOOT\n");
		pDriverData->isSuspend = TOUCH_FALSE;
		pDriverData->lpwgSetting.lcdState = 1;
		pDriverData->lpwgSetting.mode = -1;
		/* if there is no CFW, driver will remain NORMAL state. ( LAF / FOTA ) */
	} else if (newState == STATE_NORMAL) {
		TOUCH_LOG("STATE = NORMAL\n");
	} else if (newState == STATE_OFF) {
		TOUCH_LOG("STATE = OFF\n");
	} else if (newState == STATE_KNOCK_ON_ONLY) {
		TOUCH_LOG("STATE = KNOCK_ON_ONLY\n");
	} else if (newState == STATE_KNOCK_ON_CODE) {
		TOUCH_LOG("STATE = KNOCK_ON_CODE\n");
	} else if (newState == STATE_NORMAL_HOVER) {
		TOUCH_LOG("STATE = NORMAL_HOVER\n");
	} else if (newState == STATE_HOVER) {
		TOUCH_LOG("STATE = HOVER\n");
	} else if (newState == STATE_UPDATE_FIRMWARE) {
		TOUCH_LOG("STATE = UPDATE_FIRMWARE\n");
	} else if (newState == STATE_SELF_DIAGNOSIS) {
		TOUCH_LOG("STATE = SELF_DIAGNOSIS\n");
	} else if (newState == STATE_UNKNOWN) {
		TOUCH_LOG("STATE = UNKNOWN\n");
	} else {
		TOUCH_WARN("Invalid state ( %d )\n", newState);
		ret = TOUCH_FAIL;
	}

	if (ret == TOUCH_SUCCESS) {
		pDriverData->currState = newState;
		pDriverData->nextState = pDriverData->currState;
	}

}

static void report_finger(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	u32 reportedFinger = pDriverData->reportData.finger;

	u32 newFinger = 0;
	u32 changedFinger = 0;
	u32 pressedFinger = 0;
	u32 releasedFinger = 0;
	int i, j = 0;

	for (i = 0; i < pReadData->count; i++) {
		input_mt_slot(pDriverData->input_dev, pReadData->fingerData[i].id);

		input_report_abs(pDriverData->input_dev, ABS_MT_TRACKING_ID, pReadData->fingerData[i].id);
		input_report_abs(pDriverData->input_dev, ABS_MT_POSITION_X, pReadData->fingerData[i].x);
		input_report_abs(pDriverData->input_dev, ABS_MT_POSITION_Y, pReadData->fingerData[i].y);
		input_report_abs(pDriverData->input_dev, ABS_MT_PRESSURE, pReadData->fingerData[i].pressure);
		input_report_abs(pDriverData->input_dev, ABS_MT_WIDTH_MAJOR, pReadData->fingerData[i].width_major);
		input_report_abs(pDriverData->input_dev, ABS_MT_WIDTH_MINOR, pReadData->fingerData[i].width_minor);
		input_report_abs(pDriverData->input_dev, ABS_MT_ORIENTATION, pReadData->fingerData[i].orientation);
	}

	for (i = 0; i < pReadData->count; i++)
		newFinger |= 1 << pReadData->fingerData[i].id;

	changedFinger = reportedFinger ^ newFinger;
	pressedFinger = newFinger & changedFinger;
	releasedFinger = reportedFinger & changedFinger;

	for (i = 0; i < MAX_FINGER; i++) {
		if ((pressedFinger >> i) & 0x1) {
			for (j = 0; j < pReadData->count; j++) {
				if (pReadData->fingerData[j].id   == i) {
					if (lockscreen_stat) {
						TOUCH_LOG("[FINGER] PRESS<%d> x = xxxx, y = xxxx, z= xxxx\n",
							pReadData->fingerData[j].id);
					} else {
						TOUCH_LOG("[FINGER] PRESS<%d> x = %d, y = %d, z= %d\n",
							pReadData->fingerData[j].id,
							pReadData->fingerData[j].x,
							pReadData->fingerData[j].y,
							pReadData->fingerData[j].pressure);
					}
				}
			}
		}

		if ((releasedFinger >> i) & 0x1) {
			TOUCH_LOG("[FINGER] RELEASE<%d>\n", i);
			input_mt_slot(pDriverData->input_dev, i);
			input_report_abs(pDriverData->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}

	pDriverData->reportData.finger = newFinger;

	input_sync(pDriverData->input_dev);

}

static void report_key(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	u16 keyIndex = pReadData->keyData.index;
	u16 keyPressed = pReadData->keyData.pressed;
	u32 keyCode = pDriverData->mConfig.button_name[keyIndex-1];
	u32 reportedKeyIndex = pDriverData->reportData.key;
	u32 reportedKeyCode = pDriverData->mConfig.button_name[reportedKeyIndex-1];

	if (reportedKeyIndex == 0) {
		if (keyPressed == KEY_PRESSED) {
			input_report_key(pDriverData->input_dev, keyCode, KEY_PRESSED);
			TOUCH_LOG("KEY REPORT : PRESS KEY[%d]\n", keyCode);
			reportedKeyIndex = keyIndex;
		} else {
			TOUCH_WARN("Invalid key report from device(keyIndex=%d, Pressed=%d)\n", keyIndex, keyPressed);
		}
	} else {
		if (keyPressed == KEY_PRESSED) {
			/* release previous key first */
			input_report_key(pDriverData->input_dev, reportedKeyCode, KEY_RELEASED);
			TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", reportedKeyCode);
			input_sync(pDriverData->input_dev);

			/* report new key */
			input_report_key(pDriverData->input_dev, keyCode, KEY_PRESSED);
			TOUCH_LOG("KEY REPORT : PRESS KEY[%d]\n", keyCode);
			reportedKeyIndex = keyIndex;
		} else {
			if (reportedKeyIndex == keyIndex) {
				input_report_key(pDriverData->input_dev, keyCode, KEY_RELEASED);
				TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", keyCode);
				reportedKeyIndex = 0;
			} else {
				TOUCH_ERR("Invalid key report from device(keyIndex=%d, Pressed=%d)\n",
						keyIndex, keyPressed);

				/* protection code */
				input_report_key(pDriverData->input_dev, reportedKeyIndex, KEY_RELEASED);
				TOUCH_LOG("KEY REPORT : RELEASE KEY[%d]\n", reportedKeyIndex);
				reportedKeyIndex = 0;
			}
		}
	}

	pDriverData->reportData.key = reportedKeyIndex;

	input_sync(pDriverData->input_dev);
}

static void cancel_key(TouchDriverData *pDriverData)
{
	u32 reportedKeyIndex = pDriverData->reportData.key;
	u32 reportedKeyCode = pDriverData->mConfig.button_name[reportedKeyIndex-1];

	if (reportedKeyIndex) {
		input_report_key(pDriverData->input_dev, reportedKeyCode, KEY_CANCELED);
		TOUCH_LOG("KEY REPORT : CANCEL KEY[%d]\n", reportedKeyCode);
		reportedKeyIndex = 0;
	}

	pDriverData->reportData.key = reportedKeyIndex;

	input_sync(pDriverData->input_dev);
}

static void send_uevent(TouchDriverData *pDriverData, u8 eventIndex)
{
	if (eventIndex < NUM_OF_UEVENT) {
		wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(3000));
		kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, touch_uevent[eventIndex]);

		if (eventIndex == UEVENT_KNOCK_ON) {
			pDriverData->reportData.knockOn = 1;
		} else if (eventIndex == UEVENT_KNOCK_CODE) {
			pDriverData->reportData.knockCode = 1;
		} else if (eventIndex == UEVENT_SWIPE_UP ||
				eventIndex == UEVENT_SWIPE_DOWN) {
			pDriverData->reportData.knockCode = 1;
		} else {
			TOUCH_WARN("UEVENT_SIGNATURE is not supported\n");
		}
	} else {
		TOUCH_ERR("Invalid event index ( index = %d )\n", eventIndex);
	}
}

static void report_hover(TouchDriverData *pDriverData, TouchReadData *pReadData)
{
	if (pDriverData->reportData.hover != pReadData->hoverState) {
		pDriverData->reportData.hover = pReadData->hoverState;

		if (pReadData->hoverState == HOVER_NEAR) {
			/* TBD */
			TOUCH_LOG("HOVER REPORT : NEAR\n");
		} else {
			/* TBD */
			TOUCH_LOG("HOVER REPORT : FAR\n");
		}
	} else {
		TOUCH_WARN("Duplicated Hover Event from Device ( %d )\n", pReadData->hoverState);
	}
}

static void release_all_finger(TouchDriverData *pDriverData)
{
	if (pDriverData->reportData.finger) {
		TouchReadData readData;

		memset(&readData, 0x0, sizeof(TouchReadData));

		readData.type = DATA_FINGER;
		readData.count = 0;
		report_finger(pDriverData, &readData);
		TOUCH_LOG("all finger released\n");
	}
}

static void release_all_key(TouchDriverData *pDriverData)
{
	if (pDriverData->reportData.key) {
		TouchReadData readData;

		memset(&readData, 0x0, sizeof(TouchReadData));

		readData.type = DATA_KEY;
		readData.keyData.index = pDriverData->reportData.key;
		readData.keyData.pressed = KEY_RELEASED;
		report_key(pDriverData, &readData);
		TOUCH_LOG("all key released\n");
	}
}

static void release_all_touch_event(TouchDriverData *pDriverData)
{
	release_all_finger(pDriverData);

	if (pDriverData->mConfig.button_support)
		release_all_key(pDriverData);
}

static void WqTouchInit(struct work_struct *work_init)
{
	TouchDriverData *pDriverData = container_of(to_delayed_work(work_init),
		TouchDriverData, work_init);

	mutex_lock(pMutexTouch);

	TouchDisableIrq();
	release_all_touch_event(pDriverData);

	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);
	if (pDriverData->nextState == STATE_NORMAL_HOVER) {
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, pDriverData->nextState,
							&pDriverData->lpwgSetting);
	}

	SetDriverState(pDriverData, pDriverData->nextState);

	pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);
	TouchEnableIrq();

	mutex_unlock(pMutexTouch);
}

/****************************************************************************
* Interrupt Service Routine ( Triggered by HW Interrupt )
****************************************************************************/
#if defined(TOUCH_PLATFORM_QCT)
static irqreturn_t TouchIrqHandler(int irq, void *dev_id)
{
	if (atomic_read(&pm_state) >= PM_SUSPEND) {
		TOUCH_LOG("IRQ in suspend\n");
		atomic_set(&pm_state, PM_SUSPEND_IRQ);
		wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(3000));

		return IRQ_HANDLED;
	}

	if (pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}

	return IRQ_HANDLED;
}
#elif defined(TOUCH_PLATFORM_MTK)
static void TouchIrqHandler(void)
{
	if (pWorkTouch != NULL) {
		/* trigger work queue to process interrupt */
		queue_delayed_work(touch_wq, pWorkTouch, 0); /* It will call "WqTouchIrqHandler()" */
	}
}
#else
#error "Platform should be defined"
#endif


/****************************************************************************
* Triggered by ISR ( TouchIrqHandler() )
****************************************************************************/
static void WqTouchIrqHandler(struct work_struct *work_irq)
{
	 TouchDriverData *pDriverData = container_of(to_delayed_work(work_irq),
		 TouchDriverData, work_irq);
	 int ret = 0;

	 TouchReadData readData;

	mutex_lock(pMutexTouch);

	/* TBD : init buffer */
	memset(&readData, 0x0, sizeof(TouchReadData));
	readData.type = DATA_UNKNOWN;

	/* do TouchIC specific interrupt processing */
	ret = pDeviceSpecificFunc->InterruptHandler(pDriverData->client, &readData);

	/* do processing according to the data from TouchIC */
	if (readData.type == DATA_FINGER) {
		if (pDriverData->currState == STATE_NORMAL || pDriverData->currState == STATE_NORMAL_HOVER) {
#if defined(CONFIG_LGE_DYNAMIC_FPS)
			if (pDriverData->fpsChanged) {
				int fd;
				char fps_value[2] = "60";
				mm_segment_t old_fs = get_fs();

				set_fs(KERNEL_DS);

				fd = sys_open("sys/class/graphics/fb0/dynamic_fps", O_WRONLY, 0664);

				if (fd < 0) {
			        TOUCH_ERR("open error : sys/class/graphics/fb0/dynamic_fps\n");
					//return;
			    }

				sys_write(fd, fps_value, 2);
			    sys_close(fd);

				set_fs(old_fs);

				TOUCH_DBG("FPS is Changed to 60Hz\n");
				pDriverData->fpsChanged = 0;
				pDeviceSpecificFunc->NotifyHandler(pDriverData->client,
												NOTIFY_FPS_CHANGED, 1);
			}

#endif
			/* report cancel key to CFW */
			if ((pDriverData->mConfig.button_support) && (readData.count != 0))
				cancel_key(pDriverData);

			/* report finger data to CFW */
			report_finger(pDriverData, &readData);
		} else {
			TOUCH_WARN("Unmatched event from device ( event = FINGER )\n");
		}
	} else if (readData.type == DATA_KEY) {
		if (pDriverData->currState == STATE_NORMAL || pDriverData->currState == STATE_NORMAL_HOVER) {
			if (pDriverData->mConfig.button_support) {
				if (pDriverData->reportData.finger == 0) {
					/* report key to CFW */
					report_key(pDriverData, &readData);
				} else {
					TOUCH_LOG("Finger event exist so key event was ignored\n");
				}
			} else {
				TOUCH_ERR("Invalid event from device ( event = KEY )\n");
			}
		} else {
			TOUCH_WARN("Unmatched event from device ( event = KEY )\n");
		}
	} else if (readData.type == DATA_KNOCK_ON) {
		if (pDriverData->currState == STATE_KNOCK_ON_ONLY || pDriverData->currState == STATE_KNOCK_ON_CODE) {
			/* report knock-on event to CFW */
			send_uevent(pDriverData, UEVENT_KNOCK_ON);
		} else {
			TOUCH_WARN("Unmatched event from device ( event = KNOCK_ON )\n");
		}
	} else if (readData.type == DATA_KNOCK_CODE) {
		if (pDriverData->currState == STATE_KNOCK_ON_CODE) {
			pDriverData->reportData.knockCount = readData.count;
			memcpy(pDriverData->reportData.knockData, readData.knockData,
				readData.count * sizeof(TouchPoint));

			/* report knock-code event to CFW */
			send_uevent(pDriverData, UEVENT_KNOCK_CODE);
		} else {
			TOUCH_WARN("Unmatched event from device ( event = KNOCK_CODE )\n");
		}
	} else if (readData.type == DATA_SWIPE_UP || readData.type == DATA_SWIPE_DOWN) {
		if (lockscreen_stat) {
			swipe_check_call = 1;
			pDriverData->reportData.knockCount = readData.count;
			memcpy(pDriverData->reportData.knockData, readData.knockData,
				readData.count * sizeof(TouchPoint));

			/* report swipe event to CFW */
			if (readData.type == DATA_SWIPE_UP)
				send_uevent(pDriverData, UEVENT_SWIPE_UP);
			else
				send_uevent(pDriverData, UEVENT_SWIPE_DOWN);
		}
	} else if (readData.type == DATA_HOVER) {
		if (pDriverData->currState == STATE_NORMAL_HOVER || pDriverData->currState == STATE_HOVER) {
			/* report hover state */
			report_hover(pDriverData, &readData);
		} else {
			TOUCH_WARN("Unmatched event from device ( event = HOVER )\n");
		}
	} else if (readData.type == DATA_LPWG_FAIL) {
		/* Do nothing */
	} else {
		TOUCH_WARN("Unknown event from device ( event = %d, STATE = %d )\n", readData.type,
				pDriverData->currState);
	}

	mutex_unlock(pMutexTouch);

	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("Abnormal IC status. Touch IC will be reset\n");
		queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
	}
}


/****************************************************************************
* Global Functions
****************************************************************************/
static void WqfirmwareUpgrade(struct work_struct *work_upgrade)
{
	TouchDriverData *pDriverData = container_of(to_delayed_work(work_upgrade),
		TouchDriverData, work_upgrade);

	int i = 0;
	int result = TOUCH_SUCCESS;
	char *pFilename = NULL;

	mutex_lock(pMutexTouch);
	wake_lock(pWakeLockTouch);

	TOUCH_FUNC();

	TouchDisableIrq();

	SetDriverState(pDriverData, STATE_UPDATE_FIRMWARE);

	if (pDriverData->useDefaultFirmware == TOUCH_FALSE)
		pFilename = pDriverData->fw_image;

	for (i = 0; i < MAX_FW_UPGRADE_RETRY; i++) {
		result = pDeviceSpecificFunc->UpdateFirmware(pDriverData->client, pFilename);
		if (result == TOUCH_SUCCESS) {
			TOUCH_LOG("Firmware upgrade was Succeeded\n");
			break;
		}

		TOUCH_WARN("Retry firmware upgrade\n");
	}

	if (result == TOUCH_FAIL)
		TOUCH_ERR("failed to upgrade firmware\n");

	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);
	pDeviceSpecificFunc->ReadIcFirmwareInfo(pDriverData->client, &pDriverData->icFwInfo);

	SetDriverState(pDriverData, STATE_NORMAL);

	pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);
	TouchEnableIrq();

	wake_unlock(pWakeLockTouch);
	mutex_unlock(pMutexTouch);

}

static void UpdateLpwgSetting (LpwgSetting *pLpwgSetting, LpwgCmd lpwgCmd, int *param)
{
	u8 str[20] = {0};
	int rst = 0;

	switch (lpwgCmd) {
	case LPWG_CMD_MODE:
		pLpwgSetting->mode = param[0];
		WRITE_BUFFER(str, rst, "%s", "MODE");
		break;

	case LPWG_CMD_LCD_PIXEL_SIZE:
		pLpwgSetting->lcdPixelSizeX = param[0];
		pLpwgSetting->lcdPixelSizeY = param[1];
		WRITE_BUFFER(str, rst, "%s", "PIXEL_SIZE");
		break;

	case LPWG_CMD_ACTIVE_TOUCH_AREA:
		pLpwgSetting->activeTouchAreaX1 = param[0];
		pLpwgSetting->activeTouchAreaX2 = param[1];
		pLpwgSetting->activeTouchAreaY1 = param[2];
		pLpwgSetting->activeTouchAreaY2 = param[3];
		WRITE_BUFFER(str, rst, "%s", "ACTIVE_AREA");
		break;

	case LPWG_CMD_TAP_COUNT:
		pLpwgSetting->tapCount = param[0];
		WRITE_BUFFER(str, rst, "%s", "TAP_COUNT");
		break;

	case LPWG_CMD_TAP_DISTANCE:
		TOUCH_WARN("Invalide LPWG Command ( LPWG_CMD_TAP_DISTANCE )\n");
		break;

	case LPWG_CMD_LCD_STATUS:
		pLpwgSetting->lcdState = param[0];
		WRITE_BUFFER(str, rst, "%s", "LCD");
		break;

	case LPWG_CMD_PROXIMITY_STATUS:
		pLpwgSetting->proximityState = param[0];
		WRITE_BUFFER(str, rst, "%s", "PROXIMITY");
		break;

	case LPWG_CMD_FIRST_TWO_TAP:
		pLpwgSetting->isFirstTwoTapSame = param[0];
		WRITE_BUFFER(str, rst, "%s", "FIRST_TWO_TAP");
		break;

	case LPWG_CMD_UPDATE_ALL:
		pLpwgSetting->mode = param[0];
		pLpwgSetting->lcdState = param[1];
		pLpwgSetting->proximityState = param[2];
		pLpwgSetting->coverState = param[3];
		WRITE_BUFFER(str, rst, "%s", "ALL");
		break;

	case LPWG_CMD_CALL:
		pLpwgSetting->callState = param[0];
		WRITE_BUFFER(str, rst, "%s", "CALL");
		break;

	default:
		TOUCH_ERR("Invalide LPWG Command ( Type = %d )\n", lpwgCmd);
		break;
	}

	TOUCH_LOG("LPWG SETTING : CMD[%s] M[%d] L[%d] P[%d] Cover[%d] Call[%d]\n",
		str, pLpwgSetting->mode, pLpwgSetting->lcdState, pLpwgSetting->proximityState,
		pLpwgSetting->coverState, pLpwgSetting->callState);

}

static TouchState DecideNextDriverState(LpwgSetting *pLpwgSetting)
{
	TouchState nextState = STATE_UNKNOWN;

	if (pLpwgSetting->lcdState == 1) {
		if (pLpwgSetting->callState == 2)
			nextState = STATE_NORMAL_HOVER;
		else
			nextState = STATE_NORMAL;
	} else {
		if (pLpwgSetting->callState == 2) {
			nextState = STATE_HOVER;
		} else {
			if (pLpwgSetting->mode == 0) {
				nextState = STATE_OFF;
			} else if (pLpwgSetting->mode == 1) {
				nextState = STATE_KNOCK_ON_ONLY;
			} else if (pLpwgSetting->mode == 2) {
				nextState = STATE_KNOCK_ON_CODE; /* TBD : CFW Bug but Cover it. Report it to CFW. */
			} else if (pLpwgSetting->mode == 3) {
				nextState = STATE_KNOCK_ON_CODE;
			} else {
				TOUCH_ERR("Invalid Mode Setting ( mode = %d )\n", pLpwgSetting->mode);
				nextState = STATE_KNOCK_ON_CODE;
			}
		}
	}

	return nextState;
}

/****************************************************************************
* CFW will use it to check if knock-on is supported or not.
* If you return "0", CFW will not send LPWG command.
****************************************************************************/
static ssize_t show_knock_on_type(struct i2c_client *client, char *buf)
{
	int ret = 0;

	WRITE_SYSBUF(buf, ret, "%d\n", 1);

	return ret;
}


/****************************************************************************
* CFW will use it to send LPWG command.
****************************************************************************/
static ssize_t store_lpwg_notify(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	LpwgCmd lpwgCmd = LPWG_CMD_UNKNOWN;
	LpwgSetting *pLpwgSetting = NULL;
	TouchState nextState = STATE_UNKNOWN;

	int type = 0;
	int value[4] = {0};

	/* Get command and parameter from buffer */
	sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);

	lpwgCmd = (LpwgCmd)type;

	/* load stored previous setting */
	pLpwgSetting = &pDriverData->lpwgSetting;

	/* Notify additional processing */
	if (lpwgCmd == LPWG_CMD_CALL)
	{
		pDeviceSpecificFunc->NotifyHandler(pDriverData->client, NOTIFY_CALL, value[0]);
		#if defined(ENABLE_NOTIFY_CALL_STATE)
		if ( value[0] == 1 && swipe_check_call == 1 )
		{
			mutex_lock(pMutexTouch);
			TouchDisableIrq();
			release_all_touch_event(pDriverData);
			pDeviceSpecificFunc->Reset(pDriverData->client);
			pDeviceSpecificFunc->InitRegister(pDriverData->client);
			pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);
			TouchEnableIrq();
			mutex_unlock(pMutexTouch);
		}
		#endif
	}
	else if (lpwgCmd == LPWG_CMD_UPDATE_ALL)
		pDeviceSpecificFunc->NotifyHandler(pDriverData->client, NOTIFY_Q_COVER, value[3]);

	#if !defined(ENABLE_HOVER_DETECTION)
	if (lpwgCmd == LPWG_CMD_CALL)
		return count;
	#endif

	mutex_lock(pMutexTouch);

	/* update new lpwg setting */
	UpdateLpwgSetting(pLpwgSetting, lpwgCmd, value);

	if ((lpwgCmd != LPWG_CMD_UPDATE_ALL) && (lpwgCmd != LPWG_CMD_CALL)) {
		mutex_unlock(pMutexTouch);
		return count;
	}

	/* decide next driver state */
	nextState = DecideNextDriverState(pLpwgSetting);

	/* apply it using device driver function */
	if ((nextState != STATE_UNKNOWN) && (pDriverData->currState != nextState)) {

		#if defined(TOUCH_TYPE_INCELL)
		if ((pDriverData->currState != STATE_NORMAL) && (nextState != STATE_NORMAL)) {
			pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState, &pDriverData->lpwgSetting);
			SetDriverState(pDriverData, nextState);
		} else {
			/* store next state to use later ( suspend or resume ) */
			pDriverData->nextState = nextState;
			TOUCH_LOG("LPWG Setting will be processed on suspend or resume\n");
		}

		#else /* General Add-on type touch */

		if (nextState == STATE_NORMAL || nextState == STATE_NORMAL_HOVER) {
			if ((pDriverData->currState == STATE_NORMAL) ||
				(pDriverData->currState == STATE_NORMAL_HOVER)) {
				pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState,
									&pDriverData->lpwgSetting);
				SetDriverState(pDriverData, nextState);
			} else {
				pDriverData->nextState = nextState;
				queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
			}
		} else {
			if ((pDriverData->currState == STATE_NORMAL) || (pDriverData->currState == STATE_NORMAL_HOVER))
				release_all_touch_event(pDriverData);

			pDeviceSpecificFunc->SetLpwgMode(pDriverData->client, nextState, &pDriverData->lpwgSetting);
			SetDriverState(pDriverData, nextState);
		}

		#endif
	}

	mutex_unlock(pMutexTouch);

	return count;
}

/****************************************************************************
* CFW will use it to read knock-code data
****************************************************************************/
static ssize_t show_lpwg_data(struct i2c_client *client, char *buf)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	int i = 0;
	int ret = 0;

	mutex_lock(pMutexTouch);

	/* We already get the data on ISR, so we can return the data immediately */
	for (i = 0; i < pDriverData->reportData.knockCount; i++) {
		if (pDriverData->reportData.knockData[i].x == -1 && pDriverData->reportData.knockData[i].y == -1)
			break;

		WRITE_SYSBUF(buf, ret, "%d %d\n", pDriverData->reportData.knockData[i].x,
				pDriverData->reportData.knockData[i].y);
	}

	TOUCH_LOG("LPWG data was read by CFW\n");

	mutex_unlock(pMutexTouch);

	return ret;
}

/****************************************************************************
* CFW will use it to give a feedback ( result ) on the event we sent before on ISR
****************************************************************************/
static ssize_t store_lpwg_data(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int reply = 0;

	sscanf(buf, "%d", &reply);

	mutex_lock(pMutexTouch);

	if (reply == 1) {
		TOUCH_LOG("LPWG result was informed by CFW ( Code is matched )\n");
		/* Code is matched, do something for leaving "LPWG Mode" if you need.
		But normally "Resume" will be triggered soon. */
	} else {
		TOUCH_LOG("LPWG result was informed by CFW ( Code is NOT matched )\n");
	}

	/* clear knock data */
	pDriverData->reportData.knockOn = 0;
	pDriverData->reportData.knockCode = 0;
	memset(pDriverData->reportData.knockData, 0x00, sizeof(pDriverData->reportData.knockData));

	wake_unlock(pWakeLockTouch);
	mutex_unlock(pMutexTouch);

	return count;
}

/****************************************************************************
* show_firmware is uesd for firmware information in hidden menu
****************************************************************************/
static ssize_t show_firmware(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	WRITE_SYSBUF(buf, ret, "\n======== IC Firmware Info ========\n");

	WRITE_SYSBUF(buf, ret,	"FW Version: %d.%02d\n", pDriverData->icFwInfo.isOfficial,
			pDriverData->icFwInfo.version);

	WRITE_SYSBUF(buf, ret, "\n====== Binary Firmware Info ======\n");

	WRITE_SYSBUF(buf, ret,	"Bin Version: %d.%02d\n", pDriverData->binFwInfo.isOfficial,
			pDriverData->binFwInfo.version);

	return ret;
}

/****************************************************************************
* "at%touchfwver" will use it to get firmware information of touch IC
****************************************************************************/
static ssize_t show_atcmd_fw_ver(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	WRITE_SYSBUF(buf, ret, "V%d.%02d (%d/%d/%d)\n",
			pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version,
			pDriverData->icFwInfo.moduleMakerID, pDriverData->icFwInfo.moduleVersion,
			pDriverData->icFwInfo.modelID);

	mutex_unlock(pMutexTouch);

	return ret;
}

/****************************************************************************
* "testmode" will use it to get firmware information of touch IC
****************************************************************************/
static ssize_t show_testmode_fw_ver(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	WRITE_SYSBUF(buf, ret, "V%d.%02d (%d/%d/%d)\n",
			pDriverData->icFwInfo.isOfficial, pDriverData->icFwInfo.version,
			pDriverData->icFwInfo.moduleMakerID, pDriverData->icFwInfo.moduleVersion,
			pDriverData->icFwInfo.modelID);

	mutex_unlock(pMutexTouch);

	return ret;
}


/****************************************************************************
* AAT will use it to get the result of self-diagnosis
****************************************************************************/
static ssize_t show_sd_info(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int channelStatus = 0;
	int rawStatus = 0;
	u8 *pBuf = NULL;
	int bufSize = 2*1024;
	int dataLen = 0;

	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	/* allocate buffer for additional debugging information */
	pBuf = kzalloc(bufSize, GFP_KERNEL);
	if (pBuf == NULL) {
		TOUCH_ERR("failed to allocate memory for self diagnosis\n");
		return ret;
	}

	TouchDisableIrq();

	SetDriverState(pDriverData, STATE_SELF_DIAGNOSIS);

	/* TBD : consider in case of LCD Off ( means LPWG mode ) */
	pDeviceSpecificFunc->DoSelfDiagnosis(client, &rawStatus, &channelStatus, pBuf, bufSize, &dataLen);
	pDeviceSpecificFunc->Reset(pDriverData->client);
	pDeviceSpecificFunc->InitRegister(pDriverData->client);

	SetDriverState(pDriverData, STATE_NORMAL);

	pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);
	TouchEnableIrq();

	mutex_unlock(pMutexTouch);

	/* basic information ( return data string format can't be changed ) */
	WRITE_SYSBUF(buf, ret, "========RESULT=======\n");
	WRITE_SYSBUF(buf, ret, "Channel Status : %s", (channelStatus == TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	WRITE_SYSBUF(buf, ret, "Raw Data : %s", (rawStatus == TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");

	if (dataLen > bufSize) {
		TOUCH_ERR("buffer size was overflowed ( use size = %d )\n", dataLen);
		kfree(pBuf);
		return ret;
	}

	/* addition information for debugging */
	memcpy(buf+ret, pBuf, dataLen);
	ret += dataLen;

	kfree(pBuf);

	return ret;
}

#if defined(TOUCH_PLATFORM_MTK)
static ssize_t store_mfts_onoff(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	int isOn = 0;

	sscanf(buf, "%d", &isOn);

	if (isOn < 0 || isOn > 1) {
		TOUCH_WARN("Invalid input value ( isOn = %d )\n", isOn);
		return count;
	}

	if (isOn) {
		TouchPower(1);
		queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
	} else {
		TouchPower(0);
	}

	return count;
}
#endif

/****************************************************************************
* Developer will use it to upgrade firmware with filename
****************************************************************************/
static ssize_t store_upgrade(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	if (count > MAX_FILENAME)
		return count;

	mutex_lock(pMutexTouch);

	pDriverData->useDefaultFirmware = TOUCH_FALSE;

	memset(pDriverData->fw_image, 0x00, sizeof(pDriverData->fw_image));
	sscanf(buf, "%s", pDriverData->fw_image);

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);

	mutex_unlock(pMutexTouch);

	return count;
}

/****************************************************************************
* Developer will use it to upgrade firmware with default firmware
****************************************************************************/
static ssize_t store_rewrite_bin_fw(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	mutex_lock(pMutexTouch);

	pDriverData->useDefaultFirmware = TOUCH_TRUE;

	queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);

	mutex_unlock(pMutexTouch);

	return count;
}

/****************************************************************************
* Developer will use it to read register from or write register to touch IC
* To Write : echo write $reg $data > ic_rw
* To Read : echo read $reg > ic_rw ==> cat > ic_rw
****************************************************************************/
static int regAddr, readCount;
static ssize_t show_ic_rw(struct i2c_client *client, char *buf)
{
	int result = 0;
	int ret = 0;
	int readValue = 0;

	int i = 0;

	mutex_lock(pMutexTouch);

	for (i = 0; i < readCount; i++) {
		result = pDeviceSpecificFunc->AccessRegister(client, READ_IC_REG, regAddr, &readValue);
		if (result == TOUCH_FAIL) {
			TOUCH_ERR("failed to read register ( reg = %d )\n", regAddr);
			break;
		}

		WRITE_SYSBUF(buf, ret, "0x%04X=%d\n", regAddr, readValue);
		regAddr++;
	}

	mutex_unlock(pMutexTouch);

	TOUCH_DBG("Read IC Register ( Begin )\n");
	TOUCH_DBG("%s\n", buf);
	TOUCH_DBG("Read IC Register ( End )\n");

	return ret;
}

static ssize_t store_ic_rw(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;

	u8 cmd[30] = {0};
	int reg = 0;
	int data = 0;

	sscanf(buf, "%s %d %d", cmd, &reg, &data);

	if ((strcmp(cmd, "write") && strcmp(cmd, "read")))
		return count;

	mutex_lock(pMutexTouch);

	if (strcmp(cmd, "write") == 0) {
		ret = pDeviceSpecificFunc->AccessRegister(client, WRITE_IC_REG, reg, &data);
		if (ret == TOUCH_FAIL)
			TOUCH_ERR("failed to write register ( reg = %d, data =%d )\n", reg, data);
		else
			TOUCH_DBG("Success to write register ( reg = %d, data =%d )\n", reg, data);
	} else {
		TOUCH_DBG("Ready to read register ( reg = %d )\n", reg);
		regAddr = reg;
		readCount = data;
		if (readCount == 0)
			readCount = 1;
	}

	mutex_unlock(pMutexTouch);

	return count;
}

/****************************************************************************
* show_touch_reset is uesd for camera test of low temperature
****************************************************************************/
static ssize_t show_touch_reset(struct i2c_client *client, char *buf)
{
	int ret = 0, retVal = 0;
	struct file *fd = NULL;
	char temp[6] = {0,};
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	fd = filp_open("/sys/class/power_supply/battery/temp", O_RDONLY , 0666);
	if(!IS_ERR(fd))
	{
		retVal = fd->f_op->read(fd, temp, 5, &(fd->f_pos));
		filp_close(fd, NULL);
		TOUCH_LOG("File Open Success for battery temp\n");
	}

	if ( (retVal == 5) && (temp[0] == '-') && (pDriverData->currState == STATE_NORMAL) )
	{
		if ( (temp[1] >= '1') && (temp[2] >= '0') )
		{
			TOUCH_LOG("temperature = %c%c%c\n", temp[0], temp[1], temp[2]);
			queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		}
	}

	set_fs(old_fs);

	return ret;
}

static ssize_t store_keyguard_info(struct i2c_client *client,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	switch (value) {
	case 0:
		lockscreen_stat = 0;
		TOUCH_DBG("Lockscreen unlocked\n");
		break;
	case 1:
		lockscreen_stat = 1;
		TOUCH_DBG("Lockscreen locked\n");
		break;
	default:
	    break;
	}

	return count;
}

static LGE_TOUCH_ATTR(keyguard, S_IRUGO | S_IWUSR, NULL, store_keyguard_info);
static LGE_TOUCH_ATTR(knock_on_type, S_IRUGO | S_IWUSR, show_knock_on_type, NULL);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_testmode_fw_ver, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd_info, NULL);
#if defined(TOUCH_PLATFORM_MTK)
static LGE_TOUCH_ATTR(mfts_onoff, S_IRUGO | S_IWUSR, NULL, store_mfts_onoff);
#endif
static LGE_TOUCH_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, store_upgrade);
static LGE_TOUCH_ATTR(rewrite_bin_fw, S_IRUGO | S_IWUSR, NULL, store_rewrite_bin_fw);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, show_ic_rw, store_ic_rw);
static LGE_TOUCH_ATTR(touch_reset, S_IRUGO | S_IWUSR, show_touch_reset, NULL);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_keyguard.attr,
	&lge_touch_attr_knock_on_type.attr,
	&lge_touch_attr_lpwg_notify.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_testmode_ver.attr,
#if defined(TOUCH_PLATFORM_MTK)
	&lge_touch_attr_mfts_onoff.attr,
#endif
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_fw_upgrade.attr,
	&lge_touch_attr_rewrite_bin_fw.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_touch_reset.attr,
	NULL,
};

static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj,
	struct attribute *attr, char *buf)
{
	TouchDriverData *pDriverData = container_of(lge_touch_kobj,
		TouchDriverData, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(pDriverData->client, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj,
	struct attribute *attr,
			      const char *buf, size_t count)
{
	TouchDriverData *pDriverData = container_of(lge_touch_kobj,
		TouchDriverData, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(pDriverData->client, buf, count);

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

static int get_attribute_array_size(struct attribute **list)
{
	int i = 0;

	while (list[i] != NULL && i < MAX_ATTRIBUTE_ARRAY_SIZE)
		i++;

	return i <= MAX_ATTRIBUTE_ARRAY_SIZE ? i : 0;
}

static int sysfs_register(TouchDriverData *pDriverData,
	struct attribute **attribute_list)
{
	struct attribute **new_attribute_list;

	int ret = 0;
	int n1 = get_attribute_array_size(lge_touch_attribute_list);
	int n2 = attribute_list ? get_attribute_array_size(attribute_list) : 0;

	TOUCH_FUNC();

	new_attribute_list = devm_kzalloc(&pDriverData->client->dev, (n1+n2+1) * sizeof(struct attribute *),
						GFP_KERNEL);
	if (new_attribute_list == NULL) {
		TOUCH_ERR("Fail to allocation memory\n");
		return -ENOMEM;
	}

	memcpy(new_attribute_list, lge_touch_attribute_list, n1 * sizeof(struct attribute *));

	if (attribute_list)
		memcpy(new_attribute_list + n1, attribute_list, n2 * sizeof(struct attribute *));

	lge_touch_kobj_type.default_attrs = new_attribute_list;

	ret = subsys_system_register(&touch_subsys, NULL);
	if (ret < 0) {
		TOUCH_ERR("Fail to register subsys ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = device_register(&device_touch);
	if (ret < 0) {
		TOUCH_ERR("Fail to register device ( error = %d )\n", ret);
		return -ENODEV;
	}

	ret = kobject_init_and_add(&pDriverData->lge_touch_kobj,
			&lge_touch_kobj_type,
			pDriverData->input_dev->dev.kobj.parent, "%s", LGE_TOUCH_NAME);
	if (ret < 0) {
		TOUCH_ERR("Fail to init and add kobject ( error = %d )\n", ret);
		device_unregister(&device_touch);
		return -ENODEV;
	}

	return TOUCH_SUCCESS;
}

static void sysfs_unregister(TouchDriverData *pDriverData)
{
	kobject_del(&pDriverData->lge_touch_kobj);
	device_unregister(&device_touch);
	devm_kfree(&pDriverData->client->dev, lge_touch_kobj_type.default_attrs);
}

static int register_input_dev(TouchDriverData *pDriverData)
{
	int ret = 0;
	int idx = 0;

	TOUCH_FUNC();

	pDriverData->input_dev = input_allocate_device();
	if (pDriverData->input_dev == NULL) {
		TOUCH_ERR("failed at input_allocate_device()\n");
		return TOUCH_FAIL;
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

	if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B) {
		#if defined(KERNEL_ABOVE_3_4_67)
		ret = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id, 0);
		#else
		ret = input_mt_init_slots(pDriverData->input_dev, pDriverData->mConfig.max_id);
		#endif
		if (ret < 0) {
			TOUCH_ERR("failed at input_mt_init_slots() ( error = %d )\n", ret);
			input_free_device(pDriverData->input_dev);
			return TOUCH_FAIL;
		}
	} else {
		input_set_abs_params(pDriverData->input_dev, ABS_MT_TRACKING_ID, 0, pDriverData->mConfig.max_id, 0, 0);
	}

	if (pDriverData->mConfig.button_support) {
		set_bit(EV_KEY, pDriverData->input_dev->evbit);
		for (idx = 0; idx < pDriverData->mConfig.number_of_button; idx++)
			set_bit(pDriverData->mConfig.button_name[idx], pDriverData->input_dev->keybit);
	}

	ret = input_register_device(pDriverData->input_dev);
	if (ret < 0) {
		TOUCH_ERR("failed at input_register_device() ( error = %d )\n", ret);
		if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B)
			input_mt_destroy_slots(pDriverData->input_dev);

		input_free_device(pDriverData->input_dev);
		return TOUCH_FAIL;
	}

	input_set_drvdata(pDriverData->input_dev, pDriverData);

	return TOUCH_SUCCESS;
}

static void unregister_input_dev(TouchDriverData *pDriverData)
{
	if (pDriverData->mConfig.protocol_type == MT_PROTOCOL_B)
		input_mt_destroy_slots(pDriverData->input_dev);

	input_unregister_device(pDriverData->input_dev);

}

static int touch_common_notifier_callback(struct notifier_block *self,
											unsigned long event, void *data)
{
	TouchDriverData *pDriverData = container_of(self, TouchDriverData, common_notif);

	mutex_lock(pMutexTouch);

	switch (event) {
	case LCD_EVENT_TOUCH_LPWG_ON:
		swipe_check_call = 0;
		cancel_delayed_work_sync(&pDriverData->work_init);
		release_all_touch_event(pDriverData);

		pDriverData->lpwgSetting.lcdState = 0;
		TOUCH_DBG("[IN] LCD Sleep\n");

		pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client,
							pDriverData->nextState, &pDriverData->lpwgSetting);
		SetDriverState(pDriverData, pDriverData->nextState);
		break;
	case LCD_EVENT_TOUCH_LPWG_OFF:
		pDriverData->lpwgSetting.lcdState = 1;
		TOUCH_DBG("[OUT] LCD Sleep\n");

		pDriverData->nextState = DecideNextDriverState(&pDriverData->lpwgSetting);
		pDeviceSpecificFunc->SetLpwgMode(pDriverData->client,
							pDriverData->nextState, &pDriverData->lpwgSetting);
		SetDriverState(pDriverData, pDriverData->nextState);

		queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		break;
	case LCD_EVENT_FPS_CHANGED:
		pDriverData->fpsChanged = 1;
		pDeviceSpecificFunc->NotifyHandler(pDriverData->client,
											NOTIFY_FPS_CHANGED, 0);
		break;
	default:
		break;
	}

	mutex_unlock(pMutexTouch);

	return TOUCH_SUCCESS;
}

static void touch_ta_status_callback(struct power_supply *psy)
{
	TouchDriverData *pDriverData = container_of(psy, TouchDriverData, ta_status);
	struct power_supply *status = NULL;
	union power_supply_propval ret;
	static int old_ta_status = 0;

	mutex_lock(pMutexTouch);

	status = power_supply_get_by_name("ac");
	if (status != NULL) {
		goto touch_ta_check_online;
	}

	status = power_supply_get_by_name("usb");
	if (status != NULL) {
		goto touch_ta_check_online;
	}

	mutex_unlock(pMutexTouch);

	return;

touch_ta_check_online:
	status->get_property(status, POWER_SUPPLY_PROP_ONLINE, &ret);

	if (old_ta_status != ret.intval) {
		TOUCH_DBG("[TA] %s\n", ret.intval ? "Connected" : "Disconnected");

		old_ta_status = ret.intval;
		pDeviceSpecificFunc->NotifyHandler(pDriverData->client,
											NOTIFY_TA_STATUS, ret.intval);
	}

	mutex_unlock(pMutexTouch);
}

static int touch_pm_suspend(struct device *dev)
{
	TOUCH_FUNC();

#if defined(TOUCH_PLATFORM_QCT)
	atomic_set(&pm_state, PM_SUSPEND);
#endif

	return TOUCH_SUCCESS;
}

static int touch_pm_resume(struct device *dev)
{
	TOUCH_FUNC();

#if defined(TOUCH_PLATFORM_QCT)
	if (atomic_read(&pm_state) == PM_SUSPEND_IRQ) {
		TouchDriverData *pDriverData = dev_get_drvdata(dev);
		struct irq_desc *desc;
		int nCnt = 0;

		do {
			desc = irq_to_desc(pDriverData->client->irq);
			if (desc == NULL)
				msleep(100);
		} while (desc == NULL && nCnt++ < 3);

		if (nCnt >= 3) {
			TOUCH_LOG("Null Pointer from irq_to_desc\n");
			return -ENOMEM;
		}

		atomic_set(&pm_state, PM_RESUME);

		irq_set_pending(pDriverData->client->irq);
		check_irq_resend(desc, pDriverData->client->irq);

		return TOUCH_SUCCESS;
	}

	atomic_set(&pm_state, PM_RESUME);
#endif

	return TOUCH_SUCCESS;
}

static int touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	TouchDriverData *pDriverData;
	struct attribute **specific_attribute_list = NULL;

	TOUCH_FUNC();

	/* memory allocation for globally used driver data */
	pDriverData = devm_kzalloc(&client->dev, sizeof(TouchDriverData), GFP_KERNEL);
	if (pDriverData == NULL) {
		TOUCH_ERR("fail to allocation memory\n");
		return -ENOMEM;
	}

	/* get boot mode for later */
	pDriverData->bootMode = TouchGetBootMode();

	/* set driver state to boot */
	SetDriverState(pDriverData, STATE_BOOT);

	/* set driver data to i2c client data */
	pDriverData->client = client;
	i2c_set_clientdata(client, pDriverData);

	/* get model configuratio */
	TouchGetModelConfig(pDriverData);

	/* register input device */
	ret = register_input_dev(pDriverData);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("failed at register_input_dev()\n");
		goto exit_register_input_dev_fail;
	}

	/* register sysfs ( core driver + TouchIC specific ) */
	specific_attribute_list = pDeviceSpecificFunc->device_attribute_list;
	ret = sysfs_register(pDriverData, specific_attribute_list);
	if (ret < 0) {
		TOUCH_ERR("failed at sysfs_register()\n");
		goto exit_sysfs_register_fail;
	}

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

	ret = pDeviceSpecificFunc->Initialize(client);
	if (ret  == TOUCH_FAIL)
		goto exit_device_init_fail;

	ret = pDeviceSpecificFunc->InitRegister(client);
	if (ret  == TOUCH_FAIL)
		goto exit_device_init_fail;

	ret = pDeviceSpecificFunc->ReadIcFirmwareInfo(client, &pDriverData->icFwInfo);
	if (ret  == TOUCH_FAIL)
		goto exit_device_init_fail;

	ret = pDeviceSpecificFunc->GetBinFirmwareInfo(client, NULL, &pDriverData->binFwInfo);
	if (ret  == TOUCH_FAIL)
		goto exit_device_init_fail;

	if (pDriverData->bootMode == BOOT_OFF_CHARGING) {
		pDriverData->lpwgSetting.mode = 0; /* there is no CFW, but we need to save power */
	} else {
		/* Register & Disable Interrupt */
		ret = TouchRegisterIrq(pDriverData, (irq_handler_t)TouchIrqHandler);
		if (ret == TOUCH_FAIL)
			goto exit_device_init_fail;
	}

	TouchDisableIrq();

	/* set and register touch cmd callback*/
	pDriverData->common_notif.notifier_call = touch_common_notifier_callback;
	touch_register_client(&pDriverData->common_notif);

	/* register ta status callback */
	pDriverData->ta_status.name = "touch";
	pDriverData->ta_status.type = POWER_SUPPLY_TYPE_UNKNOWN;
	pDriverData->ta_status.external_power_changed = touch_ta_status_callback;
	ret = power_supply_register(&client->dev, &pDriverData->ta_status);
	if (ret < 0) {
		TOUCH_ERR("failed at power_supply_register()\n");
		goto exit_device_init_fail;
	}

	/* Set driver state to normal */
	SetDriverState(pDriverData, STATE_NORMAL);

	/* clear interrupt of touch IC */
	pDeviceSpecificFunc->ClearInterrupt(pDriverData->client);

	/* Enable interrupt of AP */
	TouchEnableIrq();

	/* Trigger firmware upgrade if needed */
	if (pDriverData->icFwInfo.isOfficial) {
		if (pDriverData->binFwInfo.version != pDriverData->icFwInfo.version) {
			pDriverData->useDefaultFirmware = TOUCH_TRUE;
			queue_delayed_work(touch_wq, &pDriverData->work_upgrade, 0);
		} else {
			queue_delayed_work(touch_wq, &pDriverData->work_init, 0);
		}
	}

	return TOUCH_SUCCESS;

exit_device_init_fail:

	wake_lock_destroy(&pDriverData->lpwg_wake_lock);

	sysfs_unregister(pDriverData);

exit_sysfs_register_fail:

	unregister_input_dev(pDriverData);

exit_register_input_dev_fail:

	devm_kfree(&pDriverData->client->dev, pDriverData);

	return TOUCH_FAIL;

}

static int touch_remove(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();

	free_irq(pDriverData->client->irq, pDriverData);

	wake_lock_destroy(&pDriverData->lpwg_wake_lock);

	sysfs_unregister(pDriverData);

	unregister_input_dev(pDriverData);

	devm_kfree(&pDriverData->client->dev, pDriverData);

	return TOUCH_SUCCESS;
}


static struct i2c_device_id lge_ts_id[] = {
	{LGE_TOUCH_NAME, 0},
};

static const struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_pm_suspend,
	.resume = touch_pm_resume,
};


static struct i2c_driver lge_touch_driver = {
	.probe = touch_probe,
	.remove = touch_remove,
	.id_table = lge_ts_id,
	.driver = {
		.name = LGE_TOUCH_NAME,
		.owner = THIS_MODULE,
		.pm = &touch_pm_ops,
	},
};

static int __init touch_init(void)
{
	int result = TOUCH_SUCCESS;
	int ret = 0;
	int idx = touch_module;
	TouchDeviceSpecificFunction **pDeviceFunc = NULL;
	struct of_device_id **pMatchtableList = NULL;

	TOUCH_FUNC();

	TouchGetDeviceSpecificDriver(&pDeviceFunc);
	TouchGetDeviceSpecificMatchTable(&pMatchtableList);

	ret = TouchInitializePlatform();
	if (ret == TOUCH_FAIL)
		return -ENODEV;

	#if defined(TOUCH_PLATFORM_MTK)
	i2c_register_board_info(0, &i2c_tpd, 1);
	#endif

	/* turn on the power */
	TouchPower(1);

	/* reset */
	pDeviceFunc[idx]->Reset(NULL);

	/* check if device is connected */
	result = pDeviceFunc[idx]->Connect();
	if (result == TOUCH_FAIL) {
		TOUCH_ERR("No device connected\n");
		return -ENODEV;
	}

	if (pDeviceFunc[idx] == NULL) {
		TOUCH_ERR("No device connected\n");
		return -ENODEV;
	}

	pDeviceSpecificFunc = pDeviceFunc[idx];
	lge_touch_driver.driver.of_match_table = pMatchtableList[idx];
	TOUCH_LOG("Touch Match table = %s\n", pMatchtableList[idx]->compatible);

	touch_wq = create_singlethread_workqueue("touch_wq");
	if (touch_wq == NULL) {
		TOUCH_ERR("failed to create workqueue\n");
		return -ENODEV;
	}

	if (i2c_add_driver(&lge_touch_driver)) {
		TOUCH_ERR("failed at i2c_add_driver()\n");
		destroy_workqueue(touch_wq);
		return -ENODEV;
	}

	return TOUCH_SUCCESS;

}

static void __exit touch_exit(void)
{
	TOUCH_FUNC();

	i2c_del_driver(&lge_touch_driver);

	if (touch_wq)
		destroy_workqueue(touch_wq);
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("D3 BSP Touch Team");
MODULE_DESCRIPTION("LGE Touch Unified Driver");
MODULE_LICENSE("GPL");

/* End Of File */

