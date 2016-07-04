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
#define LGTP_MODULE "[MIT300]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_4/lgtp_common.h>

#include <linux/input/unified_driver_4/lgtp_common_driver.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_i2c.h>
#include <linux/input/unified_driver_4/lgtp_platform_api_misc.h>
#include <linux/input/unified_driver_4/lgtp_device_mit300_m2.h>


/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
#if defined( TOUCH_MODEL_M2)
static const char defaultFirmware[] = "melfas/mit300/m2/melfas_mip4.bin";
#endif

#if defined(TOUCH_MODEL_PH1)
static const char defaultFirmware[] = "melfas/mit300/ph1/melfas_mip4.bin";
#endif
#define BLU_JITTER_MAX  40
#define BLU_AVERAGE_JITTER  27

static struct melfas_ts_data *ts = NULL;
struct delayed_work work_cover;
struct delayed_work work_backlight_off;
struct delayed_work work_backlight_on;
int cover_status = 0;

#if defined(ENABLE_SWIPE_MODE)
static int get_swipe_mode = 1;
/*
static int wakeup_by_swipe = 0;
*/
extern int lockscreen_stat;
#endif

#if defined(CONFIG_MACH_MSM8916_M216N_KR) || defined(CONFIG_MACH_MSM8916_M216_GLOBAL_COM)
extern void rt8542_backlight_off(void);
extern void rt8542_backlight_on(int level);
#endif
#if defined(CONFIG_MACH_MSM8916_M209_TRF_US) || defined(CONFIG_MACH_MSM8916_M209_TRF_US_VZW)
extern void sm5306_backlight_off(void);
extern void sm5306_backlight_on(int level);
#endif

extern int max_data;
extern int min_data;
extern int jitter_average_max;
extern struct workqueue_struct* touch_wq;
extern int cradle_smart_cover_status(void);
extern s16 mit_data[MAX_ROW][MAX_COL];
/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern bool lge_get_mfts_mode(void);



/****************************************************************************
* Local Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Functions
****************************************************************************/
static void change_cover_func(struct work_struct *work_cover)
{
    u8 wbuf[4];
    struct i2c_client *client = Touch_Get_I2C_Handle();
    
    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_WINDOW_MODE;
    wbuf[2] = cover_status;
       
    if( Mit300_I2C_Write(client, wbuf, 3) )
    {
       TOUCH_ERR("change_cover_func failed\n");
    }else{
       TOUCH_LOG("change_cover_func status=%d\n",cover_status);
    }
}

static void backlight_off_func(struct work_struct *work_cover)
{
#if defined(CONFIG_MACH_MSM8916_M216N_KR) || defined(CONFIG_MACH_MSM8916_M216_GLOBAL_COM)
	rt8542_backlight_off();
#endif
#if defined(CONFIG_MACH_MSM8916_M209_TRF_US) || defined(CONFIG_MACH_MSM8916_M209_TRF_US_VZW)
	sm5306_backlight_off();
#endif
}
static void backlight_on_func(struct work_struct *work_cover)
{
#if defined(CONFIG_MACH_MSM8916_M216N_KR) || defined(CONFIG_MACH_MSM8916_M216_GLOBAL_COM)
	rt8542_backlight_on(255);
#endif
#if defined(CONFIG_MACH_MSM8916_M209_TRF_US) || defined(CONFIG_MACH_MSM8916_M209_TRF_US_VZW)
	sm5306_backlight_on(255);
#endif
}


void MIT300_Set_BootCoverMode(int status)
{
	INIT_DELAYED_WORK(&work_cover, change_cover_func);
    INIT_DELAYED_WORK(&work_backlight_off, backlight_off_func);
    INIT_DELAYED_WORK(&work_backlight_on, backlight_on_func);
	if(lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K || lge_get_boot_mode() == LGE_BOOT_MODE_QEM_56K) {
		TOUCH_ERR("LGE_BOOT_MODE = %d\n", lge_get_boot_mode());
	} else {
		cover_status = status;
	}
}

void MIT300_Set_CoverMode(int status)
{
	if(lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K || lge_get_boot_mode() == LGE_BOOT_MODE_QEM_56K) {
		TOUCH_ERR("LGE_BOOT_MODE = %d\n", lge_get_boot_mode());
	}
}


static void MIT300_WriteFile(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n", my_date.tm_mon + 1,my_date.tm_mday, my_date.tm_hour, my_date.tm_min, my_date.tm_sec, (unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}


/****************************************************************************
* Device Specific Functions
****************************************************************************/
int mip_i2c_dummy(struct i2c_client* client,  char *write_buf, unsigned int write_len)
{
	int retry = 3;
	int res = 0;

	while (retry--) {
		TOUCH_FUNC();
		res = Mit300_I2C_Write(client, write_buf, write_len);
		if(res < 0) {
			TOUCH_ERR("i2c_transfer - errno[%d]\n", res);
		} else {
			return TOUCH_SUCCESS;
		}
	}

	return TOUCH_FAIL;
}


int mip_lpwg_config(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_IDLE_REPORTRATE;
	wbuf[2] = 20;							// LPWG_IDLE_REPORTRATE
	wbuf[3] = 40;							// LPWG_ACTIVE_REPORTRATE
	wbuf[4] = 30;							// LPWG_SENSITIVITY
	wbuf[5] = (0 + ACTIVE_AREA_GAP) & 0xFF;						// LPWG_ACTIVE_AREA (horizontal start low byte)
	wbuf[6] = (0 + ACTIVE_AREA_GAP) >> 8 & 0xFF; 				// LPWG_ACTIVE_AREA (horizontal start high byte)
	wbuf[7] = (0 + ACTIVE_AREA_GAP) & 0xFF;						// LPWG_ACTIVE_AREA (vertical start low byte)
	wbuf[8] = (0 + ACTIVE_AREA_GAP) >> 8 & 0xFF;				// LPWG_ACTIVE_AREA (vertical start high byte)
	wbuf[9] = (720 - ACTIVE_AREA_GAP) & 0xFF;					// LPWG_ACTIVE_AREA (horizontal end low byte)
	wbuf[10] = (720 - ACTIVE_AREA_GAP) >> 8 & 0xFF;				// LPWG_ACTIVE_AREA (horizontal end high byte)
	wbuf[11] = (1280 - ACTIVE_AREA_GAP) & 0xFF; 				// LPWG_ACTIVE_AREA (vertical end low byte)
	wbuf[12] = (1280 - ACTIVE_AREA_GAP) >> 8 & 0xFF;			// LPWG_ACTIVE_AREA (vertical end high byte)
	wbuf[13] = LPWG_DEBUG_ENABLE;							// LPWG_FAIL_REASON

	if( Mit300_I2C_Write(client, wbuf, 14) )
	{
		TOUCH_ERR("mip_lpwg_config failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_on(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE;
	wbuf[2] = 1;							// LPWG_ENABLE
	wbuf[3] = 2;							// LPWG_TAP_COUNT
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP (high byte)
	wbuf[6] = 0 & 0xFF;						// LPWG_MIN_DISTANCE (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE (high byte)
	wbuf[8] = 10 & 0xFF;					// LPWG_MAX_DISTANCE (low byte)
	wbuf[9] = 10 >> 8 & 0xFF;				// LPWG_MAX_DISTANCE (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;				// LPWG_MAX_INTERTAP_TIME (high byte)
	wbuf[14] = (ts->lpwgSetting.isFirstTwoTapSame ? KNOCKON_DELAY : 0) & 0xFF;		// LPWG_INTERTAP_DELAY (low byte)
	wbuf[15] = ((ts->lpwgSetting.isFirstTwoTapSame ? KNOCKON_DELAY : 0) >> 8) & 0xFF;	// LPWG_INTERTAP_DELAY (high byte)

	if( Mit300_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock on Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_config_knock_code(struct i2c_client* client)
{
	u8 wbuf[32];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE2;
	wbuf[2] = 1;							// LPWG_ENABLE2
	wbuf[3] = ts->lpwgSetting.tapCount;		// LPWG_TAP_COUNT2
	wbuf[4] = 10 & 0xFF;					// LPWG_TOUCH_SLOP2 (low byte)
	wbuf[5] = 10 >> 8 & 0xFF;				// LPWG_TOUCH_SLOP2 (high byte)
	wbuf[6] = 0 & 0xFF;						// LPWG_MIN_DISTANCE2 (low byte)
	wbuf[7] = 0 >> 8 & 0xFF;				// LPWG_MIN_DISTANCE2 (high byte)
	wbuf[8] = 65535 & 0xFF;					// LPWG_MAX_DISTANCE2 (low byte)
	wbuf[9] = 65535 >>8 & 0xFF;				// LPWG_MAX_DISTANCE2 (high byte)
	wbuf[10] = 0 & 0xFF;					// LPWG_MIN_INTERTAP_TIME2 (low byte)
	wbuf[11] = 0 >> 8 & 0xFF;				// LPWG_MIN_INTERTAP_TIME2 (high byte)
	wbuf[12] = 700 & 0xFF;					// LPWG_MAX_INTERTAP_TIME2 (low byte)
	wbuf[13] = 700 >> 8 & 0xFF;				// LPWG_MAX_INTERTAP_TIME2 (high byte)
	wbuf[14] = 250 & 0xFF;					// LPWG_INTERTAP_DELAY2 (low byte)
	wbuf[15] = 250 >> 8 & 0xFF;				// LPWG_INTERTAP_DELAY2 (high byte)

	if( Mit300_I2C_Write(client, wbuf, 16) )
	{
		TOUCH_ERR("Knock code Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_debug_enable(struct i2c_client* client, int enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_LPWG_DEBUG_ENABLE;
	wbuf[2] = enable;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("LPWG debug Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_enable_sensing(struct i2c_client* client, bool enable)
{
	u8 wbuf[4];

	TOUCH_LOG("mip_lpwg_enable_sensing [%d]\n", enable);

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_ENABLE_SENSING;
	wbuf[2] = enable;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_enable_sensing failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

int mip_lpwg_start(struct i2c_client* client)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_LPWG;
	wbuf[1] = MIP_R1_LPWG_START;
	wbuf[2] = 1;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("mip_lpwg_start failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

static int lpwg_control(struct i2c_client *client, TouchState newState)
{
	TOUCH_FUNC();

	switch( newState )
	{
		case STATE_NORMAL:
			break;

		case STATE_KNOCK_ON_ONLY:
            if(cover_status){
               mip_lpwg_enable_sensing(client, 0);
               mip_lpwg_start(client);
               TOUCH_LOG("cover_status is closed, sensing disable\n");
            }else{
               mip_lpwg_config(client);
               mip_lpwg_config_knock_on(client);
               if( LPWG_DEBUG_ENABLE ) mip_lpwg_debug_enable(client, 1);
               if( ts->currState == STATE_OFF )mip_lpwg_enable_sensing(client, 1);
               mip_lpwg_start(client);
            }
			TouchEnableIrq();
			break;

		case STATE_KNOCK_ON_CODE:
            if(cover_status){
                mip_lpwg_enable_sensing(client, 0);
                mip_lpwg_start(client);
                TOUCH_LOG("cover_status is closed, sensing disable\n");
            }else{
			    mip_lpwg_config(client);
			    mip_lpwg_config_knock_on(client);
			    mip_lpwg_config_knock_code(client);
			    if( LPWG_DEBUG_ENABLE )mip_lpwg_debug_enable(client, 1);
			    if( ts->currState == STATE_OFF )mip_lpwg_enable_sensing(client, 1);
                mip_lpwg_start(client);
            }
			TouchEnableIrq();
		    break;

		case STATE_OFF:
			TouchDisableIrq();
			mip_lpwg_enable_sensing(client, 0);
			mip_lpwg_start(client);
			break;

		default:
			TOUCH_ERR("invalid touch state ( %d )\n", newState);
			break;
	}

	return TOUCH_SUCCESS;
}

int mip_palm_rejection_enable(struct i2c_client* client, bool enable)
{
	u8 wbuf[4];

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_PALM_REJECTION;
	wbuf[2] = enable;

	if( Mit300_I2C_Write(client, wbuf, 3) )
	{
		TOUCH_ERR("Palm Rejection Setting failed\n");
		return TOUCH_FAIL;
	}

	return 0;
}

static ssize_t show_rawdata(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int rawdataStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();

	// intensity check
	ret = MIT300_GetTestResult(client, buf, &rawdataStatus, RAW_DATA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get raw data\n");
		ret = sprintf(buf, "failed to get raw data\n");
	}

	return ret;
}

static ssize_t show_intensity(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int intensityStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();

	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for self diagnosis\n");
	}

	TOUCH_FUNC();

	// intensity check
	ret = MIT300_GetTestResult(client, buf, &intensityStatus, INTENSITY_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get intensity data\n");
		ret = sprintf(buf, "failed to get intensity data\n");
	}

	return ret;
}

static ssize_t show_jitter(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	int jitterStatus = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	if (pDriverData == NULL){
		TOUCH_ERR("failed to get pDriverData for jitter test\n");
		return ret;
	}

	TOUCH_FUNC();

	if (!(pDriverData->lpwgSetting.lcdState || LPWG_DEBUG_ENABLE)){
		ret = sprintf(buf,"Please turn on the LCD or enable debug mode.\n");
		return ret;
	}

	ret = MIT300_GetTestResult(client, buf, &jitterStatus, JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get jitter data\n");
		ret = sprintf(buf, "failed to get jitter data\n");
	}

	return ret;
}

static ssize_t store_reg_control(TouchDriverData *pDriverData, const char *buf, size_t count)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    int reg_addr[2] = {0};
    int cmd = 0;
    int value = 0;
    uint8_t write_buf[50] = {0};
    uint8_t read_buf[50] = {0};
    int i = 0;
    int len = 2;
    if ( sscanf(buf, "%d %x %x %d", &cmd, &reg_addr[0], &reg_addr[1], &value) != 4) {
        TOUCH_LOG("data parsing fail.\n");
        return -EINVAL;
    }
    TOUCH_LOG("%d, 0x%x, 0x%x, %d\n", cmd, reg_addr[0], reg_addr[1], value);
    
    switch (cmd) {
        case 1:
            write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
            if( Mit300_I2C_Read(client, write_buf, len, read_buf, value) )
            {
                TOUCH_LOG("store_reg_control failed\n");
            }
            
            for (i = 0; i < value; i ++) {
                TOUCH_LOG("read_buf=[%d]\n",read_buf[i]);
            }
            break;
        case 2:
            write_buf[0] = reg_addr[0];
			write_buf[1] = reg_addr[1];
            if (value >= 256) {
				write_buf[2] = (value >> 8);
				write_buf[3] = (value & 0xFF);
				len = len + 2;
			} else {
				write_buf[2] = value;
				len++;
			}
            if( Mit300_I2C_Write(client, write_buf, len) )
            {
                 TOUCH_ERR("store_reg_control failed\n");
            }
            break;
        default:
            TOUCH_LOG("usage: echo [1(read)|2(write)], [reg address0], [reg address1], [length(read)|value(write)] > reg_control\n");
            break;
    }
    return count;
}

static ssize_t show_lcd_status(TouchDriverData *pDriverData, char *buf)
{
	int ret = 0;
	WRITE_SYSBUF(buf, ret, "LCD Status : %s", (ts->currState == STATE_NORMAL) ? "On\n" : "Off\n");
	return ret;
}

static LGE_TOUCH_ATTR(intensity, S_IRUGO | S_IWUSR, show_intensity, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, NULL);
static LGE_TOUCH_ATTR(jitter, S_IRUGO | S_IWUSR, show_jitter, NULL);
static LGE_TOUCH_ATTR(reg_control,  S_IRUGO | S_IWUSR, NULL, store_reg_control);
static LGE_TOUCH_ATTR(lcd_status, S_IRUGO | S_IWUSR, show_lcd_status, NULL);


static struct attribute *MIT300_attribute_list[] = {
	&lge_touch_attr_intensity.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_jitter.attr,
    &lge_touch_attr_reg_control.attr,
    &lge_touch_attr_lcd_status.attr,
	NULL,
};


static int MIT300_Initialize(TouchDriverData *pDriverData)
{
	struct i2c_client *client = Touch_Get_I2C_Handle();
    TOUCH_FUNC();

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;
  
	return TOUCH_SUCCESS;
}

static void MIT300_Reset_Dummy(void)
{

}

static int MIT300_InitRegister(void)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    u8 wbuf[4];
	if(PALM_REJECTION_ENABLE)
		mip_palm_rejection_enable(client,1);

	/* IMPLEMENT : Register initialization after reset */
    if(lge_get_boot_mode() == LGE_BOOT_MODE_QEM_130K) {
        TOUCH_ERR("LGE_BOOT_MODE_QEM_130K\n");
        mip_lpwg_start(client);
        wbuf[0] = 0x06;
		wbuf[1] = 0x18;
		wbuf[2] = 1;
		if( Mit300_I2C_Write(client, wbuf, 3) )
		{
			TOUCH_ERR("mip_lpwg_start failed\n");
			return TOUCH_FAIL;
		}
	}
     if(cover_status){
		if(ts != NULL && ts->currState != STATE_BOOT && ts->currState == STATE_NORMAL) queue_delayed_work(touch_wq, &work_cover, msecs_to_jiffies(10));
     }
	return TOUCH_SUCCESS;
}

void MIT300_Reset(int status, int delay)
{
#if defined (TOUCH_PLATFORM_QCT)
	if(lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		TOUCH_LOG("CHARGERLOGO_MODE, Skip T_RESET pin control (Always Down)\n");
		return;
	}
#endif
    if(!status)TouchDisableIrq();
	TouchSetGpioReset(status);
	 if( delay <= 0 || delay > 1000 )
        {
            TOUCH_LOG("%s exeeds limit %d\n", __func__, delay);
            return;
        }

	if(delay<=20)
		mdelay(delay);
    else
		msleep(delay);
    if(status){
        MIT300_InitRegister();
        TouchEnableIrq();
    }
}

static int MIT300_InterruptHandler(TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	u8 i = 0;
	u8 wbuf[8] = {0};
	u8 rbuf[256] = {0};
	u32 packet_size = 0;
	u8 packet_type = 0;
	u8 alert_type = 0;
	u8 index = 0;
	u8 state = 0;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	pData->type = DATA_UNKNOWN;
	pData->count = 0;

	if (LPWG_DEBUG_ENABLE == 0 && (ts->currState == STATE_KNOCK_ON_ONLY || ts->currState == STATE_KNOCK_ON_CODE)) {
		if(mip_i2c_dummy(client, wbuf, 2) == TOUCH_FAIL){
			TOUCH_ERR("Fail to send dummy packet\n");
			return TOUCH_FAIL;
		}
	}

	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, 1) )
	{
		TOUCH_ERR("Read packet info\n");
		return TOUCH_FAIL;
	}

	packet_size = (rbuf[0] & 0x7F);
	packet_type = ((rbuf[0] >> 7) & 0x1);

	if( packet_size == 0 )
		return TOUCH_SUCCESS;

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if( Mit300_I2C_Read(client, wbuf, 2, rbuf, packet_size) )
	{
		TOUCH_ERR("Read packet data\n");
		return TOUCH_FAIL;
	}

	//Event handler
	if( packet_type == 0 )	/* Touch event */
	{
		for( i = 0 ; i < packet_size ; i += 6 )
		{
			u8 *tmp = &rbuf[i];

			if( (tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0 )
			{
				TOUCH_LOG("use sofrware key\n");
				continue;
			}

			index = (tmp[0] & 0xf) - 1;
			state = (tmp[0] & 0x80) ? 1 : 0;

			if( (index < 0) || (index > MAX_NUM_OF_FINGERS - 1) )
			{
				TOUCH_ERR("invalid touch index (%d)\n", index);
				return TOUCH_FAIL;
			}

			pData->type = DATA_FINGER;

			if( state )
			{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->x = tmp[2] | ((tmp[1] & 0x0f) << 8);
				pFingerData->y = tmp[3] | ((tmp[1] & 0xf0) << 4);
				pFingerData->width_major = tmp[5];
				pFingerData->width_minor = 0;
				pFingerData->orientation = 0;
				pFingerData->pressure = tmp[4];
				if( tmp[4] < 1 )
					pFingerData->pressure = 1;
				else if( tmp[4] > 255 - 1 )
					pFingerData->pressure = 255 - 1;
				if(PALM_REJECTION_ENABLE &&(tmp[0] & MIP_EVENT_INPUT_PALM) >> 4 )
					pFingerData->pressure = 255;
				pData->count++;
				pFingerData->status = FINGER_PRESSED;
                if(cradle_smart_cover_status()){
                    if(pFingerData->x<614){
                        pFingerData->status = FINGER_UNUSED;
                    }
                }              
			}else{
				pFingerData = &pData->fingerData[index];
				pFingerData->id = index ;
				pFingerData->status = FINGER_RELEASED;
			}
		}
	}
	else	/* Alert event */
	{
		alert_type = rbuf[0];

		if( alert_type == MIP_ALERT_ESD )	//ESD detection
		{
			TOUCH_LOG("ESD Detected!\n");
			TOUCH_LOG("ESD Frame Count = %d\n",rbuf[1]);
#if defined(CONFIG_LGE_LCD_ESD)
			return TOUCH_ESD;
#endif
		}
		else if( alert_type == MIP_ALERT_WAKEUP )	//Wake-up gesture
		{
			if( rbuf[1] == MIP_EVENT_GESTURE_DOUBLE_TAP )
			{
				TOUCH_LOG("Knock-on Detected\n");
				pData->type = DATA_KNOCK_ON;
			}
			else if( rbuf[1] == MIP_EVENT_GESTURE_MULTI_TAP )
			{
				TOUCH_LOG("Knock-code Detected\n");
				pData->type = DATA_KNOCK_CODE;

				for( i = 2 ; i < packet_size ; i += 3 )
				{
					u8 *tmp = &rbuf[i];
					pData->knockData[((i + 1) / 3) - 1].x = tmp[1] | ((tmp[0] & 0xf) << 8);
					pData->knockData[((i + 1) / 3) - 1].y = tmp[2] | (((tmp[0] >> 4) & 0xf) << 8);
					pData->count++;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit300_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else if( alert_type == MIP_ALERT_F1 )	//Gesture Fail Reason
		{
			if( rbuf[1] == MIP_LPWG_EVENT_TYPE_FAIL )
			{
				switch( rbuf[2] )
				{
					case OUT_OF_AREA:
						TOUCH_LOG("LPWG FAIL REASON = Out of Area\n");
						break;
					case PALM_DETECTED:
						TOUCH_LOG("LPWG FAIL REASON = Palm\n");
						break;
					case DELAY_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Delay Time\n");
						break;
					case TAP_TIME:
						TOUCH_LOG("LPWG FAIL REASON = Tap Time\n");
						break;
					case TAP_DISTACE:
						TOUCH_LOG("LPWG FAIL REASON = Tap Distance\n");
						break;
					case TOUCH_SLOPE:
						TOUCH_LOG("LPWG FAIL REASON = Touch Slope\n");
						break;
					case MULTI_TOUCH:
						TOUCH_LOG("LPWG FAIL REASON = Multi Touch\n");
						break;
					case LONG_PRESS:
						TOUCH_LOG("LPWG FAIL REASON = Long Press\n");
						break;
					default:
						TOUCH_LOG("LPWG FAIL REASON = Unknown Reason\n");
						break;
				}
			}
			else
			{
				//Re-enter tap mode
				wbuf[0] = MIP_R0_CTRL;
				wbuf[1] = MIP_R1_CTRL_POWER_STATE;
				wbuf[2] = MIP_CTRL_POWER_LOW;
				if( Mit300_I2C_Write(client, wbuf, 3) )
				{
					TOUCH_ERR("mip_i2c_write failed\n");
					return TOUCH_FAIL;
				}
			}
		}
		else
		{
			TOUCH_LOG("Unknown alert type [%d]\n", alert_type);
		}
	}
	return TOUCH_SUCCESS;

}

static int MIT300_ReadIcFirmwareInfo( TouchFirmwareInfo *pFwInfo)
{
	u8 wbuf[2] = {0, };
	u8 version[2] = {0, };
	int ret = 0;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_CUSTOM;

	ret = Mit300_I2C_Read(client, wbuf, 2, version, 2);
	if( ret == TOUCH_FAIL )
		return TOUCH_FAIL;

	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = version[1];
	pFwInfo->version = version[0];

	TOUCH_LOG("IC F/W Version = v%X.%02X ( %s )\n", version[1], version[0], pFwInfo->isOfficial ? "Official Release" : "Test Release");

	return TOUCH_SUCCESS;
}

static int MIT300_GetBinFirmwareInfo( char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 version[2] = {0, };
	u8 *pFwFilename = NULL;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	mip_bin_fw_version(ts, fw->data, fw->size, version);

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

static int MIT300_UpdateFirmware( char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;
	const struct firmware *fw = NULL;
    struct i2c_client *client = Touch_Get_I2C_Handle();

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = mip_flash_fw(ts, fw->data, fw->size, false, true);
	if( ret < fw_err_none ) {
		return TOUCH_FAIL;
	}

	release_firmware(fw);

	return TOUCH_SUCCESS;
}

static int MIT300_SetLpwgMode( TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;
    struct i2c_client *client = Touch_Get_I2C_Handle();
    
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

static int MIT300_DoSelfDiagnosis(int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	/* CAUTION : be careful not to exceed buffer size */
	char *sd_path = "/mnt/sdcard/touch_self_test.txt";
	int ret,i = 0;
	int dataLen = 0;
    int jitterMax[2] = {0,};
    int jitterAverageMax[2] = {0,};
    int jitterMaxCount = 0;
    int jitterAverageCount = 0;
	struct i2c_client *client = Touch_Get_I2C_Handle();
	memset(pBuf, 0, bufSize);
	*pDataLen = 0;
	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */

	*pRawStatus = TOUCH_SUCCESS;
	*pChannelStatus = TOUCH_SUCCESS;

	MIT300_WriteFile(sd_path, pBuf, 1);
	msleep(30);

	// raw data check
	ret = MIT300_GetTestResult(client, pBuf, pRawStatus, RAW_DATA_SHOW);
	if( ret < 0 ) {
		TOUCH_ERR("failed to get raw data\n");
		memset(pBuf, 0, bufSize);
		*pRawStatus = TOUCH_FAIL;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// cm_delta check
	ret = MIT300_GetTestResult(client, pBuf, pChannelStatus, DELTA_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get delta data\n");
		memset(pBuf, 0, bufSize);
		*pChannelStatus = TOUCH_FAIL;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// cm_jitter check
	ret = MIT300_GetTestResult(client, pBuf, pChannelStatus, JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get jitter data\n");
		memset(pBuf, 0, bufSize);
		*pChannelStatus = TOUCH_FAIL;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

    // blu jitter check
    for(i = 0; i < 2; i++){
        queue_delayed_work(touch_wq, &work_backlight_off, msecs_to_jiffies(50));
        queue_delayed_work(touch_wq, &work_backlight_on, msecs_to_jiffies(250));

	    // cm_jitter check
	    ret = MIT300_GetTestResult(client, pBuf, pChannelStatus, JITTER_SHOW);
	    if (ret < 0) {
			TOUCH_ERR("failed to get jitter data\n");
			memset(pBuf, 0, bufSize);
			*pChannelStatus = TOUCH_FAIL;
	    }
        jitterMax[i] = max_data;
        jitterAverageMax[i] = jitter_average_max;
        if(jitterMax[i]>BLU_JITTER_MAX)jitterMaxCount++;
        if(jitterAverageMax[i]>BLU_AVERAGE_JITTER)jitterAverageCount++;
	    MIT300_WriteFile(sd_path, pBuf, 0);
	    msleep(30);
	    memset(pBuf, 0, bufSize);
    }
    dataLen += sprintf(pBuf,"Jitter Max: %d,%d\nJitter Average Max: %d,%d\n"
    ,jitterMax[0],jitterMax[1]
    ,jitterAverageMax[0],jitterAverageMax[1]);

#if defined(CONFIG_MACH_MSM8916_M216N_KR) || defined(CONFIG_MACH_MSM8916_M216_GLOBAL_COM)
	rt8542_backlight_on(177);
#endif
#if defined(CONFIG_MACH_MSM8916_M209_TRF_US) || defined(CONFIG_MACH_MSM8916_M209_TRF_US_VZW)
	sm5306_backlight_on(177);
#endif

    if(jitterMaxCount>0||jitterAverageCount>0)*pChannelStatus = TOUCH_FAIL;
    else *pChannelStatus = TOUCH_SUCCESS;

	// open short check
    ret = MIT300_GetTestResult(client, pBuf, pChannelStatus, OPENSHORT_SHOW);
    if (ret < 0) {
       TOUCH_ERR("failed to get open short data\n");
       memset(pBuf, 0, bufSize);
       *pChannelStatus = TOUCH_FAIL;
    }
    MIT300_WriteFile(sd_path, pBuf, 0);
    msleep(30);
    memset(pBuf, 0, bufSize);
    
    // open short 2 check (MUX)
    ret = MIT300_GetTestResult(client, pBuf, pChannelStatus, MUXSHORT_SHOW);
    if (ret < 0) {
        TOUCH_ERR("failed to get open short (mux) data\n");
        memset(pBuf, 0, bufSize);
        *pChannelStatus = TOUCH_FAIL;
    }
    MIT300_WriteFile(sd_path, pBuf, 0);
    msleep(30);
    memset(pBuf, 0, bufSize);

//	dataLen = sprintf(pBuf, "%s", "======ADDITIONAL======\n");
//	dataLen += sprintf(pBuf+dataLen, "Delta Test: %s", (deltaStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
//	dataLen += sprintf(pBuf+dataLen, "Jitter Test: %s", (jitterStatus==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
}
static int MIT300_DoSelfDiagnosis_Lpwg(int* lpwgStatus, char* pBuf, int bufSize, int* pDataLen)
{
    char *sd_path = "/mnt/sdcard/touch_self_test.txt";
	int ret = 0;
	int dataLen = 0;
    struct i2c_client *client = Touch_Get_I2C_Handle();
    int deltaStatus = 0;
	int jitterStatus = 0;
    memset(pBuf, 0, bufSize);
    *pDataLen = 0;
    TOUCH_FUNC();
        
    mip_lpwg_enable_sensing(client,1);
    msleep(1000);
    
    mip_lpwg_debug_enable(client, 1);
    msleep(10);
    
    mip_lpwg_start(client);
    msleep(10);

    *lpwgStatus = TOUCH_SUCCESS;
    
    MIT300_WriteFile(sd_path, pBuf, 1);
    msleep(30);
    
   	// cm_delta check
	ret = MIT300_GetTestResult(client, pBuf, &deltaStatus, LPWG_JITTER_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get delta data\n");
		memset(pBuf, 0, bufSize);
		deltaStatus = TOUCH_FAIL;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

	// cm_jitter check
	ret = MIT300_GetTestResult(client, pBuf, &jitterStatus, LPWG_ABS_SHOW);
	if (ret < 0) {
		TOUCH_ERR("failed to get jitter data\n");
		memset(pBuf, 0, bufSize);
		jitterStatus = TOUCH_FAIL;
	}
	MIT300_WriteFile(sd_path, pBuf, 0);
	msleep(30);
	memset(pBuf, 0, bufSize);

    dataLen += sprintf(pBuf, "LPWG Test: %s", ((deltaStatus+jitterStatus)==TOUCH_SUCCESS) ? "Pass\n" : "Fail\n");
    *lpwgStatus = (deltaStatus+jitterStatus);

    *pDataLen = dataLen;
    
    mip_lpwg_debug_enable(client, 0);
    msleep(10);
    mip_lpwg_start(client);
    return TOUCH_SUCCESS;
    /*
error :
    mip_lpwg_debug_enable(client, 0);
    msleep(10);
    mip_lpwg_start(client);
	return TOUCH_FAIL;*/
}

static void MIT300_PowerOn(int isOn)
{
    
}

static void MIT300_ClearInterrupt(void)
{

}

static void MIT300_NotifyHandler(TouchNotify notify, int data)
{
	if(notify == NOTIFY_Q_COVER){
		cover_status = data;
		if(ts != NULL && ts->currState != STATE_BOOT) queue_delayed_work(touch_wq, &work_cover, msecs_to_jiffies(1));
	}
}
static int MIT300_MftsControl(TouchDriverData *pDriverData)
{
    struct i2c_client *client = Touch_Get_I2C_Handle();
    TOUCH_FUNC();

	if(lge_get_mfts_mode() && !pDriverData->mfts_lpwg){
		mip_lpwg_enable_sensing(client, 0);
		mip_lpwg_start(client);
	}
	return TOUCH_SUCCESS;
}

TouchDeviceControlFunction MIT300_Func = {
    .Power                  = MIT300_PowerOn,
	.Initialize 			= MIT300_Initialize,
	.Reset 					= MIT300_Reset_Dummy,
	.InitRegister 			= MIT300_InitRegister,
	.ClearInterrupt         = MIT300_ClearInterrupt,
	.InterruptHandler 		= MIT300_InterruptHandler,
	.ReadIcFirmwareInfo 	= MIT300_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo 	= MIT300_GetBinFirmwareInfo,
	.UpdateFirmware 		= MIT300_UpdateFirmware,
	.SetLpwgMode 			= MIT300_SetLpwgMode,
	.DoSelfDiagnosis 		= MIT300_DoSelfDiagnosis,
	.DoSelfDiagnosis_Lpwg	= MIT300_DoSelfDiagnosis_Lpwg,
	.device_attribute_list 	= MIT300_attribute_list,
	.NotifyHandler          = MIT300_NotifyHandler,
	.MftsControl            = MIT300_MftsControl,
};


/* End Of File */


