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
 *    File  	: lgtp_device_ft6x36.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[FT6X36]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_ft6x36.h>

#include "focaltech_ctl.h"
#include "ft6x06_ex_fun.h"

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/* focaltech vendor app want to use sysfs*/
#define TPD_I2C_ADDRESS 0x38
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define FTS_CTL_IIC


#define EVENT_NONE		    0x00
#define EVENT_ABS		    0x01
#define EVENT_KEY	    	0x02
/*TO DO - NSM 
EVENT_LPWG Saperate EVENT_KNOCK / EVENT_KNOCK_ONCODE*/
#define EVENT_KNOCK_ON          0x03
#define EVENT_KNOCK_CODE        0x04
#define EVENT_KNOCK_OVER        0x05
#define EVENT_HOVERING_NEAR     0x06
#define EVENT_HOVERING_FAR      0x07
#define EVENT_GEST		        0x04
#define EVENT_MOUSE		        0x08


/*knock on &Touble Tap Cotrol Register*/
#define FT_INT_STATUS           0x9B
#define FT_LPWG_INT_DELAY       0xDF
#define FT_LPWG_CONTROL_REG     0xD0

#define FT_MULTITAP_COUNT_REG	0xE4 /* for Knock code */

#define FT_KNOCK_READ_DATA_REG 0xD3
#define KNOCK_TAP_COUNT         0xD4
#define FT_TOUCHKEY_STATUS_REG 0x9A

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MAJ_VER	0xB1
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

    /* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4

enum {
    FT_BLOADER_VERSION_LZ4 = 0,
    FT_BLOADER_VERSION_Z7 = 1,
    FT_BLOADER_VERSION_GZF = 2,
};

enum {
    FT_FT5336_FAMILY_ID_0x11 = 0x11,
    FT_FT5336_FAMILY_ID_0x12 = 0x12,
    FT_FT5336_FAMILY_ID_0x13 = 0x13,
    FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

/* position of button*/
#define X_POS_OF_BACKKEY    60
#define X_POS_OF_HOMEKEY    180
#define X_POS_OF_SIMSWITCHKEY   300
#define X_POS_OF_MENUKEY    420
#define Y_POS_OF_TOUCHKEY   900

/****************************************************************************
 * Macros
 ****************************************************************************/
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

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct Ft6x36DriverDataTag {
	struct i2c_client	*client;
	TouchState currState;
	LpwgSetting lpwgSetting;

} Ft6x36DriverData;

/****************************************************************************
* Variables
****************************************************************************/


static const char defaultFirmware[] = "focaltech/Y30_FT6336_27.img";


static Ft6x36DriverData *pDeviceData = NULL;
int enable_iic_dev = 0;

/****************************************************************************
* Extern Variable Prototypes
****************************************************************************/


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int Ft6x36_Set_PDN(struct i2c_client *client);
static int Ft6x36_Set_KnockOn(struct i2c_client *client);
static int Ft6x36_Set_KnockCode(struct i2c_client *client, LpwgSetting  *pLpwgSetting);
static int Ft6x36_Set_Hover(struct i2c_client *client, u8 on);


/****************************************************************************
* Local Functions
****************************************************************************/

/*  to get   ic infor from firwmare*/
int get_ic_info(struct i2c_client * client)
{
	u8 reg_value = 0;

	Touch_I2C_Read(client, FT6x06_REG_FW_VER, &reg_value, 1);
	TOUCH_LOG( "Firmware version = 0x%x in Touch IC\n", reg_value);

	Touch_I2C_Read(client, FT6x06_REG_POINT_RATE, &reg_value, 1);
	TOUCH_LOG(" report rate is %dHz.\n", reg_value * 10);

	Touch_I2C_Read(client, FT6x06_REG_THGROUP, &reg_value, 1);
	TOUCH_LOG(" touch threshold is %d.\n", reg_value * 4);

	return 0;
}

int FirmwareUpgrade(struct i2c_client * client, const struct firmware *fw_img)
{
	u8 * pbt_buf = NULL;
	int i_ret;
	int fw_len = fw_img->size;

	/*FW upgrade*/
	pbt_buf = (u8*)fw_img->data;

    /*call the upgrade function*/
    i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, fw_len);
      if (i_ret != 0)
    {
       TOUCH_ERR("Firwmare upgrade failed. err=%d.\n", i_ret);
    }
    else
    {
#ifdef AUTO_CLB
      fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
    }

	return i_ret;
}

/****************************************************************************
* Global Functions
****************************************************************************/

static ssize_t show_use_iic_dev(struct i2c_client *client, char *buf)
{
    int ret = 0;

    ret = sprintf(buf, "%u\n", enable_iic_dev);

    return ret;
}

static ssize_t store_use_iic_dev(struct i2c_client *client, const char *buf, size_t count)
{
    int value = 0;

    sscanf(buf, "%d", &value);

    if (value < 0 || value > 1) {
        TOUCH_DBG("Invalid enable_rmi_dev value:%d\n", value);
        return count;
    }

    TOUCH_DBG("enable_iic_dev:%u value: %d \n", enable_iic_dev ,value);

    if (enable_iic_dev==0 && value==1) {

#ifdef SYSFS_DEBUG
        ft6x06_create_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
        if (ft_rw_iic_drv_init(client) < 0)
			TOUCH_ERR("[FTS] create fts control iic driver failed\n");
#endif
#ifdef FTS_APK_DEBUG
        ft6x06_create_apk_debug_channel(client);
#endif
        enable_iic_dev=value;

    }
    else if(enable_iic_dev==1 && value==0){

#if 0 /*to do disable debug func, please reboot the device*/
#ifdef SYSFS_DEBUG
        ft6x06_release_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
        ft_rw_iic_drv_exit();
#endif
#ifdef FTS_APK_DEBUG
        ft6x06_release_apk_debug_channel();
#endif

        enable_iic_dev=value;
#endif
    }
    return count;

}



static ssize_t show_Model_Info(struct i2c_client *client, char *buf)
{
    int ret = 0;	
    
    ret = sprintf(buf, "======== Model info ========\n");
    if( TouchReadMakerId() == 0 )
    {
        TOUCH_LOG("Touch IC : LeadingUI\n");
        ret += sprintf(buf+ret, "Maker ID PIN: 0\n");
        ret += sprintf(buf+ret, "Module Product : SUNTEL\n");
        ret += sprintf(buf+ret, "Touch IC : LeadingUI\n");
        //return TOUCH_SUCCESS;
    }
    else if(TouchReadMakerId() == 1 )
    {
        ret += sprintf(buf+ret, "Maker ID PIN: 1\n");
        ret += sprintf(buf+ret, "Module Product : LGIT\n");
        ret += sprintf(buf+ret, "Touch IC : Focaltech\n");
    }
    else
	{
	TOUCH_LOG("Test code\n");
	}	
	return ret;
}
static ssize_t store_Set_vp(struct i2c_client *client, const char *buf, size_t count)
{
    int cmd = 0;
    TOUCH_LOG("[store_Set_vp]\n");
    sscanf(buf, "%d", &cmd);
    
    if(cmd == 1)
    {
        Ft6x36_Set_Hover(client, 1);
        TOUCH_LOG("Set hover mode\n");
    }
    else
    {
        Ft6x36_Set_Hover(client, 0);        
    }
    
	return count;
}
static LGE_TOUCH_ATTR(Set_vp, S_IRUGO | S_IWUSR, NULL, store_Set_vp);
static LGE_TOUCH_ATTR(enable_iic_dev, S_IRUGO | S_IWUSR, show_use_iic_dev, store_use_iic_dev);
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);

static struct attribute *Ft6x36_attribute_list[] = {
	&lge_touch_attr_enable_iic_dev.attr,
    &lge_touch_attr_Model_Info.attr,
    &lge_touch_attr_Set_vp.attr,
    NULL,
};

int Ft6x36_Initialize(struct i2c_client *client)
{

	TOUCH_FUNC();

	pDeviceData = devm_kzalloc(&client->dev, sizeof(Ft6x36DriverData), GFP_KERNEL);
	if( pDeviceData == NULL )
	{
		TOUCH_ERR("Fail to allocate device specific driver data\n");
		goto earlyReturn;
	}

	pDeviceData->client = client;

	return TOUCH_SUCCESS;

earlyReturn:

	return TOUCH_FAIL;

}

void Ft6x36_Reset(struct i2c_client *client)
{
	TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(200);
}

int Ft6x36_Connect(void)
{
    /*To Do - NSM */
    /*change to i2c connection */
#if 1
    int ret = 0;
    u8 reg = 0;
    u8 data = 0;
    TOUCH_FUNC();

    reg = FT6x06_REG_FW_VER;
    
    ret = touch_i2c_read_for_query( TPD_I2C_ADDRESS, &reg, 1, &data, 1);
    if( ret == TOUCH_SUCCESS )
    {
        TOUCH_LOG("FOCALTECH was detected\n");
        return TOUCH_SUCCESS;
    } 
    else 
    {
        TOUCH_LOG("FOACALTECH was NOT detected\n");
        return TOUCH_FAIL;
    }

#else
	if( TouchReadMakerId() == 1 )
	{
		TOUCH_LOG("Ft6x36 was detected\n");
		return TOUCH_SUCCESS;
	}
	else 
	{
		TOUCH_LOG("Ft6x36 was NOT detected\n");
		return TOUCH_FAIL;
	}
#endif


   
}

static int Ft6x36_InitRegister(struct i2c_client *client)
{
	TOUCH_FUNC();

	/* IMPLEMENT : Register initialization after reset */
	pDeviceData->currState = STATE_NORMAL;
    return TOUCH_SUCCESS;
}

static int get_lpwg_data(struct i2c_client *client, TouchReadData *pData, LpwgSetting  *pLpwgSetting)
{
    u8 i = 0;
    u8 buffer[50] = {0,};

    if(Touch_I2C_Read(client, FT_KNOCK_READ_DATA_REG, buffer, pLpwgSetting->tapCount*4 + 2) == 0)
    {
        pData->count = buffer[1];
        TOUCH_LOG("[NSM]TAP COUNT = %d\n", buffer[1]);
    }
    else
    {
        TOUCH_ERR("KNOCK TAP DATA Read Fail.\n");
        goto error;
    }

    if(!buffer[1])
    {
        TOUCH_LOG("TAP COUNT = %d\n", buffer[1]);
        goto error;
    }
    
    for(i = 0; i< buffer[1]; i++)
    {
        pData->knockData[i].x = GET_X_POSITION(buffer[4*i+2], buffer[4*i+3]);
        pData->knockData[i].y = GET_Y_POSITION(buffer[4*i+4], buffer[4*i+5]);
        TOUCH_LOG("LPWG data [%d, %d]\n", pData->knockData[i].x, pData->knockData[i].y);
    }
    
    return TOUCH_SUCCESS;
error:
    return TOUCH_FAIL;
}

static void Ft6x36_ClearInterrupt(struct i2c_client *client)
{
	
	return;
}

int Ft6x36_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{

    TouchFingerData *pFingerData = NULL;

	int i = 0;
	int ret = -1;
    u8 touch_event=0;
    u8 buf[POINT_READ_BUF] = { 0 };
	u8 pointid = FT_MAX_ID;
	u8 touch_count = 0;
	u8 touchkey_pressed=0;
    u8 event_type = 0;
	static u8 pressure_change = 0;
	

	pData->type = DATA_UNKNOWN;
	pData->count = 0;
    /* read Interrupt status */
    ret = Touch_I2C_Read(client, FT_INT_STATUS, &event_type, sizeof(u8));
    if(ret <0)
    {
        TOUCH_ERR("read int status failed.\n");
        return TOUCH_FAIL;
    }
    switch(event_type)
    {
        case EVENT_ABS :
        case EVENT_KEY :
       
	ret = Touch_I2C_Read(client, 0x00, buf, POINT_READ_BUF);
	if (ret < 0) {
		TOUCH_ERR("read touchdata failed.\n");
		return TOUCH_FAIL;
	}

	//TOUCH_LOG(" GEST_ID[%x] TD_STATUS[%x] \n", buf[1], buf[2]&0xf);
	touch_count = buf[2]&0xf;
	pressure_change ^= 1;//temp

	for (i = 0; i < touch_count; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		pFingerData = &pData->fingerData[pData->count];
		touch_event = buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;

		pFingerData->x = (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
						8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		pFingerData->y = (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
						8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];

		//TOUCH_DBG(" ID <%d> pos(%4d,%4d) Status[%d]\n",
		//pFingerData->id,pFingerData->x,
		//pFingerData->y,touch_event);

		if(pFingerData->y==Y_POS_OF_TOUCHKEY){/* touch(button) key*/
		/* if the touch position is a touchkey area, it will be skiped. */
		/*The touch key will be checked using FT_TOUCHKEY_STATUS_REG*/
			continue;
		}

        if(touch_event== 0 ||touch_event==2){
			/* for press*/
            if(pData->type == DATA_KEY){
                if(touch_event==0)
                    pData->keyData.pressed=KEY_PRESSED;
            }
            else{
				pFingerData->id  = pointid;
                pFingerData->width_major = 15;
                pFingerData->width_minor = 10;
                pFingerData->orientation = 1;

                /* to prevent Hovering detected in CFW.*/
                pFingerData->pressure =100 + pressure_change;
				pData->count++;
            }
        }
        else if(touch_event == 1){
           /* for Release */
           if(pData->type == DATA_KEY){
                pData->keyData.pressed=KEY_RELEASED;
           }
           else{
               /*nothing to do*/
           }

		}

	}

        if(pData->count >0)
            pData->type = DATA_FINGER;

        if(pData->type !=DATA_FINGER){
           touch_event=0;
           Touch_I2C_Read(client, FT_TOUCHKEY_STATUS_REG, &touch_event, 1);

          // TOUCH_LOG("Keyintsts:[%d] key [%d] event [%d] \n",
          //  touch_event&0x1,(touch_event>>1&7),(touch_event>>4&3) );

           if(touch_event&0x1){
               /*touch key event pressent*/
               touchkey_pressed=touch_event>>4&3;
                    if(touchkey_pressed == 0){
                        pData->keyData.pressed = KEY_PRESSED;
                        pData->keyData.index = touch_event>>1&7;
                        pData->type = DATA_KEY;
                    }
                    else if(touchkey_pressed == 1){
                        pData->keyData.pressed = KEY_RELEASED;
                        pData->keyData.index = touch_event>>1&7;
                        pData->type = DATA_KEY;

                    }
                    else{
                        /*nothing to do*/
                    }

           }
        }

        if(pData->type != DATA_KEY)
            pData->type = DATA_FINGER;
        break;

        case EVENT_KNOCK_ON :
            pData->type = DATA_KNOCK_ON;
            TOUCH_LOG("[KNOCK ON] Event Type = %d\n", event_type);
            //Ft6x36_Set_KnockOn(client);
            break;

        case EVENT_KNOCK_CODE :
            pData->type = DATA_KNOCK_CODE;
            get_lpwg_data(client, pData, &pDeviceData->lpwgSetting);
            TOUCH_LOG("[KNOCK CODE] Event Type = %d\n", event_type);;
            Ft6x36_Set_KnockCode(client, &pDeviceData->lpwgSetting);
            break;

        case EVENT_KNOCK_OVER :
            pData->type = DATA_KNOCK_CODE;
            pData->knockData[0].x = 1;
            pData->knockData[0].y = 1;
            pData->knockData[1].x = -1;
            pData->knockData[1].y = -1;
            TOUCH_LOG("[KNOCK CODE OVER] Event Type = %d\n", event_type);
            break;
        
        case EVENT_HOVERING_NEAR :
            /*
            pData->type = DATA_HOVER_NEAR; 
            pData->hoverState = 0;
            TOUCH_LOG("[HOVERING NEAR] Event Type = %d\n", event_type);
            */
            break;
        
        case EVENT_HOVERING_FAR :
            /*
            pData->type = DATA_HOVER_FAR;
            pData->hoverState = 1;
            TOUCH_LOG("[HOVERING FAR] Event Type = %d\n", event_type);
            */
            break;
       default:
            TOUCH_LOG("[Unknown] Event Type = %d\n",event_type);
            break;
          
            
    }
    return TOUCH_SUCCESS;
/*
    fail:
    return TOUCH_FAIL;
*/
}

static int Ft6x36_ReadIcFirmwareInfo(struct i2c_client *client,TouchFirmwareInfo *pFwInfo)
{
    int result = TOUCH_SUCCESS;
	u8 readData = 0;
     
	TOUCH_FUNC();

	/* IMPLEMENT : read IC firmware information function */
    pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0;
    
	if(Touch_I2C_Read(client, FT6x06_REG_FW_VER, &readData, 1) != 0)
	{
        TOUCH_ERR("Firmware version read fail (0xA6)\n");
        result = TOUCH_FAIL;
    }
    pFwInfo->version = readData;
    //pFwInfo->isofficial = reg_value >> 7;
    pFwInfo->isOfficial = 1;

    TOUCH_LOG("IC Firmware Official = %d\n", pFwInfo->isOfficial);
	TOUCH_LOG("IC Firmware Version = 0x%02X\n", readData);      
/*
	Touch_I2C_Read(client, FT6x06_REG_POINT_RATE, &reg_value, 1);
	TOUCH_LOG("Report rate = %dHz.\n",reg_value * 10);

	Touch_I2C_Read(client, FT6x06_REG_THGROUP, &reg_value, 1);
	TOUCH_LOG("Touch threshold = %d.\n", reg_value * 4);
*/
	return result;
}

static int Ft6x36_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
    const struct firmware *fw = NULL;
    int result = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
    u8 *pFw = NULL;
	
	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
        goto earlyReturn;
    }

	pFw = (u8 *)(fw->data);

	/* IMPLEMENT : parse and get firmware information function */
    /* to do update version information*/
	pFwInfo->moduleMakerID = 0;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = 0;
	pFwInfo->version = 0;

    if(fw->size>0x10a)
    {
        //pFwInfo->isOfficial = (u8)pFw[xxx];
        pFwInfo->version = (u8)pFw[0x10a];/*just update firmware ver*/
    }
    TOUCH_LOG("BIN Firmware Official = %d\n", pFwInfo->isOfficial);
    TOUCH_LOG("BIN Firmware version = 0x%02x\n", pFwInfo->version);

	/* Free firmware image buffer */
	release_firmware(fw);

	return result;
earlyReturn :
    return result;
}

static int Ft6x36_UpdateFirmware(struct i2c_client *client,
    char *pFilename)
{
    int result = TOUCH_SUCCESS;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", result);
        result = TOUCH_FAIL;
        return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);
	result = FirmwareUpgrade(client, fw);
    if(result != 0)
    {
        TOUCH_ERR("failed at FirmwareUpgrade() ( error = %d )\n", result);
        result = TOUCH_FAIL;
        goto earlyReturn;
    }

	/* Free firmware image buffer */
	release_firmware(fw);
	return result;
earlyReturn : 
    release_firmware(fw);
    return result;
}

static int Ft6x36_Set_PDN(struct i2c_client *client)
{
    /* To Do - power off sequence */
    return TOUCH_SUCCESS;
}

static int Ft6x36_Set_KnockOn(struct i2c_client *client)
{
    u8 buf = 0;
    buf = 0x01;
    if (Touch_I2C_Write(client, FT_LPWG_CONTROL_REG, &buf, 1) < 0) 
    {
        TOUCH_ERR("KNOCK_ON Enable write fail\n");
        return TOUCH_FAIL;
    }
    buf = 0x00;

    /*Add interrupt delay time set 0*/
    if(Touch_I2C_Write(client, FT_LPWG_INT_DELAY, &buf, 1) < 0)
    {
        TOUCH_ERR("KNOCK ON INT DELAY TIME write fail\n");
        return TOUCH_FAIL;
    }

    
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_ONLY\n");
    return TOUCH_SUCCESS;
}

static int Ft6x36_Set_KnockCode(struct i2c_client *client, LpwgSetting  *pLpwgSetting)
{
    u8 buf = 0;
    buf = 0x3;
    if (Touch_I2C_Write(client, FT_LPWG_CONTROL_REG, &buf, 1) < 0) 
    {
       TOUCH_ERR("KNOCK_CODE Enable write fail\n");
       return TOUCH_FAIL;
    }

    buf = (u8)pLpwgSetting->tapCount;
    if(Touch_I2C_Write(client, FT_MULTITAP_COUNT_REG, &buf, 1) <0)
    {
        TOUCH_ERR("KNOCK_CODE Tab count write fail\n");
    }
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_CODE\n");

    
    return TOUCH_SUCCESS;
}


static int Ft6x36_Set_Hover(struct i2c_client *client, u8 on)
{
    u8 buf = 0;
    buf = on ? 0x01 : 0x00;
    
    if(Touch_I2C_Write(client, 0xB0, &buf, 1)<0)    
    {     
        TOUCH_LOG("Hover on setting fail.\n");  
        return TOUCH_FAIL;
    }
    else
    {
        TOUCH_LOG("Mode Changed to HOVER %s.\n", on ? "On" : "Off");
    }
    return TOUCH_SUCCESS;
}

static int Ft6x36_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int result = TOUCH_SUCCESS;
    TouchDriverData *pDriverData = i2c_get_clientdata(client);
	TOUCH_FUNC();

	/* IMPLEMENT : each mode change function */
	memcpy(&pDeviceData->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

    TouchDisableIrq();
	if( pDeviceData->currState != newState ) 
	{
        pDeviceData->currState = newState;
        switch(pDeviceData->currState)
        {
           case STATE_NORMAL :
                Ft6x36_Reset(client);
            break;
            
            case STATE_KNOCK_ON_ONLY :
                result = Ft6x36_Set_KnockOn(client);
                break;

            case STATE_KNOCK_ON_CODE :
                result = Ft6x36_Set_KnockCode(client, pLpwgSetting);
                break;

            case STATE_NORMAL_HOVER :
                result = Ft6x36_Set_Hover(client, 1);
                break;

            case STATE_HOVER :
                if(pDriverData->reportData.hover == 1)
                {
                    result = Ft6x36_Set_Hover(client, 0);
                    if( pLpwgSetting->mode == 1 ) 
                    {
                        result = Ft6x36_Set_KnockOn(client);
                    }
                    if( pLpwgSetting->mode == 2 ) 
                    {
                        result = Ft6x36_Set_KnockCode(client, pLpwgSetting);
                    }
                    if( pLpwgSetting->mode == 3 ) 
                    {
                        /*not support*/
                    }
                }
                else
                {
                }
                break;
            case STATE_OFF : 
                result = Ft6x36_Set_PDN(client);
                break;
            default : 
                TOUCH_ERR("Unknown State %d", newState);
                break;
        }
    }
    TouchEnableIrq();
    
	return result; 
}

static int Ft6x36_DoSelfDiagnosis(struct i2c_client *client,
    int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	TOUCH_FUNC();
	/* CAUTION : be careful not to exceed buffer size */

	/* IMPLEMENT : self-diagnosis function */
	*pRawStatus = 1;
	*pChannelStatus = 1;

	dataLen += sprintf(pBuf, "%s", "========= Additional Information =========\n");
	dataLen += sprintf(pBuf+dataLen, "%s", "Device Name = Dummy\n");

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;

}

int Ft6x36_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	

	return TOUCH_SUCCESS;
	
}
/* This code is TestCode */


TouchDeviceSpecificFunction Ft6x36_Func = {

	.Initialize = Ft6x36_Initialize,
	.Reset = Ft6x36_Reset,
	.Connect = Ft6x36_Connect,
	.InitRegister = Ft6x36_InitRegister,
    .ClearInterrupt = Ft6x36_ClearInterrupt,
    .InterruptHandler = Ft6x36_InterruptHandler,
	.ReadIcFirmwareInfo = Ft6x36_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Ft6x36_GetBinFirmwareInfo,
	.UpdateFirmware = Ft6x36_UpdateFirmware,
	.SetLpwgMode = Ft6x36_SetLpwgMode,
	.DoSelfDiagnosis = Ft6x36_DoSelfDiagnosis,
    .AccessRegister = Ft6x36_AccessRegister,
    .device_attribute_list = Ft6x36_attribute_list,
    
};

/* End Of File */
