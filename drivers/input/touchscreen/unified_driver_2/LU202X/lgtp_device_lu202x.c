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
 *    File  	: lgtp_device_lu202x.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[LU202X]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>
#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_lu202x.h>
#include <linux/input/unified_driver_2/lgtp_model_config.h>
#include "TestLimits_lu202x.h"

#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define TPD_I2C_ADDRESS				0x0E
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)

#define LU202X_MAX_KEY 4
#define MAX_FINGER_NUM 2
#if defined(TOUCH_MODEL_CF)
#define MAX_CHANNEL 32
#else
#define MAX_CHANNEL 34
#endif

#define OLD_PANEL_PRODUCT_ID 0x01
#define NEW_PANEL_PRODUCT_ID 0x02
/* LeadingUI Firmware */
#define FW_SIZE 		        30*1024
#define CFG_SIZE 		        1*1024
#define FAC_SIZE		        1*1024

#define FAC_POS			        0xFC00
#define FW_POS			        0x8000

/* Knock On/Code */
#define KNOCK_ON_STATUS		    0x0082
#define KNOCK_TAP_COUNT	    	0x0083
#define KNOCK_STATUS	    	0x00C0
#define KNOCK_TAP_THON		    0x00C1
#define KNOCK_EXCEPT_PALM_ONCH	0x00C5
#define KNOCK_WAKEUP_INTERVAL	0x00C9
#define KNOCK_TAPOFF_TIMEOUT	0x00D2
#define KNOCK_ON_TAP_COUNT	    0x00D4
#define KNOCK_ON_REPORT_DELAY   0x00D5

#define KNOCK_CODE_TAPOFF_TIMEOUT	0x00DB
#define KNOCK_CODE_TAP_COUNT		0x00DD


/* Touch Event Type */
#define TYPE_PRESS		    0x01
#define TYPE_MOVE		    0x02
#define TYPE_RELEASE		0x03

/* Key Event Type */
#define KEY_PRESSED		    1
#define KEY_RELEASED		0
#define CANCEL_KEY		    0xFF

#define SCREEN_MAX_X    	1280
#define SCREEN_MAX_Y    	800
#define PRESS_MAX       	255

#define EVENT_NONE		    0x00
#define EVENT_ABS		    0x01
#define EVENT_KEY	    	0x02
#define EVENT_KNOCK_ON          0x03
#define EVENT_KNOCK_CODE        0x04
#define EVENT_KNOCK_OVER        0x05
#define EVENT_HOVERING_NEAR     0x06
#define EVENT_HOVERING_FAR      0x07
#define EVENT_GLANCE_VIEW       0x08
#define EVENT_GEST		        0x04
#define EVENT_MOUSE		        0x08

#define FWSTATUS_NORMAL		    0x00
#define FWSTATUS_INITREQ	    0xFF
#define FWSTATUS_CHFAIL	    	0xfe
#define FWSTATUS_CALFAIL	    0xfd

#define I2C_DEVICE_ADDRESS_LEN	2
#define MAX_TRANSACTION_LENGTH	8

#define FW_STATUS               0x0000
#define INT_INFORM              0x0001
#define TOUCH_VALID             0x0002
#define TOUCH_KEY               0x0003
#define TOUCH_FINGER            0x0005 

#define FW_VERSION_REG		    0x0080

#define LU202x_MODE_ADDR	0x00E0
#define LU202x_CMDACK_ADDR	0x00ED

#define LU202x_DEVICEID_ADDR	0x10FD
#define LU202x_I2CDONE_ADDR	0x10FF
#define LU202x_CMDReply_ADDR	0x0100
#define LU202x_RAWCAP_ADDR LU202x_CMDReply_ADDR+2
#define LU202x_JITTERCAP_ADDR LU202x_RAWCAP_ADDR+(MAX_CHANNEL * 2)
#define LU202x_AUTOCYCLE_ADDR LU202x_JITTERCAP_ADDR+(MAX_CHANNEL * 2)

#define CMD_I2C_DONE		    0x01
#define CMD_LU202x_CHANGEMODE	0xA3
#define CMD_LU202x_DEVINFO 	0xAA
#define CMD_LU202x_CHCAPTEST	0xB6
#define LU202x_CHCAPTEST_Reply	0xC1

/* Command */
#define LU202x_CMD_FW_CHECKSUM_ADDR	0x0158

/* Major Mode */
#define CMD_LU202x_NORMODE		0x00
#define CMD_LU202x_PDN			0x01
#define CMD_LU202x_DEBUG		0x02
#define CMD_LU202x_IDLE_DOUBLETAB	0x11
#define CMD_LU202x_IDLE_MULTITAB	0x11

/* Minor Mode */
#define CMD_LU202x_NONE		0x0000
#define CMD_LU202x_REFCAP	0x0000

#define HEADER_SIZE 0

/* USERMODE CHANGE */
#define ACCESS_CTRL         0x1000
#define USER_SPACE          0x1001
#define USER_PASSWORD       0x1003
#define ROM_CONTROL         0x1000
#define TSP_INFORM          0x0038
/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct Lu202xDriverDataTag {

	TouchState deviceState;

} Lu202xDriverData;

/****************************************************************************
* Variables
****************************************************************************/
typedef struct {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount; 		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
} lu202x_tpd;

typedef struct {
	u8 status[MAX_FINGER_NUM];
	u8 id[MAX_FINGER_NUM];
	u16 x[MAX_FINGER_NUM];
	u16 y[MAX_FINGER_NUM];
} touch_info;

static int pressed_key = 0;

static Lu202xDriverData gDeviceData = { STATE_NORMAL };
static const char *defaultFirmware[] = {"leadingUI/F580_OFFI_1_11_20150701A.img",
	"leadingUI/F580_OFFI_1_21_20150701B.img"};
int hall_status;


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int Lu202x_Set_Hover(struct i2c_client *client);
void Lu202x_Reset(struct i2c_client *client);
static int Lu202x_Set_PDN(struct i2c_client *client);
static void lu202x_get_rawdata(struct i2c_client *client, u16 *ch_cap_data, u8 mode);
int print_raw(char *buf, u16 *ch_cap_data, u8 mode, int offset);
static int LU202x_update_setmode(struct i2c_client *client, int mode);
static int LU202x_readFW(struct i2c_client *client, unsigned char *pBuf, int addr, int size);
static void lu202x_write_file(char *filename, char *data, int time, int size);


/****************************************************************************
* Local Functions
****************************************************************************/
static int SleepInLu202x( struct i2c_client *client )
{
	u8 i2c_done = CMD_I2C_DONE;
	Lu20xx_I2C_Write ( client, LU202x_I2CDONE_ADDR, &i2c_done, 1 );
	
	return TOUCH_SUCCESS;
}

static int WakeUpLu202x( void )
{
	int result = TOUCH_SUCCESS;
	int loopCount = 500;

	TouchIntPinToggle();
	while ( TouchReadInterrupt() && loopCount )
	{
		usleep ( 1000 );
		loopCount--;
	}

	if( TouchReadInterrupt() == 1 )
	{
		TOUCH_ERR("Failed to wakeup Lu202x\n");
		result = TOUCH_FAIL;
	}

	return result;
	
}


static int LU202x_FlashReadyCheck(struct i2c_client *client)
{
	u8 status = 0;
    TOUCH_FUNC();
	while (1) {
		  // Flash Ready Check
		if (Lu20xx_I2C_Read(client, 0x1000, &status, 1) == -1) {
			TOUCH_ERR("Read status operation failed\n" );		 
			return -1;
		}
        

		if ((status & 0x40) == 0x40) {
			break;
		}
	}
	msleep(100);
    return 0;
}

static int LU202x_PageErase(struct i2c_client *client, int addr)
{
	u8 Cmd[2] = {0, 0};
	
	u8 pagenum = addr/1024;	
    TOUCH_FUNC();


	/* Erase */
	// main Block Select
	Cmd[0] = 0x02;
   	if(Lu20xx_I2C_Write(client, 0x1001, Cmd, 1) == -1){
		TOUCH_ERR("Main Block Select command operation failed\n" );
		goto ERASE_FAIL;
		}

	// Erase PageNum Write
	Cmd[0] = pagenum;
	if(Lu20xx_I2C_Write(client, 0x1006, Cmd, 1) == -1){
		TOUCH_ERR(" Erage Page Write command operation failed\n" );
		goto ERASE_FAIL;
		}

	// Erase Function SelectEra
	Cmd[0] = 0x82;
	if(Lu20xx_I2C_Write(client, 0x1000, Cmd, 1) == -1){
		TOUCH_ERR("Page Erase Function Select command operation failed\n" );
		goto ERASE_FAIL;
		}

	if(LU202x_FlashReadyCheck(client) == -1){
		TOUCH_ERR("Flash Ready failed\n" );
		goto ERASE_FAIL;
		}

	return 0;

ERASE_FAIL:
	return -1;
}

static int LU202x_PageWrite(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};
    TOUCH_FUNC();
	// All Area Enable
	Cmd[0] = 0x01;
	if (Lu20xx_I2C_Write(client, 0x1001, Cmd, 1) == -1)	{
			TOUCH_ERR("Main Block Select operation failed\n" );
			return -1;
		}

	// I2C E-Flash Program Enable (Program Enable Fuction Select)
	Cmd[0] = 0x88;
	if (Lu20xx_I2C_Write(client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_ERR("Program Fuction Select operation failed\n" );
			return -1;
		}

	// Data Write 
    if (Lu20xx_I2C_Write(client, addr, pBuf, size) == -1)	{
			TOUCH_ERR("Data Write operation failed\n" );
			return -1;
		}

	return 0;
}

static int LU202x_PageRead(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
	u8 Cmd[2] = {0, 0};

	// Main Block Select
	Cmd[0] = 0x01;
	if (Lu20xx_I2C_Write(client, 0x1001, Cmd, 1) == -1)	{
			TOUCH_ERR("Main Block Select operation failed\n" );
			return -1;
		}

	// Read Function Select
	Cmd[0] = 0x81;
	if (Lu20xx_I2C_Write(client, 0x1000, Cmd, 1) == -1)	{
			TOUCH_ERR("Read Function operation failed\n" );
			return -1;
		}

	// Data Read
	if (Lu20xx_I2C_Read(client, addr, pBuf, size) == -1)	{
			TOUCH_ERR("Data Read operation failed\n" );
			return -1;
		}

	return 0;
}

static int Lu202x_programFW(struct i2c_client *client, u8 *pBuf, int addr, int size)
{
    int i = 0;
	int W_addr = addr;
	int R_addr = addr;
    TOUCH_FUNC();
	for(i=0; i<size; i+= 256, W_addr+=256)
	{
		if((W_addr % 1024) == 0)
		{	// 1K Erase
			if(LU202x_PageErase(client, W_addr) == -1){
				TOUCH_ERR("Data Page Erase failed \n");
				return -1;
				}
		}
											
		// 256Bytes Write * 4
		if(LU202x_PageWrite(client, (pBuf+i), W_addr , 256) == -1){
			TOUCH_ERR("Data Write failed \n");
			return -1;
			}
	}

	TOUCH_LOG("%s Writing (%d/%d) bytes\n",
				(size == FAC_SIZE) ? "FACTORY" : "FIRMWARE", i, size );

   // Data Check
   if(0)
   {
	for(i=0; i<size+FAC_SIZE; i+=1024, R_addr+=1024)
	{
        TOUCH_LOG("R_ADDRESS = 0x%x", R_addr);
		if(LU202x_PageRead(client, (pBuf+i), R_addr, 1024) == -1){
			TOUCH_ERR("Data Read for Check operation failed\n" );
			return -1;
		}
        else
        {
            TOUCH_LOG("DATA CHECK SUCCESS [%d] page", i);
        }
	}
   }
    TOUCH_LOG("END PROGRAM FW");
	return 0;
}

static int Lu202x_ReadChecksum( struct i2c_client *client, char *pCheckSum )
{
	int result = TOUCH_SUCCESS;
	
	u8 temp[3] = {CMD_LU202x_DEVINFO, 0, 0};

	if( WakeUpLu202x() )
	{
		result = TOUCH_FAIL;
		goto exit;
	}

	/* Change Checksum Mode */
	Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, temp, 3);
	if( SleepInLu202x(client) )
	{
		TOUCH_ERR("Failed to change mode\n");
		result = TOUCH_FAIL;
		goto exit;
	}
	msleep(2);
	
	/* Checksum Mode check */
	Lu20xx_I2C_Read(client, LU202x_CMDACK_ADDR, temp, 3);
	if ( temp[2] != CMD_LU202x_DEVINFO )
	{
		TOUCH_ERR("Failed to read ack\n");
		result = TOUCH_FAIL;
		goto exit;
	}
	mdelay(2);

	Lu20xx_I2C_Read(client, LU202x_CMD_FW_CHECKSUM_ADDR, pCheckSum, 6);
	if( SleepInLu202x(client) )
	{
		TOUCH_ERR("Failed to read checksum\n");
		result = TOUCH_FAIL;
		goto exit;
	}

exit:
	
	return result;
	
}

/****************************************************************************
* Global Functions
****************************************************************************/

int Lu202x_Initialize(struct i2c_client *client)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

void Lu202x_Reset(struct i2c_client *client)
{
	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(200);

	TOUCH_LOG("LU202X was reset\n");

	gDeviceData.deviceState = STATE_NORMAL;
}

static ssize_t store_Set_vp(struct i2c_client *client, const char *buf, size_t count)
{
    int cmd = 0;
    TOUCH_LOG("[store_Set_vp]\n");
    sscanf(buf, "%d", &cmd);

    if(cmd == 1)
    {
        Lu202x_Set_Hover(client);
        TOUCH_LOG("Set hover mode\n");
    }
    else
    {
        Lu202x_Reset(client);    
    }

    return count;
}
static LGE_TOUCH_ATTR(Set_vp, S_IRUGO | S_IWUSR, NULL, store_Set_vp);


static ssize_t store_Power_Control(struct i2c_client *client, const char *buf, size_t count)
{
    static struct regulator *vdd_dd = NULL;
    int error = 0;
    int cmd = 0;
    
    sscanf(buf, "%d", &cmd);
    if( vdd_dd == NULL ) {
        vdd_dd = regulator_get(&client->dev, "vdd");
        if (IS_ERR(vdd_dd))
        {
            error = PTR_ERR(vdd_dd);
            TOUCH_ERR("failed to get regulator ( error = %d )\n",error);
            return count;
        }
        error = regulator_set_voltage(vdd_dd, 2800000, 2800000);
        if (error < 0) {
            TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", error);
            return count;
        }
    }
    
    
    if(cmd == 1)
    {
        error = regulator_enable(vdd_dd);
        if (error < 0) {
            TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
            return count;
        }
        msleep(15);
    }
    else
    {
        error = regulator_disable(vdd_dd);
        if (error < 0) {
            TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
            return count;
        }
    }
    

    return count;
}
static LGE_TOUCH_ATTR(Power_Control, S_IRUGO | S_IWUSR, NULL, store_Power_Control);

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
	return ret;
}
static LGE_TOUCH_ATTR(Model_Info, S_IRUGO | S_IWUSR, show_Model_Info, NULL);

static ssize_t show_Disable_Irq(struct i2c_client *client, char *buf)
{
    int ret = 0;	
    ret = sprintf(buf, "Disable IRQ\n");
    TouchDisableIrq();
	return ret;
}
static LGE_TOUCH_ATTR(Disable_Irq, S_IRUGO | S_IWUSR, show_Disable_Irq, NULL);

static ssize_t store_Set_PDN_byHallIC(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	if (sscanf(buf, "%d", &hall_status) <= 0)
		return count;

	TOUCH_LOG("Called by Hall IC.\n");

	switch(hall_status)
	{
	case COVER_CLOSED :
		TOUCH_LOG("Folder Closed in miniOS.\n");
		TouchDisableIrq();
		release_all_touch_event(pDriverData);
		Lu202x_Set_PDN(client);
		break;
	case COVER_OPENED :
		TOUCH_LOG("Folder Open in miniOS.\n");
		Lu202x_Reset(client);
		TouchEnableIrq();
		break;
	default :
		break;
	}

	return count;
}
static LGE_TOUCH_ATTR(hall_status, S_IRUGO | S_IWUSR, NULL, store_Set_PDN_byHallIC);

static ssize_t show_lu202x_reference(struct i2c_client *client, char *buf)
{
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	TouchDisableIrq();

	lu202x_get_rawdata(client, ch_cap_data, GET_REFERENCE);
	ret = print_raw(buf, ch_cap_data, GET_REFERENCE, ret);

	TouchEnableIrq();

	return ret;
}
static LGE_TOUCH_ATTR(lu202x_rawcap, S_IRUGO | S_IWUSR, show_lu202x_reference, NULL);

static ssize_t show_lu202x_jitter_cap(struct i2c_client *client, char *buf)
{
	u16 ch_cap_data[MAX_CHANNEL] = {0,};
	int ret = 0;

	TouchDisableIrq();

	lu202x_get_rawdata(client, ch_cap_data, GET_JITTER);
	ret = print_raw(buf, ch_cap_data, GET_JITTER, ret);

	TouchEnableIrq();

	return ret;
}
static LGE_TOUCH_ATTR(lu202x_jitter_cap, S_IRUGO | S_IWUSR, show_lu202x_jitter_cap, NULL);

static void lu202x_write_file(char *filename, char *data, int time, int size)
{
	int fd = 0;
	int flag = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n", \
		my_date.tm_mon + 1,my_date.tm_mday, \
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec, \
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	if (!strcmp(filename, FW_FILE_PATH)) {
		flag = O_WRONLY|O_CREAT;
	} else if (!strcmp(filename, SELF_DIAGNOSTIC_FILE_PATH)) {
		flag = O_WRONLY|O_CREAT|O_APPEND;
	}
	fd = sys_open(filename, flag, 0666);

	if(fd < 0) {
		TOUCH_LOG("write open failed, fd : %d\n", fd);
		return;
	} else {
		TOUCH_LOG("write open success, fd : %d\n",fd);
	}

	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
			TOUCH_ERR("Time write success.\n");
		}
		if (size > 0) {
			sys_write(fd, data, size);
		}
		sys_close(fd);
	}
	set_fs(old_fs);
}


static ssize_t show_lu202x_read_fw(struct i2c_client *client, char *buf)
{
	int ret = 0;
	u8 *pBuf = NULL;
	u32 size = 0;
	u32 position = 0;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);

	wake_lock(&pDriverData->lpwg_wake_lock);
	TouchDisableIrq();

	/* Set mode */
	if (LU202x_update_setmode(client, 1) == -1) {
		TOUCH_ERR("Set Mode failed\n");
		goto exit;
	}

	size = FW_SIZE + CFG_SIZE + FAC_SIZE;
	position = FW_POS;

	pBuf = kmalloc(size, GFP_KERNEL);
	/* Read Cal Data */
	if (LU202x_readFW(client, pBuf, position, size) == -1) {
		TOUCH_ERR("Read Cal Data failed\n" );
		goto exit;
	}
	/* IC Firmware binary write to file */
	lu202x_write_file(FW_FILE_PATH, pBuf, 0, size);

exit :
	LU202x_update_setmode(client, 0);
	Lu202x_Reset(client);

	kfree(pBuf);
	TouchEnableIrq();
	wake_unlock(&pDriverData->lpwg_wake_lock);

	return ret;
}

static LGE_TOUCH_ATTR(read_fw_dump, S_IRUGO | S_IWUSR, show_lu202x_read_fw, NULL);

static struct attribute *lu202x_attribute_list[] = {
	&lge_touch_attr_Model_Info.attr,
    &lge_touch_attr_Set_vp.attr,
    &lge_touch_attr_Disable_Irq.attr,
    &lge_touch_attr_Power_Control.attr,
    &lge_touch_attr_hall_status.attr,
    &lge_touch_attr_lu202x_rawcap.attr,
    &lge_touch_attr_lu202x_jitter_cap.attr,
    &lge_touch_attr_read_fw_dump.attr,
	NULL,
};

int Lu202x_Connect(void)
{
#if 1 /*                                                 */
	int ret = 0;
	u8 reg[2] = {0};
	u8 data[2] = {0};

	TOUCH_FUNC();

	reg[0] = 0x10;
	reg[1] = 0xFD;

	ret = touch_i2c_read_for_query( TPD_I2C_ADDRESS, reg, 2, data, 2);
	if( ret == TOUCH_SUCCESS )
	{
		if( (data[0] == 0x20) && (data[1] == 0x20) )
		{
			TOUCH_LOG("LU202X was detected\n");
            if(TouchReadMakerId() == 1)
            {
                TOUCH_LOG("Module is LGIT\n");
            }
            else
            {
                TOUCH_LOG("Module is Suntel\n");
            }
			return TOUCH_SUCCESS;
		}
		else
		{
			TOUCH_LOG("LU202X was detected but deviceID is not matched\n");
			return TOUCH_FAIL;
		}
	} 
	else 
	{
		TOUCH_LOG("LU202X was NOT detected\n");
		return TOUCH_FAIL;
	}
#else
	if( TouchReadMakerId() == 0 )
	{
		TOUCH_LOG("LU202X was detected\n");
		return TOUCH_SUCCESS;
	}
	else 
	{
		TOUCH_LOG("LU202X was NOT detected\n");
		return TOUCH_FAIL;
	}
#endif
}

int Lu202x_InitRegister(struct i2c_client *client)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

/*                                                 */
static int get_lpwg_data(struct i2c_client *client, TouchReadData *pData)
{
	u8 i = 0;
	u8 tap_count = 0;
	u8 buffer[12 * 4] = {0,};
	if(Lu20xx_I2C_Read(client, KNOCK_TAP_COUNT, &tap_count, sizeof(u8)) == 0)
	{	
        pData->count = tap_count;
	}
	else
   	{
        TOUCH_ERR("KNOCK_TAP_COUNT Read Fail\n");
        goto error;
	}

	if (!tap_count)
	{
        TOUCH_LOG("TAP COUNT = %d\n",tap_count);
		goto error;
	}
    
	if( Lu20xx_I2C_Read(client, KNOCK_TAP_COUNT+1, buffer,4*tap_count) != 0 )
	{
        TOUCH_ERR("LPWG Data Read Fail\n");
		goto error; 
	}
	
	for (i = 0; i < tap_count; i++)
	{
		pData->knockData[i].x = (buffer[4*i+1] << 8 | buffer[4*i]);
		pData->knockData[i].y = (buffer[4*i+3] << 8 | buffer[4*i+2]);
        /*This code is only debugging*/
        //TOUCH_LOG("LPWG data [%d, %d]\n", pData->knockData[i].x, pData->knockData[i].y);
    }
	SleepInLu202x(client);
	return TOUCH_SUCCESS;
error:
	SleepInLu202x(client);
	return TOUCH_FAIL;
}
/* GLANCE VIEW TEST VERSION */
/*
static int get_glance_view_data(struct i2c_client *client, TouchReadData *pData)
{
    u8 buffer[12*4] = {0,};
    
    if( Lu20xx_I2C_Read(client, KNOCK_TAP_COUNT+1, buffer,8) != 0 )
	{
        TOUCH_ERR("LPWG Data Read Fail\n");
		goto error; 
	}
    pData->knockData[0].x = (buffer[4*0+1] << 8 | buffer[4*0]);
    pData->knockData[0].y = (buffer[4*0+3] << 8 | buffer[4*0+2]);

    pData->knockData[1].x = (buffer[4*1+1] << 8 | buffer[4*1]);
    pData->knockData[1].y = (buffer[4*1+3] << 8 | buffer[4*1+2]);

    TOUCH_LOG("[GLANCE VIEW]-Start Point x = %d, y = %d\n",pData->knockData[0].x, pData->knockData[0].y);
    TOUCH_LOG("[GLANCE VIEW]-End Point x = %d, y = %d\n",pData->knockData[1].x, pData->knockData[1].y);
                
    SleepInLu202x(client);
    return TOUCH_SUCCESS;
error:
    SleepInLu202x(client);
    return TOUCH_FAIL;        
}
*/
static void Lu202x_ClearInterrupt(struct i2c_client *client)
{
	
   
	return;
}

int Lu202x_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	int i=0;
	TouchFingerData *pFingerData = NULL;
	TouchKeyData *pKeyData = NULL;
	lu202x_tpd touch_data;
	touch_info info;
	static u8 pressure_temp = 0;
    u8 valid = 0;
	memset(&info, 0x0, sizeof(touch_info));
	memset(&touch_data, 0x0, sizeof(lu202x_tpd));

	pressure_temp ^= 1;

	pData->type = DATA_UNKNOWN;
	pData->count = 0;
    
    if(Lu20xx_I2C_Read(client, INT_INFORM, (u8 *)&touch_data.EventType, 1) != 0)
    {
        TOUCH_ERR("Read Interrupt Status Fail. (0x0000)\n");
        goto fail;
    }
    
    if(Lu20xx_I2C_Read(client, TOUCH_VALID, &valid, 1) != 0)
    {
        TOUCH_ERR("Read Touch valid count Fail. (0x0002)\n");
        goto fail;
    }
    
    switch(touch_data.EventType)
    {
        case EVENT_ABS :            
            if(Lu20xx_I2C_Read(client, TOUCH_FINGER, (u8 *)&touch_data.Point, 8) != 0)
            {
                TOUCH_ERR("Read touch data Fail. (0x0005)\n");
                goto fail;
            }
            SleepInLu202x(client);
            pData->type = DATA_FINGER;
            for ( i = 0 ; i < valid; i++ )
            {
            	info.x[i] = ( ( touch_data.Point[i*4+1] & 0x07 ) << 8 ) | touch_data.Point[i*4];
            	info.y[i] = ( ( touch_data.Point[i*4+2] & 0x38 ) << 5 ) | ( ( touch_data.Point[i*4+2] & 0x07 ) << 5 ) | ( ( touch_data.Point[i*4+1] >> 3 ) & 0x1f );
            	info.id[i] = ( ( touch_data.Point[i*4+3] & 0x07 ) << 3 ) | ( ( touch_data.Point[i*4+2] >> 6 ) & 0x03 );
            	info.status[i] = ( touch_data.Point[i*4+3] >> 3 ) & 0x03;
            	if(info.status[i] == TYPE_PRESS)
            	{
            		pFingerData = &pData->fingerData[pData->count];
            		pFingerData->id = info.id[i];
            		pFingerData->x  = info.x[i];
            		pFingerData->y  = info.y[i];
            		pFingerData->width_major = 15;
            		pFingerData->width_minor = 10;
            		pFingerData->orientation = 1;
            		pFingerData->pressure = 20 + pressure_temp;
            		pData->count++;
            	}
            }
            break; 

        case EVENT_KEY : 
            if(Lu20xx_I2C_Read(client, TOUCH_KEY, (u8 *)&touch_data.KeyData, 2) != 0)
            {
                TOUCH_ERR("Read touch key data Fail. (0x0003)\n");
                 goto fail;
            }
            SleepInLu202x(client);
    		pData->type = DATA_KEY;
    		pData->count++;
    		pKeyData = &pData->keyData;
    		if ( touch_data.KeyData[0] == 0 )
    		{
    			pKeyData->index = pressed_key;
    			pKeyData->pressed = KEY_RELEASED;
    			TOUCH_LOG ( "Touch Key[%d] was Released\n", pKeyData->index );
    			pressed_key = 0;
    		}
    		else
    		{
    			pressed_key = touch_data.KeyData[0];
    			pKeyData->index = pressed_key;
    			pKeyData->pressed = KEY_PRESSED;
    			TOUCH_LOG ( "Touch Key[%d] was Pressed\n", pKeyData->index );
    		}
            break;
		
        case EVENT_KNOCK_ON :
            pData->type = DATA_KNOCK_ON;
            TOUCH_LOG("[KNOCK ON] Event Type = %d\n", touch_data.EventType);
            break;

        case EVENT_KNOCK_CODE : 
            pData->type = DATA_KNOCK_CODE;
            get_lpwg_data(client, pData);
            TOUCH_LOG("[KNOCK CODE]Event Type = %d\n", touch_data.EventType);
            break;
        
        case EVENT_KNOCK_OVER :
            pData->type = DATA_KNOCK_CODE;
            pData->knockData[0].x = 1;
            pData->knockData[0].y = 1;
            pData->knockData[1].x = -1;
            pData->knockData[1].y = -1;
            TOUCH_LOG("[KNOCK CODE OVER] Event Type = %d\n", touch_data.EventType);
            break;

        case EVENT_HOVERING_NEAR :
            /*//
            pData->type = DATA_HOVER_NEAR; 
            pData->hoverState = 0;
            TOUCH_LOG("[HOVERING NEAR] Event Type = %d\n", touch_data.EventType);
            */
            /*To Do*/
            TOUCH_LOG("[HOVERING NEAR] Event Type = %d\n", touch_data.EventType);
            break;
        case EVENT_HOVERING_FAR :
            /*
            pData->type = DATA_HOVER_FAR;
            pData->hoverState = 1;
            TOUCH_LOG("[HOVERING FAR] Event Type = %d\n", touch_data.EventType);
            */
            /*To Do*/
            TOUCH_LOG("[HOVERING FAR] Event Type = %d\n", touch_data.EventType);
            break;
        case EVENT_GLANCE_VIEW : 
            /*
            pData->type = DATA_SWIPE;
            get_glance_view_data(client, pData);
            TOUCH_LOG("[GLANCE VIEW] Event Type = %d\n", touch_data.EventType);
            */
            TOUCH_LOG("[GLANCE VIEW] Event Type = %d\n", touch_data.EventType);
        default:
            TOUCH_LOG("[Unknown] Event Type = %d\n",touch_data.EventType);
            break;
		
	}
    //SleepInLu202x(client);
    return TOUCH_SUCCESS;
fail:
    SleepInLu202x(client);
    return TOUCH_FAIL;
}

static int ReadModuleInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
    
    u8 Cmd[2] = {0,};
    u8 info[8] = {0,};
    TOUCH_FUNC();

    /*Enter user mode*/
    //User mode enter (LU2020)
    Cmd[0] = 0x80;  
    if (Lu20xx_I2C_Write(client, ACCESS_CTRL, Cmd, 1) == -1) {
        TOUCH_LOG("Access Rom Ctrl \n" );
        return TOUCH_FAIL;
    }

    Cmd[0] = 0x75;
    Cmd[1] = 0x6C;
    if (Lu20xx_I2C_Write(client, USER_PASSWORD, Cmd, 2) == -1) {
        TOUCH_LOG("Set password operation failed\n" );
        return TOUCH_FAIL;
    }

    Cmd[0] = 0x08;
    if (Lu20xx_I2C_Write(client, USER_SPACE, Cmd, 1) == -1) {
        TOUCH_LOG("Set password operation failed\n" );
        return TOUCH_FAIL;
    }

    Cmd[0] = 0x81;
    if (Lu20xx_I2C_Write(client, ACCESS_CTRL, Cmd, 1) == -1) {
        TOUCH_LOG("Set password operation failed\n" );
        return TOUCH_FAIL;
    }

    if (Lu20xx_I2C_Read(client, TSP_INFORM, info, 8) == -1) {
        TOUCH_LOG("User Area-Read  failed\n" );
        return TOUCH_FAIL;
    }
    TOUCH_LOG("Chip ID = %2x%2x\n", info[0], info[1]);
    TOUCH_LOG("Vendor ID = %d\n", info[2]);
    TOUCH_LOG("Product ID = %d\n", info[3]);

    pFwInfo->moduleMakerID = info[2];
	pFwInfo->productID = info[3];
    //pFwInfo->moduleVersion= 255;
    Lu202x_Reset(client);    
    return TOUCH_SUCCESS;   
    
}

int Lu202x_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	int result = TOUCH_SUCCESS;

	u8 readData[4] = {0};
	char checksum[6] = {0};
	
	TOUCH_FUNC();

	WakeUpLu202x();
	if(Lu20xx_I2C_Read(client, FW_VERSION_REG-2, readData, 4) != 0)
	{
        TOUCH_ERR("Firmware version read fail (0x0080)\n");
        result = TOUCH_FAIL;
    }
	SleepInLu202x(client);	

    ReadModuleInfo(client, pFwInfo);	
    
    pFwInfo->isOfficial = readData[2] >>7;
	pFwInfo->version = readData[3]; /* release version */

	/* Reset to read checksum */
	Lu202x_Reset(client);

	/* Read IC Checksum*/
	if (Lu202x_ReadChecksum(client, checksum)) {
		TOUCH_ERR("Failed at read checksum\n");
	}
	memcpy(pFwInfo->checksum, checksum, 6);

	TOUCH_LOG("IC Firmware Official = %d\n", pFwInfo->isOfficial);
	TOUCH_LOG("IC Firmware Version = 0x%02X 0x%02X\n", readData[2], readData[3]);
	TOUCH_LOG("IC Firmware Version = 0x%04X\n", pFwInfo->version);
	/* Read Checksum */
	TOUCH_LOG("I2C read checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			checksum[0], checksum[1], checksum[2], checksum[3], checksum[4], checksum[5]);

	return result;
}

int Lu202x_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	const struct firmware *fw = NULL;
    int result = TOUCH_SUCCESS;
    char *pFwFilename = NULL;
	u8 *pFw = NULL;
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	unsigned int sum = 0;
	int i = 0;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		if (pDriverData->icFwInfo.productID == OLD_PANEL_PRODUCT_ID)
			pFwFilename = (char *)defaultFirmware[0];
		else if (pDriverData->icFwInfo.productID == NEW_PANEL_PRODUCT_ID)
			pFwFilename = (char *)defaultFirmware[1];
		else
			pFwFilename = (char *)defaultFirmware[1];
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		goto earlyReturn;
	}
	pFw = (u8 *)(fw->data);
	pFwInfo->isOfficial = pFw[0x79FE]>>7;
	pFwInfo->version = pFw[0x79FF]; /* release version */

	/* Read Binary Checksum - firmware */
	/* Read 30kbytes by 4bytes unit */
	/* 30k = 30 * 1024 / 4*/
	for (i=0; i<7680; i++) {
		sum += (*(fw->data+i*4 + HEADER_SIZE))       |
			(*(fw->data+i*4 + 1 + HEADER_SIZE) << 8) |
			(*(fw->data+i*4 + 2 + HEADER_SIZE) <<16) |
			(*(fw->data+i*4 + 3 + HEADER_SIZE) <<24  );
	}
	TOUCH_LOG("sum = %x ", sum);
	pFwInfo->checksum[0] = (u8)((sum & 0x000000FF));
	pFwInfo->checksum[1] = (u8)((sum & 0x0000FF00) >>8);
	pFwInfo->checksum[2] = (u8)((sum & 0x00FF0000) >>16);
	pFwInfo->checksum[3] = (u8)((sum & 0xFF000000) >>24);
	sum = 0;

	/* Read Binary Checksum - parameter */
	/* Read 1kbytes by 2bytes unit */
	/* [ Header 16Bytes | Firmware Data 30720Bytes | Not include checksum data 12Bytes | Parameter Data 1011Bytes | not use 1bytes]*/
	/* 30k = 1 * 1024 / 2*/
	//HEADER_SIZE + 30720 + 12
	for (i=0; i<506; i++) {
		if (i == 505) {
			sum += (*(fw->data+i*2 + HEADER_SIZE + 30720 + 13));
			break;
		}
		sum += ((*(fw->data+i*2 + HEADER_SIZE + 30720 + 13))
				| (*(fw->data+i*2 +1 + HEADER_SIZE + 30720 + 13) << 8));
	}
	pFwInfo->checksum[4] = (u8)((sum & 0x000000FF));
	pFwInfo->checksum[5] = (u8)((sum & 0x0000FF00) >> 8);

	TOUCH_LOG("BIN Firmware Official = %d\n", pFwInfo->isOfficial);
	TOUCH_LOG("BIN Firmware Version = 0x%02x 0x%02x", pFw[0x79FE], pFw[0x79FF]);
	TOUCH_LOG("BIN Firmware Version = 0x%04X\n", pFwInfo->version);
	TOUCH_LOG("BIN Firmware checksum = 0x%04X\n", pFwInfo->version);
	/* Calculated Checksum*/
	TOUCH_LOG("Fw binary checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			pFwInfo->checksum[0], pFwInfo->checksum[1], pFwInfo->checksum[2], pFwInfo->checksum[3], pFwInfo->checksum[4], pFwInfo->checksum[5]);

	/* Free firmware image buffer */
	release_firmware(fw);

earlyReturn:
	return result;

}


int Lu202x_UpdateFirmware(struct i2c_client *client, char *pFilename )
{
	int result = TOUCH_SUCCESS;

	unsigned int sum = 0;
	int i = 0;

	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	u8 Cmd[2] = {0,};
	u8 bin_checksum[6]= {0,};

    char *pFwFilename = NULL;
	char checksum[6] = {0};

    TouchDriverData *pDriverData = i2c_get_clientdata(client);

	TOUCH_FUNC();
	
	if( pFilename == NULL ) 
    {
		if (pDriverData->icFwInfo.productID == OLD_PANEL_PRODUCT_ID)
			pFwFilename = (char *)defaultFirmware[0];
		else if (pDriverData->icFwInfo.productID == NEW_PANEL_PRODUCT_ID)
			pFwFilename = (char *)defaultFirmware[1];
		else
			pFwFilename = (char *)defaultFirmware[1];
	} 
    else 
    {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	result = request_firmware(&fw, pFwFilename, &client->dev);
	if( result )
	{
		TOUCH_ERR("Failed at request_firmware() ( error = %d )\n", result);
		result = TOUCH_FAIL;
		return result;
	}
    else
    {
        pBin = (u8 *)(fw->data+HEADER_SIZE); /* header size is 16bytes */
    }
    
    WakeUpLu202x();
    
    Cmd[0] = 0x80;
    if (Lu20xx_I2C_Write(client, 0x1000, Cmd, 1) == -1) 
	{
        TOUCH_ERR("Set mode operation failed\n" );
        goto earlyReturn;
    }

    Cmd[0] = 0x75;
    Cmd[1] = 0x6C;
    
    if (Lu20xx_I2C_Write(client, 0x1003, Cmd, 2) == -1) 
    {
        TOUCH_ERR("Set password operation failed\n" );
        goto earlyReturn;
    }
    
    /*This part is LU2020 firmware update. */
    if(Lu202x_programFW(client, pBin, FW_POS, FW_SIZE + CFG_SIZE) == - 1)
    {
		TOUCH_ERR("Failed to program firmware\n");
        result = TOUCH_FAIL;
        goto earlyReturn;
    }
  
	/* Reset to read checksum */
	Lu202x_Reset(client);

    /* Read Binary Checksum - firmware */
    /* Read 30kbytes by 4bytes unit */
    /* 30k = 30 * 1024 / 4*/
    for( i = 0; i<7680; i++ )
    {

      sum += (*(fw->data+i*4 +HEADER_SIZE ))       | 
             (*(fw->data+i*4 +1 +HEADER_SIZE ) << 8) | 
             (*(fw->data+i*4 +2 +HEADER_SIZE ) <<16) |
             (*(fw->data+i*4 +3 +HEADER_SIZE ) <<24  );
    }    
    TOUCH_LOG("sum = %x ", sum);
    bin_checksum[0] = (u8)((sum & 0x000000FF) );
    bin_checksum[1] = (u8)((sum & 0x0000FF00) >> 8 );
    bin_checksum[2] = (u8)((sum & 0x00FF0000) >>16 );
    bin_checksum[3] = (u8)((sum & 0xFF000000) >>24 );
    sum = 0;
      
    /* Read Binary Checksum - parameter */
    /* Read 1kbytes by 2bytes unit */
    /* [ Header 16Bytes | Firmware Data 30720Bytes | Not include checksum data 12Bytes | Parameter Data 1011Bytes | not use 1bytes]*/
    /* 30k = 1 * 1024 / 2*/
    //HEADER_SIZE + 30720 + 12
    for( i = 0; i< 506; i++)
    {
        if(i == 505)
        {
            sum += (*(fw->data+i*2 + HEADER_SIZE + 30720 + 13 ) );
            break;
        }
         sum += ((*(fw->data+i*2 + HEADER_SIZE + 30720 + 13 )  ) 
             | (*(fw->data+i*2 +1 + HEADER_SIZE + 30720 + 13 ) <<8 ));
    }
    bin_checksum[4] = (u8)((sum & 0x000000FF) );
    bin_checksum[5] = (u8)((sum & 0x0000FF00) >> 8);
    
    /* Read IC Checksum*/
    result = Lu202x_ReadChecksum(client, checksum);
	if( result )
	{
		TOUCH_ERR("Failed at read checksum\n");
		result = TOUCH_FAIL;
		goto earlyReturn;
	}

    /* Calculated Checksum*/
    TOUCH_LOG("Fw binary checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
        bin_checksum[0], bin_checksum[1], bin_checksum[2], bin_checksum[3], bin_checksum[4], bin_checksum[5] );

    /* Read Checksum */
	TOUCH_LOG("I2C read checksum = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		checksum[0], checksum[1], checksum[2], checksum[3], checksum[4], checksum[5] );

	/* Compare checksum */
	if( memcmp((u8 *)(bin_checksum), checksum, 6) )
	{
		TOUCH_ERR("Checksum value is not same. Failed to program firmware\n");
		result = TOUCH_FAIL;
	}
    

earlyReturn:

    SleepInLu202x(client);
        
    /* Free firmware image buffer */
    release_firmware(fw);
	
	/* Reset ??? */
	Lu202x_Reset(client);
	return result;
	
}

static void Lu202x_Set_Check(struct i2c_client *client)
{
    u8 buf[5] = {CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
    if(Lu20xx_I2C_Read(client, LU202x_CMDACK_ADDR, buf, 3) == -1)
    {
       TOUCH_ERR("Read Cmd ACK fail.\n");
    }
    else
    {
       if(buf[2] != CMD_LU202x_CHANGEMODE)
       {
           TOUCH_ERR("Mode Change fail.\n");
       }
    }
}

static int Lu202x_Set_PDN(struct i2c_client *client)
{
    u8 buf[5] = {CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
    WakeUpLu202x();
    buf[1] = 0x01;
    if(Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buf, sizeof(buf)) != 0)
    {
        TOUCH_ERR(" Mode chage power down mode fail.\n");
        goto fail;
    }   
    
    SleepInLu202x(client);
    Lu202x_Set_Check(client);
    TOUCH_LOG("Changed to PDN mode.\n");
    return TOUCH_SUCCESS;
fail:
    SleepInLu202x(client);
    TOUCH_LOG("Mode Changed to PDN mode fail.\n");
    return TOUCH_FAIL;    
}

static int LU202x_update_setmode(struct i2c_client *client, int mode)
{
	u8 Cmd[2] = {0, 0};

	/* FW Upgrade mode */
	if (mode) {
		disable_irq(client->irq);

		Cmd[0] = 0x80;
		if (Lu20xx_I2C_Write(client, 0x1000, Cmd, 1) == -1) {
			TOUCH_ERR("Set mode operation failed\n");
			return -1;
		}

		Cmd[0] = 0x75;
		Cmd[1] = 0x6C;
		if (Lu20xx_I2C_Write(client, 0x1003, Cmd, 2) == -1) {
			TOUCH_ERR("Set password operation failed\n" );
			return -1;
		}
	} else {
		gpio_direction_input(TOUCH_GPIO_INTERRUPT);
		enable_irq(client->irq);
	}
	TOUCH_LOG("LU202x_update_setmode is success\n");

	return 0;
}

static int LU202x_readFW(struct i2c_client *client, unsigned char *pBuf, int addr, int size)
{
	u8 *temp;
	u8 Cmd[2] = {0, 0};
	int i;

	temp = (u8 *) kmalloc(sizeof(u8)*1024, GFP_KERNEL);
	/* Set Read Command */
	Cmd[0] = 0x01;
	Cmd[1] = 0x81;
	if (Lu20xx_I2C_Write(client, 0x1001, &Cmd[0], 1) == -1)	{
		TOUCH_ERR("Read Cmd operation failed @ 0x1001\n" );
		goto fail;
	}
	if (Lu20xx_I2C_Write(client, 0x1000, &Cmd[1], 1) == -1)	{
		TOUCH_ERR("Read Cmd operation failed @ 0x1000\n" );
		goto fail;
	}

	for (i = 0; i < size; i += 1024) {
		if (Lu20xx_I2C_Read(client, addr+i, temp, 1024) == -1) {
			TOUCH_ERR("bin write operation failed %d\n", i );
			goto fail;
		}

		memcpy(&pBuf[i], temp, 1024);
	}

	kfree(temp);
	return 1;
fail :
	kfree(temp);
	return -1;
}

static int Lu202x_Set_KnockOn(struct i2c_client *client)
{
    u8 buf[5] = {CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
    u8 temp = 0;
    WakeUpLu202x();
	buf[1] = 0x11;
	buf[3] = 0x0;
	if(Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buf, sizeof(buf)) != 0)
	{
        TOUCH_ERR(" Mode chage knock on fail.\n");
        goto fail;
    }
	temp = 5;
	if(Lu20xx_I2C_Write(client, KNOCK_STATUS, &temp, 1) != 0)
    {
        TOUCH_ERR(" Write KNOCK_STATUS fail.\n");
        goto fail;
    }
	temp = 2;
	if(Lu20xx_I2C_Write(client, KNOCK_ON_TAP_COUNT, &temp, 1) !=0)
	{
        TOUCH_ERR(" Mode KNOCK_ON_TAP_COUNT fail.\n");
        goto fail;
    }

    temp = 0;
    if(Lu20xx_I2C_Write(client, KNOCK_ON_REPORT_DELAY, &temp, 1) !=0)
    {
        TOUCH_ERR(" Write KNOCK_ON_REPORT_DELAY fail.\n");
        goto fail;
    }
    
	SleepInLu202x(client);
    Lu202x_Set_Check(client);
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_ONLY\n");    
    return TOUCH_SUCCESS;

fail:
    SleepInLu202x(client);
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_ONLY fail.\n");    
    return TOUCH_FAIL;
}

static int Lu202x_Set_KnockCode(struct i2c_client *client, LpwgSetting  *pLpwgSetting)
{
    u8 buf[5] = {CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
    u8 temp = 0;
    
    WakeUpLu202x();
    buf[1] = 0x11;
    buf[3] = 0x0;
    if(Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buf, sizeof(buf)) != 0)
    {
        TOUCH_ERR(" Mode chage knock code on fail.\n");
        goto fail;
    }
    
    temp = 7;
    if(Lu20xx_I2C_Write(client, KNOCK_STATUS, &temp, 1) != 0)
    {
        TOUCH_ERR(" Write KNOCK_STATUS fail.\n");
        goto fail;
    }
    
    temp = 200;
    if(Lu20xx_I2C_Write(client, KNOCK_CODE_TAPOFF_TIMEOUT, &temp, 1) != 0)
    {
        TOUCH_ERR(" Write KNOCK_CODE_TAPOFF_TIMEOUT fail.\n");
        goto fail;
    }
    temp = (u8)pLpwgSetting->tapCount;
    if(Lu20xx_I2C_Write(client, KNOCK_CODE_TAP_COUNT, &temp, 1) != 0)
    {
        TOUCH_ERR(" Write KNOCK_CODE_TAP_COUNT fail.\n");
        goto fail;
    }

    SleepInLu202x(client);
    Lu202x_Set_Check(client);
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_CODE.\n");
    return TOUCH_SUCCESS;
fail :
    SleepInLu202x(client);    
    TOUCH_LOG("LPWG Mode Changed to KNOCK_ON_CODE Fail.\n");
    return TOUCH_FAIL;

}

static int Lu202x_Set_Hover(struct i2c_client *client)
{
    u8 buffer[5] = {CMD_LU202x_CHANGEMODE, 0, 0, 0, 0};
    int ret = 0;
    WakeUpLu202x();
    buffer[1] = 0x12;
    buffer[3] = 0x0;
    ret = Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buffer, sizeof(buffer));
    if( ret == TOUCH_FAIL ) 
    {
    	TOUCH_LOG("Hover on setting fail.\n");
        goto fail;
    }
    SleepInLu202x(client);
    Lu202x_Set_Check(client);
    TOUCH_LOG("Mode Changed to HOVER.\n");
    return TOUCH_SUCCESS;            
fail:
    SleepInLu202x(client);
    TOUCH_LOG("Mode Changed to HOVER fail.\n");
    return TOUCH_FAIL;
}

int Lu202x_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int result = TOUCH_SUCCESS;
    TouchDriverData *pDriverData = i2c_get_clientdata(client);
 
	TOUCH_FUNC();

	gDeviceData.deviceState = newState;

    TouchDisableIrq();

    switch(newState)
    {
        case STATE_NORMAL :
            Lu202x_Reset(client);
            break;
            
        case STATE_KNOCK_ON_ONLY :
            result = Lu202x_Set_KnockOn(client);
            break;

        case STATE_KNOCK_ON_CODE :
            result = Lu202x_Set_KnockCode(client, pLpwgSetting);
            break;

        case STATE_NORMAL_HOVER :
            result = Lu202x_Set_Hover(client);
            break;

        case STATE_HOVER :
            if(pDriverData->reportData.hover == 1)
            {
                if( pLpwgSetting->mode == 1 ) 
                {
                    result = Lu202x_Set_KnockOn(client);
                }
                if( pLpwgSetting->mode == 2 ) 
                {
                    result = Lu202x_Set_KnockCode(client, pLpwgSetting);
                }
                if( pLpwgSetting->mode == 3 ) 
                {
                    /*not support*/
                }
            }
            break;
        case STATE_OFF : 
            result = Lu202x_Set_PDN(client);
            break;
        default : 
            TOUCH_ERR("Unknown State %d", newState);
            break;
    }
    msleep(2);

    TouchEnableIrq();

	return result;
}

int print_raw(char *buf, u16 *ch_cap_data, u8 mode, int offset)
{
	int i = 0;

	offset += snprintf(buf + offset, PAGE_SIZE, "==============================================\n");
	offset += snprintf(buf + offset, PAGE_SIZE, "         Touch Panel %s value\n", \
			(mode == GET_REFERENCE) ? "Reference" : (mode == GET_JITTER) ? "Jitter" : "Delta");
	offset += snprintf(buf+offset, PAGE_SIZE, "==============================================\n");

	for (i = 0; i < MAX_CHANNEL / 2; i++) {
		offset += snprintf(buf + offset, PAGE_SIZE, "left [%02d]-[%05d] | [%02d]-[%05d] right\n",
			i, ch_cap_data[i], i+MAX_CHANNEL/2, ch_cap_data[i+MAX_CHANNEL/2]);
		}

	return offset;
}

static void lu202x_get_rawdata(struct i2c_client *client, u16 *ch_cap_data, u8 mode)
{
	u8 buf[3] = {CMD_LU202x_CHCAPTEST, 0, 0};
	u8 reply[2] = {0,};
	u8 i = 0, size = 0;
	u8 cpa_read_val[MAX_CHANNEL*2] = {0,};

	WakeUpLu202x();

	/* Check the raw cap & jitter over range. */
	/* Change Cap Test mode */
	if( Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buf, 3) == -1)
	{
	    TOUCH_ERR("Change Cap Test mode fail.\n");
	    goto fail;
	}
	SleepInLu202x(client);
	msleep(2);

	WakeUpLu202x();

	/* Cap Test Mode Change Check(Reply Type Check) */
	if(Lu20xx_I2C_Read(client, LU202x_CMDACK_ADDR, buf, 3) == -1)
	{
	    TOUCH_ERR("Read Cmd ACK fail.\n");
	    goto fail;
	}
	else
	{
	    if(buf[2] != CMD_LU202x_CHCAPTEST)
	    {
		TOUCH_ERR("Mode Change fail.\n");
		goto fail;
	    }
	}

	if(Lu20xx_I2C_Read(client, LU202x_CMDReply_ADDR, reply, 2) == -1)
	{
	    TOUCH_ERR("Reply Data Read fail.\n");
	    goto fail;
	}
	else
	{
	    if( reply[0] != LU202x_CHCAPTEST_Reply )
	    {
		TOUCH_ERR("Cap Test reply fail\n");
		goto fail;
	    }
	}
	/* raw cap data + jiter data size. - Sensor Channel * 4 +6 */
	size = reply[1] - 6;
	TOUCH_LOG("Channel Size : %d", size / 4);
	if(size > MAX_CHANNEL*4) {
		size = MAX_CHANNEL*4;
	}

	/* Read data of channel */
	if (mode == GET_REFERENCE) {
		Lu20xx_I2C_Read(client, LU202x_RAWCAP_ADDR, cpa_read_val, size / 2);
	} else if (mode == GET_JITTER) {
		Lu20xx_I2C_Read(client, LU202x_JITTERCAP_ADDR, cpa_read_val, size / 2);
	}
	SleepInLu202x(client);

	for (i = 0; i < (size / 4); i++) {
		ch_cap_data[i] = cpa_read_val[i * 2] | (cpa_read_val[i * 2 + 1] << 8);
		TOUCH_LOG("Ch[%d] Cap data = %d\n", i, ch_cap_data[i]);
	}

	return;
fail://I2c Fail
	SleepInLu202x(client);
	return;
}

static int LU202x_ChCap_Test(struct i2c_client *client,char* pBuf ,int* pRawStatus, int* pChannelStatus, int* pDataLen)
{   
    u8 buf[3] = {CMD_LU202x_CHCAPTEST, 0, 0};
    u8 reply[2] = {0,};
    u8 Raw_Cap_Value[MAX_CHANNEL * 2] = {0, };
    u8 Jitter_Value[MAX_CHANNEL * 2] = {0, };
    u8 AutoCycle[4] = {0,};
    u8 size = 0;
    u8 i = 0;
    u8 fw_status = 0;
    int dataLen = 0;

    TouchDriverData *pDriverData = i2c_get_clientdata(client);

    *pRawStatus = 0;
    *pChannelStatus = 0;
     WakeUpLu202x();

    /* Check the channel status. */
    if(Lu20xx_I2C_Read(client, FW_STATUS, &fw_status, 1) != 0 )
    {
       TOUCH_ERR("Read Firmware Status Fail. (0x0000)\n"); 
       TOUCH_ERR("There is no product data or I2C Fail.\n");
       goto fail;
    }
    else
    {
       if(fw_status != 0)
       {
           TOUCH_ERR("Firmware Status Fail(Sensor channel open fail). (0x0000)\n");
            *pChannelStatus = 1;
       }
    }

    dataLen += sprintf(pBuf + dataLen, "=====Test Start=====\n");
    dataLen += sprintf(pBuf + dataLen, "    -RawCap Value-                 -Jitter Value-    \n");
        

    TOUCH_LOG("=====Test Start=====\n");
    TOUCH_LOG("    -RawCap Value-                 -Jitter Value-    \n");

    /* Check the raw cap & jitter over range. */
    /* Change Cap Test mode */
    if( Lu20xx_I2C_Write(client, LU202x_MODE_ADDR, buf, 3) == -1)
    {
        TOUCH_ERR("Change Cap Test mode fail.\n");
        goto fail;
    }
    SleepInLu202x(client);
    msleep(2);

    WakeUpLu202x();

    /* Cap Test Mode Change Check(Reply Type Check) */
    if(Lu20xx_I2C_Read(client, LU202x_CMDACK_ADDR, buf, 3) == -1)
    {
        TOUCH_ERR("Read Cmd ACK fail.\n");
        goto fail;
    }
    else
    {
        if(buf[2] != CMD_LU202x_CHCAPTEST)
        {
            TOUCH_ERR("Mode Change fail.\n");
            goto fail;
        }
    }

    if(Lu20xx_I2C_Read(client, LU202x_CMDReply_ADDR, reply, 2) == -1)
    {
        TOUCH_ERR("Reply Data Read fail.\n");
        goto fail;
    }
    else
    {
        if( reply[0] != LU202x_CHCAPTEST_Reply )
        {
            TOUCH_ERR("Cap Test reply fail\n");
            goto fail;
        }
    }
    /* raw cap data + jiter data size. - Sensor Channel * 4 +6 */
    size = reply[1] - 6;

    /* Read Raw Cap value(Read data size = 32 * 2, channel number 32) */
    if( Lu20xx_I2C_Read(client, LU202x_RAWCAP_ADDR, Raw_Cap_Value, size / 2) == -1)
    { 
        TOUCH_ERR("Raw cap value read fail \n");
        goto fail;
    }

    if( Lu20xx_I2C_Read(client, LU202x_JITTERCAP_ADDR, Jitter_Value, size / 2)== -1)
    {
        TOUCH_ERR("Jitter value read fail \n");
        goto fail;
    }

    if( Lu20xx_I2C_Read(client, LU202x_AUTOCYCLE_ADDR, AutoCycle, sizeof(AutoCycle))== -1)
    {
        TOUCH_ERR("Auto cycle read fail \n");
        goto fail;
    }
       
    /* Print Raw Data, Jitter data, Auto cycle */
    for(i = 0; i < size/4; i++)
    {       
        TOUCH_LOG("Raw cap value [%d] = %d , Jitter value [%d] = %d\n", i,
        ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)), i , ((Jitter_Value[i * 2]) | (Jitter_Value[i*2 + 1]<< 8)) );
        dataLen += sprintf(pBuf + dataLen, "Raw cap value [%d] = %d , Jitter value [%d] = %d\n", i,
        ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)), i , ((Jitter_Value[i * 2]) | (Jitter_Value[i*2 + 1]<< 8)) );
    }

    /* Check Raw Data, Jitter data */
    for(i = 0; i < size/4; i++)
    {
        if( ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)) == 0 )//Check the channel open or short.
        {
            dataLen += sprintf(pBuf + dataLen, "!!!Error_Open/Short!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));     
            TOUCH_ERR("!!!Error_Open/Short!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));
            *pChannelStatus = 1;
        }

        if( ((Jitter_Value[i * 2]) | (Jitter_Value[i*2 + 1]<< 8)) > 120 )//Check the Jitter Value.
        {
            dataLen += sprintf(pBuf + dataLen, "!!!Error_Over Jitter Range!!! %d channel Jitter value [%d]\n", i, ((Jitter_Value[i * 2]) | (Jitter_Value[i*2 + 1]<< 8)));     
            TOUCH_ERR("!!!Error_Over Jitter Range!!! %d channel Jitter value [%d]\n", i, ((Jitter_Value[i * 2]) | (Jitter_Value[i*2 + 1]<< 8)));
           // *pRawStatus = 1;
        }

	if(pDriverData->mfts_enable) {
		if( (((Raw_Cap_Value[i *2]) | (Raw_Cap_Value[i*2 +1]<<8)) < Lu202x_mfts_LowerImageLimit[i]) | (((Raw_Cap_Value[i *2]) | (Raw_Cap_Value[i*2 +1]<<8)) > Lu202x_mfts_UpperImageLimit[i]) )
		{
		    dataLen += sprintf(pBuf + dataLen, "!!Error_ Raw cap Range!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));
		    TOUCH_ERR("!!Error_ Raw cap Range!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));
		    *pRawStatus = 1;
		}
	} else {
		if( (((Raw_Cap_Value[i *2]) | (Raw_Cap_Value[i*2 +1]<<8)) < Lu202x_LowerImageLimit[i]) | (((Raw_Cap_Value[i *2]) | (Raw_Cap_Value[i*2 +1]<<8)) > Lu202x_UpperImageLimit[i]) )
		{
		    dataLen += sprintf(pBuf + dataLen, "!!Error_ Raw cap Range!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));
		    TOUCH_ERR("!!Error_ Raw cap Range!!! %d channel Raw cap value [%d]\n", i, ((Raw_Cap_Value[i * 2]) | (Raw_Cap_Value[i*2 + 1]<< 8)));
		    *pRawStatus = 1;
		}
	}
    }

    TOUCH_LOG("View area cycle = %d , Button area cycle = %d\n", ((AutoCycle[0]) | (AutoCycle[1]<< 8)), ((AutoCycle[2]) | (AutoCycle[3]<< 8)) );
    dataLen += sprintf(pBuf + dataLen, "View area cycle = %d , Button area cycle = %d\n", ((AutoCycle[0]) | (AutoCycle[1]<< 8)), ((AutoCycle[2]) | (AutoCycle[3]<< 8)) );

    *pDataLen += dataLen;
    SleepInLu202x(client);

    return TOUCH_SUCCESS;
    fail://I2c Fail
    SleepInLu202x(client);
    return TOUCH_FAIL;
}


void Lu202x_sd_write ( char *data )
{
	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND|O_SYNC, 0644);

	if (fd >= 0)
	{
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}

	set_fs(old_fs);
}


int Lu202x_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	int dataLen = 0;

	/* CAUTION : be careful not to exceed buffer size */

	/* do implementation for self-diagnosis */
	LU202x_ChCap_Test(client, pBuf, pRawStatus, pChannelStatus, &dataLen);
	dataLen += sprintf(pBuf+dataLen,  "======== RESULT File =======\n");
	dataLen += sprintf(pBuf+dataLen, "Channel Status : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
	dataLen += sprintf(pBuf+dataLen,  "Raw Data : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");

    TOUCH_LOG("======== RESULT File =======\n");
    TOUCH_LOG("Channel Status : %s\n", (*pChannelStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");
    TOUCH_LOG("Raw Data : %s\n", (*pRawStatus == TOUCH_SUCCESS) ? "Pass" : "Fail");

    Lu202x_sd_write(pBuf);
	return TOUCH_SUCCESS;
	
}

int Lu202x_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;
	
	switch( cmd )
	{
		case READ_IC_REG:
			ret = Lu20xx_I2C_Read(client, (u16)reg, (u8 *)pValue, 1);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Lu20xx_I2C_Write(client, (u16)reg, (u8 *)*pValue, 1);
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

static void Lu202x_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch( notify )
	{
		case NOTIFY_CALL:
			{
				TOUCH_LOG("LU202X NOTIFY_CALL Not Implemented !!\n");
			}
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

int write_to_file(const char *path, int i)
{
	int fd;
	char buf[20];
	size_t count;

	TOUCH_LOG("%s : Called by Hall IC\n", __FUNCTION__);

	fd = sys_open(path, O_WRONLY, 0);

	if(fd == -1) {
		TOUCH_ERR("Write to file failed - %s\n", path);
		return -1;
	}

	sprintf(buf, "%d", i);

	count = sys_write(fd, buf, strlen(buf));

	sys_close(fd);

	return count;
}



TouchDeviceSpecificFunction Lu202x_Func = {
	.Initialize = Lu202x_Initialize,
	.Reset = Lu202x_Reset,
	.Connect = Lu202x_Connect,
	.InitRegister = Lu202x_InitRegister,
	.ClearInterrupt = Lu202x_ClearInterrupt,
	.InterruptHandler = Lu202x_InterruptHandler,
	.ReadIcFirmwareInfo = Lu202x_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = Lu202x_GetBinFirmwareInfo,
	.UpdateFirmware = Lu202x_UpdateFirmware,
	.SetLpwgMode = Lu202x_SetLpwgMode,
	.DoSelfDiagnosis = Lu202x_DoSelfDiagnosis,
	.AccessRegister = Lu202x_AccessRegister,
	.device_attribute_list = lu202x_attribute_list,
	.NotifyHandler = Lu202x_NotifyHandler,
};


/* End Of File */


