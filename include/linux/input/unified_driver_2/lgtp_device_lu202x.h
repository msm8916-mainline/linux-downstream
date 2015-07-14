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
 *    File  	: lgtp_device_lu202x.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description : 
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_LU202X_H_ )
#define _LGTP_DEVICE_LU202X_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/

/* Firmware File Path */
#define FW_FILE_PATH "/mnt/sdcard/lu202x_fw.bin"
#define SELF_DIAGNOSTIC_FILE_PATH "/mnt/sdcard/touch_self_test.txt"

/* LeadingUI Firmware */
#define FW_SIZE 		30*1024
#define CFG_SIZE 		1*1024
#define FAC_SIZE		1*1024

#define FAC_POS			0xFC00
#define FW_POS			0x8000

enum {
	GET_REFERENCE = 0,
	GET_JITTER,
};

enum fw_region {
	FACTORY_REGION = 0,
	FIRMWARE_REGION,
};

enum {
	EMPTY_DATA = 0,
	EXIST_DATA,
};

struct lu202x_fac_data {
	u8 act_cycle[2];
	u8 btn_cycle[2];
	u8 vendor;
	u8 version;
	u8 date[6];
	u8 cap_result;
	u8 pos_result;
	u8 reserve[50];
};

#endif /* _LGTP_DEVICE_LU202X_H_ */
/* End Of File */

