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
 *    File  	: lgtp_device_mit200.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_MIT200_H_ )
#define _LGTP_DEVICE_MIT200_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define FW_VER_INFO_NUM			4
#define MAX_NUM_OF_FINGERS	      10
#define MIT_FINGER_EVENT_SZ		8
#define MIT_LPWG_EVENT_SZ		3
#define PAGE_HEADER				3
#define PAGE_DATA				1024
#define PAGE_CRC				2
#define PACKET_SIZE		(PAGE_HEADER + PAGE_DATA + PAGE_CRC)
#define KNOCKON_DELAY   			700
#define MAX_COL				15
#define MAX_ROW				25

#define TOUCH_GPIO_INTERRUPT (13+902)
#define TOUCH_GPIO_SDA 	     (18+902)
#define TOUCH_GPIO_SCL         (19+902)

/****************************************************************************
* Type Definitions
****************************************************************************/
#define MIT_REGH_CMD		0x10

#define MIT_ROW_NUM             0x0B
#define MIT_COL_NUM             0x0C
#define MIT_FW_VERSION	     0xC2
#define MIT_FW_PRODUCT	     0xE0

#define MIT_LPWG_EVENT		0xE
#define MIT_ERROR_EVENT		0xF
#define MIT_EVENT_PKT_SZ		0x0F
#define MIT_INPUT_EVENT		0x10

#define MIT_REGL_UCMD				0xA0
#define MIT_REGL_EVENT_PKT_SZ			0x0F
#define MIT_REGL_INPUT_EVENT			0x10
#define MIT_REGL_UCMD_RESULT_LENGTH	0xAE
#define MIT_REGL_UCMD_RESULT			0xAF

/* Universal commands */
#define MIT_UNIV_ENTER_TESTMODE		0x40
#define MIT_UNIV_TESTA_START			0x41
#define MIT_UNIV_GET_RAWDATA			0x44
#define MIT_UNIV_TESTB_START			0x48
#define MIT_UNIV_GET_OPENSHORT_TEST	0x50
#define MIT_UNIV_EXIT_TESTMODE		0x6F
#define MIT_UNIV_GET_READ_OTP_STATUS	0x77
#define MIT_UNIV_SEND_THERMAL_INFO		0x58

/* MIT 200 LPWG Registers */
#define MIT_LPWG_IDLE_REPORTRATE_REG     	0x60
#define MIT_LPWG_ACTIVE_REPORTRATE_REG   0x61
#define MIT_LPWG_SENSITIVITY_REG         	0x62
#define MIT_LPWG_ACTIVE_AREA_REG         	0x63

#define MIT_LPWG_TCI_ENABLE_REG          	0x70
#define MIT_LPWG_TOUCH_SLOP_REG          	0x71
#define MIT_LPWG_TAP_MIN_DISTANCE_REG    0x72
#define MIT_LPWG_TAP_MAX_DISTANCE_REG    0x73
#define MIT_LPWG_MIN_INTERTAP_REG        	0x74
#define MIT_LPWG_MAX_INTERTAP_REG        	0x76
#define MIT_LPWG_TAP_COUNT_REG           	0x78
#define MIT_LPWG_INTERRUPT_DELAY_REG     	0x79

#define MIT_LPWG_TCI_ENABLE_REG2         	0x80
#define MIT_LPWG_TOUCH_SLOP_REG2         	0x81
#define MIT_LPWG_TAP_MIN_DISTANCE_REG2   0x82
#define MIT_LPWG_TAP_MAX_DISTANCE_REG2   0x83
#define MIT_LPWG_MIN_INTERTAP_REG2       	0x84
#define MIT_LPWG_MAX_INTERTAP_REG2       	0x86
#define MIT_LPWG_TAP_COUNT_REG2          	0x88
#define MIT_LPWG_INTERRUPT_DELAY_REG2    0x89

#define MIT_LPWG_STORE_INFO_REG          	0x8F
#define MIT_LPWG_START_REG               	0x90
#define MIT_LPWG_PANEL_DEBUG_REG        	0x91
#define MIT_LPWG_FAIL_REASON_REG        	0x92

#define MIT_ERRORCODE_FAIL_REASON		0x14
#define MIT_SWIPE_FAIL_REASON			0x15

#if defined(ENABLE_SWIPE_MODE)
/* Swipe Registers */
#define MIT_LPWG_SWIPE_ENABLE_REG				0x40
#define MIT_LPWG_SWIPE_DISTANCE_REG			0x41
#define MIT_LPWG_SWIPE_RATIO_THD_REG			0x42
#define MIT_LPWG_SWIPE_RATIO_CHECK_PERIOD_REG	0x43
#define MIT_LPWG_SWIPE_MIN_TIME_THD_REG			0x44
#define MIT_LPWG_SWIPE_MAX_TIME_THD_REG			0x45
#define MIT_LPWG_SWIPE_INTERRUPT_STATUS_REG		0x46
#define MIT_LPWG_SWIPE_FAIL_REASON_REG			0x47
#define MIT_LPWG_SWIPE_TIME_REG				0x48
#endif
/****************************************************************************
* Exported Variables
****************************************************************************/
struct mit200_dev {
	u8 row_num;
	u8 col_num;
};

struct mit200_module {
	u8 product_code[24];
	u8 version[2];
	u8 otp;
};

struct mit200_limit_value {
	int	raw_data_max;
	int	raw_data_min;
	int	raw_data_margin;
	int	raw_data_otp_min;
	int	raw_data_otp_max;
	int	open_short_min;
	int	slope_max;
	int	slope_min;
};

struct melfas_ts_data {
	struct i2c_client	*client;
	struct mit200_dev	dev;
	struct mit200_module	module;
	struct mit200_limit_value	*limit;
	struct regulator 	*vdd;
	uint16_t	*mit_data[MAX_ROW];
	s16	*intensity_data[MAX_ROW];
	u8 test_mode;
	u8 lpwg_debug_enable;
	u8 lpwg_fail_reason;
#if defined(ENABLE_SWIPE_MODE)
	u8 swipe_fail_reason;
#endif
	char buf[PACKET_SIZE];
	bool	selfdiagnostic_state[3];
	int r_max;
	int r_min;
	int o_max;
	int o_min;
	int s_max;
	int s_min;
	int check_openshort;
	int count_short;

	TouchState currState;
	LpwgSetting lpwgSetting;
};

enum TCI_CTRL {
	IDLE_REPORTRATE_CTRL	= 1,
	ACTIVE_REPORTRATE_CTRL,
	SENSITIVITY_CTRL,

	TCI_ENABLE_CTRL	= 11,
	TOUCH_SLOP_CTRL,
	TAP_MIN_DISTANCE_CTRL,
	TAP_MAX_DISTANCE_CTRL,
	MIN_INTERTAP_CTRL,
	MAX_INTERTAP_CTRL,
	TAP_COUNT_CTRL,
	INTERRUPT_DELAY_CTRL,

	TCI_ENABLE_CTRL2	= 21,
	TOUCH_SLOP_CTRL2,
	TAP_MIN_DISTANCE_CTRL2,
	TAP_MAX_DISTANCE_CTRL2,
	MIN_INTERTAP_CTRL2,
	MAX_INTERTAP_CTRL2,
	TAP_COUNT_CTRL2,
	INTERRUPT_DELAY_CTRL2,

	LPWG_STORE_INFO_CTRL	= 31,
	LPWG_START_CTRL,
	LPWG_PANEL_DEBUG_CTRL,
	LPWG_FAIL_REASON_CTRL,

#if defined(ENABLE_SWIPE_MODE)
	LPWG_SWIPE_ENABLE_CTRL	= 35,
	LPWG_SWIPE_DISTANCE_CTRL,
	LPWG_SWIPE_RATIO_THD_CTRL,
	LPWG_SWIPE_RATIO_CHECK_PERIOD_CTRL,
	LPWG_SWIPE_MIN_TIME_THD_CTRL,
	LPWG_SWIPE_MAX_TIME_THD_CTRL,
	LPWG_SWIPE_FAIL_REASON_CTRL,
#endif
};

enum {
	SD_RAWDATA = 0,
	SD_OPENSHORT,
	SD_SLOPE,
};

enum {
	RAW_DATA_SHOW = 0,
	RAW_DATA_STORE,
	OPENSHORT,
	OPENSHORT_STORE,
	SLOPE,
	CRACK_CHECK,
};

enum  {
	OTP_NOT_SUPPORTED = 0,
	OTP_NONE,
	OTP_APPLIED,
};

enum {
	FAIL_MULTI_TOUCH = 1,
	FAIL_TOUCH_SLOP,
	FAIL_TAP_DISTANCE,
	FAIL_TAP_TIME,
	FAIL_TOTAL_COUNT,
	FAIL_DELAY_TIME,
	FAIL_PALM,
	FAIL_ACTIVE_AREA
};

#if defined(ENABLE_SWIPE_MODE)
enum {
	FAIL_FINGER = 1,
	FAIL_MULTI_FINGER,
	FAIL_FASTER_MIN_TIME,
	FAIL_EXCEED_MAX_TIME,
	FAIL_DIRECTION_UP,
	FAIL_EXCEED_RATIO_THD,
	FAIL_RESERVED
};

enum {
	SWIPE_RESERVED = 0,
	SWIPE_ENABLE,
};
#endif

/****************************************************************************
* Macros
****************************************************************************/


/****************************************************************************
* Global Function Prototypes
****************************************************************************/
extern int MIT_FirmwareUpgrade(struct melfas_ts_data *ts, const char* fw_path);
ssize_t Mit200_Get_SelfD_Result(struct melfas_ts_data *ts, char *pBuf, int type);
ssize_t Mit200_Delta_Show(struct melfas_ts_data *ts, char *buf);

#endif /* _LGTP_DEVICE_MIT200_H_ */

/* End Of File */

