/*  apds9960.h - Linux kernel modules for Gesture + RGB + ambient light + proximity sensor
 *
 *  Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2012 Avago Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/************************************************
Change history

Ver		When			Who		Why
---		----			---		---
1.0.0	19-Aug-2013		KK		Initial draft
1.0.1	26-Aug-2013		KK		Revise gesture algorithm
1.0.2	29-Aug-2013		KK		Change GTHR_IN and GTHR_OUT
1.0.3	03-Sep-2013		KK		Correct divide by zero error in AveragingRawData()
1.0.4	05-Sep-2013		KK		Accept old and latest ID value
1.0.5	17-Sep-2013		KK		Return if sample size is less than or equal to 4 in GestureDataProcessing();
								Correct error in AveragingRawData()
1.0.6	27-Sep-2013		KK		Simplify GestureDataProcessing() and revise Gesture Calibration
								Added Up/Down/Left/Right U-Turn gesture detection
************************************************/
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define	KK_DEBUG			1	// for debug use only
#define LINUX_KERNEL_2_6_X	1
#define APDS9960_DRV_NAME	"apds9960"
#define DRIVER_VERSION		"1.0.1"

#define APDS_IOCTL_PS_ENABLE		1
#define APDS_IOCTL_PS_GET_ENABLE	2
#define APDS_IOCTL_PS_POLL_DELAY	3
#define APDS_IOCTL_ALS_ENABLE		4
#define APDS_IOCTL_ALS_GET_ENABLE	5
#define APDS_IOCTL_ALS_POLL_DELAY	6
#define APDS_IOCTL_PS_GET_PDATA		7	// pdata
#define APDS_IOCTL_ALS_GET_CH0DATA	8	// ch0data
#define APDS_IOCTL_ALS_GET_CH1DATA	9	// ch1data
#define APDS_IOCTL_ALS_GET_CDATA	10	// cdata
#define APDS_IOCTL_ALS_GET_RDATA	11	// rdata
#define APDS_IOCTL_ALS_GET_GDATA	12	// gdata
#define APDS_IOCTL_ALS_GET_BDATA	13	// bdata
#define APDS_IOCTL_GESTURE_ENABLE	14
#define APDS_IOCTL_GESTURE_GET_ENABLE	15

#define APDS_DISABLE_PS				0
#define APDS_ENABLE_PS				1
#define APDS_DISABLE_ALS			0
#define APDS_ENABLE_ALS_NO_INT	    1
#define APDS_ENABLE_ALS_WITH_INT	2
#define APDS_DISABLE_GESTURE		0
#define APDS_ENABLE_GESTURE			1

#define APDS_ALS_POLL_SLOW			0	// 1 Hz (1s)
#define APDS_ALS_POLL_MEDIUM		1	// 10 Hz (100ms)
#define APDS_ALS_POLL_FAST			2	// 20 Hz (50ms)
#define APDS_ALS_CALIBRATION		0
#define APDS_PS_CALIBRATION			1
#define APDS_PS_GESTURE_CALIBRATION	2
#define	APDS9960_ENABLE_REG			0x80
#define	APDS9960_ATIME_REG			0x81
#define	APDS9960_WTIME_REG			0x83
#define	APDS9960_AILTL_REG			0x84
#define	APDS9960_AILTH_REG			0x85
#define	APDS9960_AIHTL_REG			0x86
#define	APDS9960_AIHTH_REG			0x87
#define	APDS9960_PITLO_REG			0x89
#define	APDS9960_PITHI_REG			0x8B
#define	APDS9960_PERS_REG			0x8C
#define	APDS9960_CONFIG_REG			0x8D
#define	APDS9960_PPULSE_REG			0x8E
#define	APDS9960_CONTROL_REG		0x8F
#define	APDS9960_AUX_REG			0x90
#define	APDS9960_REV_REG			0x91
#define	APDS9960_ID_REG				0x92
#define	APDS9960_STATUS_REG			0x93
#define	APDS9960_CDATAL_REG			0x94
#define	APDS9960_CDATAH_REG			0x95
#define	APDS9960_RDATAL_REG			0x96
#define	APDS9960_RDATAH_REG			0x97
#define	APDS9960_GDATAL_REG			0x98
#define	APDS9960_GDATAH_REG			0x99
#define	APDS9960_BDATAL_REG			0x9A
#define	APDS9960_BDATAH_REG			0x9B
#define	APDS9960_PDATA_REG			0x9C
#define	APDS9960_POFFSET_UR_REG		0x9D
#define	APDS9960_POFFSET_DL_REG		0x9E
#define	APDS9960_CONFIG2_REG		0x9F
#define	APDS9960_GTHR_IN_REG		0xA0
#define	APDS9960_GTHR_OUT_REG		0xA1
#define	APDS9960_GCONF1_REG			0xA2
#define	APDS9960_GCONF2_REG			0xA3
#define	APDS9960_GOFFSET_U_REG		0xA4
#define	APDS9960_GOFFSET_D_REG		0xA5
#define	APDS9960_GPULSE_REG			0xA6
#define	APDS9960_GOFFSET_L_REG		0xA7
#define	APDS9960_GOFFSET_R_REG		0xA9
#define	APDS9960_GCONF3_REG			0xAA
#define	APDS9960_GCTRL_REG			0xAB
#define	APDS9960_GFIFO_LVL_REG		0xAE
#define	APDS9960_GSTATUS_REG		0xAF
#define	APDS9960_TEST2_REG			0xC3	// use for soft_reset
#define	APDS9960_GFIFO0_REG			0xFC	// U
#define	APDS9960_GFIFO1_REG			0xFD	// D
#define	APDS9960_GFIFO2_REG			0xFE	// L
#define	APDS9960_GFIFO3_REG			0xFF	// R
#define APDS9960_PWR_DOWN			0x00    /* PON = 0 */
#define APDS9960_PWR_ON				0x01    /* PON = 1 */
#define APDS9960_ALS_ENABLE			0x02    /* AEN */
#define APDS9960_PS_ENABLE			0x04    /* PEN */
#define APDS9960_WAIT_ENABLE		0x08    /* WEN */
#define APDS9960_ALS_INT_ENABLE		0x10    /* AIEN */
#define APDS9960_PS_INT_ENABLE		0x20    /* PIEN */
#define APDS9960_GESTURE_ENABLE		0x40    /* GEN */
#define	APDS9960_PDRIVE_100mA	0x00
#define	APDS9960_PDRIVE_50mA	0x40
#define	APDS9960_PDRIVE_25mA	0x80
#define	APDS9960_PDRIVE_12_5mA	0xC0
#define	APDS9960_PGAIN_1X		0x00
#define	APDS9960_PGAIN_2X		0x04
#define	APDS9960_PGAIN_4X		0x08
#define	APDS9960_PGAIN_8X		0x0C
#define	APDS9960_AGAIN_1X		0x00
#define	APDS9960_AGAIN_4X		0x01
#define	APDS9960_AGAIN_16X		0x02
#define	APDS9960_AGAIN_64X		0x03
#define	APDS9960_GGAIN_1X		0x00
#define	APDS9960_GGAIN_2X		0x20
#define	APDS9960_GGAIN_4X		0x40
#define	APDS9960_GGAIN_8X		0x60
#define CMD_FORCE_INT		0xE4
#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_ALL_INT		0xE7
#define APDS9960_PULSE_LEN_4US	0x00
#define APDS9960_PULSE_LEN_8US	0x40
#define APDS9960_PULSE_LEN_16US	0x80
#define APDS9960_PULSE_LEN_32US	0xC0
#define	APDS9960_LED_BOOST_100	0x00
#define	APDS9960_LED_BOOST_150	0x10
#define	APDS9960_LED_BOOST_200	0x20
#define	APDS9960_LED_BOOST_300	0x30
#define APDS9960_100MS_ADC_TIME	0xD6  /* 100.8ms integration time */
#define APDS9960_50MS_ADC_TIME	0xEB  /* 50.4ms integration time */
#define APDS9960_27MS_ADC_TIME	0xF6  /* 24ms integration time */
#define APDS9960_PPERS_0	0x00  /* Every proximity ADC cycle */
#define APDS9960_PPERS_1	0x10  /* 1 consecutive proximity value out of range */
#define APDS9960_PPERS_2	0x20  /* 2 consecutive proximity value out of range */
#define APDS9960_PPERS_3	0x30  /* 3 consecutive proximity value out of range */
#define APDS9960_PPERS_4	0x40  /* 4 consecutive proximity value out of range */
#define APDS9960_PPERS_5	0x50  /* 5 consecutive proximity value out of range */
#define APDS9960_PPERS_6	0x60  /* 6 consecutive proximity value out of range */
#define APDS9960_PPERS_7	0x70  /* 7 consecutive proximity value out of range */
#define APDS9960_PPERS_8	0x80  /* 8 consecutive proximity value out of range */
#define APDS9960_PPERS_9	0x90  /* 9 consecutive proximity value out of range */
#define APDS9960_PPERS_10	0xA0  /* 10 consecutive proximity value out of range */
#define APDS9960_PPERS_11	0xB0  /* 11 consecutive proximity value out of range */
#define APDS9960_PPERS_12	0xC0  /* 12 consecutive proximity value out of range */
#define APDS9960_PPERS_13	0xD0  /* 13 consecutive proximity value out of range */
#define APDS9960_PPERS_14	0xE0  /* 14 consecutive proximity value out of range */
#define APDS9960_PPERS_15	0xF0  /* 15 consecutive proximity value out of range */
#define APDS9960_APERS_0	0x00  /* Every ADC cycle */
#define APDS9960_APERS_1	0x01  /* 1 consecutive als value out of range */
#define APDS9960_APERS_2	0x02  /* 2 consecutive als value out of range */
#define APDS9960_APERS_3	0x03  /* 3 consecutive als value out of range */
#define APDS9960_APERS_5	0x04  /* 5 consecutive als value out of range */
#define APDS9960_APERS_10	0x05  /* 10 consecutive als value out of range */
#define APDS9960_APERS_15	0x06  /* 15 consecutive als value out of range */
#define APDS9960_APERS_20	0x07  /* 20 consecutive als value out of range */
#define APDS9960_APERS_25	0x08  /* 25 consecutive als value out of range */
#define APDS9960_APERS_30	0x09  /* 30 consecutive als value out of range */
#define APDS9960_APERS_35	0x0A  /* 35 consecutive als value out of range */
#define APDS9960_APERS_40	0x0B  /* 40 consecutive als value out of range */
#define APDS9960_APERS_45	0x0C  /* 45 consecutive als value out of range */
#define APDS9960_APERS_50	0x0D  /* 50 consecutive als value out of range */
#define APDS9960_APERS_55	0x0E  /* 55 consecutive als value out of range */
#define APDS9960_APERS_60	0x0F  /* 60 consecutive als value out of range */
#define	APDS9960_STATUS_ASAT	0x80	/* ALS Saturation */
#define APDS9960_STATUS_PSAT	0x40	/* PS Saturation - analog saturated, not a proximity detection */
#define	APDS9960_STATUS_PINT	0x20	/* PS Interrupt status */
#define	APDS9960_STATUS_AINT	0x10	/* ALS Interrupt status */
#define APDS9960_STATUS_IINT	0x08	/* Irbeam Interrupt status */
#define	APDS9960_STATUS_GINT	0x04	/* Gesture Interrupt status */
#define	APDS9960_STATUS_PVALID	0x02	/* PS data valid status */
#define	APDS9960_STATUS_AVALID	0x01	/* ALS data valid status */
#define	APDS9960_GFIFO_CLR	0x04	/* Clear all gesture status: GFIFO, GINT, GVALID, GFIFO_OV, GFIFO_LVL */
#define APDS9960_GIEN		0x02	/* Gesture interrupt enable */
#define APDS9960_GMODE		0x01	/* GMODE */
#define	APDS9960_GFIFO_OV	0x02	/* Fifo overflow */
#define APDS9960_GVALID		0x01	/* Fifo valid status */
#define	APDS9960_GFIFO_1_LEVEL	0x00
#define	APDS9960_GFIFO_4_LEVEL	0x40
#define	APDS9960_GFIFO_8_LEVEL	0x80
#define	APDS9960_GFIFO_16_LEVEL	0xC0
#define	APDS9960_GTIME_0MS		0x00
#define	APDS9960_GTIME_2_8MS	0x01
#define	APDS9960_GTIME_5_6MS	0x02
#define	APDS9960_GTIME_8_4MS	0x03
#define	APDS9960_GTIME_14MS		0x04
#define	APDS9960_GTIME_22_4MS	0x05
#define	APDS9960_GTIME_30_8MS	0x06
#define	APDS9960_GTIME_39_2MS	0x07
#define APDS9960_GDRIVE_100MA	0x00
#define	APDS9960_GDRIVE_50MA	0x08
#define	APDS9960_GDRIVE_25MA	0x10
#define APDS9960_GDRIVE_12_5MA	0x18

#define APDS9960_MAX_GESTURE_SAMPLES	32	// total FIFO size

#define APDS9960_LUX_GA1	417 //5000  /* no cover glass - for Fluorescent Daylight 6500K */
#define APDS9960_LUX_GA2	105 //5000   /* no cover glass - for Incandescent 2600K */
#define APDS9960_LUX_GA3	484 //1000   /* no cover glass - for Fluorescent Warm White 2700K */
#define APDS9960_CCT_GA1	90  /* no cover glass - for Fluorescent Daylight 6500K */
#define APDS9960_CCT_GA2	56  //1000  /* no cover glass - for Incandescent 2600K */
#define APDS9960_CCT_GA3	87  //1000  /* no cover glass - for Fluorescent Warm White 2700K */

#define	APDS9960_PPULSE_FOR_PS		5
#define APDS9960_PPULSE_LEN_FOR_PS	APDS9960_PULSE_LEN_8US
#define APDS9960_PDRVIE_FOR_PS		APDS9960_PDRIVE_100mA
#define APDS9960_PGAIN_FOR_PS		APDS9960_PGAIN_2X

#define	APDS9960_PPULSE_FOR_GESTURE		16
#define APDS9960_PPULSE_LEN_FOR_GESTURE	APDS9960_PULSE_LEN_8US
#define APDS9960_PDRVIE_FOR_GESTURE		APDS9960_PDRIVE_100mA
#define APDS9960_PGAIN_FOR_GESTURE		APDS9960_PGAIN_4X

#define APDS9960_GPULSE		    16
#define APDS9960_GPULSE_LEN	    APDS9960_PULSE_LEN_8US
#define	APDS9960_GGAIN		    APDS9960_GGAIN_4X
#define APDS9960_GTIME		    APDS9960_GTIME_0MS
#define APDS9960_GDRIVE		    APDS9960_GDRIVE_100MA
#define APDS9960_GESTURE_FIFO	APDS9960_GFIFO_4_LEVEL

#define	APDS9960_PS_LED_BOOST	    APDS9960_LED_BOOST_100
#define	APDS9960_GESTURE_LED_BOOST	APDS9960_LED_BOOST_150

#define APDS9960_PS_DETECTION_THRESHOLD	    150
#define APDS9960_PS_HSYTERESIS_THRESHOLD	100

#define	APDS9960_NEAR_THRESHOLD_LOW	0
#define APDS9960_FAR_THRESHOLD_HIGH	255

#define APDS9960_ALS_CALIBRATED_LUX	300
#define APDS9960_ALS_CALIBRATED_CCT	5000

#define APDS9960_PS_CALIBRATED_XTALK_BASELINE	10
#define APDS9960_PS_CALIBRATED_XTALK			20

#define APDS9960_ALS_THRESHOLD_HSYTERESIS	    20	/* 20 = 20% */

#define GESTURE_SENSITIVITY_LEVEL1	10
#define GESTURE_SENSITIVITY_LEVEL2	10

#define	GESTURE_GTHR_IN	    50
#define GESTURE_GTHR_OUT	40
#define GESTURE_THRESHOLD_OUT	(GESTURE_GTHR_OUT-10)

#define SENSOR_DEFAULT_DELAY	( 200 )		/* 10 ms unit=1/10ms(100us) */
#define SENSOR_MIN_DELAY		(  30 )		/*  3 ms unit=1/10ms(100us) */
#define SENSOR_MAX_DELAY		( 200 )		/* 20 ms unit=1/10ms(100us) */

typedef enum 
{
    APDS9960_ALS_RES_24MS  = 0,    /* 24ms integration time */ 
    APDS9960_ALS_RES_50MS  = 1,    /* 50ms integration time */
    APDS9960_ALS_RES_100MS = 2     /* 100ms integration time */
} apds9960_als_res_e;

typedef enum 
{
    APDS9960_ALS_GAIN_1X    = 0,    /* 1x AGAIN */ 
    APDS9960_ALS_GAIN_4X    = 1,    /* 4x AGAIN */
    APDS9960_ALS_GAIN_16X   = 2,    /* 16x AGAIN */
    APDS9960_ALS_GAIN_64X   = 3     /* 64x AGAIN */
} apds9960_als_gain_e;

enum 
{
	DIR_NONE,
	DIR_DOWN,
	DIR_UP,
	DIR_LEFT,
	DIR_RIGHT,
	DIR_UP_RIGHT,
	DIR_RIGHT_DOWN,
	DIR_DOWN_LEFT,
	DIR_LEFT_UP,
	DIR_NEAR,
	DIR_FAR,
	DIR_CIRCLE_CW,
	DIR_CIRCLE_ACW,
	DIR_UP_U_TURN,
	DIR_DOWN_U_TURN,
	DIR_LEFT_U_TURN,
	DIR_RIGHT_U_TURN,
	DIR_ALL
};

enum 
{
	NA_STATE,
	UD_FLAT_STATE,
	UD_POSITIVE_STATE,
	UD_NEGATIVE_STATE,
	LR_FLAT_STATE,
	LR_POSITIVE_STATE,
	LR_NEGATIVE_STATE,
	NEAR_STATE,
	FAR_STATE,
	CIRCLE_CW_STATE,
	CIRCLE_ACW_STATE,
	ALL_STATE
};

enum 
{
	CIRCLE_NA,
	CIRCLE_0,
	CIRCLE_1,
	CIRCLE_2,
	CIRCLE_3,
	CIRCLE_4,
	CIRCLE_5,
	CIRCLE_6,
	CIRCLE_7,
	CIRCLE_ALL
};

enum 
{
	GESTURE_ZONE_NA, // init
	GESTURE_ZONE_1,	// -ve UD, +ve LR
	GESTURE_ZONE_2,	// +ve UD, +ve LR
	GESTURE_ZONE_3,	// +ve UD, -ve LR
	GESTURE_ZONE_4, // -ve UD, -ve LR
	GESTURE_ZONE_UNKNOWN, // unknown
	GESTURE_ALL_ZONE
};

typedef struct gesture_data_type
{
	int u_data[32];
	int d_data[32];
	int l_data[32];
	int r_data[32];
	int index;
	int total_gestures;
	int in_threshold;
	int out_threshold;
}gesture_data_type;

struct alsps_cal
{
	unsigned int l_cal;
	unsigned int p_cal_ur;
	unsigned int p_cal_dl;
	unsigned int g_cal_u;
	unsigned int g_cal_d;
	unsigned int g_cal_l;
	unsigned int g_cal_r;
	unsigned int g_cal_ur;
	unsigned int g_cal_dl;
};

//tmg399x data
enum 
{
    TMG399X_ALS_GAIN_MASK = (3 << 0),
	TMG399X_PRX_GAIN_MASK = (3 << 2),
	TMG399X_LDRIVE_MASK   = (3 << 6),
	TMG399X_ALS_AGL_MASK  = (1 << 2),
    
	TMG399X_ALS_AGL_BOOST = 2,
	
    TMG399X_ATIME_PER_100 = 278,
	
    SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	
    MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 5,
	
    GAIN_SWITCH_LEVEL = 100,
};

static uint8_t const als_gains[] = 
{
	1,
	4,
	16,
	64
};

struct lux_segment 
{
	int d_factor;
	int r_coef;
	int g_coef;
	int b_coef;
	int ct_coef;
	int ct_offset;
};

//here ams_support is for IC light calculate compatible
//0:both method,1:only tmg399x method
#if defined(PD1420L) || defined(PD1420LG4) || defined(PD1420F) || defined(PD1420V)|| defined(PD1420F_EX)
static int ams_support=1;
#define D_Factor1   4007
#define R_Coef1     -150
#define G_Coef1     1000
#define B_Coef1     2000
#define CT_Coef1    2040
#define CT_Offset1  1945
#elif defined(PD1408L) || defined(PD1408LG4) || defined(PD1408F) || defined(PD1408F_EX) || defined(PD1408V) || defined(PD1421V)
static int ams_support=1;
#define D_Factor1	3360   //2183
#define R_Coef1		635     //588
#define G_Coef1		1000   //1000
#define B_Coef1		308   //404
#define CT_Coef1	      10484  //7393
#define CT_Offset1	-3218  //262
#else
static int ams_support=0;
#define D_Factor1	3360   //2183
#define R_Coef1		635     //588
#define G_Coef1		1000   //1000
#define B_Coef1		308   //404
#define CT_Coef1	      10484  //7393
#define CT_Offset1	-3218  //262
#endif

struct apds9960_data {
    struct i2c_client *client;
    struct delayed_work	dwork;		/* for PS interrupt */	
	struct delayed_work	als_dwork;	/* for ALS polling */	
	struct input_dev *input_dev_als;	
	struct input_dev *input_dev_ps; 
    struct input_dev *input_dev_gs;
    struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
    struct sensors_classdev gs_cdev;
    struct workqueue_struct *eint_queue;
	struct work_struct	eint_work; 

    //for report poll
    struct workqueue_struct *poll_queue;
    struct delayed_work		 ps_report_work;
    struct delayed_work		 als_report_work;

    int gpio_int;
    int irq;

    struct alsps_cal cal;	
	int suspended;	
	unsigned int enable_suspended_value;	/* suspend_resume usage */	
	unsigned long enable;
    
    //regsister
	unsigned int atime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppulse;
	unsigned int control;
	unsigned int aux;
	unsigned int poffset_ur;
	unsigned int poffset_dl;
	unsigned int config2;
	unsigned int gthr_in;
	unsigned int gthr_out;
	unsigned int gconf1;
	unsigned int gconf2;
	unsigned int goffset_u;
	unsigned int goffset_d;
	unsigned int gpulse;
	unsigned int goffset_l;
	unsigned int goffset_r;
	unsigned int gconf3;
	unsigned int gctrl;
	unsigned int cdata;
	unsigned int rdata;
	unsigned int gdata;
	unsigned int bdata;
	unsigned int pdata;
	
    //ps parameters
    atomic_t ps_in_threshold;
    atomic_t ps_out_threshold;
    atomic_t ps_status;                //current prox state,  1: far away    0: close
    atomic_t ps_base_value;            //the base value is from framework
    atomic_t sys_prox_status;          //prox state in framework
    atomic_t ps_first_int;
    atomic_t ps_data_ready;

	unsigned int ps_data;
	unsigned int ps_ppulse;
	unsigned int ps_poffset_ur;
	unsigned int ps_poffset_dl;
    
    unsigned int enable_ps_sensor;
    unsigned int eng_ps_sensor;
    atomic_t enabling_ps;
    
	atomic_t ps_poll_delay;
    
    //gesture parameters
	unsigned int gesture_ppulse;
	unsigned int gesture_poffset_ur;
	unsigned int gesture_poffset_dl;
	unsigned int gesture_goffset_u;
	unsigned int gesture_goffset_d;
	unsigned int gesture_goffset_l;
	unsigned int gesture_goffset_r;

    atomic_t gesture_calibration;
    atomic_t gesture_start_flag;

    unsigned int gesture_int_cont;
    unsigned int gesture_cont_int_cont;
    unsigned int gesture_ps_excp_cont;
    atomic_t gesture_exit_flag;

    unsigned int enable_gesture_sensor;
    unsigned int eng_gesture_sensor;
    atomic_t enabling_gesture;

    atomic_t gs_poll_delay;

    //als parameters
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;			/* to store ALS data */
	int als_prev_lux;				/* to store previous lux value */
	unsigned int als_gain;		    /* needed for Lux calculation */
	//unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	unsigned int als_atime_index;	/* storage for als integratiion time */
	unsigned int als_again_index;	/* storage for als GAIN */
	unsigned int cct;		        /* color temperature */
	unsigned int lux;		        /* lux */ 
	int  RGB_COE_X[3];
	int  RGB_COE_Y[3];
	int  RGB_COE_Z[3];
	unsigned int lux_GA1;
	unsigned int lux_GA2;
	unsigned int lux_GA3;
	unsigned int cct_GA1;
	unsigned int cct_GA2;
	unsigned int cct_GA3;
    //parameters for tmg399x
    unsigned int saturation;
    unsigned int cpl; 
    int ir;
    struct lux_segment segment; 

    unsigned int enable_als_sensor;
    unsigned int eng_als_sensor;

    atomic_t als_poll_delay;

	struct workqueue_struct 	*gs_workqueue;
	struct delayed_work			gs_report_work;
    struct delayed_work			gs_resume_work;
    struct delayed_work         gs_ps_monitor;
    struct delayed_work         gs_resume_delay;
	struct hrtimer				gs_polling_timer;
	struct hrtimer                   ps_timer;
	ktime_t                            ps_delay;
	int							gs_delay;
	ktime_t						gs_polling_delay;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif
};

const int DIRECTION_RIGHT 			= 0x8;
const int DIRECTION_LEFT 			= 0x4;
const int DIRECTION_TOP 			= 0x2;
const int DIRECTION_BOTTOM 			= 0x1;

#if defined(PD1408L) || defined(PD1408LG4) || defined(PD1408F) || defined(PD1408F_EX)||defined(PD1408V) || defined(PD1421V)
#define ROTATE_180_DEGREE 1
#else
#define ROTATE_180_DEGREE 0
#endif
