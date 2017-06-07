/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.h
 * FUNCTION     : Driver header for RPR0521, Ambient Light Sensor(ALS) IC
 * PROGRAMMED   : Sensor application development group
 * AUTHOR       : Masafumi Seike
 * REMARKS    :
 * COPYRIGHT    : Copyright (C) 2014 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/
#ifndef _ROHM_RPR0521_I2C_H_
#define _ROHM_RPR0521_I2C_H_

#include "rohm_rpr0521_i2c_if.h"

#define RPR0521_DRIVER_VER ("0.1.1")
#define CALC_ERROR         (0x80000000)
#define SM_TIME_UNIT       (1000)
#define MN_TIME_UNIT       (1000000)
#define MASK_CHAR          (0xFF)
#define CLR_LOW2BIT        (0xFC)
#define CLR_LOW4BIT        (0xF0)
#define INIT_MODE_MASK     (0xC0)
#define UNRELATEDNESS      (0xFF)
#define IRQ_NON_USE        (0)
#define IRQ_USE            (1)
#define MASK_LONG          (0xFFFFFFFF)

#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif

/* structure to read data value from sensor */
typedef struct {
    unsigned short ps_data;          /* data value of PS data from sensor        */
    unsigned short als_data0;        /* data value of ALS data0 from sensor      */
    unsigned short als_data1;        /* data value of ALS data1 from sensor      */
} READ_DATA_BUF;

/* structure to set initial value to sensor */
typedef struct {
    unsigned char  mode_ctl;         /* value of PS and ALS function             */
    unsigned char  psals_ctl;        /* value of PS and ALS control              */
    unsigned char  ps_ctl;           /* value of PS interrupt persistence        */
    unsigned char  intr;             /* interruption setting value               */
    unsigned short psth_upper;       /* threshold value of high level for PS     */
    unsigned short psth_low;         /* threshold value of low level for PS      */
    unsigned short alsth_upper;      /* threshold value of high level for ALS    */
    unsigned short alsth_low;        /* threshold value of low level for ALS     */
} INIT_ARG;

/* structure to read state value from sensor */
typedef struct {
    unsigned char als_state;         /* state value of ALS from sensor           */
    unsigned char ps_state;          /* state value of PS from sensor            */
} PWR_ST;

/* structure to activate sensor */
typedef struct {
    unsigned char power_als;         /* value of whether to start ALS or not     */
    unsigned char power_ps;          /* value of whether to start PS or not      */
    unsigned char intr;              /* value of whether to use interrupt or not */
} POWERON_ARG;

typedef struct {
    long long      data;
    long long      data0;
    long long      data1;
    unsigned char  gain_data0;
    unsigned char  gain_data1;
    unsigned long  dev_unit;
    unsigned short als_time;
    unsigned short als_data0;
    unsigned short als_data1;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;

typedef struct {
    unsigned char time;
    unsigned char gain;
    unsigned char led_current;
} DEVICE_VAL;

/************ define parameter for register ************/
/* REG_SYSTEMCONTROL(0x40) */
#define REG_SW_NOTRESET     (0 << 7)
#define REG_SW_RESET        (1 << 7)
#define REG_INT_NOTRESET    (0 << 6)
#define REG_INT_RESET       (1 << 6)

/* REG_MODECONTROL(0x41) */
#define PWRON_ALS           (7)
#define PWRON_PS            (6)
#define PWRON_ALS_EN        (1 << PWRON_ALS)
#define PWRON_PS_EN         (1 << PWRON_PS)
#define PS_PULSE_200        (0 << 5)	//LED pulse width 200
#define PS_PULSE_330        (1 << 5)	//LED pulse width 330
#define NORMAL_MODE         (0 << 4)
#define LOW_NOISE_MODE      (1 << 4)	//twice measurement mode
#define MTIME_ALSPS_0_0		(0x0)
#define MTIME_ALSPS_0_10	(0x1)
#define MTIME_ALSPS_0_40	(0x2)
#define MTIME_ALSPS_0_100	(0x3)
#define MTIME_ALSPS_0_400	(0x4)
#define MTIME_ALSPS_100_50	(0x5)
#define MTIME_ALSPS_100_100	(0x6)
#define MTIME_ALSPS_100_400	(0x7)
#define MTIME_ALSPS_400_50	(0x8)
#define MTIME_ALSPS_400_100	(0x9)
#define MTIME_ALSPS_400_0	(0xA)
#define MTIME_ALSPS_400_400	(0xB)
#define MTIME_ALSPS_50_50	(0xC)

#define MEASUREMENT_MAX     (0x0C)

/* REG_ALSPSCONTROL(0x42) */
#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)
#define ALSGAIN_X1X1        (0x0 << 2)
#define ALSGAIN_X1X2        (0x1 << 2)
#define ALSGAIN_X2X2        (0x5 << 2)
#define ALSGAIN_X64X64      (0xA << 2)
#define ALSGAIN_X128X64     (0xE << 2)
#define ALSGAIN_X128X128    (0xF << 2)
#define REG_ALSPSCTL_MAX    (0x3F)

/* REG_PERSIST(0x43) */
#define INFRARED_LEVEL_LOW		(0x0 << 6)
#define INFRARED_LEVEL_HIGH		(0x1 << 6)
#define INFRARED_LEVEL_TOOHIGH	(0x3 << 6)
#define PS_GAIN_X1				(0x0 << 4)
#define PS_GAIN_X2				(0x1 << 4)
#define PS_GAIN_X4				(0x2 << 4)
#define PERSISTENCE				(3)
#define PERSISTENCE_MAX     	(0x0F)

/* REG_INTERRUPT(0x4A) */
#define PS_INT_ENABLE		(1 << 7)
#define ALS_INT_ENABLE		(1 << 6)
#define PS_THH_ONLY         (0 << 4)
#define PS_THH_BOTH_HYS     (1 << 4)
#define PS_THH_BOTH_OUTSIDE (2 << 4)
#define POLA_ACTIVEL        (0 << 3)
#define POLA_INACTIVEL      (1 << 3)
#define OUTPUT_ANYTIME      (0 << 2)
#define OUTPUT_LATCH        (1 << 2)
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)
#define REG_INTERRUPT_MAX   (0x2F)

/* moved mode of ALS or PS  */
#define CTL_STANDBY         (0)
#define CTL_STANDALONE      (1)

/* REG_PSTH(0x4B) */
#define REG_PSTH_MAX        (0xFFF)

/* REG_PSTL(0x4D) */
#define REG_PSTL_MAX        (0xFFF)

/* REG_ALS_DATA0_TH(0x4F) */
#define REG_ALS_DATA0_TH_MAX (0xFFF)

/* REG_ALS_DATA0_TL(0x51) */
#define REG_ALS_DATA0_TL_MAX (0xFFF)


#endif /* _ROHM_RPR0521_I2C_H_ */
