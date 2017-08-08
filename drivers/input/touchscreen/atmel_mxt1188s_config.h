
/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/input/atmel_mxt1188S.h>


/* F/W 2.0 Object List
*
*  T5, T6, T7, T8, T9,
*  T15, T18, T19,
*  T24, T25, T27
*  T37, T38,
*  T40, T42, T43, T44, T46, T47,
*  T55, T56, T57,
*  T61, T62, T63, T65, T66, T68,
*  T70, T71, T73, T77, T79
*
*/

static u8 t7_config_s[] = { MXT_GEN_POWER_T7,
	0x20, 0x09, 0x05, 0x43,
#ifdef USE_FW_11AA
	0x00
#endif
};

static u8 t8_config_s[] = { MXT_GEN_ACQUIRE_T8,
	0x4E, 0x00, 0x03, 0x01, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00,
#ifdef USE_FW_11AA
	0x00, 0x00, 0x00, 0x00, 0x00
#endif
};

static u8 t9_config_s[] = { MXT_TOUCH_MULTI_T9,
#ifdef USE_FW_11AA
	0x8B, 0x00, 0x00, 0x1B, 0x2C, 0x00, 0x9B, 0x32, 0x02, 0x04,
#else
	0x83, 0x00, 0x00, 0x1B, 0x2C, 0x00, 0x9B, 0x32, 0x02, 0x04,
#endif
	0x05, 0x08, 0x07, 0x31, 0x0A, 0x0A, 0x1E, 0x0A, 0xFF, 0x0F,
	0xFF, 0x0F, 0x10, 0x10, 0x00, 0x00, 0x8A, 0x10, 0x00, 0x00,
	0x32, 0x0A, 0x2A, 0x2A, 0x00, 0x96, 0x05, 0xC1, 0x0F, 0x6E,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x00
};

static u8 t15_config_s[] = { MXT_TOUCH_KEYARRAY_T15,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0
};

static u8 t18_config_s[] = { MXT_SPT_COMMSCONFIG_T18,
	0, 0
};

static u8 t19_config_s[] = { MXT_SPT_GPIOPWM_T19,
	0, 0, 0, 0, 0, 0
};

static u8 t24_config_s[] = { MXT_PROCI_ONETOUCH_T24,
	0x00, 0x01, 0x08, 0x00, 0x08, 0x7F, 0, 0, 0, 0,
	0, 0, 0, 0, 0, /*0x96*/0x90, 0x01, 0, 0
};

static u8 t25_config_s[] = { MXT_SPT_SELFTEST_T25,
	0x03, 0, 0x9C, 0x63, 0x14, 0x50, 0, 0, 0, 0,
	0, 0, 0, 0, 0xC8, 0x88, 0x13, 0, 0, 0,
	0
};
/*
static u8 t27_config_s[] = { MXT_PROCI_TWOTOUCH_T27,
	0, 0, 0, 0, 0, 0, 0
};
*/
static u8 t37_config_s[] = { MXT_DEBUG_DIAGNOSTIC_T37,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00
};

static u8 t38_config_s[] = { MXT_SPT_USERDATA_T38,
/*SET 1*/0x01, 0x23, 0x05, 0xE2, /*SET 2*/0xFF, 0x01, 0x0A, 0x32, /*SET 3*/0xFF, 0x01,
	0x00, 0x00, /*TIMER 0*/0x03, 0x01, 0x00, 0xE8, 0x03, /*TIMER 1*/0x03, 0x01, 0x00,
	0x40, 0x1F, /*COIN*/0x08, 0x05, 0x1E, 0x00, 0x00, 0x00, /*Abnormal Cal*/0x08, 0x19,
	0x06, 0x06, 0x05, /*Last Antitouch*/0x03, 0x0A, 0x19, 0x32, /*BLANK*/0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static u8 t40_config_s[] = { MXT_PROCI_GRIP_T40,
	0x00, 0x1E, 0x1E, 0x28, 0x28
};

static u8 t42_config_s[] = { MXT_PROCI_TOUCHSUPPRESSION_T42,
	0x37, 0x00, 0x46, 0x50, 0xB4, 0x00, 0x00, 0xC8, 0x03, 0x01,
	0x00, 0x04, 0x00
};

static u8 t43_config_s[] = { MXT_SPT_DIGITIZER_T43,
	0x88, 0x00, 0x7D, 0x00, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00
};

static u8 t46_config_s[] = { MXT_SPT_CTECONFIG_T46,
	0x04, 0x00, 0x08, 0x18, 0x00, 0x00, 0x01, 0x00, 0x00, 0x0F,
	0x0A
};

static u8 t47_config_s[] = { MXT_PROCI_STYLUS_T47,
	0x49, 0x14, 0x50, 0xFF, 0x02, 0x14, 0x14, 0xB4, 0x01, 0x08,
	0xEC, 0xEC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x94, 0x23, 0x00, 0x00, 0x00, 0x00
};

static u8 t55_config_s[] = { MXT_PROCI_ADAPTIVETHRESHOLD_T55,
	0, 0, 0, 0, 0, 0, 0
};

static u8 t56_config_s[] = { MXT_PROCI_SHIELDLESS_T56,
	0x03, 0x00, 0x01, 0x29, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0D,
	0x0E, 0x0E, 0x0D, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0F,
	0x0F, 0x0F, 0x0F, 0x0F, 0x0E, 0x0F, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
};

static u8 t57_config_s[] = { MXT_PROCI_EXTRATOUCHSCREENDATA_T57,
	0xE3, 0x32, 0x01
};

static u8 t61_config_s[] = { MXT_SPT_TIMER_T61,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
};

static u8 t62_config_s[] = { MXT_PROCG_NOISESUPPRESSION_T62,
	0x01, 0x28, 0x50, 0x06, 0x0B, 0x19, 0x20, 0x00, 0x32, 0x00,
	0x23, 0x2B, 0x1E, 0x0C, 0x05, 0x32, 0x0A, 0x05, 0x05, 0xAA,
	0x23, 0x0F, 0x18, 0x0A, 0x64, 0x0C, 0x08, 0x06, 0x3F, 0x00,
	0x23, 0x2B, 0x1E, 0x0C, 0x82, 0x46, 0x02, 0x05, 0x03, 0x00,
	0x0A, 0x0A, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x80,
	0x00, 0x28, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static u8 t63_config_s[] = { MXT_PROCI_ACTIVE_STYLUS_T63,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00
};

static u8 t65_config_s[] = { MXT_PROCI_LENSBENDING_T65,
	0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static u8 t66_config_s[] = { MXT_GOLDENREFERENCES_T66,
	0x00, 0x00, 0x00
};

static u8 t70_config_s[] = { MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,
/*instance 0*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 1*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 2*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 3*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 4*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 5*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 6*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 7*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 8*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 9*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 10*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
/*instance 11*/0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
};

static u8 t71_config_s[] = { MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
	0x01, 0x01, 0x00, 0x2C, 0x01, 0x00, 0x0F, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0
};
static u8 t73_config_s[] = { MXT_PROCI_ZONEINDICATION_T73,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static u8 t77_config_s[] = { MXT_SPT_CTESCANCONFIG_T77,
	0X09, 0X04, 0X78, 0X28
};

static u8 t79_config_s[] = { MXT_SPT_TOUCHEVENTTRIGGER_T79,
	0X00, 0X00, 0X00
};
#ifdef USE_FW_11AA
static u8 t82_config_s[] = { MXT_SPT_NOISESUPEXTENSION_T82,
	0x00, 0x00, 0x00
};
#endif

#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
static u8 t93_config_s[] = { MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93,
	0x01, 0x00, 0x00, 0xFF, 0x0F, 0x00, 0x00, 0xFF, 0x0F, 0xFA,
	0x00, 0xFF, 0x0F, 0x00, 0x14, 0x00, 0x32, 0x02
};
#endif
static u8 end_config_s[] = { MXT_RESERVED_T255 };

static const u8 *mxt1188S_config[] = {
	t37_config_s,
	t38_config_s,
	t71_config_s,
	t7_config_s,
	t8_config_s,
	t9_config_s,
	t15_config_s,
	t18_config_s,
	t19_config_s,
	t24_config_s,
	t25_config_s,
//	t27_config_s,
	t40_config_s,
	t42_config_s,
	t43_config_s,
	t46_config_s,
	t47_config_s,
	t55_config_s,
	t56_config_s,
	t57_config_s,
	t61_config_s,
	t62_config_s,
	t63_config_s,
	t65_config_s,
	t66_config_s,
	t70_config_s,
	t73_config_s,
	t77_config_s,
	t79_config_s,
#ifdef USE_FW_11AA
	t82_config_s,
#endif
#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
	t93_config_s,
#endif
	end_config_s
};

// {byte_offset, value}

static u8 t9_normal_config[] = { MXT_TOUCH_MULTI_T9, 1,
	7, 50
};

static u8 t9_charger_config[] = { MXT_TOUCH_MULTI_T9, 1,
	7, 70
};

static u8 t47_normal_config[] = { MXT_PROCI_STYLUS_T47, 1,
	8, 1,
};

static u8 t47_charger_config[] = { MXT_PROCI_STYLUS_T47, 1,
	8, 2,
};

static const u8 *mxt1188S_restore_config[] = {
	t9_normal_config,
	t47_normal_config,
	end_config_s
};

static const u8 *mxt1188S_charger_config[] = {
	t9_charger_config,
	t47_charger_config,
	end_config_s
};

static u8 t7_pen_config[] = { MXT_GEN_POWER_T7, 1,
	1, 255
};

static u8 t7_unpen_config[] = { MXT_GEN_POWER_T7, 1,
	1, 9
};

static u8 t9_pen_config[] = { MXT_TOUCH_MULTI_T9, 8,
	10, 0,
	11, 3,
	12, 2,
	31, 40,
	36, 4,
	37, 210,
	39, 80,
	45, 180,
};

static u8 t42_pen_config[] = { MXT_PROCI_TOUCHSUPPRESSION_T42, 3,
	2, 20,
	3, 30,
	4, 200,
};


static u8 t9_unpen_config[] = { MXT_TOUCH_MULTI_T9, 8,
	10, 5,
	11, 8,
	12, 7,
	31, 10,
	36, 5,
	37, 193,
	39, 110,
	45, 130,
};

static u8 t42_unpen_config[] = { MXT_PROCI_TOUCHSUPPRESSION_T42, 3,
	2, 70,
	3, 80,
	4, 180,
};


static u8 t9_pen_high_speed_config[] = { MXT_TOUCH_MULTI_T9, 4,
	11, 5,
	12, 3,
	36, 4,
	37, 190,
};

static u8 t9_pen_low_speed_config[] = { MXT_TOUCH_MULTI_T9, 4,
	11, 10,
	12, 10,
	36, 3,
	37, 220,
};

static const u8 *mxt1188S_pen_config[] = {
	t7_pen_config,
	t9_pen_config,
	t42_pen_config,
	end_config_s
};

static const u8 *mxt1188S_pen_high_speed_config[] = {
	t9_pen_high_speed_config,
	end_config_s
};

static const u8 *mxt1188S_pen_low_speed_config[] = {
	t9_pen_low_speed_config,
	end_config_s
};

static const u8 *mxt1188S_unpen_config[] = {
	t7_unpen_config,
	t9_unpen_config,
	t42_unpen_config,
	end_config_s
};

static u8 t7_sus_config[] = { MXT_GEN_POWER_T7,
#ifdef USE_FW_11AA
	3,
#else
	2,
#endif
	0, 64,
	1, 12,
#ifdef USE_FW_11AA
	4, 1,
#endif
};

static u8 t7_act_config[] = { MXT_GEN_POWER_T7,
#ifdef USE_FW_11AA
	3,
#else
	2,
#endif
	0, 32,
	1, 9,
#ifdef USE_FW_11AA
	4, 0,
#endif
};

static u8 t8_sus_config[] = { MXT_GEN_ACQUIRE_T8,
#ifdef USE_FW_11AA
	7,
#else
	6,
#endif
	2, 12,
	4, 15,
	6, 1,
	7, 200,
	8, 1,
	9, 216,
#ifdef USE_FW_11AA
	14, 1,
#endif
};

static u8 t8_act_config[] = { MXT_GEN_ACQUIRE_T8,
#ifdef USE_FW_11AA
	3,
#else
	2,
#endif
	2, 3,
	4, 0,
#ifdef USE_FW_11AA
	14, 0,
#endif
};

static u8 t9_sus_config[] = { MXT_TOUCH_MULTI_T9,
#ifdef USE_FW_11AA
	18,  /* The number of register configuration */
#else
	11,  /* The number of register configuration */
#endif
	0, 129,
	1, 2,
	2, 2,
	3, 23,
	4, 40,
	7, 60,
#ifdef USE_FW_11AA
	11, 150,	/* Movement hysteresis, initial */
	12, 100,	/* Movement hysteresis, next */
	14, 2,		/* Number of touch */
#endif
	18, 02,
	19, 10,
	20, 255,
	21, 15,
	30, 253,
#ifdef USE_FW_11AA
	35, 128, /* 0x80, CFG */
	36, 197, /* 0xC5, MoveFilter2 */
	39, 0,    /* Tracking thresholdscaling factor */
	45, 0,    /* MRGTHR adjustment strength */
#endif
};

static u8 t9_act_config[] = { MXT_TOUCH_MULTI_T9,
#ifdef USE_FW_11AA
	18,  /* The number of register configuration */
#else
	11,  /* The number of register configuration */
#endif
	0, 139,
	1, 0,
	2, 0,
	3, 27,
	4, 44,
	7, 50,
#ifdef USE_FW_11AA
	11, 8,  /* Movement hysteresis, initial */
	12, 7,  /* Movement hysteresis, next */
	14, 10, /* Number of touch */
#endif
	18, 255,
	19, 15,
	20, 255,
	21, 15,
	30, 50,
#ifdef USE_FW_11AA
	35, 150,  /* 0x96, CFG */
	36, 5,     /* 0x05, MoveFilter2 */
	39, 110,  /* 0x6E, Tracking thresholdscaling factor */
	45, 130,  /* 0x82, MRGTHR adjustment strength */
#endif
};

static u8 t24_sus_config[] = { MXT_PROCI_ONETOUCH_T24, 1,
#ifdef USE_FW_11AA
	0, 15,
#else
	0, 3
#endif
};

static u8 t24_act_config[] = { MXT_PROCI_ONETOUCH_T24, 1,
	0, 0,
};

static u8 t40_sus_config[] = {MXT_PROCI_GRIP_T40, 1,
	0, 1,
};

static u8 t40_act_config[] = {MXT_PROCI_GRIP_T40, 1,
	0, 0,
};

static u8 t42_sus_config[] = {MXT_PROCI_TOUCHSUPPRESSION_T42,
#ifdef USE_FW_11AA
	5,
#else
	4,
#endif
	0, 1,
#ifdef USE_FW_11AA
	1, 25,
#endif
	2, 40,
	3, 40,
	10, 6,
};

static u8 t42_act_config[] = {MXT_PROCI_TOUCHSUPPRESSION_T42,
#ifdef USE_FW_11AA
	5,
#else
	4,
#endif
	0, 55,
#ifdef USE_FW_11AA
	1, 0,
#endif
	2, 70,
	3, 80,
	10, 0,
};

static u8 t46_sus_config[] = {MXT_SPT_CTECONFIG_T46, 1,
	3, 16,
};

static u8 t46_act_config[] = {MXT_SPT_CTECONFIG_T46, 1,
	3, 24,
};

static u8 t47_sus_config[] = {MXT_PROCI_STYLUS_T47, 1,
	0, 64,
};

static u8 t47_act_config[] = {MXT_PROCI_STYLUS_T47, 1,
	0, 73,
};

static u8 t57_sus_config[] = {MXT_PROCI_EXTRATOUCHSCREENDATA_T57, 1,
	0, 0
};

static u8 t57_act_config[] = {MXT_PROCI_EXTRATOUCHSCREENDATA_T57, 1,
	0, 227
};

static u8 t62_sus_config[] = { MXT_PROCG_NOISESUPPRESSION_T62, 1,
	0, 0
};

static u8 t62_act_config[] = { MXT_PROCG_NOISESUPPRESSION_T62, 1,
	0, 1
};

static u8 t65_sus_config[] = {MXT_PROCI_LENSBENDING_T65, 1,
	0, 0
};

static u8 t65_act_config[] = {MXT_PROCI_LENSBENDING_T65, 1,
	0, 1
};

#ifdef USE_FW_11AA
static u8 t70_sus_config[] = {MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70, 12,
	0, 1,
	1, 2,
	3, 61,
	9, 4,
	10, 1,
	11, 2,
	13, 24,
	17, 5,
	20, 1,
	21, 13,
	23, 24,
	27, 6,
};

static u8 t70_act_config[] = {MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70, 12,
	0, 0,
	1, 0,
	3, 0,
	9, 0,
	10, 0,
	11, 0,
	13, 0,
	17, 0,
	20, 0,
	21, 0,
	23, 0,
	27, 0,
};
#endif

static u8 t77_sus_config[] = {MXT_SPT_CTESCANCONFIG_T77, 1,
	0, 9
};

static u8 t77_act_config[] = {MXT_SPT_CTESCANCONFIG_T77, 1,
	0, 8
};

#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
//17th bit in T93 is written number of udf's tab separately (data->g_tap_cnt).
//When UDF mode is used, screen size in T9 is adjusted by current.
static u8 t93_sus_config[] = {MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 1,
	0, 15
};

static u8 t93_act_config[] = {MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 1,
	0, 1
};
#endif

static const u8 *mxt1188S_sus_config[] = {
	t9_sus_config,
	t7_sus_config,
	t8_sus_config,
	t46_sus_config,
	t47_sus_config,
	t62_sus_config,
	t77_sus_config,
	t24_sus_config,
	t40_sus_config,
	t42_sus_config,
 	t57_sus_config,
	t65_sus_config,
#ifdef USE_FW_11AA
	t70_sus_config,
#endif
#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
	t93_act_config,
#endif
	end_config_s
};

static const u8 *mxt1188S_act_config[] = {
	t9_act_config,
	t7_act_config,
	t8_act_config,
	t24_act_config,
	t40_act_config,
	t42_act_config,
	t46_act_config,
	t47_act_config,
	t57_act_config,
	t62_act_config,
	t65_act_config,
#ifdef USE_FW_11AA
	t70_act_config,
#endif
	t77_act_config,
	end_config_s
};

static const u8 *mxt1188s_sus_charger_config[] = {
	t9_sus_config,
	t7_sus_config,
	t8_sus_config,
	t24_sus_config,
	t40_sus_config,
	t42_sus_config,
	t46_sus_config,
	t47_sus_config,
	t57_sus_config,
#ifdef USE_FW_11AA
	t70_sus_config,
#endif
#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
	t93_act_config,
#endif
	end_config_s
};

#ifdef CONFIG_TOUCHSCREEN_LGE_LPWG
static const u8 *mxt1188S_udf_sus_config[] = {
	t9_sus_config,
	t7_sus_config,
	t8_sus_config,
	t46_sus_config,
	t47_sus_config,
	t62_sus_config,
	t77_sus_config,
	t24_act_config,
	t40_sus_config,
	t42_sus_config,
 	t57_sus_config,
	t65_sus_config,
#ifdef USE_FW_11AA
	t70_sus_config,
#endif
	t93_sus_config,
	end_config_s
};

static const u8 *mxt1188S_udf_act_config[] = {
	t9_act_config,
	t7_act_config,
	t8_act_config,
	t24_act_config,
	t40_act_config,
	t42_act_config,
	t46_act_config,
	t47_act_config,
	t57_act_config,
	t62_act_config,
	t65_act_config,
#ifdef USE_FW_11AA
	t70_act_config,
#endif
	t77_act_config,
	t93_act_config,
	end_config_s
};

static const u8 *mxt1188s_udf_sus_charger_config[] = {
	t9_sus_config,
	t7_sus_config,
	t8_sus_config,
	t24_act_config,
	t40_sus_config,
	t42_sus_config,
	t46_sus_config,
	t47_sus_config,
	t57_sus_config,
#ifdef USE_FW_11AA
	t70_sus_config,
#endif
      t93_sus_config,
	end_config_s
};
#endif
