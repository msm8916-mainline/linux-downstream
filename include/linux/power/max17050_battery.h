/*
 *  Copyright (C) 2012 LG Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17050_BATTERY_H_
#define __MAX17050_BATTERY_H_

#define MAX17050_STATUS_BATTABSENT	(1 << 3)
#define MAX17050_BATTERY_FULL		100
#define MAX17050_DEFAULT_SNS_RESISTOR	10000

#define CONFIG_LGE_PM_MAX17050_POLLING

/* #define MAX17050_DEBUG */
/* Voltage Base */
/*#define CONFIG_LGE_PM_MAX17050_SOC_VF*/
/* Current Base */
#define CONFIG_LGE_PM_MAX17050_SOC_REP

/* Number of words in model characterisation data */
#define MODEL_SIZE			32

enum cell_type{
 LGC_LLL,
 TCD_AAC
};
struct max17050_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);

	bool enable_current_sense;
	bool ext_batt_psy;
	u16 empty_soc;
	u16 full_soc;

	/*
	 * R_sns in micro-ohms.
	 * default 10000 (if r_sns = 0) as it is the recommended value by
	 * the datasheet although it can be changed by board designers.
	 */
	unsigned int r_sns;

	int alert_gpio;

	u16 config;
	u16 filtercfg;
	u16 relaxcfg;
	u16 learncfg;
	u16 misccfg;
	u16 fullsocthr;
	u16 iavg_empty;

	u16 rcomp0;
	u16 tempco;
	u16 tempnom;
	u16 ichgterm;
	u16 tgain;
	u16 toff;

	u16 vempty;
	u16 qrtable00;
	u16 qrtable10;
	u16 qrtable20;
	u16 qrtable30;

	u16 capacity;
	u16 vf_fullcap;

	u16 param_version;
	u16 full_design;

	int rescale_soc;
	int rescale_factor;

	/* model characterisation data */
	u8 model_80[MODEL_SIZE];
	u8 model_90[MODEL_SIZE];
	u8 model_A0[MODEL_SIZE];
};

#define MAX17050_STATUS				0x00
#define MAX17050_V_ALRT_THRESHOLD	0x01
#define MAX17050_T_ALRT_THRESHOLD	0x02
#define MAX17050_SOC_ALRT_THRESHOLD	0x03
#define MAX17050_AT_RATE			0x04
#define MAX17050_REM_CAP_REP		0x05
#define MAX17050_SOC_REP			0x06
#define MAX17050_AGE				0x07
#define MAX17050_TEMPERATURE		0x08
#define MAX17050_V_CELL				0x09
#define MAX17050_CURRENT			0x0A
#define MAX17050_AVERAGE_CURRENT	0x0B

#define MAX17050_SOC_MIX			0x0D
#define MAX17050_SOC_AV				0x0E
#define MAX17050_REM_CAP_MIX		0x0F
#define MAX17050_FULL_CAP			0x10
#define MAX17050_TTE				0x11
#define MAX17050_QRTABLE00			0x12
#define MAX17050_FULL_SOC_THR		0x13

#define MAX17050_AVERAGE_TEMP		0x16
#define MAX17050_CYCLES				0x17
#define MAX17050_DESIGN_CAP			0x18
#define MAX17050_AVERAGE_V_CELL		0x19
#define MAX17050_MAX_MIN_TEMP		0x1A
#define MAX17050_MAX_MIN_VOLTAGE	0x1B
#define MAX17050_MAX_MIN_CURRENT	0x1C
#define MAX17050_CONFIG				0x1D
#define MAX17050_I_CHG_TERM			0x1E
#define MAX17050_REM_CAP_AV			0x1F

#define MAX17050_CUSTOMVER			0x20
#define MAX17050_VERSION			0x21
#define MAX17050_QRTABLE10			0x22
#define MAX17050_FULL_CAP_NOM		0x23
#define MAX17050_TEMP_NOM			0x24
#define MAX17050_TEMP_LIM			0x25

#define MAX17050_AIN				0x27
#define MAX17050_LEARN_CFG			0x28
#define MAX17050_FILTER_CFG			0x29
#define MAX17050_RELAX_CFG			0x2A
#define MAX17050_MISC_CFG			0x2B
#define MAX17050_T_GAIN				0x2C
#define MAX17050_T_OFF				0x2D
#define MAX17050_C_GAIN				0x2E
#define MAX17050_C_OFF				0x2F

#define MAX17050_QRTABLE20			0x32

#define MAX17050_FULLCAP0			0x35
#define MAX17050_I_AVG_EMPTY		0x36
#define MAX17050_F_CTC				0x37
#define MAX17050_RCOMP_0			0x38
#define MAX17050_TEMP_CO			0x39
#define MAX17050_V_EMPTY			0x3A

#define MAX17050_F_STAT				0x3D
#define MAX17050_TIMER				0x3E
#define MAX17050_SHDN_TIMER			0x3F

#define MAX17050_QRTABLE30			0x42

#define MAX17050_D_QACC				0x45
#define MAX17050_D_PACC				0x46

#define MAX17050_VFSOC0				0x48

#define MAX17050_QH0				0x4C
#define MAX17050_QH					0x4D

#define MAX17050_VFSOC0_LOCK		0x60
#define MAX17050_MODEL_LOCK1		0x62
#define MAX17050_MODEL_LOCK2		0x63

#define MAX17050_MODEL_TABLE_80		0x80
#define MAX17050_MODEL_TABLE_90		0x90
#define MAX17050_MODEL_TABLE_A0		0xA0

#define MAX17050_V_FOCV				0xFB
#define MAX17050_SOC_VF				0xFF

#endif /* __MAX17050_BATTERY_H_ */
