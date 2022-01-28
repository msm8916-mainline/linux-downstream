/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __BMS_BATTERYDATA_H
#define __BMS_BATTERYDATA_H

#include <linux/errno.h>

#define FCC_CC_COLS		5
#define FCC_TEMP_COLS		8

#define PC_CC_ROWS             31
#define PC_CC_COLS             13

#define PC_TEMP_ROWS		31
#define PC_TEMP_COLS		8

#define ACC_IBAT_ROWS		4
#define ACC_TEMP_COLS		3

#define MAX_SINGLE_LUT_COLS	20

#define MAX_BATT_ID_NUM		4
#define DEGC_SCALE		10

struct single_row_lut {
	int x[MAX_SINGLE_LUT_COLS];
	int y[MAX_SINGLE_LUT_COLS];
	int cols;
};

/**
 * struct sf_lut -
 * @rows:	number of percent charge entries should be <= PC_CC_ROWS
 * @cols:	number of charge cycle entries should be <= PC_CC_COLS
 * @row_entries:	the charge cycles/temperature at which sf data
 *			is available in the table.
 *		The charge cycles must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which sf data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @sf:		the scaling factor data
 */
struct sf_lut {
	int rows;
	int cols;
	int row_entries[PC_CC_COLS];
	int percent[PC_CC_ROWS];
	int sf[PC_CC_ROWS][PC_CC_COLS];
};

/**
 * struct pc_temp_ocv_lut -
 * @rows:	number of percent charge entries should be <= PC_TEMP_ROWS
 * @cols:	number of temperature entries should be <= PC_TEMP_COLS
 * @temp:	the temperatures at which ocv data is available in the table
 *		The temperatures must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which ocv data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @ocv:	the open circuit voltage
 */
struct pc_temp_ocv_lut {
	int rows;
	int cols;
	int temp[PC_TEMP_COLS];
	int percent[PC_TEMP_ROWS];
	int ocv[PC_TEMP_ROWS][PC_TEMP_COLS];
};

struct ibat_temp_acc_lut {
	int rows;
	int cols;
	int temp[ACC_TEMP_COLS];
	int ibat[ACC_IBAT_ROWS];
	int acc[ACC_IBAT_ROWS][ACC_TEMP_COLS];
};

struct batt_ids {
	int kohm[MAX_BATT_ID_NUM];
	int num;
};

enum battery_type {
	BATT_UNKNOWN = 0,
	BATT_PALLADIUM,
	BATT_DESAY,
	BATT_OEM,
	BATT_QRD_4V35_2000MAH,
	BATT_QRD_4V2_1300MAH,
};

/**
 * struct bms_battery_data -
 * @fcc:		full charge capacity (mAmpHour)
 * @fcc_temp_lut:	table to get fcc at a given temp
 * @pc_temp_ocv_lut:	table to get percent charge given batt temp and cycles
 * @pc_sf_lut:		table to get percent charge scaling factor given cycles
 *			and percent charge
 * @rbatt_sf_lut:	table to get battery resistance scaling factor given
 *			temperature and percent charge
 * @default_rbatt_mohm:	the default value of battery resistance to use when
 *			readings from bms are not available.
 * @delta_rbatt_mohm:	the resistance to be added towards lower soc to
 *			compensate for battery capacitance.
 * @rbatt_capacitve_mohm: the resistance to be added to compensate for
 *				battery capacitance
 * @flat_ocv_threshold_uv: the voltage where the battery's discharge curve
 *				starts flattening out.
 * @max_voltage_uv:	max voltage of the battery
 * @cutoff_uv:		cutoff voltage of the battery
 * @iterm_ua:		termination current of the battery when charging
 *			to 100%
 * @batt_id_kohm:	the best matched battery id resistor value
 */

struct bms_battery_data {
	unsigned int		fcc;
	struct single_row_lut	*fcc_temp_lut;
	struct single_row_lut	*fcc_sf_lut;
	struct pc_temp_ocv_lut	*pc_temp_ocv_lut;
	struct ibat_temp_acc_lut *ibat_acc_lut;
	struct sf_lut		*pc_sf_lut;
	struct sf_lut		*rbatt_sf_lut;
	int			default_rbatt_mohm;
	int			delta_rbatt_mohm;
	int			rbatt_capacitive_mohm;
	int			flat_ocv_threshold_uv;
	int			max_voltage_uv;
	int			cutoff_uv;
	int			iterm_ua;
	int			batt_id_kohm;
	const char		*battery_type;
};

#if defined(CONFIG_PM8921_BMS) || \
	defined(CONFIG_PM8921_BMS_MODULE) || \
	defined(CONFIG_QPNP_BMS) || \
	defined(CONFIG_QPNP_VM_BMS)
extern struct bms_battery_data  palladium_1500_data;
extern struct bms_battery_data  desay_5200_data;
extern struct bms_battery_data  oem_batt_data;
extern struct bms_battery_data QRD_4v35_2000mAh_data;
extern struct bms_battery_data  qrd_4v2_1300mah_data;
/* the corresponding relationship of the battery ID and the battery internal Resistance value.
            formula:  ( R_pull-up / "qcom,batt-id-kohm" ) = (refs-vdd/(batt ID * scale))
                           R_pull-up = "qcom,rpull-up-kohm = <100>"
                           refs-vdd = "qcom,vref-batt-therm = <1800000>"
                           scale = 300000

    the battery ID: enum battery_id element in Batterydata-lib.h.
    the battery internal Resistance value: the dtsi property value of "qcom,batt-id-kohm" in batterydata-palladium-xxxx.dtsi
    B_THE_FIRST_SUPPLIER ----> qcom,batt-id-kohm = <20>
    B_THE_SECEND_SUPPLIER ----> qcom,batt-id-kohm = <50>
    B_THE_THIRD_SUPPLIER ----> qcom,batt-id-kohm = <100>
    B_THE_FOURTH_SUPPLIER ----> qcom,batt-id-kohm = <200>
    
    this above relationship must be accurate, otherwise bms can not load the right paramter of battery curve according to battery id.
*/
enum battery_id{
	B_UNKOWN = 0,
    B_THE_1_SUPPLIER,
    B_THE_2_SUPPLIER,
    B_THE_3_SUPPLIER,
    B_THE_4_SUPPLIER,
    B_THE_5_SUPPLIER,
    B_THE_6_SUPPLIER,
    B_THE_7_SUPPLIER,
	B_MAX,
};
static const char * const battery_id_strings[] = {
	"unkown battery supplier",
	"the 1 battery supplier : 33k--(20kohm)",
	"the 2 battery supplier : 100k--(50kohm)",
	"the 3 battery supplier : 10k--(100kohm)",
	"the 4 battery supplier : 33K,4.7uf--(200kohm)",
	"the 5 battery supplier : 33k,10uf--(500kohm)",
	"the 6 battery supplier : 100k,2.2uf--(0kohm)",
	"the 7 battery supplier : 100k,10uf--(-699kohm)",
	"out of the battery supplier range",
};
struct tc_item{
	int		min;
	int		max;
	int		scale;
};
struct r_item{
	int		v1min;
	int		v1max;
	int		id;
};
struct rc_item{
	int		v1min;
	int		v1max;
	int		v2min;
	int		v2max;
	int		id;
};
/*
#if 1//defined(PD1401V) || defined(PD1401F)||defined(PD1401F_EX)
static struct tc_item tc_items[] = {
	{
		.min	= 549,
		.max	= 8888,
		.scale	= 0,
	},
	{
		.min	= 451,
		.max	= 550,
		.scale	= 45,
	},
	{
		.min	= 51,
		.max	= 450,
		.scale	= 50,
	},
	{
		.min	= 1,
		.max	= 50,
		.scale	= 30,
	},
	{
		.min	= -29,
		.max	= 0,
		.scale	= 30,
	},
	{
		.min	= -79,
		.max	= -30,
		.scale	= 15,
	},
	{
		.min	= -8888,
		.max	= -80,
		.scale	= 0,
	},
};
#else
static const struct tc_item tc_items[] = {
	{
		.min	= 551,
		.max	= 8888,
		.scale	= 0,
	},
	{
		.min	= 451,
		.max	= 550,
		.scale	= 45,
	},
	{
		.min	= 51,
		.max	= 450,
		.scale	= 50,
	},
	{
		.min	= 1,
		.max	= 50,
		.scale	= 30,
	},
	{
		.min	= -30,
		.max	= 0,
		.scale	= 25,
	},
	{
		.min	= -79,
		.max	= -31,
		.scale	= 15,
	},
	{
		.min	= -8888,
		.max	= -80,
		.scale	= 0,
	},
};
#endif
static const struct rc_item rc_items[] = { 
	{
		.v1min	= 746,
		.v1max	= 966,
		.v2min	= 337,
		.v2max	= 509,
		.id		= B_73_ATL,
	},
	{
		.v1min	= 1117,
		.v1max	= 1398,
		.v2min	= 825,
		.v2max	= 1113,
		.id		= B_73_SONY,
	},
};
*/
int interpolate_fcc(struct single_row_lut *fcc_temp_lut, int batt_temp);
int interpolate_scalingfactor(struct sf_lut *sf_lut, int row_entry, int pc);
int interpolate_scalingfactor_fcc(struct single_row_lut *fcc_sf_lut,
				int cycles);
int interpolate_pc(struct pc_temp_ocv_lut *pc_temp_ocv,
				int batt_temp_degc, int ocv);
int interpolate_ocv(struct pc_temp_ocv_lut *pc_temp_ocv,
				int batt_temp_degc, int pc);
int interpolate_slope(struct pc_temp_ocv_lut *pc_temp_ocv,
					int batt_temp, int pc);
int interpolate_acc(struct ibat_temp_acc_lut *ibat_acc_lut,
					int batt_temp, int ibat);
int linear_interpolate(int y0, int x0, int y1, int x1, int x);
int is_between(int left, int right, int value);
#else
static inline int interpolate_fcc(struct single_row_lut *fcc_temp_lut,
			int batt_temp)
{
	return -EINVAL;
}
static inline int interpolate_scalingfactor(struct sf_lut *sf_lut,
			int row_entry, int pc)
{
	return -EINVAL;
}
static inline int interpolate_scalingfactor_fcc(
			struct single_row_lut *fcc_sf_lut, int cycles)
{
	return -EINVAL;
}
static inline int interpolate_pc(struct pc_temp_ocv_lut *pc_temp_ocv,
			int batt_temp_degc, int ocv)
{
	return -EINVAL;
}
static inline int interpolate_ocv(struct pc_temp_ocv_lut *pc_temp_ocv,
			int batt_temp_degc, int pc)
{
	return -EINVAL;
}
static inline int interpolate_slope(struct pc_temp_ocv_lut *pc_temp_ocv,
					int batt_temp, int pc)
{
	return -EINVAL;
}
static inline int linear_interpolate(int y0, int x0, int y1, int x1, int x)
{
	return -EINVAL;
}
static inline int is_between(int left, int right, int value)
{
	return -EINVAL;
}
static inline int interpolate_acc(struct ibat_temp_acc_lut *ibat_acc_lut,
						int batt_temp, int ibat)
{
	return -EINVAL;
}
#endif

#endif
