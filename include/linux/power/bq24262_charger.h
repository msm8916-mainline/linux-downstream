/*
* Copyright(c) 2013, LGE Inc. All rights reserved.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#ifndef __BQ24262_CHARGER_H__
#define __BQ24262_CHARGER_H__
#include <linux/power_supply.h>
#define BQ24262_NAME  "bq24262"
#ifndef CONFIG_LGE_CHARGER_TEMP_SCENARIO
enum {
	DISCHG_BATT_TEMP_OVER_60,
	DISCHG_BATT_TEMP_57_60,
	DISCHG_BATT_TEMP_UNDER_57,
	CHG_BATT_TEMP_LEVEL_1, // OVER_55
	CHG_BATT_TEMP_LEVEL_2, // 46_55
	CHG_BATT_TEMP_LEVEL_3, // 42_45
	CHG_BATT_TEMP_LEVEL_4, // M4_41
	CHG_BATT_TEMP_LEVEL_5, // M10_M5
	CHG_BATT_TEMP_LEVEL_6, // UNDER_M10
};
enum {
	DISCHG_BATT_NORMAL_STATE,
	DISCHG_BATT_WARNING_STATE,
	DISCHG_BATT_POWEROFF_STATE,
	CHG_BATT_NORMAL_TEMP_STATE,
	CHG_BATT_DC_CURRENT_STATE,
	CHG_BATT_WARNING_STATE,
	CHG_BATT_STOP_CHARGING_STATE,
};
struct bq24262_platform_data {
	int int_gpio;
	int chg_current_ma;
	int term_current_ma;
	int temp_level_1;
	int temp_level_2;
	int temp_level_3;
	int temp_level_4;
	int temp_level_5;
	int *thermal_mitigation;
	int thermal_levels;
	unsigned int max_bat_chg_current;
	int regulation_mV;
};
#endif
int bq24262_set_usb_power_supply_type(enum power_supply_type type);
int bq24262_charger_register_vbus_sn(void (*callback)(int));
int bq24262_is_charger_plugin(void);
void bq24262_charger_unregister_vbus_sn(void (*callback)(int));

extern int32_t bq24262_is_ready(void);
int bq24262_get_prop_batt_temp_raw(void);
#endif
