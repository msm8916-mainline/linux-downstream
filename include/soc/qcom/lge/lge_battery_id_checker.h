#ifndef __LGE_BATTERY_ID_CHECKER_H
#define __LGE_BATTERY_ID_CHECKER_H

enum {
	BATT_ID_UNKNOWN    = 0,
	BATT_ID_DS2704_N   = 17,
	BATT_ID_DS2704_L   = 32,
	BATT_ID_DS2704_C   = 48,
	BATT_ID_ISL6296_N  = 73,
	BATT_ID_ISL6296_L  = 94,
	BATT_ID_ISL6296_C  = 105,
	BATT_ID_RA4301_VC0 = 130,
	BATT_ID_RA4301_VC1 = 147,
	BATT_ID_RA4301_VC2 = 162,
	BATT_ID_SW3800_VC0 = 187,
	BATT_ID_SW3800_VC1 = 204,
	BATT_ID_SW3800_VC2 = 219,
	BATT_NOT_PRESENT  = 200,
};
bool is_lge_battery_valid(void);
int read_lge_battery_id(void);
extern int lge_battery_info;

#endif
