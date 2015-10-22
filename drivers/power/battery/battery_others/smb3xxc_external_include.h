#ifndef _SMB345C_EXTERNAL_INCLUDE_H
#define _SMB345C_EXTERNAL_INCLUDE_H 1

#include <linux/seq_file.h>
enum {
    USB_IN,
    AC_IN,
    CABLE_OUT,
    ENABLE_5V,
    DISABLE_5V,
};

typedef enum {
    BALANCE = 0,
    JEITA,
    FLAGS,
} charging_toggle_level_t;

/* Charger JEITA State by Battery Temperature
 * LTP: Low Temperature Protection
 * RTS: Rome Temperature Status
 * OTS: Over Temperature Status
 * OTP: Over Temperature Protection
 */
typedef enum {
    LTP = 0,
    RTS,
    OTS,
    OTP,
} charger_jeita_state_t;

/* New Charger JEITA State by Battery Temperature & Battery Voltage
 * FREEZE: Low Temperature Protection
 * COLD: Low Temperature Status
 * ROOM: Room Temperature Status
 * HOT: High Temperature Status
 * OVERHEAT: Over Temperature Protection
 */
typedef enum {
    FREEZE = 0,
    COLD,
    ROOM,
    HOT,
    OVERHEAT
} charger_jeita_status_t;

typedef enum {
    LOW_TEMPR = 0,
    MIDDLE_TEMPR,
    HIGH_TEMPR,
} float_volt_tempr_threshold_t;

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB345CCharger(int usb_state);
extern int pad_in_otg(int otg_state);
extern int crequest_power_supply_changed(void);
extern int smb3xxc_get_charger_type(void);
extern bool cget_sw_charging_toggle(void);
extern void caicl_dete_worker(struct work_struct *dat);
extern bool cexternal_power_source_present(void);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb345c_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb345c_charging_toggle(charging_toggle_level_t level, bool on);

/* To know the charging status
 *
 * return true when charger is charging */
int smb345c_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
int smb345c_charging_toggle(charging_toggle_level_t level, bool on);

/* To know if charger has an error
 *
 * return true means charger has an error */
bool smb345c_has_charger_error(void);

/* To acquire the value from charger ic registers
 *
 * return nothing */
int smb345c_dump_registers(struct seq_file *s);

/* To acquire the value from pmic registers
 *
 * return nothing */
int pmic_dump_registers(int dump_mask, struct seq_file *s);

#define ID_BYTES    2
#define B1IDMIN     0x460
#define B1IDMAX     0x462
#define B2IDMIN     0x4A1
#define B2IDMAX     0x4A3
#define B3IDMIN     0x4E2
#define B3IDMAX     0x4E4
int pmic_get_batt_id(void);

typedef enum {
    ID51K = 51,
    ID100K = 100,
    ID10K = 10,
} valid_battery_id_t;

bool is_battery_id_51k(void);
bool is_battery_id_100k(void);
bool is_battery_id_10k(void);

/* To config hard/soft limit cell temperature
 * monitor for soc control JEITA function
 *
 * return 0 means config completely */
int smb345c_soc_control_jeita(void);

/* To config hard/soft limit cell temperature
 * monitor for charger control JEITA function
 *
 * return 0 means config completely */
int smb345c_charger_control_jeita(void);

/* To config float voltage. Must aquired
 * the battery temperature from Gauge IC
 *
 * return 0 means config completely */
int smb345c_soc_control_float_vol(int bat_temp);

/* To config new charger jeita. Must aquired
 * the battery temperature/volt from Gauge IC
 *
 * return POWER_SUPPLY_STATUS */
int smb3xxc_jeita_control(int bat_tempr, int bat_volt, bool exec_jeita);

#define SMB345C_DEV_NAME "smb3xxc"
#define MSIC_CHRG_REG_DUMP_OTHERS	(1 << 3)

/* battery temperature unit: 0.1C */
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD         500
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD_PF400CG 450

#if defined(CONFIG_PF400CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  600
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 450
#elif defined(CONFIG_ME175CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#elif defined(CONFIG_A400CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  600
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#elif defined(CONFIG_A450CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  600
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 450
#else
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#endif

#endif /* _SMB345C_EXTERNAL_INCLUDE_H */

