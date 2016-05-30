#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#if defined(CONFIG_MACH_MSM8939_ALTEV2_VZW) || defined(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM) || defined(CONFIG_MACH_MSM8939_P1BC_SPR_US) || defined(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR) || \
	defined(CONFIG_MACH_MSM8939_P1BSSN_BELL_CA) || defined(CONFIG_MACH_MSM8939_P1BSSN_VTR_CA)
typedef enum {
	HW_REV_0 = 0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
} hw_rev_type;
#else
typedef enum {
        HW_REV_EVB1 = 0,
        HW_REV_EVB2,
        HW_REV_A,
        HW_REV_B,
        HW_REV_C,
        HW_REV_D,
        HW_REV_E,
        HW_REV_F,
        HW_REV_1_0,
        HW_REV_1_1,
        HW_REV_MAX
} hw_rev_type;
#endif

extern char *rev_str[];

hw_rev_type lge_get_board_revno(void);
char *lge_get_board_rev(void);
enum acc_cable_type {
	NO_INIT_CABLE = 0,
	CABLE_MHL_1K,
	CABLE_U_28P7K,
	CABLE_28P7K,
	CABLE_56K,
	CABLE_100K,
	CABLE_130K,
	CABLE_180K,
	CABLE_200K,
	CABLE_220K,
	CABLE_270K,
	CABLE_330K,
	CABLE_620K,
	CABLE_910K,
	CABLE_NONE
};

struct chg_cable_info {
	enum acc_cable_type cable_type;
	unsigned ta_ma;
	unsigned usb_ma;
};

void get_cable_data_from_dt(void *of_node);

struct qpnp_vadc_chip;
int lge_pm_get_cable_info(struct qpnp_vadc_chip *, struct chg_cable_info *);
void lge_pm_read_cable_info(struct qpnp_vadc_chip *);
enum acc_cable_type lge_pm_get_cable_type(void);
unsigned lge_pm_get_ta_current(void);
unsigned lge_pm_get_usb_current(void);
#ifdef CONFIG_USB_EMBEDDED_BATTERY_REBOOT
int lge_get_android_dlcomplete(void);
#endif
#if defined(CONFIG_LCD_KCAL)
struct kcal_data {
	int red;
	int green;
	int blue;
};

struct kcal_platform_data {
	int (*set_values) (int r, int g, int b);
	int (*get_values) (int *r, int *g, int *b);
	int (*refresh_display) (void);
};
#endif
#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
int lge_pre_self_diagnosis_pass(char *dev_code);
#endif
#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
struct pre_selfd_platform_data {
	int (*set_values) (int r, int g, int b);
	int (*get_values) (int *r, int *g, int *b);
};
#endif
enum cn_prop_type {
	CELL_U32 = 0,
	CELL_U64,
	STRING,
};

int __init lge_init_dt_scan_chosen(unsigned long node, const char *uname,
		int depth, void *data);

void get_dt_cn_prop(const char *name, void *value);
void get_dt_cn_prop_str(const char *name, char *value);
void get_dt_cn_prop_u64(const char *name, uint64_t *u64);
void get_dt_cn_prop_u32(const char *name, uint32_t *u32);


#ifdef CONFIG_LGE_LCD_TUNING
struct lcd_platform_data {
	int (*set_values) (int *tun_lcd_t);
	int (*get_values) (int *tun_lcd_t);
	};
void __init lge_add_lcd_misc_devices(void);
#endif
enum lge_laf_mode_type {
	LGE_LAF_MODE_NORMAL = 0,
	LGE_LAF_MODE_LAF,
};
enum lge_laf_mode_type lge_get_laf_mode(void);
#if defined(CONFIG_LCD_KCAL)
void __init lge_add_lcd_kcal_devices(void);
#endif
#if !defined(CONFIG_MACH_MSM8939_ALTEV2_VZW) && defined(CONFIG_LGE_QFPROM_INTERFACE) && !defined(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM) && !defined(CONFIG_MACH_MSM8939_P1BC_SPR_US)  && !defined(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR) && \
	!defined(CONFIG_MACH_MSM8939_P1BSSN_BELL_CA) && !defined(CONFIG_MACH_MSM8939_P1BSSN_VTR_CA)
void __init lge_add_qfprom_devices(void);
#endif
#if defined(CONFIG_LGE_DIAG_USB_ACCESS_LOCK) || defined(CONFIG_LGE_DIAG_ENABLE_SYSFS)
int __init lge_add_diag_devices(void);
#endif
#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
struct pseudo_batt_info_type {
	int mode;
	int id;
	int therm;
	int temp;
	int volt;
	int capacity;
	int charging;
};
void pseudo_batt_set(struct pseudo_batt_info_type *);
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
enum {
	BATT_ID_UNKNOWN = 0,
	BATT_ID_DS2704_N,
	BATT_ID_DS2704_L,
	BATT_ID_DS2704_C,
	BATT_ID_ISL6296_N,
	BATT_ID_ISL6296_L,
	BATT_ID_ISL6296_C,
	BATT_ID_RA4301_VC0,
	BATT_ID_RA4301_VC1,
	BATT_ID_RA4301_VC2,
	BATT_ID_SW3800_VC0,
	BATT_ID_SW3800_VC1,
	BATT_ID_SW3800_VC2,
};
bool is_lge_battery_valid(void);
int read_lge_battery_id(void);
extern int lge_battery_info;
#endif
enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_56K,
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_910K,
	LGE_BOOT_MODE_PIF_56K,
	LGE_BOOT_MODE_PIF_130K,
	LGE_BOOT_MODE_PIF_910K,
	LGE_BOOT_MODE_MINIOS    /* LGE_UPDATE for MINIOS2.0 */
};
#ifdef CONFIG_USB_G_LGE_ANDROID
int  __init lge_add_android_usb_devices(void);
#endif
enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_factory_boot(void);

#if defined(CONFIG_LGE_KSWITCH)
#define LGE_KSWITCH_UART_DISABLE    BIT(3)
extern int lge_get_kswitch_status(void);
#endif

#endif

extern int lge_get_bootreason(void);
