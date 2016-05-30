#include <linux/kernel.h>
#include <linux/string.h>

#include <soc/qcom/lge/board_lge.h>

//#include <asm/system_info.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#ifdef CONFIG_USB_G_LGE_ANDROID
#include <linux/init.h>
#include <linux/platform_device.h>
#include <soc/qcom/lge/lge_android_usb.h>
#endif

#ifdef CONFIG_LGE_QSDL_SUPPORT
#include <soc/qcom/lge/lge_qsdl.h>
#endif

#define PROP_VAL_MAX_SIZE 50

struct chg_cable_info_table {
	int threshhold;
	enum acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};

#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
	.name = "lge-qfprom",
	.id = -1,
};

static int __init lge_add_qfprom_devices(void)
{
	return platform_device_register(&qfprom_device);
}
arch_initcall(lge_add_qfprom_devices);
#endif

#define ADC_NO_INIT_CABLE   0
#define C_NO_INIT_TA_MA     0
#define C_NO_INIT_USB_MA    0
#define ADC_CABLE_NONE      1900000
#define C_NONE_TA_MA        700
#define C_NONE_USB_MA       500

#define MAX_CABLE_NUM		15
static bool cable_type_defined;
static struct chg_cable_info_table pm8941_acc_cable_type_data[MAX_CABLE_NUM];

#ifdef CONFIG_LGE_KSWITCH
static int s_kswitch_flag;
#endif

static int cn_arr_len = 3;
struct cn_prop {
	char *name;
	enum cn_prop_type type;
	uint32_t cell_u32;
	uint64_t cell_u64;
	char str[PROP_VAL_MAX_SIZE];
	uint8_t is_valid;
};

static struct cn_prop cn_array[] = {
	{
		.name = "lge,log_buffer_phy_addr",
		.type = CELL_U32,
	},
	{
		.name = "lge,sbl_delta_time",
		.type = CELL_U32,
	},
	{
		.name = "lge,lk_delta_time",
		.type = CELL_U64,
	},
};

int __init lge_init_dt_scan_chosen(unsigned long node,
		const char *uname, int depth, void *data)
{
	int len;
	int i;
	enum cn_prop_type type;
	char *p;
	uint32_t *u32;
	void *temp;

	if (depth != 1 || (strcmp(uname, "chosen") != 0
					   && strcmp(uname, "chosen@0") != 0))
		return 0;
	for (i = 0; i < cn_arr_len; i++) {
		type = cn_array[i].type;
		temp = (void*)of_get_flat_dt_prop(node, cn_array[i].name, &len);
		if (temp == NULL)
			continue;
		if (type == CELL_U32) {
			u32 = (uint32_t*)of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (u32 != NULL)
				cn_array[i].cell_u32 = of_read_ulong(u32, 1);
		} else if (type == CELL_U64) {
			u32 = (uint32_t*)of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (u32 != NULL)
				cn_array[i].cell_u64 = of_read_number(u32, 2);
		} else {
			p = (char*)of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if (p != NULL)
				strlcpy(cn_array[i].str, p, len);
		}
		cn_array[i].is_valid = 1;
	}

	return 0;
}

void get_dt_cn_prop_u32(const char *name, uint32_t *u32)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u32 = cn_array[i].cell_u32;
			return;
		}
	}
	pr_err("The %s node have not property value\n", name);
}

void get_dt_cn_prop_u64(const char *name, uint64_t *u64)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u64 = cn_array[i].cell_u64;
			return;
		}
	}
	pr_err("The %s node have not property value\n", name);
}

void get_dt_cn_prop_str(const char *name, char *value)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			strlcpy(value, cn_array[i].str,
					strlen(cn_array[i].str));
			return;
		}
	}
	pr_err("The %s node have not property value\n", name);
}

#if defined(CONFIG_LGE_DIAG_USB_ACCESS_LOCK) || defined(CONFIG_LGE_DIAG_ENABLE_SYSFS)
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, //&lg_diag_cmd_pdata
	},
};

int __init lge_add_diag_devices(void)
{
	return platform_device_register(&lg_diag_cmd_device);
}
arch_initcall(lge_add_diag_devices);
#endif
void get_cable_data_from_dt(void *of_node)
{
	int i;
	u32 cable_value[3];
	struct device_node *node_temp = (struct device_node *)of_node;
	const char *propname[MAX_CABLE_NUM] = {
		"lge,no-init-cable",
		"lge,cable-mhl-1k",
		"lge,cable-u-28p7k",
		"lge,cable-28p7k",
		"lge,cable-56k",
		"lge,cable-100k",
		"lge,cable-130k",
		"lge,cable-180k",
		"lge,cable-200k",
		"lge,cable-220k",
		"lge,cable-270k",
		"lge,cable-330k",
		"lge,cable-620k",
		"lge,cable-910k",
		"lge,cable-none"
	};
	if (cable_type_defined) {
		pr_info("Cable type is already defined\n");
		return;
	}

	for (i = 0 ; i < MAX_CABLE_NUM ; i++) {
		of_property_read_u32_array(node_temp, propname[i],
				cable_value, 3);
		pm8941_acc_cable_type_data[i].threshhold = cable_value[0];
		pm8941_acc_cable_type_data[i].type = i;
		pm8941_acc_cable_type_data[i].ta_ma = cable_value[1];
		pm8941_acc_cable_type_data[i].usb_ma = cable_value[2];
	}
	cable_type_defined = 1;
}

int lge_pm_get_cable_info(struct qpnp_vadc_chip *vadc,
		struct chg_cable_info *cable_info)
{
	char *type_str[] = {
		"NOT INIT", "MHL 1K", "U_28P7K", "28P7K", "56K",
		"100K", "130K", "180K", "200K", "220K",
		"270K", "330K", "620K", "910K", "OPEN"
	};

	struct qpnp_vadc_result result;
	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;

	int table_size = ARRAY_SIZE(pm8941_acc_cable_type_data);
	int acc_read_value = 0;
	int acc_read_value_data[3] = {0,};
	int i, j, rc;
	int count = 3;

	if (!info) {
		pr_err("%s : invalid info parameters\n",
				__func__);
		return -1;
	}

	if (!vadc) {
		pr_err("%s : invalid vadc parameters\n",
				__func__);
		return -1;
	}

	if (!cable_type_defined) {
		pr_err("%s : cable type is not defined yet.\n",
				__func__);
		return -1;
	}

	for (i = 0; i < count; i++) {
		rc = qpnp_vadc_read(vadc, P_MUX1_1_1, &result);
		if (rc < 0) {
			if (rc == -ETIMEDOUT) {
				/* reason: adc read timeout,
				 * assume it is open cable
				 */
				info->cable_type = CABLE_NONE;
				info->ta_ma = C_NONE_TA_MA;
				info->usb_ma = C_NONE_USB_MA;
			}

			pr_err("%s : adc read error - %d\n",
					__func__, rc);
			return rc;
		}

		acc_read_value_data[i] = (int)result.physical;
		pr_info("%s : acc_read_value - %d\n",
				__func__, (int)result.physical);
		mdelay(10);

		for (j = 0; j < i; j++) {
			if (abs(acc_read_value_data[i] - acc_read_value_data[j]) > 100000) {
				count = 0;
				acc_read_value = 1800000;
				pr_info("%s: abnormal acc_read_value\n", __func__);
				break;
			} else {
				acc_read_value = (int)result.physical;
			}
		}
	}

	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume : adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		table = &pm8941_acc_cable_type_data[i];

		if (acc_read_value <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;
			break;
		}
	}

	pr_err("\n\n[PM] Cable detected: %d(%s)(%d, %d)\n\n",
			acc_read_value, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);
	return 0;
}

static struct chg_cable_info lge_cable_info;

enum acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(struct qpnp_vadc_chip *vadc)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(vadc, &lge_cable_info);
}
#ifdef CONFIG_USB_EMBEDDED_BATTERY_REBOOT
/*
   for download complete using LAF image
   return value : 1 --> right after laf complete & reset
*/

int android_dlcomplete = 0;

int __init lge_android_dlcomplete(char *s)
{
	if(strncmp(s,"1",1) == 0)
		android_dlcomplete = 1;
	else
		android_dlcomplete = 0;
			printk("androidboot.dlcomplete = %d\n", android_dlcomplete);

	return 1;
}
__setup("androidboot.dlcomplete=", lge_android_dlcomplete);

int lge_get_android_dlcomplete(void)
{
	return android_dlcomplete;
}
#endif

/* for board revision */
static hw_rev_type lge_bd_rev = HW_REV_A;

/* CAUTION: These strings are come from LK. */
#if defined(CONFIG_MACH_MSM8939_ALTEV2_VZW) || defined(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM) || defined(CONFIG_MACH_MSM8939_P1BC_SPR_US) || defined(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR) || \
	defined (CONFIG_MACH_MSM8939_P1BSSN_BELL_CA) || defined (CONFIG_MACH_MSM8939_P1BSSN_VTR_CA)
char *rev_str[] = {"rev_0", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_10", "rev_11","reserved"};
#else
char *rev_str[] = {"evb1", "evb2", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_f", "rev_10", "rev_11","reserved"};
#endif

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
extern void pseudo_batt_ibatmax_set(void);

struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};

void pseudo_batt_set(struct pseudo_batt_info_type *info)
{
	struct power_supply *batt_psy;

	pr_err("pseudo_batt_set\n");

	batt_psy = power_supply_get_by_name("battery");

	if (!batt_psy) {
		pr_err("called before init\n");
		return;
	}

	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

#ifndef CONFIG_BQ24296_CHARGER
#if !defined(CONFIG_MACH_MSM8939_ALTEV2_VZW) && !defined(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM) && !defined(CONFIG_MACH_MSM8939_P1BC_SPR_US) && !defined(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR) && \
	!defined(CONFIG_MACH_MSM8939_P1BSSN_BELL_CA) && !defined(CONFIG_MACH_MSM8939_P1BSSN_VTR_CA)
	pseudo_batt_ibatmax_set();
#endif
#endif
	power_supply_changed(batt_psy);
}
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER

int lge_battery_info = BATT_ID_UNKNOWN;
bool is_lge_battery_valid(void)
{

	if(lge_pm_get_cable_type()== CABLE_56K ||
		lge_pm_get_cable_type()== CABLE_130K ||
		lge_pm_get_cable_type()== CABLE_910K)
		return true;

	if (lge_battery_info == BATT_ID_DS2704_N ||
		lge_battery_info == BATT_ID_DS2704_L ||
		lge_battery_info == BATT_ID_DS2704_C ||
		lge_battery_info == BATT_ID_ISL6296_N ||
		lge_battery_info == BATT_ID_ISL6296_L ||
		lge_battery_info == BATT_ID_ISL6296_C ||
		lge_battery_info == BATT_ID_RA4301_VC0 ||
		lge_battery_info == BATT_ID_RA4301_VC1 ||
		lge_battery_info == BATT_ID_RA4301_VC2 ||
		lge_battery_info == BATT_ID_SW3800_VC0 ||
		lge_battery_info == BATT_ID_SW3800_VC1 ||
		lge_battery_info == BATT_ID_SW3800_VC2)
		return true;

	return false;

}
//EXPORT_SYMBOL(is_lge_battery_valid);

int read_lge_battery_id(void)
{
#if !defined(CONFIG_MACH_MSM8939_P1BDSN_GLOBAL_COM) && !defined(CONFIG_MACH_MSM8939_P1BSSN_SKT_KR)
		return lge_battery_info;
#else
		return 1;
#endif
}
//EXPORT_SYMBOL(read_lge_battery_id);

static int __init battery_information_setup(char *batt_info)
{
        if(!strcmp(batt_info, "DS2704_N"))
                lge_battery_info = BATT_ID_DS2704_N;
        else if(!strcmp(batt_info, "DS2704_L"))
                lge_battery_info = BATT_ID_DS2704_L;
        else if(!strcmp(batt_info, "DS2704_C"))
                lge_battery_info = BATT_ID_DS2704_C;
        else if(!strcmp(batt_info, "ISL6296_N"))
                lge_battery_info = BATT_ID_ISL6296_N;
        else if(!strcmp(batt_info, "ISL6296_L"))
                lge_battery_info = BATT_ID_ISL6296_L;
        else if(!strcmp(batt_info, "ISL6296_C"))
                lge_battery_info = BATT_ID_ISL6296_C;
        else if(!strcmp(batt_info, "RA4301_VC0"))
                lge_battery_info = BATT_ID_RA4301_VC0;
        else if(!strcmp(batt_info, "RA4301_VC1"))
                lge_battery_info = BATT_ID_RA4301_VC1;
        else if(!strcmp(batt_info, "RA4301_VC2"))
                lge_battery_info = BATT_ID_RA4301_VC2;
        else if(!strcmp(batt_info, "SW3800_VC0"))
                lge_battery_info = BATT_ID_SW3800_VC0;
        else if(!strcmp(batt_info, "SW3800_VC1"))
                lge_battery_info = BATT_ID_SW3800_VC1;
        else if(!strcmp(batt_info, "SW3800_VC2"))
                lge_battery_info = BATT_ID_SW3800_VC2;
        else
                lge_battery_info = BATT_ID_UNKNOWN;

        printk(KERN_INFO "Battery : %s %d\n", batt_info, lge_battery_info);

        return 1;
}

__setup("lge.battid=", battery_information_setup);
#endif

static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = (hw_rev_type) i;
			/* it is defined externally in <asm/system_info.h> */
		//	system_rev = lge_bd_rev;
			break;
		}
	}

	pr_info("BOARD : LGE %s\n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
	return lge_bd_rev;
}

char *lge_get_board_rev(void)
{
	char *name;
	name = rev_str[lge_bd_rev];
	return name;
}

#if defined(CONFIG_LCD_KCAL)
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;

extern int kcal_set_values(int kcal_r, int kcal_g, int kcal_b);
static int __init display_kcal_setup(char *kcal)
{
	char vaild_k = 0;
	int kcal_r = 255;
	int kcal_g = 255;
	int kcal_b = 255;

	sscanf(kcal, "%d|%d|%d|%c", &kcal_r, &kcal_g, &kcal_b, &vaild_k );
	pr_info("kcal is %d|%d|%d|%c\n", kcal_r, kcal_g, kcal_b, vaild_k);
	printk("display_kcal_setup ==> kcal_r=[%d], kcal_g=[%d], kcal_b=[%d], vaild_k=[%c]\n", kcal_r, kcal_g, kcal_b, vaild_k);

	if (vaild_k != 'K') {
		pr_info("kcal not calibrated yet : %d\n", vaild_k);
		kcal_r = kcal_g = kcal_b = 255;
		pr_info("set to default : %d\n", kcal_r);
	}

	kcal_set_values(kcal_r, kcal_g, kcal_b);
	return 1;
}
__setup("lge.kcal=", display_kcal_setup);
#endif

/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
static int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	/* LGE_UPDATE_S for MINIOS2.0 */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/* LGE_UPDATE_E for MINIOS2.0 */

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

int lge_get_factory_boot(void)
{
	int res;

	/*   if boot mode is factory,
	 *   cable must be factory cable.
	 */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_56K:
	case LGE_BOOT_MODE_PIF_130K:
	case LGE_BOOT_MODE_PIF_910K:
	case LGE_BOOT_MODE_MINIOS:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}
	return res;
}

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;
static int __init lge_laf_mode_init(char *s)
{
	if (strcmp(s, ""))
		lge_laf_mode = LGE_LAF_MODE_LAF;
	return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
	return lge_laf_mode;
}

#ifdef CONFIG_USB_G_LGE_ANDROID
static int get_factory_cable(void)
{
	int res = 0;

	/* if boot mode is factory, cable must be factory cable. */
	switch (lge_boot_mode) {
	case LGE_BOOT_MODE_QEM_56K:
	case LGE_BOOT_MODE_PIF_56K:
		res = LGEUSB_FACTORY_56K;
		break;

	case LGE_BOOT_MODE_QEM_130K:
	case LGE_BOOT_MODE_PIF_130K:
		res = LGEUSB_FACTORY_130K;
		break;

	case LGE_BOOT_MODE_QEM_910K:
	case LGE_BOOT_MODE_PIF_910K:
		res = LGEUSB_FACTORY_910K;
		break;

	default:
		res = 0;
		break;
	}

	return res;
}

struct lge_android_usb_platform_data lge_android_usb_pdata = {
	.vendor_id = 0x1004,
	.factory_pid = 0x6000,
	.iSerialNumber = 0,
	.product_name = "LGE Android Phone",
	.manufacturer_name = "LG Electronics Inc.",
	.factory_composition = "acm,diag",
	.get_factory_cable = get_factory_cable,
};

static struct platform_device lge_android_usb_device = {
	.name = "lge_android_usb",
	.id = -1,
	.dev = {
		.platform_data = &lge_android_usb_pdata,
	},
};

int  __init lge_add_android_usb_devices(void)
{
	return platform_device_register(&lge_android_usb_device);
}
module_init(lge_add_android_usb_devices);
#endif

static int lge_boot_reason = -1; /* undefined for error checking */
static int __init lge_check_bootreason(char *reason)
{
	int ret = 0;

	/* handle corner case of kstrtoint */
	if (!strcmp(reason, "0xffffffff")) {
		lge_boot_reason = 0xffffffff;
		return 1;
	}
	ret = kstrtoint(reason, 16, &lge_boot_reason);
	if (!ret)
		printk(KERN_INFO "LGE REBOOT REASON: %x\n", lge_boot_reason);
	else
		printk(KERN_INFO "LGE REBOOT REASON: Couldn't get bootreason - %d\n",ret);
	return 1;
}
__setup("lge.bootreason=", lge_check_bootreason);

int lge_get_bootreason(void)
{
	return lge_boot_reason;
}

#ifdef CONFIG_LGE_KSWITCH
static
int atoi(const char* str)
{
	int val = 0;

	if (str == NULL)
	{
		printk(KERN_CRIT "[KSwitch] kill switch flag string pointer is NULL\n");
		return -1;
	}

	for (;;str++)
	{
		switch (*str)
		{
			case '0' ... '9':
				val = 10*val + (*str - '0');
				break;
			default:
				return val;
		}
	}
}

static
int __init kswitch_setup(char* value)
{
	s_kswitch_flag = atoi(value);

	if (s_kswitch_flag < 0)
	{
		printk(KERN_CRIT "[KSwitch] malformed kswitch flag value is used to setup: 0x%x\n", s_kswitch_flag);
		s_kswitch_flag = 0;
	}

	return 1;
}
__setup("kswitch=", kswitch_setup);

int  lge_get_kswitch_status()
{
	return s_kswitch_flag;
}
#endif/* CONFIG_LGE_KSWITCH */

#ifdef CONFIG_LGE_QSDL_SUPPORT
static struct lge_qsdl_platform_data lge_qsdl_pdata = {
	.oneshot_read = 0,
	.using_uevent = 0
};

static struct platform_device lge_qsdl_device = {
	.name = LGE_QSDL_DEV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &lge_qsdl_pdata,
	}
};

static int  __init lge_add_qsdl_device(void)
{
  return platform_device_register(&lge_qsdl_device);
}

arch_initcall(lge_add_qsdl_device);

#endif /* CONFIG_LGE_QSDL_SUPPORT */
