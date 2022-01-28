/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
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
#define DEBUG 	1
#define pr_fmt(fmt) "SMB358 %s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>

#include <linux/regulator/driver.h>
#include <linux/bitops.h>
#include <linux/ratelimit.h>
#include <linux/wakelock.h>

#ifdef pr_debug 
#undef pr_debug
#define pr_debug(fmt, ...) \
    printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#endif

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_B_REG			0x3C
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F
/*otg config registers*/
#define OTG_TLIM_THERM_CNTRL_REG 0x0A

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_AICL_BEHAVIOR_BIT		BIT(1)
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	(BIT(5) | BIT(4))
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		(BIT(5) | BIT(4))
#define CHG_CTRL_AICL_BEHAVIOR_MASK		BIT(1)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define STAT_AND_TIMER_CTRL_COMPLTE_CHARGE_TIMEOUT (BIT(2) | BIT(3))

#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		(BIT(5) | BIT(6))
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		(BIT(5) | BIT(6))

#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_DISABLE_BIT		BIT(4)
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F
#define STATUS_INT_RECHGING_BIT			BIT(4)
/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status  bits */
#define STATUS_C_CHARGING_MASK			(BIT(1) | BIT(2))
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			(BIT(2) | BIT(1))
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_PORT_OTHER			BIT(0)|BIT(1)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_ACA_A			BIT(0)|BIT(2)
#define STATUS_D_PORT_ACA_B			BIT(1)|BIT(2)
#define STATUS_D_PORT_ACA_C			BIT(0)|BIT(1)|BIT(2)
#define STATUS_D_PORT_ACA_DOCK		BIT(3)
#define CHG_OTG_CUR_MASK			(BIT(2) | BIT(3))
#define CHG_OTG_CUR_750MA			BIT(3)
#define CHG_OTG_CUR_900MA			(BIT(2) | BIT(3))
#define CHG_PIN_CTRL_PRE_BIAS_MASK	BIT(0)
#define SWITCH_FREQUENCY_EN_BIT  	BIT(7)
#define SWITCH_FREQUENCY_EN_MASK  	BIT(7)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB358_CHG_CUR_DEFAULT_MA	1000
#define SMB358_FB_ON_CUR_DEFAULT_MA	900
#define SMB358_FASTCHG_CUR_LIMIT_DEFAULT_MA 1300

#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50
#define SMB358_NAME		"smb358"

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
};
enum {
	SMB358_CHG_OV_STATUS = 0,
	SMB358_BAT_OV_STATUS = 3,
	SMB358_CHG_TIMEOUT_STATUS,
	SMB358_CHG_TERM_STATUS = 10,
};
struct smb358_irq_bit{
	int			reg;
	int			reg_bit;
	int			sta_bit;
	bool		clr_en;
};
/*status register to dev->status bit map*/
static struct smb358_irq_bit irq_bits_lut[] = {
	{
		.reg		= IRQ_E_REG,
		.reg_bit	= 2,
		.sta_bit	= SMB358_CHG_OV_STATUS,
		.clr_en		= 1,
	},
};
struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			battery_missing;
	const char		*bms_psy_name;	
	bool			resume_completed;
	bool			irq_waiting;
	struct mutex		read_write_lock;

	/* status tracking */
	bool			batt_full;
	bool			batt_full_flag;
    bool            batt_recharging_state;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	int			charging_disabled_status;

	int			charging_disabled;
	int			fastchg_current_max_ma;
    int			chg_current_ma;
	int			workaround_flags;

	int			chg_input_cur_limit_max;
	int			chg_fastchg_cur_limit_max;
	int			fb_on_cur_limit_max;

	struct delayed_work timer_work;
	int the_timer;
	bool term_reg_flag;
	bool charger_enable_soft_term;
	int soft_term_capacity;
	int soft_term_timer;

	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
    struct power_supply	*qpnp_bat_psy;
	struct power_supply	batt_psy;
    struct power_supply		dc_psy;

	struct smb358_regulator	otg_vreg;
	struct mutex		irq_complete;

	struct dentry		*debug_root;
	u32			peek_poke_address;
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			bat_present_decidegc;
	struct regulator	*vcc_i2c;
	int			irq_gpio;
    long		status;
    struct wake_lock	chg_wake_lock;
	struct wake_lock	unplug_wake_lock;
	struct wake_lock	plug_wake_lock;
    int soc;
    int ui_soc;
	int cal_soc;
    int         usb_psy_type;
    long		psy_status;
    int 	chg_phase;
    unsigned long    charge_begin;
    int			chg_tmout_mins;
    struct mutex		set_cur_irq;
    struct mutex		uv_irq;
    struct mutex		apsd_irq;
	bool  full_flag;
    int empty_enabled;
	unsigned long bat_vol;
    int dcp_flag;
	int charger_psy_type;
	int fast_chg_flag;
	int usb_dataline_connect;
	int batt_1C_current_ma;
	const char * project_name;
	bool	use_gpio_control_charger;
	int		charger_enable_gpio;
};
struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb358_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

struct chg_current_map {
	int	chg_current_ma;
	u8	value;
};

static int chg_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};
int pa_leaked_restore_mode = 0;
static bool bbk_cmcc_attribute = false;
extern  unsigned int is_atboot;

/* add supplied to "bms" function */
static char *pm_batt_supplied_to[] = {
	"bms",
};
static char *pm_power_supplied_to[] = {
	//"battery",
	  "cms",
};
static void dump_regs(struct smb358_charger *chip);
static int smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}
static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		pr_err("Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);
    
	return rc;
}

static int smb358_fastchg_current_set(struct smb358_charger *chip, int fastchg_current)
{
	int i;
	u8 value = 0;

	if ((fastchg_current < SMB358_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB358_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
					fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dmA\n",
					fastchg_current);
		i = 0;
	}

	//i = i << SMB358_FAST_CHG_SHIFT;
	value = (i << SMB358_FAST_CHG_SHIFT)&SMB_FAST_CHG_CURRENT_MASK;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
			fastchg_current, value);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, value);
}
static void smb358_current_limit_set(struct smb358_charger *chip,int current_ma)
{
	int i;
	u8 value,rc;
	value = 0;
	
	for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
		if (chg_current[i] <= current_ma)
			break;
	}
	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
		i = 0;
	}
	
	//i = i << 4;
	value = (i<<4)&0xf0;
	pr_err("current_ma:%d, value:%d\n", current_ma, value);
	rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					AC_CHG_CURRENT_MASK, value);
	if (rc)
		dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);

	return;
}
static int  smb358_recharging_state(struct smb358_charger *chip)
{
	int rc = 0;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}
	//pr_debug("smb358_recharging_state,IRQ_C_REG = %d !\n",reg);

    if(reg & STATUS_INT_RECHGING_BIT){
        //chip->batt_full = false;
        //chip->batt_recharging_state = true;
        return true;
    }else{
        //chip->batt_recharging_state = false; 
        return false;
    }

	return (chip->batt_full == true || chip->batt_recharging_state == true);
}
#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
#define VFLOAT_4370MV		4370

static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (VFLOAT_4370MV == vfloat_mv)
		temp = 0x2C;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV - 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);
	pr_warn("enable\n");
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);
	pr_warn("disable\n");
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};
static int smb358_chg_otg_disable(struct smb358_charger *chip)
{
	int rc = 0;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	return rc;
}
static int smb358_chg_otg_enable(struct smb358_charger *chip)
{
	int rc = 0;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
	return rc;

}
static int smb358_set_otg_vbus(struct smb358_charger *chip, int scope)
{
    int on = 0;
    on = scope == POWER_SUPPLY_SCOPE_SYSTEM ? 1 : 0;

    if(on)
        smb358_chg_otg_enable(chip);
    else
        smb358_chg_otg_disable(chip);

    return on;
}
static int smb358_regulator_init(struct smb358_charger *chip)
{
	int rc = 0;

	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};	

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}

	}

	return rc;
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_charging_disable(struct smb358_charger *chip,int reason, int disable);						
static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	/* setup defaults for CHG_CNTRL_REG */
	reg = 0x0;//CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK | CHG_CTRL_AICL_BEHAVIOR_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup defaults for PIN_CTRL_REG */
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK | CHG_PIN_CTRL_PRE_BIAS_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup USB suspend and APSD  */
	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd){
		reg = CHG_CTRL_APSD_EN_BIT;
	rc = smb358_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	    }
    }

	/* Fault and Status IRQ configuration */

	reg = FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		  | FAULT_INT_INPUT_OV_BIT;		
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}

	reg = STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHGING_BIT |
	STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
	/* setup THERM Monitor */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_DISABLE_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* setup freq */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		SWITCH_FREQUENCY_EN_MASK, SWITCH_FREQUENCY_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* disable complete charge timeout */
	rc = smb358_masked_write(chip, STAT_AND_TIMER_CTRL_REG,
		STAT_AND_TIMER_CTRL_COMPLTE_CHARGE_TIMEOUT, STAT_AND_TIMER_CTRL_COMPLTE_CHARGE_TIMEOUT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STAT_AND_TIMER_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* set the fast charge current limit */
	rc = smb358_fastchg_current_set(chip,chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
		}

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else{
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->recharge_disabled)
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
		return rc;
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv >= 300)
			reg = VFLT_300MV;
		else if (200 <= chip->recharge_mv && chip->recharge_mv < 300)
			reg = VFLT_200MV;
		else if (100 <= chip->recharge_mv && chip->recharge_mv < 200)
			reg = VFLT_100MV;
		else if (50 <= chip->recharge_mv && chip->recharge_mv < 100)
			reg = VFLT_50MV;
		else
			reg = VFLT_50MV;

		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	/* enable/disable charging */
	rc = smb358_charging_disable(chip, USER, !!chip->charging_disabled);
	if (rc)
		dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
			
    rc = smb358_masked_write(chip, OTG_TLIM_THERM_CNTRL_REG,
					CHG_OTG_CUR_MASK, CHG_OTG_CUR_900MA);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set otg reg rc = %d\n", rc);
		return rc;
	}	

	return rc;
}

static int smb358_charger_pin_enable(struct smb358_charger *chip, bool enable)
{
	int rc = 0;
	if(enable){
		if(chip->charger_enable_gpio){
			gpio_direction_output(chip->charger_enable_gpio,0);
			gpio_direction_output(chip->charger_enable_gpio,1);
			pr_err("charger enable pin set high, enable charger\n");
		}
	}else{
		if(chip->charger_enable_gpio){
			gpio_direction_output(chip->charger_enable_gpio,0);
			gpio_direction_output(chip->charger_enable_gpio,0);
			pr_err("charger enable pin set low, disable charger\n");
		}
	}
	return rc;
}

static int __smb358_charging_disable(struct smb358_charger *chip, bool disable)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT diable = %d, rc = %d\n",
				disable, rc);
	return rc;
}


static bool smb358_is_charging(struct smb358_charger *chip)
{
	int rc;
	bool charging = 0;
	u8 temp = 0;
	
	rc = smb358_read_reg(chip, 	STATUS_C_REG, &temp);

	if (rc < 0)
		pr_err("Couldn't read charging\n");
	
	charging = temp & 0x01;
	return charging;

}

static int smb358_charging_disable(struct smb358_charger *chip,
						int reason, int disable)
{
	int rc = 0;
 /*
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_err("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;
  */
	if (!!disable == !!chip->charging_disabled_status)
	    goto skip;
	rc = __smb358_charging_disable(chip, !!disable);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
	/* update power_supply to let online status changed */
		power_supply_changed(chip->usb_psy);
	}

skip:
	chip->charging_disabled_status = disable;
	return rc;
}

static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};
static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};
static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	bool recharging = smb358_recharging_state(chip);

	if(recharging || (chip->batt_full_flag && chip->cal_soc >= 98)){
		return POWER_SUPPLY_STATUS_FULL;
	}

	if(chip->batt_full_flag && chip->cal_soc < 98){
		chip->batt_full = false;
		chip->batt_full_flag = false;
	}

	if(chip->batt_full && chip->ui_soc >= 100){
		chip->batt_full_flag = true;
		return POWER_SUPPLY_STATUS_FULL;
	}else if(chip->batt_full && chip->ui_soc < 100){
		return POWER_SUPPLY_STATUS_CHARGING;
	}

//	if (smb358_recharging_state(chip) == true && chip->cal_soc >= 96)
//		return POWER_SUPPLY_STATUS_FULL;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT))
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->qpnp_bat_psy)
		chip->qpnp_bat_psy = power_supply_get_by_name("qpnp-battery");
	if (chip->qpnp_bat_psy) {
		/* if battery has been registered, use the present property */
		chip->qpnp_bat_psy->get_property(chip->qpnp_bat_psy,
					POWER_SUPPLY_PROP_PRESENT, &ret);
       
		return ret.intval;
	}
	return 1;
}

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };
	static bool the_flag = true;
	static int count = 0;
	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if (!chip->chg_present) {
		if (count++ > 100)
			count = 0;
		if (count % 4 == 0)
			dump_regs(chip);
	}
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
        chip->soc= ret.intval;
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_AVG, &ret);
		chip->cal_soc = ret.intval;
        pr_err("soc=%d, ui_soc=%d, cal_soc:%d, bat_vol=%ld, chg_present=%d\n",chip->soc, chip->ui_soc, chip->cal_soc, chip->bat_vol, chip->chg_present);
        if(chip->batt_full == true)
        {
        	the_flag = false;
            chip->ui_soc = chip->soc;
            chip->full_flag = true;
            return chip->ui_soc;
        }
        chip->ui_soc = chip->soc;
        if(chip->chg_present){
			if(the_flag == true)
				if(chip->ui_soc == 100 && chip->full_flag == false)
					chip->full_flag = true;
			if(chip->ui_soc < 100){
				the_flag = false;
				chip->full_flag = false;
			}
            if(chip->ui_soc >=100 && !chip->full_flag)
                chip->ui_soc = 99;
        }else{
        	if(the_flag == true)
				if(chip->ui_soc == 100 && chip->full_flag == false)
					chip->full_flag = true;
            if(chip->ui_soc<100){
				the_flag = false;
                if(chip->full_flag==true)
                    chip->full_flag=false;
            }else if(chip->ui_soc>=100 && chip->full_flag==false){
                chip->ui_soc = 99;
            }
        }
		//add for factory test
		//if(!chip->chg_present){
			if(chip->bat_vol > 3750000 && !chip->ui_soc)
				return 50;
		//}
		//end
			
	    return chip->ui_soc;
	}
	dev_dbg(chip->dev, "%s: Couldn't get bms_psy,return default capacity\n",__func__);
	return SMB358_DEFAULT_BATT_CAPACITY;
}

static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;
    int chg_state = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	reg &= STATUS_C_CHARGING_MASK;

	if ((reg == STATUS_C_FAST_CHARGING) || (reg == STATUS_C_TAPER_CHARGING))
		chg_state = POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_PRE_CHARGING)
		chg_state = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		chg_state = POWER_SUPPLY_CHARGE_TYPE_NONE;

     if (chip->chg_phase != chg_state) 
     {
    	if (chg_state == POWER_SUPPLY_CHARGE_TYPE_NONE) {
    		pr_debug("Charging stopped.\n");
			
    	} else{
			pr_debug("Charging started.\n");
            chip->charge_begin = jiffies;
    	}
     }
     
    chip->chg_phase = chg_state;

    return chg_state;
}

static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

#define DEFAULT_TEMP 250
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (!smb358_get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);

	return (int)results.physical;
}

static int
smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int vbat_uv,i;
	int vbat_uv_sum = 0;
	for(i=0; i<3; i++){
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			chip->bat_vol = 0;
			pr_err("Unable to read vbat rc=%d\n", rc);
			return 0;
		}
		vbat_uv_sum += results.physical;
	}
	vbat_uv = vbat_uv_sum/3;
	chip->bat_vol = vbat_uv;
	return vbat_uv;
}
static int
smb358_get_prop_battery_health_status(struct smb358_charger *chip)
{
	unsigned long timeout; 
	if(chip->charge_begin > 0)
	{
		timeout= chip->charge_begin + chip->chg_tmout_mins * 60 * HZ;
		if(chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE &&
			time_after(jiffies, timeout)){
			if(bbk_cmcc_attribute)
				pr_warn("ignore time out for ccmc\n");//ignore for ccmc
			else
				set_bit(SMB358_CHG_TIMEOUT_STATUS, &chip->psy_status);
		}

		if(chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE){
			pr_err("charging runs %d seconds,status=%ld,chg_tmout_mins=%d\n", jiffies_to_msecs(jiffies-chip->charge_begin)/ 1000,chip->psy_status,chip->chg_tmout_mins);
		}
	}

	return chip->psy_status;
}
static void dump_regs(struct smb358_charger *chip);
static void smb358_chgin_check(struct smb358_charger *chip);
static int smb358_get_chg_type(struct smb358_charger *chip)
{   
    int rc; u8 reg = 0; 
    enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

    rc = smb358_read_reg(chip, STATUS_D_REG, &reg); 
    if (rc) {       
    dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);     
    return rc;  
    }   
    //pr_debug("STATUS_D_REG=%x\n",reg); 
    reg &= 0x0f;    
    switch (reg) {  
        case STATUS_D_PORT_CDP:     
        type = POWER_SUPPLY_TYPE_USB_CDP;       
        break;  
        case STATUS_D_PORT_DCP:     
        type = POWER_SUPPLY_TYPE_USB_DCP;       
        break;  
        case STATUS_D_PORT_SDP:     
        type = POWER_SUPPLY_TYPE_USB;       
        break;  
		default:        
        type = POWER_SUPPLY_TYPE_USB_CDP;        
    }   
        return type;
}

static int smb358_select_current_by_icl(struct smb358_charger *chip, int type, int fast_current_max)
{
		int mA;
        
		if(POWER_SUPPLY_TYPE_USB == type){
			pr_info("charging with USB\n");
			mA = min(chip->chg_current_ma, 500);
			smb358_current_limit_set(chip, mA);
		}else if(POWER_SUPPLY_TYPE_USB_FLOATED == type){
			pr_info("charging with FLOAT CHARGER\n");
			if(bbk_cmcc_attribute)
				mA = min(chip->chg_current_ma, 500);
			else
				mA = min(chip->chg_current_ma, 900);
			smb358_current_limit_set(chip, mA);
		}else if(POWER_SUPPLY_TYPE_USB_CDP == type){
			pr_info("charging with USB_CDP\n");
			mA = min(chip->chg_current_ma, chip->chg_input_cur_limit_max);
			smb358_current_limit_set(chip, mA);
		}else if(POWER_SUPPLY_TYPE_USB_DCP == type){
			pr_info("charging with USB_DCP\n");
			mA = min(chip->chg_current_ma, chip->chg_input_cur_limit_max);
			smb358_current_limit_set(chip, mA);
		}else{
		   return -1;
		}
		/*because input the smallest 300ma, so if it smaller than 300ma, set ibat to 200ma*/
		if(mA < 300 && mA > 0)
			fast_current_max = 200;
		smb358_fastchg_current_set(chip,fast_current_max);
		return 0;
}
static int smb358_select_current_by_cc(struct smb358_charger *chip, int type, int limit_current_max)
{
		int mA;
        
		smb358_current_limit_set(chip,limit_current_max);
		
		if(POWER_SUPPLY_TYPE_USB == type || POWER_SUPPLY_TYPE_USB_FLOATED == type){
			pr_info("charging with USB\n");
			mA = min(chip->chg_current_ma, 500);
			smb358_current_limit_set(chip, mA);
			smb358_fastchg_current_set(chip, limit_current_max);
		}else if(POWER_SUPPLY_TYPE_USB_CDP == type){
			pr_info("charging with USB_CDP\n");
			mA = min(chip->chg_current_ma, chip->chg_input_cur_limit_max);
			smb358_fastchg_current_set(chip, mA);
			
		}else if(POWER_SUPPLY_TYPE_USB_DCP == type){
			pr_info("charging with USB_DCP\n");
			mA = min(chip->chg_current_ma, chip->chg_input_cur_limit_max);
			smb358_fastchg_current_set(chip, mA);
		}else{
		   return -1;
		}

		return 0;
}
static int smb358_wait_chg_status(struct smb358_charger *chip, int chg)
{
	int retry = 0;
	for(retry = 0; retry < 15; retry++){
		if (!chip->chg_present) {
			pr_warn("usb not present\n");
			return 0;
		}
		smb358_get_prop_charge_type(chip);
		pr_err("chg=%d,chg_phase=%d,retry=%d\n",chg,chip->chg_phase,retry);
		if(chg &&  chip->chg_phase > POWER_SUPPLY_CHARGE_TYPE_NONE)
			return 0;
		if(!chg && chip->chg_phase == POWER_SUPPLY_CHARGE_TYPE_NONE)
			return 0;
		msleep(200);
	}
	pr_err("wait %s status timeout!\n", chg ? "chg" : "none-chg");
	return -ETIME;
}
static int smb358_set_usb_chg_current(struct smb358_charger *chip,
		int chg_curr_now_ma)
{
	int usb_psy_type,rc;
	int reg1 = 0,reg2 = 0,mask = 0;
	bool enable = true;
	union power_supply_propval charger_type = {0,};
	reg1 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	reg2 |= CMD_B_CHG_HC_ENABLE_BIT;
	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	mutex_lock(&chip->set_cur_irq);
	if(chip->usb_dataline_connect){
		if(chip->dcp_flag >= 1 && !chip->fast_chg_flag){
			chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_CHG_TYPE, &charger_type);
			usb_psy_type = charger_type.intval;
			if(usb_psy_type <= 1){
				usb_psy_type = POWER_SUPPLY_TYPE_USB;
			}
			if(is_atboot){
				usb_psy_type = smb358_get_chg_type(chip);
			}
			chip->charger_psy_type = usb_psy_type;
			pr_err("dcp_flag=%d,usb_psy_type=%d,fast_chg_flag=%d\n", chip->dcp_flag,usb_psy_type,chip->fast_chg_flag);
		}else{
			usb_psy_type = smb358_get_chg_type(chip);
			chip->charger_psy_type = usb_psy_type;
			if(chip->charger_psy_type == POWER_SUPPLY_TYPE_USB_CDP || chip->charger_psy_type == POWER_SUPPLY_TYPE_USB_DCP){
				chip->fast_chg_flag = 1;
			}else{
				chip->fast_chg_flag = 0;
			}
			pr_err("dcp_flag=%d,358_usb_psy_type=%d,fast_chg_flag=%d\n", chip->dcp_flag,usb_psy_type,chip->fast_chg_flag);
		}
	}
	else{
		chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_CHG_TYPE, &charger_type);
			usb_psy_type = charger_type.intval;
		pr_err("usb type:%d\n", usb_psy_type);
		if(usb_psy_type == POWER_SUPPLY_TYPE_UNKNOWN){
			if(is_atboot){
				usb_psy_type = POWER_SUPPLY_TYPE_USB_DCP;
			}else{
				usb_psy_type = POWER_SUPPLY_TYPE_USB;
			}
		}
		chip->charger_psy_type = usb_psy_type;
	}
	smb358_enable_volatile_writes(chip);
	chip->chg_current_ma = chg_curr_now_ma;

	if(!chip->chg_current_ma)
		enable = false;

	if(chip->chg_current_ma == chip->fb_on_cur_limit_max){//FB_ON_CURRENT 900
		rc = smb358_masked_write(chip, CMD_B_REG, mask, reg1);
		rc = smb358_select_current_by_cc(chip, usb_psy_type, chip->chg_fastchg_cur_limit_max);
		if(rc < 0){
		clear_bit(SMB358_CHG_TIMEOUT_STATUS, &chip->psy_status);
		enable = false;
		}
	}else{
		rc = smb358_masked_write(chip, CMD_B_REG, mask, reg1);
		rc = smb358_select_current_by_icl(chip, usb_psy_type, chip->chg_fastchg_cur_limit_max);
		if(rc<0){
		clear_bit(SMB358_CHG_TIMEOUT_STATUS, &chip->psy_status);
		enable = false;
		}
	}
	pr_err("chip->chg_current_ma=%d\n",chip->chg_current_ma);
	if(enable){
		rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);
		if(!chip->usb_dataline_connect){
			rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG,0x10, 0x00);
			if (rc < 0)
				dev_err(chip->dev, "Couldn't set register control rc = %d\n", rc);
		}

        if(!chip->empty_enabled)
        {
    		//unset the susp bit here 
    		rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, 0);
    		if (rc < 0)
    			dev_err(chip->dev, "Couldn't disable suspend rc = %d\n", rc);
            
    		smb358_charging_disable(chip,USER, 0);
        }		
	}else{
		smb358_charging_disable(chip,USER, 1);
	}
	//smb358_get_prop_charge_type(chip);
	smb358_wait_chg_status(chip, !chip->charging_disabled_status);
    dump_regs(chip);
    mutex_unlock(&chip->set_cur_irq);
    return chip->chg_current_ma;
}


static void smb358_timer_work(struct work_struct *work)
{
	int work_period_ms = msecs_to_jiffies(10000);

	struct smb358_charger *chip = container_of(work,
					struct smb358_charger,
					timer_work.work);

	if(chip->the_timer > 0)
		chip->the_timer--;
	pr_err("### discharging ### %d \n",chip->the_timer);

	if(chip->soc >= chip->soft_term_capacity && chip->the_timer > 0){
		schedule_delayed_work(&chip->timer_work,
			round_jiffies_relative(work_period_ms));
	}else if(chip->soc < chip->soft_term_capacity){
		chip->the_timer = 0;
	}
}

static int chg_term(struct smb358_charger *chip, u8 status);
static int smb358_is_termination(struct smb358_charger *chip)
{
	int rc = 0;
	u8 rt_stat = 0;
	static u8 prev_rt_stat = 0;

	rc = smb358_read_reg(chip, IRQ_C_REG, &rt_stat);	
	if ((prev_rt_stat ^ rt_stat) && (rt_stat & IRQ_C_TERM_BIT)) {
		pr_warn("charging terminate(not irq)\n");
		chg_term(chip,1);
		chip->term_reg_flag = true;
		cancel_delayed_work_sync(&chip->timer_work);
		chip->the_timer = 0;
		smb358_get_prop_charge_type(chip);
		power_supply_changed(&chip->batt_psy);
	}
	prev_rt_stat = rt_stat;

	return 0;
}
static int smb358_is_termination_of_soft(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };
	int soc = 0;

	if(smb358_is_charging(chip)){
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
			soc = ret.intval;
			if(soc >= chip->soft_term_capacity){
				cancel_delayed_work_sync(&chip->timer_work);
				pr_err("### charging ### %d \n",chip->the_timer);
				if(chip->the_timer++ >= chip->soft_term_timer){
					chip->the_timer--;
					pr_err("### charging terminate(not smb358) ###\n");
					msleep(5000);
					chg_term(chip,1);
					power_supply_changed(&chip->batt_psy);
				}
			}
		}
	}else{
	}
	return 0;
}
extern char* get_bbk_board_version(void);
static int smb358_check_vbat_max(struct smb358_charger *chip)
{
	union power_supply_propval val = {0,};
	static int mv = 0;
	int rc;
	char *board_version = NULL;

	if(!strncmp(chip->project_name, "PD1505", 6)){
		board_version = get_bbk_board_version();
		if(!board_version){
			pr_err("failed to get board_version\n");
		}else{
			pr_err("get board_version %s\n",board_version);
			if(board_version[0] == '1' && board_version[1] == '1'){
				mv = 4350;
				pr_err("%s set float voltage %d\n",chip->project_name,mv);
			}
		}
	}

	if(mv != chip->vfloat_mv){
		if(!chip->bms_psy){
			pr_err("### bms not found! ###\n");
			return -1;
		}else{
			chip->bms_psy->get_property(chip->bms_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
			pr_err("#### get battery max_voltage_uv : %d ####\n", val.intval);
			if(val.intval == 4350)
				mv = 4350;
			else if(val.intval == 4380 || val.intval == 4400)
				mv = 4370;
			else
				mv = val.intval;

			if(mv != 0){
				pr_err("### final set vbat = %d ###\n",mv);
				/* set the float voltage */
				rc = smb358_float_voltage_set(chip, mv);
				if (rc < 0) {
					dev_err(chip->dev,"Couldn't set float voltage rc = %d\n", rc);
					return rc;
				}
				chip->vfloat_mv = mv;
				return 1;
			}
		}
	}
	return 0;
}

static int smb358_set_chg_current_now(struct smb358_charger *chip,
		int current_ma)
{
	union power_supply_propval charger_type = {0,};
    if(current_ma < 0){
		pr_err("invalid current %d\n", current_ma);
		return 0;
	}
    smb358_chgin_check(chip);
	//if(current_ma >= 1000)
	//	current_ma = chip->chg_input_cur_limit_max;
	/*limit the current for 700mA adapter*/
	if(current_ma > chip->chg_input_cur_limit_max)
		current_ma = chip->chg_input_cur_limit_max;
    if(chip->dcp_flag < 5)
        chip->dcp_flag += 1;
    else
        chip->dcp_flag = 5;
	chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_CHG_TYPE, &charger_type);
	pr_err("dcp_flag=%d,chip->chg_current_ma=%d,current_ma=%d,charger_psy_type=%d,charger_type=%d,fast_chg_flag=%d\n",
		chip->dcp_flag,chip->chg_current_ma,current_ma,chip->charger_psy_type,charger_type.intval,chip->fast_chg_flag);
	if(chip->usb_dataline_connect){
		if(chip->chg_current_ma != current_ma || (chip->charger_psy_type != charger_type.intval && chip->dcp_flag >= 1 && !chip->fast_chg_flag)){
			chip->chg_current_ma = current_ma;
		    smb358_set_usb_chg_current(chip, current_ma);
			pr_debug("current_ma=%d\n", current_ma);
		}
	}
	else{
		if(chip->chg_current_ma != current_ma || (chip->charger_psy_type != charger_type.intval)){
			chip->chg_current_ma = current_ma;
			pr_debug("current_ma=%d\n", current_ma);
		    smb358_set_usb_chg_current(chip, current_ma);
		}
	}
	dump_regs(chip);
	if (!chip->term_reg_flag)
		smb358_is_termination(chip);
	if(chip->charger_enable_soft_term){
		if (!chip->batt_full)
			smb358_is_termination_of_soft(chip);
	}
	smb358_check_vbat_max(chip);
	if(chip->chg_present){
        if (!wake_lock_active(&chip->chg_wake_lock)){
			pr_debug("enable charger get chg_wake_lock\n");
        	wake_lock(&chip->chg_wake_lock);
		}

	}else{
		if (wake_lock_active(&chip->chg_wake_lock)){
			pr_debug("disable charger release chg_wake_lock\n");
			wake_unlock(&chip->chg_wake_lock);
		}	
	}
    return 0;
}

static int
get_prop_current_now(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0,};
	int current_now = 0;
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		current_now = ret.intval;
		return abs(current_now);
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}
static int
get_prop_current_input_max(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0,};
	/*
	struct power_supply *qpnp_bat_psy = power_supply_get_by_name("qpnp-battery");
	if (qpnp_bat_psy) {
		qpnp_bat_psy->get_property(qpnp_bat_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &ret);

		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}
	
	return 0;
	*/
	return ret.intval;
}

static int smb358_get_prop_current_input_max(struct smb358_charger *chip)
{
	return chip->chg_current_ma * 1000;
}
static bool smb358_is_charger_present(struct smb358_charger *chip)
{
	union power_supply_propval dc_in = {0,};
    if(test_bit(SMB358_CHG_OV_STATUS, &chip->psy_status))
        return 1;

	chip->usb_psy->get_property(chip->usb_psy, POWER_SUPPLY_PROP_DC_RESENT, &dc_in);
	
	return dc_in.intval;
}
static void set_prop_charging_empty(struct smb358_charger *chip, 
	int enabled)
{
	int empty_enabled = !!enabled;
	pr_info("set charge empty:%d\n", empty_enabled);
	if(empty_enabled){
        chip->empty_enabled = empty_enabled;
		smb358_masked_write(chip, CMD_A_REG, BIT(7), BIT(7));
        /*disable charging*/
        smb358_charging_disable(chip,USER, 1);
		/*set smb358 to suspend mode*/
		smb358_masked_write(chip, CMD_A_REG, BIT(2), BIT(2));
	} else {
	    chip->empty_enabled = empty_enabled;
		smb358_masked_write(chip, CMD_A_REG, BIT(7), BIT(7));
		smb358_masked_write(chip, CMD_A_REG, BIT(2), 0);
        smb358_charging_disable(chip,USER, 0);
	}
}
static int
smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}

	return 0;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb358_charging_disable(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SCOPE:
	    smb358_set_otg_vbus(chip, val->intval);
	    break;
	case POWER_SUPPLY_PROP_DC_CUTOFF:
		set_prop_charging_empty(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb358_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        //!(chip->charging_disabled_status & USER);
		val->intval = chip->charging_disabled_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb358_get_prop_battery_voltage_now(chip);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	    val->intval = chip->vfloat_mv * 1000;
	    break;
    case POWER_SUPPLY_PROP_DC_RESENT:
        val->intval = chip->chg_present;
        break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = get_prop_current_input_max(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_full;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int smb358_dc_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, dc_psy);

	switch (prop) {
    case POWER_SUPPLY_PROP_CURRENT_NOW:
 		smb358_set_chg_current_now(chip, val->intval);
        break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pr_debug("set bbk_cmcc_attribute=%d\n",val->intval);
		bbk_cmcc_attribute = !!val->intval;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int smb358_dc_get_property_mains(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb358_is_charger_present(chip);
		break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = smb358_get_prop_battery_health_status(chip);
        break;
    case POWER_SUPPLY_PROP_CURRENT_MAX:
        val->intval = chip->batt_1C_current_ma;
        break;
    case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
        val->intval = smb358_get_prop_current_input_max(chip);
        break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		pr_debug("get bbk_cmcc_attribute=%d\n",bbk_cmcc_attribute);
		val->intval = bbk_cmcc_attribute;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
//wangyong add for USB connect Noise  1:connect 0:disconnect
void charger_connect_judge(char on_or_off);//QIUGUIFU ADD for 8926
static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	/*
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

    struct power_supply *usb_psy = power_supply_get_by_name("usb");
    if(usb_psy == NULL){
    printk(KERN_ERR "%s: usb is null\n", __func__);
    return 0;
    }	
	*/
	static int flag = false;
	charger_connect_judge(1);//qiuguifu add 
	if (chip->disable_apsd || status == 0) {
		dev_dbg(chip->dev, "APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}
    mutex_lock(&chip->apsd_irq);	
/*
	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}


	reg &= 0x0f;
	switch (reg) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;
    chip->usb_psy_type = type;

	pr_debug("APSD complete. USB type detected=%d chg_present=%d",
						type, chip->chg_present);

	power_supply_set_charge_type(usb_psy, type);
*/	
	chip->chg_present = !!status;
	 /* SMB is now done sampling the D+/D- lines, indicate USB driver */
	pr_err("usb_psy present=%d",chip->chg_present);
	if(!flag && chip->chg_present){
		power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);
		flag = true;
	}
	power_supply_set_present(chip->usb_psy, chip->chg_present);
	if(chip->chg_present){
		if (wake_lock_active(&chip->plug_wake_lock)){
			pr_debug("release plug_wake_lock\n");
			wake_unlock(&chip->plug_wake_lock);
		}
		wake_lock_timeout(&chip->plug_wake_lock,1*HZ);
		pr_debug("get chg_wake_lock\n");
	}
	if (pa_leaked_restore_mode) {
		mutex_unlock(&chip->apsd_irq);
		return 0;
	}
    chip->chg_current_ma = chip->chg_input_cur_limit_max;
	smb358_set_usb_chg_current(chip, chip->chg_current_ma);
	pr_debug("%s updating batt psy\n", __func__);
    mutex_unlock(&chip->apsd_irq);
	return 0;
}
static int chg_uv(struct smb358_charger *chip, u8 status)
{
	/* use this to detect USB insertion only if !apsd */
	mutex_lock(&chip->uv_irq);
	if (chip->disable_apsd && status == 0) {

		chip->chg_present = true;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
		power_supply_changed(chip->usb_psy);
		power_supply_changed(&chip->dc_psy);
        chip->chg_current_ma = chip->chg_input_cur_limit_max;
	    smb358_set_usb_chg_current(chip, chip->chg_current_ma);
		power_supply_changed(&chip->batt_psy);
	}
	if (status != 0) {
		charger_connect_judge(0);//qiuguifu add
		chip->chg_present = false;
        chip->batt_full = false;
		chip->batt_full_flag = false;
        chip->empty_enabled = false;
		chip->batt_recharging_state = false;
		chip->term_reg_flag = false;
        chip->dcp_flag = 0;
		chip->charger_psy_type = 0;
		chip->fast_chg_flag = 0;
		clear_bit(SMB358_CHG_TIMEOUT_STATUS, &chip->psy_status);
		//fake bat_full clear the_timer
		if(chip->charger_enable_soft_term){
			if(chip->soc >= chip->soft_term_capacity && chip->the_timer > 0)
				schedule_delayed_work(&chip->timer_work, 0);
			else
				chip->the_timer = 0;
		}
	    /* we can't set usb_psy as UNKNOWN here, will lead USERSPACE issue */
        pr_err("usb_psy absent=%d\n",chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
		power_supply_changed(chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
	}

	if (!status) {
        if (!wake_lock_active(&chip->chg_wake_lock)){
			pr_debug("get chg_wake_lock\n");
        	wake_lock(&chip->chg_wake_lock);
		}
	}else{
		if (wake_lock_active(&chip->chg_wake_lock)){	
			if (wake_lock_active(&chip->unplug_wake_lock)){
				pr_debug("release unplug_wake_lock\n");
				wake_unlock(&chip->unplug_wake_lock);
			}
			wake_lock_timeout(&chip->unplug_wake_lock, 2*HZ);
			pr_debug("release chg_wake_lock\n");
			wake_unlock(&chip->chg_wake_lock);
		}
    }
	dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);
    mutex_unlock(&chip->uv_irq);
	return 0;
}
static void smb358_chgin_check(struct smb358_charger *chip)
{
    u8 reg,rc;
	bool power_ok;
	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		return;
	}
	power_ok = !(reg & IRQ_E_INPUT_UV_BIT);

	if (power_ok ^ chip->chg_present)
    {
        if(power_ok)
        {
            chg_uv(chip, 0);
			apsd_complete(chip, 1);
    	} else{ 
    		chg_uv(chip, 1);
		}
    }

    return;
}
static int chg_ov(struct smb358_charger *chip, u8 status)
{
	/* disable charging? */
	u8 psy_health_sts;
	if (status)
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	power_supply_set_health_state(
				chip->usb_psy, psy_health_sts);
	power_supply_changed(chip->usb_psy);

	return 0;
}

static int fast_chg(struct smb358_charger *chip, u8 status)
{
	//power_supply_changed(&chip->batt_psy);
	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	chip->batt_full = !!status;
    pr_err("%s:batt_full=%d\n", __func__,chip->batt_full);
	if(chip->batt_full)
		chip->batt_recharging_state = true;
		
	return 0;
}

#define HYSTERISIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0;
	int temp;
    return;
	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb358_get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp > chip->hot_bat_decidegc) {
			/* Normal to hot */
			bat_hot = true;
			bat_cold = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
			/* shall we need add high_temp here? */
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >
			chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC) {
			/* Cool to normal */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;

			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp > chip->bat_present_decidegc) {
			/* Present to cold */
			bat_hot = false;
			bat_cold = true;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			/* Cold to present */
			bat_cold = true;
			bat_hot = false;
			bat_present = false;
			chip->adc_param.high_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (chip->bat_present_decidegc < temp &&
				temp < chip->cold_bat_decidegc) {
			/* Normal to cold */
			bat_hot = false;
			bat_cold = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <
				chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC) {
			/* Warm to normal */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;

			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if (bat_hot || bat_cold || chip->battery_missing)
			smb358_charging_disable(chip, THERMAL, 1);
		else
			smb358_charging_disable(chip, THERMAL, 0);
	}

	pr_debug("hot %d, cold %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
			chip->batt_hot, chip->batt_cold, chip->battery_missing,
			chip->adc_param.low_temp, chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

/* hot/cold function must be implemented as adc notification */
/* if using PMIC btm part, can omit these, because no irq will generate */
static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	if (status)
		chip->battery_missing = true;
	else
		chip->battery_missing = false;

	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_hot",
			},
			{
				.name		= "vbat_low",
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_term",
				.smb_irq	= chg_term,
			},
			{
				.name		= "taper",
			},
			{
				.name		= "recharge",
			},
			{
				.name		= "fast_chg",
				.smb_irq	= fast_chg,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
			},
			{
				.name		= "aicl_complete",
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "usbin_uv",
				.smb_irq        = chg_uv,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
			},
			{
				.name		= "unknown",
			},
			{
				.name		= "unknown",
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
			},
			{
				.name		= "otg_det",
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat,reg_val;
	int rc;
	int handler_count = 0;
    unsigned long val;

	mutex_lock(&chip->irq_complete);	
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_warn(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;
	
	for(i = 0; i < ARRAY_SIZE(irq_bits_lut); i++){
		rc = smb358_read_reg(chip, irq_bits_lut[i].reg, &reg_val);
        val = reg_val;
		if(val >= 0  && test_bit(irq_bits_lut[i].reg_bit, &val))
			set_bit(irq_bits_lut[i].sta_bit, &chip->psy_status);
		else if(val >= 0 && irq_bits_lut[i].clr_en)
			clear_bit(irq_bits_lut[i].sta_bit, &chip->psy_status);
	}

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed){
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;
				pr_debug("handler count = %d name=%s \n", handler_count,handlers[i].irq_info[j].name);
			}

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	dev_dbg(chip->dev, "%s: chg_present = %d\n", __func__, present);
	if (present != chip->chg_present) {
		chip->chg_present = present;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	return IRQ_HANDLED;
}

static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = container_of(psy, struct smb358_charger, batt_psy);
	if (chip->bms_psy_name){
		if(!chip->bms_psy)
			chip->bms_psy = power_supply_get_by_name((char *)chip->bms_psy_name);
	}

}

static ssize_t Show_CMCC_Attribute(struct device *dev,struct device_attribute *attr, char *buf)
{

	pr_err("[CMCC_Attribute] Show_CMCC_Attribute,bbk_cmcc_attribute=%d\n", bbk_cmcc_attribute);
	return sprintf(buf, "%d\n", bbk_cmcc_attribute);
}

static ssize_t Store_CMCC_Attribute(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{

	if(buf!=NULL && size!=0 && strstr(buf, "LG4")!= NULL)
		bbk_cmcc_attribute = true;
	else
		bbk_cmcc_attribute = false;

	pr_err("[CMCC_Attribute] Store_CMCC_Attribute ,bbk_cmcc_attribute =%d\n",bbk_cmcc_attribute);

	return size;
}

static DEVICE_ATTR(CMCC_Attribute,0664,Show_CMCC_Attribute,Store_CMCC_Attribute);
static struct attribute *cmcc_attrs[] = {
	&dev_attr_CMCC_Attribute.attr,
	NULL
};
static const struct attribute_group cmcc_attr_group = {
	.attrs = cmcc_attrs,
};


#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb358_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb358_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb358_charger *chip = data;

	smb358_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

#if 0
static void dump_regs(struct smb358_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct smb358_charger *chip)
{
    int rc;
    u8 reg;
	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
    pr_err("0x30 = 0x%02x\n", reg);
	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
    pr_err("0x37 = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, IRQ_D_REG, &reg);
    pr_err("0x38 = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
    pr_err("0x39 = 0x%02x\n", reg);
	rc = smb358_read_reg(chip, IRQ_F_REG, &reg);
    pr_err("0x3A = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, STATUS_B_REG, &reg);
    pr_err("0x3C = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
    pr_err("0x3D = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
    pr_err("0x3E = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, STATUS_E_REG, &reg);
    pr_err("0x3F = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, CHG_CURRENT_CTRL_REG, &reg);
    pr_err("0x00 = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
    pr_err("0x01 = 0x%02x\n", reg);
    rc = smb358_read_reg(chip, VARIOUS_FUNC_REG, &reg);
    pr_err("0x02 = 0x%02x\n", reg);
	rc = smb358_read_reg(chip, VFLOAT_REG, &reg);
    pr_err("0x03 = 0x%02x\n", reg);
	rc = smb358_read_reg(chip, CHG_CTRL_REG, &reg);
    pr_err("0x04 = 0x%02x\n", reg);
	rc = smb358_read_reg(chip, 0x0A, &reg);
    pr_err("0x0A = 0x%02x\n", reg);
}
#endif

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charging-disabled");
	if (pa_leaked_restore_mode) {
		chip->charging_disabled = 1;
	}
	pr_warn("pa_leaked_restore_mode = %d,charging_disabled=%d\n",
			pa_leaked_restore_mode,chip->charging_disabled);
	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	//soft termination
	chip->charger_enable_soft_term = of_property_read_bool(node,
					"vivo,charger-enable-soft-term");
	rc = of_property_read_u32(node, "vivo,soft-term-capacity", &chip->soft_term_capacity);
	if (rc < 0)
		chip->soft_term_capacity = 99;
	rc = of_property_read_u32(node, "vivo,soft-term-timer", &chip->soft_term_timer);
	if (rc < 0)
		chip->soft_term_timer = 120;

	//project name
	rc = of_property_read_string(node, "vivo,project-name",
							&(chip->project_name));
	if(rc){
		pr_debug("vivo,project-name property do not find\n");
		chip->project_name = "default";
	}

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		dev_dbg(chip->dev, "Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,chg-input-cur-limit-max", &chip->chg_input_cur_limit_max);
	if (rc < 0)
		chip->chg_input_cur_limit_max = SMB358_CHG_CUR_DEFAULT_MA;

	rc = of_property_read_u32(node, "qcom,chg-fastchg-cur-limit-max", &chip->chg_fastchg_cur_limit_max);
	if (rc < 0)
		chip->chg_fastchg_cur_limit_max = SMB358_FASTCHG_CUR_LIMIT_DEFAULT_MA;

	rc = of_property_read_u32(node, "qcom,fb-on-cur-limit-max", &chip->fb_on_cur_limit_max);
	if (rc < 0)
		chip->fb_on_cur_limit_max = SMB358_FB_ON_CUR_DEFAULT_MA;

	rc = of_property_read_u32(node, "vivo,batt-1C-current-ma", &chip->batt_1C_current_ma);
	if (rc < 0)
		chip->batt_1C_current_ma = SMB358_FAST_CHG_MAX_MA;


	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold_bat_decidegc",
						&chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot_bat_decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,bat_present_decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	rc = of_property_read_u32(node, "qcom,chg-current-ma",
						&chip->chg_current_ma);
	if (rc < 0)
		chip->chg_current_ma = SMB358_CHG_CUR_DEFAULT_MA;
	rc = of_property_read_u32(node, "summit,chg-tmout-mins",
				   &(chip->chg_tmout_mins));
	if (rc<0) {
		pr_err("Unable to read chg_tmout_mins.\n");
	}

	chip->usb_dataline_connect = of_property_read_bool(node, "vivo,usb_data_line_connect");

	chip->use_gpio_control_charger = of_property_read_bool(node, "use-gpio-control-charger");
	if(chip->use_gpio_control_charger){
		chip->charger_enable_gpio= of_get_named_gpio(node, "charger-enable-gpio", 0);
		if (chip->charger_enable_gpio < 0) {
			pr_err("failed to get charger enable gpio\n");
		}
	}

	pr_debug("recharge_disabled = %d, recharge-mv = %d,",
			chip->recharge_disabled, chip->recharge_mv);
	pr_debug("vfloat_mv = %d, iterm-disabled = %d,",
			chip->vfloat_mv, chip->iterm_ma);
	pr_debug("input_current = %d, fastchg_current = %d, charging_disabled = %d,",
			chip->chg_input_cur_limit_max, chip->fastchg_current_max_ma,
					chip->charging_disabled);
	pr_debug("disable-apsd = %d bms = %s cold_bat_degree = %d,",
		chip->disable_apsd, chip->bms_psy_name,
					chip->cold_bat_decidegc);
	pr_debug("hot_bat_degree = %d, bat_present_decidegc = %d\n",
		chip->hot_bat_decidegc, chip->bat_present_decidegc);
	pr_debug("usb data line connect to smb358:%d\n", chip->usb_dataline_connect);
	pr_debug("use charger enable pin control:%d\n", chip->use_gpio_control_charger);
	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}
	/* Use PMIC BTM way to detect battery presence */
	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	/* For current design, can ignore this */
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine initial status\n");
	return rc;
}
static int dc_psy_register(struct smb358_charger *chip)
{
        int rc;
		chip->dc_psy.name = "smb-dc";
		chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
		chip->dc_psy.supplied_to = pm_power_supplied_to;
		chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
		chip->dc_psy.properties = pm_power_props_mains;
		chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
		chip->dc_psy.get_property = smb358_dc_get_property_mains;
		chip->dc_psy.set_property = smb358_dc_set_property;


        rc = power_supply_register(chip->dev, &chip->dc_psy);

        return rc;

}
#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000
static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;
	if (pa_leaked_restore_mode) {
		pr_warn("in pa leaked restroe mode,not probe smb358\n");
		return 0;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}
	//power_supply_set_supply_type(usb_psy,
					//POWER_SUPPLY_TYPE_UNKNOWN);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
    chip->charging_disabled_status = 1;
	chip->chg_present = false;
	chip->batt_recharging_state = false;
	chip->full_flag = false;
    chip->empty_enabled = false;
	chip->charge_begin = -1;
	/* early for VADC get, defer probe if needed */
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		return rc;
	}

	INIT_DELAYED_WORK(&chip->timer_work, smb358_timer_work);

	chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
	if (IS_ERR(chip->adc_tm_dev)) {
		rc = PTR_ERR(chip->adc_tm_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("adc_tm property missing\n");
		return rc;
	}

	/* i2c pull up Regulator configuration */
	chip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(chip->vcc_i2c)) {
		dev_err(&client->dev,
				"%s: Failed to get vcc_i2c regulator\n",
					__func__);
		rc = PTR_ERR(chip->vcc_i2c);
		goto err_get_vtg_i2c;
	}

	if (regulator_count_voltages(chip->vcc_i2c) > 0) {
		rc = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&client->dev,
				"regulator vcc_i2c set failed, rc = %d\n",
					rc);
			goto err_set_vtg_i2c;
		}
	}

	rc = regulator_enable(chip->vcc_i2c);
	if (rc) {
		dev_err(&client->dev,
			"Regulator vcc_i2c enable failed "
				"rc=%d\n", rc);
		return rc;
	}

	/* probe the device to check if its actually connected */
	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB358, device absent, rc = %d\n", rc);
		goto err_set_vtg_i2c;
	}

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		goto err_set_vtg_i2c;
	}

	i2c_set_clientdata(client, chip);

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb358_battery_get_property;
	chip->batt_psy.set_property	= smb358_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb358_batt_property_is_writeable;
	chip->batt_psy.properties	= smb358_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb358_battery_properties);
	chip->batt_psy.external_power_changed = smb358_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);

	chip->resume_completed = true;

	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
    mutex_init(&chip->set_cur_irq);
    mutex_init(&chip->uv_irq);
    mutex_init(&chip->apsd_irq);

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc = %d\n",
				rc);
		goto err_set_vtg_i2c;
	}

	if(chip->charger_enable_gpio){
		rc =  gpio_request_one(chip->charger_enable_gpio, GPIOF_DIR_OUT, "smb358 charger enable number");
		if (rc) {
			pr_err("failed to request data_gpio\n");
			goto err_charger_enable_gpio;
		}
		smb358_charger_pin_enable(chip, true);
	}

    rc = dc_psy_register(chip);
    if (rc < 0) {
        pr_err("power_supply_register dc failed rc=%d\n", rc);
        goto unregister_dc;
    }
	//dump_regs(chip);

	rc = smb358_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb358 ragulator rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}   

	rc = smb358_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't intialize hardware rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}
	
	// move wake_lock initial befor determine_initial_state - add by ccx
	wake_lock_init(&chip->chg_wake_lock,
		       WAKE_LOCK_SUSPEND, SMB358_NAME);
	wake_lock_init(&chip->unplug_wake_lock,
				WAKE_LOCK_SUSPEND, "smb358_unplug_timeout_lock");
	wake_lock_init(&chip->plug_wake_lock,
				WAKE_LOCK_SUSPEND, "smb358_plug_timeout_lock");
	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	/* We will not use it by default */
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
				irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}

	/* We will not use it by default */
	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node,
				"qcom,irq-gpio", 0, NULL);

	/* add some irq gpio get verify operation */
	if (gpio_is_valid(chip->irq_gpio)) {
		rc = gpio_request(chip->irq_gpio, "smb358_irq");
		if (rc)
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
		rc = gpio_direction_input(chip->irq_gpio);
		if (rc) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
		}
	}
    //client->irq = gpio_to_irq(chip->irq_gpio);
	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed STAT irq=%d request rc = %d\n",
				client->irq, rc);
			goto fail_chg_valid_irq;
		}
		enable_irq_wake(client->irq);
	}
	
	/* add hot/cold temperature monitor */
	chip->adc_param.low_temp = chip->cold_bat_decidegc;
	chip->adc_param.high_temp = chip->hot_bat_decidegc;
	chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
	chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	chip->adc_param.btm_ctx = chip;
	chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
	chip->adc_param.channel = LR_MUX1_BATT_THERM;

	/* update battery missing info in tm_channel_measure*/
	rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param);
	if (rc)
		pr_err("requesting ADC error %d\n", rc);

	chip->debug_root = debugfs_create_dir("smb358", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create cnfg debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create status debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create cmd debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create address debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create data debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create data debug file ent = %s\n",
				ent->d_iname);

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent || IS_ERR(ent))
			dev_err(chip->dev,
				"Couldn't create count debug file ent = %s\n",
				ent->d_iname);
	}

	//dump_regs(chip);
	rc = sysfs_create_group(&chip->dev->kobj,
					&cmcc_attr_group);
	if (rc)
		pr_err("cmcc attribute register fail\n");

	dev_info(chip->dev, "SMB358 successfully probed. charger=%d, batt=%d\n",
			chip->chg_present, smb358_get_prop_batt_present(chip));
	return 0;

fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_smb358_hw_init:
	power_supply_unregister(&chip->batt_psy);
	regulator_unregister(chip->otg_vreg.rdev);
unregister_dc:
    power_supply_unregister(&chip->dc_psy);
err_charger_enable_gpio:
	if (chip->charger_enable_gpio)
		gpio_free(chip->charger_enable_gpio);
err_set_vtg_i2c:
	if (regulator_count_voltages(chip->vcc_i2c) > 0)
		regulator_set_voltage(chip->vcc_i2c, 0, SMB_I2C_VTG_MAX_UV);

err_get_vtg_i2c:
	regulator_put(chip->vcc_i2c);

	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	regulator_disable(chip->vcc_i2c);
	regulator_put(chip->vcc_i2c);
    
    mutex_destroy(&chip->set_cur_irq);
    mutex_destroy(&chip->uv_irq);
    mutex_destroy(&chip->apsd_irq);
	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	pr_debug("come in \n");

	rc = regulator_disable(chip->vcc_i2c);
	if (rc) {
		dev_err(chip->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		mutex_unlock(&chip->irq_complete);
		return rc;
	}
    
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	pr_debug("come in \n");

	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		smb358_chg_stat_handler(client->irq,chip);
		pr_debug("smb358_resume , chip->irq_waiting = true\n");
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	rc = regulator_enable(chip->vcc_i2c);
	if (rc) {
		dev_err(chip->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
