/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>

#include "smb3xxc-me372cg-charger.h"
#include "smb3xxc_external_include.h"
#include "asus_battery.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/HWVersion.h>

/*qcom*/
#include <linux/uaccess.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

extern int entry_mode;
extern int Read_HW_ID(void);
extern int g_Flag1;
extern int g_Flag4;
extern int cos_keep_power_on_with_cb81;
static int HW_ID;
extern int smb345_get_charging_status(void); // pad charging status

/* I2C communication related */
#define I2C_RETRY_COUNT 6
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41

#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346C_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346C_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346C_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346C_VALUE_500 0x10
#define CFG_CURRENT_LIMIT_SMB358_CHARGER_INHIBIT_CONTROL_MASK   BIT(2) | BIT(3)
#define CFG_VARIOUS_FUNCS            0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB        BIT(2)
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE    BIT(4)
#define CFG_VARIOUS_FUNCS_BATTERY_OV    BIT(1)
#define CFG_FLOAT_VOLTAGE            0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK    0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT    6
#define CFG_STAT                0x05
#define CFG_STAT_DISABLED            BIT(5)
#define CFG_STAT_ACTIVE_HIGH            BIT(7)
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
#define CFG_THERM                0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT    0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK    0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT    2
#define CFG_THERM_MONITOR_DISABLED        BIT(4)
#define CFG_SYSOK                0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED    BIT(2)
#define CFG_OTHER                0x09
#define CFG_OTHER_RID_MASK            0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN        0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C        0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG        0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW        BIT(5)
#define CFG_OTG                    0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK        0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT        4
#define CFG_OTG_CC_COMPENSATION_MASK        0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT        6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK    0x03
#define CFG_TEMP_LIMIT                0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK        0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT        0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK        0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT        2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK        0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT        4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK        0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT        6
#define CFG_FAULT_IRQ                0x0c
#define CFG_FAULT_IRQ_DCIN_UV            BIT(2)
#define CFG_FAULT_IRQ_OTG_UV            BIT(5)
#define CFG_STATUS_IRQ                0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT        BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER    BIT(4)
#define CFG_ADDRESS                0x0e

/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

#define STATUS_UPDATE_INTERVAL            (HZ * 60)

struct smb345c_otg_event {
    struct list_head    node;
    bool            param;
};

struct smb345c_charger {
    struct mutex        lock;
    struct i2c_client    *client;
    /* qcom */
    struct device        *dev;

    struct power_supply    mains;
    struct power_supply    usb;
    struct power_supply    battery;
    bool            mains_online;
    bool            usb_online;
    bool            charging_enabled;
    bool            running;
    struct dentry        *dentry;
    struct dentry        *dentry2;
    struct otg_transceiver    *otg;
    struct notifier_block    otg_nb;
    struct work_struct    otg_work;
    struct work_struct  chrgr_type_work;
    struct delayed_work aicl_dete_work;
    struct workqueue_struct *chrgr_work_queue;
    struct list_head    otg_queue;
    spinlock_t        otg_queue_lock;
    struct wake_lock wakelock;
    bool            otg_enabled;
    bool            otg_battery_uv;
    const struct smb345c_charger_platform_data    *pdata;
    charger_jeita_status_t charger_jeita_status;

    /* qcom */
    int cover_i2c_enable_gpio;
};

static bool g_charging_toggle = true;


static struct smb345c_charger *smb345c_dev;

/* AICL Results Table (mA) : 3Fh */
/* smb358 */
static const unsigned int aicl_results[] = {
    300,
    500,
    700,
    1000,
    1200,
    1300,
    1800,
    2000
};

#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"

#define SMB345C_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG            0x00
#define INPUT_CURRENT_LIMIT_REG    0x01
#define VAR_FUNC_REG            0x02
#define FLOAT_VOLTAGE_REG        0x03
#define CHG_CTRL_REG            0x04
#define STAT_TIMER_REG            0x05
#define PIN_ENABLE_CTRL_REG        0x06
#define THERM_CTRL_A_REG        0x07
#define SYSOK_USB3_SELECT_REG    0x08
#define OTHER_CTRL_A_REG        0x09
#define OTG_TLIM_THERM_CNTRL_REG                0x0A

#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG    0x0B
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK            SMB345C_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            SMB345C_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            SMB345C_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK            SMB345C_MASK(2, 6)

#define FAULT_INTERRUPT_REG        0x0C
#define STATUS_INTERRUPT_REG    0x0D
#define I2C_BUS_SLAVE_REG        0x0E //chris: add
#define CMD_A_REG        0x30
#define CMD_B_REG        0x31
#define CMD_C_REG        0x33
#define INTERRUPT_A_REG        0x35
#define INTERRUPT_B_REG        0x36
#define INTERRUPT_C_REG        0x37
#define INTERRUPT_D_REG        0x38
#define INTERRUPT_E_REG        0x39
#define INTERRUPT_F_REG        0x3A
#define STATUS_A_REG    0x3B
#define STATUS_B_REG    0x3C
#define STATUS_C_REG    0x3D
#define STATUS_D_REG    0x3E
#define STATUS_E_REG    0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK        SMB345C_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT        BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK            SMB345C_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK        SMB345C_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK            SMB345C_MASK(3, 5)
#define TERMINATION_CURRENT_MASK        SMB345C_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK    SMB345C_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK                SMB345C_MASK(6, 0)
#define CHG_ENABLE_BIT            BIT(1)
#define VOLATILE_W_PERM_BIT        BIT(7)
#define USB_SELECTION_BIT        BIT(1)
#define SYSTEM_FET_ENABLE_BIT    BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT    BIT(2)
#define BATT_OV_END_CHG_BIT        BIT(1)
#define VCHG_FUNCTION            BIT(0)
#define CURR_TERM_END_CHG_BIT    BIT(6)

#define OTGID_PIN_CONTROL_MASK    SMB345C_MASK(2, 6)
#define OTGID_PIN_CONTROL_BITS    BIT(6)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345C_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345C_MASK(2, 0)

#define CHARGE_CURRENT_COMPENSATION         SMB345C_MASK(2, 6)
#define CHARGE_CURRENT_COMPENSATION_VALUE   0x00

#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS
#define SMB358_FAST_CHG_CURRENT_MASK            SMB345C_MASK(3, 5)
#define SMB345C_FAST_CHG_CURRENT_MASK            SMB345C_MASK(3, 5)
#define SMB358_TERMINATION_CURRENT_MASK         SMB345C_MASK(3, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_200mA    BIT(0) | BIT(1) | BIT(2)
#define SMB358_TERMINATION_CURRENT_VALUE_80mA    BIT(1) | BIT(0)
#define SMB358_TERMINATION_CURRENT_VALUE_100mA   BIT(2)
#define SMB358_FAST_CHG_CURRENT_VALUE_450mA    BIT(5)
#define SMB358_FAST_CHG_CURRENT_VALUE_600mA    BIT(6)
#define SMB358_FAST_CHG_CURRENT_VALUE_900mA    BIT(5) | BIT(6)
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    BIT(5) | BIT(6) | BIT(7)

static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);

extern bool IS_CA81(void);
extern bool IS_CB81(void);

bool COVER_ATTACHED(void)
{
#if defined(CONFIG_Z380C)
    if (HW_ID == HW_ID_ER)
        return gpio_get_value(COVER_ATTACH_GPIO) ? false : true;
#endif
    return gpio_get_value(MULT_NP_DET) ? true : false;
}
EXPORT_SYMBOL_GPL(COVER_ATTACHED);

bool COVER_ATTACHED_UPI(void)
{
    return gpio_get_value(COVER_ATTACH_GPIO) ? false : true;
}
EXPORT_SYMBOL_GPL(COVER_ATTACHED_UPI);

bool VBUS_IN(void)
{
    if (gpio_get_value(VBUS_IN_DET_N))
        return false;
    return true;
}
EXPORT_SYMBOL_GPL(VBUS_IN);

bool DCIN_VBUS(void)
{
    if (gpio_get_value(DCIN_VBUS_DET_N))
        return false;
    return true;
}
EXPORT_SYMBOL_GPL(DCIN_VBUS);

bool CHARGING_FULL(void);

static int smb345c_read_reg(struct i2c_client *client, int reg,
                u8 *val, int ifDebug)
{
    s32 ret;
    struct smb345c_charger *smb345c_chg;

    if (!COVER_ATTACHED_UPI()) return -1;

    smb345c_chg = i2c_get_clientdata(client);

    //wake_lock(&smb345c_chg->wakelock);
    ret = i2c_smbus_read_byte_data(smb345c_chg->client, reg);
    //wake_unlock(&smb345c_chg->wakelock);
    if (ret < 0) {
        dev_err(&smb345c_chg->client->dev,
            "i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
        return ret;
    } else {
        *val = ret;
    }
    if (ifDebug)
        pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(*val));

    return 0;
}

static int smb345c_write_reg(struct i2c_client *client, int reg,
                        u8 val)
{
    s32 ret;
    struct smb345c_charger *smb345c_chg;

    if (!COVER_ATTACHED_UPI()) return -1;

    smb345c_chg = i2c_get_clientdata(client);

    //wake_lock(&smb345c_chg->wakelock);
    ret = i2c_smbus_write_byte_data(smb345c_chg->client, reg, val);
    //wake_unlock(&smb345c_chg->wakelock);
    if (ret < 0) {
        dev_err(&smb345c_chg->client->dev,
            "i2c write fail: can't write %02X to %02X: %d\n",
            val, reg, ret);
        return ret;
    }
    return 0;
}

static int smb345c_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345c_read_reg(client, reg, &temp, 0);
        if (rc) {
            retry_count--;
            pr_err("*smb345c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int smb345c_masked_write(struct i2c_client *client, int reg,
        u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
    rc = smb345c_read_reg(client, reg, &temp, 0);
    if (rc) {
        retry_count--;
        pr_err("*smb345c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
    rc = smb345c_write_reg(client, reg, temp);
    if (rc) {
        retry_count--;
        pr_err("*smb345c_write failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345c_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}

static int smb345c_read(struct smb345c_charger *smb, u8 reg)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        if (!COVER_ATTACHED_UPI()) return -1;

        //wake_lock(&smb->wakelock);
        ret = i2c_smbus_read_byte_data(smb->client, reg);
        //wake_unlock(&smb->wakelock);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to read reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int smb345c_write(struct smb345c_charger *smb, u8 reg, u8 val)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        if (!COVER_ATTACHED_UPI()) return -1;

        //wake_lock(&smb->wakelock);
        ret = i2c_smbus_write_byte_data(smb->client, reg, val);
        //wake_unlock(&smb->wakelock);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to write reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int smb345c_set_writable(struct smb345c_charger *smb, bool writable)
{
    int ret;

    ret = smb345c_read(smb, CMD_A);
    if (ret < 0)
        return ret;

    if (writable)
        ret |= CMD_A_ALLOW_WRITE;
    else
        ret &= ~CMD_A_ALLOW_WRITE;

    return smb345c_write(smb, CMD_A, ret);
}

// =============================== power supply ===============================
bool COVER_OTG_PIN_CONTROL_ENABLE(void)
{
    if (!smb345c_dev) {
        pr_info("%s: ERROR: smb345c_dev is null "
            "due to probe function has error\n",
            __func__);
        return false;
    }

    if (!COVER_ATTACHED_UPI())
        return false;

    /* check if 09h[7:6]=="01" */
    if (smb345c_masked_read(smb345c_dev->client, 0x09, BIT(7) | BIT(6)) & BIT(6))
        return true;

    return false;
}

static enum power_supply_property cover_power_properties[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static void cover_external_power_changed(struct power_supply *psy);
static int cover_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val);

static struct power_supply cover_power_supplies[] = {
    {
        .name = "pack_ac",
        .type = POWER_SUPPLY_TYPE_PACK_AC,
        .properties = cover_power_properties,
        .external_power_changed = cover_external_power_changed,
        .num_properties = ARRAY_SIZE(cover_power_properties),
        .get_property = cover_power_get_property,
    },
};

static void cover_external_power_changed(struct power_supply *psy)
{
    power_supply_changed(&cover_power_supplies[0]);
}

static int cover_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    int ret = 0;
    int charging_status = POWER_SUPPLY_STATUS_UNKNOWN;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (COVER_ATTACHED_UPI() && IS_CB81()) {
            if (entry_mode == 4)
                charging_status = smb345_get_charging_status();
            if (entry_mode == 1 && COVER_OTG_PIN_CONTROL_ENABLE())
            {
                val->intval = 1;
            }
            else if (entry_mode == 4 && (cos_keep_power_on_with_cb81 || charging_status == POWER_SUPPLY_STATUS_CHARGING || charging_status == POWER_SUPPLY_STATUS_FULL))
            {
                BAT_DBG("cos_keep_power_on_with_cb81: %d, charging_status: %d\n", cos_keep_power_on_with_cb81, charging_status);
                val->intval = 1;
            }
            else
                val->intval = 0;
        }
        else {
            val->intval = 0;
        }
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}
// =============================== power supply ===============================

/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl,
                    size_t size, unsigned int val)
{
    if (val >= size)
        return tbl[size-1];;
    return tbl[val];
}

/* Acquire cover attached/dettached status */
static ssize_t get_cover_status(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = COVER_ATTACHED_UPI() ? 1 : 0;
    return sprintf(buf, "%d\n", ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
int cover_get_aicl_results(void)
{
    int ret;

    ret = smb345c_read(smb345c_dev, STAT_E);
    if (ret < 0) {
        BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
        return ret;
    }

    ret &= 0x0F;
    return hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret);
}
EXPORT_SYMBOL_GPL(cover_get_aicl_results);

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    if (!smb345c_dev) {
        pr_info("%s: ERROR: smb345c_dev is null "
            "due to probe function has error\n",
            __func__);
        return sprintf(buf, "%d\n", -EINVAL);
    }

    if (!COVER_ATTACHED_UPI())
        return -9999;

    ret = smb345c_read(smb345c_dev, STAT_E);
    if (ret<0) {
        pr_info("%s: ERROR: i2c read error\n", __func__);
        return sprintf(buf, "%d\n", -EIO);
    }

    ret &= 0x0F;
    return sprintf(buf, "%d\n",
            hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = smb345c_get_charging_status();
    if (ret == POWER_SUPPLY_STATUS_CHARGING ||
        ret == POWER_SUPPLY_STATUS_FULL ||
        ret == POWER_SUPPLY_STATUS_NOT_CHARGING)
        ret = 1;
    else
        ret = 0;
    return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static DEVICE_ATTR(cover_status, S_IRUGO, get_cover_status, NULL);
static struct attribute *dev_attrs[] = {
    &dev_attr_input_current.attr,
    &dev_attr_charge_status.attr,
    &dev_attr_cover_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};

/*----------------------------------------------------------------------------*/
void cover_usbin(bool turn_on)
{
    int ret;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return;
    }

    BAT_DBG(" %s:\n", __func__);

    if (!COVER_ATTACHED_UPI()) return;

    if (turn_on) {
        /* enable Cover Charger USBIN */
        ret = smb345c_masked_write(smb345c_dev->client,
                0x30,
                BIT(2),
                0);
    }
    else {
        /* suspend Cover Charger USBIN */
        ret = smb345c_masked_write(smb345c_dev->client,
                0x30,
                BIT(2),
                BIT(2));
    }
    if (ret)
        BAT_DBG(" %s: i2c communication failure!", __func__);
}
EXPORT_SYMBOL_GPL(cover_usbin);

#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
int smb3xxc_soc_control_jeita(void)
{
    int ret;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    BAT_DBG(" %s:\n", __func__);

    if (!COVER_ATTACHED_UPI()) return 0;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return ret;

    /* Set Hard Hot Limit as 72 Deg. C 0Bh[5:4]="11" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x0B,
            BIT(5) | BIT(4),
            BIT(5) | BIT(4));

    /* Set Soft Hot Limit Behavior & Soft Cold Tempr Limit as No Response 07h[3:0]="0000" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x07,
            BIT(3) | BIT(2) | BIT(1) | BIT(0),
            0);

    if (ret)
        BAT_DBG(" %s: i2c communication failure!\n", __func__);

    return ret;
}
#else
int smb3xxc_soc_control_jeita(void) { return 0; }
#endif
EXPORT_SYMBOL_GPL(smb3xxc_soc_control_jeita);

#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
int smb3xxc_charger_control_jeita(void)
{
    int ret;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    BAT_DBG(" %s:\n", __func__);

    if (!COVER_ATTACHED_UPI()) return 0;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return ret;

    /* Set Hard Hot Limit as 59 Deg. C 0Bh[5:4]="01" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x0B,
            BIT(5) | BIT(4),
            BIT(4));

    /* Set Soft Hot Limit Behavior as Float Voltage Compensation 07h[1:0]="10" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x07,
            BIT(1) | BIT(0),
            BIT(1));

    /* Set Soft Cold Limit Behavior as Charge Current Compensation 07h[3:2]="01" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x07,
            BIT(3) | BIT(2),
            BIT(2));

    /* Charging Enable */
    smb345c_charging_toggle(JEITA, true);
    if (ret)
        BAT_DBG(" %s: i2c communication failure!\n", __func__);

    return ret;
}
#else
int smb3xxc_charger_control_jeita(void) { return 0; }
#endif
EXPORT_SYMBOL_GPL(smb3xxc_charger_control_jeita);

int smb345c_get_soc_control_float_vol(void)
{
    int ret;

    /* read 03h[5:0]="011110" or "101010"*/
    ret = smb345c_masked_read(smb345c_dev->client,
                FLOAT_VOLTAGE_REG,
                FLOAT_VOLTAGE_MASK);

    return ret;
}

#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
int smb3xxc_jeita_control(int bat_tempr, int bat_volt, bool exec_jeita)
{
    static charger_jeita_status_t ori_charger_jeita_status = ROOM;
    int ret;
    int Vflt;
    int Vflt_IV;
    int Volt_IV;
    int Fast_Current_High;
    int Fast_Current_Low;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return POWER_SUPPLY_STATUS_DISCHARGING;
    }

    if (exec_jeita) {

    smb3xxc_soc_control_jeita();

    if (IS_CA81()) {
        /* 4.35V */
        Vflt = BIT(5) | BIT(3) | BIT(1) | BIT(0);
        /* 4.1V */
        Vflt_IV = BIT(4) | BIT(3) | BIT(2) | BIT(1);
        /* 4.1V */
        Volt_IV = 4100;

        Fast_Current_High = BIT(6) | BIT(5);
        Fast_Current_Low = BIT(5);
    }
    else if (IS_CB81()) {
        /* 4.2V */
        Vflt = BIT(5) | BIT(1) | BIT(0);
        /* 4.0V */
        Vflt_IV = BIT(4) | BIT(3) | BIT(0);
        /* 4.0V */
        Volt_IV = 4000;

        Fast_Current_High = BIT(7) | BIT(6) | BIT(5);
        Fast_Current_Low = BIT(6) | BIT(5);
    }
    else {
        /* 4.35V */
        Vflt = BIT(5) | BIT(3) | BIT(1) | BIT(0);
        /* 4.1V */
        Vflt_IV = BIT(4) | BIT(3) | BIT(2) | BIT(1);
        /* 4.1V */
        Volt_IV = 4100;

        Fast_Current_High = BIT(6) | BIT(5);
        Fast_Current_Low = BIT(5);
    }
    }

    ori_charger_jeita_status = smb345c_dev->charger_jeita_status;

    /* condition judgement 1: */
    if (ori_charger_jeita_status == FREEZE) {
        if (bat_tempr >= 45)
            /* control diagram II: */
            goto control_diagram_II;
        else
            /* control diagram I: */
            goto control_diagram_I;
    }

    /* condition judgement 2: */
    if (ori_charger_jeita_status == COLD) {
        if (bat_tempr >= 180) {
            /* control diagram III: */
            goto control_diagram_III;
        }
        else if (bat_tempr < 15) {
            /* control diagram I: */
            goto control_diagram_I;
        }
        else {
            /* control diagram II: */
            goto control_diagram_II;
        }
    }

    /* condition judgement 3: */
    if (ori_charger_jeita_status == ROOM) {
        if (bat_tempr < 150) {
            /* control diagram II: */
            goto control_diagram_II;
        }
        else if (bat_tempr >= 500) {
            if (bat_volt >= Volt_IV) {
                /* control diagram V: */
                goto control_diagram_V;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
        else
            /* control diagram III: */
            goto control_diagram_III;
    }

    /* condition judgement 4: */
    if (ori_charger_jeita_status == HOT) {
        // state before is "Charging Disable"
        if (smb345c_get_soc_control_float_vol() == Vflt) {
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                if (bat_volt >= Volt_IV) {
                    /* control diagram V: */
                    goto control_diagram_V;
                }
                else {
                    /* control diagram IV: */
                    goto control_diagram_IV;
                }
            }
        }
        else { // state before is "Charging Enable"
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
    }

    /* condition judgement 5: */
    if (ori_charger_jeita_status == OVERHEAT) {
        if (bat_tempr < 520) {
            /* control diagram IV: */
            goto control_diagram_IV;
        }
        else {
            /* control diagram VI: */
            goto control_diagram_VI;
        }
    }

    BAT_DBG_E(" %s: *** ERROR ***\n", __func__);
    goto others;

control_diagram_I:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_Low);
    ret = smb345c_charging_toggle(JEITA, false);
    }
    smb345c_dev->charger_jeita_status = FREEZE;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_II:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_Low);
    ret = smb345c_charging_toggle(JEITA, true);
    }
    smb345c_dev->charger_jeita_status = COLD;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_III:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_High);
    ret = smb345c_charging_toggle(JEITA, true);
    }
    smb345c_dev->charger_jeita_status = ROOM;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_IV:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt_IV);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_High);
    ret = smb345c_charging_toggle(JEITA, true);
    }
    smb345c_dev->charger_jeita_status = HOT;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_V:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_High);
    ret = smb345c_charging_toggle(JEITA, false);
    }
    smb345c_dev->charger_jeita_status = HOT;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_VI:
    if (exec_jeita) {
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    ret = smb345c_masked_write(smb345c_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            Fast_Current_High);
    ret = smb345c_charging_toggle(JEITA, false);
    }
    smb345c_dev->charger_jeita_status = OVERHEAT;
    if (COVER_ATTACHED_UPI())
    if (ori_charger_jeita_status != smb345c_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345c_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

others:
    return POWER_SUPPLY_STATUS_CHARGING;
}
#else
int smb3xxc_jeita_control(int bat_tempr, int bat_volt, bool exec_jeita)
{
    return POWER_SUPPLY_STATUS_CHARGING;
}
#endif

#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
int smb3xxc_jeita_control_for_sw_gauge(int usb_state, bool exec_jeita)
{
    int ret;
    int batt_tempr = 250;/* unit: C  */
    int batt_volt = 4000;/* unit: mV */

    if (COVER_ATTACHED_UPI()) {

    /* acquire battery temperature here */
    ret = get_battery_temperature(&batt_tempr);
    if (ret) {
        BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
        return ret;
    }

    /* acquire battery voltage here */
    ret = get_battery_voltage(&batt_volt);
    if (ret) {
        BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
        return ret;
    }

    BAT_DBG(" %s: T:%d(0.1C), V:%dmV\n", __func__, batt_tempr, batt_volt);
    }

    /* exec jeita function */
    ret = smb3xxc_jeita_control(batt_tempr, batt_volt, exec_jeita);

    return ret;
}
#else
int smb3xxc_jeita_control_for_sw_gauge(int usb_state, bool exec_jeita) { return 0; }
#endif
EXPORT_SYMBOL_GPL(smb3xxc_jeita_control_for_sw_gauge);

bool disable_cover_otg(void)
{
    int ret;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return false;
    }

    if (!COVER_ATTACHED_UPI()) return false;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return false;

    /* Set OTG/ID as I2C control 09h[7:6]="00" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x09,
            BIT(6) | BIT(7),
            0);
    if (ret < 0)
        return false;

    /* Disable OTG 30h[4]="0" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x30,
            BIT(4),
            0);
    if (ret < 0)
        return false;

    BAT_DBG(" disable cover OTG\n");
    g_Flag1 = 0;
    g_Flag4 = 0;
    return true;
}
EXPORT_SYMBOL_GPL(disable_cover_otg);

#if defined(CONFIG_Z380KL) || defined(CONFIG_Z380C)
int smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed)
{
    int ret;
    int Vflt;

    if (!smb345c_dev) {
        pr_err("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return -EINVAL;
    }

    if (!COVER_ATTACHED_UPI()) return 0;

    /* enable write 30h[7]="1" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x30,
            BIT(7),
            BIT(7));

    if (IS_CA81()) {
        /* set fast charge current: 900mA */
        /* set pre charge current: 150mA */
        /* set termination current: 100mA */
        if (cover_changed_cable_changed)
        ret = smb345c_write(smb345c_dev,
            0x00,
            0x64);
        /* set cold soft limit current: 450mA write 0Ah[7:6]="01"*/
        ret = smb345c_masked_write(smb345c_dev->client,
            0x0a,
            BIT(7) | BIT(6),
            BIT(6));
        /* CA81 Float Voltage: 4.35V */
        Vflt = BIT(5) | BIT(3) | BIT(1) | BIT(0);
    }
    else if (IS_CB81()) {
        /* set fast charge current: 2000mA */
        /* set pre charge current: 250mA */
        /* set termination current: 200mA */
        if (cover_changed_cable_changed)
        ret = smb345c_write(smb345c_dev,
            0x00,
            0xEF);
        /* set cold soft limit current: 450mA write 0Ah[7:6]="11"*/
        ret = smb345c_masked_write(smb345c_dev->client,
            0x0a,
            BIT(7) | BIT(6),
            BIT(7) | BIT(6));
        /* CB81 Float Voltage: 4.2V */
        Vflt = BIT(5) | BIT(1) | BIT(0);
    }
    else {
        BAT_DBG(" unknown cover type for initial current setting\n");
        Vflt = BIT(5) | BIT(3) | BIT(1) | BIT(0);
    }
    if (ret < 0)
        goto fail;

    /* set Battery OV does not end charge cycle. 02h[1]="0" */
    ret = smb345c_masked_write(smb345c_dev->client,
            0x02,
            BIT(1),
            0);
    if (ret < 0)
        goto fail;

    /* set Float Voltage: 4.35V/4.2V. 03h[5:0]="101011" or "100011" */
    if (cover_changed_cable_changed)
    ret = smb345c_masked_write(smb345c_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            Vflt);
    if (ret < 0)
        goto fail;

    if (initial) {
        /* APSD disable */
        ret = smb345c_masked_write(smb345c_dev->client,
                0x04,
                BIT(2),
                0);
        if (ret < 0)
            goto fail;

        /* USB5/1/HC: I2C control */
        ret = smb345c_masked_write(smb345c_dev->client,
                0x06,
                BIT(4),
                0);
        if (ret < 0)
            goto fail;

        /* USB/HC Mode: High Current Mode */
        ret = smb345c_masked_write(smb345c_dev->client,
                0x31,
                BIT(0),
                BIT(0));
        if (ret < 0)
            goto fail;
    }

fail:
    return ret;
}
#else
int smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed) { return 0; }
#endif
EXPORT_SYMBOL_GPL(smb3xxc_pre_config);

/*----------------------------------------------------------------------------*/

void cover_otg_current(int curr)
{
    int reg_val;

    if (!smb345c_dev) {
        pr_err("%s: smb345c_dev is null "
            "due to driver probed isn't ready\n",
            __func__);
        return;
    }

    /* config cover otg current 0Ah[3:2]="??" */
    switch (curr) {
    case 250:
        reg_val = 0x00;
        break;
    case 500:
        reg_val = BIT(2);
        break;
    case 750:
        reg_val = BIT(3);
        break;
    case 900:
        reg_val = BIT(3) | BIT(2);
        break;
    }

    if (smb345c_set_writable(smb345c_dev, true) < 0)
        return;

    if (smb345c_masked_write(smb345c_dev->client, 0x0A, BIT(3) | BIT(2), reg_val))
        dev_err(&smb345c_dev->client->dev,
            "%s: fail to config cover otg current\n",
            __func__);
}
EXPORT_SYMBOL_GPL(cover_otg_current);

void cover_otg(int on)
{
    if (!smb345c_dev) {
        pr_err("%s: smb345c_dev is null "
            "due to driver probed isn't ready\n",
            __func__);
        return;
    }

    if (smb345c_set_writable(smb345c_dev, true) < 0)
        return;

    /* Enable Cover OTG - 09h[7:6]="01" */
    if (on) {
        if (smb345c_masked_write(smb345c_dev->client, 0x09, BIT(7) | BIT(6), BIT(6)))
            dev_err(&smb345c_dev->client->dev,
                "%s: fail to enable cover otg\n",
                __func__);
    }
    else {
        disable_cover_otg();
    }
}
EXPORT_SYMBOL_GPL(cover_otg);

void cover_aicl(int I_USB_IN)
{
    int reg_val;

    if (!smb345c_dev) {
        pr_err("%s: smb345c_dev is null "
            "due to driver probed isn't ready\n",
            __func__);
        return;
    }

    if (I_USB_IN == 500)
        reg_val = 0x10;
    else if (I_USB_IN == 300)
        reg_val = 0x00;
    else if (I_USB_IN == 1200)
        reg_val = 0x40;
    else
        reg_val = 0x10;

    if (smb345c_set_writable(smb345c_dev, true) < 0)
        return;

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345c_masked_write(smb345c_dev->client, 0x02, BIT(4), 0)) {
        dev_err(&smb345c_dev->client->dev,
            "%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_USB_IN=500mA,300mA - Write 01h[7:4]="0000","0001" */
    if (smb345c_masked_write(smb345c_dev->client, 0x01, 0xf0, reg_val)) {
        dev_err(&smb345c_dev->client->dev,
            "%s: fail to set max current limits for USB_IN\n",
            __func__);
        return;
    }

    /* Enable AICL - Write 02h[4]="1" */
    if (smb345c_masked_write(smb345c_dev->client, 0x02, BIT(4), 0x10)) {
        dev_err(&smb345c_dev->client->dev,
            "%s: fail to enable AICL\n", __func__);
        return;
    }
}
EXPORT_SYMBOL_GPL(cover_aicl);

/* write 06h[6:5]="00" or "11" */
int smb345c_charging_toggle(charging_toggle_level_t level, bool on)
{
    int ret = 0;
    int charging_toggle;
    static bool jeita_want_charging = true;
    static bool balan_want_charging = true;
    static charging_toggle_level_t old_lvl = JEITA;
    char *level_str[] = {
        "BALANCE",
        "JEITA",
        "FLAGS",
    };

    if (!smb345c_dev) {
        pr_info("Warning: smb345c_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    charging_toggle = g_charging_toggle;

    BAT_DBG(" %s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
        __func__,
        level_str[old_lvl],
        charging_toggle ? "YES" : "NO",
        level_str[level],
        on ? "YES" : "NO");

    /* reset to default if AudioCover/PowerBank doesn't attached */
    if (!COVER_ATTACHED_UPI()) {
        balan_want_charging = true;
        jeita_want_charging = true;
    }

    /* do charging or not? */
    if (level != FLAGS) {
        if (on) {
            /* want to start charging? */
            if (level == JEITA) {
                jeita_want_charging = true;
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl == FLAGS) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E(" %s: * reject RESTART charging to Cover! *"
                            "(old_lvl != JEITA)\n",
                            __func__);
                        return -1;
                    }
                    else if (!balan_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E(" %s: * reject RESTART charging to Cover! *"
                            "(!balan_want_charging)\n",
                            __func__);
                        return -1;
                    }
                }
            }
            else if (level == BALANCE) {
                balan_want_charging = true;
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl == FLAGS) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E(" %s: * reject RESTART charging to Cover! *"
                            "(old_lvl != BALANCE)\n",
                            __func__);
                        return -1;
                    }
                    else if (!jeita_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E(" %s: * reject RESTART charging to Cover! *"
                            "(!jeita_want_charging)\n",
                            __func__);
                        return -1;
                    }
                }
            }
            else {
                /* what the hell are you? */
            }
        }
        else {
            /* FLAGS stop charging before. reject RESTOP charging to Cover again */
            if (old_lvl == FLAGS && !charging_toggle) {
                BAT_DBG_E(" %s: * reject STOP charging again to Cover! *\n",
                    __func__);
                return -1;
            }
            /* want to stop charging? just do it! */
            if (level == JEITA)
                jeita_want_charging = false;
            if (level == BALANCE)
                balan_want_charging = false;
        }
    }
    else {
        /* it's the highest level. just do it! */
    }

    /* level value assignment */
    old_lvl = level;

    if (!on)
        BAT_DBG_E(" %s: *** charging toggle: OFF ***\n",
            __func__);
    else
        BAT_DBG(" %s: --------------- charging toggle: ON ---------------\n",
            __func__);

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return ret;

    /* Config CFG_PIN register */

    ret = smb345c_read(smb345c_dev, CFG_PIN);
    if (ret < 0)
        goto out;

    /*
     * Make the charging functionality controllable by a write to the
     * command register unless pin control is specified in the platform
     * data.
     */
    ret &= ~CFG_PIN_EN_CTRL_MASK;
    if (on) {
        /* set Pin Controls - active low (ME371MG connect EN to GROUND) */
        ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
    } else {
        /* Do nothing, 0 means i2c control
            . I2C Control - "0" in Command Register disables charger */
    }

    ret = smb345c_write(smb345c_dev, CFG_PIN, ret);
    if (ret < 0)
        goto out;

    g_charging_toggle = on;

out:
    return ret;
}
EXPORT_SYMBOL_GPL(smb345c_charging_toggle);

/* return:
    false: means fail
    true:  means success
*/
bool STOP_CHARGING(void)
{
    int ret;

    if (!smb345c_dev)
        return false;

    if (!COVER_ATTACHED_UPI())
        return false;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return false;

    ret = smb345c_charging_toggle(FLAGS, false);
    if (ret < 0)
        return false;

    BAT_DBG_E(" stop cover charging successfully\n");
    return true;
}
EXPORT_SYMBOL_GPL(STOP_CHARGING);

/* return:
    false: means fail
    true:  means success
*/
bool START_CHARGING(void)
{
    int ret;

    if (!smb345c_dev)
        return false;

    if (!COVER_ATTACHED_UPI())
        return false;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return false;

    ret = smb345c_charging_toggle(FLAGS, true);
    if (ret < 0)
        return false;

    BAT_DBG_E(" start cover charging successfully\n");
    return true;
}
EXPORT_SYMBOL_GPL(START_CHARGING);

bool COVER_PRE_CHARGING_MODE(void)
{
    int ret;

    if (!smb345c_dev)
        return false;

    if (!COVER_ATTACHED_UPI())
        return false;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return false;

    ret = smb345c_read(smb345c_dev, STAT_C);
    if (ret < 0)
        return false;

    if (!(ret & BIT(2)) &&
        (ret & BIT(1))  &&
        (ret & BIT(0))) {
        BAT_DBG_E(" PRE_CHARGING_MODE: true\n");
        return true;
    }
    else {
        BAT_DBG_E(" %s: PRE_CHARGING_MODE: false ret=0x%02X\n", __func__, ret);
    }

    return false;
}
EXPORT_SYMBOL_GPL(COVER_PRE_CHARGING_MODE);

bool smb345c_has_charger_error(void)
{
    int ret;

    if (!smb345c_dev)
        return -EINVAL;

    ret = smb345c_read(smb345c_dev, STAT_C);
    if (ret < 0)
        return true;

    if (ret & STAT_C_CHARGER_ERROR)
        return true;

    return false;
}

int smb345c_get_charging_status(void)
{
    int ret, status;

    if (!smb345c_dev)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    if (!COVER_ATTACHED_UPI())
        return POWER_SUPPLY_STATUS_UNKNOWN;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    ret = smb345c_read(smb345c_dev, STAT_C);
    if (ret < 0)
        return POWER_SUPPLY_STATUS_UNKNOWN;

#if 0
    dev_info(&smb345c_dev->client->dev,
            "Charging Status: STAT_C:0x%x\n", ret);
#endif

    if ((ret & STAT_C_CHARGER_ERROR) ||
        (ret & STAT_C_HOLDOFF_STAT)) {
        /* set to NOT CHARGING upon charger error
         * or charging has stopped.
         */
        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else {
        if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
            /* set to charging if battery is in pre-charge,
             * fast charge or taper charging mode.
             */
            status = POWER_SUPPLY_STATUS_CHARGING;
        } else if (CHARGING_FULL()) {
            /* set the status to FULL if battery is not in pre
             * charge, fast charge or taper charging mode AND
             * charging is terminated at least once.
             */
            status = POWER_SUPPLY_STATUS_FULL;
        } else {
            /* in this case no charger error or termination
             * occured but charging is not in progress!!!
             */
            status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
    }

    return status;
}
EXPORT_SYMBOL_GPL(smb345c_get_charging_status);

bool CHARGING_FULL(void)
{
    int ret;

    if (!smb345c_dev)
        return false;

    if (!COVER_ATTACHED_UPI())
        return false;

    ret = smb345c_set_writable(smb345c_dev, true);
    if (ret < 0)
        return false;

    ret = smb345c_read(smb345c_dev, IRQSTAT_C);
    if (ret < 0)
        return false;

    if (ret & BIT(0)) {
        /* set to almost full if battery is in termination charging current mode */
        BAT_DBG_E(" COVER TERMINATION CHARING CURRENT hit...\n");
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(CHARGING_FULL);

#ifndef ME372CG_OTHER_BUILD
int smb345c_dump_registers(struct seq_file *s)
{
    struct smb345c_charger *smb;
    int ret;
    u8 reg;

    if (s) {
        smb = s->private;
    }
    else {
        if (!smb345c_dev) {
            BAT_DBG(" %s: smb345c_dev is null!\n",
                __func__);
            return -1;
        }
        else {
            smb = smb345c_dev;
        }
    }
    if (!COVER_ATTACHED_UPI()) {
        pr_info("* Cover is dettached *\n");
        return 0;
    }

    BAT_DBG(" %s:\n", __func__);
    BAT_DBG(" Control registers:\n");
    BAT_DBG(" ==================\n");
    BAT_DBG(" #Addr\t#Value\n");

    for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
        ret = smb345c_read(smb, reg);
        BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(ret));
        if (s)
            seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
                "\n", reg, BYTETOBINARY(ret));
    }
    BAT_DBG("\n");
    if (s)
        seq_printf(s, "\n");

    BAT_DBG(" Command registers:\n");
    BAT_DBG(" ==================\n");
    BAT_DBG(" #Addr\t#Value\n");
    if (s) {
        seq_printf(s, "Command registers:\n");
        seq_printf(s, "==================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }

    ret = smb345c_read(smb, CMD_A);
    BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
        "\n", CMD_A, BYTETOBINARY(ret));
    if (s)
        seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
            "\n", CMD_A, BYTETOBINARY(ret));
    ret = smb345c_read(smb, CMD_B);
    BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
        "\n", CMD_B, BYTETOBINARY(ret));
    if (s)
        seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
            "\n", CMD_B, BYTETOBINARY(ret));
    ret = smb345c_read(smb, CMD_C);
    BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
        "\n", CMD_C, BYTETOBINARY(ret));
    if (s)
        seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
            "\n", CMD_C, BYTETOBINARY(ret));
    BAT_DBG("\n");
    if (s)
        seq_printf(s, "\n");

    BAT_DBG(" Interrupt status registers:\n");
    BAT_DBG(" ===========================\n");
    BAT_DBG(" #Addr\t#Value\n");
    if (s) {
        seq_printf(s, "Interrupt status registers:\n");
        seq_printf(s, "===========================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }
    for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
        ret = smb345c_read(smb, reg);
        BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(ret));
        if (s)
            seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
                "\n", reg, BYTETOBINARY(ret));
    }
    BAT_DBG("\n");
    if (s)
        seq_printf(s, "\n");

    BAT_DBG(" Status registers:\n");
    BAT_DBG(" =================\n");
    BAT_DBG(" #Addr\t#Value\n");
    if (s) {
        seq_printf(s, "Status registers:\n");
        seq_printf(s, "=================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }
    for (reg = STAT_A; reg <= STAT_E; reg++) {
        ret = smb345c_read(smb, reg);
        BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(ret));
        if (s)
            seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
                "\n", reg, BYTETOBINARY(ret));
    }

    return 0;
}
#else
int smb345c_dump_registers(struct seq_file *s) { return 0; }
#endif
EXPORT_SYMBOL_GPL(smb345c_dump_registers);

static int smb345c_debugfs_show(struct seq_file *s, void *data)
{
    seq_printf(s, "Control registers:\n");
    seq_printf(s, "==================\n");
    seq_printf(s, "#Addr\t#Value\n");

    smb345c_dump_registers(s);

    return 0;
}

static int smb345c_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, smb345c_debugfs_show, inode->i_private);
}

static const struct file_operations smb345c_debugfs_fops = {
    .open        = smb345c_debugfs_open,
    .read        = seq_read,
    .llseek        = seq_lseek,
    .release    = single_release,
};

static inline struct power_supply *get_psy_battery(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_PACK_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

static inline int get_battery_temperature(int *tempr)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
    if (!ret)
        *tempr = val.intval;
    /* Force use Fake Temperature as use of UPI gauge */
    if (*tempr == -2731) *tempr = 250;

    return ret;
}

static inline int get_battery_voltage(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *volt = val.intval;

    return ret;
}

static inline int get_battery_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

static int smb345c_parse_dt(struct smb345c_charger *chip)
{
    int ret = 0;
    struct device_node *node = chip->client->dev.of_node;
    if (!node) {
        pr_err("No DT data Failing Probe\n");
        return -EINVAL;
    }

    /*-------------------------- ENABLE COVER I2C ---------------------------*/
    chip->cover_i2c_enable_gpio =
        of_get_named_gpio(node, "qcom,cover-i2c-enable-gpio", 0);
    pr_info("qcom,cover-i2c-enable-gpio = %d.\n", chip->cover_i2c_enable_gpio);
    if (!gpio_is_valid(chip->cover_i2c_enable_gpio)) {
        pr_err("gpio is not valid: cover_i2c_enable_gpio\n");
        return -EINVAL;
    }

    /* request it and configure gpio as output */
    ret = devm_gpio_request_one(chip->dev,
                                chip->cover_i2c_enable_gpio,
                                GPIOF_OUT_INIT_HIGH,
                                "cover_i2c_en_gpio");
    if (ret) {
        dev_err(&chip->client->dev, "gpio_request failed for %d ret=%d\n",
            chip->cover_i2c_enable_gpio, ret);
        return ret;
    }

    /* config default value to output high : force enable */
    gpio_set_value(chip->cover_i2c_enable_gpio, 1);

    return ret;
}

static int smb345c_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct smb345c_charger *smb;
    int ret;

    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
    if (!smb)
        return -ENOMEM;

    i2c_set_clientdata(client, smb);

    smb->client = client;
    /* qcom */
    smb->dev = &client->dev;

    /* qcom */
    ret = smb345c_parse_dt(smb);
    if (ret < 0) {
        dev_err(&client->dev,
        "Couldn't to parse dt ret = %d\n", ret);
        goto error;
    }

    /* enable register writing - chris */
    if (!COVER_ATTACHED_UPI()) {
        pr_info("* Cover is dettached *\n");
    }
    else {
    ret = smb345c_set_writable(smb, true);
    if (ret < 0)
        dev_err(dev, "*failed to enable write\n");//goto error;
    }

    smb345c_dev = smb;
    smb345c_dump_registers(NULL);
    smb->charger_jeita_status = ROOM;

    /* Refer to SMB345C Application Note 72 to solve serious problems */
    if (!COVER_ATTACHED_UPI()) {
        pr_info("* Cover is dettached *\n");
    }
    else {
    ret = smb345c_masked_write(smb->client,
            OTG_TLIM_THERM_CNTRL_REG,
            OTG_CURRENT_LIMIT_AT_USBIN_MASK,
            OTG_CURRENT_LIMIT_250mA);
    if (ret < 0)
        dev_err(dev, "*failed to set OTG boost 250mA\n");//goto error;
    }

    /* Init Runtime PM State */
    pm_runtime_put_noidle(&smb->client->dev);
    pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

    smb->running = true;
    smb->dentry = debugfs_create_file("smbc", S_IRUGO, NULL, smb,
                      &smb345c_debugfs_fops);
    ret = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

    BAT_DBG(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

    disable_cover_otg();

    ret = power_supply_register(dev, &cover_power_supplies[0]);
    if (ret) {
        dev_err(dev,"Fail to register power supply AC\n");
        goto error;
    }

    return 0;

error:
    return ret;
}

static int smb345c_remove(struct i2c_client *client)
{
    struct smb345c_charger *smb = i2c_get_clientdata(client);

    if (!IS_ERR_OR_NULL(smb->dentry))
        debugfs_remove(smb->dentry);

    smb->running = false;

    pm_runtime_get_noresume(&smb->client->dev);

    return 0;
}

static void smb345c_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

    if (entry_mode == 4) disable_cover_otg();

    /* cover charger control jeita */
    smb3xxc_charger_control_jeita();

    /* registers dump */
    smb345c_dump_registers(NULL);
}

#ifdef CONFIG_PM
static int smb345c_prepare(struct device *dev)
{
    struct smb345c_charger *smb = dev_get_drvdata(dev);

    dev_info(&smb->client->dev, "smb345c suspend\n");

    return 0;
}

static void smb345c_complete(struct device *dev)
{
    struct smb345c_charger *smb = dev_get_drvdata(dev);

    dev_info(&smb->client->dev, "smb345c resume\n");

}
#else
#define smb345c_prepare NULL
#define smb345c_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb345c_runtime_suspend(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int smb345c_runtime_resume(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int smb345c_runtime_idle(struct device *dev)
{

    dev_info(dev, "%s called\n", __func__);
    return 0;
}
#else
#define smb345c_runtime_suspend    NULL
#define smb345c_runtime_resume    NULL
#define smb345c_runtime_idle    NULL
#endif

static const struct of_device_id smb3xxc_match[] = {
    { .compatible = "qcom,smb3xxc" },
    { },
};

static const struct i2c_device_id smb345c_id[] = {
    {SMB345C_DEV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, smb345c_id);

static const struct dev_pm_ops smb345c_pm_ops = {
    .prepare        = smb345c_prepare,
    .complete        = smb345c_complete,
    .runtime_suspend    = smb345c_runtime_suspend,
    .runtime_resume        = smb345c_runtime_resume,
    .runtime_idle        = smb345c_runtime_idle,
};

static struct i2c_driver smb345c_driver = {
    .driver = {
        .name    = SMB345C_DEV_NAME,
        .owner    = THIS_MODULE,
        .pm    = &smb345c_pm_ops,
        .of_match_table = of_match_ptr(smb3xxc_match),
    },
    .probe        = smb345c_probe,
    .remove        = smb345c_remove,
    .shutdown    = smb345c_shutdown,
    .id_table    = smb345c_id,
};

static int __init smb345c_init(void)
{
    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
    HW_ID = Read_HW_ID();
    return i2c_add_driver(&smb345c_driver);
}
module_init(smb345c_init);

static void __exit smb345c_exit(void)
{
    i2c_del_driver(&smb345c_driver);
}
module_exit(smb345c_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SMB345C battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb345c");

