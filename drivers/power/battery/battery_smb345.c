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

#include "smb345-me372cg-charger.h"
#include "smb345_external_include.h"
#include "asus_battery.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/HWVersion.h>

/*qcom*/
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/usb/msm_hsusb.h>
#define  QCOM_PLATFORM 0
#define COVER_ATTACH_GPIO 1011
#define USB_HS_ID 1012
#define MULT_NP_DET 902

static int cover_type_report(struct notifier_block *nb, unsigned long event, void *ptr);
static struct notifier_block cover_type_notifier = {
    .notifier_call = cover_type_report,
};
extern int cover_cable_status_register_client(struct notifier_block *nb);
extern int cover_cable_status_unregister_client(struct notifier_block *nb);
static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr);
static struct notifier_block charger_type_notifier = {
    .notifier_call = charger_type_report,
};
extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);

struct smb345_otg_regulator {
    struct regulator_desc	rdesc;
    struct regulator_dev	*rdev;
};

extern int powerless_leave(bool status);
extern void upi_ug31xx_attach(bool);
extern bool COVER_ATTACHED(void);
extern bool COVER_ATTACHED_UPI(void);
extern bool CHARGING_FULL(void);
extern bool COVER_PRE_CHARGING_MODE(void);
extern bool VBUS_IN(void);
extern bool DCIN_VBUS(void);
extern bool IS_CA81(void);
extern bool IS_CB81(void);
extern bool _IS_CA81_(void);
extern bool _IS_CB81_(void);
extern void SET_COVER_DETACHED(void);
extern bool disable_cover_otg(void);
extern int  smb3xxc_pre_config(bool initial, bool cover_changed_cable_changed);
extern int  smb3xxc_soc_control_jeita(void);
extern int  smb3xxc_charger_control_jeita(void);
extern int  smb345c_charging_toggle(charging_toggle_level_t level, bool on);
extern int  smb345c_dump_registers(struct seq_file *s);
extern int  smb3xxc_jeita_control_for_sw_gauge(int usb_state, bool exec_jeita);
extern void cover_aicl(int I_USB_IN);
extern void cover_usbin(bool turn_on);
extern void cover_otg_current(int curr);
extern void cover_otg(int on);
extern int cover_get_aicl_results(void);

static int HW_ID;
extern int Read_HW_ID(void);
extern int entry_mode;
//static void smb345_config_max_current(int usb_state, bool cover_changed_cable_changed);
static void smb3xx_config_max_current(int usb_state, bool cover_changed_cable_changed);
void pad_cover_aicl(int pad_bat_rsoc, int cover_rsoc);
void cover_otg_control(void);
void cover_with_sdp(int pad_bat_rsoc, int cover_rsoc, bool cover_changed_cable_changed);
static bool g_is_power_supply_registered = false;
int cos_keep_power_on_with_cb81 = false;
int g_Flag1 = 0;
int g_Flag4 = 0;
static int g_Flag2 = 0;
static int g_Flag3 = 0;
static bool upi_ug31xx_attach_state = false;
static int powerless_cover = false;
static int set_powerless_leave = false;
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
static bool screen_off = false;
#endif

static const char *getCableCoverString[] = {
    "USB_IN",
    "AC_IN",
    "CABLE_OUT",
    "ENABLE_5V",
    "DISABLE_5V",
    "COVER_ATTACH",
    "COVER_DETTACH",
};

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
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
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

struct smb345_otg_event {
    struct list_head    node;
    bool            param;
};

struct smb345_charger {
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
    bool            otg_enabled;
    bool            otg_battery_uv;
    const struct smb345_charger_platform_data    *pdata;
    charger_jeita_status_t charger_jeita_status;
    /* wake lock to prevent S3 during charging */
    struct wake_lock wakelock;

    /* qcom */
    struct smb345_otg_regulator    otg_vreg;
    int smb_otg_en_gpio;
    int smb_acok_gpio;
    int vbus_in_det_n_gpio;
    int dcin_vbus_in_det_n_gpio;
    int smb_chg_en_gpio;
    spinlock_t        ibat_change_lock;
    struct power_supply    *usb_psy;
    struct delayed_work    cover_detect_work;
    struct delayed_work    cos_cb81_power_detect_work;
};

struct wake_lock wlock;
struct wake_lock wlock_t;
#ifdef ME372CG_ENG_BUILD
bool eng_charging_limit = true;
bool flag_eng_stop_charging = false;
int charging_limit_threshold = CHARGING_LIMIT_THRESHOLD;
static ssize_t nasus_charging_limit_toggle_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[64] = {0};

    if (count > sizeof(proc_buf)) {
        BAT_DBG("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    BAT_DBG_E(" %s: %s", __func__, proc_buf);

    if (!strncmp("1", proc_buf, 1)) {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
        BAT_DBG_E(" %s: turn on charging limit: %s", __func__, proc_buf);
    }
    else if (!strncmp("0", proc_buf, 1)) {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
        BAT_DBG_E(" %s: turn off charging limit: %s", __func__, proc_buf);
    }

    aicl_dete_worker(NULL);
    request_power_supply_changed();

    return count;
}
static int nasus_charging_limit_toggle_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: ENG charging limit %s\n", __func__, eng_charging_limit ? "On" : "Off");
    return 0;
}
static int nasus_charging_limit_toggle_open(struct inode *inode, struct file *file)
{
    return single_open(file, nasus_charging_limit_toggle_read, NULL);
}
//==============================================================================
static int nasus_charging_threshold_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", charging_limit_threshold);
    BAT_DBG_E(" %s:%d\n", __func__, charging_limit_threshold);

    return 0;
}
static ssize_t nasus_charging_threshold_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[64];
    char backup_fbuf[4];
    char default_charging_limit[4];
    static bool backup_file_ready = false;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    int byte_count= 0;
    s32 res;

    if (count > sizeof(proc_buf)) {
        BAT_DBG("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (proc_buf[0] == 's') {

        BAT_DBG("%s: ************ CHARGER LIMIT THRESHOLD start ************\n",
            __func__);

        /* partition is ready for writing/reading */
        backup_file_ready = true;

        /* read value from file */
        fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE, O_RDWR, 0666);
        if (!IS_ERR_OR_NULL(fp) ) {
            /* Get current segment descriptor and Set segment
            *  descriptor associated to kernel space.
            */
            old_fs = get_fs();
            set_fs(KERNEL_DS);

            /* read file */
            byte_count= fp->f_op->read(fp, backup_fbuf, sizeof(backup_fbuf), &fp->f_pos);
            BAT_DBG(" %s: read data from file: %s\n", __func__, backup_fbuf);

            /* write default value to file if it's empty */
            if (!strncmp("", backup_fbuf, 1)) {
                BAT_DBG_E(" %s: file exist but is empty! Write default to file\n",
                    __func__);

                sprintf(default_charging_limit, "%d", charging_limit_threshold);
                fp->f_op->llseek(fp, 0, 0);
                fp->f_op->write(fp,
                    default_charging_limit,
                    sizeof(default_charging_limit),
                    &fp->f_pos);

                goto file_open_but_error;
            }

            /* give up if fail to transition */
            if (kstrtos32(backup_fbuf, 10, &res)) {
                BAT_DBG_E(" %s: kstrtos32 error!", __func__);

                goto file_open_but_error;
            }
            BAT_DBG(" %s: read data from file(int): %d\n", __func__, res);

            /* replace the charging limit threshold with new value */
            charging_limit_threshold = res;

            /* Restore segment descriptor
            */
            set_fs(old_fs);

            /* Close file operation
            */
            filp_close(fp, NULL);
        }
        else {
            BAT_DBG_E("%s: file open error (%s)\n",
                __func__,
                CHARGING_LIMIT_THRESHOLD_FILE);
            return -EFAULT;
        }
    }
    else {
        /* directly return if backup file is not ready */
        if (!backup_file_ready) {
            BAT_DBG_E("%s: backup file not ready!\n", __func__);
            return count;
        }

        /* give up if fail to transition */
        if (kstrtos32_from_user(buf, count, 10, &res)) {
            BAT_DBG_E(" %s: kstrtos32 error!", __func__);
            BAT_DBG_E(" %s: proc_buf is %s", __func__, proc_buf);
            return -EFAULT;
        }

        /* replace the charging limit threshold with new value */
        if (0 <= res && res <= 100) {

            /* backup new value to file */
            fp = filp_open(CHARGING_LIMIT_THRESHOLD_FILE,
                    O_WRONLY | O_CREAT | O_TRUNC,
                    0666);
            if (!IS_ERR_OR_NULL(fp) ) {
                /* Get current segment descriptor and Set segment
                *  descriptor associated to kernel space.
                */
                old_fs = get_fs();
                set_fs(KERNEL_DS);

                /* replace the charging limit threshold with new value */
                charging_limit_threshold = res;

                sprintf(default_charging_limit, "%d", charging_limit_threshold);
                fp->f_op->llseek(fp, 0, 0);
                fp->f_op->write(fp,
                    default_charging_limit,
                    sizeof(default_charging_limit),
                    &fp->f_pos);

                /* Restore segment descriptor
                */
                set_fs(old_fs);

                /* Close file operation
                */
                filp_close(fp, NULL);
            }
            else {
                BAT_DBG_E("%s: file open error (%s)\n",
                    __func__,
                    CHARGING_LIMIT_THRESHOLD_FILE);
                return -EFAULT;
            }
        }
        else {
            BAT_DBG_E(" %s: out of normal range, skip it!\n", __func__);
            return -EFAULT;
        }
    }

    aicl_dete_worker(NULL);

    return count;

file_open_but_error:
    /* Restore segment descriptor
    */
    set_fs(old_fs);

    /* Close file operation
    */
    filp_close(fp, NULL);
    return -EFAULT;
}
static int nasus_charging_threshold_open(struct inode *inode, struct file *file)
{
    return single_open(file, nasus_charging_threshold_read, NULL);
}

int init_asus_charging_limit_toggle(void)
{
    struct proc_dir_entry *entry;
    struct proc_dir_entry *entry2;

    /* proc interface for charging limit threshold */
    static const struct file_operations asus_charging_threshold_fops = {
        .owner = THIS_MODULE,
        .open = nasus_charging_threshold_open,
        .read = seq_read,
        .write = nasus_charging_threshold_write,
        .llseek = seq_lseek,
        .release = single_release,
    };

    static const struct file_operations charging_limit_toggle_fops = {
        .owner = THIS_MODULE,
        .open = nasus_charging_limit_toggle_open,
        .read = seq_read,
        .write = nasus_charging_limit_toggle_write,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/charger_limit_enable", 0666, NULL,
        &charging_limit_toggle_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }

    entry2 = proc_create("driver/charging_limit_threshold", 0666, NULL,
        &asus_charging_threshold_fops);
    if (!entry2) {
        BAT_DBG_E("Unable to create /proc/driver/charging_limit_threshold\n");
        return -EINVAL;
    }

    return 0;
}
#else
static int twinsheaddragon_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: Enable\n", __func__);

    setSMB345Charger(AC_IN);

    return 0;
}
static int twinsheaddragon_open(struct inode *inode, struct file *file)
{
    return single_open(file, twinsheaddragon_read, NULL);
}
int init_asus_charging_limit_toggle(void)
{
    struct proc_dir_entry *entry;

    static const struct file_operations twinsheaddragon_fops = {
        .owner = THIS_MODULE,
        .open = twinsheaddragon_open,
        .read = seq_read,
        .write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/twinsheaddragon", 0666, NULL,
        &twinsheaddragon_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create twinsheaddragon\n");
        return -EINVAL;
    }

    return 0;
}
#endif

/* global charger type variable lock */
DEFINE_MUTEX(g_usb_state_lock);
static int g_usb_state = CABLE_OUT;
static int g_cover_state = COVER_DETTACH;
static int g_extern_device = CABLE_OUT;
/* global software charging toggle lock */
DEFINE_MUTEX(g_charging_toggle_lock);
static bool g_charging_toggle = true;


static struct smb345_charger *smb345_dev;

/* AICL Results Table (mA) : 3Fh */
#if defined(CONFIG_A400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_FE380CG) || defined(CONFIG_FE170CG) || defined(CONFIG_FE171MG)
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
#else
/* PF400CG, Z580C, Z300KL, A503ML adopted smb345, smb346, smb347 */
static const unsigned int aicl_results[] = {
    300,
    500,
    700,
    900,
    1200,
    1500,
    1800,
    2000,
    2200,
    2500,
};
#endif

#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

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
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 6)

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
#define CHG_STATUS_MASK        SMB345_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT        BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK            SMB345_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK        SMB345_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define TERMINATION_CURRENT_MASK        SMB345_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK    SMB345_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK                SMB345_MASK(6, 0)
#define CHG_ENABLE_BIT            BIT(1)
#define VOLATILE_W_PERM_BIT        BIT(7)
#define USB_SELECTION_BIT        BIT(1)
#define SYSTEM_FET_ENABLE_BIT    BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT    BIT(2)
#define BATT_OV_END_CHG_BIT        BIT(1)
#define VCHG_FUNCTION            BIT(0)
#define CURR_TERM_END_CHG_BIT    BIT(6)

#define OTGID_PIN_CONTROL_MASK    SMB345_MASK(2, 6)
#define OTGID_PIN_CONTROL_BITS    BIT(6)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)

#define CHARGE_CURRENT_COMPENSATION         SMB345_MASK(2, 6)
#define CHARGE_CURRENT_COMPENSATION_VALUE   0x00

#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS
#define SMB358_FAST_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define SMB345_FAST_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define SMB358_TERMINATION_CURRENT_MASK         SMB345_MASK(3, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_200mA    BIT(0) | BIT(1) | BIT(2)
#define SMB358_TERMINATION_CURRENT_VALUE_80mA    BIT(1) | BIT(0)
#define SMB358_TERMINATION_CURRENT_VALUE_100mA   BIT(2)
#define SMB358_FAST_CHG_CURRENT_VALUE_450mA    BIT(5)
#define SMB358_FAST_CHG_CURRENT_VALUE_600mA    BIT(6)
#define SMB358_FAST_CHG_CURRENT_VALUE_900mA    BIT(5) | BIT(6)
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    BIT(5) | BIT(6) | BIT(7)

void COVER_CHARGING_TOGGLE(bool toggle)
{
    if (!smb345_dev) return;
    if (!gpio_is_valid(smb345_dev->smb_chg_en_gpio)) {
        pr_err("%s: smb_chg_en_gpio is not valid\n", __func__);
        return;
    }
    gpio_set_value(smb345_dev->smb_chg_en_gpio, toggle ? 1 : 0);
}

static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);
static inline int get_pack_bat_voltage(int *volt);
static inline int get_pack_bat_rsoc(int *rsoc);

static char *supply_list[] = {
    "pack_ac",
    "pack_bat",
};

#if 0
static const char *chgr_to_string(enum power_supply_type psy_type)
{
    switch (psy_type) {
    case POWER_SUPPLY_TYPE_USB:             return "POWER_SUPPLY_TYPE_USB";
    case POWER_SUPPLY_TYPE_USB_DCP:         return "POWER_SUPPLY_TYPE_USB_DCP";
    case POWER_SUPPLY_TYPE_USB_CDP:         return "POWER_SUPPLY_TYPE_USB_CDP";
    case POWER_SUPPLY_TYPE_USB_ACA:         return "POWER_SUPPLY_TYPE_USB_ACA";
    case POWER_SUPPLY_TYPE_WIRELESS:        return "POWER_SUPPLY_TYPE_WIRELESS";
    case POWER_SUPPLY_TYPE_BMS:             return "POWER_SUPPLY_TYPE_BMS";
    case POWER_SUPPLY_TYPE_USB_PARALLEL:    return "POWER_SUPPLY_TYPE_USB_PARALLEL";
    case POWER_SUPPLY_TYPE_BATTERY:         return "POWER_SUPPLY_TYPE_BATTERY";
    case POWER_SUPPLY_TYPE_UPS:             return "POWER_SUPPLY_TYPE_UPS";
    case POWER_SUPPLY_TYPE_MAINS:           return "POWER_SUPPLY_TYPE_MAINS";
    case POWER_SUPPLY_TYPE_UNKNOWN:         return "POWER_SUPPLY_TYPE_UNKNOWN";
    default:                                return "INVALID_CHARGER";
    }
}

/* qcom */
static void smb345_ac_external_power_changed(struct power_supply *psy)
{
    static int usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
    union power_supply_propval ret = {0,};
    unsigned long flags;

    spin_lock_irqsave(&smb345_dev->ibat_change_lock, flags);

    if (!smb345_dev->usb_psy)
        smb345_dev->usb_psy = power_supply_get_by_name("usb");
    smb345_dev->usb_psy->get_property(smb345_dev->usb_psy, POWER_SUPPLY_PROP_TYPE, &ret);

    pr_info("%s: * %s *\n", __func__, chgr_to_string(ret.intval));
    spin_unlock_irqrestore(&smb345_dev->ibat_change_lock, flags);

    /* ignore it with duplicate notification */
    if (usb_type == ret.intval)
        return;
    usb_type = ret.intval;

    switch (usb_type) {
    case POWER_SUPPLY_TYPE_USB_DCP:
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_USB_ACA:
    case POWER_SUPPLY_TYPE_WIRELESS:
    case POWER_SUPPLY_TYPE_BMS:
    case POWER_SUPPLY_TYPE_USB_PARALLEL:
    case POWER_SUPPLY_TYPE_UPS:
    case POWER_SUPPLY_TYPE_MAINS:
        break;
    case POWER_SUPPLY_TYPE_USB:
        break;
    case POWER_SUPPLY_TYPE_BATTERY:
    case POWER_SUPPLY_TYPE_UNKNOWN:
        break;
    default:
        break;
    }
}
#endif

static enum power_supply_property asus_power_properties[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
};

static int smb345_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val);

static struct power_supply smb345_power_supplies[] = {
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
#if QCOM_PLATFORM
        .external_power_changed = smb345_ac_external_power_changed,
#endif
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = smb345_power_get_property,
    },
    {
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = smb345_power_get_property,
    },
    {
        .name = "otg",
        .type = POWER_SUPPLY_TYPE_OTG,
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = smb345_power_get_property,
    },
};

static int smb345_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    int ret=0;
    int usb_state;
    int chrg_status;

    mutex_lock(&g_usb_state_lock);
    usb_state = g_usb_state;
    mutex_unlock(&g_usb_state_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
            val->intval = (usb_state == USB_IN) ? 1 : 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            val->intval = (usb_state == AC_IN) ? 1 : 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_OTG) {
            val->intval = gpio_get_value(USB_HS_ID) ? 0 : 1;
        }
        else {
            ret = -EINVAL;
        }
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
            /* for ATD test to acquire the status about charger ic */
            if (!smb345_has_charger_error()) {
                val->intval = 1;
                return 0;
            }

            chrg_status = smb345_get_charging_status();
            if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
                val->intval = 1;
                return 0;
            }
            else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
                val->intval = 1;
                return 0;
            }
            val->intval = 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            if (!smb345_has_charger_error()) {
                val->intval = 1;
                return 0;
            }

            chrg_status = smb345_get_charging_status();
            if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
                val->intval = 1;
                return 0;
            }
            else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
                val->intval = 1;
                return 0;
            }
            val->intval = 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_OTG) {
            val->intval = gpio_get_value(USB_HS_ID) ? 0 : 1;
        }
        else
            ret = -EINVAL;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

extern bool get_sw_charging_toggle()
{
    bool ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return false;
    }

    mutex_lock(&g_charging_toggle_lock);
    ret = g_charging_toggle;
    mutex_unlock(&g_charging_toggle_lock);

    return ret;
}

extern int smb3xx_get_charger_type()
{
    int ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return CABLE_OUT;
    }

    mutex_lock(&g_usb_state_lock);
        ret = g_usb_state;
    mutex_unlock(&g_usb_state_lock);

    return ret;
}

int request_power_supply_changed()
{
    int ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return -1;
    }

    if (!g_is_power_supply_registered)
        return -1;

    power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
#if QCOM_PLATFORM
    power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
#endif
    power_supply_changed(&smb345_power_supplies[2]);

    return ret;
}

static int smb345_register_power_supply(struct device *dev)
{
    int ret = 0;

    //ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_USB-1]);
    if (ret) {
        BAT_DBG_E("Fail to register power supply USB\n");
        goto batt_err_reg_fail_usb;
    }

    ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_AC-1]);
    if (ret) {
        BAT_DBG_E("Fail to register power supply AC\n");
        goto batt_err_reg_fail_ac;
    }

    ret = power_supply_register(dev, &smb345_power_supplies[2]);
    if (ret) {
        BAT_DBG_E("Fail to register power supply OTG\n");
        goto batt_err_reg_fail_ac;
    }

    g_is_power_supply_registered = true;
    return 0;

batt_err_reg_fail_ac:
#if QCOM_PLATFORM
    power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
#endif
batt_err_reg_fail_usb:
    return ret;
}

static int smb345_read_reg(struct i2c_client *client, int reg,
                u8 *val, int ifDebug)
{
    s32 ret;
    struct smb345_charger *smb345_chg;

    smb345_chg = i2c_get_clientdata(client);
    ret = i2c_smbus_read_byte_data(smb345_chg->client, reg);
    if (ret < 0) {
        dev_err(&smb345_chg->client->dev,
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

static int smb345_write_reg(struct i2c_client *client, int reg,
                        u8 val)
{
    s32 ret;
    struct smb345_charger *smb345_chg;

    smb345_chg = i2c_get_clientdata(client);

    ret = i2c_smbus_write_byte_data(smb345_chg->client, reg, val);
    if (ret < 0) {
        dev_err(&smb345_chg->client->dev,
            "i2c write fail: can't write %02X to %02X: %d\n",
            val, reg, ret);
        return ret;
    }
    return 0;
}

static int smb345_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345_read_reg(client, reg, &temp, 0);
        if (rc) {
            retry_count--;
            pr_err("*smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
        u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
    rc = smb345_read_reg(client, reg, &temp, 0);
    if (rc) {
        retry_count--;
        pr_err("*smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
    rc = smb345_write_reg(client, reg, temp);
    if (rc) {
        retry_count--;
        pr_err("*smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}

static int smb345_read(struct smb345_charger *smb, u8 reg)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        ret = i2c_smbus_read_byte_data(smb->client, reg);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to read reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int smb345_write(struct smb345_charger *smb, u8 reg, u8 val)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        ret = i2c_smbus_write_byte_data(smb->client, reg, val);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to write reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int smb345_set_writable(struct smb345_charger *smb, bool writable)
{
    int ret;

    ret = smb345_read(smb, CMD_A);
    if (ret < 0)
        return ret;

    if (writable)
        ret |= CMD_A_ALLOW_WRITE;
    else
        ret &= ~CMD_A_ALLOW_WRITE;

    return smb345_write(smb, CMD_A, ret);
}

int smb345_get_soc_control_float_vol(int bat_tempr, int bat_volt)
{
    int ret;

    /* read 03h[5:0]="011110" or "101010"*/
    ret = smb345_masked_read(smb345_dev->client,
                FLOAT_VOLTAGE_REG,
                FLOAT_VOLTAGE_MASK);

    BAT_DBG_E(" Charger get battery info T:%d, V:%d, 03h[5:0]=" BYTETOBINARYPATTERN "\n",
        bat_tempr, bat_volt, BYTETOBINARY(ret));
    return ret;
}

#if defined(CONFIG_Z380KL)
int smb3xx_soc_control_jeita(void)
{
    int ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    BAT_DBG(" %s:\n", __func__);

    /* Set Hard Hot Limit as 72 Deg. C 0Bh[5:4]="11" */
    ret = smb345_masked_write(smb345_dev->client,
            0x0B,
            BIT(5) | BIT(4),
            BIT(5) | BIT(4));

    /* Set Soft Hot Limit Behavior as No Response 07h[1:0]="00" */
    ret = smb345_masked_write(smb345_dev->client,
            0x07,
            BIT(1) | BIT(0),
            0);

    /* Set Soft Cold Temperature Limit as No Response 07h[3:2]="00" */
    ret = smb345_masked_write(smb345_dev->client,
            0x07,
            BIT(3) | BIT(2),
            0);

    /* cover soc control jeita */
    if (VBUS_IN() && gpio_get_value(USB_HS_ID) && COVER_ATTACHED_UPI())
    ret = smb3xxc_soc_control_jeita();

    if (ret)
        BAT_DBG(" %s: i2c communication failure!", __func__);

    return ret;
}
#else
int smb3xx_soc_control_jeita(void) { return 0; }
#endif

#if defined(CONFIG_Z380KL)
int smb3xx_charger_control_jeita(void)
{
    int ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    BAT_DBG(" %s:\n", __func__);

    ret = smb345_set_writable(smb345_dev, true);

    /* Set Hard Hot Limit as 53 Deg. C 0Bh[5:4]="00" */
    ret = smb345_masked_write(smb345_dev->client,
            0x0B,
            BIT(5) | BIT(4),
            0);

    /* Set Soft Hot Limit Behavior as Float Voltage Compensation 07h[1:0]="10" */
    ret = smb345_masked_write(smb345_dev->client,
            0x07,
            BIT(1) | BIT(0),
            BIT(1));

    /* Set Soft Cold Temp Limit as Charger Current Compensation 07h[3:2]="01" */
    ret = smb345_masked_write(smb345_dev->client,
            0x07,
            BIT(3) | BIT(2),
            BIT(2));

    /* Charging Enable */
    ret = smb345_charging_toggle(JEITA, true);

    /* cover charger control jeita */
    if (VBUS_IN() && gpio_get_value(USB_HS_ID) && COVER_ATTACHED_UPI())
    ret = smb3xxc_charger_control_jeita();

    if (ret)
        BAT_DBG(" %s: i2c communication failure!", __func__);

    return ret;
}
#else
int smb3xx_charger_control_jeita(void) { return 0; }
#endif

#if defined(CONFIG_Z380KL)
int smb3xx_jeita_control(int bat_tempr, int bat_volt, bool exec_jeita)
{
    static charger_jeita_status_t ori_charger_jeita_status = ROOM;
    int ret;

    if (!smb345_dev) {
        pr_err("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return POWER_SUPPLY_STATUS_DISCHARGING;
    }

    if (exec_jeita) smb3xx_soc_control_jeita();

    ori_charger_jeita_status = smb345_dev->charger_jeita_status;

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
            if (bat_volt >= 4100) {
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
        if (smb345_get_soc_control_float_vol(bat_tempr, bat_volt) == 0x2A) {
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                if (bat_volt >= 4100) {
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
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    }
    smb345_dev->charger_jeita_status = FREEZE;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_II:
    if (exec_jeita) {
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    }
    smb345_dev->charger_jeita_status = COLD;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_III:
    if (exec_jeita) {
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    }
    smb345_dev->charger_jeita_status = ROOM;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_IV:
    if (exec_jeita) {
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
                     BIT(4) | BIT(3) | BIT(2) | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    }
    smb345_dev->charger_jeita_status = HOT;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_V:
    if (exec_jeita) {
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    }
    smb345_dev->charger_jeita_status = HOT;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_VI:
    if (exec_jeita) {
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
                     BIT(4) | BIT(3) | BIT(2) | BIT(1));
    ret = smb345_masked_write(smb345_dev->client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    }
    smb345_dev->charger_jeita_status = OVERHEAT;
    if (ori_charger_jeita_status != smb345_dev->charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            smb345_dev->charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

others:
    return POWER_SUPPLY_STATUS_CHARGING;
}
#else
int smb3xx_jeita_control(int bat_tempr, int bat_volt, bool exec_jeita)
{
    return POWER_SUPPLY_STATUS_CHARGING;
}
#endif

int smb3xx_jeita_control_for_sw_gauge(int usb_state)
{
    int ret;
    int batt_tempr = 250;/* unit: C  */
    int batt_volt = 4000;/* unit: mV */
    bool exec_jeita = false;

#if 0
    if (usb_state != AC_IN && usb_state != USB_IN && !COVER_ATTACHED_UPI())
        return 0;
#endif

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

    /* pad jeita function */
    if (usb_state == AC_IN ||
        (usb_state == USB_IN && !COVER_ATTACHED()) ||
        (COVER_ATTACHED_UPI() && IS_CB81()) ||
        (usb_state == USB_IN && COVER_ATTACHED_UPI() && g_Flag2 == 1))
        exec_jeita = true;

    ret = smb3xx_jeita_control(batt_tempr, batt_volt, exec_jeita);
    exec_jeita = false;

    /* cover jeita */
    if (COVER_ATTACHED_UPI())
        if ((usb_state == AC_IN) || 
            (usb_state == USB_IN && g_Flag2 == 0))
            exec_jeita = true;

    ret = smb3xxc_jeita_control_for_sw_gauge(usb_state, exec_jeita);

    //request_power_supply_changed();
    return ret;
}

/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl,
                    size_t size, unsigned int val)
{
    if (val >= size)
        return tbl[size-1];;
    return tbl[val];
}

/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
    int ret;

    ret = smb345_read(smb345_dev, STAT_E);
    if (ret < 0) {
        BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
        return ret;
    }

    ret &= 0x0F;
    return hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    if (!smb345_dev) {
        pr_info("%s: ERROR: smb345_dev is null "
            "due to probe function has error\n",
            __func__);
        return sprintf(buf, "%d\n", -EINVAL);
    }

    ret = smb345_read(smb345_dev, STAT_E);
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

    ret = smb345_get_charging_status();
    if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
        ret = 1;
    else
        ret = 0;
    return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
    &dev_attr_input_current.attr,
    &dev_attr_charge_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};

#define SMB_DUMP(...) \
do { \
        local_len = sprintf(page, __VA_ARGS__); \
        len += local_len; \
        page += local_len; \
}while(0);

/*----------------------------------------------------------------------------*/


#if defined(CONFIG_Z300KL) || defined(CONFIG_Z380KL)
static int config_otg_regs(int toggle, bool no_power_cover_test)
{
    int ret;

    if (toggle) {
        /* GPIO_17 set output High */
        if (!no_power_cover_test) gpio_set_value(smb345_dev->smb_otg_en_gpio, 1);

        /* Set OTG current limit to 250mA 0Ah[3:2]="01" */
        ret = smb345_masked_write(smb345_dev->client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(2));
        if (ret) {
            pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
            return ret;
        }

        if (!no_power_cover_test) {
        /* set OTG to Pin Control 09h[7:6]="01" */
        ret = smb345_masked_write(smb345_dev->client,
            0x09,
            BIT(7) | BIT(6),
            BIT(6));
        if (ret) {
            pr_err("fail to set OTG enable bit 09h ret=%d\n", ret);
            return ret;
        }
        }
        else {
            /* set OTG to I2C Control 09h[7:6]="00" */
            ret = smb345_masked_write(smb345_dev->client,
                0x09,
                BIT(7) | BIT(6),
                0);
            if (ret) {
                pr_err("fail to set OTG I2C control ret=%d\n", ret);
                return ret;
            }

            /* OTG enable by I2C 30h[4]="1" */
            ret = smb345_masked_write(smb345_dev->client,
                0x30,
                BIT(4),
                BIT(4));
            if (ret) {
                pr_err("fail to set OTG enable by I2C ret=%d\n", ret);
                return ret;
            }
        }

        /* Set OTG current limit to 500mA 0Ah[3:2]="10" */
        ret = smb345_masked_write(smb345_dev->client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(3));
        if (ret) {
            pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
            return ret;
        }
    }
    else {
        /* Set OTG current limit to 250mA 0Ah[3:2]="01" */
        ret = smb345_masked_write(smb345_dev->client,
            0x0A,
            BIT(3) | BIT(2),
            BIT(2));
        if (ret) {
            pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
            return ret;
        }

        /* GPIO_17 set output Low */
        if (!no_power_cover_test) gpio_set_value(smb345_dev->smb_otg_en_gpio, 0);

        if (no_power_cover_test) {
            /* OTG disable by I2C 30h[4]="0" */
            ret = smb345_masked_write(smb345_dev->client,
                0x30,
                BIT(4),
                0);
            if (ret) {
                pr_err("fail to set OTG disable by I2C ret=%d\n", ret);
                return ret;
            }

            /* set OTG to Pin Control 09h[7:6]="01" */
            ret = smb345_masked_write(smb345_dev->client,
                0x09,
                BIT(7) | BIT(6),
                BIT(6));
            if (ret) {
                pr_err("fail to set OTG to Pin control ret=%d\n", ret);
                return ret;
            }
        }
    }

    BAT_DBG(" %s: %s\n", __func__, toggle ? "ON" : "OFF");
    return ret;
}
#else
static int config_otg_regs(int toggle, bool no_power_cover_test) { return 0; }
#endif
/*----------------------------------------------------------------------------*/

static int otg(int toggle, bool no_power_cover_test)
{
    int ret;

    if (!smb345_dev) {
        pr_info("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    ret = smb345_set_writable(smb345_dev, true);
    if (ret < 0)
        return ret;


    ret = config_otg_regs(toggle, no_power_cover_test);
    if (ret < 0)
        return ret;

    smb345_dev->otg_enabled = (toggle > 0 ? true : false);
    return 0;
}

#if defined(CONFIG_Z380KL)
static int aicl_cur_control(int usb_state)
{
    int aicl_result;
    static bool first_enter = true;

    if (!COVER_ATTACHED()) {
        /* ignore non-AC_IN */
        if (usb_state != AC_IN) {
            /* reset to default if non-AC_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* AC_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        aicl_result = get_aicl_results();
        if (aicl_result < 1200) {
            dev_err(&smb345_dev->client->dev,
                "%s: * re-config AC_IN when aicl result(%dmA) < 1200mA.\n",
                __func__,
                aicl_result);
            smb3xx_config_max_current(usb_state, false);
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n",
                __func__);
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control(int usb_state, bool rsoc_changed, int pad_bat_rsoc, int cover_rsoc)
{
    int aicl_result;
    int cover_aicl_result;
    static bool first_enter = true;

    if (COVER_ATTACHED()) {
        /* ignore non-AC_IN */
        if (usb_state != AC_IN) {
            /* reset to default if non-AC_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* AC_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        cover_aicl_result = cover_get_aicl_results();
        aicl_result = get_aicl_results();
        if (aicl_result <= 500 || cover_aicl_result < 500) {
            BAT_DBG(
                "%s: * re-config AC_IN & Cover when aicl result(%dmA) <= 500mA.\n",
                __func__,
                aicl_result);

            /* cover pre config */
            smb3xxc_pre_config(true, true);
            smb345_set_writable(smb345_dev, true);

            /* disable cover otg */
            if (DCIN_VBUS()) disable_cover_otg();

            /* Enable Cover Charging with DCP */
            COVER_CHARGING_TOGGLE(true);

            /* Enable PAD Charger USBIN 30h[2]="0" */
            if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
                BAT_DBG(
                    "%s: fail to set max current limits for USB_IN\n",
                    __func__);

            /* Suspend PAD Charger DCIN 31h[2]="1" */
            if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                BAT_DBG(
                    "%s: fail to set max current limits for USB_IN\n",
                    __func__);

            /* check PAD battery rsoc & Cover battery rsoc */
            pad_cover_aicl(pad_bat_rsoc, cover_rsoc);
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(usb_state);
#endif
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n",
                __func__);
            if (rsoc_changed) {
                BAT_DBG(" %s: rsoc changed -> A\n", __func__);
                /* check PAD battery rsoc & Cover battery rsoc */
                pad_cover_aicl(pad_bat_rsoc, cover_rsoc);
#if defined(CONFIG_Z380C)
                usb_to_battery_callback(usb_state);
#endif
            }
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control_sdp(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc)
{
    static bool first_enter = true;

    if (COVER_ATTACHED()) {
        /* ignore non-USB_IN */
        if (usb_state != USB_IN) {
            /* reset to default if non-USB_IN */
            first_enter = true;
            return 0;
        }
        else {
            /* USB_IN */
            if (first_enter) {
                /* don't AICL when first enter */
                first_enter = false;
                return 0;
            }
        }

        if (rsoc_changed) {
            BAT_DBG(" %s: rsoc changed -> C\n", __func__);
            /* sdp_ */
            cover_with_sdp(pad_rsoc, cover_rsoc, rsoc_changed);
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(usb_state);
#endif
        }
    }
    else first_enter = true;

    return 0;
}
static int pad_cover_aicl_cur_control_no_cable(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc)
{
    int aicl_result;
    static bool first_enter = true;
    bool B_state_start = false;

    if (COVER_ATTACHED() && IS_CB81()) {
        if (usb_state != CABLE_OUT) {
            first_enter = true;
            return 0;
        }
        else {
            if (first_enter) {
                first_enter = false;
BAT_DBG("%s:\n", __func__);
                return 0;
            }
        }

        if (cover_rsoc >= 0 && cover_rsoc < 5 && !VBUS_IN()) {
            disable_cover_otg();
            BAT_DBG(" %s: disable cover otg due to cover rsoc < 5\n", __func__);
#if defined(CONFIG_Z380C)
            usb_to_battery_callback(NO_CABLE);
#endif
            //msleep(1000);
            smb345_set_writable(smb345_dev, true);
            return 0;
        }

        aicl_result = get_aicl_results();
        if (aicl_result <= 700) {
            if (g_Flag1 == 0 && pad_rsoc >= 70) {
                BAT_DBG("%s: * No need to re-config when aicl result(%dmA) <= 700mA."
                    " g_Flag1=0, pad_rsoc=%d\n" ,__func__, aicl_result, pad_rsoc);
                return 0;
            }
            BAT_DBG("%s: * re-config when aicl result(%dmA) <= 700mA.\n",
                __func__,
                aicl_result);

            /* cover pre config */
            smb3xxc_pre_config(true, true);

            COVER_CHARGING_TOGGLE(false);
            B_state_start = true;
        }
        else {
            BAT_DBG(" %s: execute AICL routine control work\n", __func__);
        }

        if (B_state_start || rsoc_changed) {
            BAT_DBG(" %s: rsoc changed -> B, g_Flag1=%d\n", __func__, g_Flag1);
            if (pad_rsoc >= 90 && g_Flag1 == 1) {

                BAT_DBG(" %s: * Suspend Pad USBIN DCIN *\n", __func__);

                /* reset to default */
                g_Flag1 = 0;
                g_Flag4 = 1;

                /* USB Suspend Mode: I2C Control 02h[7]="1" */
                if (smb345_masked_write(smb345_dev->client, 0x02, BIT(7), BIT(7)))
                    dev_err(&smb345_dev->client->dev,
                        "%s: fail to set USB Suspend Mode to I2C Control\n",
                        __func__);
                /* Suspend Pad Charger USBIN 30h[2]="1" */
                if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
                    dev_err(&smb345_dev->client->dev,
                        "%s: fail to Suspend Pad Charger USBIN\n",
                        __func__);
                /* Suspend Pad Charger DCIN 31h[2]="1" */
                if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                    dev_err(&smb345_dev->client->dev,
                        "%s: fail to set Suspend Pad Charger DCIN\n",
                    __func__);

                cover_otg(1);

#if defined(CONFIG_Z380C)
                usb_to_battery_callback(NO_CABLE);
#endif
            }
            else if ((pad_rsoc < 70 && g_Flag1 == 0) || (pad_rsoc >= 70 && g_Flag1 == 0 && pad_rsoc < 90 && g_Flag4 == 0)) {
                cover_otg_control();

                /* debug */
                smb345_dump_registers(NULL);
                smb345c_dump_registers(NULL);

#if defined(CONFIG_Z380C)
                usb_to_battery_callback(NO_CABLE);
#endif
            }
        }
    }
    else first_enter = true;

    return 0;
}
#else
static int aicl_cur_control(int usb_state) { return 0; }
static int pad_cover_aicl_cur_control(int usb_state, bool rsoc_changed, int pad_bat_rsoc, int cover_rsoc) { return 0; }
static int pad_cover_aicl_cur_control_no_cable(int usb_state, bool rsoc_changed, int pad_rsoc, int cover_rsoc) { return 0; }
#endif

void cover_wakelock(void)
{
    if (IS_CB81() || (VBUS_IN() && COVER_ATTACHED_UPI())) {
        if (!wake_lock_active(&wlock_t)) {
            BAT_DBG(" %s: Cover_WakeLock: *LOCK*\n", __func__);
            wake_lock(&wlock_t);
        }
    }
    else {
        if (wake_lock_active(&wlock_t)) {
            BAT_DBG(" %s: Cover_WakeLock: *UNLOCK*\n", __func__);
            wake_unlock(&wlock_t);
        }
    }
}

void aicl_dete_worker(struct work_struct *dat)
{
    int usb_state;
    int rsoc;
    int cover_rsoc;
    static int old_rsoc = -1;
    static int old_cover_rsoc = -1;
    bool rsoc_changed = false;

    BAT_DBG(" %s:\n", __func__);

    mutex_lock(&g_usb_state_lock);
    usb_state = g_usb_state;
    mutex_unlock(&g_usb_state_lock);

    /* acquire pad battery rsoc here */
    if (get_battery_rsoc(&rsoc)) {
        BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
        goto final;
    }
    else {
        if (old_rsoc != rsoc) {
            BAT_DBG(" %s: * RSOC changed from %d%% to %d%% *\n", __func__, old_rsoc, rsoc);
            old_rsoc = rsoc;
            rsoc_changed = true;
        }
    }

    /* acquire cover battery rsoc */
    if (COVER_ATTACHED_UPI()) {
        if (get_pack_bat_rsoc(&cover_rsoc)) {
            BAT_DBG(" %s: fail to get cover battery rsoc\n", __func__);
            goto final;
        }
        else {
            if (old_cover_rsoc != cover_rsoc || CHARGING_FULL()) {
                BAT_DBG(" %s: * cover RSOC changed from %d%% to %d%% *\n",
                    __func__, old_cover_rsoc, cover_rsoc);
                    old_cover_rsoc = cover_rsoc;
                    rsoc_changed = true;
            }
        }
    }

    if (COVER_ATTACHED_UPI())
        BAT_DBG(" %s: ==========================Pad Rsoc: %d%%, Cover Rsoc: %d%%==========================\n",
            __func__, rsoc, cover_rsoc);
    else
        BAT_DBG(" %s: ==========================Pad Rsoc: %d%%==========================\n",
            __func__, rsoc);

    smb345_set_writable(smb345_dev, true);
    aicl_cur_control(usb_state);
    pad_cover_aicl_cur_control(usb_state, rsoc_changed, rsoc, cover_rsoc);
    pad_cover_aicl_cur_control_sdp(usb_state, rsoc_changed, rsoc, cover_rsoc);
    pad_cover_aicl_cur_control_no_cable(usb_state, rsoc_changed, rsoc, cover_rsoc);
    smb3xx_jeita_control_for_sw_gauge(usb_state);

    if (rsoc==5 || rsoc==1)
        smb345_dump_registers(NULL);
    if (cover_rsoc==5 || cover_rsoc==1)
        smb345c_dump_registers(NULL);

    cover_wakelock();

#if 0
    if (dat) {
        if (IS_CB81() && cover_rsoc<=5)
            schedule_delayed_work(&smb345_dev->aicl_dete_work, 5*HZ);
        else
            schedule_delayed_work(&smb345_dev->aicl_dete_work, 30*HZ);
    }
#endif

    if (COVER_ATTACHED_UPI()) {
        if (cover_rsoc>=1) {
            if (!set_powerless_leave) {
                set_powerless_leave = true;
                powerless_leave(true);
                BAT_DBG(" %s: powerless_leave(true)\n", __func__);

                //power_supply_set_present(smb345_dev->usb_psy, VBUS_IN() ? 1 : 0);
            }
        }
        else {
            if (set_powerless_leave) {
                set_powerless_leave = false;
                powerless_leave(false);
                BAT_DBG(" %s: powerless_leave(false)\n", __func__);

                //power_supply_set_present(smb345_dev->usb_psy, VBUS_IN() ? 1 : 0);
            }
        }
    }

final:
    if (dat) {
        if (IS_CB81() && rsoc_changed)
            schedule_delayed_work(&smb345_dev->aicl_dete_work, HZ);
        else
            schedule_delayed_work(&smb345_dev->aicl_dete_work, 30*HZ);
    }
}
EXPORT_SYMBOL(aicl_dete_worker);

#if defined(CONFIG_Z380KL)
static int smb3xx_pre_config(bool cover_changed_cable_changed)
{
    int ret;

    smb345_set_writable(smb345_dev, true);

    /* set fast charge current: 2000mA */
    /* set pre charge current: 250mA */
    /* set termination current: 200mA */
    /**/
    if (cover_changed_cable_changed)
    ret = smb345_write(smb345_dev,
            0x00,
            0xbc);
    if (ret < 0)
        goto fail;

    /* set cold soft limit current: 900mA write 0Ah[7:6]="10"*/
    ret = smb345_masked_write(smb345_dev->client,
            0x0a,
            BIT(6) | BIT(7),
            BIT(7));
    if (ret < 0)
        goto fail;

    /* set Battery OV does not end charge cycle. 02h[5]="0", 02h[1]="0" */
    ret = smb345_masked_write(smb345_dev->client,
            0x02,
            BIT(5),
            0);
    if (ret < 0)
        goto fail;
    ret = smb345_masked_write(smb345_dev->client,
            0x02,
            BIT(1),
            0);
    if (ret < 0)
        goto fail;

    /* set Float Voltage: 4.34V. 03h[5:0]="101010" */
    if (cover_changed_cable_changed)
    ret = smb345_masked_write(smb345_dev->client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5) | BIT(3) | BIT(1));
    if (ret < 0)
        goto fail;

    /* set OTG/ID to Pin Control. 09h[7:6]="01" */
    ret = smb345_masked_write(smb345_dev->client,
            0x09,
            BIT(7) | BIT(6),
            BIT(6));
    if (ret < 0)
        goto fail;

    /* USB Suspend Mode: I2C control. 02h[7]="1" */
    ret = smb345_masked_write(smb345_dev->client,
            0x02,
            BIT(7),
            BIT(7));
    if (ret < 0)
        goto fail;

    /* Enable PAD Charger USBIN 30h[2]="0" */
    ret = smb345_masked_write(smb345_dev->client,
            0x30,
            BIT(2),
            0);
    if (ret < 0)
        goto fail;

    /* Enable PAD Charger DCIN 31h[2]="0" */
    ret = smb345_masked_write(smb345_dev->client,
            0x31,
            BIT(2),
            0);
    if (ret < 0)
        goto fail;

fail:
    return ret;
}
#else
static int smb3xx_pre_config(bool cover_changed_cable_changed) { return 0; }
#endif

/*----------------------------------------------------------------------------*/

#if defined(CONFIG_Z380KL)
static void smb3xx_config_max_current(int usb_state, bool cover_changed_cable_changed)
{
    if (usb_state == AC_IN && !COVER_ATTACHED()) {

        smb345_set_writable(smb345_dev, true);

        /* check if AICL Result < 1200mA 3fh[3:0]="0100" */
        if (smb345_masked_read(smb345_dev->client, 0x3f, 0x0f) < 0x04 || cover_changed_cable_changed) {

            /* Disable AICL - Write 02h[4]="0" */
            if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), 0x00)) {
                dev_err(&smb345_dev->client->dev,
                "%s: fail to disable AICL\n", __func__);
                return;
            }

            /* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
            if (smb345_masked_write(smb345_dev->client, 0x01, 0x0f, 0x04)) {
                dev_err(&smb345_dev->client->dev,
                "%s: fail to set max current limits for USB_IN\n",
                __func__);
                return;
            }

            /* Enable AICL - Write 02h[4]="1" */
            if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), 0x10)) {
                dev_err(&smb345_dev->client->dev,
                "%s: fail to enable AICL\n", __func__);
                return;
            }

            BAT_DBG(" %s: done\n", __func__);
        }
    }
}
#else
static void smb3xx_config_max_current(int usb_state, bool cover_changed_cable_changed) {}
#endif

//static void smb345_config_max_current(int usb_state, bool cover_changed_cable_changed)
//{
//    if (usb_state != AC_IN && usb_state != USB_IN && !COVER_ATTACHED())
//        return;

//    if (!smb345_dev) {
//        pr_err("%s: smb345_dev is null "
//            "due to driver probed isn't ready\n",
//            __func__);
//        return;
//    }

//    /* Allow violate register can be written - Write 30h[7]="1" */
//    if (smb345_set_writable(smb345_dev, true) < 0) {
//        dev_err(&smb345_dev->client->dev,
//        "%s: smb345_set_writable failed!\n", __func__);
//        return;
//    }

//    smb3xx_pre_config(cover_changed_cable_changed);
    //smb3xx_config_max_current(usb_state);
    //smb3xx_soc_control_jeita();

//    pr_info("%s: charger type:%d done.\n", __func__, usb_state);
//}

void pad_aicl_dcin(int I_DC_IN)
{
    int reg_val;

    if (I_DC_IN == 700)
        reg_val = BIT(5);
    else if (I_DC_IN == 900)
        reg_val = BIT(5) | BIT(4);
    else
        reg_val = BIT(5) | BIT(4);

    smb345_set_writable(smb345_dev, true);

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), 0)) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_DC_IN=700mA,900mA - Write 01h[7:4]="0010","0011" */
    if (smb345_masked_write(smb345_dev->client, 0x01, 0xf0, reg_val)) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to set max current limits for DC_IN\n",
            __func__);
        return;
    }

    /* Enable AICL - Write 02h[4]="1" */
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), BIT(4))) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to enable AICL\n", __func__);
        return;
    }
}

void pad_aicl(int I_USB_IN)
{
    int reg_val;

    if (I_USB_IN == 700)
        reg_val = BIT(1);
    else if (I_USB_IN == 900)
        reg_val = BIT(0) | BIT(1);
    else if (I_USB_IN == 1200)
        reg_val = BIT(2);
    else
        reg_val = BIT(1);

    smb345_set_writable(smb345_dev, true);

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), 0)) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_USB_IN=700mA,900mA - Write 01h[3:0]="0010","0011" */
    if (smb345_masked_write(smb345_dev->client, 0x01, 0x0f, reg_val)) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to set max current limits for USB_IN\n",
            __func__);
        return;
    }

    /* Enable AICL - Write 02h[4]="1" */
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), BIT(4))) {
        dev_err(&smb345_dev->client->dev,
            "%s: fail to enable AICL\n", __func__);
        return;
    }
}

void pad_cover_aicl(int pad_bat_rsoc, int cover_rsoc)
{
    if (g_Flag3 && !COVER_ATTACHED_UPI())
        goto final;

    if (pad_bat_rsoc == 100) {

        if (cover_rsoc >= 90)
            goto final;

        BAT_DBG(" %s: BALANCE 1.\n", __func__);
        /* cover config aicl */
        cover_aicl(1200);
        
        /* Suspend PAD Charger USBIN 30h[2]="1" */
        if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
            __func__);

        /* enable Cover Charger USBIN */
        cover_usbin(true);

        return;
    }

    if (cover_rsoc == 100) {
        BAT_DBG(" %s: BALANCE 2.\n", __func__);
        /* pad aicl */
        pad_aicl(1200);

        /* enable PAD Charger USBIN 30h[2]="0" */
        if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
            BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
            __func__);

        /* suspend Cover Charger USBIN */
        cover_usbin(false);

        return;
    }

    if (pad_bat_rsoc > 70 && cover_rsoc < 70) {
        if (pad_bat_rsoc > 90) {
            BAT_DBG(" %s: BALANCE 3.\n", __func__);
            /* cover config aicl */
            cover_aicl(1200);
        
            /* Suspend PAD Charger USBIN 30h[2]="1" */
            if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
                BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                __func__);

            /* enable Cover Charger USBIN */
            cover_usbin(true);

            return;
        }
        else {
            goto final;
        }
    }

    if (pad_bat_rsoc < 70 && cover_rsoc > 70) {
        if (cover_rsoc > 90) {
            BAT_DBG(" %s: BALANCE 4.\n", __func__);
            /* pad aicl */
            pad_aicl(1200);

            /* enable PAD Charger USBIN 30h[2]="0" */
            if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
                BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                __func__);

            /* suspend Cover Charger USBIN */
            cover_usbin(false);

            return;
        }
        else {
            if (pad_bat_rsoc < 30) {
                BAT_DBG(" %s: BALANCE 5.\n", __func__);
                /* pad aicl */
                pad_aicl(900);
                /* cover config aicl */
                cover_aicl(300);

                /* enable PAD Charger USBIN 30h[2]="0" */
                if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
                    BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                    __func__);

                /* eanble Cover Charger USBIN */
                cover_usbin(true);

                return;
            }
            else {
                goto final;
            }
        }
    }
    else {
            if (pad_bat_rsoc < 30) {
                BAT_DBG(" %s: BALANCE 5.\n", __func__);
                /* pad aicl */
                pad_aicl(900);
                /* cover config aicl */
                cover_aicl(300);

                /* enable PAD Charger USBIN 30h[2]="0" */
                if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
                    BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
                    __func__);

                /* eanble Cover Charger USBIN */
                cover_usbin(true);

                return;
            }
            else {
                goto final;
            }
    }

final:
    BAT_DBG(" %s: BALANCE final\n", __func__);
    /* pad aicl */
    pad_aicl(700);
    /* cover config aicl */
    cover_aicl(500);

    /* enable PAD Charger USBIN 30h[2]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
        BAT_DBG("%s: fail to Suspend PAD Charger USBIN\n",
        __func__);

    /* eanble Cover Charger USBIN */
    cover_usbin(true);

}

void cover_otg_control(void)
{
    BAT_DBG(" %s:", __func__);
    /* Enable PAD Charger DCIN 31h[2]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), 0))
        dev_err(&smb345_dev->client->dev,
            "%s: fail to set max current limits for USB_IN\n",
            __func__);

    /* set Cover OTG output current = 900mA and Enable Cover OTG */
    cover_otg_current(900);
    cover_otg(1);

    /* Set Input Current Limit with AICL */
    pad_aicl_dcin(900);
    g_Flag1 = 1;
    g_Flag4 = 1;
}

void sdp_charger_control_with_cover(void)
{
    BAT_DBG(" %s:\n", __func__);

    /* Disable Cover Charging with SDP */
    COVER_CHARGING_TOGGLE(false);

    /* Enable PAD Charger USBIN 30h[2]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
        BAT_DBG("%s: fail to set max current limits for USB_IN\n",
        __func__);
    /* Enable PAD Charger DCIN 31h[2]="0" */
    if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), 0))
        BAT_DBG("%s: fail to Enable PAD Charger DCIN\n",
        __func__);

    /* aicl when cover attache with SDP */
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), 0))
        BAT_DBG("%s: fail to Disable AICL\n",
        __func__);
    if (smb345_masked_write(smb345_dev->client, 0x01, 0x0f, BIT(0)))
        BAT_DBG("%s: fail to set Input Current Limit 500mA\n",
        __func__);
    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(4), BIT(4)))
        BAT_DBG("%s: fail to Enable AICL\n",
        __func__);
}

void cover_with_sdp(int pad_bat_rsoc, int cover_rsoc, bool cover_changed_cable_changed)
{
    BAT_DBG(" %s:\n", __func__);

    if (pad_bat_rsoc >= 70) {
        if (g_Flag2 == 1) {
            if (pad_bat_rsoc >= 90) {
                if (cover_rsoc == 100) {
                    g_Flag2 = 1;
                    sdp_charger_control_with_cover();
                }
                else
                    g_Flag2 = 0;
            }
            else {
                // Pad JEITA rule
            }
        }
        else {
            if (cover_rsoc == 100) {
                g_Flag2 = 1;
                sdp_charger_control_with_cover();
            }
            else
                g_Flag2 = 0;
        }
    }
    else { // pad bat rsoc < 70%
        if (g_Flag2 == 0) {
            g_Flag2 = 1;
            sdp_charger_control_with_cover();

            // Pad JEITA rule
        }
        else {
            if (pad_bat_rsoc >= 90) {
                g_Flag2 = 0;
            }
            else {
                // Pad JEITA rule
            }
        }
    }

    if (g_Flag2 == 0) {
        BAT_DBG("%s: * SDP suspend Pad Charger USBIN,DCIN with Cover *\n",
            __func__);

        /* USB Suspend Mode: I2C Control 02h[7]="1" */
        if (smb345_masked_write(smb345_dev->client, 0x02, BIT(7), BIT(7)))
            BAT_DBG("%s: fail to set USB Suspend Mode to I2C Control\n",
                __func__);
        /* Pad Charger USBIN Suspend 30h[2]="1" */
        if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to set Pad Charger Suspend\n",
                __func__);
        /* Pad Charger DCIN Suspend 31h[2]="1" */
        if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
            BAT_DBG("%s: fail to set Pad Charger DCIN Suspend\n",
                __func__);

        /* Enable Cover Charging with SDP */
        COVER_CHARGING_TOGGLE(true);

        /* Enable Cover Charger Setting with SDP */
        smb3xxc_pre_config(false, cover_changed_cable_changed);

        /* Cover AICL */
        cover_aicl(500);

        /* start upi */
        if (upi_ug31xx_attach_state == false && COVER_ATTACHED()) {
            if (IS_CA81())
                schedule_delayed_work(&smb345_dev->cover_detect_work, 30*HZ);
            else
                schedule_delayed_work(&smb345_dev->cover_detect_work, 60*HZ);
        }

        // cover JEITA rule
    }
}

int setSMB345Charger(int usb_state)
{
    int ret = 0;
    int cover_vbat = 0;
    int cover_rsoc = -1;
    int pad_bat_rsoc = -1;
    static int old_usb_state = CABLE_OUT;
    static int old_cover_state = COVER_DETTACH;
    static bool cover_changed_cable_changed;

    BAT_DBG_E(" =================================== %s"
              " ===================================\n", __func__);
    mutex_lock(&g_usb_state_lock);
    if (usb_state < COVER_ATTACH) {
        g_usb_state = usb_state;
    }
    if (usb_state >= COVER_ATTACH) {
        g_cover_state = usb_state;
    }

    /* Reset to 0 if Cover attach then dettach OR cable insert then remove */
    if (old_usb_state != CABLE_OUT && g_usb_state == CABLE_OUT) {
        g_Flag1 = 0;
        g_Flag4 = 0;
        g_Flag2 = 0;
    }
    if (old_cover_state != COVER_DETTACH && g_cover_state == COVER_DETTACH) {
        g_Flag1 = 0;
        g_Flag4 = 0;
        g_Flag2 = 0;
    }

    /* detect cable changed or cover changed for JEITA */
    if ((old_usb_state != g_usb_state) || (old_cover_state != g_cover_state))
        cover_changed_cable_changed = true;
    else
        cover_changed_cable_changed = false;

    BAT_DBG("### old_usb_state:%s, g_usb_state:%s, old_cover_state:%s, g_cover_state:%s ###\n",
        getCableCoverString[old_usb_state],
        getCableCoverString[g_usb_state],
        getCableCoverString[old_cover_state],
        getCableCoverString[g_cover_state]);

    /* FIXME: workaround for USB driver (ignore some abnormal status) */
    if (COVER_ATTACHED() && old_usb_state == AC_IN && g_usb_state == DISABLE_5V) {
        BAT_DBG("*****************Ignore abnormal cable type*****************\n");
        mutex_unlock(&g_usb_state_lock);
        return 0;
    }

    if (g_Flag3 && !COVER_ATTACHED_UPI()) {
        if (old_cover_state == COVER_DETTACH && g_cover_state == COVER_ATTACH) {
            if (!VBUS_IN()) {
            BAT_DBG("%s: NFC: return (No cable)\n", __func__);
            old_cover_state = g_cover_state;
            mutex_unlock(&g_usb_state_lock);
            return 0;
            }
        }
        else if (old_usb_state == g_usb_state) {
            BAT_DBG("%s: NFC: return (Cable does't change)\n", __func__);
            mutex_unlock(&g_usb_state_lock);
            return 0;
        }
    }
    if (old_cover_state == g_cover_state && old_usb_state == g_usb_state) {
        BAT_DBG(" same COVER state & same CABLE state! return!!!\n");
        if (upi_ug31xx_attach_state == false && COVER_ATTACHED_UPI() && VBUS_IN()) {
            upi_ug31xx_attach_state = true;
            upi_ug31xx_attach(true);
        }
        mutex_unlock(&g_usb_state_lock);
        return 0;
    }
    mutex_unlock(&g_usb_state_lock);

    if (old_cover_state == COVER_DETTACH &&
        g_cover_state == COVER_ATTACH    &&
        COVER_ATTACHED_UPI()){
        cancel_delayed_work(&smb345_dev->aicl_dete_work);
        BAT_DBG(" sleep 4s for UPI gauge driver...\n");
        msleep(4000);
        schedule_delayed_work(&smb345_dev->aicl_dete_work, 5*HZ);
    }

    if (g_Flag3 != 1) {
    ret = get_pack_bat_voltage(&cover_vbat);
    if (ret) {
        /* fail to acquire vbat. reset value to default */
        dev_warn(&smb345_dev->client->dev,
            "%s: fail to acquire cover vbat. reset value to default\n",
            __func__);
        cover_vbat = 0;
        ret = 0;
    }
    ret = get_pack_bat_rsoc(&cover_rsoc);
    if (ret) {
        /* fail to acquire cover rsoc. reset value to default */
        dev_warn(&smb345_dev->client->dev,
            "%s: fail to acquire cover rsoc. reset value to default\n",
            __func__);
        cover_rsoc = -1;
        ret = 0;
    }
    ret = get_battery_rsoc(&pad_bat_rsoc);
    if (ret) {
        /* fail to acquire pad battery rsoc. reset value to default */
        dev_warn(&smb345_dev->client->dev,
            "%s: fail to acquire pad battery rsoc. reset value to default\n",
            __func__);
        pad_bat_rsoc = -1;
        ret = 0;
    }
    else {
        BAT_DBG_E(
            "%s: cover_vbat(%dmV),cover_rsoc(%d%%),pad_bat_rsoc(%d%%)\n",
            __func__,
            cover_vbat, cover_rsoc, pad_bat_rsoc);
    }
    }

    switch (g_usb_state)
    {
    case USB_IN:
        BAT_DBG_E(" usb_state: USB_IN\n");
        smb3xx_pre_config(true);
        power_supply_set_online(smb345_dev->usb_psy, 1);
    case DISABLE_5V:
        if (g_usb_state == DISABLE_5V) {
            BAT_DBG_E(" usb_state: DISABLE_5V\n");
            ret = otg(0, false);
            if (COVER_ATTACHED())
                BAT_DBG_E(" CA81: %s, CB81: %s\n", IS_CA81() ? "Y" : "N", IS_CB81() ? "Y" : "N");
            if (g_is_power_supply_registered)
                power_supply_changed(&smb345_power_supplies[2]);
        }
    case CABLE_OUT:
        if (g_usb_state == CABLE_OUT) {
            BAT_DBG_E(" usb_state: CABLE_OUT\n");
            if (upi_ug31xx_attach_state == false && COVER_ATTACHED()) {
                BAT_DBG_E(" cancel delay work for init UPI gauge if cable remove with Cover attached\n");
                cancel_delayed_work(&smb345_dev->cover_detect_work);
            }
            power_supply_set_online(smb345_dev->usb_psy, 0);
            if (old_usb_state == AC_IN) {
                if (entry_mode == 4 || entry_mode == 1) {
                    if (wake_lock_active(&wlock)) {
                        BAT_DBG(" %s: cos_ac_wakelock -> wake unlock\n", __func__);
                        wake_unlock(&wlock);
                    }
                }
            }
            smb345_charging_toggle(FLAGS, true);
            smb3xx_pre_config(true);
            if (g_cover_state == COVER_DETTACH) {
                COVER_CHARGING_TOGGLE(false);
                otg(0, false);
                break;
            }
            if (entry_mode == 4 && IS_CB81()) {
                cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);
                schedule_delayed_work(&smb345_dev->cos_cb81_power_detect_work, 30*HZ);
                cos_keep_power_on_with_cb81 = true;
            }
        }
    case AC_IN:
        if (g_usb_state == AC_IN) {
            BAT_DBG_E(" usb_state: AC_IN\n");
            smb3xx_pre_config(true);
            if (entry_mode == 4 || entry_mode == 1) {
                if (!wake_lock_active(&wlock)) {
                    BAT_DBG(" %s: cos_ac_wakelock -> wake lock\n", __func__);
                    wake_lock(&wlock);
                }
            }
        }

        if (smb345_dev && (VBUS_IN() || COVER_ATTACHED())) {

            //mutex_lock(&g_usb_state_lock);
            //smb345_config_max_current(g_usb_state, cover_changed_cable_changed);
            //mutex_unlock(&g_usb_state_lock);

            if (COVER_ATTACHED()) {
                if (!VBUS_IN() && cover_rsoc != -1 && cover_vbat != 0 && cover_rsoc < 5) {
                    disable_cover_otg();
pr_info("10\n");
                }
                else {
                    disable_cover_otg();
pr_info("11\n");
                    smb3xxc_pre_config(true, cover_changed_cable_changed);
                    if (VBUS_IN() && gpio_get_value(USB_HS_ID)) {
                        if (g_Flag3 && !COVER_ATTACHED_UPI()) otg(0, true);
pr_info("12\n");
                        smb345_set_writable(smb345_dev, true);

                        if (g_usb_state == AC_IN) {
                            if (old_cover_state == COVER_DETTACH && g_cover_state == COVER_ATTACH && IS_CB81()) {
                                pr_info("sleep 2s for DCP...\n");
                                msleep(2000);
                            }
pr_info("12a\n");
                            /* re-judge again if sleep 2s */
                            if (VBUS_IN() && COVER_ATTACHED() && (g_usb_state == AC_IN)) {
pr_info("12b\n");
                                /* Enable Cover Charging with DCP */
                                COVER_CHARGING_TOGGLE(true);
                                /* Enable PAD Charger USBIN 30h[2]="0" */
                                if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), 0))
                                    dev_err(&smb345_dev->client->dev,
                                        "%s: fail to set max current limits for USB_IN\n",
                                        __func__);
                                /* Suspend PAD Charger DCIN 31h[2]="1" */
                                if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                                    dev_err(&smb345_dev->client->dev,
                                        "%s: fail to Suspend PAD Charger DCIN\n",
                                        __func__);
                                /* check PAD battery rsoc & Cover battery rsoc */
                                pad_cover_aicl(pad_bat_rsoc, cover_rsoc);

                                /* start upi */
                                if (upi_ug31xx_attach_state == false && COVER_ATTACHED()) {
pr_info("12c\n");
                                    if (IS_CA81())
                                        schedule_delayed_work(&smb345_dev->cover_detect_work, 30*HZ);
                                    else
                                        schedule_delayed_work(&smb345_dev->cover_detect_work, 60*HZ);
                                }
                            }
pr_info("13\n");
                        }
                        else if (g_usb_state == USB_IN) { // SDP
                            if (g_Flag3 == 1 && !COVER_ATTACHED_UPI()) {
pr_info("14a\n");
                                /* NFC device: turn on cable power to NFC Cover */
                                /* set USBIN Suspend Mode = I2C Control 02h[7]=1 */
                                if (smb345_masked_write(smb345_dev->client, 0x02, BIT(7), BIT(7)))
                                    dev_err(&smb345_dev->client->dev,
                                        "%s: fail to set USBIN Suspend Mode to I2C Control\n",
                                        __func__);
                                /* set Pad Charger Suspend 30h[2]=1 */
                                if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
                                    dev_err(&smb345_dev->client->dev,
                                        "%s: fail to set Pad Charger Suspend\n",
                                        __func__);
                                /* set Pad Charger DCIN Suspend 31h[2]=1 */
                                if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                                    dev_err(&smb345_dev->client->dev,
                                        "%s: fail to set Pad Charger DCIN Suspend\n",
                                        __func__);
                                COVER_CHARGING_TOGGLE(true);
                            }
                            else {
                                cover_with_sdp(pad_bat_rsoc, cover_rsoc, cover_changed_cable_changed);
                            }
                        }
                        else {
                            // CABLE_OUT ? VBUS_IN detect
pr_info("23\n");
                            msleep(2000);
                            if (!VBUS_IN() && g_usb_state == CABLE_OUT) {
                                if (COVER_ATTACHED() && IS_CB81()) {
                                    if (pad_bat_rsoc < 90)
                                        cover_otg_control();
                                    else {
pr_info("23a\n");
                                        /* reset to default */
                                        g_Flag1 = 0;
                                        g_Flag4 = 1;

                                        /* USB Suspend Mode: I2C Control 02h[7]="1" */
                                        if (smb345_masked_write(smb345_dev->client, 0x02, BIT(7), BIT(7)))
                                            dev_err(&smb345_dev->client->dev,
                                                "%s: fail to set USB Suspend Mode to I2C Control\n",
                                                __func__);
                                        /* Suspend Pad Charger USBIN 30h[2]="1" */
                                        if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
                                            dev_err(&smb345_dev->client->dev,
                                                "%s: fail to Suspend Pad Charger USBIN\n",
                                                __func__);
                                        /* Suspend Pad Charger DCIN 31h[2]="1" */
                                        if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                                            dev_err(&smb345_dev->client->dev,
                                                "%s: fail to set Suspend Pad Charger DCIN\n",
                                            __func__);

                                        cover_otg(1);
                                    }
pr_info("24\n");
                                }
                            }
                            else {
                                pr_info("*********************************************************\n");
                                pr_info("                           BUG                           \n");
                                pr_info("*********************************************************\n");
                            }
                        }
                    }
                    else {
                        COVER_CHARGING_TOGGLE(false);
                        if (IS_CB81()) {
pr_info("18\n");
                            msleep(2000);
                            /* must confirm there is no plugged-in OTG cable */
                            if (!VBUS_IN() && COVER_ATTACHED_UPI() && IS_CB81() && gpio_get_value(USB_HS_ID)) {
                                if (pad_bat_rsoc < 90)
                                    cover_otg_control();
                                else {
pr_info("18a\n");
                                    /* reset to default */
                                    g_Flag1 = 0;
                                    g_Flag4 = 1;

                                    /* USB Suspend Mode: I2C Control 02h[7]="1" */
                                    if (smb345_masked_write(smb345_dev->client, 0x02, BIT(7), BIT(7)))
                                        dev_err(&smb345_dev->client->dev,
                                            "%s: fail to set USB Suspend Mode to I2C Control\n",
                                            __func__);
                                    /* Suspend Pad Charger USBIN 30h[2]="1" */
                                    if (smb345_masked_write(smb345_dev->client, 0x30, BIT(2), BIT(2)))
                                        dev_err(&smb345_dev->client->dev,
                                            "%s: fail to Suspend Pad Charger USBIN\n",
                                            __func__);
                                    /* Suspend Pad Charger DCIN 31h[2]="1" */
                                    if (smb345_masked_write(smb345_dev->client, 0x31, BIT(2), BIT(2)))
                                        dev_err(&smb345_dev->client->dev,
                                            "%s: fail to set Suspend Pad Charger DCIN\n",
                                            __func__);

                                    cover_otg(1);
                                }
                            }
                        }
                        else if (g_Flag3 && !COVER_ATTACHED_UPI() && gpio_get_value(USB_HS_ID) && !VBUS_IN()) {
pr_info("19\n");
                            COVER_CHARGING_TOGGLE(true);
                            otg(1, true);
                        }
pr_info("20\n");
                    }
                }
            }
            else {
pr_info("22\n");
                COVER_CHARGING_TOGGLE(false);

                smb3xx_config_max_current(g_usb_state, cover_changed_cable_changed);
                if (old_cover_state == COVER_ATTACH && g_cover_state == COVER_DETTACH) {
                    cancel_delayed_work(&smb345_dev->aicl_dete_work);
                    aicl_dete_worker(NULL);
                    schedule_delayed_work(&smb345_dev->aicl_dete_work, HZ);
                }
            }
        }

        /* regard OTG CABLE OUT as CABLE OUT */
        mutex_lock(&g_usb_state_lock);
        if (g_usb_state == DISABLE_5V) {
            old_usb_state = CABLE_OUT;
            g_usb_state = CABLE_OUT;
        }
        mutex_unlock(&g_usb_state_lock);

        request_power_supply_changed();
        break;

    case ENABLE_5V:
        BAT_DBG_E(" usb_state: ENABLE_5V\n");
        BAT_DBG_E(" USB_HS_ID: %s\n", gpio_get_value(USB_HS_ID) ? "H" : "L");

        if (COVER_ATTACHED())
            disable_cover_otg();
        COVER_CHARGING_TOGGLE(false);

        mutex_lock(&g_usb_state_lock);
        if (old_usb_state != ENABLE_5V && g_usb_state == ENABLE_5V) {
            msleep(500);
            ret = otg(1, false);
            if (g_is_power_supply_registered)
                power_supply_changed(&smb345_power_supplies[2]);
        }
        mutex_unlock(&g_usb_state_lock);
        break;

    default:
        BAT_DBG_E(" ERROR: wrong usb state value = %d\n", usb_state);
        ret = 1;
        break;
    }

#if defined(CONFIG_Z380C)
    switch (g_usb_state) {
    case USB_IN:
        usb_to_battery_callback(USB_PC);
        break;
    case DISABLE_5V:
    case ENABLE_5V:
    case CABLE_OUT:
        usb_to_battery_callback(NO_CABLE);
        break;
    case AC_IN:
        usb_to_battery_callback(USB_ADAPTER);
        break;
    }
#endif

    mutex_lock(&g_usb_state_lock);
    /* assign value to old variable */
    old_usb_state = g_usb_state;
    old_cover_state = g_cover_state;
    mutex_unlock(&g_usb_state_lock);

    smb345_dump_registers(NULL);
    smb345c_dump_registers(NULL);

    cover_wakelock();
    return ret;
}

/* write 06h[6:5]="00" or "11" */
int smb345_charging_toggle(charging_toggle_level_t level, bool on)
{
    int ret = 0;
#if defined(ME372CG_ENG_BUILD)
    int rsoc;
#endif
    int charging_toggle;
    static bool jeita_want_charging = true;
    static bool balan_want_charging = true;
    static charging_toggle_level_t old_lvl = JEITA;
    char *level_str[] = {
        "BALANCE",
        "JEITA",
        "FLAGS",
    };

    if (!smb345_dev) {
        pr_info("Warning: smb345_dev is null "
            "due to probe function has error\n");
        return 1;
    }

    mutex_lock(&g_charging_toggle_lock);
    charging_toggle = g_charging_toggle;
    mutex_unlock(&g_charging_toggle_lock);

    BAT_DBG(" %s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
        __func__,
        level_str[old_lvl],
        charging_toggle ? "YES" : "NO",
        level_str[level],
        on ? "YES" : "NO");

    /* reset to default if AudioCover/PowerBank doesn't attached */
    if (!COVER_ATTACHED_UPI()) {
        balan_want_charging = true;
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
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(old_lvl != JEITA)\n",
                            __func__);
                        return -1;
                    }
                    else if (!balan_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
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
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
                            "(old_lvl != BALANCE)\n",
                            __func__);
                        return -1;
                    }                    
                    else if (!jeita_want_charging) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *"
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

    ret = smb345_set_writable(smb345_dev, true);
    if (ret < 0)
        return ret;

    /* Config CFG_PIN register */

    ret = smb345_read(smb345_dev, CFG_PIN);
    if (ret < 0)
        goto out;

#if defined(ME372CG_ENG_BUILD)
    /* acquire battery rsoc here */
    if (get_battery_rsoc(&rsoc)) {
        BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
    }
    else {
        BAT_DBG(" %s: rsoc=%d\n", __func__, rsoc);
        if (rsoc >= 0 && rsoc <= 100) {
            if (eng_charging_limit) {
                if (rsoc > charging_limit_threshold) {
                    /* stop charging as phone battery rsoc >= 60% in ENG build */
                    on = false;
                    flag_eng_stop_charging = true;
                    BAT_DBG_E(" %s: *** I. ENG charging toggle: OFF *** -> rsoc=%d\n",
                        __func__, rsoc);
                }
                else if (rsoc >= (charging_limit_threshold - 15) && flag_eng_stop_charging && VBUS_IN()) {
                    /* stop charging as flag is true */
                    on = false;
                    BAT_DBG_E(" %s: *** II. ENG charging toggle: OFF *** -> rsoc=%d\n",
                        __func__, rsoc);
                }
                else
                    flag_eng_stop_charging = false;
            }
            else
                flag_eng_stop_charging = false;
        }
        else
            flag_eng_stop_charging = false;
    }
#endif

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

    ret = smb345_write(smb345_dev, CFG_PIN, ret);
    if (ret < 0)
        goto out;

    mutex_lock(&g_charging_toggle_lock);
    g_charging_toggle = on;
    mutex_unlock(&g_charging_toggle_lock);

out:
    return ret;
}

static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
    struct smb345_charger *smb = data;
    int charger_type;

    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    wake_lock_timeout(&smb->wakelock, 4*HZ);

    pm_runtime_get_sync(&smb->client->dev);

    if (gpio_get_value(smb->smb_acok_gpio)) {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (HIGH) <<<\n", __func__);

        /* reset to default as missing external power source */
        mutex_lock(&g_charging_toggle_lock);
#ifdef ME372CG_ENG_BUILD
        flag_eng_stop_charging = false;
#endif
        mutex_unlock(&g_charging_toggle_lock);
    }
    else {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (LOW) <<<\n", __func__);

        /* ME372CG charge current control algorithm:
           config the charge current only when
           Vbus is legal (a valid input voltage
           is present)
        */
        mutex_lock(&g_usb_state_lock);
        charger_type = g_usb_state;
        dev_warn(&smb->client->dev,
            "%s: config current when inok interrupt\n", __func__);
        mutex_unlock(&g_usb_state_lock);
        request_power_supply_changed();
    }

    pm_runtime_put_sync(&smb->client->dev);
    return IRQ_HANDLED;
}

static int smb345_inok_gpio_init(struct smb345_charger *smb)
{
    int ret, irq;

    /* request it and configure gpio as input */
    ret = devm_gpio_request_one(smb->dev,
                                smb->smb_acok_gpio,
                                GPIOF_IN,
                                "smb_acok_gpio");
    if (ret) {
        dev_err(&smb->client->dev, "gpio_request failed for %d ret=%d\n",
            smb->smb_acok_gpio, ret);
        return ret;
    }

    irq = gpio_to_irq(smb->smb_acok_gpio);

    if (gpio_get_value(smb->smb_acok_gpio))
        dev_info(&smb->client->dev, ">>> INOK (HIGH) <<<\n");
    else
        dev_info(&smb->client->dev, ">>> INOK (LOW) <<<\n");

    ret = request_threaded_irq(irq, NULL, smb345_inok_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    smb->client->name,
                    smb);
    if (ret < 0) {
        dev_info(&smb->client->dev, "config INOK gpio as IRQ fail!\n");
        goto fail_gpio;
    }

    return 0;

fail_gpio:
    gpio_free(smb->smb_acok_gpio);
    return ret;
}

static irqreturn_t vbus_in_det_n_interrupt(int irq, void *data)
{
    struct smb345_charger *smb = data;

    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    wake_lock_timeout(&smb->wakelock, 4*HZ);

    pm_runtime_get_sync(&smb->client->dev);

    if (gpio_get_value(smb->vbus_in_det_n_gpio)) {
        dev_warn(&smb->client->dev,
            "%s: >>> VBUS_IN_DET_N pin (HIGH) <<<\n", __func__);
        if (COVER_ATTACHED_UPI()) {
            if (entry_mode == 4 && IS_CB81()) {
                cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);
                schedule_delayed_work(&smb345_dev->cos_cb81_power_detect_work, 30*HZ);
                cos_keep_power_on_with_cb81 = true;
            }
        }
        power_supply_set_present(smb->usb_psy, 0);
    }
    else {
        dev_warn(&smb->client->dev,
            "%s: >>> VBUS_IN_DET_N pin (LOW) <<<\n", __func__);
        if (COVER_ATTACHED_UPI()) {
            if (entry_mode == 4 && IS_CB81()) {
                cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);
                schedule_delayed_work(&smb345_dev->cos_cb81_power_detect_work, 30*HZ);
                cos_keep_power_on_with_cb81 = true;
            }
            disable_cover_otg();
            COVER_CHARGING_TOGGLE(false);
        }
        power_supply_set_present(smb->usb_psy, 1);
    }

    pm_runtime_put_sync(&smb->client->dev);
    return IRQ_HANDLED;
}

static int vbus_in_det_n_gpio_init(struct smb345_charger *smb)
{
    int ret, irq;

    /* request it and configure gpio as input */
    ret = devm_gpio_request_one(smb->dev,
                                smb->vbus_in_det_n_gpio,
                                GPIOF_IN,
                                "vbus_in_det_n");
    if (ret) {
        dev_err(&smb->client->dev, "gpio_request failed for %d ret=%d\n",
            smb->vbus_in_det_n_gpio, ret);
        return ret;
    }

    irq = gpio_to_irq(smb->vbus_in_det_n_gpio);

    if (gpio_get_value(smb->vbus_in_det_n_gpio)) {
        dev_info(&smb->client->dev, ">>> VBUS_IN_DET_N (HIGH) <<<\n");
        power_supply_set_present(smb->usb_psy, 0);
    }
    else {
        dev_info(&smb->client->dev, ">>> VBUS_IN_DET_N (LOW) <<<\n");
        power_supply_set_present(smb->usb_psy, 1);
    }

    ret = request_threaded_irq(irq, NULL, vbus_in_det_n_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    smb->client->name,
                    smb);
    if (ret < 0) {
        dev_info(&smb->client->dev, "config VBUS_IN_DET_N gpio as IRQ fail!\n");
        goto fail_gpio;
    }

    return 0;

fail_gpio:
    gpio_free(smb->vbus_in_det_n_gpio);
    return ret;
}

static irqreturn_t dcin_vbus_in_det_n_interrupt(int irq, void *data)
{
    struct smb345_charger *smb = data;

    if (gpio_get_value(smb->dcin_vbus_in_det_n_gpio)) {
        dev_warn(&smb->client->dev,
            "%s: >>> DCIN_VBUS_IN_DET_N pin (HIGH) <<<\n", __func__);
    }
    else {
        dev_warn(&smb->client->dev,
            "%s: >>> DCIN_VBUS_IN_DET_N pin (LOW) <<<\n", __func__);
    }
    return IRQ_HANDLED;
}


static int dcin_vbus_in_det_n_gpio_init(struct smb345_charger *smb)
{
    int ret, irq;

    /* request it and configure gpio as input */
    ret = devm_gpio_request_one(smb->dev,
                                smb->dcin_vbus_in_det_n_gpio,
                                GPIOF_IN,
                                "dcin_vbus_in_det_n");
    if (ret) {
        dev_err(&smb->client->dev, "gpio_request failed for %d ret=%d\n",
            smb->dcin_vbus_in_det_n_gpio, ret);
        return ret;
    }

    irq = gpio_to_irq(smb->dcin_vbus_in_det_n_gpio);

    if (gpio_get_value(smb->dcin_vbus_in_det_n_gpio))
        dev_info(&smb->client->dev, ">>> DCIN_VBUS_IN_DET_N (HIGH) <<<\n");
    else
        dev_info(&smb->client->dev, ">>> DCIN_VBUS_IN_DET_N (LOW) <<<\n");

    ret = request_threaded_irq(irq, NULL, dcin_vbus_in_det_n_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_ONESHOT |
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    smb->client->name,
                    smb);
    if (ret < 0) {
        dev_info(&smb->client->dev, "config DCIN_VBUS_IN_DET_N gpio as IRQ fail!\n");
        goto fail_gpio;
    }

    return 0;

fail_gpio:
    gpio_free(smb->dcin_vbus_in_det_n_gpio);
    return ret;
}


bool smb345_has_charger_error(void)
{
    int ret;

    if (!smb345_dev)
        return -EINVAL;

    ret = smb345_read(smb345_dev, STAT_C);
    if (ret < 0)
        return true;

    if (ret & STAT_C_CHARGER_ERROR)
        return true;

    return false;
}

static bool PAD_CHARGING_FULL(void)
{
    int ret;

    if (!smb345_dev)
        return false;

    ret = smb345_set_writable(smb345_dev, true);
    if (ret < 0)
        return false;

    ret = smb345_read(smb345_dev, IRQSTAT_C);
    if (ret < 0)
        return false;

    if (ret & BIT(0)) {
        /* set to almost full if battery is in termination charging current mode */
        BAT_DBG_E(" PAD TERMINATION CHARING CURRENT hit...\n");
        return true;
    }

    return false;
}

int smb345_get_charging_status(void)
{
    int ret, status;

    if (!smb345_dev)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    ret = smb345_read(smb345_dev, STAT_C);
    if (ret < 0)
        return POWER_SUPPLY_STATUS_UNKNOWN;

    //BAT_DBG(" Charging Status: STAT_C:0x%x\n", ret);

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
        } else if (PAD_CHARGING_FULL()) {
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
EXPORT_SYMBOL(smb345_get_charging_status);

#ifndef ME372CG_OTHER_BUILD
int smb345_dump_registers(struct seq_file *s)
{
    struct smb345_charger *smb;
    int ret;
    u8 reg;

    if (s) {
        smb = s->private;
    }
    else {
        if (!smb345_dev) {
            BAT_DBG(" %s: smb345_dev is null!\n",
                __func__);
            return -1;
        }
        else {
            smb = smb345_dev;
        }
    }

    BAT_DBG(" %s:\n", __func__);
    BAT_DBG(" Control registers:\n");
    BAT_DBG(" ==================\n");
    BAT_DBG(" #Addr\t#Value\n");

    for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
        ret = smb345_read(smb, reg);
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

    ret = smb345_read(smb, CMD_A);
    BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
        "\n", CMD_A, BYTETOBINARY(ret));
    if (s)
        seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
            "\n", CMD_A, BYTETOBINARY(ret));
    ret = smb345_read(smb, CMD_B);
    BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
        "\n", CMD_B, BYTETOBINARY(ret));
    if (s)
        seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
            "\n", CMD_B, BYTETOBINARY(ret));
    ret = smb345_read(smb, CMD_C);
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
        ret = smb345_read(smb, reg);
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
        ret = smb345_read(smb, reg);
        BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(ret));
        if (s)
            seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
                "\n", reg, BYTETOBINARY(ret));
    }

    return 0;
}
#else
int smb345_dump_registers(struct seq_file *s) { return 0; }
#endif

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
    seq_printf(s, "Control registers:\n");
    seq_printf(s, "==================\n");
    seq_printf(s, "#Addr\t#Value\n");

    smb345_dump_registers(s);

    return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, smb345_debugfs_show, inode->i_private);
}

static const struct file_operations smb345_debugfs_fops = {
    .open        = smb345_debugfs_open,
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
        if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

static inline struct power_supply *get_psy_pack_bat(void)
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
        *volt = val.intval / 1000;

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

static inline int get_pack_bat_voltage(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *volt = val.intval / 1000;

    return ret;
}

static inline int get_pack_bat_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_pack_bat();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

void chrgr_type_work_func(struct work_struct *work)
{ setSMB345Charger(g_extern_device); }

static int smb345_otg_regulator_enable(struct regulator_dev *rdev)
{
    pr_info("-------------------------------------------------------------------%s:\n", __func__);
    return 0;
}

static int smb345_otg_regulator_disable(struct regulator_dev *rdev)
{
    pr_info("-------------------------------------------------------------------%s:\n", __func__);
    return 0;
}

static int smb345_otg_regulator_is_enable(struct regulator_dev *rdev)
{
    struct smb345_charger *chip = rdev_get_drvdata(rdev);
    return chip->otg_enabled;
}

struct regulator_ops smb345_otg_reg_ops = {
    .enable		= smb345_otg_regulator_enable,
    .disable	= smb345_otg_regulator_disable,
    .is_enabled	= smb345_otg_regulator_is_enable,
};

static int smb345_regulator_init(struct smb345_charger *chip)
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
		chip->otg_vreg.rdesc.ops = &smb345_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static int smb345_parse_dt(struct smb345_charger *chip)
{
    int ret = 0;
    struct pinctrl  *pinctrl;
    struct pinctrl_state    *pin_default_acok;

    struct device_node *node = chip->client->dev.of_node;
    if (!node) {
        pr_err("No DT data Failing Probe\n");
        return -EINVAL;
    }

    /*----------------------------- OTG -----------------------------*/

    chip->smb_otg_en_gpio = of_get_named_gpio(node, "qcom,smb-otg-en-gpio", 0);
    pr_info("qcom,smb-otg-en-gpio = %d.\n", chip->smb_otg_en_gpio);

    if (!gpio_is_valid(chip->smb_otg_en_gpio)) {
        pr_err("gpio is not valid: smb_otg_en_gpio\n");
        return -EINVAL;
    }

    /*----------------------------- ACOK -----------------------------*/
    chip->smb_acok_gpio = of_get_named_gpio(node, "qcom,smb-acok-gpio", 0);
    pr_info("qcom,smb-acok-gpio = %d.\n", chip->smb_acok_gpio);

    if (!gpio_is_valid(chip->smb_acok_gpio)) {
        pr_err("gpio is not valid: smb_acok_gpio\n");
        return -EINVAL;
    }

    /*----------------------------- VBUS_IN -----------------------------*/
    chip->vbus_in_det_n_gpio = of_get_named_gpio(node, "qcom,vbus-in-det-n-gpio", 0);
    pr_info("qcom,vbus-in-det-n-gpio = %d.\n", chip->vbus_in_det_n_gpio);

    if (!gpio_is_valid(chip->vbus_in_det_n_gpio)) {
        pr_err("gpio is not valid: vbus_in_det_n_gpio\n");
        return -EINVAL;
    }

    /*----------------------------- DCIN_VBUS_IN -----------------------------*/
    chip->dcin_vbus_in_det_n_gpio = of_get_named_gpio(node, "qcom,dcin-vbus-in-det-n-gpio", 0);
    pr_info("qcom,dcin-vbus-in-det-n-gpio = %d.\n", chip->dcin_vbus_in_det_n_gpio);

    if (!gpio_is_valid(chip->dcin_vbus_in_det_n_gpio)) {
        pr_err("gpio is not valid: dcin_vbus_in_det_n_gpio\n");
        return -EINVAL;
    }

    /*----------------------------- CHG_EN -----------------------------*/
    chip->smb_chg_en_gpio = of_get_named_gpio(node, "qcom,cover-charging-enable-gpio", 0);
    pr_info("qcom,cover-charging-enable-gpio = %d.\n", chip->smb_chg_en_gpio);

    if (!gpio_is_valid(chip->smb_chg_en_gpio)) {
        pr_err("gpio is not valid: qcom,cover-charging-enable-gpio\n");
        return -EINVAL;
    }

    /*----------------------------- PIN CTRL -----------------------------*/

    pinctrl = devm_pinctrl_get(&chip->client->dev);
    if (IS_ERR_OR_NULL(pinctrl)) {
        dev_err(&chip->client->dev, "%s(): Failed to get pinctrl\n",
            __func__);
        return PTR_ERR(pinctrl);
    }

    pin_default_acok = pinctrl_lookup_state(pinctrl, "default");
    if (IS_ERR_OR_NULL(pin_default_acok)) {
        dev_err(&chip->client->dev, "%s(): Failed to look up default acok state\n",
            __func__);
        return PTR_ERR(pin_default_acok);
    }

    ret = pinctrl_select_state(pinctrl, pin_default_acok);
    if (ret) {
        dev_err(&chip->client->dev, "%s(): Can't select pinctrl state\n",
            __func__);
        return ret;
    }

    /*----------------------------- GPIO -----------------------------*/

    /* request it and configure gpio as output */
    ret = devm_gpio_request_one(chip->dev,
                                chip->smb_otg_en_gpio,
                                GPIOF_OUT_INIT_LOW,
                                "smb_otg_en_gpio");
    if (ret) {
        dev_err(&chip->client->dev, "gpio_request failed for %d ret=%d\n",
            chip->smb_otg_en_gpio, ret);
        return ret;
    }

    /* config default value to output low */
    gpio_set_value(chip->smb_otg_en_gpio, 0);

    /* request it and configure gpio as output */
    ret = devm_gpio_request_one(chip->dev,
                                chip->smb_chg_en_gpio,
                                GPIOF_OUT_INIT_LOW,
                                "smb_chg_en_gpio");
    if (ret) {
        dev_err(&chip->client->dev, "gpio_request failed for %d ret=%d\n",
            chip->smb_chg_en_gpio, ret);
        return ret;
    }

    /* config default value to output low */
    gpio_set_value(chip->smb_chg_en_gpio, 0);

    /* INOK pin configuration */
    ret = smb345_inok_gpio_init(chip);
    if (ret < 0) {
        dev_err(&chip->client->dev, "fail to initialize INOK gpio: %d\n", ret);
        return ret;
    }

    /* VBUS_IN_DET_N pin configuration */
    ret = vbus_in_det_n_gpio_init(chip);
    if (ret < 0) {
        dev_err(&chip->client->dev, "fail to initialize VBUS_IN_DET_N gpio: %d\n", ret);
        return ret;
    }

    /* DCIN_VBUS_IN_DET_N pin configuration */
    if (HW_ID > HW_ID_MP) {
        ret = dcin_vbus_in_det_n_gpio_init(chip);
        if (ret < 0) {
            dev_err(&chip->client->dev, "fail to initialize DCIN_VBUS_IN_DET_N gpio: %d\n", ret);
            return ret;
        }
    }

    return 0;
}

static void cos_cb81_power_detect_work_func(struct work_struct *work)
{
    cos_keep_power_on_with_cb81 = false;
}

static void cover_detect_work_func(struct work_struct *work)
{
    if (COVER_ATTACHED_UPI()) {
        BAT_DBG_E(" CA81: %s, CB81: %s\n",
            _IS_CA81_() ? "Y" : "N",
            _IS_CB81_() ? "Y" : "N");

        disable_cover_otg();

        if (gpio_get_value(USB_HS_ID) && VBUS_IN()) {
            COVER_CHARGING_TOGGLE(true);

            /* If Cover Charger IC is in precharge mode plugged with
                USB cable, skip initialize UPI gauge driver attached func.
                It means Cover battery is low power. It's risky to
                communication with UPI gauge.
            */
            if (COVER_PRE_CHARGING_MODE()) {
                goto final;
            }
            if (entry_mode == 4) {
                goto final;
            }
        }

        /* inform UPI for cover attach only if gpio109 dectect low */
        upi_ug31xx_attach_state = true;
        upi_ug31xx_attach(true);
    }
    else { // gpio109 is High
        /* re-schedule it if gpio109 detect High && gpio0 dectect High:
           It's No power cover or Cover with low low battery. We cannot
           inform UPI gauge when Cover attached Due to that there is no
           UPI gauge on No Power Cover(may cause lots of I2C failure).
           If Cover with low low battery, we also cannot inform UPI gauge
           driver when Cover attached due to that UPI gauge is supplied
           by Vbat. It may cause lots of I2C failure if UPI gauge isn't
           power on.
        */
        if (COVER_ATTACHED()) {
            if (!VBUS_IN()) {
                /* It means No cable. Need Pad to power supply to Cover */
                /* enable OTG 5V boost from Pad Battery to Cover */
                BAT_DBG_E(" * turn on OTG 5V boost from Pad to Cover *\n");
                COVER_CHARGING_TOGGLE(true);
                otg(1, true);

                msleep(1500);

                if (COVER_ATTACHED_UPI()) {
                    BAT_DBG_E(" CA81: %s, CB81: %s\n",
                        _IS_CA81_() ? "Y" : "N",
                        _IS_CB81_() ? "Y" : "N");

                    BAT_DBG_E(" * turn off OTG 5V boost from Pad to Cover *\n");
                    COVER_CHARGING_TOGGLE(false);
                    otg(0, true);
                }
                else {
                    if (COVER_ATTACHED()) {
                        g_Flag3 = 1;
                        BAT_DBG(" ********************** NFC **********************\n");
                    }
                    else {
                        COVER_CHARGING_TOGGLE(false);
                        otg(0, true);
                    }
                }
            }
            else {
                BAT_DBG(" power from USB cable will supply to Cover");

                /* confirm there is no OTG cable when vbus present */
                if (gpio_get_value(USB_HS_ID)) {
                    /* turn off otg_en */
                    gpio_set_value(smb345_dev->smb_otg_en_gpio, 0);

                    /* turn on cover_chg_en */
                    BAT_DBG_E(" * turn on COVER_CHG_EN from Pad to Cover *\n");
                    COVER_CHARGING_TOGGLE(true);
                    msleep(1500);

                    if (COVER_ATTACHED_UPI()) {
                        BAT_DBG_E(" CA81: %s, CB81: %s\n",
                            _IS_CA81_() ? "Y" : "N",
                            _IS_CB81_() ? "Y" : "N");

                        BAT_DBG_E(" * keep COVER_CHG_EN from Pad to Cover *\n");
                        //powerless_cover = true;
                        //COVER_CHARGING_TOGGLE(false);
                        //msleep(500);
                    }
                    else {
                        BAT_DBG_E(" CA81: %s, CB81: %s\n",
                            _IS_CA81_() ? "Y" : "N",
                            _IS_CB81_() ? "Y" : "N");

                        if (COVER_ATTACHED()) {
                            g_Flag3 = 1;
                            BAT_DBG(" ********************** NFC **********************\n");
                        }
                        else {
                            BAT_DBG_E(" * turn off COVER_CHG_EN from Pad to Cover(hardware error) *\n");
                            COVER_CHARGING_TOGGLE(false);
                        }
                    }
                }
            }
        }
        else {
            upi_ug31xx_attach_state = false;
            upi_ug31xx_attach(false);
        }
    }

final:
    setSMB345Charger(COVER_ATTACHED() ? COVER_ATTACH : COVER_DETTACH);
}

static int cover_type_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    switch (event) {
    case COVER_IN:
        BAT_DBG_E("-----------------------------------------------COVER_IN\n");
        cancel_delayed_work(&smb345_dev->cover_detect_work);
        schedule_delayed_work(&smb345_dev->cover_detect_work, 0);
        if (entry_mode == 4) {
            cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);
            schedule_delayed_work(&smb345_dev->cos_cb81_power_detect_work, 30*HZ);
            cos_keep_power_on_with_cb81 = true;
        }
        break;
    case COVER_OUT:
        BAT_DBG_E("-----------------------------------------------COVER_OUT\n");
        SET_COVER_DETACHED();
        if (entry_mode == 4) {
            cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);
            cos_keep_power_on_with_cb81 = false;
        }
        g_Flag1 = 0;
        g_Flag4 = 0;
        g_Flag2 = 0;
        if (g_Flag3) {
            COVER_CHARGING_TOGGLE(false);
            otg(0, true);
            BAT_DBG_E(" * turn off OTG 5V boost from Pad to Cover *\n");
        }
        g_Flag3 = 0;
        powerless_cover = false;
        set_powerless_leave = false;
        smb345c_charging_toggle(FLAGS, true);
        cancel_delayed_work(&smb345_dev->cover_detect_work);
        schedule_delayed_work(&smb345_dev->cover_detect_work, 0);
        break;
    case COVER_ENUMERATION:
        BAT_DBG_E("-----------------------------------------------COVER_ENUMERATION\n");
        break;
    default:
        BAT_DBG_E("-----------------------------------------------COVER_EMPTY *\n");
        break;
    }

    return NOTIFY_OK;
}

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void smb345_screen_changed_listener(const int state)
{
    if (state == NOTIFY_WHEN_SCREEN_OFF) {
        BAT_DBG(" %s: SCREEN OFF\n", __func__);

        /* JEITA function by charger protection */
        screen_off = true;
        mutex_lock(&g_usb_state_lock);
        if (g_usb_state == AC_IN || g_usb_state == USB_IN || IS_CB81())
        {
            smb3xx_soc_control_jeita();
        }
        mutex_unlock(&g_usb_state_lock);
    }
    else if (state == NOTIFY_WHEN_SCREEN_ON) {
        BAT_DBG(" %s: SCREEN ON\n", __func__);

        screen_off = false;
        /* JEITA function by charger protection */
        mutex_lock(&g_usb_state_lock);
        if (g_usb_state == AC_IN || g_usb_state == USB_IN || IS_CB81())
        {
            smb3xx_soc_control_jeita();
        }
        mutex_unlock(&g_usb_state_lock);
    }
}
#endif

static int smb345_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct smb345_charger *smb;
    struct power_supply *usb_psy;
    int ret;

    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy) {
        dev_err(&client->dev, "%s: USB psy not found; deferring probe\n",
            __func__);
        return -EPROBE_DEFER;
    }

    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
    if (!smb)
        return -ENOMEM;

    i2c_set_clientdata(client, smb);

    smb->client = client;
    /* qcom */
    smb->dev = &client->dev;
    smb->usb_psy = usb_psy;

    /* init wake lock */
    wake_lock_init(&smb->wakelock,
        WAKE_LOCK_SUSPEND, "smb345_wakelock");

    /* init wake lock in COS */
    if (entry_mode == 4 || entry_mode == 1) {
        BAT_DBG(" %s: wake lock init: cos_ac_wakelock\n",
            __func__);
        wake_lock_init(&wlock, WAKE_LOCK_SUSPEND, "cos_ac_wakelock");
    }

    /* Special Cover Wake Lock */
    wake_lock_init(&wlock_t, WAKE_LOCK_SUSPEND, "Cover_WakeLock");

    /* init spin lock */
    spin_lock_init(&smb->ibat_change_lock);

    /* init charger jeita state by battery temperature to RTS */
    smb->charger_jeita_status = ROOM;

    /* enable register writing - chris */
    ret = smb345_set_writable(smb, true);
    if (ret < 0)
        goto error;

    smb345_dev = smb;
    smb345_dump_registers(NULL);

    /* Refer to SMB345 Application Note 72 to solve serious problems */
    ret = smb345_masked_write(smb->client,
            OTG_TLIM_THERM_CNTRL_REG,
            OTG_CURRENT_LIMIT_AT_USBIN_MASK,
            OTG_CURRENT_LIMIT_250mA);
    if (ret < 0)
        goto error;

    /* Init Runtime PM State */
    pm_runtime_put_noidle(&smb->client->dev);
    pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

    /* qcom */
    ret = smb345_parse_dt(smb);
    if (ret < 0) {
        dev_err(&client->dev,
        "Couldn't to parse dt ret = %d\n", ret);
        goto error;
    }

    ret = smb345_regulator_init(smb);
    if  (ret) {
        dev_err(&client->dev,
        "Couldn't initialize smb345 ragulator rc=%d\n", ret);
        goto error;
    }

    smb->running = true;
    smb->dentry = debugfs_create_file("smb", S_IRUGO, NULL, smb,
                      &smb345_debugfs_fops);
    ret = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

    ret = smb345_register_power_supply(&client->dev);
    if (ret < 0)
        goto error;

    BAT_DBG(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

    ret = init_asus_charging_limit_toggle();
    if (ret) {
        dev_warn(&smb345_dev->client->dev, "Unable to create proc file\n");
        goto error;
    }

    INIT_WORK(&smb345_dev->chrgr_type_work, chrgr_type_work_func);
    INIT_DEFERRABLE_WORK(&smb345_dev->aicl_dete_work, aicl_dete_worker);
    INIT_DEFERRABLE_WORK(&smb345_dev->cover_detect_work, cover_detect_work_func);
    INIT_DEFERRABLE_WORK(&smb345_dev->cos_cb81_power_detect_work, cos_cb81_power_detect_work_func);
    cover_cable_status_register_client(&cover_type_notifier);
    cable_status_register_client(&charger_type_notifier);

    schedule_delayed_work(&smb345_dev->aicl_dete_work, 30*HZ);
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
    callback_struct = register_screen_state_notifier(&smb345_screen_changed_listener);
#endif
    return 0;

error:
    wake_lock_destroy(&smb->wakelock);
    return ret;
}

static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    switch (event) {
    case POWER_SUPPLY_TYPE_UNKNOWN:
        BAT_DBG_E("-----------------------------------------------TYPE_NONE\n");
#ifdef ME372CG_ENG_BUILD
        flag_eng_stop_charging = false;
#endif
        /* wake lock to prevent system instantly
           enter S3 while it's in resuming flow */
        wake_lock_timeout(&smb345_dev->wakelock, 4*HZ);
        g_extern_device = CABLE_OUT;
        schedule_work(&smb345_dev->chrgr_type_work);
        break;
    case POWER_SUPPLY_TYPE_USB:
        BAT_DBG_E("-----------------------------------------------USB_SDP\n");
        if (!VBUS_IN()) {
            pr_err("*********************************************************\n");
            pr_err("                           BUG                           \n");
            pr_err("                  NO USB CONNECTOR VBUS                  \n");
            pr_err("*********************************************************\n");
            break;
        }
        g_extern_device = USB_IN;
        schedule_work(&smb345_dev->chrgr_type_work);
        break;
    case POWER_SUPPLY_TYPE_USB_DCP:
        BAT_DBG_E("-----------------------------------------------USB_DCP\n");
        if (!VBUS_IN()) {
            pr_err("*********************************************************\n");
            pr_err("                           BUG                           \n");
            pr_err("                  NO USB CONNECTOR VBUS                  \n");
            pr_err("*********************************************************\n");
            break;
        }
        g_extern_device = AC_IN;
        schedule_work(&smb345_dev->chrgr_type_work);
        break;
    case POWER_SUPPLY_TYPE_OTG:
        BAT_DBG_E("-----------------------------------------------OTG\n");
        g_extern_device = ENABLE_5V;
        schedule_work(&smb345_dev->chrgr_type_work);
        break;
    case POWER_SUPPLY_TYPE_OTGOUT:
        BAT_DBG_E("-----------------------------------------------OTG_OUT\n");
        g_extern_device = DISABLE_5V;
        schedule_work(&smb345_dev->chrgr_type_work);
        break;
    default:
        BAT_DBG_E("-----------------------------------------------DAMN *\n");
        break;
    }

    return NOTIFY_OK;
}

static int smb345_remove(struct i2c_client *client)
{
    struct smb345_charger *smb = i2c_get_clientdata(client);

    /* qcom */
    regulator_unregister(smb->otg_vreg.rdev);

    if (!IS_ERR_OR_NULL(smb->dentry))
        debugfs_remove(smb->dentry);

    smb->running = false;

    wake_lock_destroy(&smb->wakelock);
    pm_runtime_get_noresume(&smb->client->dev);

    return 0;
}

static void smb345_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

    cancel_delayed_work(&smb345_dev->aicl_dete_work);
    cancel_delayed_work(&smb345_dev->cover_detect_work);
    cancel_delayed_work(&smb345_dev->cos_cb81_power_detect_work);

    /* charger control jeita */
    smb3xx_charger_control_jeita();

    /* registers dump */
    smb345_dump_registers(NULL);

    /* turn off OTG */
    otg(0, false);

    /* turn off Cover CHG EN */
    COVER_CHARGING_TOGGLE(false);
}

#ifdef CONFIG_PM
static int smb345_prepare(struct device *dev)
{
    struct smb345_charger *smb = dev_get_drvdata(dev);

    dev_info(&smb->client->dev, "smb345 suspend\n");
    BAT_DBG(" smb3xx prepare\n");

    /*
     * disable irq here doesn't mean smb345 interrupt
     * can't wake up system. smb345 interrupt is triggered
     * by GPIO pin, which is always active.
     * When resume callback calls enable_irq, kernel
     * would deliver the buffered interrupt (if it has) to
     * driver.
     */
    if (smb->client->irq > 0)
        disable_irq(smb->client->irq);

    return 0;
}

static void smb345_complete(struct device *dev)
{
    struct smb345_charger *smb = dev_get_drvdata(dev);

    dev_info(&smb->client->dev, "smb345 resume\n");
    BAT_DBG(" smb3xx complete\n");

}
#else
#define smb345_prepare NULL
#define smb345_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb345_runtime_suspend(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int smb345_runtime_resume(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int smb345_runtime_idle(struct device *dev)
{

    dev_info(dev, "%s called\n", __func__);
    return 0;
}
#else
#define smb345_runtime_suspend    NULL
#define smb345_runtime_resume    NULL
#define smb345_runtime_idle    NULL
#endif

static const struct of_device_id smb3xx_match[] = {
    { .compatible = "qcom,smb345" },
    { },
};

static const struct i2c_device_id smb345_id[] = {
    {SMB345_DEV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, smb345_id);

static const struct dev_pm_ops smb345_pm_ops = {
    .prepare        = smb345_prepare,
    .complete        = smb345_complete,
    .runtime_suspend    = smb345_runtime_suspend,
    .runtime_resume        = smb345_runtime_resume,
    .runtime_idle        = smb345_runtime_idle,
};

static struct i2c_driver smb345_driver = {
    .driver = {
        .name    = SMB345_DEV_NAME,
        .owner    = THIS_MODULE,
        .pm    = &smb345_pm_ops,
        .of_match_table = of_match_ptr(smb3xx_match),
    },
    .probe        = smb345_probe,
    .remove        = smb345_remove,
    .shutdown    = smb345_shutdown,
    .id_table    = smb345_id,
};

static int __init smb345_init(void)
{
    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
    HW_ID = Read_HW_ID();
    return i2c_add_driver(&smb345_driver);
}
module_init(smb345_init);

static void __exit smb345_exit(void)
{
    i2c_del_driver(&smb345_driver);
}
module_exit(smb345_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SMB345 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb345");

