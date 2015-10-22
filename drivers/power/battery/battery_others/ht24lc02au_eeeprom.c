/*
 * Copyright (c) 2015, ASUSTek, Inc. All Rights Reserved.
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

#define HT24LC02AU_DEV_NAME "ht24lc02au"
#define I2C_RETRY_COUNT 2
#define I2C_RETRY_DELAY 5
#define I2C_WRITE_DELAY 1

/*
ISN: isn string
SSN: ssn string
MOD: model name string
COV: cover type string
ACT: activated flag string
*/

/* max number of bytes (each register in EEPROM can store 1 byte) */
#define NUM_BYTE_ISN    32
#define NUM_BYTE_SSN    20
#define NUM_BYTE_MOD     5
#define NUM_BYTE_RSV     3
#define NUM_BYTE_TIN     1
#define NUM_BYTE_COV     2
#define NUM_BYTE_ACT     1

/* start register address where to stored data (0x00 ~ 0xFF) */
#define ISN_START_REG    0
#define SSN_START_REG    (ISN_START_REG + NUM_BYTE_ISN)
#define MOD_START_REG    (SSN_START_REG + NUM_BYTE_SSN)
#define RSV_START_REG    (MOD_START_REG + NUM_BYTE_MOD)
#define TIN_START_REG    (RSV_START_REG + NUM_BYTE_RSV)
#define COV_START_REG    (TIN_START_REG + NUM_BYTE_TIN)
#define ACT_START_REG    (COV_START_REG + NUM_BYTE_COV)

DEFINE_MUTEX(cover_type_lock);
/*
    0: unknown
    1: CB81
    2: CA81
*/
static int cover_type;

extern bool COVER_ATTACHED(void);
extern bool COVER_ATTACHED_UPI(void);

struct ht24lc02au_eeprom {
    struct mutex        lock;
    struct i2c_client    *client;
    struct device        *dev;
    struct dentry        *dentry;
};
struct ht24lc02au_eeprom *ht24lc02au;

static int ht24lc02au_read(struct ht24lc02au_eeprom *ht24, u8 reg)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    if (!COVER_ATTACHED_UPI())
        return -EINVAL;

    do
    {
        ret = i2c_smbus_read_byte_data(ht24->client, reg);
        if (ret < 0) {
            retry_count--;
            dev_warn(&ht24->client->dev, "fail to read reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int ht24lc02au_write(struct ht24lc02au_eeprom *ht24, u8 reg, u8 val)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    if (!COVER_ATTACHED_UPI())
        return -EINVAL;

    do
    {
        ret = i2c_smbus_write_byte_data(ht24->client, reg, val);
        if (ret < 0) {
            retry_count--;
            dev_warn(&ht24->client->dev, "fail to write reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
        else msleep(I2C_WRITE_DELAY);
    } while (ret < 0 && retry_count > 0);

    return ret;
}

/*
return:
    0      means success
    others means fail
*/
int WRITE_EEPROM(u8 address, u8 value)
{
    int ret;

    if (!ht24lc02au)
        return -ENODEV;

    ret = ht24lc02au_write(ht24lc02au, address, value);
    if (ret < 0)
        return ret;

    return 0;
}
EXPORT_SYMBOL_GPL(WRITE_EEPROM);

/*
return:
    Value read from EEPROM.
    Negative value means fail to read.
*/
int READ_EEPROM(u8 address)
{
    if (!ht24lc02au)
        return -ENODEV;

    return ht24lc02au_read(ht24lc02au, address);
}
EXPORT_SYMBOL_GPL(READ_EEPROM);

bool IS_CA81(void)
{
    int ctype;

    mutex_lock(&cover_type_lock);
    ctype = cover_type;
    mutex_unlock(&cover_type_lock);

    return (ctype == 2);
}
EXPORT_SYMBOL_GPL(IS_CA81);
bool _IS_CA81_(void)
{
    /* CB81 is '1' */
    char CA81_TYPE = '2';

    if (!ht24lc02au)
        return false;

    if (ht24lc02au_read(ht24lc02au, 0x3D) == (int)CA81_TYPE)
    {
        mutex_lock(&cover_type_lock);
        cover_type = 2;
        mutex_unlock(&cover_type_lock);
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(_IS_CA81_);

bool IS_CB81(void)
{
    int ctype;

    mutex_lock(&cover_type_lock);
    ctype = cover_type;
    mutex_unlock(&cover_type_lock);

    return (ctype == 1);
}
EXPORT_SYMBOL_GPL(IS_CB81);
bool _IS_CB81_(void)
{
    /* CB81 is '1' */
    char CB81_TYPE = '1';

    if (!ht24lc02au)
        return false;

    if (ht24lc02au_read(ht24lc02au, 0x3D) == (int)CB81_TYPE)
    {
        mutex_lock(&cover_type_lock);
        cover_type = 1;
        mutex_unlock(&cover_type_lock);
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(_IS_CB81_);
void SET_COVER_DETACHED(void)
{
    mutex_lock(&cover_type_lock);
    cover_type = 0;
    mutex_unlock(&cover_type_lock);
}
EXPORT_SYMBOL_GPL(SET_COVER_DETACHED);

static bool CA81_WITH_TIN_PLATE(void)
{
    int temp = -1;

    temp = ht24lc02au_read(ht24lc02au, 0x3C);
    BAT_DBG_E(" temp = %d\n", temp);
    if (temp == (int)0x31)
    //if (ht24lc02au_read(ht24lc02au, 0x3C) == 0x31);
        return true;
    return false;
}
bool IS_FAIL_CA81(void)
{
    if (IS_CA81()) {
        if (CA81_WITH_TIN_PLATE())
            return false;
        else
            return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(IS_FAIL_CA81);

int ht24lc02au_write_all_registers(struct seq_file *s, u8 val)
{
    struct ht24lc02au_eeprom *ht24;
    u16 reg, regl, regh;
    int ret;
    u16 array[16] = {0};

    ht24 = ht24lc02au;

    for (regh = 0x00; regh <= 0xF0; regh+=0x10) {
        for (regl = 0x00; regl <= 0x0F; regl+=0x01) {
            reg = regl + regh;
            if (reg >= 0xff) break; // FIXME: cannot read reg0xff
            ret = ht24lc02au_write(ht24, reg, val);
        }
        memset(array, 0, sizeof(array));
    }

    pr_info("\n");
    if (s)
        seq_printf(s, "\n");

    return 0;
}

int ht24lc02au_dump_registers(struct seq_file *s)
{
    struct ht24lc02au_eeprom *ht24;
    u16 reg, regl, regh;
    u16 array[16] = {0};
    ht24 = ht24lc02au;

    if (!COVER_ATTACHED_UPI()) {
        pr_info("* Cover is dettached *\n");
        return 0;
    }
    /* config output high to enable COVER I2C*/
    gpio_set_value(COVER_I2C_ENABLE_GPIO, 1);

    pr_info("========================================== EEPROM registers ==========================================\n");
    pr_info("Array | 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F\n");
    pr_info("------|-----------------------------------------------------------------------------------------------\n");
    if (s) {
        seq_printf(s, "========================================== EEPROM registers ==========================================\n");
        seq_printf(s, "Array | 0x00  0x01  0x02  0x03  0x04  0x05  0x06  0x07  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F\n");
        seq_printf(s, "------|-----------------------------------------------------------------------------------------------\n");
    }
    for (regh = 0x00; regh <= 0xF0; regh+=0x10) {
        for (regl = 0x00; regl <= 0x0F; regl+=0x01) {
            reg = regl + regh;
            if (reg >= 0xff) break; // FIXME: cannot read reg0xff
            array[regl] = ht24lc02au_read(ht24, reg);
        }

        pr_info(" 0x%02x | 0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n", regh, array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], array[13], array[14], array[15]);
        if (s)
            seq_printf(s, " 0x%02x | 0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n", regh, array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], array[13], array[14], array[15]);
        memset(array, 0, sizeof(array));
    }

    pr_info("\n");
    if (s)
        seq_printf(s, "\n");

    return 0;
}

#ifdef CHRIS

static int ht24lc02au_read_reg(struct i2c_client *client, int reg,
                u8 *val, int ifDebug)
{
    s32 ret;
    struct ht24lc02au_eeprom *ht24lc02au_chg;

    ht24lc02au_chg = i2c_get_clientdata(client);
    ret = i2c_smbus_read_byte_data(ht24lc02au_chg->client, reg);
    if (ret < 0) {
        dev_err(&ht24lc02au_chg->client->dev,
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

static int ht24lc02au_write_reg(struct i2c_client *client, int reg,
                        u8 val)
{
    s32 ret;
    struct ht24lc02au_eeprom *ht24lc02au_chg;

    ht24lc02au_chg = i2c_get_clientdata(client);

    ret = i2c_smbus_write_byte_data(ht24lc02au_chg->client, reg, val);
    if (ret < 0) {
        dev_err(&ht24lc02au_chg->client->dev,
            "i2c write fail: can't write %02X to %02X: %d\n",
            val, reg, ret);
        return ret;
    }
    return 0;
}

static int ht24lc02au_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = ht24lc02au_read_reg(client, reg, &temp, 0);
        if (rc) {
            retry_count--;
            pr_err("*ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int ht24lc02au_masked_write(struct i2c_client *client, int reg,
        u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
    rc = ht24lc02au_read_reg(client, reg, &temp, 0);
    if (rc) {
        retry_count--;
        pr_err("*ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("ht24lc02au_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
    rc = ht24lc02au_write_reg(client, reg, temp);
    if (rc) {
        retry_count--;
        pr_err("*ht24lc02au_write failed: reg=%03X, rc=%d\n", reg, rc);
        msleep(I2C_RETRY_DELAY);
    }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("ht24lc02au_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}
#endif

static int ht24lc02au_debugfs_show(struct seq_file *s, void *data)
{
    ht24lc02au_dump_registers(s);

    return 0;
}

static int ht24lc02au_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, ht24lc02au_debugfs_show, inode->i_private);
}

static const struct file_operations ht24lc02au_debugfs_fops = {
    .open        = ht24lc02au_debugfs_open,
    .read        = seq_read,
    .llseek        = seq_lseek,
    .release    = single_release,
};

static ssize_t asus_proc_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
    char proc_buf[5] = {'\0'};
    int val;

    if (count > sizeof(proc_buf)) {
        BAT_DBG_E("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buf, count)) {
        BAT_DBG_E("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    sscanf(proc_buf, "%X", &val);
    ht24lc02au_write_all_registers(NULL, val);

    return count;
}
static int asus_proc_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: \n", __func__);
    return 0;
}
static int asus_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, asus_proc_read, NULL);
}
int init_asus_proc_toggle(void)
{
    struct proc_dir_entry *entry;

    static const struct file_operations asus_proc_fops = {
        .owner = THIS_MODULE,
        .open = asus_proc_open,
        .read = seq_read,
        .write = asus_proc_write,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/eepromw", 0666, NULL,
        &asus_proc_fops);
    if (!entry) {
        pr_info("Unable to create proc/driver/eepromw\n");
        return -EINVAL;
    }

    return 0;
}

/* ============================ Device Attribute ============================ */
static ssize_t get_isn_info(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[64] = {'\0'}; /* 32 byte isn */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = ISN_START_REG; reg < NUM_BYTE_ISN; reg++)
        array[reg] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_isn_info(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 32 byte isn from 0 - 31 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("buf size: %d, count: %d\n", sizeof(buf), count);
    for (reg = ISN_START_REG; reg < NUM_BYTE_ISN; reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + reg));
    }

    return count;
}
static ssize_t get_ssn_info(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[32] = {'\0'}; /* 20 byte ssn */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = SSN_START_REG; reg < (SSN_START_REG + NUM_BYTE_SSN); reg++)
        array[reg - SSN_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_ssn_info(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 20 byte ssn from 32 - 51 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %d\n", count);
    for (reg = SSN_START_REG; reg < (SSN_START_REG + NUM_BYTE_SSN); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - SSN_START_REG)));
    }

    return count;
}
static ssize_t get_model_name(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[8] = {'\0'}; /* 5 byte model name */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = MOD_START_REG; reg < (MOD_START_REG + NUM_BYTE_MOD); reg++)
        array[reg - MOD_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_model_name(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 9 byte model name from 52 - 60 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %d\n", count);
    for (reg = MOD_START_REG; reg < (MOD_START_REG + NUM_BYTE_MOD); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - MOD_START_REG)));
    }

    return count;
}
static ssize_t get_cover_type(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[8] = {'\0'}; /* 2 byte cover type */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = COV_START_REG; reg < (COV_START_REG + NUM_BYTE_COV); reg++)
        array[reg - COV_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_cover_type(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 2 byte cover type from 61 - 62 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %d\n", count);
    for (reg = COV_START_REG; reg < (COV_START_REG + NUM_BYTE_COV); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - COV_START_REG)));
    }

    return count;
}
static ssize_t get_activ_flag(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[2] = {'\0'}; /* 1 byte activated flag */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = ACT_START_REG; reg < (ACT_START_REG + NUM_BYTE_ACT); reg++)
        array[reg - ACT_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_activ_flag(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 1 byte activated flag from 63 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %d\n", count);
    for (reg = ACT_START_REG; reg < (ACT_START_REG + NUM_BYTE_ACT); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - ACT_START_REG)));
    }

    return count;
}
static ssize_t get_tin_plate(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u16 reg;
    char array[2] = {'\0'}; /* 1 byte activated flag */
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    for (reg = TIN_START_REG; reg < (TIN_START_REG + NUM_BYTE_TIN); reg++)
        array[reg - TIN_START_REG] = ht24lc02au_read(ht24, reg);

    return snprintf(buf, PAGE_SIZE, "%s", array);
}
static ssize_t set_tin_plate(struct device *dev, struct device_attribute *attr,
                          const char *buf, size_t count)
{
    /* 1 byte activated flag from 63 */
    int ret, reg;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return count;
    if (!COVER_ATTACHED_UPI())
        return count;

    pr_info("count: %d\n", count);
    for (reg = TIN_START_REG; reg < (TIN_START_REG + NUM_BYTE_TIN); reg++) {
        ret = ht24lc02au_write(ht24, reg, *(buf + (reg - TIN_START_REG)));
    }

    return count;
}
static ssize_t cover_activate(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    u8 active;
    int ret;
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    if (!ht24)
        return snprintf(buf, PAGE_SIZE, "unknown\n");
    if (!COVER_ATTACHED_UPI())
        return snprintf(buf, PAGE_SIZE, "cover dettached\n");

    active = 0x31;
    ret = ht24lc02au_write(ht24, ACT_START_REG, active);
    if (!ret)
        return snprintf(buf, PAGE_SIZE, "PASS");
    return snprintf(buf, PAGE_SIZE, "FAIL");
}
static DEVICE_ATTR(isn_info, S_IRUGO | S_IWUSR, get_isn_info, set_isn_info);
static DEVICE_ATTR(ssn_info, S_IRUGO | S_IWUSR, get_ssn_info, set_ssn_info);
static DEVICE_ATTR(model_name, S_IRUGO | S_IWUSR, get_model_name, set_model_name);
static DEVICE_ATTR(tin_plate, S_IRUGO | S_IWUSR, get_tin_plate, set_tin_plate);
static DEVICE_ATTR(cover_type, S_IRUGO | S_IWUSR, get_cover_type, set_cover_type);
static DEVICE_ATTR(activ_flag, S_IRUGO | S_IWUSR, get_activ_flag, set_activ_flag);
static DEVICE_ATTR(activate, S_IRUGO | S_IWUSR, cover_activate, set_activ_flag);
static struct attribute *dev_attrs[] = {
    &dev_attr_isn_info.attr,
    &dev_attr_ssn_info.attr,
    &dev_attr_model_name.attr,
    &dev_attr_tin_plate.attr,
    &dev_attr_cover_type.attr,
    &dev_attr_activ_flag.attr,
    &dev_attr_activate.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
/* ============================ Device Attribute ============================ */

static int ht24lc02au_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct ht24lc02au_eeprom *ht24;
    int ret = 0;

    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    ht24 = devm_kzalloc(dev, sizeof(*ht24), GFP_KERNEL);
    if (!ht24)
        return -ENOMEM;

    i2c_set_clientdata(client, ht24);
    ht24->client = client;
    ht24->dev = dev;
    ht24lc02au = ht24;

    ht24lc02au_dump_registers(NULL);

#if 0
    if (COVER_ATTACHED_UPI()) {
    if (!ht24lc02au_write_all_registers(NULL, 0xCC))
        ht24lc02au_dump_registers(NULL);
    }
#endif

    ht24->dentry = debugfs_create_file("eeprom", S_IRUGO, NULL, ht24,
                      &ht24lc02au_debugfs_fops);
    ret = init_asus_proc_toggle();
    ret = sysfs_create_group(&client->dev.kobj, &dev_attr_grp);
    BAT_DBG(" %s: CA81: %s\n", __func__, _IS_CA81_() ? "Yes" : "No");
    BAT_DBG(" %s: CB81: %s\n", __func__, _IS_CB81_() ? "Yes" : "No");
    BAT_DBG(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

    if (IS_FAIL_CA81())
        BAT_DBG(" **************************************************************** fail CA81(No Tin Plate)\n");

    return ret;
}

static int ht24lc02au_remove(struct i2c_client *client)
{
    //struct ht24lc02au_eeprom *ht24 = i2c_get_clientdata(client);

    return 0;
}

static void ht24lc02au_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

}

#ifdef CONFIG_PM
static int ht24lc02au_prepare(struct device *dev)
{
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    dev_info(&ht24->client->dev, "ht24lc02au suspend\n");
    return 0;
}

static void ht24lc02au_complete(struct device *dev)
{
    struct ht24lc02au_eeprom *ht24 = dev_get_drvdata(dev);

    dev_info(&ht24->client->dev, "ht24lc02au resume\n");

}
#else
#define ht24lc02au_prepare NULL
#define ht24lc02au_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int ht24lc02au_runtime_suspend(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int ht24lc02au_runtime_resume(struct device *dev)
{
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int ht24lc02au_runtime_idle(struct device *dev)
{

    dev_info(dev, "%s called\n", __func__);
    return 0;
}
#else
#define ht24lc02au_runtime_suspend    NULL
#define ht24lc02au_runtime_resume    NULL
#define ht24lc02au_runtime_idle    NULL
#endif

static const struct of_device_id ht24lc02au_match[] = {
    { .compatible = "holtek,ht24lc02au" },
    { },
};

static const struct i2c_device_id ht24lc02au_id[] = {
    {HT24LC02AU_DEV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, ht24lc02au_id);

static const struct dev_pm_ops ht24lc02au_pm_ops = {
    .prepare        = ht24lc02au_prepare,
    .complete        = ht24lc02au_complete,
    .runtime_suspend    = ht24lc02au_runtime_suspend,
    .runtime_resume        = ht24lc02au_runtime_resume,
    .runtime_idle        = ht24lc02au_runtime_idle,
};

static struct i2c_driver ht24lc02au_driver = {
    .driver = {
        .name    = HT24LC02AU_DEV_NAME,
        .owner    = THIS_MODULE,
        .pm    = &ht24lc02au_pm_ops,
        .of_match_table = of_match_ptr(ht24lc02au_match),
    },
    .probe        = ht24lc02au_probe,
    .remove        = ht24lc02au_remove,
    .shutdown    = ht24lc02au_shutdown,
    .id_table    = ht24lc02au_id,
};

static int __init ht24lc02au_init(void)
{
    BAT_DBG(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
    return i2c_add_driver(&ht24lc02au_driver);
}
module_init(ht24lc02au_init);

static void __exit ht24lc02au_exit(void)
{
    i2c_del_driver(&ht24lc02au_driver);
}
module_exit(ht24lc02au_exit);

module_param(cover_type, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(COVER_TYPE, "Cover Type Id");

MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("HOLTEK EEPROM driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ht24lc02au");
