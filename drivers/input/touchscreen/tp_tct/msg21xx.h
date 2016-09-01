#ifndef __MSG21XX_TS_H__
#define __MSG21XX_TS_H__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
//#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/input.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
#include <linux/input/vir_ps.h>
#endif

#define FT_FW_NAME_MAX_LEN	50

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define SLAVE_I2C_ID_DBBUS         (0xC4>>1)
#define SLAVE_I2C_ID_DWI2C      (0x4C>>1)

struct msg21xx_ts_platform_data {
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	int (*power_init) (bool);
	int (*power_on) (bool);
};

struct msg21xx_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct msg21xx_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;

#if defined(MSTAR_PWRON_UPGRADE)
	struct delayed_work mstar_update_work;
#endif

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
	struct mutex ts_mutex;

#if defined(MSTAR_TP_GESTURE)
	u8 gesture_id;  /* set from APP, need to open gesture function */
	u8 gesture_set;  /* gesture function should be open in suspend, close in resume */
	u8 gesture_code;  /* gesture codes get from TP */
	u8 gesture_pnum;  /* the number of points the gesture contains of */
	u8 gesture_on;  /* the state indicates whether the gesture are openned successfully */
#endif
};

#if defined(MSTAR_PWRON_UPGRADE)
extern int mstar_fw_upgrade_start(const u8 *data, u32 data_len, bool force);
extern u8 mstar_init_update_proc(struct msg21xx_ts_data *ts);
#endif

#ifdef MSTAR_ITO_TEST
extern void ito_test_create_entry(struct i2c_client *i2c_client);
#endif

#if defined(MSTAR_TP_GESTURE)
extern char *fw_version;
extern int get_customer_firmware_version(void);
extern void mstar_tp_gestures_register ( struct msg21xx_ts_data *data);
extern int mstar_tp_interrupt(struct msg21xx_ts_data *tsdata, unsigned char checksum, unsigned char *data);
extern int mstar_tp_suspend(struct msg21xx_ts_data *data);
extern int mstar_tp_resume(struct msg21xx_ts_data *data);
extern void keyset_mstartp_gesture(struct input_dev *input_dev);
extern int write_i2c_seq(unsigned char addr, unsigned char *buf,	unsigned short size);
#endif

#endif
