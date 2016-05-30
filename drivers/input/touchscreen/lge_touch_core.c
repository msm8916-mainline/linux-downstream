/* Lge_touch_core.c
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
//#include <linux/sysdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/regulator/consumer.h>
#include <linux/input/lge_touch_core.h>

#ifdef CONFIG_MTK_TOUCHPANEL
#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>

#include "tpd.h"
#else
#include <mach/board.h>
#endif

#include "lge_touch_platform.h"

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 4, 67))
#define KERNEL_ABOVE_3_4_67
#endif
#define MAX_ATTRIBUTE_ARRAY_SIZE 		30

struct touch_device_driver	*touch_device_func;
struct workqueue_struct		*touch_wq;
struct delayed_work 		*thread_irq;

int lpwg_value[4] = {0};

/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lge_touch_core/parameters/debug_mask
 */
u32 touch_debug_mask = DEBUG_BASE_INFO | DEBUG_LPWG;

/* set_touch_handle / get_touch_handle
 *
 * Developer can save their object using 'set_touch_handle'.
 * Also, they can restore that using 'get_touch_handle'.
 */
void set_touch_handle(struct i2c_client *client, void *h_touch)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	ts->h_touch = h_touch;
}

void *get_touch_handle(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	return ts->h_touch;
}

/* send_uevent
 *
 * It will be used to send u-event to Android-framework.
 */
static struct bus_type touch_subsys = {
	.name = LGE_TOUCH_NAME,
	.dev_name = "lge_touch",
};

static struct device device_touch = {
	.id    = 0,
	.bus   = &touch_subsys,
};

void send_uevent(char *string[2])
{
	kobject_uevent_env(&device_touch.kobj, KOBJ_CHANGE, string);
	TOUCH_DEBUG(DEBUG_BASE_INFO, "uevent[%s]\n", string[0]);
}

/* send_uevent_lpwg
 *
 * It uses wake-lock in order to prevent entering the sleep-state,
 * during recognition or verification.
 */
#define VALID_LPWG_UEVENT_SIZE 3
static char *lpwg_uevent[VALID_LPWG_UEVENT_SIZE][2] = {
{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL},
{"TOUCH_GESTURE_WAKEUP=SIGNATURE", NULL}
};

void send_uevent_lpwg(struct i2c_client *client, int type)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(3000));

	if (type > 0 && type <= VALID_LPWG_UEVENT_SIZE
		&& atomic_read(&ts->state.uevent_state) == UEVENT_IDLE) {
		atomic_set(&ts->state.uevent_state, UEVENT_BUSY);
		send_uevent(lpwg_uevent[type-1]);
	}
}

/* update_status
 *
 * Other drivers can notify their status to touch driver.
 * Do not use 'i2c_client' in other function.
 */
struct i2c_client	*client_only_for_other_module;
void update_status(int code, int value)
{
	struct lge_touch_data *ts
		= i2c_get_clientdata(client_only_for_other_module);
	struct state_info *state = &ts->state;

	if (code == NOTIFY_TA_CONNECTION) {
		atomic_set(&state->ta_state,
				value ? TA_CONNECTED : TA_DISCONNECTED);
	} else if (code == NOTIFY_TEMPERATURE_CHANGE) {
		atomic_set(&state->temperature_state, value);
	} else if (code == NOTIFY_PROXIMITY) {
		atomic_set(&state->proximity_state,
				value ? PROXIMITY_NEAR : PROXIMITY_FAR);
	} else if (code == NOTIFY_HALL_IC) {
		atomic_set(&state->hallic_state,
				value ? HALL_COVERED : HALL_NONE);
	} else if (code == NOTIFY_KEYGUARD) {
		atomic_set(&state->keyguard_state, value);
	} else if (code == NOTIFY_IME) {
		atomic_set(&state->ime_state, value);
	}

	state->code = (u8)code;
	state->value = (u32)value;

	TOUCH_DEBUG(DEBUG_BASE_INFO, "code[%d] value[%d]\n", code, value);

	queue_delayed_work(touch_wq, &ts->work_notify, 0);
}
EXPORT_SYMBOL(update_status);

/* report_key
 *
 * report H/W key event
 */
static inline void report_key(struct input_dev *dev,
	unsigned int code, int value)
{
	input_report_key(dev, code, value);
	TOUCH_DEBUG(DEBUG_BUTTON | DEBUG_BASE_INFO , "KEY[%d] is %s(%d)\n",
			code, value ? "pressed" : "released", value);
}

/* key_event
 *
 * Key event processing (only for H/W key)
 */
static int key_event(struct lge_touch_data *ts)
{
	struct touch_data *curr_data = &ts->ts_curr_data;
	struct touch_data *prev_data = &ts->ts_prev_data;
	struct b_data *c_button = &curr_data->button_data;
	struct b_data *p_button = &prev_data->button_data;

	if (curr_data->total_num > 0) {
		/* button release process */
		if (p_button->key_code != 0
				&& (p_button->state == BUTTON_PRESSED)) {
			report_key(ts->input_dev,
				p_button->key_code, BUTTON_CANCLED);
			c_button->state = BUTTON_CANCLED;
		}
	} else {
		/* case: curr. keycode is different from prev. */
		if (c_button->key_code != p_button->key_code) {
			if (p_button->state != BUTTON_RELEASED)
				report_key(ts->input_dev,
					p_button->key_code, BUTTON_RELEASED);
			if (c_button->state == BUTTON_PRESSED)
				report_key(ts->input_dev,
					c_button->key_code, BUTTON_PRESSED);
		} else { /* case: curr. keycode is same as prev. */
			if (p_button->state != BUTTON_CANCLED) {
				if (c_button->state != p_button->state)
					report_key(ts->input_dev,
						c_button->key_code,
						c_button->state);
			} else {
				if (c_button->state == BUTTON_PRESSED)
					report_key(ts->input_dev,
						c_button->key_code,
						BUTTON_PRESSED);
			}
		}
	}

	TOUCH_DEBUG(DEBUG_BUTTON,
		"C_button: code[%d] state[%d], P_button: code[%d] state[%d]\n",
		curr_data->button_data.key_code, curr_data->button_data.state,
		prev_data->button_data.key_code, prev_data->button_data.state);

	return 0;
}

/* report_event
 *
 * report abs event.
 * support : Both Protocol-A and Protocol-B
 */
static int report_event(const struct lge_touch_data *ts)
{
	const struct touch_data *curr_data = &ts->ts_curr_data;
	const struct touch_data *prev_data = &ts->ts_prev_data;
	const struct t_data *c_data;
	const struct t_data *p_data;

	static int touch_count_num = 0;
	int i = 0;
	u16 new_id = 0;
	u32 new_id_mask = 0;

	/* press */
	for (i = 0; i < curr_data->total_num; i++) {
		c_data = &curr_data->abs_data[i];
		if (!(curr_data->report_id_mask & (1 << c_data->id))
				|| c_data->id < 0 || c_data->id >= MAX_FINGER)
			continue;

		new_id = c_data->id;
		new_id_mask |= 1 << new_id;

		if (ts->pdata->role->protocol_type == MT_PROTOCOL_B)
			input_mt_slot(ts->input_dev, new_id);

		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, new_id);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, c_data->x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, c_data->y);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				c_data->pressure);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				c_data->width_major);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR,
				c_data->width_minor);
		input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
				c_data->orientation);

		TOUCH_DEBUG(DEBUG_ABS,
			"<%d:%d> pos[%4d,%4d] wm[%2d] wn[%2d] o[%2d] p[%3d]\n",
			c_data->id, new_id, c_data->x, c_data->y,
			c_data->width_major, c_data->width_minor,
			c_data->orientation, c_data->pressure);

		if ((ts->ts_curr_data.report_id_mask & (1 << ts->ts_curr_data.abs_data[i].id))
			&& !(ts->ts_prev_data.report_id_mask & (1 << ts->ts_curr_data.abs_data[i].id))) {
			++touch_count_num;

			TOUCH_DEBUG(DEBUG_ABS | DEBUG_BASE_INFO,"%d finger pressed : <%d> x[%4d] y[%4d] z[%3d]\n",
							touch_count_num, new_id,
							ts->ts_curr_data.abs_data[i].x,
							ts->ts_curr_data.abs_data[i].y,
							ts->ts_curr_data.abs_data[i].pressure);
		}

		if (ts->pdata->role->protocol_type == MT_PROTOCOL_A)
			input_mt_sync(ts->input_dev);
	}

	/* release */
	if (ts->pdata->role->protocol_type == MT_PROTOCOL_A) {
		if (curr_data->total_num == 0)
			input_mt_sync(ts->input_dev);
	} else {
		for (i = 0; i < ts->ts_prev_data.total_num; i++) {
			c_data = &curr_data->abs_data[i];
			p_data = &prev_data->abs_data[i];
			if (!(curr_data->report_id_mask & (1 << p_data->id))
					&& (prev_data->report_id_mask
						& (1 << p_data->id))
					&& c_data->id >= 0
					&& c_data->id < MAX_FINGER) {

				new_id = p_data->id;

				input_mt_slot(ts->input_dev, new_id);
				input_report_abs(ts->input_dev,
						ABS_MT_TRACKING_ID, -1);
				TOUCH_DEBUG(DEBUG_ABS,
					"<%d:%d>release\n", p_data->id, new_id);

				touch_count_num--;

				TOUCH_DEBUG(DEBUG_ABS | DEBUG_BASE_INFO, "touch_release[ ] : <%d> x[%4d] y[%4d]\n",
					new_id,
					ts->ts_prev_data.abs_data[i].x,
					ts->ts_prev_data.abs_data[i].y);
			}
		}
	}

	input_sync(ts->input_dev);

	if (curr_data->id_mask != prev_data->id_mask
			|| curr_data->report_id_mask
			!= prev_data->report_id_mask)
		TOUCH_DEBUG(DEBUG_ABS,
			"report[0x%x] : curr - id_mask[0x%x/0x%x] total_num[%d]"
			"/ prev - id_mask[0x%x/0x%x] total_num[%d]\n",
			new_id_mask, curr_data->id_mask,
			curr_data->report_id_mask, curr_data->total_num,
			prev_data->id_mask, prev_data->report_id_mask,
			prev_data->total_num);

	return 0;
}

/* release_all_touch_event
  *
  * before turn off the power, all events should be released.
  */
static void release_all_touch_event(struct lge_touch_data *ts)
{
	mutex_lock(&ts->thread_lock);

	memset(&ts->ts_curr_data, 0, sizeof(struct touch_data));
	if (ts->ts_prev_data.total_num)
		report_event(ts);
	if (ts->pdata->caps->button_support)
		key_event(ts);

	mutex_unlock(&ts->thread_lock);
}

/* send_touch_event
  *
  * Other module can send an touch event by this function
  */
void send_touch_event(struct touch_data *data)
{
	struct lge_touch_data *ts
		= i2c_get_clientdata(client_only_for_other_module);

	mutex_lock(&ts->thread_lock);

	memcpy(&ts->ts_curr_data, data, sizeof(struct touch_data));
	report_event(ts);

	mutex_unlock(&ts->thread_lock);
}
EXPORT_SYMBOL(send_touch_event);

/* power_control
 *
 * 'power_state' can has only 'ON' or 'OFF'. (not 'SLEEP' or 'WAKE')
 * During firmware upgrade, power state will not be changed.
 */
static int power_control(struct lge_touch_data *ts, int on_off)
{
	if (atomic_read(&ts->state.upgrade_state) == UPGRADE_START) {
		TOUCH_DEBUG(DEBUG_BASE_INFO,
			"'Firmware-upgrade' is not finished,"
			"so power cannot be changed.\n");
		return 0;
	}

	/* To ignore the probe time */
	if (ts->input_dev != NULL)
		release_all_touch_event(ts);

	if (atomic_read(&ts->state.power_state) != on_off) {
		DO_IF(touch_device_func->power(ts->client, on_off) != 0, error);
		atomic_set(&ts->state.power_state, on_off);
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO, "power_state[%d]\n", on_off);
	return 0;
error:
	return -1;
}

/* interrupt_control
 *
 * It cannot defend 'critical-section', perfectly,
 * but the possibility of an error occuring (race-condition) is very low.
 * (so, it is not a big problem, now. - I think.)
 *
 * It only can prevent execute twice, either 'enable_irq' or 'disable_irq'.
 */
static int interrupt_control(struct lge_touch_data *ts, int on_off)
{
	if (atomic_read(&ts->state.interrupt_state) != on_off) {
		atomic_set(&ts->state.interrupt_state, on_off);
		if (ts->pdata->role->wake_up_by_touch)
			on_off ? enable_irq_wake(ts->client->irq)
				: disable_irq_wake(ts->client->irq);
		else
			on_off ? ts_enable_irq(ts->client->irq)
				: ts_disable_irq(ts->client->irq);
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO, "interrupt_state[%d]\n", on_off);
	return 0;
}

/* safety_reset
 *
 * 1. release all events if it needs to.
 * 2. turn off the power.
 * 3. sleep (reset_delay)ms.
 * 4. turn on the power
 * 5. sleep (booting_delay)ms.
 *
 * After 'safety_reset', 'touch_ic_init' should be executed.
 */
static void safety_reset(struct lge_touch_data *ts)
{
	TOUCH_TRACE();

	DO_SAFE(power_control(ts, POWER_OFF), error);
	msleep(ts->pdata->role->reset_delay);
	DO_SAFE(power_control(ts, POWER_ON), error);
	msleep(ts->pdata->role->booting_delay);

	return;
error:
	/* TO_DO : error handling, if it needs */
	return;
}

/* touch_ic_init
 *
 * initialize the device_IC and variables.
 *
 * If you modify this function, please check the mutex.
 * Mutex should be unlocked when the thread exits this function.
 */
static int touch_ic_init(struct lge_touch_data *ts, int is_error)
{
	TOUCH_TRACE();

	mutex_lock(&ts->thread_lock);

	DO_IF(touch_device_func->init(ts->client) != 0, error);

	memset(&ts->ts_curr_data, 0, sizeof(struct touch_data));
	memset(&ts->ts_prev_data, 0, sizeof(struct touch_data));
	memset(&ts->ts_report_data, 0, sizeof(struct touch_data));

	mutex_unlock(&ts->thread_lock);
	return 0;

error:
	mutex_unlock(&ts->thread_lock);

	if (!is_error) {
		safety_reset(ts);
		return touch_ic_init(ts, 1);
	} else {
		power_control(ts, POWER_OFF);
		return -1;
	}
}

/* touch_init_func
 *
 * In order to reduce the booting-time,
 * we used delayed_work_queue instead of msleep or mdelay.
 */
static void touch_init_func(struct work_struct *work_init)
{
	struct lge_touch_data *ts = container_of(to_delayed_work(work_init),
		struct lge_touch_data, work_init);

	TOUCH_TRACE();

	DO_SAFE(touch_ic_init(ts, 0), error);

	interrupt_control(ts, INTERRUPT_ENABLE);

	return;
error:
	/* TO_DO : error handling, if it needs */
	return;
}

/* firmware_upgrade_func
 *
 */
static void firmware_upgrade_func(struct work_struct *work_upgrade)
{
	struct lge_touch_data *ts = container_of(to_delayed_work(work_upgrade),
		struct lge_touch_data, work_upgrade);

	TOUCH_TRACE();

	interrupt_control(ts, INTERRUPT_DISABLE);

	if (atomic_read(&ts->state.power_state) == POWER_OFF) {
		power_control(ts, POWER_ON);
		msleep(ts->pdata->role->booting_delay);
	}

	if(ts->fw_info.fw_force_upgrade == 1) {
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Start FW force upgrade\n");

		atomic_set(&ts->state.upgrade_state, UPGRADE_START);
		touch_device_func->fw_upgrade(ts->client, ts->fw_info.fw_path);
		atomic_set(&ts->state.upgrade_state, UPGRADE_FINISH);
		safety_reset(ts);
	}
	else {
		memset(&ts->fw_info, 0, sizeof(struct touch_firmware_module));
		touch_device_func->fw_ic_info(ts->client, &ts->fw_info);

		if(ts->fw_info.fw_type == OFFICIAL_VERSION
			&& strcmp(ts->fw_info.fw_bin, ts->fw_info.fw_ic)) {
			TOUCH_DEBUG(DEBUG_BASE_INFO, "Start FW upgrade\n");

			atomic_set(&ts->state.upgrade_state, UPGRADE_START);
			touch_device_func->fw_upgrade(ts->client, ts->fw_info.fw_path);
			atomic_set(&ts->state.upgrade_state, UPGRADE_FINISH);
			safety_reset(ts);
		}
	}

	interrupt_control(ts, INTERRUPT_ENABLE);
	touch_ic_init(ts, 0);

	memset(&ts->fw_info, 0, sizeof(struct touch_firmware_module));
	return;
}

/* touch_notify_func
 *
 * notify the status of other modules to specific driver.
 */
static void touch_notify_func(struct work_struct *work_notify)
{
	struct lge_touch_data *ts = container_of(to_delayed_work(work_notify),
		struct lge_touch_data, work_notify);

	TOUCH_TRACE();

	touch_device_func->notify(ts->client, ts->state.code, ts->state.value);

	return;
}

/* touch_thread_irq_handler
 *
 * If you modify this function, please check the mutex.
 * Mutex should be unlocked when the thread exits this function.
 *
 * HANDLE_RET : It is used for handling the return value.
 *
 */
#define HANDLE_RET(ret)				\
do {						\
	switch (ret) {				\
	case NO_ERROR:				\
		break;				\
	case NO_FILTER:				\
		goto do_not_filter;		\
		break;				\
	case IGNORE_EVENT_BUT_SAVE_IT:		\
		goto do_not_report;		\
		break;				\
	case IGNORE_EVENT: 			\
		goto ignore; 			\
		break;				\
	default: 				\
		goto error; 			\
		break;				\
	}					\
} while (0)

#ifdef CONFIG_MTK_TOUCHPANEL
static void touch_irq_handler(void)
#else
static irqreturn_t touch_irq_handler(int irq, void *dev_id)
#endif
{
	TOUCH_TRACE();

	if(thread_irq != NULL)
		queue_delayed_work(touch_wq, thread_irq, 0);

	return IRQ_HANDLED;
}

 static void touch_thread_irq_handler(struct work_struct *work_irq)
 {
	struct lge_touch_data *ts = container_of(to_delayed_work(work_irq),
		 struct lge_touch_data, work_irq);
	enum error_type ret = NO_ERROR;

	TOUCH_TRACE();

/*	if (atomic_read(&ts->state.pm_state) == PM_SUSPEND) {
		atomic_set(&ts->state.pm_state, PM_SUSPEND_IRQ);
		wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(1000));
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Interrupt occurs in suspend\n");
		return;
	}
*/
	mutex_lock(&ts->thread_lock);

	wake_lock_timeout(&ts->lpwg_wake_lock, msecs_to_jiffies(1000));

	HANDLE_RET(ret = touch_device_func->data(ts->client,
				 &ts->ts_curr_data, &ts->ts_prev_data));

	 /* After filtering, report_id_mask will be changed. */
	ts->ts_curr_data.report_id_mask = ts->ts_curr_data.id_mask;

 do_not_filter:
	if (ts->pdata->caps->button_support)
		report_key(ts->input_dev,
		ts->ts_curr_data.button_data.key_code,
		ts->ts_curr_data.button_data.state);

	DO_SAFE(report_event(ts), error);
	memcpy(&ts->ts_report_data, &ts->ts_curr_data,
				sizeof(struct touch_data));

 do_not_report:
	memcpy(&ts->ts_prev_data, &ts->ts_curr_data, sizeof(struct touch_data));
	memset(&ts->ts_curr_data, 0, sizeof(struct touch_data));
 ignore:
	mutex_unlock(&ts->thread_lock);
	return;

 error:
	mutex_unlock(&ts->thread_lock);
	safety_reset(ts);
	touch_ic_init(ts, 0);
	TOUCH_ERR_MSG("Interrupt Handling fail\n");
	return;
 }

 /* Sysfs - platform_data
  *
  * show_platform_data : Print all values of platform_data.
  * store_platform_data : User can change only the 'role'.
  */
static ssize_t show_platform_data(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	struct touch_platform_data *pdata = ts->pdata;
	int ret = 0;

	ret = sprintf(buf, "====== Platform data ======\n");
	ret += sprintf(buf+ret, "int_pin[%d] reset_pin[%d]\n",
			pdata->int_pin, pdata->reset_pin);
	ret += sprintf(buf+ret, "caps:\n");
	ret += sprintf(buf+ret, "\t%25s = %d\n", "button_support",
			pdata->caps->button_support);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "number_of_button",
			pdata->caps->number_of_button);
	ret += sprintf(buf+ret, "\t%25s = %d, %d, %d, %d\n", "button_name",
			pdata->caps->button_name[0],
			pdata->caps->button_name[1],
			pdata->caps->button_name[2],
			pdata->caps->button_name[3]);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_x", pdata->caps->max_x);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_y", pdata->caps->max_y);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_pressure",
			pdata->caps->max_pressure);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_width",
			pdata->caps->max_width);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_orientation",
			pdata->caps->max_width);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "max_id",
			pdata->caps->max_id);
	ret += sprintf(buf+ret, "role:\n");
	ret += sprintf(buf+ret, "\t%25s = %d\n", "protocol_type",
			pdata->role->protocol_type);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "report_mode",
			pdata->role->report_mode);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "delta_pos_threshold",
			pdata->role->delta_pos_threshold);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "booting_delay",
			pdata->role->booting_delay);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "reset_delay",
			pdata->role->reset_delay);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "wake_up_by_touch",
			pdata->role->wake_up_by_touch);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "use_sleep_mode",
			pdata->role->use_sleep_mode);
	ret += sprintf(buf+ret, "\t%25s = 0x%lx\n", "irqflags",
			pdata->role->irqflags);
	ret += sprintf(buf+ret, "pwr:\n");
	ret += sprintf(buf+ret, "\t%25s = %d\n", "use_regulator",
			pdata->pwr->use_regulator);
	ret += sprintf(buf+ret, "\t%25s = %s\n", "vdd", pdata->pwr->vdd);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "vdd_voltage",
			pdata->pwr->vdd_voltage);
	ret += sprintf(buf+ret, "\t%25s = %s\n", "vio", pdata->pwr->vio);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "vio_voltage",
			pdata->pwr->vio_voltage);
	ret += sprintf(buf+ret, "\t%25s = %s\n", "power",
			pdata->pwr->power ? "YES" : "NO");
	ret += sprintf(buf+ret, "firmware:\n");
	ret += sprintf(buf+ret, "\t%25s = %s\n", "fw_image",
			pdata->fw->fw_image);
	ret += sprintf(buf+ret, "\t%25s = %d\n", "need_upgrade",
			pdata->fw->need_upgrade);

	return ret;
}

static ssize_t store_platform_data(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	unsigned char string[30] = {0};
	u32 value = 0;

	/* It only can change the 'role'.
	 * (because changing 'others' is so dangerous)
	 * Therefore, if you want to change some variables by using 'sysfs',
	 * insert them in 'role' structure.
	 * After store, device should be re-initialized.
	 */

	sscanf(buf, "%s %d", string, &value);

	if (!strcmp(string, "protocol_type"))
		ts->pdata->role->protocol_type = value;
	else if (!strcmp(string, "report_mode"))
		ts->pdata->role->report_mode = value;
	else if (!strcmp(string, "delta_pos_threshold"))
		ts->pdata->role->delta_pos_threshold = value;
	else if (!strcmp(string, "booting_delay"))
		ts->pdata->role->booting_delay = value;
	else if (!strcmp(string, "reset_delay"))
		ts->pdata->role->reset_delay = value;
	else if (!strcmp(string, "wake_up_by_touch"))
		ts->pdata->role->wake_up_by_touch = value;
	else if (!strcmp(string, "use_sleep_mode"))
		ts->pdata->role->use_sleep_mode = value;
	else if (!strcmp(string, "irqflags"))
		ts->pdata->role->irqflags = value;

	return count;
}

/* Sysfs - power_ctrl
 *
 * store_power_ctrl : User can control the power - on, off, reset, init.
 */
static ssize_t store_power_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	unsigned char string[30] = {0};
	u32 value = 0;

	sscanf(buf, "%s %d", string, &value);

	if (!strcmp(string, "reset")) {
		safety_reset(ts);
		touch_ic_init(ts, 0);
	} else if (!strcmp(string, "init")) {
		touch_ic_init(ts, 0);
	} else if (!strcmp(string, "power")) {
		if (value)
			touch_device_func->power(ts->client, POWER_ON);
		else
			touch_device_func->power(ts->client, POWER_OFF);
	} else if (!strcmp(string, "safe_power")) {
		if (value) {
			power_control(ts, POWER_ON);
			msleep(ts->pdata->role->booting_delay);
		} else
			power_control(ts, POWER_OFF);
	}

	return count;
}

/* Sysfs -ic_rw
 *
 * show_ic_rw : User can read the register using 'reg' and 'value'.
 * Both 'reg' and 'value' are assigned by 'store_ic_rw'. Use 'assign' command.
 * store_ic_rw : User can write values to registers.
 *
 * reg, value : these variables are used to read the register.
 */
static int reg, value;

static ssize_t show_ic_rw(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	u32 ret = 0, tmp = 0;

	if (atomic_read(&ts->state.power_state) == POWER_OFF)
		return ret;

	do {
		touch_device_func->ic_ctrl(ts->client,
				IC_CTRL_READ, reg++, &tmp);
		ret += sprintf(buf+ret, "%d\n", tmp);
	} while (--value > 0);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "%s\n", buf);
	return ret;
}

static ssize_t store_ic_rw(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	unsigned char string[30] = {0};
	int temp[2] = {0};
	u32 ret = 0;

	sscanf(buf, "%s %d %d", string, &temp[0], &temp[1]);

	if (atomic_read(&ts->state.power_state) == POWER_OFF ||
			(strcmp(string, "write") && strcmp(string, "assign")))
		return count;

	reg = temp[0];
	value = temp[1];
	if (!strcmp(string, "write")) {
		u32 write_data = ((0xFF & reg) << 16) | (0xFF & value);
		touch_device_func->ic_ctrl(ts->client,
				IC_CTRL_WRITE, write_data, &ret);
	}

	TOUCH_DEBUG(DEBUG_BASE_INFO,
		"%s - reg[%d] value[%d] return[%d]\n", string, reg, value, ret);

	return count;
}

/* Sysfs - notify
 *
 * shoe_notify : Print states of both TA and Temperature.
 * store_notify : Notify changes of device environment such as TA-connection
 * or temperature to specific_driver.
 */
static ssize_t show_notify(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	ret = sprintf(buf, "TA[%d] Temperature[%d] Proximity[%d]\n"
			"Hall_IC[%d] Keyguard[%d] IME[%d]",
			atomic_read(&ts->state.ta_state),
			atomic_read(&ts->state.temperature_state),
			atomic_read(&ts->state.proximity_state),
			atomic_read(&ts->state.hallic_state),
			atomic_read(&ts->state.keyguard_state),
			atomic_read(&ts->state.ime_state));
	return ret;
}

static ssize_t store_notify(struct i2c_client *client,
	const char *buf, size_t count)
{
	int code = 0, value = 0;

	sscanf(buf, "notify: code[%d] value[%d]", &code, &value);
	update_status(code, value);

	return count;
}

/* Sysfs - firmware_upgrade
 *
 * store_upgrade : upgrade the firmware.
 */
static ssize_t store_upgrade(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	char path[256] = {0};

	sscanf(buf, "%s", path);

	memset(ts->fw_info.fw_path, 0x00, sizeof(ts->fw_info.fw_path));
	memcpy(ts->fw_info.fw_path, path, sizeof(ts->fw_info.fw_path));
	ts->fw_info.fw_force_upgrade = 1;

	queue_delayed_work(touch_wq, &ts->work_upgrade, 0);

	return count;
}

/* Sysfs - firmware_upgrade
 *
 * store_upgrade : upgrade the firmware.
 */
static ssize_t store_rewrite_bin_fw(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	strncpy(ts->fw_info.fw_path, ts->pdata->fw->fw_image, sizeof(ts->fw_info.fw_path));
	ts->fw_info.fw_force_upgrade = 1;

	queue_delayed_work(touch_wq, &ts->work_upgrade, 0);

	return count;
}

/* Sysfs - lpwg_data (Low Power Wake-up Gesture)
 *
 * read : "x1 y1\n x2 y2\n ..."
 * write : 1=SUCCESS, 0=FAIL
 */
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG];
static ssize_t show_lpwg_data(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	int i = 0, ret = 0;

	TOUCH_TRACE();

	memset(lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG);

	mutex_lock(&ts->thread_lock);
	touch_device_func->lpwg(client, LPWG_READ, 0, lpwg_data);
	mutex_unlock(&ts->thread_lock);

	for (i = 0; i < MAX_POINT_SIZE_FOR_LPWG; i++) {
		if (lpwg_data[i].x == -1 && lpwg_data[i].y == -1)
			break;
		ret += sprintf(buf+ret, "%d %d\n",
			lpwg_data[i].x, lpwg_data[i].y);
	}

	return ret;
}

static ssize_t store_lpwg_data(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	int reply = 0;

	TOUCH_TRACE();

	sscanf(buf, "%d", &reply);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "LPWG: data_reply[%d]\n", reply);

	mutex_lock(&ts->thread_lock);
	touch_device_func->lpwg(client, LPWG_REPLY, reply, NULL);
	mutex_unlock(&ts->thread_lock);

	atomic_set(&ts->state.uevent_state, UEVENT_IDLE);
	wake_unlock(&ts->lpwg_wake_lock);

	return count;
}

/* Sysfs - lpwg_notify (Low Power Wake-up Gesture)
 *
 * write
 * 1 : ENABLE/DISABLE
 * 2 : LCD SIZE
 * 3 : ACTIVE AREA
 * 4 : TAP COUNT
 * 5 : TAP DISTANCE
 * 6 : LCD ON/OFF
 * 7 : SENSOR STATUS
 * 8 : Double Tap Check
 */
static ssize_t store_lpwg_notify(struct i2c_client *client,
	const char *buf, size_t count)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	int type = 0;
	int value[4] = {0};

	TOUCH_TRACE();

	sscanf(buf, "%d %d %d %d %d",
		&type, &value[0], &value[1], &value[2], &value[3]);

	TOUCH_DEBUG(DEBUG_BASE_INFO, "LPWG: type[%d] value[%d/%d/%d/%d]\n",
		type, value[0], value[1], value[2], value[3]);

	switch (type) {
	case LPWG_ENABLE:
		lpwg_value[0] = value[0];
		break;
	case LPWG_SET_LCD_AREA:
		break;
	case LPWG_SET_ACTIVE_AREA:
		break;
	case LPWG_TAP_COUNT:
	case LPWG_LENGTH_BETWEEN_TAP:
	case LPWG_DOUBLE_TAP_CHECK:
		touch_device_func->lpwg(ts->client,
				type, value[0], NULL);
		break;
	case LPWG_EARLY_SUSPEND:
		break;
	case LPWG_SENSOR_STATUS:
		lpwg_value[2] = value[0];
		break;
	case LPWG_UPDATE_ALL:
		memcpy(lpwg_value, value, sizeof(lpwg_value));
		break;
	default:
		break;
	}

	mutex_lock(&ts->thread_lock);

	if(atomic_read(&ts->state.power_state) == POWER_SLEEP) {
		touch_device_func->lpwg(ts->client,
				type, (int) &lpwg_value[0], NULL);

		memset(lpwg_value, 0x00, sizeof(lpwg_value));
		TOUCH_DEBUG(DEBUG_BASE_INFO, "LPWG: type[%d] update value\n", type);
	}

	mutex_unlock(&ts->thread_lock);

	return count;
}

/* store_keyguard
 *
 * It should be removed and replaced with 'store_notify' on next OS.
 */
static ssize_t store_keyguard(struct i2c_client *client,
	const char *buf, size_t count)
{
	int value = 0;
	sscanf(buf, "%d", &value);

	update_status(NOTIFY_KEYGUARD, value);
	return count;
}

/* store_ime_status
 *
 * It should be removed and replaced with 'store_notify' on next OS.
 */
static ssize_t store_ime_status(struct i2c_client *client,
	const char *buf, size_t count)
{
	int value = 0;
	sscanf(buf, "%d", &value);

	update_status(NOTIFY_IME, value);
	return count;
}

static ssize_t show_sd_info(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	int raw_status = 0;
	int ch_status = 0;

	interrupt_control(ts, INTERRUPT_DISABLE);

	ret = touch_device_func->sd(client, buf, &raw_status, &ch_status);

	interrupt_control(ts, INTERRUPT_ENABLE);

	ret = sprintf(buf, "========RESULT=======\n");
	ret += sprintf(buf+ret, "Channel Status : %s", (ch_status) ? "Pass\n" : "Fail");
	ret += sprintf(buf+ret, "Raw Data : %s", (raw_status > 0) ? "Pass\n" : "Fail\n");

	return ret;
}

static LGE_TOUCH_ATTR(platform_data,
		S_IRUGO | S_IWUSR, show_platform_data, store_platform_data);
static LGE_TOUCH_ATTR(power_ctrl, S_IRUGO | S_IWUSR, NULL, store_power_ctrl);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, show_ic_rw, store_ic_rw);
static LGE_TOUCH_ATTR(notify, S_IRUGO | S_IWUSR, show_notify, store_notify);
static LGE_TOUCH_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, store_upgrade);
static LGE_TOUCH_ATTR(rewrite_bin_fw, S_IRUGO | S_IWUSR, NULL, store_rewrite_bin_fw);
static LGE_TOUCH_ATTR(lpwg_data,
		S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);
static LGE_TOUCH_ATTR(keyguard, S_IRUGO | S_IWUSR, NULL, store_keyguard);
static LGE_TOUCH_ATTR(ime_status, S_IRUGO | S_IWUSR, NULL, store_ime_status);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd_info, NULL);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_platform_data.attr,
	&lge_touch_attr_power_ctrl.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_notify.attr,
	&lge_touch_attr_fw_upgrade.attr,
	&lge_touch_attr_rewrite_bin_fw.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_lpwg_notify.attr,
	&lge_touch_attr_keyguard.attr,
	&lge_touch_attr_ime_status.attr,
	&lge_touch_attr_sd.attr,
	NULL,
};

static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj,
	struct attribute *attr, char *buf)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
		struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts->client, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj,
	struct attribute *attr,
			      const char *buf, size_t count)
{
	struct lge_touch_data *ts = container_of(lge_touch_kobj,
		struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts->client, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops	= &lge_touch_sysfs_ops,
};

/*
static struct sysdev_class lge_touch_sys_class = {
	.name	= LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id	= 0,
	.cls	= &lge_touch_sys_class,
};
*/

/* get_dts_data
 *
 * make platform data
 */
#define GET_PROPERTY_U8(np, string, target)					\
	do {										\
		u32 tmp_val = 0;							\
		if (of_property_read_u32(np, string, &tmp_val) < 0) 	\
			target = 0;							\
		else									\
			target = (u8) tmp_val;		\
	} while(0)
#define GET_PROPERTY_U32(np, string, target)					\
do {										\
	u32 tmp_val = 0;							\
	if (of_property_read_u32(np, string, &tmp_val) < 0)		\
		target = -1;							\
	else									\
		target = tmp_val;						\
} while(0)

#define GET_PROPERTY_U32_ARRAY(np, string, target, size)			\
do {										\
	struct property *prop = of_find_property(np, string, NULL); \
	if (prop && prop->value && prop->length == size)	{		\
		int i = 0;							\
		const u8 *iprop = prop->value;					\
		for (i = 0; i < prop->length; i++)				\
			target[i] = (u32)iprop[i];				\
	}									\
} while(0)

#define GET_PROPERTY_STRING(np, string, target)					\
do {										\
	const char *tmp_val = np->name;						\
	if (of_property_read_string(np, string, &tmp_val) < 0)	\
		strncpy(target, " ", 1);					\
	else {									\
		int len = strlen(tmp_val);					\
		memcpy(target, tmp_val, len);					\
	}									\
} while(0)

#if defined(CONFIG_ARCH_MSM8916)
static int pinctrl_init(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR(ts->ts_pinctrl)) {
		if (PTR_ERR(ts->ts_pinctrl) == -EPROBE_DEFER) {
			TOUCH_INFO_MSG("ts_pinctrl == -EPROBE_DEFER\n");
			return -EPROBE_DEFER;
		}
		TOUCH_INFO_MSG("Target does not use pinctrl(ts->ts_pinctrl == NULL) \n");
		ts->ts_pinctrl = NULL;
	}

	if (ts->ts_pinctrl) {
		ts->ts_pinset_state_active = pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
		if (IS_ERR(ts->ts_pinset_state_active))
			TOUCH_ERR_MSG("cannot get ts pinctrl active state\n");

		ts->ts_pinset_state_suspend = pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
		if (IS_ERR(ts->ts_pinset_state_suspend))
			TOUCH_ERR_MSG("cannot get ts pinctrl active state\n");

		if (ts->ts_pinset_state_active) {
			DO_SAFE(pinctrl_select_state(ts->ts_pinctrl, ts->ts_pinset_state_active), error);
		} else {
			TOUCH_INFO_MSG("pinctrl active == NULL \n");
		}
	}

	return 0;
error:
	return -1;
}
#endif

static struct touch_platform_data *get_dts_data(struct device *dev)
{
	struct touch_platform_data *p_data;
	struct device_node *np;

	TOUCH_TRACE();

	ASSIGN(np = dev->of_node, error_mem);
	ASSIGN(p_data = devm_kzalloc(dev,
		sizeof(struct touch_platform_data), GFP_KERNEL), error_mem);
	ASSIGN(p_data->caps = devm_kzalloc(dev,
		sizeof(struct touch_device_caps), GFP_KERNEL), error_mem);
	ASSIGN(p_data->role = devm_kzalloc(dev,
		sizeof(struct touch_operation_role), GFP_KERNEL), error_mem);
	ASSIGN(p_data->pwr = devm_kzalloc(dev,
		sizeof(struct touch_power_module), GFP_KERNEL), error_mem);
	ASSIGN(p_data->fw = devm_kzalloc(dev,
		sizeof(struct touch_firmware_module), GFP_KERNEL), error_mem);

	/* GPIO */
	p_data->reset_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
	p_data->int_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, NULL);

	/* CAPS */
	GET_PROPERTY_U32(np, "button_support", p_data->caps->button_support);
	GET_PROPERTY_U32(np, "number_of_button",
		p_data->caps->number_of_button);
	GET_PROPERTY_U32_ARRAY(np, "button_name",
		p_data->caps->button_name, p_data->caps->number_of_button);
	GET_PROPERTY_U32(np, "max_x", p_data->caps->max_x);
	GET_PROPERTY_U32(np, "max_y", p_data->caps->max_y);
	GET_PROPERTY_U32(np, "max_pressure", p_data->caps->max_pressure);
	GET_PROPERTY_U32(np, "max_width", p_data->caps->max_width);
	GET_PROPERTY_U32(np, "max_orientation", p_data->caps->max_orientation);
	GET_PROPERTY_U32(np, "max_id", p_data->caps->max_id);

	/* ROLE */
	GET_PROPERTY_U32(np, "protocol_type", p_data->role->protocol_type);
	GET_PROPERTY_U32(np, "report_mode", p_data->role->report_mode);
	GET_PROPERTY_U32(np, "delta_pos_threshold",
		p_data->role->delta_pos_threshold);
	GET_PROPERTY_U32(np, "booting_delay", p_data->role->booting_delay);
	GET_PROPERTY_U32(np, "reset_delay", p_data->role->reset_delay);
	GET_PROPERTY_U32(np, "wake_up_by_touch",
		p_data->role->wake_up_by_touch);
	GET_PROPERTY_U32(np, "use_sleep_mode", p_data->role->use_sleep_mode);
	GET_PROPERTY_U32(np, "irqflags", p_data->role->irqflags);

	/* POWER */
	GET_PROPERTY_U32(np, "use_regulator", p_data->pwr->use_regulator);
	GET_PROPERTY_STRING(np, "vdd", p_data->pwr->vdd);
	GET_PROPERTY_U32(np, "vdd_voltage", p_data->pwr->vdd_voltage);
	GET_PROPERTY_STRING(np, "vio", p_data->pwr->vio);
	GET_PROPERTY_U32(np, "vio_voltage", p_data->pwr->vio_voltage);

	/* FIRMWARE */
	GET_PROPERTY_STRING(np, "fw_image", p_data->fw->fw_image);
	GET_PROPERTY_U32(np, "need_upgrade", p_data->fw->need_upgrade);

	return p_data;

error_mem:
	return NULL;
}

static struct touch_platform_data *get_board_data(struct device *dev)
{
	struct touch_platform_data *p_data;

	TOUCH_TRACE();

	ASSIGN(p_data = devm_kzalloc(dev,
		sizeof(struct touch_platform_data), GFP_KERNEL), error_mem);
	ASSIGN(p_data->caps = devm_kzalloc(dev,
		sizeof(struct touch_device_caps), GFP_KERNEL), error_mem);
	ASSIGN(p_data->role = devm_kzalloc(dev,
		sizeof(struct touch_operation_role), GFP_KERNEL), error_mem);
	ASSIGN(p_data->pwr = devm_kzalloc(dev,
		sizeof(struct touch_power_module), GFP_KERNEL), error_mem);
	ASSIGN(p_data->fw = devm_kzalloc(dev,
		sizeof(struct touch_firmware_module), GFP_KERNEL), error_mem);

	/* GPIO */
#ifdef CONFIG_MTK_TOUCHPANEL
    p_data->reset_pin 	= GPIO_CTP_RST_PIN;
    p_data->int_pin 	= GPIO_CTP_EINT_PIN;
#endif

	p_data->caps->button_support 	= 0;
    p_data->role->protocol_type 	= MT_PROTOCOL_B;
	p_data->role->wake_up_by_touch	= 0;

	p_data->caps->max_x 			= 720;
	p_data->caps->max_y 			= 1280;
	p_data->caps->max_pressure		= 0xff;
	p_data->caps->max_width			= 15;
	p_data->caps->max_orientation	= 1;
	p_data->caps->max_id			= 10;

	return p_data;

error_mem:
	return NULL;
}

/* check_platform_data
 *
 */
static int check_platform_data(const struct touch_platform_data *p_data)
{
	struct touch_device_caps *caps = p_data->caps;
	struct touch_operation_role *role = p_data->role;
	struct touch_power_module *pwr = p_data->pwr;

	/* caps */
	if (caps->button_support)
		ERROR_IF(caps->number_of_button <= 0
			|| caps->number_of_button > MAX_BUTTON,
			"0 < number_of_button <= MAX_BUTTON[4]\n", error);

	ERROR_IF(caps->max_x <= 0 || caps->max_y <= 0
		|| caps->max_pressure <= 0 || caps->max_width <= 0
		|| caps->max_orientation <= 0 || caps->max_id <= 0,
		"These information should be supported"
		"[id, x, y, pressure, width, orientation]\n", error);

	ERROR_IF(caps->max_id > MAX_FINGER,
		"0 < max_id < MAX_FINGER[10]\n", error);

	/* role */
	ERROR_IF(role->booting_delay <= 0 || role->reset_delay <= 0,
		 "booting_delay and reset_delay should be defined.\n", error);

	/* power */
	if (pwr->use_regulator)
		ERROR_IF(!pwr->vdd || !pwr->vio,
			 "VDD, VIO should be defined"
			 "if use_regulator is true\n", error);
	else
		ERROR_IF(!pwr->power,
			 "power ctrl function should be defined"
			 "if use_regulator is false\n", error);
	return 0;
error:
	return -1;
}

/* sysfs_register
 *
 * get_attribute_array_size
 * : attribute_list should has NULL value at the end of list.
 */

int get_attribute_array_size(struct attribute **list)
{
	int i = 0;

	while (list[i] != NULL && i < MAX_ATTRIBUTE_ARRAY_SIZE)
		i++;

	return i <= MAX_ATTRIBUTE_ARRAY_SIZE ? i : 0;
}

static int sysfs_register(struct lge_touch_data *ts,
	struct attribute **attribute_list)
{
	struct attribute **new_attribute_list;

	int ret = 0;
	int n1 = get_attribute_array_size(lge_touch_attribute_list);
	int n2 = attribute_list ? get_attribute_array_size(attribute_list) : 0;

	TOUCH_DEBUG(DEBUG_BASE_INFO, "n1[%d] n2[%d]\n", n1, n2);

	ASSIGN(new_attribute_list = devm_kzalloc(&ts->client->dev,
		(n1+n2+1) * sizeof(struct attribute *), GFP_KERNEL), err_mem);

	memcpy(new_attribute_list,
		lge_touch_attribute_list, n1 * sizeof(struct attribute *));

	if (attribute_list)
		memcpy(new_attribute_list + n1,
			attribute_list, n2 * sizeof(struct attribute *));

	lge_touch_kobj_type.default_attrs = new_attribute_list;

/*	DO_SAFE(ret = sysdev_class_register(&lge_touch_sys_class),
			err_sysdev_class_register);
	DO_SAFE(ret = sysdev_register(&lge_touch_sys_device),
			err_sysdev_register);*/

	ret = subsys_system_register(&touch_subsys, NULL);
	if (ret < 0)
		TOUCH_ERR_MSG("%s, bus is not registered, ret : %d\n", __func__, ret);

	ret = device_register(&device_touch);
	if (ret < 0)
		TOUCH_ERR_MSG("%s, device is not registered, ret : %d\n", __func__, ret);

	DO_SAFE(ret = kobject_init_and_add(&ts->lge_touch_kobj,
			&lge_touch_kobj_type,
			ts->input_dev->dev.kobj.parent, "%s", LGE_TOUCH_NAME),
			err_kobject_init_and_add);

	return 0;

err_kobject_init_and_add:
	kobject_del(&ts->lge_touch_kobj);
err_mem:
	return ret;
}

static int get_platform_data(struct lge_touch_data *ts)
{
	struct touch_platform_data **p_data = &ts->pdata;
	struct i2c_client *client = ts->client;

	TOUCH_TRACE();

	DO_IF(i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C) == 0, error);

//	DO_SAFE(get_platform_data(&ts->pdata, client), error);

	if (client->dev.of_node) {
		ASSIGN(*p_data = get_dts_data(&client->dev), error);
		DO_SAFE(check_platform_data(*p_data), error);
	}
	else {
		ASSIGN(*p_data = get_board_data(&client->dev), error);
	}

	return NO_ERROR;

error:
	TOUCH_ERR_MSG("fail : get platform data\n");
	return ERROR;
}

static int set_gpio(struct lge_touch_data *ts)
{
	TOUCH_TRACE();

#if defined(CONFIG_ARCH_MSM8916)
	DO_SAFE(pinctrl_init(ts->client), error);
#endif

	if (ts->pdata->reset_pin > 0) {
		DO_SAFE(ts_gpio_init(ts->pdata->reset_pin,
			"touch_reset", GPIO_RST_PIN), error);
		ts_gpio_direction_output(ts->pdata->reset_pin, 1);
	}

	if (ts->pdata->int_pin > 0) {
		DO_SAFE(ts_gpio_init(ts->pdata->int_pin,
			"touch_int", GPIO_INT_PIN), error);
		ts_gpio_direction_input(ts->pdata->int_pin);
	}

	return NO_ERROR;

error:
	TOUCH_ERR_MSG("fail : set gpio pin\n");
	return ERROR;
}

static int register_input_dev(struct lge_touch_data *ts)
{
	int btn_num = 0;

	TOUCH_TRACE();

	ASSIGN(ts->input_dev = input_allocate_device(),
		err_input_allocate_device);
	ts->input_dev->name = "touch_dev";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev,
		ABS_MT_POSITION_X, 0, ts->pdata->caps->max_x, 0, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_POSITION_Y, 0, ts->pdata->caps->max_y, 0, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_PRESSURE, 0, ts->pdata->caps->max_pressure, 0, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_WIDTH_MAJOR, 0, ts->pdata->caps->max_width, 0, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_WIDTH_MINOR, 0, ts->pdata->caps->max_width, 0, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_ORIENTATION, 0, ts->pdata->caps->max_orientation, 0, 0);

	if (ts->pdata->role->protocol_type == MT_PROTOCOL_A)
		input_set_abs_params(ts->input_dev,
			ABS_MT_TRACKING_ID, 0, ts->pdata->caps->max_id, 0, 0);
	else
#ifdef KERNEL_ABOVE_3_4_67
		DO_SAFE(input_mt_init_slots(ts->input_dev,
					ts->pdata->caps->max_id, 0), err_mt_init_slots);
#else
		DO_SAFE(input_mt_init_slots(ts->input_dev,
			ts->pdata->caps->max_id), err_mt_init_slots);
#endif

	if (ts->pdata->caps->button_support) {
		set_bit(EV_KEY, ts->input_dev->evbit);
		for (btn_num = 0; btn_num < ts->pdata->caps->number_of_button; btn_num++)
			set_bit(ts->pdata->caps->button_name[btn_num],
				ts->input_dev->keybit);
	}

	DO_SAFE(input_register_device(ts->input_dev), err_input_register_device);

	input_set_drvdata(ts->input_dev, ts);

	return NO_ERROR;

err_mt_init_slots:
	if (ts->pdata->role->protocol_type == MT_PROTOCOL_B)
		input_mt_destroy_slots(ts->input_dev);
err_input_register_device:
	input_unregister_device(ts->input_dev);
err_input_allocate_device:
	input_free_device(ts->input_dev);

	TOUCH_ERR_MSG("fail : register input device\n");
	return ERROR;
}

static void init_sys_sync_func(struct lge_touch_data *ts)
{
	TOUCH_TRACE();

	mutex_init(&ts->thread_lock);

	INIT_DELAYED_WORK(&ts->work_init, touch_init_func);
	INIT_DELAYED_WORK(&ts->work_upgrade, firmware_upgrade_func);
	INIT_DELAYED_WORK(&ts->work_notify, touch_notify_func);

	wake_lock_init(&ts->lpwg_wake_lock, WAKE_LOCK_SUSPEND, "touch_lpwg");
}

static int touch_suspend(struct device *dev)
{
	struct lge_touch_data *ts =  dev_get_drvdata(dev);

	TOUCH_TRACE();

	cancel_delayed_work_sync(&ts->work_init);
	cancel_delayed_work_sync(&ts->work_upgrade);
	cancel_delayed_work_sync(&ts->work_notify);

	atomic_set(&ts->state.uevent_state, UEVENT_IDLE);
	touch_device_func->suspend(ts->client);
	power_control(ts, ts->pdata->role->use_sleep_mode
		? POWER_SLEEP : POWER_OFF);

	return 0;
}

static int touch_resume(struct device *dev)
{
	struct lge_touch_data *ts =  dev_get_drvdata(dev);

	TOUCH_TRACE();

	power_control(ts, ts->pdata->role->use_sleep_mode
		? POWER_WAKE : POWER_ON);
	touch_device_func->resume(ts->client);

	queue_delayed_work(touch_wq, &ts->work_init,
		msecs_to_jiffies(ts->pdata->role->booting_delay));

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct lge_touch_data *ts =
		container_of(h, struct lge_touch_data, early_suspend);

	touch_suspend(&ts->client->dev);
}

static void touch_late_resume(struct early_suspend *h)
{
	struct lge_touch_data *ts =
		container_of(h, struct lge_touch_data, early_suspend);

	touch_resume(&ts->client->dev);
}

#elif defined(CONFIG_FB)
static int touch_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct fb_event *evdata = (struct fb_event *)data;
	struct lge_touch_data *ts =
		container_of(self, struct lge_touch_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)evdata->data;

		if (*blank == FB_BLANK_UNBLANK) {
			mutex_lock(&ts->thread_lock);
			touch_device_func->lpwg(ts->client,
				LPWG_DISABLE, (int) &lpwg_value[0], NULL);
			mutex_unlock(&ts->thread_lock);

			touch_resume(&ts->client->dev);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			mutex_lock(&ts->thread_lock);
			touch_device_func->lpwg(ts->client,
				LPWG_UPDATE_ALL, (int) &lpwg_value[0], NULL);
			mutex_unlock(&ts->thread_lock);

			touch_suspend(&ts->client->dev);
		}
	}

	return 0;
}
#endif

static int touch_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct lge_touch_data *ts;
	struct attribute **specific_attribute_list;

	TOUCH_TRACE();

	ASSIGN(ts = devm_kzalloc(&client->dev,
		sizeof(struct lge_touch_data), GFP_KERNEL), error);

	ASSIGN(ts->client = client_only_for_other_module = client, error);
	i2c_set_clientdata(client, ts);

	DO_SAFE(get_platform_data(ts), error);
	DO_SAFE(set_gpio(ts), error);

	init_sys_sync_func(ts);

	memset(&ts->state, 0, sizeof(struct state_info));
	DO_IF(touch_device_func->probe(ts->client, ts->pdata,
		&ts->state, &specific_attribute_list) != 0, error);

	DO_SAFE(power_control(ts, POWER_ON), error);
	msleep(ts->pdata->role->booting_delay);

	DO_SAFE(register_input_dev(ts), error);

	DO_SAFE(ts_register_irq(ts,
		(void*)touch_irq_handler,
		(void*)touch_thread_irq_handler),
		err_request_threaded_irq);

	thread_irq = &ts->work_irq;

	queue_delayed_work(touch_wq, &ts->work_init, 0);
	queue_delayed_work(touch_wq, &ts->work_upgrade, 0);

	DO_SAFE(sysfs_register(ts, specific_attribute_list),
			err_request_threaded_irq);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = touch_early_suspend;
		ts->early_suspend.resume = touch_late_resume;
		register_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB)
		ts->fb_notif.notifier_call = touch_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
#endif

	return NO_ERROR;

err_request_threaded_irq:
	free_irq(ts->client->irq, ts);
error:
	TOUCH_ERR_MSG("probe failed\n");
	return ERROR;
}

static int touch_remove(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	TOUCH_TRACE();

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

	kobject_del(&ts->lge_touch_kobj);
//	sysdev_unregister(&lge_touch_sys_device);
//	sysdev_class_unregister(&lge_touch_sys_class);

	wake_lock_destroy(&ts->lpwg_wake_lock);

	interrupt_control(ts, INTERRUPT_DISABLE);
	free_irq(ts->client->irq, ts);

	if (ts->pdata->role->protocol_type == MT_PROTOCOL_B)
		input_mt_destroy_slots(ts->input_dev);

	touch_device_func->remove(ts->client);
	power_control(ts, POWER_OFF);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	devm_kfree(&ts->client->dev, ts);

	return 0;
}

static int touch_pm_suspend(struct device *dev)
{
//	struct lge_touch_data *ts = dev_get_drvdata(dev);

	TOUCH_TRACE();
//	atomic_set(&ts->state.pm_state, PM_SUSPEND);

	return 0;
}

static int touch_pm_resume(struct device *dev)
{
//	struct lge_touch_data *ts = dev_get_drvdata(dev);

	TOUCH_TRACE();

/*	if (atomic_read(&ts->state.pm_state) == PM_SUSPEND_IRQ) {
		struct irq_desc *desc = irq_to_desc(ts->client->irq);
		irq_set_pending(ts->client->irq);
		check_irq_resend(desc, ts->client->irq);
		TOUCH_DEBUG(DEBUG_BASE_INFO, "Resend an interrupt\n");
	}

	atomic_set(&ts->state.pm_state, PM_RESUME);
*/
	return 0;
}

static struct dev_pm_ops touch_pm_ops = {
	.suspend = touch_pm_suspend,
	.resume = touch_pm_resume,
};

static struct i2c_device_id lge_ts_id[] = {
	{LGE_TOUCH_NAME, 0},
};

static struct i2c_driver lge_touch_driver = {
	.probe   	= touch_probe,
	.remove	 	= touch_remove,
	.id_table 	= lge_ts_id,
	.driver	 	= {
		.name   = LGE_TOUCH_NAME,
		.owner	= THIS_MODULE,
		.pm		= &touch_pm_ops,
	},
};

int touch_driver_register(struct touch_device_driver *driver,
	struct of_device_id *match_table)
{
	int ret = 0;
	TOUCH_TRACE();

	touch_device_func = driver;
	ASSIGN(touch_wq = create_singlethread_workqueue("touch_wq"),
		err_create_workqueue);

	lge_touch_driver.driver.of_match_table = match_table;
	DO_SAFE(ret = i2c_add_driver(&lge_touch_driver), err_i2c_add_driver);

	return 0;

err_i2c_add_driver:
	destroy_workqueue(touch_wq);
err_create_workqueue:
	return ret;
}

void touch_driver_unregister(void)
{
	TOUCH_TRACE();

	i2c_del_driver(&lge_touch_driver);
	touch_device_func = NULL;

	if (touch_wq)
		destroy_workqueue(touch_wq);
}
