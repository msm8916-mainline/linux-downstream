/*
 * Copyright (c) 2011,2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define FUNCTION_DATA f11_data
#define FNUM 11

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/rmi.h>
#include "rmi_driver.h"
#include <linux/input/mt.h>
#include <linux/workqueue.h>

//#if defined(BBK_DCLICK_WAKE)
#include <linux/time.h>
#include <linux/kthread.h>
//#endif

#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_touchscreen_common.h>

#define RESUME_REZERO (1 && defined(CONFIG_PM))
#if RESUME_REZERO
#include <linux/delay.h>
#define DEFAULT_REZERO_WAIT_MS	40
#endif

#ifndef MT_TOOL_MAX
#define MT_TOOL_MAX MT_TOOL_PEN
#endif

//wangyong add for switch fast and energy relaxtion interface
//#define SWITCH_FAST_ENERGY_RALAX  
struct rmi_function_container *rmi_get_function_container(
					struct rmi_driver_data *ddata, int function_number);
int rmi_f11_get_sensor_electrodes(struct rmi_function_container *fc,
									int *rx_num, int *tx_num);
																						
extern int hardware_id_check(void);

/* wangyuanliang add temply for disable relax*/
extern struct rmi_function_container *rmi_get_function_container(
							struct rmi_driver_data *ddata, int function_number);
int rmi_f11_get_sensor_electrodes(struct rmi_function_container *fc,
											int *rx_num, int *tx_num);

#define F11_MAX_NUM_OF_SENSORS		8
#define F11_MAX_NUM_OF_FINGERS		10
#define F11_MAX_NUM_OF_TOUCH_SHAPES	16

#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127

#define FINGER_STATE_MASK	0x03
#define GET_FINGER_STATE(f_states, i) \
	((f_states[i / 4] >> (2 * (i % 4))) & FINGER_STATE_MASK)

#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8

#define F11_CEIL(x, y) (((x) + ((y)-1)) / (y))
#define INBOX(x, y, box) (x >= box.x && x < (box.x + box.width) \
			&& y >= box.y && y < (box.y + box.height))

#define DEFAULT_XY_MAX 9999
#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 2255 //qiuguifu modify. 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 1
#define DEFAULT_MAX_ABS_MT_TRACKING_ID 10
#define MAX_NAME_LENGTH 256

//#if defined(BBK_DCLICK_WAKE)
#define DCLICK_DATA_OFFSET 54
#define F51_BASE_ADDR 0x0400
//#define DCLICK_DATA_OFFSET 67
//#endif

#if 0
#define RX_NUM_REG_OFFSET 0x34
#define TX_NUM_REG_OFFSET 0x35
#else
#define RX_NUM_REG_OFFSET 0x27
#define TX_NUM_REG_OFFSET 0x28
#endif

static ssize_t f11_flip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_flip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_clip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t f11_test_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf);

static ssize_t f11_clip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_offset_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t f11_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count);

static ssize_t f11_swap_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t f11_rezero_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

#if RESUME_REZERO
static ssize_t f11_rezeroOnResume_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_rezeroOnResume_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);
static ssize_t f11_rezeroWait_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_rezeroWait_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);
#endif

extern struct rmi_function_container *rmi_get_function_container(
					struct rmi_driver_data *ddata, int function_number);
static void rmi_f11_free_memory(struct rmi_function_container *fc);

static int rmi_f11_initialize(struct rmi_function_container *fc);

static int rmi_f11_create_sysfs(struct rmi_function_container *fc);

static int rmi_f11_config(struct rmi_function_container *fc);

static int rmi_f11_reset(struct rmi_function_container *fc);

static int rmi_f11_register_devices(struct rmi_function_container *fc);

static void rmi_f11_free_devices(struct rmi_function_container *fc);
extern int rmi_dev_get_fn54_data(struct rmi_function_container *fc, char **dest_data,
					unsigned char function_number);

//#if defined(BBK_DCLICK_WAKE)
static void rmi_f11_dclick_timer_func(unsigned long data);

static void rmi_f11_dclick_timer_work_func(struct work_struct *work);
//#endif

#if defined(BBK_TS_TRACE_BASELINE)
static ssize_t f11_baseline_trace_info_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf);
#endif

static struct device_attribute attrs[] = {
	__ATTR(flip, RMI_RW_ATTR, f11_flip_show, f11_flip_store),
	__ATTR(clip, RMI_RW_ATTR, f11_clip_show, f11_clip_store),
	__ATTR(test, RMI_RO_ATTR, f11_test_show, NULL),
	__ATTR(offset, RMI_RW_ATTR, f11_offset_show, f11_offset_store),
	__ATTR(swap, RMI_RW_ATTR, f11_swap_show, f11_swap_store),
	__ATTR(relreport, RMI_RW_ATTR, f11_relreport_show, f11_relreport_store),
	__ATTR(maxPos, RMI_RO_ATTR, f11_maxPos_show, rmi_store_error),
#if RESUME_REZERO
	__ATTR(rezeroOnResume, RMI_RW_ATTR, f11_rezeroOnResume_show,
		f11_rezeroOnResume_store),
	__ATTR(rezeroWait, RMI_RW_ATTR, f11_rezeroWait_show,
		f11_rezeroWait_store),
#endif
	__ATTR(rezero, RMI_WO_ATTR, rmi_show_error, f11_rezero_store),
#if defined(BBK_TS_TRACE_BASELINE)
	__ATTR(baseline_trace_info, RMI_RO_ATTR, f11_baseline_trace_info_show, NULL),
#endif
};


union f11_2d_commands {
	struct {
		u8 rezero:1;
	};
	u8 reg;
};

struct f11_2d_device_query {
	union {
		struct {
			u8 nbr_of_sensors:3;
			u8 has_query9:1;
			u8 has_query11:1;
		};
		u8 f11_2d_query0;
	};

	union {
		struct {
			u8 has_z_tuning:1;
			u8 has_pos_interpolation_tuning:1;
			u8 has_w_tuning:1;
			u8 has_pitch_info:1;
			u8 has_default_finger_width:1;
			u8 has_segmentation_aggressiveness:1;
			u8 has_tx_rw_clip:1;
			u8 has_drumming_correction:1;
		};
		u8 f11_2d_query11;
	};
};

union f11_2d_query9 {
	struct {
		u8 has_pen:1;
		u8 has_proximity:1;
		u8 has_palm_det_sensitivity:1;
		u8 has_suppress_on_palm_detect:1;
		u8 has_two_pen_thresholds:1;
		u8 has_contact_geometry:1;
	};
	u8 reg;
};

struct f11_2d_sensor_query {
	union {
		struct {
			/* query1 */
			u8 number_of_fingers:3;
			u8 has_rel:1;
			u8 has_abs:1;
			u8 has_gestures:1;
			u8 has_sensitivity_adjust:1;
			u8 configurable:1;
			/* query2 */
			u8 num_of_x_electrodes:7;
			/* query3 */
			u8 num_of_y_electrodes:7;
			/* query4 */
			u8 max_electrodes:7;
		};
		u8 f11_2d_query1__4[4];
	};

	union {
		struct {
			u8 abs_data_size:2;
			u8 has_anchored_finger:1;
			u8 has_adj_hyst:1;
			u8 has_dribble:1;
			u8 has_bending_correct:1;
			u8 has_large_obj_supress:1;
			u8 has_jitter_filter:1;
		};
		u8 f11_2d_query5;
	};

	u8 f11_2d_query6;

	union {
		struct {
			u8 has_single_tap:1;
			u8 has_tap_n_hold:1;
			u8 has_double_tap:1;
			u8 has_early_tap:1;
			u8 has_flick:1;
			u8 has_press:1;
			u8 has_pinch:1;
			u8 padding:1;

			u8 has_palm_det:1;
			u8 has_rotate:1;
			u8 has_touch_shapes:1;
			u8 has_scroll_zones:1;
			u8 has_individual_scroll_zones:1;
			u8 has_multi_finger_scroll:1;
		};
		u8 f11_2d_query7__8[2];
	};

	union f11_2d_query9 query9;

	union {
		struct {
			u8 nbr_touch_shapes:5;
		};
		u8 f11_2d_query10;
	};
};
union f11_2d_ctrl0_9 {
	struct {
		/* F11_2D_Ctrl0 */
		u8 reporting_mode:3;
		u8 abs_pos_filt:1;
		u8 rel_pos_filt:1;
		u8 rel_ballistics:1;
		u8 dribble:1;
		u8 report_beyond_clip:1;
		/* F11_2D_Ctrl1 */
		u8 palm_detect_thres:4;
		u8 motion_sensitivity:2;
		u8 man_track_en:1;
		u8 man_tracked_finger:1;
		/* F11_2D_Ctrl2 and 3 */
		u8 delta_x_threshold:8;
		u8 delta_y_threshold:8;
		/* F11_2D_Ctrl4 and 5 */
		u8 velocity:8;
		u8 acceleration:8;
		/* F11_2D_Ctrl6 thru 9 */
		u16 sensor_max_x_pos:12;
		u8 ctrl7_reserved:4;
		u16 sensor_max_y_pos:12;
		u8 ctrl9_reserved:4;
	};
	struct {
		u8 regs[10];
		u16 address;
	};
};
union f11_2d_ctrl10 {
	struct {
		u8 single_tap_int_enable:1;
		u8 tap_n_hold_int_enable:1;
		u8 double_tap_int_enable:1;
		u8 early_tap_int_enable:1;
		u8 flick_int_enable:1;
		u8 press_int_enable:1;
		u8 pinch_int_enable:1;
	};
	u8 reg;
};

union f11_2d_ctrl11 {
	struct {
		u8 palm_detect_int_enable:1;
		u8 rotate_int_enable:1;
		u8 touch_shape_int_enable:1;
		u8 scroll_zone_int_enable:1;
		u8 multi_finger_scroll_int_enable:1;
	};
	u8 reg;
};

union f11_2d_ctrl12 {
	struct {
		u8 sensor_map:7;
		u8 xy_sel:1;
	};
	u8 reg;
};

union f11_2d_ctrl14 {
	struct {
		u8 sens_adjustment:5;
		u8 hyst_adjustment:3;
	};
	u8 reg;
};

union f11_2d_ctrl15 {
	struct {
		u8 max_tap_time:8;
	};
	u8 reg;
};

union f11_2d_ctrl16 {
	struct {
		u8 min_press_time:8;
	};
	u8 reg;
};

union f11_2d_ctrl17 {
	struct {
		u8 max_tap_distance:8;
	};
	u8 reg;
};

union f11_2d_ctrl18_19 {
	struct {
		u8 min_flick_distance:8;
		u8 min_flick_speed:8;
	};
	u8 reg[2];
};

union f11_2d_ctrl20_21 {
	struct {
		u8 pen_detect_enable:1;
		u8 pen_jitter_filter_enable:1;
		u8 ctrl20_reserved:6;
		u8 pen_z_threshold:8;
	};
	u8 reg[2];
};

/* These are not accessible through sysfs yet. */
union f11_2d_ctrl22_26 {
	struct {
		/* control 22 */
		u8 proximity_detect_int_en:1;
		u8 proximity_jitter_filter_en:1;
		u8 f11_2d_ctrl6_b3__7:6;

		/* control 23 */
		u8 proximity_detection_z_threshold;

		/* control 24 */
		u8 proximity_delta_x_threshold;

		/* control 25 */
		u8 proximity_delta_y_threshold;

		/* control 26 */
		u8 proximity_delta_z_threshold;
	};
	u8 regs[5];
};

union f11_2d_ctrl27 {
	struct {
		/* control 27 */ //haspalmdetectsensitivity or has suppressonpalmdetect
		u8 palm_detecy_sensitivity:4;
		u8 suppress_on_palm_detect:1;
		u8 f11_2d_ctrl27_b5__7:3;
	};
	u8 regs[1];
};

union f11_2d_ctrl28 {
	struct {
		/* control 28 */ //has_multifingerscroll
		u8 multi_finger_scroll_mode:2;
		u8 edge_motion_en:1;
		u8 f11_2d_ctrl28b_3:1;
		u8 multi_finger_scroll_momentum:4;
	};
	u8 regs[1];
};

union f11_2d_ctrl29_30 {
	struct {
		/* control 29 */ //hasztuning
		u8 z_touch_threshold;

		/* control 30 */ //hasztuning
		u8 z_touch_hysteresis;
	};
	struct {
		u8 regs[2];
		u16 address;
	};
};


struct  f11_2d_ctrl {
	union f11_2d_ctrl0_9 *ctrl0_9;
	union f11_2d_ctrl10		*ctrl10;
	union f11_2d_ctrl11		*ctrl11;
	union f11_2d_ctrl12		*ctrl12;
	u8				ctrl12_size;
	union f11_2d_ctrl14		*ctrl14;
	union f11_2d_ctrl15		*ctrl15;
	union f11_2d_ctrl16		*ctrl16;
	union f11_2d_ctrl17		*ctrl17;
	union f11_2d_ctrl18_19		*ctrl18_19;
	union f11_2d_ctrl20_21		*ctrl20_21;
	union f11_2d_ctrl22_26 *ctrl22_26;
	union f11_2d_ctrl27 *ctrl27;
	union f11_2d_ctrl28 *ctrl28;
	union f11_2d_ctrl29_30 *ctrl29_30;
};

struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_y:4;
	u8 w_x:4;
	u8 z;
};

struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
};

struct f11_2d_data_8 {
	u8 single_tap:1;
	u8 tap_and_hold:1;
	u8 double_tap:1;
	u8 early_tap:1;
	u8 flick:1;
	u8 press:1;
	u8 pinch:1;
};

struct f11_2d_data_9 {
	u8 palm_detect:1;
	u8 rotate:1;
	u8 shape:1;
	u8 scrollzone:1;
	u8 finger_count:3;
};

struct f11_2d_data_10 {
	u8 pinch_motion;
};

struct f11_2d_data_10_12 {
	u8 x_flick_dist;
	u8 y_flick_dist;
	u8 flick_time;
};

struct f11_2d_data_11_12 {
	u8 motion;
	u8 finger_separation;
};

struct f11_2d_data_13 {
	u8 shape_n;
};

struct f11_2d_data_14_15 {
	u8 horizontal;
	u8 vertical;
};

struct f11_2d_data_14_17 {
	u8 x_low;
	u8 y_right;
	u8 x_upper;
	u8 y_left;
};

struct f11_2d_data_28 {
	u8 bending:1;
	u8 large_supress:1;
	u8 reserved:6;
};
	

struct f11_2d_data {
	u8				*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
	const struct f11_2d_data_28	*extended_status;
};

//#if defined(BBK_DCLICK_WAKE)
#define TRIP_X_AREA	50
#define TRIP_Y_AREA	50
#define DCLICK_TWO_FINGER_AREA_X 100
#define DCLICK_TWO_FINGER_AREA_Y 100
//#endif

#if defined(BBK_TS_TRACE_BASELINE)
#define TRACE_X_LENTH_LIMIT	30 /* 1.5 mm */
#define TRACE_Y_LENTH_LIMIT 30
#define TRACE_POINT_TIMES	50
#define MAX_BASELINE_DELTA	80
#define ABNORMAL_CHANNEL_COUNT	1  /* 0D buuton only one channel */
#define ORIGINAL_BASE_SHOULD_JUDGE_TIME 15
#define MAX_ABNORMAL_COUNT	5  /* if one point baseline delta check abnormal these times
								   we think the original baseline may be illegal
								   at this point */
#define FORCE_CAL                 2

struct finger_point {
	int x;
	int y;
	int count;
};
#endif

struct f11_2d_sensor {
	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_query sens_query;
	struct f11_2d_data data;
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 finger_tracker[F11_MAX_NUM_OF_FINGERS];
	u8 *data_pkt;
	int pkt_size;
	u8 sensor_index;
	struct rmi_f11_virtualbutton_map virtualbutton_map;
	char input_name[MAX_NAME_LENGTH];
	char input_phys[MAX_NAME_LENGTH];
	struct input_dev *input;
	struct input_dev *mouse_input;

	/* make dclick parmeters in sensor struct */
//#if defined(BBK_DCLICK_WAKE)
	int first_press_x;
	int first_press_y;
	int second_press_x;
	int second_press_y;
	int pressed_count;
	int release_count;
	bool is_dclick_valide;
	bool has_dclick_timer_start;
	struct timer_list dclick_timer;
	struct work_struct dclick_timer_work;
	int dclick_dimension_x_min;
	int dclick_dimension_x_max;
	int dclick_dimension_y_min;
	int dclick_dimension_y_max;
//#endif
	struct timer_list one_finger_timer;
    struct work_struct finger_timer_work;  //wangyong 
    int timer_flag;
	int down_count;
	#if defined(SWITCH_FAST_ENERGY_RALAX)
	int realse_3count;
	u8 fast_relax;
	u16 f54_control_base_addr;
	u16 f54_command_base_addr;
	#endif
#if defined(BBK_TS_TRACE_BASELINE)
	bool has_orig_baseline_init;
	bool orig_baseline_legal;
	bool need_force_cal;
	bool need_check_baseline;
	bool need_get_org_baseline;
	bool need_check_org_baseline;
	int orig_baseline_judge_times; /* use to count how many times has check original's legal*/
	int rx_line;
	int tx_line;
	int channel_count;
	int *baseline_original;
	int *baseline_check;
	int *baseline_abnormal_count; /* use to check the original baseline legal,but now we
									 have touch key on sensor, so if only one channel
									 abnormal we should think there may be a rement point
									 so this array is no use now but do't delete for futher
									*/
	//int max_count;
	struct rmi_function_container *fc54;
	struct finger_point finger_point_tracer[F11_MAX_NUM_OF_FINGERS];
	struct work_struct ts_trace_work;
	struct workqueue_struct *ts_trace_workqueue;
#endif
	//wangyong
	int home_state;
};

struct f11_data {
	struct f11_2d_device_query dev_query;
	struct f11_2d_ctrl dev_controls;
	struct mutex dev_controls_mutex;
	struct mutex input_slot_mutex;//Add by qiuguifu for input slot Concurrency issue.
#if	RESUME_REZERO
	u16 rezero_wait_ms;
	bool rezero_on_resume;
#endif
//#if defined(BBK_DCLICK_WAKE)
	bool whether_in_dclick_mode;
	bool has_tp_suspend;
//#endif
	struct f11_2d_sensor sensors[F11_MAX_NUM_OF_SENSORS];
};

struct f11_data * globle_f11_data;

#if defined(BBK_TS_TRACE_BASELINE)
/* for 0D key*/
struct f11_data *global_f11_data;
#endif 

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

static void (*rmi_f11_abs_pos_report)(struct f11_2d_sensor *sensor,
					u8 finger_state, u8 n_finger, struct rmi_driver_data *ddata);

/* ctrl sysfs files */
show_store_union_struct_prototype(abs_pos_filt)
show_store_union_struct_prototype(z_touch_threshold)
show_store_union_struct_prototype(z_touch_hysteresis)

/* This is a group in case we add the other ctrls. */
static struct attribute *attrs_ctrl0[] = {
	attrify(abs_pos_filt),
	NULL
};
static struct attribute_group attrs_control0 = GROUP(attrs_ctrl0);

static struct attribute *attrs_ctrl29_30[] = {
	attrify(z_touch_threshold),
	attrify(z_touch_hysteresis),
	NULL
};
static struct attribute_group attrs_control29_30 = GROUP(attrs_ctrl29_30);

/** F11_INACCURATE state is overloaded to indicate pen present. */
#define F11_PEN F11_INACCURATE

static int get_tool_type(struct f11_2d_sensor *sensor, u8 finger_state) {
	if (sensor->sens_query.query9.has_pen && finger_state == F11_PEN)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}
	if (axis_align->flip_x)
		x = min(F11_REL_POS_MAX, -x);
	if (axis_align->flip_y)
		y = min(F11_REL_POS_MAX, -y);

	if (x || y) {
		input_report_rel(sensor->input, REL_X, x);
		input_report_rel(sensor->input, REL_Y, y);
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
	}
	input_sync(sensor->mouse_input);
}

static void rmi_f11_release_point(struct f11_2d_sensor *sensor) 
{
	int i;
	if( sensor == NULL)
	{
		VIVO_TS_LOG_ERR("[%s]: sensor is NULL\n",__func__);
		return ;
	}
	if(globle_f11_data == NULL )
	{
		VIVO_TS_LOG_ERR("[%s]: globle_f11_data is NULL\n",__func__);
		return ;
	}
	mutex_lock(&globle_f11_data->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.
	for (i = 0; i < sensor->nbr_fingers; i++) {
		input_mt_slot(sensor->input, i);
		input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER, 0);
	}
	input_sync(sensor->input);
	mutex_unlock(&globle_f11_data->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.
}

#if defined(BBK_TS_TRACE_BASELINE)


static void rmi_f11_force_cal(struct f11_2d_sensor *sensor)
{
	int result;
	u8 command;
	struct rmi_driver_data *ddata = dev_get_drvdata(sensor->input->dev.parent);

	command = (unsigned char)FORCE_CAL;

	if (!sensor->fc54) {
		sensor->fc54 = rmi_get_function_container(ddata, 0x54);
		if (!sensor->fc54) {
			VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n" __func__);
			return;
		}
	}

	result = rmi_write_block(sensor->fc54->rmi_dev, sensor->fc54->fd.command_base_addr,
						&command, 1);
	if (result < 0) {
		VIVO_TS_LOG_ERR("[%s]: Could not write command to 0x%x\n",
				__func__, sensor->fc54->fd.command_base_addr);
	}
}

static void rmi_f11_reset_trace_data(struct f11_2d_sensor *sensor)
{
	int i;

	sensor->has_orig_baseline_init = false;
	sensor->need_get_org_baseline = false;
	sensor->orig_baseline_legal = false;
	sensor->need_force_cal = false;
	sensor->need_check_baseline = false;
	sensor->orig_baseline_judge_times = 0;

	for (i = 0; i < sensor->channel_count; i++) {
		sensor->baseline_abnormal_count[i] = 0;
	}
}

static int rmi_f11_get_baseline(struct f11_2d_sensor *sensor, int *data)
{
	char *tmp_data;
	int i;
	int rc = 0;
	int limit = 2 * sensor->channel_count;
	struct rmi_driver_data *ddata = dev_get_drvdata(sensor->input->dev.parent);

	if (!sensor->fc54) {
		sensor->fc54 = rmi_get_function_container(ddata, 0x54);
		if (!sensor->fc54) {
			VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
			return -EINVAL;
		}
	}
	
	rc = rmi_dev_get_fn54_data(sensor->fc54, &tmp_data, 9);
	if (rc < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return rc;
	}

	for (i = 0; i < limit; i += 2) {
		data[i / 2] = (int)((u16)(tmp_data[i + 1]) << 8 | tmp_data[i]);
	}

	return 0;
	//sensor->has_orig_baseline_init = true;
}

static void rmi_f11_get_orig_baseline(struct f11_2d_sensor *sensor)
{
	char *tmp_data;
	int i;
	int rc = 0;
	int limit = 2 * sensor->channel_count;
	struct rmi_driver_data *ddata = dev_get_drvdata(sensor->input->dev.parent);

	if (rmi_f11_get_baseline(sensor, sensor->baseline_original)) {
		VIVO_TS_LOG_ERR("[%s]:get original baseline failed\n", __func__);
		return 0;
	}

	sensor->has_orig_baseline_init = true;
}
static int rmi_f11_makesure_orig_baseline_legal(struct f11_2d_sensor *sensor)
{
	int i;
	int count = 0;
	int delta;

	if (rmi_f11_get_baseline(sensor, sensor->baseline_check)) {
		VIVO_TS_LOG_ERR("[%s]:get baseline failed\n", __func__);
		return 0;
	}

	for (i = 0; i < sensor->channel_count; i++) {
		delta = sensor->baseline_check[i] - sensor->baseline_original[i];

		/* if one channel abnormal times larger than MAX_ABNORMAL_COUNT
		   the channel's original baseline maybe catched when finger press
		*/
		if (delta > MAX_BASELINE_DELTA || delta < -MAX_BASELINE_DELTA) {
			sensor->baseline_abnormal_count[i]++;
			VIVO_TS_LOG_ERR("[%s]:The channel (%d, %d) baseline check diff\n", __func__, i / sensor->rx_line, 
					i % sensor->rx_line);
			/* need to be correct
			   for if we check baseline many times whether should we 
			   use a max cuont or a percentage
			*/

			/* only one channel abnormal will force calibrate the Sensor
			   no need to check one point abnormal times
			*/
			if (sensor->baseline_abnormal_count[i] > MAX_ABNORMAL_COUNT) {
				VIVO_TS_LOG_ERR("[%s]:The channel (%d, %d) original baseline may be catched when touch\n", __func__,
					i / sensor->rx_line, i % sensor->rx_line);
				/* only assignment one channel but not all is based on 
				   the baseline won't change a lot in use it's a tracer
				   no need to be very exact
				*/
				sensor->baseline_original[i] = sensor->baseline_check[i];
				sensor->baseline_abnormal_count[i] = 0;
			}		
		}
	}

	sensor->orig_baseline_judge_times++;

	if (count > (sensor->channel_count * 3 / 4)) {
		VIVO_TS_LOG_ERR("[%s]:The abnormal channel more than 3/4 of channel count so recatch orignal baseline\n", __func__);
		memcpy(sensor->baseline_original, sensor->baseline_check,
				sensor->channel_count * sizeof(int));
		sensor->orig_baseline_judge_times = 0;
		sensor->orig_baseline_legal = false;
		for (i = 0; i < sensor->channel_count; i++)
			sensor->baseline_abnormal_count[i] = 0;
		count = 0;
	}

	if (sensor->orig_baseline_judge_times >= ORIGINAL_BASE_SHOULD_JUDGE_TIME) {
		/* has judge ORIGINAL_BASE_SHOULD_JUDGE_TIME times*/
		sensor->orig_baseline_legal = true;
	}

	return count;
}

static int rmi_f11_get_abnormal_baseline_channel_count(struct f11_2d_sensor *sensor)
{
	int i;
	int count = 0;
	int delta;

	if (rmi_f11_get_baseline(sensor, sensor->baseline_check)) {
		VIVO_TS_LOG_ERR("[%s]:get baseline failed\n", __func__);
		return 0;
	}

	for (i = 0; i < sensor->channel_count; i++) {
		delta = sensor->baseline_check[i] - sensor->baseline_original[i];

		/* if one channel abnormal times larger than MAX_ABNORMAL_COUNT
		   the channel's original baseline maybe catched when finger press
		*/
		if (delta > MAX_BASELINE_DELTA || delta < -MAX_BASELINE_DELTA) {
			count++;
			VIVO_TS_LOG_ERR("[%s]:The channel (%d, %d) baseline check diff\n", __func__, i / sensor->rx_line, 
					i % sensor->rx_line);
		}
	}

	if (count > (sensor->channel_count * 3 / 4)) {
		VIVO_TS_LOG_ERR("[%s]:The abnormal channel more than 3/4 of channel count so recatch orignal baseline\n", __func__);
		memcpy(sensor->baseline_original, sensor->baseline_check,
				sensor->channel_count * sizeof(int));
		sensor->orig_baseline_judge_times = 0;
		sensor->orig_baseline_legal = false;
		for (i = 0; i < sensor->channel_count; i++)
			sensor->baseline_abnormal_count[i] = 0;
		count = 0;
	}

	return count;

}

/* return 1 if need force calibrate else return 0 */
static int rmi_f11_check_baseline_delta(struct f11_2d_sensor *sensor)
{
	int count;

	count = rmi_f11_get_abnormal_baseline_channel_count(sensor);

	if (count > ABNORMAL_CHANNEL_COUNT) {
		VIVO_TS_LOG_ERR("[%s]:There may be a remanent point need to force calibrate when all fingers release\n", __func__);
		/* clibrate immediately and at next release event calibrate again*/
		rmi_f11_force_cal(sensor);
		rmi_f11_reset_trace_data(sensor);
		sensor->need_force_cal = true;
		return 1;
	}

	return 0;

}

/* global function for 0d key */
int rmi_f11_check_baseline(void)
{
	return rmi_f11_check_baseline_delta(&global_f11_data->sensors[0]);
}

static void rmi_f11_release_finger_trace(struct f11_2d_sensor *sensor, u8 n_finger)
{
	sensor->finger_point_tracer[n_finger].x = 0;
	sensor->finger_point_tracer[n_finger].y = 0;
	sensor->finger_point_tracer[n_finger].count = 0;
}

/* return 1 if need reset else return 0 */
static int rmi_f11_trace_finger(struct f11_2d_sensor *sensor, u8 n_finger, 
								int x_point, int y_point)
{
	int delta_x = sensor->finger_point_tracer[n_finger].x - x_point;
	int delta_y = sensor->finger_point_tracer[n_finger].y - y_point;

	if ((delta_x < TRACE_X_LENTH_LIMIT && delta_x > -TRACE_X_LENTH_LIMIT)
			&& (delta_y < TRACE_Y_LENTH_LIMIT && delta_y > -TRACE_Y_LENTH_LIMIT)) {
		sensor->finger_point_tracer[n_finger].count++;
		if (sensor->finger_point_tracer[n_finger].count > TRACE_POINT_TIMES) {
			VIVO_TS_LOG_ERR("[%s]:The point (%d, %d) has been pressed more than %d times\n", __func__,
					x_point, y_point, TRACE_POINT_TIMES);
			sensor->need_check_baseline = true;
#if 0
			if (rmi_f11_check_baseline_delta(sensor) == 1) {
				/* return 1 means need force calibrate */
				return 1;
			}
#endif
			/* has check the baseline whether legal
			   reset the count ignore whether  reset the TP or not
			*/
			sensor->finger_point_tracer[n_finger].count = 0;
		}
	}else{
		sensor->finger_point_tracer[n_finger].x = x_point;
		sensor->finger_point_tracer[n_finger].y = y_point;
		sensor->finger_point_tracer[n_finger].count = 0;
		//rmi_f11_release_finger_trace(sensor, n_finger);
	}

	return 0;

}

static void rmi_f11_trace_work_func(struct work_struct *work)
{
	struct f11_2d_sensor *sensor = container_of(work, struct f11_2d_sensor, ts_trace_work);

	if (!sensor->has_orig_baseline_init) {
		rmi_f11_get_orig_baseline(sensor);
	}else if(sensor->need_check_org_baseline){
		(void)rmi_f11_makesure_orig_baseline_legal(sensor);
		sensor->need_check_org_baseline = false;
	}else{
		/* may loss one check chance in the else of need_check_org_baseline??
		   rmi_f11_check_baseline_delta in multiple press
		   but rmi_f11_makesure_orig_baseline_legal in release
		   so only multiple press happen and then release happen
		   will loss the chance to do rmi_f11_check_baseline_delta
		   but if there is release event no need to rmi_f11_check_baseline_delta
		*/
		(void)rmi_f11_check_baseline_delta(sensor);
	}
}
#endif


static void rmi_f11_abs_pos_report_normal(struct f11_2d_sensor *sensor,
					u8 finger_state, u8 n_finger, struct rmi_driver_data *ddata)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	int prev_state = sensor->finger_tracker[n_finger];
	int x, y, z;
	int w_x, w_y, w_max, w_min, orient;
	int temp;

#if 0
	if (prev_state && !finger_state) {
		/* this is a release */
		x = y = z = w_max = w_min = orient = 0;
	} else if (!prev_state && !finger_state) {
#endif

	if (!prev_state && !finger_state) {
		/* nothing to report */
		return;
	}
	else
	{
		x = ((data->abs_pos[n_finger].x_msb << 4) |
			data->abs_pos[n_finger].x_lsb);
		y = ((data->abs_pos[n_finger].y_msb << 4) |
			data->abs_pos[n_finger].y_lsb);
		z = data->abs_pos[n_finger].z;
		w_x = data->abs_pos[n_finger].w_x;
		w_y = data->abs_pos[n_finger].w_y;
		w_max = max(w_x, w_y);
		w_min = min(w_x, w_y);

		if (axis_align->swap_axes) {
			temp = x;
			x = y;
			y = temp;
			temp = w_x;
			w_x = w_y;
			w_y = temp;
		}

		orient = w_x > w_y ? 1 : 0;

		if (axis_align->flip_x)
			x = max(sensor->max_x - x, 0);

		if (axis_align->flip_y)
			y = max(sensor->max_y - y, 0);

		/*
		** here checking if X offset or y offset are specified is
		**  redundant.  We just add the offsets or, clip the values
		**
		** note: offsets need to be done before clipping occurs,
		** or we could get funny values that are outside
		** clipping boundaries.
		*/
		x += axis_align->offset_X;
		y += axis_align->offset_Y;
		x =  max(axis_align->clip_X_low, x);
		y =  max(axis_align->clip_Y_low, y);
		if (axis_align->clip_X_high)
			x = min(axis_align->clip_X_high, x);
		if (axis_align->clip_Y_high)
			y =  min(axis_align->clip_Y_high, y);

		//liuyunfeng add for delete X[0~44 1070~1120] Y[2000~2126]  
		#if 0 
		//defined(PD1309L) || defined(PD1309LG4)
		if(prev_state && ((x<44 && y>2000) || (x>1070 && y>2000)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		else if(!prev_state && ((x<44 && y>2000) || (x>1070 && y>2000)))
		{
			sensor->finger_tracker[n_finger] = 0;
			
		 	return;
		}
		//defined(PD1403L)||defined(PD1403LG4)||defined(PD1403F)||defined(PD1403V)
		if(prev_state && ((x<66 && y>2220) || (x>1110 && y>2220)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		
		 	return;
		}
		else if(!prev_state && ((x<66 && y>2220) || (x>1110&& y>2220)))
		{
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		//defined(PD1227L) || defined(PD1227LG4) || defined(PD1227V) || defined(TD1404)|| defined(TD1401) 
		if(prev_state && ((x<56 && y>2333) || (x>1210 && y>2333)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		else if(!prev_state && ((x<56 && y>2333) || (x>1210 && y>2333)))
		{
			sensor->finger_tracker[n_finger] = 0;
			
		 	return;
		}
		//defined(PD1410F) || defined(PD1410V) || defined(PD1410F_EX) 
		if(prev_state && ((x<91 && y>2220) || (x>1175 && y>2220)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;

		 	return;
		}
		else if(!prev_state && ((x<91 && y>2220) || (x>1175 && y>2220)))
		{

			sensor->finger_tracker[n_finger] = 0;
			
		 	return;
		}
		// defined (PD1302) || defined (PD1302F) || defined (PD1302F_EX)
		//wangyong add for delete X[0~70 1010~1080] Y[1920~2062]  
		if(prev_state && ((x<70 && y>1920) || (x>1010 && y>1920)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		
		 	return;
		}
		else if(!prev_state && ((x<70 && y>1920) || (x>1010 && y>1920)))
		{
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		//defined(PD1402)||defined(PD1402LG4)
		if(prev_state && ((x<66 && y>2220) || (x>1110 && y>2220)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		
		 	return;
		}
		else if(!prev_state && ((x<66 && y>2220) || (x>1110&& y>2220)))
		{
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		#else
		//wangyong add for delete X[0~115 1350~1456] Y[2560~2700]  
		if(prev_state && ((x<44 && y>2010) || (x>1064 && y>2010)))
		{
			input_mt_slot(sensor->input, n_finger);
			input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,0);
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		else if(!prev_state && ((x<44 && y>2010) || (x>1064 && y>2010)))
		{
			sensor->finger_tracker[n_finger] = 0;
		 	return;
		}
		#endif
		x = (x*(ddata->lcd_dimension_x))/(ddata->ts_dimension_x);
		y = (y*(ddata->lcd_dimension_y))/(ddata->ts_dimension_y);
	}
#ifndef CONFIG_RMI4_F11_PEN
	/* Some UIs ignore W of zero, so we fudge it to 1 for pens. */
	if (sensor->sens_query.query9.has_pen &&
			get_tool_type(sensor, finger_state) == MT_TOOL_PEN) {
		w_max = max(1, w_max);
		w_min = max(1, w_min);
	}
#endif

#if defined(BBK_TS_TRACE_BASELINE)
	if (finger_state != 0) {
		if (sensor->has_orig_baseline_init) {
			if (rmi_f11_trace_finger(sensor, n_finger, x, y) == 1) {
				VIVO_TS_LOG_INF("[%s]: there may be a rement point\n", __func__);
				return;
			}
		}
	}else{
		rmi_f11_release_finger_trace(sensor, n_finger);
	}
#endif

	input_mt_slot(sensor->input, n_finger);
	input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER,
							finger_state != 0);

	if (finger_state != 0) {

#ifdef ABS_MT_PRESSURE
		input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
#endif

		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, w_max);
		//input_report_abs(sensor->input, ABS_MT_WIDTH_MAJOR, 1);
		//input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, w_min);
		//input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
		//input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);
#ifdef	CONFIG_RMI4_F11_PEN
		if (sensor->sens_query.query9.has_pen) {
			input_report_abs(sensor->input, ABS_MT_TOOL_TYPE,
					get_tool_type(sensor, finger_state));
		}
#endif
	}

	/* MT sync between fingers */
	//input_mt_sync(sensor->input);
	sensor->finger_tracker[n_finger] = finger_state;

}

#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
static int rmi_f11_virtual_button_handler(struct f11_2d_sensor *sensor)
{
	int i;
	int x;
	int y;
	struct rmi_f11_virtualbutton_map *virtualbutton_map;
	if (sensor->sens_query.has_gestures &&
		sensor->data.gest_1->single_tap) {
		virtualbutton_map = &sensor->virtualbutton_map;
		x = ((sensor->data.abs_pos[0].x_msb << 4) |
			sensor->data.abs_pos[0].x_lsb);
		y = ((sensor->data.abs_pos[0].y_msb << 4) |
			sensor->data.abs_pos[0].y_lsb);
		for (i = 0; i < virtualbutton_map->buttons; i++) {
			if (INBOX(x, y, virtualbutton_map->map[i])) {
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 1);
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 0);
				input_sync(sensor->input);
				return 0;
			}
		}
	}
	return 0;
}
#else
#define rmi_f11_virtual_button_handler(sensor)
#endif

//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
static void rmi_f11_handle_large_obj_supression(struct f11_2d_sensor *sensor)
{
		input_report_key(sensor->input, KEY_TS_LARGE_SUPPRESSION, 1);
		msleep(10);
		input_report_key(sensor->input, KEY_TS_LARGE_SUPPRESSION, 0);
		input_sync(sensor->input);
}


static void rmi_f11_calling_3finger_handler(struct f11_2d_sensor *sensor, struct rmi_driver_data *ddata)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = GET_FINGER_STATE(f_state, i);

		if (finger_state == F11_RESERVED) {
			VIVO_TS_LOG_ERR("[%s]: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
				(finger_state == F11_INACCURATE)) {
			finger_pressed_count++;
		}
	}	

	if(finger_pressed_count >=3 )
	{
		VIVO_TS_LOG_DBG("[%s]: press finger more than 3 !!!=========== \n",__func__);
		rmi_f11_handle_large_obj_supression(sensor);
		ddata->is_calling = 0;
	}

}
//#endif


//wangyong add 

static void finger_timer_work_func(struct work_struct *work)
{
	struct f11_2d_sensor *sensor = container_of(work, struct f11_2d_sensor,finger_timer_work);

	if(sensor->timer_flag == 1)
	{
		rmi_f11_release_point(sensor);
	}

		
}
static void finger_timer_func(unsigned long data)
{
	struct f11_2d_sensor *sensor = (struct f11_2d_sensor *)data;

	schedule_work(&sensor->finger_timer_work);
}

//#if defined(BBK_GLOVES_MODE)
static int finger_down_release_deal(struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;


	//wangyong add

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = GET_FINGER_STATE(f_state, i);

		if (finger_state == F11_RESERVED) {
			VIVO_TS_LOG_ERR("[%s]: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
				(finger_state == F11_INACCURATE)) {
			finger_pressed_count++;
			if(finger_pressed_count>1)
				return 0;
		}
	}	
		if(finger_pressed_count ==1)
		{

		
				sensor->down_count++;
				if(sensor->timer_flag==1)
				{
					sensor->timer_flag++;
				}
		}

		else if(finger_pressed_count ==0)
		{
				if(sensor->timer_flag==1)
				{
					sensor->timer_flag++;
				}
					
	
				if(sensor->down_count>=1 && sensor->down_count<=3)
				{
	
					mod_timer(&sensor->one_finger_timer,jiffies + msecs_to_jiffies(30));
					sensor->timer_flag = 1;
					return 1;
				}
				else
				{
			
					sensor->down_count = 0;
					sensor->timer_flag = 0;
					del_timer_sync(&sensor->one_finger_timer);
				}
						
		}
		else
		{	

			sensor->down_count = 0;
			sensor->timer_flag = 0;
			del_timer_sync(&sensor->one_finger_timer);
		}
		//end
		return 0;
}
//#endif
//end

static void rmi_f11_finger_handler(struct f11_2d_sensor *sensor, struct rmi_driver_data *ddata)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;
	struct rmi_function_container *fc11;
	struct f11_data *f11;

	if(!ddata)
	{
		VIVO_TS_LOG_ERR("[%s]: driver data is NULL\n", __func__);
		return;		
	}
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11)
	{
	 	VIVO_TS_LOG_ERR("[%s]:Get fc11 failed===\n",__func__);
		return;
	}
	f11 = fc11->data;	
	if (!f11)
	{
	 	VIVO_TS_LOG_ERR("[%s]:Get f11 failed===\n",__func__);
		return;
	}
//#if defined(BBK_GLOVES_MODE)
    if(vivo_touchscreen_is_support(FS_GLOVES_MODE)) {
		if(1 == ddata->ts_gloves_mode ) {
			if(finger_down_release_deal(sensor))
				return;
			//wangyong add
		}
	}
//#endif
	mutex_lock(&f11->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.
	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = GET_FINGER_STATE(f_state, i);

		if (finger_state == F11_RESERVED) {
			VIVO_TS_LOG_ERR("[%s]: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
				(finger_state == F11_INACCURATE)) {
			finger_pressed_count++;
		}

		if (sensor->data.abs_pos)
			rmi_f11_abs_pos_report(sensor, finger_state, i, ddata);

		if (sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}
	//input_report_key(sensor->input, BTN_TOUCH, finger_pressed_count);
	input_sync(sensor->input);
	mutex_unlock(&f11->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.
	
	if(finger_pressed_count >=3 )
	{
		ddata->largetouch_flag = 1;
	}
	else
	{
		
		ddata->largetouch_flag = 0;
		ddata->mute_switch = 1; 
	}	
}

void rmi_f11_finger_realse(struct rmi_driver_data *ddata)
{
	int i,j;
	struct rmi_function_container *fc11;
	struct f11_data *f11;

	if(!ddata)
	{
		VIVO_TS_LOG_ERR("[%s]: driver data is NULL\n", __func__);
		return;		
	}
	
	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11)
	{
	 	VIVO_TS_LOG_ERR("[%s]:Get fc11 failed===\n",__func__);
		return;
	}
	else
	{
		f11 = fc11->data;	
		if(f11 == NULL )
		{
			VIVO_TS_LOG_ERR("[%s]: globle_f11_data is NULL\n",__func__);
			return ;
		}
		mutex_lock(&f11->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.
		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++)
		{
			for (j = 0; j < f11->sensors[i].nbr_fingers; j++)
			{
				input_mt_slot(f11->sensors[i].input, j);
				input_mt_report_slot_state(f11->sensors[i].input, MT_TOOL_FINGER, false);
			}
					
			input_sync(f11->sensors[i].input);
		}
		mutex_unlock(&f11->input_slot_mutex);
		ddata->AA_area_point_pressed = 0;
		ddata->AA_area_point_release_time = jiffies;
	}

}

static int f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_query *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->number_of_fingers == 5 ? 10 :
				query->number_of_fingers + 1);

	sensor->pkt_size = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->f11_2d_query7__8[0])
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1])
		sensor->pkt_size += sizeof(u8);

	if (query->has_pinch || query->has_flick || query->has_rotate) {
		sensor->pkt_size += 3;
		if (!query->has_flick)
			sensor->pkt_size--;
		if (!query->has_rotate)
			sensor->pkt_size--;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size += F11_CEIL(query->nbr_touch_shapes + 1, 8);

	if (query->has_large_obj_supress)
		sensor->pkt_size += sizeof(u8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = sensor->data_pkt;
	i = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);
	}

	if (query->has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (query->f11_2d_query7__8[0]) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1]) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					(data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes) {
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];
		i += 1;
	}

//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
    if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
		if (query->has_large_obj_supress) {
			data->extended_status = (struct f11_2d_data_28 *)&sensor->data_pkt[i];
		}
	}
//#endif

	return 0;
}

static void f11_free_control_regs(struct f11_2d_ctrl *ctrl)
{
	kfree(ctrl->ctrl10);
	kfree(ctrl->ctrl11);
	kfree(ctrl->ctrl14);
	kfree(ctrl->ctrl15);
	kfree(ctrl->ctrl16);
	kfree(ctrl->ctrl17);
	kfree(ctrl->ctrl18_19);
	kfree(ctrl->ctrl20_21);
	kfree(ctrl->ctrl22_26);
	kfree(ctrl->ctrl27);
	kfree(ctrl->ctrl28);
	kfree(ctrl->ctrl29_30);
	ctrl->ctrl10 = NULL;
	ctrl->ctrl11 = NULL;
	ctrl->ctrl14 = NULL;
	ctrl->ctrl15 = NULL;
	ctrl->ctrl16 = NULL;
	ctrl->ctrl17 = NULL;
	ctrl->ctrl18_19 = NULL;
	ctrl->ctrl20_21 = NULL;
	ctrl->ctrl22_26 = NULL;
	ctrl->ctrl27 = NULL;
	ctrl->ctrl28 = NULL;
	ctrl->ctrl29_30 = NULL;
}

static int f11_read_control_regs(struct rmi_device *rmi_dev,
					   struct f11_2d_ctrl *ctrl,
					   u16 ctrl_base_addr) {
	u16 read_address = ctrl_base_addr;
	int error = 0;

	ctrl->ctrl0_9->address = read_address;
	error = rmi_read_block(rmi_dev, read_address, (u8 *) ctrl->ctrl0_9->regs,
		sizeof(ctrl->ctrl0_9->regs));
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl0, code: %d.\n", __func__, error);
		return error;
	}
	read_address = read_address + sizeof(ctrl->ctrl0_9->regs);

	if (ctrl->ctrl10) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl10->reg, sizeof(union f11_2d_ctrl10));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl10, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl10);
	}

	if (ctrl->ctrl11) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl11->reg, sizeof(union f11_2d_ctrl11));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl11, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl11);
	}

	if (ctrl->ctrl14) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl14->reg, sizeof(union f11_2d_ctrl14));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl14, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl14);
	}

	if (ctrl->ctrl15) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl15->reg, sizeof(union f11_2d_ctrl15));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl15, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl15);
	}

	if (ctrl->ctrl16) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl16->reg, sizeof(union f11_2d_ctrl16));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl16, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl16);
	}

	if (ctrl->ctrl17) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl17->reg, sizeof(union f11_2d_ctrl17));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl17, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl17);
	}

	if (ctrl->ctrl18_19) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl18_19->reg, sizeof(union f11_2d_ctrl18_19));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl18_19, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl20_21->reg, sizeof(union f11_2d_ctrl20_21));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl20_21, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl22_26->regs, sizeof(union f11_2d_ctrl22_26));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl22_26, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl27->regs, sizeof(union f11_2d_ctrl27));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl27, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl28->regs, sizeof(union f11_2d_ctrl28));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl28, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl28);
	}

	if (ctrl->ctrl29_30) {
		ctrl->ctrl29_30->address = read_address;
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl29_30->regs, sizeof(ctrl->ctrl29_30->regs));
		if (error < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to read F11 ctrl29_30, code: %d.\n", __func__,
				error);
			return error;
		}
		read_address = read_address + sizeof(ctrl->ctrl29_30->regs);
	}
	return 0;
}

static int f11_allocate_control_regs(struct rmi_device *rmi_dev,
				struct f11_2d_device_query *device_query,
				struct f11_2d_sensor_query *sensor_query,
				struct f11_2d_ctrl *ctrl,
				u16 ctrl_base_addr) {
	// ctrl = kzalloc(sizeof(struct f11_2d_ctrl),GFP_KERNEL);
	int error = 0;
	ctrl->ctrl0_9 = kzalloc(sizeof(union f11_2d_ctrl0_9),
				       GFP_KERNEL);
	if (!ctrl->ctrl0_9) {
		error = -ENOMEM;
		goto error_exit;
	}
	if (sensor_query->f11_2d_query7__8[0]) {
		ctrl->ctrl10 = kzalloc(sizeof(union f11_2d_ctrl10),
				       GFP_KERNEL);
		if (!ctrl->ctrl10) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (sensor_query->f11_2d_query7__8[1]) {
		ctrl->ctrl11 = kzalloc(sizeof(union f11_2d_ctrl11),
				       GFP_KERNEL);
		if (!ctrl->ctrl11) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (device_query->has_query9 && sensor_query->query9.has_pen) {
		ctrl->ctrl20_21 = kzalloc(sizeof(union f11_2d_ctrl20_21),
					  GFP_KERNEL);
		if (!ctrl->ctrl20_21) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (device_query->has_query9 && sensor_query->query9.has_proximity) {
		ctrl->ctrl22_26 = kzalloc(sizeof(union f11_2d_ctrl22_26),
					  GFP_KERNEL);
		if (!ctrl->ctrl22_26) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (device_query->has_query9 && (sensor_query->query9.has_palm_det_sensitivity || sensor_query->query9.has_suppress_on_palm_detect)) {
		ctrl->ctrl27 = kzalloc(sizeof(union f11_2d_ctrl27),
					  GFP_KERNEL);
		if (!ctrl->ctrl27) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (sensor_query->has_multi_finger_scroll) {
		ctrl->ctrl28 = kzalloc(sizeof(union f11_2d_ctrl28),
					  GFP_KERNEL);
		if (!ctrl->ctrl28) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	if (device_query->has_query11 && device_query->has_z_tuning) {
		ctrl->ctrl29_30 = kzalloc(sizeof(union f11_2d_ctrl29_30),
					  GFP_KERNEL);
		if (!ctrl->ctrl29_30) {
			error = -ENOMEM;
			goto error_exit;
		}
	}

	return f11_read_control_regs(rmi_dev, ctrl, ctrl_base_addr);

error_exit:
	f11_free_control_regs(ctrl);
	return error;
}

static int f11_write_control_regs(struct rmi_device *rmi_dev,
					struct f11_2d_sensor_query *query,
					struct f11_2d_ctrl *ctrl,
					u16 ctrl_base_addr)
{
	u16 write_address = ctrl_base_addr;
	int error;

	error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl0_9->regs,
				 sizeof(ctrl->ctrl0_9->regs));
	if (error < 0)
		return error;
	write_address += sizeof(ctrl->ctrl0_9);

	if (ctrl->ctrl10) {
		error = rmi_write_block(rmi_dev, write_address,
					&ctrl->ctrl10->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl11) {
		error = rmi_write_block(rmi_dev, write_address,
					&ctrl->ctrl11->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl12 && ctrl->ctrl12_size && query->configurable) {
		if (ctrl->ctrl12_size > query->max_electrodes) {
			VIVO_TS_LOG_ERR("[%s]: invalid cfg size:%d, should be < %d.\n",
				__func__, ctrl->ctrl12_size,
				query->max_electrodes);
			return -EINVAL;
		}
		error = rmi_write_block(rmi_dev, write_address,
						&ctrl->ctrl12->reg,
						ctrl->ctrl12_size);
		if (error < 0)
			return error;
		write_address += ctrl->ctrl12_size;
	}

	if (ctrl->ctrl14) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl14->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl15) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl15->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl16) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl16->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl17) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl17->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl18_19) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl18_19->reg, sizeof(union f11_2d_ctrl18_19));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl20_21->reg,
					sizeof(union f11_2d_ctrl20_21));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl22_26->regs,
					sizeof(union f11_2d_ctrl22_26));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl27->regs,
					sizeof(union f11_2d_ctrl27));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl28->regs,
					sizeof(union f11_2d_ctrl28));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl28);
	}

	if (ctrl->ctrl29_30) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl29_30->regs,
					sizeof(union f11_2d_ctrl29_30));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl29_30);
	}

	return 0;
}

static int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_2d_sensor_query *query, u16 query_base_addr)
{
	int query_size;
	int rc;

	rc = rmi_read_block(rmi_dev, query_base_addr, query->f11_2d_query1__4,
					sizeof(query->f11_2d_query1__4));
	if (rc < 0)
		return rc;
	query_size = rc;

	if (query->has_abs) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query5);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_rel) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query6);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_gestures) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query->f11_2d_query7__8,
					sizeof(query->f11_2d_query7__8));
		if (rc < 0)
			return rc;
		query_size += sizeof(query->f11_2d_query7__8);
	}

	if (query->has_touch_shapes) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query10);
		if (rc < 0)
			return rc;
		query_size++;
	}

	return query_size;
}

static const char  *virtual_key_str =NULL;
static void s3202_set_virtual_key_string(const char *vkey)
{
     virtual_key_str = vkey;
}
static ssize_t s3202_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
    #if 1
	    int rc = -EINVAL;
	    if(virtual_key_str != NULL)
		{
		    VIVO_TS_LOG_INF("[%s]: VKEY= %s \n",__func__,virtual_key_str);
		    rc = sprintf(buf,"%s\n",virtual_key_str);  
		}
		else
		{
		    VIVO_TS_LOG_ERR("[%s]: VKEY support failed or please check dts !!\n",__func__);
		}
		
		return rc;
	#else
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":117:1345:130:100"
	//":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":228:1345:90:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":360:1345:140:100"
	//":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":491:1345:90:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":603:1345:130:100"
	"\n");
	#endif

}

static struct kobj_attribute s3202_vkeys_attr = {
	.attr = {
		.name = "virtualkeys.sensor00fn11",
		.mode = S_IRUGO,
	},
	.show = &s3202_vkeys_show,
};


static struct attribute *s3202_properties_attrs[] = {
	&s3202_vkeys_attr.attr,
	NULL
};
static struct attribute_group s3202_properties_attr_group = {
	.attrs = s3202_properties_attrs,
};
static struct kobject *properties_kobj;
/* This operation is done in a number of places, so we have a handy routine
 * for it.
 */
static void f11_set_abs_params(struct rmi_function_container *fc, int index)
{
	struct f11_data *instance_data =  fc->data;
	struct f11_2d_sensor *sensor = &instance_data->sensors[index];
	struct input_dev *input = sensor->input;
	struct rmi_driver_data *ddata = rmi_get_driverdata(fc->rmi_dev);
	
	
	//char product_id[RMI_PRODUCT_ID_LENGTH];
	//int rc = 0;
	int err = 0;
	int ogs_flag = 0;
	int device_x_max =
		instance_data->dev_controls.ctrl0_9->sensor_max_x_pos;
	int device_y_max =
		instance_data->dev_controls.ctrl0_9->sensor_max_y_pos;
	int x_min, x_max, y_min, y_max;
	//chenyunzhe add -------------------
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	if (sensor->axis_align.swap_axes) {
		int temp = device_x_max;
		device_x_max = device_y_max;
		device_y_max = temp;
	}

	/* Use the max X and max Y read from the device, or the clip values,
	 * whichever is stricter.
	 */
	x_min = sensor->axis_align.clip_X_low;
	if (sensor->axis_align.clip_X_high)
		x_max = min((int) device_x_max,
			sensor->axis_align.clip_X_high);
	else
		x_max = device_x_max;

	y_min = sensor->axis_align.clip_Y_low;
	if (sensor->axis_align.clip_Y_high)
		y_max = min((int) device_y_max,
			sensor->axis_align.clip_Y_high);
	else
		y_max = device_y_max;

	input_mt_init_slots(input, DEFAULT_MAX_ABS_MT_TRACKING_ID,0);
	

#ifdef ABS_MT_PRESSURE
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
#endif

	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
#if 0
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			DEFAULT_MAX_ABS_MT_TRACKING_ID, 0, 0);
#endif
	rmi_f11_abs_pos_report = rmi_f11_abs_pos_report_normal;
     
//	rc = rmi_read_block(fc->rmi_dev, 0x00b5, product_id,RMI_PRODUCT_ID_LENGTH);
	//if (strncmp(product_id, "BIE-OGS", 7) == 0 ||strncmp(product_id, "TRY-OGS", 7) == 0 ||strncmp(product_id, "TRY-GFF", 7) == 0)
	
			
	{
		ogs_flag = 1;
		properties_kobj = kobject_create_and_add("board_properties",NULL);
		if (properties_kobj)
			err = sysfs_create_group(properties_kobj,&s3202_properties_attr_group);
		if (!properties_kobj ||err)
		{
			VIVO_TS_LOG_DBG("[%s]: failed to create board_properties\n",__func__);
			//goto exit_check_functionality_failed;
		}
		//wangyong add
#if 0
	if (sensor->boot_mode == META_BOOT || sensor->boot_mode == RECOVERY_BOOT   ||sensor->boot_mode == FACTORY_BOOT
			|| sensor->boot_mode == ADVMETA_BOOT   || sensor->boot_mode == ATE_FACTORY_BOOT  )
	{
		input_set_capability(input, EV_KEY, 172);
		//input_set_capability(input, EV_KEY, KEY_MENU);
		//input_set_capability(input, EV_KEY, KEY_BACK);
		rmi_f11_abs_pos_report = rmi_f11_abs_pos_report_recovery;
	}
#endif
}
    
	/* TODO get max_x_pos (and y) from control registers. */
	if (ogs_flag == 1) {
		ddata->is_button_independent = false;
		//input_set_abs_params(input, ABS_MT_POSITION_X,
		//		0, 1266, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X,
				0, pdata->lcd_dimension_x, 0, 0);
		//input_set_abs_params(input, ABS_MT_POSITION_Y,
		//		0, 2220, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y,
				0, pdata->lcd_dimension_y, 0, 0);
	//#if defined(BBK_DCLICK_WAKE)
        if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			//sensor->dclick_dimension_x_min = 160;
			//sensor->dclick_dimension_x_max = 1106;
			//sensor->dclick_dimension_y_min = 160;
			//sensor->dclick_dimension_y_max = 2060;
			
			sensor->dclick_dimension_x_min = pdata->dclick_dimension_min_x;
			sensor->dclick_dimension_x_max = pdata->dclick_dimension_max_x;
			sensor->dclick_dimension_y_min = pdata->dclick_dimension_min_y;
			sensor->dclick_dimension_y_max = pdata->dclick_dimension_max_y;
		}
	//#endif

	}else{
		input_set_abs_params(input, ABS_MT_POSITION_X,
				x_min, x_max, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y,
				y_min, y_max, 0, 0);
	//#if defined(BBK_DCLICK_WAKE)
		if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			//sensor->dclick_dimension_x_min = 140;
			//sensor->dclick_dimension_x_max = x_max - 140;
			//sensor->dclick_dimension_y_min = 140;
			//sensor->dclick_dimension_y_max = y_max - 140;
			sensor->dclick_dimension_x_min = pdata->dclick_dimension_min_x;
			sensor->dclick_dimension_x_max = pdata->dclick_dimension_max_x;
			sensor->dclick_dimension_y_min = pdata->dclick_dimension_min_y;
			sensor->dclick_dimension_y_max = pdata->dclick_dimension_max_y;
		}
	//#endif
	}
#ifdef	CONFIG_RMI4_F11_PEN
	if (sensor->sens_query.query9.has_pen)
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_MAX, 0, 0);
#endif
}

#if defined(BBK_TS_TRACE_BASELINE)
static int rmi_f11_trace_baseline_init(struct rmi_function_container *fc, int index)
{
	struct f11_data *instance_data =  fc->data;
	struct f11_2d_sensor *sensor = &instance_data->sensors[index];
	struct rmi_driver_data *ddata = rmi_get_driverdata(fc->rmi_dev);
	char ts_workqueu_name[10];
	int rc = 0;

	sensor->has_orig_baseline_init = false;
	sensor->need_get_org_baseline = false;
	sensor->orig_baseline_legal = false;
	sensor->orig_baseline_judge_times = 0;
	sensor->need_force_cal = false;
	sensor->need_check_baseline = false;
	//sensor->max_count = (int)(((unsigned long)(1 << (sizeof(int) * 8))) - 1);

#if 0
	sensor->fc54 = rmi_get_function_container(ddata, 0x54);
	if (!sensor->fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return -EINVAL;
	}
#endif
	sprintf(ts_workqueu_name, "ts_trace%d", index);
	sensor->ts_trace_workqueue = create_singlethread_workqueue(ts_workqueu_name);
	if (!sensor->ts_trace_workqueue) {
		VIVO_TS_LOG_ERR("[%s]:Creat workqueue failed\n", __func__);
		goto err_creat_workqueue;
	}
	INIT_WORK(&sensor->ts_trace_work, rmi_f11_trace_work_func);

	rc = rmi_f11_get_sensor_electrodes(fc, &sensor->rx_line, &sensor->tx_line);
	if (rc < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get f11 sensor electrodes failed\n", __func__);
		goto err_get_elecs;
	}

	if (ddata->is_button_independent) 
		sensor->rx_line++;

	sensor->channel_count = sensor->rx_line * sensor->tx_line;

	sensor->baseline_original = kzalloc(sensor->channel_count * sizeof(int), 
											GFP_KERNEL);
	if (!sensor->baseline_original) {
		VIVO_TS_LOG_ERR("[%s]:Alloc baseline_original failed\n", __func__);
		goto err_alloc_bl_orig;
	}

	sensor->baseline_check = kzalloc(sensor->channel_count * sizeof(int), 
											GFP_KERNEL);
	if (!sensor->baseline_check) {
		VIVO_TS_LOG_ERR("[%s]:Alloc baseline_check failed\n", __func__);
		goto err_alloc_bl_check;
	}

	sensor->baseline_abnormal_count = kzalloc(sensor->channel_count * sizeof(int), 
											GFP_KERNEL);
	if (!sensor->baseline_abnormal_count) {
		VIVO_TS_LOG_ERR("[%s]:Alloc baseline_abnormal_count failed\n", __func__);
		goto err_alloc_bl_count;
	}

	return 0;
err_alloc_bl_count:
	kfree(sensor->baseline_check);
err_alloc_bl_check:
	kfree(sensor->baseline_original);
err_alloc_bl_orig:
err_creat_workqueue:
	return -ENOMEM;
err_get_elecs:
	return -EIO;
}
#endif

static int rmi_f11_init(struct rmi_function_container *fc)
{
	int rc;

	rc = rmi_f11_initialize(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_register_devices(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_create_sysfs(fc);
	if (rc < 0)
		goto err_free_data;

	return 0;

err_free_data:
	rmi_f11_free_memory(fc);

	return rc;
}

static void rmi_f11_free_memory(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	if (f11) {
		f11_free_control_regs(&f11->dev_controls);
		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++)
			kfree(f11->sensors[i].virtualbutton_map.map);
		kfree(f11);
		fc->data = NULL;
	}
}
//add qgf 
#if defined(SWITCH_FAST_ENERGY_RALAX)
void rmi_f11_set_realse_3count_zero(void)
{
	int i;
	for (i = 0; i < globle_f11_data->dev_query.nbr_of_sensors + 1; i++) {
		globle_f11_data->sensors[i].realse_3count = 0;
	}
}
#endif
//end
static int rmi_f11_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11;
	u8 query_offset;
	u16 query_base_addr;
	u16 control_base_addr;
	u16 max_x_pos, max_y_pos, temp;
	int rc;
	int i;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	VIVO_TS_LOG_DBG("[%s]:Initializing F11 values for %s.\n", __func__,
		 pdata->sensor_name);

	/*
	** init instance data, fill in values and create any sysfs files
	*/
	f11 = kzalloc(sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;
	globle_f11_data = f11;
	fc->data = f11;
#if	RESUME_REZERO
	f11->rezero_on_resume = true;
	f11->rezero_wait_ms = DEFAULT_REZERO_WAIT_MS;
#endif
//#if defined(BBK_DCLICK_WAKE)
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		f11->whether_in_dclick_mode = false;
		f11->has_tp_suspend = false;
	}
//#endif
	mutex_init(&f11->input_slot_mutex);
	
	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &f11->dev_query.f11_2d_query0);
	if (rc < 0)
		return rc;

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		f11->sensors[i].sensor_index = i;

		rc = rmi_f11_get_query_parameters(rmi_dev,
					&f11->sensors[i].sens_query,
					query_offset);
		if (rc < 0)
			return rc;
		query_offset += rc;

		if (f11->dev_query.has_query9) {
			rc = rmi_read(rmi_dev, query_offset,
				      &f11->sensors[i].sens_query.query9.reg);
			if (rc < 0) {
				VIVO_TS_LOG_ERR("[%s]:Failed to read query 9.\n", __func__);
				return rc;
			}
			query_offset += rc;
		}

		rc = f11_allocate_control_regs(rmi_dev,
				&f11->dev_query, &f11->sensors[i].sens_query,
				&f11->dev_controls, control_base_addr);
		if (rc < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to initialize F11 control params.\n", __func__);
			return rc;
		}

		f11->sensors[i].axis_align = pdata->axis_align;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			(u8 *)&max_x_pos, sizeof(max_x_pos));
		if (rc < 0)
			return rc;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			(u8 *)&max_y_pos, sizeof(max_y_pos));
		if (rc < 0)
			return rc;

		if (pdata->axis_align.swap_axes) {
			temp = max_x_pos;
			max_x_pos = max_y_pos;
			max_y_pos = temp;
		}
		f11->sensors[i].max_x = max_x_pos;
		f11->sensors[i].max_y = max_y_pos;

		rc = f11_2d_construct_data(&f11->sensors[i]);
		if (rc < 0)
			return rc;

//#if defined(BBK_DCLICK_WAKE)
        if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			setup_timer(&f11->sensors[i].dclick_timer, 
					rmi_f11_dclick_timer_func, (unsigned long)&f11->sensors[i]);
			INIT_WORK(&f11->sensors[i].dclick_timer_work, rmi_f11_dclick_timer_work_func);
			f11->sensors[i].has_dclick_timer_start = false;
			f11->sensors[i].is_dclick_valide = false;
			f11->sensors[i].pressed_count = 0;
			f11->sensors[i].release_count = 0;
			f11->sensors[i].first_press_x = -1;
			f11->sensors[i].first_press_y = -1;
			f11->sensors[i].second_press_x = -1;
			f11->sensors[i].second_press_y = -1;
		}
//#endif

    //chenyunzhe add  for set VKEY------------------
    s3202_set_virtual_key_string(pdata->virtual_key_string);
	 
	//wangyong add
	setup_timer(&f11->sensors[i].one_finger_timer, 
				finger_timer_func, (unsigned long)&f11->sensors[i]);
	INIT_WORK(&f11->sensors[i].finger_timer_work, finger_timer_work_func);
	f11->sensors[i].down_count= 0;
	f11->sensors[i].timer_flag= 0;
	//end
		
	#if defined(SWITCH_FAST_ENERGY_RALAX)
	f11->sensors[i].realse_3count = 0;
	f11->sensors[i].fast_relax = 0;
	f11->sensors[i].f54_control_base_addr = 0;
	f11->sensors[i].f54_control_base_addr = 0;
	#endif
	//wangyong
	f11->sensors[i].home_state = 0;
	}
	mutex_init(&f11->dev_controls_mutex);
	return 0;
}

static int rmi_f11_register_devices(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	struct input_dev *input_dev;
	struct input_dev *input_dev_mouse;
	int sensors_itertd = 0;
	int i;
	int rc;
#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
	struct rmi_f11_virtualbutton_map *vm_sensor;
	struct rmi_f11_virtualbutton_map *vm_pdata;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
#endif

#if defined(BBK_TS_TRACE_BASELINE)
	global_f11_data = f11;
#endif

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		sensors_itertd = i;
		input_dev = input_allocate_device();
		if (!input_dev) {
			rc = -ENOMEM;
			goto error_unregister;
		}

		f11->sensors[i].input = input_dev;
		/* TODO how to modify the dev name and
		* phys name for input device */
		sprintf(f11->sensors[i].input_name, "%sfn%02x",
			dev_name(&rmi_dev->dev), fc->fd.function_number);
		input_dev->name = f11->sensors[i].input_name;
		sprintf(f11->sensors[i].input_phys, "%s/input0",
			input_dev->name);
		input_dev->phys = f11->sensors[i].input_phys;
		input_dev->dev.parent = &rmi_dev->dev;
		input_set_drvdata(input_dev, f11);

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
#ifdef INPUT_PROP_DIRECT
		set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

		f11_set_abs_params(fc, i);

		VIVO_TS_LOG_DBG("[%s]: Sensor %d hasRel %d.\n",
			__func__, i, f11->sensors[i].sens_query.has_rel);
		if (f11->sensors[i].sens_query.has_rel) {
			set_bit(EV_REL, input_dev->evbit);
			set_bit(REL_X, input_dev->relbit);
			set_bit(REL_Y, input_dev->relbit);
		}
//#if defined(BBK_DCLICK_WAKE)
        if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
			input_set_capability(input_dev, EV_KEY, KEY_WAKEUP_SWIPE);
		}
//#endif
//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
    if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
		input_set_capability(input_dev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);
	}
//#endif
	
	//start add for testing by wangyong 
    input_set_capability(input_dev, EV_KEY, KEY_LEFT	);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT	);
	input_set_capability(input_dev, EV_KEY, KEY_UP		);
	input_set_capability(input_dev, EV_KEY, KEY_O			);
	input_set_capability(input_dev, EV_KEY, KEY_W			);
	input_set_capability(input_dev, EV_KEY, KEY_E			);
	input_set_capability(input_dev, EV_KEY, KEY_M			);
	input_set_capability(input_dev, EV_KEY, KEY_C			);
	input_set_capability(input_dev, EV_KEY, KEY_F			);
	input_set_capability(input_dev, EV_KEY, KEY_A			);
    //end
	rc = input_register_device(input_dev);
		if (rc < 0) {
			input_free_device(input_dev);
			f11->sensors[i].input = NULL;
			goto error_unregister;
		}

#if defined(BBK_TS_TRACE_BASELINE)
		rc = rmi_f11_trace_baseline_init(fc, i);
		if (rc < 0)
			goto error_unregister;
#endif


		/* how to register the virtualbutton device */
#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
		if (f11->sensors[i].sens_query.has_gestures) {
			int j;

			vm_sensor = &f11->sensors[i].virtualbutton_map;
			vm_pdata = pdata->virtualbutton_map;
			if (!vm_pdata) {
				VIVO_TS_LOG_ERR("[%s]:Failed to get the pdata virtualbutton map.\n", __func__);
				goto error_unregister;
			}
			vm_sensor->buttons = vm_pdata->buttons;
			vm_sensor->map = kcalloc(vm_pdata->buttons,
					sizeof(struct virtualbutton_map),
					GFP_KERNEL);
			if (!vm_sensor->map) {
				VIVO_TS_LOG_ERR("[%s]:Failed to allocate the virtualbutton map.\n", __func__);
				rc = -ENOMEM;
				goto error_unregister;
			}
			/* set bits for each button... */
			for (j = 0; j < vm_pdata->buttons; j++) {
				memcpy(&vm_sensor->map[j], &vm_pdata->map[j],
					sizeof(struct virtualbutton_map));
				set_bit(vm_sensor->map[j].code,
					f11->sensors[i].input->keybit);
			}
		}

#endif

		if (f11->sensors[i].sens_query.has_rel) {
			/*create input device for mouse events  */
			input_dev_mouse = input_allocate_device();
			if (!input_dev_mouse) {
				rc = -ENOMEM;
				goto error_unregister;
			}

			f11->sensors[i].mouse_input = input_dev_mouse;
			input_dev_mouse->name = "rmi_mouse";
			input_dev_mouse->phys = "rmi_f11/input0";

			input_dev_mouse->id.vendor  = 0x18d1;
			input_dev_mouse->id.product = 0x0210;
			input_dev_mouse->id.version = 0x0100;

			set_bit(EV_REL, input_dev_mouse->evbit);
			set_bit(REL_X, input_dev_mouse->relbit);
			set_bit(REL_Y, input_dev_mouse->relbit);

			set_bit(BTN_MOUSE, input_dev_mouse->evbit);
			/* Register device's buttons and keys */
			set_bit(EV_KEY, input_dev_mouse->evbit);
			set_bit(BTN_LEFT, input_dev_mouse->keybit);
			set_bit(BTN_MIDDLE, input_dev_mouse->keybit);
			set_bit(BTN_RIGHT, input_dev_mouse->keybit);

			rc = input_register_device(input_dev_mouse);
			if (rc < 0) {
				input_free_device(input_dev_mouse);
				f11->sensors[i].mouse_input = NULL;
				goto error_unregister;
			}

			set_bit(BTN_RIGHT, input_dev_mouse->keybit);
		}

	}

	return 0;

error_unregister:
	for (; sensors_itertd > 0; sensors_itertd--) {
		if (f11->sensors[sensors_itertd].input) {
			if (f11->sensors[sensors_itertd].mouse_input) {
				input_unregister_device(
				   f11->sensors[sensors_itertd].mouse_input);
				f11->sensors[sensors_itertd].mouse_input = NULL;
			}
			input_unregister_device(f11->sensors[i].input);
			f11->sensors[i].input = NULL;
		}
		kfree(f11->sensors[i].virtualbutton_map.map);
	}

	return rc;
}

static void rmi_f11_free_devices(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		if (f11->sensors[i].input)
			input_unregister_device(f11->sensors[i].input);
		if (f11->sensors[i].sens_query.has_rel &&
				f11->sensors[i].mouse_input)
			input_unregister_device(f11->sensors[i].mouse_input);
	}
}

static int rmi_f11_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;
	struct f11_data *f11 = fc->data;

	VIVO_TS_LOG_DBG("[%s]:Creating sysfs files.\n", __func__);
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to create sysfs file for %s.", __func__,
				attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}
	if (sysfs_create_group(&fc->dev.kobj, & attrs_control0) < 0 ) {
	    VIVO_TS_LOG_ERR("[%s]:Failed to create query sysfs files.", __func__);
		return -ENODEV;
	}
	if (f11->dev_controls.ctrl29_30) {
		if (sysfs_create_group(&fc->dev.kobj, &attrs_control29_30) < 0 ) {
			VIVO_TS_LOG_ERR("[%s]:Failed to create query sysfs files.", __func__);
			return -ENODEV;
		}
	}

	return 0;

err_remove_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
						  &attrs[attr_count].attr);
	sysfs_remove_group(&fc->dev.kobj, &attrs_control0);
	if (f11->dev_controls.ctrl29_30) {
		sysfs_remove_group(&fc->dev.kobj, &attrs_control29_30);
	}
	return rc;
}

static int rmi_f11_config(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;
	int rc;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		rc = f11_write_control_regs(fc->rmi_dev,
				   &f11->sensors[i].sens_query,
				   &f11->dev_controls,
				   fc->fd.query_base_addr);
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int rmi_f11_reset(struct rmi_function_container *fc)
{
	/* we do nothing here */
	return 0;
}

//#if defined(BBK_DCLICK_WAKE)
static void wakeup_system_O(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_O, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_O, 0);
	input_sync(sensor->input);
}

static void wakeup_system_M(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_M, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_M, 0);
	input_sync(sensor->input);
}

static void wakeup_system_e(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_E, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_E, 0);
	input_sync(sensor->input);
}

static void wakeup_system_C(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_C, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_C, 0);
	input_sync(sensor->input);
}


static void wakeup_system_W(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_W, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_W, 0);
	input_sync(sensor->input);
}

static void wakeup_system_F(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_F, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_F, 0);
	input_sync(sensor->input);
}

static void wakeup_system_A(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_A, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_A, 0);
	input_sync(sensor->input);
}
//#endif



//#if defined(BBK_DCLICK_WAKE)
static void wakeup_system_dclick(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_WAKEUP, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_WAKEUP, 0);
	input_sync(sensor->input);
}

static void wakeup_system_swipe_down(struct f11_2d_sensor *sensor)
{
	input_report_key(sensor->input, KEY_WAKEUP_SWIPE, 1);
	input_sync(sensor->input);
	input_report_key(sensor->input, KEY_WAKEUP_SWIPE, 0);
	input_sync(sensor->input);

}

static void wakeup_system_swipe_LRUp(struct f11_2d_sensor *sensor, int id, int on)
{
	int key_code = 0;
	if(on & 0x01)
	{
		if(id == 0x01)
			key_code = KEY_RIGHT;
		if(id == 0x02)
			key_code = KEY_LEFT;
	}

	if((id == 0x08)&& (on & 0x02))
		key_code = KEY_UP;
	if(!key_code)
		return;
	else
	{
		input_report_key(sensor->input, key_code, 1);
		input_sync(sensor->input);
		input_report_key(sensor->input, key_code, 0);
		input_sync(sensor->input);
	}
}


static void rmi_f11_dclick_timer_work_func(struct work_struct *work)
{
	struct f11_2d_sensor *sensor = container_of(work, struct f11_2d_sensor,
													dclick_timer_work);

	if (sensor->is_dclick_valide) {
		if (sensor->pressed_count >= 2 && sensor->release_count >= 2)
			wakeup_system_dclick(sensor);
	}

	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
}

static void rmi_f11_dclick_timer_func(unsigned long data)
{
	struct f11_2d_sensor *sensor = (struct f11_2d_sensor *)data;

	VIVO_TS_LOG_ERR("[%s]: dclick timer time out\n", __func__);
	VIVO_TS_LOG_ERR("[%s]: is this time dclick valide -- %s\n",
				__func__, sensor->is_dclick_valide?"yes":"no");
	VIVO_TS_LOG_ERR("[%s]: pressed_count(%d) release_count(%d)\n",
				__func__, sensor->pressed_count, sensor->release_count);
	VIVO_TS_LOG_ERR("[%s]: first_press_x(%d) first_press_y(%d)\n",
				__func__, sensor->first_press_x, sensor->first_press_y);
	VIVO_TS_LOG_ERR("[%s]: second_press_x(%d) second_press_y(%d)\n",
				__func__, sensor->second_press_x, sensor->second_press_y);

	sensor->has_dclick_timer_start = false;
	schedule_work(&sensor->dclick_timer_work);
}

static void rmi_f11_cancel_dclick_trace(struct f11_2d_sensor *sensor)
{

	sensor->has_dclick_timer_start = false;
	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
	del_timer_sync(&sensor->dclick_timer);
}

static bool rmi_f11_whether_point_int_dclick_dimension(struct f11_2d_sensor *sensor,
				int point_x, int point_y)
{
	if (point_x < sensor->dclick_dimension_x_min 
				|| point_x > sensor->dclick_dimension_x_max
				|| point_y < sensor->dclick_dimension_y_min
				|| point_y > sensor->dclick_dimension_y_max) {
		return false;
	}

	return true;
}

static void rmi_f11_judge_dclick(struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	int pre_state = sensor->finger_tracker[0];
	struct f11_2d_data *data = &sensor->data;
	int x, y;
	bool whether_point_legal;
	u8 finger_state;
	u8 finger_0_state = 0;
	u8 finger_pressed_count;
	u8 i;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		finger_state = GET_FINGER_STATE(f_state, i);
		if (i == 0) {
			finger_0_state = finger_state;
		}
		if (finger_state == F11_PRESENT || finger_state == F11_INACCURATE)
			finger_pressed_count++;
	}

	if (finger_pressed_count > 1) {
		VIVO_TS_LOG_ERR("[%s]: More than one finger pressed on the TP\n", __func__);
		if (sensor->has_dclick_timer_start)
			rmi_f11_cancel_dclick_trace(sensor);

		//sensor->finger_tracker[0] = GET_FINGER_STATE(f_state, 0);

		return;
	}

	x = ((data->abs_pos[0].x_msb << 4) |
		data->abs_pos[0].x_lsb);
	y = ((data->abs_pos[0].y_msb << 4) |
		data->abs_pos[0].y_lsb);

	if (!pre_state && !finger_0_state) {
		/* Invalide event nothing to do */
	} else if (!pre_state && finger_0_state) {

		whether_point_legal = rmi_f11_whether_point_int_dclick_dimension(sensor,
										x, y);

		if (!whether_point_legal) {
			if (sensor->has_dclick_timer_start) {
				VIVO_TS_LOG_ERR("[%s]: The point not in dclick dimension cancel trace\n",
						__func__);
				rmi_f11_cancel_dclick_trace(sensor);
			}else{
				VIVO_TS_LOG_ERR("[%s]: The point not in dclick dimension nothing to do\n",
						__func__);
			}

			return;
		}
		/* the first down event of one time press */
		if (!sensor->has_dclick_timer_start) {
			sensor->first_press_x = x;
			sensor->first_press_y = y;
			sensor->pressed_count++;
			
			VIVO_TS_LOG_INF("[%s]: first press start timer\n", __func__);
			mod_timer(&sensor->dclick_timer,
					jiffies + msecs_to_jiffies(500));
			sensor->has_dclick_timer_start = true;
			sensor->is_dclick_valide = true;
		}else{
			sensor->second_press_x = x;
			sensor->second_press_y = y;
			sensor->pressed_count++;

			VIVO_TS_LOG_INF("[%s]: second press start x(%d), y(%d)\n", __func__, x, y);

			if ((x - sensor->first_press_x < -DCLICK_TWO_FINGER_AREA_X
						|| x - sensor->first_press_x > DCLICK_TWO_FINGER_AREA_X)
					|| (y - sensor->first_press_y < -DCLICK_TWO_FINGER_AREA_Y
						|| y - sensor->first_press_y > DCLICK_TWO_FINGER_AREA_Y)) {
				VIVO_TS_LOG_INF("[%s]: The distance of the two down is too large\n", 
								__func__);
				rmi_f11_cancel_dclick_trace(sensor);
			}
		}
	} else {
		/* the pre_state is down event */
		if (finger_0_state) {
			/* down event trace double click */
			if (sensor->pressed_count == 1 && sensor->release_count == 0) {
				if ((x - sensor->first_press_x < -TRIP_X_AREA 
							|| x - sensor->first_press_x > TRIP_X_AREA) 
						|| (y - sensor->first_press_y < -TRIP_Y_AREA
							|| y - sensor->first_press_y > TRIP_Y_AREA)) {
					VIVO_TS_LOG_ERR("[%s]: finger triped in one time down\n", __func__);
					rmi_f11_cancel_dclick_trace(sensor);
				}
			}else if (sensor->pressed_count == 2 && sensor->release_count == 1) {
				if ((x - sensor->second_press_x < -TRIP_X_AREA 
							|| x - sensor->second_press_x > TRIP_X_AREA) 
						|| (y - sensor->second_press_y < -TRIP_Y_AREA
							|| y - sensor->second_press_y > TRIP_Y_AREA)) {
					VIVO_TS_LOG_ERR("[%s]: finger triped in one time down\n", __func__);
					rmi_f11_cancel_dclick_trace(sensor);
				}
			}else{
				/* should not happen nothing to do */
			}
		}else{
			sensor->release_count++;
		}
	}
//	sensor->finger_tracker[0] = finger_0_state;
}
//#endif


#if defined(SWITCH_FAST_ENERGY_RALAX)
static void rmi_f11_close_fast_relax(struct rmi_function_container *fc, struct f11_2d_sensor *sensor)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *ddata = rmi_get_driverdata(rmi_dev);
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;
	u8 fast_ratio = 0;
	int ret;
	u8 temp;
	struct rmi_function_container * f54;
	

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		finger_state = GET_FINGER_STATE(f_state, i);
		if (finger_state == F11_PRESENT || finger_state == F11_INACCURATE)
			finger_pressed_count++;
	}

	if (finger_pressed_count > 1) 
		return;
	if (finger_pressed_count == 0)
		sensor->realse_3count++;

	if(sensor->realse_3count == 1)//just check 1 release
	{
		if(sensor->fast_relax == 0)
		{
			f54 = rmi_get_function_container(ddata, 0x54);
			if (!f54) {
				VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
				return -EINVAL;
			}
			sensor->f54_control_base_addr  = f54->fd.control_base_addr;
			sensor->f54_command_base_addr = f54->fd.command_base_addr;
			VIVO_TS_LOG_INF("[%s]:f54_control_base_addr = %x, f54_command_base_addr = %x~~~~~~~~~\n",__func__,sensor->f54_control_base_addr,sensor->f54_command_base_addr);
			ret = rmi_read(rmi_dev, sensor->f54_control_base_addr +16, &(sensor->fast_relax));
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error in =======\n",__func__);
				return ret;
			}
			VIVO_TS_LOG_INF("[%s]:sensor->fast_relax = %d~~~~~~~\n",__func__,sensor->fast_relax);

		}

		//set Fast Relaxation Rate = 0 ---2D
		ret = rmi_write(rmi_dev, sensor->f54_control_base_addr +16, 0);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
			return ret;
	  	}
	  //set fast relaxation rate = 0  --- 0D
		ret = rmi_write(rmi_dev, sensor->f54_control_base_addr +85, 0);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
			return ret;
		}
		//disable Energy Ratio Relaxation
		ret = rmi_read(rmi_dev, sensor->f54_control_base_addr, &temp);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error  =======\n",__func__);
			return ret;
		}
		temp = temp & 0xdf;
		VIVO_TS_LOG_INF("[%s]:disable Energy~~~~temp |  0xdf = %x~~~~~~~~\n",__func__,temp);
		ret = rmi_write(rmi_dev, sensor->f54_control_base_addr, temp);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error in=======\n",__func__);
			return ret;
		}
		
		//Force update
		ret = rmi_read(rmi_dev,sensor->f54_command_base_addr, &temp);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error in =======\n",__func__);
			return ret;
		}
		temp = temp | 0x04;
		VIVO_TS_LOG_INF("[%s]:Force update~~temp |  0x04 = %x~~~~~~~~\n",__func__,temp);
		ret = rmi_write(rmi_dev, sensor->f54_command_base_addr, temp);
		if (ret < 0)
		{
			VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error in ========\n",__func__);
			return ret;
		}
			
	}		

}
#endif

static void rmi_f11_set_release_point_jiffies(struct rmi_function_container *fc,
										struct f11_2d_sensor *sensor)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *ddata = rmi_get_driverdata(rmi_dev);
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 finger_pressed_count;
	u8 i;
	

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		finger_state = GET_FINGER_STATE(f_state, i);
		if (finger_state == F11_PRESENT || finger_state == F11_INACCURATE) {
			finger_pressed_count++;
		}
	}

	if (finger_pressed_count == 0) {
#if defined(BBK_TS_TRACE_BASELINE)
		if (sensor->need_force_cal) {
			VIVO_TS_LOG_ERR("[%s]:Need to calibrate when all finger released\n", __func__);
			rmi_f11_force_cal(sensor);
			rmi_f11_reset_trace_data(sensor);
		}else{
			if (!sensor->has_orig_baseline_init) {
				sensor->need_get_org_baseline = true;
				//rmi_f11_get_orig_baseline(fc, sensor);
			} else {
			/* 
				judge the original legal when all fingers release
				make sure the original baseline is legal
				if has judged ORIGINAL_BASE_SHOULD_JUDGE_TIME times
				we think the original baseline is legal
			*/
				if (!sensor->orig_baseline_legal) {
					sensor->need_check_org_baseline = true;
					/* use need_check_baseline to make queue the work */
					sensor->need_check_baseline = true;
					//(void)rmi_f11_makesure_orig_baseline_legal_and_get_abnormal_ch_count(sensor);
				}
			}
		}

#endif

		ddata->AA_area_point_pressed = 0;
		ddata->AA_area_point_release_time = jiffies;
	}else{
		if (ddata->AA_area_point_pressed == 0) {
			ddata->AA_area_point_pressed = 1;
		}
	}
}
#if 0
extern bool lis3dh_for_ts_judge_dir(void);
#endif

extern bool (*acc_for_ts_judge_dir)(void);

int rmi_f11_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *ddata = rmi_get_driverdata(rmi_dev); /* for disable report when lcd off */
	struct f11_data *f11 = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u16 data_base_addr_offset = 0;
	int error;
	int i;
	

	bool is_phone_handstand = false;

//#if defined(BBK_DCLICK_WAKE)
	int j;
	u8 dclick_data;
    u8 temp_buf[8];
	u8 lsb_buf;
	u8 msb_buf;
	u16 temp;
//#endif
    
	//chenyunzhe add (must)----------------------------------------------
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	
		if(ddata->largetouch_flag)
			ddata->mute_switch = 0;
			
	VIVO_TS_LOG_DBG("[%s]: ========is called!=========== \n",__func__);

#if 0 //defined(BBK_TS_BASELINE_TRACE)
	if (!sensor->fc54) {
		sensor->fc54 = rmi_get_function_container(ddata, 0x54);
		if (!sensor->fc54) {
			VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
			return -EINVAL;
		}
	}
#endif
	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		error = rmi_read_block(rmi_dev,
				data_base_addr + data_base_addr_offset,
				f11->sensors[i].data_pkt,
				f11->sensors[i].pkt_size);

		if (error < 0)
			return error;

		rmi_f11_set_release_point_jiffies(fc, &f11->sensors[i]);

	#if defined(SWITCH_FAST_ENERGY_RALAX)
		if(f11->has_tp_suspend == false && f11->sensors[i].realse_3count <1)
			rmi_f11_close_fast_relax(fc,&f11->sensors[i]);
	#endif
//#if defined(BBK_LARGE_OBJ_SUPPRESSION)
        if(vivo_touchscreen_is_support(FS_LARGE_OBJ_SUPPRESSION)) {
			VIVO_TS_LOG_DBG("[%s]:  ddata->is_calling = %d, ddata->mute_switch = %d, ddata->largetouch_flag = %d  @@@@@@@@\n ", __func__,ddata->is_calling, ddata->mute_switch, ddata->largetouch_flag);

			if(ddata->is_calling)
			{	
				if(ddata->mute_switch)
				{
					if (f11->sensors[i].data.extended_status)
					{	
					
						if (f11->sensors[i].data.extended_status->large_supress)
						{
							VIVO_TS_LOG_DBG("[%s]: ======== large supress now!!=========== \n", __func__);
						
							rmi_f11_handle_large_obj_supression(&f11->sensors[i]);
							ddata->is_calling = 0;
						}
					}

					if (!f11->has_tp_suspend) 
					{	
						rmi_f11_calling_3finger_handler(&f11->sensors[i],ddata);
					}
				}
			}
		}
//#endif

//#if defined(BBK_DCLICK_WAKE)
        if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
			if (ddata->ts_dclick_simulate_switch == 1) {
				rmi_f11_judge_dclick(&f11->sensors[i]);
				//return 0;
			}

		/**wangyong add for all gesture expect down swipe and dclick ***
		* 0x01: Lift or Right swipe  0x02: up swipe 	0x04: O 
		* 0x08: W			 0x10: M			   0x20: e		0x40: C 
		*/	
			VIVO_TS_LOG_DBG("[%s]: has_lcd_shutoff = %d, ts_dclick_switch = %d, swipe_down_switch =%d, gesture_switch = %d,whether_in_dclick_mode = %d\n", __func__, ddata->has_lcd_shutoff,ddata->ts_dclick_switch,ddata->swipe_down_switch,ddata->gesture_switch,f11->whether_in_dclick_mode);
			if (ddata->has_lcd_shutoff && (ddata->ts_dclick_switch || ddata->swipe_down_switch || ddata->gesture_switch)) {
				if ((!f11->has_tp_suspend)&& ddata->ts_dclick_switch){
					rmi_f11_judge_dclick(&f11->sensors[i]);
				}else{

					if(NULL == acc_for_ts_judge_dir)
							is_phone_handstand = false;
					else
							is_phone_handstand = acc_for_ts_judge_dir();

					VIVO_TS_LOG_INF("[%s]: is_phone_handstand = %d ", __func__, is_phone_handstand);

					
					if (is_phone_handstand) {	
						return 0;
					}
					if (f11->whether_in_dclick_mode) {
						error= rmi_read_block(rmi_dev,
								data_base_addr + data_base_addr_offset + DCLICK_DATA_OFFSET,
								&dclick_data, 1);
						
						error= rmi_read_block(rmi_dev,
								data_base_addr + data_base_addr_offset + DCLICK_DATA_OFFSET+1,
								temp_buf, 8);
						  VIVO_TS_LOG_INF("[%s]: dclick_data = 0x%x, temp_buf[6] = 0x%x \n", __func__,dclick_data, temp_buf[6]);
						  wake_lock_timeout(&ddata->wakelock, HZ);
						//read 0x0400
						
						VIVO_TS_LOG_INF("[%s]: tsd(%d %d) lcd(%d %d )\n",__func__,pdata->ts_dimension_x, pdata->ts_dimension_y,  
			                                                 pdata->lcd_dimension_x, pdata->lcd_dimension_y);
						for(j=0; j<12; j++)
						{
							error= rmi_read_block(rmi_dev,F51_BASE_ADDR+j*2,&lsb_buf,1);
							error= rmi_read_block(rmi_dev,F51_BASE_ADDR+j*2+1,&msb_buf,1);
							ddata->buf_gesture[j] =  ((msb_buf<<8) | lsb_buf);	
						#if 1 //chenyunzhe modify
						    if( j%2 == 0)
								ddata->buf_gesture[j] = (ddata->buf_gesture[j] * pdata->lcd_dimension_x)/pdata->ts_dimension_x;//x---720/1266
							else
								ddata->buf_gesture[j] = (ddata->buf_gesture[j] * pdata->lcd_dimension_y)/pdata->ts_dimension_y;//y---1280/2220

							if(( j == 1 ) && ( ddata->buf_gesture[j] > pdata->lcd_dimension_y ) )
								ddata->buf_gesture[1] = pdata->lcd_dimension_y;
                        #else						
							if( j%2 == 0)
								ddata->buf_gesture[j] = (ddata->buf_gesture[j]*720)/1266;//x---720/1266
							else
								ddata->buf_gesture[j] = (ddata->buf_gesture[j]*1280)/2220;//y---1280/2220

							if(( j == 1 ) && ( ddata->buf_gesture[j] > 1280 ) )
								ddata->buf_gesture[1] =1280;
						#endif
						}

						
						temp = ddata->buf_gesture[10];
						ddata->buf_gesture[10] = ddata->buf_gesture[2];
						ddata->buf_gesture[2] = temp;

						temp = ddata->buf_gesture[11];
						ddata->buf_gesture[11] = ddata->buf_gesture[3];
						ddata->buf_gesture[3] = temp;
									
						
						if ((dclick_data & 0x01)&& ddata->ts_dclick_switch) {
							VIVO_TS_LOG_INF("[%s]: [SYNA] TP wake up the system with double tap\n",__func__);
							wakeup_system_dclick(&f11->sensors[0]);
					
						}

						if (dclick_data & 0x02)
						{
							error= rmi_read_block(rmi_dev,F51_BASE_ADDR+24,&lsb_buf,1);
							VIVO_TS_LOG_INF("[%s]: TP wake up the system with swip id = %d, gesture_switch = %d\n",__func__,lsb_buf,ddata->gesture_switch);
							/* Syna 3202 sometime may Error Identification. So Need correct it here. Add qiuguifu*/
							if(lsb_buf == 0x04 && ddata->buf_gesture[1] > ddata->buf_gesture[11] )
							{
								lsb_buf = 0x08;
								VIVO_TS_LOG_INF("[%s]: Get swipe error.Error Correction. lsb_buf = %d \n",__func__,lsb_buf);
							}
							if(lsb_buf == 0x08 && ddata->buf_gesture[1] < ddata->buf_gesture[11] )
							{
								lsb_buf = 0x04;
								VIVO_TS_LOG_INF("[%s]: Get swipe error.Error Correction. lsb_buf = %d \n",__func__,lsb_buf);
							}
							/*Add qiuguifu End*/
							if((lsb_buf == 0x04)&&(ddata->swipe_down_switch))
								wakeup_system_swipe_down(&f11->sensors[0]);
							if(((lsb_buf == 0x01) || (lsb_buf == 0x02) || (lsb_buf == 0x08)) && ((ddata->gesture_switch & 0x01)|| (ddata->gesture_switch & 0x02)))
								wakeup_system_swipe_LRUp(&f11->sensors[0],lsb_buf,ddata->gesture_switch);
					
						}

						if ((dclick_data & 0x08)&& (ddata->gesture_switch & 0x04)) {
							VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== O ====\n",__func__);
							error= rmi_read_block(rmi_dev,F51_BASE_ADDR+24,&lsb_buf,1);
							ddata->buf_gesture[19] = lsb_buf;
							wakeup_system_O(&f11->sensors[0]);
						}

						if (dclick_data & 0x40) {
							if((temp_buf[6] == 0x63)&& (ddata->gesture_switch & 0x40))
							{
								VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== C ====\n",__func__);
								wakeup_system_C(&f11->sensors[0]);
							}

							if((temp_buf[6] == 0x65)&& (ddata->gesture_switch & 0x20))
							{
								VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== e ====\n",__func__);
								wakeup_system_e(&f11->sensors[0]);
							}

							if((temp_buf[6] == 0x6d)&& (ddata->gesture_switch & 0x10))
							{
								VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== M ====\n",__func__);
								wakeup_system_M(&f11->sensors[0]);
							}

							if((temp_buf[6] == 0x77)&& (ddata->gesture_switch & 0x08))
							{
								VIVO_TS_LOG_INF("[%s]: TP wake up the system with ==== W ====\n",__func__);
								wakeup_system_W(&f11->sensors[0]);
							}
										
							//0x02:@ 0x04:f    0x7c:@  0x7b:f
							if((temp_buf[6] == 0x7b) && (ddata->gesture_switch_export & 0x04))
							{
								VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== F ====\n",__func__);
								wakeup_system_F(&f11->sensors[0]);
							}
							if((temp_buf[6] == 0x7c) && (ddata->gesture_switch_export & 0x02))
							{
								VIVO_TS_LOG_INF("[%s]:  TP wake up the system with ==== @ ====\n",__func__);
								wakeup_system_A(&f11->sensors[0]);
							}							
							
						}
					}
				}
				
			if (ddata->has_lcd_shutoff) {
				VIVO_TS_LOG_INF("[%s]:LCD has shut off\n", __func__);
				}
				return 0;
			}
        }
//#endif
		rmi_f11_finger_handler(&f11->sensors[i],ddata);
		rmi_f11_virtual_button_handler(&f11->sensors[i]);
		data_base_addr_offset += f11->sensors[i].pkt_size;

#if defined(BBK_TS_TRACE_BASELINE)
	if (f11->sensors[i].need_check_baseline || f11->sensors[i].need_get_org_baseline) {
		//(void)rmi_f11_check_baseline_delta(sensor);
		queue_work(f11->sensors[i].ts_trace_workqueue, &f11->sensors[i].ts_trace_work);
		/* don't assignment in work func for may another irq happened before work func */
		f11->sensors[i].need_check_baseline = false;
		f11->sensors[i].need_get_org_baseline = false;
	}
#endif

	}

	return 0;
}

#if RESUME_REZERO
//static void rmi_ten_fingers_release(struct f11_2d_sensor *sensor);

static int rmi_f11_resume(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *data = fc->data;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};
	int retval = 0;
	int i;
//#if defined(BBK_DCLICK_WAKE)
	u8 report_mode;
//#endif

	VIVO_TS_LOG_DBG("[%s]:Resuming...\n", __func__);

//#if defined(BBK_DCLICK_WAKE)
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
		retval = rmi_read_block(rmi_dev, fc->fd.control_base_addr,
						&report_mode, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to read report mode\n", __func__);
			report_mode = 0x00;
		}

		report_mode &= 0xf8;
		
		retval = rmi_write_block(rmi_dev, fc->fd.control_base_addr,
						&report_mode, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write report mode\n", __func__);
			return retval;
		}

		data->whether_in_dclick_mode = false;
		data->has_tp_suspend = false;
	}
//#endif


	for (i = 0; i < data->dev_query.nbr_of_sensors + 1; i++) {
		//rmi_ten_fingers_release(&data->sensors[i]);
		rmi_f11_release_point(&data->sensors[i]);
		#if defined(SWITCH_FAST_ENERGY_RALAX)
		data->sensors[i].realse_3count = 0;
		#endif
	}


	if (!data->rezero_on_resume)
		return 0;

	if (data->rezero_wait_ms)
		mdelay(data->rezero_wait_ms);

	commands.rezero = 1;
	retval = rmi_write_block(rmi_dev, fc->fd.command_base_addr,
			&commands.reg, sizeof(commands.reg));
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to issue rezero command, error = %d.",
			__func__, retval);
		return retval;
	}

	return retval;
}
#endif /* RESUME_REZERO */

//#if defined(BBK_DCLICK_WAKE)
static int rmi_f11_suspend(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *ddata = rmi_get_driverdata(rmi_dev);
	int i;
#if defined(SWITCH_FAST_ENERGY_RALAX)
	struct rmi_function_container * f54;
	int ret;
	u8 temp;
#endif
    if(vivo_touchscreen_is_support(FS_DCLICK_WAKE)) {
  
		ddata->AA_area_point_pressed = 0;

		if(ddata->ts_dclick_simulate_switch)
			ddata->ts_dclick_simulate_switch = 0;


		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
			rmi_f11_cancel_dclick_trace(&f11->sensors[i]);
			f11->sensors[i].release_count = 0;
			#if defined(SWITCH_FAST_ENERGY_RALAX)
			if(f11->sensors[i].fast_relax == 0)
			{
				f54 = rmi_get_function_container(ddata, 0x54);
				if (!f54) {
					VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
					return -EINVAL;
				}
				f11->sensors[i].f54_control_base_addr  = f54->fd.control_base_addr;
				f11->sensors[i].f54_command_base_addr = f54->fd.command_base_addr;
				VIVO_TS_LOG_INF("[%s]:f54_control_base_addr = %x, f54_command_base_addr = %x~~~~~~~~~\n",__func__,f11->sensors[i].f54_control_base_addr,f11->sensors[i].f54_command_base_addr);
				ret = rmi_read(rmi_dev, f11->sensors[i].f54_control_base_addr +16, &(f11->sensors[i].fast_relax));
				if (ret < 0)
				{
					VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error  =======\n",__func__);
					return ret;
				}
				VIVO_TS_LOG_INF("[%s]:f11->sensors[i].fast_relax = %d~~~~~\n",__func__,f11->sensors[i].fast_relax);

			}

			//set Fast Relaxation Rate to orig
			VIVO_TS_LOG_INF("[%s]:set Fast Relaxation Rate to orig~~~~~f11->sensors[i].fast_relax = %d~~~~~~~~~\n",__func__,f11->sensors[i].fast_relax);
			ret = rmi_write(rmi_dev, f11->sensors[i].f54_control_base_addr +16,  f11->sensors[i].fast_relax);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error=======\n",__func__);
				return ret;
			}
			//disable Energy Ratio Relaxation
			ret = rmi_read(rmi_dev, f11->sensors[i].f54_control_base_addr, &temp);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error =======\n",__func__);
				return ret;
			}
			temp = temp | 0x20;
			VIVO_TS_LOG_INF("[%s]:disable Energy~~~temp |  0x20 = %x~~~~~~~~\n",__func__,temp);
			ret = rmi_write(rmi_dev, f11->sensors[i].f54_control_base_addr, temp);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error =======\n",__func__);
				return ret;
			}
			
			//Force update
			ret = rmi_read(rmi_dev, f11->sensors[i].f54_command_base_addr, &temp);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_read is error =======\n",__func__);
				return ret;
			}
			temp = temp | 0x04;
			VIVO_TS_LOG_INF("[%s]:Force update~~~temp | 0x04 = %x~~~~~~~~\n",__func__,temp);
			ret = rmi_write(rmi_dev, f11->sensors[i].f54_command_base_addr, temp);
			if (ret < 0)
			{
				VIVO_TS_LOG_ERR("[%s]:=======rmi_write is error in %s =======\n",__func__);
				return ret;
			}
			#endif
				
		}
		f11->has_tp_suspend = true;
	}

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		//rmi_ten_fingers_release(&data->sensors[i]);
		rmi_f11_release_point(&f11->sensors[i]);
		#if defined(SWITCH_FAST_ENERGY_RALAX)
		f11->sensors[i].realse_3count = 0;
		#endif
	}
	
	return 0;
}
//#endif

//#if defined(BBK_DCLICK_WAKE)
int rmi_f11_change_report_mode_to_dclick(struct rmi_function_container *fc, bool yes)
{
	u8 report_mode;
	int retval = 0;
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *data = fc->data;

	retval = rmi_read_block(rmi_dev, fc->fd.control_base_addr,
					&report_mode, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]: failed to read report mode\n", __func__);
		return retval;
	}
	else 
		retval = 0;

	if (yes) {
		report_mode = (report_mode & 0xf8) | 0x04;
		VIVO_TS_LOG_INF("[%s]: report_mode = %x======\n",__func__,report_mode);

		retval = rmi_write_block(rmi_dev, fc->fd.control_base_addr,
						&report_mode, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write report mode\n", __func__);
			return retval;
		}
		else
			retval = 0;

		data->whether_in_dclick_mode = true;
	}else{
		report_mode &= 0xf8;
		VIVO_TS_LOG_INF("[%s]: report_mode = %x======\n",__func__,report_mode);
		retval = rmi_write_block(rmi_dev, fc->fd.control_base_addr,
						&report_mode, 1);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to write report mode\n", __func__);
			return retval;
		}
		else
			retval = 0;
	
		data->whether_in_dclick_mode = false;
	}

	return retval;
}

void rmi_f11_get_dclick_flag_info(struct rmi_function_container *fc, 
									int *whether_in_dclick_mode, int *has_tp_suspend)
{
	struct f11_data *data = fc->data;

	*whether_in_dclick_mode = (int)data->whether_in_dclick_mode;
	*has_tp_suspend = (int)data->has_tp_suspend;
}

void rmi_f11_set_dclick_flag_info(struct rmi_function_container *fc, 
									int *whether_in_dclick_mode, int *has_tp_suspend)
{
	struct f11_data *data = fc->data;

	data->whether_in_dclick_mode = (bool)*whether_in_dclick_mode;
	data->has_tp_suspend = (bool)*has_tp_suspend;
}
//#endif

static void rmi_f11_remove(struct rmi_function_container *fc)
{
	int attr_count = 0;
	struct f11_data *f11 = fc->data;
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	}

	sysfs_remove_group(&fc->dev.kobj, &attrs_control0);
	if (f11->dev_controls.ctrl29_30) {
		sysfs_remove_group(&fc->dev.kobj, &attrs_control29_30);
	}

	rmi_f11_free_devices(fc);

	rmi_f11_free_memory(fc);

}

static struct rmi_function_handler function_handler = {
	.func = 0x11,
	.init = rmi_f11_init,
	.config = rmi_f11_config,
	.reset = rmi_f11_reset,
	.attention = rmi_f11_attention,
	.remove = rmi_f11_remove,
#if	RESUME_REZERO
#if defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND)
	.late_resume = rmi_f11_resume,
#elif defined(CONFIG_FB)
	.late_resume = rmi_f11_resume,
#else
	.resume = rmi_f11_resume,
#endif  /* defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_RMI4_SPECIAL_EARLYSUSPEND) */
//#if defined(BBK_DCLICK_WAKE)
	.early_suspend = rmi_f11_suspend,
//#endif
#endif
};

static int __init rmi_f11_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]: register failed!\n", __func__);
		return error;
	}

	return 0;
}

static void __exit rmi_f11_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
			data->sensors[0].max_x, data->sensors[0].max_y);
}

#if defined(BBK_TS_TRACE_BASELINE)
static ssize_t f11_baseline_trace_info_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;
	int count = 0;
	int i;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	count += sprintf(&buf[count], "has_orig_baseline_init is %s\n", 
				data->sensors[0].has_orig_baseline_init?"true":"false");
	count += sprintf(&buf[count], "orig_baseline_legal is %s\n", 
				data->sensors[0].orig_baseline_legal?"true":"false");
	count += sprintf(&buf[count], "need_force_cal is %s\n", 
				data->sensors[0].need_force_cal?"true":"false");
	count += sprintf(&buf[count], "orig_baseline_judge_times is %d\n", 
				data->sensors[0].orig_baseline_judge_times);
	count += sprintf(&buf[count], "rx_line is %d\n", data->sensors[0].rx_line);
	count += sprintf(&buf[count], "tx_line is %d\n", data->sensors[0].tx_line);
	count += sprintf(&buf[count], "channel_count is %d\n", data->sensors[0].channel_count);
	count += sprintf(&buf[count], "need_check_baseline is %d\n", 
				data->sensors[0].need_check_baseline);
	count += sprintf(&buf[count], "need_get_org_baseline is %d\n", 
				data->sensors[0].need_get_org_baseline);
	count += sprintf(&buf[count], "need_check_org_baseline is %d\n", 
				data->sensors[0].need_check_org_baseline);
	//count += sprintf(&buf[count], "max_count is %d\n", data->sensors[0].max_count);

	count += sprintf(&buf[count], "\nbaseline_original is:");
	for (i = 0; i < data->sensors[0].channel_count; i++) {
		if (i % data->sensors[0].rx_line == 0)
			count += sprintf(&buf[count], "\n");
		count += sprintf(&buf[count], "%d ", data->sensors[0].baseline_original[i]);
	}

	count += sprintf(&buf[count], "\n");
	count += sprintf(&buf[count], "\nbaseline_check is:");
	for (i = 0; i < data->sensors[0].channel_count; i++) {
		if (i % data->sensors[0].rx_line == 0)
			count += sprintf(&buf[count], "\n");
		count += sprintf(&buf[count], "%d ", data->sensors[0].baseline_check[i]);
	}

	count += sprintf(&buf[count], "\n");
	count += sprintf(&buf[count], "\nbaseline_abnormal_count is:");
	for (i = 0; i < data->sensors[0].channel_count; i++) {
		if (i % data->sensors[0].rx_line == 0)
			count += sprintf(&buf[count], "\n");
		count += sprintf(&buf[count], "%d ", data->sensors[0].baseline_abnormal_count[i]);
	}
	count += sprintf(&buf[count], "\n");

	return count;
}
#endif

static ssize_t f11_flip_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
			data->sensors[0].axis_align.flip_x,
			data->sensors[0].axis_align.flip_y);
}

static ssize_t f11_flip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_X, new_Y;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u %u", &new_X, &new_Y) != 2)
		return -EINVAL;
	if (new_X < 0 || new_X > 1 || new_Y < 0 || new_Y > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.flip_x = new_X;
	instance_data->sensors[0].axis_align.flip_y = new_Y;

	return count;
}

static ssize_t f11_swap_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->sensors[0].axis_align.swap_axes);
}

static ssize_t f11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int newSwap;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u", &newSwap) != 1)
		return -EINVAL;
	if (newSwap < 0 || newSwap > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.swap_axes = newSwap;

	f11_set_abs_params(fc, 0);

	return count;
}

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->
			sensors[0].axis_align.rel_report_enabled);
}

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.rel_report_enabled = new_value;

	return count;
}

static ssize_t f11_offset_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d %d\n",
			instance_data->sensors[0].axis_align.offset_X,
			instance_data->sensors[0].axis_align.offset_Y);
}

static ssize_t f11_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	int new_X, new_Y;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%d %d", &new_X, &new_Y) != 2)
		return -EINVAL;
	instance_data->sensors[0].axis_align.offset_X = new_X;
	instance_data->sensors[0].axis_align.offset_Y = new_Y;

	return count;
}
/*
static void rmi_ten_fingers_release(struct f11_2d_sensor *sensor)
{
	int i;
	printk("SYNA %s is called! \n",__func__);
	if(globle_f11_data == NULL )
	{
		printk("SYNA %s : globle_f11_data is NULL\n",__func__);
		return ;
	}
	mutex_lock(&globle_f11_data->input_slot_mutex);//qiuguifu add for input slot Concurrency issue.

	for (i = 0; i < sensor->nbr_fingers; i++) {
	input_mt_slot(sensor->input, i);
	input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER, true);
		input_report_abs(sensor->input, ABS_MT_PRESSURE, 53);
		input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, 50);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, 2);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, 200+5*i);
	}
	input_sync(sensor->input);
	mutex_unlock(&globle_f11_data->input_slot_mutex);
	msleep(5);
	mutex_lock(&globle_f11_data->input_slot_mutex);
	for (i = sensor->nbr_fingers -1; i >= 0; i--){
		input_mt_slot(sensor->input, i);
		input_mt_report_slot_state(sensor->input, MT_TOOL_FINGER, false);
	}
	input_sync(sensor->input);
	mutex_unlock(&globle_f11_data->input_slot_mutex);
}
	
*/
static ssize_t f11_test_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	struct f11_2d_sensor *sensor = NULL; 
//	int i;
	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	sensor = &instance_data->sensors[0];
	//rmi_ten_fingers_release(sensor);
	return 0;


}
static ssize_t f11_clip_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
			instance_data->sensors[0].axis_align.clip_X_low,
			instance_data->sensors[0].axis_align.clip_X_high,
			instance_data->sensors[0].axis_align.clip_Y_low,
			instance_data->sensors[0].axis_align.clip_Y_high);
}

static ssize_t f11_clip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_X_low, new_X_high, new_Y_low, new_Y_high;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u %u %u %u",
		   &new_X_low, &new_X_high, &new_Y_low, &new_Y_high) != 4)
		return -EINVAL;
	if (new_X_low < 0 || new_X_low >= new_X_high || new_Y_low < 0
	    || new_Y_low >= new_Y_high)
		return -EINVAL;
	instance_data->sensors[0].axis_align.clip_X_low = new_X_low;
	instance_data->sensors[0].axis_align.clip_X_high = new_X_high;
	instance_data->sensors[0].axis_align.clip_Y_low = new_Y_low;
	instance_data->sensors[0].axis_align.clip_Y_high = new_Y_high;

	/*
	** for now, we assume this is sensor index 0
	*/
	f11_set_abs_params(fc, 0);

	return count;
}

static ssize_t f11_rezero_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int rezero;
	int retval = 0;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};

	fc = to_rmi_function_container(dev);

	if (sscanf(buf, "%u", &rezero) != 1)
		return -EINVAL;
	if (rezero < 0 || rezero > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (rezero) {
		commands.rezero = 1;
		retval = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
				&commands.reg, sizeof(commands.reg));
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]: failed to issue rezero command, error = %d.",
				__func__, retval);
			return retval;
		}
	}

	return count;
}

#if RESUME_REZERO
static ssize_t f11_rezeroOnResume_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int newValue;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &newValue) != 1)
		return -EINVAL;
	if (newValue < 0 || newValue > 1) {
		VIVO_TS_LOG_ERR("[%s]:rezeroOnResume must be either 1 or 0.\n", __func__);
		return -EINVAL;
	}

	instance_data->rezero_on_resume = (newValue != 0);

	return count;
}

static ssize_t f11_rezeroOnResume_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->rezero_on_resume);
}

static ssize_t f11_rezeroWait_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int newValue;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &newValue) != 1)
		return -EINVAL;
	if (newValue < 0) {
		VIVO_TS_LOG_ERR("[%s]:rezeroWait must be 0 or greater.\n", __func__);
		return -EINVAL;
	}

	instance_data->rezero_wait_ms = (newValue != 0);

	return count;
}

static ssize_t f11_rezeroWait_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->rezero_wait_ms);
}

#endif

int rmi_f11_get_sensor_electrodes(struct rmi_function_container *fc,
									int *rx_num, int *tx_num)
{
	int retval;
	u8 rx, tx;

	retval = rmi_read_block(fc->rmi_dev, 
			fc->fd.control_base_addr + RX_NUM_REG_OFFSET,
			&rx, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read rx num failed\n", __func__);
		return -1;
	}

	retval = rmi_read_block(fc->rmi_dev, 
			fc->fd.control_base_addr + TX_NUM_REG_OFFSET,
			&tx, 1);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read tx num failed\n", __func__);
		return -1;
	}

	*rx_num = rx;
	*tx_num = tx;

	return 0;
}

/* Control sysfs files */
show_store_union_struct_unsigned(dev_controls, ctrl0_9, abs_pos_filt)
show_store_union_struct_unsigned(dev_controls, ctrl29_30, z_touch_threshold)
show_store_union_struct_unsigned(dev_controls, ctrl29_30, z_touch_hysteresis)
module_init(rmi_f11_module_init);
module_exit(rmi_f11_module_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI F11 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
