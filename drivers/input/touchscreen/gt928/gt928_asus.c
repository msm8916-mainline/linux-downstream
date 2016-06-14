#include <linux/irq.h>
#include <linux/slab.h>
#include <gt928.h>
#include <linux/kthread.h>

// add by leo for cable status notifier ++
#include <linux/power_supply.h>
#include <linux/notifier.h>
// add by leo for cable status notifier --

#define FW_CHECK		1
#define FW_UPDATE		2
#define DEBUG_FLAG		3
#define INTR_PIN			4
#define RST_PIN			5
#define IRQ_ENABLE		6
#define IC_RESET			7
#define WORK_FUNC		8
#define ESD_ENABLE		9
#define TP_ID_PIN		10
#define READ_TP_ID		11
#define WR_NODE 		12
#define CLEAN_CFG		13
#define GESTURE_FUNC	14
#define READ_DISPLAY_ID	15

#define TOUCH_FW_UPGRADE_SUCCESS	0
#define TOUCH_FW_UPGRADE_FAIL		1
#define TOUCH_FW_UPGRADE_PROCESS	2
#define TOUCH_FW_UPGRADE_INIT		3

#define TOUCH_IOCTL_MAGIC 't'
#define TOUCH_IOCTL_FW_CHECK			_IOR(TOUCH_IOCTL_MAGIC, 1, int)
#define TOUCH_IOCTL_FW_UPDATE		_IOR(TOUCH_IOCTL_MAGIC, 2, int)
#define TOUCH_IOCTL_FORCE_FW_UPDATE	_IOR(TOUCH_IOCTL_MAGIC, 3, int)
#define TOUCH_IOCTL_READ_CMD			_IOR(TOUCH_IOCTL_MAGIC, 4, int)
#define TOUCH_IOCTL_WRITE_CMD			_IOW(TOUCH_IOCTL_MAGIC, 5, int)
#define TOUCH_IOCTL_GESTURE_CMD		_IOW(TOUCH_IOCTL_MAGIC, 6, int)

bool GT928_IsChecked = false;
int GT928_FWChecking = 0;
int GT928_FWForceUpdate = 0;
int GT928_CFGCheckResult = NO_NEED_TO_UPDATE;
int GT928_FWCheckResult = NO_NEED_TO_UPDATE;
int GT928_CFGUpdateResult = FAIL;
int GT928_FWUpdateResult = FAIL;
u8 GT928_IC_CFGversion = 0;
u8 GT928_File_CFGversion = 0;
u16 GT928_IC_FWversion = 0;
u16 GT928_File_FWversion = 0;

static unsigned int debug_function = 0;
static int test_result = 0;
static int ap_progress = 0;

static struct proc_dir_entry *gt928_proc_debug_file = NULL;
static struct proc_dir_entry *gt928_proc_tp_debug_file = NULL;
static struct proc_dir_entry *gt928_proc_gesture_file = NULL;

s8 gt928_read_cfg_version(struct i2c_client *client);
int gt928_get_display_id(void);
int gt928_get_tp_id(int source);

typedef struct
{
	u8  hw_info[4];	//hardware info
	u8  pid[8];		//product id
	u16 vid;			 //version id
} st_fw_head;
#pragma pack()

typedef struct
{
	u8 force_update;
	u8 fw_flag;
	struct file *file;
	struct file *cfg_file;
	st_fw_head  ic_fw_msg;
	mm_segment_t old_fs;
	u32 fw_total_len;
	u32 fw_burned_len;
} st_update_msg;
extern st_update_msg update_msg;

extern unsigned int entry_mode;
extern int build_version;
extern int gt928_debug;

#if DRIVER_SEND_AC_CONFIG
extern int ac_old_status;
#endif

extern struct workqueue_struct *goodix_wq;
extern struct i2c_client * i2c_connect_client;

// add by leo for AC config ++
#if DRIVER_SEND_AC_CONFIG
extern struct delayed_work  gt928_send_ac_config_work;
extern struct workqueue_struct * gt928_send_ac_config_workqueue;
extern u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH];
#endif
// add by leo for AC config --

extern u8 gup_init_update_proc(struct goodix_ts_data *ts);
extern u8 gup_check_update_file(struct i2c_client *client, st_fw_head* fw_head, u8* path);
extern u8 gup_get_ic_fw_msg(struct i2c_client *client);
extern u8 gup_enter_update_judge(st_fw_head *fw_head);

extern s8 gtp_i2c_test(struct i2c_client *client);
extern s8 gt9xx_read_Config_Checksum(void);
extern s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gup_i2c_write(struct i2c_client *client, u8 *buf, s32 len);
extern s32 gtp_read_version(struct i2c_client *client, u16* version);
extern s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern s32 gup_update_proc(void *dir);
extern s32 init_wr_node(struct i2c_client *client);
extern s32 gtp_send_cfg(struct i2c_client *client);
extern int Read_TP_ID(void);
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
extern int gt928_FW_update_check(void);
extern void gtp_irq_disable(struct goodix_ts_data *ts);
extern void gtp_irq_enable(struct goodix_ts_data *ts);
extern void gtp_reset_guitar(struct i2c_client *client, s32 ms);
extern void gtp_esd_switch(struct i2c_client *client, s32 on);
extern void gt928_create_proc_diag_file(void);
extern void gt928_create_proc_test_file(void);
extern void gt928_remove_proc_diag_file(void);
extern void gt928_remove_proc_test_file(void);
extern void gt9xx_tp_enable(struct i2c_client * client);
extern void gt9xx_tp_disable(struct i2c_client *client);

/*******************************************************
	Attribute
*******************************************************/
static ssize_t gt928_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret = 0, status = 0;
	//ret = gtp_i2c_test(i2c_connect_client);
	if(ret < 0)
		status = 0;
	else
		status = 1;
	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}
DEVICE_ATTR(touch_status, (S_IWUSR | S_IRUGO), gt928_status, NULL);

static ssize_t gt928_display_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	gt928_get_display_id();

	return sprintf(buf, "%d\n", ts->DISPLAY_ID);
}
static DEVICE_ATTR(display_id, (S_IWUSR | S_IRUGO), gt928_display_id, NULL);

static ssize_t gt928_tp_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	gt928_get_tp_id(SELF);

	return sprintf(buf, "%d\n", ts->TP_ID);
}
static DEVICE_ATTR(tp_id, (S_IWUSR | S_IRUGO), gt928_tp_id, NULL);

static ssize_t gt928_get_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 version_info;
	int err = 0;
	//u8 checksum[10] = {0};

	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	err = gtp_read_version(ts->client, &version_info);
	if(err < 0)
	{
		GTP_ERROR("Read IC FW version failed.");
	}

	err = gt928_read_cfg_version(i2c_connect_client);
	if(err < 0)
	{
		GTP_ERROR("Read IC CFG version failed.");
	}
	/*
	//check chksum first byte
	checksum[0] = (0x80ff >> 8), checksum[1] = (0x80ff & 0xff);
	gtp_i2c_read(ts->client, checksum, sizeof(checksum));

	SET_INFO_LINE_INFO("CFG Checksum(0x80FF) = %x", checksum[2]); //printf first byte
	*/

	if(ts->fw_ver[5] == 0x00)
	{
		return sprintf(buf, "IC Version: %c%c%c_%02x%02x-%d\n", ts->fw_ver[2], ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[7], ts->fw_ver[6], ts->config_ver);
	}
	else
	{
		return sprintf(buf, "IC Version: %c%c%c%c_%02x%02x-%d\n", ts->fw_ver[2], ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[5], ts->fw_ver[7], ts->fw_ver[6], ts->config_ver);
	}
}
static DEVICE_ATTR(tp_fw_version, (S_IWUSR | S_IRUGO), gt928_get_fw_version, NULL);

static ssize_t gt928_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "gt928_debug = %d\n", gt928_debug);
}
static ssize_t gt928_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &gt928_debug);

	printk("////////////////////		%s: set gt928_debug = %d\n", __func__, gt928_debug);
	return count;
}
static DEVICE_ATTR(gt928_debug, (S_IWUSR | S_IRUGO), gt928_debug_show, gt928_debug_store);

static struct attribute *gt928_attributes[] =
{
	&dev_attr_touch_status.attr,
	&dev_attr_tp_id.attr,
	&dev_attr_display_id.attr,
	&dev_attr_tp_fw_version.attr,
	&dev_attr_gt928_debug.attr,
	NULL
};

const struct attribute_group gt928_attr_group =
{
	.attrs = gt928_attributes,
};

#if 0
/*******************************************************
	Function
*******************************************************/
int gt928_touch_update_progress(int update_progress)
{
	mm_segment_t oldfs;
	char Progress_file_path[] = "/data/touch_update_progress";
	struct file *filePtr = NULL;
	int len = 0;
	loff_t pos = 0;
	char temp_progress[3];

	if(ap_progress < update_progress || update_progress == 0)
	{
		GTP_INFO("%d.", update_progress);
		ap_progress = update_progress;
		sprintf(temp_progress, "%d\n", update_progress);
		//GTP_DEBUG("////////////////////		%s: write %d done.", __func__, update_progress);
		filePtr = filp_open(Progress_file_path, O_RDWR | O_CREAT, (S_IWUSR | S_IRUGO));
		if(!IS_ERR_OR_NULL(filePtr))
		{
			oldfs = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
			set_fs(oldfs);
			filp_close(filePtr, NULL);
			//GTP_DEBUG("%s: write %s done.", __func__, Progress_file_path);
			return 0;
		}
		else if(PTR_ERR(filePtr) == -ENOENT)
		{
			GTP_ERROR("%s: %s not found", __func__, Progress_file_path);
			return 1;
		}
		else
		{
			GTP_ERROR("%s: %s open error", __func__, Progress_file_path);
			return 1;
		}
	}
	return 0;
}
#endif

s32 gt928_send_cfg(struct i2c_client *client, u8 *iconfig)
{
	u8 *gt928_config;
	s32 ret = 2;
	s32 i = 0;
	s32 gt928_cfg_len = 0;
	//struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_INFO("send config by ourself.");
	gt928_cfg_len = GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH;
	gt928_config = (u8*)kzalloc(GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH, GFP_KERNEL);

	gt928_config[0] = GTP_REG_CONFIG_DATA >> 8;
	gt928_config[1] = GTP_REG_CONFIG_DATA & 0xff;

	for(i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		gt928_config[i + 2] = iconfig[i];
		printk("%02x   ", (gt928_config+2)[i]);
		if((i + 1) % 10 == 0)
		{
			printk("\n");
		}
	}
	printk("\n");

	i = 0;
	while(i ++ < 5)
	{
		ret = gup_i2c_write(client, gt928_config, gt928_cfg_len);
		if(ret > 0)
		{
			GTP_INFO("Send config SUCCESS by ourself.");
			break;
		}
		GTP_ERROR("Send config i2c error by ourself.");
	}

	msleep(100);
	if(gt928_debug)
		gt9xx_read_Config_Checksum();
	return ret;
}

s8 gt928_read_cfg_version(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	s8 ret = -1;

	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	while(retry++ < 5)
	{
		ret = gtp_i2c_read(client, test, 3);
		if(ret > 0)
		{
			ts->config_ver = test[2];
			GTP_INFO("IC Config Version: %d", ts->config_ver);
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
	}
	return ret;
}

int gt928_get_display_id(void)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	//ts->DISPLAY_ID = (gpio_get_value(DISPLAYID_GPIO) == 0 ? 0 : 1);
	GTP_INFO("%s: DISPLAY_ID = %d", __func__, ts->DISPLAY_ID);

	return 0;
}

int gt928_get_tp_id(int source)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	ts->TP_ID = 0;

	if(source == SELF)
	{
		#if 0
		GTP_INFO("%s: (TP_ID2 , TP_ID1) = (%d , %d)", __func__, gpio_get_value(TPID2_GPIO) == 0 ? 0 : 1, gpio_get_value(TPID1_GPIO) == 0 ? 0 : 1);
		if((!gpio_get_value(TPID2_GPIO)) && (!gpio_get_value(TPID1_GPIO))) // GIS Black TP, TP_ID(2,1)=(0,0)
		{
			ts->TP_ID = 0;
			GTP_INFO("%s: GIS Black TP Lens. (0, 0)", __func__);
		}
		else if((!gpio_get_value(TPID2_GPIO)) && (gpio_get_value(TPID1_GPIO))) // GIS White TP, TP_ID(2,1)=(0,1)
		{
			ts->TP_ID = 1;
			GTP_INFO("%s: GIS White TP Lens. (0, 1)", __func__);
		}
		else if((gpio_get_value(TPID2_GPIO)) && (!gpio_get_value(TPID1_GPIO))) // TopTouch White TP, TP_ID(2,1)=(1,0)
		{
			ts->TP_ID = 2;
			GTP_INFO("%s: TopTouch White TP Lens. (1, 0)", __func__);
		}
		else if((gpio_get_value(TPID2_GPIO)) && (gpio_get_value(TPID1_GPIO))) // TopTouch Black TP, TP_ID(2,1)=(1,1)
		{
			ts->TP_ID = 3;
			GTP_INFO("%s: TopTouch Black TP Lens. (1, 1)", __func__);
		}
		else
		{
			ts->TP_ID = 1; // default
			GTP_ERROR("%s: Unknow TP Lens. (%d , %d)", __func__, gpio_get_value(TPID2_GPIO) == 0 ? 0 : 1, gpio_get_value(TPID1_GPIO) == 0 ? 0 : 1);
		}
		#endif
	}
	else
	{
		ts->TP_ID = Read_TP_ID();
		if(ts->TP_ID == 0)
		{
			GTP_INFO("%s: GIS Black TP Lens. (0, 0)", __func__);
		}
		else if(ts->TP_ID == 1)
		{
			GTP_INFO("%s: GIS White TP Lens. (0, 1)", __func__);
		}
		else if(ts->TP_ID == 2)
		{
			GTP_INFO("%s: TopTouch White TP Lens. (1, 0)", __func__);
		}
		else if(ts->TP_ID == 3)
		{
			GTP_INFO("%s: TopTouch Black TP Lens. (1, 1)", __func__);
		}
		else
		{
			GTP_ERROR("%s: Unknow TP Lens. (ts->TP_ID = %d)", __func__, ts->TP_ID);
			GTP_ERROR("%s: Set Default TP ID = 1.", __func__);
			ts->TP_ID = 1;
		}
	}

	GTP_INFO("%s: ts->TP_ID = %d", __func__, ts->TP_ID);
	return ts->TP_ID;
}

// add by leo for cable status notifier ++
#if DRIVER_SEND_AC_CONFIG
int gt928_cable_status_handler(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	switch(event)
	{
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			GTP_INFO("%s: TYPE_NONE", __func__);
			ts->ac_in = 0;
			break;

		// charger
		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			GTP_INFO("%s: TYPE_USB_SDP", __func__);
			ts->ac_in = 1;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
			GTP_INFO("%s: TYPE_USB_DCP", __func__);
			ts->ac_in = 1;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			GTP_INFO("%s: TYPE_USB_CDP", __func__);
			ts->ac_in = 1;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
			GTP_INFO("%s: TYPE_USB_ACA", __func__);
			ts->ac_in = 1;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_AC:
			GTP_INFO("%s: TYPE_AC", __func__);
			ts->ac_in = 1;
			break;

		case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
			GTP_INFO("%s: ACA_DOCK", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_ACA_A:
			GTP_INFO("%s: ACA_A", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_ACA_B:
			GTP_INFO("%s: ACA_B", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_ACA_C:
			GTP_INFO("%s: ACA_C", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_SE1:
			GTP_INFO("%s: SE1", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_MHL:
			GTP_INFO("%s: MHL", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_B_DEVICE:
			GTP_INFO("%s: B_DEVICE", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_OTG:
			GTP_INFO("%s: OTG", __func__);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_OTG_OUT:
			GTP_INFO("%s: OTG_OUT", __func__);
			break;
		default:
			GTP_INFO("%s: wrong cable type ...", __func__);
			break;
	}

	//if(ts->ac_in!=ac_old_status)
	//{
	//    GTP_INFO("%s: AC status changed, call gt928_send_ac_config_work",__func__);
	queue_delayed_work(gt928_send_ac_config_workqueue, &gt928_send_ac_config_work, 0.1 * HZ);
	//    ac_old_status = ts->ac_in;
	//}
	//else
	//GTP_INFO("%s: skip gt928_send_ac_config_work due to ac_in = %d.",__func__, ts->ac_in);

	return NOTIFY_OK;
}

struct notifier_block gt928_cable_status_notifier =
{
	.notifier_call = gt928_cable_status_handler,
};
#endif
// add by leo for cable status notifier --

int gt928_FW_check(void)
{
	//int ret = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GT928_CFGCheckResult = NO_NEED_TO_UPDATE;
	GT928_FWCheckResult = NO_NEED_TO_UPDATE;
	GT928_FWChecking = 1;

#if GT928_FW_MAPPING_IMAGE
	GTP_INFO("%s: ts->fw_ver[7] = %02x (%d), ts->fw[6] = %02x (%d).", __func__, ts->fw_ver[7], ts->fw_ver[7], ts->fw_ver[6], ts->fw_ver[6]);
	GTP_INFO("%s: ts->config_ver = %d.", __func__, ts->config_ver);
	if(build_version == 1)
	{
		GTP_INFO("%s: FW version should be %s", __func__, "928_1060-68");
		if((ts->fw_ver[2] == '9') && (ts->fw_ver[3] == '2') && (ts->fw_ver[4] == '8') && (ts->fw_ver[7] == 16) && (ts->fw_ver[6] == 96))  // fw version
			GT928_FWCheckResult = NO_NEED_TO_UPDATE;
		else
			GT928_FWCheckResult = NEED_TO_UPDATE;

		if(ts->config_ver == 68) // config version
			GT928_CFGCheckResult = NO_NEED_TO_UPDATE;
		else
			GT928_CFGCheckResult = NEED_TO_UPDATE;
	}
	else
	{
		GTP_INFO("%s: FW version should be %s", __func__, "928_1080-86");
		if((ts->fw_ver[2] == '9') && (ts->fw_ver[3] == '2') && (ts->fw_ver[4] == '8') && (ts->fw_ver[7] == 16) && (ts->fw_ver[6] == 128))  // fw version
			GT928_FWCheckResult = NO_NEED_TO_UPDATE;
		else
			GT928_FWCheckResult = NEED_TO_UPDATE;

		if(ts->config_ver == 86) // config version
			GT928_CFGCheckResult = NO_NEED_TO_UPDATE;
		else
			GT928_CFGCheckResult = NEED_TO_UPDATE;
	}
#else
	if(!GT928_IsChecked)
	{
		ret = gup_update_proc(NULL);
	}
	else
	{
		test_result = gup_init_update_proc(ts);
		if(test_result < 0)
		{
			GTP_ERROR("%s: Create update thread error.", __func__);
		}
		msleep(5000); // 5 sec
	}
	GT928_IsChecked = true;
#endif

	GTP_INFO("%s: ////////////////////		Config %s", __func__, (GT928_CFGCheckResult == NEED_TO_UPDATE) ? "need to update" : "no need to update");
	GTP_INFO("%s: ////////////////////		FW %s", __func__, (GT928_FWCheckResult == NEED_TO_UPDATE) ? "need to update" : "no need to update");

	GT928_FWChecking = 0;

	if((GT928_CFGCheckResult == NEED_TO_UPDATE) || (GT928_FWCheckResult == NEED_TO_UPDATE))
		return NEED_TO_UPDATE;
	else
		return NO_NEED_TO_UPDATE;
}

static int gt928_FW_update(void)
{
	u8 config_clean[] = CTP_CFG_CLEAN;
	//int check_reuslt = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GT928_CFGUpdateResult = SUCCESS;
	GT928_FWUpdateResult = SUCCESS;

	//gt928_touch_update_progress(0); // add by leo
	/*
	ts->check_result = gt928_FW_check();
	if(ts->check_result == NO_NEED_TO_UPDATE)
	{
		//TODO
	}
	*/

#if GT928_FW_MAPPING_IMAGE
	gt928_send_cfg(ts->client, config_clean);
	msleep(100);
#endif

	//gt928_touch_update_progress(10); // add by leo

	GT928_FWChecking = 0;

	test_result = gup_init_update_proc(ts);
	if(test_result < 0)
	{
		GTP_ERROR("%s: Create update thread error.", __func__);
	}

	//if(((!GT928_FWChecking) && (GT928_FWCheckResult)) || GT928_FWForceUpdate)gt928_touch_update_progress(90); // add by leo

	//msleep(10000);

	//if(((!GT928_FWChecking) && (GT928_FWCheckResult)) || GT928_FWForceUpdate)gt928_touch_update_progress(100); // add by leo

	GTP_INFO("%s:GT928_CFGUpdateResult (%s).", __func__, (GT928_CFGUpdateResult == SUCCESS) ? "SUCCES" : "FAIL");
	GTP_INFO("%s:GT928_FWUpdateResult (%s).", __func__, (GT928_FWUpdateResult == SUCCESS) ? "SUCCES" : "FAIL");
	// add by leo for AC config ++
#if DRIVER_SEND_AC_CONFIG
	ts->skip_ac_config = 0;
	GTP_INFO("%s: resend AC config after FW update (ts->ac_in=%d, ts->skip_ac_config=%d).", __func__, ts->ac_in, ts->skip_ac_config);
	queue_delayed_work(gt928_send_ac_config_workqueue, &gt928_send_ac_config_work, 1.5 * HZ);
#endif
	// add by leo for AC config --
	if(GT928_FWUpdateResult && GT928_CFGUpdateResult)
		return SUCCESS;
	else
		return FAIL;
}
#if 0
int gt928_leave_FW_update_mode(void)
{
	u8 sensor_id = 0;
	u16 version_info;
	int ret = 0, retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	while(retry++ < 3)
	{
		//GTP_INFO("retry %d times", retry);
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
		if(SUCCESS == ret)
		{
			if(sensor_id >= 0x06)
			{
				GTP_ERROR("Wrong sensor_id(0x%02X)! retry %d times", sensor_id, retry);
				GTP_GPIO_OUTPUT(TPID1_GPIO, 0);
				msleep(30);
				gtp_reset_guitar(ts->client, 20);
				//ts->pnl_init_error = 0;
			}
			else
			{
				GTP_DEBUG("Current sensor_id(0x%02X)!", sensor_id);
				break;
			}
		}
		else
		{
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			ts->pnl_init_error = 1;
			return -1;
		}
	}

	return 0;
}
#endif
/*******************************************************
	Touch Switch Device
*******************************************************/
ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	if(ts->fw_ver[5] == 0x00)
	{
		return sprintf(buf, "%c%c%c_%02x%02x-%d\n", ts->fw_ver[2], ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[7], ts->fw_ver[6], ts->config_ver);
	}
	else
	{
		return sprintf(buf, "%c%c%c%c_%02x%02x-%d\n", ts->fw_ver[2], ts->fw_ver[3], ts->fw_ver[4], ts->fw_ver[5], ts->fw_ver[7], ts->fw_ver[6], ts->config_ver);
	}
}
ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	int ret = 0, status = 0;

	//ret = gtp_i2c_test(i2c_connect_client);
	if(ret < 0)
		status = 0;
	else
		status = 1;
	return sprintf(buf, "%d\n", status);
}

/*******************************************************
	I/O Control
*******************************************************/
static int gt928_open(struct inode *inode, struct file *file)
{
	GTP_INFO("%s:[%d]: ++", __func__, __LINE__);
	return nonseekable_open(inode, file);
}

static int gt928_release(struct inode *inode, struct file *file)
{
	GTP_INFO("%s:[%d]: ++", __func__, __LINE__);
	return 0;
}

static long gt928_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, val = 0, update_result = 0;
	//struct task_struct *thread = NULL;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GTP_INFO("%s:[%d]: cmd = 0x%x.", __func__, __LINE__, (u32)cmd);
	ap_progress = 0;

	if(_IOC_TYPE(cmd) != TOUCH_IOCTL_MAGIC)
	{
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		ret =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if(ret)
	{
		return -EFAULT;
	}

	switch(cmd)
	{

		case TOUCH_IOCTL_FW_CHECK:

			GTP_INFO("%s:[%d]: TOUCH_IOCTL_FW_CHECK.", __func__, __LINE__);

			//check_reuslt = gt928_FW_check();

			GTP_INFO("%s:[%d]: ts->check_result = %s", __func__, __LINE__, (ts->check_result == NEED_TO_UPDATE) ? "NEED_TO_UPDATE" : "NO_NEED_TO_UPDATE");
			if(ts->check_result == NEED_TO_UPDATE)
				return put_user(0, (unsigned long __user *)arg);
			else
				return put_user(1, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_FW_UPDATE:

			GTP_INFO("%s:[%d]: TOUCH_IOCTL_FW_UPDATE.", __func__, __LINE__);

#if GT928_FW_MAPPING_IMAGE
			GT928_FWForceUpdate = 1;
#endif
			update_result = gt928_FW_update();
#if GT928_FW_MAPPING_IMAGE
			GT928_FWForceUpdate = 0;
#endif

			if(update_result == SUCCESS)
				return put_user(0, (unsigned long __user *)arg);
			else
				return put_user(1, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_FORCE_FW_UPDATE:

			GTP_INFO("%s:[%d]: TOUCH_IOCTL_FORCE_FW_UPDATE.", __func__, __LINE__);

			GT928_FWForceUpdate = 1;
			update_result = gt928_FW_update();
			GTP_INFO("%s:[%d]: GT928 FW update %s", __func__, __LINE__, (update_result == SUCCESS) ? "SUCCESS" : "FAIL");
			GT928_FWForceUpdate = 0;

			if(update_result == SUCCESS)
				return put_user(0, (unsigned long __user *)arg);
			else
				return put_user(1, (unsigned long __user *)arg);
			break;

		case TOUCH_IOCTL_READ_CMD:

			GTP_INFO("%s:[%d]: TOUCH_IOCTL_READ_CMD.", __func__, __LINE__);

			break;

		case TOUCH_IOCTL_WRITE_CMD:

			GTP_INFO("%s:[%d]: TOUCH_IOCTL_WRITE_CMD.", __func__, __LINE__);

			if(get_user(val, (unsigned long __user *)arg))
				return -EFAULT;

			switch(val)
			{
				case 0:
				case 1:
					gt928_debug = val;
					GTP_INFO("%s: Debug log is %s\n", __func__, (gt928_debug == 1) ? "opened" : "closed");
					break;

				default:
					GTP_ERROR("%s:[%d]: incorrect val (%d).", __func__, __LINE__, val);
			}
			break;

		case TOUCH_IOCTL_GESTURE_CMD:
			break;

		default:
			GTP_ERROR("%s:[%d]: incorrect cmd (%d).", __func__, __LINE__, _IOC_NR(cmd));
			return -EINVAL;
	}

	return 0;
}

static struct file_operations gt928_fops =
{
	.owner = THIS_MODULE,
	.open = gt928_open,
	.release = gt928_release,
	//.unlocked_ioctl = gt928_ioctl,
	.compat_ioctl = gt928_ioctl
};

struct miscdevice gt928_misc_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = TOUCH_MDEV_NAME,
	.fops = &gt928_fops,
};

/*******************************************************
	Proc
*******************************************************/
// add by leo for gesture wake up ++
static ssize_t gt928_proc_gesture_read(struct seq_file *buf, void *v)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GTP_INFO("%s: ts->gesture = 0x%08x", __func__, ts->gesture);

	seq_printf(buf, "ts->gesture = 0x%08x\n", ts->gesture);
	return 0;
}

static ssize_t gt928_proc_gesture_write(struct file *filp, const char *buf, size_t len, loff_t *data)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GTP_INFO("%s: GESTURE_DOUBLE_CLICK = %c, GESTURE_ENABLE = %c, GESTURE_W = %c, GESTURE_S = %c, GESTURE_E = %c, GESTURE_C = %c, GESTURE_Z = %c, GESTURE_V = %c",
	         __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

	if((buf[0] == '0') && (buf[1] == '0'))
	{
		ts->gesture = (ts->gesture & (~GESTURE_ENABLE));
		GTP_INFO("%s: Gesture Disable (ts->gesture = 0x%08x)", __func__, ts->gesture);
	}
	else
	{
		if(buf[0] != '0')  // double click
			ts->gesture = (ts->gesture | GESTURE_DOUBLE_CLICK);
		else
			ts->gesture = (ts->gesture & (~GESTURE_DOUBLE_CLICK));

		if(buf[2] != '0') // W
			ts->gesture = (ts->gesture | GESTURE_W);
		else
			ts->gesture = (ts->gesture & (~GESTURE_W));

		if(buf[3] != '0') // S
			ts->gesture = (ts->gesture | GESTURE_S);
		else
			ts->gesture = (ts->gesture & (~GESTURE_S));

		if(buf[4] != '0') // e
			ts->gesture = (ts->gesture | GESTURE_E);
		else
			ts->gesture = (ts->gesture & (~GESTURE_E));

		if(buf[5] != '0') // C
			ts->gesture = (ts->gesture | GESTURE_C);
		else
			ts->gesture = (ts->gesture & (~GESTURE_C));

		if(buf[6] != '0') // Z
			ts->gesture = (ts->gesture | GESTURE_Z);
		else
			ts->gesture = (ts->gesture & (~GESTURE_Z));

		if(buf[7] != '0') // V
			ts->gesture = (ts->gesture | GESTURE_V);
		else
			ts->gesture = (ts->gesture & (~GESTURE_V));

		ts->gesture = (ts->gesture | GESTURE_ENABLE);
		GTP_INFO("%s: Gesture Enable (ts->gesture = 0x%08x)", __func__, ts->gesture);
	}

	/*
	if(buf[1] != '0')
	{
		if(buf[0] != '0')  // double click
			ts->gesture = (ts->gesture | GESTURE_DOUBLE_CLICK);
		else
			ts->gesture = (ts->gesture & (~GESTURE_DOUBLE_CLICK));

		if(buf[2] != '0') // W
			ts->gesture = (ts->gesture | GESTURE_W);
		else
			ts->gesture = (ts->gesture & (~GESTURE_W));

		if(buf[3] != '0') // S
			ts->gesture = (ts->gesture | GESTURE_O);
		else
			ts->gesture = (ts->gesture & (~GESTURE_O));

		if(buf[4] != '0') // e
			ts->gesture = (ts->gesture | GESTURE_E);
		else
			ts->gesture = (ts->gesture & (~GESTURE_E));

		if(buf[5] != '0') // C
			ts->gesture = (ts->gesture | GESTURE_C);
		else
			ts->gesture = (ts->gesture & (~GESTURE_C));

		if(buf[6] != '0') // Z
			ts->gesture = (ts->gesture | GESTURE_Z);
		else
			ts->gesture = (ts->gesture & (~GESTURE_Z));

		if(buf[7] != '0') // V
			ts->gesture = (ts->gesture | GESTURE_V);
		else
			ts->gesture = (ts->gesture & (~GESTURE_V));

		ts->gesture = (ts->gesture | GESTURE_ENABLE);
		GTP_INFO("%s: Gesture Enable (ts->gesture = 0x%08x)", __func__, ts->gesture);
	}
	else
	{
		ts->gesture = (ts->gesture & (~GESTURE_ENABLE));
		GTP_INFO("%s: Gesture Disable (ts->gesture = 0x%08x)", __func__, ts->gesture);
	}
	*/
	return len;
}

static int gt928_proc_gesture_open(struct inode *inode, struct  file *file)
{
	return single_open(file, gt928_proc_gesture_read, NULL);
}

static const struct file_operations gt928_gesture_fops =
{
	.owner = THIS_MODULE,
	.open = gt928_proc_gesture_open,
	.read = seq_read,
	.write = gt928_proc_gesture_write,
};

void gt928_create_proc_gesture_file(void)
{
	gt928_proc_gesture_file = proc_create(GT928_PROC_GESTURE_FILE, 0666, NULL, &gt928_gesture_fops);
	if(gt928_proc_gesture_file)
	{
		GTP_INFO("%s:[%d]: proc gesture file create sucessed", __func__, __LINE__);
	}
	else
	{
		GTP_ERROR("%s:[%d]: proc gesture file create failed", __func__, __LINE__);
	}
	return;
}

void gt928_remove_proc_gesture_file(void)
{
	extern struct proc_dir_entry proc_root;
	GTP_INFO("%s:[%d]: proc gesture file removed.", __func__, __LINE__);
	remove_proc_entry(GT928_PROC_GESTURE_FILE, &proc_root);
	return;
}
// add by leo for gesture wake up --


// add by leo for log tool ++
static ssize_t gt928_proc_tp_debug_read(struct seq_file *buf, void *v)
{
	GTP_INFO("%s: gt928_debug = %d", __func__, gt928_debug);
	seq_printf(buf, "Debug log is %s\n", (gt928_debug == 1) ? "opened" : "closed");

	return 0;
}

static ssize_t gt928_proc_tp_debug_write(struct file *filp, const char *buf, size_t len, loff_t *data)
{
	debug_function = DEBUG_FLAG;
	GTP_INFO("%s: debug_function = DEBUG_FLAG", __func__);

	if(buf[0] == '1')
	{
		gt928_debug = 1;
	}
	else if(buf[0] == '0')
	{
		gt928_debug = 0;
	}
	return len;
}

static int gt928_proc_tp_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, gt928_proc_tp_debug_read, NULL);
}

static const struct file_operations gt928_tp_debug_fops =
{
	.owner = THIS_MODULE,
	.open = gt928_proc_tp_debug_open,
	.read = seq_read,
	.write = gt928_proc_tp_debug_write,
};

void gt928_create_proc_tp_debug_file(void)
{
	gt928_proc_tp_debug_file = proc_create(GT928_PROC_TP_DEBUG_FILE, 0666, NULL, &gt928_tp_debug_fops);
	if(gt928_proc_tp_debug_file)
	{
		GTP_INFO("%s:[%d]: proc TP debug file create sucessed", __func__, __LINE__);
	}
	else
	{
		GTP_ERROR("%s:[%d]: proc TP debug file create failed", __func__, __LINE__);
	}
	return;
}

void gt928_remove_proc_tp_debug_file(void)
{
	extern struct proc_dir_entry proc_root;
	GTP_INFO("%s:[%d]: proc TP debug file removed.", __func__, __LINE__);
	remove_proc_entry(GT928_PROC_TP_DEBUG_FILE, &proc_root);
	return;
}
// add by leo for log tool --

static ssize_t gt928_proc_debug_read(struct seq_file *buf, void *v)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	switch(debug_function)
	{
		case FW_CHECK:
			GTP_INFO("%s: test_result = %d", __func__, test_result);
			//GTP_INFO("%s: gt928_fw_update = %d", __func__, gt928_fw_update);
			//seq_printf(buf, "%s\n", __func__, (gt928_fw_update==SUCCESS)?"SUCCES":"FAIL");
			break;

		case FW_UPDATE:
			GTP_INFO("%s: test_result = %d", __func__, test_result);
			GTP_INFO("%s: GT928_FWUpdateResult = %d", __func__, GT928_FWUpdateResult);
			seq_printf(buf, "%s\n", (GT928_FWUpdateResult == SUCCESS) ? "SUCCES" : "FAIL");
			break;

		case DEBUG_FLAG:
			GTP_INFO("%s: gt928_debug = %d", __func__, gt928_debug);
			seq_printf(buf, "Debug log is %s\n", (gt928_debug == 1) ? "opened" : "closed");
			break;

		case INTR_PIN:
			seq_printf(buf, "INTR_PIN is %s\n", (gpio_get_value(ts->pdata->irq_gpio) == 0) ? "High" : "Low");
			break;

		case RST_PIN:
			seq_printf(buf, "RST_PIN is %s\n", (gpio_get_value(ts->pdata->reset_gpio) == 0) ? "High" : "Low");
			break;

		case IRQ_ENABLE:
			GTP_INFO("%s: ts->irq_is_disable = %d", __func__, ts->irq_is_disable);
			seq_printf(buf, "IRQ is %s\n", (ts->irq_is_disable == 1) ? "disable" : "enable");
			break;

		case IC_RESET:
			break;

		case WORK_FUNC:
			break;

		case ESD_ENABLE:
#if GTP_ESD_PROTECT
			GTP_INFO("%s: ts->clk_tick_cnt = %d", __func__, ts->clk_tick_cnt);
			GTP_INFO("%s: ts->esd_running = %d", __func__, ts->esd_running);
			seq_printf(buf, "ESD is %s, cycle time is %d sec\n", (ts->esd_running == 1) ? "disable" : "enable", ts->clk_tick_cnt / 100);
#endif
			break;

		case TP_ID_PIN:
			seq_printf(buf, "TPID1_PIN is %s\n", (gpio_get_value(ts->pdata->reset_gpio) == 0) ? "High" : "Low");
			break;

		case READ_TP_ID:
			seq_printf(buf, "TP ID = %d\n", ts->TP_ID);
			break;

		case WR_NODE:
			break;

		case CLEAN_CFG:
			break;

		case GESTURE_FUNC:
			seq_printf(buf, "ts->gesture = 0x%08x\n", ts->gesture);
			break;

		default:
			return -EINVAL;
	}
	return 0;
}

static ssize_t gt928_proc_debug_write(struct file *filp, const char *buf, size_t len, loff_t *data)
{
	u8 config_clean[] = CTP_CFG_CLEAN;
	int ret = 0, pin = 0;
	//struct task_struct *thread = NULL;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	GTP_INFO("%s: debug comamnd:", __func__);
	ap_progress = 0;

	//FW CHECK
	if((buf[0] == 'f') && (buf[1] == 'w') && (buf[2] == 'c') && (buf[3] == 'h') && (buf[4] == 'e') && (buf[5] == 'c') && (buf[6] == 'k'))
	{
		debug_function = FW_CHECK;
		GTP_INFO("%s: debug_function = FW_CHECK", __func__);
#if 0
		thread = kthread_run(gt928_version_compare, (void*)NULL, "gt928_touch_fw_check");
		if(IS_ERR(thread))
		{
			GTP_ERROR("%s: Failed to create update thread.", __func__);
		}
#endif
		ts->check_result = gt928_FW_check();
		if(ts->check_result == NEED_TO_UPDATE)
			GTP_INFO("GT928%s%s need to update", (GT928_CFGCheckResult == NEED_TO_UPDATE) ? " CFG" : "", (GT928_FWCheckResult == NEED_TO_UPDATE) ? " FW" : "");
		else
			GTP_INFO("GT928 no need to update");

		goto proc_debug_write_end;
	}

	//FW UPDATE
	if((buf[0] == 'f') && (buf[1] == 'w') && (buf[2] == 'u') && (buf[3] == 'p') && (buf[4] == 'd') && (buf[5] == 'a') && (buf[6] == 't') && (buf[7] == 'e'))
	{
		debug_function = FW_UPDATE;
		GTP_INFO("%s:[%d]: debug_function = FW_UPDATE", __func__, __LINE__);

		if(buf[9] == '1')
		{
			GTP_INFO("%s:[%d]: Force FW Update !!", __func__, __LINE__);
			GT928_FWForceUpdate = 1;
		}

		ret = gt928_FW_update();
		GTP_INFO("%s:[%d]: GT928 FW update %s", __func__, __LINE__, (ret == SUCCESS) ? "SUCCESS" : "FAIL");
		GT928_FWForceUpdate = 0;

		goto proc_debug_write_end;
	}

	// DEBUG
	if((buf[0] == 'd') && (buf[1] == 'e') && (buf[2] == 'b') && (buf[3] == 'u') && (buf[4] == 'g'))
	{
		debug_function = DEBUG_FLAG;
		GTP_INFO("%s: debug_function = DEBUG_FLAG", __func__);

		if(buf[6] == '1')
		{
			GTP_INFO("%s: debug log ON", __func__);
			gt928_debug = 1;
		}
		else if(buf[6] == '0')
		{
			GTP_INFO("%s: debug log OFF", __func__);
			gt928_debug = 0;
		}

		goto proc_debug_write_end;
	}

	//INTEERUPT PIN
	if((buf[0] == 'i') && (buf[1] == 'n') && (buf[2] == 't') && (buf[3] == 'r') && (buf[4] == 'p') && (buf[5] == 'i') && (buf[6] == 'n'))
	{
		debug_function = INTR_PIN;
		GTP_INFO("%s: debug_function = INTR_PIN", __func__);

		if(buf[8] == '0')
		{
			GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 0);
			GTP_INFO("%s: Set GPIO INT to output low", __func__);
		}
		else if(buf[8] == '1')
		{
			GTP_GPIO_OUTPUT(ts->pdata->irq_gpio, 1);
			GTP_INFO("%s: Set GPIO INT to output high", __func__);
		}
		else if(buf[8] == '2')
		{
			GTP_GPIO_AS_INPUT(ts->pdata->irq_gpio);
			GTP_INFO("%s: Set GPIO INT to input", __func__);
		}

		goto proc_debug_write_end;
	}

	// RESET PIN
	if((buf[0] == 'r') && (buf[1] == 's') && (buf[2] == 't') && (buf[3] == 'p') && (buf[4] == 'i') && (buf[5] == 'n'))
	{
		debug_function = RST_PIN;
		GTP_INFO("%s: debug_function = RST_PIN", __func__);

		if(buf[7] == '0')
		{
			GTP_GPIO_OUTPUT(ts->pdata->reset_gpio, 0);
			GTP_INFO("%s: Set GPIO RST to output low", __func__);
		}
		else if(buf[7] == '1')
		{
			GTP_GPIO_OUTPUT(ts->pdata->reset_gpio, 1);
			GTP_INFO("%s: Set GPIO RST to output high", __func__);
		}

		goto proc_debug_write_end;
	}

	// IRQ ENABLE
	if((buf[0] == 'i') && (buf[1] == 'r') && (buf[2] == 'q'))
	{
		debug_function = IRQ_ENABLE;
		GTP_INFO("%s: ts->irq_is_disable = IRQ_ENABLE", __func__);

		if(buf[4] == '0')
		{
			gtp_irq_disable(ts);
			GTP_INFO("%s: ts->irq_is_disable = %d", __func__, ts->irq_is_disable);
		}
		else if(buf[4] == '1')
		{
			gtp_irq_enable(ts);
			GTP_INFO("%s: ts->irq_is_disable = %d", __func__, ts->irq_is_disable);
		}

		goto proc_debug_write_end;
	}

	// IC REST
	if((buf[0] == 'r') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 'e') && (buf[4] == 't'))
	{
		debug_function = IC_RESET;
		GTP_INFO("%s: ts->irq_is_disable = IC_RESET", __func__);
		gtp_reset_guitar(ts->client, 50);

		goto proc_debug_write_end;
	}

	// TRIGGER WORK FUNCTION
	if((buf[0] == 'w') && (buf[1] == 'o') && (buf[2] == 'r') && (buf[3] == 'k'))
	{
		debug_function = WORK_FUNC;
		GTP_INFO("%s: ts->irq_is_disable = WORK_FUNC", __func__);
		queue_work(goodix_wq, &ts->work);

		goto proc_debug_write_end;
	}

#if GTP_ESD_PROTECT
	// ESD ENABLE
	if((buf[0] == 'e') && (buf[1] == 's') && (buf[2] == 'd'))
	{
		debug_function = ESD_ENABLE;
		GTP_INFO("%s: debug_function = ESD_ENABLE", __func__);

		if(buf[4] == '1')
		{
			gtp_esd_switch(ts->client, SWITCH_ON);
			GTP_INFO("%s: ts->esd_running = %d", __func__, ts->esd_running);
		}
		else if(buf[4] == '0')
		{
			gtp_esd_switch(ts->client, SWITCH_OFF);
			GTP_INFO("%s: ts->esd_running = %d", __func__, ts->esd_running);
		}

		goto proc_debug_write_end;
	}
#endif

#if DRIVER_SEND_AC_CONFIG
	//AC IN TEST
	if((buf[0] == 'a') && (buf[1] == 'c'))
	{
		if(buf[3] == '0')
		{
			ts->ac_in = 0;
			ac_old_status = -1;
			GTP_INFO("%s: ts->ac_in = 0", __func__);
		}
		else if(buf[3] == '1')
		{
			ts->ac_in = 1;
			ac_old_status = -1;
			GTP_INFO("%s: ts->ac_in = 1", __func__);
		}
		queue_delayed_work(gt928_send_ac_config_workqueue, &gt928_send_ac_config_work, 0.1 * HZ);

		goto proc_debug_write_end;
	}

	if((buf[0] == 'c') && (buf[1] == 'h') && (buf[2] == 'e') && (buf[3] == 'c') && (buf[4] == 'k') && (buf[5] == 's') && (buf[6] == 'u') && (buf[7] == 'm'))
	{
		gt9xx_read_Config_Checksum();
		goto proc_debug_write_end;
	}
#endif

	// TP ID PIN
	if((buf[0] == 't') && (buf[1] == 'p') && (buf[2] == 'i') && (buf[3] == 'd') && (buf[4] == 'p') && (buf[5] == 'i') && (buf[6] == 'n'))
	{
		debug_function = TP_ID_PIN;
		GTP_INFO("%s: debug_function = TP_ID_PIN", __func__);

		if(buf[8] == '0')
		{
			//pin = TPID0_GPIO;
			GTP_INFO("%s: TPID 0", __func__);
		}
		else if(buf[8] == '1')
		{
			//pin = TPID1_GPIO;
			GTP_INFO("%s: TPID 1", __func__);
		}
		else if(buf[8] == '2')
		{
			//pin = TPID2_GPIO;
			GTP_INFO("%s: TPID 2", __func__);
		}

		if(buf[10] == '0')
		{
			GTP_GPIO_OUTPUT(pin, 0);
			GTP_INFO("%s: Set GPIO to output low", __func__);
		}
		else if(buf[10] == '1')
		{
			GTP_GPIO_OUTPUT(pin, 1);
			GTP_INFO("%s: Set GPIO to output high", __func__);
		}
		else if(buf[10] == '2')
		{
			GTP_GPIO_AS_INPUT(pin);
			GTP_INFO("%s: Set GPIO to input", __func__);
		}

		goto proc_debug_write_end;
	}

	// READ DISPLAY ID
	if((buf[0] == 'd') && (buf[1] == 'i') && (buf[2] == 's') && (buf[3] == 'p') && (buf[4] == 'l') && (buf[5] == 'a') && (buf[6] == 'y') && (buf[7] == 'i') && (buf[8] == 'd'))
	{
		debug_function = READ_DISPLAY_ID;
		GTP_INFO("%s: debug_function = READ_DISPLAY_ID", __func__);

		//ts->DISPLAY_ID = (gpio_get_value(DISPLAYID_GPIO) == 0 ? 0 : 1);
		GTP_INFO("%s: DISPLAY_ID = %d", __func__, ts->DISPLAY_ID);

		goto proc_debug_write_end;
	}

	// READ TP ID
	if((buf[0] == 't') && (buf[1] == 'p') && (buf[2] == 'i') && (buf[3] == 'd'))
	{
		debug_function = READ_TP_ID;
		GTP_INFO("%s: debug_function = READ_TP_ID", __func__);

		if(buf[5] == '1')
		{
			//GTP_INFO("%s: TPID2_GPIO = %d", __func__, gpio_get_value(TPID2_GPIO));
			//GTP_INFO("%s: TPID1_GPIO = %d", __func__, gpio_get_value(TPID1_GPIO));
			//GTP_INFO("%s: TPID0_GPIO = %d",__func__,gpio_get_value(TPID0_GPIO));

			gt928_get_tp_id(SELF);
		}
		/*
		else if(buf[5] == '2')
		{
			GTP_GPIO_AS_INT(TPID1_GPIO);
			msleep(5);

			GTP_INFO("%s: TPID2_GPIO = %d",__func__,gpio_get_value(TPID2_GPIO));
			GTP_INFO("%s: TPID1_GPIO = %d",__func__,gpio_get_value(TPID1_GPIO));
			//GTP_INFO("%s: TPID0_GPIO = %d",__func__,gpio_get_value(TPID0_GPIO));

			gt928_get_tp_id(SELF);
			GTP_GPIO_OUTPUT(TPID1_GPIO, 0);
		}
		*/
		else
		{
			gt928_get_tp_id(SYSTEM);
		}

		goto proc_debug_write_end;
	}

	// CREATE WR NODE
	if((buf[0] == 'g') && (buf[1] == 'm') && (buf[2] == 'n') && (buf[3] == 'o') && (buf[4] == 'd') && (buf[5] == 'e'))
	{
		debug_function = WR_NODE;
		GTP_INFO("%s: debug_function = WR_NODE", __func__);
		init_wr_node(i2c_connect_client);

		goto proc_debug_write_end;
	}

	// CLEAN CONFIG VERSION
	if((buf[0] == 'c') && (buf[1] == 'l') && (buf[2] == 'e') && (buf[3] == 'a') && (buf[4] == 'n'))
	{
		debug_function = CLEAN_CFG;
		GTP_INFO("%s: debug_function = CLEAN_CFG", __func__);
		gt928_send_cfg(ts->client, config_clean);
		GTP_INFO("%s: ////////////////		gt928_send_cfg finished", __func__);

		goto proc_debug_write_end;
	}

	// GESTURE WAKE UP FUNCTIONS
	if((buf[0] == 'g') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 't') && (buf[4] == 'u') && (buf[5] == 'r') && (buf[6] == 'e'))
	{
		debug_function = GESTURE_FUNC;
		GTP_INFO("%s: debug_function = GESTURE_FUNC", __func__);
		if(buf[8] == '0')
		{
			ts->gesture = (ts->gesture & (~GESTURE_ENABLE));
			GTP_INFO("%s: Gesture Disable (ts->gesture = 0x%08x)", __func__, ts->gesture);
			goto proc_debug_write_end;
		}
		else if(buf[8] == '1')
		{
			ts->gesture = (ts->gesture | GESTURE_ENABLE);
			GTP_INFO("%s: Gesture Enable (ts->gesture = 0x%08x)", __func__, ts->gesture);
			goto proc_debug_write_end;
		}
		else if((buf[8] == 'u') && (buf[9] == 'p')) // up
		{
			if(buf[11] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_UP));
			else if(buf[11] == '1')
				ts->gesture = (ts->gesture | GESTURE_UP);
		}
		else if((buf[8] == 'd') && (buf[9] == 'o') && (buf[10] == 'w') && (buf[11] == 'n')) // down
		{
			if(buf[13] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_DOWN));
			else if(buf[13] == '1')
				ts->gesture = (ts->gesture | GESTURE_DOWN);
		}
		else if((buf[8] == 'l') && (buf[9] == 'e') && (buf[10] == 'f') && (buf[11] == 't')) // left
		{
			if(buf[13] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_LEFT));
			else if(buf[13] == '1')
				ts->gesture = (ts->gesture | GESTURE_LEFT);
		}
		else if((buf[8] == 'r') && (buf[9] == 'i') && (buf[10] == 'g') && (buf[11] == 'h') && (buf[12] == 't')) // right
		{
			if(buf[14] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_RIGHT));
			else if(buf[14] == '1')
				ts->gesture = (ts->gesture | GESTURE_RIGHT);
		}
		else if(buf[8] == 'w') // w
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_W));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_W);
		}
		else if(buf[8] == 'o') // o
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_O));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_O);
		}
		else if(buf[8] == 'm') // m
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_M));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_M);
		}
		else if(buf[8] == 'e') // e
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_E));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_E);
		}
		else if(buf[8] == 'c') //c
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_C));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_C);
		}
		else if(buf[8] == 'z') // z
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_Z));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_Z);
		}
		else if(buf[8] == 's') // s
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_S));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_S);
		}
		else if(buf[8] == '^') // ^
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_INVERTED_V));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_INVERTED_V);
		}
		else if((buf[8] == 'g') && (buf[9] == 't')) // >
		{
			if(buf[11] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_HORIZONTAL_V));
			else if(buf[11] == '1')
				ts->gesture = (ts->gesture | GESTURE_HORIZONTAL_V);
		}
		else if(buf[8] == 'v') // v
		{
			if(buf[10] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_V));
			else if(buf[10] == '1')
				ts->gesture = (ts->gesture | GESTURE_V);
		}
		else if((buf[8] == 's') && (buf[9] == 'l') && (buf[10] == 'i') && (buf[11] == 'p')) // slip
		{
			if(buf[13] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_SLIP));
			else if(buf[13] == '1')
				ts->gesture = (ts->gesture | GESTURE_SLIP);
		}
		else if((buf[8] == 'd') && (buf[9] == 'c')) // double click
		{
			if(buf[11] == '0')
				ts->gesture = (ts->gesture & (~GESTURE_DOUBLE_CLICK));
			else if(buf[11] == '1')
				ts->gesture = (ts->gesture | GESTURE_DOUBLE_CLICK);
		}
		else
		{
			GTP_INFO("%s: Gesture Option: left, up, right, w, o, m, e ,c ,slip, z, s, v, gt, >, dc, down", __func__);
		}

		GTP_INFO("%s: ts->gesture = 0x%08x", __func__, ts->gesture);
		goto proc_debug_write_end;
	}

	// JUST FOR TEST
	if((buf[0] == 't') && (buf[1] == 'e') && (buf[2] == 's') && (buf[3] == 't'))
	{
		GTP_INFO("%s: debug_function = testtest", __func__);
		if(buf[5] == '1')
		{
			GTP_INFO("%s: report gesture KEY_POWER test.", __func__);
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == '2')
		{
			GTP_INFO("%s: report gesture KEY_POWER2 test.", __func__);
			input_report_key(ts->input_dev, KEY_POWER2, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_POWER2, 0);
			input_sync(ts->input_dev);
		}
		else if((buf[5] == 'd') && (buf[6] == 'c'))
		{
			GTP_INFO("%s: report gesture Double Click test.", __func__);
			input_report_key(ts->input_dev, KEY_F24, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F24, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'w')
		{
			GTP_INFO("%s: report gesture W test.", __func__);
			input_report_key(ts->input_dev, KEY_F23, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F23, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 's')
		{
			GTP_INFO("%s: report gesture S test.", __func__);
			input_report_key(ts->input_dev, KEY_F22, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F22, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'e')
		{
			GTP_INFO("%s: report gesture e test.", __func__);
			input_report_key(ts->input_dev, KEY_F21, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F21, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'c')
		{
			GTP_INFO("%s: report gesture C test.", __func__);
			input_report_key(ts->input_dev, KEY_F20, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F20, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'z')
		{
			GTP_INFO("%s: report gesture S test.", __func__);
			input_report_key(ts->input_dev, KEY_F19, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F19, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'v')
		{
			GTP_INFO("%s: report gesture V test.", __func__);
			input_report_key(ts->input_dev, KEY_F18, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F18, 0);
			input_sync(ts->input_dev);
		}
		else if(buf[5] == 'o')
		{
			GTP_INFO("%s: report gesture V test.", __func__);
			input_report_key(ts->input_dev, KEY_F17, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev, KEY_F17, 0);
			input_sync(ts->input_dev);
		}
		else
		{
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 400);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 500);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 65);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 65);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			input_mt_sync(ts->input_dev);

			msleep(30);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);

			input_sync(ts->input_dev);
		}

		goto proc_debug_write_end;
	}


	GTP_ERROR("%s: Unknow Command.", __func__);
	GTP_ERROR("%s: Command List:\n	fwcheck\n	fwupdate\n	debug\n	intrpin\n	rstpin\n	irq\n	reset\n	work\n	esd\n	tpidpin\n	tpid\n	gmnode", __func__);

proc_debug_write_end:
	return len;
}

static int gt928_proc_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, gt928_proc_debug_read, NULL);
}

static const struct file_operations gt928_debug_fops =
{
	.owner = THIS_MODULE,
	.open = gt928_proc_debug_open,
	.read = seq_read,
	.write = gt928_proc_debug_write,
};

void gt928_create_proc_debug_file(void)
{
	gt928_proc_debug_file = proc_create(GT928_PROC_DEBUG_FILE, 0666, NULL, &gt928_debug_fops);
	if(gt928_proc_debug_file)
	{
		GTP_INFO("%s:[%d]: proc debug file create sucessed", __func__, __LINE__);
	}
	else
	{
		GTP_ERROR("%s:[%d]: proc debug file create failed", __func__, __LINE__);
	}

	return;
}

void gt928_remove_proc_debug_file(void)
{
	extern struct proc_dir_entry proc_root;
	GTP_INFO("%s:[%d]: proc debug file removed.", __func__, __LINE__);
	remove_proc_entry(GT928_PROC_DEBUG_FILE, &proc_root);

	return;
}
