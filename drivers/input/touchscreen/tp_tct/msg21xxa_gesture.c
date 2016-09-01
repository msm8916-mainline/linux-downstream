#include "msg21xx.h"

#ifdef MSTAR_PWRON_UPGRADE
extern int mstar_fw_upgrade(struct device *dev, bool force);

static ssize_t mstar_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct msg21xx_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc = -EINVAL;

	if (NULL == ts) {
		printk("%s %d failed ! \n", __func__, __LINE__);
		return -2;
	}

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&ts->input_dev->mutex);
	if (1 == val)
		mstar_fw_upgrade(dev, true);
	mutex_unlock(&ts->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0222, NULL,
				mstar_force_update_fw_store);
#endif

#ifdef MSTAR_TP_GESTURE
static struct class * tp_gesture_class;
static struct device * tp_gesture_dev;

/*
0000 0001 DoubleClick
0000 0010 Up Direction
0000 0100 Down Direction
0000 1000 Left Direction
0001 0000 Right Direction
0001 1111 All Of Five Funciton
*/

#define MSG_GESTURE_FUNCTION_DOUBLECLICK_FLAG  0x01
#define MSG_GESTURE_FUNCTION_UPDIRECT_FLAG     0x02
#define MSG_GESTURE_FUNCTION_DOWNDIRECT_FLAG   0x04
#define MSG_GESTURE_FUNCTION_LEFTDIRECT_FLAG   0x08
#define MSG_GESTURE_FUNCTION_RIGHTDIRECT_FLAG  0x10
#define MSG_GESTURE_FUNCTION_ALL5ON_FLAG  0x1F

#define MSG_GESTURE_DIRECTION_ON_FLAG  0x09

static ssize_t tp_gesture_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct msg21xx_ts_data *data = NULL;
	int ret = -EINVAL;
	data = dev_get_drvdata(dev);

	if (NULL == data) {
		printk("%s %d failed ! \n", __func__, __LINE__);
		return ret;
	}

	if (0x00 == data->gesture_pnum) {
	       ret = snprintf(buf, 50, "0x%02x,%d\n", data->gesture_code, data->gesture_pnum);
		data->gesture_code = 0;
		data->gesture_pnum = 0;
	} else
		printk(">>-- %s %d error \n", __func__, __LINE__);

       return ret;
}

static ssize_t tp_gesture_id_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct msg21xx_ts_data *data = NULL;
	unsigned long val = 0;
	ssize_t ret = -EINVAL;

	data = dev_get_drvdata(dev);
	if (NULL == data) {
		printk("%s %d failed ! \n", __func__, __LINE__);
		return ret;
	}

	if (data->suspended)
		return ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if ( 0 == val )
	{
		data->gesture_id = 0x00;
		device_init_wakeup(&data->client->dev, 0);
	} else
	if ( val > 0 )
	{
		data->gesture_id = val;
		device_init_wakeup(&data->client->dev, 1);
	} else {
		printk("%s %d invalid  command(%lu)! \n", __func__, __LINE__, val);
		return -1;
	}
	printk("gesture_id got from app = %d \n", data->gesture_id);

	return size;
}

static ssize_t tp_fw_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	get_customer_firmware_version();
	ret = snprintf(buf, 100, "Mstar TP fw_version is %s.\n", fw_version);

	return ret;
}

#if 0
static ssize_t tp_kreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct msg21xx_ts_data *data = NULL;
	struct i2c_client *client = NULL;
       int ret;

	u8 reg_addr, reg_data0, reg_data1;
	int err;

	data = dev_get_drvdata(dev);
	client = data->client;

	reg_data0 = 0;
	reg_data1 = 0;

	reg_addr = 0x96;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_data0, 1);
	if (err < 0)
		dev_err(&client->dev, "reg_data0");

	reg_addr = 0xB0;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_data1, 1);
	if (err < 0)
		dev_err(&client->dev, "reg_data1");

       ret = snprintf(buf, 100, "FocalTech (0x96) is 0x%x, fw_version (0xB0) is 0x%x.\n", reg_data0, reg_data1);

       return ret;
}
#endif

static DEVICE_ATTR(tp_gesture_id, 0644, tp_gesture_id_show, tp_gesture_id_store);

static DEVICE_ATTR(tp_fw_version, 0444, tp_fw_version_show, NULL);
#if 0
static DEVICE_ATTR(tp_kreg_val, 0444, tp_kreg_show, NULL);
#endif
void mstar_tp_gestures_register ( struct msg21xx_ts_data *data)
{
	int rc = 0;
	tp_gesture_class = class_create(THIS_MODULE, "tp_gesture");
	if (IS_ERR(tp_gesture_class))
		pr_err("Failed to create class(tp_gesture_class)!\n");

	tp_gesture_dev = device_create(tp_gesture_class, NULL, 0, NULL, "tp_device");
	if (IS_ERR(tp_gesture_dev))
		pr_err("Failed to create device(lcd_ce_ctrl)!\n");

	// tp_gesture
	rc = device_create_file(tp_gesture_dev, &dev_attr_tp_gesture_id);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tp_gesture_id.attr.name);

	rc = device_create_file(tp_gesture_dev, &dev_attr_tp_fw_version);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tp_fw_version.attr.name);

#ifdef MSTAR_PWRON_UPGRADE
	rc = device_create_file(tp_gesture_dev, &dev_attr_force_update_fw);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_force_update_fw.attr.name);
#endif

#if 0
	rc = device_create_file(tp_gesture_dev, &dev_attr_tp_kreg_val);
	if ( rc < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tp_kreg_val.attr.name);
#endif
	dev_set_drvdata(tp_gesture_dev, data);
//	dev_set_drvdata(lcd_ce_dev, NULL);

}

int mstar_tp_interrupt(struct msg21xx_ts_data *tsdata, unsigned char checksum, unsigned char *data)
{
	int rc = 0;

	if( (tsdata->gesture_id > 0) && (0x01 == tsdata->gesture_set) )
	{

		if(( data[0] == 0x52 ) && ( data[1] == 0xFF ) && ( data[2] == 0xFF ) && ( data[3] == 0xFF ) && ( data[4] == 0xFF ) && ( data[6] == 0xFF ) && ( checksum  == data[7] ))
		{
			if(data[5] == 0x58)
			{
				/* double click gesture */
				if (0x01 == tsdata->gesture_id)
					tsdata->gesture_code = 0x01;
				else if (0x02 == tsdata->gesture_id)
					tsdata->gesture_code = 0x02;
			}
			else if(data[5] == 0x60)
			{
				/* up direction gesture */
				if (MSG_GESTURE_DIRECTION_ON_FLAG == tsdata->gesture_id)
					tsdata->gesture_code = 0x05;
			}
			else if(data[5] == 0x61)
			{
				/* down direction gesture */
				if (MSG_GESTURE_DIRECTION_ON_FLAG == tsdata->gesture_id)
					tsdata->gesture_code = 0x06;
			}
			else if(data[5] == 0x62)
			{
				/* left direction gesture */
				if (MSG_GESTURE_DIRECTION_ON_FLAG == tsdata->gesture_id)
					tsdata->gesture_code = 0x07;
			}
			else if(data[5] == 0x63)
			{
				/* right direction gesture */
				if (MSG_GESTURE_DIRECTION_ON_FLAG == tsdata->gesture_id)
					tsdata->gesture_code = 0x08;
			} else
				printk(">>-- %s gesture mode = 0x%0x \n", __func__, data[5]);

			tsdata->gesture_pnum = 0x0;
		}

		if ( tsdata->gesture_code > 0 ) {
			if ( ( 0x01 == tsdata->gesture_id) ||(MSG_GESTURE_DIRECTION_ON_FLAG == tsdata->gesture_id) ){
				input_report_key(tsdata->input_dev, KEY_UNLOCK, 1);
				input_sync(tsdata->input_dev);
				input_report_key(tsdata->input_dev, KEY_UNLOCK, 0);
				input_sync(tsdata->input_dev);
			} else
			if ( 0x02 == tsdata->gesture_id) {
				input_report_key(tsdata->input_dev, KEY_POWER, 1);
				input_sync(tsdata->input_dev);
				input_report_key(tsdata->input_dev, KEY_POWER, 0);
				input_sync(tsdata->input_dev);
			}
		}

		rc = 1;
	}

	return rc;
}

static int mstar_open_gesture(void)
{
	unsigned char dbbus_tx_data[3];
	int rc = 0;

	/**********1st open command*********/
	dbbus_tx_data[0] = 0x58;
	dbbus_tx_data[1] = 0x00;

	/*
	0000 0001 DoubleClick
	0000 0010 Up Direction
	0000 0100 Down Direction
	0000 1000 Left Direction
	0001 0000 Right Direction
	0001 1111 All Of Five Funciton
	*/

	dbbus_tx_data[2] = (0xFF & MSG_GESTURE_FUNCTION_ALL5ON_FLAG);

	if( (dbbus_tx_data[2] >= 0x01) && (dbbus_tx_data[2] <= MSG_GESTURE_FUNCTION_ALL5ON_FLAG) ) {

		rc = write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
		if (rc < 0) {
			printk("%s line(%d) rc=%d \n", __func__, __LINE__, rc);
			return rc;
		}
		msleep(20);
		return 0;
	} else {
		printk("%s error gesture cmd(0x%x) \n", __func__, dbbus_tx_data[2]);
		return -2;
	}
}

#define GESTURE_OPERATION_RETRY 1

int mstar_tp_suspend(struct msg21xx_ts_data *data)
{
	int i, err = 0;

//	if ( ( data->gesture_id > 0) &&(0x00 == data->gesture_set) )
	if ( data->gesture_id > 0)
	{
		for (i = 0; i < GESTURE_OPERATION_RETRY; i ++) {
			err = mstar_open_gesture();
			if (0 == err)
				break;
		}
		printk(">>-- %s i = %d, err = %d \n", __func__, i, err);

		if (0 == err) {
			data->gesture_set = 0x01;
			if (device_may_wakeup(&data->client->dev))
			{
				enable_irq_wake(data->client->irq);
			}
			data->suspended = true;
			data->gesture_on = 1;
		} else {
			printk(">>-- %s open gesture err(%d) \n", __func__, err);
			data->gesture_on = 0;
		}
	}

	return err;
}

int mstar_tp_resume(struct msg21xx_ts_data *data)
{

//	if ( ( data->gesture_id > 0) &&(0x01 == data->gesture_set) )
	if ( data->gesture_id > 0)
	{
		data->gesture_set = 0x00;
		if (device_may_wakeup(&data->client->dev))
		{
			disable_irq_wake(data->client->irq);
		}

		data->suspended = false;
		data->gesture_on = 0;
	}

	return 0;
}

void keyset_mstartp_gesture(struct input_dev *input_dev)
{
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_UNLOCK);
}
#endif

#ifdef MSTAR_PWRON_UPGRADE
int mstar_fw_upgrade(struct device *dev, bool force)
{
	struct msg21xx_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;

	if (data->suspended) {
		printk("Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		printk("Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	printk(">>-- %s line=%d, request_firmware=%d, fw_name=%s fw->size=%zu \n", __func__, __LINE__, rc, data->fw_name, fw->size);

	/* start firmware upgrade */
	rc = mstar_fw_upgrade_start(fw->data, fw->size, force);
	if (rc < 0)
		printk("update failed (%d). try later...\n", rc);

	release_firmware(fw);
	return rc;
}

static void mstar_init_update_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct msg21xx_ts_data *ts;
	struct device *dev;

	delay_work = to_delayed_work(work);
	ts = container_of(delay_work, struct msg21xx_ts_data, mstar_update_work);
	dev = &ts->input_dev->dev;

	mutex_lock(&ts->input_dev->mutex);
	mstar_fw_upgrade(dev, false);
//	mstar_fw_upgrade(dev, true);
	mutex_unlock(&ts->input_dev->mutex);
}

u8 mstar_init_update_proc(struct msg21xx_ts_data *ts)
{
	dev_dbg(&ts->client->dev, "Ready to run update work.");

	INIT_DELAYED_WORK(&ts->mstar_update_work, mstar_init_update_work);
	schedule_delayed_work(&ts->mstar_update_work,
		msecs_to_jiffies(3000));

	return 0;
}
#endif

