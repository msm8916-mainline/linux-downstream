/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
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
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/regulator/consumer.h>

// ASUS BSP +++
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
extern int Read_PCB_ID(void);
// ASUS BSP ---
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static uint8_t otp_data[32];
uint16_t total_24 = 0, total_32 = 0, check_sum_24 = 0, check_sum_32 = 0;

// For DIT VCM debug Interface +++
#define CAMERA_PROC_OTP_FILE "otp"
#define CAMERA_PROC_OTP_UNIQUE_ID_FILE "otp_uid"

static struct proc_dir_entry *camera_proc_otp_file;
static struct proc_dir_entry *camera_proc_otp_uid_file;

static int camera_proc_otp_read(struct seq_file *buf, void *v){
	int i;
	if (total_32 != check_sum_32) {
		for (i = 0;i < 24;i++){
		    seq_printf(buf, "0x%X", otp_data[i]);
		        if((i+1) % 8 != 0)
				seq_printf(buf, " ");
			  else
				seq_printf(buf, "\n");
		}
	}
	else{
		for (i = 0;i < 32;i++){
		    seq_printf(buf, "0x%X", otp_data[i]);
			  if((i+1) % 8 != 0)
				seq_printf(buf, " ");
			  else
				seq_printf(buf, "\n");
		}
	}

	return 0;
}

static int camera_proc_otp_open(struct inode *inode, struct  file *file) {
    return single_open(file, camera_proc_otp_read, NULL);
}

static const struct file_operations camera_otp_fops = {
        .owner = THIS_MODULE,
        .open = camera_proc_otp_open,
        .read = seq_read,
};

static int camera_proc_otp_uid_read(struct seq_file *buf, void *v){
	int i;

	for (i = 10;i < 22;i++)
		seq_printf(buf, "%X", otp_data[i]);
	seq_printf(buf, "\n");
	return 0;
}

static int camera_proc_otp_uid_open(struct inode *inode, struct  file *file) {
    return single_open(file, camera_proc_otp_uid_read, NULL);
}

static const struct file_operations camera_otp_uid_fops = {
        .owner = THIS_MODULE,
        .open = camera_proc_otp_uid_open,
        .read = seq_read,
};

void create_camera_proc_file(void){

	camera_proc_otp_file = proc_create(CAMERA_PROC_OTP_FILE, 0666, NULL, &camera_otp_fops);
	if(camera_proc_otp_file){
		printk("proc otp file create sucessed!\n");
	}
	else{
		printk("proc otp file create failed!\n");
	}
	camera_proc_otp_uid_file = proc_create(CAMERA_PROC_OTP_UNIQUE_ID_FILE, 0666, NULL, &camera_otp_uid_fops);
	if(camera_proc_otp_uid_file){
		printk("proc otp_uid file create sucessed!\n");
	}
	else{
		printk("proc otp_uid file create failed!\n");
	}
}

void remove_camera_proc_file(void)
{
    remove_proc_entry(CAMERA_PROC_OTP_FILE, NULL);
    remove_proc_entry(CAMERA_PROC_OTP_UNIQUE_ID_FILE, NULL);
}
//For DIT VCM Debug and ATD flash Interface---

//ASUS_BSP+++
void getPCBID(int *ppcb_id, int *pPROJ_ID, int *pZ380KL_Refresh)
{
	
	int pcb_id=-1, project_id=-1, hardware_id=-1, offset_0=-1, offset_1=-1, offset_2=-1, offset_3=-1, offset_4=-1, offset_5=-1; //ASUS_BSP+++
	int Z380KL_Refresh = 0, PROJECT_ID = -1;

	PROJECT_ID = Read_PROJ_ID();
	pr_info("%s(%d):Project ID = %d\n",
		__func__, __LINE__, PROJECT_ID);

	pcb_id = Read_PCB_ID();
	pr_info("%s(%d):PCB ID = %x\n",
		__func__, __LINE__, pcb_id);

	switch (PROJECT_ID) {
		case PROJ_ID_Z380KL:
	
			project_id = pcb_id & 0x7;
			hardware_id = (pcb_id & 0x38) >> 3;
			offset_0 = (pcb_id & 0x40) >> 6;
			offset_1 = (pcb_id & 0x80) >> 7;
			offset_2 = (pcb_id & 0x100) >> 8;
//			Z380KL_Refresh = (pcb_id & 0x200) >> 9;
			offset_3 = (pcb_id & 0xE00) >> 9;
			offset_4 = (pcb_id & 0x3000) >> 12;
			offset_5 = (pcb_id & 0xC000) >> 14;

			if ((offset_3 == 5)||(offset_3 == 7)) {
				Z380KL_Refresh = 1;
			}

			//offset_0: DDR Vendor, offset_1: DDR Density, offset_2: TP, offset_3: LCM, offset_4: Rear CAM, offset_5: Front CAM
			//PCB_ID = pentry->project_id | pentry->hardware_id << 3 | pentry->offset_0 << 6 | pentry->offset_1 << 7 | pentry->offset_2 << 8 | pentry->offset_3 << 9 | pentry->offset_4 << 12 | pentry->offset_5 << 14;

			pr_info("%s: Z380KL_Refresh=%d, Project ID=%d, Hardware ID=%d, DDR Vendor=%d, DDR Density=%d, TP=%d, LCM=%d, Rear CAM=%d, Front CAM=%d\n", __FUNCTION__, Z380KL_Refresh, project_id, hardware_id, offset_0, offset_1, offset_2, offset_3, offset_4, offset_5);
			break;
		default:
			break;
	}

	*ppcb_id = pcb_id;
	*pPROJ_ID = PROJECT_ID;
	*pZ380KL_Refresh = Z380KL_Refresh;
}
//ASUS_BSP---

static struct v4l2_file_operations msm_sensor_v4l2_subdev_fops;
static void msm_sensor_adjust_mclk(struct msm_camera_power_ctrl_t *ctrl)
{
	int idx;
	struct msm_sensor_power_setting *power_setting;
	for (idx = 0; idx < ctrl->power_setting_size; idx++) {
		power_setting = &ctrl->power_setting[idx];
		if (power_setting->seq_type == SENSOR_CLK &&
			power_setting->seq_val ==  SENSOR_CAM_MCLK) {
			if (power_setting->config_val == 24000000) {
				power_setting->config_val = 23880000;
				CDBG("%s MCLK request adjusted to 23.88MHz\n"
							, __func__);
			}
			break;
		}
	}

	return;
}

static int32_t msm_camera_get_power_settimgs_from_sensor_lib(
	struct msm_camera_power_ctrl_t *power_info,
	struct msm_sensor_power_setting_array *power_setting_array)
{
	int32_t rc = 0;
	uint32_t size;
	struct msm_sensor_power_setting *ps;
	bool need_reverse = 0;

	if ((NULL == power_info->power_setting) ||
		(0 == power_info->power_setting_size)) {

		ps = power_setting_array->power_setting;
		size = power_setting_array->size;
		if ((NULL == ps) || (0 == size)) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -EINVAL;
			goto FAILED_1;
		}

		power_info->power_setting =
		kzalloc(sizeof(*ps) * size, GFP_KERNEL);
		if (!power_info->power_setting) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto FAILED_1;
		}
		memcpy(power_info->power_setting,
			power_setting_array->power_setting,
			sizeof(*ps) * size);
		power_info->power_setting_size = size;
	}

	ps = power_setting_array->power_down_setting;
	size = power_setting_array->size_down;
	if (NULL == ps || 0 == size) {
		ps = power_info->power_setting;
		size = power_info->power_setting_size;
		need_reverse = 1;
	}

	power_info->power_down_setting =
	kzalloc(sizeof(*ps) * size, GFP_KERNEL);
	if (!power_info->power_down_setting) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_UP;
	}
	memcpy(power_info->power_down_setting,
		ps,
		sizeof(*ps) * size);
	power_info->power_down_setting_size = size;

	if (need_reverse) {
		int c, end = size - 1;
		struct msm_sensor_power_setting power_down_setting_t;
		for (c = 0; c < size/2; c++) {
			power_down_setting_t =
				power_info->power_down_setting[c];
			power_info->power_down_setting[c] =
				power_info->power_down_setting[end];
			power_info->power_down_setting[end] =
				power_down_setting_t;
			end--;
		}
	}

	return 0;
FREE_UP:
	kfree(power_info->power_setting);
FAILED_1:
	return rc;
}

static int32_t msm_sensor_get_dt_data(struct device_node *of_node,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, i = 0, ret = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	s_ctrl->sensordata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = s_ctrl->sensordata;

	rc = of_property_read_string(of_node, "qcom,sensor-name",
		&sensordata->sensor_name);
	CDBG("%s qcom,sensor-name %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSORDATA;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&s_ctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, s_ctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		s_ctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	rc = msm_sensor_get_sub_module_index(of_node, &sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSORDATA;
	}

	/* Get sensor mount angle */
	if (0 > of_property_read_u32(of_node, "qcom,mount-angle",
		&sensordata->sensor_info->sensor_mount_angle)) {
		/* Invalidate mount angle flag */
		CDBG("%s:%d Default sensor mount angle\n",
			__func__, __LINE__);
		sensordata->sensor_info->is_mount_angle_valid = 0;
		sensordata->sensor_info->sensor_mount_angle = 0;
	} else {
		sensordata->sensor_info->is_mount_angle_valid = 1;
	}
	CDBG("%s qcom,mount-angle %d\n", __func__,
		sensordata->sensor_info->sensor_mount_angle);
	if (0 > of_property_read_u32(of_node, "qcom,sensor-position",
		&sensordata->sensor_info->position)) {
		CDBG("%s:%d Default sensor position\n", __func__, __LINE__);
		sensordata->sensor_info->position = 0;
	}
	CDBG("%s qcom,sensor-position %d\n", __func__,
		sensordata->sensor_info->position);
	if (0 > of_property_read_u32(of_node, "qcom,sensor-mode",
		&sensordata->sensor_info->modes_supported)) {
		CDBG("%s:%d Default sensor mode\n", __func__, __LINE__);
		sensordata->sensor_info->modes_supported = 0;
	}
	CDBG("%s qcom,sensor-mode %d\n", __func__,
		sensordata->sensor_info->modes_supported);

	s_ctrl->set_mclk_23880000 = of_property_read_bool(of_node,
						"qcom,mclk-23880000");

	CDBG("%s qcom,mclk-23880000 %d\n", __func__,
		s_ctrl->set_mclk_23880000);

	rc = msm_sensor_get_dt_csi_data(of_node, &sensordata->csi_lane_params);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSOR_INFO;
	}

	rc = msm_camera_get_dt_vreg_data(of_node,
			&sensordata->power_info.cam_vreg,
			&sensordata->power_info.num_vreg);
	if (rc < 0)
		goto FREE_CSI;

	rc = msm_camera_get_dt_power_setting_data(of_node,
			sensordata->power_info.cam_vreg,
			sensordata->power_info.num_vreg,
			&sensordata->power_info);


	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_VREG;
	}


	rc = msm_camera_get_power_settimgs_from_sensor_lib(
			&sensordata->power_info,
			&s_ctrl->power_setting_array);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_VREG;
	}

	sensordata->power_info.gpio_conf = kzalloc(
			sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if (!sensordata->power_info.gpio_conf) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto FREE_PS;
	}
	gconf = sensordata->power_info.gpio_conf;

	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_CONF;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_CONF;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_REQ_TBL;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_SET_TBL;
		}
	}
	rc = msm_sensor_get_dt_actuator_data(of_node,
					     &sensordata->actuator_info);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_GPIO_PIN_TBL;
	}

	sensordata->slave_info = kzalloc(sizeof(struct msm_camera_slave_info),
		GFP_KERNEL);
	if (!sensordata->slave_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto FREE_ACTUATOR_INFO;
	}

	rc = of_property_read_u32_array(of_node, "qcom,slave-id",
		id_info, 3);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SLAVE_INFO;
	}

	sensordata->slave_info->sensor_slave_addr = id_info[0];
	sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	sensordata->slave_info->sensor_id = id_info[2];
	CDBG("%s:%d slave addr 0x%x sensor reg 0x%x id 0x%x\n",
		__func__, __LINE__,
		sensordata->slave_info->sensor_slave_addr,
		sensordata->slave_info->sensor_id_reg_addr,
		sensordata->slave_info->sensor_id);

	/*Optional property, don't return error if absent */
	ret = of_property_read_string(of_node, "qcom,vdd-cx-name",
		&sensordata->misc_regulator);
	CDBG("%s qcom,misc_regulator %s, rc %d\n", __func__,
		 sensordata->misc_regulator, ret);

	kfree(gpio_array);

	return rc;

FREE_SLAVE_INFO:
	kfree(s_ctrl->sensordata->slave_info);
FREE_ACTUATOR_INFO:
	kfree(s_ctrl->sensordata->actuator_info);
FREE_GPIO_PIN_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
FREE_GPIO_SET_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_set_tbl);
FREE_GPIO_REQ_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
FREE_GPIO_CONF:
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
FREE_PS:
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
FREE_VREG:
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
FREE_CSI:
	kfree(s_ctrl->sensordata->csi_lane_params);
FREE_SENSOR_INFO:
	kfree(s_ctrl->sensordata->sensor_info);
FREE_SENSORDATA:
	kfree(s_ctrl->sensordata);
	kfree(gpio_array);
	return rc;
}

static void msm_sensor_misc_regulator(
	struct msm_sensor_ctrl_t *sctrl, uint32_t enable)
{
	int32_t rc = 0;
	if (enable) {
		sctrl->misc_regulator = (void *)rpm_regulator_get(
			&sctrl->pdev->dev, sctrl->sensordata->misc_regulator);
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(sctrl->misc_regulator,
				RPM_REGULATOR_MODE_HPM);
			if (rc < 0) {
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
				rpm_regulator_put(sctrl->misc_regulator);
			}
		} else {
			pr_err("%s: Failed to vote for rpm regulator on %s: %d\n",
				__func__,
				sctrl->sensordata->misc_regulator, rc);
		}
	} else {
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(
				(struct rpm_regulator *)sctrl->misc_regulator,
				RPM_REGULATOR_MODE_AUTO);
			if (rc < 0)
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
			rpm_regulator_put(sctrl->misc_regulator);
		}
	}
}

int32_t msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (!s_ctrl->pdev && !s_ctrl->sensor_i2c_client->client)
		return 0;
	kfree(s_ctrl->sensordata->slave_info);
	kfree(s_ctrl->sensordata->cam_slave_info);
	kfree(s_ctrl->sensordata->actuator_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_set_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
	kfree(s_ctrl->sensordata->csi_lane_params);
	kfree(s_ctrl->sensordata->sensor_info);
	kfree(s_ctrl->sensordata->power_info.clk_info);
	kfree(s_ctrl->sensordata);

	remove_camera_proc_file(); //ASUS_BSP+++
	return 0;
}

static struct msm_cam_clk_info cam_8960_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_clk", 24000000},
};

static struct msm_cam_clk_info cam_8610_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

//ASUS_BSP+++ VCM noise reduction
extern  void asus_actuator_close(void);
int isPowerup=0;
//ASUS_BSP+++ VCM noise reduction

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_power_ctrl_t *power_info;
	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_i2c_client *sensor_i2c_client;

	if (!s_ctrl) {
		pr_err("%s:%d failed: s_ctrl %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

        //ASUS_BSP+++ VCM noise reduction
	if(isPowerup){
		asus_actuator_close();
	}
	//ASUS_BSP+++ VCM noise reduction

	power_info = &s_ctrl->sensordata->power_info;
	sensor_device_type = s_ctrl->sensor_device_type;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;

	if (!power_info || !sensor_i2c_client) {
		pr_err("%s:%d failed: power_info %p sensor_i2c_client %p\n",
			__func__, __LINE__, power_info, sensor_i2c_client);
		return -EINVAL;
	}

        isPowerup=0; //ASUS_BSP+++ VCM noise reduction

	return msm_camera_power_down(power_info, sensor_device_type,
		sensor_i2c_client);
}

int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	uint32_t retry = 0;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!power_info || !sensor_i2c_client || !slave_info ||
		!sensor_name) {
		pr_err("%s:%d failed: %p %p %p %p\n",
			__func__, __LINE__, power_info,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	if (s_ctrl->set_mclk_23880000)
		msm_sensor_adjust_mclk(power_info);

	for (retry = 0; retry < 3; retry++) {
		rc = msm_camera_power_up(power_info, s_ctrl->sensor_device_type,
			sensor_i2c_client);
		if (rc < 0)
			return rc;
		rc = msm_sensor_check_id(s_ctrl);
		if (rc < 0) {
			msm_camera_power_down(power_info,
				s_ctrl->sensor_device_type, sensor_i2c_client);
			msleep(20);
			continue;
		} else {
			break;
		}
	}

        //ASUS_BSP+++ VCM noise reduction
	if(rc==0){
		isPowerup=1;
	}
	//ASUS_BSP+++ VCM noise reduction

	return rc;
}

int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	static int pcb_id = -1, PROJ_ID = -1, Z380KL_Refresh = -1; //ASUS_BSP+++

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	pr_err("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);

	//ASUS_BSP+++
    if (pcb_id==-1) {
	    getPCBID(&pcb_id, &PROJ_ID, &Z380KL_Refresh);
    }

	if (PROJ_ID == PROJ_ID_Z380KL) {
//		Z380KL_Refresh = 1; //Test Code, need to be removed before checkin
    	if (Z380KL_Refresh==0) {
	      pr_info("%s, Z380KL Project", __FUNCTION__);
		  if (chipid == 0x2680) {
		  	pr_err("%s: Wrong vga module = OV2680\n", __func__);
			return -2680;
		  }
  		  if (chipid == 0x885a) {
		  	pr_err("%s: Wrong camera module = OV8856\n", __func__);
			return -8856;
		  }
	    } else {
		  pr_info("%s, Z380KL Refresh Project", __FUNCTION__);
		  if (chipid == 0x20ff) {
		  	pr_err("%s: Wrong vga module = HM2051\n", __func__);
			return -2051;
		  }
  		  if (chipid == 0x1481) {
		  	pr_err("%s: Wrong camera module = T4K35\n", __func__);
			return -1481;
		  }
		}
	}
	//ASUS_BSP---
	
	if (chipid != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

//ASUS_BSP+++
int msm_sensor_read_otp(struct msm_sensor_ctrl_t *s_ctrl){

	struct msm_sensor_otp_data *buf;
	const char *sensor_name;
	int package, i, ret ;

	uint16_t read_value[32];
	uint16_t local_data;
	static int pcb_id = -1, PROJ_ID = -1, Z380KL_Refresh = -1; //ASUS_BSP+++

	sensor_name = s_ctrl->sensordata->sensor_name;
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	if((strcmp("ov5670",sensor_name) == 0)||(strcmp("ov5670_30010a3",sensor_name) == 0)||(strcmp("ov8856",sensor_name) == 0)){
		int start_addr, end_addr;
		pr_err("Back Camera %s\n", sensor_name);
		// otp valid after mipi on and sw stream on
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		    s_ctrl->sensor_i2c_client, 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		// disable OTP_DPC
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client, 0x5002, &local_data, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client, 0x5002, ((0x00&0x02) | (local_data&(~0x02))),
			MSM_CAMERA_I2C_BYTE_DATA);

		for (package = 2; package >=0; package--){
			if(package == 0){
			    start_addr = 0x7010;
			    end_addr   = 0x702f;
			}
			else if(package == 1){
			    start_addr = 0x7030;
			    end_addr   = 0x704f;
			}
			else if(package == 2){
			    start_addr = 0x7050;
			    end_addr   = 0x706f;
			}
			pr_err("package = %d, start_addr = 0x%x, end_addr = 0x%x\n",
				package, start_addr, end_addr);
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d84, 0xc0, MSM_CAMERA_I2C_BYTE_DATA);

			//partial mode OTP write start address
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d88, (start_addr >> 8)&0xff, MSM_CAMERA_I2C_BYTE_DATA);
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d89, start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

			// partial mode OTP write end address
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d8A, (end_addr >> 8)&0xff, MSM_CAMERA_I2C_BYTE_DATA);
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d8B, end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

			// read otp into buffer
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

			msleep(5);

			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client, 0x3d81, &local_data, MSM_CAMERA_I2C_BYTE_DATA);

			for (i = 0; i < 32; i++){
				s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read( s_ctrl->sensor_i2c_client,
					start_addr + i, &read_value[i], MSM_CAMERA_I2C_BYTE_DATA);
				// pr_err("start_addr 0x%x, value: 0x%x",start_addr + i, read_value[i]);
			}
			for(i = 0; i < 30; i++){
				total_32+=read_value[i];
			}

			check_sum_32 = (read_value[30] << 8) + read_value [31];

			pr_err("\n%s Check Data Package %d, 0x%X 0x%X\n", __func__, package, read_value[0], read_value[1]);
			if((read_value[0]!=0 || read_value[1]!=0) && (read_value[0]!=0xff || read_value[1]!=0xff)){
				// 32 bytes OTP
				if(total_32 == check_sum_32){
				    pr_err("32byte OTP\n");
				    // clear otp buffer
				    for (i = start_addr; i<=end_addr; i++)
				        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
						    s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

				    // enable OTP_DPC
				    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				        s_ctrl->sensor_i2c_client, 0x5002, &local_data,MSM_CAMERA_I2C_BYTE_DATA);
				    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				        s_ctrl->sensor_i2c_client, 0x5002, ((0x02&0x02) | (local_data&(~0x02))),
				        MSM_CAMERA_I2C_BYTE_DATA);
				    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				        s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				    goto out;
				}
				else{
				    pr_err("24byte OTP\n");
				    // clear otp buffer
				    for (i = start_addr; i<=end_addr; i++){
				        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				            s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

				        for (package = 2; package >=0; package--){
				            if(package == 0){
				                start_addr = 0x7010;
				                end_addr   = 0x7027;
				            }
				            else if(package == 1){
				                start_addr = 0x7028;
				                end_addr   = 0x703f;
				            }
				            else if(package == 2){
				                start_addr = 0x7040;
				                end_addr   = 0x7057;
				            }
				            pr_err("package = %d, start_addr = 0x%x, end_addr = 0x%x\n",
					            package, start_addr, end_addr);

				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d84, 0xc0, MSM_CAMERA_I2C_BYTE_DATA);

				            //partial mode OTP write start address
				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d88, (start_addr >> 8)&0xff, MSM_CAMERA_I2C_BYTE_DATA);
				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d89, start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

				            // partial mode OTP write end address
				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d8A, (end_addr >> 8)&0xff, MSM_CAMERA_I2C_BYTE_DATA);
				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d8B, end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

				            // read otp into buffer
				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

				            msleep(5);

				            s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				                s_ctrl->sensor_i2c_client, 0x3d81, &local_data, MSM_CAMERA_I2C_BYTE_DATA);

				            for (i = 0; i < 24; i++){
				                s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read( s_ctrl->sensor_i2c_client,
				                    start_addr + i, &read_value[i], MSM_CAMERA_I2C_BYTE_DATA);
				                // pr_err("start_addr 0x%x, value: 0x%x",start_addr + i, read_value[i]);
				            }
				            pr_err("\n%s Check Data Package %d, 0x%X 0x%X\n", __func__, package, read_value[0], read_value[1]);
				            if((read_value[8]!=0 || read_value[9]!=0) && (read_value[8]!=0xff || read_value[9]!=0xff)){
				                // clear otp buffer
				                for (i = start_addr; i<=end_addr; i++)
				                    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                        s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

				                // enable OTP_DPC
				                s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				                    s_ctrl->sensor_i2c_client, 0x5002, &local_data,MSM_CAMERA_I2C_BYTE_DATA);
				                s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                    s_ctrl->sensor_i2c_client, 0x5002, ((0x02&0x02) | (local_data&(~0x02))),
				                    MSM_CAMERA_I2C_BYTE_DATA);
				                s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				                    s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				                goto out;
	                        }
	                    }
	                }
	            }
	        }
	    }
	    // clear otp buffer
	    for (i = start_addr; i<=end_addr; i++)
	        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
	            s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	    // enable OTP_DPC
	    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
	        s_ctrl->sensor_i2c_client, 0x5002, &local_data,
	        MSM_CAMERA_I2C_BYTE_DATA);
	    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
	        s_ctrl->sensor_i2c_client, 0x5002, ((0x02&0x02) | (local_data&(~0x02))),
	        MSM_CAMERA_I2C_BYTE_DATA);
	    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
	        s_ctrl->sensor_i2c_client, 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	    return -EIO;
	}
	if(strcmp("t4k35",sensor_name) == 0){
	    uint16_t data;

	    pr_err("Back Camera %s\n",sensor_name);
	    ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
	    s_ctrl->sensor_i2c_client, 0x3500, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	    if (ret) {
	        pr_err("failed to write OTP_ENBL\n");
	        return ret;
	    }

	for (package = 2; package >=0; package--){
		//set page NO.
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client, 0x3502, package, MSM_CAMERA_I2C_BYTE_DATA);

		//Start OTP access
		ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client, 0x3500, 0x81, MSM_CAMERA_I2C_BYTE_DATA);
		if (ret) {
			pr_err("failed to set OTP_STA\n");
			return ret;
		}

		//Check access status
		for(i=0; i<10; i++) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client, 0x3500, &data,
				MSM_CAMERA_I2C_BYTE_DATA);
			if((data & 0x80) >> 7 == 0) {
				printk("%s OTP Data Ready\n", __func__);
				break;
			}
			msleep(10);
		}
		//Reading the OTP data array
		for (i = 0; i < 32; i++){
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client, 0x3504 + i, &read_value[i],
				MSM_CAMERA_I2C_BYTE_DATA);
			//pr_err("start_addr 0x%x, value: 0x%x", 0x3504 + i, read_value[i]);
		}
		printk("%s Check Data Package %d, 0x%X 0x%X\n", __func__, package, read_value[0], read_value[1]);

		if((read_value[0]!=0 || read_value[1]!=0) && (read_value[0]!=0xff || read_value[1]!=0xff)){
			//Disable OTP access
			ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
				s_ctrl->sensor_i2c_client, 0x3500, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		if (ret) {
			pr_err("failed to disable OTP_STA\n");
			return ret;
		}
		for(i = 0; i < 30; i++){
			if(i < 22){
				total_24+=read_value[i];
			}
			total_32+=read_value[i];
		}

		check_sum_32 = (read_value[30] << 8) + read_value [31];
		check_sum_24 = (read_value[22] << 8) + read_value[23];

		goto out;
			}
		}

		pr_err("failed to read OTP value\n");
		//Disable OTP access
		ret = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                           s_ctrl->sensor_i2c_client, 0x3500, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		if (ret) {
			pr_err("failed to disable OTP_STA\n");
			return ret;
		}
		return -EIO;
	}
	else{
		return 0;
	}

out:
	// memcpy
	for(i = 0; i < 32; i++)
		otp_data[i] = (uint8_t) read_value[i];

	printk("%s, Module ID = 0x%x\n", __func__, read_value[8]);
	printk("%s, OTP Value = \n", __func__);
	
	if (total_32 != check_sum_32) {
		for (i = 0;i < 24;i++){
		    printk("0x%X", otp_data[i]);
		      if((i+1) % 8 != 0)
				printk(" ");
			  else
				printk("\n");
		}
	}
	else{
		for (i = 0;i < 32;i++){
		    printk("0x%X", otp_data[i]);
			  if((i+1) % 8 != 0)
				printk(" ");
			  else
				printk("\n");
		}
	}

	//ASUS_BSP+++
    if (pcb_id==-1) {
	    getPCBID(&pcb_id, &PROJ_ID, &Z380KL_Refresh);
    }

	if (PROJ_ID == PROJ_ID_Z380KL) {
//		Z380KL_Refresh = 1; //Test Code, need to be removed before checkin
    	if (Z380KL_Refresh==0) {
	      pr_info("%s, Z380KL Project", __FUNCTION__);
		  if (read_value[8]==0x30) {
			printk("Wrong Module\n");
			return -5670;
		  }

	    } else {
		  pr_info("%s, Z380KL Refresh Project", __FUNCTION__);
		  if (read_value[8]==0x1e) {
			printk("Wrong Module\n");
			return -5670;
		  }
		}
	}
	//ASUS_BSP---



	buf->af_inf_pos = read_value[0]<<8 | read_value[1];
	buf->af_30cm_pos = read_value[2]<<8 | read_value[3];
	buf->af_10cm_pos = read_value[4]<<8 | read_value[5];
	buf->af_start_curr = read_value[6]<<8 | read_value[7];
	buf->module_id = read_value[8];
	buf->vendor_id = read_value[9];

	create_camera_proc_file();
	return 0;

}

//ASUS_BSP---

static struct msm_sensor_ctrl_t *get_sctrl(struct v4l2_subdev *sd)
{
	return container_of(container_of(sd, struct msm_sd_subdev, sd),
		struct msm_sensor_ctrl_t, msm_sd);
}

static void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &s_ctrl->stop_setting);
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return;
}

static int msm_sensor_get_af_status(struct msm_sensor_ctrl_t *s_ctrl,
			void __user *argp)
{
	/* TO-DO: Need to set AF status register address and expected value
	We need to check the AF status in the sensor register and
	set the status in the *status variable accordingly*/
	return 0;
}

static long msm_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	void __user *argp = (void __user *)arg;
	if (!s_ctrl) {
		pr_err("%s s_ctrl NULL\n", __func__);
		return -EBADF;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
#ifdef CONFIG_COMPAT
		if (is_compat_task())
			rc = s_ctrl->func_tbl->sensor_config32(s_ctrl, argp);
		else
#endif
			rc = s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
		return rc;
	case VIDIOC_MSM_SENSOR_GET_AF_STATUS:
		return msm_sensor_get_af_status(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_RELEASE:
	case MSM_SD_SHUTDOWN:
		msm_sensor_stop_stream(s_ctrl);
		return 0;
	case MSM_SD_NOTIFY_FREEZE:
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_sensor_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG32:
		cmd = VIDIOC_MSM_SENSOR_CFG;
	default:
		return msm_sensor_subdev_ioctl(sd, cmd, arg);
	}
}

long msm_sensor_subdev_fops_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_sensor_subdev_do_ioctl);
}

static int msm_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);
		conf_array.qup_i2c_batch = conf_array32.qup_i2c_batch;

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
			(void *)(conf_array.reg_setting),
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		if (copy_from_user(&read_config,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		if (copy_to_user(&read_config.data,
			(void *)&local_data, sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting32 conf_array32;
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_seq_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_POWER_DOWN:
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting32 stop_setting32;
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(&stop_setting32,
				(void *)compat_ptr((cdata->cfg.setting)),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		stop_setting->addr_type = stop_setting32.addr_type;
		stop_setting->data_type = stop_setting32.data_type;
		stop_setting->delay = stop_setting32.delay;
		stop_setting->size = stop_setting32.size;
		stop_setting->qup_i2c_batch = stop_setting32.qup_i2c_batch;

		reg_setting = compat_ptr(stop_setting32.reg_setting);

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		if (copy_from_user(&read_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		if (copy_to_user(&read_config.data,
			(void *)&local_data, sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size ||
			write_config.conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_POWER_DOWN:
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static int msm_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (!on && s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static int msm_sensor_v4l2_enum_fmt(struct v4l2_subdev *sd,
	unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);

	if ((unsigned int)index >= s_ctrl->sensor_v4l2_subdev_info_size)
		return -EINVAL;

	*code = s_ctrl->sensor_v4l2_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops msm_sensor_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops msm_sensor_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops msm_sensor_subdev_ops = {
	.core = &msm_sensor_subdev_core_ops,
	.video  = &msm_sensor_subdev_video_ops,
};

static struct msm_sensor_fn_t msm_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_read_otp = msm_sensor_read_otp,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_write_conf_tbl = msm_camera_qup_i2c_write_conf_tbl,
};

int32_t msm_sensor_platform_probe(struct platform_device *pdev,
				  const void *data)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl =
		(struct msm_sensor_ctrl_t *)data;
	struct msm_camera_cci_client *cci_client = NULL;
	uint32_t session_id;
	unsigned long mount_pos = 0;
	s_ctrl->pdev = pdev;
	CDBG("%s called data %p\n", __func__, data);
	CDBG("%s pdev name %s\n", __func__, pdev->id_entry->name);
	if (pdev->dev.of_node) {
		rc = msm_sensor_get_dt_data(pdev->dev.of_node, s_ctrl);
		if (rc < 0) {
			pr_err("%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
	}
	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	/* TODO: get CCI subdev */
	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_client->sid =
		s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;
	if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
		s_ctrl->sensor_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;
	s_ctrl->sensordata->power_info.clk_info =
		kzalloc(sizeof(cam_8974_clk_info), GFP_KERNEL);
	if (!s_ctrl->sensordata->power_info.clk_info) {
		pr_err("%s:%d failed nomem\n", __func__, __LINE__);
		kfree(cci_client);
		return -ENOMEM;
	}
	memcpy(s_ctrl->sensordata->power_info.clk_info, cam_8974_clk_info,
		sizeof(cam_8974_clk_info));
	s_ctrl->sensordata->power_info.clk_info_size =
		ARRAY_SIZE(cam_8974_clk_info);
	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		kfree(s_ctrl->sensordata->power_info.clk_info);
		kfree(cci_client);
		return rc;
	}

	pr_info("%s %s probe succeeded\n", __func__,
		s_ctrl->sensordata->sensor_name);
	v4l2_subdev_init(&s_ctrl->msm_sd.sd,
		s_ctrl->sensor_v4l2_subdev_ops);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, pdev);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =
		s_ctrl->msm_sd.sd.name;

	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
					sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	rc = camera_init_v4l2(&s_ctrl->pdev->dev, &session_id);
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
	msm_sensor_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	msm_sensor_v4l2_subdev_fops.compat_ioctl32 =
		msm_sensor_subdev_fops_ioctl;
#endif
	s_ctrl->msm_sd.sd.devnode->fops =
		&msm_sensor_v4l2_subdev_fops;

	CDBG("%s:%d\n", __func__, __LINE__);

	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	CDBG("%s:%d\n", __func__, __LINE__);
	return rc;
}

int msm_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id, struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t session_id;
	unsigned long mount_pos = 0;
	CDBG("%s %s_i2c_probe called\n", __func__, client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!client->dev.of_node) {
		CDBG("msm_sensor_i2c_probe: of_node is NULL");
		s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
		if (!s_ctrl) {
			pr_err("%s:%d sensor ctrl structure NULL\n", __func__,
				__LINE__);
			return -EINVAL;
		}
		s_ctrl->sensordata = client->dev.platform_data;
	} else {
		CDBG("msm_sensor_i2c_probe: of_node exisists");
		rc = msm_sensor_get_dt_data(client->dev.of_node, s_ctrl);
		if (rc < 0) {
			pr_err("%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
	}

	s_ctrl->sensor_device_type = MSM_CAMERA_I2C_DEVICE;
	if (s_ctrl->sensordata == NULL) {
		pr_err("%s %s NULL sensor data\n", __func__, client->name);
		return -EFAULT;
	}

	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		s_ctrl->sensordata->power_info.dev = &client->dev;
		if (s_ctrl->sensordata->slave_info->sensor_slave_addr)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensordata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;
	if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
		s_ctrl->sensor_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	if (!client->dev.of_node) {
		s_ctrl->sensordata->power_info.clk_info =
			kzalloc(sizeof(cam_8960_clk_info), GFP_KERNEL);
		if (!s_ctrl->sensordata->power_info.clk_info) {
			pr_err("%s:%d failed nomem\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(s_ctrl->sensordata->power_info.clk_info,
			cam_8960_clk_info, sizeof(cam_8960_clk_info));
		s_ctrl->sensordata->power_info.clk_info_size =
			ARRAY_SIZE(cam_8960_clk_info);
	} else {
		s_ctrl->sensordata->power_info.clk_info =
			kzalloc(sizeof(cam_8610_clk_info), GFP_KERNEL);
		if (!s_ctrl->sensordata->power_info.clk_info) {
			pr_err("%s:%d failed nomem\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(s_ctrl->sensordata->power_info.clk_info,
			cam_8610_clk_info, sizeof(cam_8610_clk_info));
		s_ctrl->sensordata->power_info.clk_info_size =
			ARRAY_SIZE(cam_8610_clk_info);
	}

	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__, client->name);
		kfree(s_ctrl->sensordata->power_info.clk_info);
		return rc;
	}

	CDBG("%s %s probe succeeded\n", __func__, client->name);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->msm_sd.sd, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, client);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =
		s_ctrl->msm_sd.sd.name;
	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
					sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	rc = camera_init_v4l2(&s_ctrl->sensor_i2c_client->client->dev,
		&session_id);
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
	CDBG("%s:%d\n", __func__, __LINE__);

	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	return rc;
}

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                       rc = -ENOMEM;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_cam_clk_info      *clk_info = NULL;
	unsigned long mount_pos = 0;

	/* Validate input parameters */
	if (!s_ctrl) {
		pr_err("%s:%d failed: invalid params s_ctrl %p\n", __func__,
			__LINE__, s_ctrl);
		return -EINVAL;
	}

	if (!s_ctrl->sensor_i2c_client) {
		pr_err("%s:%d failed: invalid params sensor_i2c_client %p\n",
			__func__, __LINE__, s_ctrl->sensor_i2c_client);
		return -EINVAL;
	}

	/* Initialize cci_client */
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s:%d failed: no memory cci_client %p\n", __func__,
			__LINE__, s_ctrl->sensor_i2c_client->cci_client);
		return -ENOMEM;
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = s_ctrl->sensor_i2c_client->cci_client;

		/* Get CCI subdev */
		cci_client->cci_subdev = msm_cci_get_subdev();

		/* Update CCI / I2C function table */
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_cci_func_tbl;
	} else {
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl) {
			CDBG("%s:%d\n", __func__, __LINE__);
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_qup_func_tbl;
		}
	}

	/* Update function table driven by ioctl */
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;

	/* Update v4l2 subdev ops table */
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	/* Initialize clock info */
	clk_info = kzalloc(sizeof(cam_8974_clk_info), GFP_KERNEL);
	if (!clk_info) {
		pr_err("%s:%d failed no memory clk_info %p\n", __func__,
			__LINE__, clk_info);
		rc = -ENOMEM;
		goto FREE_CCI_CLIENT;
	}
	memcpy(clk_info, cam_8974_clk_info, sizeof(cam_8974_clk_info));
	s_ctrl->sensordata->power_info.clk_info = clk_info;
	s_ctrl->sensordata->power_info.clk_info_size =
		ARRAY_SIZE(cam_8974_clk_info);

	/* Update sensor mount angle and position in media entity flag */
	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
					sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	return 0;

FREE_CCI_CLIENT:
	kfree(cci_client);
	return rc;
}
