/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
* 14/03/07      Hu Jin       add OTP for s5k5e2_rio5_ctcc(Ewelly)
* 14/05/06      YU Qijiang   add OTP for s5k5e2_pop10_cmcc
* 14/05/26      Hu Jin       /4 for 1st format version.
* ------------------------------------------------------------------
*/

#ifndef OV5670_M823_ORANGE_V4L2_TCT_OTP
#define OV5670_M823_ORANGE_V4L2_TCT_OTP

/* This file is added for bayer sensor OV5670 Truly FF/AF modules. */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

#define RG_RATIO_TYPICAL  593
#define BG_RATIO_TYPICAL  593

struct ov5670_otp_struct {
    uint16_t module_integrator_id;
    uint16_t lens_id;
	uint16_t vcm_id;
	uint16_t driver_ic;
    uint16_t production_year;
    uint16_t production_month;
    uint16_t production_day;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
};
extern int actuator_exist;
static int32_t ov5670_otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl, 
    uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 
		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

static uint16_t ov5670_otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, 
	uint32_t addr)
{
	uint16_t data = 0;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	return data;
}

#if 0
static uint32_t otp_get_modinfo_group(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint16_t flag = 0, temp = 0;

    ov5670_otp_i2c_write(s_ctrl, 0x0100, 0x01);
	//set 0x5002[1] to 0
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    ov5670_otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    //partial mode OTP write start address
    ov5670_otp_i2c_write(s_ctrl, 0x3d88, 0x70);
    ov5670_otp_i2c_write(s_ctrl, 0x3d89, 0x10);
    // partial mode OTP write end address
    ov5670_otp_i2c_write(s_ctrl, 0x3d8A, 0x70);
    ov5670_otp_i2c_write(s_ctrl, 0x3d8B, 0x10);
    // read otp into buffer
    ov5670_otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    flag = ov5670_otp_i2c_read(s_ctrl, 0x7010);
    // clear otp buffer
    ov5670_otp_i2c_write(s_ctrl, 0x7010, 0x00);
	//set 0x5002[1] to 1
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));

	printk(KERN_DEBUG"%s:%d, modinfo flag=%u\n", __func__, __LINE__, flag);
    if ((flag & 0xC0) == 0x40) {
		return 1;
    } else if ((flag & 0x30) == 0x10) {
        return 2;
    } else if ((flag & 0x0C) == 0x04) {
        return 3;
    } else {
        return 0;
    }
}
#endif
// read out the awb group index.
// return: 1,2,3 for success,
//         0 for fail.
static uint32_t otp_get_awb_group(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint16_t flag = 0, temp = 0;
	
    ov5670_otp_i2c_write(s_ctrl, 0x0100, 0x01);
    // set 0x5002[1] to 0
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    ov5670_otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    // partial mode OTP write start address
    ov5670_otp_i2c_write(s_ctrl, 0x3d88, 0x70);
    ov5670_otp_i2c_write(s_ctrl, 0x3d89, 0x26);
    // partial mode OTP write end address
    ov5670_otp_i2c_write(s_ctrl, 0x3d8A, 0x70);
    ov5670_otp_i2c_write(s_ctrl, 0x3d8B, 0x26);
    // read otp into buffer
    ov5670_otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    flag = ov5670_otp_i2c_read(s_ctrl, 0x7026);
    // clear otp buffer
    ov5670_otp_i2c_write(s_ctrl, 0x7026, 0x00);
    //set 0x5002[1] to 1
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
	
    printk(KERN_DEBUG"%s:%d, awb flag=%u\n", __func__, __LINE__, flag);
    if ((flag & 0xC0) == 0x40) {
		return 1;
    } else if ((flag & 0x30) == 0x10) {
        return 2;
    } else if ((flag & 0x0C) == 0x04) {
        return 3;
    } else {
        return 0;
    }
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0
#if 0
static int32_t otp_read_modinfo(struct msm_sensor_ctrl_t *s_ctrl, 
    uint32_t index, struct ov5670_otp_struct *otp_ptr)
{
    uint16_t temp = 0;
    uint32_t i = 0, start_addr = 0, end_addr = 0;

    if (index == 1) {
    	start_addr = 0x7011;
    	end_addr = 0x7017;
    } else if (index == 2) {
    	start_addr = 0x7018;
    	end_addr = 0x701E;
    } else if (index == 3) {
    	start_addr = 0x701F;
    	end_addr = 0x7025;
    } else {
        printk(KERN_ERR"%s:%d, invalid group id=%u\n", __func__, __LINE__, index);
		return -EINVAL;
    }
    //set 0x5002[1] to 0
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    ov5670_otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    //partial mode OTP write start address
    ov5670_otp_i2c_write(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff);
    ov5670_otp_i2c_write(s_ctrl, 0x3d89, (start_addr & 0xff));
    // partial mode OTP write end address
    ov5670_otp_i2c_write(s_ctrl, 0x3d8A, (end_addr >> 8) & 0xff);
    ov5670_otp_i2c_write(s_ctrl, 0x3d8B, (end_addr & 0xff));
    // read otp into buffer
    ov5670_otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    (*otp_ptr).module_integrator_id = ov5670_otp_i2c_read(s_ctrl, start_addr);
    (*otp_ptr).lens_id = ov5670_otp_i2c_read(s_ctrl, (start_addr + 1));
	(*otp_ptr).vcm_id = ov5670_otp_i2c_read(s_ctrl, (start_addr + 2));
	(*otp_ptr).driver_ic = ov5670_otp_i2c_read(s_ctrl, (start_addr + 3));
    (*otp_ptr).production_year = ov5670_otp_i2c_read(s_ctrl, (start_addr + 4));
    (*otp_ptr).production_month = ov5670_otp_i2c_read(s_ctrl, (start_addr + 5));
    (*otp_ptr).production_day = ov5670_otp_i2c_read(s_ctrl, (start_addr + 6));
	printk(KERN_DEBUG"%s:%d, module id=0x%x, lens id=0x%x, vcm id=0x%x, driver ic=0x%x, "
		   "year=%u, month=%u, day=%u\n",
		   __func__, __LINE__,
		   (*otp_ptr).module_integrator_id, (*otp_ptr).lens_id, 
		   (*otp_ptr).vcm_id, (*otp_ptr).driver_ic,
		   (*otp_ptr).production_year, (*otp_ptr).production_month,
		   (*otp_ptr).production_day);
    // clear otp buffer
    for (i = start_addr; i <= end_addr; i++) {
    	ov5670_otp_i2c_write(s_ctrl, i, 0x00);
    }
    //set 0x5002[1] to 1
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
    return 0;
}
#endif
// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return:0
static int32_t ov5670_otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t index, struct ov5670_otp_struct *otp_ptr)
{
    uint32_t i = 0, start_addr = 0, end_addr = 0;
    uint16_t temp = 0, rg_gld = 0, bg_gld = 0;
	
    if (index == 1) {
    	start_addr = 0x7028;
    	end_addr = 0x702F;
    } else if (index == 2) {
    	start_addr = 0x7031;
    	end_addr = 0x7038;
    } else if (index == 3) {
    	start_addr = 0x703A;
    	end_addr = 0x7041;
    } else {
        printk(KERN_ERR"%s:%d, invalid group id=%u\n", __func__, __LINE__, index);
		return -EINVAL;
    }
	// set 0x5002[1] to 0
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    ov5670_otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    // partial mode OTP write start address
    ov5670_otp_i2c_write(s_ctrl, 0x3d88, ((start_addr >> 8) & 0xff));
    ov5670_otp_i2c_write(s_ctrl, 0x3d89, (start_addr & 0xff));
    // partial mode OTP write end address
    ov5670_otp_i2c_write(s_ctrl, 0x3d8A, ((end_addr >> 8) & 0xff));
    ov5670_otp_i2c_write(s_ctrl, 0x3d8B, (end_addr & 0xff));
    // read otp into buffer
    ov5670_otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    temp = ov5670_otp_i2c_read(s_ctrl, (start_addr + 3));
    (*otp_ptr).rg_ratio = (ov5670_otp_i2c_read(s_ctrl, start_addr) << 2) + ((temp >> 6) & 0x03);
    (*otp_ptr).bg_ratio = (ov5670_otp_i2c_read(s_ctrl, (start_addr + 1)) << 2) + ((temp >> 4) & 0x03);
	printk(KERN_DEBUG"%s:%d, group=%u, [RG_Typical:RG]=[%u:%u], [BG_Typical:BG]=[%u:%u]\n", 
		    __func__, __LINE__,
		    index, 
		    RG_RATIO_TYPICAL, (*otp_ptr).rg_ratio, 
		    BG_RATIO_TYPICAL, (*otp_ptr).bg_ratio);
    temp = ov5670_otp_i2c_read(s_ctrl, (start_addr + 7));
    rg_gld = (ov5670_otp_i2c_read(s_ctrl, (start_addr + 4)) << 2) + ((temp >> 6) & 0x03);
    bg_gld = (ov5670_otp_i2c_read(s_ctrl, (start_addr + 5)) << 2) + ((temp >> 4) & 0x03);
    printk(KERN_DEBUG"%s:%d, group=%u, rg_gld=%u, bg_gld=%u\n",
		   __func__, __LINE__,
		   index,
		   rg_gld, bg_gld);
    // clear otp buffer
    for (i = start_addr; i <= end_addr; i++) {
    	ov5670_otp_i2c_write(s_ctrl, i, 0x00);
    }
    //set 0x5002[1] to 1
    temp = ov5670_otp_i2c_read(s_ctrl, 0x5002);
    ov5670_otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
    return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0
static int32_t ov5670_otp_update_awb(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    if (R_gain > 0x400) {
    	ov5670_otp_i2c_write(s_ctrl, 0x5032, (R_gain >> 8));
    	ov5670_otp_i2c_write(s_ctrl, 0x5033, (R_gain & 0x00ff));
    }
    if (G_gain > 0x400) {
    	ov5670_otp_i2c_write(s_ctrl, 0x5034, (G_gain >> 8));
    	ov5670_otp_i2c_write(s_ctrl, 0x5035, (G_gain & 0x00ff));
    }
    if (B_gain > 0x400) {
    	ov5670_otp_i2c_write(s_ctrl, 0x5036, (B_gain >> 8));
    	ov5670_otp_i2c_write(s_ctrl, 0x5037, (B_gain & 0x00ff));
    }
    return 0;
}

int32_t ov5670_m823_orange_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct ov5670_otp_struct current_otp = {0};
    uint32_t otp_index = 0;
    uint32_t nR_G_gain = 0, nB_G_gain = 0, nG_G_gain = 0, nBase_gain = 0, 
		R_gain = 0, B_gain = 0, G_gain = 0;

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	//mutex_lock(s_ctrl->msm_sensor_mutex);
    // get first OTP group with valid data
    otp_index = otp_get_awb_group(s_ctrl);
    if (otp_index >= 1 && otp_index <= 3) {
        printk(KERN_DEBUG"%s:%d, OTP awb group=%u\n", __func__, __LINE__, otp_index);
        ov5670_otp_read_awb(s_ctrl, otp_index, &current_otp);
        //calculate G gain
        printk(KERN_DEBUG"OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", 
        				RG_RATIO_TYPICAL, current_otp.rg_ratio,
        				BG_RATIO_TYPICAL, current_otp.bg_ratio);
        nR_G_gain = (RG_RATIO_TYPICAL * 1000) / current_otp.rg_ratio;
        nB_G_gain = (BG_RATIO_TYPICAL * 1000) / current_otp.bg_ratio;
        nG_G_gain = 1000;
        if (nR_G_gain < 1000 || nB_G_gain < 1000) {
            if (nR_G_gain < nB_G_gain) {
            	nBase_gain = nR_G_gain;
            } else {
            	nBase_gain = nB_G_gain;
            }
        } else {
        	nBase_gain = nG_G_gain;
        }
        R_gain = 0x400 * nR_G_gain / (nBase_gain);
        B_gain = 0x400 * nB_G_gain / (nBase_gain);
        G_gain = 0x400 * nG_G_gain / (nBase_gain);
        ov5670_otp_update_awb(s_ctrl, R_gain, G_gain, B_gain);
    }
	//mutex_unlock(s_ctrl->msm_sensor_mutex);
	printk(KERN_DEBUG"%s:%d, exit\n", __func__, __LINE__);
	return 0;
}

#if 0
uint16_t ov5670_qtech_m823_otp_get_vcm_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct ov5670_otp_struct current_otp = {0};
	uint32_t otp_index = 0;

    printk(KERN_DEBUG"%s:%d\n", __func__, __LINE__);
	//mutex_lock(s_ctrl->msm_sensor_mutex);
	otp_index = otp_get_modinfo_group(s_ctrl);
    if (otp_index >= 1 && otp_index <= 3) {
        printk(KERN_DEBUG"%s:%d, OTP modinfo group=%u\n", __func__, __LINE__, otp_index);
    	otp_read_modinfo(s_ctrl, otp_index, &current_otp);
    }
	//mutex_unlock(s_ctrl->msm_sensor_mutex);
	printk(KERN_DEBUG"%s:%d, exit\n", __func__, __LINE__);
	if (current_otp.vcm_id)
		actuator_exist = 1;
	else
		actuator_exist = 0;
	return (current_otp.vcm_id);
}
#endif

#endif

