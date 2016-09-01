/* This file is added for bayer sensor S5K5E2 modules. */

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

struct s5k5e2_otp_struct {
    uint16_t RG_gain, BG_gain, G_gain;
    uint16_t golden_RG, golden_BG, golden_G;
    uint16_t typical_RG, typical_BG;
};

static int32_t otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl, 
    uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 
		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

static uint16_t otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, 
	uint32_t addr)
{
	uint16_t data = 0;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	return data;
}

static uint32_t otp_get_awb_page(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t flag = 0;

	otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A02, 0x02); // set page 2
	otp_i2c_write(s_ctrl, 0x0A00, 0x01); // otp read enable
	mdelay(1);
	flag = otp_i2c_read(s_ctrl, 0x0A43); // read awb flag
	if ((flag & 0xFF) == 0x01) {
	    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	    otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	    return 2; // get page 2
	}
	otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A02, 0x03); // set page 3
	otp_i2c_write(s_ctrl, 0x0A00, 0x01); // otp read enable
	mdelay(1);
	flag = otp_i2c_read(s_ctrl, 0x0A43); // read awb flag
	if ((flag & 0xFF) == 0x01) {
	    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	    otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	    return 3; // get page 3
	}
    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A02, 0x04); // set page 4
	otp_i2c_write(s_ctrl, 0x0A00, 0x01); // otp read enable
	mdelay(1);
	flag = otp_i2c_read(s_ctrl, 0x0A43); // read awb flag
	if ((flag & 0xFF) == 0x01) {
	    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	    otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	    return 4; // get page 4
	}
    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
    otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	return 0; // invalid page
}

static int32_t otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t otp_index, struct s5k5e2_otp_struct *otp_ptr)
{
    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A02, otp_index); // set page
	otp_i2c_write(s_ctrl, 0x0A00, 0x01); // otp read enable
	mdelay(1);
	otp_ptr->RG_gain = 
		(otp_i2c_read(s_ctrl, 0x0A0F) << 8) + otp_i2c_read(s_ctrl, 0x0A10);
	otp_ptr->BG_gain = 
		(otp_i2c_read(s_ctrl, 0x0A11) << 8) + otp_i2c_read(s_ctrl, 0x0A12);
	otp_ptr->G_gain = 
		(otp_i2c_read(s_ctrl, 0x0A13) << 8) + otp_i2c_read(s_ctrl, 0x0A14);
	otp_ptr->golden_RG = 
		(otp_i2c_read(s_ctrl, 0x0A16) << 8) + otp_i2c_read(s_ctrl, 0x0A17);
	otp_ptr->golden_BG = 
		(otp_i2c_read(s_ctrl, 0x0A18) << 8) + otp_i2c_read(s_ctrl, 0x0A19);
	otp_ptr->golden_G =
		(otp_i2c_read(s_ctrl, 0x0A1A) << 8) + otp_i2c_read(s_ctrl, 0x0A1B);
    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
    otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	return 0;
}

static int32_t otp_update_awb(struct msm_sensor_ctrl_t *s_ctrl,
    struct s5k5e2_otp_struct *otp_ptr)
{
    uint32_t G_gain_R = 0, G_gain_B = 0, R_gain = 0, B_gain = 0, G_gain = 0;
	
    if(otp_ptr->BG_gain < otp_ptr->typical_BG) {
        if (otp_ptr->RG_gain < otp_ptr->typical_RG){
            G_gain = 0x400;
            B_gain = 0x400 * otp_ptr->typical_BG / otp_ptr->BG_gain;
            R_gain = 0x400 * otp_ptr->typical_RG / otp_ptr->RG_gain;
        }else {
            R_gain = 0x400;
            G_gain = 0x400 * otp_ptr->RG_gain / otp_ptr->typical_RG;
            B_gain = G_gain * otp_ptr->typical_BG / otp_ptr->BG_gain;
        }
    }else {
        if (otp_ptr->RG_gain < otp_ptr->typical_RG) {
            B_gain = 0x400;
            G_gain = 0x400 * otp_ptr->BG_gain / otp_ptr->typical_BG;
            R_gain = G_gain * otp_ptr->typical_RG / otp_ptr->RG_gain;
        }else {
            G_gain_B = 0x400 * otp_ptr->BG_gain / otp_ptr->typical_BG;
            G_gain_R = 0x400 * otp_ptr->RG_gain / otp_ptr->typical_RG;
            if(G_gain_B > G_gain_R ) {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * otp_ptr->typical_RG / otp_ptr->RG_gain;
            }
            else {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain= G_gain * otp_ptr->typical_BG / otp_ptr->BG_gain;
            }
        }
    }
    //add for awb OTP calibration, factory forget to down scale gain
    R_gain = R_gain >> 2;
	G_gain = G_gain >> 2;
	B_gain = B_gain >> 2;
	//R Gain
	otp_i2c_write(s_ctrl, 0x0210, (R_gain >> 8));
	otp_i2c_write(s_ctrl, 0x0211, (R_gain & 0xff));
	//Gr gain
	otp_i2c_write(s_ctrl, 0x020E, (G_gain >> 8));
	otp_i2c_write(s_ctrl, 0x020F, (G_gain & 0xff));
	//Gb gain
	otp_i2c_write(s_ctrl, 0x0214, (G_gain >> 8));
	otp_i2c_write(s_ctrl, 0x0215, (G_gain & 0xff));
	//B gain
	otp_i2c_write(s_ctrl, 0x0212, (B_gain >> 8));
	otp_i2c_write(s_ctrl, 0x0213, (B_gain & 0xff));
	
	//printk(KERN_DEBUG "%s, R_gain=%u, G_gain=%u, B_gain=%u\n",
    //			__func__, R_gain, G_gain, B_gain);
    			
    return 0;
}

int32_t s5k5e2_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct s5k5e2_otp_struct current_otp = {0};
    uint32_t otp_index = 0;
	uint16_t flag = 0;

//	mutex_lock(s_ctrl->msm_sensor_mutex);
	/* LSC */
    otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A02, 0x05); // set page
	otp_i2c_write(s_ctrl, 0x0A00, 0x01); // otp read enable
	mdelay(1);
	flag = otp_i2c_read(s_ctrl, 0x0A43);
	if ((flag & 0xFF) == 0x01) {
		printk(KERN_DEBUG"%s, lsc valid, flag=%u\n", __func__, flag);
		otp_i2c_write(s_ctrl, 0x0100, 0x00);
		otp_i2c_write(s_ctrl, 0x3400, 0x00);
		mdelay(10);
		otp_i2c_write(s_ctrl, 0x0100, 0x01);
	} else {
	    printk(KERN_DEBUG"%s, lsc empty/invalid\n", __func__);
	}
	otp_i2c_write(s_ctrl, 0x0A00, 0x04); // otp read disable
	otp_i2c_write(s_ctrl, 0x0A00, 0x00);
	
    /* AWB */
    otp_index = otp_get_awb_page(s_ctrl);
    if (otp_index == 2 || otp_index == 3 || otp_index == 4) {
		printk(KERN_DEBUG"%s, awb valid, page=%u\n", __func__, otp_index);
        otp_read_awb(s_ctrl, otp_index, &current_otp);
		printk(KERN_DEBUG"%s, RG_gain=%u, BG_gain=%u, G_gain=%u\n"
    			"%s, golden_RG=%u, golden_BG=%u, golden_G=%u\n",
    			__func__, current_otp.RG_gain, current_otp.BG_gain, current_otp.G_gain,
    			__func__, current_otp.golden_RG, current_otp.golden_BG, current_otp.golden_G);
    	if(current_otp.RG_gain != 0 && current_otp.BG_gain != 0) {
			/* different modules have different typical RG/BG values */
			if (!strcmp(s_ctrl->sensordata->sensor_name, "s5k5e2_alto4evdo")) {
				current_otp.typical_RG = 803;
				current_otp.typical_BG = 656;
			} else {
				current_otp.typical_RG = 911;
				current_otp.typical_BG = 744;
			}
			printk(KERN_DEBUG"%s, sensor_name=%s\n"
				"%s, typical_RG=%u, typical_BG=%u\n", 
				__func__, s_ctrl->sensordata->sensor_name,
				__func__, current_otp.typical_RG, current_otp.typical_BG);
            otp_update_awb(s_ctrl, &current_otp);
    	}
    } else {
        printk(KERN_DEBUG"%s, awb empty/invalid\n", __func__);
    }
//	mutex_unlock(s_ctrl->msm_sensor_mutex);
	
	return 0;
}

EXPORT_SYMBOL(s5k5e2_otp_config);

