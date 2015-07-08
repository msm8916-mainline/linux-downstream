#include "msm_sensor.h"
#include "msm_sensor_otp.h"
#include "msm_cci.h"

static int OV5693_RG_RATIO_TYPICAL = 0x118;
static int OV5693_BG_RATIO_TYPICAL = 0x11b;

static int OV13850_RG_RATIO_TYPICAL = 0x126;
static int OV13850_BG_RATIO_TYPICAL = 0x127;

static unsigned short OV13850_EEPROM_ADDR = 0x58;
static unsigned short OV13850_SENSOR_ADDR = 0x36;

static struct msm_sensor_ctrl_t *ov5693_otp_s_ctrl = NULL;
static struct msm_sensor_ctrl_t *ov13850_otp_s_ctrl = NULL;

int32_t ov5693_write_i2c(uint16_t addr, uint16_t data) {
	return ov5693_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		ov5693_otp_s_ctrl->sensor_i2c_client, addr, data,
		MSM_CAMERA_I2C_BYTE_DATA);
}

uint16_t ov5693_read_i2c(uint16_t addr) {
	uint16_t temp;

	ov5693_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		ov5693_otp_s_ctrl->sensor_i2c_client, addr, &temp,
		MSM_CAMERA_I2C_BYTE_DATA);

	return temp;
}

int32_t ov13850_otp_write_i2c(uint16_t addr, uint16_t data) {
	return ov13850_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		ov13850_otp_s_ctrl->sensor_i2c_client, addr, data,
		MSM_CAMERA_I2C_BYTE_DATA);
}

uint16_t ov13850_otp_read_i2c(uint16_t addr) {
	uint16_t temp;

	ov13850_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		ov13850_otp_s_ctrl->sensor_i2c_client, addr, &temp,
		MSM_CAMERA_I2C_BYTE_DATA);

	return temp;
}


/*==================================================================
  =============               ov5693                 ===============
  ==================================================================*/


// index: index of otp group. (1, 2, 3)
// return:  0, group index is empty
// 			1, group index has invalid data
// 			2, group index has valid data
int ov5693_check_otp_wb(int index) {
	int flag, i;
	int bank, address;

	// select bank index
	bank = 0xc0 | index;
	ov5693_write_i2c(0x3d84, bank);

	// read otp into buffer
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	// read flag
	address = 0x3d00;
	flag = ov5693_read_i2c(address);
	flag = flag & 0xc0;

	// clear otp buffer
	for (i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	if (flag == 0x00) {
		return 0;
	} else if(flag & 0x80) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// return:  0, group index is empty
// 			1, group index has invalid data
// 			2, group index has valid data
int ov5693_check_otp_lenc(int index) {
	int flag, i, bank;
	int address;

	// select bank: 4, 8, 12
	bank = 0xc0 | (index * 4);
	ov5693_write_i2c(0x3d84, bank);

	// read otp into buffer
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	// read flag
	address = 0x3d00;
	flag = ov5693_read_i2c(address);
	flag = flag & 0xc0;

	// clear otp buffer
	for (i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	if (flag == 0x00) {
		return 0;
	} else if (flag & 0x80) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct_ov5693
// return:  0,
int ov5693_read_otp_wb(int index, struct otp_struct_ov5693 *otp_ptr) {
	int i, bank;
	int address;
	int temp;

	// select bank index
	bank = 0xc0 | index;
	ov5693_write_i2c(0x3d84, bank);

	// read otp into buffer
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	address = 0x3d00;
	(*otp_ptr).module_integrator_id = ov5693_read_i2c(address + 1);
	(*otp_ptr).lens_id = ov5693_read_i2c(address + 2);
	(*otp_ptr).production_year = ov5693_read_i2c(address + 3);
	(*otp_ptr).production_month = ov5693_read_i2c(address + 4);
	(*otp_ptr).production_day = ov5693_read_i2c(address + 5);
	temp = ov5693_read_i2c(address + 10);
	(*otp_ptr).rg_ratio = (ov5693_read_i2c(address + 6)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (ov5693_read_i2c(address + 7)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (ov5693_read_i2c(address + 8) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (ov5693_read_i2c(address + 9)<<2) + (temp & 0x03);
	(*otp_ptr).user_data[0] = ov5693_read_i2c(address + 11);
	(*otp_ptr).user_data[1] = ov5693_read_i2c(address + 12);
	(*otp_ptr).user_data[2] = ov5693_read_i2c(address + 13);
	(*otp_ptr).user_data[3] = ov5693_read_i2c(address + 14);
	(*otp_ptr).user_data[4] = ov5693_read_i2c(address + 15);

	/*
	printk("==========OV5693 OTP INFO BEGIN==========\n");
	printk("%s-module_integrator_id	= 0x%x\n", __func__, otp_ptr->module_integrator_id);
	printk("%s-lens_id				= 0x%x\n", __func__, otp_ptr->lens_id);
	printk("%s-production_year		= 0x%x\n", __func__, otp_ptr->production_year);
	printk("%s-production_month		= 0x%x\n", __func__, otp_ptr->production_month);
	printk("%s-production_day		= 0x%x\n", __func__, otp_ptr->production_day);
	printk("%s-rg_ratio				= 0x%x\n", __func__, otp_ptr->rg_ratio);
	printk("%s-bg_ratio				= 0x%x\n", __func__, otp_ptr->bg_ratio);
	printk("%s-light_rg				= 0x%x\n", __func__, otp_ptr->light_rg);
	printk("%s-light_bg				= 0x%x\n", __func__, otp_ptr->light_bg);
	printk("%s-lenc[0]				= 0x%x\n", __func__, otp_ptr->lenc[0]);
	printk("%s-lenc[1]				= 0x%x\n", __func__, otp_ptr->lenc[1]);
	printk("==========OV5693 OTP INFO END==========\n");
	*/

	// clear otp buffer
	for (i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct_ov5693
// return:  0,
int ov5693_read_otp_lenc(int index, struct otp_struct_ov5693 *otp_ptr) {
	int bank, i;
	int address;

	// select bank: 4, 8, 12
	bank = 0xc0 | (index * 4);
	ov5693_write_i2c(0x3d84, bank);

	// read otp into buffer
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	address = 0x3d01;
	for (i = 0; i < 15; i++) {
		(* otp_ptr).lenc[i] = ov5693_read_i2c(address);
		address++;
	}

	// clear otp buffer
	for (i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	// select 2nd bank
	bank++;
	ov5693_write_i2c(0x3d84, bank);

	// read otp
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	address = 0x3d00;
	for (i = 15; i < 31; i++) {
		(* otp_ptr).lenc[i] = ov5693_read_i2c(address);
		address++;
	}

	// clear otp buffer
	for (i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	// select 3rd bank
	bank++;
	ov5693_write_i2c(0x3d84, bank);

	// read otp
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	address = 0x3d00;
	for (i = 31; i < 47; i++) {
		(* otp_ptr).lenc[i] = ov5693_read_i2c(address);
		address++;
	}

	// clear otp buffer
	for(i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	// select 4th bank
	bank++;
	ov5693_write_i2c(0x3d84, bank);

	// read otp
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	address = 0x3d00;
	for(i = 47; i < 62; i++) {
		(* otp_ptr).lenc[i] = ov5693_read_i2c(address);
		address++;
	}

	// clear otp buffer
	for(i = 0; i < 16; i++) {
		ov5693_write_i2c(0x3d00 + i, 0x00);
	}

	return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int ov5693_update_awb_gain(int R_gain, int G_gain, int B_gain) {

	if (R_gain > 0x400) {
		ov5693_write_i2c(0x3400, R_gain>>8);
		ov5693_write_i2c(0x3401, R_gain & 0x00ff);
	}
	if (G_gain > 0x400) {
		ov5693_write_i2c(0x3402, G_gain>>8);
		ov5693_write_i2c(0x3403, G_gain & 0x00ff);
	}
	if (B_gain > 0x400) {
		ov5693_write_i2c(0x3404, B_gain>>8);
		ov5693_write_i2c(0x3405, B_gain & 0x00ff);
	}

	return 0;
}

// otp_ptr: pointer of otp_struct_ov5693
int ov5693_update_lenc(struct otp_struct_ov5693 * otp_ptr) {
	int i, temp;

	temp = ov5693_read_i2c(0x5000);
	temp = 0x80 | temp;
	ov5693_write_i2c(0x5000, temp);
	for (i = 0; i < 62; i++) {
		ov5693_write_i2c(0x5800 + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}

// call this function after OV5693 initialization
// return value: 0 update success
// 1, no OTP
int ov5693_update_otp_wb(struct msm_sensor_ctrl_t *s_ctrl) {
	struct otp_struct_ov5693 current_otp;
	int i;
	int otp_index;
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg, bg;
	ov5693_otp_s_ctrl = s_ctrl;

	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for (i = 1; i <= 3; i++) {
		temp = ov5693_check_otp_wb(i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}

	if (i > 3) {
		// no valid wb OTP data
		return 1;
	}

	ov5693_read_otp_wb(otp_index, &current_otp);

	if (current_otp.light_rg == 0) {
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	} else {
		rg = current_otp.rg_ratio * ((current_otp.light_rg + 512) / 1024);
	}

	if(current_otp.light_bg == 0) {
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	} else {
		bg = current_otp.bg_ratio * ((current_otp.light_bg +512) / 1024);
	}

	//calculate G gain
	//0x400 = 1x gain
	if (bg < OV5693_BG_RATIO_TYPICAL) {
		if (rg < OV5693_RG_RATIO_TYPICAL) {
			// current_otp.bg_ratio < OV5693_BG_RATIO_TYPICAL &&
			// current_otp.rg_ratio < OV5693_RG_RATIO_TYPICAL
			G_gain = 0x400;
			B_gain = 0x400 * OV5693_BG_RATIO_TYPICAL / bg;
			R_gain = 0x400 * OV5693_RG_RATIO_TYPICAL / rg;
		} else {
			// current_otp.bg_ratio < OV5693_BG_RATIO_TYPICAL &&
			// current_otp.rg_ratio >= OV5693_RG_RATIO_TYPICAL
			R_gain = 0x400;
			G_gain = 0x400 * rg / OV5693_RG_RATIO_TYPICAL;
			B_gain = G_gain * OV5693_BG_RATIO_TYPICAL /bg;
		}
	} else {
		if (rg < OV5693_RG_RATIO_TYPICAL) {
			// current_otp.bg_ratio >= OV5693_BG_RATIO_TYPICAL &&
			// current_otp.rg_ratio < OV5693_RG_RATIO_TYPICAL
			B_gain = 0x400;
			G_gain = 0x400 * bg / OV5693_BG_RATIO_TYPICAL;
			R_gain = G_gain * OV5693_RG_RATIO_TYPICAL / rg;
		} else {
			// current_otp.bg_ratio >= OV5693_BG_RATIO_TYPICAL &&
			// current_otp.rg_ratio >= OV5693_RG_RATIO_TYPICAL
			G_gain_B = 0x400 * bg / OV5693_BG_RATIO_TYPICAL;
			G_gain_R = 0x400 * rg / OV5693_RG_RATIO_TYPICAL;

			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * OV5693_RG_RATIO_TYPICAL /rg;
			} else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * OV5693_BG_RATIO_TYPICAL / bg;
			}
		}
	}

	ov5693_update_awb_gain(R_gain, G_gain, B_gain);
	return 0;
}

// call this function after OV5693 initialization
// return value: 0 update success
// 1, no OTP
int ov5693_update_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl) {
	struct otp_struct_ov5693 current_otp;
	int i;
	int otp_index;
	int temp;
	ov5693_otp_s_ctrl = s_ctrl;

	// check first lens correction OTP with valid data
	for (i = 1; i <= 3; i++) {
		temp = ov5693_check_otp_lenc(i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if(i > 3) {
		// no valid WB OTP data
		return 1;
	}

	ov5693_read_otp_lenc(otp_index, &current_otp);
	ov5693_update_lenc(&current_otp);

	// success
	return 0;
}

// call this function after OV5693 initialization
// return value:  1 use CP data from REG3D0A
// 2 use Module data from REG3D0B
// 0 data Error
int ov5693_update_blc_ratio(struct msm_sensor_ctrl_t *s_ctrl) {
	int K;
	int temp;
	ov5693_otp_s_ctrl = s_ctrl;

	ov5693_write_i2c(0x3d84, 0xdf);
	ov5693_write_i2c(0x3d81, 0x01);
	usleep(5);

	K = ov5693_read_i2c(0x3d0b);
	if (K != 0) {
		if (K >= 0x15 && K <= 0x40) {
			// auto load mode
			temp = ov5693_read_i2c(0x4008);
			temp &= 0xfb;
			ov5693_write_i2c(0x4008, temp);
			temp = ov5693_read_i2c(0x4000);
			temp &= 0xf7;
			ov5693_write_i2c(0x4000, temp);

			return 2;
		}
	}

	K = ov5693_read_i2c(0x3d0a);
	if (K >= 0x10 && K <= 0x40) {
		// manual load mode
		ov5693_write_i2c(0x4006, K);
		temp = ov5693_read_i2c(0x4008);
		temp &= 0xfb;
		ov5693_write_i2c(0x4008, temp);
		temp = ov5693_read_i2c(0x4000);
		temp |= 0x08;
		ov5693_write_i2c(0x4000, temp);

		return 1;
	} else {
		// set to default
		ov5693_write_i2c(0x4006, 0x20);
		temp = ov5693_read_i2c(0x4008);
		temp &= 0xfb;
		ov5693_write_i2c(0x4008, temp);
		temp = ov5693_read_i2c(0x4000);
		temp |= 0x08;
		ov5693_write_i2c(0x4000, temp);

		return 0;
	}
}

int ov5693_update_otp(struct msm_sensor_ctrl_t *s_ctrl) {

	ov5693_update_otp_lenc(s_ctrl);
	ov5693_update_otp_wb(s_ctrl);
	//ov5693_update_blc_ratio(s_ctrl);

	return 0;
}

/*==================================================================
  =============               ov13850                ===============
  ==================================================================*/

int adjust_i2c_address(struct msm_sensor_ctrl_t *s_ctrl, unsigned short addr) {
	printk("%s:%d addr: 0x%x \n", __func__, __LINE__, addr);

	if (s_ctrl->sensor_i2c_client->client) {
		s_ctrl->sensor_i2c_client->client->addr = addr;
	} else if (s_ctrl->sensor_i2c_client->cci_client) {
		s_ctrl->sensor_i2c_client->cci_client->sid = addr;
	} else {
		printk("%s:%d client and cci_client is null! \n", __func__, __LINE__);
		return 1;
	}
	return 0;
}

// index: index of otp group. (1, 2,...,OTP_DRV_INFO_GROUP_COUNT)
// otp_ptr: pointer of otp_struct
// return: 0,
int ov13850_read_otp_info(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct_ov13850 *otp_ptr) {
	int i = 0;
	unsigned short nFlag, value_0x059E;
	uint16_t eeprom_value[415], addr;
	unsigned int eeprom_value_total = 0x00;

	ov13850_otp_s_ctrl = s_ctrl;
	adjust_i2c_address(s_ctrl, OV13850_EEPROM_ADDR);
	usleep(5);

	// 1. read flag
	nFlag = ov13850_otp_read_i2c(OTP_DRV_START_ADDR);
	if ((nFlag & 0x01) == 0x00) {
		printk("%s:%d, invalid otp info! flag = 0x%04x\n", __func__, __LINE__, nFlag);
		return 1;
	}

	// 2. calculate sum of value
	for (addr = OTP_DRV_START_ADDR; addr <= OTP_DRV_END_ADDR; addr++) {
		eeprom_value[i] = (unsigned short)ov13850_otp_read_i2c(addr);
		/*
		printk("%s:%d: add:0x%04x, value[%03d]: 0x%04x \n", __func__, __LINE__,
			addr, i, eeprom_value[i]);
		*/
		i++;
	}

	value_0x059E = eeprom_value[414];

	// except 0x0400 and 0x059E, SUM is 413.
	for (i = 1; i <= 413; i++) {
		eeprom_value_total += eeprom_value[i];
		//printk("%s:%d: total: 0x%04x \n",  __func__, __LINE__, eeprom_value_total);
	}

	if (value_0x059E != (eeprom_value_total % 0xFF + 1)) {
		printk("%s:%d Verify failed! value_0x059E: %x, eeprom_value_total: %x --> %x \n",
			__func__, __LINE__, value_0x059E, eeprom_value_total,
			(eeprom_value_total % 0xFF + 1));
		return 1;
	}

	// 3. read module info
	(*otp_ptr).module_integrator_id = eeprom_value[1];
	(*otp_ptr).lens_id = eeprom_value[9];
	(*otp_ptr).production_year = eeprom_value[5];
	(*otp_ptr).production_month = eeprom_value[6];
	(*otp_ptr).production_day = eeprom_value[7];

	// 4. read wb info
	(*otp_ptr).rg_ratio = (eeprom_value[19] << 2) + ((eeprom_value[20] >> 6) & 0x03);
	(*otp_ptr).bg_ratio = (eeprom_value[21] << 2) + ((eeprom_value[22] >> 6) & 0x03);
	(*otp_ptr).light_rg = 0;
	(*otp_ptr).light_bg = 0;

	// 5. read lenc info
	// from 0x042F(1071) to 0x0596(1430). SUM is 360.
	for (i = 0; i < OTP_DRV_LSC_SIZE; i++) {
		(*otp_ptr).lenc[i] = eeprom_value[47 + i];
	}

	// 6. read vcm info
	//(*otp_ptr).VCM_start = (ov13850_otp_read_i2c(OTP_DRV_START_ADDR)<<2) | ((temp>>6) & 0x03);
	//(*otp_ptr).VCM_end = (ov13850_otp_read_i2c(OTP_DRV_START_ADDR + 1) << 2) | ((temp>>4) & 0x03);
	//(*otp_ptr).VCM_dir = (temp>>2) & 0x03;

	/*
	printk("==========OV13850 OTP INFO BEGIN==========\n");
	printk("%s-module_integrator_id	= 0x%x\n", __func__, otp_ptr->module_integrator_id);
	printk("%s-lens_id				= 0x%x\n", __func__, otp_ptr->lens_id);
	printk("%s-production_year		= 0x%x\n", __func__, otp_ptr->production_year);
	printk("%s-production_month		= 0x%x\n", __func__, otp_ptr->production_month);
	printk("%s-production_day		= 0x%x\n", __func__, otp_ptr->production_day);
	printk("%s-rg_ratio				= 0x%x\n", __func__, otp_ptr->rg_ratio);
	printk("%s-bg_ratio				= 0x%x\n", __func__, otp_ptr->bg_ratio);
	printk("%s-light_rg				= 0x%x\n", __func__, otp_ptr->light_rg);
	printk("%s-light_bg				= 0x%x\n", __func__, otp_ptr->light_bg);
	printk("%s-lenc[0]				= 0x%x\n", __func__, otp_ptr->lenc[0]);
	printk("%s-lenc[1]				= 0x%x\n", __func__, otp_ptr->lenc[1]);
	//printk("%s-VCM_start			= 0x%x\n", __func__, otp_ptr->VCM_start);
	//printk("%s-VCM_end			= 0x%x\n", __func__, otp_ptr->VCM_end);
	//printk("%s-VCM_dir			= 0x%x\n", __func__, otp_ptr->VCM_dir);
	printk("==========OV13850 OTP INFO END==========\n");
	*/

	adjust_i2c_address(s_ctrl, OV13850_SENSOR_ADDR);

	return 0;
}

int ov13850_apply_otp(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct_ov13850 *otp_ptr) {
	int i;
	int temp;
	int rg, bg;
	int nR_G_gain, nB_G_gain, nG_G_gain;
	int nBase_gain;

	ov13850_otp_s_ctrl = s_ctrl;

	// 1. update wb
	if ((*otp_ptr).light_rg == 0) {
		// no light source information in OTP, light factor = 1
		rg = (*otp_ptr).rg_ratio;
	} else {
		rg = (*otp_ptr).rg_ratio * ((*otp_ptr).light_rg + 512) / 1024;
	}

	if ((*otp_ptr).light_bg == 0) {
		// not light source information in OTP, light factor = 1
		bg = (*otp_ptr).bg_ratio;
	} else {
		bg = (*otp_ptr).bg_ratio * ((*otp_ptr).light_bg + 512) / 1024;
	}

	//calculate G gain
	nR_G_gain = (OV13850_RG_RATIO_TYPICAL * 1000) / rg;
	nB_G_gain = (OV13850_BG_RATIO_TYPICAL * 1000) / bg;
	nG_G_gain = 1000;

	if (nR_G_gain < 1000 || nB_G_gain < 1000) {
		if (nR_G_gain < nB_G_gain)
			nBase_gain = nR_G_gain;
		else
			nBase_gain = nB_G_gain;
	} else {
		nBase_gain = nG_G_gain;
	}

	nR_G_gain = 0x400 * nR_G_gain / (nBase_gain);
	nB_G_gain = 0x400 * nB_G_gain / (nBase_gain);
	nG_G_gain = 0x400 * nG_G_gain / (nBase_gain);

	if (nR_G_gain > 0x400) {
		ov13850_otp_write_i2c(0x5056, nR_G_gain >> 8);
		ov13850_otp_write_i2c(0x5057, nR_G_gain & 0x00ff);
	}

	if(nG_G_gain > 0x400) {
		ov13850_otp_write_i2c(0x5058, nG_G_gain >> 8);
		ov13850_otp_write_i2c(0x5059, nG_G_gain & 0x00ff);
	}

	if(nB_G_gain > 0x400) {
		ov13850_otp_write_i2c(0x505A, nB_G_gain >> 8);
		ov13850_otp_write_i2c(0x505B, nB_G_gain & 0x00ff);
	}

	// 2. update lenc
	temp = ov13850_otp_read_i2c(0x5000);
	temp = 0x01 | temp;
	ov13850_otp_write_i2c(0x5000, temp);

	for (i = 0; i < OTP_DRV_LSC_SIZE; i++) {
		ov13850_otp_write_i2c(OTP_DRV_LSC_REG_ADDR + i, (*otp_ptr).lenc[i]);
	}

	// 3. update VCM [TODO:]

	return 0;
}

int ov13850_update_otp(struct msm_sensor_ctrl_t *s_ctrl) {
	struct otp_struct_ov13850 current_otp;
	int ret;

	ret = ov13850_read_otp_info(s_ctrl, &current_otp);
	if (ret == 1) {
		printk("%s:%d read otp error! \n", __func__, __LINE__);
		adjust_i2c_address(s_ctrl, OV13850_SENSOR_ADDR);
		return 0;
	}
	ov13850_apply_otp(s_ctrl, &current_otp);

	return 0;
}
