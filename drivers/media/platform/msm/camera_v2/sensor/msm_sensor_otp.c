#include "msm_sensor.h"
#include "msm_sensor_otp.h"
#include "msm_cci.h"

static int OV13850_RG_RATIO_TYPICAL = 0x12d;
static int OV13850_BG_RATIO_TYPICAL = 0x148;

static int OV13850_QTECH_RG_RATIO_TYPICAL = 0x12d;
static int OV13850_QTECH_BG_RATIO_TYPICAL = 0x148;

static unsigned short OV13850_EEPROM_ADDR = 0x58;
static unsigned short OV13850_SENSOR_ADDR = 0x36;

static struct msm_sensor_ctrl_t *ov13850_otp_s_ctrl = NULL;
static struct msm_sensor_ctrl_t *hi545_otp_s_ctrl = NULL;
int module_info[16], awb_info[29];

static int HI545_RG_RATIO_TYPICAL = 0x164;
static int HI545_BG_RATIO_TYPICAL = 0x137;


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

int32_t hi545_otp_write_i2c(uint16_t addr, uint16_t data) {
	return hi545_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		hi545_otp_s_ctrl->sensor_i2c_client, addr, data,
		MSM_CAMERA_I2C_BYTE_DATA);
}

uint16_t hi545_otp_read_i2c(uint16_t addr) {
	uint16_t temp;

	hi545_otp_s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		hi545_otp_s_ctrl->sensor_i2c_client, addr, &temp,
		MSM_CAMERA_I2C_BYTE_DATA);

	return temp;
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



	printk("==========OV13850 OTP INFO BEGIN==========\n");
	printk("%s-module_integrator_id	= 0x%x\n", __func__, otp_ptr->module_integrator_id);
	printk("%s-lens_id				= 0x%x\n", __func__, otp_ptr->lens_id);
	printk("%s-production_year		= 0x%x\n", __func__, otp_ptr->production_year);
	printk("%s-production_month		= 0x%x\n", __func__, otp_ptr->production_month);
	printk("%s-production_day		= 0x%x\n", __func__, otp_ptr->production_day);
	printk("%s-rg_ratio				= 0x%x\n", __func__, otp_ptr->rg_ratio);
	printk("%s-bg_ratio				= 0x%x\n", __func__, otp_ptr->bg_ratio);
	//printk("%s-light_rg				= 0x%x\n", __func__, otp_ptr->light_rg);
	//printk("%s-light_bg				= 0x%x\n", __func__, otp_ptr->light_bg);
	printk("%s-lenc[0]				= 0x%x\n", __func__, otp_ptr->lenc[0]);
	printk("%s-lenc[1]				= 0x%x\n", __func__, otp_ptr->lenc[1]);
	//printk("%s-VCM_start			= 0x%x\n", __func__, otp_ptr->VCM_start);
	//printk("%s-VCM_end			= 0x%x\n", __func__, otp_ptr->VCM_end);
	//printk("%s-VCM_dir			= 0x%x\n", __func__, otp_ptr->VCM_dir);
	printk("==========OV13850 OTP INFO END==========\n");



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
	//struct otp_struct_ov13850 current_otp;
	struct otp_struct_ov13850 *p_current_otp;

	int ret;

	p_current_otp = kmalloc(sizeof(struct otp_struct_ov13850), GFP_KERNEL);

	if(!p_current_otp) {
		printk("%s:%d kmalloc error! \n", __func__, __LINE__);
		return 0;	//may be changed by camera owner later
	}

	ret = ov13850_read_otp_info(s_ctrl, p_current_otp);
	if (ret == 1) {
		printk("%s:%d read otp error! \n", __func__, __LINE__);
		adjust_i2c_address(s_ctrl, OV13850_SENSOR_ADDR);
		return 0;
	}
	ov13850_apply_otp(s_ctrl, p_current_otp);

	kfree(p_current_otp);

	return 0;
}

/* ------------ ov13850_qtech update OTP entry  -------------*/
int ov13850_qtech_apply_otp(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct_ov13850 *otp_ptr) {
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
	nR_G_gain = (OV13850_QTECH_RG_RATIO_TYPICAL * 1000) / rg;
	nB_G_gain = (OV13850_QTECH_BG_RATIO_TYPICAL * 1000) / bg;
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

int ov13850_qtech_update_otp(struct msm_sensor_ctrl_t *s_ctrl) {
	//struct otp_struct_ov13850 current_otp;
	struct otp_struct_ov13850 *p_current_otp;
	int ret;

	p_current_otp = kmalloc(sizeof(struct otp_struct_ov13850), GFP_KERNEL);
	if(!p_current_otp) {
		printk("%s:%d kmalloc error! \n", __func__, __LINE__);
		return 0;	//may be changed by camera owner later
	}

	ret = ov13850_read_otp_info(s_ctrl, p_current_otp);
	if (ret == 1) {
		printk("%s:%d read otp error! \n", __func__, __LINE__);
		adjust_i2c_address(s_ctrl, OV13850_SENSOR_ADDR);
		return 0;
	}
	ov13850_qtech_apply_otp(s_ctrl, p_current_otp);

	kfree(p_current_otp);

	return 0;
}
/* ------------ ov13850_qtech update OTP end  -------------*/

int ov13850_match_module_id(struct msm_sensor_ctrl_t *s_ctrl){
	int mid=0;
	ov13850_otp_s_ctrl = s_ctrl;
	adjust_i2c_address(s_ctrl, OV13850_EEPROM_ADDR);
	usleep(5);
	mid = ov13850_otp_read_i2c(OTP_DRV_MID_ADDR);
	adjust_i2c_address(s_ctrl, OV13850_SENSOR_ADDR);
	return mid;
}

/* ------------ HI545 UPDATE OTP -------------*/
/*
 * How to read otp info data:
 * 1. put address want to read to 0x010a and 0100b.
 * 2. set 0x0102 to 0x00 [OTP mode].
 * 3. read data from 0108.
 * 4. set it to default after reading all.
 */

/* set to OTP read mode before read */
void hi545_otp_before_read(uint16_t addr) {
	//start address H
	hi545_otp_write_i2c(0x010a, ((addr >> 8) & 0xff));
	//start address L
	hi545_otp_write_i2c(0x010b, (addr & 0xff));
	// single write
	hi545_otp_write_i2c(0x0102, 0x00);
	usleep(5);
}

int hi545_otp_read(uint16_t addr) {
	hi545_otp_before_read(addr);

	return hi545_otp_read_i2c(0x0108);
}

/* set to default mode after reading */
void hi545_otp_after_read(void) {
	hi545_otp_write_i2c(0x0118, 0x00);
	usleep(5);
	hi545_otp_write_i2c(0x003e, 0x00);
	hi545_otp_write_i2c(0x0118, 0x01);
	usleep(5);
}

void hi545_initial_otp_setting(void) {
	//sleep on
	hi545_otp_write_i2c(0x0118, 0x00);
	usleep(5);

	//pll disable
	hi545_otp_write_i2c(0x0f02, 0x00);
	//CP TRI_H
	hi545_otp_write_i2c(0x011a, 0x01);

	//IPGM TRIM_H
	hi545_otp_write_i2c(0x011b, 0x09);
	//Fsync Output enable
	hi545_otp_write_i2c(0x0d04, 0x01);
	//Fsync Output Drivability
	hi545_otp_write_i2c(0x0d00, 0x07);

	//TG MCU enable
	hi545_otp_write_i2c(0x004c, 0x01);
	//OTP R/W
	hi545_otp_write_i2c(0x003e, 0x01);

	//sleep off
	hi545_otp_write_i2c(0x0118, 0x01);
	usleep(5);
}

int hi545_read_otp_info(void) {
	int i;
	int flag, group = -1;
	int MODULE_FLAG_ADDRESS = 0x1801, MODULE_INFO_START_ADDRESS = 0x1802;
	int AWB_FLAG_ADDRESS = 0x1832, AWB_INFO_START_ADDRESS = 0x1833;
	int sum = 0;

	hi545_initial_otp_setting();

	flag = hi545_otp_read(MODULE_FLAG_ADDRESS);

	// 1.1. check module flag
	if ((flag & 0xc0) == 0x40) { // group 1
		group = 1;
		MODULE_INFO_START_ADDRESS = 0x1802;
	} else if ((flag & 0x30) == 0x10) { // group 2
		group = 2;
		MODULE_INFO_START_ADDRESS = 0x1812;
	} else if ((flag & 0x0c) == 0x04) { // group 3
		group = 3;
		MODULE_INFO_START_ADDRESS = 0x1822;
	} else {
		printk("%s:%d: invalid module flag: 0x%04x \n", __func__, __LINE__, flag);
		return -1;
	}

	// 1.2. read module info
	for (i = 0; i <= 15; i++) {
		module_info[i] = hi545_otp_read(MODULE_INFO_START_ADDRESS + i);
		if (i != 15) {
			sum += module_info[i];
		}
	}

	// 1.3. check sum
	if (module_info[15] != (sum % 0xFF + 1)) {
		printk("%s:%d: module info sum is incorrect. \n", __func__, __LINE__);
		return -1;
	}

	// 2.1. check awb flag
	flag = hi545_otp_read(AWB_FLAG_ADDRESS);

	if ((flag & 0xc0) == 0x40) { // group 1
		group = 1;
		AWB_INFO_START_ADDRESS = 0x1833;
	} else if ((flag & 0x30) == 0x10) { // group 2
		group = 2;
		AWB_INFO_START_ADDRESS = 0x1850;
	} else if ((flag & 0x0c) == 0x04) { // group 3
		group = 3;
		AWB_INFO_START_ADDRESS = 0x186d;
	} else {
		printk("%s:%d: invalid awb flag: 0x%04x \n", __func__, __LINE__, flag);
		return -1;
	}

	// 2.2. read awb info
	sum = 0;
	for (i = 0; i <= 28; i++) {
		awb_info[i] = hi545_otp_read(AWB_INFO_START_ADDRESS + i);
		if (i != 28) {
			sum += awb_info[i];
		}
	}

	// 2.3. check sum
	if (awb_info[28] != (sum % 0xFF + 1)) {
		printk("%s:%d: awb info sum is incorrect. \n", __func__, __LINE__);
		return -1;
	}

	hi545_otp_after_read();
	return 0;
}

void hi545_apply_otp(void) {
	int rg_ratio = 0, bg_ratio = 0;
	int R_gain = 0, B_gain = 0;

	rg_ratio = (awb_info[0] << 2) + ((awb_info[1] >> 6) & 0x03);
	bg_ratio = (awb_info[2] << 2) + ((awb_info[3] >> 6) & 0x03);

	R_gain = 0x0100 * HI545_RG_RATIO_TYPICAL / rg_ratio;
	B_gain = 0x0100 * HI545_BG_RATIO_TYPICAL / bg_ratio;

	hi545_otp_write_i2c(0x050c, (R_gain >> 8));
	hi545_otp_write_i2c(0x050d, (R_gain & 0xff));

	hi545_otp_write_i2c(0x050e, (B_gain >> 8));
	hi545_otp_write_i2c(0x050f, (B_gain & 0xff));
}

int hi545_update_otp(struct msm_sensor_ctrl_t *s_ctrl) {
	int ret;

	hi545_otp_s_ctrl = s_ctrl;

	ret = hi545_read_otp_info();
	if (ret == -1) {
		printk("%s:%d read otp error! \n", __func__, __LINE__);
		hi545_otp_after_read();
		return 0;
	}

	hi545_apply_otp();

	return 0;
}