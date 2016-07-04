//EEPROM MAP & CheckSum Code Refinement, Camera-Driver@lge.com, 2015-06-11
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

int32_t msm_eeprom_checksum_imtech(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		return rc;
	}

	if(!strncmp("hi841", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_imtech_hi841(e_ctrl);
	}
	else if(!strncmp("t4kb3", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_imtech_t4kb3(e_ctrl);
	}
	else if(!strncmp("ov8858", e_ctrl->eboard_info->eeprom_name, 6)) {
		rc = msm_eeprom_checksum_imtech_ov8858(e_ctrl);
	}
	else {
		pr_err("%s vendor = imtech, name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}

	return rc;
}

int32_t msm_eeprom_checksum_lgit(struct msm_eeprom_ctrl_t *e_ctrl) {
	int32_t rc = -EFAULT;
	uint8_t eeprom_ver = 0xff;

	if(e_ctrl->cal_data.num_data < 0x770) {
		pr_err("%s num_data = %d\n", __func__, e_ctrl->cal_data.num_data);
		//return rc;
	}else{
		eeprom_ver = e_ctrl->cal_data.mapdata[0x770];
		pr_err("%s eeprom_ver = 0x%02X\n", __func__, eeprom_ver);
	}

	switch(eeprom_ver) {
		case 0x0d:
			if(!strncmp("t4ka3", e_ctrl->eboard_info->eeprom_name, 5)) {
				rc = msm_eeprom_checksum_lgit_v0d_t4ka3(e_ctrl);
			}
			else {
				rc = msm_eeprom_checksum_lgit_v0d(e_ctrl);
			}
			break;
		default:
			pr_info("eeprom ver = 0x%x\n", eeprom_ver);
			if(!strncmp("hi553", e_ctrl->eboard_info->eeprom_name, 5))
				rc = msm_eeprom_checksum_lgit_hi553(e_ctrl);
			break;
	}
	return rc;
}

int32_t msm_eeprom_checksum_cowell(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = -EFAULT;

	//Cowell Module have not set EEPROM VERSION [0x770] yet (2015-06-11)
	if(!strncmp("hi841", e_ctrl->eboard_info->eeprom_name, 5)) {
		rc = msm_eeprom_checksum_cowell_hi841(e_ctrl);
	}
	else if(!strncmp("zc533", e_ctrl->eboard_info->eeprom_name, 5)) {
		if (e_ctrl->cal_data.mapdata[0x700] == 0x10) {
			rc = msm_eeprom_checksum_cowell_imx258(e_ctrl);
		}
	}
	else {
		pr_err("%s failed to identifying eeprom version\n", __func__);
		pr_err("%s vendor = cowell, name = %s\n", __func__, e_ctrl->eboard_info->eeprom_name);
	}

	return rc;
}

// Helper function for shifted add
uint32_t shiftedSum(struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr, Endian endian)
{
	int addr = 0;
	int sum = 0;

	//input validataion
	int diff = endAddr - startAddr;
	if (diff > 4 || diff < -4) {
		pr_err("%s faild: exceed 32bit numbers\n", __func__);
		return 0;
	}

	//cumulative addition with shifting
	if (endian == BigEndian) {
		for (addr = startAddr; addr <= endAddr; addr++) {
			sum = (sum << 8) + (e_ctrl->cal_data.mapdata[addr]);
			//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
			//			addr, e_ctrl->cal_data.mapdata[addr]);
		}
	}
	else { //LittleEndian
		for (addr = endAddr; addr >= startAddr; addr--) {
			sum = (sum << 8) + (e_ctrl->cal_data.mapdata[addr]);
			//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
			//			addr, e_ctrl->cal_data.mapdata[addr]);
		}
	}

	return sum;
}

uint32_t accumulation(struct msm_eeprom_ctrl_t *e_ctrl, uint32_t startAddr, uint32_t endAddr)
{
	int addr = 0;
	int sum = 0;

	//input validataion
	int diff = endAddr - startAddr;
	if (diff < 0) {
		pr_err("%s failed: you must set (endAddr >= startAddr)\n", __func__);
		return 0;
	}

	//cumulative addition with shifting
	for (addr = startAddr; addr <= endAddr; addr++) {
		sum += (e_ctrl->cal_data.mapdata[addr]);
		//pr_err("[CHECK] e_ctrl->cal_data.mapdata[0x%04X]: 0x%04X\n",
		//			addr, e_ctrl->cal_data.mapdata[addr]);
	}

	return sum;
}