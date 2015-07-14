/* lge_ts_mms100s.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: WX-BSP-TS@lge.com
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
 #define LGTP_MODULE "[MIT_ISC]"
 
#include <linux/input/unified_driver_2/lgtp_common.h>	 
#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_mit200.h>

#define ISC_MASS_ERASE			{0xFB, 0x4A, 0x00, 0x15, 0x00, 0x00}
#define ISC_PAGE_WRITE			{0xFB, 0x4A, 0x00, 0x5F, 0x00, 0x00}
#define ISC_FLASH_READ			{0xFB, 0x4A, 0x00, 0xC2, 0x00, 0x00}
#define ISC_STATUS_READ			{0xFB, 0x4A, 0x00, 0xC8, 0x00, 0x00}
#define ISC_EXIT				{0xFB, 0x4A, 0x00, 0x66, 0x00, 0x00}

#define FW_BLOCK_SIZE			128

#define MAX_ITERATOR			30000
#define RETRY_COUNT				3
#define WAIT_TIME				50

struct isc_packet {
	u8 	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

static int mit_isc_check_status(struct i2c_client *client)
{
	u8 cmd[6] = ISC_STATUS_READ;
	u8 buf = 0;
	int ret = 0;

	TOUCH_FUNC();

	ret = MIT200_I2C_Read(client, cmd, 6, &buf, 1);
	if (ret) {
		return TOUCH_FAIL;
	}

	if (buf != 0xAD) {
		TOUCH_ERR("failed to read isc status \n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mit_isc_exit(struct i2c_client *client)
{
	u8 cmd[6] = ISC_EXIT;
	u8 buf = 0;
	int ret = 0;

	TOUCH_FUNC();

	ret = MIT200_I2C_Write(client, cmd, 6, &buf, 1);
	if (ret) {
		TOUCH_ERR("failed to isc exit\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mit_isc_verify_erased(struct i2c_client *client)
{
	u8 cmd[6] = ISC_FLASH_READ;
	u8 buf[4] = {0, };
	int ret = 0;

	TOUCH_FUNC();

	ret = MIT200_I2C_Read(client, cmd, 6, buf, 4);
	if (ret) {
		return TOUCH_FAIL;
	}

	if (buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != 0xFF || buf[3] != 0xFF) {
		TOUCH_ERR("failed to erase IC \n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mit_isc_mass_erase(struct i2c_client *client)
{
	u8 cmd[6] = ISC_MASS_ERASE;
	u8 buf = 0;
	int ret = 0;

	TOUCH_FUNC();

	ret = MIT200_I2C_Write(client, cmd, 6, &buf, 1);
	if (ret) {
		TOUCH_ERR("failed to send message for erase\n");
		return TOUCH_FAIL;
	}

	msleep(WAIT_TIME);

	ret = mit_isc_check_status(client);	
	if (ret) {
		TOUCH_ERR("failed to read isc status\n");
		return TOUCH_FAIL;
	}

	msleep(WAIT_TIME);

	ret = mit_isc_verify_erased(client);	
	if (ret) {
		TOUCH_ERR("failed to erase verify\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mit_isc_flash(struct i2c_client *client, const struct firmware *fw)
{
	u8 cmd[6] = ISC_PAGE_WRITE;
	int wrttensize = 0;
	int addr = 0;
	int ret = 0;

	TOUCH_FUNC();
	TOUCH_LOG("F/W Writing... \n");

	for (addr = ((int)fw->size) - FW_BLOCK_SIZE; addr >= 0; addr -= FW_BLOCK_SIZE ) {
		cmd[4] = (addr & 0xFF00) >> 8;
		cmd[5] = (addr & 0x00FF) >> 0;

		ret = MIT200_I2C_Write(client, cmd, 6, (u8 *)&fw->data[addr], FW_BLOCK_SIZE);
		if (ret) {
			TOUCH_ERR("failed to write fw data\n");
			return TOUCH_FAIL;
		}

		wrttensize += FW_BLOCK_SIZE;
		if (wrttensize % (FW_BLOCK_SIZE * 50) == 0) {
			TOUCH_LOG("\t Updated %5d / %5d bytes\n", wrttensize, (int)fw->size);
		}
	}

	TOUCH_LOG("\t Updated %5d / %5d bytes\n", wrttensize, (int)fw->size);

	ret = mit_isc_check_status(client); 
	if (ret) {
		TOUCH_ERR("failed to read isc status\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mit_doFirmwareUpgrade(struct i2c_client *client, const struct firmware *fw)
{
	int ret = 0;
	int nCnt = 0;

	TOUCH_FUNC();

	do {
		ret = mit_isc_mass_erase(client);

		ret = mit_isc_flash(client, fw);

		ret = mit_isc_exit(client);

		nCnt++;
	} while(ret == TOUCH_FAIL && nCnt < RETRY_COUNT);

	if (nCnt == RETRY_COUNT) {
		TOUCH_ERR("failed to upgrade FW\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

int MIT_FirmwareUpgrade(struct melfas_ts_data* ts, const char* fw_path)
{
	struct i2c_client *client = ts->client;
	const struct firmware *fw = NULL;
	int ret = 0;

	TOUCH_FUNC();

	if (fw_path == NULL) {
		TOUCH_ERR("fw path is NULL!!\n");
		return TOUCH_FAIL;
	}

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, fw_path, &client->dev);
	if(ret) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	if (fw->size != 64*1024) {
		TOUCH_ERR("F/W file is not for MIT-200\n");
		return TOUCH_FAIL;
	}

	if (memcmp("T2H0", &fw->data[0xFFF0], 4)) {
		TOUCH_ERR("F/W file is not for MIT-200\n");
		return TOUCH_FAIL;
	}

	ret = mit_doFirmwareUpgrade(client, fw);

	return TOUCH_SUCCESS;
}

