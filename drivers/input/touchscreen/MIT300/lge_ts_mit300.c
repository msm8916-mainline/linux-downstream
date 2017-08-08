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

/* History :
 *
 */

#include "lge_ts_melfas.h"

//ISC Info
#define ISC_PAGE_SIZE				128

//ISC Command
#define ISC_CMD_ENTER				{0xFB,0x4A,0x00,0x65,0x00,0x00,0x00,0x00}
#define ISC_CMD_READ_STATUS		{0xFB,0x4A,0x36,0xC2,0x00,0x00,0x00,0x00}
#define ISC_CMD_ERASE_PAGE		{0xFB,0x4A,0x00,0x8F,0x00,0x00,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE		{0xFB,0x4A,0x00,0x54,0x00,0x00,0x00,0x00}
#define ISC_CMD_READ_PAGE			{0xFB,0x4A,0x00,0xC2,0x00,0x00,0x00,0x00}
#define ISC_CMD_EXIT				{0xFB,0x4A,0x00,0x66,0x00,0x00,0x00,0x00}

//ISC Status
#define ISC_STATUS_BUSY			0x96
#define ISC_STATUS_DONE			0xAD

#define MAX_ITERATOR	30000

extern int is_probed;

struct isc_packet {
	u8 	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

bool test_busy = false;


extern int mip_i2c_write(struct i2c_client *client, char *write_buf, unsigned int write_len);

/**
* Read ISC status
*/
static int mip_isc_read_status(struct mit_data *info)
{
	struct i2c_client *client = info->client;
	u8 cmd[8] =  ISC_CMD_READ_STATUS;
	u8 result = 0;
	int cnt = 100;
	int ret = 0;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 8,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &result,
			.len = 1,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	do {
		ret = i2c_transfer(client->adapter, &msg[0], 1);
		ret += i2c_transfer(client->adapter, &msg[1], 1);

		if(ret!=ARRAY_SIZE(msg)){
			dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
			return -1;
		}

		if(result == ISC_STATUS_DONE){
			ret = 0;
			break;
		}
		else if(result == ISC_STATUS_BUSY){
			ret = -1;
			//msleep(1);
		}
		else{
			dev_err(&info->client->dev, "%s [ERROR] wrong value [0x%02X]\n", __func__, result);
			ret = -1;
			msleep(1);
		}
	} while (--cnt);

	if (!cnt) {
		dev_err(&info->client->dev, "%s [ERROR] count overflow - cnt [%d] status [0x%02X]\n", __func__, cnt, result);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return ret;

ERROR:
	return ret;
}

/**
* Command : Erase Page
*/
static int mip_isc_erase_page(struct mit_data *info, int offset)
{
	u8 write_buf[8] =ISC_CMD_ERASE_PAGE;

	struct i2c_msg msg[1] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 8,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);
	if(i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)){
		dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
		goto ERROR;
	}

	if(mip_isc_read_status(info) != 0){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Read Page
*/
int mip_isc_read_page(struct mit_data *info, int offset, u8 *data)
{
	u8 write_buf[8] =ISC_CMD_READ_PAGE;
	int ret = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = 8,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = data,
			.len = ISC_PAGE_SIZE,
		},
	};

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);

	ret = i2c_transfer(info->client->adapter, &msg[0], 1);
	ret += i2c_transfer(info->client->adapter, &msg[1], 1);

	if(ret!=ARRAY_SIZE(msg)){
		dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Program Page
*/
static int mip_isc_program_page(struct mit_data *info, int offset, const u8 *data, int length)
{
	u8 write_buf[8 + ISC_PAGE_SIZE] = ISC_CMD_PROGRAM_PAGE;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if(length > ISC_PAGE_SIZE){
		dev_err(&info->client->dev, "%s [ERROR] page length overflow\n", __func__);
		goto ERROR;
	}

	write_buf[4] = (u8)((offset >> 24) & 0xFF);
	write_buf[5] = (u8)((offset >> 16) & 0xFF);
	write_buf[6] = (u8)((offset >> 8) & 0xFF);
	write_buf[7] = (u8)(offset & 0xFF);

	memcpy(&write_buf[8], data, length);

	if(i2c_master_send(info->client, write_buf, (length + 8)) != (length + 8)){
		dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
		goto ERROR;
	}

	if(mip_isc_read_status(info) != 0){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE] - Offset[0x%04X] Length[%d]\n", __func__, offset, length);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Enter ISC
*/
static int mip_isc_enter(struct mit_data *info)
{
	u8 write_buf[8] = ISC_CMD_ENTER;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if(i2c_master_send(info->client, write_buf, 8) != 8){
		dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
		goto ERROR;
	}

	if(mip_isc_read_status(info) != 0){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;

ERROR:
	return -1;
}

/**
* Command : Exit ISC
*/
int mip_isc_exit(struct mit_data *info)
{
	u8 write_buf[8] = ISC_CMD_EXIT;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	if(i2c_master_send(info->client, write_buf, 8) != 8){
		dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

	return 0;

ERROR:
	return -1;
}

/**
* Flash chip firmware (main function)
*/
int mip_flash_fw(struct mit_data *info, const u8 *fw_data, size_t fw_size, bool force, bool section)
{
	struct i2c_client *client = info->client;
	struct mip_bin_tail *bin_info;
	int ret = 0;
	int retry = 3;
	int offset = 0;
	int offset_start = 0;
	int bin_size = 0;
	u8 *bin_data = NULL;
	u16 tail_size = 0;
	u8 tail_mark[4] = MIP_BIN_TAIL_MARK;
	u16 ver_chip;

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	//Check tail size
	tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];
	if(tail_size != MIP_BIN_TAIL_SIZE){
		dev_err(&client->dev, "%s [ERROR] wrong tail size [%d]\n", __func__, tail_size);
		ret = fw_err_file_type;
		goto ERROR;
	}

	//Check bin format
	if(memcmp(&fw_data[fw_size - tail_size], tail_mark, 4)){
		dev_err(&client->dev, "%s [ERROR] wrong tail mark\n", __func__);
		ret = fw_err_file_type;
		goto ERROR;
	}

	//Read bin info
	bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];

	dev_dbg(&client->dev, "%s - bin_info : bin_len[%d] hw_cat[0x%2X] date[%4X] time[%4X] tail_size[%d]\n", __func__, bin_info->bin_length, bin_info->hw_category, bin_info->build_date, bin_info->build_time, bin_info->tail_size);

#if MIP_FW_UPDATE_DEBUG
	print_hex_dump(KERN_ERR, MIP_DEVICE_NAME " Bin Info : ", DUMP_PREFIX_OFFSET, 16, 1, bin_info, tail_size, false);
#endif

	//Check chip code
	if(memcmp(bin_info->chip_name, CHIP_FW_CODE, 4)){
		dev_err(&client->dev, "%s [ERROR] F/W file is not for %s\n", __func__, CHIP_NAME);
		ret = fw_err_file_type;
		goto ERROR;
	}

	//Check F/W version
	dev_info(&client->dev, "%s - F/W file version [0x%04X]\n", __func__, bin_info->ver_app);

	dev_info(&client->dev, "%s - is_probed [%d]\n", __func__, is_probed);
	if(!is_probed){
		info->module.bin_version[0] = (bin_info->ver_app >> 8) & 0xFF;
		info->module.bin_version[1] =  bin_info->ver_app;
		strncpy(info->module.bin_chip_name, bin_info->chip_name, strlen(bin_info->chip_name));
	}
	if(force == true){
		//Force update
		dev_info(&client->dev, "%s - Skip chip firmware version check\n", __func__);
	}
	else{
		//Read firmware version from chip
		while(retry--){
			if(!mip_get_fw_version_u16(info->client, &ver_chip)){
				break;
			}
			else{
				mip_reboot(info->client);
			}
		}
		if(retry < 0){
			dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n", __func__);
		}
		else{
			dev_info(&client->dev, "%s - Chip firmware version [0x%04X]\n", __func__, ver_chip);

			//Compare version
			if((ver_chip == bin_info->ver_app)){
				dev_info(&client->dev, "%s - Chip firmware is already up-to-date\n", __func__);
				ret = fw_err_uptodate;
				goto EXIT;
			}
		}
	}

	//Read bin data
	bin_size = bin_info->bin_length;
	bin_data = kzalloc(sizeof(u8) * (bin_size), GFP_KERNEL);
	memcpy(bin_data, fw_data, bin_size);

	//Enter ISC mode
	dev_dbg(&client->dev,"%s - Enter ISC mode\n", __func__);
	ret = mip_isc_enter(info);
	if(ret != 0){
		dev_err(&client->dev,"%s [ERROR] mip_isc_erase_page\n", __func__);
		ret = fw_err_download;
		goto ERROR;
	}

	//Erase first page
	offset = 0;
	dev_dbg(&client->dev, "%s - Erase first page : Offset[0x%04X]\n", __func__, offset);
	ret = mip_isc_erase_page(info, offset);
	if(ret != 0){
		dev_err(&client->dev,"%s [ERROR] mip_isc_erase_page\n", __func__);
		ret = fw_err_download;
		goto ERROR;
	}

	//Program & Verify
	dev_dbg(&client->dev, "%s - Program & Verify\n", __func__);

	offset_start = 0;
	offset = bin_size - ISC_PAGE_SIZE;
	while(offset >= offset_start){
		//Program
		if(mip_isc_program_page(info, offset, &bin_data[offset], ISC_PAGE_SIZE)){
			dev_err(&client->dev, "%s [ERROR] mip_isc_program_page : offset[0x%04X]\n", __func__, offset);
			ret = fw_err_download;
			goto ERROR;
		}
		dev_dbg(&client->dev, "%s - mip_isc_program_page : offset[0x%04X]\n", __func__, offset);

		offset -= ISC_PAGE_SIZE;
	}

	//Exit ISC mode
	dev_dbg(&client->dev, "%s - Exit\n", __func__);
	mip_isc_exit(info);

	//Reset chip
	mip_reboot(info->client);

        msleep(100);

	//Check chip firmware version
	if(mip_get_fw_version_u16(info->client, &ver_chip)){
		dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n", __func__);
		ret = fw_err_download;
		goto ERROR;
	}
	else{
		if((ver_chip == bin_info->ver_app)){
			dev_dbg(&client->dev, "%s - Version check OK\n", __func__);
		}
		else{
			dev_err(&client->dev, "%s [ERROR] Version mismatch after flash. Chip[0x%04X] File[0x%04X]\n", __func__, ver_chip, bin_info->ver_app);
			ret = fw_err_download;
			goto ERROR;
		}
	}

	goto EXIT;

ERROR:
	//Reset chip
	mip_reboot(info->client);

	dev_err(&client->dev, "%s [ERROR]\n", __func__);
EXIT:
	dev_dbg(&client->dev, "%s [DONE]\n", __func__);

	if(bin_data != NULL)
		kfree(bin_data);

	return ret;
}


int mit_isc_fwupdate(struct mit_data *info, struct touch_fw_info *fw_info)
{
	int retires = 3;
	int ret;

	do {
		ret = mip_flash_fw(info, fw_info->fw->data, fw_info->fw->size, false, true);
		if(ret >= fw_err_none){
			break;
		}
	} while (--retires);

	if (!retires) {
		dev_err(&info->client->dev, "%s [ERROR] mip_flash_fw failed\n", __func__);
		ret = -1;
	}


	if(ret < 0){
		goto ERROR;
	}

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
	return -1;

}


/**
* Process table data
*/
static int mip_proc_table_data(struct mit_data *ts, u8 size, u8 data_type_size, u8 data_type_sign, u8 buf_addr_h, u8 buf_addr_l, u8 row_num, u8 col_num, u8 buf_col_num, u8 rotate, u8 key_num)
{

	char data[10];
	int i_col, i_row;
	int i_x, i_y;
	int lim_x, lim_y;
	int lim_col, lim_row;
	int max_x = 0;
	int max_y = 0;
	bool flip_x = false;
	int sValue = 0;
	unsigned int uValue = 0;
	int value = 0;
	u8 wbuf[8];
	u8 rbuf[512];
	unsigned int buf_addr;
	int offset;
	int data_size = data_type_size;
	int data_sign = data_type_sign;
	int has_key = 0;
	//int size_screen = col_num * row_num;

//	u8 *print_buf = NULL;
//	int *image_buf = NULL;

	/*

	if(info->print_buf == NULL){
		info->print_buf = kzalloc(sizeof(u8) * 4096, GFP_KERNEL);
	}
	if(info->image_buf == NULL){
		info->image_buf = kzalloc(sizeof(int) * ((info->node_x * info->node_y) + info->node_key), GFP_KERNEL);
	}
		//Node info
	ts->dev.col_num;
	ts->dev.row_num;
	ts->dev.key_num;
	*/






	memset(data, 0, 10);

	printk(KERN_INFO "%s [START]\n", __func__);

	//set axis
	if(rotate == 0){
		max_x = col_num;
		max_y = row_num;
		if(key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = false;
	}
	else if(rotate == 1){
		max_x = row_num;
		max_y = col_num;
		if(key_num > 0){
			max_y += 1;
			has_key = 1;
		}
		flip_x = true;
	}
	else{
		printk(KERN_ERR "%s [ERROR] rotate [%d]\n", __func__, rotate);
		goto ERROR;
	}

	//get table data
	lim_row = row_num + has_key;
	for(i_row = 0; i_row < lim_row; i_row++){
		//get line data
		offset = buf_col_num * data_type_size;
		size = col_num * data_type_size;

		buf_addr = (buf_addr_h << 8) | buf_addr_l | (offset * i_row);
		wbuf[0] = (buf_addr >> 8) & 0xFF;
		wbuf[1] = buf_addr & 0xFF;
		if(mit_i2c_read(ts->client, wbuf,2, rbuf, size)){
			printk(KERN_ERR "%s [ERROR] Read data buffer\n", __func__);
			goto ERROR;
		}

		//save data
		if((key_num > 0) && (i_row == (lim_row - 1))){
			lim_col = key_num;
		}
		else{
			lim_col = col_num;
		}
		for(i_col = 0; i_col < lim_col; i_col++){
			if(data_sign == 0){
				//unsigned
				if(data_size == 1){
					uValue = (u8)rbuf[i_col];
				}
				else if(data_size == 2){
					uValue = (u16)(rbuf[data_size * i_col] | (rbuf[data_size * i_col + 1] << 8));
				}
				else if(data_size == 4){
					uValue = (u32)(rbuf[data_size * i_col] | (rbuf[data_size * i_col + 1] << 8) | (rbuf[data_size * i_col + 2] << 16) | (rbuf[data_size * i_col + 3] << 24));
				}
				else{
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_size);
					goto ERROR;
				}
				value = (int)uValue;
			}
			else{
				//signed
				if(data_size == 1){
					sValue = (s8)rbuf[i_col];
				}
				else if(data_size == 2){
					sValue = (s16)(rbuf[data_size * i_col] | (rbuf[data_size * i_col + 1] << 8));
				}
				else if(data_size == 4){
					sValue = (s32)(rbuf[data_size * i_col] | (rbuf[data_size * i_col + 1] << 8) | (rbuf[data_size * i_col + 2] << 16) | (rbuf[data_size * i_col + 3] << 24));
				}
				else{
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_size);
					goto ERROR;
				}
				value = (int)sValue;
			}

			switch(rotate){
				case 0:
					//image_buf[i_row * col_num + i_col] = value;
					ts->mit_data[i_row][i_col] = value;
					ts->intensity_data[i_row][i_col] = value;
					break;
				case 1:
					if((key_num > 0) && (i_row == (lim_row - 1))){
						//image_buf[size_screen + i_col] = value;
						ts->mit_data[i_row][i_col] = value;
						ts->intensity_data[i_row][i_col] = value;
					}
					else{
						//image_buf[i_col * row_num + (row_num - 1 - i_row)] = value;
						ts->mit_data[i_col][i_row] = value;
						ts->intensity_data[i_col][i_row] = value;
					}
					break;
				default:
					printk(KERN_ERR "%s [ERROR] rotate [%d]\n", __func__, rotate);
					goto ERROR;
					break;
			}
		}
	}

	//print table header
	sprintf(data, "    ");
	//strcat(print_buf, data);
	memset(data, 0, 10);

	switch(data_size){
		case 1:
			for(i_x = 0; i_x < max_x; i_x++){
				sprintf(data, "[%2d]", i_x);
				//strcat(print_buf, data);
				memset(data, 0, 10);
			}
			break;
		case 2:
			for(i_x = 0; i_x < max_x; i_x++){
				sprintf(data, "[%4d]", i_x);
				//strcat(print_buf, data);
				memset(data, 0, 10);
			}
			break;
		case 4:
			for(i_x = 0; i_x < max_x; i_x++){
				sprintf(data, "[%5d]", i_x);
				//strcat(print_buf, data);
				memset(data, 0, 10);
			}
			break;
		default:
			printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_size);
			goto ERROR;
			break;
	}

	printk("\n");
	sprintf(data, "\n");
	//strcat(print_buf, data);
	memset(data, 0, 10);

	//print table
	lim_y = max_y;
	for(i_y = 0; i_y < lim_y; i_y++){
		//print line header
		if((key_num > 0) && (i_y == (lim_y -1))){
			sprintf(data, "[TK]");
		}
		else{
			sprintf(data, "[%2d]", i_y);
		}
		//strcat(print_buf, data);
		memset(data, 0, 10);

		//print line
		if((key_num > 0) && (i_y == (lim_y - 1))){
			lim_x = key_num;
		}
		else{
			lim_x = max_x;
		}
		for(i_x = 0; i_x < lim_x; i_x++){
			switch(data_size){
				//ts->mit_data[row][col]
				case 1:
					//printk(" %3d", image_buf[i_y * max_x + i_x]);
					//sprintf(data, " %3d", image_buf[i_y * max_x + i_x]);
					break;
				case 2:
					//printk(" %5d", image_buf[i_y * max_x + i_x]);
					//sprintf(data, " %5d", image_buf[i_y * max_x + i_x]);
					break;
				case 4:
					//printk(" %6d", image_buf[i_y * max_x + i_x]);
					//sprintf(data, " %6u", image_buf[i_y * max_x + i_x]);

					break;
				default:
					printk(KERN_ERR "%s [ERROR] data_size [%d]\n", __func__, data_size);
					goto ERROR;
					break;
			}

			//strcat(print_buf, data);
			memset(data, 0, 10);
		}

		sprintf(data, "\n");
		//strcat(print_buf, data);
		memset(data, 0, 10);
	}

	sprintf(data, "\n");
	//strcat(print_buf, data);
	memset(data, 0, 10);



	printk(KERN_INFO "%s [DONE]\n", __func__);
	return 0;

ERROR:

	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return 1;
}

/**
* Get ready status
*/
int mip_get_ready_status(struct mit_data *ts)
{
	u8 wbuf[16];
	u8 rbuf[16] = {0};
	int ret = 0;

	//dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if(mit_i2c_read(ts->client, wbuf, 2, rbuf, 1)<0){
		printk(KERN_ERR "%s [ERROR] mip_i2c_read\n", __func__);
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)){
		//printk(KERN_INFO "%s - status [0x%02X]\n", __func__, ret);
	}
	else{
		printk(KERN_ERR "%s [ERROR] Unknown status [0x%02X]\n", __func__, ret);
		goto ERROR;
	}

	if(ret == MIP_CTRL_STATUS_LOG){
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if(mip_i2c_write(ts->client, wbuf, 3)){
			printk(KERN_ERR "%s [ERROR] mip_i2c_write\n", __func__);
		}
	}

	//printk(KERN_INFO "%s [DONE]\n", __func__);
	return ret;

ERROR:
	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return -1;
}


int mip_get_image(struct mit_data *ts, u8 image_type){

  //struct lge_touch_data *tsdata;
        int busy_cnt = 100;
	int wait_cnt = 100;
	u8 wbuf[8];
	u8 rbuf[512] = {0};
	u8 size = 0;
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
	u8 buf_addr_h;
	u8 buf_addr_l;




	printk(KERN_INFO "%s [START]\n", __func__);
	printk(KERN_INFO "%s - image_type[%d]\n", __func__, image_type);

  //tsdata = i2c_get_clientdata(ts->client);
	while(busy_cnt--){
		if(test_busy == false){
			break;
		}
		printk(KERN_INFO "%s - busy_cnt[%d]\n", __func__, busy_cnt);
		msleep(5);
	}
	//mutex_lock(&tsdata->thread_lock);
	test_busy = true;
	//mutex_unlock(&tsdata->thread_lock);

	//memset(info->print_buf, 0, PAGE_SIZE);

	//check image type
	switch(image_type){
		case MIP_IMG_TYPE_INTENSITY:
			printk(KERN_INFO "=== Intensity Image ===\n");
			//sprintf(info->print_buf, "\n=== Intensity Image ===\n\n");
			break;
		case MIP_IMG_TYPE_RAWDATA:
			printk(KERN_INFO "=== Rawdata Image ===\n");
			//sprintf(info->print_buf, "\n=== Rawdata Image ===\n\n");
			break;
		default:
			printk(KERN_ERR "%s [ERROR] Unknown image type\n", __func__);
			//sprintf(info->print_buf, "\nERROR : Unknown image type\n\n");
			goto ERROR;
			break;
	}
	//set interrupt mode none
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_INTERRUPT;
	wbuf[2] = 0;
	if(mip_i2c_write(ts->client, wbuf, 3)<0){
		printk(KERN_ERR "%s [ERROR] Write interrupt none\n", __func__);
		goto ERROR;
	}

	//set image type
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = image_type;
	if (mip_i2c_write(ts->client, wbuf, 3) < 0) {
		printk(KERN_ERR "%s [ERROR] Write image type\n", __func__);
		goto ERROR;
	}

	printk(KERN_INFO "%s - set image type\n", __func__);

	//wait ready status
	wait_cnt = 100;
	while(wait_cnt--){
		if(mip_get_ready_status(ts) == MIP_CTRL_STATUS_READY){
			break;
		}
		msleep(10);

		printk(KERN_INFO "%s - wait [%d]\n", __func__, wait_cnt);
	}

	if(wait_cnt <= 0){
		printk(KERN_ERR "%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}

	printk(KERN_INFO "%s - ready\n", __func__);

	//data format
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_DATA_FORMAT;
  if(mit_i2c_read(ts->client, wbuf,2, rbuf, 6)<0){
		printk(KERN_ERR "%s [ERROR] Read data format\n", __func__);
		goto ERROR;
	}
	row_num = rbuf[0];
	col_num = rbuf[1];
	buffer_col_num = rbuf[2];
	rotate = rbuf[3];
	key_num = rbuf[4];
	data_type = rbuf[5];

	data_type_sign = (data_type & 0x80) >> 7;
	data_type_size = data_type & 0x7F;

	printk(KERN_INFO "%s - row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", __func__, row_num, col_num, buffer_col_num, rotate, key_num);
	printk(KERN_INFO "%s - data_type[0x%02X] data_sign[%d] data_size[%d]\n", __func__, data_type, data_type_sign, data_type_size);

	//get buf addr
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_BUF_ADDR;
	//if(mip_i2c_read(info, wbuf, 2, rbuf, 2)){
	if(mit_i2c_read(ts->client, wbuf, 2,rbuf, 2)<0){
		printk(KERN_ERR "%s [ERROR] Read buf addr\n", __func__);
		goto ERROR;
	}

	buf_addr_l = rbuf[0];
	buf_addr_h = rbuf[1];
	printk(KERN_INFO "%s - buf_addr[0x%02X 0x%02X]\n", __func__, buf_addr_h, buf_addr_l);

	//print data
	if(mip_proc_table_data(ts, size, data_type_size, data_type_sign, buf_addr_h, buf_addr_l, row_num, col_num, buffer_col_num, rotate, key_num )){
		printk(KERN_ERR "%s [ERROR] mip_proc_table_data\n", __func__);
		goto ERROR;
	}

	//clear image type
	wbuf[0] = MIP_R0_IMAGE;
	wbuf[1] = MIP_R1_IMAGE_TYPE;
	wbuf[2] = MIP_IMG_TYPE_NONE;
	//if(mip_i2c_write(info, wbuf, 3)){
	if (mip_i2c_write(ts->client, wbuf, 3) < 0) {
		printk(KERN_ERR "%s [ERROR] Clear image type\n", __func__);
		goto ERROR;
	}

	//set interrupt mode
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_INTERRUPT;
	wbuf[2] = 1;
	if(mip_i2c_write(ts->client, wbuf, 3)<0){
		printk(KERN_ERR "%s [ERROR] Write interrupt mode\n", __func__);
		goto ERROR;
	}

	//exit
	//mutex_lock(&tsdata->thread_lock);
	test_busy = false;
	//mutex_unlock(&tsdata->thread_lock);

	printk(KERN_INFO "%s [DONE]\n", __func__);
	return 0;

ERROR:
	//mutex_lock(&tsdata->thread_lock);
	test_busy = false;
	//mutex_unlock(&tsdata->thread_lock);

	printk(KERN_ERR "%s [ERROR]\n", __func__);
	return 1;


}

static int get_intensity(struct mit_data *ts)
{
  //struct mit_data *ts, char *buf, u8 image_type
  if(mip_get_image(ts, MIP_IMG_TYPE_INTENSITY)){
  TOUCH_INFO_MSG("Err : %s\n", __func__);
  return -1;
  }
  return 0;

#if 0
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u8 write_buf[8] = {0};
	u8 read_buf[60] = {0};
	u8 nLength = 0;
	s16 temp_data[MAX_COL][MAX_ROW]={{0}};

	TOUCH_TRACE_FUNC();

	if (ts->dev.col_num > MAX_COL) {
		TOUCH_INFO_MSG("Err : ts->dev.col_num > MAX_COL, EXIT\n");
		return -1;
	}

	for (col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = 0x70;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf, 5) != 5) {
			TOUCH_INFO_MSG("intensity i2c send failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf, 2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		if (i2c_master_recv(client,read_buf, 1) != 1) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		nLength = read_buf[0];
		if ( nLength > sizeof(read_buf)) {
			TOUCH_ERR_MSG("stack overflow - nLength: %d > read_buf size:%d \n", nLength, sizeof(read_buf));
			return -1;
		}
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client, write_buf,2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		if (i2c_master_recv(client,read_buf, nLength) != nLength) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		nLength >>= 1;
		if (nLength > MAX_ROW) {
			TOUCH_INFO_MSG("Err : nLeangth > MAX_ROW, EXIT\n");
			return -1;
		}

		for (row = 0 ; row <nLength ; row++) {
			temp_data[col][row] = (s16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
		}
	}

	for (row = 0; row < MAX_ROW; row++) {
		for (col = 0; col < MAX_COL; col++) {
			ts->intensity_data[row][col] = temp_data[col][row];
		}
	}

	return 0;
#endif

}

static int  print_intensity(struct mit_data *ts, char *buf) {
	int col = 0;
	int row = 0;
	int ret = 0;

	ret += sprintf(buf + ret, "Start-Intensity\n\n");

	for (row = 0 ; row < ts->dev.row_num ; row++) {
		printk("[Touch] [%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		for (col = 0 ; col < ts->dev.col_num ; col++) {
			ret += sprintf(buf + ret,"%4d ", ts->intensity_data[row][col]);
			printk("%4d ", ts->intensity_data[row][col]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}

	return ret;
}

ssize_t mit_delta_show(struct i2c_client *client, char *buf)
{
	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	int i = 0;
	int ret = 0;

	TOUCH_TRACE_FUNC();

	if (ts->pdata->curr_pwr_state == POWER_OFF) {
		TOUCH_INFO_MSG("intensity printf failed - Touch POWER OFF\n");
		return 0;
	}

	for ( i = 0 ; i < ts->dev.row_num ; i++) {
		memset(ts->intensity_data[i], 0, sizeof(uint16_t) * ts->dev.col_num);
	}

	touch_disable(ts->client->irq);
	if (get_intensity(ts) == -1) {
		TOUCH_INFO_MSG("intensity printf failed\n");
		goto error;
	}
	ret = print_intensity(ts, buf);
	if ( ret < 0) {
		TOUCH_ERR_MSG("fail to print intensity data\n");
		goto error;
	}

	touch_enable(ts->client->irq);

	return ret;

error :
	touch_enable(ts->client->irq);
	return -1;
}





static int get_rawdata(struct mit_data *ts)
{
#if 1
  //struct mit_data *ts, char *buf, u8 image_type
  if(mip_get_image(ts, MIP_IMG_TYPE_RAWDATA)){
  TOUCH_INFO_MSG("Err : %s\n", __func__);
  return -1;
  }
  return 0;

#else
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u32 limit_upper = 0;
	u32 limit_lower = 0;
	u8 write_buf[8] = {0,};
	u8 read_buf[25 * 8] = {0,};
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t temp_data[MAX_COL][MAX_ROW]={{0}};
	TOUCH_TRACE_FUNC();

	if (ts->dev.col_num > MAX_COL) {
		TOUCH_INFO_MSG("Err : ts->dev.col_num > MAX_COL, EXIT\n");
		return -1;
	}

	for (col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_RAWDATA;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf,5) != 5) {
			dev_err(&client->dev, "rawdata i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf,2) != 2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,1) != 1) {
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		if ( nLength > sizeof(read_buf)) {
			TOUCH_ERR_MSG("stack overflow - nLength: %d > read_buf size:%d \n", nLength, sizeof(read_buf));
			return -1;
		}
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client,write_buf,2) != 2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,nLength) != nLength) {
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		if (nLength > MAX_ROW) {
			TOUCH_INFO_MSG("Err : nLeangth > MAX_ROW, EXIT\n");
			return -1;
		}

		for (row = 0 ; row <nLength ; row++) {
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			temp_data[col][row] = nReference;
		}
	}

	if (ts->module.otp == OTP_APPLIED) {
		limit_upper = ts->pdata->limit->raw_data_otp_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_otp_min - ts->pdata->limit->raw_data_margin;
	} else {
		limit_upper = ts->pdata->limit->raw_data_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_min - ts->pdata->limit->raw_data_margin;
	}

	for (row = 0; row < MAX_ROW; row++) {
		for (col = 0; col < MAX_COL; col++) {
			ts->mit_data[row][col]	= temp_data[col][row];
			if (ts->mit_data[row][col] < limit_lower || ts->mit_data[row][col] > limit_upper)
				ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0;
		}
	}
	return 0;
#endif

}


int mip_run_test(struct mit_data *ts, u8 test_type)
{
	int busy_cnt = 50;
	int wait_cnt = 50;
	u8 wbuf[8];
	u8 rbuf[512] = {0};
	u8 size = 0;
	u8 row_num;
	u8 col_num;
	u8 buffer_col_num;
	u8 rotate;
	u8 key_num;
	u8 data_type;
	u8 data_type_size;
	u8 data_type_sign;
	u8 buf_addr_h;
	u8 buf_addr_l;

	dev_dbg(&ts->client->dev, "%s [START]\n", __func__);
	dev_dbg(&ts->client->dev, "%s - test_type[%d]\n", __func__, test_type);
	while(busy_cnt--){
		if(test_busy == false){
			break;
		}
		msleep(10);
	}
//	mutex_lock(&info->lock);
	test_busy = true;
//	mutex_unlock(&info->lock);
	//memset(info->print_buf, 0, PAGE_SIZE);
	//check test type

	//if (ts->test_mode == RAW_DATA_SHOW || ts->test_mode == RAW_DATA_STORE || ts->test_mode == SLOPE) {

	switch(test_type){

		case MIP_TEST_TYPE_CM_DELTA:
			printk("=== Cm Delta Test ===\n");
			//sprintf(info->print_buf, "\n=== Cm Delta Test ===\n\n");
			break;
		case MIP_TEST_TYPE_CM_JITTER:
			printk("=== Cm Jitter Test ===\n");
			//sprintf(info->print_buf, "\n=== Cm Jitter Test ===\n\n");
			break;
		case MIP_TEST_TYPE_SHORT:
			printk("=== Short Test ===\n");
			//sprintf(info->print_buf, "\n=== Short Test ===\n\n");
			break;
		default:
			dev_err(&ts->client->dev, "%s [ERROR] Unknown test type\n", __func__);
			//sprintf(info->print_buf, "\nERROR : Unknown test type\n\n");
			goto ERROR;
			break;
	}
	//set interrupt mode none
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_INTERRUPT;
	wbuf[2] = 0;
	if(mip_i2c_write(ts->client, wbuf, 3)<0){
		printk(KERN_ERR "%s [ERROR] Write interrupt none\n", __func__);
		goto ERROR;
	}

	//set test mode
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_MODE;
	wbuf[2] = MIP_CTRL_MODE_TEST_CM;
	if(mip_i2c_write(ts->client, wbuf, 3)<0){
		dev_err(&ts->client->dev, "%s [ERROR] Write test mode\n", __func__);
		goto ERROR;
	}
	//wait ready status
	wait_cnt = 100;
	while(wait_cnt--){
		if(mip_get_ready_status(ts) == MIP_CTRL_STATUS_READY){
			break;
		}
		msleep(50);

		dev_dbg(&ts->client->dev, "%s - wait [%d]\n", __func__, wait_cnt);
	}

	if(wait_cnt <= 0){
		dev_err(&ts->client->dev, "%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}
	dev_dbg(&ts->client->dev, "%s - set control mode\n", __func__);

	//set test type
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_TYPE;
	wbuf[2] = test_type;
	if(mip_i2c_write(ts->client, wbuf, 3)){
		dev_err(&ts->client->dev, "%s [ERROR] Write test type\n", __func__);
		goto ERROR;
	}
	dev_dbg(&ts->client->dev, "%s - set test type\n", __func__);
	//wait ready status
	wait_cnt = 100;
	while(wait_cnt--){
		if(mip_get_ready_status(ts) == MIP_CTRL_STATUS_READY){
			break;
		}
		msleep(10);

		dev_dbg(&ts->client->dev, "%s - wait [%d]\n", __func__, wait_cnt);
	}

	if(wait_cnt <= 0){
		dev_err(&ts->client->dev, "%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}
	dev_dbg(&ts->client->dev, "%s - ready\n", __func__);

	//data format
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_DATA_FORMAT;
	if(mit_i2c_read(ts->client, wbuf, 2, rbuf, 6)<0){
		dev_err(&ts->client->dev, "%s [ERROR] Read data format\n", __func__);
		goto ERROR;
	}
	row_num = rbuf[0];
	col_num = rbuf[1];
	buffer_col_num = rbuf[2];
	rotate = rbuf[3];
	key_num = rbuf[4];
	data_type = rbuf[5];

	data_type_sign = (data_type & 0x80) >> 7;
	data_type_size = data_type & 0x7F;

	dev_dbg(&ts->client->dev, "%s - row_num[%d] col_num[%d] buffer_col_num[%d] rotate[%d] key_num[%d]\n", __func__, row_num, col_num, buffer_col_num, rotate, key_num);
	dev_dbg(&ts->client->dev, "%s - data_type[0x%02X] data_sign[%d] data_size[%d]\n", __func__, data_type, data_type_sign, data_type_size);

	//get buf addr
	wbuf[0] = MIP_R0_TEST;
	wbuf[1] = MIP_R1_TEST_BUF_ADDR;
	if(mit_i2c_read(ts->client, wbuf, 2, rbuf, 2)<0){
		dev_err(&ts->client->dev, "%s [ERROR] Read buf addr\n", __func__);
		goto ERROR;
	}

	buf_addr_l = rbuf[0];
	buf_addr_h = rbuf[1];
	dev_dbg(&ts->client->dev, "%s - buf_addr[0x%02X 0x%02X]\n", __func__, buf_addr_h, buf_addr_l);

	//print data
	if(mip_proc_table_data(ts, size, data_type_size, data_type_sign, buf_addr_h, buf_addr_l, row_num, col_num, buffer_col_num, rotate, key_num)){
		dev_err(&ts->client->dev, "%s [ERROR] mip_proc_table_data\n", __func__);
		goto ERROR;
	}
	//set normal mode
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_MODE;
	wbuf[2] = MIP_CTRL_MODE_NORMAL;
	if(mip_i2c_write(ts->client, wbuf, 3)){
		dev_err(&ts->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
		goto ERROR;
	}
	//wait ready status
	wait_cnt = 100;
	while(wait_cnt--){
		if(mip_get_ready_status(ts) == MIP_CTRL_STATUS_READY){
			break;
		}
                msleep(10);

		dev_dbg(&ts->client->dev, "%s - wait [%d]\n", __func__, wait_cnt);
	}

	if(wait_cnt <= 0){
		dev_err(&ts->client->dev, "%s [ERROR] Wait timeout\n", __func__);
		goto ERROR;
	}

	dev_dbg(&ts->client->dev, "%s - set normal mode\n", __func__);

	//set interrupt mode
	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_INTERRUPT;
	wbuf[2] = 1;
	if(mip_i2c_write(ts->client, wbuf, 3)<0){
		printk(KERN_ERR "%s [ERROR] Write interrupt mode\n", __func__);
		goto ERROR;
	}

	//exit
	//mutex_lock(&info->lock);
	test_busy = false;
	//mutex_unlock(&info->lock);

	dev_dbg(&ts->client->dev, "%s [DONE]\n", __func__);
	return 0;
ERROR:
	//mutex_lock(&info->lock);
	test_busy = false;
	//mutex_unlock(&info->lock);

	dev_err(&ts->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

#if 0
static int get_openshort(struct mit_data *ts)
{


	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u8 write_buf[8];
	u8 read_buf[MAX_ROW * 8];
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t temp_data[MAX_COL][MAX_ROW]={{0}};
	ts->count_short = 0;

	if (ts->pdata->check_openshort == 0)
		return 0;

	TOUCH_TRACE_FUNC();
	if (ts->dev.col_num > MAX_COL) {
		TOUCH_INFO_MSG("Err : ts->dev.col_num > MAX_COL, EXIT\n");
		return -1;
	}

	for (col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_OPENSHORT_TEST;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf,5)!=5) {
			dev_err(&client->dev, "openshort i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf,2)!=2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,1)!=1) {
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		if ( nLength > sizeof(read_buf) ) {
			TOUCH_ERR_MSG("stack overflow - nLength: %d > read_buf size:%d \n", nLength, sizeof(read_buf));
			return -1;
		}
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client,write_buf,2)!=2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,nLength)!=nLength) {
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		if (nLength > MAX_ROW) {
			TOUCH_INFO_MSG("Err : nLeangth > MAX_ROW, EXIT\n");
			return -1;
		}

		for (row = 0 ; row <nLength ; row++) {
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			temp_data[col][row] = nReference;
		}
	}

	for (row = 0; row < MAX_ROW ; row++) {
		for (col = 0; col < MAX_COL; col++) {
			ts->mit_data[row][col] = temp_data[col][row];
			if (ts->mit_data[row][col] < ts->pdata->limit->open_short_min) {
				ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 0;
				ts->count_short++;
			}
		}
	}

	return 0;

}
#endif
static int  print_rawdata(struct mit_data *ts, char *buf,int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	u32 limit_upper = 0;
	u32 limit_lower = 0;
	ts->r_min = ts->mit_data[0][0];
	ts->r_max = ts->mit_data[0][0];

	for (row = 0 ; row < ts->dev.row_num ; row++) {
		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
		}

		for (col = 0 ; col < ts->dev.col_num ; col++) {

			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
			printk("%5d ", ts->mit_data[row][col]);
			if (type == RAW_DATA_STORE) {
				ret += sprintf(buf + ret,",");
			}

			ts->r_min = (ts->r_min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->r_min;
			ts->r_max = (ts->r_max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->r_max;

		}

		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == RAW_DATA_SHOW) {
		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",ts->r_max , ts->r_min, ts->r_max - ts->r_min);
		TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",ts->r_max , ts->r_min, ts->r_max - ts->r_min);
	}

	if (ts->module.otp == OTP_APPLIED) {
		limit_upper = ts->pdata->limit->raw_data_otp_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_otp_min - ts->pdata->limit->raw_data_margin;
	} else {
		limit_upper = ts->pdata->limit->raw_data_max + ts->pdata->limit->raw_data_margin;
		limit_lower = ts->pdata->limit->raw_data_min - ts->pdata->limit->raw_data_margin;
	}

	for (row = 0; row < ts->dev.row_num; row++) {
		for (col = 0; col < ts->dev.col_num; col++) {
			if (ts->mit_data[row][col] < limit_lower || ts->mit_data[row][col] > limit_upper)
				ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0;
		}
	}

	return ret;
}

static int  print_cm_delta_data(struct mit_data *ts, char *buf, int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	ts->d_min = ts->mit_data[0][0];
	ts->d_max = ts->mit_data[0][0];

	for (row = 0 ; row < ts->dev.row_num ; row++) {
		if (type == CM_DELTA_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0 ; col < ts->dev.col_num ; col++) {

			printk("%5d ", ts->mit_data[row][col]);
			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
			if (type == CM_DELTA_STORE) {
				ret += sprintf(buf + ret,",");
			}

			ts->d_min = (ts->d_min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->d_min;
			ts->d_max = (ts->d_max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->d_max;

		}

		if (type == CM_DELTA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == CM_DELTA_SHOW) {

		if (ts->d_min < ts->pdata->limit->cm_delta)
			ts->pdata->selfdiagnostic_state[SD_CM_DELTA] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",ts->d_max , ts->d_min, ts->d_max - ts->d_min);
		TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",ts->d_max , ts->d_min, ts->d_max - ts->d_min);

		TOUCH_INFO_MSG("CM DELTA TEST SPEC : %d\n", ts->pdata->limit->cm_delta);
		ret += sprintf(buf + ret,"CM DELTA TEST SPEC : %d\n", ts->pdata->limit->cm_delta);
		if(ts->pdata->selfdiagnostic_state[SD_CM_DELTA] ==  1) {
			TOUCH_INFO_MSG("CM DELTA Test : Pass\n\n");
			ret += sprintf(buf + ret,"CM DELTA Test : PASS\n\n");
		} else {
			TOUCH_INFO_MSG("CM DELTA Test : Fail\n\n");
			ret += sprintf(buf + ret,"CM DELTA Test : FAIL\n\n");
		}
	}
	return ret;
}

static int  print_cm_jitter_data(struct mit_data *ts, char *buf, int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	ts->j_min = ts->mit_data[0][0];
	ts->j_max = ts->mit_data[0][0];

	for (row = 0 ; row < ts->dev.row_num ; row++) {
		if (type == CM_JITTER_SHOW) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0 ; col < ts->dev.col_num ; col++) {

			printk("%5d ", ts->mit_data[row][col]);
			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
			if (type == CM_JITTER_STORE) {
				ret += sprintf(buf + ret,",");
			}

			ts->j_min = (ts->j_min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->j_min;
			ts->j_max = (ts->j_max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->j_max;

		}

		if (type == CM_JITTER_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == CM_JITTER_SHOW) {

		if (ts->j_max > ts->pdata->limit->cm_jitter)
			ts->pdata->selfdiagnostic_state[SD_CM_JITTER] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",ts->j_max , ts->j_min, ts->j_max - ts->j_min);
		TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",ts->j_max , ts->j_min, ts->j_max - ts->j_min);

		TOUCH_INFO_MSG("CM JITTER TEST SPEC: %d\n", ts->pdata->limit->cm_jitter);
		ret += sprintf(buf + ret,"CM JITTER TEST SPEC: %d\n", ts->pdata->limit->cm_jitter);
		if(ts->pdata->selfdiagnostic_state[SD_CM_JITTER] ==  1) {
			TOUCH_INFO_MSG("CM JITTER Test : Pass\n\n");
			ret += sprintf(buf + ret,"CM JITTER Test : PASS\n\n");
		} else {
			TOUCH_INFO_MSG("OpenShort Test : Fail\n\n");
			ret += sprintf(buf + ret,"CM JITTER Test : FAIL\n\n");
		}
	}
	return ret;
}

static int  print_openshort_data(struct mit_data *ts, char *buf, int type)
{
	int col = 0;
	int row = 0;
	int ret = 0;
	ts->o_min = ts->mit_data[0][0];
	ts->o_max = ts->mit_data[0][0];

	for (row = 0 ; row < ts->dev.row_num ; row++) {
		if (type == OPENSHORT) {
			printk("[Touch] [%2d]  ", row);
			ret += sprintf(buf + ret,"[%2d]  ", row);
		}

		for (col = 0 ; col < ts->dev.col_num ; col++) {

			printk("%5d ", ts->mit_data[row][col]);
			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
			if (type == OPENSHORT_STORE) {
				ret += sprintf(buf + ret,",");
			}

			ts->o_min = (ts->o_min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->o_min;
			ts->o_max = (ts->o_max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : ts->o_max;

		}

		if (type == OPENSHORT) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}

	if (type == OPENSHORT) {
		if ((ts->o_min < ts->pdata->limit->open_short_min) || (ts->o_max > ts->pdata->limit->open_short_max))
			ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 0;

		printk("\n");
		ret += sprintf(buf + ret,"\n");

		ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",ts->o_max , ts->o_min, ts->o_max - ts->o_min);
		TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",ts->o_max , ts->o_min, ts->o_max - ts->o_min);

		TOUCH_INFO_MSG("OPEN / SHORT TEST SPEC(MIN : %d, MAX : %d)\n", ts->pdata->limit->open_short_min, ts->pdata->limit->open_short_max);
		ret += sprintf(buf + ret,"OPEN / SHORT TEST SPEC(MIN : %d, MAX : %d)\n", ts->pdata->limit->open_short_min, ts->pdata->limit->open_short_max);
		if(ts->pdata->selfdiagnostic_state[SD_OPENSHORT] ==  1) {
			TOUCH_INFO_MSG("OpenShort Test : Pass\n\n");
			ret += sprintf(buf + ret,"OpenShort Test : PASS\n\n");
		} else {
			TOUCH_INFO_MSG("OpenShort Test : Fail\n\n");
			ret += sprintf(buf + ret,"OpenShort Test : FAIL\n\n");
		}
	}
	return ret;
}

#if 0
static int  check_slope_data(struct mit_data *ts, char *buf)
{
	int row = 0;
	int col = 0;
	int ret = 0;
	int i = 0;
	uint16_t denominator = 1;
	//uint16_t get_data[MAX_ROW][MAX_COL]={{0}};
	uint16_t *get_data[MAX_ROW];
	ts->s_min = 500;
	ts->s_max = 0;


	for (i = 0; i < ts->dev.row_num; i++) {
				get_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
	}


	for (col = 0 ; col < ts->dev.col_num ; col++) {
		for (row = 1 ; row < ts->dev.row_num - 1 ; row++) {
			denominator = (ts->mit_data[row - 1][col] + ts->mit_data[row + 1][col]) / 2;

			if (denominator != 0) {
				get_data[row][col] = (ts->mit_data[row][col] * 100) / denominator;
			} else {
				get_data[row][col] = 0;
			}

			if (get_data[row][col] > 999) {
				get_data[row][col] = 999;
			}

			if (get_data[row][col] < ts->pdata->limit->slope_min || get_data[row][col] > ts->pdata->limit->slope_max)
				ts->pdata->selfdiagnostic_state[SD_SLOPE] = 0;

			ts->s_min = (ts->s_min > get_data[row][col]) ? get_data[row][col] : ts->s_min;
			ts->s_max = (ts->s_max < get_data[row][col]) ? get_data[row][col] : ts->s_max;

		}
	}

	for (row = 1 ; row < ts->dev.row_num - 1 ; row++) {
		ret += sprintf(buf + ret,"[%2d]  ",row);
		printk("[Touch] [%2d]  ",row);

		for (col = 0 ; col < ts->dev.col_num ; col++) {
			if (get_data[row][col] == 0) {
				ret += sprintf(buf + ret,"ERR ");
				printk("ERR ");
			} else {
				ret += sprintf(buf + ret,"%3d ", get_data[row][col]);
				printk("%3d ", get_data[row][col]);
			}
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}
	printk("\n");
	ret += sprintf(buf + ret,"\n");

	ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",ts->s_max , ts->s_min, ts->s_max - ts->s_min);
	TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",ts->s_max , ts->s_min, ts->s_max - ts->s_min);

	ret += sprintf(buf + ret,"Slope Spec(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->slope_max, ts->pdata->limit->slope_min);
	TOUCH_INFO_MSG("Slope Spec(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->slope_max, ts->pdata->limit->slope_min);

	if (ts->pdata->selfdiagnostic_state[SD_SLOPE] == 0) {
		for (row = 1 ; row < ts->dev.row_num - 1 ; row++) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
			for (col = 0 ; col < ts->dev.col_num ; col++) {
				if (get_data[row][col] >= ts->pdata->limit->slope_min && get_data[row][col] <= ts->pdata->limit->slope_max) {
					ret += sprintf(buf + ret," ,");
					printk(" ,");
				} else {
					ret += sprintf(buf + ret,"X,");
					printk("X,");
				}
			}
			printk("\n");
			ret += sprintf(buf + ret,"\n");
		}

		TOUCH_INFO_MSG("Slope : FAIL\n\n");
		ret += sprintf(buf + ret,"Slope : FAIL\n\n");
	} else {
		TOUCH_INFO_MSG("Slope : PASS\n\n");
		ret += sprintf(buf + ret,"Slope : PASS\n\n");
	}



  for (i = 0; i < ts->dev.row_num; i++) {
		if (get_data[i] != NULL) {
			kfree(get_data[i]);
	  }
  }

	return ret;
}
#endif

ssize_t mit_get_test_result(struct i2c_client *client, char *buf, int type)
{

	struct mit_data *ts = (struct mit_data *) get_touch_handle_(client);
	char temp_buf[255] = {0,};
	int i = 0;
	int ret = 0;
	int fd = 0;
	char data_path[64] = {0,};
	char *read_buf = NULL;
	int retry_max = 3;
	int retry_count = 0;

	mm_segment_t old_fs = get_fs();

	for ( i = 0 ; i < ts->dev.row_num ; i++) {
		memset(ts->mit_data[i], 0, sizeof(uint16_t) * ts->dev.col_num);
	}

	read_buf = kzalloc(sizeof(u8) * 4096,GFP_KERNEL);
	if (read_buf == NULL) {
		TOUCH_ERR_MSG("read_buf mem_error\n");
		goto mem_error;
	}

	ts->test_mode = type;


	if(CM_DELTA_SHOW == type || CM_DELTA_STORE == type) {
		while(retry_count++ < retry_max){
		   if(mip_run_test(ts, MIP_TEST_TYPE_CM_DELTA)){
			   dev_err(&ts->client->dev, "%s [ERROR] mip_run_test MIP_TEST_TYPE_CM_DELTA\n", __func__);
			   ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
			   TOUCH_INFO_MSG("%s retry (%d/%d) \n", __func__, retry_count, retry_max);
			   mit_power_reset(ts);
			   mdelay(100);

		   }
		   else break;
		   if (retry_count >= retry_max) {
				TOUCH_INFO_MSG("%s all retry failed \n", __func__);
				goto error;
		   }
		}
	}
	if(CM_JITTER_SHOW == type || CM_JITTER_STORE == type) {
		retry_count = 0;
		while(retry_count++ < retry_max){
		   if(mip_run_test(ts, MIP_TEST_TYPE_CM_JITTER)){
			   dev_err(&ts->client->dev, "%s [ERROR] mip_run_test MIP_TEST_TYPE_CM_JITTER\n", __func__);
			   ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
			   TOUCH_INFO_MSG("%s retry (%d/%d) \n", __func__, retry_count, retry_max);
			   mit_power_reset(ts);
			   mdelay(100);

		   }
		   else break;
		   if (retry_count >= retry_max) {
				TOUCH_INFO_MSG("%s all retry failed \n", __func__);
				goto error;
		   }
		}
	}
	if(OPENSHORT == type || OPENSHORT_STORE == type ) {
		retry_count = 0;
		while(retry_count++ < retry_max){
		   if(mip_run_test(ts, MIP_TEST_TYPE_SHORT)){
			   dev_err(&ts->client->dev, "%s [ERROR] mip_run_test MIP_TEST_TYPE_SHORT\n", __func__);
			   ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
			   TOUCH_INFO_MSG("%s retry (%d/%d) \n", __func__, retry_count, retry_max);
			   mit_power_reset(ts);
			   mdelay(100);
		   }
		   else break;
		   if (retry_count >= retry_max) {
				TOUCH_INFO_MSG("%s all retry failed \n", __func__);
				goto error;
		   }
		}
	}
	retry_count = 0;

	if (ts->test_mode == RAW_DATA_SHOW || ts->test_mode == RAW_DATA_STORE ) {
		retry_count = 0;
	    while(retry_count++ < retry_max){
		   if(get_rawdata(ts) == -1){
			   TOUCH_ERR_MSG("getting raw data failed\n");
			   ret = snprintf(buf, PAGE_SIZE, "%s\n", "ERROR");
			   TOUCH_INFO_MSG("%s retry (%d/%d) \n", __func__, retry_count, retry_max);
			   mit_power_reset(ts);
			   mdelay(100);
		   }
		   else break;
		   if (retry_count >= retry_max) {
				TOUCH_INFO_MSG("%s all retry failed \n", __func__);
				goto error;
		   }
	   }
	}

	switch(type) {
		case RAW_DATA_SHOW:
			ret = print_rawdata(ts, buf, type);
			if (ret < 0) {
				TOUCH_ERR_MSG("fail to print raw data\n");
				ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0;
				goto error;
			}
			break;
		case RAW_DATA_STORE:
			snprintf(temp_buf, strlen(buf), "%s", buf);
			sprintf(data_path, "/sdcard/%s.csv", temp_buf);

			ret = print_rawdata(ts, read_buf, type);
			if (ret < 0) {
				TOUCH_ERR_MSG("fail to print raw data\n");
				ts->pdata->selfdiagnostic_state[SD_RAWDATA] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_INFO_MSG("%s saved \n", data_path);
			} else {
				TOUCH_INFO_MSG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case OPENSHORT :
			if (ts->pdata->check_openshort == 1)
				ret = print_openshort_data(ts, buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 0;
				goto error;
			}
			break;
		case OPENSHORT_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_openshort.csv", temp_buf);
			if (ts->pdata->check_openshort == 1)
				ret = print_openshort_data(ts, read_buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_OPENSHORT] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_INFO_MSG("%s saved \n", data_path);
			} else {
				TOUCH_INFO_MSG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case CM_DELTA_SHOW:
				ret = print_cm_delta_data(ts, buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_CM_DELTA] = 0;
				goto error;
			}
			break;
		case CM_DELTA_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_cmdelta.csv", temp_buf);
			ret = print_cm_delta_data(ts, read_buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_CM_DELTA] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_INFO_MSG("%s saved \n", data_path);
			} else {
				TOUCH_INFO_MSG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case CM_JITTER_SHOW:
				ret = print_cm_jitter_data(ts, buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_CM_JITTER] = 0;
				goto error;
			}
			break;
		case CM_JITTER_STORE :
			snprintf(temp_buf,strlen(buf),"%s", buf);
			sprintf(data_path,"/sdcard/%s_cmjitter.csv", temp_buf);
			ret = print_cm_jitter_data(ts, read_buf, type);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_CM_JITTER] = 0;
				goto error;
			}

			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY | O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_INFO_MSG("%s saved \n", data_path);
			} else {
				TOUCH_INFO_MSG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
/*
		case SLOPE :
			ret = check_slope_data(ts, buf);
			if ( ret < 0) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[SD_SLOPE] = 0;
				goto error;
			}
			break;
*/
		default :
			TOUCH_INFO_MSG("type = default[%d]\n", type);
			break;
		}

	if (read_buf != NULL)
		kfree(read_buf);

	return ret;

error :
	if (read_buf != NULL)
		kfree(read_buf);

	return -1;

mem_error :
	if (read_buf != NULL)
		kfree(read_buf);
	return -1;


}

