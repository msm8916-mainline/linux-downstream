/*
 * Copyright (c) 2012 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include <linux/kernel.h>
#include<linux/moduleparam.h>
#include <linux/rmi.h>
#include <linux/time.h>
#include <linux/gpio.h>
//#include <mach/gpiomux.h>
#include <linux/of_gpio.h>
#include "rmi_driver.h"
#include "rmi_f01.h"
#include "rmi_f34.h"

#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_touchscreen_common.h>



#define HAS_BSR_MASK 0x20

#define CHECKSUM_OFFSET 0
#define BOOTLOADER_VERSION_OFFSET 0x07
#define IMAGE_SIZE_OFFSET 0x08
#define CONFIG_SIZE_OFFSET 0x0C
#define PRODUCT_ID_OFFSET 0x10
#define PRODUCT_ID_SIZE 10
#define PRODUCT_INFO_OFFSET 0x1E
#define PRODUCT_INFO_SIZE 2

#define F01_RESET_MASK 0x01

#define ENABLE_WAIT_US (300 * 1000)

/*chenyunzhe modify start   .....*/

struct firmware_data_map_with_gpio {
	int module_id;
	int lcd_id;
	u8 *firmware_data;
};

struct product_gpio_fw_disc {
    const char *product_name;  //Such as "PD1401F ",...and so on
	u8    *default_firmware_data;
    int   fw_cnt;
	struct firmware_data_map_with_gpio *data_maps;
};

struct firmware_data_map_with_product_id {
	char product_id[10];   //ts-ic's id
    int lcd_id;
	u8 *firmware_data;
};

struct product_pid_fw_disc {
    const char *product_name;   //Such as "PD1401F ",...and so on
	u8    *default_firmware_data;
    int   fw_cnt;
	struct firmware_data_map_with_product_id *data_maps;
};

struct firmware_data_map_with_lcd_id {
	int lcd_id;
	u8 *firmware_data;
};

struct product_lid_fw_disc {
    const char *product_name;  //Such as "PD1401F ",...and so on
	u8     *default_firmware_data;
    int    fw_cnt;
	struct firmware_data_map_with_lcd_id *data_maps;
};



struct product_fw_disc {
    struct product_gpio_fw_disc *gpio_fw_disc;
	int gpio_fw_disc_cnt;
	struct product_pid_fw_disc *pid_fw_disc;
	int pid_fw_disc_cnt;
	struct product_lid_fw_disc  *lid_fw_disc;
	int lid_fw_disc_cnt;
};


/*  
   GPIO_IN_CTW_ID1_PIN GPIO_IN_CTW_ID2_PIN is used for module check
   and we have most two gpio for module check
   if only one used the other should define as -1
   if both are -1 then there is no gpio for module check

   module_id_map is the map from gpio read value to module number we use
   if only one gpio is used array[0] <--> gpio = 0 array[1] <--> gpio = 1
   if two use array[0] <--> gpio1 = 0 gpio2 = 0 array[1] <--> gpio1 = 1 gpio2 = 0
			  array[3] <--> gpio1 = 0 gpio2 = 1 array[3] <--> gpio1 = 1 gpio2 = 1
*/

//static bool  is_try_module = false;
#if 0
static bool is_lens_changed = false;
#endif
static int GPIO_IN_CTW_ID1_PIN = -1;
static int GPIO_IN_CTW_ID2_PIN = -1;



#define DEFAULT_LCD_ID 0

static u8 TRY_GFFW_TRY[]={
	#include "PD1401V_TRY_gffw_TRY_fw.i"
};
static u8 BIE_GFFW_BYD[]={
	#include "PD1401V_bie_gffw_byd_fw.i"
};
static u8 YFT_GFFW_BYD[]={
	#include "PD1401V_yft_gffw_byd_fw.i"
};

static u8 JTH_GFFW_BYD[]={
	#include "PD1401V_JTH_gffw_byd_fw.i"
};

static u8 PD1502F_EX_YFT_GFFW_TRY[]={
	#include "PD1502F_EX_TRY_gffw_TRY_fw.i"
};
static u8 PD1502A_YFT_GFFW_TRY[]={
	#include "PD1502A_TRY_gffw_TRY_fw.i"
};
static u8 PD1502A_YFT_GFFW_LNS[]={
	#include "PD1502A_LNS_gffw_LNS_fw.i"
};
static u8 PD1309F_EX_XXXX[]={
	#include "PD1309F_EX_XXXX.i"
};


static struct firmware_data_map_with_gpio pd1502f_ex_gpio_data_maps[] = {
    {0,DEFAULT_LCD_ID,PD1502F_EX_YFT_GFFW_TRY},	
}; 

static struct firmware_data_map_with_gpio pd1502a_gpio_data_maps[] = {
    {0,DEFAULT_LCD_ID,PD1502A_YFT_GFFW_TRY},
	{1,DEFAULT_LCD_ID,PD1502A_YFT_GFFW_LNS},	
	{2,DEFAULT_LCD_ID,PD1502A_YFT_GFFW_LNS},	
	{3,DEFAULT_LCD_ID,PD1502A_YFT_GFFW_LNS},	
}; 

static struct firmware_data_map_with_gpio pd1401cl_gpio_data_maps[] = {
    {0,DEFAULT_LCD_ID,TRY_GFFW_TRY},
	{1,DEFAULT_LCD_ID,BIE_GFFW_BYD},
	{2,DEFAULT_LCD_ID,YFT_GFFW_BYD},
}; 
static struct firmware_data_map_with_gpio pd1309fex_gpio_data_maps[] = {
    {0,DEFAULT_LCD_ID,PD1309F_EX_XXXX},
	{1,DEFAULT_LCD_ID,PD1309F_EX_XXXX},
	{2,DEFAULT_LCD_ID,PD1309F_EX_XXXX},
	{3,DEFAULT_LCD_ID,PD1309F_EX_XXXX},
}; 


static struct product_gpio_fw_disc all_gpio_fw_discs[] = {
    {
      .product_name = "PD1401CL",
	  .default_firmware_data = TRY_GFFW_TRY,
	  .fw_cnt = ARRAY_SIZE(pd1401cl_gpio_data_maps),
	  .data_maps = pd1401cl_gpio_data_maps,
	},
	{
      .product_name = "PD1502F_EX",
	  .default_firmware_data = PD1502F_EX_YFT_GFFW_TRY,
	  .fw_cnt = ARRAY_SIZE(pd1502f_ex_gpio_data_maps),
	  .data_maps = pd1502f_ex_gpio_data_maps,
	},
	{
      .product_name = "PD1502A",
	  .default_firmware_data = PD1502A_YFT_GFFW_TRY,
	  .fw_cnt = ARRAY_SIZE(pd1502a_gpio_data_maps),
	  .data_maps = pd1502a_gpio_data_maps,
	},
	{
      .product_name = "PD1309F_EX",
	  .default_firmware_data = PD1309F_EX_XXXX,
	  .fw_cnt = ARRAY_SIZE(pd1309fex_gpio_data_maps),
	  .data_maps = pd1309fex_gpio_data_maps,
	},
	//TODO: add product here following...
};

static struct firmware_data_map_with_product_id pd1401cl_pid_data_maps[] = {
    {"TRY-GFFW",DEFAULT_LCD_ID, TRY_GFFW_TRY},
	{"BIE-GFFW",DEFAULT_LCD_ID, BIE_GFFW_BYD},
	{"YFT-GFFW",DEFAULT_LCD_ID, YFT_GFFW_BYD},
	{"YFT2GFFW",DEFAULT_LCD_ID, YFT_GFFW_BYD},
	{"70-3GFFW",DEFAULT_LCD_ID, TRY_GFFW_TRY},
	{"40-3GFFW",DEFAULT_LCD_ID, JTH_GFFW_BYD},
}; 

static struct product_pid_fw_disc all_pid_fw_discs[] = {
    {
      .product_name = "PD1401CL",
	  .default_firmware_data = NULL,
	  .fw_cnt =  ARRAY_SIZE(pd1401cl_pid_data_maps),
	  .data_maps =  pd1401cl_pid_data_maps,
	},
	//TODO: add product here following...
};

static struct firmware_data_map_with_lcd_id pd1401cl_lcd_id_data_maps[] = {
    {0x32,TRY_GFFW_TRY},
	{0x22,BIE_GFFW_BYD},
	{0x23,YFT_GFFW_BYD},
}; 
static struct product_lid_fw_disc all_lid_fw_discs[] = {
    {
      .product_name = "PD1401CL",
	  .default_firmware_data = NULL,
	  .fw_cnt = ARRAY_SIZE(pd1401cl_lcd_id_data_maps),
	  .data_maps = pd1401cl_lcd_id_data_maps,
	},
	//TODO: add product here following...
};

static struct product_fw_disc all_product_fw_discs ={
    .gpio_fw_disc = all_gpio_fw_discs,
	.gpio_fw_disc_cnt = ARRAY_SIZE(all_gpio_fw_discs),
	.pid_fw_disc = all_pid_fw_discs,
	.pid_fw_disc_cnt = ARRAY_SIZE(all_pid_fw_discs),
	.lid_fw_disc = all_lid_fw_discs,
	.lid_fw_disc_cnt = ARRAY_SIZE(all_lid_fw_discs),
};

static struct product_gpio_fw_disc       *curr_product_gpio_fw_disc = NULL;
static struct product_pid_fw_disc        *curr_product_pid_fw_disc = NULL;
static struct product_lid_fw_disc        *curr_product_lid_fw_disc = NULL;
static         u8 *FIRMWARE_DATA;

/*chenyunzhe modify end   .....*/

//#define BBK_TOUCH_IC_IS_COF_MODE 1//add by qiuguifu. COF Project have no DEFAULT FW.
//static u8 *FIRMWARE_DATA_DEFAULT = NULL;

/** Image file V5, Option 0
 */
struct image_header {
	u32 checksum;
	unsigned int image_size;
	unsigned int config_size;
	unsigned char options;
	unsigned char bootloader_version;
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	unsigned char product_info[PRODUCT_INFO_SIZE];
};

static u32 extract_u32(const u8 *ptr)
{
	return (u32)ptr[0] +
		(u32)ptr[1] * 0x100 +
		(u32)ptr[2] * 0x10000 +
		(u32)ptr[3] * 0x1000000;
}

struct reflash_data {
	struct rmi_device *rmi_dev;
	struct pdt_entry *f01_pdt;
	union f01_basic_queries f01_queries;
	u8 product_id[RMI_PRODUCT_ID_LENGTH+1];
	struct pdt_entry *f34_pdt;
	u8 bootloader_id[2];
	union f34_query_regs f34_queries;
	union f34_control_status f34_controls;
	const u8 *firmware_data;
	const u8 *config_data;
};

extern struct rmi_function_container *rmi_get_function_container(
					struct rmi_driver_data *ddata, int function_number);
extern int rmi_f11_get_sensor_electrodes(struct rmi_function_container *fc,
							int *rx_num, int *tx_num);
extern int rmi_dev_get_fn54_data(struct rmi_function_container *fc, char **dest_data,
					unsigned char function_number);
/* If this parameter is true, we will update the firmware regardless of
 * the versioning info.
 */
static bool force = 1;
module_param(force, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(param, "Force reflash of RMI4 devices");

/* If this parameter is not NULL, we'll use that name for the firmware image,
 * instead of getting it from the F01 queries.
 */
static char *img_name = "test";
module_param(img_name, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(param, "Name of the RMI4 firmware image");

#define RMI4_IMAGE_FILE_REV1_OFFSET 30
#define RMI4_IMAGE_FILE_REV2_OFFSET 31
#define IMAGE_FILE_CHECKSUM_SIZE 4
#define FIRMWARE_IMAGE_AREA_OFFSET 0x100

static void extract_header(const u8 *data, int pos, struct image_header *header)
{
	header->checksum = extract_u32(&data[pos + CHECKSUM_OFFSET]);
	header->bootloader_version = data[pos + BOOTLOADER_VERSION_OFFSET];
	header->image_size = extract_u32(&data[pos + IMAGE_SIZE_OFFSET]);
	header->config_size = extract_u32(&data[pos + CONFIG_SIZE_OFFSET]);
	memcpy(header->product_id, &data[pos + PRODUCT_ID_OFFSET],
	       RMI_PRODUCT_ID_LENGTH);
	header->product_id[PRODUCT_ID_SIZE] = 0;
	memcpy(header->product_info, &data[pos + PRODUCT_INFO_OFFSET],
	       PRODUCT_INFO_SIZE);
}

static int rescan_pdt(struct reflash_data *data)
{
	int retval;
	bool f01_found;
	bool f34_found;
	struct pdt_entry pdt_entry;
	int i;
	struct rmi_device *rmi_dev = data->rmi_dev;
	struct pdt_entry *f34_pdt = data->f34_pdt;
	struct pdt_entry *f01_pdt = data->f01_pdt;

	/* Per spec, once we're in reflash we only need to look at the first
	 * PDT page for potentially changed F01 and F34 information.
	 */
	for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) {
		retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					sizeof(pdt_entry));
		if (retval != sizeof(pdt_entry)) {
			VIVO_TS_LOG_ERR("[%s]:Read PDT entry at %#06x failed: %d.\n", __func__,
				i, retval);
			return retval;
		}

		if (RMI4_END_OF_PDT(pdt_entry.function_number))
			break;

		if (pdt_entry.function_number == 0x01) {
			memcpy(f01_pdt, &pdt_entry, sizeof(pdt_entry));
			f01_found = true;
		} else if (pdt_entry.function_number == 0x34) {
			memcpy(f34_pdt, &pdt_entry, sizeof(pdt_entry));
			f34_found = true;
		}
	}

	if (!f01_found) {
		VIVO_TS_LOG_ERR("[%s]:Failed to find F01 PDT entry.\n", __func__);
		retval = -ENODEV;
	} else if (!f34_found) {
		VIVO_TS_LOG_ERR("[%s]:Failed to find F34 PDT entry.\n", __func__);
		retval = -ENODEV;
	} else
		retval = 0;

	return retval;
}

static int read_f34_controls(struct reflash_data *data)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f34_controls.address,
			  data->f34_controls.regs);
	if (retval < 0)
		return retval;
	//dev_info(&data->rmi_dev->dev, "Last F34 status byte: %#04x\n", data->f34_controls.regs[0]);
	return 0;
}

static int read_f01_status(struct reflash_data *data,
			   union f01_device_status *device_status)
{
	int retval;

	retval = rmi_read(data->rmi_dev, data->f01_pdt->data_base_addr,
			  device_status->regs);
	if (retval < 0)
		return retval;
	VIVO_TS_LOG_INF("[%s]:Last F01 status byte: %#04x\n", __func__, device_status->regs[0]);
	return 0;
}

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

/* Wait until the status is idle and we're ready to continue */
static int wait_for_idle(struct reflash_data *data, int timeout_ms)
{
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	int count = 0;
	union f34_control_status *controls = &data->f34_controls;
	int retval;

	do {
		if (count || timeout_count == 1)
			usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);
		retval = read_f34_controls(data);
		count++;
		if (retval < 0)
			continue;
		else if (IS_IDLE(controls))
			return 0;
	} while (count < timeout_count);

	VIVO_TS_LOG_ERR("[%s]:ERROR: Timeout waiting for idle status, last status: %#04x.\n", __func__,
		controls->regs[0]);
	VIVO_TS_LOG_ERR("[%s]:Command: %#04x\n", __func__, controls->command);
	VIVO_TS_LOG_ERR("[%s]:Status:  %#04x\n", __func__, controls->status);
	VIVO_TS_LOG_ERR("[%s]:Enabled: %d\n", __func__, controls->program_enabled);
	VIVO_TS_LOG_ERR("[%s]:Idle:    %d\n", __func__, IS_IDLE(controls));
	return -ETIMEDOUT;
}


static int read_f01_queries(struct reflash_data *data)
{
	int retval;
	u16 addr = data->f01_pdt->query_base_addr;

	retval = rmi_read_block(data->rmi_dev, addr, data->f01_queries.regs,
				ARRAY_SIZE(data->f01_queries.regs));
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read F34 queries (code %d).\n", __func__, retval);
		return retval;
	}
	addr += ARRAY_SIZE(data->f01_queries.regs);

	retval = rmi_read_block(data->rmi_dev, addr, data->product_id,
				RMI_PRODUCT_ID_LENGTH);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read product ID (code %d).\n", __func__, retval);
		return retval;
	}
	data->product_id[RMI_PRODUCT_ID_LENGTH] = 0;
	VIVO_TS_LOG_INF("[%s]:F01 Product id:   %s\n", __func__,
			data->product_id);
	VIVO_TS_LOG_INF("[%s]:F01 product info: %#04x %#04x\n", __func__,
			data->f01_queries.productinfo_1,
			data->f01_queries.productinfo_2);

	return 0;
}

static int read_f34_queries(struct reflash_data *data)
{
	int retval;
	u8 id_str[3];

	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->query_base_addr,
				data->bootloader_id, 2);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read F34 bootloader_id (code %d).\n", __func__,
			retval);
		return retval;
	}
	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->query_base_addr+2,
			data->f34_queries.regs,
			ARRAY_SIZE(data->f34_queries.regs));
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read F34 queries (code %d).\n", __func__, retval);
		return retval;
	}
	data->f34_queries.block_size =
			le16_to_cpu(data->f34_queries.block_size);
	data->f34_queries.fw_block_count =
			le16_to_cpu(data->f34_queries.fw_block_count);
	data->f34_queries.config_block_count =
			le16_to_cpu(data->f34_queries.config_block_count);
	id_str[0] = data->bootloader_id[0];
	id_str[1] = data->bootloader_id[1];
	id_str[2] = 0;
#ifdef DEBUG
	VIVO_TS_LOG_INF("[%s]:Got F34 data->f34_queries.\n", __func__);
	VIVO_TS_LOG_INF("[%s]:F34 bootloader id: %s (%#04x %#04x)\n", __func__,
		 id_str, data->bootloader_id[0], data->bootloader_id[1]);
	VIVO_TS_LOG_INF("[%s]:F34 has config id: %d\n", __func__,
		 data->f34_queries.has_config_id);
	VIVO_TS_LOG_INF("[%s]:F34 unlocked:      %d\n", __func__,
		 data->f34_queries.unlocked);
	VIVO_TS_LOG_INF("[%s]:F34 regMap:        %d\n", __func__,
		 data->f34_queries.reg_map);
	VIVO_TS_LOG_INF("[%s]:F34 block size:    %d\n", __func__,
		 data->f34_queries.block_size);
	VIVO_TS_LOG_INF("[%s]:F34 fw blocks:     %d\n", __func__,
		 data->f34_queries.fw_block_count);
	VIVO_TS_LOG_INF("[%s]:F34 config blocks: %d\n", __func__,
		 data->f34_queries.config_block_count);
#endif

	data->f34_controls.address = data->f34_pdt->data_base_addr +
			F34_BLOCK_DATA_OFFSET + data->f34_queries.block_size;

	return 0;
}

static int write_bootloader_id(struct reflash_data *data)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;
	struct pdt_entry *f34_pdt = data->f34_pdt;

	retval = rmi_write_block(rmi_dev,
			f34_pdt->data_base_addr + F34_BLOCK_DATA_OFFSET,
			data->bootloader_id, ARRAY_SIZE(data->bootloader_id));
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to write bootloader ID. Code: %d.\n", __func__, retval);
		return retval;
	}

	return 0;
}

static int write_f34_command(struct reflash_data *data, u8 command)
{
	int retval;
	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = rmi_write(rmi_dev, data->f34_controls.address, command);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to write F34 command %#04x. Code: %d.\n", __func__,
			command, retval);
		return retval;
	}

	return 0;
}

static int enter_flash_programming(struct reflash_data *data)
{
	int retval;
	union f01_device_status device_status;
//	struct rmi_device *rmi_dev = data->rmi_dev;

	retval = write_bootloader_id(data);
	if (retval < 0)
		return retval;

	VIVO_TS_LOG_INF("[%s]:Enabling flash programming.\n", __func__);
	retval = write_f34_command(data, F34_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = wait_for_idle(data, F34_ENABLE_WAIT_MS);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Did not reach idle state after %d ms. Code: %d.\n", __func__,
			F34_ENABLE_WAIT_MS, retval);
		return retval;
	}
	msleep(10);	/* wyl add */
	read_f34_controls(data);	/* wyl add */
	if (!data->f34_controls.program_enabled) {
		VIVO_TS_LOG_ERR("[%s]:Reached idle, but programming not enabled (current status register: %#04x.\n", __func__, data->f34_controls.regs[0]);
		VIVO_TS_LOG_ERR("[%s]:Control reg addr is %d\n", __func__, data->f34_controls.address);
		read_f01_status(data, &device_status);
		VIVO_TS_LOG_INF("[%s]:Rereading F34 status.\n", __func__);
		read_f34_controls(data);
		return -EINVAL;
	}
	VIVO_TS_LOG_INF("[%s]:HOORAY! Programming is enabled!\n", __func__);

	retval = rescan_pdt(data);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed to rescan pdt.  Code: %d.\n", __func__,
			retval);
		return retval;
	}

	retval = read_f01_status(data, &device_status);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read F01 status after enabling reflash. Code: %d.\n", __func__,
			retval);
		return retval;
	}
	if (!(device_status.flash_prog)) {
		VIVO_TS_LOG_ERR("[%s]:Device reports as not in flash programming mode.\n", __func__);
		return -EINVAL;
	}

	retval = read_f34_queries(data);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:F34 queries failed, code = %d.\n", __func__,
			retval);
		return retval;
	}

	return retval;
}

static void reset_device(struct reflash_data *data)
{
	int retval;
	struct rmi_device_platform_data *pdata =
		to_rmi_platform_data(data->rmi_dev);

	VIVO_TS_LOG_INF("[%s]:Resetting...\n", __func__);
	retval = rmi_write(data->rmi_dev, data->f01_pdt->command_base_addr,
			   F01_RESET_MASK);
	if (retval < 0)
		VIVO_TS_LOG_ERR("[%s]:WARNING - post-flash reset failed, code: %d.\n", __func__,
			 retval);
	msleep(pdata->reset_delay_ms);
	VIVO_TS_LOG_INF("[%s]:Reset completed.\n", __func__);
}

/*
 * Send data to the device one block at a time.
 */
static int write_blocks(struct reflash_data *data, u8 *block_ptr,
			u16 block_count, u8 cmd)
{
	int block_num;
	u8 zeros[] = {0, 0};
	int retval;
	u16 addr = data->f34_pdt->data_base_addr + F34_BLOCK_DATA_OFFSET;

	retval = rmi_write_block(data->rmi_dev, data->f34_pdt->data_base_addr,
				 zeros, ARRAY_SIZE(zeros));
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to write initial zeros. Code=%d.\n", __func__,
			retval);
		return retval;
	}

	for (block_num = 0; block_num < block_count; ++block_num) {
		retval = rmi_write_block(data->rmi_dev, addr, block_ptr,
					 data->f34_queries.block_size);
		if (retval < 0) {
			VIVO_TS_LOG_ERR("[%s]:Failed to write block %d. Code=%d.\n", __func__,
				block_num, retval);
			return retval;
		}

		retval = write_f34_command(data, cmd);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Failed to write command for block %d. Code=%d.\n", __func__,
				block_num, retval);
			return retval;
		}


		retval = wait_for_idle(data, F34_IDLE_WAIT_MS);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Failed to go idle after writing block %d. Code=%d.\n", __func__,
				block_num, retval);
			return retval;
		}

		block_ptr += data->f34_queries.block_size;
	}

	return 0;
}

static int write_firmware(struct reflash_data *data)
{
	return write_blocks(data, (u8 *) data->firmware_data,
		data->f34_queries.fw_block_count, F34_WRITE_FW_BLOCK);
}

static int write_configuration(struct reflash_data *data)
{
	return write_blocks(data, (u8 *) data->config_data,
		data->f34_queries.config_block_count, F34_WRITE_CONFIG_BLOCK);
}

static void reflash_firmware(struct reflash_data *data)
{
#ifdef DEBUG
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
#endif
	int retval = 0;

	retval = enter_flash_programming(data);
	if (retval)
		return;

	retval = write_bootloader_id(data);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed write bootloader id\n", __func__);
		return;
	}

#ifdef	DEBUG
	VIVO_TS_LOG_INF("[%s]:Erasing FW...\n", __func__);
	getnstimeofday(&start);
#endif
	retval = write_f34_command(data, F34_ERASE_ALL);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Write Erase Command Error\n", __func__);
		return;
	}

	retval = wait_for_idle(data, F34_ERASE_WAIT_MS);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed to reach idle state. Code: %d.\n", __func__, retval);
		return;
	}
#ifdef	DEBUG
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	VIVO_TS_LOG_INF("[%s]:Erase complete, time: %lld ns.\n", __func__, duration_ns);
#endif

	if (data->firmware_data) {
#ifdef	DEBUG
		VIVO_TS_LOG_INF("[%s]:Writing firmware...\n", __func__);
		getnstimeofday(&start);
#endif
		retval = write_firmware(data);
		if (retval) {
			VIVO_TS_LOG_ERR("[%s]:Failed write firmware\n", __func__);
			return;
		}
#ifdef	DEBUG
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		VIVO_TS_LOG_INF("[%s]:Done writing FW, time: %lld ns.\n", __func__, duration_ns);
#endif
	}

	if (data->config_data) {
#ifdef	DEBUG
		VIVO_TS_LOG_INF("[%s]:Writing configuration...\n", __func__);
		getnstimeofday(&start);
#endif
		retval = write_configuration(data);
		if (retval)
			return;
#ifdef	DEBUG
		getnstimeofday(&end);
		duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
		VIVO_TS_LOG_INF("[%s]:Done writing config, time: %lld ns.\n", __func__, duration_ns);
#endif
	}

	VIVO_TS_LOG_INF("[%s]: Firmware Update Success!!!\n", __func__);

}

int syna3202_get_lcd_id(void)
{    
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_LCDID || 
	       vivo_touchscreen_is_support(FS_TS_FW_UPGRADE_LCD_ID_REF)) {
	   //todo:return current lcd_id,but now we return default lcd_id first
	     return  DEFAULT_LCD_ID ;   
	}  
    
	return DEFAULT_LCD_ID;
}

static int initial_set_fw_data(void)
{ 
    const char *product_name = NULL;
	struct product_gpio_fw_disc *gpio_fw_disc = NULL;
	struct product_pid_fw_disc  *pid_fw_disc = NULL;
	struct product_lid_fw_disc  *lid_fw_disc = NULL;
	int gpio_fw_disc_cnt;
	int pid_fw_disc_cnt;
	int lid_fw_disc_cnt;
	int i;
	
	int rc = -1;
	
    vivo_touchscreen_get_product_name(&product_name);
	if(NULL == product_name)
	{
	    VIVO_TS_LOG_ERR("[%s]:no product name!! pls check device tree\n",__func__);
		return rc;
	}
	
	VIVO_TS_LOG_INF("[%s]: product name=%s\n",__func__,product_name);
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_GPIO) {
	    gpio_fw_disc     = all_product_fw_discs.gpio_fw_disc;
		gpio_fw_disc_cnt = all_product_fw_discs.gpio_fw_disc_cnt;
		
		for(i=0; i<gpio_fw_disc_cnt; i++) {
		     if(!strcmp(gpio_fw_disc[i].product_name,product_name)) {
			    curr_product_gpio_fw_disc =  &gpio_fw_disc[i];
				rc = 0;
				break;
			 }
		}
		
	} 
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_ICID) {
	    pid_fw_disc     = all_product_fw_discs.pid_fw_disc;
		pid_fw_disc_cnt = all_product_fw_discs.pid_fw_disc_cnt;
		
		for(i=0; i<pid_fw_disc_cnt; i++) {
		     if(!strcmp(pid_fw_disc[i].product_name,product_name)) {
			    curr_product_pid_fw_disc =  &pid_fw_disc[i];
				rc = 0;
				break;
			 }
		}
	} 
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_LCDID) {
	    lid_fw_disc     = all_product_fw_discs.lid_fw_disc;
		lid_fw_disc_cnt = all_product_fw_discs.lid_fw_disc_cnt;
		
		for(i=0; i<lid_fw_disc_cnt; i++) {
		     if(!strcmp(lid_fw_disc[i].product_name,product_name)) {
			    curr_product_lid_fw_disc =  &lid_fw_disc[i];
				rc = 0;
				break;
			 }
		}
	} 
	
	
	return rc;
}

/* wyl add */
/* need to add the firmware data config func*/
int hardware_id_check(void)
{
    int module_id = -1;
	int module_id_1 = -1;
	int module_id_2 = -1;
	int rc;

	GPIO_IN_CTW_ID1_PIN = vivo_touchscreen_get_module_id_gpio(1);
	GPIO_IN_CTW_ID2_PIN = vivo_touchscreen_get_module_id_gpio(2);

	VIVO_TS_LOG_INF("[%s]:PINCTL:PIN1:%d,PIN2:%d\n", __func__, GPIO_IN_CTW_ID1_PIN, GPIO_IN_CTW_ID2_PIN);

	vivo_touchscreen_id_pinctrl(1);
	
	if (GPIO_IN_CTW_ID1_PIN != -1) {
	    rc = gpio_request(GPIO_IN_CTW_ID1_PIN, "ctw_id1_pin");
		if (!rc) {
			msleep(10);
			//module_id_1 = gpio_direction_input(GPIO_IN_CTW_ID1_PIN);			
			module_id_1 = gpio_get_value(GPIO_IN_CTW_ID1_PIN);
		} else {
			VIVO_TS_LOG_ERR("[%s]:request ctw_id1 gpio failed, rc=%d\n",__func__,rc);	
            //return module_id;		
		}		
	}

	if (GPIO_IN_CTW_ID2_PIN != -1) {
		rc = gpio_request(GPIO_IN_CTW_ID2_PIN, "ctw_id2_pin");
		if (!rc) {
			msleep(10);
			//imodule_id_2 = gpio_direction_input(GPIO_IN_CTW_ID2_PIN);	
			module_id_2 = gpio_get_value(GPIO_IN_CTW_ID2_PIN);
		} else {
			VIVO_TS_LOG_ERR("[%s]:request ctw_id2 gpio failed, rc=%d\n",__func__,rc);	
            //return module_id;			
		}
	}

	vivo_touchscreen_id_pinctrl(0);

	if (module_id_1 == -1) {
		VIVO_TS_LOG_ERR("[%s]: No gpio for check the module is\n", __func__);
		return -1;
	}else{
		if (module_id_2 == -1) {
			VIVO_TS_LOG_ERR("[%s]: Only one gpio for module check\n", __func__);
			VIVO_TS_LOG_ERR("[%s]: module_id_1 is %d\n", __func__, module_id_1);
			
			module_id = module_id_1;
			/*
			if (!module_id_map) {
				printk(KERN_ERR "%s: There's no module_id_map array\n", __func__);
				return -1;
			} else {
				return module_id_map[module_id_1];
			}
			*/
		}else{
			VIVO_TS_LOG_INF("[%s]: Two gpio for module check\n", __func__);
			VIVO_TS_LOG_INF("[%s]: module_id_1(%d) module_id_2(%d)\n", __func__,
							module_id_1, module_id_2);
							
			module_id = (module_id_2 << 1) | module_id_1;
			
			/*
			if (!module_id_map) {
				printk(KERN_ERR "%s: There's no module_id_map array\n", __func__);
				return -1;
			} else {
				return module_id_map[(module_id_2 << 1) | module_id_1];
			}
			*/
		}
	}
    VIVO_TS_LOG_INF("[%s]:PINCTL:module_id:%d\n", __func__, module_id);	
	
	return module_id;
}

/* won't return NULL */
static u8 *get_host_firmware_data_with_gpio_id(int module_id)
{
#if 1
    int i;
    u8 *ret = NULL;
	
	VIVO_TS_LOG_INF("[%s]: module id is %x\n",__func__,module_id);
	
    if(curr_product_gpio_fw_disc != NULL)
	{
		for (i=0; i<curr_product_gpio_fw_disc->fw_cnt; i++) {
	         if(curr_product_gpio_fw_disc->data_maps[i].module_id == module_id) {
			    if(syna3202_get_lcd_id() == curr_product_gpio_fw_disc->data_maps[i].lcd_id)
			        return curr_product_gpio_fw_disc->data_maps[i].firmware_data;
			 }
	    }
		
		ret = curr_product_gpio_fw_disc->default_firmware_data;
    }
	
	VIVO_TS_LOG_INF("[%s]:There is no firmware data map the gpio id use default\n",__func__);
	
	return ret;
#else
	int i;
   
	
	VIVO_TS_LOG_ERR("[%s]:module id is %x\n", __func__, module_id);

	for (i = 0; i < ARRAY_SIZE(fw_data_map_gpio); i++) {
		if (fw_data_map_gpio[i].module_id == module_id) {
			if (!fw_data_map_gpio[i].firmware_data) {
				return FIRMWARE_DATA_DEFAULT;
			}else{
				return fw_data_map_gpio[i].firmware_data;
			}
		}
	}

	VIVO_TS_LOG_INF("[%s]:There is no firmware data map the module id use default\n", __func__);
	return FIRMWARE_DATA_DEFAULT;
#endif
}


static u8 *get_host_firmware_data_with_lcd_id(int lcd_id)
{
    int i;
	u8 *ret = NULL;
	

	VIVO_TS_LOG_INF("[%s]:lcd id is %x\n", __func__, lcd_id);
	
    if(curr_product_lid_fw_disc != NULL)
	{
		for (i=0; i<curr_product_lid_fw_disc->fw_cnt; i++) {
	         if(curr_product_lid_fw_disc->data_maps[i].lcd_id == lcd_id) {
			        return curr_product_lid_fw_disc->data_maps[i].firmware_data;		
			 }
	    }
		
		ret = curr_product_lid_fw_disc->default_firmware_data;
    } 
	
	VIVO_TS_LOG_INF("[%s]:There is no firmware data map the lcd id use default \n",__func__);
	
	return ret;

}
/* won't return NULL */
static u8 *get_host_firmware_data_with_product_id(struct reflash_data *data)
{	
#if 1
    int i;
	u8 *ret = NULL;
	u8 *module_flag = data->product_id ;
	
    if(curr_product_pid_fw_disc != NULL)
	{
	    for (i=0; i<curr_product_pid_fw_disc->fw_cnt; i++) {
	         if(strncmp(curr_product_pid_fw_disc->data_maps[i].product_id, module_flag, 8) == 0) {
			    if(syna3202_get_lcd_id() == curr_product_pid_fw_disc->data_maps[i].lcd_id)
			        return curr_product_pid_fw_disc->data_maps[i].firmware_data;
			 }
	    }
	    
		ret = curr_product_pid_fw_disc->default_firmware_data;
	}

    VIVO_TS_LOG_INF("[%s]:There is no firmware data map the pid id use default module_flag=%s\n",__func__,module_flag);
	
	return ret;    
#else
    int i;
	u8 *module_flag = data->product_id ;
	
	for (i = 0; i < ARRAY_SIZE(fw_data_map_with_pd_id); i++) {
		if (strncmp(fw_data_map_with_pd_id[i].product_id, module_flag, 8) == 0) {
			if (!fw_data_map_with_pd_id[i].firmware_data) {
				return FIRMWARE_DATA_DEFAULT;
			}else {
				return fw_data_map_with_pd_id[i].firmware_data;
			}
		}
	}
	
	VIVO_TS_LOG_INF("[%s]:There is no firmware data map the product id use default\n", __func__);
	
	return FIRMWARE_DATA_DEFAULT;

#endif
}

#if 0
bool rmi_get_lens_module(void)
{

	return is_lens_changed;
}
#endif
/*
#if defined(PD1401F)||defined(PD1401F_EX)||defined(PD1401V) 

bool rmi_get_module(void)
{
	return is_try_module;
}
#endif
*/

static int get_host_version_and_data(int lcd_id,int hardware_id, struct reflash_data *data)
{
    int retval = -1;
	
	VIVO_TS_LOG_INF("[%s]:\n", __func__);
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_ICID) {
		FIRMWARE_DATA = get_host_firmware_data_with_product_id(data);		
	}
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_GPIO) {
		FIRMWARE_DATA = get_host_firmware_data_with_gpio_id(hardware_id);	
	}

	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_LCDID) {
			FIRMWARE_DATA = get_host_firmware_data_with_lcd_id(lcd_id);	
	}	
	
	if(FIRMWARE_DATA != NULL)
	    retval = FIRMWARE_DATA[0xb103];
	
	return retval;
}

static int get_host_module_id(void) 
{
	return FIRMWARE_DATA[0xb101];
}

int rmi_get_version(struct rmi_device *dev, u16 f34_pdt_control_base_addr)
{
	int retval;
	struct f34_userdata version_info;

	retval = rmi_read_block(dev, f34_pdt_control_base_addr,
			(u8 *)&version_info, 4);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read version info failed\n", __func__);
		return -1;
	}

	return version_info.version;
}

int rmi_get_module_id(struct rmi_device *dev, u16 f34_pdt_control_base_addr)
{
	int retval;
	struct f34_userdata version_info;

	retval = rmi_read_block(dev, f34_pdt_control_base_addr,
			(u8 *)&version_info, 4);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read version info failed\n", __func__);
		return -1;
	}

	return version_info.module_id;
}
#if 0
#define OLD_MODULE_POINT_VAL_LIMIT   700
static bool need_delay_update_fw = false; /* for f54 register delay */

bool get_is_update_need_redo(void) {
	return need_delay_update_fw;
}
static char* get_rawdata(struct rmi_driver_data *ddata, int *tx, int *rx)
{
	struct rmi_function_container *fc54, *fc11;
	int error;
	char *data;

	fc54 = rmi_get_function_container(ddata, 0x54);
	if (!fc54) {
		VIVO_TS_LOG_ERR("[%s]:Get fc54 failed\n", __func__);
		return NULL;
	}

	fc11 = rmi_get_function_container(ddata, 0x11);
	if (!fc11) {
		VIVO_TS_LOG_ERR("[%s]:Get fc11 failed\n", __func__);
		return NULL;
	}

	error = rmi_f11_get_sensor_electrodes(fc11, rx, tx);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get sensor elsectrodes failed\n", __func__);
		return NULL;
	}

	error = rmi_dev_get_fn54_data(fc54, &data, 3);
	if (error < 0) {
		VIVO_TS_LOG_ERR("[%s]:Get baseline data failed\n", __func__);
		return NULL;
	}

#if 0
	/* all 1227L is 2D sensor */
	if (ddata->is_button_independent) 
		*rx++; /* 0D sensor use one rx*/
#endif
		
		
	return data;
}

static bool is_module_old(struct rmi_driver_data *ddata)
{
	char *data;
	int tx, rx;
	int couner_p1_val, couner_p2_val;

	data = get_rawdata(ddata, &tx, &rx);
	if(!data) {
		VIVO_TS_LOG_ERR("[%s]:Get rawdata failed\n", __func__);
		/* return false for making sure the new modules' fw will be update 
			 if there's a i2c's issue make the rawdata get failed
		*/
		return false;
	}
	
	couner_p1_val = (u16)(data[(rx -1) * 2 + 1]) << 8 | (u16)data[(rx - 1) * 2];
	couner_p2_val = (u16)((data[(tx - 1) * rx * 2 + (rx - 1) * 2 + 1]) << 8 | (u16)data[(tx - 1) * rx * 2 + (rx - 1) * 2]);
	
	if ((couner_p1_val < OLD_MODULE_POINT_VAL_LIMIT ||couner_p1_val > 5000)
			&&( couner_p2_val < OLD_MODULE_POINT_VAL_LIMIT || couner_p2_val > 5000) ) {
		VIVO_TS_LOG_ERR("[%s]:The bie module is old couner_p1_val(%d) couner_p2_val(%d)\n", __func__,
								couner_p1_val, couner_p2_val);
		return true;
	} else {
		VIVO_TS_LOG_ERR("[%s]:The bie module is new couner_p1_val(%d) couner_p2_val(%d)\n", __func__,
								couner_p1_val, couner_p2_val);
		return false;
	}
	
}

static bool judge_version_info(struct rmi_driver_data *ddata, int ic_version, int need_up_version)
{
		bool is_old = false;

			VIVO_TS_LOG_INF("[%s]: need_delay_update_fw is %d\n", __func__, need_delay_update_fw);
		if (ic_version > 0x08) {
			/* the old module should not has the version larger than 0x08 */
			VIVO_TS_LOG_ERR("[%s]:The ic's version is larger than 0x08\n", __func__);
			if (ic_version != need_up_version) {
				VIVO_TS_LOG_ERR("[%s]:host firmware version is not equal the ICs version need to upgrade\n", __func__);
				return true;
			} else {
				VIVO_TS_LOG_INF("[%s]:host firmware version is the same as the ICs No need to upgrade\n", __func__);
				return false;
			}
		} else {
			/* ic's version is equal or less than 0x08 need to update delay */
			if (!need_delay_update_fw) {
				need_delay_update_fw = true;
				return false;
			} else {
				VIVO_TS_LOG_INF("[%s]:need_delay_update_fw is true so gono check\n", __func__);
			}
			
			/* ic version less than 0x08 the old module judge method is reliable */
			is_old = is_module_old(ddata);
			
			if (ic_version == 0x08) {
				if (is_old) {
					FIRMWARE_DATA = BIE1GFFW_OLD_VERSION;
					VIVO_TS_LOG_ERR("[%s]:Bie old module & version is 0x08 so need to update fw\n", __func__);
					return true;
				} else {
					if (ic_version != need_up_version) {
						VIVO_TS_LOG_INF("[%s]:Bie new module host version is not equal the ic's version(0x08) need to update fw\n", __func__);
						return true;
					} else {
						VIVO_TS_LOG_INF("[%s]:Bie new module & version is 0x08 so don't need to update fw\n", __func__);
						return false;
					}
				}
			} else {
				/* ic's version less than 0x08 */
				/* maybe some back check make the new module update the old fw version */
				VIVO_TS_LOG_INF("[%s]:the ICs version is less then 0x08\n", __func__);
				if (is_old) {
					VIVO_TS_LOG_INF("[%s]:the module is old\n", __func__);
					if (need_up_version < 0x08) {
						if (ic_version != need_up_version) {
							VIVO_TS_LOG_INF("[%s]:host firmware version is less than 0x08 not equal the ICs version need to upgrade\n", __func__);
							return true;
						}  else {
							VIVO_TS_LOG_INF("[%s]:host firmware version is less than 0x08 & equal the ICs version no need to upgrade\n", __func__);
							return false;
						}
					} else if (need_up_version == 0x08){
							if (ic_version != 0x07) {
								VIVO_TS_LOG_INF("[%s]:host firmware version is 0x08 & need up 0x07 fw\n", __func__);
								FIRMWARE_DATA = BIE1GFFW_OLD_VERSION;
								return true;
							} else {
								VIVO_TS_LOG_INF("[%s]:host firmware version is 0x08 & ic's version 0x07 no nedd to update fw\n", __func__);
								return false;
							}
					} else {
						VIVO_TS_LOG_INF("[%s]:host firmware version is larger than 0x08 no need to upgrade\n", __func__);
						return false;
					}
				} else {
					if (ic_version != need_up_version) {
						VIVO_TS_LOG_INF("[%s]:New module, host firmware version is not equal to the ICs version need to upgrade\n", __func__);
						return true;
					} else {
						VIVO_TS_LOG_INF("[%s]:New module, host firmware version is equal to the ICs version no need to upgrade\n", __func__);
						return false;
					}
				}
			}
		}

		return false;
}
#endif
/* Returns false if the firmware should not be reflashed.
 */
 
static bool go_nogo(struct reflash_data *data)
{
	int retval;
	struct f34_userdata version_info;
	int host_version; 
	int image_to_up_id;
	int hardware_id = -1;
	int lcd_id = -1;

	struct rmi_driver_data *ddata = rmi_get_driverdata(data->rmi_dev);
	
	retval = rmi_read_block(data->rmi_dev, data->f34_pdt->control_base_addr,
					(u8 *)&version_info, 4);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:read version info failed\n",__func__);
		return true;
	}

	VIVO_TS_LOG_ERR("[%s]:reserve1 = 0x%02x\n", __func__, version_info.reserve1);
	VIVO_TS_LOG_ERR("[%s]:module_id = 0x%02x\n", __func__, version_info.module_id);
	VIVO_TS_LOG_ERR("[%s]:reserve2 = 0x%02x\n", __func__, version_info.color);
	VIVO_TS_LOG_ERR("[%s]:version = 0x%02x\n", __func__, version_info.version);
	
	if(initial_set_fw_data() < 0) {
	    VIVO_TS_LOG_ERR("[%s]: initial_set_fw_data fail return!!\n",__func__);
	    return false;
	}
	
	VIVO_TS_LOG_INF("[%s]: initial_set_fw_data succeed return!!\n",__func__);
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_GPIO) {
	    hardware_id = hardware_id_check();    
	    if(-1 == hardware_id) {
		    VIVO_TS_LOG_ERR("[%s]: invalid hardware_id with gpio upgrade!!\n",__func__);
		    return false;
		}
	} 
	
	VIVO_TS_LOG_INF("[%s]: %d\n",__func__,hardware_id);
	
	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_LCDID) {
	    lcd_id = syna3202_get_lcd_id();    
	} 

	host_version = get_host_version_and_data(lcd_id,hardware_id, data);
	VIVO_TS_LOG_ERR("[%s]: get_host_version_and_data  return %d!!\n",__func__,host_version);
	if(host_version < 0) {
	     VIVO_TS_LOG_ERR("[%s]:get_host_version_and_data failed!!!\n",__func__);
		 return false;
	}
	
	image_to_up_id = get_host_module_id(); 

	VIVO_TS_LOG_ERR("[%s]:host_version is 0x%02x\n", __func__, host_version);
	VIVO_TS_LOG_ERR("[%s]:the id of image which will be updated is 0x%02x\n", __func__,
			image_to_up_id);
	/* 
	   if hardware_id == -1 that means no gpio is used to check the hardware module
	   so only check the firmware version
	   use image_to_up_id not hardware id for may default image will be updated many
	   times when there is no sensor on PCB
	*/
	
	if(ddata->probe_over_flag)  
		return true;	
		
	if (hardware_id == -1 || image_to_up_id ==  version_info.module_id) {
		if (version_info.version != host_version) {
			VIVO_TS_LOG_INF("[%s]:host firmware version is not equal the ICs version need to upgrade\n", __func__);
			return true;
		}else{
			VIVO_TS_LOG_ERR("[%s]:host firmware version is the same as the ICs No need to upgrade\n", __func__);
			return false;
		}
	}else{
		VIVO_TS_LOG_INF("[%s]:Ic's firware module is not equal to hardware check\n", __func__);
		return true;
	}
}

void get_firmware_data(u8 **data, char name[], struct device *dev)
{
	int retval;
	const struct firmware *fw_entry = NULL;

	if (!name) {
		*data = FIRMWARE_DATA;
	} else {
		retval = request_firmware(&fw_entry, name, dev);
		if (retval != 0) {
			VIVO_TS_LOG_ERR("[%s]:Firmware %s not available, code = %d\n", __func__,
					name, retval);
			*data = NULL;
			return;
		}
		*data = (u8 *)fw_entry->data;
	}
}

int rmi4_fw_update(struct rmi_device *rmi_dev,
		struct pdt_entry *f01_pdt, struct pdt_entry *f34_pdt, char name[])
{
#ifdef DEBUG
	struct timespec start;
	struct timespec end;
	s64 duration_ns;
#endif
	/* wyl modifed not to use system firmware to update
	   for request_firmware return failed when filesystem mount
	   */
	//char firmware_name[RMI_PRODUCT_ID_LENGTH + 12];
	//const struct firmware *fw_entry = NULL;
	//unsigned char *fw_data = NULL;
	u8 *fw_data = NULL;
	int retval;
	struct image_header header;
	union pdt_properties pdt_props;
	struct reflash_data data = {
		.rmi_dev = rmi_dev,
		.f01_pdt = f01_pdt,
		.f34_pdt = f34_pdt,
	};
	/*add by qiuguifu,if ic in the bootloader mode, need to update the firmware.*/
	union f01_device_status device_status;
	
	retval = rmi_read(rmi_dev, f01_pdt->data_base_addr,
			  device_status.regs);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read device status.\n", __func__);
		return retval;
	}
	if (device_status.flash_prog){
		VIVO_TS_LOG_ERR("[%s]:WARNING: RMI4 device is in bootloader mode!----QQQQQ\n", __func__);
	}
	
	VIVO_TS_LOG_INF("[%s]:called.\n", __func__);
#ifdef	DEBUG
	getnstimeofday(&start);
#endif

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, pdt_props.regs);
	if (retval < 0) {
		VIVO_TS_LOG_ERR("[%s]:Failed to read PDT props at %#06x (code %d). Assuming 0x00.\n", __func__,
			 PDT_PROPERTIES_LOCATION, retval);
	}
	if (pdt_props.has_bsr) {
		VIVO_TS_LOG_ERR("[%s]:Firmware update for LTS not currently supported.\n", __func__);
		return -1;
	}

	retval = read_f01_queries(&data);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:F01 queries failed, code = %d.\n", __func__,
			retval);
		return -1;
	}
	retval = read_f34_queries(&data);
	if (retval) {
		VIVO_TS_LOG_ERR("[%s]:F34 queries failed, code = %d.\n", __func__,
			retval);
		return -1;
	}
#if 0
	snprintf(firmware_name, sizeof(firmware_name), "%s.img",
			img_name ? img_name : data.product_id);
	VIVO_TS_LOG_INF("[%s]:Requesting %s.\n", __func__, firmware_name);
	/* wyl delete */
	retval = request_firmware(&fw_entry, "test.img", &rmi_dev->dev);
	if (retval != 0) {
		VIVO_TS_LOG_ERR("[%s]:Firmware %s not available, code = %d\n", __func__,
			firmware_name, retval);
		return;
	}
#endif

	/* make the go_nogo front for firmware data is assigned in it */
	if (!name) {
		if (!(go_nogo(&data) || device_status.flash_prog)) {
			VIVO_TS_LOG_ERR("[%s]:Go/NoGo said don't reflash.and IC is not in bootloader mode.\n", __func__);
			return 0;
		}
	}

	get_firmware_data(&fw_data, name, &rmi_dev->dev);
	if (!fw_data) {
		VIVO_TS_LOG_ERR("[%s]:Get firmware data failed\n", __func__);
		return -1;
	}
	
	VIVO_TS_LOG_INF("[%s]: %p %p\n",__func__,fw_data,PD1502F_EX_YFT_GFFW_TRY);

	extract_header(fw_data, 0, &header);

#ifdef	DEBUG
	//dev_info(&rmi_dev->dev, "Got firmware, size: %d.\n", fw_entry->size);
	VIVO_TS_LOG_ERR("[%s]:Img checksum:           %#08X\n", __func__,
		 header.checksum);
	VIVO_TS_LOG_ERR("[%s]:Img image size:         %d\n", __func__,
		 header.image_size);
	VIVO_TS_LOG_ERR("[%s]:Img config size:        %d\n", __func__,
		 header.config_size);
	VIVO_TS_LOG_ERR("[%s]:Img bootloader version: %d\n", __func__,
		 header.bootloader_version);
	VIVO_TS_LOG_ERR("[%s]:Img product id:         %s\n", __func__,
		 header.product_id);
	VIVO_TS_LOG_ERR("[%s]:Img product info:       %#04x %#04x\n", __func__,
		 header.product_info[0], header.product_info[1]);
#endif

	if (header.image_size)
		data.firmware_data = fw_data + F34_FW_IMAGE_OFFSET;
	if (header.config_size)
		data.config_data = fw_data + F34_FW_IMAGE_OFFSET +
			header.image_size;

#if 0
	if (!name) {
		if (go_nogo(&data, &header)) {
			reflash_firmware(&data);
			msleep(100);
			reset_device(&data);
		} else
			VIVO_TS_LOG_INF("[%s]:Go/NoGo said don't reflash.\n", __func__);
	} else {
#endif
		reflash_firmware(&data);
		reset_device(&data);
	//}


#if 0
	if (fw_entry)
		release_firmware(fw_entry);
#endif
#ifdef	DEBUG
	getnstimeofday(&end);
	duration_ns = timespec_to_ns(&end) - timespec_to_ns(&start);
	VIVO_TS_LOG_INF("[%s]:Time to reflash: %lld ns.\n", __func__, duration_ns);
#endif

	return 0;
}
