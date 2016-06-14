#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/HWVersion.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "cadiz.h"
#include <linux/pm_qos.h>

extern int Read_HW_ID(void);

static struct i2c_client *cadiz_client;
int cadiz_ldo_enable;
int cadiz_rst;
struct clk *s_clk;
int cadiz_12;
struct regulator *vdd;
struct regulator *tcon_vdd;
static struct miscdevice cadiz_dev;
static struct dentry *cadiz_debugfs;
static struct pm_qos_request tput_pm_qos_req;
static bool cadiz_ready = false;

typedef struct {
	u16 addr;
	u8 param;
}initcode;

initcode cadiz_boot1[]= {
	{0x0830,0x00},
	{0x0200,0x00},
	{0x0201,0x00},
	{0x0202,0x00},
	{0x0203,0x00},
	{0x0274,0x00},
	{0x0275,0x00},
	{0x0276,0x00},
	{0x0277,0x00},
	{0x0600,0x00},
	{0x0601,0x00},
	{0x0602,0x00},
	{0x0603,0x00},
	{0x0820,0x32},
	{0x0821,0x04},
	{0x0822,0x33},
	{0x0823,0x33},
	{0x0824,0x33},
	{0x0825,0x03},
	{0x0826,0x00},
	{0x0827,0x00},
	{0x0840,0x40},
	{0x0841,0x7B},
	{0x0842,0x14},
	{0x0843,0x00},
	{0x0900,0x06},
	{0x0901,0x01},
	{0x0902,0x20},
	{0x0903,0x03},
	{0x0904,0x00},
	{0x0905,0x05},
	{0x0906,0x68},
	{0x0907,0x00},
	{0x0908,0x00},
	{0x0909,0x00},
	{0x090A,0x7C},
	{0x090B,0x1C},
	{0x090C,0x00},
	{0x090D,0x00},
	{0x090E,0x00},
	{0x090F,0x00},
	{0x0914,0x01},
	{0x0915,0x10},
	{0x0916,0xD8},
	{0x0917,0x07},
	{0x0940,0x05},
	{0x0941,0x00},
	{0x0942,0x06},
	{0x0943,0x00},
	{0x020C,0x00},
	{0x020D,0x62},
	{0x020E,0x00},
	{0x020F,0x00},
	{0x0220,0x20},
	{0x0221,0x03},
	{0x0222,0x00},
	{0x0223,0x05},
	{0x0228,0x12},
	{0x0229,0x00},
	{0x022A,0x00},
	{0x022B,0x00},
	{0x022C,0x3C},
	{0x022D,0x00},
	{0x022E,0x00},
	{0x022F,0x00},
	{0x0230,0x4B},
	{0x0231,0x00},
	{0x0232,0x00},
	{0x0233,0x00},
	{0x0234,0x58},
	{0x0235,0x02},
	{0x0236,0x00},
	{0x0237,0x00},
	{0x0238,0x0A},
	{0x0239,0x00},
	{0x023A,0x00},
	{0x023B,0x00},
	{0x023C,0x18},
	{0x023D,0x00},
	{0x023E,0x00},
	{0x023F,0x00},
	{0x0240,0x18},
	{0x0241,0x00},
	{0x0242,0x00},
	{0x0243,0x00},
	{0x0244,0x1C},
	{0x0245,0x00},
	{0x0246,0x00},
	{0x0247,0x00},
	{0x0258,0x02},
	{0x0259,0x00},
	{0x025A,0x00},
	{0x025B,0x00},
	{0x025C,0x07},	//dis-continue mode
	{0x025D,0x00},
	{0x025E,0x00},
	{0x025F,0x00},
	{0x0260,0x0C},
	{0x0261,0x00},
	{0x0262,0x00},
	{0x0263,0x00},
	{0x0264,0x12},
	{0x0265,0x00},
	{0x0266,0x27},
	{0x0267,0x00},
	{0x0268,0x03},
	{0x0269,0x00},
	{0x026A,0x00},
	{0x026B,0x00},
	{0x026C,0x00},
	{0x026D,0x0C},
	{0x026E,0x05},
	{0x026F,0x0A},
	{0x0270,0x00},
	{0x0271,0x16},
	{0x0272,0x05},
	{0x0273,0x0A},
	{0x0278,0x12},	//Modify MIPI rising/falling timing
	{0x0279,0x04},
	{0x027A,0x84},
	{0x027B,0x3B},
	{0x027C,0x08},
	{0x027D,0x38},
	{0x027E,0x32},
	{0x027F,0xB5},
	{0x0280,0x00},
	{0x0281,0x00},
	{0x0282,0x40},
	{0x0283,0x10},
	{0x0284,0x00},
	{0x0285,0x40},
	{0x0286,0x44},
	{0x0287,0x02},
	{0x02A8,0x01},
	{0x02A9,0x00},
	{0x02AA,0x00},
	{0x02AB,0x00},
	{0x000C,0x60},
	{0x000D,0x00},
	{0x000E,0x00},
	{0x000F,0x00},
	{0x0024,0x01},
	{0x0025,0x00},
	{0x0026,0x00},
	{0x0027,0x00},
	{0x002C,0x06},
	{0x002D,0xFF},
	{0x002E,0x00},
	{0x002F,0x00},
	{0x0034,0x1A},
	{0x0035,0x84},
	{0x0036,0x04},
	{0x0037,0x3A},
	{0x0038,0x08},
	{0x0039,0x38},
	{0x003A,0x32},
	{0x003B,0xB5},
	{0x003C,0x1C},
	{0x003D,0x04},
	{0x003E,0x00},
	{0x003F,0x00},
	{0x0040,0x00},
	{0x0041,0x00},
	{0x0042,0x44},
	{0x0043,0x02},
	{0x0058,0x18},
	{0x0059,0x00},
	{0x005A,0x0A},
	{0x005B,0x00},
	{0x090C,0x00},
	{0x090D,0x00},
	{0x090E,0xF8},
	{0x090F,0x07}
};

initcode cadiz_ipc_ibc[]={
	{0x0C14,0x00},
	{0x0C15,0x00},
	{0x0C16,0x00},
	{0x0C17,0x02},
	{0x0C20,0x00},
	{0x0C21,0x02},
	{0x0C22,0x00},
	{0x0C23,0x01},
	{0x0C34,0x10},
	{0x0C35,0x82},
	{0x0C36,0x05},
	{0x0C37,0x51},
	{0x0C28,0x00},	//screen flick
	{0x0C29,0x00},
	{0x0C2A,0x01},
	{0x0C2B,0x00},
	{0x0C2C,0x54},
	{0x0C2D,0x05},
	{0x0C2E,0x00},
	{0x0C2F,0x00},
	{0x0C30,0x00},
	{0x0C31,0x00},
	{0x0C32,0x50},
	{0x0C33,0x15},
	{0x0A00,0x80},
	{0x0A01,0x00},
	{0x0A02,0x04},
	{0x0A03,0x00},
	{0x0A04,0x20},
	{0x0A05,0x03},
	{0x0A06,0x00},
	{0x0A07,0x00},
	{0x0A08,0x00},
	{0x0A09,0x05},
	{0x0A0A,0x00},
	{0x0A0B,0x00},
	{0x0A0C,0x00},
	{0x0A0D,0x00},
	{0x0A0E,0x00},
	{0x0A0F,0x00},
	{0x0A10,0x00},
	{0x0A11,0x00},
	{0x0A12,0x00},
	{0x0A13,0x88},
	{0x0A14,0x01},
	{0x0A15,0x00},
	{0x0A16,0x00},
	{0x0A17,0x00},
	{0x0A18,0x80},
	{0x0A19,0x80},
	{0x0A1A,0x80},
	{0x0A1B,0x80},
	{0x0A1C,0x00},
	{0x0A1D,0x00},
	{0x0A1E,0x02},
	{0x0A1F,0x08},
	{0x0A20,0x0A},
	{0x0A21,0x06},
	{0x0A22,0x1F},
	{0x0A23,0x3F},
	{0x0A24,0x16},
	{0x0A25,0x16},
	{0x0A26,0x04},
	{0x0A27,0x04},
	{0x0A28,0x0A},
	{0x0A29,0x00},
	{0x0A2A,0x04},
	{0x0A2B,0x04},
	{0x0A2C,0x04},
	{0x0A2D,0x04},
	{0x0A2E,0x02},
	{0x0A2F,0x04},
	{0x0A30,0x04},
	{0x0A31,0x01},
	{0x0A32,0x30},
	{0x0A33,0xFF},
	{0x0A34,0x80},
	{0x0A35,0x30},
	{0x0A36,0x37},
	{0x0A37,0xFF},
	{0x0A38,0x00},
	{0x0A39,0x00},
	{0x0A3A,0x68},
	{0x0A3B,0x00},
	{0x0A3C,0x58},
	{0x0A3D,0x02},
	{0x0A3E,0xA0},
	{0x0A3F,0x00},
	{0x0A40,0xC0},
	{0x0A41,0x03},
	{0x0A42,0x06},
	{0x0A43,0x00},
	{0x0A44,0x85},
	{0x0A45,0x00},
	{0x0A46,0x02},
	{0x0A47,0x00},
	{0x0A48,0xD5},
	{0x0A49,0x00},
	{0x0A4A,0x3A},
	{0x0A4B,0x00},
	{0x0A4C,0x93},
	{0x0A4D,0x04},
	{0x0A4E,0xEC},
	{0x0A4F,0x01},
	{0x0A50,0x33},
	{0x0A51,0x01},
	{0x0A52,0x78},
	{0x0A53,0x68},
	{0x0A54,0x2C},
	{0x0A55,0x71},
	{0x0A56,0x84},
	{0x0A57,0x03},
	{0x0A58,0x24},
	{0x0A59,0x00},
	{0x0A5A,0x00},
	{0x0A5B,0x67},
	{0x0A5C,0xC0},
	{0x0A5D,0x88},
	{0x0A5E,0xB6},
	{0x0A5F,0xAF},
	{0x0A60,0x00},
	{0x0A61,0xFF},
	{0x0A62,0xF0},
	{0x0A63,0x88},
	{0x0A64,0x22},
	{0x0A65,0x3A},
	{0x0A66,0xD6},
	{0x0A67,0xA7},
	{0x0A68,0x60},
	{0x0A69,0xC5},
	{0x0A6A,0xB0},
	{0x0A6B,0x20},
	{0x0A6C,0xCC},
	{0x0A6D,0x8A},
	{0x0A6E,0x8A},
	{0x0A6F,0x83},
	{0x0A70,0x84},
	{0x0A71,0x86},
	{0x0A72,0x89},
	{0x0A73,0x83},
	{0x0A74,0x91},
	{0x0A75,0x8B},
	{0x0A76,0x01},
	{0x0A77,0x00},
	{0x0A80,0x00},
	{0x0A81,0x00},
	{0x0A82,0x04},
	{0x0A83,0x00},
	{0x0A84,0x20},
	{0x0A85,0x03},
	{0x0A86,0x04},
	{0x0A87,0x00},
	{0x0A88,0x00},
	{0x0A89,0x05},
	{0x0A8A,0x60},
	{0x0A8B,0x00},
	{0x0A8C,0x65},
	{0x0A8D,0x04},
	{0x0A8E,0x42},
	{0x0A8F,0x00},
	{0x0A90,0x09},
	{0x0A91,0x3D},
	{0x0A92,0x00},
	{0x0A93,0x00},
	{0x0A94,0x20},
	{0x0A95,0x00},
	{0x0A96,0x64},
	{0x0A97,0x00},
	{0x0A98,0xA0},
	{0x0A99,0x00},
	{0x0A9A,0x00},
	{0x0A9B,0x10},
	{0x0A9C,0x00},
	{0x0A9D,0x00},
	{0x0A9E,0x01},
	{0x0A9F,0x00},
	{0x0AA0,0xF5},
	{0x0AA1,0x04},
	{0x0AA2,0x08},
	{0x0AA3,0x00},
	{0x0AA8,0x00},
	{0x0AA9,0x00},
	{0x0AAA,0x78},
	{0x0AAB,0x00},
	{0x0AAC,0x30},
	{0x0AAD,0x02},
	{0x0AAE,0x26},
	{0x0AAF,0x00},
	{0x0AB0,0x80},
	{0x0AB1,0x04},
	{0x0AB2,0x00},
	{0x0AB3,0x00},
	{0x0AB4,0x00},
	{0x0AB5,0x2A},
	{0x0AB6,0x00},
	{0x0AB7,0x00},
	{0x0AB8,0x21},
	{0x0AB9,0x3F},
	{0x0ABA,0x00},
	{0x0ABB,0x01},
	{0x0ABC,0x0F},
	{0x0ABD,0x80},
	{0x0ABE,0xB8},
	{0x0ABF,0x1F},
	{0x0AC0,0xC0},
	{0x0AC1,0x64},
	{0x0AC2,0x02},
	{0x0AC3,0x80},
	{0x0AC4,0x26},
	{0x0AC5,0x00},
	{0x0AC6,0x88},
	{0x0AC7,0x20},
	{0x0AC8,0x00},
	{0x0AC9,0x40},
	{0x0ACA,0x00},
	{0x0ACB,0x3F},
	{0x0ACC,0x00},
	{0x0ACD,0x43},
	{0x0ACE,0x00},
	{0x0ACF,0x4A},
	{0x0AD0,0x20},
	{0x0AD1,0x33},
	{0x0AD2,0x7F},
	{0x0AD3,0x96},
	{0x0AD4,0x06},
	{0x0AD5,0x0F},
	{0x0AD6,0x21},
	{0x0AD7,0x28},
	{0x0AD8,0x2C},
	{0x0AD9,0x7F},
	{0x0ADA,0xC8},
	{0x0ADB,0x96},
	{0x0ADC,0x0F},
	{0x0ADD,0x19},
	{0x0ADE,0x28},
	{0x0ADF,0x2C},
	{0x0AE0,0x00},
	{0x0AE1,0x00},
	{0x0AE2,0x00},
	{0x0AE3,0x00},
	{0x0AFC,0x00},
	{0x0AFD,0x00},
	{0x0AFE,0x00},
	{0x0AFF,0x08}
};

initcode cadiz_boot2[]={
	{0x001C,0x05},
	{0x001D,0x00},
	{0x001E,0x00},
	{0x001F,0x00},
	{0x0000,0x01},
	{0x0001,0x00},
	{0x0002,0x00},
	{0x0003,0x00},
	{0x0940,0x05},
	{0x0941,0x06},
	{0x0942,0x06},
	{0x0943,0x00}
};

initcode cadiz_boot3[]={
	{0x0274,0x01},
	{0x0200,0x01},
	{0x0006,0xFF}
};


initcode cadiz_boot4[]={
	{0x0000,0x01},
	{0x0941,0x07},
	{0x0941,0x01},
	{0x0005,0xFF},
	{0x0C0B,0xFF},
	{0x0C0A,0xB6}
};

int cadiz_boot1_len = 0;
int cadiz_ipc_ibc_len = 0;

static u8 boot1_0x0902;
static u8 boot1_0x0903;
static u8 boot1_0x0904;
static u8 boot1_0x0905;
static u8 boot1_0x0916;
static u8 boot1_0x0917;
#define CADIZ_BYPASS_SIZE 3
static u16 cadiz_bypass[CADIZ_BYPASS_SIZE];

int cadiz_get_init_boot1(void)
{
	int i, check_sum;
	int arraysize= sizeof(cadiz_boot1)/sizeof(cadiz_boot1[0]);
	u16 htotal = 0;
	printk("[DISPLAY] %s: Enter\n",__func__);

	check_sum = 0;
	for(i = 0; i < arraysize; i++) {
		switch (cadiz_boot1[i].addr) {
			case 0x0902:
				boot1_0x0902 = cadiz_boot1[i].param;
				check_sum++;
				break;
			case 0x0903:
				boot1_0x0903 = cadiz_boot1[i].param;
				check_sum++;
				break;
			case 0x0904:
				boot1_0x0904 = cadiz_boot1[i].param;
				check_sum++;
				break;
			case 0x0905:
				boot1_0x0905 = cadiz_boot1[i].param;
				check_sum++;
				break;
			case 0x0916:
				boot1_0x0916 = cadiz_boot1[i].param;
				htotal |= cadiz_boot1[i].param;
				check_sum++;
				break;
			case 0x0917:
				boot1_0x0917 = cadiz_boot1[i].param;
				htotal |= cadiz_boot1[i].param<<8;
				check_sum++;
				break;
		}
		if (check_sum == 6)
			break;
	}

	cadiz_bypass[0] = 0x0;
	cadiz_bypass[1] = 7 * (htotal/2) + 247;
	cadiz_bypass[2] = 7 * (htotal/2) + 188;

	printk("[DISPLAY] cadiz_bypass= 0x%x, 0x%x, 0x%x\n", cadiz_bypass[0], cadiz_bypass[1], cadiz_bypass[2]);

	printk("[DISPLAY] %s: End\n",__func__);
	return 0;
}

int cadiz_i2c_reg_read(u16 reg, u8 *value)
{
	int r;
	struct i2c_client *client = cadiz_client;
	u8 tx_data[] = {
		reg >> 8 ,	//SubAddress(MSB) 8bit
		reg & 0xff,	//SubAddress(LSB) 8bit
	};

	u8 rx_data[1]={0};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

#if 1
	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	printk("%s: 0x%04xh = 0x%02x (r = %d)\n", __func__, reg, rx_data[0], r);

	*value = rx_data[0];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);
#endif
	return r;
}

int cadiz_i2c_reg_write(u16 reg, u8 value)
{
	int r;
	struct i2c_client *client = cadiz_client;
	u8 tx_data[] = {
		reg >> 8 ,	//SubAddress(MSB) 8bit
		reg & 0xff,	//SubAddress(LSB) 8bit
		value & 0xff,	//Data 8bit
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

#if 1
	pm_qos_update_request(&tput_pm_qos_req, 0);

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	pm_qos_update_request(&tput_pm_qos_req, PM_QOS_DEFAULT_VALUE);
	//printk("%s: 0x%04x, 0x%02x ===> 0x%02x, 0x%02x, 0x%02x\n", __func__, reg, value, tx_data[0], tx_data[1], tx_data[2]);
#endif
	return 0;
}

EXPORT_SYMBOL(cadiz_i2c_reg_write);

int cadiz_i2c_seq_write(initcode boot[], int arraysize)
{
	int r, i, count = 0;
	struct i2c_client *client = cadiz_client;
	u16 next_reg = 0;
	u8 tx_data[200] = {0};
	struct i2c_msg msgs = {
			.addr = client->addr,
			.flags = 0,
			.buf = 0,
			.len = 0,
	};

	memset(tx_data, 0x0, sizeof(tx_data));

	pm_qos_update_request(&tput_pm_qos_req, 0);

	for(i = 0; i < arraysize; i++) {
		if((next_reg != boot[i].addr) || count == 50) {
			if(count > 0) {
				msgs.buf = tx_data;
				msgs.len = count;
				r = i2c_transfer(client->adapter, &msgs, 1);
				if (r < 0) {
					dev_err(&client->dev, "%s: reg 0x%02x%02x error %d\n",
						__func__, tx_data[0], tx_data[1], r);
					return r;
				}
				memset(tx_data, 0x0, sizeof(tx_data));
			}

			tx_data[0] = boot[i].addr >> 8;
			tx_data[1] = boot[i].addr & 0xff;
			count = 2;
		}

		next_reg = boot[i].addr + 1;
		tx_data[count++] = boot[i].param & 0xff;
	}

	msgs.buf = tx_data;
	msgs.len = count;
	r = i2c_transfer(client->adapter, &msgs, 1);
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%02x%02x error %d\n",
			__func__, tx_data[0], tx_data[1], r);
		return r;
	}

	pm_qos_update_request(&tput_pm_qos_req, PM_QOS_DEFAULT_VALUE);

	return 0;
}

int cadiz_init_boot1(void){
	int r = 0;
	int hw_id = 0;
	int arraysize= sizeof(cadiz_boot1)/sizeof(cadiz_boot1[0]);

	hw_id = Read_HW_ID();

	printk("%s: arraysize=%d\n", __func__, arraysize);

#if 0
	for(i = 0 ;i < arraysize; i++){
		//printk("%s: i=%d, 0x%02x 0x%02x\n", __func__,i, cadiz_boot1[i].addr,cadiz_boot1[i].param);

		r = cadiz_i2c_reg_write(cadiz_boot1[i].addr, cadiz_boot1[i].param);
		if(r != 0) return -EIO;
	}
#else
	r = cadiz_i2c_seq_write(&cadiz_boot1[0], arraysize);
	if(r != 0) return -EIO;
#endif

	if(hw_id == 3)
		cadiz_i2c_reg_write(0x025C, 0x05);

	printk("[DISPLAY] %s: End\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cadiz_init_boot1);

int cadiz_init_ipc_ibc(void){
	int r = 0;
	int arraysize= sizeof(cadiz_ipc_ibc)/sizeof(cadiz_ipc_ibc[0]);
	printk("%s: arraysize=%d\n", __func__, arraysize);

#if 0
	for(i = 0 ;i < arraysize; i++){
		//printk("%s: i=%d, 0x%02x 0x%02x\n", __func__,i, cadiz_ipc_ibc[i].addr,cadiz_ipc_ibc[i].param);
		r = cadiz_i2c_reg_write(cadiz_ipc_ibc[i].addr, cadiz_ipc_ibc[i].param);
		if(r != 0) return -EIO;
	}
#else
	r = cadiz_i2c_seq_write(&cadiz_ipc_ibc[0], arraysize);
	if(r != 0) return -EIO;
#endif

	printk("[DISPLAY] %s: End\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cadiz_init_ipc_ibc);

int cadiz_init_boot2(void){
	int r = 0;
	int arraysize= sizeof(cadiz_boot2)/sizeof(cadiz_boot2[0]);
	printk("%s: arraysize=%d\n", __func__, arraysize);

#if 0
	for(i = 0 ;i < arraysize; i++){
		//printk("%s: i=%d, 0x%02x 0x%02x\n", __func__,i, cadiz_boot2[i].addr,cadiz_boot2[i].param);
		r = cadiz_i2c_reg_write(cadiz_boot2[i].addr, cadiz_boot2[i].param);
		if(r != 0) return -EIO;
	}
#else
	r = cadiz_i2c_seq_write(&cadiz_boot2[0], arraysize);
	if(r != 0) return -EIO;
#endif

	printk("[DISPLAY] %s: End\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cadiz_init_boot2);

int cadiz_init_boot3(void){
	int i=0, r=0;
	int arraysize= sizeof(cadiz_boot3)/sizeof(cadiz_boot3[0]);
	printk("%s: arraysize=%d\n", __func__, arraysize);

	usleep_range(1000, 1000);

	for(i = 0 ;i < arraysize; i++){
		//printk("%s: i=%d, 0x%02x 0x%02x\n", __func__,i, cadiz_boot2[i].addr,cadiz_boot2[i].param);
		r = cadiz_i2c_reg_write(cadiz_boot3[i].addr, cadiz_boot3[i].param);
		if(r != 0) return -EIO;
	}

	printk("[DISPLAY] %s: End\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cadiz_init_boot3);

int cadiz_init_boot4(void){
	int i=0, r=0;
	int arraysize= sizeof(cadiz_boot4)/sizeof(cadiz_boot4[0]);
	printk("%s: arraysize=%d\n", __func__, arraysize);

	for(i = 0 ;i < arraysize; i++){
		//printk("%s: i=%d, 0x%02x 0x%02x\n", __func__,i, cadiz_boot2[i].addr,cadiz_boot2[i].param);
		r = cadiz_i2c_reg_write(cadiz_boot4[i].addr, cadiz_boot4[i].param);
		if(r != 0) return -EIO;
	}

	cadiz_ready = true;
	printk("[DISPLAY] %s: End\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cadiz_init_boot4);

#define CADIZ_POLLING_COUNT 10
int cadiz_init_polling(u16 reg, u8 value){
	int i=0, r=0;
	u8 rdata=0;

	for(i = 0 ;i < CADIZ_POLLING_COUNT; i++){
		r = cadiz_i2c_reg_read(reg, &rdata);

		if(r < 0){
			break;
		}else if(rdata!=value){
			usleep_range(10000, 10000);
			continue;
		}else{
			break;
		}
	}

	if(rdata!=value){
		printk("[DISPLAY] %s: Polling FAIL :0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n", __func__, reg, rdata, r, value);
		return -EIO;
	}else{
		printk("[DISPLAY] %s: Polling SUCCESS 0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n", __func__, reg, rdata, r,value);
		return 0;
	}
}

EXPORT_SYMBOL(cadiz_init_polling);

void cadiz_suspend(void)
{
	int ret = 0;

	printk("[DISPLAY] %s: Enter\n",__func__);
	cadiz_ready = false;

	if(cadiz_rst)
	   gpio_direction_output(cadiz_rst,0);
	usleep_range(2000, 2000);

	ret = regulator_set_optimum_mode(tcon_vdd, 100);
	printk("[DISPLAY] regulator_set_optimum_mode(): ret = %d \n", ret);
	ret = regulator_disable(tcon_vdd);
	usleep_range(3000, 3000);

	if(cadiz_ldo_enable)
		gpio_direction_output(cadiz_ldo_enable,0);
	usleep_range(500, 500);

	clk_disable_unprepare(s_clk);
	printk("[DISPLAY] clk_disable_unprepare\n");
	usleep_range(2000, 2000);

	ret = regulator_disable(vdd);
	if(cadiz_12)
		gpio_direction_output(cadiz_12,0);

	return;
}
EXPORT_SYMBOL(cadiz_suspend);

void cadiz_resume(void)
{
	int ret=0;

	printk("[DISPLAY] %s: Enter\n",__func__);
	if(cadiz_12) {
		ret = gpio_direction_output(cadiz_12,1);
		printk("[DISPLAY] cadiz set 1.2v ret = %d\n", ret);
	}
	ret = regulator_set_voltage(vdd, 1800000, 1800000);
	printk("[DISPLAY] cadiz l15 set 1.8v ret = %d \n",ret);
	ret = regulator_enable(vdd);
	printk("[DISPLAY] cadiz l15 enable ret = %d \n",ret);
	usleep_range(500, 500);

	ret |= clk_prepare_enable(s_clk);
	printk("[DISPLAY] clk_prepare_enable ret = %d \n",ret);
	usleep_range(5000, 5000);

	if(cadiz_ldo_enable) {
		ret = gpio_direction_output(cadiz_ldo_enable,1);
		printk("[DISPLAY] tcon set 1.8v ret = %d\n", ret);
	}

	ret = regulator_set_voltage(tcon_vdd, 3300000, 3300000);
	printk("[DISPLAY] tcon l11 set 3.3v ret = %d \n",ret);
	ret = regulator_set_optimum_mode(tcon_vdd, 157000);
	printk("[DISPLAY] regulator_set_optimum_mode(): ret = %d \n", ret);
	ret = regulator_enable(tcon_vdd);
	printk("[DISPLAY] tcon l11 enable ret = %d \n",ret);
	usleep_range(200, 200);

	return;
}
EXPORT_SYMBOL(cadiz_resume);

void cadiz_rest_gpio(void)
{
	printk("[DISPLAY] %s: Enter\n",__func__);

	if(cadiz_rst)
		gpio_direction_output(cadiz_rst,1);
	usleep_range(2000, 2000);

	return;
}
EXPORT_SYMBOL(cadiz_rest_gpio);

int mCadizSupport=-1;
int is_cadiz_support(void)
{
	int hw_id;

	if(mCadizSupport == -1){
		hw_id = Read_HW_ID();
		mCadizSupport = hw_id ? 1 : 0;
		printk("[DISPLAY] %s: Enter mCadizSupport=%d hw_id=%d\n", __func__, mCadizSupport, hw_id);
	}

	pr_debug("[DISPLAY] %s: Enter mCadizSupport=%d\n", __func__, mCadizSupport);

	return mCadizSupport;
}
EXPORT_SYMBOL(is_cadiz_support);

bool cadiz_register_store(u16 store_reg, u8 store_val){
	int j;

	//keep register 0x0841 initial value on 0x7B
	if (store_reg == 0x0841)
		return true;

	//check boot1
	for(j = 0; j < cadiz_boot1_len; j++){
		if(store_reg == cadiz_boot1[j].addr){
			cadiz_boot1[j].addr = store_reg;
			cadiz_boot1[j].param = store_val;
			return true;
		}
	}

	//check ipc_ibc
	for(j = 0; j < cadiz_ipc_ibc_len; j++){
		if(store_reg == cadiz_ipc_ibc[j].addr){
			cadiz_ipc_ibc[j].addr = store_reg;
			cadiz_ipc_ibc[j].param = store_val;
			return true;
		}
	}

	pr_info("WARN: reg: 0x%08x,0x%08x does not belong to boot1 or ipc_ibc\n", store_reg, store_val);

	return false;
}

static ssize_t send_cadiz_i2c_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int x0=0, x1=0;
	int r=0;

	sscanf(buf, "%x %x", &x0, &x1);
	r = cadiz_i2c_reg_write(x0, x1);
	cadiz_register_store(x0,x1);

	printk("[DISPLAY] send i2c 0x%04x,0x%02x : ret = %d\n",x0,x1,r);

	return count;
}

static u16 read_cadiz_i2c_reg;
static u8 read_cadiz_i2c_rdata;
static ssize_t read_cadiz_i2c_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int x0=0;
	u8 rdata=0;
	int r=0;

	sscanf(buf, "%x", &x0);
	r = cadiz_i2c_reg_read(x0, &rdata);

	if(r<=0){
		read_cadiz_i2c_reg = 0x0;
		read_cadiz_i2c_rdata = 0x0;
	}else{
		read_cadiz_i2c_reg = x0;
		read_cadiz_i2c_rdata = rdata;
	}

	printk("[DISPLAY] read i2c 0x%04xh = 0x%02x : ret = %d\n",x0,rdata,r);

	return count;
}

static ssize_t read_cadiz_i2c_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x 0x%02x\n",read_cadiz_i2c_reg,read_cadiz_i2c_rdata);
}

DEVICE_ATTR(send_cadiz_i2c,S_IRUGO | S_IWUSR, NULL ,send_cadiz_i2c_store);
DEVICE_ATTR(read_cadiz_i2c,S_IRUGO | S_IWUSR, read_cadiz_i2c_show ,read_cadiz_i2c_store);

static struct attribute *cadiz_attrs[] = {
		&dev_attr_send_cadiz_i2c.attr,
		&dev_attr_read_cadiz_i2c.attr,
        NULL
};

static struct attribute_group cadiz_attr_group = {
        .attrs = cadiz_attrs,
        .name = "cadiz",
};

int cadiz_dump_error_info(void)
{
	int i;
	int arraysize;
	printk("---------- %s: Start ----------\n",__func__);

	arraysize= sizeof(cadiz_boot1)/sizeof(cadiz_boot1[0]);
	for (i = 0; i < arraysize; i++) {
		if ((cadiz_boot1[i].addr == 0x0841) || (cadiz_boot1[i].addr == 0x0901) ||
			(cadiz_boot1[i].addr == 0x0908) || (cadiz_boot1[i].addr == 0x0909))
			printk("[DISPLAY] cadiz_boot1: reg(0x%04x) val(0x%02x)\n", cadiz_boot1[i].addr, cadiz_boot1[i].param);
	}

	arraysize= sizeof(cadiz_ipc_ibc)/sizeof(cadiz_ipc_ibc[0]);
	for (i = 0; i < arraysize; i++)
		printk("[DISPLAY] cadiz_ipc_ibc: reg(0x%04x) val(0x%02x)\n", cadiz_ipc_ibc[i].addr, cadiz_ipc_ibc[i].param);

	printk("----------- %s: End -----------\n",__func__);
	return 0;
}

static long cadiz_dev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	cadiz_settings cadiz_regs;
	int i, len, r;
	u8 rdata=0;
	u16 h_disp=0, v_disp=0, h_total=0;
	int j;
	u8 cadiz_0x0aff, result=0x0;

	if (!cadiz_ready) {
		dev_err(&cadiz_client->dev, "not ready now!!!\n");
		return -EINVAL;
	}

	//dev_info(&cadiz_client->dev, "%s\n", __func__);
	switch(cmd) {
	case CADIZ_IOCTL_SET_REGISTERS:
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EFAULT;
		}

		len = cadiz_regs.len;
		if(len > MAX_CADIZ_REGS) {
			dev_err(&cadiz_client->dev, "%d is greater than %d\n", len, MAX_CADIZ_REGS);
			return -EINVAL;
		}
		if(len <= 0 ) {
			dev_err(&cadiz_client->dev, "%d is invalid length\n", len);
			return -EINVAL;
		}

//---------------------------------------------------------------------------------------------
		for(i = 0; i < len; i++) {
			switch (cadiz_regs.regs[i][0]) {
			//h_total
			case 0x0908:
				h_total |= cadiz_regs.regs[i][1];
				break;
			case 0x0909:
				h_total |= cadiz_regs.regs[i][1]<<8;
				break;
			//h_disp
			case 0x0A0C:
				h_disp |= cadiz_regs.regs[i][1];
				break;
			case 0x0A0D:
				h_disp |= cadiz_regs.regs[i][1]<<8;
				break;
			//v_disp
			case 0x0A10:
				v_disp |= cadiz_regs.regs[i][1];
				break;
			case 0x0A11:
				v_disp |= cadiz_regs.regs[i][1]<<8;
				break;
			//cadiz mode
			case 0x0AFF:
				cadiz_0x0aff = cadiz_regs.regs[i][1];
				dev_info(&cadiz_client->dev, "0x0AFF= 0x%x\n", cadiz_regs.regs[i][1]);
				break;
			}
		}
		if (h_disp || v_disp) {
			if ((h_disp > 0x320) || (v_disp > 0x500)) {
				result |= 0x1;
			}
		}
		if (h_total) {
			result |= 0x2;
			for (j = 0; j < CADIZ_BYPASS_SIZE; j++) {
				if (h_total == cadiz_bypass[j])
					result &= 0x1;
			}
		}
		if (result)
			cadiz_dump_error_info();
//---------------------------------------------------------------------------------------------

		for(i = 0; i < len; i++) {
			//h_disp or v_disp value error, resolution bigger than 800*1280
			if (result & 0x1) {
				dev_err(&cadiz_client->dev, "ERROR h_disp= 0x%x, v_disp= 0x%x\n", h_disp, v_disp);
				switch (cadiz_regs.regs[i][0]) {
				//h_disp: 0x320
				case 0x0A0C:
					cadiz_regs.regs[i][1] = 0x20;
					break;
				case 0x0A0D:
					cadiz_regs.regs[i][1] = 0x3;
					break;
				//v_disp: 0x500
				case 0x0A10:
					cadiz_regs.regs[i][1] = 0x0;
					break;
				case 0x0A11:
					cadiz_regs.regs[i][1] = 0x5;
					break;
				}
			}
			//h_total value error
			if (result & 0x2) {
				dev_err(&cadiz_client->dev, "ERROR h_total= 0x%x\n", h_total);
				switch (cadiz_regs.regs[i][0]) {
				case 0x0908:
					if (cadiz_0x0aff == 0xbf)
						cadiz_regs.regs[i][1] = cadiz_bypass[1]&0xff;
					else if (cadiz_0x0aff == 0x0e)
						cadiz_regs.regs[i][1] = cadiz_bypass[2]&0xff;
					break;
				case 0x0909:
					if (cadiz_0x0aff == 0xbf)
						cadiz_regs.regs[i][1] = cadiz_bypass[1]>>8;
					else if (cadiz_0x0aff == 0x0e)
						cadiz_regs.regs[i][1] = cadiz_bypass[2]>>8;
					break;
				}
			}
			//show 0x0908, 0x0909 info
			switch (cadiz_regs.regs[i][0]) {
			case 0x0908:
			case 0x0909:
				dev_info(&cadiz_client->dev, "0x%04x= 0x%02x\n", cadiz_regs.regs[i][0], cadiz_regs.regs[i][1]);
				break;
			}

			if(cadiz_register_store(cadiz_regs.regs[i][0],cadiz_regs.regs[i][1])){
				r = cadiz_i2c_reg_write(cadiz_regs.regs[i][0], cadiz_regs.regs[i][1]);
				if (r < 0)
					return -EIO;
			}else{
				return -EINVAL;
			}
		}

		break;
	case CADIZ_IOCTL_GET_REGISTERS:
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EFAULT;
		}

		len = cadiz_regs.len;
		if(len > MAX_CADIZ_REGS) {
			dev_err(&cadiz_client->dev, "%d is greater than %d\n", len, MAX_CADIZ_REGS);
			return -EINVAL;
		}
		if(len <= 0 ) {
			dev_err(&cadiz_client->dev, "%d is invalid length\n", len);
			return -EINVAL;
		}

		for(i = 0; i < len; i++) {
#if 1
			switch (cadiz_regs.regs[i][0]) {
			case 0x0902:
				cadiz_regs.regs[i][1] = boot1_0x0902;
				//dev_err(&cadiz_client->dev, "read 0x0902= %x\n", cadiz_regs.regs[i][1]);
				break;
			case 0x0903:
				cadiz_regs.regs[i][1] = boot1_0x0903;
				//dev_err(&cadiz_client->dev, "read 0x0903= %x\n", cadiz_regs.regs[i][1]);
				break;
			case 0x0904:
				cadiz_regs.regs[i][1] = boot1_0x0904;
				//dev_err(&cadiz_client->dev, "read 0x0904= %x\n", cadiz_regs.regs[i][1]);
				break;
			case 0x0905:
				cadiz_regs.regs[i][1] = boot1_0x0905;
				//dev_err(&cadiz_client->dev, "read 0x0905= %x\n", cadiz_regs.regs[i][1]);
				break;
			case 0x0916:
				cadiz_regs.regs[i][1] = boot1_0x0916;
				//dev_err(&cadiz_client->dev, "read 0x0916= %d\n", cadiz_regs.regs[i][1]);
				break;
			case 0x0917:
				cadiz_regs.regs[i][1] = boot1_0x0917;
				//dev_err(&cadiz_client->dev, "read 0x0917= %d\n", cadiz_regs.regs[i][1]);
				break;
			default:
				r = cadiz_i2c_reg_read(cadiz_regs.regs[i][0], &rdata);
				if(r>=0){
					cadiz_regs.regs[i][1]=rdata;
				}
				//dev_err(&cadiz_client->dev, "default read 0x%x= 0x%x\n", cadiz_regs.regs[i][0], cadiz_regs.regs[i][1]);
			}
#else
			r = cadiz_i2c_reg_read(cadiz_regs.regs[i][0], &rdata);
			if(r>=0){
				cadiz_regs.regs[i][1]=rdata;
			}
#endif
		}

		if (copy_to_user((void __user *)arg, &cadiz_regs,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EFAULT;
		}

		break;
	default:
		dev_err(&cadiz_client->dev, "%s:unknown cmd=%d\n", __func__, cmd);
		return -EINVAL;
	}
	//dev_info(&cadiz_client->dev, "%s End\n", __func__);
	return 0;
}

static int cadiz_dev_open(struct inode *inode, struct file *filp)
{
	dev_info(&cadiz_client->dev, "%s\n", __func__);
	return nonseekable_open(inode, filp);
}

static const struct file_operations cadiz_dev_fops = {
	.owner = THIS_MODULE,
	.open  = cadiz_dev_open,
	.unlocked_ioctl = cadiz_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cadiz_dev_ioctl,
#endif
};

static int dbg_read_ipc_ibc_show(struct seq_file *s, void *unused)
{
	int i;
	int arraysize= sizeof(cadiz_ipc_ibc)/sizeof(cadiz_ipc_ibc[0]);

	pr_info("%s\n", __func__);

	for(i = 0; i < arraysize; i++)
		seq_printf(s, "%d:reg(0x%04x) val(0x%02x)\n"
			, i, cadiz_ipc_ibc[i].addr, cadiz_ipc_ibc[i].param);

	return 0;
}

static int dbg_read_ipc_ibc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_read_ipc_ibc_show, inode->i_private);
}

static const struct file_operations dbg_read_ipc_ibc_fops = {
	.open		= dbg_read_ipc_ibc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int cadiz_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int addr;
	int ret=0;

	if(!is_cadiz_support()) {
		printk("[DISPLAY] No support cadiz %s\n", __func__);
		return ret;
	}

	printk("[DISPLAY] %s: Enter\n",__func__);

	cadiz_client = client;
	addr = client->addr;
	printk("[DISPLAY] %s: slave address=0x%x\n", __func__, addr);

	vdd = regulator_get(&client->dev, "cadiz_18_vdd");
	tcon_vdd = regulator_get(&client->dev, "tcon_ldo_vdd");

	cadiz_ldo_enable = of_get_named_gpio_flags(client->dev.of_node, "sony,cadiz-ldo-en",0, NULL);
	if (gpio_request(cadiz_ldo_enable, "cadiz_ldo_enable") != 0)
		printk("[DISPLAY] %s: request cadiz_ldo_enable failed \n", __func__);

	cadiz_rst = of_get_named_gpio_flags(client->dev.of_node, "sony,cadiz-rst",0, NULL);
	if (gpio_request(cadiz_rst, "cadiz_rst") != 0)
		printk("[DISPLAY] %s: request cadiz_rst failed \n", __func__);

	cadiz_12= of_get_named_gpio_flags(client->dev.of_node, "qcom,platform-cadiz-12-gpio",0, NULL);
	if (gpio_request(cadiz_12, "cadiz_12") != 0)
		printk("[DISPLAY] %s: request cadiz_12 failed \n", __func__);

	//register sysfs
	ret = sysfs_create_group(&client->dev.kobj, &cadiz_attr_group);
	if(ret){
		printk("[DISPLAY] %s: sysfs_create_group failed \n", __func__);
	}

	//debugfs
	cadiz_debugfs = debugfs_create_dir("cadiz", NULL);
	if (IS_ERR(cadiz_debugfs)){
		printk("[DISPLAY] %s: debugfs_create_dir failed \n", __func__);
	}else{
		debugfs_create_symlink("read_reg",cadiz_debugfs,
			"/sys/devices/soc.0/78b5000.i2c/i2c-1/1-0010/cadiz/read_cadiz_i2c");
		debugfs_create_symlink("write_reg",cadiz_debugfs,
			"/sys/devices/soc.0/78b5000.i2c/i2c-1/1-0010/cadiz/send_cadiz_i2c");
		debugfs_create_file("show_ipc_ibc", S_IRUGO, cadiz_debugfs, NULL, &dbg_read_ipc_ibc_fops);
	}

	//clk
	s_clk  = clk_get(&client->dev, "ref_clk");

	if (s_clk == NULL)
		printk("[DISPLAY] s_clk == NULL\n");
	else
		printk("[DISPLAY] s_clk != NULL\n");

	ret = clk_set_rate(s_clk, 19200000);
	printk("[DISPLAY] clk_set_rate == %d\n", ret);

	//Power
	ret = regulator_enable(vdd);
	printk("[DISPLAY] tcon l15 enable ret = %d \n",ret);
	ret = regulator_set_voltage(tcon_vdd, 3300000, 3300000);
	printk("[DISPLAY] tcon l11 set 3.3v ret = %d \n",ret);
	ret = regulator_set_optimum_mode(tcon_vdd, 157000);
	printk("[DISPLAY] regulator_set_optimum_mode(): ret = %d \n", ret);
	ret = regulator_enable(tcon_vdd);
	printk("[DISPLAY] tcon l11 enable ret = %d \n",ret);
	usleep_range(500, 500);

	//misc device
	cadiz_dev.minor = MISC_DYNAMIC_MINOR;
	cadiz_dev.name = "cadiz";
	cadiz_dev.fops = &cadiz_dev_fops;
	if(misc_register(&cadiz_dev)) {
		dev_err(&client->dev, "%s:fail to register misc device\n", __func__);
	}

	//init array length for restore
	cadiz_boot1_len = sizeof(cadiz_boot1)/sizeof(cadiz_boot1[0]);
	cadiz_ipc_ibc_len = sizeof(cadiz_ipc_ibc)/sizeof(cadiz_ipc_ibc[0]);
	printk("[DISPLAY] %s: cadiz_boot1_len=%d, cadiz_ipc_ibc_len=%d\n", __func__, cadiz_boot1_len, cadiz_ipc_ibc_len);

	//QOS
	pm_qos_add_request(&tput_pm_qos_req, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	//cadiz store init register
	cadiz_get_init_boot1();
	cadiz_ready = true;

	return 0;
}

static struct of_device_id cadiz_i2c_table[] = {
	{ .compatible = "sony,cadiz"}, //Compatible node must match dts
	{ },
};

static const struct i2c_device_id cadiz_id[] = {
	{ "cadiz", 0 },
	{ },
};

static struct i2c_driver cadiz_driver = {
	.driver = {
		.name = "cadiz",
		.owner = THIS_MODULE,
		.of_match_table = cadiz_i2c_table,
	},
	.probe = cadiz_probe,
	.id_table = cadiz_id,
};


static int __init cadiz_I2C_init(void)
{
	int ret = 0;
	printk("[DISPLAY] %s: Enter\n", __func__);
	ret = i2c_add_driver(&cadiz_driver);
	printk("[DISPLAY] %s: ret=%d", __func__, ret);

	return ret;
}

static void __exit cadiz_I2C_exit(void)
{
	return;
}

module_init(cadiz_I2C_init);
module_exit(cadiz_I2C_exit);

MODULE_DESCRIPTION("cadiz");
MODULE_LICENSE("GPL v2");
