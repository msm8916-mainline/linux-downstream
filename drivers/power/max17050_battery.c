/*
 * Fuel gauge driver for Maxim 17050 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
 *
 * Copyright (C) 2012 LG Electronics
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
 * This driver is based on max17040_battery.c
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/power/max17050_battery.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>


static struct i2c_client *max17050_i2c_client;

u16 pre_soc = 100;
u16 real_soc = 100;

struct max17050_chip {
	struct i2c_client *client;
	/*struct power_supply battery;*/
	struct max17050_platform_data *pdata;
	struct delayed_work	max17050_monitor_work;
};

static int max17050_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17050_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17050_multi_write_data(struct i2c_client *client,
			int reg, const u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17050_multi_read_data(struct i2c_client *client,
			int reg, u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client,
			reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

int max17050_get_mvolts(void)
{
	u16 read_reg;
	int vbatt_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_info("[MAX17050] %s: i2c NULL vbatt = 800 mV\n", __func__);
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_debug("%s: vbatt = %d mV\n", __func__, vbatt_mv);

	return vbatt_mv;

}

int max17050_suspend_get_mvolts(void)
{
	u16 read_reg;
	int vbatt_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_info("[MAX17050] %s: i2c NULL vbatt = 3950 mV\n", __func__);
		return 800;
	}

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_debug("%s: vbatt = %d mV\n", __func__, vbatt_mv);

	return vbatt_mv;

}

int max17050_get_capacity_percent(void)
{
	int battery_soc = 0;
	int read_reg = 0;

	u8 upper_reg;
	u8 lower_reg;
	if (max17050_i2c_client == NULL) {
		return 80;
	} else {
		read_reg = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_VF);

		if (read_reg < 0) {
			pr_err("[MAX17050] %s: i2c Read Fail battery SOC = %d\n",
				__func__, pre_soc);
			return pre_soc;
		}
		upper_reg = (read_reg & 0xFF00)>>8;
		lower_reg = (read_reg & 0xFF);

		pr_debug("%s: read_reg = %X  upper_reg = %X lower_reg = %X\n",
			__func__, read_reg, upper_reg, lower_reg);

		/* SOC scaling for stable max SOC and changed Cut-off */
		/* Adj SOC = (FG SOC-Emply)/(Full-Empty)*100 */
		/* cut off vol 3.48V : (soc - 1.132%)*100/(94.28% - 1.132%) */
		/* full capacity soc 106.5% , real battery SoC 100.7% */
		battery_soc = ((upper_reg * 256)+lower_reg)*10/256;

		pr_debug("%s: battery_soc  = %d\n", __func__, battery_soc);

		battery_soc = (battery_soc * 100) * 100;
		battery_soc = (battery_soc / 9440) - 1; /* 105.8% scailing */

		if (((battery_soc/10) < 1) && ((battery_soc%10) >= 5))
			battery_soc = 10;

		battery_soc /= 10;

		pr_debug("%s: battery_soc  = %d (upper_reg = %d lower_reg = %d)\n",
			__func__, battery_soc, upper_reg, lower_reg);

		real_soc = battery_soc;

		if (battery_soc >= 100)
			battery_soc = 100;

		if (battery_soc < 0)
			battery_soc = 0;

	}
		pre_soc = battery_soc;

		return battery_soc;

}

int max17050_get_current(void)
{
	u16 read_reg;
	int ibatt_ma;
	int avg_ibatt_ma;
	u16 sign_bit;

	if (max17050_i2c_client == NULL) {
		pr_info("[MAX17050] %s: i2c NULL", __func__);
		return 999;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_CURRENT);
	if (read_reg < 0)
		return 999; /*Error Value return.*/

	sign_bit = (read_reg & 0x8000)>>15;

	if (sign_bit == 1)
		ibatt_ma = (15625 * (read_reg  - 65536))/100000;
	else
		ibatt_ma = (15625 * read_reg) / 100000;

	read_reg = max17050_read_reg(max17050_i2c_client,
		MAX17050_AVERAGE_CURRENT);
	if (read_reg < 0)
		return 999;/*Error Value return.*/

	sign_bit = (read_reg & 0x8000)>>15;

	if (sign_bit == 1)
		avg_ibatt_ma = (15625 * (read_reg  - 65536)) / 100000;
	else
		avg_ibatt_ma = (15625 * read_reg) / 100000;

	pr_debug("%s: I_batt = %d mA avg_I_batt = %d mA\n",
		__func__, ibatt_ma, avg_ibatt_ma);

	return avg_ibatt_ma;

}

int max17050_read_battery_age(void)
{
	u16 read_reg;
	int battery_age;

	if (max17050_i2c_client == NULL) {
		pr_debug("MAX17050] %s: i2c NULL battery age: 800\n", __func__);
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_AGE);
	if (read_reg < 0)
		return 999; /*Error Value return.*/

	battery_age = (read_reg >> 8);

	pr_debug("%s: battery_age = %d\n", __func__, battery_age);

	return battery_age;
}

bool max17050_battery_full_info_print(void)
{
	u16 read_reg;
	int battery_age;
	int battery_remain_capacity;
	int battery_time_to_empty_sec;
	int battery_soc;
	int battery_voltage;
	int battery_temp;
	int battery_current;
	int battery_full_cap;

	battery_age = max17050_read_battery_age();

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_REM_CAP_REP);

	battery_remain_capacity = (5 * read_reg)/10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);

	battery_full_cap = (5 * read_reg)/10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_TTE);

	battery_time_to_empty_sec = (5625 * read_reg)/1000;

	battery_soc = max17050_get_capacity_percent();

	battery_voltage = max17050_get_mvolts();

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_TEMPERATURE);

	battery_temp = (read_reg >> 8);

	if (battery_temp > 127)
		battery_temp = battery_temp - 256;
	else
		battery_temp = battery_temp;

	battery_current = max17050_get_current();

	pr_info("* -- max17050 battery full info print ---------- *\n");
	pr_info("battery age = %d %%    remain capacity = %d mAh\n",
		battery_age, battery_remain_capacity);
	pr_info("battery current = %d mA     Time to Empty = %d min\n",
		battery_current, battery_time_to_empty_sec/60);
	pr_info("battery SOC = %d %%    Voltage = %d mV\n",
		battery_soc, battery_voltage);
	pr_info("battery TEMP = %d C   full capacity = %d mAh\n",
		battery_temp, battery_full_cap);
	pr_info("* ---------------------------------------------- *\n");

	return 0;
}

static void max17050_monitor_work(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip,
				max17050_monitor_work.work);

	max17050_battery_full_info_print();

	schedule_delayed_work(&chip->max17050_monitor_work,
			msecs_to_jiffies(20000));
}

bool max17050_i2c_write_and_verify(u8 addr, u16 value)
{
	u16 read_reg;

	max17050_write_reg(max17050_i2c_client, addr, value);

	read_reg = max17050_read_reg(max17050_i2c_client, addr);

	if (read_reg == value) {
		pr_debug("[MAX17050] %s() Addr = 0x%X,", __func__, addr);
		pr_debug(" Value = 0x%X Success\n", value);
		return 1;
	} else {
		pr_debug("[MAX17050] %s() Addr = 0x%X,", __func__, addr);
		pr_debug(" Value = 0x%X Fail to write.", value);
		pr_debug(" Write once more.\n");
		max17050_write_reg(max17050_i2c_client, addr, value);
		return 0;
	}

	return 1;

}


static int max17050_new_custom_model_write(void)
{
	/*u16 ret;*/
	u16 read_reg;
	/*u16 write_reg;*/
	u16 vfsoc;
	u16 full_cap_0;
	u16 rem_cap;
	u16 rep_cap;
	u16 dQ_acc;
	u16 capacity = 0x17B6;
	/*u16 capacity = 0x1802;*/

	u16 i;

	u8 custom_model_1[32] = {
		0xC0, 0x9C, 0xC0, 0xA5, 0xF0, 0xAD, 0x30, 0xB6,
		0x90, 0xB9, 0x30, 0xBC, 0x30, 0xBD, 0xB0, 0xBD,
		0xC0, 0xBE, 0xF0, 0xBF, 0x30, 0xC2, 0x70, 0xC7,
		0x40, 0xCA, 0x00, 0xCD, 0x60, 0xD2, 0x00, 0xD8,
	};

	u8 custom_model_2[32] = {
		0x90, 0x00, 0xA0, 0x01, 0xA0, 0x01, 0x00, 0x0D,
		0x00, 0x0E, 0x10, 0x19, 0x20, 0x32, 0x70, 0x11,
		0xC0, 0x10, 0xD0, 0x0C, 0xF0, 0x08, 0xA0, 0x08,
		0xF0, 0x08, 0xF0, 0x06, 0xF0, 0x06, 0xF0, 0x06,
	};

	u8 custom_model_3[32] = {
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
	};

	u8 read_custom_model_1[32];
	u8 read_custom_model_2[32];
	u8 read_custom_model_3[32];

	pr_debug("[MAX17050] %s()  Start\n", __func__);

	/*0. Check for POR or Battery Insertion*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
	pr_debug("[MAX17050] %s()  MAX17050_STATUS = 0x%X\n", __func__, read_reg);
	if (read_reg == 0) {
		pr_debug("[MAX17050] IC Non-Power-On-Reset state.");
		pr_debug(" Go to max17050_save_bat_info_to_flash.\n");
		return 2; /*Go to save the custom model.*/
	} else {
		pr_debug("[MAX17050] IC Power On Reset state. Start Custom Model Write.\n");

		/*1. Delay 500mS*/
		msleep(500);

		/*1.1 Version Check*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_VERSION);
		pr_debug("[MAX17050] %s()  MAX17050_VERSION = 0x%X\n", __func__, read_reg);
		if (read_reg != 0xAC) {
			pr_debug("[MAX17050]  Version Check Error.");
			pr_debug(" Version Check = 0x%x\n", read_reg);
			return 1; /*Version Check Error*/
		}

		/*2. Initialize Configuration*/
		max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG, 0x506B);
		/* External temp and enable alert function , temp only = 0x2100 */
		max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG, 0x2104);
		max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG, 0x87A4);
		max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG, 0x2607);
		max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, 0x0870);
		max17050_write_reg(max17050_i2c_client, MAX17050_FULL_SOC_THR, 0x5F00);

		/*4. Unlock Model Access*/
		max17050_write_reg(max17050_i2c_client, 0x62, 0x0059);
		max17050_write_reg(max17050_i2c_client, 0x63, 0x00C4);

		/*5. Write/Read/Verify the Custom Model*/
		max17050_multi_write_data(max17050_i2c_client,
				0x80, &custom_model_1[0], 32);
		max17050_multi_write_data(max17050_i2c_client,
				0x90, &custom_model_2[0], 32);
		max17050_multi_write_data(max17050_i2c_client,
				0xA0, &custom_model_3[0], 32);

		/*For test only. Read back written-custom model data.*/
		max17050_multi_read_data(max17050_i2c_client,
				0x80, &read_custom_model_1[0], 32);
		max17050_multi_read_data(max17050_i2c_client,
				0x90, &read_custom_model_2[0], 32);
		max17050_multi_read_data(max17050_i2c_client,
				0xA0, &read_custom_model_3[0], 32);

		/*Compare with original one.*/
		for (i = 0 ; i < 32 ; i++) {
			if (read_custom_model_1[i] != custom_model_1[i]) {
				pr_debug("[MAX17050] Custom Model");
				pr_debug(" 1[%d]	Write Error\n", i);
			}
		}

		for (i = 0 ; i < 32 ; i++) {
			if (read_custom_model_2[i] != custom_model_2[i]) {
				pr_debug("[MAX17050] Custom Model");
				pr_debug(" 2[%d]	Write Error\n", i);
			}
		}

		for (i = 0 ; i < 32 ; i++) {
			if (read_custom_model_3[i] != custom_model_3[i]) {
				pr_debug("[MAX17050] Custom Model");
				pr_debug(" 3[%d] Write Error\n", i);
			}
		}
		/*For Test only end.*/

		/*8. Lock Model Access*/
		max17050_write_reg(max17050_i2c_client, 0x62, 0x0000);
		max17050_write_reg(max17050_i2c_client, 0x63, 0x0000);

		/*9. Verify the Model Access is locked.*/
		/*Skip.*/

		/*10. Write Custom Parameters*/
		max17050_i2c_write_and_verify(MAX17050_RCOMP_0, 0x0048);
		max17050_i2c_write_and_verify(MAX17050_TEMP_CO, 0x1D2A);
		max17050_i2c_write_and_verify(MAX17050_TEMP_NOM, 0x1400);
		max17050_write_reg(max17050_i2c_client, MAX17050_T_GAIN, 0xE932);
		max17050_write_reg(max17050_i2c_client, MAX17050_T_OFF, 0x2381);

		/*Termination Current 400mA*/
		max17050_write_reg(max17050_i2c_client, MAX17050_I_CHG_TERM, 0x0280);
		max17050_i2c_write_and_verify(MAX17050_V_EMPTY, 0xA2DA);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_00, 0x2602);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_10, 0x1A82);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_20, 0x0A04);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_30, 0x0804);

		/*11. Update Full Capacity Parameters*/
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);

		/*13. Delay at least 350mS*/
		msleep(360);

		/*14. Write VFSOC value to VFSOC 0*/
		vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
		pr_debug("[MAX17050] %s()  vfsoc = 0x%X\n", __func__, vfsoc);
		max17050_write_reg(max17050_i2c_client, 0x60, 0x0080);
		max17050_i2c_write_and_verify(0x48, vfsoc);
		max17050_write_reg(max17050_i2c_client, 0x60, 0x0000);

		/*15. Write temperature (default 20 deg C)*/
		max17050_i2c_write_and_verify(MAX17050_TEMPERATURE, 0x1400);

		/*16. Load New Capacity Parameters*/
		full_cap_0 =  max17050_read_reg(max17050_i2c_client, 0x35);
		pr_debug("[MAX17050] %s()  full_cap_0 = %d  = 0x%X\n",
				__func__, full_cap_0, full_cap_0);
		rem_cap = (vfsoc * full_cap_0) / 25600;
		pr_debug("[MAX17050] %s()  rem_cap = %d  = 0x%X\n",
				__func__, rem_cap, rem_cap);
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
		rep_cap = rem_cap;
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
		dQ_acc = (capacity / 4);
		max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
		max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x3200);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_SOC_REP, vfsoc);

		/*17. Initialization Complete*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
		max17050_i2c_write_and_verify(MAX17050_STATUS, (read_reg & 0xFFFD));

		/*End of the Custom Model step 1.*/
		pr_debug("[MAX17050] End of the max17050_new_custom_model_write.\n");

		pr_debug("[MAX17050] %s()  End\n", __func__);
	}
		return 0; /*Success to write.*/

}

static int max17050_force_custom_model_write(void)
{
	/*u16 ret;*/
	u16 read_reg;
	/*u16 write_reg;*/
	u16 vfsoc;
	u16 full_cap_0;
	u16 rem_cap;
	u16 rep_cap;
	u16 dQ_acc;
	u16 capacity = 0x1770;

	u16 i;

	u8 custom_model_1[32] = {
		0x30, 0x7F, 0x10, 0xB6, 0xD0, 0xB7, 0xA0, 0xB9,
		0xC0, 0xBB, 0xA0, 0xBC, 0xE0, 0xBC, 0x30, 0xBD,
		0x90, 0xBD, 0xD0, 0xBE, 0xE0, 0xC1, 0xF0, 0xC3,
		0x10, 0xC6, 0x30, 0xC9, 0x60, 0xCC, 0x90, 0xCF,
	};

	u8 custom_model_2[32] = {
		0xE0, 0x00, 0x70, 0x0E, 0xF0, 0x0D, 0x00, 0x0F,
		0x20, 0x15, 0xF0, 0x46, 0xC0, 0x38, 0x70, 0x19,
		0x00, 0x18, 0x60, 0x0C, 0x20, 0x0D, 0xC0, 0x0C,
		0xE0, 0x07, 0xF0, 0x08, 0xF0, 0x08, 0xF0, 0x08,
	};

	u8 custom_model_3[32] = {
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
		0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
	};

	u8 read_custom_model_1[32];
	u8 read_custom_model_2[32];
	u8 read_custom_model_3[32];

	pr_debug("[MAX17050] %s()  Start\n", __func__);

	/*0. No Check for POR or Battery Insertion*/
	pr_debug("[MAX17050]Start Force Custom Model Write.\n");

	/*1. Delay 500mS*/
	msleep(500);

	/*1.1 Version Check*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_VERSION);
	pr_debug("[MAX17050] %s()  MAX17050_VERSION = 0x%X\n", __func__, read_reg);

	/*2. Initialize Configuration*/
	max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG, 0x506B);
	max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG, 0x2100);
	max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG, 0x87A4);
	max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG, 0x2607);
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, 0x0870);
	max17050_write_reg(max17050_i2c_client, MAX17050_FULL_SOC_THR, 0x5F00);

	/*4. Unlock Model Access*/
	max17050_write_reg(max17050_i2c_client, 0x62, 0x0059);
	max17050_write_reg(max17050_i2c_client, 0x63, 0x00C4);

	/*5. Write/Read/Verify the Custom Model*/
	max17050_multi_write_data(max17050_i2c_client,
			0x80, &custom_model_1[0], 32);
	max17050_multi_write_data(max17050_i2c_client,
			0x90, &custom_model_2[0], 32);
	max17050_multi_write_data(max17050_i2c_client,
			0xA0, &custom_model_3[0], 32);

	/*For test only. Read back written-custom model data.*/
	max17050_multi_read_data(max17050_i2c_client,
			0x80, &read_custom_model_1[0], 32);
	max17050_multi_read_data(max17050_i2c_client,
			0x90, &read_custom_model_2[0], 32);
	max17050_multi_read_data(max17050_i2c_client,
			0xA0, &read_custom_model_3[0], 32);

	/*Compare with original one.*/
	for (i = 0 ; i < 32 ; i++) {
		if (read_custom_model_1[i] != custom_model_1[i]) {
			pr_debug("[MAX17050] Custom Model");
			pr_debug(" 1[%d]	Write Error\n", i);
		}
	}

	for (i = 0 ; i < 32 ; i++) {
		if (read_custom_model_2[i] != custom_model_2[i]) {
			pr_debug("[MAX17050] Custom Model");
			pr_debug(" 2[%d]	Write Error\n", i);
		}
	}

	for (i = 0 ; i < 32 ; i++) {
		if (read_custom_model_3[i] != custom_model_3[i]) {
			pr_debug("[MAX17050] Custom Model");
			pr_debug(" 3[%d]	Write Error\n", i);
		}
	}
	/*For Test only end.*/

	/*8. Lock Model Access*/
	max17050_write_reg(max17050_i2c_client, 0x62, 0x0000);
	max17050_write_reg(max17050_i2c_client, 0x63, 0x0000);

	/*9. Verify the Model Access is locked.*/
	/*Skip.*/

	/*10. Write Custom Parameters*/
	max17050_i2c_write_and_verify(MAX17050_RCOMP_0, 0x0058);
	max17050_i2c_write_and_verify(MAX17050_TEMP_CO, 0x2230);
	max17050_i2c_write_and_verify(MAX17050_TEMP_NOM, 0x1400);
	max17050_write_reg(max17050_i2c_client, MAX17050_T_GAIN, 0xE932);
	max17050_write_reg(max17050_i2c_client, MAX17050_T_OFF, 0x2381);

	/*Termination Current 150mA*/
	max17050_write_reg(max17050_i2c_client, MAX17050_I_CHG_TERM, 0x03C0);
	max17050_i2c_write_and_verify(MAX17050_V_EMPTY, 0xACDA);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_00, 0x9680);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_10, 0x3800);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_20, 0x3805);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_30, 0x2513);

	/*11. Update Full Capacity Parameters*/
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
	max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);

	/*13. Delay at least 350mS*/
	msleep(360);

	/*14. Write VFSOC value to VFSOC 0*/
	vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
	pr_debug("[MAX17050] %s()  vfsoc = 0x%X\n", __func__, vfsoc);
	max17050_write_reg(max17050_i2c_client, 0x60, 0x0080);
	max17050_i2c_write_and_verify(0x48, vfsoc);
	max17050_write_reg(max17050_i2c_client, 0x60, 0x0000);

	/*15. Write temperature (default 20 deg C)*/
	max17050_i2c_write_and_verify(MAX17050_TEMPERATURE, 0x1400);

	/*16. Load New Capacity Parameters*/
	full_cap_0 =  max17050_read_reg(max17050_i2c_client, 0x35);
	pr_debug("[MAX17050] %s()  full_cap_0 = %d  = 0x%X\n",
			__func__, full_cap_0, full_cap_0);
	rem_cap = (vfsoc * full_cap_0) / 25600;
	pr_debug("[MAX17050] %s()  rem_cap = %d  = 0x%X\n",
			__func__, rem_cap, rem_cap);
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
	rep_cap = rem_cap;
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
	dQ_acc = (capacity / 4);
	max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
	max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x3200);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
	max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);
	max17050_write_reg(max17050_i2c_client, MAX17050_SOC_REP, vfsoc);

	/*17. Initialization Complete*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
	max17050_i2c_write_and_verify(MAX17050_STATUS, (read_reg & 0xFFFD));

	/*End of the Custom Model step 1.*/
	pr_debug("[MAX17050] End of the max17050_new_custom_model_write.\n");

	pr_debug("[MAX17050] %s()  End\n", __func__);
	return 0; /*Success to write.*/

}

int max17050_battery_exchange_program(void)
{
	int ret = 0;

	/*Call max17050_new_custom_model_write*/
	ret = max17050_force_custom_model_write();

	if (ret == 2)
		pr_debug("NON-POWER_ON reset. Proceed Booting.\n");
	/*Error occurred. Write once more.*/
	else if (ret == 1)
		max17050_force_custom_model_write();
	/*New Custom model write End.
	Go to max17050_restore_bat_info_from_flash.*/
	else if (ret == 0)
		pr_debug("POWER_ON reset. Custom Model Success.\n");

	return 0;

}

bool max17050_quick_start(void)
{
	u16 read_reg;
	u16 write_reg;
	u16 check_reg;
	u16 capacity = 0x17B6;
	/*u16 capacity = 0x1802;*/

	/*1. Set the QuickStart and Verify bits*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg | 0x1400;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);

	/*2. Verify no memory leaks during Quickstart writing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	check_reg = read_reg & 0x1000;
	if (check_reg != 0x1000) {
		pr_debug(" [MAX17050] quick_start error !!!!\n");
		return 1;
	}

	/*3. Clean the Verify bit*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg & 0xefff;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);

	/*4. Verify no memory leaks during Verify bit clearing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	check_reg = read_reg & 0x1000;
	if (check_reg != 0x0000) {
		pr_debug(" [MAX17050] quick_start error !!!!\n");
		return 1;
	}
	/*5. Delay 500ms*/
	msleep(500);

	/*6. Writing and Verify FullCAP Register Value*/
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);

	/*7. Delay 500ms*/
	msleep(500);

	return 0;

}

static int max17050_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17050_chip *chip;

	int ret = 0;
	pr_debug("[MAX17050] %s()  Start\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	if (!(&client->dev.of_node)) {
		pr_debug("max17050_probe of_node err.\n");
		goto error;
	}

	i2c_set_clientdata(client, chip);

	max17050_i2c_client = client;

	/*Call max17050_new_custom_model_write*/
	ret = max17050_new_custom_model_write();

	if (ret == 2)
		pr_debug("NON-POWER_ON reset. Proceed Booting.\n");
	/*Error occurred. Write once more.*/
	else if (ret == 1)
		max17050_new_custom_model_write();
	/*New Custom model write End.
	Go to max17050_restore_bat_info_from_flash.*/
	else if (ret == 0)
		pr_debug("POWER_ON reset. Custom Model Success.\n");

	pr_debug("[MAX17050] %s()  End\n", __func__);

	INIT_DELAYED_WORK(&chip->max17050_monitor_work, max17050_monitor_work);
	schedule_delayed_work(&chip->max17050_monitor_work, 0);


	return 0;
error:
	kfree(chip);
	return ret;
}

static int max17050_remove(struct i2c_client *client)
{
	struct max17050_chip *chip = i2c_get_clientdata(client);

	/*power_supply_unregister(&chip->battery);*/
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static struct of_device_id max17050_match_table[] = {
	{ .compatible = "maxim,max17050", },
	{ },
};

static const struct i2c_device_id max17050_id[] = {
	{ "max17050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17050_id);

static struct i2c_driver max17050_i2c_driver = {
	.driver	= {
		.name	= "max17050",
		.owner = THIS_MODULE,
		.of_match_table = max17050_match_table,
	},
	.probe		= max17050_probe,
	.remove		= max17050_remove,
	.id_table	= max17050_id,
};

static int __init max17050_init(void)
{
	return i2c_add_driver(&max17050_i2c_driver);
}
module_init(max17050_init);

static void __exit max17050_exit(void)
{
	i2c_del_driver(&max17050_i2c_driver);
}
module_exit(max17050_exit);

MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("MAX17050 Fuel Gauge");
MODULE_LICENSE("GPL");

