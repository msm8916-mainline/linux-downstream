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
#include <linux/power/max17050_battery.h>
#include <linux/delay.h>
#include <linux/module.h>

#ifdef CONFIG_LGE_PM
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <soc/qcom/smem.h>
#endif

#ifdef CONFIG_MACH_LGE
#ifdef CONFIG_64BIT
#include <soc/qcom/lge/board_lge.h>
#else
#include <mach/board_lge.h>
#endif
#endif

static struct i2c_client *max17050_i2c_client;

u16 pre_soc = 100;
u16 real_soc = 100;

struct max17050_chip {
	struct i2c_client *client;
	/*struct power_supply battery;*/
	struct power_supply	*batt_psy;
	struct power_supply 	*ac_psy;
	struct power_supply     battery;
	struct max17050_platform_data *pdata;
	struct delayed_work	max17050_monitor_work;
};
/* 130411 junnyoung.jang@lge.com Implement Power test SOC quickstart */
int lge_power_test_flag_max17050 = 1;
/* 130411 junnyoung.jang@lge.com Implement Power test SOC quickstart */

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
		/* change current base(SOC_REP) to voltage base(SOC_VF) */
		read_reg = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_VF);

		if (read_reg < 0) {
			pr_info("[MAX17050] %s: i2c Read Fail battery SOC = %d\n",
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
		battery_soc = (battery_soc / 9340) - 1; /* 106.8% scailing */

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
/* G2 DCM */
bool max17050_write_temp(int battery_temp)
{
	u16 ret;
	u16 write_temp;

	battery_temp = battery_temp/10;

	if (battery_temp < 0)
		write_temp = (battery_temp + 256)<<8;
	else
		write_temp = (battery_temp)<<8;

	pr_debug("max17050_write_temp   - battery_temp (%d)\n", battery_temp);

	ret = max17050_write_reg(max17050_i2c_client,
		MAX17050_TEMPERATURE, write_temp);

	if (ret < 0) {
		pr_debug("max17050_write_temp error.\n");
		return 1;
	}

	return 0;
}
/* G2 DCM */
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
/* not G2 DCM */
static void max17050_monitor_work(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip,
				max17050_monitor_work.work);

	max17050_battery_full_info_print();

	schedule_delayed_work(&chip->max17050_monitor_work,
			msecs_to_jiffies(20000));
}
/* not G2 DCM */
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
	u16 qh_register;
	u16 capacity = 0x0F35; //VW820 2100mAh profile

	u16 i;

/* VW820 LGC Cell */
	u8 custom_model_1[32] = {
		0xa0, 0x86, 0xb0, 0xb5, 0xf0, 0xb5, 0xe0, 0xb7,
		0xc0, 0xb9, 0x40, 0xbc, 0xe0, 0xbc, 0xb0, 0xbd,
		0xf0, 0xbe, 0x50, 0xc0, 0xe0, 0xc0, 0x10, 0xc3,
		0x00, 0xc6, 0x20, 0xca, 0xd0, 0xcf, 0xb0, 0xd5,
	};

	u8 custom_model_2[32] = {
		0x50, 0x00, 0x00, 0x1a, 0x10, 0x10, 0x00, 0x0c,
		0x00, 0x0c, 0x00, 0x34, 0x30, 0x1f, 0xd0, 0x14,
		0xe0, 0x12, 0xf0, 0x11, 0xc0, 0x07, 0xf0, 0x09,
		0xc0, 0x09, 0xf0, 0x07, 0xe0, 0x06, 0xe0, 0x06,
	};

	u8 custom_model_3[32] = {

		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
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
		/* External temp and enable alert function , temp only = 0x2100 */
		max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG, 0x2104);
		max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG, 0x87A4);
		max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG, 0x506B);
		max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG, 0x2606);
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
		/* VW820 LG Cell define */
		max17050_i2c_write_and_verify(MAX17050_RCOMP_0, 0x00B6);
		max17050_i2c_write_and_verify(MAX17050_TEMP_CO, 0x4771);
		max17050_i2c_write_and_verify(MAX17050_I_CHG_TERM, 0x0500);
		max17050_write_reg(max17050_i2c_client, MAX17050_T_GAIN, 0xE932);
		max17050_write_reg(max17050_i2c_client, MAX17050_T_OFF, 0x2381);

		max17050_i2c_write_and_verify(MAX17050_V_EMPTY, 0xAA56);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_00, 0x5683);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_10, 0x2F80);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_20, 0x1B83);
		max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_30, 0x1402);

		/*11. Update Full Capacity Parameters*/
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);

		/*13. Delay at least 350mS*/
		msleep(360);

		/*14. Write VFSOC value to VFSOC 0 and QH0*/
		vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
		pr_debug("[MAX17050] %s()  vfsoc = 0x%X\n", __func__, vfsoc);
		max17050_write_reg(max17050_i2c_client, 0x60, 0x0080);
		max17050_i2c_write_and_verify(0x48, vfsoc);
		qh_register = max17050_read_reg(max17050_i2c_client, MAX17050_QH);
		max17050_write_reg(max17050_i2c_client, 0x4C, qh_register);
		max17050_write_reg(max17050_i2c_client, 0x60, 0x0000);

		/*15. Advance to Coulomb-Counter Mode */
		max17050_i2c_write_and_verify(MAX17050_CYCLES, 0x0060);

		/*16. Load New Capacity Parameters*/
		full_cap_0 = capacity;
		rem_cap = (vfsoc * full_cap_0) / 25600;
		pr_debug("[MAX17050] %s()  rem_cap = %d  = 0x%X\n",
				__func__, rem_cap, rem_cap);
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
		rep_cap = rem_cap;
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
		dQ_acc = (capacity / 16);
		max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x0C80);
		max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
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
	u16 qh_register;
	u16 capacity = 0x0F35; //VW820 2100mAh profile

	u16 i;

/* VW820 LGC Cell */
	u8 custom_model_1[32] = {
		0xa0, 0x86, 0xb0, 0xb5, 0xf0, 0xb5, 0xe0, 0xb7,
		0xc0, 0xb9, 0x40, 0xbc, 0xe0, 0xbc, 0xb0, 0xbd,
		0xf0, 0xbe, 0x50, 0xc0, 0xe0, 0xc0, 0x10, 0xc3,
		0x00, 0xc6, 0x20, 0xca, 0xd0, 0xcf, 0xb0, 0xd5,
	};

	u8 custom_model_2[32] = {
		0x50, 0x00, 0x00, 0x1a, 0x10, 0x10, 0x00, 0x0c,
		0x00, 0x0c, 0x00, 0x34, 0x30, 0x1f, 0xd0, 0x14,
		0xe0, 0x12, 0xf0, 0x11, 0xc0, 0x07, 0xf0, 0x09,
		0xc0, 0x09, 0xf0, 0x07, 0xe0, 0x06, 0xe0, 0x06,
	};

	u8 custom_model_3[32] = {

		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
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
	} else { /* Step indent level back to syncronize with origin */
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
	/* External temp and enable alert function , temp only = 0x2100 */
	max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG, 0x2104);
	max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG, 0x87A4);
	max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG, 0x506B);
	max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG, 0x2606);
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
	/* VW820 LG Cell define */
	max17050_i2c_write_and_verify(MAX17050_RCOMP_0, 0x00B6);
	max17050_i2c_write_and_verify(MAX17050_TEMP_CO, 0x4771);
	max17050_i2c_write_and_verify(MAX17050_I_CHG_TERM, 0x0500);
	max17050_write_reg(max17050_i2c_client, MAX17050_T_GAIN, 0xE932);
	max17050_write_reg(max17050_i2c_client, MAX17050_T_OFF, 0x2381);

	max17050_i2c_write_and_verify(MAX17050_V_EMPTY, 0xAA56);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_00, 0x5683);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_10, 0x2F80);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_20, 0x1B83);
	max17050_i2c_write_and_verify(MAX17050_Q_RESIDUAL_30, 0x1402);

	/*11. Update Full Capacity Parameters*/
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP, capacity);
	max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP, capacity);
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM, capacity);

	/*13. Delay at least 350mS*/
	msleep(360);

	/*14. Write VFSOC value to VFSOC 0 and QH0*/
	vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
	pr_debug("[MAX17050] %s()  vfsoc = 0x%X\n", __func__, vfsoc);
	max17050_write_reg(max17050_i2c_client, 0x60, 0x0080);
	max17050_i2c_write_and_verify(0x48, vfsoc);
	qh_register = max17050_read_reg(max17050_i2c_client, MAX17050_QH);
	max17050_write_reg(max17050_i2c_client, 0x4C, qh_register);
	max17050_write_reg(max17050_i2c_client, 0x60, 0x0000);

	/*15. Advance to Coulomb-Counter Mode */
	max17050_i2c_write_and_verify(MAX17050_CYCLES, 0x0060);

	/*16. Load New Capacity Parameters*/
	full_cap_0 = capacity;
	rem_cap = (vfsoc * full_cap_0) / 25600;
	pr_debug("[MAX17050] %s()  rem_cap = %d  = 0x%X\n",
			__func__, rem_cap, rem_cap);
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
	rep_cap = rem_cap;
	max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
	dQ_acc = (capacity / 16);
	max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x0C80);
	max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
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
	/* VW820 2100mAh profile */
	u16 capacity = 0x0F35; //VW820 2100mAh profile
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

int max17050_get_soc_for_charging_complete_at_cmd(void)
{

	int guage_level = 0;

/*	pm8921_charger_enable(0);
	pm8921_disable_source_current(1);*/

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* Reduce charger source */
	external_smb349_enable_charging(0);
#endif

	pr_debug(" [AT_CMD][at_fuel_guage_level_show] max17050_quick_start\n");
	max17050_quick_start();
	guage_level = max17050_get_capacity_percent();
	if (guage_level != 0) {
		if (guage_level >= 80)
			guage_level = (real_soc * 962) / 1000;
		else
			guage_level = (guage_level * 100) / 100;
	}

	if (guage_level > 100)
		guage_level = 100;
	else if (guage_level < 0)
		guage_level = 0;

/*	pm8921_disable_source_current(0);
	pm8921_charger_enable(1);*/

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* Restore charger source */
	external_smb349_enable_charging(1);
#endif

	pr_debug(" [AT_CMD][at_fuel_guage_soc_for_charging_complete]");
	pr_debug(" BATT guage_level = %d ,", guage_level);
	pr_debug(" real_soc = %d\n", real_soc);

	return guage_level;
}
EXPORT_SYMBOL(max17050_get_soc_for_charging_complete_at_cmd);

int max17050_get_battery_mvolts(void)
{
	return max17050_get_mvolts();
}
EXPORT_SYMBOL(max17050_get_battery_mvolts);

u8 at_cmd_buf[2] = {0xff, 0xff};
int max17050_get_battery_capacity_percent(void)
{
	if (at_cmd_buf[0] == 1)
		return at_cmd_buf[1];

	return max17050_get_capacity_percent();
}
EXPORT_SYMBOL(max17050_get_battery_capacity_percent);

int max17050_get_battery_current(void)
{
	return max17050_get_current();
}
EXPORT_SYMBOL(max17050_get_battery_current);

bool max17050_write_battery_temp(int battery_temp)
{
	return max17050_write_temp(battery_temp);
}
EXPORT_SYMBOL(max17050_write_battery_temp);


int max17050_get_battery_age(void)
{
	return max17050_read_battery_age();
}
EXPORT_SYMBOL(max17050_get_battery_age);

/* For max17050 AT cmd */
bool max17050_set_battery_atcmd(int flag, int value)
{
	bool ret;

	u16 soc_read;
	u16 vbat_mv;

	if (flag == 0) {
		/*Call max17050 Quick Start function.*/
		ret = max17050_quick_start();

		if (ret == 0) {
			/*Read and Verify Outputs*/
			soc_read = max17050_get_capacity_percent();
			vbat_mv = max17050_suspend_get_mvolts();
			pr_debug("[MAX17050] max17050_quick_start end");
			pr_debug(" Reset_SOC = %d %% vbat = %d mV\n",
				soc_read, vbat_mv);

			if ((vbat_mv >= 4100) && (soc_read < 91)) {
				at_cmd_buf[0] = 1;
				at_cmd_buf[1] = 100;
				pr_debug("[MAX17050] max17050_quick_start error correction works.\n");
				return 1;
			} else
				at_cmd_buf[0] = 0;
		} else {
				at_cmd_buf[0] = 1;
				at_cmd_buf[1] = 100;
				pr_debug("[MAX17050] max17050_quick_start error correction works.\n");
				return 1;
		}
	} else if (flag == 1) {
		at_cmd_buf[0] = 0;
	}

	return 0;
}

static ssize_t at_fuel_guage_reset_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	bool ret = 0;

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* Reduce charger source */
	external_smb349_enable_charging(0);
#endif

	ret = max17050_set_battery_atcmd(0, 100);  /* Reset the fuel guage IC*/
	if (ret == 1)
		pr_debug("at_fuel_guage_reset_show error.\n");

	r = snprintf(buf, PAGE_SIZE, "%d\n", true);
	/*at_cmd_force_control = TRUE;*/

	msleep(100);

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
	/* Restore charger source */
	external_smb349_enable_charging(1);
#endif

	return r;
}

static ssize_t at_fuel_guage_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int guage_level = 0;

	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {
		/*pm8921_charger_enable(0);
		pm8921_disable_source_current(1);*/

		pr_debug(" [AT_CMD][at_fuel_guage_level_show]");
		pr_debug(" max17050_quick_start\n");

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
		/* Reduce charger source */
		external_smb349_enable_charging(0);
#endif

		max17050_quick_start();
		guage_level = max17050_get_capacity_percent();
		if (guage_level != 0) {
			if (guage_level >= 80)
				guage_level = (real_soc * 962) / 1000;
			else
				guage_level = (guage_level * 100) / 100;
		}

		if (guage_level > 100)
			guage_level = 100;
		else if (guage_level < 0)
			guage_level = 0;

		pr_debug(" [AT_CMD][at_fuel_guage_level_show]");
		pr_debug(" BATT guage_level = %d ,", guage_level);
		pr_debug(" real_soc = %d\n", real_soc);

		/*pm8921_disable_source_current(0);
		pm8921_charger_enable(1);*/

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
		/* Restore charger source */
		external_smb349_enable_charging(1);
#endif

		return snprintf(buf, PAGE_SIZE, "%d\n", guage_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	guage_level = max17050_get_capacity_percent();
	pr_debug(" [AT_CMD][at_fuel_guage_level_show]");
	pr_debug(" not quick start BATT guage_level = %d\n", guage_level);
	r = snprintf(buf, PAGE_SIZE, "%d\n", guage_level);

	return r;
}

static ssize_t at_batt_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int battery_level = 0;


	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {
		/*pm8921_charger_enable(0);
		pm8921_disable_source_current(1);*/

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
		/* Reduce charger source */
		external_smb349_enable_charging(0);
#endif

		pr_debug(" [AT_CMD][at_batt_level_show] max17050_quick_start\n");

		max17050_quick_start();
		battery_level =  max17050_get_battery_mvolts();

		/*pm8921_disable_source_current(0);
		pm8921_charger_enable(1);*/

#if defined(CONFIG_MACH_MSM8974_G2_DCM)
		/* Restore charger source */
		external_smb349_enable_charging(1);
#endif

		pr_debug(" [AT_CMD][at_batt_level_show] BATT LVL = %d\n",
				battery_level);

		return snprintf(buf, PAGE_SIZE, "%d\n", battery_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */

	battery_level =  max17050_get_battery_mvolts();
	pr_debug(" [AT_CMD][at_batt_level_show]");
	pr_debug("	not quick start BATT LVL = %d\n", battery_level);

	r = snprintf(buf, PAGE_SIZE, "%d\n", battery_level);

	return r;
}

DEVICE_ATTR(at_fuelrst, 0644, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0644, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_batl, 0644, at_batt_level_show, NULL);

#ifdef CONFIG_LGE_PM
static int max17050_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = max17050_get_battery_mvolts();
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = max17050_get_battery_capacity_percent();
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
static enum power_supply_property max17050_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};
static char *pm_power_supplied_to[] = {
	"battery",
};

static struct power_supply max17050_ps = {
	.name = "fuelgauge",
	.type = POWER_SUPPLY_TYPE_FUELGAUGE/*POWER_SUPPLY_TYPE_MAINS*/,
	.supplied_to = pm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties = max17050_battery_props,
	.num_properties = ARRAY_SIZE(max17050_battery_props),
	.get_property = max17050_get_property,
};

#endif


static int max17050_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17050_chip *chip;

	int ret = 0;
	int rc = 0;
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

	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_fuelrst */
	ret = device_create_file(&client->dev, &dev_attr_at_fuelrst);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_fuelrst_failed;
	}
	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_fuelval */
	ret = device_create_file(&client->dev, &dev_attr_at_fuelval);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_fuelval_failed;
	}
	/* sysfs path : /sys/bus/i2c/drivers/max17050/1-0036/at_batl */
	ret = device_create_file(&client->dev, &dev_attr_at_batl);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_batl_failed;
	}

#ifdef CONFIG_LGE_PM
	chip->battery = max17050_ps;

	rc = power_supply_register(&chip->client->dev, &chip->battery);
	if (rc < 0) {
		pr_err("[2222]batt failed to register rc = %d\n", rc);
	}
#endif


	pr_debug("[MAX17050] %s()  End\n", __func__);

	INIT_DELAYED_WORK(&chip->max17050_monitor_work, max17050_monitor_work);
	schedule_delayed_work(&chip->max17050_monitor_work, 0);


	return 0;
err_create_file_fuelrst_failed:
	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
err_create_file_fuelval_failed:
	device_remove_file(&client->dev, &dev_attr_at_fuelval);
err_create_file_batl_failed:
	device_remove_file(&client->dev, &dev_attr_at_batl);
error:
	kfree(chip);
	return ret;
}

static int max17050_remove(struct i2c_client *client)
{
	struct max17050_chip *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_at_fuelrst);
	device_remove_file(&client->dev, &dev_attr_at_fuelval);
	device_remove_file(&client->dev, &dev_attr_at_batl);

	/*power_supply_unregister(&chip->battery);*/
	i2c_set_clientdata(client, NULL);
#ifdef CONFIG_LGE_PM
	power_supply_unregister(&chip->battery);
#endif
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


