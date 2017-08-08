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
#define pr_fmt(fmt)	"%s: " fmt, __func__

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

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#ifdef CONFIG_64BIT
#include <soc/qcom/lge/board_lge.h>
#else
#include <mach/board_lge.h>
#endif
#endif

#ifdef CONFIG_LGE_PM_MAX17050_POLLING
#define MAX17050_POLLING_PERIOD_20	20000
#define MAX17050_POLLING_PERIOD_10	10000
#define MAX17050_POLLING_PERIOD_5	5000
#endif

/* Factory cable type */
#define LT_CABLE_56K		6
#define LT_CABLE_130K		7
#define LT_CABLE_910K		11

/* Status register bits */
#define STATUS_POR_BIT		(1 << 1)
#define STATUS_BI_BIT		(1 << 11)
#define STATUS_BR_BIT		(1 << 15)

/* Interrupt config/status bits */
#define CFG_ALRT_BIT_ENBL	(1 << 2)
#define CFG_EXT_TEMP_BIT	(1 << 8)
#define STATUS_INTR_SOCMIN_BIT	(1 << 10)
#define STATUS_INTR_SOCMAX_BIT	(1 << 14)

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
	struct work_struct work;
	struct mutex mutex;
	struct delayed_work	max17050_monitor_work;

	bool suspended;
	bool init_done;
	bool use_ext_temp;

	int alert_gpio;
	int soc_rep;
	int soc_vf;
	int prev_soc;
	int last_soc;

/* test debug */
	int before_soc_rep;
	int before_soc_vf;
	int avg_ibatt;
};

static struct max17050_chip *ref;
/* 130411 junnyoung.jang@lge.com Implement Power test SOC quickstart */
static unsigned int cable_smem_size;
int lge_power_test_flag_max17050 = 1;
int lge_power_init_flag_max17050 = 0;

/* Battery Profile by Cell */
int cell_info = 0;
/* Default Rescale soc & factor */
int rescale_soc = 9300;
int rescale_factor = 0;

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

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

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
		pr_err("i2c NULL vbatt = 800 mV\n");
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_debug("vbatt = %d mV\n", vbatt_mv);

	return vbatt_mv;

}

int max17050_get_ocv_mvolts(void)
{
	u16 read_reg;
	int ocv_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_err("i2c NULL vbatt = 800 mV\n");
		return 800;
	}
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_FOCV);
	if (read_reg < 0)
		return 800;

	ocv_mv = (read_reg >> 4);
	ocv_mv = (ocv_mv * 125) / 100;

	pr_debug("ocv = %d mV : 0x%X\n", ocv_mv, read_reg);

	return ocv_mv;

}

int max17050_suspend_get_mvolts(void)
{
	u16 read_reg;
	int vbatt_mv;

	/*if (max17050_nobattery)
		return 3950;*/
	if (max17050_i2c_client == NULL) {
		pr_err("i2c NULL vbatt = 3950 mV\n");
		return 800;
	}

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_V_CELL);
	if (read_reg < 0)
		return 800;

	vbatt_mv = (read_reg >> 3);
	vbatt_mv = (vbatt_mv * 625) / 1000;

	pr_debug("vbatt = %d mV\n", vbatt_mv);

	return vbatt_mv;

}

int max17050_get_capacity_percent(void)
{
	int battery_soc = 0;
	int battery_soc_vf = 0;
	int read_reg = 0;
	int read_reg_vf = 0;

	u8 upper_reg;
	u8 lower_reg;
	u8 upper_reg_vf;
	u8 lower_reg_vf;
	if (max17050_i2c_client == NULL) {
		return 80;
	} else {
		/* change current base(SOC_REP) to voltage base(SOC_VF) */
		read_reg = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_REP);

		read_reg_vf = max17050_read_reg(max17050_i2c_client,
			MAX17050_SOC_VF);

		if (read_reg < 0) {
			pr_err("i2c Read Fail battery SOC = %d\n", pre_soc);
			return pre_soc;
		}

		if (read_reg_vf < 0) {
			pr_err("i2c Read Fail battery SOC = %d\n", pre_soc);
			return pre_soc;
		}

		upper_reg = (read_reg & 0xFF00)>>8;
		lower_reg = (read_reg & 0xFF);

		pr_debug("SOC_REP : read_reg = %X  upper_reg = %X lower_reg = %X\n",
			read_reg, upper_reg, lower_reg);

		upper_reg_vf = (read_reg_vf & 0xFF00)>>8;
		lower_reg_vf = (read_reg_vf & 0xFF);

		pr_debug("SOC_VF : read_reg = %X  upper_reg = %X lower_reg = %X\n",
			read_reg_vf, upper_reg_vf, lower_reg_vf);

		pr_debug("SOC_REP : read_reg = %X : SOC_VF : read_reg = %X \n",
			read_reg, read_reg_vf);

		/* SOC scaling for stable max SOC and changed Cut-off */
		/* Adj SOC = (FG SOC-Emply)/(Full-Empty)*100 */
		/* cut off vol 3.48V : (soc - 1.132%)*100/(94.28% - 1.132%) */
		/* full capacity soc 106.5% , real battery SoC 100.7% */
		battery_soc = ((upper_reg * 256)+lower_reg)*10/256;

		battery_soc_vf = ((upper_reg_vf * 256)+lower_reg_vf)*10/256;

		/* test debug */
		ref->before_soc_rep = battery_soc;
		ref->before_soc_vf = battery_soc_vf;

		pr_debug("Before_rescailing SOC_VF = %d : SOC_REP = %d\n",
			battery_soc_vf, battery_soc);

		battery_soc = (battery_soc * 100) * 100;
		battery_soc = (battery_soc / (ref->pdata->rescale_soc))
				- (ref->pdata->rescale_factor);
		/* 106.9% scailing */

		/* Cutoff SoC different By Cell */
		if (((battery_soc/10) < 1) && ((battery_soc%10) >= 1))
				battery_soc = 10;

		battery_soc /= 10;

		battery_soc_vf = (battery_soc_vf * 100) * 100;
		battery_soc_vf = (battery_soc_vf / (ref->pdata->rescale_soc))
				- (ref->pdata->rescale_factor);
		/* 106.9% scailing */

		if (((battery_soc_vf/10) < 1) && ((battery_soc_vf%10) >= 1))
				battery_soc_vf = 10;

		battery_soc_vf /= 10;

		pr_debug("Rescale_soc = %d : Rescale_factor = %d\n",
			ref->pdata->rescale_soc, ref->pdata->rescale_factor);

		pr_debug("After_rescailing battery_soc  = %d (upper_reg = %d lower_reg = %d)\n",
			battery_soc, upper_reg, lower_reg);

		pr_debug("After_rescailing battery_soc_vf  = %d (upper_reg_vf = %d lower_reg_vf = %d)\n",
			battery_soc_vf, upper_reg_vf, lower_reg_vf);

		pr_debug("After_rescailing SOC_VF  = %d : SOC_REP  = %d\n",
			battery_soc_vf, battery_soc);

		ref->soc_rep = battery_soc;
		ref->soc_vf = battery_soc_vf;

#ifdef CONFIG_LGE_PM_MAX17050_SOC_REP
		real_soc = battery_soc;
		if (battery_soc >= 100)
			battery_soc = 100;
		if (battery_soc < 0)
			battery_soc = 0;
	}
	pre_soc = battery_soc;
	return battery_soc;
#endif
#ifdef CONFIG_LGE_PM_MAX17050_SOC_VF
		real_soc = battery_soc_vf;
		if (battery_soc_vf >= 100)
			battery_soc_vf = 100;
		if (battery_soc_vf < 0)
			battery_soc_vf = 0;
	}
	pre_soc = battery_soc_vf;
	return battery_soc_vf;
#endif
}

int max17050_get_current(void)
{
	u16 read_reg;
	int ibatt_ma;
	int avg_ibatt_ma;
	u16 sign_bit;

	if (max17050_i2c_client == NULL) {
		pr_err("i2c NULL \n");
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

	/* reverse (charging is negative by convention) */
	ibatt_ma *= -1;

	read_reg = max17050_read_reg(max17050_i2c_client,
		MAX17050_AVERAGE_CURRENT);
	if (read_reg < 0)
		return 999;/*Error Value return.*/

	sign_bit = (read_reg & 0x8000)>>15;

	if (sign_bit == 1)
		avg_ibatt_ma = (15625 * (read_reg  - 65536)) / 100000;
	else
		avg_ibatt_ma = (15625 * read_reg) / 100000;

	/* reverse (charging is negative by convention) */
	avg_ibatt_ma *= -1;

	ref->avg_ibatt = avg_ibatt_ma;

	pr_debug("I_batt = %d mA avg_I_batt = %d mA\n",
		ibatt_ma, avg_ibatt_ma);

	return ibatt_ma;

}

#define DEFAULT_TEMP	25
int max17050_write_temp(void)
{
	int battery_temp;
	int batt_temp_raw;

	u16 read_reg;
	u16 write_temp;

	union power_supply_propval val = {0,};

	if (ref->use_ext_temp) {
		ref->batt_psy->get_property(ref->batt_psy,
					POWER_SUPPLY_PROP_TEMP,&val);

		batt_temp_raw = val.intval;
		pr_debug("batt_temp_raw from power_supply : %d\n", batt_temp_raw);

		if(batt_temp_raw < 0) /*Negative temperature*/
			write_temp = 0xFF00 & (u16)(batt_temp_raw * 256 / 10);
		else /*Positive temperature*/
			write_temp = (u16)(batt_temp_raw * 256 / 10);

		max17050_write_reg(max17050_i2c_client, MAX17050_TEMPERATURE, write_temp);

		/*At least 3mS of delay added between Write and Read functions*/
		msleep(4);

		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_TEMPERATURE);
		pr_debug("battery_temp %X\n", read_reg);

		if (batt_temp_raw < 0) {
			battery_temp = batt_temp_raw / 10;
		} else {
			battery_temp = (read_reg * 10 / 256) / 10;
		}

	} else {
		pr_debug("Not use external batt temp : default batt_temp 25C\n");
		battery_temp = DEFAULT_TEMP;
	}

	pr_debug("battery_temp %d\n", battery_temp);

	return battery_temp;
}

int max17050_read_battery_age(void)
{
	u16 read_reg;
	int battery_age;

	if (max17050_i2c_client == NULL) {
		pr_err("MAX17050] %s: i2c NULL battery age: 800\n", __func__);
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
	int battery_voltage_ocv;

	battery_age = max17050_read_battery_age();

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_REM_CAP_REP);

	battery_remain_capacity = (5 * read_reg)/10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_FULL_CAP);

	battery_full_cap = (5 * read_reg)/10;

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_TTE);

	battery_time_to_empty_sec = (5625 * read_reg)/1000;

	battery_soc = max17050_get_capacity_percent();

	ref->last_soc = battery_soc;

	battery_voltage = max17050_get_mvolts();

	battery_voltage_ocv = max17050_get_ocv_mvolts();

	battery_temp = max17050_write_temp();

	battery_current = max17050_get_current();

	pr_debug("Full_capacity = %d mAh ( remain %d mAh )  Time_to_Empty = %d min\n",
		battery_full_cap, battery_remain_capacity, battery_time_to_empty_sec/60);
	pr_debug("I_batt = %d (avg %d)mA V_batt = %d mV OCV = %d mV batt_TEMP_raw = %d C\n",
		battery_current, ref->avg_ibatt, battery_voltage,
		battery_voltage_ocv, battery_temp);
#ifdef CONFIG_LGE_PM_MAX17050_SOC_REP
	pr_debug("Report SOC = %d %% RawSOC_REP = %d %% ( %d ) SOC_VF = %d %% ( %d )\n",
		battery_soc, ref->soc_rep, ref->before_soc_rep, ref->soc_vf, ref->before_soc_vf);
#endif
#ifdef CONFIG_LGE_PM_MAX17050_SOC_VF
	pr_debug("Report SOC = %d %% RawSOC_VF = %d %% ( %d ) SOC_REP = %d %% ( %d )\n",
		battery_soc, ref->soc_vf, ref->before_soc_vf, ref->soc_rep, ref->before_soc_rep);
#endif

	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_RCOMP_0);

	pr_info("F_Cap : %d ( %d ), T_Emp : %d, I_batt : %d ( %d ), V_batt : %d, OCV: %d,\
		TEMP : %d, SOC : %d, RawSOC_REP : %d ( %d ), SOC_VF : %d ( %d ),\
		RCOMP : 0x%X\n",
		battery_full_cap, battery_remain_capacity, battery_time_to_empty_sec/60,
		battery_current, ref->avg_ibatt, battery_voltage, battery_voltage_ocv,
		battery_temp, battery_soc, ref->soc_rep, ref->before_soc_rep,
		ref->soc_vf, ref->before_soc_vf, read_reg);

	return 0;
}

static void max17050_monitor_work(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip,
				max17050_monitor_work.work);

	max17050_battery_full_info_print();

#ifdef CONFIG_LGE_PM_MAX17050_POLLING
	if (chip->prev_soc != chip->last_soc) {
		pr_info("Update batt_psy\n");
		power_supply_changed(chip->batt_psy);
	}
	pr_info("prev_soc:%d, last_soc:%d\n", chip->prev_soc, chip->last_soc);

	chip->prev_soc = chip->last_soc;

	if (8 <= chip->last_soc) {
		/* 8% more 20sec polling */
		schedule_delayed_work(&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_20));
	} else if (3 < chip->last_soc && chip->last_soc < 8) {
		/* 4%~7% 10sec polling */
		schedule_delayed_work(&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_10));
	} else {
		/* 0%~3% 5sec polling */
		schedule_delayed_work(&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_5));
	}
#else
		schedule_delayed_work(&chip->max17050_monitor_work,
				msecs_to_jiffies(MAX17050_POLLING_PERIOD_20));
#endif
}

bool max17050_i2c_write_and_verify(u8 addr, u16 value)
{
	u16 read_reg;

	max17050_write_reg(max17050_i2c_client, addr, value);
	/*Delay at least 3mS*/
	msleep(4);
	read_reg = max17050_read_reg(max17050_i2c_client, addr);

	if (read_reg == value) {
		pr_debug("Addr = 0x%X  Value = 0x%X  Success\n", addr, value);
		return 1;
	} else {
		pr_debug("Addr = 0x%X  Value = 0x%X  Fail to write.", addr, value);
		pr_debug(" Write once more.\n");
		max17050_write_reg(max17050_i2c_client, addr, value);
		return 0;
	}

	return 1;
}

#ifdef CONFIG_LGE_PM
static void external_charger_enable(bool enable)
{
	union power_supply_propval val = {0,};
	val.intval = enable;
	ref->batt_psy->set_property(ref->batt_psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
}
#endif

static int max17050_new_custom_model_write(void)
{
	/*u16 ret;*/
	u16 read_reg;
	/*u16 write_reg;*/
	u16 vfsoc;
	u16 rem_cap;
	u16 rep_cap;
	u16 dQ_acc;
	u16 qh_register;

	u16 i;

	u8 read_custom_model_80[MODEL_SIZE];
	u8 read_custom_model_90[MODEL_SIZE];
	u8 read_custom_model_A0[MODEL_SIZE];

	pr_info("Model_data Start\n");

	/*0. Check for POR or Battery Insertion*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
	pr_info("MAX17050_STATUS = 0x%X\n", read_reg);
	if ((read_reg & (STATUS_POR_BIT | STATUS_BI_BIT)) == 0) {
		pr_info("NON-POWER_ON reset. Proceed Booting.\n");
		return 2; /*Go to save the custom model.*/
	} else {
		pr_info("POWER_ON Reset state. Start Custom Model Write.\n");

		/*1. Delay 500mS*/
		msleep(500);

		/*1.1 Version Check*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_VERSION);
		pr_info("MAX17050_VERSION = 0x%X\n", read_reg);
		if (read_reg != 0xAC) {
			pr_err("[MAX17050]  Version Check Error.");
			pr_err(" Version Check = 0x%x\n", read_reg);
			return 1; /*Version Check Error*/
		}

		/*2. Initialize Configuration*/
		/* External temp and enable alert function , temp only = 0x2100 */
		max17050_write_reg(max17050_i2c_client, MAX17050_CONFIG,
							ref->pdata->config);
		max17050_write_reg(max17050_i2c_client, MAX17050_FILTER_CFG,
							ref->pdata->filtercfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_RELAX_CFG,
							ref->pdata->relaxcfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_LEARN_CFG,
							ref->pdata->learncfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG,
							ref->pdata->misccfg);
		max17050_write_reg(max17050_i2c_client, MAX17050_FULL_SOC_THR,
							ref->pdata->fullsocthr);
		max17050_write_reg(max17050_i2c_client, MAX17050_I_AVG_EMPTY,
							ref->pdata->iavg_empty);

		/*4. Unlock Model Access*/
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0x59);
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0xc4);

		/*5. Write/Read/Verify the Custom Model*/
		max17050_multi_write_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_80, ref->pdata->model_80, MODEL_SIZE);
		max17050_multi_write_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_90, ref->pdata->model_90, MODEL_SIZE);
		max17050_multi_write_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_A0, ref->pdata->model_A0, MODEL_SIZE);

		/*For test only. Read back written-custom model data.*/
		max17050_multi_read_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_80, read_custom_model_80, MODEL_SIZE);
		max17050_multi_read_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_90, read_custom_model_90, MODEL_SIZE);
		max17050_multi_read_data(max17050_i2c_client,
				MAX17050_MODEL_TABLE_A0, read_custom_model_A0, MODEL_SIZE);

		/*Print read_custom_model print */
		for (i = 0; i < MODEL_SIZE; i++) {
			pr_debug("Model_data_80 %d = 0x%x\n", i, read_custom_model_80[i]);
		}
		for (i = 0; i < MODEL_SIZE; i++) {
			pr_debug("Model_data_90 %d = 0x%x\n", i, read_custom_model_90[i]);
		}
		for (i = 0; i < MODEL_SIZE; i++) {
			pr_debug("Model_data_A0 %d = 0x%x\n", i, read_custom_model_A0[i]);
		}

		/*Compare with original one.*/
		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_80[i] != ref->pdata->model_80[i]) {
				pr_info("[MAX17050] Custom Model");
				pr_info(" 1[%d]	Write Error\n", i);
			}
		}
		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_90[i] != ref->pdata->model_90[i]) {
				pr_info("[MAX17050] Custom Model");
				pr_info(" 2[%d]	Write Error\n", i);
			}
		}
		for (i = 0 ; i < MODEL_SIZE ; i++) {
			if (read_custom_model_A0[i] != ref->pdata->model_A0[i]) {
				pr_info("[MAX17050] Custom Model");
				pr_info(" 3[%d] Write Error\n", i);
			}
		}

		/*8. Lock Model Access*/
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK1, 0);
		max17050_write_reg(max17050_i2c_client, MAX17050_MODEL_LOCK2, 0);

		/*9. Verify the Model Access is locked.*/
		/*Skip.*/

		/*10. Write Custom Parameters*/
		max17050_i2c_write_and_verify(MAX17050_RCOMP_0,
							ref->pdata->rcomp0);
		max17050_i2c_write_and_verify(MAX17050_TEMP_CO,
							ref->pdata->tempco);
		max17050_i2c_write_and_verify(MAX17050_TEMP_NOM,
							ref->pdata->tempnom);
		max17050_i2c_write_and_verify(MAX17050_I_CHG_TERM,
							ref->pdata->ichgterm);
		max17050_i2c_write_and_verify(MAX17050_T_GAIN,
							ref->pdata->tgain);
		max17050_i2c_write_and_verify(MAX17050_T_OFF,
							ref->pdata->toff);
		max17050_i2c_write_and_verify(MAX17050_V_EMPTY,
							ref->pdata->vempty);
		max17050_i2c_write_and_verify(MAX17050_QRTABLE00,
							ref->pdata->qrtable00);
		max17050_i2c_write_and_verify(MAX17050_QRTABLE10,
							ref->pdata->qrtable10);
		max17050_i2c_write_and_verify(MAX17050_QRTABLE20,
							ref->pdata->qrtable20);
		max17050_i2c_write_and_verify(MAX17050_QRTABLE30,
							ref->pdata->qrtable30);

		/*11. Update Full Capacity Parameters*/
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
							ref->pdata->capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
							ref->pdata->vf_fullcap);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
							ref->pdata->vf_fullcap);

		/*13. Delay at least 350mS*/
		msleep(360);

		/*14. Write VFSOC value to VFSOC 0 and QH0*/
		vfsoc = max17050_read_reg(max17050_i2c_client, MAX17050_SOC_VF);
		pr_info("vfsoc = 0x%X\n", vfsoc);
		max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0x0080);
		max17050_i2c_write_and_verify(MAX17050_VFSOC0, vfsoc);
		qh_register = max17050_read_reg(max17050_i2c_client, MAX17050_QH);
		max17050_write_reg(max17050_i2c_client, MAX17050_QH0, qh_register);
		max17050_write_reg(max17050_i2c_client, MAX17050_VFSOC0_LOCK, 0);

		/*15. Advance to Coulomb-Counter Mode */
		max17050_i2c_write_and_verify(MAX17050_CYCLES, 0x0060);

		/*16. Load New Capacity Parameters*/
		rem_cap = (vfsoc * ref->pdata->vf_fullcap) / 25600;
		pr_info("rem_cap = %d  = 0x%X\n", rem_cap, rem_cap);
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_MIX, rem_cap);
		rep_cap = rem_cap;
		max17050_i2c_write_and_verify(MAX17050_REM_CAP_REP, rep_cap);
		dQ_acc = (ref->pdata->vf_fullcap / 16);
		max17050_i2c_write_and_verify(MAX17050_D_PACC, 0x0C80);
		max17050_i2c_write_and_verify(MAX17050_D_QACC, dQ_acc);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
							ref->pdata->capacity);
		max17050_write_reg(max17050_i2c_client, MAX17050_DESIGN_CAP,
							ref->pdata->vf_fullcap);
		max17050_i2c_write_and_verify(MAX17050_FULL_CAP_NOM,
							ref->pdata->vf_fullcap);
		max17050_write_reg(max17050_i2c_client, MAX17050_SOC_REP, vfsoc);

		/*17. Initialization Complete*/
		read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_STATUS);
		max17050_i2c_write_and_verify(MAX17050_STATUS, (read_reg & 0xFFFD));

		/*End of the Custom Model step 1.*/
		pr_info("[MAX17050] End of the max17050_new_custom_model_write.\n");

		pr_info("Model_data End\n");
	}
		return 0; /*Success to write.*/
}

#define MAX17050_QUICKSTART_VERIFY_CNT 2
bool max17050_quick_start(void)
{
	u16 read_reg;
	u16 write_reg;
	u16 check_reg;

	int retry_cnt = 0;

	pr_info("[MAX17050] quick_start\n");

QUICK_STEP1:
	/*1. Set the QuickStart and Verify bits*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg | 0x1400;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);

	/*2. Verify no memory leaks during Quickstart writing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	check_reg = read_reg & 0x1000;
	if (check_reg != 0x1000) {
		if (retry_cnt <= MAX17050_QUICKSTART_VERIFY_CNT) {
			pr_err("[MAX17050] quick_start error STEP2 retry:%d\n", ++retry_cnt);
			goto QUICK_STEP1;
		} else {
			pr_err("[MAX17050] quick_start error !!!!\n");
			return 1;
		}
	}
	retry_cnt = 0;

QUICK_STEP3:
	/*3. Clean the Verify bit*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	write_reg = read_reg & 0xefff;
	max17050_write_reg(max17050_i2c_client, MAX17050_MISC_CFG, write_reg);

	/*4. Verify no memory leaks during Verify bit clearing*/
	read_reg = max17050_read_reg(max17050_i2c_client, MAX17050_MISC_CFG);
	check_reg = read_reg & 0x1000;
	if (check_reg != 0x0000) {
		if (retry_cnt <= MAX17050_QUICKSTART_VERIFY_CNT) {
			pr_err("[MAX17050] quick_start error STEP4 retry:%d\n", ++retry_cnt);
			goto QUICK_STEP3;
		} else {
			pr_err("[MAX17050] quick_start error !!!!\n");
			return 1;
		}
	}

	/*5. Delay 500ms*/
	msleep(500);

	/*6. Writing and Verify FullCAP Register Value*/
	max17050_i2c_write_and_verify(MAX17050_FULL_CAP,
						ref->pdata->capacity);

	/*7. Delay 500ms*/
	msleep(500);
	pr_info("[MAX17050] quick_start done\n");

	return 0;
}

int max17050_get_soc_for_charging_complete_at_cmd(void)
{

	int guage_level = 0;

	/* Reduce charger source */
	external_charger_enable(0);

	pr_info(" [AT_CMD][at_fuel_guage_level_show] max17050_quick_start\n");
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

	/* Restore charger source */
	external_charger_enable(1);

	pr_info(" [AT_CMD][at_fuel_guage_soc_for_charging_complete]");
	pr_info(" BATT guage_level = %d ,", guage_level);
	pr_info(" real_soc = %d\n", real_soc);

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

	if(lge_power_init_flag_max17050 == 1)
	{
		max17050_quick_start();
		lge_power_init_flag_max17050 = 0;
	}

	return max17050_get_capacity_percent();
}
EXPORT_SYMBOL(max17050_get_battery_capacity_percent);

int max17050_get_battery_current(void)
{
	return max17050_get_current();
}
EXPORT_SYMBOL(max17050_get_battery_current);

int max17050_write_battery_temp(void)
{
	return max17050_write_temp();
}
EXPORT_SYMBOL(max17050_write_battery_temp);

int max17050_get_battery_age(void)
{
	return max17050_read_battery_age();
}
EXPORT_SYMBOL(max17050_get_battery_age);

int max17050_get_fulldesign(void)
{
	/* if fuel gauge is not initialized, */
	if (ref == NULL) {
		return 2000;
	}
	return ref->pdata->full_design;
}
EXPORT_SYMBOL(max17050_get_fulldesign);

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

	pr_info(" [AT_CMD][at_fuel_guage_reset_show] start\n");

	/* Reduce charger source */
	external_charger_enable(0);

	ret = max17050_set_battery_atcmd(0, 100);  /* Reset the fuel guage IC*/
	if (ret == 1)
		pr_info("at_fuel_guage_reset_show error.\n");

	r = snprintf(buf, PAGE_SIZE, "%d\n", true);
	/*at_cmd_force_control = TRUE;*/

	msleep(100);

	/* Restore charger source */
	external_charger_enable(1);

	pr_info(" [AT_CMD][at_fuel_guage_reset_show] end\n");

	return r;
}

static ssize_t at_fuel_guage_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int guage_level = 0;

	pr_info(" [AT_CMD][at_fuel_guage_level_show] start\n");

	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {

		/* Reduce charger source */
		external_charger_enable(0);

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

		pr_info(" [AT_CMD][at_fuel_guage_level_show] end\n");
		pr_info(" BATT guage_level = %d ,", guage_level);
		pr_info(" real_soc = %d\n", real_soc);

		/* Restore charger source */
		external_charger_enable(1);

		return snprintf(buf, PAGE_SIZE, "%d\n", guage_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	guage_level = max17050_get_capacity_percent();
	pr_info(" [AT_CMD][at_fuel_guage_level_show] end\n");
	pr_info(" not quick start BATT guage_level = %d\n", guage_level);
	r = snprintf(buf, PAGE_SIZE, "%d\n", guage_level);

	return r;
}

static ssize_t at_batt_level_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	int r = 0;
	int battery_level = 0;

	pr_info(" [AT_CMD][at_batt_level_show] start\n");

	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */
	if (lge_power_test_flag_max17050 == 1) {

		/* Reduce charger source */
		external_charger_enable(0);

		max17050_quick_start();
		battery_level =  max17050_get_battery_mvolts();

		/* Restore charger source */
		external_charger_enable(1);

		pr_info(" [AT_CMD][at_batt_level_show] end\n");
		pr_info(" BATT LVL = %d\n", battery_level);

		return snprintf(buf, PAGE_SIZE, "%d\n", battery_level);
	}
	/* 121128 doosan.baek@lge.com Implement Power test SOC quickstart */

	battery_level =  max17050_get_battery_mvolts();
	pr_info(" [AT_CMD][at_batt_level_show] end\n");
	pr_info("not quick start BATT LVL = %d\n", battery_level);

	r = snprintf(buf, PAGE_SIZE, "%d\n", battery_level);

	return r;
}

DEVICE_ATTR(at_fuelrst, 0444, at_fuel_guage_reset_show, NULL);
DEVICE_ATTR(at_fuelval, 0444, at_fuel_guage_level_show, NULL);
DEVICE_ATTR(at_batl, 0444, at_batt_level_show, NULL);

#ifdef CONFIG_LGE_PM
static enum power_supply_property max17050_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};
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
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = max17050_get_battery_current();
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = max17050_get_fulldesign();
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

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

#ifdef CONFIG_LGE_PM_MAX17050_SOC_ALERT
static void max17050_set_soc_thresholds(struct max17050_chip *chip,
								s16 threshold)
{
	s16 soc_now;
	s16 soc_max;
	s16 soc_min;

	soc_now = max17050_read_reg(chip->client, MAX17050_SOC_REP) >> 8;

	pr_info("soc_now %d\n", soc_now);

	soc_max = soc_now + threshold;
	if (soc_max > 100)
		soc_max = 100;
	soc_min = soc_now - threshold;
	if (soc_min < 0)
		soc_min = 0;

	max17050_write_reg(chip->client, MAX17050_SOC_ALRT_THRESHOLD,
		(u16)soc_min | ((u16)soc_max << 8));
}

static irqreturn_t max17050_interrupt(int id, void *dev)
{
	struct max17050_chip *chip = dev;
	struct i2c_client *client = chip->client;
	u16 val;

	pr_info("Interrupt occured, ID = %d\n", id);

	val = max17050_read_reg(client, MAX17050_STATUS);

	/* Signal userspace when the capacity exceeds the limits */
	if ((val & STATUS_INTR_SOCMIN_BIT) || (val & STATUS_INTR_SOCMAX_BIT)) {
		/* Clear interrupt status bits */
		max17050_write_reg(client, MAX17050_STATUS, val &
			~(STATUS_INTR_SOCMIN_BIT | STATUS_INTR_SOCMAX_BIT));

		/* Reset capacity thresholds */
		max17050_set_soc_thresholds(chip, 5);

		power_supply_changed(chip->batt_psy);
	}

	return IRQ_HANDLED;
}

static void max17050_complete_init(struct max17050_chip *chip)
{
	struct i2c_client *client = chip->client;
	int val;

	if (client->irq) {
		/* Set capacity thresholds to +/- 5% of current capacity */
		max17050_set_soc_thresholds(chip, 5);

		/* Enable capacity interrupts */
		val = max17050_read_reg(client, MAX17050_CONFIG);
		max17050_write_reg(client, MAX17050_CONFIG,
						val | CFG_ALRT_BIT_ENBL);
		pr_info("MAX17050_CONFIG_val after write = 0x%X\n", val);
	}

	chip->init_done = true;
}

static void max17050_init_worker(struct work_struct *work)
{
	struct max17050_chip *chip = container_of(work,
				struct max17050_chip, work);

	// max17050_init_chip(chip);
	max17050_complete_init(chip);
}
#endif

static struct max17050_platform_data *
max17050_get_pdata(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 prop;
	struct max17050_platform_data *pdata;

	int i;
	int ret;

	if (!np)
		return dev->platform_data;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

#ifdef CONFIG_LGE_PM_MAX17050_SOC_ALERT
	pdata->alert_gpio = of_get_named_gpio(np, "maxim,alert_gpio", 0);
	pr_err("Get alert_gpio %d\n", pdata->alert_gpio);
#endif

	/*
	 * Require current sense resistor value to be specified for
	 * current-sense functionality to be enabled at all.
	 */
	if (of_property_read_u32(np, "maxim,rsns-microohm", &prop) == 0) {
		pdata->r_sns = prop;
		pdata->enable_current_sense = true;
	} else {
		pdata->r_sns = MAX17050_DEFAULT_SNS_RESISTOR;
	}

	/* Battery Cell Profile */
	if (cell_info == TCD_AAC) {
		ret = of_property_read_u8_array(np, "maxim,model_80_d",
					pdata->model_80, MODEL_SIZE);
		ret = of_property_read_u8_array(np, "maxim,model_90_d",
					pdata->model_90, MODEL_SIZE);
		ret = of_property_read_u8_array(np, "maxim,model_A0_d",
					pdata->model_A0, MODEL_SIZE);
		if (of_property_read_u32(np, "maxim,iavg_empty_d", &prop) == 0)
			pdata->iavg_empty = prop;
		if (of_property_read_u32(np, "maxim,rcomp0_d", &prop) == 0)
			pdata->rcomp0 = prop;
		if (of_property_read_u32(np, "maxim,tempco_d", &prop) == 0)
			pdata->tempco = prop;
		if (of_property_read_u32(np, "maxim,ichgterm_d", &prop) == 0)
			pdata->ichgterm = prop;
		if (of_property_read_u32(np, "maxim,vempty_d", &prop) == 0)
			pdata->vempty = prop;
		if (of_property_read_u32(np, "maxim,qrtable00_d", &prop) == 0)
			pdata->qrtable00 = prop;
		if (of_property_read_u32(np, "maxim,qrtable10_d", &prop) == 0)
			pdata->qrtable10 = prop;
		if (of_property_read_u32(np, "maxim,qrtable20_d", &prop) == 0)
			pdata->qrtable20 = prop;
		if (of_property_read_u32(np, "maxim,qrtable30_d", &prop) == 0)
			pdata->qrtable30 = prop;
		if (of_property_read_u32(np, "maxim,capacity_d", &prop) == 0)
			pdata->capacity = prop;
		if (of_property_read_u32(np, "maxim,vf_fullcap_d", &prop) == 0)
			pdata->vf_fullcap = prop;
		ret = of_property_read_u32(np, "maxim,rescale_factor_d",
			&pdata->rescale_factor);
	} else { /*cell_info == LGC_LLL*/
		ret = of_property_read_u8_array(np, "maxim,model_80_l",
					pdata->model_80, MODEL_SIZE);
		ret = of_property_read_u8_array(np, "maxim,model_90_l",
					pdata->model_90, MODEL_SIZE);
		ret = of_property_read_u8_array(np, "maxim,model_A0_l",
					pdata->model_A0, MODEL_SIZE);
		if (of_property_read_u32(np, "maxim,iavg_empty_l", &prop) == 0)
			pdata->iavg_empty = prop;
		if (of_property_read_u32(np, "maxim,rcomp0_l", &prop) == 0)
			pdata->rcomp0 = prop;
		if (of_property_read_u32(np, "maxim,tempco_l", &prop) == 0)
			pdata->tempco = prop;
		if (of_property_read_u32(np, "maxim,ichgterm_l", &prop) == 0)
			pdata->ichgterm = prop;
		if (of_property_read_u32(np, "maxim,vempty_l", &prop) == 0)
			pdata->vempty = prop;
		if (of_property_read_u32(np, "maxim,qrtable00_l", &prop) == 0)
			pdata->qrtable00 = prop;
		if (of_property_read_u32(np, "maxim,qrtable10_l", &prop) == 0)
			pdata->qrtable10 = prop;
		if (of_property_read_u32(np, "maxim,qrtable20_l", &prop) == 0)
			pdata->qrtable20 = prop;
		if (of_property_read_u32(np, "maxim,qrtable30_l", &prop) == 0)
			pdata->qrtable30 = prop;
		if (of_property_read_u32(np, "maxim,capacity_l", &prop) == 0)
			pdata->capacity = prop;
		if (of_property_read_u32(np, "maxim,vf_fullcap_l", &prop) == 0)
			pdata->vf_fullcap = prop;
		ret = of_property_read_u32(np, "maxim,rescale_factor_l",
			&pdata->rescale_factor);
	}

	if (of_property_read_u32(np, "maxim,relaxcfg", &prop) == 0)
		pdata->relaxcfg = prop;
	if (of_property_read_u32(np, "maxim,config", &prop) == 0)
		pdata->config = prop;
	if (of_property_read_u32(np, "maxim,filtercfg", &prop) == 0)
		pdata->filtercfg = prop;
	if (of_property_read_u32(np, "maxim,learncfg", &prop) == 0)
		pdata->learncfg = prop;
	if (of_property_read_u32(np, "maxim,misccfg", &prop) == 0)
		pdata->misccfg = prop;
	if (of_property_read_u32(np, "maxim,fullsocthr", &prop) == 0)
		pdata->fullsocthr = prop;
	if (of_property_read_u32(np, "maxim,tempnom", &prop) == 0)
		pdata->tempnom = prop;
	if (of_property_read_u32(np, "maxim,tgain", &prop) == 0)
		pdata->tgain = prop;
	if (of_property_read_u32(np, "maxim,toff", &prop) == 0)
		pdata->toff = prop;

	ret = of_property_read_u32(np, "maxim,rescale_soc",
			&pdata->rescale_soc);

	if (of_property_read_u32(np, "maxim,param-version", &prop) == 0)
		pdata->param_version = prop;
	if (of_property_read_u32(np, "maxim,full_design", &prop) == 0)
		pdata->full_design = prop;

	/* Debug log model data by dtsi parsing */
	for (i = 0; i < MODEL_SIZE; i++) {
		pr_debug("Model_data_80 %d = 0x%x\n", i, pdata->model_80[i]);
	}
	for (i = 0; i < MODEL_SIZE; i++) {
		pr_debug("Model_data_90 %d = 0x%x\n", i, pdata->model_90[i]);
	}
	for (i = 0; i < MODEL_SIZE; i++) {
		pr_debug("Model_data_A0 %d = 0x%x\n", i, pdata->model_A0[i]);
	}

	pr_info("Platform data : "\
			"rcomp =0x%X, "\
			"tempco =0x%X, "\
			"ichgterm =0x%X, "\
			"vempty =0x%X, "\
			"full_design =0x%X, "\
			"rescale_soc =%d, "\
			"rescale_factor =%d\n",
			pdata->rcomp0,
			pdata->tempco,
			pdata->ichgterm,
			pdata->vempty,
			pdata->vf_fullcap,
			pdata->rescale_soc,
			pdata->rescale_factor);

	return pdata;
}

static int max17050_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17050_chip *chip;

	int ret = 0;
	int rc = 0;
	int cable_type;
	int64_t batt_id;

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size, 0, 0));

	pr_info("Start\n");

	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	if (cable_type == LT_CABLE_56K || cable_type == LT_CABLE_130K ||
					cable_type == LT_CABLE_910K) {
		lge_power_init_flag_max17050 = 1;
		pr_info("cable_type is = %d factory_mode quick start \n", cable_type);
	}
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		pr_err("batt_psy is not yet ready\n");
		ret = -EPROBE_DEFER;
		goto error;
	}

	batt_id = read_lge_battery_id();
	if (batt_id) {
		if (batt_id == BATT_ID_SW3800_VC0 || batt_id == BATT_ID_RA4301_VC1) {
			cell_info = LGC_LLL;/* LGC Battery */
			pr_info("LGC profile\n");
		} else if (batt_id == BATT_ID_RA4301_VC0 || batt_id == BATT_ID_SW3800_VC1) {
			cell_info = TCD_AAC;/*Sanyo Tocad Battery */
			pr_info("TCD profile\n");
		} else {
			cell_info = LGC_LLL;
			pr_info("Unknown cell, Using LGC profile\n");
		}
	}

	chip->pdata = max17050_get_pdata(&client->dev);
	if (!chip->pdata) {
		pr_err("max17050_probe of_node err.\n");
		goto error;
	}

	i2c_set_clientdata(client, chip);

	max17050_i2c_client = client;

	ref = chip;

	/*Call max17050_new_custom_model_write*/
	ret = max17050_new_custom_model_write();

	if (ret == 2)
		pr_info("NON-POWER_ON reset. Proceed Booting.\n");
	/*Error occurred. Write once more.*/
	else if (ret == 1)
		max17050_new_custom_model_write();
	/*New Custom model write End.
	Go to max17050_restore_bat_info_from_flash.*/
	else if (ret == 0)
		pr_info("POWER_ON reset. Custom Model Success.\n");

	/*Check to enable external battery temperature from CONFIG*/
	chip->use_ext_temp = (chip->pdata->config & CFG_EXT_TEMP_BIT);
	pr_info("use_ext_temp = %d\n", chip->use_ext_temp);

#ifdef CONFIG_LGE_PM_MAX17050_SOC_ALERT
	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL,
					max17050_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					chip->battery.name, chip);
		if (ret) {
			dev_err(&client->dev, "cannot enable irq");
			return ret;
		} else {
			enable_irq_wake(client->irq);
		}
	}

	INIT_WORK(&chip->work, max17050_init_worker);
	mutex_init(&chip->mutex);
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
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
#endif

#ifdef CONFIG_LGE_PM
	chip->battery = max17050_ps;

	rc = power_supply_register(&chip->client->dev, &chip->battery);
	if (rc < 0) {
		pr_err("[2222]batt failed to register rc = %d\n", rc);
	}
#endif

	pr_info("End\n");

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

static int max17050_pm_prepare(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17050_chip *chip = i2c_get_clientdata(client);

	/* Cancel any pending work, if necessary */
	cancel_delayed_work_sync(&ref->max17050_monitor_work);

	chip->suspended = true;
	if (chip->client->irq) {
		disable_irq(chip->client->irq);
		enable_irq_wake(chip->client->irq);
	}
	return 0;
}

static void max17050_pm_complete(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17050_chip *chip = i2c_get_clientdata(client);

	chip->suspended = false;
	if (chip->client->irq) {
		disable_irq_wake(chip->client->irq);
		enable_irq(chip->client->irq);
	}

	/* Schedule update, if needed */
	schedule_delayed_work(&ref->max17050_monitor_work, msecs_to_jiffies(HZ));
}

static const struct dev_pm_ops max17050_pm_ops = {
	.prepare = max17050_pm_prepare,
	.complete = max17050_pm_complete,
};

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
		.pm = &max17050_pm_ops,
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
