/*
 * MAX14656 USB Charger Detector driver
 *
 * Copyright (C) 2014 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#ifdef CONFIG_LGE_PM
enum max14656_chg_type {
	NO_CHARGER 		 = 0,
	SDP_CHARGER,
	CDP_CHARGER,
	DCP_CHARGER,
	APPLE_500MA_CHARGER,
	APPLE_1A_CHARGER,
	APPLE_2A_CHARGER,
	SPECIAL_500MA_CHARGER,
	APPLE_12W,
};
#endif

struct max14656_chip {
	struct i2c_client 		*client;
	struct power_supply 	*batt_psy;
	struct power_supply 	detect_psy;
	struct delayed_work 	irq_work;

	int irq;
	int int_gpio;
	int chg_detect_done;
	int chg_type;
	int dcd_timeout;
};
