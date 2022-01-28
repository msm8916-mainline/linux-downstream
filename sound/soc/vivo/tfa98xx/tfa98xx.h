/*
 * Copyright (C) NXP Semiconductors (PLMA)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef CONFIG_SND_SOC_TFA98XX_DEBUG
#define CONFIG_SND_SOC_TFA98XX_DEBUG
#endif

#ifndef _TFA98XX_H
#define _TFA98XX_H

#include <sound/core.h>
#include <sound/soc.h>

/* Revision IDs for tfa98xx variants */
#define REV_TFA9887	0x12
#define REV_TFA9890	0x80
#define REV_TFA9895	0x12
#define REV_TFA9897	0x97


struct tfaprofile;
struct nxpTfaDevice;
struct nxpTfaProfile;
struct nxpTfaVolumeStep2File;

struct tfaprofile {
	struct nxpTfaProfile *profile;
	struct nxpTfaVolumeStep2File *vp;
	int vsteps;
	int vstep;
	int index;
	int state;
	char *name;
};

struct tfa98xx_firmware {
	void			*base;
	struct nxpTfaDevice	*dev;
	char			*name;
};

struct tfa98xx_para{
	int		imped_min;
	int 	imped_max;
	int 	fres_min;
	int		fres_max;
};

struct tfa98xx {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct mutex dsp_init_lock;
	int dsp_init;
	int sysclk;
	u8 rev;
	u8 subrev;
	int vstep_ctl;
	int vstep_current;
	int profile_ctl;
	int profile_current;
	int profile_count;
	int mtp_backup;
	int has_drc;
	int rate;
	struct tfaprofile *profiles;
	struct tfa98xx_firmware fw;
	u8 reg;
#ifdef CONFIG_SND_SOC_TFA98XX_DEBUG
	int dsp_msg_retries;
#endif
	int imped_val;
	int calibration;
	int speaker_on;
	const char *name;
	struct tfa98xx_para para;
	unsigned int tfa98xx_rst_gpio;
	struct regulator *tfa98xx_vdd_regulator; //qbq add for enable vdd of tfa98xx
};

#endif
