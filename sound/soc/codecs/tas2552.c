/*
 * ALSA SoC Texas Instruments TAS2552 Mono Audio Amplifier
 *
 * Copyright (C) 2014 Texas Instruments Inc.
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/tas2552-plat.h>

#include "tas2552.h"
#define TAS2552_DEBUG

static struct reg_default tas2552_reg_defs[] = {
	{TAS2552_CFG_1, 0x16},//0x01
	{TAS2552_CFG_3, 0x5E},//0x03
	{TAS2552_DOUT, 0x00},//0x04
	{TAS2552_OUTPUT_DATA, 0xC8},//0x07
	{TAS2552_PDM_CFG, 0x02},//0x11
	{TAS2552_PGA_GAIN, 0x10},//0x12
	{TAS2552_BOOST_PT_CTRL, 0x0F},//0x14
	{TAS2552_LIMIT_LVL_CTRL, 0x0C},//0x0d
	{TAS2552_LIMIT_RATE_HYS, 0x20},//0x0e
	{TAS2552_CFG_2, 0xEA},//0x02
	{TAS2552_SER_CTRL_1, 0x00},//0x05
	{TAS2552_SER_CTRL_2, 0x00},//0x06
	{TAS2552_PLL_CTRL_1, 0x10},//0x08
	{TAS2552_PLL_CTRL_2, 0x00},//0x09
	{TAS2552_PLL_CTRL_3, 0x00},//0x03
	{TAS2552_BTIP, 0x8f},//0x0b
	{TAS2552_BTS_CTRL, 0x80},//0x0c
	{TAS2552_LIMIT_RELEASE, 0x05},//0x0f
	{TAS2552_LIMIT_INT_COUNT, 0x00},//0x10
	{TAS2552_EDGE_RATE_CTRL, 0x40},//0x13
	{TAS2552_VBAT_DATA, 0x00},//0x19
};

/* Add this below to enable register debugging
#define TAS2552_DEBUG*/

struct tas2552_data {
	struct mutex mutex;
	struct snd_soc_codec *codec;
	struct regmap *regmap;
	struct i2c_client *tas2552_client;
	unsigned char regs[TAS2552_VBAT_DATA];
	int power_gpio;
	u8 power_state:1;
};
static struct i2c_client *tas2552_client;//yxw add for test

static int tas2552_power(struct tas2552_data *data, u8 power)
{
	int	ret = 0;

	BUG_ON(data->tas2552_client == NULL);
	printk(" %s power %d \n",__func__,power);
	mutex_lock(&data->mutex);
	if (power == data->power_state)
		goto exit;

	if (power) {
		if (data->power_gpio >= 0)
			gpio_set_value(data->power_gpio, 1);
		msleep(200);
		data->power_state = 1;
	} else {
		if (data->power_gpio >= 0)
			gpio_set_value(data->power_gpio, 0);
		msleep(200);
		data->power_state = 0;
	}

exit:
	mutex_unlock(&data->mutex);
	return ret;
}

#ifdef TAS2552_DEBUG
struct tas2552_reg {
	const char *name;
	uint8_t reg;
	int writeable;
} tas2552_regs[] = {
	{ "STATUS",		TAS2552_DEVICE_STATUS, 1 },
	{ "CFG1",		TAS2552_CFG_1, 1 },
	{ "CFG2",		TAS2552_CFG_2, 1 },
	{ "CFG3",		TAS2552_CFG_3, 1 },
	{ "DOUT",		TAS2552_DOUT, 1 },
	{ "SER_CTRL_1",	TAS2552_SER_CTRL_1, 1 },
	{ "SER_CTRL_2",	TAS2552_SER_CTRL_2, 1 },
	{ "OUTPUT_DATA", TAS2552_OUTPUT_DATA, 1 },
	{ "PLL_CTRL_1",	TAS2552_PLL_CTRL_1, 1 },
	{ "PLL_CTRL_2",	TAS2552_PLL_CTRL_2, 1 },
	{ "PLL_CTRL_3",	TAS2552_PLL_CTRL_3, 1 },
	{ "BTIP", TAS2552_BTIP, 1 },
	{ "BTS_CTRL", TAS2552_BTS_CTRL, 1 },
	{ "LIMIT_LVL_CTRL",	TAS2552_LIMIT_LVL_CTRL, 1 },
	{ "LIMIT_RATE_HYS",	TAS2552_LIMIT_RATE_HYS, 1 },
	{ "IMIT_RELEASE",	TAS2552_LIMIT_RELEASE, 1 },
	{ "LIMIT_INT_COUNT", TAS2552_LIMIT_INT_COUNT, 1 },
	{ "PDM_CFG",	TAS2552_PDM_CFG, 1 },
	{ "PGA_GAIN",	TAS2552_PGA_GAIN, 1 },
	{ "EDGE_CTRL",	TAS2552_EDGE_RATE_CTRL, 1 },
	{ "BOOST_CTRL",	TAS2552_BOOST_PT_CTRL, 1 },
	{ "VER_NUM", TAS2552_VER_NUM, 1 },
	{ "VBAT_DATA", TAS2552_VBAT_DATA, 1 },
};

static int tas2552_i2c_read(struct tas2552_data *tas_data, int reg)
{
	int val;

	if (WARN_ON(!tas_data->tas2552_client))
		return -EINVAL;

	/* If powered off, return the cached value */
	mutex_lock(&tas_data->mutex);
	if (1/*tas_data->power_state*/) {
		val = i2c_smbus_read_byte_data(tas_data->tas2552_client, reg);
		if (val < 0)
			dev_err(&tas_data->tas2552_client->dev,
					"Read failed %i\n", val);
		else
			tas_data->regs[reg] = val;
	} else {
		val = tas_data->regs[reg];
	}

	mutex_unlock(&tas_data->mutex);
	return val;
}

static int tas2552_i2c_write(struct tas2552_data *tas_data, int reg, u8 value)
{
	int val = 0;

	if (WARN_ON(!tas_data->tas2552_client))
		return -EINVAL;
	printk(" %s reg 0x%x val 0x%x",__func__,reg,value);
	mutex_lock(&tas_data->mutex);
	if (1/*tas_data->power_state*/) {
		val = i2c_smbus_write_byte_data(tas_data->tas2552_client,
										reg, value);
		if (val < 0) {
			dev_err(&tas_data->tas2552_client->dev,
					"Write failed %i\n", val);
			goto write_err;
		}
	}

	/* Either powered on or off, we save the context */
	tas_data->regs[reg] = value;

write_err:
	mutex_unlock(&tas_data->mutex);
	return val;
}

static ssize_t tas2552_registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	unsigned i, n, reg_count;
	u8 read_buf;
	struct tas2552_data *data = dev_get_drvdata(dev);

	tas2552_power(data, 1);
	reg_count = sizeof(tas2552_regs) / sizeof(tas2552_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		read_buf = tas2552_i2c_read(data, tas2552_regs[i].reg);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%-20s = 0x%02X\n",
			       tas2552_regs[i].name,
			       read_buf);
	}
	tas2552_power(data, 0);
	return n;
}

static ssize_t tas2552_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];
	struct tas2552_data *data = dev_get_drvdata(dev);

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	tas2552_power(data, 1);
	reg_count = sizeof(tas2552_regs) / sizeof(tas2552_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tas2552_regs[i].name)) {
			if (tas2552_regs[i].writeable) {
				error = tas2552_i2c_write(data, tas2552_regs[i].reg, value);
				if (error) {
					pr_err("%s:Failed to write %s\n",
						__func__, name);
					return -1;
				}
			} else {
				pr_err("%s:Register %s is not writeable\n",
						__func__, name);
					return -1;
			}
			return count;
		}
	}
	tas2552_power(data, 0);
	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		tas2552_registers_show, tas2552_registers_store);

static struct attribute *tas2552_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group tas2552_attr_group = {
	.attrs = tas2552_attrs,
};
#endif

static void tas2552_sw_shutdown(struct tas2552_data *tas_data, int sw_shutdown)
{
	u8 cfg1_reg = 0;
    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	if (sw_shutdown)
		cfg1_reg |= (sw_shutdown << 1);
	else
		cfg1_reg &= ~TAS2552_SWS_MASK;

    printk(" tas2552 %s line %d cfg1_reg 0x%x\n",__func__,__LINE__,cfg1_reg);
	snd_soc_update_bits(tas_data->codec, TAS2552_CFG_1,
						 TAS2552_SWS_MASK, cfg1_reg);
}

static void tas2552_init(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, TAS2552_CFG_1, 0x16);
	snd_soc_write(codec, TAS2552_CFG_3, 0x5E);
	snd_soc_write(codec, TAS2552_DOUT, 0x00);
	snd_soc_write(codec, TAS2552_OUTPUT_DATA, 0xC8);
	snd_soc_write(codec, TAS2552_PDM_CFG, 0x02);
	snd_soc_write(codec, TAS2552_PGA_GAIN, 0x10);
	snd_soc_write(codec, TAS2552_BOOST_PT_CTRL, 0x0F);
	snd_soc_write(codec, TAS2552_LIMIT_LVL_CTRL, 0x0C);
	snd_soc_write(codec, TAS2552_LIMIT_RATE_HYS, 0x20);
	snd_soc_write(codec, TAS2552_CFG_2, 0xEA);
}

static int tas2552_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	u8 wclk_reg;
	struct snd_soc_codec *codec = dai->codec;

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	/* Setting DAC clock dividers based on substream sample rate. */
	switch (params_rate(params)) {
	case 8000:
		wclk_reg = TAS2552_8KHZ;
		break;
	case 11025:
		wclk_reg = TAS2552_11_12KHZ;
		break;
	case 16000:
		wclk_reg = TAS2552_16KHZ;
		break;
	case 32000:
		wclk_reg = TAS2552_32KHZ;
		break;
	case 22050:
	case 24000:
		wclk_reg = TAS2552_22_24KHZ;
		break;
	case 44100:
	case 48000:
		wclk_reg = TAS2552_44_48KHZ;
		break;
	case 96000:
		wclk_reg = TAS2552_88_96KHZ;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_CFG_3, TAS2552_WCLK_MASK, wclk_reg);

	return 0;
}

static int tas2552_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 serial_format;
	struct snd_soc_codec *codec = dai->codec;

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		serial_format = 0x00;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		serial_format = TAS2552_WORD_CLK_MASK;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		serial_format = TAS2552_BIT_CLK_MASK;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		serial_format = (TAS2552_BIT_CLK_MASK | TAS2552_WORD_CLK_MASK);
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_SER_CTRL_1,
						(TAS2552_BIT_CLK_MASK | TAS2552_WORD_CLK_MASK),
						serial_format);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		serial_format = 0x0;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		serial_format = TAS2552_DAIFMT_DSP;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		serial_format = TAS2552_DAIFMT_RIGHT_J;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		serial_format = TAS2552_DAIFMT_LEFT_J;
		break;

	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_SER_CTRL_1, TAS2552_DATA_FORMAT_MASK,
						serial_format);

	return 0;
}

static int tas2552_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *data = dev_get_drvdata(dai->dev);

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	/* TODO fill in the PLL control registers for J & D */
	switch (freq) {
	case 12288000:
	case 26000000:
	case 19200000:
	case 24576000:
		break;
	case 48000:
	case 32576:
		break;
	default:
		break;
	}

	tas2552_sw_shutdown(data, 1);

	snd_soc_update_bits(codec, TAS2552_CFG_2, TAS2552_PLL_ENABLE, 0);

	snd_soc_write(codec, TAS2552_PLL_CTRL_1, 0x10);
	snd_soc_write(codec, TAS2552_PLL_CTRL_2, 0x00);
	snd_soc_write(codec, TAS2552_PLL_CTRL_3, 0x00);

	snd_soc_update_bits(codec, TAS2552_CFG_2, TAS2552_PLL_ENABLE,
						TAS2552_PLL_ENABLE);

	tas2552_sw_shutdown(data, 0);

	return 0;
}

static int tas2552_mute(struct snd_soc_dai *dai, int mute)
{
	u8 cfg1_reg;
	struct snd_soc_codec *codec = dai->codec;

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	if (mute)
		cfg1_reg |= TAS2552_MUTE_MASK;
	else
		cfg1_reg &= ~TAS2552_MUTE_MASK;

	snd_soc_update_bits(codec, TAS2552_CFG_1, TAS2552_MUTE_MASK, cfg1_reg);

	return 0;
}

static void tas2552_init_code(void)
{
	int i;
	int val;
	
	i = i2c_smbus_read_byte_data(tas2552_client, 0x16);
	printk("%s get 0x16 = 0x%x \n",__func__,i);
	val = i2c_smbus_write_byte_data(tas2552_client,	0x01,0x12 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x02,0xe2 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x08,0x20 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x09,0x00 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x0a,0x00 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x03,0x4d );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x04,0x00 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x05,0x00 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x06,0x00 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x07,0xc8 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x12,0x16 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x14,0x0f );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x0d,0xc0 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x0e,0x20 );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x02,0xea );
	val = i2c_smbus_write_byte_data(tas2552_client,	0x01,0x10 );
	if (val < 0) 
		printk("error write reg");

	i = i2c_smbus_read_byte_data(tas2552_client, 0x02);
	printk("%s get 0x02 = 0x%x \n",__func__,i);
	return;
}

static int tas2552_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	tas2552_power(tas2552, 1);
	tas2552_init_code();//yxw add for test
	tas2552_sw_shutdown(tas2552, 1);

	/* Turn on Class D amplifier */
	snd_soc_update_bits(codec, TAS2552_CFG_2, TAS2552_CLASSD_EN_MASK,
						TAS2552_CLASSD_EN);

	tas2552_sw_shutdown(tas2552, 0);

	return 0;
}

static void tas2552_shutdown(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	
	tas2552_sw_shutdown(tas2552, 1);
	tas2552_power(tas2552, 0);
}

static struct snd_soc_dai_ops tas2552_speaker_dai_ops = {
	.hw_params	= tas2552_hw_params,
	.set_sysclk	= tas2552_set_dai_sysclk,
	.set_fmt	= tas2552_set_dai_fmt,
	.startup	= tas2552_startup,
	.shutdown	= tas2552_shutdown,
	.digital_mute = tas2552_mute,
};

/* Formats supported by TAS2552 driver. */
#define TAS2552_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* TAS2552 dai structure. */
static struct snd_soc_dai_driver tas2552_dai[] = {
	{
		.name = "tas2552-amplifier",
		.playback = {
			.stream_name = "Speaker",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = TAS2552_FORMATS,
		},
		.ops = &tas2552_speaker_dai_ops,
	},
};

/*
 * DAC digital volumes. From -7 to 24 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, -7, 100, 24);

static const struct snd_kcontrol_new tas2552_snd_controls[] = {
	SOC_SINGLE_TLV("Speaker Driver Playback Volume",
			 TAS2552_PGA_GAIN, 0, 0x1f, 1, dac_tlv),
};

static int tas2552_codec_probe(struct snd_soc_codec *codec)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

    printk(" tas2552 %s line %d \n",__func__,__LINE__);
	tas2552->codec = codec;
	tas2552_power(tas2552, 1);
	if(0)
		tas2552_init(codec);

	return 0;
}

static int tas2552_codec_remove(struct snd_soc_codec *codec)
{
	struct tas2552_data *tas2552 = snd_soc_codec_get_drvdata(codec);

	tas2552_power(tas2552, 0);

	return 0;
};

static struct snd_soc_codec_driver soc_codec_dev_tas2552 = {
	.probe = tas2552_codec_probe,
	.remove = tas2552_codec_remove,
	.controls = tas2552_snd_controls,
	.num_controls = ARRAY_SIZE(tas2552_snd_controls),
};

static const struct regmap_config tas2552_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = TAS2552_MAX_REG,
	.reg_defaults = tas2552_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(tas2552_reg_defs),
	.cache_type = REGCACHE_RBTREE,
};

static int tas2552_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev;
	struct tas2552_data *data;
	struct tas2552_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret;
	int i;

	printk("fjz %s %d \n",__func__,__LINE__);
	dev = &client->dev;
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	if (pdata) {
		data->power_gpio = pdata->power_gpio;
	} else if (np) {
		data->power_gpio = of_get_named_gpio(np, "qcom,enable-gpio", 0);
	} else {
		dev_err(dev, "Platform data not set\n");
		return -ENODEV;
	}

	data->regmap = devm_regmap_init_i2c(client, &tas2552_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	data->tas2552_client = client;
	data->regmap = devm_regmap_init_i2c(client, &tas2552_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	dev_set_drvdata(&client->dev, data);

	mutex_init(&data->mutex);

	if (data->power_gpio >= 0) {
		ret = devm_gpio_request(dev, data->power_gpio, "tas2552 enable");
		if (ret < 0) {
			dev_err(dev, "Failed to request power GPIO (%d)\n",
				data->power_gpio);
			goto err_gpio;
		}
		printk("%s set gpio 1 \n",__func__);
		gpio_direction_output(data->power_gpio, 1);
	}
	/* The TAS5086 always returns 0x03 in its TAS5086_DEV_ID register */
	ret = regmap_read(data->regmap, 0x16, &i);
	if (ret < 0)
		return ret;

	if (i != 0x8) {
		dev_err(dev,
				"Failed to identify TAs2552 codec (got %02x)\n", i);
		return -ENODEV;
	}
	tas2552_client = client;
	tas2552_init_code();//yxw add for test

	ret = snd_soc_register_codec(&client->dev,
				      &soc_codec_dev_tas2552,
				      tas2552_dai, ARRAY_SIZE(tas2552_dai));
	if (ret < 0)
		dev_err(&client->dev, "Failed to register codec: %d\n", ret);

#ifdef TAS2552_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &tas2552_attr_group);
	if (ret) {
		pr_err("%s:  Cannot create sysfs group\n", __func__);
		goto err_sysfs;
	}
#endif
	return 0;
#ifdef TAS2552_DEBUG
err_sysfs:
	snd_soc_unregister_codec(&client->dev);
#endif
err_gpio:
	data->tas2552_client = NULL;
	return ret;
}

static int tas2552_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id tas2552_id[] = {
	{ "tas2552-codec", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2552_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tas2552_of_match[] = {
	{ .compatible = "ti,tas2552", },
	{},
};
MODULE_DEVICE_TABLE(of, tas2552_of_match);
#endif

static struct i2c_driver tas2552_i2c_driver = {
	.driver = {
		.name = "tas2552-codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tas2552_of_match),
	},
	.probe = tas2552_probe,
	.remove = tas2552_i2c_remove,
	.id_table = tas2552_id,
};

module_i2c_driver(tas2552_i2c_driver);

MODULE_AUTHOR("Dan Muprhy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TAS2552 Audio amplifier driver");
MODULE_LICENSE("GPL");
