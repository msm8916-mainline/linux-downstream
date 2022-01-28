/* Add by ChenJinQuan<chenjinquan@vivo.com> for vivo codec. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/qpnp/clkdiv.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "vivo-codec-common.h"

enum {
	VIVO_CODEC_HP_CONTROL = 0,
	VIVO_CODEC_MAX_REG
};

struct vivo_codec_gpio
{
	int gpio;
	enum of_gpio_flags flags;
};

struct vivo_codec_prv
{
	struct snd_soc_codec *codec;
	struct audio_params params;
	bool is_hifi_dac_on;
	int cache[VIVO_CODEC_MAX_REG]; 
	struct vivo_codec_function *fun;
};

static struct vivo_codec_prv *vivo_codec;

static unsigned int vivo_codec_read(
		struct snd_soc_codec *codec,
		unsigned int reg)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
			__func__);
		return -EINVAL;
	}

	pr_info("%s:reg 0x%x\n", __func__, reg);

	if (reg <= VIVO_CODEC_MAX_REG)
		return vivo_codec_prv->cache[reg];

	return 0;
}

static int vivo_codec_write(
		struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
			__func__);
		return -EINVAL;
	}

	pr_info("%s:reg 0x%x, value 0x%x\n", __func__, reg, value);

	vivo_codec_prv->cache[reg] = value;

	return 0;
}

static int vivo_codec_power_up(enum vivo_codec_id id)
{
	pr_info("%s:id %d\n", __func__, id);

	return 0;
}

static int vivo_codec_power_down(enum vivo_codec_id id)
{
	pr_info("%s:id %d\n", __func__, id);

	return 0;
}

static int vivo_codec_hw_reset(enum vivo_codec_id id)
{
	pr_info("%s:id %d\n", __func__, id);

	return 0;
}

static int vivo_codec_hp_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	pr_info("%s:event %d, %s%s\n", __func__, event,
		event & (SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU) ? "power up" : "",
		event & (SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD) ? "power down" : "");

	return 0;
}

static const char * const hp_path_text[]={
		"Normal"};

static const struct soc_enum hp_path_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM,
		0, 1, hp_path_text);

static const struct snd_kcontrol_new hp_path_mux =
	SOC_DAPM_ENUM("HP Path Mux", hp_path_enum);

static const struct snd_kcontrol_new hpout_mixer =
	SOC_DAPM_SINGLE("HiFi DAC Switch", VIVO_CODEC_HP_CONTROL,
		0, 1, 0);

static const struct snd_soc_dapm_widget vivo_codec_dapm_widgets[] = {

	/* HP */
	SND_SOC_DAPM_HP("HP", vivo_codec_hp_event),
	SND_SOC_DAPM_OUTPUT("HPOUT"),
	SND_SOC_DAPM_MIXER("HPOUT Mixer", SND_SOC_NOPM, 0, 0,
		&hpout_mixer, 1),
	SND_SOC_DAPM_DAC_E("HiFi DAC", NULL, SND_SOC_NOPM, 0, 0,
		NULL,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* This will be used in msm8x16.c, route HEADPHONE -> HPOUT Path */
	SND_SOC_DAPM_MUX("HPOUT Path",
		SND_SOC_NOPM, 0, 0, &hp_path_mux),

	SND_SOC_DAPM_AIF_IN_E("HiFi I2S In", "HiFi Playback", 0, SND_SOC_NOPM,
			0, 0, NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("HiFi I2S Out", "HiFi Capture", 0, SND_SOC_NOPM,
			0, 0, NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route vivo_codec_audio_map[] = {
	{"HP", NULL, "HPOUT"},
	{"HPOUT", NULL, "HPOUT Mixer"},
	{"HPOUT Mixer", "HiFi DAC Switch", "HiFi DAC"},
	{"HiFi DAC", NULL, "HiFi I2S In"},
};

static int vivo_codec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;
	int format, rate;

	if (!vivo_codec_prv || !vivo_codec_prv->fun) {
		pr_err("%s:vivo_codec_prv or vivo_codec_prv->fun is NULL\n",
			__func__);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	default:
		return -EINVAL;
	}

	rate = params_rate(params);

	pr_err("%s: format %d, rate %d\n", __func__, format, rate);

	vivo_codec_prv->params.rate = rate;

	if (format != vivo_codec_prv->params.pcm_format) {
		vivo_codec_prv->params.pcm_format = format;

		/* update HiFi format */
		if (vivo_codec_prv->fun->hifi_dac_enable &&
			vivo_codec_prv->is_hifi_dac_on == true) {
			vivo_codec_prv->fun->hifi_dac_enable(
				&vivo_codec_prv->params,
				false);
			msleep(10);
			vivo_codec_prv->fun->hifi_dac_enable(
				&vivo_codec_prv->params,
				true);
		}
	}

	return 0;
}

static int vivo_codec_set_fmt(struct snd_soc_dai *dai,
	unsigned int fmt)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	pr_info("%s:fmt 0x%x\n", __func__, fmt);

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
			__func__);
		return -EINVAL;
	}

	vivo_codec_prv->params.i2s_format = fmt;

	return 0;
}

static int vivo_codec_mute_stream(struct snd_soc_dai *dai,
	int mute, int stream)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;
	int ret = 0;

	if (!vivo_codec_prv || !vivo_codec_prv->fun) {
		pr_err("%s:vivo_codec_prv or vivo_codec_prv->fun is NULL\n",
			__func__);
		return -EINVAL;
	}

	/* Smart PA need i2s clk to PD/PU */
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* Enable HiFi DAC */
		if (vivo_codec_prv->is_hifi_dac_on &&
			vivo_codec_prv->fun->hifi_dac_enable &&
			mute) {
		
			pr_info("%s: pown down HiFi DAC\n", __func__);
		
			/* Disable HiFi DAC */
			if (vivo_codec_prv->is_hifi_dac_on) {
			
				ret = vivo_codec_prv->fun->hifi_dac_enable(
						&vivo_codec_prv->params,
						false);
				if (ret)
					pr_err("%s:call hifi_dac_enable error %d\n",
						__func__, ret);

				vivo_codec_prv->is_hifi_dac_on = false;
			}

		} else if (!vivo_codec_prv->is_hifi_dac_on &&
			vivo_codec_prv->fun->hifi_dac_enable &&
			!mute && vivo_codec_prv->cache[VIVO_CODEC_HP_CONTROL]) {
			pr_info("%s: pown up HiFi DAC\n", __func__);
	
			ret = vivo_codec_prv->fun->hifi_dac_enable(
					&vivo_codec_prv->params,
					true);
			if (ret)
				pr_err("%s:call hifi_dac_enable error %d\n",
					__func__, ret);

			vivo_codec_prv->is_hifi_dac_on = true;
		}

	}

	return ret;
}


static struct snd_soc_dai_ops vivo_codec_ops = {
	.hw_params = vivo_codec_hw_params,
	.set_fmt = vivo_codec_set_fmt,
	.mute_stream = vivo_codec_mute_stream,
};

static struct snd_soc_dai_driver vivo_codec_dai[] = {
	{
		.name = "VIVO-HiFi",
		.id = 0,
		.playback = {
			.stream_name = "HiFi Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &vivo_codec_ops,
	},
};

static int vivo_codec_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_probe(struct snd_soc_codec *codec)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	pr_info("%s\n", __func__);

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
			__func__);
		return -EINVAL;
	}

	vivo_codec_prv->codec = codec;

	return 0;
}

static int vivo_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver vivo_codec_driver = {
	.probe = 	vivo_codec_probe,
	.remove = 	vivo_codec_remove,
	.suspend =	vivo_codec_suspend,
	.resume = 	vivo_codec_resume,
	.read = vivo_codec_read,
	.write = vivo_codec_write,
	.dapm_widgets = vivo_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vivo_codec_dapm_widgets),
	.dapm_routes = vivo_codec_audio_map,
	.num_dapm_routes = ARRAY_SIZE(vivo_codec_audio_map),
};

static int vivo_codec_platform_probe(
	struct platform_device *pdev)
{
	struct vivo_codec_prv *vivo_codec_prv;
	struct vivo_codec_function *vivo_codec_function;
	int ret = 0;

	dev_err(&pdev->dev, "%s() pd1524\n", __func__);

	vivo_codec_function = kmalloc(sizeof(struct vivo_codec_function), GFP_KERNEL);
	if (!vivo_codec_function) {
		dev_err(&pdev->dev,"%s:vivo_codec_function malloc failed\n", __func__);
		return -ENOMEM;
	}

	vivo_codec_prv = kmalloc(sizeof(struct vivo_codec_prv), GFP_KERNEL);
	if (!vivo_codec_prv) {
		dev_err(&pdev->dev,"%s:vivo_codec_prv malloc failed\n", __func__);
		ret = -ENOMEM;
		goto err_kmalloc;
	}

	vivo_codec_function->power_up = vivo_codec_power_up;
	vivo_codec_function->power_down = vivo_codec_power_down;
	vivo_codec_function->hw_reset = vivo_codec_hw_reset;
	vivo_codec_function->mclk_enable = NULL;
	vivo_codec_function->mclk_disable = NULL;
	
	vivo_codec_function->hifi_clk_enable = NULL;
	vivo_codec_function->hifi_dac_enable = NULL;
	vivo_codec_function->hifi_dac_mute = NULL;
	vivo_codec_function->smart_pa_enable = NULL;
	vivo_codec_function->smart_pa_set_mode = NULL;
	vivo_codec_function->smart_pa_mute = NULL;

	set_vivo_codec_function(vivo_codec_function);

	dev_err(&pdev->dev, "%s() pd1524 vivo_codec_fun(%p)%p\n",
		__func__, &vivo_codec_function, vivo_codec_function);

	vivo_codec_prv->params.rate = 48000;
	vivo_codec_prv->params.pcm_format = SNDRV_PCM_FORMAT_S16_LE;
	vivo_codec_prv->params.i2s_format = SND_SOC_DAIFMT_CBS_CFS |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
	vivo_codec_prv->params.private_params = NULL;
	vivo_codec_prv->cache[VIVO_CODEC_HP_CONTROL] = false;
	vivo_codec_prv->is_hifi_dac_on = false;
	vivo_codec_prv->codec = NULL;
	vivo_codec_prv->fun = vivo_codec_function;
	vivo_codec = vivo_codec_prv;

	dev_set_name(&pdev->dev, "%s", "vivo-codec");

	ret = snd_soc_register_codec(&pdev->dev, &vivo_codec_driver,
		vivo_codec_dai, ARRAY_SIZE(vivo_codec_dai));
	if(ret < 0) {
		pr_err("soc register error %s,rc=%d\n", __func__, ret);
		goto err_register_codec;
	}

	pr_info("%s:complete\n", __func__);

	return 0;

err_register_codec:
	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec = NULL;
	}
err_kmalloc:
	if (vivo_codec_function) {
		kfree(vivo_codec_function);
		set_vivo_codec_function(NULL);
	}

	return ret;
}

static int vivo_codec_platform_remove(
	struct platform_device *pdev)
{
	if (vivo_codec && vivo_codec->fun) {
		kfree(vivo_codec->fun);
		vivo_codec->fun = NULL;
	}

	if (vivo_codec) {
		kfree(vivo_codec);
		vivo_codec = NULL;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vivo_codec_of_match[] = {
	{.compatible = "vivo,vivo-codec-pd1524",},
	{},
};
#else
#define vivo_codec_of_match 0
#endif

static struct platform_driver vivo_codec_platform_driver = {
	.probe = vivo_codec_platform_probe,
	.remove = vivo_codec_platform_remove,
	.driver = {
		.name = "vivo-codec-pd1524",
		.owner = THIS_MODULE,
		.of_match_table = vivo_codec_of_match,
	},
};

module_platform_driver(vivo_codec_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_DESCRIPTION(" vivo hifi codec driver");
