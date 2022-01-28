/*
 * vivo-soc-core.c
 *
 * Author: Jear.Chen <chenjinquan@vivo.com.cn>
 *
 *  TODO:
 *   o Add hw rules to enforce rates, etc.
 *   o More testing with other codecs/machines.
 *   o Add more codecs and platforms to ensure good API coverage.
 *   o Support TDM on PCM and I2S
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <sound/ac97_codec.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <sound/initval.h>

#include <sound/vivo-soc-core.h>

static DEFINE_MUTEX(client_mutex);
static LIST_HEAD(runtime_list);

int vivo_snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	/*struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int platform_max;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	if (platform_max == 1 && !strstr(kcontrol->id.name, " Volume"))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = snd_soc_volsw_is_stereo(mc) ? 2 : 1;
	uinfo->value.integer.min = 0;
	if (mc->min < 0 && (uinfo->type == SNDRV_CTL_ELEM_TYPE_INTEGER))
		uinfo->value.integer.max = platform_max - mc->min;
	else
		uinfo->value.integer.max = platform_max;

	dev_err(codec->dev, "vivo-ASoC: info_volsw platform_max %d, mc->max %d, reg %u, rreg %u, shift %u, rshift %u\n",
			platform_max, mc->max, mc->reg, mc->rreg, mc->shift, mc->rshift);
*/
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}


static int vivo_snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct vivo_snd_soc_runtime *vivo_rtd;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (!strcmp(ucontrol->id.name, vivo_rtd->link->name)) {
			dev_err(vivo_rtd->codec->dev, "vivo-ASoC: get_volsw dai active %d\n",
				vivo_rtd->active);
			ucontrol->value.integer.value[0] = vivo_rtd->active;
		}
	}

	return 0;
}

static int vivo_snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct vivo_snd_soc_runtime *vivo_rtd;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (!strcmp(ucontrol->id.name, vivo_rtd->link->name)) {
			dev_err(vivo_rtd->codec->dev, "vivo-ASoC: put_volsw dai active %ld\n",
				ucontrol->value.integer.value[0]);
			vivo_rtd->active = !!ucontrol->value.integer.value[0];
		}
	}

	return 0;
}

static int vivo_soc_add_dai_link_control(struct snd_soc_codec *codec,
		struct snd_soc_dai_link *link)
{
	struct snd_kcontrol_new control;
	int ret;

	control = (struct snd_kcontrol_new) {\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = link->name, \
		.info = vivo_snd_soc_info_volsw, .get = vivo_snd_soc_get_volsw,\
		.put = vivo_snd_soc_put_volsw, \
		.private_value =  SOC_SINGLE_VALUE(0, 0, 1, 0) };

	ret = snd_ctl_add(codec->card->snd_card,
					snd_soc_cnew(&control,
					codec, control.name,
					codec->name_prefix));
	if (ret < 0) {
		dev_err(codec->dev, "vivo-ASoC: Failed to add control: %s: %d\n",
			control.name, ret);
		return ret;
	}

	dev_err(codec->dev, "vivo-ASoC: Add control: %s\n",
		control.name);

	return 0;
}

int init_vivo_soc_dais(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai = NULL;
	struct vivo_snd_soc_runtime *vivo_rtd;
	int i;

	if (!card->name || !card->dev)
		return -EINVAL;

	dev_err(card->dev, "vivo-ASoC: init vivo soc dais\n");

	for (i = 0; i < card->num_links; i++) {
		struct snd_soc_dai_link *link = &card->dai_link[i];

		/*
		 * Codec must be specified by 1 of name or OF node,
		 * not both or neither.
		 */
		if (!!link->codec_name == !!link->codec_of_node) {
			dev_err(card->dev, "vivo-ASoC: Neither/both codec"
				" name/of_node are set for %s\n", link->name);
			continue;
		}

		/* Codec DAI name must be specified */
		if (!link->codec_dai_name) {
			dev_err(card->dev, "vivo-ASoC: codec_dai_name not"
				" set for %s\n", link->name);
			continue;
		}

		/*
		 * CPU device may be specified by either name or OF node, but
		 * can be left unspecified, and will be matched based on DAI
		 * name alone..
		 */
		if (link->cpu_name && link->cpu_of_node) {
			dev_err(card->dev, "vivo-ASoC: Neither/both "
				"cpu name/of_node are set for %s\n",link->name);
			continue;
		}

		/*
		 * At least one of CPU DAI name or CPU device name/node must be
		 * specified
		 */
		if (!link->cpu_dai_name &&
		    !(link->cpu_name || link->cpu_of_node)) {
			dev_err(card->dev, "vivo-ASoC: Neither cpu_dai_name nor "
				"cpu_name/of_node are set for %s\n", link->name);
			continue;
		}

		/* If cpu dai name is not 'vivo-snd-soc-dummy-dai', ignore. */
		if (!link->cpu_dai_name || (link->cpu_dai_name &&
			strcmp("vivo-snd-soc-dummy-dai", link->cpu_dai_name)))
			continue;

		/* Find CODEC from registered CODECs */
		list_for_each_entry(codec, &card->codec_dev_list, card_list) {
			if (link->codec_of_node) {
				if (codec->dev->of_node != link->codec_of_node)
					continue;
			} else {
				if (strcmp(codec->name, link->codec_name))
					continue;
			}

			/*
			 * CODEC found, so find CODEC DAI from registered DAIs from
			 * this CODEC
			 */
			codec_dai = snd_soc_get_dai(codec, link->codec_dai_name);
			if (!codec_dai)
				continue;

			dev_err(card->dev, "vivo-ASoC: get codec dai %s\n", codec_dai->name);

			vivo_rtd = kzalloc(sizeof(struct vivo_snd_soc_runtime), GFP_KERNEL);
			if (vivo_rtd == NULL)
				return -ENOMEM;

			vivo_rtd->codec = codec;
			vivo_rtd->codec_dai = codec_dai;
			vivo_rtd->link = link;
			vivo_rtd->active = 0;

			mutex_lock(&client_mutex);
			list_add(&vivo_rtd->list, &runtime_list);
			mutex_unlock(&client_mutex);

			vivo_soc_add_dai_link_control(codec, link);
		}
	}

	return 0;
}

void deinit_vivo_soc_dais(void)
{
	struct vivo_snd_soc_runtime *vivo_rtd;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		mutex_lock(&client_mutex);
		list_del(&vivo_rtd->list);
		mutex_unlock(&client_mutex);

		kfree(vivo_rtd);
	}
	return;
}

static void vivo_soc_dapm_stream_event(struct snd_soc_dai *codec_dai, int stream,
	int event)
{
	struct snd_soc_dapm_widget *w_codec;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		w_codec = codec_dai->playback_widget;
	} else {
		w_codec = codec_dai->capture_widget;
	}

	if (w_codec) {

		dapm_mark_dirty(w_codec, "stream event");

		switch (event) {
		case SND_SOC_DAPM_STREAM_START:
			w_codec->active = 1;
			break;
		case SND_SOC_DAPM_STREAM_STOP:
			w_codec->active = 0;
			break;
		case SND_SOC_DAPM_STREAM_SUSPEND:
		case SND_SOC_DAPM_STREAM_RESUME:
		case SND_SOC_DAPM_STREAM_PAUSE_PUSH:
		case SND_SOC_DAPM_STREAM_PAUSE_RELEASE:
			break;
		}
	}

	dev_err(codec_dai->dev, "vivo-ASoC: stream event\n");
}

void vivo_snd_soc_dapm_stream_event(struct snd_soc_dai *dai, int stream,
			      int event)
{
	struct snd_soc_card *card = dai->card;

	dev_err(card->dev, "vivo-ASoC: vivo_snd_soc_dapm_stream_event\n");

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	vivo_soc_dapm_stream_event(dai, stream, event);
	mutex_unlock(&card->dapm_mutex);
	snd_soc_dapm_sync(&card->dapm);
}

static int vivo_dummy_codec_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set sysclk\n");
			if (codec_dai->driver->ops->set_sysclk) {
				ret = codec_dai->driver->ops->set_sysclk(codec_dai,
					clk_id, freq, dir);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set sysclk error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_pll(struct snd_soc_dai *dai,
		int pll_id, int source,
		unsigned int freq_in,
		unsigned int freq_out)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set pll\n");
			if (codec_dai->driver->ops->set_pll) {
				ret = codec_dai->driver->ops->set_pll(codec_dai,
					pll_id, source, freq_in, freq_out);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set pll error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_clkdiv(struct snd_soc_dai *dai,
		int div_id, int div)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set clkdiv\n");
			if (codec_dai->driver->ops->set_clkdiv) {
				ret = codec_dai->driver->ops->set_clkdiv(codec_dai,
					div_id, div);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set clkdiv error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set fmt\n");
			if (codec_dai->driver->ops->set_fmt) {
				ret = codec_dai->driver->ops->set_fmt(codec_dai,
					fmt);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set fmt error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set tdm slot\n");
			if (codec_dai->driver->ops->set_tdm_slot) {
				ret = codec_dai->driver->ops->set_tdm_slot(codec_dai,
					tx_mask, rx_mask, slots, slot_width);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set tdm slot error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_channel_map(struct snd_soc_dai *dai,
		unsigned int tx_num, unsigned int *tx_slot,
		unsigned int rx_num, unsigned int *rx_slot)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set channel map\n");
			if (codec_dai->driver->ops->set_channel_map) {
				ret = codec_dai->driver->ops->set_channel_map(codec_dai,
					tx_num, tx_slot, rx_num, rx_slot);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set channel map error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_set_tristate(struct snd_soc_dai *dai,
		int tristate)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: set tristate\n");
			if (codec_dai->driver->ops->set_tristate) {
				ret = codec_dai->driver->ops->set_tristate(codec_dai,
					tristate);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI set tristate error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_digital_mute(struct snd_soc_dai *dai,
		int mute)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: digital mute\n");
			if (codec_dai->driver->ops->digital_mute) {
				ret = codec_dai->driver->ops->digital_mute(codec_dai,
					mute);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI digital mute error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_mute_stream(struct snd_soc_dai *dai,
		int mute, int stream)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: mute stream\n");
			if (codec_dai->driver->ops->mute_stream) {
				ret = codec_dai->driver->ops->mute_stream(codec_dai,
					mute, stream);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI mute stream error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: startup\n");
			if (codec_dai->driver->ops->startup) {
				ret = codec_dai->driver->ops->startup(substream,
					codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI startup error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static void vivo_dummy_codec_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: shutdown\n");

			if (codec_dai->driver->ops->shutdown)
				codec_dai->driver->ops->shutdown(substream, codec_dai);

			/* Just setted widget->active, widgets will be updated later in ASoC. */
			vivo_snd_soc_dapm_stream_event(codec_dai, substream->stream,
						SND_SOC_DAPM_STREAM_STOP);
		}
	}
}

static int vivo_dummy_codec_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: hw params\n");
			if (codec_dai->driver->ops->hw_params) {
				ret = codec_dai->driver->ops->hw_params(substream,
					params, codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI hw params error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: hw free\n");
			if (codec_dai->driver->ops->hw_free) {
				ret = codec_dai->driver->ops->hw_free(substream,
					codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI hw free error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: prepare\n");
			if (codec_dai->driver->ops->prepare) {
				ret = codec_dai->driver->ops->prepare(substream,
					codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI prepare error: %d\n",
						ret);
			}

			/* Just setted widget->active, widgets will be updated later in ASoC. */
			vivo_snd_soc_dapm_stream_event(codec_dai, substream->stream,
						SND_SOC_DAPM_STREAM_START);
		}
	}

	return 0;
}

static int vivo_dummy_codec_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: trigger\n");
			if (codec_dai->driver->ops->trigger) {
				ret = codec_dai->driver->ops->trigger(substream,
					cmd, codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI trigger error: %d\n",
						ret);
			}
		}
	}

	return 0;
}

static int vivo_dummy_codec_bespoke_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: bespoke trigger\n");
			if (codec_dai->driver->ops->bespoke_trigger) {
				ret = codec_dai->driver->ops->bespoke_trigger(substream,
					cmd, codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI bespoke trigger error: %d\n",
						ret);
			}
		}
	}

	return 0;
}
/*
static snd_pcm_sframes_t vivo_dummy_codec_delay(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct vivo_snd_soc_runtime *vivo_rtd;
	struct snd_soc_dai *codec_dai;
	int ret;

	list_for_each_entry(vivo_rtd, &runtime_list, list) {
		if (vivo_rtd->active) {
			codec_dai = vivo_rtd->codec_dai;

			dev_err(codec_dai->dev, "vivo-ASoC: delay\n");
			if (codec_dai->driver->ops->delay) {
				ret = codec_dai->driver->ops->delay(substream,
					codec_dai);
				if (ret < 0)
					dev_err(codec_dai->dev, "vivo-ASoC: DAI delay error: %d\n",
						ret);
			}
		}
	}

	return 0;
}
*/
static struct snd_soc_dai_ops vivo_dummy_codec_ops = {
	.set_sysclk = vivo_dummy_codec_set_sysclk,
	.set_pll = vivo_dummy_codec_set_pll,
	.set_clkdiv = vivo_dummy_codec_set_clkdiv,

	.set_fmt = vivo_dummy_codec_set_fmt,
	.set_tdm_slot = vivo_dummy_codec_set_tdm_slot,
	.set_channel_map = vivo_dummy_codec_set_channel_map,
	.set_tristate = vivo_dummy_codec_set_tristate,

	.digital_mute = vivo_dummy_codec_digital_mute,
	.mute_stream = vivo_dummy_codec_mute_stream,

	.startup = vivo_dummy_codec_startup,
	.shutdown = vivo_dummy_codec_shutdown,
	.hw_params = vivo_dummy_codec_hw_params,
	.hw_free = vivo_dummy_codec_hw_free,
	.prepare = vivo_dummy_codec_prepare,
	.trigger = vivo_dummy_codec_trigger,
	.bespoke_trigger = vivo_dummy_codec_bespoke_trigger,

	//.delay = vivo_dummy_codec_delay,
};

static struct snd_soc_codec_driver vivo_dummy_codec;

#define STUB_RATES	SNDRV_PCM_RATE_8000_192000
#define STUB_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE | \
			SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static struct snd_soc_dai_driver vivo_dummy_dai = {
	.name = "vivo-snd-soc-dummy-dai",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates		= STUB_RATES,
		.formats	= STUB_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 384,
		.rates = STUB_RATES,
		.formats = STUB_FORMATS,
	 },
	 .ops = &vivo_dummy_codec_ops,
};

static int vivo_snd_soc_dummy_probe(struct platform_device *pdev)
{
	int ret;

	printk("vivo_snd_soc_dummy_probe()\n");

	ret = snd_soc_register_codec(&pdev->dev, &vivo_dummy_codec, &vivo_dummy_dai, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int vivo_snd_soc_dummy_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver vivo_soc_dummy_driver = {
	.driver = {
		.name = "vivo-snd-soc-dummy",
		.owner = THIS_MODULE,
	},
	.probe = vivo_snd_soc_dummy_probe,
	.remove = vivo_snd_soc_dummy_remove,
};

static struct platform_device *vivo_soc_dummy_dev;

int __init vivo_snd_soc_dummy_init(void)
{
	int ret;

	printk("vivo_snd_soc_dummy_init()\n");

	vivo_soc_dummy_dev = platform_device_alloc("vivo-snd-soc-dummy", -1);
	if (!vivo_soc_dummy_dev)
		return -ENOMEM;

	ret = platform_device_add(vivo_soc_dummy_dev);
	if (ret != 0) {
		platform_device_put(vivo_soc_dummy_dev);
		return ret;
	}

	ret = platform_driver_register(&vivo_soc_dummy_driver);
	if (ret != 0)
		platform_device_unregister(vivo_soc_dummy_dev);

	return ret;
}
module_init(vivo_snd_soc_dummy_init);

void __exit vivo_snd_soc_dummy_exit(void)
{
	platform_device_unregister(vivo_soc_dummy_dev);
	platform_driver_unregister(&vivo_soc_dummy_driver);
}
module_exit(vivo_snd_soc_dummy_exit);

/* Module information */
MODULE_AUTHOR("Jear.Chen chenjinquan@vivo.com.cn");
MODULE_DESCRIPTION("vivo ALSA SoC Core");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-audio");
