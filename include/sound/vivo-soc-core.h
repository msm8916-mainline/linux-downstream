/*
 * vivo-soc-core.c
 *
 * Author: Jear.Chen <chenjinquan@vivo.com.cn>
 *
 */

#ifndef __VIVO_SOC_CORE_H
#define __VIVO_SOC_CORE_H

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>

struct vivo_snd_soc_runtime {
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai_link *link;

	unsigned int active;

	struct list_head list;
};

int init_vivo_soc_dais(struct snd_soc_card *card);
void deinit_vivo_soc_dais(void);

#endif /* __VIVO_SOC_CORE_H */
