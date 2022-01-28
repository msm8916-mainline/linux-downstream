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

struct vivo_codec_function * vivo_codec_fun = NULL;

struct vivo_codec_function * get_vivo_codec_function(void)
{
	return vivo_codec_fun;
}

void set_vivo_codec_function(struct vivo_codec_function *fun)
{
	vivo_codec_fun = fun;
}
