//liuxudong A for cs8422 driver header
#ifndef __TLV320_H__
#define __TLV320_H__

#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/stat.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/interrupt.h>

#define TLV320_PAGE_CTL		0x00
#define TLV320_SW_RST		0x01
#define TLV320_CLK_IN_MUX	0x04
#define TLV320_PLL_P_R_VAL	0x05
#define TLV320_PLL_J_VAL	0x06
#define TLV320_PLL_D_VAL_MSB	0x07
#define TLV320_PLL_D_VAL_LSB	0x08
#define TLV320_NADC_VAL		0x12
#define TLV320_MADC_VAL		0x13
#define TLV320_AOSR_VAL		0x14
#define TLV320_IADC_VAL		0x15
#define TLV320_CLKOUT_MUX	0x19
#define TLV320_CLKOUT_M_VAL		0x1a
#define TLV320_ADC_INF_CTL_1	0x1b
#define TLV320_ADC_INF_CTL_2	0x1d
#define TLV320_BDIV_N_VAL	0x1e

#define TLV320_DOUT_CTL		0x35

#define BBK_VIVO_AUDIO_DEBUG 1

struct tlv320_regulator {
	const char *name;
};
struct tlv320_dev_platform_data{
	char *driver_name;
	int rst_gpio;
	int pm_conv_gpio;
	struct tlv320_regulator *vd_regulator, *va_regulator;
};

#endif
