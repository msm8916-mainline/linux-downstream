//liuxudong C for cs4398
#ifndef __CS4398_H__
#define __CS4398_H__

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

// cs4398 register map
#define CS4398_CHIP_ID_REG	          0x01
#define CS4398_MODE_CTL_REG           0x02
#define CS4398_VOL_MIX_INV_CTL_REG    0x03
#define CS4398_MUTE_CTL_REG			  0x04
#define CS4398_A_VOL_CTL_REG		  0x05
#define CS4398_B_VOL_CTL_REG	      0x06
#define CS4398_RAMP_FILTER_CTL_REG 	  0x07
#define CS4398_MISC_CTL_REG			  0x08
#define CS4398_MISC_CTL2_REG          0x09

#define CS4398_ID 0x72
#ifdef BBK_AUDIO_KERNEL

#else
#define CS4398_RST_GPIO	70
#endif
struct cs4398_regulator
{
	const char *name;
};

struct cs4398_device_platform_data{
	char *driver_name;
	int rst_gpio;
	struct cs4398_regulator *vd_regulator,*vl_regulator;
};

#endif
