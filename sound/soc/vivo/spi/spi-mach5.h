/*
 * mach5 driver
 *
 * Copyright (c) 2014-2015 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.	In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *	claim that you wrote the original software. If you use this software
 *	in a product, an acknowledgment in the product documentation would be
 *	appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *	misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#ifndef MACH5_H
#define MACH5_H

#include <linux/compat.h>
#define MACH5_INPUT_MIC1		(1)
#define MACH5_INPUT_MIC2		(2)
#define MACH5_INPUT_MIC3		(3)
#define MACH5_INPUT_MIC4		(4)
#define MACH5_INPUT_MIC5		(5)
#define MACH5_INPUT_MUTE		(0)
#define MACH5_OUTPUT_SP			(1)
#define MACH5_OUTPUT_HP			(2)
#define MACH5_OUTPUT_EXTOUT		(3)

struct read_reg {
	__u8		address;	/* I/F Address */
	__u64		data;		/* Read Pointer */
	__u32		size;		/* Read Size */
};

struct write_reg {
	__u64		data;		/* Write Pointer */
	__u32		size;		/* Write Size */
};

struct start_info {
	__u32		input;		/* input kind */
	__u32		output;		/* output kind */
	__u32		mic_vol;	/* mic volume */
};

struct read_reg_2 {
	__u8		address;	/* I/F Address */
	__u64		data;		/* Read Pointer */
	__u32		size;		/* Read Size */
	__u8		retry;		/* Retry Count */
};

#define MACH5_IOC_MAGIC		'x'
#define MACH5_IOCTL_READ_REG	_IOR(MACH5_IOC_MAGIC, 0, struct read_reg)
#define MACH5_IOCTL_WRITE_REG	_IOW(MACH5_IOC_MAGIC, 1, struct write_reg)
#define MACH5_IOCTL_START	_IOW(MACH5_IOC_MAGIC, 2, struct start_info)
#define MACH5_IOCTL_STOP	_IO(MACH5_IOC_MAGIC, 3)
#define MACH5_IOCTL_READ_REG_2	_IOR(MACH5_IOC_MAGIC, 4, struct read_reg_2)
#define MACH5_IOCTL_CLOCK_ON	_IOW(MACH5_IOC_MAGIC, 11, unsigned int)
#define MACH5_IOCTL_CLOCK_OFF	_IOW(MACH5_IOC_MAGIC, 12, unsigned int)
#define MACH5_IOCTL_LDOD_ON		_IOW(MACH5_IOC_MAGIC, 13, unsigned int)
#define MACH5_IOCTL_LDOD_OFF	_IOW(MACH5_IOC_MAGIC, 14, unsigned int)

#define MACH5_IOCTL_MIC_SWCH_ON _IOW(MACH5_IOC_MAGIC, 7, unsigned int)
#define MACH5_IOCTL_MIC_SWCH_OFF _IOW(MACH5_IOC_MAGIC, 8, unsigned int)
#define MACH5_IOCTL_HP_SWCH_ON _IOW(MACH5_IOC_MAGIC, 9, unsigned int)
#define MACH5_IOCTL_HP_SWCH_OFF _IOW(MACH5_IOC_MAGIC, 10, unsigned int)
#endif
