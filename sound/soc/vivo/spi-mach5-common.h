/*
 * mach5 driver
 *
 * Copyright (c) 2014 Yamaha Corporation
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

struct read_reg {
	unsigned long	address;	/* I/F Address 8*/
	void __user *data;		/* Read Pointer 12*/
	unsigned int	size;		/* Read Size 4*/
};

struct write_reg {
	void __user *data;		/* Write Pointer 12*/
	unsigned int	size;		/* Write Size 4 8*/
};

#ifdef CONFIG_COMPAT
struct read_reg32 {
	u32	address;	/* I/F Address 8*/
	u32 data;		/* Read Pointer 12*/
	u32	size;		/* Read Size 4*/
};

struct write_reg32 {
	u32 data;		/* Write Pointer 12*/
	u32	size;		/* Write Size 4 8*/
};
#endif

#define MACH5_IOC_MAGIC		'x'
#define MACH5_IOCTL_READ_REG	_IOR(MACH5_IOC_MAGIC, 0, struct read_reg)
#define MACH5_IOCTL_WRITE_REG	_IOW(MACH5_IOC_MAGIC, 1, struct write_reg)
#define MACH5_IOCTL_CLOCK_ON	_IOW(MACH5_IOC_MAGIC, 2, unsigned int)
#define MACH5_IOCTL_CLOCK_OFF	_IOW(MACH5_IOC_MAGIC, 3, unsigned int)
#define MACH5_IOCTL_LDOD_ON		_IOW(MACH5_IOC_MAGIC, 4, unsigned int)
#define MACH5_IOCTL_LDOD_OFF	_IOW(MACH5_IOC_MAGIC, 5, unsigned int)

#define MACH5_IOCTL_MIC_SWCH_ON _IOW(MACH5_IOC_MAGIC, 7, unsigned int)
#define MACH5_IOCTL_MIC_SWCH_OFF _IOW(MACH5_IOC_MAGIC, 8, unsigned int)
#define MACH5_IOCTL_HP_SWCH_ON _IOW(MACH5_IOC_MAGIC, 9, unsigned int)
#define MACH5_IOCTL_HP_SWCH_OFF _IOW(MACH5_IOC_MAGIC, 10, unsigned int)
#endif
