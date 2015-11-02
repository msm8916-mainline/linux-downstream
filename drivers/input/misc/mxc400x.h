/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for mxc400x magnetic sensor chip.
 */
#ifndef __MXC400X_H__
#define __MXC400X_H__

#include <linux/ioctl.h>

#define MXC400X_I2C_NAME		"mxc400x"

#define MXC400X_I2C_ADDR		0x15
#define MXC400X_ID				0x02

#define MXC400X_REG_CTRL		0x0D
#define MXC400X_REG_BITS		0x08
#define MXC400X_REG_DATA		0x03
#define MXC400X_REG_PRODUCTID_0		0x0E
#define MXC400X_REG_TEMP        0x09

#define MXC400X_AWAKE		0x40	/* power on */
#define MXC400X_SLEEP		0x01	/* power donw */

#define MXC400X_BW_50HZ    0x00
#define MXC400X_RANGE_2G   0x00
#define MXC400X_RANGE_4G   0x20
#define MXC400X_RANGE_8G   0x40

/* Use 'm' as magic number */
#define MXC400X_IOM			'm'

/* IOCTLs for MXC400X device */

#define MXC400X_IOCTL_READ_REG           _IOWR(MXC400X_IOM, 0x23, unsigned char)
#define MXC400X_IOCTL_WRITE_REG          _IOW(MXC400X_IOM, 0x24, unsigned char[2])  
#define MXC400X_IOCTL_READXYZ		 _IOR(MXC400X_IOM, 0x03, int[4])


#endif /* __MXC400X_H__ */

