/* include/linux/i2c/elan_ektf2k.h - ELAN EKTF2136 touchscreen driver
 *opyright (C) 2011 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define ELAN_X_MAX 720

#define ELAN_Y_MAX 1280

#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H
#define ELAN_KTF2K_NAME "ekt3xxx"

struct elan_ktf2k_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;

	const char *name;
	u32 irq_gpio;
	u32 reset_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio_flags;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	u32 irqflags;
};

#endif /* _LINUX_ELAN_KTF2K_H */

#ifdef IAP_PORTION
int Update_FW_One(struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);
#endif
