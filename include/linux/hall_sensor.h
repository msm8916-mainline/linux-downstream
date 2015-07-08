/*******************************************************************************************
* Copyright 2012 xxx Corporation. All rights reserved.
*
* Unless you and xxx execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this software
* in any way with any other Broadcom software provided under a license other than
* the GPL, without xxx's express prior written consent.
* *******************************************************************************************/
struct hall_platform_data {
	const char* name;
	unsigned int int_gpio;
	unsigned int input_type;
	unsigned int active_gpio_level;
	unsigned int active_code;
	unsigned int inactive_code;
	unsigned int debounce_interval;
};

