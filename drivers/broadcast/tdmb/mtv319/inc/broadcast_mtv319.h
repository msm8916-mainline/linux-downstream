/**===================================================================
 * Copyright(c) 2009 LG Electronics Inc. All Rights Reserved
 *
 * File Name : broadcast_mtv319.h
 * Description : EDIT HISTORY FOR MODULE
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 * when			model		who			what
 * 10.27.2009		android		inb612		Create for Android platform
====================================================================**/
#ifndef _BROADCAST_MTV319_H_
#define _BROADCAST_MTV319_H_
#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"

typedef struct
{
	void		(*tdmb_pwr_on)(void);
	void		(*tdmb_pwr_off)(void);
}broadcast_pwr_func;

struct broadcast_tdmb_data
{
	void (*pwr_on)(void);
	void (*pwr_off)(void);
};

extern int broadcast_mtv319_drv_if_power_on(void);
extern int broadcast_mtv319_drv_if_power_off(void);
extern int broadcast_mtv319_drv_if_init(void);
extern int broadcast_mtv319_drv_if_stop(void);
extern int broadcast_mtv319_drv_if_set_channel(unsigned int freq_num, unsigned int subch_id, unsigned int op_mode);
extern int broadcast_mtv319_drv_if_detect_sync(int op_mode);
extern int broadcast_mtv319_drv_if_get_sig_info(struct broadcast_tdmb_sig_info *dmb_bb_info);
extern int broadcast_mtv319_drv_if_get_fic(char* buffer, unsigned int* buffer_size);
extern int broadcast_mtv319_drv_if_get_msc(char** buffer_ptr, unsigned int* buffer_size, unsigned int user_buffer_size);
extern int broadcast_mtv319_drv_if_reset_ch(void);
extern int broadcast_mtv319_drv_if_user_stop(int mode);
extern int broadcast_mtv319_drv_if_select_antenna(unsigned int sel);
extern int broadcast_mtv319_drv_if_isr(void);

int tdmb_mtv319_power_on(void);
int tdmb_mtv319_power_off(void);
int tdmb_mtv319_select_antenna(unsigned int sel);
int tdmb_mtv319_i2c_write_burst(uint16 waddr, uint8* wdata, int length);
int tdmb_mtv319_i2c_read_burst(uint16 raddr, uint8* rdata, int length);
int tdmb_mtv319_mdelay(int32 ms);
void tdmb_mtv319_Must_mdelay(int32 ms);
void tdmb_mtv319_interrupt_lock(void);
void tdmb_mtv319_interrupt_free(void);
int tdmb_mtv319_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length);
void tdmb_mtv319_set_userstop(int mode);
int tdmb_mtv319_tdmb_is_on(void);

#if defined(CONFIG_ARCH_MSM8994)
#define __broadcast_dev_exit_p(x)        x
#define __broadcast_dev_init            __init
#elif defined(CONFIG_ARCH_MSM8226)
#define __broadcast_dev_exit_p(x)        __devexit_p(x)
#define __broadcast_dev_init            __devinit
#elif defined(CONFIG_ARCH_MSM8916)
#define __broadcast_dev_exit_p(x)        x
#define __broadcast_dev_init            __devinit
#endif

#endif

