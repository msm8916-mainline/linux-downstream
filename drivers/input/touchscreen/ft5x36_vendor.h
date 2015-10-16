/*
 focaltelech tp firmware update infomations
 
 Date           Author       Module    vendor id    Old_ver    New_ver
 2015.06.12     pangle       shenyue     0xA0         null       0x01
 2015.07.15     pangle       shenyue     0xA0         0x01       0x0B
 2015.07.27     pangle       shenyue     0xA0         0x0B       0x0C
 2015.07.28     pangle       shenyue     0xA0         0x0C       0x0D
 2015.08.05     pangle       shenyue     0xA0         0x0D       0x0E
 2015.08.12     pangle       shenyue     0xA0         0x0E       0x0F
 */
#ifndef __FOCALTECH_VENDOR_H__
#define __FOCALTECH_VENDOR_H__

#include "ft5x36_vendor_id.h"


#if defined(CONFIG_QL620_BASE) || defined(CONFIG_QW620_BASE)
static unsigned char FT6X36_FIRMWARE0x0A_TOPTOUCH[] = {
#if defined(CONFIG_QL620_BASE) || defined(CONFIG_QW620_BASE)
	#include "ft5x36_firmware/QW620_QL620_acer_FT6336_0xA0_Ver0x0F_20150810.h"
#endif
}; 
#endif

//added by pangle for 860 fw upgrade at 20150413 begin
#if defined(CONFIG_QL860_BASE)
static unsigned char FT6X36_FIRMWARE0x9b_JINLONG[] = {
#if defined(CONFIG_QL860_BASE)
	//#include "ft5x36_firmware/QL860_IBD_Ft6336_JINLONG0x9b_Ver0x11_20150413.h"
	#include "ft5x36_firmware/QL860_IBD_Ft6336_JINLONG0x9b_Ver0x14_20150521.h"
#endif
}; 
#endif
//added by pangle for 860 fw upgrade at 20150413 begin



#endif
