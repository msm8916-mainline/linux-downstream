/*
 * =====================================================================================
 *
 *       Filename:  vivo_sensor_config.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  04/01/2015 09:48:48 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  haosirong (haosirong@vivo.com.cn), 
 *        Company:  vivo
 *
 * =====================================================================================
 */

 #ifndef __VIVO_SENSOR_CONFIG_H__
 #define __VIVO_SENSOR_CONFIG_H__
 
 #include <linux/bbk_alsps_para.h>

typedef struct {
   const char  *product_name;
}sensor_config;
 
extern int bbk_ps_para_count;
extern int bbk_als_para_count;

extern struct ps_para *bbk_ps_paras;
extern struct als_para *bbk_als_paras;

extern unsigned char bbk_product_model_for_alsps[];
 #endif
