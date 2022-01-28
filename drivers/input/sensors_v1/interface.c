/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name		: interface.c
 * Authors		: weitianlei (weitianlei@vivo.com.cn)

 * Version		: V.1.0.0
 * Date			: 2015/3/26
 * Description		: for TW Double-click 
 *
 *******************************************************************************
 *
 * weitianlei add for ts use acc data don't consider the synchronization for if 
 * write then the gsensor has suspend won't has other's control at the same time
 * if read has make some synchronization condition happened the effect is not 
 * serios for just sleep once all recover 
 *
 ******************************************************************************
    Version History.
	V 1.0.0		First Release
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>

bool (*acc_for_ts_judge_dir)(void) = NULL;
EXPORT_SYMBOL_GPL(acc_for_ts_judge_dir);

MODULE_DESCRIPTION("acc interface for TW");
MODULE_AUTHOR("weitianlei (weitianlei@vivo.com.cn)");
MODULE_LICENSE("GPL");