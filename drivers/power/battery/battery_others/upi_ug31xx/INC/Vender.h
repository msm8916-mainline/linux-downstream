/**
 * @file Vender.h
 *
 *  Vender.h controls the features enabled or disabled
 *
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#ifndef __VENDER_H__
#define __VENDER_H__

#define IC_NAME       "UP3105"

/// =============================================================///
/// Feature area
/// =============================================================///

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 1. Platform 
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_PLAT_WINDOWS                                      ///< (DEFAULT : OFF), This feature enables the compile of WINDOWS
//#define FEATURE_PLAT_LINUX                                        ///< (DEFAULT : OFF), This feature enables the compile of Linux
//#define FEATURE_PLAT_UBOOT                                        ///< (DEFAULT : OFF), This feature enables the compile of uBoot
//#define FEATURE_PLAT_ARM_M0                                       ///< (DEFAULT : OFF), This feature enables the compile of multicell on ARM M0
//#define FEATURE_PLAT_POWERBANK_ARM_M0                             ///< (DEFAULT : OFF), This feature enables the compile of powerbank on ARM M0

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 2. DEBUG
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_PRINT_LOGD                             ///< (DEFAULT : ON), This feature enables print LOGD
#define FEATURE_PRINT_LOGE                             ///< (DEFAULT : ON), This feature enables print LOGE
//#define FEATURE_PRINT_SBS_DATA                                       ///< (DEFAULT : OFF), This feature enables print SBS data
#define FEATURE_PRINT_OTP_DATA                                       ///< (DEFAULT : OFF), This feature enables print OTP data
//#define FEATURE_PRINT_ADC_FACTOR                                   ///< (DEFAULT : OFF), This feature enables print ADC factors
//#define FEATURE_BUFFER_ENABLE                                         ///< (DEFAULT : OFF), This feature enables buffer for saving log
//#define FEATURE_PRINT_SOCKET_LOG                                     ///< (DEFAULT : OFF), This feature enables print socket log

//#define FEATURE_TEST_MAINLOOP                                        ///< (DEFAULT : OFF), This feature enables Test Mainloop

//#define FEATURE_FAKE_MEASUREMENT                                 ///< (DEFAULT : OFF), This feature enables fake measurement
//#define FEATURE_FAKE_I2C_CMD                                        ///< (DEFAULT : OFF), This feature enables fake I2C command
#define FEATURE_DUMP_LOG                                              ///< (DEFAULT : ON), This feature enables print log to dump fiile

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 3. GPIO
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_GIO_ENABLE                                         ///< (DEFAULT : ON), This feature enables and disable the GPIO control


///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 4.  ADC Filter
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATRUE_IIR_FILTER_ENABLE                                 ///< (DEFAULT : ON), This feature enables ADC IIR Filter (Volt, IT,ET)

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 5. Restore from IC
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
#define FEATURE_RESTORE_FROM_IC                                   ///< (DEFAULT : ON), This feature enables restore RM, FCC From IC

///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
/// 6. For ASUS
///-----------------------------------------------------------------------------------------------------------------------------------------------------------///
//#define FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION                   ///< (DEFAULT : OFF), This feature is asus Tiny Board production flow used
//#define FEATURE_ASUS_DRV_VERSION                                  ///< (DEFAULT : OFF), This feature is used for ASUS driver version

#endif ///< __VENDER_H__


