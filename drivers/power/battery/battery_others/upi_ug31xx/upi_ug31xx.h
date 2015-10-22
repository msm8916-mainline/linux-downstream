/**
 * @file upi_ug31xx.h
 *
 * UPI uG31xx gauge driver header
 *
 * @version $Revision$
 * @author AllenTeng <allen.kuoliang.teng@gmail.com>
 * @note
 */

#ifndef __UPI_UG31XX_H__
#define __UPI_UG31XX_H__

/// =============================================
/// Header file
/// =============================================

#include "INC/UpiDef.h"

/// =============================================
/// Definition
/// =============================================

#define UPI_UG31XX_DEV_NAME             ("ug31xx_battery")
#define UPI_UG31XX_POWER_NAME           ("pack_bat")
#define UPI_UG31XX_IOC_NAME             ("upi_ug31xx")
#define UPI_UG31XX_IOC_MAGIC            ('U')
#define UPI_UG31XX_IOC_SBS_RW           (_IOWR(UPI_UG31XX_IOC_MAGIC, 1, unsigned char *))
#define UPI_UG31XX_IOC_SBS_WW           (_IOWR(UPI_UG31XX_IOC_MAGIC, 2, unsigned char *))
#define UPI_UG31XX_IOC_SBS_RP           (_IOWR(UPI_UG31XX_IOC_MAGIC, 3, unsigned char *))
#define UPI_UG31XX_IOC_SBS_WP           (_IOWR(UPI_UG31XX_IOC_MAGIC, 4, unsigned char *))
#define UPI_UG31XX_IOC_CMD(cmd)         ((_IOC_DIR(cmd) << 16) | _IOC_NR(cmd))
#define UPI_UG31XX_CABLE_OUT            (0)
#define UPI_UG31XX_CABLE_IN             (1)
#define UPI_UG31XX_CABLE_POLL_CNT       (5)
#define UPI_UG31XX_CABLE_POLL_TIME      (1)
#define UPI_UG31XX_POWER_CHANGE_TIME    (60)
#define UPI_UG31XX_KBO_PASS_CNT         (20)
#define UPI_UG31XX_KBO_FAIL_CNT         (10)
#define UPI_UG31XX_KBO_INTERVAL_MS      (350)
#define UPI_UG31XX_KBO_UPPER_BOUND      (127)
#define UPI_UG31XX_KBO_LOWER_BOUND      (-127)
#define UPI_UG31XX_KBO_CHK_TIME_MS      (100)
#define UPI_UG31XX_KBO_CHK_TIMEOUT      (1000)
#define UPI_UG31XX_KBO_RTN_PASS         (0)
#define UPI_UG31XX_KBO_RTN_CHK_TIMEOUT  (1)
#define UPI_UG31XX_KBO_RTN_OUT_OF_RANGE (2)
#define UPI_UG31XX_KBO_RTN_UNKNOWN      (3)
#define UPI_UG31XX_KBO_RTN_WT_EEP_FAIL  (4)
#define UPI_UG31XX_KBO_RTN_CMP_EEP_FAIL (5)
#define UPI_UG31XX_KBO_EEPROM_ADDR      (0x4C)
#define UPI_UG31XX_KBO_EEPROM_SIZE      (4)
#define UPI_UG31XX_KBO_CMP_EEP_RETRY    (3)

/// =============================================
/// Variables
/// =============================================

EXTERN unsigned char FactoryGGBXFile_A023Cl[];
EXTERN unsigned char FactoryGGBXFile_A0249l[];
EXTERN unsigned char FactoryGGBXFile_A025Jl[];
EXTERN unsigned char FactoryGGBXFile_A0262h[];
EXTERN unsigned char FactoryGGBXFile_A027Co[];

/// =============================================
/// Functions
/// =============================================

EXTERN GBOOL _I2cExeCmd(GI2CCmdType *pI2CCmd);

#endif  ///< end of __UPI_UG31XX_H__

