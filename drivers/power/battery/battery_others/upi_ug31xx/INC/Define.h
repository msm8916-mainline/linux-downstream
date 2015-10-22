/**
 * @file define.h
 *
 *  Define and macro for all the source and header file
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#ifndef __DEFINE_H__
#define __DEFINE_H__

///-------------------------------------------------------------------------------///
/// General typedef
///-------------------------------------------------------------------------------///
/// GDWORD
typedef unsigned int         GDWORD;
/// GSHORT
typedef short                GSHORT;
/// GWORD
typedef unsigned short       GWORD;
/// GBYTE
typedef unsigned char        GBOOL;
/// GBYTE
typedef unsigned char        GBYTE;
/// GCHAR
typedef char                 GCHAR;
/// GSigned byte
typedef char                 GSBYTE;
/// GU4
typedef unsigned int         GU4;
/// GU2
typedef unsigned short       GU2;
/// GU1
typedef unsigned char        GU1;

/// int64
typedef long long            GINT64;

/// int32
typedef int                  GINT32;

/// int
typedef int                  GINT;

/// int32
typedef short                GINT16;

/// GI4
typedef long                 GI4;
/// GI2
typedef short                GI2;
/// GI1
typedef char                 GI1;
/// GVOID
#if defined(FEATURE_PLAT_ARM_M0)
#define GVOID                void
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
#define GVOID                void
#else  ///< for FEATURE_PLAT_POWERBANK_ARM_M0
typedef void                 GVOID;
#endif ///< for FEATURE_PLAT_POWERBANK_ARM_M0

///-------------------------------------------------------------------------------///
/// General define
///-------------------------------------------------------------------------------///

#define MAX_CELL_NUM      (3)
#define CELL_NUM_2        (2)
#define CELL_NUM_3        (3)

///-------------------------------------------------------------------------------///
/// Boolean define
///-------------------------------------------------------------------------------///
#define GTRUE        (1)
#define GFALSE       (0)

///-------------------------------------------------------------------------------///
///  Register Declaration
///-------------------------------------------------------------------------------///
#define EXTERN extern
#define STATIC static
#define CONST const

#if defined(FEATURE_PLAT_WINDOWS)
#define PACK
#define PACKRAM
#define __func__ __FUNCTION__
#else  ///< FEATURE_PLAT_WINDOWS 
  #if defined(FEATURE_PLAT_ARM_M0)
  #define PACK __attribute__ ((packed))
  #define PACKRAM
  #elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  #define PACK __attribute__ ((packed))
  #define PACKRAM
  #else  ///< FEATURE_PLAT_POWERBANK_ARM_M0
  #define PACK  __attribute__ ((__packed__))
  #define PACKRAM  __attribute__ ((__packed__))
  #endif///< for FEATURE_PLAT_POWERBANK_ARM_M0
#endif ///< for FEATURE_PLAT_WINDOWS

///-------------------------------------------------------------------------------///
/// Log
///-------------------------------------------------------------------------------///
#ifdef FEATURE_PRINT_LOGD
#define GLOGD     UpiPrintDbg
#define GLOGE     UpiPrintDbgE
#else ///< FEATURE_PRINT_LOGD
#define GLOGD     //
#define GLOGE     //
#endif  ///< FEATURE_PRINT_LOGD
#define DUMP_FILE_NAME        ("DUMP")
#define DUMP_FILE_SIZE        (1024*1024*10)  ///< 10MB
#define DUMP_KEY_WORD         ("UPI>")
#define DUMP_BUFFER_SIZE      (1024*16)       ///< 16kB


///-------------------------------------------------------------------------------///
/// BYTE Operation
///-------------------------------------------------------------------------------///
#define BYTE0(x) (GBYTE)((GWORD)x)
#define BYTE1(x) (GBYTE)((GWORD)x>>8)

#define GLOBYTE(x) (GBYTE)((GWORD)(x))
#define GHIBYTE(x) (GBYTE)((GWORD)(x)>>8)

///-------------------------------------------------------------------------------///
/// BYTE swap high/low bytes
///-------------------------------------------------------------------------------///
#define BYTE_SWAP(x)  (x=(((x)&0xff)<< 8) | ((GWORD)(x)>>8))

/// Big Endian to Little Endian
#define BE2LE(x)      (GWORD)((((x)&0xff)<< 8) | ((GWORD)(x)>>8))

/// Little Endian to Big Endian
#define LE2BE(x)      (GWORD)((((x)&0xff)<< 8) | ((GWORD)(x)>>8))

///-------------------------------------------------------------------------------///
/// Register operation
///-------------------------------------------------------------------------------///
#define READ_REG(REG)                   ReadRegByte(REG)
#define WRITE_REG(REG,VALUE)            WriteRegByte(REG,VALUE)

#define READ_REG16(REG)                 ReadRegWord(REG)            
#define WRITE_REG16(REG,VALUE)          WriteRegWord(REG,VALUE)

//#define READ_REG32(REG)                 
//#define WRITE_REG32(REG,VALUE)          

#define SET_BIT(REG, BIT)               WRITE_REG(REG, READ_REG(REG) | (BIT));
#define UNSET_BIT(REG, BIT)             WRITE_REG(REG, (READ_REG(REG) & (GBYTE)(~(BIT))));
#define CLR_BIT(REG, BIT)               WRITE_REG(REG, (READ_REG(REG) & (GBYTE)(~(BIT))));

///-------------------------------------------------------------------------------///
/// Mememoy Control
///-------------------------------------------------------------------------------///
#define UPI_MEMSET(d, c, n)     UpiMemset(d, c, n)
#define UPI_MEMCPY(s, d, n)     UpiMemcpy(s, d, n)
#define UPI_ABS(s)              UpiAbs(s)

///-------------------------------------------------------------------------------///
/// Time
///-------------------------------------------------------------------------------///
#define UPI_LOCALTIME(x)        UpiGetLocalTime(x)
#define UPI_SLEEP(x)            UpiSleep(x)

///-------------------------------------------------------------------------------///
/// I2C Control
///-------------------------------------------------------------------------------///
#define I2C_SLAVE_ADDR          (0x70) ///< I2C Slave Address
#define I2C_SECURE_ADDR         (0x80) ///< If register address > 0x80, then do security read
#define I2C_SECURE_CODE         (0x5A)

#define I2C_MAX_RETRY_CNT       (10)      ///<  max register read retry count

#define I2C_STS_NO_ERR          (0)
#define I2C_STS_SLAVE_NOT_EXIST (1<<0)    ///< 0: normal, 1: slave is not existed

///-------------------------------------------------------------------------------///
/// EEPROM
///-------------------------------------------------------------------------------///
#define EEPROM_SLAVE_ADDR       (0x51) ///< i2c slave address
#define EEPROM_WRITE_SIZE       (4)
#define EEPROM_START_REG        (0x4C)
#define EEPROM_START_BYTE       (0x46)
#define EEPROM_EXT_REG          (0xAB)

///-------------------------------------------------------------------------------///
/// State
///-------------------------------------------------------------------------------///
#define STATE_INIT              (0)
#define STATE_STANDBY           (1)
#define STATE_DISCHARGING       (2)
#define STATE_CHARGING          (3)
#define STATE_FULL_CHARGED      (4)
#define STATE_FULL_DISCHARGED   (5)

///-------------------------------------------------------------------------------///
/// System Mode
///-------------------------------------------------------------------------------///
#define SYS_MODE_PAUSE                (1<<0)  ///<  BIT0 : 0-->Run ; 1 --> PAUSE
#define SYS_MODE_ACTIVATE             (1<<1)  ///<  BIT1 : 0-->N/A ; 1 --> activate system
#define SYS_MODE_DEACTIVATE           (1<<2)  ///<  BIT2 : 0-->N/A ; 1 --> deactive system 
#define SYS_MODE_INIT_CAP             (1<<3)  ///<  BIT3 : 0-->N/A ; 1 --> Capacity is initialized 
#define SYS_MODE_RESTORED             (1<<4)  ///<  BIT4 : 0-->N/A ; 1 --> Capacity is initialized 
#define SYS_MODE_FULL_DEBUG_MSG       (1<<5)  ///<  BIT5 : 0-->N/A ; 1 --> Full debug message
#define SYS_MODE_MEAS_STABLE          (1<<6)  ///<  BIT6 : 0-->N/A ; 1 --> Measurement stable
#define SYS_MODE_SAVE_DATA_START      (1<<7)  ///<  BIT7 : 0-->N/A ; 1 --> Save data to IC is started

///-------------------------------------------------------------------------------///
/// ADC
///-------------------------------------------------------------------------------///
#define ADC1_QUEUE_SIZE         (4)
#define ADC2_QUEUE_SIZE         (3)

#define ADC2_VCELL1             (0)
#define ADC2_VCELL2             (1)
#define ADC2_VCELL3             (2)

#define ADC2_FAIL_CRITERIA      (5)

#ifdef  FEATRUE_IIR_FILTER_ENABLE

#define ADC_FILTER_VCELL1(code) (ADCFilterVCell(ADC2_VCELL1, code))
#define ADC_FILTER_VCELL2(code) (ADCFilterVCell(ADC2_VCELL2, code))
#define ADC_FILTER_VCELL3(code) (ADCFilterVCell(ADC2_VCELL3, code))

#else   ///< else of FEATRUE_IIR_FILTER_ENABLE

#define ADC_FILTER_VCELL1(code) (code)
#define ADC_FILTER_VCELL2(code) (code)
#define ADC_FILTER_VCELL3(code) (code)

#endif  ///< end of FEATRUE_IIR_FILTER_ENABLE

#define ADC_FILTER_CURR_THRD    (256)

///---------------------------------------------------------///
/// ADC1 Factor Calculation
///---------------------------------------------------------///
#define ADC_TEMPERATURE_GAIN_CONST            (1000)

#define ADC1_CODE_100MV_NEGATIVE              (0xFF00)
#define ADC1_CODE_200MV_NEGATIVE              (0xFE00)
#define ADC1_CP_CODE_25_100MV                 (12288)
#define ADC1_CP_CODE_25_200MV                 (24576)
#define ADC1_DELTA_CODE_25_100MV_SIGN_BIT     (1<<8)
#define ADC1_DELTA_CODE_25_200MV_SIGN_BIT     (1<<9)
#define ADC1_TEMPERATURE_GAIN_100MV           (869600)
#define ADC1_TEMPERATURE_GAIN_200MV           (-695680)
#define ADC1_TEMPERATURE_OFFSET_SIGN_BIT      (1<<8)
#define ADC1_TEMPERATURE_OFFSET_NEGATIVE      (0xFF00)

///---------------------------------------------------------///
/// ADC1 Calibrate Factor
///---------------------------------------------------------///
#define ADC1_IDEAL_CODE_100MV     (614)
#define ADC1_IDEAL_CODE_200MV     (1229)
#define ADC1_IDEAL_CODE_DELTA     (ADC1_IDEAL_CODE_200MV - ADC1_IDEAL_CODE_100MV)

///---------------------------------------------------------///
/// ADC2 Factor Calculation
///---------------------------------------------------------///
#define ADC2_CODE_100MV_NEGATIVE              (0xFFC0)
#define ADC2_CODE_200MV_NEGATIVE              (0xFF80)
#define ADC2_CP_CODE_25_100MV                 (3072)
#define ADC2_CP_CODE_25_200MV                 (6144)
#define ADC2_DELTA_CODE_25_100MV_SIGN_BIT     (1<<6)
#define ADC2_DELTA_CODE_25_200MV_SIGN_BIT     (1<<7)
#define ADC2_TEMPERATURE_GAIN_100MV           (-149130)
#define ADC2_TEMPERATURE_GAIN_200MV           (-136937)

///---------------------------------------------------------///
/// ADC2 Ideal Code ( For Voltage Measurement)
///---------------------------------------------------------///
#define ADC2_IDEAL_CODE_100MV     (ADC2_CP_CODE_25_100MV)
#define ADC2_IDEAL_CODE_200MV     (ADC2_CP_CODE_25_200MV)
#define ADC2_IDEAL_CODE_DELTA     (ADC2_IDEAL_CODE_200MV - ADC2_IDEAL_CODE_100MV)

///---------------------------------------------------------///
///  Ideal IT code for 25 and 80 oC
///---------------------------------------------------------///
#define IT_IDEAL_CODE_25      (24310)
#define IT_IDEAL_CODE_80      (28612)
#define IT_IDEAL_CODE_DELTA   (IT_IDEAL_CODE_80 - IT_IDEAL_CODE_25)

///---------------------------------------------------------///
///  VOLTAGE code Threshod
///---------------------------------------------------------///
#define VOLT_THRESHOD_HIGHER    (6685)  ///< 4500 mv  
#define VOLT_THRESHOD_LOWER     (4313)   ///< 2900 mv

///---------------------------------------------------------///
///  Ideal IT code Gain and Offset
///---------------------------------------------------------///
#define AMBIENT_TEMPERATURE_IN_FT (220)
#define IT_CONST                  (100)
#define IT_GAIN                   (392)
#define IT_OFFSET                 (11172)

///-------------------------------------------------------------------------------///
/// Mearurement Data
///-------------------------------------------------------------------------------///
#define MAX_MEAS_QUEUE_SIZE     (5)
#define INIT_DELTAQ_INDEX       (MAX_MEAS_QUEUE_SIZE - 1)     ///> [YL]: In order to avoid assign the same buffer when gauge power on 
                                                              ///        cause pre deltaQ and now deltaQ the smae ; 2015/08/11

///-------------------------------------------------------------------------------///
/// Mearurement : Current
///-------------------------------------------------------------------------------///
#define ADC1_VOLTAGE_100MV    (-5000)                                   ///< [AT-PM] : Unit in uV ; 01/25/2013
#define ADC1_VOLTAGE_200MV    (-10000)                                  ///< [AT-PM] : Unit in uV ; 01/25/2013
#define ADC1_VOLTAGE_DELTA    (ADC1_VOLTAGE_200MV - ADC1_VOLTAGE_100MV)

///-------------------------------------------------------------------------------///
/// Mearurement : Capacity Conversion (Couloumb)
///-------------------------------------------------------------------------------///
#define COULOMB_COUNTER_WEIGHT           (4096)   ///< 2^12, Coulomb Counter accumulate current with left shift :  Sum =  ADC1-Conv-N-times * (ave_curr- adc1-offset)/2 ==>  Sum >> 12

///-------------------------------------------------------------------------------///
/// Mearurement : Voltage
///-------------------------------------------------------------------------------///
#define CALIBRATION_FACTOR_CONST    (1000)

#define ADC2_VOLTAGE_100MV    (3000)                                    ///< [AT-PM] : Unit in mV ; 01/25/2013
#define ADC2_VOLTAGE_200MV    (4000)                                    ///< [AT-PM] : Unit in mV ; 01/25/2013
#define ADC2_VOLTAGE_DELTA    (ADC2_VOLTAGE_200MV - ADC2_VOLTAGE_100MV)

///---------------------------------------------------------///
/// Mearurement : External Temperature
///---------------------------------------------------------///
#define MAX_ET_CODE_DIFF  (200)
#define MIN_ET_CODE_DIFF  (-200)
#define ET_NUMS           (19)      ///< [AT-PM] : -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80 ; 01/25/2013

///-------------------------------------------------------------------------------///
/// Measurement : Board Factor
///-------------------------------------------------------------------------------///
#define BOARD_FACTOR_CONST                  (1000)

///-------------------------------------------------------------------------------///
/// Time Constant
///-------------------------------------------------------------------------------///
#define TIME_MSEC_TO_HOUR                 (3600*1000) ///< 1HR = 3600_1000 MSEC
#define TIME_SEC_TO_HOUR                  (3600)  ///< 1HR = 3600 SEC
#define TIME_MSEC_TO_SEC                  (1000)  ///< 1SEC = 1000 MS
#define TIME_SEC_TO_MIN                   (60)    ///< 1MIN =  60 SEC
#define TIME_MIN_TO_HOUR                  (60)    ///< 1HR = 60 MIN

#define TIME_NORMAL_NEXT_ROUND            (1000)
#define TIME_RESET_NEXT_ROUND             (300)
#define TIME_CALIBRATE_ROUND              (350)

///-------------------------------------------------------------------------------///
/// CONFIG
///-------------------------------------------------------------------------------///
#define OPCFG_SBS_TEMP_SOURCE                    (1<<0)
  #define OPCFG_SBS_TEMP_SOURCE_INT              (0<<0)
  #define OPCFG_SBS_TEMP_SOURCE_EXT              (1<<0)
#define OPCFG_PRODUCT_TYPE                       (3<<6)
  #define OPCFG_PRODUCT_TYPE_UG3105              (0<<6)
  #define OPCFG_PRODUCT_TYPE_UG310X              (1<<6)
  #define OPCFG_PRODUCT_TYPE_RSVD2               (2<<6)
  #define OPCFG_PRODUCT_TYPE_RSVD3               (3<<6)

#define GET_PRODUCT_TYPE(x)       ((x & OPCFG_PRODUCT_TYPE) >> 6)

///-------------------------------------------------------------------------------///
/// SBS
///-------------------------------------------------------------------------------///
#define SBS_ADC_EOC_VTM           (1<<0)
#define SBS_ADC_EOC_GG            (1<<1)
#define SBS_ADC_EOC_IT            (1<<2)
#define SBS_ADC_EOC_ET            (1<<3)

#define SBS_RSOC_TIMER_INIT       (0xffff)
#define SBS_RSOC_TIMER_MIN(x)     ((x & 0xff00) >> 8)
#define SBS_RSOC_TIMER_SEC(x)     ((x & 0x00ff))

#define SBS_RSOC_OPTIMIZE_TIMEOUT (100)

///-------------------------------------------------------------------------------///
/// OTP Control
///-------------------------------------------------------------------------------///
#define OTP_RAW_SIZE          (4)
#define OTP_SIGN_BYTE_SHIFT   (256)

#define OTP1_E0_INDEX         (0)
#define OTP1_E1_INDEX         (OTP1_E0_INDEX + 1)
#define OTP1_E2_INDEX         (OTP1_E1_INDEX + 1)
#define OTP1_E3_INDEX         (OTP1_E2_INDEX + 1)

#define OTP2_F0_INDEX         (0)
#define OTP2_F1_INDEX         (OTP2_F0_INDEX + 1)
#define OTP2_F2_INDEX         (OTP2_F1_INDEX + 1)
#define OTP2_F3_INDEX         (OTP2_F2_INDEX + 1)

#define OTP3_F4_INDEX         (0)
#define OTP3_F5_INDEX         (OTP3_F4_INDEX + 1)
#define OTP3_F6_INDEX         (OTP3_F5_INDEX + 1)
#define OTP3_F7_INDEX         (OTP3_F6_INDEX + 1)

#define OTP4_F8_INDEX         (0)
#define OTP4_F9_INDEX         (OTP4_F8_INDEX + 1)
#define OTP4_FA_INDEX         (OTP4_F9_INDEX + 1)
#define OTP4_FB_INDEX         (OTP4_FA_INDEX + 1)

#define OTP5_FC_INDEX         (0)
#define OTP5_FD_INDEX         (OTP5_FC_INDEX + 1)
#define OTP5_FE_INDEX         (OTP5_FD_INDEX + 1)
#define OTP5_FF_INDEX         (OTP5_FE_INDEX + 1)

#define OTP6_70_INDEX         (0)
#define OTP6_71_INDEX         (OTP6_70_INDEX + 1)
#define OTP6_72_INDEX         (OTP6_71_INDEX + 1)
#define OTP6_73_INDEX         (OTP6_72_INDEX + 1)

///-------------------------------------------------------------------------------///
/// OTP Data
///-------------------------------------------------------------------------------///

#define OTP_PRODUCT_TYPE_0      (0)     ///< Mass production
#define OTP_PRODUCT_TYPE_1      (1)     ///< ADC1 offset temperature compensation
#define OTP_PRODUCT_TYPE_2      (2)
#define OTP_PRODUCT_TYPE_3      (3)

///-------------------------------------------------------------------------------///
/// Gauge Status
///-------------------------------------------------------------------------------///
#define GAUGE_STATUS_RESET_DONE         (1<<0)   ///< Reset done flag will be set when loop > 5
  #define GAUGE_RESET_DONE_COUNT          (3)
#define GAUGE_STATUS_SKIP_ROUND         (1<<1)   ///< Skip round
#define GAUGE_STATUS_OTP_LOADED         (1<<2)   ///< OTP Loaded
#define GAUGE_STATUS_OTP_LOADED_PRINT   (1<<3)   ///< OTP Loaded Print
#define GAUGE_STATUS_CONFIG_LOADED      (1<<4)   ///< Config file loaded
#define GAUGE_STATUS_MEAS_STABLE        (1<<5)   ///< Measurment data is stable
  #define GAUGE_MEAS_STABLE_COUNT         (GAUGE_RESET_DONE_COUNT + 3)
#define GAUGE_STATUS_SERVICE_SBS_CMD    (1<<6)   ///< Service SBS command

///-------------------------------------------------------------------------------///
/// Gauge ERR
///-------------------------------------------------------------------------------///
#define GAUGE_ERR_NO_ERROR              (0x0000) ///< No Error

#define GAUGE_ERR_I2C_FAIL              (1<<0)   ///< I2C transfer failed 

#define GAUGE_ERR_LV_FAIL               (1<<1)   ///< low voltage failed 
#define GAUGE_ERR_ADC1_IT_GAIN_FAIL     (1<<2)   ///< ADC1 IT Gain Error, ex: Divide by Zero 
#define GAUGE_ERR_ADC1_GAIN_FAIL        (1<<3)   ///< ADC1 Curr Gain Error, ex: Divide by Zero 
#define GAUGE_ERR_ADC1_CAP_GAIN_FAIL    (1<<4)   ///< ADC1 Cap Gain Error,Divide by Zero 
#define GAUGE_ERR_ADC2_GAIN_FAIL        (1<<5)   ///< ADC2 Gain Error,Divide by Zero 
#define GAUGE_ERR_GG_RUN_SET_FAIL       (1<<6)   ///< GG_RUN is cleared error
#define GAUGE_ERR_OTP_CHECK_FAIL        (1<<7)   ///< OTP data all 0 error
#define GAUGE_ERR_BGR_TUNE_FAIL         (1<<8)   ///< BGRTUNE out of range error

///-------------------------------------------------------------------------------///
/// Gauge Warning
///-------------------------------------------------------------------------------///
#define GAUGE_WARN_NO_WARNING           (0x0000) ///< No Warning

#define GAUGE_WARN_COUNTER_FIXED_FAIL   (1<<0)   ///< Code counter fixed Warning
#define GAUGE_WARN_VOLT_FIXED_FAIL      (1<<1)   ///< Voltage code fixed Warning
#define GAUGE_WARN_CONFIG_SAVE_FAIL     (1<<2)   ///< Save to config file failed warning

///-------------------------------------------------------------------------------///
/// Alarm Status
///-------------------------------------------------------------------------------///
#define ALARM_STS_MASK_EN                 (255<<8)
#define ALARM_STS_UV_EN                   (1<<8)
#define ALARM_STS_MASK_VOLT               (3<<0)
#define ALARM_STS_OV                      (1<<1)
#define ALARM_STS_UV                      (1<<0)

#define ALARM_THRD_MAX_VALUE              (65535)
#define ALARM_THRD_MIN_VALUE              (0)
///-------------------------------------------------------------------------------///
/// Temperature Index
///-------------------------------------------------------------------------------///
#define TEMP_IDX_HT                       (0)   ///< Temperature Index for HT ,   Temperature > (45 + 25)/2 = 35 oC 
#define TEMP_IDX_MT                       (1)   ///< Temperature Index for MT1 ,  Temperature > (25 + 15)/2 = 20 oC 
#define TEMP_IDX_LT1                      (2)   ///< Temperature Index for MT ,   Temperature > (15 +  5)/2 = 10 oC 
#define TEMP_IDX_LT                       (3)   ///< Temperature Index for LT1 ,  Temperature < 10 oC 

//#define TEMP_IDX_OT                       (4)   ///< Temperature Index for HT ,   Temperature > 55 oC
//#define TEMP_IDX_UT                       (5)   ///< Temperature Index for HT ,   Temperature > 55 oC

///-------------------------------------------------------------------------------///
/// Capacity Algorithm
///-------------------------------------------------------------------------------///
#define CAP_BATT_LOG_HEADER               ("<BATT>[UG][CP]")

#define CAP_CNTL_RESET                    (1<<0)
#define CAP_CNTL_SET_FULL                 (1<<1)
#define CAP_CNTL_SET_RSOC                 (1<<2)

#define CAP_CHG_STATE_FULL_CHARGE         (1<<0)

#define CAP_DSG_STATE_QD                  (1<<0)
#define CAP_DSG_STATE_CROSS_EDV0          (1<<1)
#define CAP_DSG_STATE_CROSS_EDV1          (1<<2)
#define CAP_DSG_STATE_CROSS_EDV2          (1<<3)
#define CAP_DSG_STATE_CROSS_EDV3          (1<<4)
#define CAP_DSG_STATE_REACH_EDV0          (1<<5)
#define CAP_DSG_STATE_REACH_EDV1          (1<<6)
#define CAP_DSG_STATE_REACH_EDV2          (1<<7)
#define CAP_DSG_STATE_REACH_EDV3          (1<<8)

#define CAP_OCV_TABLE_ELEMENT_CNT         (21)

#define CAP_UPDATE_PTR_CNT                (4)

#define CAP_CONST_RSOC_UNIT               (1000)
#define CAP_CONST_PERCENTAGE              (100)
#define CAP_CONST_EDV_DELAY_SEC           (3)
#define CAP_CONST_RM_STEP_SIZE            (200)
#define CAP_CONST_ROUND_BASE              (10)
#define CAP_CONST_ROUND_BY_5              (5)

#define CAP_SBS_CMD_RESET                 (0x0001)
#define CAP_SBS_CMD_SET_FULL              (0x0002)
#define CAP_SBS_CMD_SET_RSOC              (0x0003)      ///< [AT-PM] : The target RSOC is set at high byte ; 08/26/2015
  #define CAP_SBS_CMD_SET_RSOC_MASK       (0x00ff)
  #define CAP_SBS_CMD_SET_RSOC_VALUE      (0xff00)

///-------------------------------------------------------------------------------///
/// Voltage Gauge Capacity Algorithm
///-------------------------------------------------------------------------------///
#define VCAP_BATT_LOG_HEADER              ("<BATT>[UG][VCP]")

#define VCAP_CNTL_SET_FULL                (1<<0)

#define VCAP_CHG_STATE_FULL_CHARGE        (1<<0)

#define VCAP_DSG_STATE_QD                 (1<<0)
#define VCAP_DSG_STATE_CROSS_EDV0         (1<<1)
#define VCAP_DSG_STATE_CROSS_EDV1         (1<<2)
#define VCAP_DSG_STATE_CROSS_EDV2         (1<<3)
#define VCAP_DSG_STATE_REACH_EDV0         (1<<5)
#define VCAP_DSG_STATE_REACH_EDV1         (1<<6)
#define VCAP_DSG_STATE_REACH_EDV2         (1<<7)

#define VCAP_OCV_TABLE_ELEMENT_CNT        (21)

#define VCAP_UPDATE_PTR_CNT               (3)

#define VCAP_CONST_RSOC_UNIT              (1000)
#define VCAP_CONST_PERCENTAGE             (100)
#define VCAP_CONST_EDV_DELAY_SEC          (3)
#define VCAP_CONST_RM_STEP_SIZE           (200)
#define VCAP_CONST_CURRENT                (100)
#define VCAP_CONST_NEAR_FULL_TIME         (30)
#define VCAP_CONST_NEAR_FULL_RSOC         (990)

#define VCAP_CHG_TABLE_MAX                (900)

#define VCAP_SBS_CMD_SET_FULL             (0x0002)

///-------------------------------------------------------------------------------///
/// Config Default Data
///-------------------------------------------------------------------------------///
#define CONFIG_UPDATE_INTERVAL        (30*60*1000)

#define CONFIG_BOARD_OFFSET_SIGN      (0x46)      ///< [AT-PM] : 'F' for normal board offset ; 05/08/2015
#define CONFIG_BOARD_OFFSET_LSIGN     (0x66)      ///< [AT-PM] : 'f' for large board offset ; 05/08/2015

#define CONFIG_MAP_IDX_RM             (0)
#define CONFIG_MAP_IDX_FCC            (CONFIG_MAP_IDX_RM + 1)
#define CONFIG_MAP_IDX_CUR_CAP        (CONFIG_MAP_IDX_FCC + 1)
#define CONFIG_MAP_IDX_TIME_YEAR      (CONFIG_MAP_IDX_CUR_CAP + 1)
#define CONFIG_MAP_IDX_TIME_MONTH     (CONFIG_MAP_IDX_TIME_YEAR + 1)
#define CONFIG_MAP_IDX_TIME_DAY       (CONFIG_MAP_IDX_TIME_MONTH + 1)
#define CONFIG_MAP_IDX_TIME_HOUR      (CONFIG_MAP_IDX_TIME_DAY + 1)
#define CONFIG_MAP_IDX_TIME_MIN       (CONFIG_MAP_IDX_TIME_HOUR + 1)
#define CONFIG_MAP_IDX_BOARD_OFFSET   (CONFIG_MAP_IDX_TIME_MIN + 1)
#define CONFIG_MAP_IDX_TABLE_SIGN     (CONFIG_MAP_IDX_BOARD_OFFSET + 1)
#define CONFIG_MAP_IDX_TABLE_OCV      (CONFIG_MAP_IDX_TABLE_SIGN + 1)
#define CONFIG_MAP_IDX_TABLE_R        (CONFIG_MAP_IDX_TABLE_OCV + 1)
#define CONFIG_MAP_IDX_QD_0           (CONFIG_MAP_IDX_TABLE_R + 1)
#define CONFIG_MAP_IDX_QD_1           (CONFIG_MAP_IDX_QD_0 + 1)
#define CONFIG_MAP_IDX_QD_2           (CONFIG_MAP_IDX_QD_1 + 1)
#define CONFIG_MAP_IDX_QD_3           (CONFIG_MAP_IDX_QD_2 + 1)
#define CONFIG_MAP_IDX_CELL_ID        (CONFIG_MAP_IDX_QD_3 + 1)
#define CONFIG_MAP_IDX_DRV_VERSION    (CONFIG_MAP_IDX_CELL_ID + 1)
#define CONFIG_MAP_IDX_COMP_BO        (CONFIG_MAP_IDX_DRV_VERSION + 1)
#define CONFIG_MAP_IDX_GGB_VERSION    (CONFIG_MAP_IDX_COMP_BO + 1)
#define CONFIG_MAP_IDX_CHECKSUM       (CONFIG_MAP_IDX_GGB_VERSION + 1)
#define CONFIG_MAP_COUNT              (CONFIG_MAP_IDX_CHECKSUM + 1)

///-------------------------------------------------------------------------------///
/// Charge Control
///-------------------------------------------------------------------------------///
#define FC_CLEAR_RSOC                 (100)
#define FD_CLEAR_RSOC                 (3)

///-------------------------------------------------------------------------------///
/// SBS Command Page
///-------------------------------------------------------------------------------///
#define SBS_CMD_PAGE_1          (0x40)
#define SBS_CMD_PAGE_2          (0x80)
#define SBS_CMD_PAGE_3          (0xC0)
#define SBS_CMD_PAGE_4          (0xFF)

///-------------------------------------------------------------------------------///
/// Standard SBS Command
///-------------------------------------------------------------------------------///
#define SBS_MFA                 (0x00)                ///< ManufactureAccess
  #define DYNAMIC_POLL_TIME_STS_MFA (1L<<6)           ///< Status : Dynamic polling time according to RSOC
  #define RELOAD_GGB_STS_MFA        (1L<<7)           ///< Status : Reload GGB file done
  ///[YL] : Set Switch to change mode ; 20150617
  #define RSOC_DR_STS_MFA           (3L<<4)           ///< Status : RSOC status     
    #define RSOC_DR_FREE_MFA          (0L<<4)         ///< RSOC Free mode
    #define RSOC_DR_INC_MFA           (1L<<4)         ///< RSOC Increase mode
    #define RSOC_DR_DEC_MFA           (2L<<4)         ///< RSOC Decrease mode
    #define RSOC_DR_RSVD_MFA          (3L<<4)         ///< RSOC Another mode
  #define RELOAD_GGB_MFA            (0x5245)          ///< Command : Reload GGB file
  #define DYNAMIC_POLL_TIME_EN_MFA  (0x4459)          ///< Command : Enable dynamic polling time according to RSOC
  #define DYNAMIC_POLL_TIME_DIS_MFA (0x6479)          ///< Command : Disable dynamic polling time according to RSOC
  #define RSOC_FREE_RUN_CMD_MFA     (0x4515)          ///< Command : Free RSOC
  #define RSOC_KEEP_INC_CMD_MFA     (0x2909)          ///< Command : Increase RSOC
  #define RSOC_KEEP_DEC_CMD_MFA     (0x1383)          ///< Command : Decrease RSOC
#define SBS_RCA                 (0x01)                ///< RemainingCapacityAlarm
#define SBS_RTA                 (0x02)                ///< RemainingTimeAlarm
#define SBS_BM                  (0x03)                ///< BatteryMode
  #define ICC_BM                  (1L<<0)             ///< (N) internal charge control function is supported or not =>  fixed to 1
  #define PBS_BM                  (1L<<1)             ///< (N) Primary battery support is not supported => fix to 1
  #define RSVD02_BM               (1L<<2)
  #define RSVD03_BM               (1L<<3)
  #define RSVD04_BM               (1L<<4)
  #define RSVD05_BM               (1L<<5)
  #define RSVD06_BM               (1L<<6)
  #define CF_BM                   (1L<<7)             ///< (#) The flag is set when the MAX_ERR > CF MaxError limit
  #define CC_BM                   (1L<<8)             ///< (N) Enable or disable the internal charger controller   => fix to 0
  #define PB_BM                   (1L<<9)             ///< (N) Set the role of battert oacj => fix to 0
  #define F100_BM                 (1L<<10)            ///< (O) Force rsoc = 100, PCT
  #define RSVD11_BM               (1L<<11)
  #define RSVD12_BM               (1L<<12)
  #define AM_BM                   (1L<<13)            ///< (#) Enable or disable the alarm warning to the host and smart charger system  => 0: enable, 1 : disable
  #define CHGM_BM                 (1L<<14)            ///< (#) Enable or disable the charging current and charging voltage to charger => 0: enable, 1: disable
  #define CAPM_BM                 (1L<<15)            ///< (O) Set the units used for capacity information and internal calculation
  /// Mask for reserved or must set or must unset
  #define SBS_BM_MASK             (0xe383)
#define SBS_AR                  (0x04)                ///< AtRate
#define SBS_ARTF                (0x05)                ///< AtRateTimeToFull
#define SBS_ARTE                (0x06)                ///< AtRateTimeToEmpty
#define SBS_AROK                (0x07)                ///< AtRateOK
#define SBS_TEMP                (0x08)                ///< Temperature
  #define SBS_TEMP_C_2_K           (2731)
#define SBS_VOLT                (0x09)                ///< Voltage
#define SBS_CURR                (0x0a)                ///< Current
#define SBS_AVGCURR             (0x0b)                ///< AverageCurrent

#define SBS_CMD_SIZE2_ADDR0     SBS_AVGCURR           ///< From 0x00 ~ 0x0b, the  command size = 2
/// ===============================================================///
#define SBS_MAXERR              (0x0c)                ///< MaxError
#define SBS_RSOC                (0x0d)                ///< RelativeStateOfCharge
#define SBS_ASOC                (0x0e)                ///< AbsoluteStateOfCharge

#define SBS_CMD_SIZE1_ADDR0     SBS_ASOC              ///< From 0x0c ~ 0x0e, the  command size = 1
/// ===============================================================///
#define SBS_RM                  (0x0f)                ///< RemainingCapacity
#define SBS_FCC                 (0x10)                ///< FullChargeCapacity
#define SBS_RTTE                (0x11)                ///< RunTimeToEmpty
#define SBS_ATTE                (0x12)                ///< AverageTimeToEmpty
#define SBS_ATTF                (0x13)                ///< AverageTimeToFull
#define SBS_CC                  (0x14)                ///< ChargingCurrent
#define SBS_CV                  (0x15)                ///< ChargingVoltage
#define SBS_BS                  (0x16)                ///< BatteryStatus
  #define EC0_BS                  (1L<<0)             ///<  (#)  EC3, EC2, EC1, EC0 -  error code , returns of processed SBS function
  #define EC1_BS                  (1L<<1)             ///<  (#)  EC3, EC2, EC1, EC0 -  error code , returns of processed SBS function
  #define EC2_BS                  (1L<<2)             ///<  (#)  EC3, EC2, EC1, EC0 -  error code , returns of processed SBS function
  #define EC3_BS                  (1L<<3)             ///<  (#)  EC3, EC2, EC1, EC0 -  error code , returns of processed SBS function
  #define EC_MASK_BS              (0x0F)
    #define EC_OK_BS                (0x00)
    #define EC_BUSY_BS              (0x01)
    #define EC_RSVD_BS              (0x02)
    #define EC_UNSUPPORTED_BS       (0x03)
    #define EC_ACCESSDENY_BS        (0x04)
    #define EC_OVERUNDERFLOW_BS     (0x05)
    #define EC_BADSIZE_BS           (0x06)
    #define EC_UNKNOWN_BS           (0x07)
    #define EC_TIMEOUT_BS           (0x08)
  #define FD_BS                   (1L<<4)             ///<  (O)  Fully discharged
  #define FC_BS                   (1L<<5)             ///<  (O)  Fully charged
  #define DSG_BS                  (1L<<6)             ///<  (O)  Discharging => 0 : charging mode, 1: discharging mode
  #define INIT_BS                 (1L<<7)             ///<  (O)  Init mode => 1 : in the normal mode
  #define RTA_BS                  (1L<<8)             ///<  (O) Remaining time alarm => 1 remaining time alarm is set
  #define RCA_BS                  (1L<<9)             ///<  (O) Remaining capacity  alarm => 1 remaining capacity alarm is set
  #define FM_BS                   (1L<<10)            ///<   Factory mode, data flash is empty; 08/24/2010
  #define TDA_BS                  (1L<<11)            ///<  (O) Terminate charge alarm
  #define OTA_BS                  (1L<<12)            ///<  (#) Over temperature alarm , check the safety status to update the alarm
  #define EDV_BS                  (1L<<13)            ///<  (O) Reach EDV 
  #define TCA_BS                  (1L<<14)            ///<  (O) terminate charge alarm
  #define OCA_BS                  (1L<<15)            ///<  (O) Over charged  alarm

  #define FC_SET_BS               (100)               ///<  % to set the Fully charged
  /// Mask for reserved or must set or must unset
  #define SBS_BS_MASK             (0xdbff)

#define SBS_CYCCNT              (0x17)                ///< CycleCount
#define SBS_DC                  (0x18)                ///< DesignCapacity
#define SBS_DV                  (0x19)                ///< DesignVoltage
#define SBS_SPECINFO            (0x1a)                ///< SpecificationInfo
#define SBS_MFD                 (0x1b)                ///< ManufactureDate  (integer)
#define SBS_SN                  (0x1c)                ///< SerialNumber

#define SBS_RID                 (0x39)                ///< RID

#define SBS_CAP_DATA            (0x40)                ///< Capacity data
#define SBS_VCAP_DATA           (0x41)                ///< Capacity data of voltage gauge
#define SBS_ALARM               (0x42)                ///< Alarm function
  #define SBS_ALARM_UV_ENABLE     (0x5556)
  #define SBS_ALARM_UV_DISABLE    (0x7576)
#define SBS_OFF_TIME            (0x43)                ///< Power Off Time in minutes
  #define SBS_OFF_TIME_MAXIMUM  (65535)
#define SBS_VCAP_RSOC           (0x4d)                ///< RSOC of Voltage Gauge
#define SBS_VCAP_ASOC           (0x4e)                ///< ASOC of Voltage Gauge
#define SBS_VCAP_RM             (0x4f)                ///< RM of Voltage Gauge
#define SBS_VCAP_FCC            (0x50)                ///< FCC of Voltage Gauge

///-------------------------------------------------------------------------------///
/// Vedor SBS Command
///-------------------------------------------------------------------------------///
#define SBS_SYS_MODE          (0xC0)  
#define SBS_REG_ADDR          (0xC3)  
#define SBS_REG_DATA          (0xC4)
  /// High byte
  #define REG_DATA_READ               (0x00)
  #define REG_DATA_WRITE              (0x01)
#define SBS_DF_ADDR           (0xC5)
#define SBS_DF_DATA           (0xC6)
#define SBS_SUSPEND_TIME      (0xC9)
#define SBS_ERROR_STATUS      (0xF8)
#define SBS_GAUGE_STATUS      (0xFA)
#define SBS_IIR_VOLT_RATIO    (0xFD)
  #define IIR_VOLT_DEF_RATIO          (0x5014)  ///< default 80/20 ratio
#define SBS_DEF_WORD_SIZE     (2)
#define SBS_MAX_PAGE_SIZE     (64)
///-------------------------------------------------------------------------------///
/// AP Control Status
///-------------------------------------------------------------------------------///
#define AP_EXIT_STATUS          (0x01)
#define AP_IS_ACTIVE            (0x02)
#define AP_IS_KBO               (0x04)
#define AP_IS_VERIFY_MEAS       (0x08)
#define AP_IS_VERIFY_MEAS_VOLT  (0x10)
#define CHK_AP_EXIT             ((ApCtlStatus & AP_EXIT_STATUS) != AP_EXIT_STATUS)
#define CHK_AP_NOT_KBO()         ((ApCtlStatus & AP_IS_KBO) != AP_IS_KBO)
#define CHK_AP_RUNNING_KBO()     ((ApCtlStatus & AP_IS_KBO) == AP_IS_KBO)
#define CHK_AP_NOT_VERIFY_MEAS() ((ApCtlStatus & AP_IS_VERIFY_MEAS) != AP_IS_VERIFY_MEAS)
#define CHK_AP_RUNNING_VERIFY_MEAS() ((ApCtlStatus & AP_IS_VERIFY_MEAS) == AP_IS_VERIFY_MEAS)
#define LAVE_AP_KBO()            (ApCtlStatus &= ~AP_IS_KBO)
#define ENTER_AP_KBO()           (ApCtlStatus |= AP_IS_KBO)
#define LEAVE_AP_VERIFY_MEAS()   (ApCtlStatus &= ~AP_IS_VERIFY_MEAS)
#define ENTER_AP_VERIFY_MEAS()   (ApCtlStatus |= AP_IS_VERIFY_MEAS)

#define NO_ERR_PASS						(0)
#define ERR_NPP_RW_FAIL				(100)
#define ERR_NPP_WW_FAIL				(101)
#define ERR_NPP_RB_FAIL				(102)
#define ERR_NPP_WB_FAIL				(103)
#define ERR_NPP_START_FAIL		(104)
#define ERR_NPP_NOT_AVAILABLE (105)
#define ERR_NPP_RP_FAIL				(106)
#define ERR_NPP_WP_FAIL				(107)

/// [RY] : KBO return Code
#define UPI_UG31XX_KBO_RTN_PASS              (0)
#define UPI_UG31XX_KBO_RTN_CHK_TIMEOUT       (1)
#define UPI_UG31XX_KBO_RTN_OUT_OF_RANGE      (2)
#define UPI_UG31XX_KBO_RTN_UNKNOWN           (3)
#define UPI_UG31XX_KBO_RTN_WT_EEP_FAIL       (4)
#define UPI_UG31XX_KBO_RTN_CMP_EEP_FAIL      (5)
#define UPI_UG31XX_KBO_RTN_RD_EEP_FAIL       (6)
#define UPI_UG31XX_KBO_RTN_EVM_CONN_FAIL     (7)
#define UPI_UG31XX_KBO_RTN_GAUGE_CONN_FAIL   (8)
  #define CAUGE_CONN_CHK_FAIL_CNT              (10)
  #define UPI_KBO_TIME_OUT                     (20)
#define UPI_UG31XX_KBO_RTN_OVER_BOUND_FAIL   (9)
  #define UPI_UG31XX_KBO_NORMAL                (0)
  #define UPI_UG31XX_KBO_OVER_BOUND_L1         (1)
    #define EEPROM_KBO_OVER_BOUND_START_BYTE     (0x66)
  #define UPI_UG31XX_KBO_OVER_BOUND_L2         (2)

/// [AT-PM] : VERIFY_MEAS return code  ; 06/05/2015
#define UPI_UG31XX_VERIFY_MEAS_RTN_PASS       (0)
#define UPI_UG31XX_VERIFY_MEAS_RTN_TIMEOUT    (1)
  #define UPI_UG31XX_VERIFY_MEAS_TIMEOUT        (10)
#define UPI_UG31XX_VERIFY_MEAS_RTN_CONN_FAIL  (2)

#define UPI_UG31XX_SOCKET_RTN_PASS           (0x50)
#define UPI_UG31XX_SOCKET_RTN_FAIL           (0x46)

#define UPI_UG31XX_PACKET_LEN_IDX            (0)
#define UPI_UG31XX_PACKET_SIG_IDX            (1)
  #define UPI_UG31XX_PACKET_SIG                (0x55)
#define UPI_UG31XX_PACKET_HI_BOUND_IDX       (2)
#define UPI_UG31XX_PACKET_LOW_BOUND_IDX      (3)
#define UPI_UG31XX_PACKET_CHECK_SUM_IDX      (4)
#define UPI_UG31XX_PACKET_KBO_INPUT_SIZE     (5)

#define UPI_KBO_CURRENT_HI_BOUND             (127)
#define UPI_KBO_CURRENT_LO_BOUND             (-127)
#define UPI_KBO_ERR_MAX_CURR_CNT             (10)
///-------------------------------------------------------------------------------///
/// Gauge Input file Type
///-------------------------------------------------------------------------------///
#define GAUGE_TYEP_CHANGE_OTP              (1<<0L)
#define GAUGE_MAIN_LOOP_DELAY_TIME         (1<<1L)
#define GAUGE_TYPE_CONVERT                 (1<<2L)
#define GAUGE_TYPE_FAKE_I2C_CMD            (1<<4L)  ///< Enable fake I2C command

#define GAUGE_TYPE_OTP_LOOP_CNT            (10)
#define GAUGE_TYPE_MASE_PRODUCTION_CNT     (20)
#define GAUGE_TYPE_VERIFY_MEAS_CNT         (3)

#define GAUGE_INPUT_OTP_VALUE_TYPE         ("OTP")
#define GAUGE_INPUT_MAIN_LOOP_DELAY_TYPE   ("DELAY(")
#define GAUGE_INPUT_ARG_INPUT_IDX          (1)
#define GAUGE_INPUT_ARG_OUTPUT_IDX         (2)
#define GAUGE_INPUT_CONVERT                (3)
#define GAUGE_INPUT_NULL                   (0)
#define GAUGE_INPUT_TABLE_VALUE_TYPE       ("CONVERT")
#define GAUGE_INPUT_FAKE_I2C               ("FAKE_I2C")

#endif ///< __DEFINE_H__

