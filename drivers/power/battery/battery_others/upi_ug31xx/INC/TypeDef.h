/**
 * @file TypeDef.h
 *
 *  TypeDef.h define variable type
 *
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#ifdef FEATURE_PLAT_WINDOWS
  #pragma pack(push)  
  #pragma pack(1) 
#endif ///< for FEATURE_PLAT_WINDOWS

///=============================================================///
/// General type definition
///=============================================================///

/// =============================================================///
/// 2. Define the data flash class and sub-class
/// =============================================================///
#include "DataFlash.h"

///=============================================================///
/// I2C Command Object
///=============================================================///
typedef struct GI2CCmdSt
{
  GBYTE bStatus;
  GBYTE bSlaveAddr;
  GBYTE bRdCnt;
  GBYTE bWtCnt;
  GBYTE bWtBuf[16];
  GBYTE bRdBuf[16];
} PACKRAM GI2CCmdType;

///=============================================================///
/// SBS Data
///=============================================================///
typedef struct GMftDateSt {
  unsigned day : 5;
  unsigned month : 4;
  unsigned year : 7;
}GMftDateType;

typedef struct GSbsCmdSt
{
  GBYTE  bSize;        ///< default is page size  
  GBYTE  *pbData;       ///< page size data
  GBYTE  buf[32];      ///< buf
  int    errNo;
}PACKRAM GSbsCmdType;

typedef struct GSbsSt
{
  ///---------------------------------------------------------------------------///
  ///   SBS_ABBREV,      OPCODE,   IN-USE, SBS_NAME [0x00-0x07]
  ///---------------------------------------------------------------------------/// 
  ///                                 [O] -> in use
  ///                                 [X] -> not in use
  ///                                 [V] -> vendor command
  ///---------------------------------------------------------------------------///
  GWORD  wMA;           ///< 0x00,      [X],  Manufacture Access
  GWORD  wRCA;          ///< 0x01,      [X],  Remaining Capacity Alarm
  GWORD  wRTA;          ///< 0x02,      [X],  Remaining Time Alarm
  GWORD  wBM;           ///< 0x03,      [X],  Battery Mode
  GWORD  wAR;           ///< 0x04,      [X],  AtRate  
  GWORD  wARTTF;        ///< 0x05,      [X],  AtRateTimeToFull  
  GWORD  wARTTE;        ///< 0x06,      [X],  AtRateTimeToEmpty   
  GWORD  wAROK;         ///< 0x07,      [X],  AtRateOK      
  
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0x08-0x0F]
  ///---------------------------------------------------------------------------/// 
  GWORD  wTemp;         ///< 0x08,      [O],  Temperature       
  GWORD  wVolt;         ///< 0x09,      [O],  Total Battery Voltage         
  GSHORT sCurr;         ///< 0x0A,      [O],  Current
  GSHORT sAvgCurr;      ///< 0x0B,      [O],  Average Current 
  GWORD  wMaxErr;       ///< 0x0C,      [X],  Max Error
  GWORD  wRSOC;         ///< 0x0D,      [O],  RelativeStateOfCharge 
  GWORD  wASOC;         ///< 0x0E,      [X],  AbsoluteStateOfCharge 
  GWORD  wRM;           ///< 0x0F,      [O],  RemainingCapacity   
  
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0x10-0x17]
  ///---------------------------------------------------------------------------///
  GWORD  wFCC;          ///< 0x10,      [O],  FullChargeCapacity
  GWORD  wRTTE;         ///< 0x11,      [X],  RunTimeToEmpty
  GWORD  wATTE;         ///< 0x12,      [X],  AbsoluteTimeToEmpty
  GWORD  wATTF;         ///< 0x13,      [X],  AbsoluteTimeToFull
  GWORD  wCC;           ///< 0x14,      [o],  ChargeCurrent
  GWORD  wCV;           ///< 0x15,      [O],  ChargeVoltage   
  GWORD  wBS;           ///< 0x16,      [O],  Battery Status  
  GWORD  wCycCnt ;      ///< 0x17,      [O],  Cycle Count 
  
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0x18-0x1F]
  ///---------------------------------------------------------------------------/// 
  GWORD  wDC;           ///< 0x18,      [O],  DesignCapacity        
  GWORD  wDV;           ///< 0x19,      [O],  DesignVoltage
  GSHORT wSInfo;        ///< 0x1A,      [X],  SpecificationInfo
  GSHORT wMDate;        ///< 0x1B,      [X],  ManufactureDate
  GWORD  wSN;           ///< 0x1C,      [X],  SerialNumber  
  
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0x20-0x3F]
  ///---------------------------------------------------------------------------/// 
  GCHAR  strManufName[21];       ///< 0x20,      [X],  ManufacturerName                                    
  GCHAR  strDeviceName[21];      ///< 0x21,      [X],  DeviceName     
  GCHAR  strDeviceChemistry[5];  ///< 0x22,      [X],  DeviceChemistry    
  GWORD  wRid;          ///< 0x39,      [O],  RID
  GWORD  wMaxVCell;     ///< 0x3A,      [O],  MaxCellVoltage
  GWORD  wMinVCell;     ///< 0x3B,      [O],  MinCellVoltage
  GSHORT wVCell2;       ///< 0x3D,      [O],  CellVoltage2
  GSHORT wVCell1;       ///< 0x3E,      [O],  CellVoltage1
  GWORD  wVCell0;       ///< 0x3F,      [O],  CellVoltage0  

  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0x40-0x5F]
  ///---------------------------------------------------------------------------///   
  GWORD  wAlarmSts;     ///< 0x42,      [O],  Alarm Status
  GWORD  wOffTime;      ///< 0x43,      [O],  Power Off Time in minutes
  GWORD  wVRT;          ///< 0x4B,      [O],  RemainingTime of Voltage Gauge
  GWORD  wVFCT;         ///< 0x4C,      [O],  FullChargeTime of Voltage Gauge
  GWORD  wVRSOC;        ///< 0x4D,      [O],  RelativeStateOfCharge of Voltage Gauge
  GWORD  wVASOC;        ///< 0x4E,      [O],  AbsoluteStateOfCharge of Voltage Gauge
  GWORD  wVRM;          ///< 0x4F,      [O],  RemainingCapacity of Voltage Gauge
  GWORD  wVFCC;         ///< 0x50,      [O],  FullChargeCapacity of Voltage Gauge
  GWORD  wPrevRsoc;     ///< 0x5D,      [O],  Previous RSOC
  GWORD  wPrevAsoc;     ///< 0x5E,      [O],  Previous ASOC
  GWORD  wPrevRM;       ///< 0x5F,      [O],  Previous RM
  GWORD  wPrevFcc;      ///< 0x60,      [O],  Previous FCC
  GWORD  wRsocTimer;    ///< 0x61,      [O],  Timer for RSOC changing

  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0xC0-0xCF]
  ///---------------------------------------------------------------------------///   
  GWORD  wSysMode;       ///< 0xC0,     [O],  Gauge System Mode
  GWORD  wAdcEOC;        ///< 0xC1,     [O],  ADC End of conversion status  
  GWORD  wLoopIdx;       ///< 0xC2,     [O], Main loop index
  GWORD  wRegAddr;       ///< 0xC3,     [O], Register Read / Write addr
  GWORD  wRegData;       ///< 0xC4,     [O], Register Read-in / Write-out
  GWORD  wDFAddr;        ///< 0xC6,     [O], DF address Read-in / Write-out
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0xD0-0xDF]
  ///---------------------------------------------------------------------------/// 
  GWORD  wCodeCurr;      ///< 0xD0,     [O],  Code of Current
  GWORD  wCodeIT;        ///< 0xD1,     [O],  Code of Internal Temperature
  GWORD  wCodeET;        ///< 0xD2,     [O],  Code of External Temperature  
  GWORD  wCodeCC;        ///< 0xD3,     [O],  Code of Coulomb Counter   
  GWORD  wCodeCnt;       ///< 0xD4,     [O],  Code of ADC1 Counter      
  GWORD  wCodeVBat1;     ///< 0xD5,     [O],  Code of VCell 1
  GWORD  wCodeVBat2;     ///< 0xD6,     [O],  Code of VCell 2
  GWORD  wCodeVBat3;     ///< 0xD7,     [O],  Code of VCell 3   

  GSHORT sCodeETComp;    ///< 0xD8,     [O],  Compensation Code of External Temperature   
  GSHORT sCodeOffset;    ///< 0xD9,     [O],  Code of current offset        
  GWORD  wCodeRid;       ///< 0xDA,     [X],  Code of RID
  GSHORT sCodeRidComp;   ///< 0xDB,     [X],  Compensation Code of RID

  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0xE0-0xEF]
  ///---------------------------------------------------------------------------/// 
  GSHORT sCoulombCounter; ///< 0xE0,    [O],  Coulomb Counter 
  GSHORT sDeltaQ;         ///< 0xE1,    [O],  Delta Q
  GWORD  wVPack2;         ///< 0xE2,    [O],  VCell1 + VCell2
  GWORD  wVPack3;         ///< 0xE3,    [O],  VCell1 + VCell2 + VCell3
  
  ///---------------------------------------------------------------------------///
  ///    SBS_ABBREV,      OPCODE,  IN-USE, SBS_NAME  [0xF0-0xFF]
  ///---------------------------------------------------------------------------/// 
  GSHORT sCodeCurrCali;   ///< 0xF0 ,  
  GSHORT sRawCurr;        ///< 0xF1 ,            
  GWORD  wCodeVbat1Cali;  ///< 0xF2,       
  GWORD  wRawVbat1;       ///< 0xF3,
  GWORD  wCodeVbat2Cali;  ///< 0xF4, 
  GWORD  wRawVbat2;       ///< 0xF5,
  GWORD  wCodeVbat3Cali;  ///< 0xF6,       
  GWORD  wRawVbat3;       ///< 0xF7,
  GWORD  wErrorStatus;    ///< 0xF8,
  GWORD  wWarningStatus;  ///< 0xF9,
  GWORD  wGaugeStatus;    ///< 0xFA,
  GWORD  wVoltIIRRatio;   ///< 0xFD,
  GWORD  wFWVersion;      ///< 0xFE,
  GWORD  wDFVersion;      ///< 0xFF,
} PACKRAM GSbsType;

typedef struct GAdcCodeST 
{
  GSHORT sCodeCurr;      ///< 0xD0,     [O],  Code of Current
  GWORD  wCodeIT;        ///< 0xD1,     [O],  Code of Internal Temperature
  GWORD  wCodeET;        ///< 0xD2,     [O],  Code of External Temperature  
  GSHORT sCodeCC;        ///< 0xD3,     [O],  Code of Coulomb Counter   
  GWORD  wCodeCnt;       ///< 0xD4,     [O],  Code of ADC1 Counter      
  GWORD  wCodeVBat1;     ///< 0xD5,     [O],  Code of VCell 1
  GWORD  wCodeVBat2;     ///< 0xD6,     [O],  Code of VCell 2
  GWORD  wCodeVBat3;     ///< 0xD7,     [O],  Code of VCell 3 
  GWORD  wCodeRid;       ///< 0xDA,     [X],  Code of RID

  GSHORT sCodeETComp;    ///< 0xD8,     [O],  Compensation Code of External Temperature   
  GSHORT sCodeOffset;    ///< 0xD9,     [O],  Code of current offset      
  GSHORT sCodeRidComp;   ///< 0xDB,     [X],  Compensation Code of RID

  GSHORT sCodeCurrInst;
} PACKRAM GAdcCodeType;
  
///=============================================================///
/// ADC Delta Code Mapping
///=============================================================///
typedef struct AdcDeltaCodeMappingST {
  GINT32 Adc1V100;
  GINT32 Adc1V200;
  GINT32 Adc2V100;
  GINT32 Adc2V200;
} PACKRAM AdcDeltaCodeMappingType;

///=============================================================///
/// OTP Object
///=============================================================///
typedef struct GOtpDataSt
{
  ///------------------------------------///
  /// OTP Raw Data
  ///------------------------------------///
  GBYTE otp1[4];
  GBYTE otp2[4];
  GBYTE otp3[4];
  GBYTE otp4[4];
  GBYTE otp5[4];
  GBYTE otp6[4];

  ///------------------------------------///
  /// Product Type
  ///------------------------------------///
  GBYTE productType;
  
  ///------------------------------------///
  /// ADC1 Data
  ///------------------------------------///
  GBYTE indexAdc1V100T25;
  GBYTE indexAdc1V200T25;

  GWORD adc1DeltaCodeT25V100;
  GWORD adc1DeltaCodeT25V200;

  ///------------------------------------///
  /// ADC2 Data
  ///------------------------------------///
  GBYTE indexAdc2V100T25;
  GBYTE indexAdc2V200T25;

  GWORD adc2DeltaCodeT25V100;
  GWORD adc2DeltaCodeT25V200;

  ///------------------------------------///
  /// T25/T80
  ///------------------------------------///
  GWORD aveIT25;
  GWORD aveIT80;
  
  ///------------------------------------///
  /// osc 25/80
  ///------------------------------------///
  GBYTE oscDeltaCode25;
  GBYTE oscDeltaCode80;
  
  GBYTE otpCellEN;  
  
  GBYTE bgrTune;
  
  GBYTE deltaET;
  GBYTE deltaVref;
  GWORD devAddr;
  GWORD adcDelta;
  
  GWORD ftIT;
}PACKRAM  GOtpDataType;

///=============================================================///
/// ADC Data Object
///=============================================================///
typedef struct GAdcDataSt
{
  GBYTE bAdcEoc;
  GBYTE bAdcStatus; 

  ///------------------------------------///
  /// Filter Code
  ///------------------------------------///
  GINT32 dwAdcITCodeSum;    
  GBYTE bAdcITCodeCnt;  
  GWORD wAdcITCode; 

  GINT32 dwAdcETCodeSum;    
  GBYTE bAdcETCodeCnt;  
  GWORD wAdcETCode; 
  
  GINT32 dwAdcRidCodeSum;    
  GBYTE bAdcRidCodeCnt;  
  GWORD wAdcRidCode; 

  GINT32 dwAdcCurrCodeSum;
  GBYTE bAdcCurrCodeCnt;
  GSHORT sAdcCurrCode;

  GINT32 dwAdcVCell1CodeSum;   
  GBYTE bAdcVCell1CodeCnt;     
  GWORD wAdcVCell1Code;    

  GINT32 dwAdcVCell2CodeSum;   
  GBYTE bAdcVCell2CodeCnt;     
  GWORD wAdcVCell2Code;    

  GINT32 dwAdcVCell3CodeSum;   
  GBYTE bAdcVCell3CodeCnt;     
  GWORD wAdcVCell3Code;

  ///------------------------------------///
  /// ADC1 Factors
  ///------------------------------------///
  GINT16 adc1CodeT25V100;
  GINT16 adc1CodeT25V200; 

  GINT16 adc1CodeT80V100;
  GINT16 adc1CodeT80V200; 

  /// ADC1 Gain Factors
  GINT32 adc1GainSlope;
  GINT32 adc1GainFactorB;

  /// ADC1 Offset Factors
  GINT32 adc1OffsetSlope;
  GINT32 adc1OffsetFactorO;

  /// ADC1 Gain and Offset
  GINT32 adc1Gain;
  GINT32 adc1Offset;
  
  /// ADC1 Offset Temperature Compensation
  GINT16 adc1TempOffset;

  ///------------------------------------///
  /// ADC2 Factors
  ///------------------------------------///
  GINT16 adc2CodeT25V100;
  GINT16 adc2CodeT25V200;   

  GINT16 adc2CodeT80V100;
  GINT16 adc2CodeT80V200;

  /// ADC2 Gain Factors
  GINT32 adc2GainSlope;
  GINT32 adc2GainFactorB;

  /// ADC2 Offset Factors
  GINT32 adc2OffsetSlope;
  GINT32 adc2OffsetFactorO;

  /// ADC2 Gain and Offset
  GINT32 adc2Gain;
  GINT32 adc2Offset;

  ///------------------------------------///
  /// ADC1 Current OFfset for Capacity
  ///------------------------------------///
  GINT16 sCCOffset;

  ///------------------------------------///
  /// ADC Code
  ///------------------------------------///
  GAdcCodeType adcRawCode;  
  GAdcCodeType adcCaliCode;   

  ///------------------------------------///
  /// ADC2 State
  ///------------------------------------///
	GBYTE  bVoltCodeStopCnt;
	GBYTE  bVTMStopCnt;
	GBYTE bLastVTMFlag;

} PACKRAM GAdcDataType;

///=============================================================///
/// Board Factor
///=============================================================///
typedef struct PACKRAM BoardFactorST
{
  GWORD  voltageGain;
  GINT16 voltageOffset;
  GWORD  currentGain;
  GINT16 currentOffset;
  GINT16 intTempOffset;
  GINT16 extTempOffset;
} PACKRAM BoardFactorType;

///=============================================================///
/// Measurement Data
///=============================================================///
typedef struct PACKRAM GMeasDataST
{
  /// Voltage
  GWORD  wVCell1;
  GWORD  wVCell1Avg;

  GWORD  wVCell2;
  GWORD  wVCell2Avg;

  GWORD  wVCell3;
  GWORD  wVCell3Avg;

  GWORD  wMaxVCell;
  GWORD  wMinVCell;

  /// current
  GSHORT sCurr;
  GSHORT sCurrAvg;
  GSHORT sCurrRawCode;

  /// Temperarture  
  GSHORT sIntTemp;
  GSHORT sExtTemp;
  GSHORT sInstExtTemp;

  /// Capacity 
  GSHORT sDeltaQ;    ///< Delta Capacity
  GINT64 iRawCap;    ///< Current Capacity
  GINT64 iCurCap;    ///< Current Capacity
  
  /// Time
  GINT32 iConvCnt;  ///< ADC1 Conversion Count 
  GINT32 iConvTime; ///< ADC1 Conversion Time
  GINT32 iDeltaTime;  ///< ADC1 Conversion Time
  
  /// RID
  GSHORT wRid;

  GSHORT sCodeCurrCali;   ///< 0xF0,   
  GSHORT sRawCurr;       ///< 0xF1 ,       
  GWORD  wCodeVbat1Cali;  ///< 0xF2,       
  GWORD  wRawVbat1;      ///< 0xF3,
  GWORD  wCodeVbat2Cali;  ///< 0xF4,
  GWORD  wRawVbat2;      ///< 0xF5,
  GWORD  wCodeVbat3Cali;  ///< 0xF6,       
  GWORD  wRawVbat3;      ///< 0xF7,

  ///
  GBOOL bValid; 
  GBOOL bDeltaQ;         ///< [YL] : To determine restore if finish or not ; 2015/08/11
} PACKRAM GMeasDataType;

///=============================================================///
/// Time Object
///=============================================================///
typedef struct GTimeDataSt
{
  GSHORT  sec;
  GSHORT  min;
  GSHORT  hour; 
  GSHORT  day;    
  GSHORT  mon;      
  GSHORT  year;       
  GINT32  rawTime;  ///< absolute time in sec       
  GINT32  absTime;  ///< absolute time in ms
} PACKRAM GTimeDataType;

///=============================================================///
/// Data in Config File
///=============================================================///
typedef struct GChargeCtlSt
{
  GINT iTaperTimer;   ///< Taper window 
} GChargeCtlType;

///=============================================================///
/// Data in Config File
///=============================================================///
typedef struct GConfigMapSt
{
  GCHAR name[32];
  GWORD addr;
  GWORD size;
}PACKRAM GConfigMapType;

typedef struct GConfigDataSt
{
  GU2                  wRM;               ///< [AT-PM] : ADDR = 0x40 ; 04/20/2015
  GU2                  wFcc;              ///< [AT-PM] : ADDR = 0x42 ; 04/20/2015
  GSHORT               iCurCap;           ///< [AT-PM] : ADDR = 0x44 ; 04/20/2015
 
  GU2                  wYear;             ///< [AT-PM] : ADDR = 0x46 ; 04/20/2015
  GU1                  bMonth;            ///< [AT-PM] : ADDR = 0x48 ; 04/20/2015
  GU1                  bDay;              ///< [AT-PM] : ADDR = 0x49 ; 04/20/2015
  GU1                  bHour;             ///< [AT-PM] : ADDR = 0x4A ; 04/20/2015
  GU1                  bMin;              ///< [AT-PM] : ADDR = 0x4B ; 04/20/2015

  GCHAR                cBoardOffsetSign;  ///< [AT-PM] : ADDR = 0x4C ; 04/22/2015
  GSHORT               sBoardOffsetValue; ///< [AT-PM] : ADDR = 0x4D ; 04/22/2015
  GU1                  bBoardOffsetSum;   ///< [AT-PM] : ADDR = 0x4F ; 04/22/2015

  GCHAR                cTableSign;        ///< [AT-PM] : ADDR = 0x50 ; 04/20/2015
  GU2                  wOcv[21];          ///< [AT-PM] : ADDR = 0x51 ; 04/20/2015
  GU1                  bImpedence[21];    ///< [AT-PM] : ADDR = 0x7B ; 04/20/2015

  GU2                  wQD0;              ///< [AT-PM] : ADDR = 0x90 ; 04/20/2015
  GU2                  wQD1;              ///< [AT-PM] : ADDR = 0x92 ; 04/20/2015
  GU2                  wQD2;              ///< [AT-PM] : ADDR = 0x94 ; 04/20/2015
  GU2                  wQD3;              ///< [AT-PM] : ADDR = 0x96 ; 04/20/2015

  GCHAR                strCellID[8];      ///< [AT-PM] : ADDR = 0x98 ; 04/20/2015
  GCHAR                strDrvVersion[11]; ///< [AT-PM] : ADDR = 0xA0 ; 04/20/2015
  GI1                  sBoardOffsetComp;  ///< [AT-PM] : ADDR = 0xAB ; 05/11/2015
  GWORD                wGgbVersion;       ///< [AT-PM] : ADDR = 0xAC ; 04/20/2015
  GWORD                wChecksum;         ///< [AT-PM] : ADDR = 0xAE ; 04/20/2015

  GU2                  wDataSize;         ///< [AT-PM] : No save to EEPROM ; 04/22/2015
}PACKRAM GConfigDataType;

///=============================================================///
/// Capacity Algorithm 
///=============================================================///

typedef struct PACKRAM GCapDataST
{
  GBYTE bCapCntl;
  GBYTE bChgState;
  GWORD wDsgState;

  GU1 bTempIdx;
  GU1 bSocIdx;

  /// [AT-PM] : Unit in 0.1% ; 04/17/2014
  GU2 wRsoc10x;

  /// [AT-PM] : Unit in % ; 08/26/2015
  GU2 wExtRsoc;

  /// [AT-PM] : Unit in mAh ; 04/17/2014
  GU2 wRM;
  GU2 wRMLast;
  GU2 wFcc;
  GU2 wQD[CAP_UPDATE_PTR_CNT];
  GU2 wQDBuf[CAP_UPDATE_PTR_CNT];

  /// [AT-PM] : Unit in mSec ; 05/26/2014 
  GINT32 lUpdateTimeMSec[CAP_UPDATE_PTR_CNT];

  /// [AT-PM] : Unit in mV ; 07/04/2014
  GU2 wDsgVoltTable[CAP_OCV_TABLE_ELEMENT_CNT]; 
  GU2 wChgVoltTable[CAP_OCV_TABLE_ELEMENT_CNT];
  GU2 wStandbyVoltTable[CAP_OCV_TABLE_ELEMENT_CNT];
} PACKRAM GCapDataType;

///=============================================================///
/// Voltage Gauge Capacity Algorithm 
///=============================================================///

typedef struct PACKRAM GVCapDataST
{
  GBYTE bCapCntl;
  GBYTE bChgState;
  GWORD wDsgState;

  GU1 bSocIdx;

  /// [AT-PM] : Unit in 0.1% ; 04/17/2014
  GU2 wRsoc10x;

  /// [AT-PM] : Unit in min ; 07/18/2014
  GU2 wRM;
  GU2 wRMLast;
  GU2 wFcc;
  GU2 wQD[VCAP_UPDATE_PTR_CNT];
  GU2 wQDBuf[VCAP_UPDATE_PTR_CNT];
  GU2 wDeltaMin;
  GU2 wQDMin;
  GU2 wNearFullMin;

  /// [AT-PM] : Unit in sec ; 07/18/2014
  GU2 wDeltaSec;
  GU2 wBufTimeSec;
  GU2 wBufQDTimeSec;

  /// [AT-PM] : Unit in mSec ; 05/26/2014 
  GINT32 lUpdateTimeMSec[VCAP_UPDATE_PTR_CNT];
  GINT32 lUpdateChgOffsetMSec;
  GINT32 lBufTimeMSec;

  /// [AT-PM] : Unit in mV ; 07/04/2014
  GU2 wDsgVoltTable[VCAP_OCV_TABLE_ELEMENT_CNT]; 
  GU2 wChgVoltTable[VCAP_OCV_TABLE_ELEMENT_CNT];
} PACKRAM GVCapDataType;

///=============================================================///
/// General type definition
///=============================================================///

typedef struct PollingTimeSt
{
  GU1 rsoc;
  GU1 sec;
} PollingTimeType;

/// ============================================================================================================================///
/// UpiGauge
/// ============================================================================================================================///
typedef struct GaugeVarSt
{
  GlobalDFVarType   *ggb;
  GSbsCmdType       sbsCmd;
  GSbsType          sbs;
  GSbsType          repSbs;
  GOtpDataType      otp;
  GAdcDataType      adc;
  GConfigDataType   cfg;

  /// Time objects  
  GMeasDataType     measQueue[MAX_MEAS_QUEUE_SIZE];
  GMeasDataType    *pCurMeas;   

  /// Time objects
  GTimeDataType     tmNow;    ///< now time
  GTimeDataType     tmPre;    ///< previous time
  GTimeDataType     tmRes;    ///< restored time
  GTimeDataType     tmConfig; ///< config update time
  GWORD             wSuspendTime;
  
  /// cap. data 
  GCapDataType      cap;
  GVCapDataType     vCap;

  /// Charge Control
  GChargeCtlType    chgCtrl;
  
  /// Internal Registers
  GBYTE bRegIntrStatus;
  GBYTE bRegIntrStatusPre;

  /// System Status   
  GWORD  wSysMode;

  GWORD  wBatteryMode;

  GWORD  wChargeStatus;
  GWORD  wGaugeStatus;
  GWORD  wErrorStatus;  
  GWORD  wWarningStatus; 
  GWORD  wI2cErrCnt;
  GBYTE bState;
  GBYTE bPreState;  
  GWORD wFWVersion;

  GBYTE bRegArray[256];

  /// Loop Index
  GDWORD loopIdx;
} PACKRAM GaugeVarType;

#ifdef FEATURE_PLAT_WINDOWS
  #pragma pack(pop)
#endif ///< for FEATURE_PLAT_WINDOWS

#endif ///< __TYPEDEF_H__

