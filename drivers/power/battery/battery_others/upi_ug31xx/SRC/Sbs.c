/**
 * @file Sbs.c
 *
 * SBS Related Function
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#include "UpiDef.h"

///---------------------------------------------------///
/// 0x00,     [X],  Manufacture Access
///---------------------------------------------------///
/**
 * @brief SbsManuAccess
 *
 *  SbsManuAccess
 *
 * @return WORD
 *
 */
GWORD SbsManuAccess(GVOID)
{
  return ReportSBS->wMA;
}

/**
 * @brief SbsManuAccessWt
 *
 * SbsManuAccess command
 *
 * @para wData command
 * @return NULL
 */
GVOID SbsManuAccessWt(GWORD wData)
{
  if(wData == RELOAD_GGB_MFA)
  {
    ReportSBS->wMA = ReportSBS->wMA | RELOAD_GGB_STS_MFA;
    GLOGE("[%s]: Accept reload GGB command (%04x)\n", __func__, ReportSBS->wMA);
  }
  if(wData == DYNAMIC_POLL_TIME_EN_MFA)
  {
    ReportSBS->wMA = ReportSBS->wMA | DYNAMIC_POLL_TIME_STS_MFA;
    GLOGE("[%s]: Enable dynamic polling time according to RSOC (%04x)\n", __func__, ReportSBS->wMA);
  }
  if(wData == DYNAMIC_POLL_TIME_DIS_MFA)
  {
    ReportSBS->wMA = ReportSBS->wMA & (~DYNAMIC_POLL_TIME_STS_MFA);
    GLOGE("[%s]: Disable dynamic polling time according to RSOC (%04x)\n", __func__, ReportSBS->wMA);
  }
  /// [YL] : Set RSOC switch ; 20150617
  if(wData == RSOC_FREE_RUN_CMD_MFA)
  {
  	ReportSBS->wMA = ReportSBS->wMA & (~RSOC_DR_STS_MFA);
    ReportSBS->wMA = ReportSBS->wMA | RSOC_DR_FREE_MFA;
    GLOGE("[%s]: RSOC free command (%04x)\n", __func__, ReportSBS->wMA);
  }
  if(wData == RSOC_KEEP_INC_CMD_MFA)
  {
    ReportSBS->wMA = ReportSBS->wMA & (~RSOC_DR_STS_MFA);
    ReportSBS->wMA = ReportSBS->wMA | RSOC_DR_INC_MFA;
    GLOGE("[%s]: RSOC increse command (%04x)\n", __func__, ReportSBS->wMA);
  }
  if(wData == RSOC_KEEP_DEC_CMD_MFA)
  {
    ReportSBS->wMA = ReportSBS->wMA & (~RSOC_DR_STS_MFA);
    ReportSBS->wMA = ReportSBS->wMA | RSOC_DR_DEC_MFA;
    GLOGE("[%s]: RSOC decrese command (%04x)\n", __func__, ReportSBS->wMA);
  }
  
}

///---------------------------------------------------///
/// 0x01,     [X],  Remaining Capacity Alarm
///---------------------------------------------------///
/**
 * @brief SBSRemainCapacityAlarm
 *
 *  Remaining Capacity Alarm
 *
 * @return WORD
 *
 */
GWORD SbsRemainCapacityAlarm(GVOID)
{
  return ReportSBS->wRCA;
}

///---------------------------------------------------///
/// 0x02,     [X],  Remaining Time Alarm
///---------------------------------------------------///
/**
 * @brief SBSRemainTimeAlarm
 *
 *  Remaining Time Alarm
 *
 * @return WORD
 *
 */
GWORD SbsRemainTimeAlarm(GVOID)
{
  return ReportSBS->wRTA;
}

///---------------------------------------------------///
/// 0x03,     [X],  Battery Mode
///---------------------------------------------------///
/**
 * @brief SBSBattMode
 *
 * Battery Mode
 *
 * @return WORD
 *
 */
GWORD SbsBattMode(GVOID)
{
  return ReportSBS->wBM;
}

///---------------------------------------------------///
/// 0x04,     [X],  AtRate  
///---------------------------------------------------///
/**
 * @brief SBSAtRate
 *
 * At Rate
 *
 * @return WORD
 *
 */
GWORD SbsAtRate(GVOID)
{
  return ReportSBS->wAR;
}

///---------------------------------------------------///
/// 0x05,     [X],  AtRateTimeToFull  
///---------------------------------------------------///
/**
 * @brief SBSAtRate
 *
 *  AtRateTimeToFull
 *
 * @return WORD
 *
 */
GWORD SbsAtRateTimeToFull(GVOID)
{
  return ReportSBS->wATTF;
}

///---------------------------------------------------///
/// 0x06,     [X],  AtRateTimeToEmpty   
///---------------------------------------------------///
/**
 * @brief SBSAtRateTimeToEmpty
 *
 *  AtRateTimeToEmpty
 *
 * @return WORD
 *
 */
GWORD SbsAtRateTimeToEmpty(GVOID)
{
  return ReportSBS->wATTE;
}

///---------------------------------------------------///
/// 0x07,     [X],  AtRateOK      
///---------------------------------------------------///
/**
 * @brief SBSAtRateOK
 *
 *  AtRateOK
 *
 * @return WORD
 *
 */
GWORD SbsAtRateOK(GVOID)
{
  return ReportSBS->wAROK;
}

///---------------------------------------------------///
/// 0x08,     [O],  Temperature     
///---------------------------------------------------///
/**
 * @brief SBSTemp
 *
 *  Temperature
 *
 * @return WORD
 *
 */
GWORD SbsTemp(GVOID)
{
  return ReportSBS->wTemp;
}

///---------------------------------------------------///
/// 0x09,     [O],  Total Battery Voltage         
///---------------------------------------------------///
/**
 * @brief SBSVolt
 *
 *  SBSTemp
 *
 * @return WORD
 *
 */
GWORD SbsVolt(GVOID)
{
  return ReportSBS->wVolt;
}

///---------------------------------------------------///
/// 0x0A,     [O],  Current
///---------------------------------------------------///
/**
 * @brief SBSVolt
 *
 *  Total Battery Voltage
 *
 * @return WORD
 *
 */
GWORD SbsCurr(GVOID)
{
  GWORD wData;
  wData = *(GWORD *)&ReportSBS->sCurr;
  return wData;
}

///---------------------------------------------------///
/// 0x0B,     [O],  Average Current 
///---------------------------------------------------///
/**
 * @brief SBSAvgCurr
 *
 *  Average Current
 *
 * @return WORD
 *
 */
GWORD SbsAvgCurr(GVOID)
{
  GWORD wData;
  wData = *(GWORD *)&ReportSBS->sAvgCurr;
  return wData;
}

///---------------------------------------------------///
/// 0x0C,     [X],  Max Error
///---------------------------------------------------///
/**
 * @brief SBSMaxErr
 *
 *  Max Error
 *
 * @return WORD
 *
 */
GWORD SbsMaxErr(GVOID)
{
  return ReportSBS->wMaxErr;;
}

///---------------------------------------------------///
/// 0x0D,     [O],  RelativeStateOfCharge 
///---------------------------------------------------///
/**
 * @brief SBSRelativeStateOfCharge
 *
 *  RelativeStateOfCharge
 *
 * @return WORD
 *
 */
GWORD SbsRelativeStateOfCharge(GVOID)
{
  return ReportSBS->wRSOC;
}

///---------------------------------------------------///
/// 0x0E,     [X],  AbsoluteStateOfCharge 
///---------------------------------------------------///
/**
 * @brief SBSAbsoluteStateOfCharge
 *
 *  AbsoluteStateOfCharge
 *
 * @return WORD
 *
 */
GWORD SbsAbsoluteStateOfCharge(GVOID)
{
  return ReportSBS->wRSOC;
}

///---------------------------------------------------///
/// 0x0F,     [O],  RemainingCapacity   
///---------------------------------------------------///
/**
 * @brief SBSRM
 *
 *  RelativeStateOfCharge
 *
 * @return WORD
 *
 */
GWORD SbsRemainingCapacity(GVOID)
{
  return ReportSBS->wRM;
}

///---------------------------------------------------///
/// 0x10,     [O],  FullChargeCapacity
///---------------------------------------------------///
/**
 * @brief SBSFullChargeCapacity
 *
 *  RelativeStateOfCharge
 *
 * @return WORD
 *
 */
GWORD SbsFullChargeCapacity(GVOID)
{
  return ReportSBS->wFCC;
}

///---------------------------------------------------///
/// 0x11,     [X],  RunTimeToEmpty
///---------------------------------------------------///
/**
 * @brief SBSRunTimeToEmpty
 *
 *  RunTimeToEmpty
 *
 * @return WORD
 *
 */
GWORD SbsRunTimeToEmpty(GVOID)
{
  return ReportSBS->wRTTE;
}

///---------------------------------------------------///
/// 0x12,     [X],  AbsoluteTimeToEmpty
///---------------------------------------------------///
/**
 * @brief SBSAbsoluteTimeToEmpty
 *
 *  AbsoluteTimeToEmpty
 *
 * @return WORD
 *
 */
GWORD SbsAbsoluteTimeToEmpty(GVOID)
{
  return ReportSBS->wATTE;
}

///---------------------------------------------------///
/// 0x13,     [X],  AbsoluteTimeToFull
///---------------------------------------------------///
/**
 * @brief SBSAbsoluteTimeToFull
 *
 *  AbsoluteTimeToFull
 *
 * @return WORD
 *
 */
GWORD SbsAbsoluteTimeToFull(GVOID)
{
  return ReportSBS->wATTF;
}

///---------------------------------------------------///
/// 0x14,     [o],  ChargeCurrent
///---------------------------------------------------///
/**
 * @brief SBSChargeCurrent
 *
 *  ChargeCurrent
 *
 * @return WORD
 *
 */
GWORD SbsChargeCurrent(GVOID)
{
  return ReportSBS->wCC;
}

///---------------------------------------------------///
/// 0x15,     [O],  ChargeVoltage   
///---------------------------------------------------///
/**
 * @brief SBSChargeVoltage
 *
 *  ChargeVoltage
 *
 * @return WORD
 *
 */
GWORD SbsChargeVoltage(GVOID)
{
  return ReportSBS->wCV;
}

///---------------------------------------------------///
/// 0x16,     [O],  Battery Status  
///---------------------------------------------------///
/**
 * @brief SBSChargeVoltaget
 *
 *  ChargeVoltage
 *
 * @return WORD
 *
 */
GWORD SbsBatteryStatus(GVOID)
{
  return ReportSBS->wBS;
}

///---------------------------------------------------///
/// 0x17,     [O],  CycleCount  
///---------------------------------------------------///
/**
 * @brief SBSCycleCount
 *
 *  CycCnt
 *
 * @return WORD
 *
 */
GWORD SbsCycleCount(GVOID)
{
  return ReportSBS->wCycCnt;
}

///---------------------------------------------------///
/// 0x18,     [O],  DesignCapacity        
///---------------------------------------------------///
/**
 * @brief SBSDesignCapacity
 *
 *  DesignCapacity
 *
 * @return WORD
 *
 */
GWORD SbsDesignCapacity(GVOID)
{
  return ReportSBS->wDC;
}

///---------------------------------------------------///
/// 0x19,     [O],  DesignVoltage
///---------------------------------------------------///
/**
 * @brief SBSDesignVoltage
 *
 *  DesignVoltage
 *
 * @return WORD
 *
 */
GWORD SbsDesignVoltage(GVOID)
{
  return ReportSBS->wDV;
}

///---------------------------------------------------///
/// 0x1A,     [X],  SpecificationInfo
///---------------------------------------------------///
/**
 * @brief SBSSpecificationInfo
 *
 *  SpecificationInfo
 *
 * @return WORD
 *
 */
GWORD SbsSpecInfo(GVOID)
{
  GWORD wData;
  wData = *(GWORD *)&ReportSBS->wSInfo;
  return wData;
}

///---------------------------------------------------///
/// 0x1B,     [X],  ManufactureDate
///---------------------------------------------------///
/**
 * @brief SBSManufactureDate
 *
 *  ManufactureDate
 *
 * @return WORD
 *
 */
GWORD SbsManufactureDate(GVOID)
{
  return ReportSBS->wMDate;
}

///---------------------------------------------------///
/// 0x1C,     [X],  SerialNumber  
///---------------------------------------------------///
/**
 * @brief SBSSerialNumber
 *
 *  SerialNumber
 *
 * @return WORD
 *
 */
GWORD SbsSerialNumber(GVOID)
{
  return ReportSBS->wSN;
}

///---------------------------------------------------///
/// 0x20,     [X],  SbsMFName 
///---------------------------------------------------///
/**
 * @brief SbsMFName
 *
 *   Manufacturer Name
 *
 * @return WORD
 *
 */
GWORD SbsMFName(GVOID)
{
  SBSCmd->bSize = strlen(ReportSBS->strManufName);
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    SBSCmd->bSize = SBSCmd->bSize + 1;
  }
  SBSCmd->pbData = (GBYTE *)&ReportSBS->strManufName;
  return SBSCmd->bSize;
}

///---------------------------------------------------///
/// 0x21,     [X],  SbsDevName  
///---------------------------------------------------///
/**
 * @brief SbsDevName
 *
 *   Device Name
 *
 * @return WORD
 *
 */
GWORD SbsDevName(GVOID)
{
  SBSCmd->bSize = strlen(ReportSBS->strDeviceName);
  
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    SBSCmd->bSize = SBSCmd->bSize + 1;
  }
  SBSCmd->pbData = (GBYTE *)&ReportSBS->strDeviceName;
  return SBSCmd->bSize;
}

///---------------------------------------------------///
/// 0x22,     [X],  SbsDevChem  
///---------------------------------------------------///
/**
 * @brief SbsDevChem
 *
 *   Device Name
 *
 * @return WORD
 *
 */
GWORD SbsDevChem(GVOID)
{
  SBSCmd->bSize = strlen(ReportSBS->strDeviceChemistry);
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    SBSCmd->bSize = SBSCmd->bSize + 1;
  }
  SBSCmd->pbData = (GBYTE *)&ReportSBS->strDeviceChemistry;
  return SBSCmd->bSize;
}

///---------------------------------------------------///
/// 0x3A,     [O],  Maximum Cell Voltage 
///---------------------------------------------------///
/**
 * @brief SbsMaxCellVoltage
 *
 * Maximum cell voltage
 * 
 * @return WORD
 */
GWORD SbsMafData(GVOID)
{
  return 0x55AA;
}

///---------------------------------------------------///
/// 0x39,     [O],  RID
///---------------------------------------------------///
/**
 * @brief SbsRid
 *
 * RID
 * 
 * @return WORD
 */
GWORD SbsRid(GVOID)
{
  return (ReportSBS->wRid);
}

///---------------------------------------------------///
/// 0x3A,     [O],  Maximum Cell Voltage 
///---------------------------------------------------///
/**
 * @brief SbsMaxCellVoltage
 *
 * Maximum cell voltage
 * 
 * @return WORD
 */
GWORD SbsMaxCellVoltage(GVOID)
{
  return (ReportSBS->wMaxVCell);
}

///---------------------------------------------------///
/// 0x3B,     [O],  Minimum Cell Voltage
///---------------------------------------------------///
/**
 * @brief SbsMinCellVoltage
 *
 * Minimum Cell Voltage
 *
 * @return WORD
 */
GWORD SbsMinCellVoltage(GVOID)
{
  return (ReportSBS->wMinVCell);
}

///---------------------------------------------------///
/// 0x3C,     [O],  Cell Voltage 3
///---------------------------------------------------///
/**
 * @brief SbsCellVoltage3
 *
 * Cell Voltage 3, which is the highest
 *
 * @return WORD
 */
GWORD SbsCellVoltage3(GVOID)
{
  return (0);
}

///---------------------------------------------------///
/// 0x3D,     [O],  Cell Voltage 2
///---------------------------------------------------///
/**
 * @brief SbsCellVoltage2
 *
 * Cell Voltage 2
 *
 * @return WORD
 */
GWORD SbsCellVoltage2(GVOID)
{
  return (ReportSBS->wVCell2);
}

///---------------------------------------------------///
/// 0x3E,     [O],  Cell Voltage 1
///---------------------------------------------------///
/**
 * @brief SbsCellVoltage1
 *
 * Cell Voltage 1
 *
 * @return WORD
 */
GWORD SbsCellVoltage1(GVOID)
{
  return (ReportSBS->wVCell1);
}

///---------------------------------------------------///
/// 0x3F,     [O],  Cell Voltage 0 
///---------------------------------------------------///
/**
 * @brief SbsCellVoltage0
 *
 * Cell Voltage 0, which is the lowest one
 *
 * @return WORD
 */
GWORD SbsCellVoltage0(GVOID)
{
  return (ReportSBS->wVCell0);
}

///---------------------------------------------------///
/// 0x40,     [O],  Capacity data
///---------------------------------------------------///
/**
 * @brief SbsCapData
 *
 * Capacity data
 *
 * @return WORD
 */
GWORD SbsCapData(GVOID)
{
  SBSCmd->bSize = (GBYTE)((sizeof(*CAP) > 255) ? 255 : sizeof(*CAP));
  
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    SBSCmd->bSize = SBSCmd->bSize + 1;
  }
  SBSCmd->pbData = (GBYTE *)CAP;
  return ((GWORD)(SBSCmd->bSize));
}

/**
 * @brief SbsCapCmd
 *
 * Capacity command
 *
 * @return NULL
 */
GVOID SbsCapCmd(GWORD wData)
{
  UpiMutexLock((GVOID *)&UpiCapMutex);
  if(wData == CAP_SBS_CMD_RESET)
  {
    CAP->bCapCntl = CAP->bCapCntl | CAP_CNTL_RESET;
  }
  else if(wData == CAP_SBS_CMD_SET_FULL)
  {
    CAP->bCapCntl = CAP->bCapCntl | CAP_CNTL_SET_FULL;
  }
  else if((wData & CAP_SBS_CMD_SET_RSOC_MASK) == CAP_SBS_CMD_SET_RSOC)
  {
    CAP->bCapCntl = CAP->bCapCntl | CAP_CNTL_SET_RSOC;
    CAP->wExtRsoc = (wData & CAP_SBS_CMD_SET_RSOC_VALUE) >> 8;
  }
  UpiMutexUnLock((GVOID *)&UpiCapMutex);
  return;
}

///---------------------------------------------------///
/// 0x41,     [O],  Capacity data of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVCapData
 *
 * Capacity data of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVCapData(GVOID)
{
  SBSCmd->bSize = (GBYTE)((sizeof(*VCAP) > 255) ? 255 : sizeof(*VCAP));
  
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    SBSCmd->bSize = SBSCmd->bSize + 1;
  }
  SBSCmd->pbData = (GBYTE *)VCAP;
  return ((GWORD)(SBSCmd->bSize));
}

/**
 * @brief SbsVCapCmd
 *
 * Capacity command of Voltage Gauge
 *
 * @return NULL
 */
GVOID SbsVCapCmd(GWORD wData)
{
  if(wData == VCAP_SBS_CMD_SET_FULL)
  {
    UpiMutexLock((GVOID *)&UpiCapMutex);
    VCAP->bCapCntl = VCAP->bCapCntl | VCAP_CNTL_SET_FULL;
    UpiMutexUnLock((GVOID *)&UpiCapMutex);
  }
  return;
}

///---------------------------------------------------///
/// 0x42,     [O],  Alarm function 
///---------------------------------------------------///

/**
 * @brief SbsAlarm
 * 
 * Get alarm status
 *
 * @return alarm status
 */
GWORD SbsAlarm(GVOID)
{
  return (ReportSBS->wAlarmSts);
}

/**
 * @brief SbsAlarmWt
 *
 * Set alarm function
 *
 * @para wData command
 * @return NULL
 */
GVOID SbsAlarmWt(GWORD wData)
{
  switch(wData)
  {
    case SBS_ALARM_UV_ENABLE:
      ReportSBS->wAlarmSts = ReportSBS->wAlarmSts | ALARM_STS_UV_EN;
      break;
    case SBS_ALARM_UV_DISABLE:
      ReportSBS->wAlarmSts = ReportSBS->wAlarmSts & (~ALARM_STS_UV_EN);
      break;
    default:
      GLOGD("[%s]: Unknown command (%04x)\n", __func__, wData);
      break;
  }
}

///---------------------------------------------------///
/// 0x43,     [O],  Off Time function 
///---------------------------------------------------///

/**
 * @brief SbsOffTime
 * 
 * Get power off time
 *
 * @return power off time in minutes
 */
GWORD SbsOffTime(GVOID)
{
  return (ReportSBS->wOffTime);
}

///---------------------------------------------------///
/// 0x4B,     [O],  RT of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVRT
 *
 * RT of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVRT(GVOID)
{
  return (ReportSBS->wVRT);
}

///---------------------------------------------------///
/// 0x4C,     [O],  FCT of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVFct
 *
 * FCT of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVFct(GVOID)
{
  return (ReportSBS->wVFCT);
}

///---------------------------------------------------///
/// 0x4D,     [O],  RSOC of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVRsoc
 *
 * RSOC of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVRsoc(GVOID)
{
  return (ReportSBS->wVRSOC);
}

///---------------------------------------------------///
/// 0x4E,     [O],  ASOC of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVAsoc
 *
 * ASOC of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVAsoc(GVOID)
{
  return (ReportSBS->wVASOC);
}

///---------------------------------------------------///
/// 0x4F,     [O],  RM of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVRM
 *
 * RM of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVRM(GVOID)
{
  return (ReportSBS->wVRM);
}

///---------------------------------------------------///
/// 0x50,     [O],  FCC of Voltage Gauge
///---------------------------------------------------///
/**
 * @brief SbsVFcc
 *
 * FCC of Voltage Gauge
 *
 * @return WORD
 */
GWORD SbsVFcc(GVOID)
{
  return (ReportSBS->wVFCC);
}

///---------------------------------------------------///
/// 0x5D,     [O],  Previous RSOC
///---------------------------------------------------///
GWORD SbsPrevRsoc(GVOID)
{
  return (ReportSBS->wPrevRsoc);
}

///---------------------------------------------------///
/// 0x5E,     [O],  Previous ASOC
///---------------------------------------------------///
GWORD SbsPrevAsoc(GVOID)
{
  return (ReportSBS->wPrevAsoc);
}

///---------------------------------------------------///
/// 0x5F,     [O],  Previous RM
///---------------------------------------------------///
GWORD SbsPrevRM(GVOID)
{
  return (ReportSBS->wPrevRM);
}

///---------------------------------------------------///
/// 0x60,     [O],  Previous FCC
///---------------------------------------------------///
GWORD SbsPrevFcc(GVOID)
{
  return (ReportSBS->wPrevFcc);
}

///---------------------------------------------------///
/// 0x61,     [O],  RSOC Timer
///---------------------------------------------------///
GWORD SbsRsocTimer(GVOID)
{
  return (ReportSBS->wRsocTimer);
}

///---------------------------------------------------///
/// 0xC0,     [X],  SysMode 
///---------------------------------------------------///
/**
 * @brief SbsSysMode
 *
 *  System Mode
 *
 * @return WORD
 *
 */
GWORD SbsSysMode(GVOID)
{
  return ReportSBS->wSysMode;
}

/**
 * @brief SbsSysMode
 *
 *  System Mode
 *
 * @return WORD
 *
 */
GVOID SbsSysModeWt(GWORD wData)
{
  ReportSBS->wSysMode = wData;
  SBS->wSysMode = wData;
  UpiGauge.wSysMode = wData;
  GLOGD("[%s]: wSysMode = %04x / %04x / %04x / %04x\n", __func__, ReportSBS->wSysMode, SBS->wSysMode, UpiGauge.wSysMode, wData);
  return;
}

///---------------------------------------------------///
/// 0xC1,     [X],  wAdcEOC 
///---------------------------------------------------///
/**
 * @brief SbsAdcEOC
 *
 *  ADC End of conversion status  
 *
 * @return WORD
 *
 */
GWORD SbsAdcEOC(GVOID)
{
  return ReportSBS->wAdcEOC;
}


///---------------------------------------------------///
/// 0xC2,     [O],  wLoopIdx  
///---------------------------------------------------///
/**
 * @brief SbsMainLoopIdx
 *
 *  ADC End of conversion status  
 *
 * @return WORD
 *
 */
GWORD SbsMainLoopIdx(GVOID)
{
  return ReportSBS->wLoopIdx;
}

///---------------------------------------------------///
/// 0xC3,     [O],  Rester Read/ Writr Addr Set 
///---------------------------------------------------///
/**
 * @brief SbsRegAddr
 *
 *  Rester Read/ Writr Addr Get
 *
 * @return WORD
 *
 */
GWORD SbsRegAddr(GVOID)
{
  return ReportSBS->wRegAddr;
}

/**
 * @brief SbsRegAddrWt
 *
 *  Rester Read/ Writr Addr Set
 *
 * @return WORD
 *
 */
GVOID SbsRegAddrWt(GWORD wData)
{
  SBS->wRegAddr = wData;
  ReportSBS->wRegAddr = wData;
}

///---------------------------------------------------///
/// 0xC4,     [O],  Register read = 0x0000, write = 0x01##  
///---------------------------------------------------///

/**
 * @brief SbsRegData
 *
 *  Register Read/Writr Data Get
 *
 * @return WORD
 *
 */
GWORD SbsRegData(GVOID)
{
  ReportSBS->wRegData = READ_REG((GBYTE)(ReportSBS->wRegAddr));
  return ReportSBS->wRegData;
}

///---------------------------------------------------///
/// 0xC5,     [O],  Read/Write DF data
///---------------------------------------------------///

/**
 * @brief SbsDFWt
 *
 *  Rester Read/ Writr Addr Set
 *
 * @return WORD
 *
 */
GVOID SbsDFAddrWt(GWORD wData)
{
  SBS->wDFAddr = wData;
  ReportSBS->wDFAddr = wData;
}

/**
 * @brief SbsDFWt
 *
 *  Rester Read/ Writr Addr Set
 *
 * @return WORD
 *
 */
GWORD SbsDFAddr(GVOID)
{
  return ReportSBS->wDFAddr;
}

///---------------------------------------------------///
/// 0xC6,     [O],  Read DF data
///---------------------------------------------------///

/**
 * @brief SbsRegData
 *
 *  Register Read/Writr Data Get
 *
 * @return WORD
 *
 */
GWORD SbsDFData(GVOID)
{
  GWORD wSize;
  GBYTE *pData;
  pData = (GBYTE*)GGB;
  wSize = sizeof(GlobalDFVarType)-ReportSBS->wDFAddr;
  if(wSize > SBS_MAX_PAGE_SIZE)
  {
    SBSCmd->bSize = SBS_MAX_PAGE_SIZE;
  }
  else
  {
    SBSCmd->bSize = (GBYTE)wSize;
  }
  SBSCmd->pbData = (GBYTE*)(pData + ReportSBS->wDFAddr);
  return SBSCmd->bSize;
}

/**
 * @brief SbsRegData
 *
 *  Register Read/Writr Data Get
 *
 * @return WORD
 *
 */
GVOID SbsDFDataWt(GBYTE *pbData, GWORD bSize)
{
  GBYTE *pData;
  GBYTE *pDst;

  pData = (GBYTE*)GGB;
  pDst = (GBYTE *)(pData + ReportSBS->wDFAddr);
  UpiMemcpy(pDst, pbData, (GINT32)bSize);
  
}

/**
 * @brief SbsRegDataWt
 *
 *  Register Read/Writr Data Set  
 *
 * @return WORD
 *
 */
GVOID SbsRegDataWt(GWORD wData)
{
  WRITE_REG((GBYTE)(ReportSBS->wRegAddr), GLOBYTE(wData));    
}

///---------------------------------------------------///
/// 0xC7,     [0],  *CurMeas
///---------------------------------------------------///
GWORD SbsCurMeas(GVOID)
{
  SBSCmd->pbData = (GBYTE *)(CurMeas);
  SBSCmd->bSize = (GBYTE)sizeof(*CurMeas);
  return (SBSCmd->bSize);
}

///---------------------------------------------------///
/// 0xC8,     [0],  *PreMeas
///---------------------------------------------------///
GWORD SbsPreMeas(GVOID)
{
  SBSCmd->pbData = (GBYTE *)(PreMeas);
  SBSCmd->bSize = (GBYTE)sizeof(*PreMeas);
  return (SBSCmd->bSize);
}

///---------------------------------------------------///
/// 0xC9,     [0],  Suspend Time
///---------------------------------------------------///
/**
 * @brief SbsSuspendTime
 *
 * Get suspend time
 *
 * @return Suspend time in second
 */
GWORD SbsSuspendTime(GVOID)
{
  return (UpiGauge.wSuspendTime);
}

/**
 * @brief SbsSuspendTimeWt
 *
 * Set suspend time
 *
 * @para wData suspend time in second
 * @return NULL
 */
GVOID SbsSuspendTimeWt(GWORD wData)
{
  UpiMutexLock((GVOID *)&UpiTimeMutex);
  UpiGauge.wSuspendTime = wData;
  UpiMutexUnLock((GVOID *)&UpiTimeMutex);
}

///---------------------------------------------------///
/// 0xD0,     [0],  wCodeCurr
///---------------------------------------------------///
/**
 * @brief SbsCodeCurr
 *
 *   Code of Current
 *
 * @return WORD
 *
 */     
GWORD SbsCodeCurr(GVOID)  
{
  return ReportSBS->wCodeCurr;
}     

///---------------------------------------------------///
/// 0xD0,     [0],  wCodeCurr
///---------------------------------------------------///
/**
 * @brief SbsCodeCurr
 *
 *  Internal Tempature Code
 *
 * @return WORD
 *
 */     
GWORD SbsCodeIT(GVOID)        
{
  return ReportSBS->wCodeIT;
}

///---------------------------------------------------///
/// 0xD1,     [0],  wCodeET
///---------------------------------------------------///
/**
 * @brief SbsCodeET
 *
 *  External Temperature Code
 *
 * @return WORD
 *
 */ 
GWORD SbsCodeET(GVOID)      
{
  return ReportSBS->wCodeET;
}   

///---------------------------------------------------///
/// 0xD3,     [0],  wCodeCC
///---------------------------------------------------///
/**
 * @brief SbsCodeCC
 *
 *  Coulomb Counter Code
 *
 * @return WORD
 *
 */
GWORD SbsCodeCC(GVOID)        
{
  return ReportSBS->wCodeCC;
} 

///---------------------------------------------------///
/// 0xD4,     [0],  wCodeCnt
///---------------------------------------------------///
/**
 * @brief SbsCodeCnt
 *
 *  Code of ADC1 Counter 
 *
 * @return WORD
 *
 */
GWORD SbsCodeCnt(GVOID)     
{
  return ReportSBS->wCodeCnt;
} 

///---------------------------------------------------///
/// 0xD5,     [0],  wCodeVBat1
///---------------------------------------------------///
/**
 * @brief SbsCodeVBat1
 *
 *  Code of Voltage battery1
 *
 * @return WORD
 *
 */
GWORD SbsCodeVBat1(GVOID)     
{
  return ReportSBS->wCodeVBat1;
}

///---------------------------------------------------///
/// 0xD6,     [0],  wCodeVBat2
///---------------------------------------------------///
/**
 * @brief SbsCodeVBat2
 *
 *  Code  of Voltage battery1
 *
 * @return WORD
 *
 */
GWORD SbsCodeVBat2(GVOID)   
{
  return ReportSBS->wCodeVBat2;
} 

///---------------------------------------------------///
/// 0xD7,     [0],  wCodeVBat3
///---------------------------------------------------///
/**
 * @brief SbsCodeVBat3
 *
 *  RAW Code Vbat3
 *
 * @return WORD
 *
 */
GWORD SbsCodeVBat3(GVOID)     
{
  return ReportSBS->wCodeVBat3;
}

///---------------------------------------------------///
/// 0xE0,     [0],  Coulomb Counter
///---------------------------------------------------///
/**
 * @brief SbsCoulombCounter
 *
 *  Coulomb Counter
 *
 * @return WORD
 *
 */
GWORD SbsCoulombCounter(GVOID)      
{
  return *(GWORD *)&ReportSBS->sCoulombCounter;
}

///---------------------------------------------------///
/// 0xE1,     [0],  DeltaQ
///---------------------------------------------------///
/**
 * @brief SbsDeltaQ
 *
 * Delta Capacity
 *
 * @return WORD
 *
 */
GWORD SbsDeltaQ(GVOID)      
{
  return *(GWORD *)&ReportSBS->sDeltaQ;
}

///---------------------------------------------------///
/// 0xE2,     [0],  VCell1 + VCell2
///---------------------------------------------------///
/**
 * @brief SbsVPack2
 *
 * VCell1 + VCell2
 *
 * @return WORD
 *
 */
GWORD SbsVPack2(GVOID)      
{
  return *(GWORD *)&ReportSBS->wVPack2;
}

///---------------------------------------------------///
/// 0xE3,     [0],  VCell1 + VCell2 + VCell3
///---------------------------------------------------///
/**
 * @brief SbsVPack3
 *
 * VCell1 + VCell2 + VCell3
 *
 * @return WORD
 *
 */
GWORD SbsVPack3(GVOID)
{
  return *(GWORD *)&ReportSBS->wVPack3;
}

///---------------------------------------------------///
/// 0xF0,     [0],  sCodeCurrCali
///---------------------------------------------------///
/**
 * @brief SbsCodeVBat3
 *
 *  Code of after tempeature calibrate 
 *
 * @return WORD
 *
 */
GWORD SbsCodeCurrCali(GVOID)    
{
  return  *(GWORD *)&ReportSBS->sCodeCurrCali;
}

///---------------------------------------------------///
/// 0xF1,     [0],  sRawCurr
///---------------------------------------------------///
/**
 * @brief SbsRawCurr
 *
 *  current of Raw Curr
 *
 * @return WORD
 *
 */
GWORD SbsRawCurr(GVOID)   
{
  return *(GWORD *)&ReportSBS->sRawCurr;
}

///---------------------------------------------------///
/// 0xF2,     [0],  wCodeVbat1Cali
///---------------------------------------------------///
/**
 * @brief SbsCodeVbat1Cali
 *
 *  Code of calibrate vbat1
 *
 * @return WORD
 *
 */
GWORD SbsCodeVbat1Cali(GVOID) 
{
  return ReportSBS->wCodeVbat1Cali;
}

///---------------------------------------------------///
/// 0xF3,     [0],  wRawVbat1
///---------------------------------------------------///
/**
 * @brief SbsRawVbat1
 *
 *  raw bat1 value
 *
 * @return WORD
 *
 */
GWORD SbsRawVbat1(GVOID)  
{
  return ReportSBS->wRawVbat1;
}

///---------------------------------------------------///
/// 0xF4,     [0],  wCodeVbat2Cali
///---------------------------------------------------///
/**
 * @brief SbsCodeVbat2Cali
 *
 *  Code of calibrate vbat2
 *
 * @return WORD
 *
 */
GWORD SbsCodeVbat2Cali(GVOID) 
{
  return ReportSBS->wCodeVbat2Cali;
}

///---------------------------------------------------///
/// 0xF5,     [0],  wRawVbat2
///---------------------------------------------------///
/**
 * @brief SbsRawVbat2
 *
 *  raw bat2 value
 *
 * @return WORD
 *
 */
GWORD SbsRawVbat2(GVOID)  
{
  return ReportSBS->wRawVbat2;
}

///---------------------------------------------------///
/// 0xF6,     [0],  wCodeVbat3Cali
///---------------------------------------------------///
/**
 * @brief SbsCodeVbat3Cali
 *
 *  Code of calibrate vbat3
 *
 * @return WORD
 *
 */
GWORD SbsCodeVbat3Cali(GVOID) 
{
  return ReportSBS->wCodeVbat3Cali;
}

///---------------------------------------------------///
/// 0xF7,     [0],  wRawVbat3
///---------------------------------------------------///
/**
 * @brief SbsRawVbat3
 *
 *  raw bat3 value
 *
 * @return WORD
 *
 */
GWORD SbsRawVbat3(GVOID)  
{
  return ReportSBS->wRawVbat3;
}

///---------------------------------------------------///
/// 0xF8,     [0],  wErrorStatus
///---------------------------------------------------///
/**
 * @brief SbsErrorStatus
 *
 *  Error status
 *
 * @return WORD
 *
 */
GWORD SbsErrorStatus(GVOID)  
{
  return (ReportSBS->wErrorStatus);
}

///---------------------------------------------------///
/// 0xF9,     [0],  SbsWarningStatus
///---------------------------------------------------///
/**
 * @brief SbsWarningStatus
 *
 *  Warning status
 *
 * @return WORD
 *
 */
GWORD SbsWarningStatus(GVOID)  
{
  return (ReportSBS->wWarningStatus);
}

///---------------------------------------------------///
/// 0xFA,     [0],  SbsGaugeStatus
///---------------------------------------------------///
/**
 * @brief SbsGaugeStatus
 *
 *  Gauge status
 *
 * @return WORD
 *
 */
GWORD SbsGaugeStatus(GVOID)  
{
  return (ReportSBS->wGaugeStatus);
}

///---------------------------------------------------///
/// 0xFD,     [O],  Read/Write Volt IIR Filter
///---------------------------------------------------///

/**
 * @brief SbsIIRVoltRatioWt
 *
 *  IIR Volt Ratio high(old)/low(new) byte ratio
 *
 * @return 
 *
 */
GVOID SbsIIRVoltRatioWt(GWORD wData)
{
  SBS->wVoltIIRRatio = wData;
  ReportSBS->wVoltIIRRatio = wData;
}
///---------------------------------------------------///
/// 0xFD,     [0],  wVoltIIRRatio
///---------------------------------------------------///
/**
 * @brief SbsIIRVoltRatio
 *
 *  IIR Volt Ratio high(old)/low(new) byte ratio
 *
 * @return WORD
 *
 */
GWORD SbsIIRVoltRatio(GVOID) 
{
  return ReportSBS->wVoltIIRRatio;
}

///---------------------------------------------------///
/// 0xFE,     [0],  wFWVersion
///---------------------------------------------------///
/**
 * @brief SbsFWVersion
 *
 *  Data Flash version
 *
 * @return WORD
 *
 */
GWORD SbsFWVersion(GVOID) 
{
  return ReportSBS->wFWVersion;
}

///---------------------------------------------------///
/// 0xFF,     [0],  wGGBVersion
///---------------------------------------------------///
/**
 * @brief SbsDFVersion
 *
 *  Data Flash version
 *
 * @return WORD
 *
 */
GWORD SbsDFVersion(GVOID) 
{
  return ReportSBS->wDFVersion;
}


///---------------------------------------------------///
/// 0x??,     [X],  Dummy 
///---------------------------------------------------///
/**
 * @brief SBSSerialNumber
 *
 *  SbsDummy
 *
 * @return WORD
 *
 */
GWORD SbsDummy(GVOID)
{
  return 0x55AA;
}


/**
 * @brief  SmbWritePage()
 *
 *  SMB Write page
 *
 * @return NULL
 *
 */
GBOOL SmbWriteCommand(GBYTE bSmbCmd, GBYTE *pbData, GWORD wSize)
{
  GBOOL bRtn = GTRUE;
  GWORD wData;
  
  UpiMutexLock((GVOID *)&UpiSbsMutex);
  ///-------------------------------------------------------------------------------///
  /// 1. Sbs Cmd
  ///-------------------------------------------------------------------------------/// 
  if(bSmbCmd == SBS_SYS_MODE)
  {
    wData = *(GWORD *)pbData;
    SbsSysModeWt(wData);
  }
  else if(bSmbCmd == SBS_REG_ADDR)
  {
    wData = *(GWORD *)pbData;
    SbsRegAddrWt(wData);  
  }
  else if(bSmbCmd == SBS_REG_DATA)
  {
    wData = *(GWORD *)pbData;
    SbsRegDataWt(wData);    
  }
  else if(bSmbCmd == SBS_DF_ADDR)
  {
    wData = *(GWORD *)pbData;
    SbsDFAddrWt(wData);   
  }
  else if(bSmbCmd == SBS_DF_DATA)
  {
    SbsDFDataWt(pbData, wSize);
  }
  else if (bSmbCmd == SBS_IIR_VOLT_RATIO)
  {
    wData = *(GWORD *)pbData;
    SbsIIRVoltRatioWt(wData);
  }
  else if(bSmbCmd == SBS_CAP_DATA)
  {
    wData = *((GWORD *)pbData);
    SbsCapCmd(wData);
  }
  else if(bSmbCmd == SBS_VCAP_DATA)
  {
    wData = *((GWORD *)pbData);
    SbsVCapCmd(wData);
  }
  else if(bSmbCmd == SBS_MFA)
  {
    wData = *((GWORD *)pbData);
    SbsManuAccessWt(wData);
  }
  else if(bSmbCmd == SBS_SUSPEND_TIME)
  {
    wData = *((GWORD *)pbData);
    SbsSuspendTimeWt(wData);
  }

  UpiMutexUnLock((GVOID *)&UpiSbsMutex);

  return bRtn;
  
}

/**
 * @brief  SmbReadWord()
 *
 *  SMB Read Word
 *
 * @return NULL
 *
 */
GWORD SmbReadWord(GBYTE bSmbCmd)
{
  GWORD wData;
  GBYTE bCmd;
  
  ///-------------------------------------------------------------------------------///
  /// 1. Sbs Cmd
  ///-------------------------------------------------------------------------------/// 
  if(bSmbCmd < SBS_CMD_PAGE_1)
  {
    wData = (*StdCmdTable[bSmbCmd])();
  }
  else if(bSmbCmd < SBS_CMD_PAGE_2)
  {
    bCmd = bSmbCmd - SBS_CMD_PAGE_1;
    wData = (*StdCmdTablePage2[bCmd])();
  }
  else if((SBS_CMD_PAGE_3 <= bSmbCmd) &&
          (bSmbCmd <= SBS_CMD_PAGE_4))
  {
    bCmd = bSmbCmd - SBS_CMD_PAGE_3;
    wData = (*StdCmdTablePage3[bCmd])();
  }
  else
  {
    wData = SbsDummy();
  }

  return wData;
}

/**
 * @brief  SmbReadPage()
 *
 *  SMB Read Page
 *
 * @return BYTE page count
 *
 */
GBYTE SmbReadCommand(GBYTE bSmbCmd)
{
  GBYTE bCnt = SBS_DEF_WORD_SIZE; ///< defaule word data
  GBYTE bCmd;
  GWORD wData;
  
  UpiMutexLock((GVOID *)&UpiSbsMutex);
  SBSCmd->bSize = SBS_DEF_WORD_SIZE;
  ///-------------------------------------------------------------------------------///
  /// 1. Sbs Cmd
  ///-------------------------------------------------------------------------------/// 
  if(bSmbCmd < SBS_CMD_PAGE_1)
  {
    wData = (*StdCmdTable[bSmbCmd])();
  }
  else if(bSmbCmd < SBS_CMD_PAGE_2)
  {
    bCmd = bSmbCmd - SBS_CMD_PAGE_1;
    wData = (*StdCmdTablePage2[bCmd])();
  }
  else if((SBS_CMD_PAGE_3 <= bSmbCmd) &&
          (bSmbCmd <= SBS_CMD_PAGE_4))
  {
    bCmd = bSmbCmd - SBS_CMD_PAGE_3;
    wData = (*StdCmdTablePage3[bCmd])();
  }
  else
  {
    wData = SbsDummy();
  }
  /// page commnad
  if((SBSCmd->bSize == SBS_DEF_WORD_SIZE))
  {
    UpiMemcpy(SBSCmd->buf, &wData,SBS_DEF_WORD_SIZE);
    SBSCmd->pbData = (GBYTE*)&SBSCmd->buf[0];
  }
  else
  {
    bCnt = SBSCmd->bSize;
  }
  
  UpiMutexUnLock((GVOID *)&UpiSbsMutex);

  return bCnt;
}

/**
 * @brief BatteryModeUpdate
 *
 * update the batery mode flags
 *
 * @para NULL
 * @return void
 *
 */
STATIC GVOID BatteryModeUpdate(GVOID)
{
  ///-------------------------------------------------------------------------------///
  ///  1. CF_BM : The flag is set when the MAX_ERR > CF MaxError limit
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  ///  2. CC_BM : Enable or disable the internal charger controller   => fix to 0
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  ///  3. PB_BM : Set the role of battert oacj => fix to 0
  ///-------------------------------------------------------------------------------///


  ///-------------------------------------------------------------------------------///
  ///  4. AM_BM :  Enable or disable the alarm warning to the host and smart charger system  => 0: enable, 1 : disable
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  ///  5. AM_BM :  Enable or disable force RSOC to 100% => 1: enable, 0 : disable
  ///-------------------------------------------------------------------------------///
  if((UpiGauge.bState == STATE_FULL_CHARGED) &&
     (UpiGauge.bPreState == STATE_FULL_CHARGED))
  {
    SBS->wBM = SBS->wBM | F100_BM;
  }
  if(SBS->wRSOC == 100)
  {
    SBS->wBM = SBS->wBM & (~F100_BM);
  }
}

/**
 * @brief BatteryStatusUpdate
 *
 * update the batery status
 *
 * @para NULL
 * @return void
 *
 */
STATIC GVOID BatteryStatusUpdate(GVOID)
{
  ///-------------------------------------------------------------------------------///
  /// 0.  Load from NeoVar, Using Data to speed up the battery status check
  ///-------------------------------------------------------------------------------///
  /// related parameter:
  /// OCA
  /// TCA
  /// TOA
  /// RCA
  /// RTA
  /// INIT
  /// DSG   ==>if current>0
  /// FC    ==>if
  /// FD    ==>if v > edv voltage
  /// EC3
  /// EC2
  /// EC1
  /// EC0
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 1. Check charge or dis-charge
  ///-------------------------------------------------------------------------------///
  if(GGIsDischarging()==GTRUE)
  {
    /// if there is no charging current, regard it as dischargin
    SBS->wBS |= DSG_BS; 
  }
  else
  {
    SBS->wBS &= ~(DSG_BS);
  }

  ///-------------------------------------------------------------------------------///
  /// 2.  Check init bit  shall be set 1
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 3.  Check Over Temperature Alarm (OTA)
  ///-------------------------------------------------------------------------------///
  

  ///-------------------------------------------------------------------------------///
  /// 4.  Check Remaining Capacity Alarm (RCA)
  ///-------------------------------------------------------------------------------///
  

  ///-------------------------------------------------------------------------------///
  /// 5.  Check Remaining Time Alarm (RTA)
  ///-------------------------------------------------------------------------------///
  
  ///-------------------------------------------------------------------------------///
  /// 6.  Check Error Handle (EC3~EC0)
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 7.  Check TCA clear
  ///-------------------------------------------------------------------------------///
  if((UpiGauge.bState == STATE_FULL_CHARGED) &&
     (UpiGauge.bPreState == STATE_FULL_CHARGED))
  {
    SBS->wBS = SBS->wBS | TCA_BS;
  }
  
  ///-------------------------------------------------------------------------------///
  /// 8.  Check TDA clear
  ///-------------------------------------------------------------------------------///
 
  ///-------------------------------------------------------------------------------///
  /// 9.  Check FD clear
  ///-------------------------------------------------------------------------------///
  
  ///-------------------------------------------------------------------------------///
  /// 10. Check FC clear
  ///-------------------------------------------------------------------------------///
  if((UpiGauge.bState == STATE_FULL_CHARGED) &&
     (UpiGauge.bPreState == STATE_FULL_CHARGED))
  {
    SBS->wBS = SBS->wBS | FC_BS;
  }
  
  ///-------------------------------------------------------------------------------///
  /// x.  Save the Battery status back
  ///-------------------------------------------------------------------------------///

  return;
}

/**
 * @brief SbsDataOperateStatusUpdate
 *
 * update the the operate status
 *
 * @para NULL
 * @return void
 *
 */
STATIC GVOID OperateStatusUpdate(GVOID)
{
  ///-------------------------------------------------------------------------------///
  /// 1. Check DFPage dirty status
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 7. QEN_OS
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 8. VOK_OS
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 9. DSGIN_OS
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 10. XDSGI_OS
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 11. XDSG_OS
  ///-------------------------------------------------------------------------------///


  ///-------------------------------------------------------------------------------///
  /// 2. DSG_OS
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 12. Check QMAX updates are enable
  ///-------------------------------------------------------------------------------///

}

/**
 * @brief SBSStatusCheck
 *
 * update the related SBS command status
 *
 * @para NULL
 * @return void
 *
 */
GVOID SBSStatusCheck(GVOID)
{
  ///-------------------------------------------------------------------------------///
  /// 1. Update Battery Mode
  ///-------------------------------------------------------------------------------///
  BatteryModeUpdate();
  ///-------------------------------------------------------------------------------///
  /// 2. update the battery status
  ///-------------------------------------------------------------------------------///
  BatteryStatusUpdate();

  ///-------------------------------------------------------------------------------///
  /// 3. update the operate status
  ///-------------------------------------------------------------------------------///
  OperateStatusUpdate();
}

/**
 * @brief SBSCheckMA
 *
 * Check ManufacturerAccess with ReportSBS
 *
 * @return NULL
 */
GVOID SBSCheckMA(GVOID)
{
  if(ReportSBS->wMA & RELOAD_GGB_STS_MFA)
  {
    SBS->wMA = SBS->wMA | RELOAD_GGB_STS_MFA;
    /// [AT-PM] : Set SERVICE flag ; 05/08/2015
    UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
  }

  if(ReportSBS->wMA & DYNAMIC_POLL_TIME_STS_MFA)
  {
    if(!(SBS->wMA & DYNAMIC_POLL_TIME_STS_MFA))
    {
      SBS->wMA = SBS->wMA | DYNAMIC_POLL_TIME_STS_MFA;
      /// [AT-PM] : Set SERVICE flag ; 05/08/2015
      UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
    }
  }
  else
  {
    if(SBS->wMA & DYNAMIC_POLL_TIME_STS_MFA)
    {
      SBS->wMA = SBS->wMA & (~DYNAMIC_POLL_TIME_STS_MFA);
      /// [AT-PM] : Set SERVICE flag ; 05/08/2015
      UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
    }
  }
  /// [YL] : Set Flag ; 20150617
  if((ReportSBS->wMA & RSOC_DR_STS_MFA) == RSOC_DR_FREE_MFA)
  {
  	if((SBS->wMA & RSOC_DR_STS_MFA) != RSOC_DR_FREE_MFA)
  	{
  		SBS->wMA = SBS->wMA & (~RSOC_DR_STS_MFA);
      SBS->wMA = SBS->wMA | RSOC_DR_FREE_MFA;
      /// [YL] : Set SERVICE flag ; 20150617
      UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
  	}
  }
  else if((ReportSBS->wMA & RSOC_DR_STS_MFA) == RSOC_DR_INC_MFA)
  {
    if((SBS->wMA & RSOC_DR_STS_MFA) != RSOC_DR_INC_MFA)
    {
      SBS->wMA = SBS->wMA & (~RSOC_DR_STS_MFA);
      SBS->wMA = SBS->wMA | RSOC_DR_INC_MFA;
      /// [YL] : Set SERVICE flag ; 20150617
      UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
    }
  }
  else if((ReportSBS->wMA & RSOC_DR_STS_MFA) == RSOC_DR_DEC_MFA)
  {
    if((SBS->wMA & RSOC_DR_STS_MFA) != RSOC_DR_DEC_MFA)
    {
      SBS->wMA = SBS->wMA & (~RSOC_DR_STS_MFA);
      SBS->wMA = SBS->wMA | RSOC_DR_DEC_MFA;
      /// [YL] : Set SERVICE flag ; 20150617
      UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
    }
  }
  else
  {
 	  SBS->wMA = SBS->wMA & (~RSOC_DR_STS_MFA);
    SBS->wMA = SBS->wMA | RSOC_DR_FREE_MFA;
    /// [YL] : Set SERVICE flag ; 20150617
    UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
  }
 
}

/**
 * @brief SBSCheckAlarmSts
 *
 * Check AlarmStatus with ReportSBS
 *
 * @return NULL
 */
GVOID SBSCheckAlarmSts(GVOID)
{
  if((ReportSBS->wAlarmSts & ALARM_STS_MASK_EN) != (SBS->wAlarmSts & ALARM_STS_MASK_EN))
  {
    SBS->wAlarmSts = SBS->wAlarmSts & (~ALARM_STS_MASK_EN);
    SBS->wAlarmSts = SBS->wAlarmSts | (ReportSBS->wAlarmSts & ALARM_STS_MASK_EN);

    /// [AT-PM] : Set SERVICE flag ; 05/08/2015
    UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_SERVICE_SBS_CMD;
  }
}

/**
 * @brief SBSUpdateCap
 *
 * Update capacity information
 *
 * @para wRsoc target RSOC
 * @return NULL
 */
STATIC GVOID SBSUpdateCap(GWORD wRsoc)
{
  GU4 dwTmp;
  GU2 wTmpRsoc;
  GU1 bTimeout;

  /// [AT-PM] : Maximum RSOC = 100 ; 06/01/2015
  if(wRsoc > CAP_CONST_PERCENTAGE)
  {
    wRsoc = CAP_CONST_PERCENTAGE;
  }
  SBS->wRSOC = wRsoc;

  /// [AT-PM] : Limit RM ; 06/01/2015
  dwTmp = (GU4) wRsoc;
  dwTmp = dwTmp * SBS->wFCC;
  dwTmp = dwTmp / CAP_CONST_PERCENTAGE;

  /// [AT-PM] : Optimize the RM ; 08/26/2015
  bTimeout = 0;
  while(1)
  {
    wTmpRsoc = SBSCalSoc(SBS->wRM, SBS->wFCC);

    /// [AT-PM] : The boundary is reached ; 08/26/2015
    if(wTmpRsoc != wRsoc)
    {
      if(wTmpRsoc > wRsoc)
      {
        /// [AT-PM] : Decrease 1mAh to target RSOC ; 08/26/2015
        dwTmp = dwTmp - 1;
      }
      if(wTmpRsoc < wRsoc)
      {
        /// [AT-PM] : Increase 1mAh to target RSOC ; 08/26/2015
        dwTmp = dwTmp + 1;
      }
      break;
    }

    /// [AT-PM] : Move close to the un-filtered RM ; 08/26/2015
    if(dwTmp > SBS->wRM)
    {
      /// [AT-PM] : Decrease 1mAh because of target > un-filtered ; 08/26/2015
      dwTmp = dwTmp - 1;
    }
    else if(dwTmp < SBS->wRM)
    {
      /// [AT-PM] : Increase 1mAh because of target < un-filtered ; 08/26/2015
      dwTmp = dwTmp + 1;
    }
    else
    {
      /// [AT-PM] : The target RM equal to un-filtered RM ; 08/26/2015
      GLOGD("[%s]: The target RM = the un-filtered RM = %d\n", __func__, (GWORD)dwTmp);
      break;
    }

    /// [AT-PM] : Check timeout ; 08/26/2015
    bTimeout = bTimeout + 1;
    if(bTimeout > SBS_RSOC_OPTIMIZE_TIMEOUT)
    {
      GLOGE("[%s]: Timeout at %d mAh\n", __func__, (GWORD)dwTmp);
      break;
    }
  }

  /// [AT-PM] : Set the optimized RM ; 08/26/2015
  SBS->wRM = (GWORD) dwTmp;

  /// [AT-PM] : Update ASOC ; 08/26/2015
  SBS->wASOC = SBSCalSoc(SBS->wRM, GGB->cSBSConfiguration.scData.wILMD);
}

/**
 * @brief SBSCheckRsocTimer
 *
 * Check RSOC timer
 *
 * @return NULL
 */
GVOID SBSCheckRsocTimer(GVOID)
{
  GINT32 iTmp32;
  GWORD  wMin;
  GWORD  wSec;

  /// [AT-PM] : Get time ; 06/01/2015
  wMin = (GWORD)SBS_RSOC_TIMER_MIN(SBS->wRsocTimer);
  wSec = (GWORD)SBS_RSOC_TIMER_SEC(SBS->wRsocTimer);

  /// [AT-PM] : Accumulate timer ; 06/01/2015
  iTmp32 = CurMeas->iDeltaTime;
  iTmp32 = iTmp32 / TIME_MSEC_TO_SEC;
  iTmp32 = iTmp32 + wSec;

  /// [AT-PM] : Get new second of the timer ; 06/01/2015
  wSec = (iTmp32 % TIME_SEC_TO_MIN);

  /// [AT-PM] : Get the minute of the timer ; 06/01/2015
  iTmp32 = iTmp32 / TIME_SEC_TO_MIN;
  iTmp32 = iTmp32 + wMin;
  if(wMin > CAP_CONST_PERCENTAGE)
  {
    wMin = CAP_CONST_PERCENTAGE;
  }
  else
  {
    wMin = (GWORD) iTmp32;
  }

  GLOGD("[%s]: %d / %d = %d <-> %d / %d = %d (%d:%d-%04x)\n", __func__,
        SBS->wRM,     SBS->wFCC,     SBS->wRSOC,
        SBS->wPrevRM, SBS->wPrevFcc, SBS->wPrevRsoc,
        wMin,         wSec,          SBS->wRsocTimer);

  /// [AT-PM] : Limit the RSOC step ; 06/03/2015
  if(wMin > 0)
  {
    if(SBS->wPrevRsoc > SBS->wRSOC)
    {
      if((SBS->wPrevRsoc - SBS->wRSOC) > wMin)
      {
        /// [AT-PM] : Limit the RSOC ; 06/01/2015
        SBSUpdateCap(SBS->wPrevRsoc > wMin ? SBS->wPrevRsoc - wMin : 0);
      }
      /// [YL] : keep previous capacity ; 20150617
      if(((SBS->wMA & RSOC_DR_STS_MFA) == RSOC_DR_INC_MFA) &&
         (SBS->wRsocTimer != SBS_RSOC_TIMER_INIT))
      {
        SBSUpdateCap(SBS->wPrevRsoc);
      }
      /// [AT-PM] : Reset timer ; 06/01/2015
      wMin = 0;
      wSec = 0;
    }
    else if(SBS->wPrevRsoc < SBS->wRSOC)
    {
      if((SBS->wRSOC - SBS->wPrevRsoc) > wMin)
      {
        /// [AT-PM] : Limit the RSOC ; 06/01/2015
        SBSUpdateCap(SBS->wPrevRsoc + wMin);
      }
      /// [YL] : keep previous capacity ; 20150617
      if(((SBS->wMA & RSOC_DR_STS_MFA) == RSOC_DR_DEC_MFA) &&
         (SBS->wRsocTimer != SBS_RSOC_TIMER_INIT))
      {
        SBSUpdateCap(SBS->wPrevRsoc);
      }
      /// [AT-PM] : Reset timer ; 06/01/2015
      wMin = 0;
      wSec = 0;
    }
    else
    {
    }
  }
  else
  {
    /// [AT-PM] : Kept previous RSOC ; 06/03/2015
    SBSUpdateCap(SBS->wPrevRsoc);
  }

  /// [AT-PM] : Update previous information ; 06/01/2015
  SBS->wPrevRsoc  = SBS->wRSOC;
  SBS->wPrevAsoc  = SBS->wASOC;
  SBS->wPrevRM    = SBS->wRM;
  SBS->wPrevFcc   = SBS->wFCC;
  SBS->wRsocTimer = (wMin << 8) | wSec;
}

/**
 * @brief SBSCalSoc
 *
 * Calculate SOC
 *
 * @para wRM  remaining capacity
 * @para wFcc full charge capacity
 * @return wRM / wFCC in percentage
 */
GWORD SBSCalSoc(GWORD wRM, GWORD wFcc)
{
  GU4 dwTmp;

  dwTmp = (GU4)wRM;
  dwTmp = dwTmp * CAP_CONST_PERCENTAGE;
  dwTmp = dwTmp * CAP_CONST_ROUND_BASE;
  dwTmp = dwTmp / wFcc;
  dwTmp = dwTmp + CAP_CONST_ROUND_BY_5;
  dwTmp = dwTmp / CAP_CONST_ROUND_BASE;
  if((dwTmp == 0) && (wRM != 0))
  {
    dwTmp = 1;
  }
  return ((GWORD)dwTmp);
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

