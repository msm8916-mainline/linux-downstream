/**
 * @file Task.c
 *
 *  Task Oriented
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

/**
 * @brief Round Init
 *
 *  Task of to finish the init jobs that shall be done in  each round
 *
 * @return NULL
 *
 */
GVOID TaskRoundInit(GVOID)
{
  GBYTE bReg;
  
  ///-----------------------------------------------------------------///
  /// 0. Reload GGB if necessary 
  ///-----------------------------------------------------------------///
  if(SBS->wMA & RELOAD_GGB_STS_MFA)
  {
    GGgbInit();
    GLOGE("[%s]: GGB reloaded\n", __func__);

    UpiMutexLock((GVOID *)&UpiSbsMutex);
    ReportSBS->wMA        = ReportSBS->wMA & (~RELOAD_GGB_STS_MFA);
    ReportSBS->wDFVersion = GGB->cSystemData.scManufacturerInfo.wGGBVersion;
    UpiMutexUnLock((GVOID *)&UpiSbsMutex);

    SBS->wMA = SBS->wMA & (~RELOAD_GGB_STS_MFA);
  }

  ///-----------------------------------------------------------------///
  /// 1. Set current measurement pointer
  ///-----------------------------------------------------------------///
  PreMeas = CurMeas;
  CurMeas = &UpiGauge.measQueue[(UpiGauge.loopIdx % MAX_MEAS_QUEUE_SIZE)];
  UpiGauge.pCurMeas = CurMeas;
  
  ///-----------------------------------------------------------------///
  /// 2. Get time
  ///-----------------------------------------------------------------///
  UPI_MEMCPY(&UpiGauge.tmPre, &UpiGauge.tmNow, sizeof(UpiGauge.tmNow));
  UPI_LOCALTIME(&UpiGauge.tmNow);

  ///-----------------------------------------------------------------///
  /// 3. Set measurement default value
  ///-----------------------------------------------------------------///
  CurMeas->wVCell1  = 0;
  CurMeas->sCurr    = 0;
  CurMeas->sExtTemp = 0;
  CurMeas->sIntTemp = 0;  
  CurMeas->wRid     = 0;
  CurMeas->sCurrRawCode = 0;
  CurMeas->bValid = GFALSE;
  CurMeas->bDeltaQ  = GFALSE;
  
  ///-----------------------------------------------------------------///
  /// 4. Read IC internal Registers
  ///-----------------------------------------------------------------///
  bReg = READ_REG(REG_MODE);
  GLOGD("REG_MODE = %02X\n", bReg);  

  UpiGauge.bRegIntrStatusPre = UpiGauge.bRegIntrStatus;
  UpiGauge.bRegIntrStatus    = READ_REG(REG_INTR_STATUS);

  ///--------------------------------///
  /// 5. Check if IC OK
  ///--------------------------------///
  if(UpiGauge.wErrorStatus != GAUGE_ERR_NO_ERROR)
  {
    SYSReset();
    UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
    GLOGD("ERROR: Check IC FAIL (%x)!!!\n", UpiGauge.wErrorStatus);
  }

  ///--------------------------------///
  /// 6. Read internal status
  ///--------------------------------///
  GLOGD("REG_INTR_STATUS = %02x\n", UpiGauge.bRegIntrStatus);   
  if((UpiGauge.bRegIntrStatus & INTR_STATUS_LVD_STS) &&
     (!(UpiGauge.bRegIntrStatusPre & INTR_STATUS_LVD_STS)))
  {
    SYSReset();
    UpiGauge.wGaugeStatus &= ~GAUGE_STATUS_OTP_LOADED;
    UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
    GLOGD("NOTE : LEAVE LOW VOLTAGE\n");        
  }

  ///--------------------------------///
  /// 7. Check low voltage
  ///--------------------------------///
  SYSCheckLowVoltage();

  ///--------------------------------///
  /// 8. Clear warning status
  ///--------------------------------///
  UpiGauge.wWarningStatus = GAUGE_WARN_NO_WARNING;
  UpiGauge.wGaugeStatus   = UpiGauge.wGaugeStatus & (~GAUGE_STATUS_SERVICE_SBS_CMD);
}

/**
 * @brief TaskADC
 *
 *  Task of ADC
 *
 * @return NULL
 *
 */
GVOID TaskADC(GVOID)
{ 
  GWORD wLastCodeCnt;
  GWORD wLastCodeVBat1;

  ADCConvStatus();

  wLastCodeCnt = GADC->adcRawCode.wCodeCnt;
  wLastCodeVBat1 = GADC->adcRawCode.wCodeVBat1;

  SBS->wAdcEOC |= SBS_ADC_EOC_VTM;
  
  ADCFetchCode();

  if(wLastCodeVBat1 == GADC->adcRawCode.wCodeVBat1)
  {
    GADC->bVoltCodeStopCnt++;
  }
  else
  {
    GADC->bVoltCodeStopCnt = 0;
  }

  /// [AT-PM] : Check counter for ADC hang-up issue ; 05/23/2014
  if((wLastCodeCnt == GADC->adcRawCode.wCodeCnt) && 
     (UpiGauge.wGaugeStatus & GAUGE_STATUS_RESET_DONE))
  {
    GLOGE("Code counter fixed!!!\n");
    UpiGauge.wWarningStatus |= GAUGE_WARN_COUNTER_FIXED_FAIL;
    DecimateRst();
  }
  /// [FC] : Check coltage code for ADC hang-up issue ; 06/20/2014
  else if(((GADC->bVoltCodeStopCnt == ADC2_FAIL_CRITERIA) ||
           (GADC->bVTMStopCnt == ADC2_FAIL_CRITERIA)) && 
          (UpiGauge.wGaugeStatus & GAUGE_STATUS_RESET_DONE))
  {
    GLOGE("Volt code fixed!!!\n");
    UpiGauge.wWarningStatus |= GAUGE_WARN_VOLT_FIXED_FAIL;
    GADC->bVoltCodeStopCnt = 0;
    GADC->bVTMStopCnt = 0;
    DecimateRst();
  }
  else
  {
    UpiGauge.wWarningStatus &= ~GAUGE_WARN_COUNTER_FIXED_FAIL;
    UpiGauge.wWarningStatus &= ~GAUGE_WARN_VOLT_FIXED_FAIL;
  }
}

/**
 * @brief TaskCalibration
 *
 *  Task of Re-Calibration
 *
 * @return NULL
 *
 */
GVOID TaskCalibation(GVOID)
{
  GLOGD("TASK: CALIBRATION\n");
}

/**
 * @brief TaskMeasurement
 *
 *  Task of Measurement
 *
 * @return NULL
 *
 */
GVOID TaskMeasurement(GVOID)
{
  ///------------------------------------------------///
  /// Check Status
  ///------------------------------------------------///
  if(UpiGauge.wErrorStatus & GAUGE_ERR_I2C_FAIL)
  {
    return;
  }
  
  MeasPrepare();

  CurMeas->sCurr = MeasCurr();

  CurMeas->wVCell1    = MeasVCell1();
  CurMeas->wVCell2    = MeasVCell2();
  CurMeas->wVCell3    = MeasVCell3();
  CurMeas->wMaxVCell  = MeasMaxVCell();
  CurMeas->wMinVCell  = MeasMinVCell();

  CurMeas->sExtTemp   = MeasExtTemp();
  CurMeas->sIntTemp   = MeasIntTemp();
  CurMeas->wRid       = MeasRid();
  MeasConvTime();
  CurMeas->iRawCap    = MeasCapacity(); 
  
  /// Capacity Filter
  CurMeas->iCurCap    = MeasCapFilter();
  
  /// Get Delta Cap
  CurMeas->sDeltaQ = MeasDeltaQ();
  
  CurMeas->bValid = GTRUE;
}

#ifdef FEATURE_FAKE_MEASUREMENT

/**
 * @brief TaskFakeMeasurement
 *
 *  Task of Fake Measurement
 *
 * @return NULL
 *
 */
GVOID TaskFakeMeasurement(GVOID)
{
  FakeMeasPrepare();

  CurMeas->sCurr = FakeMeasCurr();

  CurMeas->wVCell1    = FakeMeasVCell1();
  CurMeas->wVCell2    = FakeMeasVCell2();
  CurMeas->wVCell3    = FakeMeasVCell3();
  CurMeas->wMaxVCell  = MeasMaxVCell();
  CurMeas->wMinVCell  = MeasMinVCell();

  CurMeas->sExtTemp   = FakeMeasExtTemp();
  CurMeas->sIntTemp   = FakeMeasIntTemp();
  CurMeas->wRid       = FakeMeasRid();
   
  MeasConvTime();
  CurMeas->iDeltaTime = FakeMeasDeltaTime();
  CurMeas->iRawCap    = MeasCapacity(); 
  
  /// Capacity Filter
  CurMeas->iCurCap    = MeasCapFilter();
  
  /// Get Delta Cap
  CurMeas->sDeltaQ = FakeMeasDeltaQ();
  
  CurMeas->bValid = GTRUE;

  UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_IT_GAIN_FAIL;
  UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_GAIN_FAIL;
  UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_CAP_GAIN_FAIL;
  UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC2_GAIN_FAIL;
  UpiGauge.wWarningStatus &= ~GAUGE_WARN_COUNTER_FIXED_FAIL;
}

#endif ///< end of FEATURE_FAKE_MEASUREMENT

/**
 * @brief TaskBatteryStatus
 *
 *  Task of Battery Status
 *
 * @return NULL
 *
 */
GVOID TaskGasGauge(GVOID)
{
  ///-----------------------------///
  /// Get Current State     
  ///-----------------------------/// 
  GGGetState();

  ///-----------------------------///
  /// Dispatch State
  ///-----------------------------/// 
  GGDispatchState();
  
}

/**
 * @brief TaskCapacity
 *
 *  Task of Capacity
 *
 * @return NULL
 *
 */
GVOID TaskCapacity(GVOID)
{
  GSHORT sOffTime;
  GSHORT sOffTimeThrd;

  UpiMutexLock((GVOID *)&UpiCapMutex);

  GLOGD("[%s]: VC#1=%d,VC#min=%d,CURR=%d,IT=%d,ET=%d,CAP=%lld,DELTAQ=%d,SYSMODE=%04x\n", __func__,
        CurMeas->wVCell1,
        CurMeas->wMinVCell,
        CurMeas->sCurr,
        CurMeas->sIntTemp,
        CurMeas->sExtTemp,
        CurMeas->iCurCap,
        CurMeas->sDeltaQ,
        UpiGauge.wSysMode);

  if(!(UpiGauge.wSysMode & SYS_MODE_INIT_CAP))
  {
    CapInit();

    if((UpiGauge.wSysMode & SYS_MODE_RESTORED) ||
       (UpiGauge.wGaugeStatus & GAUGE_STATUS_CONFIG_LOADED))
    {
      /// [AT-PM] : Check power-off time ; 06/12/2014
      GLOGD("[%s]: Power-off time = %d-%d-%d %d:%d\n", __func__,
            UpiGauge.tmRes.year,
            UpiGauge.tmRes.mon + 1,
            UpiGauge.tmRes.day,
            UpiGauge.tmRes.hour,
            UpiGauge.tmRes.min);

      if((UpiGauge.tmNow.year != UpiGauge.tmRes.year) ||
         (UpiGauge.tmNow.mon  != UpiGauge.tmRes.mon) ||
         (UpiGauge.tmNow.day  != UpiGauge.tmRes.day))
      {
        sOffTime = TIME_MIN_TO_HOUR;
        GLOGE("[%s]: Power-off date mismatched. (%04d/%02d/%02d != %04d/%02d/%02d)\n", __func__,
              UpiGauge.tmNow.year, UpiGauge.tmNow.mon, UpiGauge.tmNow.day,
              UpiGauge.tmRes.year, UpiGauge.tmRes.mon, UpiGauge.tmRes.day);
      }
      else if(UpiGauge.tmNow.hour < UpiGauge.tmRes.hour)
      {
        sOffTime = TIME_MIN_TO_HOUR;
        GLOGE("[%s]: Power-off hour overflown. (%d < %d)\n", __func__,
              UpiGauge.tmNow.hour, UpiGauge.tmRes.hour);
      }
      else
      {
        sOffTime = UpiGauge.tmNow.hour - UpiGauge.tmRes.hour;
        sOffTime = sOffTime * TIME_MIN_TO_HOUR;
        sOffTime = sOffTime + UpiGauge.tmNow.min - UpiGauge.tmRes.min;
        GLOGE("[%s]: Power-off time = %d minutes\n", __func__, sOffTime);
      }

      /// [AT-PM] : Set off time to SBS ; 08/26/2015
      if(sOffTime < 0)
      {
        /// [AT-PM] : Set off time to maximum if calculated value is negative ; 08/26/2015
        SBS->wOffTime = SBS_OFF_TIME_MAXIMUM;
      }
      else
      {
        SBS->wOffTime = (GWORD)sOffTime;
      }

      sOffTimeThrd = (GSHORT)GGB->cGasGauging.scEDVCfg.bOffTime;
      sOffTimeThrd = sOffTimeThrd * TIME_MIN_TO_HOUR;
      if(sOffTime < sOffTimeThrd)
      {
        CapUpdate();
      }
    }
    else
    {
      /// [AT-PM] : No data from IC ; 06/12/2014
      CapInitQD();

      /// [AT-PM] : Set off time to maximum ; 08/26/2015
      SBS->wOffTime = SBS_OFF_TIME_MAXIMUM;
    }
  }
  else
  {
    CapUpdate();
  }

  CapUpdateSbs();

  UpiMutexUnLock((GVOID *)&UpiCapMutex);
}

/**
 * @brief TaskCapacity
 *
 *  Task of Capacity
 *
 * @return NULL
 *
 */
GVOID TaskVCapacity(GVOID)
{
  GSHORT sOffTime;

  UpiMutexLock((GVOID *)&UpiCapMutex);

  GLOGD("[%s]: VC#1=%d,VC#min=%d,CURR=%d,IT=%d,ET=%d,CAP=%d,DELTAQ=%d\n", __func__,
        CurMeas->wVCell1,
        CurMeas->wMinVCell,
        CurMeas->sCurr,
        CurMeas->sIntTemp,
        CurMeas->sExtTemp,
        CurMeas->iCurCap,
        CurMeas->sDeltaQ);

  if(!(UpiGauge.wSysMode & SYS_MODE_INIT_CAP))
  {
    VCapInit();

    if(UpiGauge.wSysMode & SYS_MODE_RESTORED)
    {
      /// [AT-PM] : Restore capacity from mAh to time ; 07/18/2014
      VCapRestore();

      /// [AT-PM] : Check power-off time ; 06/12/2014
      GLOGD("[%s]: Power-off time = %d-%d-%d %d:%d\n", __func__,
            UpiGauge.tmRes.year,
            UpiGauge.tmRes.mon + 1,
            UpiGauge.tmRes.day,
            UpiGauge.tmRes.hour,
            UpiGauge.tmRes.min);

      sOffTime = UpiGauge.tmNow.hour - UpiGauge.tmRes.hour;
      sOffTime = sOffTime * TIME_MIN_TO_HOUR;
      sOffTime = sOffTime + UpiGauge.tmNow.min - UpiGauge.tmRes.min;

      if(sOffTime < TIME_MIN_TO_HOUR)
      {
        VCapUpdate();
      }
    }
    else
    {
      /// [AT-PM] : No data from IC ; 06/12/2014
      VCapInitQD();
    }
  }
  else
  {
    VCapUpdate();
  }

  VCapUpdateSbs();

  UpiMutexUnLock((GVOID *)&UpiCapMutex);
}

/**
 * @brief TaskSBSData
 *
 *  Task of SBS
 *
 * @return NULL
 *
 */
GVOID TaskSBSData(GVOID)
{
  GMftDateType sbsMfaDate;

  ///-----------------------------------------///
  /// Update Status
  ///-----------------------------------------///
  SBSStatusCheck();
  
  sbsMfaDate.day = UpiGauge.tmNow.day;
  sbsMfaDate.month = UpiGauge.tmNow.mon;
  sbsMfaDate.year = UpiGauge.tmNow.year;

  ///-----------------------------------------///
  /// SBS Data Update
  ///-----------------------------------------///
  SBS->wBM             = UpiGauge.wBatteryMode;

  SBS->wVolt           = CurMeas->wVCell1 + CurMeas->wVCell2 + CurMeas->wVCell3;
  SBS->sCurr           = CurMeas->sCurr;
  SBS->sAvgCurr        = CurMeas->sCurr;
  if((GGB->cConfiguration.scRegisters.dwOPCfg & OPCFG_SBS_TEMP_SOURCE) == OPCFG_SBS_TEMP_SOURCE_INT)
  {
    SBS->wTemp         = (GWORD)(CurMeas->sIntTemp + SBS_TEMP_C_2_K);
  }
  else
  {
    SBS->wTemp         = (GWORD)(CurMeas->sExtTemp + SBS_TEMP_C_2_K);
  }
  SBS->wRid            = CurMeas->wRid;
  SBS->wMaxVCell       = CurMeas->wMaxVCell;
  SBS->wMinVCell       = CurMeas->wMinVCell;
  SBS->wVCell2         = CurMeas->wVCell3;
  SBS->wVCell1         = CurMeas->wVCell2;
  SBS->wVCell0         = CurMeas->wVCell1;
  SBS->wRawVbat1       = CurMeas->wRawVbat1;
  SBS->wCodeVbat1Cali  = CurMeas->wCodeVbat1Cali;
  SBS->wRawVbat2       = CurMeas->wRawVbat2;
  SBS->wCodeVbat2Cali  = CurMeas->wCodeVbat2Cali;
  SBS->wRawVbat3       = CurMeas->wRawVbat3;
  SBS->wCodeVbat3Cali  = CurMeas->wCodeVbat3Cali;
  SBS->sCodeCurrCali   = CurMeas->sCodeCurrCali;
  SBS->sRawCurr        = CurMeas->sRawCurr;
  SBS->wLoopIdx        = (GWORD)UpiGauge.loopIdx;

  /// C0
  SBS->wSysMode        = UpiGauge.wSysMode;

  /// E0
  SBS->sCoulombCounter = (GSHORT) CurMeas->iCurCap;
  SBS->sDeltaQ         = CurMeas->sDeltaQ;
  SBS->wVPack2         = CurMeas->wVCell1 + CurMeas->wVCell2;
  SBS->wVPack3         = CurMeas->wVCell1 + CurMeas->wVCell2 + CurMeas->wVCell3;
  SBS->wDC             = GGB->cSBSConfiguration.scData.wILMD;
  SBS->wDV             = GGB->cChargeControl.scTerminationCfg.wTPVoltage;
  SBS->wMDate          = *(GWORD*)&sbsMfaDate;

  /// F0
  SBS->wErrorStatus    = UpiGauge.wErrorStatus;
  SBS->wWarningStatus  = UpiGauge.wWarningStatus;
  SBS->wGaugeStatus    = UpiGauge.wGaugeStatus;

  ///-------------------------------------------------------------------------------///
  /// 1. Mutex Lock
  ///-------------------------------------------------------------------------------/// 
  UpiMutexLock((GVOID *)&UpiSbsMutex);

  ///-------------------------------------------------------------------------------///
  /// 2. Synchronize with reporting buffer
  ///-------------------------------------------------------------------------------/// 
  SBSCheckMA();
  SBSCheckAlarmSts();
  /// [AT-PM] : Initialize previous SBS information ; 06/01/2015
  if(UpiGauge.wSysMode & SYS_MODE_INIT_CAP)
  {
    SBSCheckRsocTimer();
  }
  else
  {
    SBS->wPrevRsoc  = SBS->wRSOC;
    SBS->wPrevAsoc  = SBS->wASOC;
    SBS->wPrevRM    = SBS->wRM;
    SBS->wPrevFcc   = SBS->wFCC;
    SBS->wRsocTimer = 0;
  }

  ///-------------------------------------------------------------------------------///
  /// 3. MEMCPY
  ///-------------------------------------------------------------------------------/// 
  UpiMemcpy(ReportSBS, SBS, sizeof(GSbsType));
  
  ///-------------------------------------------------------------------------------///
  /// 4. Mutex UnLock
  ///-------------------------------------------------------------------------------/// 
  UpiMutexUnLock((GVOID *)&UpiSbsMutex);

}

/**
 * @brief TaskSystem
 *
 *  Task of system control
 *
 * @return NULL
 *
 */
GVOID TaskSystem(GVOID)
{
  ///-------------------------------------------------///
  /// Check GG_RUN
  ///-------------------------------------------------///
  SYSCheckGGRun();

  ///-------------------------------------------------///
  /// Check OTP Data
  ///-------------------------------------------------///
  SYSCheckOtp();

  ///-------------------------------------------------///
  /// Check system mode
  ///-------------------------------------------------///
  SYSCheckSysMode();

  ///-------------------------------------------------///
  /// Save data back to IC
  ///-------------------------------------------------///
  if(UpiGauge.wSysMode & SYS_MODE_SAVE_DATA_START)
  {
    SYSSaveData();
  }
}

/**
 * @brief TaskAlarm
 *
 * Task of alarm control
 *
 * @return NULL
 */
GVOID TaskAlarm(GVOID)
{
  /// [AT-PM] : Get alarm status ; 03/30/2015
  AlarmSts();

  /// [AT-PM] : UV alarm ; 03/27/2015
  AlarmUV();
}

#ifndef FEATURE_PLAT_ARM_M0

/**
 * @brief TaskDebugPrint
 *
 *  Task of Capacity
 *
 * @return NULL
 *
 */
GVOID ResetDebugPrint(GVOID)
{
   GLOGD("-------------------------------------------\n"); 
   GLOGD("OTP1 (0xE0:E3) = %02X %02X %02X %02X \n", UpiGauge.otp.otp1[0], UpiGauge.otp.otp1[1], UpiGauge.otp.otp1[2], UpiGauge.otp.otp1[3])  ;
   GLOGD("OTP2 (0xF0:F3) = %02X %02X %02X %02X \n", UpiGauge.otp.otp2[0], UpiGauge.otp.otp2[1], UpiGauge.otp.otp2[2], UpiGauge.otp.otp2[3])  ;
   GLOGD("OTP3 (0xF4:F7) = %02X %02X %02X %02X \n", UpiGauge.otp.otp3[0], UpiGauge.otp.otp3[1], UpiGauge.otp.otp3[2], UpiGauge.otp.otp3[3])  ;
   GLOGD("OTP4 (0xF8:FB) = %02X %02X %02X %02X \n", UpiGauge.otp.otp4[0], UpiGauge.otp.otp4[1], UpiGauge.otp.otp4[2], UpiGauge.otp.otp4[3])  ;
   GLOGD("OTP5 (0xFC:FF) = %02X %02X %02X %02X \n", UpiGauge.otp.otp5[0], UpiGauge.otp.otp5[1], UpiGauge.otp.otp5[2], UpiGauge.otp.otp5[3])  ;
   GLOGD("OTP6 (0x70:73) = %02X %02X %02X %02X \n", UpiGauge.otp.otp6[0], UpiGauge.otp.otp6[1], UpiGauge.otp.otp6[2], UpiGauge.otp.otp6[3])  ;
   GLOGD("productType          = %04x %d\n", GOTP->productType,          GOTP->productType);
   GLOGD("indexAdc1V100T25     = %04x %d\n", GOTP->indexAdc1V100T25,     GOTP->indexAdc1V100T25);
   GLOGD("indexAdc1V200T25     = %04x %d\n", GOTP->indexAdc1V200T25,     GOTP->indexAdc1V200T25);
   GLOGD("adc1DeltaCodeT25V100 = %04x %d\n", GOTP->adc1DeltaCodeT25V100, GOTP->adc1DeltaCodeT25V100);
   GLOGD("adc1DeltaCodeT25V200 = %04x %d\n", GOTP->adc1DeltaCodeT25V200, GOTP->adc1DeltaCodeT25V200);
   GLOGD("indexAdc2V100T25     = %04x %d\n", GOTP->indexAdc2V100T25,     GOTP->indexAdc2V100T25);
   GLOGD("indexAdc2V200T25     = %04x %d\n", GOTP->indexAdc2V200T25,     GOTP->indexAdc2V200T25);
   GLOGD("adc2DeltaCodeT25V100 = %04x %d\n", GOTP->adc2DeltaCodeT25V100, GOTP->adc2DeltaCodeT25V100);
   GLOGD("adc2DeltaCodeT25V200 = %04x %d\n", GOTP->adc2DeltaCodeT25V200, GOTP->adc2DeltaCodeT25V200);
   GLOGD("aveIT25              = %04x %d\n", GOTP->aveIT25,              GOTP->aveIT25);
   GLOGD("aveIT80              = %04x %d\n", GOTP->aveIT80,              GOTP->aveIT80);
   GLOGD("oscDeltaCode25       = %04x %d\n", GOTP->oscDeltaCode25,       GOTP->oscDeltaCode25);
   GLOGD("oscDeltaCode80       = %04x %d\n", GOTP->oscDeltaCode80,       GOTP->oscDeltaCode80);
   GLOGD("otpCellEN            = %04x %d\n", GOTP->otpCellEN,            GOTP->otpCellEN);
   GLOGD("bgrTune              = %04x %d\n", GOTP->bgrTune,              GOTP->bgrTune);
   GLOGD("deltaET              = %04x %d\n", GOTP->deltaET,              GOTP->deltaET);
   GLOGD("deltaVref            = %04x %d\n", GOTP->deltaVref,            GOTP->deltaVref);
   GLOGD("devAddr              = %04x %d\n", GOTP->devAddr,              GOTP->devAddr);
   GLOGD("ftIT                 = %04x %d\n", GOTP->ftIT,                 GOTP->ftIT);
   GLOGD("-------------------------------------------\n");
   GLOGD("adc1CodeT25V100   = %d\n", GADC->adc1CodeT25V100);
   GLOGD("adc1CodeT25V200   = %d\n", GADC->adc1CodeT25V200);
   GLOGD("adc1CodeT80V100   = %d\n", GADC->adc1CodeT80V100);
   GLOGD("adc1CodeT80V200   = %d\n", GADC->adc1CodeT80V200);
   GLOGD("adc1GainSlope     = %d\n", GADC->adc1GainSlope);
   GLOGD("adc1GainFactorB   = %d\n", GADC->adc1GainFactorB);
   GLOGD("adc1OffsetSlope   = %d\n", GADC->adc1OffsetSlope);
   GLOGD("adc1OffsetFactorO = %d\n", GADC->adc1OffsetFactorO);
   GLOGD("adc1Gain          = %d\n", GADC->adc1Gain);
   GLOGD("adc1Offset        = %d\n", GADC->adc1Offset);
   GLOGD("adc2CodeT25V100   = %d\n", GADC->adc2CodeT25V100);
   GLOGD("adc2CodeT25V200   = %d\n", GADC->adc2CodeT25V200);
   GLOGD("adc2CodeT80V100   = %d\n", GADC->adc2CodeT80V100);
   GLOGD("adc2CodeT80V200   = %d\n", GADC->adc2CodeT80V200);
   GLOGD("adc2GainSlope     = %d\n", GADC->adc2GainSlope);
   GLOGD("adc2GainFactorB   = %d\n", GADC->adc2GainFactorB);
   GLOGD("adc2OffsetSlope   = %d\n", GADC->adc2OffsetSlope);
   GLOGD("adc2OffsetFactorO = %d\n", GADC->adc2OffsetFactorO);
   GLOGD("adc2Gain          = %d\n", GADC->adc2Gain);
   GLOGD("adc2Offset        = %d\n", GADC->adc2Offset);
   GLOGD("===========================================\n"); 

}

/**
 * @brief TaskConfig
 *
 * Config file task
 *
 * @return NULL
 */
GVOID TaskConfig(GVOID)
{
  GINT32 iDeltaTime;

  iDeltaTime = UpiGauge.tmNow.absTime - UpiGauge.tmConfig.absTime; 
  if(iDeltaTime >= CONFIG_UPDATE_INTERVAL)
  {
    UPI_LOCALTIME(&UpiGauge.tmConfig);

    /// [AT-PM] : Prepare config data ; 04/20/2015
    CFGPrepareData();

    /// [AT-PM] : Save config data to file ; 04/20/2015
    CFGUpdate();
  }
}

/**
 * @brief TaskDebugPrint
 *
 *  Task of Capacity
 *
 * @return NULL
 *
 */
GVOID TaskDebugPrint(GVOID)
{   
  GU2   idx;
  GCHAR prtBuf[256];
  GU2   idxBuf;

  GLOGD("===================================================\n"); 
  GLOGD("%02d:%02d:%02d (%d ms) : dT= %d ms\n",
        UpiGauge.tmNow.hour, 
        UpiGauge.tmNow.min, 
        UpiGauge.tmNow.sec,
        UpiGauge.tmNow.absTime, 
        (CurMeas->iDeltaTime));
  GLOGD("\n");    
  GLOGD("IDX      : %d\n", UpiGauge.loopIdx);
  GLOGD("STATE    : %02X\n", UpiGauge.bState);
  GLOGD("EOC      : %04X\n", SBS->wAdcEOC);
  GLOGD("SYS MODE : %04X\n", UpiGauge.wSysMode);
  GLOGD("\n");  
  GLOGD("CODE_CURR    : %04X / %d\n", SBS->wCodeCurr,     SBS->wCodeCurr);
  GLOGD("CODE_CURR(I) : %04X / %d\n", GADC->adcRawCode.sCodeCurrInst, GADC->adcRawCode.sCodeCurrInst);
  GLOGD("CODE_CC      : %04X / %d\n", SBS->wCodeCC,       SBS->wCodeCC);  
  GLOGD("CODE_CC(R)   : %04X / %d\n", GADC->adcCaliCode.sCodeCC,       GADC->adcCaliCode.sCodeCC);  
  GLOGD("CODE_CNT     : %04X / %d\n", SBS->wCodeCnt,      SBS->wCodeCnt);   
  GLOGD("CODE_IT      : %04X / %d\n", SBS->wCodeIT,       SBS->wCodeIT);
  GLOGD("CODE_IT(R)   : %04X / %d\n", GADC->wAdcITCode,    GADC->wAdcITCode);
  GLOGD("CODE_ET      : %04X / %d\n", SBS->wCodeET,       SBS->wCodeET);  
  GLOGD("CODE_ET(R)   : %04X / %d\n", GADC->wAdcETCode,    GADC->wAdcETCode); 
  GLOGD("CODE_ET(C)   : %04X / %d\n", GADC->adcCaliCode.sCodeETComp, GADC->adcCaliCode.sCodeETComp);
  GLOGD("CODE_VC#1    : %04X / %d\n", SBS->wCodeVBat1,    SBS->wCodeVBat1);
  GLOGD("CODE_VC#1(R) : %04X / %d\n", GADC->wAdcVCell1Code, GADC->wAdcVCell1Code);    
  GLOGD("CODE_VC#2    : %04X / %d\n", SBS->wCodeVBat2,    SBS->wCodeVBat2);
  GLOGD("CODE_VC#2(R) : %04X / %d\n", GADC->wAdcVCell2Code, GADC->wAdcVCell2Code);    
  GLOGD("CODE_VC#3    : %04X / %d\n", SBS->wCodeVBat3,    SBS->wCodeVBat3);
  GLOGD("CODE_VC#3(R) : %04X / %d\n", GADC->wAdcVCell3Code, GADC->wAdcVCell3Code);
  GLOGD("CODE_OFF     : %04X / %d\n", *(GWORD *)&SBS->sCodeOffset,  SBS->sCodeOffset);      
  GLOGD("CODE_RID     : %04X / %d\n", SBS->wCodeRid,      SBS->wCodeRid);
  GLOGD("CODE_RID(R)  : %04X / %d\n", GADC->wAdcRidCode,  GADC->wAdcRidCode);
  GLOGD("CODE_RID(C)  : %04X / %d\n", GADC->adcCaliCode.sCodeRidComp, GADC->adcCaliCode.sCodeRidComp);
  GLOGD("CONV_TIME    : %d\n",        CurMeas->iConvTime);     

  if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_2)
  { 
    GLOGD("CODE_VC#2    : %04X\n", SBS->wCodeVBat2);
  }
  
  if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_3)
  { 
    GLOGD("CODE_VC#3    : %04X\n", SBS->wCodeVBat3);
  }
  GLOGD("\n");  
  GLOGD("CURR    = %-5d mA\n",  CurMeas->sCurr);
  GLOGD("VC#1    = %-5d mV\n",  CurMeas->wVCell1);
  if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_2)
  { 
    GLOGD("VC#2    : %04X\n", CurMeas->wVCell2);
  }
  if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_3)
  { 
    GLOGD("VC#3    : %04X\n", CurMeas->wVCell3);
  }
  GLOGD("IT      = %-5d oC\n",  CurMeas->sIntTemp); 
  GLOGD("ET      = %-5d oC\n",  CurMeas->sExtTemp);   
  GLOGD("CAP     = %-5lld mAH\n", CurMeas->iCurCap);    
  GLOGD("DELTAQ  = %-5d mAH\n", CurMeas->sDeltaQ); 
  GLOGD("RID     = %-5d 0.1kOhm\n",CurMeas->wRid);
  GLOGD("\n");    
  GLOGD("RSOC    = %-5d %%\n",  SBS->wRSOC);        
  GLOGD("RM      = %-5d mAH\n", SBS->wRM);      
  GLOGD("FCC     = %-5d mAH\n", SBS->wFCC);       
  GLOGD("ErrorStatus       = %d\n", UpiGauge.wErrorStatus);
  GLOGD("WarningStatus     = %d\n", UpiGauge.wWarningStatus);
  if(UpiGauge.wGaugeStatus & GAUGE_STATUS_OTP_LOADED_PRINT)
  {
    UpiGauge.wGaugeStatus &= ~(GAUGE_STATUS_OTP_LOADED_PRINT); 
    ResetDebugPrint();
  }

  #ifdef  FEATURE_PRINT_SBS_DATA
  SbsDataLog();
  #endif  ///< end of FEATURE_PRINT_SBS_DATA

  #ifdef FEATURE_PLAT_WINDOWS
  InfoToFile();
  #endif

  idx    = 0;
  idxBuf = sprintf(prtBuf, "REG-%02x:", idx);
  while(1)
  {
    idxBuf += sprintf(&prtBuf[idxBuf], "%02x ", UpiGauge.bRegArray[idx]);
    if(idx >= 255)
    {
      GLOGD("%s\n", prtBuf);
      break;
    }
    idx = idx + 1;
    if(idxBuf >= 200)
    {
      GLOGD("%s\n", prtBuf);
      idxBuf = sprintf(prtBuf, "REG-%02x:", idx);
    }
  }

  GLOGE("UG3105-%02d:%02d:%02d(%dms)>dt:%d,I:%d(%d),CC:%d(%d),CNT:%d,IT:%d(%d),ET:%d(%d,%d),VC1:%d(%d),VC2:%d(%d),VC3:%d(%d),OFF:%d,RID:%d(%d,%d),T:%d\n",
        UpiGauge.tmNow.hour, UpiGauge.tmNow.min, UpiGauge.tmNow.sec, UpiGauge.tmNow.absTime, CurMeas->iDeltaTime,
        SBS->wCodeCurr, GADC->adcRawCode.sCodeCurrInst,
        SBS->wCodeCC, GADC->adcCaliCode.sCodeCC, SBS->wCodeCnt,
        SBS->wCodeIT, GADC->wAdcITCode,
        SBS->wCodeET, GADC->wAdcETCode, GADC->adcCaliCode.sCodeETComp,
        SBS->wCodeVBat1, GADC->wAdcVCell1Code,
        SBS->wCodeVBat2, GADC->wAdcVCell2Code,
        SBS->wCodeVBat3, GADC->wAdcVCell3Code,
        SBS->sCodeOffset,
        SBS->wCodeRid, GADC->wAdcRidCode, GADC->adcCaliCode.sCodeRidComp,
        CurMeas->iConvTime);
  GLOGE("OVERALL-%02d:%02d:%02d(%dms)>dT:%d-%04x,V:%d,I:%d,IT:%d,ET:%d,R:%d-%d(%d/%d),Q:%lld(%d),RID:%d.%d,S:%04x-%04x-%04x-%04x-%02x(%d)%02x-%02x-%04x DV:%s, ID:%s-%s, GV:%04x\n",
        UpiGauge.tmNow.hour, UpiGauge.tmNow.min, UpiGauge.tmNow.sec, UpiGauge.tmNow.absTime, CurMeas->iDeltaTime, SBS->wRsocTimer,
        CurMeas->wVCell1, CurMeas->sCurr, CurMeas->sIntTemp, CurMeas->sExtTemp,
        SBS->wRSOC, CAP->wRsoc10x, SBS->wRM, SBS->wFCC,
        CurMeas->iCurCap, CurMeas->sDeltaQ,
        (CurMeas->wRid / 10), (CurMeas->wRid % 10),
        UpiGauge.wSysMode, UpiGauge.wGaugeStatus, UpiGauge.wErrorStatus, UpiGauge.wWarningStatus, UpiGauge.bState, UpiGauge.loopIdx,
        CAP->bCapCntl, CAP->bChgState, CAP->wDsgState,
        #if defined(FEATURE_ASUS_DRV_VERSION)
        DRV_VERSION_ASUS,
        #else ///< else of defined(FEATURE_ASUS_DRV_VERSION)
        DRV_VERSION,
        #endif ///< end of defined(FEATURE_ASUS_DRV_VERSION)
        GGB->cSBSConfiguration.scData.strCustomerSelfDef, GGB->cSBSConfiguration.scData.strProjectCustomerSelfDef,
        GGB->cSystemData.scManufacturerInfo.wGGBVersion);
}

#endif ///< for ifndef FEATURE_PLAT_ARM_M0

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus


