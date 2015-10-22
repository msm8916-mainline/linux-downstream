/**
 * @file GaugeMain.c
 *
 *  Mainloop of Gauge
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

GaugeVarType UpiGauge;

#if defined(FEATURE_PLAT_WINDOWS)
  GVOID* UpiSbsMutex = NULL; 
  GOtpDataType   GInputOTP;
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *UpiSbsMutex = NULL;
#else  ///< for FEATURE_PLAT_WINDOWS
  GVOID* UpiSbsMutex = NULL; 
#endif ///< for FEATURE_PLAT_WINDOWS
#if defined(FEATURE_PLAT_WINDOWS)
  GVOID* UpiCapMutex = NULL; 
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *UpiCapMutex = NULL;
#else  ///< for FEATURE_PLAT_WINDOWS
  GVOID* UpiCapMutex = NULL; 
#endif ///< for FEATURE_PLAT_WINDOWS
#if defined(FEATURE_PLAT_WINDOWS)
  GVOID* UpiTimeMutex = NULL; 
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *UpiTimeMutex = NULL;
#else  ///< for FEATURE_PLAT_WINDOWS
  GVOID* UpiTimeMutex = NULL; 
#endif ///< for FEATURE_PLAT_WINDOWS
#if defined(FEATURE_PLAT_WINDOWS)
  GVOID* UpiI2CMutex = NULL; 
#elif defined(FEATURE_PLAT_LINUX)
  struct mutex *UpiI2CMutex = NULL;
#else  ///< for FEATURE_PLAT_WINDOWS
  GVOID* UpiI2CMutex = NULL; 
#endif ///< for FEATURE_PLAT_WINDOWS
GDWORD ApCtlStatus = 0x00;
GSbsCmdType     *SBSCmd;
GSbsType        *ReportSBS;
GSbsType        *SBS;
GlobalDFVarType *GGB;
GAdcDataType    *GADC;
GOtpDataType    *GOTP;
GConfigDataType *CFG;

GMeasDataType   *PreMeas;
GMeasDataType   *CurMeas;
GCapDataType    *CAP;
GVCapDataType   *VCAP;

#ifdef  FEATURE_BUFFER_ENABLE
GCHAR           DebugLogBuf[DUMP_BUFFER_SIZE];
#endif  ///< end of FEATURE_BUFFER_ENABLE

#if defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
GBYTE GKboErrCode = UPI_UG31XX_KBO_RTN_PASS;
GSHORT sCaliCurr[GAUGE_TYPE_MASE_PRODUCTION_CNT+1]; 
GSHORT sFailCaliCurr[UPI_KBO_ERR_MAX_CURR_CNT+1]; 
int    iFailCaliCnt;
GSHORT sKboHiBound = UPI_KBO_CURRENT_HI_BOUND;
GSHORT sKboLowBound = UPI_KBO_CURRENT_LO_BOUND;
GSHORT sKboExtHiBound = UPI_KBO_CURRENT_HI_BOUND;
GSHORT sKboExtLowBound = UPI_KBO_CURRENT_LO_BOUND;
GSHORT sKboCurr = 0;

GBYTE  GVerifyMeasErrCode = UPI_UG31XX_VERIFY_MEAS_RTN_PASS;
GSHORT sVerifyMeasCurr    = 0;
GWORD  wVerifyMeasVolt    = 0;
#endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION

/**
 * @brief GgbInit
 *
 * Initialize GGB data
 *
 * @return NULL
 */
GVOID GGgbInit()
{
  UpiGauge.ggb        = (GlobalDFVarType *) FactoryGGBXFile;
  GGB                 = (GlobalDFVarType *) FactoryGGBXFile;

  SBS->wAlarmSts  = GGB->c1stLevelSafetyClass.scControl.bCntl;
  SBS->wAlarmSts  = SBS->wAlarmSts << 8;
  SBS->wDFVersion = GGB->cSystemData.scManufacturerInfo.wGGBVersion;

  UPI_MEMCPY(SBS->strDeviceChemistry, GGB->cSBSConfiguration.scData.strProjectCustomerSelfDef, sizeof(SBS->strDeviceChemistry));
  UPI_MEMCPY(SBS->strDeviceName, GGB->cSBSConfiguration.scData.strCustomerSelfDef, sizeof(SBS->strDeviceName));
  UPI_MEMCPY(SBS->strManufName, GGB->cSBSConfiguration.scData.strProject, sizeof(SBS->strManufName));
  GLOGE("[%s]: GGB Data -> %s-%s-%s-%04x\n", __func__,
        SBS->strDeviceChemistry,
        SBS->strDeviceName,
        SBS->strManufName,
        SBS->wDFVersion);
}

/**
 * @brief GMainInit
 *
 *  MainInit
 *
 * @return NULL
 *
 */
GVOID VarInit(GVOID)
{
  ///-------------------------------------------------------------------------------///
  /// 1. Zero memory
  ///-------------------------------------------------------------------------------///
  UPI_MEMSET(&UpiGauge, 0x00, sizeof(UpiGauge));
#ifdef  FEATURE_BUFFER_ENABLE
  UPI_MEMSET(&DebugLogBuf[0], 0x00, DUMP_BUFFER_SIZE);
#endif  ///< end of FEATURE_BUFFER_ENABLE
  UPI_LOCALTIME(&UpiGauge.tmNow);
  UPI_LOCALTIME(&UpiGauge.tmConfig);

  UpiGauge.wSuspendTime = 0;
  UpiMutexInit((GVOID *)&UpiI2CMutex);
  UpiMutexInit((GVOID *)&UpiTimeMutex);

  ///-------------------------------------------------------------------------------///
  /// 2. Set SbsData
  ///-------------------------------------------------------------------------------///
  SBS = &UpiGauge.sbs;
  ReportSBS = &UpiGauge.repSbs;
  SBSCmd = &UpiGauge.sbsCmd;
  ///-------------------------------------------------------------------------------///
  /// 3. Assign GGB Data
  ///-------------------------------------------------------------------------------///
  GGgbInit();

  GADC = (GAdcDataType   *) &UpiGauge.adc;
  GOTP = (GOtpDataType   *) &UpiGauge.otp;
  CFG = (GConfigDataType *) &UpiGauge.cfg;
  ///-------------------------------------------------------------------------------///
  /// 4. Set capacity algorithm data
  ///-------------------------------------------------------------------------------///
  CAP =  (GCapDataType *)  &UpiGauge.cap;
  VCAP = (GVCapDataType *) &UpiGauge.vCap;

  UpiGauge.loopIdx = 0;
  UpiGauge.wFWVersion = FW_VERSION;
  UpiGauge.wSysMode = UpiGauge.wSysMode | SYS_MODE_FULL_DEBUG_MSG;
  PreMeas = NULL;   
  CurMeas = &UpiGauge.measQueue[INIT_DELTAQ_INDEX];     
  UpiMutexInit((GVOID *)&UpiCapMutex);

  ///-------------------------------------------------------------------------------///
  /// 5. Sbs Mutex init
  ///-------------------------------------------------------------------------------///
  UpiMutexInit((GVOID *)&UpiSbsMutex);

  ///-------------------------------------------------------------------------------///
  /// 6. Sbs var init
  ///-------------------------------------------------------------------------------///
  SBS->wFWVersion = UpiGauge.wFWVersion;
  UpiMutexLock((GVOID *)&UpiSbsMutex);
  ReportSBS->wDFVersion = GGB->cSystemData.scManufacturerInfo.wGGBVersion;
  ReportSBS->wFWVersion = UpiGauge.wFWVersion;
  UpiMutexUnLock((GVOID *)&UpiSbsMutex);
  SBS->wVoltIIRRatio = IIR_VOLT_DEF_RATIO;

  ///-------------------------------------------------------------------------------///
  /// 7. AP extern data init
  ///-------------------------------------------------------------------------------///
#if defined(FEATURE_PLAT_WINDOWS)
  #if defined (FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
    dwMainLoopDelayTime = TIME_CALIBRATE_ROUND;
  #else
    dwMainLoopDelayTime = TIME_NORMAL_NEXT_ROUND;
  #endif  ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)

}

/**
 * @brief CalPollTime
 *
 * Calculate dynamic polling time according to RSOC
 *
 * @return NULL
 */
STATIC GVOID CalPollTime(GVOID)
{
  GU1 uIdx;

  if(SBS->wMA & DYNAMIC_POLL_TIME_STS_MFA)
  {
    GLOGD("[%s]: Set polling time according to RSOC\n", __func__);

    uIdx = 0;
    while(1)
    {
      if(PollTime[uIdx].rsoc <= 0)
      {
        break;
      }
      if(SBS->wRSOC >= PollTime[uIdx].rsoc)
      {
        break;
      }
      uIdx = uIdx + 1;
    }

#if defined(FEATURE_PLAT_WINDOWS)
    dwMainLoopDelayTime = (GDWORD) PollTime[uIdx].sec;
    dwMainLoopDelayTime = dwMainLoopDelayTime * TIME_MSEC_TO_SEC;
    GLOGD("[%s]: Set polling time to %d seconds\n", __func__, dwMainLoopDelayTime);
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)
  }
}

/**
 * @brief GMainInit
 *
 *  MainInit
 *
 * @return NULL
 *
 */
GVOID GMainInit(GVOID)
{
  GLOGE("===========================================\n"); 
  GLOGE("UpiGauge.wFWVersion = %d\n", FW_VERSION);
  GLOGE("===========================================\n"); 

  ///-------------------------------------------------------------------------------///
  /// 0. Mcu init
  ///-------------------------------------------------------------------------------///
#if defined(FEATURE_PLAT_ARM_M0)
  McuInit();
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  McuInit();
#endif ///< for FEATURE_PLAT_POWERBANK_ARM_M0

  ///-------------------------------------------------------------------------------///
  /// 1. Init Variables
  ///-------------------------------------------------------------------------------///
  VarInit();

  ///-------------------------------------------------------------------------------///
  /// 2. Read Register
  ///-------------------------------------------------------------------------------///
  UpiGauge.bRegIntrStatus  = READ_REG(REG_INTR_STATUS);

  ///-------------------------------------------------------------------------------///
  /// 3. Config Init
  ///-------------------------------------------------------------------------------///  
  CFGInit();

  ///-------------------------------------------------------------------------------///
  /// 4. System Init
  ///-------------------------------------------------------------------------------///
  SYSInit();
}

/**
 * @brief GMainTask
 *
 * Task of main loop
 *
 * @return NULL
 */
GVOID GMainTask(GVOID)
{
  ///-------------------------------------------------------------------------------///      
  /// TASK# : Round Init 
  ///-------------------------------------------------------------------------------///      
  TaskRoundInit();  

  ///--------------------------------///
  /// Check error status
  ///--------------------------------///
  if(UpiGauge.wErrorStatus != GAUGE_ERR_NO_ERROR)
  {
    GLOGD("ErrorStatus = %d\n", UpiGauge.wErrorStatus);
    UpiGauge.wGaugeStatus &= ~GAUGE_STATUS_RESET_DONE;
    UpiGauge.wErrorStatus = GAUGE_ERR_NO_ERROR;
    UpiGauge.loopIdx = 0;
    goto LABEL_NEXT_ROUND;
  }
    
  ///-------------------------------------------------------------------------------///      
  /// TASK# : ADC
  ///-------------------------------------------------------------------------------///     
  TaskADC();
    
  ///--------------------------------///
  /// Skip round check
  ///--------------------------------///
  if(UpiGauge.wGaugeStatus & GAUGE_STATUS_SKIP_ROUND)
  {
    UpiGauge.wGaugeStatus &= ~GAUGE_STATUS_SKIP_ROUND;
    goto LABEL_NEXT_ROUND;
  }

  ///--------------------------------///
  /// Check Reset Done
  ///--------------------------------///
  if(!(UpiGauge.wGaugeStatus & GAUGE_STATUS_RESET_DONE))
  {
    goto LABEL_NEXT_ROUND;
  }   

  ///-------------------------------------------------------------------------------///      
  /// TASK# : Measurement
  ///-------------------------------------------------------------------------------///     
  #ifdef FEATURE_FAKE_MEASUREMENT
    TaskFakeMeasurement();
  #else  ///< else of FEATURE_FAKE_MEASUREMENT
    TaskMeasurement();            
  #endif ///< end of FEATURE_FAKE_MEASUREMENT

  ///--------------------------------///
  /// Check Measurement Data Stable
  ///--------------------------------///
  if(!(UpiGauge.wGaugeStatus & GAUGE_STATUS_MEAS_STABLE))
  {
    UpiGauge.wSysMode |= SYS_MODE_MEAS_STABLE;
    goto LABEL_ESCAP_CAPACITY;
  }

  ///-------------------------------------------------------------------------------///      
  /// TASK# : Battery Status
  ///-------------------------------------------------------------------------------///         
  TaskGasGauge();

  ///-------------------------------------------------------------------------------///      
  /// TASK# : Alarm Status
  ///-------------------------------------------------------------------------------///         
  TaskAlarm();

  ///-------------------------------------------------------------------------------///      
  /// TASK# : Capacity
  ///-------------------------------------------------------------------------------///             
  TaskCapacity();
  TaskVCapacity();

  if(!(UpiGauge.wSysMode & SYS_MODE_INIT_CAP))
  {
    UpiGauge.wSysMode |= SYS_MODE_INIT_CAP;
    SBS->wRsocTimer = SBS_RSOC_TIMER_INIT;

    if(UpiGauge.wSysMode & SYS_MODE_RESTORED)
    {
      /// [AT-PM] : Data from IC ; 06/12/2014
      UpiGauge.wSysMode &= ~SYS_MODE_RESTORED;
    }
  }
    
  /// [AT-PM] : Set polling time according to RSOC ; 06/10/2015
  CalPollTime();

LABEL_ESCAP_CAPACITY:
  ///-------------------------------------------------------------------------------///      
  /// TASK# : SBS
  ///-------------------------------------------------------------------------------///             
  TaskSBSData();    
    
  ///-------------------------------------------------------------------------------///      
  /// Wait Next Round
  ///-------------------------------------------------------------------------------///
#ifndef FEATURE_PLAT_ARM_M0
  TaskDebugPrint();
#endif ///< for FEATRUE_PLAT_ARM_M0

LABEL_NEXT_ROUND:   

  ///-------------------------------------------------------------------------------///      
  /// TASK# : System Control
  ///-------------------------------------------------------------------------------///    
  TaskSystem();

  ///-------------------------------------------------------------------------------///      
  /// TASK# : Config File
  ///-------------------------------------------------------------------------------///    
  TaskConfig();

  ///-------------------------------------------------------------------------------///      
  /// Wait Next Round
  ///-------------------------------------------------------------------------------///   
  SYSWaitNextRound();
#if defined(FEATURE_PLAT_WINDOWS)
  #if defined (FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
    dwMainLoopDelayTime = TIME_CALIBRATE_ROUND;
  #else
    dwMainLoopDelayTime = TIME_NORMAL_NEXT_ROUND;
  #endif  ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)
    
  UpiGauge.bPreState =  UpiGauge.bState;
  ///--------------------------------///
  /// Set Reset Done
  ///--------------------------------///
  UpiGauge.loopIdx ++;
  if(UpiGauge.loopIdx > GAUGE_RESET_DONE_COUNT)
  {
    UpiGauge.wGaugeStatus |= GAUGE_STATUS_RESET_DONE;
  }
  if(UpiGauge.loopIdx > GAUGE_MEAS_STABLE_COUNT)
  {
    UpiGauge.wGaugeStatus |= GAUGE_STATUS_MEAS_STABLE;

    /// [AT-PM] : Set ADC1 queue to normal configuration ; 08/13/2015
    ADCUpdateQueue1();
  }
  if(UpiGauge.wSysMode & SYS_MODE_INIT_CAP)
  {
    UpiGauge.wSysMode |= SYS_MODE_SAVE_DATA_START;
  }
#ifdef FEATURE_PLAT_ARM_M0
  GPIO0_LOW();
  GPIO0_HIGH();
#endif ///< for FEATRUE_PLAT_ARM_M0
}

/**
 * @brief GMainExit
 *
 * Exit of main loop
 *
 * @return NULL
 */
GVOID GMainExit(GVOID)
{
  ///-------------------------------------------------------------------------------///      
  /// exit the mutex
  ///-------------------------------------------------------------------------------///   
  UpiMutexExit((GVOID *)&UpiTimeMutex);
  UpiMutexExit((GVOID *)&UpiSbsMutex);
  UpiMutexExit((GVOID *)&UpiCapMutex);
  UpiMutexExit((GVOID *)&UpiI2CMutex);
}

/**
 * @brief GCalKboCurr
 *
 *  GCalKboCurr
 *
 * @return avg. current
 *
 */
#ifdef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
BOOL GCalKboAvgCurr(GSHORT *pCurr)
{
  int i;
  int cnt = 0;
  GSHORT sCurr = 0;
  
  /// [RY] : add the cal. avg current ; 2015/5/ 6

  if(iFailCaliCnt >= UPI_KBO_ERR_MAX_CURR_CNT)
  {
    for(i = 0 ; i < UPI_KBO_ERR_MAX_CURR_CNT; i++)
    {
      sCurr = sCurr + sFailCaliCurr[i];
      cnt++;
    }
  }
  else
  {
  
    for(i = 0 ; i < GAUGE_TYPE_MASE_PRODUCTION_CNT; i++)
    {
      sCurr = sCurr + sCaliCurr[i];
      cnt++;
    }
  }
  
  sCurr = sCurr/cnt;
  *pCurr = sCurr;
  
  /// [RY] : add the Hi/Lo Byte external check
  if((sCurr <= sKboHiBound) &&
     (sCurr >= sKboLowBound))
  {
    return TRUE;    
  }
  else
  {
    return FALSE;    
  }
  
}
#endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
/**
 * @brief GWriteEeprom
 *
 *  WriteEeprom
 *
 * @return NULL
 *
 */
int GWriteEeprom(GVOID)
{
#ifdef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
  GI2CCmdType i2cCmd;
  BYTE buf[16];
  BOOL bRet = TRUE;
  BOOL bNoramlFlag = TRUE;
  GSHORT sCurr = 0;
  GSBYTE sExtCurr = 0;
  
  ///-----------------------------///
  ///  Prepare
  ///-----------------------------///
  memset(buf, 0 , sizeof(buf));
  /// [RY] : add the cal. avg current ; 2015/5/ 6
  bNoramlFlag = GCalKboAvgCurr(&sCurr);

  if(bNoramlFlag == TRUE)
  {
    buf[0] = EEPROM_START_BYTE;
  }
  else
  {
    buf[0] = EEPROM_KBO_OVER_BOUND_START_BYTE;
    
    if(sCurr >= sKboHiBound)
    {
      sExtCurr = sCurr - sKboHiBound;
      sCurr = sKboHiBound;
    }
     
    if (sCurr <= sKboLowBound)
    {
      sExtCurr = sCurr + sKboHiBound;
      sCurr = sKboHiBound;
    }
  }
  sKboCurr = sCurr;
  memcpy(&buf[1], &sCurr, sizeof(GSHORT));
  /// check sum
  buf[3] = buf[0] + buf[1] + buf[2]; 

  GLOGD("Board offset : %d , Write EEPROM [0x4C]:0x%02X [0x4D]:0x%X[0x4E]:0x%X[0x4F]:0x%X\n",
        sCurr, buf[0], buf[1], buf[2], buf[3]);
  
#ifdef FEATURE_FAKE_I2C_CMD
  return UPI_UG31XX_KBO_RTN_PASS;
#endif ///< for FEATURE_FAKE_I2C_CMD
  
  ///-----------------------------///
  ///  Write EEPROM
  ///-----------------------------///
  i2cCmd.bSlaveAddr = EEPROM_SLAVE_ADDR;
  i2cCmd.bWtBuf[0] = EEPROM_START_REG;
  i2cCmd.bWtBuf[1] = buf[0];
  i2cCmd.bWtBuf[2] = buf[1];
  i2cCmd.bWtBuf[3] = buf[2];
  i2cCmd.bWtBuf[4] = buf[3];
  i2cCmd.bWtCnt = 5;
  i2cCmd.bRdCnt = 0;
  
  bRet = EepromExeCmd(&i2cCmd);

  if(bRet == FALSE)
  {
    return UPI_UG31XX_KBO_RTN_WT_EEP_FAIL;
  }
  
  ///-----------------------------///
  ///  Read EEPROM
  ///-----------------------------///
  i2cCmd.bSlaveAddr = EEPROM_SLAVE_ADDR;
  i2cCmd.bWtBuf[0] = EEPROM_START_REG;
  i2cCmd.bWtCnt = 1;
  i2cCmd.bRdCnt = 4;  
  bRet = EepromExeCmd(&i2cCmd);
  if(bRet == FALSE)
  {
    return UPI_UG31XX_KBO_RTN_RD_EEP_FAIL;
  }

  ///-----------------------------///
  ///  Compare data
  ///-----------------------------///
  if((buf[0] != i2cCmd.bRdBuf[0]) ||
     (buf[1] != i2cCmd.bRdBuf[1]) ||
     (buf[2] != i2cCmd.bRdBuf[2]) ||
     (buf[3] != i2cCmd.bRdBuf[3]))
  {
    return UPI_UG31XX_KBO_RTN_CMP_EEP_FAIL;
  }
  GLOGD("[%s]: %02x %02x %02x %02x\n", __func__, i2cCmd.bRdBuf[0], i2cCmd.bRdBuf[1], i2cCmd.bRdBuf[2], i2cCmd.bRdBuf[3]);

  ///-----------------------------///
  /// write EEPROM EXTERN flow
  ///-----------------------------///
  if(bNoramlFlag == FALSE)
  {
    ///-----------------------------///
    ///  Write EEPROM
    ///-----------------------------///
    i2cCmd.bSlaveAddr = EEPROM_SLAVE_ADDR;
    i2cCmd.bWtBuf[0] = EEPROM_EXT_REG;
    i2cCmd.bWtBuf[1] = sExtCurr;
    i2cCmd.bWtCnt = 2;
    i2cCmd.bRdCnt = 0;
  
    bRet = EepromExeCmd(&i2cCmd);
    if(bRet == FALSE)
    {
      return UPI_UG31XX_KBO_RTN_WT_EEP_FAIL;
    }
      
    ///-----------------------------///
    ///  Read EEPROM
    ///-----------------------------///
    i2cCmd.bSlaveAddr = EEPROM_SLAVE_ADDR;
    i2cCmd.bWtBuf[0] = EEPROM_EXT_REG;
    i2cCmd.bWtCnt = 1;
    i2cCmd.bRdCnt = 1;  
    bRet = EepromExeCmd(&i2cCmd);
    if(bRet == FALSE)
    {
      return UPI_UG31XX_KBO_RTN_RD_EEP_FAIL;
    }
    
    ///-----------------------------///
    ///  Compare data
    ///-----------------------------///
    if (sExtCurr != i2cCmd.bRdBuf[0])
    {
      return UPI_UG31XX_KBO_RTN_CMP_EEP_FAIL;
    }
    GLOGD(" %02x\n", i2cCmd.bRdBuf[0]);
  }
#endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
  return UPI_UG31XX_KBO_RTN_PASS;
}

/**
 * @brief GMainLoop
 *
 *  Mainloop
 *
 * @return NULL
 *
 */
GVOID GMainLoop(GVOID)
{
#if defined(FEATURE_PLAT_WINDOWS)
  #ifndef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
  FILE *fpLog;
  char name[64];
  #endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
  int cnt = 0;
  int errCode = 0;
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)
 
#if defined (FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
  ///-------------------------------------------------------------------------------///
  /// 0. Calibration the Board offset
  ///-------------------------------------------------------------------------------///  
  while(CHK_AP_EXIT)
  {
    if((CHK_AP_NOT_KBO()) && (CHK_AP_NOT_VERIFY_MEAS()))
    {
      Sleep(10);
      continue;
    }
    /// [RY] : add check device connect status
    if (ChkBridgeConnect() == FALSE)
    {
      GKboErrCode        = UPI_UG31XX_KBO_RTN_EVM_CONN_FAIL;
      GVerifyMeasErrCode = UPI_UG31XX_VERIFY_MEAS_RTN_CONN_FAIL;
      LAVE_AP_KBO;  ///< [RY] : clean this time
      LEAVE_AP_VERIFY_MEAS;
      continue;
    }
    cnt = 0;
    sKboCurr = 0;
    iFailCaliCnt = 0;
#endif
    ///-------------------------------------------------------------------------------///
    /// 1. Main Init
    ///-------------------------------------------------------------------------------///  
    GMainInit();

#ifdef FEATURE_TEST_MAINLOOP
    GTestMainLoop();
#endif ///< for FEATURE_TEST_MAINLOOP

    ///-------------------------------------------------------------------------------///
    /// 2. Main loop
    ///-------------------------------------------------------------------------------/// 
    while(CHK_AP_EXIT)
    {    
    #if defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
      if(UpiGauge.wI2cErrCnt > CAUGE_CONN_CHK_FAIL_CNT)
      {
        break;
      }
    #endif  ///< end of defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION) 

      GMainTask();
      
    #if defined(FEATURE_PLAT_WINDOWS)
      if(iGaugeType == GAUGE_TYEP_CHANGE_OTP)
      {
        if(cnt >= GAUGE_TYPE_OTP_LOOP_CNT)
        {
          break;
        }
        cnt++;
      }
      
    #ifdef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
      if(UpiGauge.wGaugeStatus & GAUGE_STATUS_RESET_DONE)
      {
        if(CHK_AP_RUNNING_KBO())
        {
          /// [RY] : check the current in the hi/low bound;2015/5/7
          if((CurMeas->sCurr< sKboExtHiBound) && 
            (CurMeas->sCurr > sKboExtLowBound))
          {
            sCaliCurr[cnt] = CurMeas->sCurr;
            cnt++;
          }
          else
          {
            sFailCaliCurr[iFailCaliCnt] = CurMeas->sCurr;
            iFailCaliCnt++;
            if(iFailCaliCnt >= UPI_KBO_ERR_MAX_CURR_CNT)
            {
              cnt = GAUGE_TYPE_MASE_PRODUCTION_CNT;
              GKboErrCode = UPI_UG31XX_KBO_RTN_OVER_BOUND_FAIL;
              break;
            }
          }

          if(cnt >= GAUGE_TYPE_MASE_PRODUCTION_CNT)
          {
            break;
          }
        }
        else if(CHK_AP_RUNNING_VERIFY_MEAS())
        {
          cnt = cnt + 1;
          if(cnt >= GAUGE_TYPE_VERIFY_MEAS_CNT)
          {
            break;
          }
        }
      }
    #endif ///< FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
    #endif  ///< end of defined(FEATURE_PLAT_WINDOWS)
    }  ///< end of  while(CHK_AP_EXIT)

#if defined(FEATURE_PLAT_WINDOWS)
  #ifndef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
    /// [RY] : remove the log on the KBO ; 2015/5/6
    sprintf(name, "GaugeUg31xx_%04d%02d%02d_%02d%02d%02d.TXT",
            UpiGauge.tmNow.year + 1980, UpiGauge.tmNow.mon, UpiGauge.tmNow.day,
            UpiGauge.tmNow.hour, UpiGauge.tmNow.min, UpiGauge.tmNow.sec);
    fpLog = fopen(name, "a+");
    if(fpLog)
    {
      fprintf(fpLog, "V=%d(%d),C=%d(%d),IT=%d(%d),ET=%d(%d),RID=%d(%d)\n",
              CurMeas->wVCell1, GADC->wAdcVCell1Code,
              CurMeas->sCurr, SBS->wCodeCurr,
              CurMeas->sIntTemp, GADC->wAdcITCode,
              CurMeas->sExtTemp, GADC->wAdcETCode,
              CurMeas->wRid, GADC->wAdcRidCode);
      fclose(fpLog);
    }
  #endif  ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)

#if defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION)
    if(UpiGauge.wI2cErrCnt > CAUGE_CONN_CHK_FAIL_CNT)
    {
      GKboErrCode        = UPI_UG31XX_KBO_RTN_GAUGE_CONN_FAIL;
      GVerifyMeasErrCode = UPI_UG31XX_VERIFY_MEAS_RTN_CONN_FAIL;
      LAVE_AP_KBO();  ///< [RY] : clean this time
      LEAVE_AP_VERIFY_MEAS();
      continue;
    }
    ///-------------------------------------------------------------------------------///
    /// 3. Write the EEPROM flow 
    ///-------------------------------------------------------------------------------/// 
    if(CHK_AP_RUNNING_KBO())
    {
      /// running kbo last need write eeprom 
      cnt = I2C_MAX_RETRY_CNT;
      while(cnt)
      {
        errCode = GWriteEeprom();
        if(errCode == UPI_UG31XX_KBO_RTN_PASS)
        {
          break;
        }
        if(GKboErrCode == UPI_UG31XX_KBO_RTN_CHK_TIMEOUT)
        {
          break;
        }
        cnt--;
      }
    }
    if(CHK_AP_RUNNING_VERIFY_MEAS())
    {
      sVerifyMeasCurr = CurMeas->sCurr;
      wVerifyMeasVolt = CurMeas->wVCell1;
    }
    LAVE_AP_KBO();  ///< [RY] : clean this time
    LEAVE_AP_VERIFY_MEAS();
  }
#endif  ///< end of defined(FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION) 

  ///-------------------------------------------------------------------------------///
  /// 2. Main Exit
  ///-------------------------------------------------------------------------------/// 
  GMainExit();
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

