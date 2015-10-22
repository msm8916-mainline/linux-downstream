/**
 * @file Sys.c
 *
 *  system control
 *
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/

#include"UpiDef.h"

#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

/**
 * @brief SYSSoftReset
 *
 * Soft Reset System
 *
 * @return NULL
 *
 */
GVOID SYSSoftReset(GVOID)
{
  SET_BIT(REG_CTRL1, PORDET_W_SOFTRESET);
  CLR_BIT(REG_CTRL1, PORDET_W_SOFTRESET); 
}

/**
 * @brief SYSCheckPOR
 *
 * Power On Reset Check
 *
 * @return GTRUE, Just POR, otherwise, not in POR
 *
 */
GBOOL SYSCheckPOR(GVOID)
{
  GBYTE bReg;

  ///---------------------------------------///
  /// 1. Check POR Flag
  ///---------------------------------------///
  bReg = READ_REG(REG_CTRL1);

  if(bReg & PORDET_R_POR)
  {
    GLOGE("[%s]: %02x -> PORDET_R_POR == 1\n", __func__, bReg);
    return GTRUE;
  }

  ///---------------------------------------///
  /// 2. Check GG_RUN = 1 
  ///---------------------------------------///
  bReg = READ_REG(REG_MODE);
  if((bReg & MODE_GG_RUN) && (bReg!= 0xFF))
  {
    GLOGE("[%s]: %02x -> MODE_GG_RUN == 1\n", __func__, bReg);
    return GFALSE;
  }
  
  GLOGE("[%s]: %02x -> MODE_GG_RUN == 0\n", __func__, bReg);
  return GTRUE; 
}

/**
 * @brief SYSActive
 *
 * System Active
 *
 * @return NULL
 *
 */
GVOID SYSActivateIC(GVOID)
{
  ///----------------------------------------------///
  /// GG Reset
  ///----------------------------------------------///  
  WRITE_REG(REG_CTRL1, CTRL1_GG_RST | IO1DATA_W_HIGH);

  ///----------------------------------------------///
  /// GG Reset
  ///----------------------------------------------///  
  WRITE_REG(REG_MODE, MODE_GG_RUN);
}


/**
 * @brief SYSActive
 *
 * System Active
 *
 * @return NULL
 *
 */
GVOID SYSDeActivateIC(GVOID)
{
  ///----------------------------------------------///
  /// GG Reset
  ///----------------------------------------------///  
  CLR_BIT(REG_MODE, MODE_GG_RUN);
}

/**
 * @brief SYSActive
 *
 * System Active
 *
 * @return NULL
 *
 */
GVOID SYSSetGpio(GVOID)
{
  GBYTE bReg;

  /// [AT-PM] : Set GPIO1 as ALARM pin ; 05/11/2015
  bReg = READ_REG(REG_INTR_CTRL_A);
  bReg = bReg & (~INTR_CTRL_A_GPIO1_SEL);
  bReg = bReg | GPIO1_SEL_ALARM;
  WRITE_REG(REG_INTR_CTRL_A, bReg);

  /// [AT-PM] : Set GPIO2 as ALARM pin ; 05/11/2015
  bReg = READ_REG(REG_INTR_CTRL_A);
  bReg = bReg & (~INTR_CTRL_A_GPIO2_SEL);
  bReg = bReg | GPIO2_SEL_ALARM;
  WRITE_REG(REG_INTR_CTRL_A, bReg);

  /// [AT-PM] : Set GPIO3 as ALARM pin ; 05/11/2015
  bReg = READ_REG(REG_INTR_CTRL_D);
  bReg = bReg & (~INTR_CTRL_D_GPIO3_SEL);
  bReg = bReg | GPIO3_SEL_ALARM;
  WRITE_REG(REG_INTR_CTRL_D, bReg);

  /// [AT-PM] : Set GPIO4 as ALARM pin ; 05/11/2015
  bReg = READ_REG(REG_INTR_CTRL_D);
  bReg = bReg & (~INTR_CTRL_D_GPIO4_SEL);
  bReg = bReg | GPIO4_SEL_ALARM;
  WRITE_REG(REG_INTR_CTRL_D, bReg);
}

/**
 * @brief SYSSetIC
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSSetICType(GVOID)
{
  GBYTE bReg;

  bReg = READ_REG(REG_CELL_EN);
  bReg = bReg & (~CELL_EN_APPLICATION);
  bReg = bReg | (UpiGauge.ggb->cConfiguration.scAFE.bICType<<2);

  WRITE_REG(REG_CELL_EN, bReg);
}

/**
 * @brief SYSParseOTP1RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP1RawData(GVOID)
{
  GBYTE bOtpVal;
  GWORD wData;  
  
  ///-------------------------------------------------------------------------------///
  /// 1. Parse E0 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp1[OTP1_E0_INDEX];
  UpiGauge.otp.indexAdc1V200T25 = (bOtpVal & INDEX_ADC1_200_25_3_0);
  UpiGauge.otp.deltaVref = ((bOtpVal & DELTA_VREF_3_0) >> 4);
  UpiGauge.otp.adcDelta = ((bOtpVal & ADC_DELTA_3_0) >> 4);

  ///-------------------------------------------------------------------------------///
  /// 2. Parse E1 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp1[OTP1_E1_INDEX];
  UpiGauge.otp.indexAdc1V100T25 = (UpiGauge.otp.indexAdc1V100T25 |
                                   (bOtpVal & INDEX_ADC1_100_25_3_0));  
  UpiGauge.otp.ftIT = (UpiGauge.otp.ftIT |((bOtpVal & FT_IT_6_3)>>1));   

  ///-------------------------------------------------------------------------------///
  /// 3. Parse E2 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp1[OTP1_E2_INDEX];
  UpiGauge.otp.indexAdc2V200T25 = (UpiGauge.otp.indexAdc2V200T25|
                                   (bOtpVal & INDEX_ADC2_200_25_3_0));
  wData = ((bOtpVal & FT_IT_10_7)<<3);
  UpiGauge.otp.ftIT = (UpiGauge.otp.ftIT | wData);
    
  ///-------------------------------------------------------------------------------///
  /// 4. Parse E3 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp1[OTP1_E3_INDEX];
  UpiGauge.otp.indexAdc2V100T25 = (UpiGauge.otp.indexAdc2V100T25 |
                                   (bOtpVal & INDEX_ADC2_100_25_3_0));
  wData = ((bOtpVal & FT_IT_14_11)<<7);
  UpiGauge.otp.ftIT = (UpiGauge.otp.ftIT | wData);
    
}


/**
 * @brief SYSParseOTP2RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP2RawData(GVOID)
{
  GBYTE bOtpVal;
  
  ///-------------------------------------------------------------------------------///
  /// 1. Parse F0 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp2[OTP2_F0_INDEX];
  UpiGauge.otp.productType = ((bOtpVal & PRODUCT_TYPE)>>3);
  UpiGauge.otp.deltaET = ((bOtpVal & DELTA_ET_0) >> 5) | ((bOtpVal & DELTA_ET_1) >> 6);
  UpiGauge.otp.indexAdc2V100T25 = (UpiGauge.otp.indexAdc2V100T25 | 
                                   ((bOtpVal & INDEX_ADC2_100_25_4) >> 2));  
  UpiGauge.otp.adcDelta = UpiGauge.otp.adcDelta | 
                          ((bOtpVal & ADC_DELTA_8) >> 1) | 
                          ((bOtpVal & ADC_DELTA_7) << 2);
  
  ///-------------------------------------------------------------------------------///
  /// 2. Parse F1 registry
  ///-------------------------------------------------------------------------------///

  ///-------------------------------------------------------------------------------///
  /// 3. Parse F2 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp2[OTP2_F2_INDEX];
  UpiGauge.otp.otpCellEN = ((bOtpVal & OTP_CELL_EN)>>3);
    
  ///-------------------------------------------------------------------------------///
  /// 4. Parse F3 registry
  ///-------------------------------------------------------------------------------///
    
}

/**
 * @brief SYSParseOTP3RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP3RawData(GVOID)
{
  GBYTE bOtpVal;
  GWORD wData;
  
  ///-------------------------------------------------------------------------------///
  /// 1. Parse F4 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp3[OTP3_F4_INDEX];
  wData = (bOtpVal & ADC1DELTACODE25_200_9_8);
  UpiGauge.otp.adc1DeltaCodeT25V200 = (UpiGauge.otp.adc1DeltaCodeT25V200 |
                                       (wData << 8));
  wData = ((bOtpVal & DEVADDR_9_7)<<2);
  wData = (wData |((bOtpVal & DEVADDR_2_0)>>2));
  UpiGauge.otp.devAddr = wData;
  
  ///-------------------------------------------------------------------------------///
  /// 2. Parse F5 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp3[OTP3_F5_INDEX];
  UpiGauge.otp.bgrTune = ((bOtpVal & BGRTUNE_5_0)>>2);
  
  ///-------------------------------------------------------------------------------///
  /// 3. Parse F6 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp3[OTP3_F6_INDEX];
  if(bOtpVal & OSCDELTACODE25_SIGN)
  {
    bOtpVal = bOtpVal - OTP_SIGN_BYTE_SHIFT;
  }
  UpiGauge.otp.oscDeltaCode25 = bOtpVal;
    
  ///-------------------------------------------------------------------------------///
  /// 4. Parse F7 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp3[OTP3_F6_INDEX];
  if(bOtpVal & OSCDELTACODE25_SIGN)
  {
    bOtpVal = bOtpVal - OTP_SIGN_BYTE_SHIFT;
  }
  UpiGauge.otp.oscDeltaCode80 = bOtpVal;  
}

/**
 * @brief SYSParseOTP4RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP4RawData(GVOID)
{
  GBYTE bOtpVal;
  GWORD wData;
  
  ///-------------------------------------------------------------------------------///
  /// 1. Parse F8 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp4[OTP4_F8_INDEX];
  wData = (bOtpVal & ADC1DELTACODE25_200_7_0);
  UpiGauge.otp.adc1DeltaCodeT25V200 = (UpiGauge.otp.adc1DeltaCodeT25V200 |
                                       wData);
   
  ///-------------------------------------------------------------------------------///
  /// 2. Parse F9 registry
  ///-------------------------------------------------------------------------------///
 
  ///-------------------------------------------------------------------------------///
  /// 3. Parse FA registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp4[OTP4_FA_INDEX];
  wData = (bOtpVal & ADC1DELTACODE25_100_7_0);
  UpiGauge.otp.adc1DeltaCodeT25V100 = (UpiGauge.otp.adc1DeltaCodeT25V100 |
                                       wData);
   
  ///-------------------------------------------------------------------------------///
  /// 4. Parse FB registry
  ///-------------------------------------------------------------------------------///

}

/**
 * @brief SYSParseOTP4RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP5RawData(GVOID)
{
  GBYTE bOtpVal;
  GWORD wData;
  
  ///-------------------------------------------------------------------------------///
  /// 1. Parse FC registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp5[OTP5_FC_INDEX];
  wData = ((bOtpVal & ADC1DELTACODE25_100_8)<<8);
  UpiGauge.otp.adc1DeltaCodeT25V100 = (UpiGauge.otp.adc1DeltaCodeT25V100 |
                                       wData);
  wData = ((bOtpVal & ADC2DELTACODE25_100_6_0)>>1);
  UpiGauge.otp.adc2DeltaCodeT25V100 = (UpiGauge.otp.adc2DeltaCodeT25V100 | 
                                       wData); 
  ///-------------------------------------------------------------------------------///
  /// 2. Parse FD registry
  ///-------------------------------------------------------------------------------///
 
  ///-------------------------------------------------------------------------------///
  /// 3. Parse FE registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp5[OTP5_FE_INDEX];
  wData = (bOtpVal & ADC2DELTACODE25_200_7_0);
  UpiGauge.otp.adc2DeltaCodeT25V200 = (UpiGauge.otp.adc2DeltaCodeT25V200 |
                                       wData);
    
  ///-------------------------------------------------------------------------------///
  /// 4. Parse FF registry
  ///-------------------------------------------------------------------------------///

}

/**
 * @brief SYSParseOTP6RawData
 *
 * Set IC 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTP6RawData(GVOID)
{
  GBYTE volatile bOtpVal;
  GWORD volatile wData;
  
  ///-------------------------------------------------------------------------------///
  /// 1. parse 70 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp6[OTP6_70_INDEX];
  UpiGauge.otp.deltaVref = (UpiGauge.otp.deltaVref | 
                            ((bOtpVal & DELTA_VREF_4) << 4));
  UpiGauge.otp.deltaET = (UpiGauge.otp.deltaET | 
                          ((bOtpVal & (DELTA_ET_3_2)) << 1));
  UpiGauge.otp.aveIT25 = (GWORD) (UpiGauge.otp.aveIT25 | 
                          (bOtpVal & (AVE_IT25_7_3)));
  UpiGauge.otp.adcDelta = UpiGauge.otp.adcDelta |
                          ((bOtpVal & ADC_DELTA_6_4) << 4);
  
  ///-------------------------------------------------------------------------------///
  /// 2. Parse 71 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp6[OTP6_71_INDEX];
  wData = ((bOtpVal & AVE_IT25_15_8)<<8);
  UpiGauge.otp.aveIT25 = (UpiGauge.otp.aveIT25 | wData);
 
  ///-------------------------------------------------------------------------------///
  /// 3. Parse 72 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp6[OTP6_72_INDEX];
  UpiGauge.otp.indexAdc2V200T25 = (UpiGauge.otp.indexAdc2V200T25 | 
                                  ((bOtpVal & INDEX_ADC2_200_25_4) << 4));
   
  UpiGauge.otp.indexAdc1V100T25 = (UpiGauge.otp.indexAdc1V100T25 |
                                  ((bOtpVal & INDEX_ADC1_100_25_4) << 3));
  
  UpiGauge.otp.indexAdc1V200T25 = (UpiGauge.otp.indexAdc1V200T25 | 
                                  ((bOtpVal & INDEX_ADC1_200_25_4) << 2));
  
  UpiGauge.otp.aveIT80 = (UpiGauge.otp.aveIT80 | (bOtpVal & AVE_IT80_7_3));
  
  ///-------------------------------------------------------------------------------///
  /// 4. Parse 73 registry
  ///-------------------------------------------------------------------------------///
  bOtpVal = UpiGauge.otp.otp6[OTP6_73_INDEX];
  wData = ((bOtpVal & AVE_IT80_15_8)<<8);
  UpiGauge.otp.aveIT80 = (UpiGauge.otp.aveIT80 | wData);
  
}

/**
 * @brief SYSParseOTPRawData
 *
 * Get IC OTP raw Data 
 *
 * @return NULL
 *
 */
GVOID SYSParseOTPRawData(GVOID)
{
  SYSParseOTP1RawData();
  SYSParseOTP2RawData();
  SYSParseOTP3RawData();
  SYSParseOTP4RawData();
  SYSParseOTP5RawData();
  SYSParseOTP6RawData();
}

#if defined(FEATURE_PLAT_WINDOWS)
GVOID SysExchangeOTPValue(GVOID)
{
  /// exchange the OTP value
  if(iGaugeType == GAUGE_TYEP_CHANGE_OTP)
  {
    UpiGauge.otp.aveIT80 = GInputOTP.aveIT80;
  }
}
#endif ///< for FEATURE_PLAT_WINDOWS

/**
 * @brief SYSReadOTP
 *
 * Get IC OTP raw Data 
 *
 * @return NULL
 *
 */
GVOID SYSReadOTP(GVOID)
{
  if(UpiGauge.wGaugeStatus & GAUGE_STATUS_OTP_LOADED)
  {
    return;
  }
  UpiMemset(UpiGauge.otp.otp1, 0x00, 4);
  UpiMemset(UpiGauge.otp.otp2, 0x00, 4);
  UpiMemset(UpiGauge.otp.otp3, 0x00, 4);
  UpiMemset(UpiGauge.otp.otp4, 0x00, 4);
  UpiMemset(UpiGauge.otp.otp5, 0x00, 4);
  UpiMemset(UpiGauge.otp.otp6, 0x00, 4);

  if(!(UpiGauge.bRegIntrStatus & INTR_STATUS_AL_STS))
  {
    return;
  }
  ReadRegPage(OTP1_BYTE1, &UpiGauge.otp.otp1[0], OTP_RAW_SIZE);
  ReadRegPage(OTP2_BYTE1, &UpiGauge.otp.otp2[0], OTP_RAW_SIZE);
  ReadRegPage(OTP3_BYTE1, &UpiGauge.otp.otp3[0], OTP_RAW_SIZE);
  ReadRegPage(OTP4_BYTE1, &UpiGauge.otp.otp4[0], OTP_RAW_SIZE);
  ReadRegPage(OTP5_BYTE1, &UpiGauge.otp.otp5[0], OTP_RAW_SIZE);
  ReadRegPage(OTP6_BYTE1, &UpiGauge.otp.otp6[0], OTP_RAW_SIZE);
  SYSParseOTPRawData();

#if defined(FEATURE_PLAT_WINDOWS)
  /// [RY] : add the change the OTP value
  SysExchangeOTPValue();
#endif  ///< end of defined(FEATURE_PLAT_WINDOWS)
  
  UpiGauge.wGaugeStatus |= GAUGE_STATUS_OTP_LOADED;
  UpiGauge.wGaugeStatus |= GAUGE_STATUS_OTP_LOADED_PRINT;

}

/**
 * @brief SYSCheckLowVoltage
 *
 * Check Low Voltage
 *
 * @return BOOL
 *
 */
GBOOL SYSCheckLowVoltage()
{
  if(!(UpiGauge.bRegIntrStatus & INTR_STATUS_LVD_STS))
{
    UpiGauge.loopIdx = 0;
    UpiGauge.wGaugeStatus &=~GAUGE_STATUS_RESET_DONE;   
    UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
    
    UpiGauge.wErrorStatus |= GAUGE_ERR_LV_FAIL;

    GLOGD("ERROR: LOW VOLTAGE FAIL!!!\n");            
    return GTRUE;
  }
  
  UpiGauge.wErrorStatus &= ~GAUGE_ERR_LV_FAIL;
  return GFALSE;
}

/**
 * @brief SYSCheckPause
 *
 * Set the mainloop pause
 *
 * @return NULL
 *
 */
GVOID SYSCheckSysMode(GVOID)
{ 
  ///--------------------------------------------------///
  /// 1. BIT#0 : check system pause
  ///--------------------------------------------------///
  if(SBS->wSysMode & SYS_MODE_PAUSE)
  { 
    while(SBS->wSysMode & SYS_MODE_PAUSE)
    {   
      /// Clear WDT
  #if defined(FEATURE_PLAT_ARM_M0)
      UNLOCKREG();
      WDT->WTCR.WTR = WTCR_WTR;//Write 1 to clear for safety
      LOCKREG();
      #elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
      UNLOCKREG();
      WDT->WTCR.WTR = WTCR_WTR;//Write 1 to clear for safety
      LOCKREG();
      #endif ///< for FEATURE_PLAT_POWERBANK_ARM_M0
    }
  }
  
  ///--------------------------------------------------///
  /// 2. BIT#1 : Set GG-RUN
  ///--------------------------------------------------///  
  if(SBS->wSysMode & SYS_MODE_ACTIVATE)
  { 
    SYSActivateIC();
    SBS->wSysMode &=~(SYS_MODE_ACTIVATE);
  }
  
  ///--------------------------------------------------///
  /// 3. BIT#2 : CLEAR GG-RUN
  ///--------------------------------------------------///  
  if(SBS->wSysMode & SYS_MODE_DEACTIVATE)
  { 
    SYSDeActivateIC();
    SBS->wSysMode &=~(SYS_MODE_DEACTIVATE);
  } 
}

/**
 * @brief SYSWaitNextRound
 *
 * Wait next round
 *
 * @return NULL
 *
 */
GVOID SYSWaitNextRound(GVOID)
{
  #if defined(FEATURE_PLAT_WINDOWS)
  GTimeDataType tmTmp;
  GINT32        iDeltaTime;
  GDWORD        dwSleepTime;
  #endif ///< for FEATURE_PLAT_WINDOWS

  #if defined(FEATURE_PLAT_WINDOWS)
  UPI_LOCALTIME(&tmTmp);
  iDeltaTime  = (tmTmp.absTime < UpiGauge.tmNow.absTime) ? tmTmp.absTime : (tmTmp.absTime - UpiGauge.tmNow.absTime);
  dwSleepTime = (dwMainLoopDelayTime < (GDWORD)iDeltaTime) ? 0 : (dwMainLoopDelayTime - iDeltaTime);
  #endif ///< for FEATURE_PLAT_WINDOWS

  if(UpiGauge.wGaugeStatus & GAUGE_STATUS_SERVICE_SBS_CMD)
  {
    GLOGE("[%s]: No sleep to service SBS command\n", __func__);
    UPI_SLEEP(0);
  }
  else if(UpiGauge.wGaugeStatus & GAUGE_STATUS_RESET_DONE)
  {
  CurMeas->iDeltaTime =(UpiGauge.tmNow.absTime - UpiGauge.tmPre.absTime);
    #if defined(FEATURE_PLAT_LINUX)
    #elif defined(FEATURE_PLAT_ARM_M0)
    while(uPreSysSecCnt == uSysSecCnt);
    uPreSysSecCnt = uSysSecCnt;
    #elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
    #else
    UPI_SLEEP(dwSleepTime);
    #endif ///< end of FEATURE_PLAT_LINUX
  }
  else
  {
    #if defined(FEATURE_PLAT_LINUX)
    #elif defined(FEATURE_PLAT_ARM_M0)
    #elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
    #else
    UPI_SLEEP(dwSleepTime);
    #endif ///< end of FEATURE_PLAT_LINUX
  }

#if defined(FEATURE_PLAT_ARM_M0)
  UNLOCKREG();
  WDT->WTCR.WTR = WTCR_WTR;//Write 1 to clear for safety
  LOCKREG();
#elif defined(FEATURE_PLAT_POWERBANK_ARM_M0)
  UNLOCKREG();
  WDT->WTCR.WTR = WTCR_WTR;//Write 1 to clear for safety
  LOCKREG();
#endif ///< end of FEATURE_PLAT_POWERBANK_ARM_M0
}

/**
 * @brief SaveCheckWord
 *
 * Save and check word data
 *
 * @para reg   address of register
 * @para value word data to be saved
 * @return GTRUE if success
 */
STATIC GBOOL SaveCheckWord(GBYTE reg, GWORD value)
{
  GBYTE cnt;

  cnt = 0;
  while(1)
  {
    WRITE_REG16(reg, value);
    if(value == READ_REG16(reg))
    {
      break;
    }

    cnt = cnt + 1;
    if(cnt >= I2C_MAX_RETRY_CNT)
    {
      GLOGE("[%s]: Read back check reg[%02x] = %04x fail\n", __func__, reg, value);
      return (GFALSE);
    }
  }
  return (GTRUE);
}

/**
 * @brief SaveCheckByte
 *
 * Save and check byte data
 *
 * @para reg   address of register
 * @para value byte data to be saved
 * @return GTRUE if success
 */
STATIC GBOOL SaveCheckByte(GBYTE reg, GBYTE value)
{
  GBYTE cnt;

  cnt = 0;
  while(1)
  {
    WRITE_REG(reg, value);
    if(value == READ_REG(reg))
    {
      break;
    }

    cnt = cnt + 1;
    if(cnt >= I2C_MAX_RETRY_CNT)
    {
      GLOGE("[%s]: Read back check reg[%02x] = %02x fail\n", __func__, reg, value);
      return (GFALSE);
    }
  }
  return (GTRUE);
}

/**
 * @brief SYSSaveData
 *
 * Save information back to ug3105
 *
 * @return NULL
 *
 */
GVOID SYSSaveData(GVOID)
{
  GSHORT sCurCap;
  GWORD wCheckSum;
  GTimeDataType stTime;

  SaveCheckWord(SREG16_NAC, SBS->wRM);
  SaveCheckWord(SREG16_LMD, SBS->wFCC);

  sCurCap = (GSHORT) CurMeas->iCurCap;
  SaveCheckWord(SREG16_LAST_CAP, *(GWORD *)(&sCurCap));

  stTime.year =                UpiGauge.tmNow.year;
  stTime.mon  = (GSHORT)(GBYTE)UpiGauge.tmNow.mon;
  stTime.day  = (GSHORT)(GBYTE)UpiGauge.tmNow.day;
  stTime.hour = (GSHORT)(GBYTE)UpiGauge.tmNow.hour;
  stTime.min  = (GSHORT)(GBYTE)UpiGauge.tmNow.min;
  SaveCheckWord(SREG16_YEAR,       stTime.year);
  SaveCheckByte(SREG_MONTH, (GBYTE)stTime.mon);
  SaveCheckByte(SREG_DAY,   (GBYTE)stTime.day);
  SaveCheckByte(SREG_HOUR,  (GBYTE)stTime.hour);
  SaveCheckByte(SREG_MIN,   (GBYTE)stTime.min);

  SaveCheckWord(SREG16_QD_0, CAP->wQD[0]);
  SaveCheckWord(SREG16_QD_1, CAP->wQD[1]);
  SaveCheckWord(SREG16_QD_2, CAP->wQD[2]);
  SaveCheckWord(SREG16_QD_3, CAP->wQD[3]);

  SaveCheckWord(SREG16_VNAC,  SBS->wVRM);
  SaveCheckWord(SREG16_VLMD,  SBS->wVFCC);
  SaveCheckWord(SREG16_VQD_0, VCAP->wQD[0]);
  SaveCheckWord(SREG16_VQD_1, VCAP->wQD[1]);
  SaveCheckWord(SREG16_VQD_2, VCAP->wQD[2]);

  wCheckSum = SBS->wRM + 
              SBS->wFCC + 
              (*(GWORD *)(&sCurCap)) +
              stTime.year +
              stTime.mon +
              stTime.day +
              stTime.hour +
              stTime.min +
              CAP->wQD[0] +
              CAP->wQD[1] +
              CAP->wQD[2] +
              CAP->wQD[3] +
              SBS->wVRM +
              SBS->wVFCC +
              VCAP->wQD[0] +
              VCAP->wQD[1] +
              VCAP->wQD[2];
  SaveCheckWord(SREG16_CHECK_SUM, wCheckSum);
}

#ifdef FEATURE_RESTORE_FROM_IC  
/**
 * @brief SYSLoadData
 *
 * Load information back to ug3105
 *
 * @return NULL
 *
 */
GBOOL SYSLoadData(GVOID)
{
  GWORD wData;
  GSHORT sPreCap;
  
  wData = READ_REG16(SREG16_NAC) + 
          READ_REG16(SREG16_LMD) + 
          READ_REG16(SREG16_LAST_CAP) +
          READ_REG16(SREG16_YEAR) +
          READ_REG(SREG_MONTH) +
          READ_REG(SREG_DAY) +
          READ_REG(SREG_HOUR) +
          READ_REG(SREG_MIN) +
          READ_REG16(SREG16_QD_0) +
          READ_REG16(SREG16_QD_1) +
          READ_REG16(SREG16_QD_2) +
          READ_REG16(SREG16_QD_3) +
          READ_REG16(SREG16_VNAC) +
          READ_REG16(SREG16_VLMD) +
          READ_REG16(SREG16_VQD_0) +
          READ_REG16(SREG16_VQD_1) +
          READ_REG16(SREG16_VQD_2);
  if(wData != READ_REG16(SREG16_CHECK_SUM))
  {
    GLOGE("[%s]: Checksum fail (%04x != %04x).\n", __func__,
          wData,
          READ_REG16(SREG16_CHECK_SUM));
    return (GFALSE);
  }

  wData = READ_REG16(SREG16_LMD);
  if(wData == 0)
  {
    GLOGE("[%s]: FCC is 0\n", __func__);
    return (GFALSE);
  }
  SBS->wRM  = READ_REG16(SREG16_NAC);
  SBS->wFCC = wData; 

  /// [AT-PM] : Update RSOC ; 06/03/2015
  SBS->wRSOC = SBSCalSoc(SBS->wRM, SBS->wFCC);

  /// [AT-PM] : Update ReportSBS ; 06/03/2015
  UpiMutexLock((GVOID *)&UpiSbsMutex);
  ReportSBS->wRM   = SBS->wRM;
  ReportSBS->wFCC  = SBS->wFCC;
  ReportSBS->wRSOC = SBS->wRSOC;
  UpiMutexUnLock((GVOID *)&UpiSbsMutex);

  wData     = READ_REG16(SREG16_LAST_CAP);  
  sPreCap   = *(GSHORT *)&wData;  
  CurMeas->iCurCap = sPreCap;

  UpiGauge.tmRes.year = (GSHORT)READ_REG16(SREG16_YEAR);
  UpiGauge.tmRes.mon  = (GSHORT)READ_REG(SREG_MONTH);
  UpiGauge.tmRes.day  = (GSHORT)READ_REG(SREG_DAY);
  UpiGauge.tmRes.hour = (GSHORT)READ_REG(SREG_HOUR);
  UpiGauge.tmRes.min  = (GSHORT)READ_REG(SREG_MIN);

  wData = (GU2)READ_REG16(SREG16_QD_0);
  if(wData == 0)
  {
    GLOGE("[%s]: QD[0] = 0\n", __func__);
    return (GFALSE);
  }
  CAP->wQD[0] = wData;

  wData = (GU2)READ_REG16(SREG16_QD_1);
  if(wData == 0)
  {
    GLOGE("[%s]: QD[1] = 0\n", __func__);
    return (GFALSE);
  }
  CAP->wQD[1] = wData;

  wData = (GU2)READ_REG16(SREG16_QD_2);
  if(wData == 0)
  {
    GLOGE("[%s]: QD[2] = 0\n", __func__);
    return (GFALSE);
  }
  CAP->wQD[2] = wData;

  wData = (GU2)READ_REG16(SREG16_QD_3);
  if(wData == 0)
  {
    GLOGE("[%s]: QD[3] = 0\n", __func__);
    return (GFALSE);
  }
  CAP->wQD[3] = wData;

  wData = READ_REG16(SREG16_VLMD);
  if(wData == 0)
  {
    GLOGE("[%s]: VFCC is 0\n", __func__);
    return (GFALSE);
  }
  SBS->wVRM  = READ_REG16(SREG16_VNAC);
  SBS->wVFCC = wData;

  wData = (GU2)READ_REG16(SREG16_VQD_0);
  if(wData == 0)
  {
    GLOGE("[%s]: wVQD[0] = 0\n", __func__);
    return (GFALSE);
  }
  VCAP->wQD[0] = wData;

  wData = (GU2)READ_REG16(SREG16_VQD_1);
  if(wData == 0)
  {
    GLOGE("[%s]: wVQD[1] = 0\n", __func__);
    return (GFALSE);
  }
  VCAP->wQD[1] = wData;

  wData = (GU2)READ_REG16(SREG16_VQD_2);
  if(wData == 0)
  {
    GLOGE("[%s]: wVQD[2] = 0\n", __func__);
    return (GFALSE);
  }
  VCAP->wQD[2] = wData;
  GLOGE("[%s]: IC Data -> R:%d,F:%d,R:%d,LC:%lld,Y:%d,M:%d,D:%d,HR:%d,MIN:%d,QD:%d-%d-%d-%d\n", __func__,
        SBS->wRM, SBS->wFCC, SBS->wRSOC, CurMeas->iCurCap,
        UpiGauge.tmRes.year, UpiGauge.tmRes.mon, UpiGauge.tmRes.day, UpiGauge.tmRes.hour, UpiGauge.tmRes.min,
        CAP->wQD[0], CAP->wQD[1], CAP->wQD[2], CAP->wQD[3]);
  return (GTRUE);
}
#endif ///< for FEATURE_RESTORE_FROM_IC 

#ifdef FEATURE_RESTORE_FROM_IC  
/**
 * @brief SYSInit
 *
 *  Init System
 *
 * @return NULL
 *
 */
GBOOL SYSRestore(GVOID)
{ 
  UpiGauge.bRegIntrStatus    = READ_REG(REG_INTR_STATUS);

  SYSReadOTP();
  ADCInit();

  if(SYSLoadData() == GFALSE)
  {
    return (GFALSE);
  }
  
  UpiGauge.wSysMode |= SYS_MODE_RESTORED; 
  return (GTRUE);
}
#endif ///< for FEATURE_RESTORE_FROM_IC 

/**
 * @brief SYSReset
 *
 *  Reset System
 *
 * @return NULL
 *
 */
GVOID SYSReset(GVOID)
{
  UpiGauge.bRegIntrStatus    = READ_REG(REG_INTR_STATUS);
  
  ///-------------------------------------------------------------------------------///
  /// 1. Do  Soft-reset
  ///-------------------------------------------------------------------------------///
  SYSSoftReset();

  ///-------------------------------------------------------------------------------///
  /// 2. Active IC set GG_RUN = 1
  ///-------------------------------------------------------------------------------///
  SYSDeActivateIC();
  SYSActivateIC();
  UPI_SLEEP(TIME_RESET_NEXT_ROUND);
  ///-------------------------------------------------------------------------------///
  /// 3. Set GPIO mode
  ///-------------------------------------------------------------------------------///
  SYSSetGpio();
  
  ///-------------------------------------------------------------------------------///
  /// 4. Set IC Type
  ///-------------------------------------------------------------------------------///
  SYSSetICType();

  ///-------------------------------------------------------------------------------///
  /// 5. Read and parse OTP data
  ///-------------------------------------------------------------------------------///
  SYSReadOTP();

  ///-------------------------------------------------------------------------------///
  /// 6. Init ADC
  ///-------------------------------------------------------------------------------///
  ADCInit();
}

/**
 * @brief SYSInit
 *
 *  Init System
 *
 * @return NULL
 *
 */
GVOID SYSInit(GVOID)
{
  ///-------------------------------------------------------------------------------///
  /// 1. Check POR
  ///-------------------------------------------------------------------------------///
  if(SYSCheckPOR() == GFALSE)
  {
#ifdef FEATURE_RESTORE_FROM_IC  
    ///-------------------------------------------------------------------------------///
    /// Restore 
    ///-------------------------------------------------------------------------------///
    if(SYSRestore() == GTRUE)
    {
      UpiGauge.wGaugeStatus |= GAUGE_STATUS_RESET_DONE;
      UpiGauge.wGaugeStatus |= GAUGE_STATUS_MEAS_STABLE;
      UpiGauge.wSysMode |= SYS_MODE_MEAS_STABLE;
      return;
    }
#endif ///< for   FEATURE_RESTORE_FROM_IC 
  }

  ///-------------------------------------------------------------------------------///
  /// Totol Reset
  ///-------------------------------------------------------------------------------///
  SYSReset(); 
}

/**
 * @brief SYSCheckGGRun
 *
 * Check GG_RUN is set
 *
 * @return NULL
 */
GVOID SYSCheckGGRun(GVOID)
{
  GBYTE bReg;

  bReg = READ_REG(REG_MODE);
  if((!(bReg & MODE_GG_RUN)) || 
     (bReg == 0xFF))
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_GG_RUN_SET_FAIL;
  }
  else
  {
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_GG_RUN_SET_FAIL;
  }
}

/**
 * @brief SYSCheckOtp
 *
 * Check OTP data is not 0
 *
 * @return NULL
 */
GVOID SYSCheckOtp(GVOID)
{
  GU1 idx;

  if(!(UpiGauge.bRegIntrStatus & INTR_STATUS_AL_STS))
  {
    return;
  }

  /// [AT-PM] : Check otp raw data at least 1 not 0 ; 05/23/2014
  idx = 0;
  while(idx < 4)
  {
    if((GOTP->otp1[idx] != 0) ||
       (GOTP->otp2[idx] != 0) ||
       (GOTP->otp3[idx] != 0) ||
       (GOTP->otp4[idx] != 0) ||
       (GOTP->otp5[idx] != 0) ||
       (GOTP->otp6[idx] != 0))
    {
      UpiGauge.wErrorStatus &= ~GAUGE_ERR_OTP_CHECK_FAIL;
      break;
    }

    idx = idx + 1;
  }
  if(idx >= 4)
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_OTP_CHECK_FAIL;
  }

  /// [AT-PM] : Check BGRTUNE_5_0 should be within BGRTUNE_5_0_MAX and BGRTUNE_5_0_MIN ; 05/23/2014
  if((GOTP->bgrTune > (BGRTUNE_5_0_MAX >> 2)) ||
     (GOTP->bgrTune < (BGRTUNE_5_0_MIN >> 2)))
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_BGR_TUNE_FAIL;
  }
  else
  {
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_BGR_TUNE_FAIL;
  }
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus
