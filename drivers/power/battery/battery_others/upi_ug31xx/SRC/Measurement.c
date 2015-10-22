/**
 * @file Measurement.c
 *
 * Measurement Control
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

const BoardFactorType BoardFactor[] =
{
///--------------------------------------------------------------------------///
///    VOLT         VOLT      CURR        CURR    INT. TEMP    EXT. TEMP
///    GAIN,      OFFSET,     GAIN,     OFFSET,      OFFSET,     OFFSET,
///--------------------------------------------------------------------------///
  {     998,          1,      1075,         -4,         -23,         13  },      ///< uG3105
  {     998,          3,      1066,         10,         -23,         13  },      ///< uG310x
  {     1000,         0,      1000,          0,           0,          0  },      ///< default
  {     1000,         0,      1000,          0,           0,          0  },      ///< default
};

static const GSHORT ExtTemperatureTable[] = {
  -100,       ///< Index = 0
  -50,        ///< Index = 1
  0,          ///< Index = 2
  50,         ///< Index = 3
  100,        ///< Index = 4
  150,        ///< Index = 5
  200,        ///< Index = 6
  250,        ///< Index = 7
  300,        ///< Index = 8
  350,        ///< Index = 9
  400,        ///< Index = 10
  450,        ///< Index = 11
  500,        ///< Index = 12
  550,        ///< Index = 13
  600,        ///< Index = 14
  650,        ///< Index = 15
  700,        ///< Index = 16
  750,        ///< Index = 17
  800,        ///< Index = 18
};

static const GSHORT FTAmbientMappingTable[] = 
{
  AMBIENT_TEMPERATURE_IN_FT,            ///< Index = 0
  AMBIENT_TEMPERATURE_IN_FT + 10,       ///< Index = 1
  AMBIENT_TEMPERATURE_IN_FT - 10,       ///< Index = 2
  AMBIENT_TEMPERATURE_IN_FT + 20,       ///< Index = 3
  AMBIENT_TEMPERATURE_IN_FT - 20,       ///< Index = 4
  AMBIENT_TEMPERATURE_IN_FT + 30,       ///< Index = 5
  AMBIENT_TEMPERATURE_IN_FT - 30,       ///< Index = 6
  AMBIENT_TEMPERATURE_IN_FT + 40,       ///< Index = 7
  AMBIENT_TEMPERATURE_IN_FT - 40,       ///< Index = 8
  AMBIENT_TEMPERATURE_IN_FT + 50,       ///< Index = 9
  AMBIENT_TEMPERATURE_IN_FT - 50,       ///< Index = 10
  AMBIENT_TEMPERATURE_IN_FT + 60,       ///< Index = 11
  AMBIENT_TEMPERATURE_IN_FT - 60,       ///< Index = 12
  AMBIENT_TEMPERATURE_IN_FT + 70,       ///< Index = 13
  AMBIENT_TEMPERATURE_IN_FT - 70,       ///< Index = 14
  AMBIENT_TEMPERATURE_IN_FT,            ///< Index = 15
};

/**
 * @brief MeasPrepare
 *
 *  Prepare Measurement
 *
 * @return NULL
 *
 */
GVOID MeasPrepare(GVOID)
{
  ///--------------------------------------------///
  /// 1. Get Calibrated IT Code
  ///--------------------------------------------///
#ifdef FEATRUE_IIR_FILTER_ENABLE
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcITCode = ADCFilterIntTemp(GADC->adcRawCode.wCodeIT);
  }
  else
  {
    GADC->wAdcITCode = GADC->adcRawCode.wCodeIT;
  }
#else ///< for FEATRUE_IIR_FILTER_ENABLE
  GADC->wAdcITCode = GADC->adcRawCode.wCodeIT;
#endif ///< for FEATRUE_IIR_FILTER_ENABLE
  GADC->adcCaliCode.wCodeIT = ADCCalibrateITCode(GADC->wAdcITCode);

  ///--------------------------------------------///
  /// 2. Get Gain / Offset of ADC1,and ADC2
  ///--------------------------------------------///
  ADC1FactorGet(GADC->wAdcITCode);
  ADC2FactorGet(GADC->wAdcITCode);

  ///--------------------------------------------///
  /// 3. Set CCOffset
  ///--------------------------------------------///
  ADCSetCCOffset();
}

/**
 * @brief MeasureCurrent
 *
 *  Current Measurement
 *
 * @return NULL
 *
 */
GSHORT MeasCurr(GVOID)
{
  GINT32 sCurr = 0;
  GSHORT sRawCode;
  GINT32 sCode;
  GBYTE bProductType;

  ///----------------------------------------------------------///
  /// Get the Calibrated Code of VCell#1
  ///----------------------------------------------------------///
  CurMeas->sCurrRawCode = GADC->adcRawCode.sCodeCurr;
  #ifdef FEATRUE_IIR_FILTER_ENABLE
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    sRawCode = ADCFilterCurr(GADC->adcRawCode.sCodeCurr);
  }
  else
  {
    sRawCode = GADC->adcRawCode.sCodeCurr;
  }
  #else ///< for FEATRUE_IIR_FILTER_ENABLE
  sRawCode = GADC->adcRawCode.sCodeCurr;
  #endif ///< for FEATRUE_IIR_FILTER_ENABLE
  GADC->sAdcCurrCode = sRawCode;
  GADC->adcCaliCode.sCodeCurr = ADC1CalibrateCode(sRawCode);

  /// [RY] : add the factor after code data
  CurMeas->sCodeCurrCali = GADC->adcCaliCode.sCodeCurr;
  
  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  sCode = (GINT32)GADC->adcCaliCode.sCodeCurr;
  sCode = sCode - ADC1_IDEAL_CODE_100MV;
  sCode = sCode * ADC1_VOLTAGE_DELTA/ADC1_IDEAL_CODE_DELTA;
  sCode = sCode + ADC1_VOLTAGE_100MV;
  sCurr = sCode / GGB->cGasGauging.scCurrentThresholds.bRsense;

  /// [AT-PM] : Apply board factor ; 01/25/2013
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);  
  sCurr = sCurr - BoardFactor[bProductType].currentOffset;
  
  sCurr = sCurr * BOARD_FACTOR_CONST / BoardFactor[bProductType].currentGain;

  /// [RY] : add the factor after code data
  CurMeas->sRawCurr = (GSHORT)sCurr;

  if(CHK_AP_NOT_KBO())
  {
    /// [AT-PM] : Apply calibration factor ; 01/25/2013
    sCurr = sCurr - GGB->cCalibration.scCaliData.sAdc1PosOffset;

    if(sCurr > 0)
    {
      sCurr = sCurr * CALIBRATION_FACTOR_CONST / GGB->cCalibration.scCaliData.sAdc1PGain;
    }
    else
    {
      sCurr = sCurr * CALIBRATION_FACTOR_CONST / GGB->cCalibration.scCaliData.sAdc1NGain;
    }
  }
  return (GSHORT)sCurr;
}

/**
 * @brief MeasureVCell1
 *
 *  VCell#1 Measurement
 *
 * @return NULL
 *
 */
GWORD MeasVCell1(GVOID)
{
  GWORD wRawCode = GADC->adcRawCode.wCodeVBat1;
  GWORD wCaliCode;
  GINT32 i32Volt; 
  GBYTE bProductType;

  /// [AT-PM] : Retrun 0 if code = 0 ; 08/12/2015
  if(wRawCode == 0)
  {
    GLOGE("[%s]: CODE = 0\n", __func__);
    return (0);
  }

  ///----------------------------------------------------------///
  /// Get the Calibrated Code of VCell#1
  ///----------------------------------------------------------///
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcVCell1Code = ADC_FILTER_VCELL1(wRawCode);
  }
  else
  {
    GADC->wAdcVCell1Code = wRawCode;
  }
  GLOGD("[%s]: RAW = %d -> FILTER = %d\n", __func__, wRawCode, GADC->wAdcVCell1Code);

  GADC->adcCaliCode.wCodeVBat1 = ADC2CalibrateCode(GADC->wAdcVCell1Code);
  GLOGD("[%s]: CALIBRATED = %d\n", __func__, GADC->adcCaliCode.wCodeVBat1);

  /// [RY] : add the factor after code data
  CurMeas->wCodeVbat1Cali = GADC->adcCaliCode.wCodeVBat1;

  wCaliCode = GADC->adcCaliCode.wCodeVBat1;

  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  i32Volt = (GINT32)wCaliCode;
  i32Volt = i32Volt - ADC2_IDEAL_CODE_100MV;
  i32Volt = i32Volt * ADC2_VOLTAGE_DELTA / ADC2_IDEAL_CODE_DELTA;
  i32Volt = i32Volt + ADC2_VOLTAGE_100MV;
  GLOGD("[%s]: RAW VOLT = %d\n", __func__, i32Volt);

  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);

  /// [AT-PM] : Apply board factor ; 01/25/2013  
  i32Volt = i32Volt - BoardFactor[bProductType].voltageOffset;
  i32Volt = i32Volt * BOARD_FACTOR_CONST/BoardFactor[bProductType].voltageGain;
  GLOGD("[%s]: BOARD FACTOR = (%d,%d) -> %d\n", __func__, 
        BoardFactor[bProductType].voltageGain, 
        BoardFactor[bProductType].voltageOffset, 
        i32Volt);

  /// [RY] : add the factor after code data
  CurMeas->wRawVbat1 = (GWORD)i32Volt;

  /// [AT-PM] : Apply calibration parameter ; 01/25/2013
  i32Volt = i32Volt - GGB->cCalibration.scCaliData.sAdc2Offset;
  i32Volt = i32Volt * CALIBRATION_FACTOR_CONST/ GGB->cCalibration.scCaliData.sAdc2Gain;
  GLOGD("[%s]: CALIBRATION = (%d,%d), %d\n", __func__,
        GGB->cCalibration.scCaliData.sAdc2Gain,
        GGB->cCalibration.scCaliData.sAdc2Offset,
        i32Volt);
  return (GWORD)i32Volt;  
}

/**
 * @brief MeasureVCell2
 *
 *  VCell#2 Measurement
 *
 * @return NULL
 *
 */
GWORD MeasVCell2(GVOID)
{
  GWORD wRawCode = GADC->adcRawCode.wCodeVBat2;
  GWORD wCaliCode;
  GINT32 i32Volt; 
  GBYTE bProductType;

  /// [AT-PM] : Retrun 0 if code = 0 ; 08/12/2015
  if((wRawCode == 0) &&
     (GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_2))
  {
    GLOGE("[%s]: CODE = 0\n", __func__);
    return (0);
  }

  ///----------------------------------------------------------///
  /// Get the Calibrated Code of VCell#1
  ///----------------------------------------------------------///
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcVCell2Code = ADC_FILTER_VCELL2(wRawCode);
  }
  else
  {
    GADC->wAdcVCell2Code = wRawCode;
  }
  GADC->adcCaliCode.wCodeVBat2 = ADC2CalibrateCode(GADC->wAdcVCell2Code);

  /// [RY] : add the factor after code data
  CurMeas->wCodeVbat2Cali = GADC->adcCaliCode.wCodeVBat2;

  wCaliCode = GADC->adcCaliCode.wCodeVBat2;

  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  i32Volt = (GINT32)wCaliCode;
  i32Volt = i32Volt - ADC2_IDEAL_CODE_100MV;
  i32Volt = i32Volt * ADC2_VOLTAGE_DELTA / ADC2_IDEAL_CODE_DELTA;
  i32Volt = i32Volt + ADC2_VOLTAGE_100MV;

  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);

  /// [AT-PM] : Apply board factor ; 01/25/2013  
  i32Volt = i32Volt - BoardFactor[bProductType].voltageOffset;
  i32Volt = i32Volt * BOARD_FACTOR_CONST/BoardFactor[bProductType].voltageGain;

  /// [RY] : add the factor after code data
  CurMeas->wRawVbat2 = (GWORD)i32Volt;

  /// [AT-PM] : Apply calibration parameter ; 01/25/2013
  i32Volt = i32Volt - GGB->cCalibration.scCaliData.sVbat2Offset;
  i32Volt = i32Volt * CALIBRATION_FACTOR_CONST/ GGB->cCalibration.scCaliData.wVbat2Gain;

  /// [AT-PM] : Get cell voltage ; 06/13/2014
  i32Volt = (i32Volt*2) - CurMeas->wVCell1;

	if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_2)
	{
  	return (GWORD)i32Volt;
	}
	else
	{
		return (0);
	}
}

/**
 * @brief MeasureVCell3
 *
 *  VCell#3 Measurement
 *
 * @return NULL
 *
 */
GWORD MeasVCell3(GVOID)
{
  GWORD wRawCode = GADC->adcRawCode.wCodeVBat3;
  GWORD wCaliCode;
  GINT32 i32Volt; 
  GBYTE bProductType;

  /// [AT-PM] : Retrun 0 if code = 0 ; 08/12/2015
  if((wRawCode == 0) &&
     (GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_3))
  {
    GLOGE("[%s]: CODE = 0\n", __func__);
    return (0);
  }

  ///----------------------------------------------------------///
  /// Get the Calibrated Code of VCell#1
  ///----------------------------------------------------------///
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcVCell3Code = ADC_FILTER_VCELL3(wRawCode);
  }
  else
  {
    GADC->wAdcVCell3Code = wRawCode;
  }
  GADC->adcCaliCode.wCodeVBat3 = ADC2CalibrateCode(GADC->wAdcVCell3Code);

  /// [RY] : add the factor after code data
  CurMeas->wCodeVbat3Cali = GADC->adcCaliCode.wCodeVBat3;

  wCaliCode = GADC->adcCaliCode.wCodeVBat3;

  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  i32Volt = (GINT32)wCaliCode;
  i32Volt = i32Volt - ADC2_IDEAL_CODE_100MV;
  i32Volt = i32Volt * ADC2_VOLTAGE_DELTA / ADC2_IDEAL_CODE_DELTA;
  i32Volt = i32Volt + ADC2_VOLTAGE_100MV;

  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);

  /// [AT-PM] : Apply board factor ; 01/25/2013  
  i32Volt = i32Volt - BoardFactor[bProductType].voltageOffset;
  i32Volt = i32Volt * BOARD_FACTOR_CONST/BoardFactor[bProductType].voltageGain;

  /// [RY] : add the factor after code data
  CurMeas->wRawVbat3 = (GWORD)i32Volt;

  /// [AT-PM] : Apply calibration parameter ; 01/25/2013
  i32Volt = i32Volt - GGB->cCalibration.scCaliData.sVbat3Offset;
  i32Volt = i32Volt * CALIBRATION_FACTOR_CONST/ GGB->cCalibration.scCaliData.wVbat3Gain;

  /// [AT-PM] : Get cell voltage ; 06/13/2014
  i32Volt = (i32Volt*3) - CurMeas->wVCell1 - CurMeas->wVCell2;
	
	if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_3)
	{
  	return (GWORD)i32Volt;  
	}
	else
	{
		return (0);
	}
}

/**
 * @brief MeasureExtTemp
 *
 *  External T1 Measurement
 *
 * @return NULL
 *
 */
GSHORT MeasExtTemp(GVOID)
{
  GSHORT tmp16 = 0;
  GINT32 tmp32;
  GBYTE idx;  
  
  GWORD *pRtTable = (GWORD*)&GGB->cCalibration.scTempModel.wRtTable0;
  GBYTE bProductType;  

  GADC->adcCaliCode.wCodeET = ADCCalibrateETCode(GADC->adcRawCode.wCodeET);
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcETCode = ADCFilterExtTemp(GADC->adcCaliCode.wCodeET);
  }
  else
  {
    GADC->wAdcETCode = GADC->adcCaliCode.wCodeET;
  }

  /// [AT-PM] : Get adc code of voltage divider ; 11/01/2013
  tmp16 = (GSHORT) GGB->cCalibration.scConfig.sAdcD3;
  tmp16 = tmp16 - GADC->wAdcETCode;
  
  /// [AT-PM] : Calculate NTC resistance ; 11/01/2013
  tmp32 = (GINT32)GADC->wAdcETCode;

  if(tmp16 != 0)
  {
		if(tmp16 < 0)
		{
			tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ 1;
		}
		else
		{
      tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ tmp16;
		}
  }
	else
  {
    tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ 1;
  }
  
  idx = 0;

  ///----------------------------------------------------///
  /// Query table index with external temperature code
  ///----------------------------------------------------///  
  while(idx < ET_NUMS)
  {
    if(tmp32 >= pRtTable[idx])
    {
      break;
    }
    idx = idx + 1;
  }
  
  if(idx == 0)
  {
    /// [AT-PM] : Minimum measurable temperature ; 01/25/2013
    tmp32 = (GINT32)ExtTemperatureTable[0];
  }
  else if(idx == ET_NUMS)
  {
    /// [AT-PM] : Maximum measurable temperature ; 01/25/2013
    tmp32 = (GINT32)ExtTemperatureTable[ET_NUMS - 1];
  }
  else
  {
    /// [AT-PM] : Calculate external temperature ; 01/25/2013
    tmp32 = tmp32 - pRtTable[idx];
    tmp32 = tmp32 * (ExtTemperatureTable[idx - 1] - ExtTemperatureTable[idx]);
    tmp32 = tmp32 / (pRtTable[idx-1] - pRtTable[idx]);
    tmp32 = tmp32 + ExtTemperatureTable[idx];
  }

  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);


  /// [AT-PM] : Apply board factor ; 01/25/2013
  tmp32 = tmp32 - BoardFactor[bProductType].extTempOffset;

  /// [AT-PM] : Apply calibration factor ; 01/25/2013
  tmp32 = tmp32 - GGB->cCalibration.scConfig.sAdcD4;

  return (GSHORT)tmp32;
}

/**
 * @brief MeasRid
 *
 * RID measurement
 *
 * @return RID 
 */
GWORD MeasRid(GVOID)
{
  GSHORT tmp16;
  GINT32 tmp32;
  
  GADC->adcCaliCode.wCodeRid = ADCCalibrateRidCode(GADC->adcRawCode.wCodeRid);
  if(UpiGauge.wSysMode & SYS_MODE_MEAS_STABLE)
  {
    GADC->wAdcRidCode = ADCFilterRid(GADC->adcCaliCode.wCodeRid);
  }
  else
  {
    GADC->wAdcRidCode = GADC->adcCaliCode.wCodeRid;
  }

  /// [AT-PM] : Get adc code of voltage divider ; 03/24/2015
  tmp16 = (GSHORT) GGB->cCalibration.scConfig.sAdcD3;
  tmp16 = tmp16 - GADC->adcCaliCode.sCodeRidComp - GADC->wAdcRidCode;
  
  /// [AT-PM] : Calculate RID resistance ; 03/24/2015
  tmp32 = (GINT32)GADC->wAdcRidCode;

  if(tmp16 != 0)
  {
		if (tmp16 < 0)
		{
			tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ 1;
		}
		else
		{
      tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ tmp16;
		}
  }
  else
  {
    tmp32 = tmp32 * (GGB->cCalibration.scTempModel.wR)/ 1;
  }
  tmp32 = tmp32 / 1000;
  return (GWORD)tmp32;
}

/**
 * @brief MeasureIntTemp
 *
 *  Internal Temp Measurement
 *
 * @return NULL
 *
 */
GSHORT MeasIntTemp(GVOID)
{
  GSHORT sIntTemp;
  GSHORT ftIT;
  GSHORT sCodeIT;
  GBYTE bProductType;

  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  sCodeIT = (GSHORT)GADC->adcCaliCode.wCodeIT;
  sCodeIT = sCodeIT/2;
  sCodeIT = sCodeIT - IT_OFFSET;
  sCodeIT = sCodeIT * IT_CONST/IT_GAIN;

  /// [AT-PM] : Apply FT information ; 01/25/2013
  ftIT = (GSHORT)ADCCalibrateITCode(GOTP->ftIT);
  ftIT = ftIT/2;
  ftIT = ftIT - IT_OFFSET;
  ftIT = ftIT * IT_CONST/IT_GAIN;
  sIntTemp = sCodeIT - (ftIT - FTAmbientMappingTable[GOTP->deltaET]);
  
  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);
  sIntTemp = sIntTemp - BoardFactor[bProductType].intTempOffset;

  /// [AT-PM] : Apply calibration factor ; 01/25/2013
  sIntTemp = sIntTemp - GGB->cCalibration.scConfig.sAdcD5;
  
  return sIntTemp;
}

/**
 * @brief MeasConvTime
 *
 *   Measurement Conversion Time
 *
 * @return NULL
 *
 */
GVOID MeasConvTime(GVOID)
{
  GSHORT sDeltaCnt;
  
  CurMeas->iDeltaTime =(UpiGauge.tmNow.absTime - UpiGauge.tmPre.absTime);
  UpiMutexLock((GVOID *)&UpiTimeMutex);
  if((CurMeas->iDeltaTime / TIME_MSEC_TO_SEC) < UpiGauge.wSuspendTime)
  {
    CurMeas->iDeltaTime = CurMeas->iDeltaTime + (UpiGauge.wSuspendTime * TIME_MSEC_TO_SEC);
    GLOGE("[%s]: Compensate suspend time = %d\n", __func__, UpiGauge.wSuspendTime);
  }
  UpiGauge.wSuspendTime = 0;
  UpiMutexUnLock((GVOID *)&UpiTimeMutex);
  
  ///-----------------------------------------///
  /// Get ADC conversion count
  ///-----------------------------------------/// 
  CurMeas->iConvCnt = GADC->adcRawCode.wCodeCnt;
  
  ///-----------------------------------------///
  /// DeltaCnt = Current Count - Preious Count
  ///-----------------------------------------/// 
  if(PreMeas->bValid == GTRUE)
  {
    sDeltaCnt = (GSHORT) (CurMeas->iConvCnt - PreMeas->iConvCnt);
  }
  else
  {
    sDeltaCnt = 0;
  }

  ///-----------------------------------------///
  /// Calculate the default conversion time 
  ///-----------------------------------------///
  if(sDeltaCnt ==0)
  {
    CurMeas->iConvTime = PreMeas->iConvTime;
  }
  else
  {
    CurMeas->iConvTime = (CurMeas->iDeltaTime / sDeltaCnt);
  }

  ///-----------------------------------------///
  /// Conversion time protection
  ///-----------------------------------------///
  if(CurMeas->iConvTime <= 0)
  {
    CurMeas->iConvTime = 125; ///< default 125ms
  }
  else if(CurMeas->iConvTime > 135)
  {
    CurMeas->iConvTime = 135; ///< default max conversion time : 135ms    
  }
  else if(CurMeas->iConvTime < 110)
  {
    CurMeas->iConvTime = 110; ///< default min conversion time : 110 ms   
  }

  return;
}

/**
 * @brief MeasCapFilter
 *
 *  Capacity Measurement Filter
 *
 * @return NULL
 *
 */
GSHORT MeasCapFilter(GVOID)
{
  GINT32 sCurCap;
  GINT32 sCurRaw;
  GINT32 sCurPre;

  sCurRaw = (GINT32)(CurMeas->iRawCap);
  sCurPre = (GINT32)(PreMeas->iCurCap);

  if(PreMeas->bValid == GFALSE)
  {
    sCurCap = sCurRaw;
  }
  else
  {
    /// IIR filter
    sCurRaw = sCurRaw * 2;
    sCurPre = sCurPre * 6;
    sCurCap = sCurRaw + sCurPre;
    sCurCap = sCurCap / 8;
  }
  return ((GSHORT)sCurCap);
}
/**
 * @brief MeasDeltaQ
 *
 *  measure deltaQ
 *
 * @return NULL
 *
 */
GSHORT MeasDeltaQ(GVOID)
{
  if((PreMeas->bValid == GFALSE) && (PreMeas->bDeltaQ == GFALSE))
  {
    return 0;
  }
  else
  {
    return (GSHORT)(CurMeas->iCurCap - PreMeas->iCurCap); 
  }
}


/**
 * @brief MeasCapacity
 *
 *  Capacity Measurement Measurement
 *
 * @return NULL
 *
 */
GSHORT MeasCapacity(GVOID)
{
  GINT32 iCode;
  GINT16 sDeltaCode;

  GINT64 i4Volt; 
  GINT32 i2Volt;
  GINT32 i4Capacity; 

  GBYTE bProductType;    
  
  GINT32 iCurr;  
  
  /// Calibrate the capacity code
  GADC->adcCaliCode.sCodeCC  = ADCCalibrateCapCode(GADC->adcRawCode.sCodeCC);

  /// [AT-PM] : Convert from calibrated ADC code ; 01/25/2013
  sDeltaCode = ADC1_IDEAL_CODE_DELTA;
  iCode = (GINT32) GADC->adcCaliCode.sCodeCC;
  iCode = iCode - ADC1_IDEAL_CODE_100MV;
  
  while(1)
  {
    i4Volt = (GINT64)iCode;
    i4Volt = i4Volt * ADC1_VOLTAGE_DELTA;
    if((i4Volt < 2147483647) && (i4Volt > -2147483647))
    {
      i2Volt = (GINT32)i4Volt;
      break;
    }
    sDeltaCode = sDeltaCode/2;
    iCode = iCode/2;
  }
  
  i2Volt = i2Volt/sDeltaCode;
  i2Volt = i2Volt + ADC1_VOLTAGE_100MV;
  iCurr =  (i2Volt/ GGB->cGasGauging.scCurrentThresholds.bRsense);

  /// [AT-PM] : Apply board factor ; 01/25/2013

  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);  
  iCurr = iCurr * BOARD_FACTOR_CONST/BoardFactor[bProductType].currentGain;

  /// [AT-PM] : Apply calibration factor ; 01/25/2013
  if(iCurr > 0)
  {
    iCurr = iCurr * CALIBRATION_FACTOR_CONST / GGB->cCalibration.scCaliData.sAdc1PGain;
  }
  else
  {
    iCurr = iCurr * CALIBRATION_FACTOR_CONST / GGB->cCalibration.scCaliData.sAdc1NGain;
  }

  /// I = I * (4096)
  iCurr = iCurr * (COULOMB_COUNTER_WEIGHT);
  
  /// Q = I(mA) * T (SEC)
  i4Capacity = ((iCurr / TIME_MSEC_TO_SEC) * 125) / TIME_SEC_TO_HOUR;
  return (GSHORT) i4Capacity;
}

/**
 * @brief MeasMaxVCell
 *
 * Get maximum cell voltage
 *
 * @return maximum cell voltage
 */
GWORD MeasMaxVCell(GVOID)
{
  GWORD wMaxVCell;

  wMaxVCell = 0;

  /// [AT-PM] : Check VCell 1 ; 05/15/2014
  if(CurMeas->wVCell1 > wMaxVCell)
  {
    wMaxVCell = CurMeas->wVCell1;
  }

  /// [AT-PM] : Check VCell 2 ; 05/15/2014
  if((CurMeas->wVCell2 != 0) && 
     (CurMeas->wVCell2 > wMaxVCell))
  {
    wMaxVCell = CurMeas->wVCell2;
  }

  if((CurMeas->wVCell3 != 0) &&
     (CurMeas->wVCell3 > wMaxVCell))
  {
    wMaxVCell = CurMeas->wVCell3;
  }

  return (wMaxVCell);
}

/**
 * @brief MeasMinVCell
 *
 * Get minimum cell voltage
 *
 * @return minimum cell voltage
 */
GWORD MeasMinVCell(GVOID)
{
  GWORD wMinVCell;

  wMinVCell = 5000;

  /// [AT-PM] : Check VCell 1 ; 05/15/2014
  if(CurMeas->wVCell1 < wMinVCell)
  {
    wMinVCell = CurMeas->wVCell1;
  }

  /// [AT-PM] : Check VCell 2 ; 05/15/2014
  if((CurMeas->wVCell2 != 0) && 
     (CurMeas->wVCell2 < wMinVCell))
  {
    wMinVCell = CurMeas->wVCell2;
  }

  if((CurMeas->wVCell3 != 0) &&
     (CurMeas->wVCell3 < wMinVCell))
  {
    wMinVCell = CurMeas->wVCell3;
  }

  return (wMinVCell);
}

/**
 * @brief MeasRevertVCell1
 *
 * Revert VCell1 ADC code from voltage
 *
 * @para wVolt target voltage
 * @return VCell1 ADC code
 */
GWORD MeasRevertVCell1(GWORD wVolt)
{
  GBYTE bProductType;
  GINT32 int32AdcCode;
 
	GWORD  wAdcCode;

  /// [YL] : Apply calibration parameter revert adc code ; 20150508
  int32AdcCode = wVolt * GGB->cCalibration.scCaliData.sAdc2Gain;
  int32AdcCode = int32AdcCode / CALIBRATION_FACTOR_CONST;
  int32AdcCode = int32AdcCode + GGB->cCalibration.scCaliData.sAdc2Offset; 
  
  ///-------------------------------------------------///
  /// Get the product type
  ///   UG3105 --> (0<<6)
  ///   UG310X --> (1<<6)
  ///   RSVD2  --> (2<<6)
  ///   RSVD3  --> (3<<6)
  ///-------------------------------------------------///  
  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);

  /// [YL] : Apply board factor ; 20150508
  int32AdcCode= int32AdcCode * BoardFactor[bProductType].voltageGain; 
  int32AdcCode = int32AdcCode / BOARD_FACTOR_CONST;
  int32AdcCode = int32AdcCode + BoardFactor[bProductType].voltageOffset;

  /// [YL] : Revert calibrated ADC code ;2015/05/08
	int32AdcCode = int32AdcCode - ADC2_VOLTAGE_100MV;  
  int32AdcCode = int32AdcCode * ADC2_IDEAL_CODE_DELTA;
  int32AdcCode = int32AdcCode / ADC2_VOLTAGE_DELTA;
  int32AdcCode = int32AdcCode + ADC2_IDEAL_CODE_100MV;
  
  /// [YL] : Revert rawcode ; 2015/05/08
	wAdcCode = ADC2RevertCalibrateCode((GWORD)int32AdcCode);
	
  return (wAdcCode);
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus



