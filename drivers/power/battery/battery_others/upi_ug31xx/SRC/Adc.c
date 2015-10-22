/**
 * @file Adc.c
 *
 *  Adc Generic Function
 *
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#include "UpiDef.h"

///---------------------------------------------------------------------------///
/// ADC Queue
///---------------------------------------------------------------------------///
static const GBYTE Adc1QueueInitVal[ADC1_QUEUE_SIZE] = 
{
  (     SET_A_ET |      SET_B_ET |      SET_C_ET |  SET_D_RID_IN),   ///< 1st Conversion
  ( SET_E_RID_IN |  SET_F_RID_IN | SET_G_CURRENT | SET_H_CURRENT),   ///< 2nd Conversion
  (SET_I_CURRENT |      SET_J_IT |      SET_K_IT |      SET_L_IT),   ///< 3rd Conversion
  (SET_M_CURRENT | SET_N_CURRENT | SET_O_CURRENT | SET_P_CURRENT)    ///< 4th Conversion
};


///---------------------------------------------------------------------------///
/// ADC Queue
///---------------------------------------------------------------------------///
static const GBYTE Adc1QueueVal[ADC1_QUEUE_SIZE] = 
{
  (     SET_A_IT |      SET_B_IT | SET_C_CURRENT | SET_D_CURRENT),   ///< 1st Conversion
  (     SET_E_ET |      SET_F_ET | SET_G_CURRENT | SET_H_CURRENT),   ///< 2nd Conversion
  ( SET_I_RID_IN |  SET_J_RID_IN | SET_K_CURRENT | SET_L_CURRENT),   ///< 3rd Conversion
  (SET_M_CURRENT | SET_N_CURRENT | SET_O_CURRENT | SET_P_CURRENT)    ///< 4th Conversion
};

static const GBYTE Adc2QueueVal[ADC2_QUEUE_SIZE * MAX_CELL_NUM] = 
{
  ///-------------------------------------------------------------------------------///
  ///   1 CELL Definition
  ///-------------------------------------------------------------------------------/// 
  (SET_V1_VBAT1 |  SET_V2_VBAT1 |  SET_V3_VBAT1 |  SET_V4_VBAT1),   ///< 1st Conversion
  (SET_V5_VBAT1 |  SET_V6_VBAT1 |  SET_V7_VBAT1 |  SET_V8_VBAT1),   ///< 2nd Conversion
  (SET_V9_VBAT1 | SET_V10_VBAT1 | SET_V11_VBAT1 | SET_V12_VBAT1),   ///< 3rd Conversion
  ///-------------------------------------------------------------------------------/// 
  ///   2 CELL Definition
  ///-------------------------------------------------------------------------------/// 
  (SET_V1_VBAT1 |  SET_V2_VBAT1 |  SET_V3_VBAT2 |  SET_V4_VBAT2),   ///< 1st Conversion
  (SET_V5_VBAT1 |  SET_V6_VBAT1 |  SET_V7_VBAT2 |  SET_V8_VBAT2),   ///< 2nd Conversion
  (SET_V9_VBAT1 | SET_V10_VBAT1 | SET_V11_VBAT2 | SET_V12_VBAT2),   ///< 3rd Conversion
  ///-------------------------------------------------------------------------------/// 
  ///   3 CELL Definition
  ///-------------------------------------------------------------------------------///   
  (SET_V1_VBAT1 |  SET_V2_VBAT1 |  SET_V3_VBAT2 |  SET_V4_VBAT2),   ///< 1st Conversion
  (SET_V5_VBAT3 |  SET_V6_VBAT3 |  SET_V7_VBAT1 |  SET_V8_VBAT1),   ///< 2nd Conversion
  (SET_V9_VBAT2 | SET_V10_VBAT2 | SET_V11_VBAT3 | SET_V12_VBAT3)    ///< 3rd Conversion
};


static AdcDeltaCodeMappingType AdcDeltaCodeMapping[] =
{
///------------------------------------------------------------///
///   ADC1     ADC1   ADC2  ADC2  
///   V100,    V200,  V100, V200
///------------------------------------------------------------/// 
  { -12800, -25600, 1536,  0    },     ///< Index = 0
  { -12544, -25088, 1600,  128  },     ///< Index = 1
  { -13056, -26112, 1472, -128  },     ///< Index = 2
  { -12288, -24576, 1664,  256  },     ///< Index = 3
  { -13312, -26624, 1408, -256  },     ///< Index = 4
  { -12032, -24064, 1728,  384  },     ///< Index = 5
  { -13568, -27136, 1344, -384  },     ///< Index = 6
  { -11776, -23552, 1792,  512  },     ///< Index = 7
  { -13824, -27648, 1280, -512  },     ///< Index = 8
  { -11520, -23040, 1856,  640  },     ///< Index = 9
  { -14080, -28160, 1216, -640  },     ///< Index = 10
  { -11264, -22528, 1920,  768  },     ///< Index = 11
  { -14336, -28672, 1152, -768  },     ///< Index = 12
  { -11008, -22016, 1984,  896  },     ///< Index = 13
  { -14592, -29184, 1088, -896  },     ///< Index = 14
  { -10752, -21504, 2048,  1024 },     ///< Index = 15
  { -14848, -29696, 1024, -1024 },     ///< Index = 16
  { -10496, -20992, 2112,  1152 },     ///< Index = 17
  { -15104, -30208, 960,  -1152 },     ///< Index = 18
  { -10240, -20480, 2176,  1280 },     ///< Index = 19
  { -15360, -30720, 896,  -1280 },     ///< Index = 20
  { -9984,  -19968, 2240,  1408 },     ///< Index = 21
  { -15616, -31232, 832,  -1408 },     ///< Index = 22
  { -9728,  -19456, 2304,  1536 },     ///< Index = 23
  { -15872, -31744, 768,  -1536 },     ///< Index = 24
  { -9472,  -18944, 2368,  1664 },     ///< Index = 25
  { -16128, -32256, 704,  -1664 },     ///< Index = 26
  { -9216,  -18432, 2432,  1792 },     ///< Index = 27
  { -16384, -32768, 640,  -1792 },     ///< Index = 28
  { -8960,  -17920, 2496,  1920 },     ///< Index = 29
  { -16640, -33280, 576,  -1920 },     ///< Index = 30
  { 0,      0,       0,    0    },      ///< Index = 31
};

/**
 * @brief ADC1QueueSet
 *
 *  Set ADC1 Queue
 *
 * @return NULL
 *
 */
STATIC GVOID ADC1QueueSet(GVOID)
{
  WRITE_REG(REG_ADC_CTR_A, Adc1QueueVal[0]);
  WRITE_REG(REG_ADC_CTR_B, Adc1QueueVal[1]);
  WRITE_REG(REG_ADC_CTR_C, Adc1QueueVal[2]);
  WRITE_REG(REG_ADC_CTR_D, Adc1QueueVal[3]);  
}

/**
 * @brief ADC1QueueSet
 *
 *  Set ADC1 Queue
 *
 * @return NULL
 *
 */
STATIC GVOID ADC1QueueInitSet(GVOID)
{
  WRITE_REG(REG_ADC_CTR_A, Adc1QueueInitVal[0]);
  WRITE_REG(REG_ADC_CTR_B, Adc1QueueInitVal[1]);
  WRITE_REG(REG_ADC_CTR_C, Adc1QueueInitVal[2]);
  WRITE_REG(REG_ADC_CTR_D, Adc1QueueInitVal[3]);  
}

/**
 * @brief ADC2QueueSet
 *
 *  Set ADC2 Queue
 *
 * @return NULL
 *
 */
GVOID ADC2QueueSet(GVOID)
{
  GBYTE bCellNum = GGB->cConfiguration.scAFE.bCellNumber;
  
  WRITE_REG(REG_ADC_V1, Adc2QueueVal[(bCellNum-1)*ADC2_QUEUE_SIZE + 0]);
  WRITE_REG(REG_ADC_V2, Adc2QueueVal[(bCellNum-1)*ADC2_QUEUE_SIZE + 1]);
  WRITE_REG(REG_ADC_V3, Adc2QueueVal[(bCellNum-1)*ADC2_QUEUE_SIZE + 2]);    
}

/**
 * @brief DecimateRst
 *
 *  Decimate reset filter of ADC
 *
 * @return  _UPI_NULL_
 */
GVOID DecimateRst(GVOID)
{
  SET_BIT(REG_ALARM_EN, ALARM_EN_DECIMATE_RST);
  CLR_BIT(REG_ALARM_EN, ALARM_EN_DECIMATE_RST);   
  SET_BIT(REG_ALARM_EN, ALARM_EN_DECIMATE_RST);
  GLOGE("[%s]: DECIMATE_RST finished at %04d/%02d/%02d %02d:%02d:%02d\n", __func__,
        UpiGauge.tmNow.year, UpiGauge.tmNow.mon, UpiGauge.tmNow.day,
        UpiGauge.tmNow.hour, UpiGauge.tmNow.min, UpiGauge.tmNow.sec);
}


/**
 * @brief ADCChopperSet
 *
 *  Set ADC Chopper
 *
 * @return NULL
 *
 */
GVOID ADCChopperSet(GVOID)
{
  WRITE_REG(REG_FW_CTRL, GGB->cConfiguration.scAFE.bChopCtrl);
}

/**
 * @brief ADCEnable
 *
 *  Enable ADC
 *
 * @return NULL
 *
 */
STATIC GVOID ADCEnable(GVOID)
{
  GBYTE bReg;

  bReg = READ_REG(REG_INTR_CTRL_A);
  bReg |= (INTR_CTRL_A_ADC2_EN | INTR_CTRL_A_ADC1_EN);
  WRITE_REG(REG_INTR_CTRL_A, bReg);
}

/**
 * @brief ADCDisable
 *
 * Disable ADC
 *
 * @return NULL
 */
STATIC GVOID ADCDisable(GVOID)
{
  GBYTE bReg;

  bReg = READ_REG(REG_INTR_CTRL_A);
  bReg = bReg & (~(INTR_CTRL_A_ADC2_EN | INTR_CTRL_A_ADC1_EN));
  WRITE_REG(REG_INTR_CTRL_A, bReg);
}

/**
 * @brief ADCUpdateQueue1 
 *
 * Update ADC1 queue
 *
 * @return NULL
 */
GVOID ADCUpdateQueue1(GVOID)
{
  /// [AT-PM] : Disable ADC1 and ADC2 ; 08/13/2015
  ADCDisable();

  /// [AT-PM] : Update new ADC1 queue ; 08/13/2015
  ADC1QueueSet();

  /// [AT-PM] : Enable ADC1 and ADC2 ; 08/13/2015
  ADCEnable();

  /// [AT-PM] : Decimate reset ; 08/13/2015
  DecimateRst();
}

/**
 * @brief ADCConvStatus
 *
 *  ADC Fetch Code
 *
 * @return  _UPI_NULL_
 */
GVOID ADCConvStatus(GVOID)
{
  GBYTE bReg;
  GBYTE bRegCtrl1 = READ_REG(REG_CTRL1);

  bReg  = UpiGauge.bRegIntrStatus;
  GLOGD("===========================================\n"); 

  ///---------------------------------------------///
  /// 1. Read VBat1Ave
  ///---------------------------------------------///
  if(bRegCtrl1 & CTRL1_VTM_EOC)
  {
		/// [FC] : Check coltage code for ADC hang-up issue ; 06/20/2014
		if(GADC->bLastVTMFlag == GTRUE)
		{
			GADC->bVTMStopCnt++;
		}
		else
		{
			GADC->bVTMStopCnt = 0;
		}
    SBS->wAdcEOC |= SBS_ADC_EOC_VTM;
		GADC->bLastVTMFlag = GTRUE;
    GLOGD("VOLT FLAG = 1\n"); 
  }
  else
  {
    SBS->wAdcEOC &= ~SBS_ADC_EOC_VTM;
		GADC->bLastVTMFlag = GFALSE;
    GLOGD("VOLT FLAG = 0\n"); 
  }
  ///---------------------------------------------///
  /// 2. Read Current 
  ///---------------------------------------------/// 
  if(bRegCtrl1 & CTRL1_GG_EOC)
  {
    SBS->wAdcEOC |= SBS_ADC_EOC_GG;
    GLOGD("CURR FLAG = 1\n"); 
  }
  else
  {
    SBS->wAdcEOC &= ~SBS_ADC_EOC_GG;  
    GLOGD("CURR FLAG = 0\n"); 
  }
  
  ///---------------------------------------------///
  /// 3. Read Int. Temp 
  ///---------------------------------------------/// 
  if(bReg & INTR_STATUS_IT_STS)
  {
    SBS->wAdcEOC |= SBS_ADC_EOC_IT; 
    GLOGD("IT   FLAG = 1\n"); 
  }
  else
  {
    SBS->wAdcEOC &= ~SBS_ADC_EOC_IT;    
    GLOGD("IT   FLAG = 0\n"); 
  }
  
  ///---------------------------------------------///
  /// 4. Read Ext. Temp 
  ///---------------------------------------------///
  if(bReg & INTR_STATUS_ET_STS)
  {
    SBS->wAdcEOC |= SBS_ADC_EOC_ET;   
    GLOGD("ET   FLAG = 1\n"); 
  }
  else
  {
    SBS->wAdcEOC &= ~SBS_ADC_EOC_ET;      
    GLOGD("ET   FLAG = 0\n"); 
  }

  ///---------------------------------------------///
  /// 5. Read Coulomb Charger
  ///---------------------------------------------///

  ///---------------------------------------------///
  /// 6. Read Coulomb Charger
  ///---------------------------------------------/// 

  ///---------------------------------------------///
  /// 7. Read Counter
  ///---------------------------------------------/// 


  GLOGD("INTR_MODE = %02X\n", bReg);
  if(!(bReg & INTR_STATUS_LVD_STS))
  {
    SYSInit();
  }

}

/**
 * @brief ADCFetchCode
 *
 *  ADC Fetch Code
 *
 * @return  _UPI_NULL_
 */
GVOID ADCFetchCode(GVOID)
{ 
  GWORD wCode;
  GWORD wHighTrd;
  GWORD wLowTrd;
  GDWORD dwTemp;

  ///---------------------------------------------///
  /// 1. Read VBat1Ave
  ///---------------------------------------------///
  GADC->adcRawCode.wCodeVBat1 = READ_REG16(REG16_AVE_VBAT1);
  if(GADC->adcRawCode.wCodeVBat1 > VOLT_THRESHOD_LOWER && 
    GADC->adcRawCode.wCodeVBat1 < VOLT_THRESHOD_HIGHER)
  {
    SBS->wCodeVBat1 = GADC->adcRawCode.wCodeVBat1;
  }
  else
  {
    GADC->adcRawCode.wCodeVBat1 = SBS->wCodeVBat1;
  }
  if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_2)
  { 
    GADC->adcRawCode.wCodeVBat2 = READ_REG16(REG16_VBAT2_AVE);
    if(GADC->adcRawCode.wCodeVBat1 > VOLT_THRESHOD_LOWER && 
      GADC->adcRawCode.wCodeVBat1 < VOLT_THRESHOD_HIGHER)
    {
      SBS->wCodeVBat2 = GADC->adcRawCode.wCodeVBat2;
    }
    else
    {
      GADC->adcRawCode.wCodeVBat2 = SBS->wCodeVBat2;
    }
  }
  
	if(GGB->cConfiguration.scAFE.bCellNumber >= CELL_NUM_3)
  { 
    GADC->adcRawCode.wCodeVBat3 = READ_REG16(REG16_VBAT3_AVE);
    if(GADC->adcRawCode.wCodeVBat1 > VOLT_THRESHOD_LOWER && 
      GADC->adcRawCode.wCodeVBat1 < VOLT_THRESHOD_HIGHER)
    {
  	SBS->wCodeVBat3 = GADC->adcRawCode.wCodeVBat3;
  }
    else
    {
      GADC->adcRawCode.wCodeVBat3 = SBS->wCodeVBat3;
    }
  }

  ///---------------------------------------------///
  /// 2. Read Current 
  ///---------------------------------------------///   
  wCode = READ_REG16(REG16_AVE_CURRENT);  
  SBS->wCodeCurr = wCode ;  
  GADC->adcRawCode.sCodeCurr = *(GSHORT *) &wCode;

  wCode = READ_REG16(REG16_CURRENT);
  GADC->adcRawCode.sCodeCurrInst = *(GSHORT *) &wCode;

  ///---------------------------------------------///
  /// 3. Read Int. Temp 
  ///---------------------------------------------/// 
  GADC->adcRawCode.wCodeIT = READ_REG16(REG16_AVE_IT); 
  dwTemp = (GDWORD)GOTP->aveIT80 * 3 / 2;
  wHighTrd = (GWORD)dwTemp; 
  dwTemp = (GDWORD)GOTP->aveIT25 * 3 / 10; 
  wLowTrd = (GWORD)dwTemp;
  if(GADC->adcRawCode.wCodeIT > wLowTrd && 
  GADC->adcRawCode.wCodeIT < wHighTrd)
  {
  SBS->wCodeIT  = GADC->adcRawCode.wCodeIT;  
  }
  else
  {
    GADC->adcRawCode.wCodeIT = SBS->wCodeIT; 
  }
  ///---------------------------------------------///
  /// 4. Read Ext. Temp 
  ///---------------------------------------------/// 
  GADC->adcRawCode.wCodeET = READ_REG16(REG16_AVE_ET);   
  SBS->wCodeET  = GADC->adcRawCode.wCodeET;

  ///---------------------------------------------///
  /// 5. Read Coulomb Charger
  ///---------------------------------------------/// 
  wCode =READ_REG16(REG16_CHARGE);
  GADC->adcRawCode.sCodeCC = *(GSHORT *) &wCode; 
  SBS->wCodeCC  = GADC->adcRawCode.sCodeCC ;

  ///---------------------------------------------///
  /// 6. Read Counter
  ///---------------------------------------------/// 
  GADC->adcRawCode.wCodeCnt = READ_REG16(REG16_COUNTER);   
  SBS->wCodeCnt  = GADC->adcRawCode.wCodeCnt;  

  ///---------------------------------------------///
  /// 7. Read ADC1 offset
  ///---------------------------------------------/// 
  wCode =READ_REG16(REG16_ADC1_OFFSET);
  GADC->adcRawCode.sCodeOffset = *(GSHORT *) &wCode;
  SBS->sCodeOffset= GADC->adcRawCode.sCodeOffset;  

  ///---------------------------------------------///
  /// 8. Read RID code 
  ///---------------------------------------------/// 
  GADC->adcRawCode.wCodeRid =READ_REG16(REG16_AVE_RID);
  SBS->wCodeRid = GADC->adcRawCode.sCodeOffset;  
}

/**
 * @brief ADC1ConvertData
 *
 *  Calculate ADC1's ADC Code from OTP
 *
 * @return NULL
 *
 */
GVOID ADC1ConvertData(GVOID)
{
  GWORD  wDeltaCode;
  GINT32 dwDeltaCode;
  GBYTE  bAdcCodeIndex;

  ///----------------------------------------------------------------///
  ///
  /// 1. Get ADC1_CODE_T25_100MV
  ///
  ///----------------------------------------------------------------///
  wDeltaCode = GOTP->adc1DeltaCodeT25V100;

  ///---------------------------------------///
  /// Check signed bit
  ///---------------------------------------///
  if(wDeltaCode & ADC1_DELTA_CODE_25_100MV_SIGN_BIT)
  {
    wDeltaCode = wDeltaCode & (~ADC1_DELTA_CODE_25_100MV_SIGN_BIT);
    if(wDeltaCode != 0)
    {
      wDeltaCode = wDeltaCode + ADC1_CODE_100MV_NEGATIVE;
    }
  }
  wDeltaCode = wDeltaCode + ADC1_CP_CODE_25_100MV;
  dwDeltaCode = (GINT32)(GINT16)wDeltaCode;

  ///---------------------------------------///
  /// Get index and map the real code value  
  /// from the mapping table
  ///---------------------------------------///
  bAdcCodeIndex = GOTP->indexAdc1V100T25;  
  dwDeltaCode = dwDeltaCode + AdcDeltaCodeMapping[bAdcCodeIndex].Adc1V100;

  ///---------------------------------------///
  /// Assign the Adc1CodeT25V100mV value
  ///---------------------------------------///  
  GADC->adc1CodeT25V100 = (GINT16)dwDeltaCode;

  ///----------------------------------------------------------------///
  ///
  /// 2. Get ADC1_CODE_T25_200MV
  ///
  ///----------------------------------------------------------------///
  dwDeltaCode = GOTP->adc1DeltaCodeT25V200;

  ///---------------------------------------///
  /// Check signed bit
  ///---------------------------------------///  
  if(dwDeltaCode & ADC1_DELTA_CODE_25_200MV_SIGN_BIT)
  {
    dwDeltaCode = dwDeltaCode & (~ADC1_DELTA_CODE_25_200MV_SIGN_BIT);
    if(dwDeltaCode != 0)
    {
      dwDeltaCode = dwDeltaCode + ADC1_CODE_200MV_NEGATIVE;
    }
  }
  dwDeltaCode = dwDeltaCode + ADC1_CP_CODE_25_200MV;
  dwDeltaCode = (GINT32)(GINT16)dwDeltaCode;

  ///---------------------------------------///
  /// Get index and map the real code value  
  /// from the mapping table
  ///---------------------------------------///    
  bAdcCodeIndex = GOTP->indexAdc1V200T25;     
  dwDeltaCode = dwDeltaCode + AdcDeltaCodeMapping[bAdcCodeIndex].Adc1V200;

  ///---------------------------------------///
  /// Assign the Adc1CodeT25V100mV value
  ///---------------------------------------///
  GADC->adc1CodeT25V200 = (GINT16)dwDeltaCode;

  ///----------------------------------------------------------------///
  ///
  /// 3. Get Code for ADC1_CODE_T80_100mV and ADC1_CODE_T80_100mV
  ///
  ///----------------------------------------------------------------///

  ///  DELTA_TEMP_CODE = (AVG_IT_80_CODE - AVG_IT_25_CODE) * 1000
  dwDeltaCode = (GOTP->aveIT80 - GOTP->aveIT25) * ADC_TEMPERATURE_GAIN_CONST;

  /// [AT-PM] : Get code T80 100mV ; 01/23/2013
  GADC->adc1CodeT80V100 = (GWORD)(dwDeltaCode/ADC1_TEMPERATURE_GAIN_100MV + GADC->adc1CodeT25V100);
  
  
  /// [AT-PM] : Get code T80 200mV ; 01/23/2013
  GADC->adc1CodeT80V200 = (GWORD)(dwDeltaCode/ADC1_TEMPERATURE_GAIN_200MV + GADC->adc1CodeT25V200);

  ///----------------------------------------------------------------///
  ///
  /// 4. Get temperature compensation for ADC1 offset
  ///
  ///----------------------------------------------------------------///
  wDeltaCode = GOTP->adcDelta;
  if(wDeltaCode & ADC1_TEMPERATURE_OFFSET_SIGN_BIT)
  {
    wDeltaCode = wDeltaCode & (~ADC1_TEMPERATURE_OFFSET_SIGN_BIT);
    if(wDeltaCode != 0)
    {
      wDeltaCode = wDeltaCode + ADC1_TEMPERATURE_OFFSET_NEGATIVE;
    }
  }
  GADC->adc1TempOffset = (GINT16)wDeltaCode;
}

/**
 * @brief ADC1FactorCal
 *
 *  Calculate ADC1's Gain and Offset
 *
 * @return NULL
 *
 */
GVOID ADC1FactorCal(GVOID)
{
  GINT32 iDeltaVoltCode25;
  GINT32 iDeltaVoltCode80;

  /// [AT-PM] : Calculate gain slope and factor B ; 01/23/2013
  iDeltaVoltCode25 = (GINT32)GADC->adc1CodeT25V200 - GADC->adc1CodeT25V100;
  iDeltaVoltCode80 = (GINT32)GADC->adc1CodeT80V200 - GADC->adc1CodeT80V100;

  GADC->adc1GainSlope   = iDeltaVoltCode80 - iDeltaVoltCode25;
  GADC->adc1GainFactorB = iDeltaVoltCode25*(GOTP->aveIT80) - iDeltaVoltCode80*(GOTP->aveIT25);

  /// [AT-PM] : Calculate offset slope and factor O ; 01/23/2013
  iDeltaVoltCode25 = (GINT32)(GADC->adc1CodeT25V100*2 - GADC->adc1CodeT25V200);
  iDeltaVoltCode80 = (GINT32)(GADC->adc1CodeT80V100*2 - GADC->adc1CodeT80V200);

  GADC->adc1OffsetSlope   = iDeltaVoltCode80 - iDeltaVoltCode25;
  GADC->adc1OffsetFactorO = iDeltaVoltCode25*(GOTP->aveIT80) - iDeltaVoltCode80*(GOTP->aveIT25);
}

/**
 * @brief ADC1FactorGet
 *
 *  Get instant gain and offset for ADC1
 *
 * @return NULL
 *
 */
GVOID ADC1FactorGet(GWORD wCodeIntTemp)
{
  GINT64 i64Gain;
  GINT64 i64Offset;  
  GINT32 i32TempComp;

  /// [AT-PM] : Calculate current ADC1 gain ; 01/23/2013
  i64Gain = (GINT64)GADC->adc1GainSlope;
  i64Gain = i64Gain * (wCodeIntTemp) + GADC->adc1GainFactorB;
  GADC->adc1Gain = (GINT32)i64Gain;

  /// [AT-PM] : Calculate current ADC1 offset ; 01/23/2013
  i64Offset = (GINT64)GADC->adc1OffsetSlope;
  i64Offset = i64Offset * (wCodeIntTemp) + GADC->adc1OffsetFactorO;

  /// [AT-PM] : Temperature compensation ; 09/16/2014
  if(GOTP->productType == OTP_PRODUCT_TYPE_1)
  {
    i32TempComp = (GINT32)wCodeIntTemp;
    i32TempComp = (i32TempComp - GOTP->aveIT25) * (GADC->adc1TempOffset);
    i32TempComp = i32TempComp / (GOTP->aveIT80 - GOTP->aveIT25);
  }
  else
  {
    i32TempComp = 0;
  }

  i64Offset = i64Offset + i32TempComp;
  GADC->adc1Offset = (GINT32)i64Offset;
}

/**
 * @brief ADCFilterReset
 *
 *  Reset Filter
 *
 * @return  GVOID
 */
GVOID ADCFilterInit(GVOID)
{
  GADC->bAdcVCell1CodeCnt  = 0;
  GADC->dwAdcVCell1CodeSum = 0;

  GADC->bAdcVCell2CodeCnt  = 0;
  GADC->dwAdcVCell2CodeSum = 0;

  GADC->bAdcVCell3CodeCnt  = 0;
  GADC->dwAdcVCell3CodeSum = 0;

  GADC->bAdcITCodeCnt = 0;
  GADC->dwAdcITCodeSum = 0;  

  GADC->bAdcETCodeCnt = 0;
  GADC->dwAdcETCodeSum = 0;

  GADC->bAdcRidCodeCnt = 0;
  GADC->dwAdcRidCodeSum = 0;

  GADC->bAdcCurrCodeCnt = 0;
  GADC->dwAdcCurrCodeSum = 0;
}

/**
 * @brief ADCFilterVCell
 *
 *  VCell code fileter 
 *
 * @para bCell ADC2_VCELL1, ADC2_VCELL2, or ADC2_VCELL3
 * @para wCode adc code
 * @return  filtered code
 */
GWORD ADCFilterVCell(GBYTE bCell, GWORD wRawCode)
{
  GINT32 i32Code;
  GINT32 dwCodeSum;
  GBYTE bCodeCnt;
  GWORD wCode;

  if(bCell == ADC2_VCELL1)
  {
    dwCodeSum = GADC->dwAdcVCell1CodeSum;
    bCodeCnt  = GADC->bAdcVCell1CodeCnt;
    wCode     = GADC->wAdcVCell1Code;
  }
  else if(bCell == ADC2_VCELL2)
  {
    dwCodeSum = GADC->dwAdcVCell2CodeSum;
    bCodeCnt  = GADC->bAdcVCell2CodeCnt;
    wCode     = GADC->wAdcVCell2Code;
  }
  else if(bCell == ADC2_VCELL3)
  {
    dwCodeSum = GADC->dwAdcVCell3CodeSum;
    bCodeCnt  = GADC->bAdcVCell3CodeCnt;
    wCode     = GADC->wAdcVCell3Code;
  }
  else
  {
    return (wRawCode);
  }

  if(bCodeCnt < 5)
  {
    bCodeCnt ++;
    dwCodeSum = dwCodeSum + wRawCode;
    i32Code = (GINT32)(dwCodeSum / bCodeCnt);
  }
  else
  {
    i32Code = ((wCode * BYTE1(SBS->wVoltIIRRatio)) + (wRawCode * BYTE0(SBS->wVoltIIRRatio))) / 100;
  }

  if(bCell == ADC2_VCELL1)
  {
    GADC->dwAdcVCell1CodeSum = dwCodeSum;
    GADC->bAdcVCell1CodeCnt  = bCodeCnt;
    GADC->wAdcVCell1Code     = wCode;
  }
  else if(bCell == ADC2_VCELL2)
  {
    GADC->dwAdcVCell2CodeSum = dwCodeSum;
    GADC->bAdcVCell2CodeCnt  = bCodeCnt;
    GADC->wAdcVCell2Code     = wCode;
  }
  else if(bCell == ADC2_VCELL3)
  {
    GADC->dwAdcVCell3CodeSum = dwCodeSum;
    GADC->bAdcVCell3CodeCnt  = bCodeCnt;
    GADC->wAdcVCell3Code     = wCode;
  }
  else
  {
    return (wRawCode);
  }
  return ((GWORD) i32Code);
}

/**
 * @brief ADCFilterCurr
 *
 * Current filter
 *
 * @para sCurrCode current code
 * @return filtered current code
 */
GSHORT ADCFilterCurr(GSHORT sCurrCode)
{
  GINT32 i32Code;
  GSHORT sDeltaCode1;
  GSHORT sDeltaCode2;

  if(GADC->bAdcCurrCodeCnt < 5)
  {
    /// [AT-PM] : Get the average current code at initial stage ; 08/03/2015
    GADC->bAdcCurrCodeCnt = GADC->bAdcCurrCodeCnt + 1;
    GADC->dwAdcCurrCodeSum = GADC->dwAdcCurrCodeSum + sCurrCode;
    i32Code = GADC->dwAdcCurrCodeSum / GADC->bAdcCurrCodeCnt;
    GLOGD("[%s]: CNT = %d, SUM = %lld, RAW = %d, FILTER = %d\n", __func__, 
          GADC->bAdcCurrCodeCnt, 
          GADC->dwAdcCurrCodeSum, 
          sCurrCode, 
          i32Code);
  }
  else
  {
    /// [AT-PM] : Get the delta code for IIR filter ; 08/03/2015
    if(sCurrCode < GADC->sAdcCurrCode)
    {
      sDeltaCode1 = GADC->sAdcCurrCode - sCurrCode;
    }
    else
    {
      sDeltaCode1 = sCurrCode - GADC->sAdcCurrCode;
    }
    
    ///< [YL] : Get current Stable delta code ; 2015/08/28  
    if(CurMeas->sCurrRawCode < PreMeas->sCurrRawCode)
    {
      sDeltaCode2 = PreMeas->sCurrRawCode - CurMeas->sCurrRawCode;
    }
    else
    {
      sDeltaCode2 = CurMeas->sCurrRawCode - PreMeas->sCurrRawCode;
    }
    
    GLOGD("[%s]: DELTA CODE = | %d - %d | = %d, STABLE DELTA CODE = | %d - %d | = %d\n", __func__,
          GADC->sAdcCurrCode,
          sCurrCode,
          sDeltaCode1,
          CurMeas->sCurrRawCode,
          PreMeas->sCurrRawCode,
          sDeltaCode2);

    
    if((sDeltaCode1 > ADC_FILTER_CURR_THRD) && (sDeltaCode2 > ADC_FILTER_CURR_THRD))   
    {
      /// [AT-PM] : Use delta code as IIR filter base ; 08/03/2015
      sDeltaCode1 = sDeltaCode1 >> 1;

      i32Code = (GINT32)GADC->sAdcCurrCode;
      i32Code = i32Code * sDeltaCode1;
      i32Code = i32Code + sCurrCode;
      i32Code = i32Code / (sDeltaCode1 + 1);
      
    }
    else
    {
      /// [AT-PM] : No IIR filter ; 08/03/2015
      i32Code = (GINT32)sCurrCode;
    }
    GLOGD("[%s]: FILTER = %d\n", __func__, i32Code);
  }
  return ((GSHORT)i32Code);
}

/**
 * @brief ADCFilterIntTemp
 *
 *  Calibrate  IT Code 
 *
 * @return  calibrated code
 */
GWORD ADCFilterIntTemp(GWORD wITcode)
{
  GINT32 i32Code;

  if(GADC->bAdcITCodeCnt < 5)
  {
    GADC->bAdcITCodeCnt++;
    GADC->dwAdcITCodeSum = GADC->dwAdcITCodeSum + wITcode;
    i32Code = (GINT32) (GADC->dwAdcITCodeSum / (GADC->bAdcITCodeCnt));
    return (GWORD) i32Code;
  }

  i32Code = (GADC->wAdcITCode * 60 + wITcode * 40) / 100;

  return (GWORD) i32Code;
}

/**
 * @brief ADCFilterExtTemp
 *
 *  Calibrate  External Temperature Code 
 *
 * @return  calibrated code
 */
GWORD ADCFilterExtTemp(GWORD wETcode)
{
  GINT32 i32Code;

  if(GADC->bAdcETCodeCnt < 5)
  {
    GADC->bAdcETCodeCnt++;
    GADC->dwAdcETCodeSum = GADC->dwAdcETCodeSum + wETcode;
    i32Code = (GINT32) (GADC->dwAdcETCodeSum / (GADC->bAdcETCodeCnt));
    return (GWORD) i32Code;
  }

  i32Code = (GADC->wAdcETCode * 60 + wETcode * 40) / 100;

  return (GWORD) i32Code;
}

/**
 * @brief ADCFilterRid
 *
 *  Calibrate RID codd 
 *
 * @return  calibrated code
 */
GWORD ADCFilterRid(GWORD wRidcode)
{
  GINT32 i32Code;

  if(GADC->bAdcRidCodeCnt < 5)
  {
    GADC->bAdcRidCodeCnt++;
    GADC->dwAdcRidCodeSum = GADC->dwAdcRidCodeSum + wRidcode;
    i32Code = (GINT32) (GADC->dwAdcRidCodeSum / (GADC->bAdcRidCodeCnt));
    return (GWORD) i32Code;
  }

  i32Code = (GADC->wAdcRidCode * 60 + wRidcode * 40) / 100;

  return ((GWORD)i32Code);
}

/**
 * @brief ADCCalibrateITCode
 *
 *   Calibrate Internal Temperature Code 
 *
 * @return  calibrated code
 */
GWORD ADCCalibrateITCode(GWORD wITcode)
{
  GINT32 i32Code;
  GINT32 i32DeltaCode = GOTP->aveIT80 - GOTP->aveIT25;
  
  i32Code = (GINT32)wITcode;
  i32Code = i32Code - GOTP->aveIT25;
  i32Code = i32Code * IT_IDEAL_CODE_DELTA;
  if(i32DeltaCode!= 0)
  {
    i32Code = i32Code/i32DeltaCode;
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_IT_GAIN_FAIL;
  }
  else
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_ADC1_IT_GAIN_FAIL;
    return 0;
  }
  i32Code = i32Code + IT_IDEAL_CODE_25;

  return (GWORD) i32Code;
}

/**
 * @brief ADCCalibrateETCode
 *
 *  Calibrate External Temperature Code 
 *
 * @return  calibrated code
 */
GWORD ADCCalibrateETCode(GWORD wETcode)
{
  GSHORT sCodeCurr;
  GSHORT sCodeET;
   
  ///-------------------------------------------------------------------///
  /// [AT-PM] : Get compensation of board and r-sense ; 11/01/2013
  ///-------------------------------------------------------------------///
  sCodeCurr = GADC->sAdcCurrCode;
  sCodeCurr = sCodeCurr + GADC->sCCOffset; 
  
  ///-------------------------------------------------------------------///
  ///   CodeExtComp = CURR_CODE  * (RSense  + OffsetR) / RSense)
  ///-------------------------------------------------------------------///
  GADC->adcCaliCode.sCodeETComp = sCodeCurr * 
                                  (GGB->cGasGauging.scCurrentThresholds.bRsense + 
                                   GGB->cCalibration.scCaliData.wOffsetR) /
                                  GGB->cGasGauging.scCurrentThresholds.bRsense ;

  /// [AT-PM] : Compensate temperature code ; 11/01/2013
  sCodeET = (GSHORT)GADC->adcRawCode.wCodeET;

  /// [AT-PM] : Check the compensation code is less than ET code ; 07/20/2015
  if(sCodeET > GADC->adcCaliCode.sCodeETComp)
  {
    sCodeET = sCodeET - GADC->adcCaliCode.sCodeETComp;
  }
  else
  {
    sCodeET = 0;
  }
  return ((GWORD)sCodeET);
}

/**
 * @brief ADCCalibrateRidCode
 *
 *  Calibrate RID Code 
 *
 * @return  calibrated code
 */
GWORD ADCCalibrateRidCode(GWORD wRidCode)
{
  GSHORT sCodeCurr;
  GSHORT sCodeRid;
   
  ///-------------------------------------------------------------------///
  /// [AT-PM] : Get compensation of board and r-sense ; 11/01/2013
  ///-------------------------------------------------------------------///
  sCodeCurr = GADC->sAdcCurrCode;
  sCodeCurr = sCodeCurr + GADC->sCCOffset; 
  
  ///-------------------------------------------------------------------///
  ///   CodeExtComp = CURR_CODE  * (RSense  + OffsetR) / RSense)
  ///-------------------------------------------------------------------///
  GADC->adcCaliCode.sCodeRidComp = sCodeCurr * 
                                   (GGB->cGasGauging.scCurrentThresholds.bRsense + 
                                    GGB->cCalibration.scCaliData.wOffsetR) /
                                   GGB->cGasGauging.scCurrentThresholds.bRsense ;

  /// [AT-PM] : Compensate RID code ; 03/24/2015
  sCodeRid = (GSHORT)GADC->adcRawCode.wCodeRid;

  /// [AT-PM] : Check the compensation code is less than ET code ; 07/20/2015
  if(sCodeRid > GADC->adcCaliCode.sCodeRidComp)
  {
    sCodeRid = sCodeRid - GADC->adcCaliCode.sCodeRidComp;
  }
  else
  {
    sCodeRid = 0;
  }
  return ((GWORD)sCodeRid);
}

/**
 * @brief ADC1CalibrateCode
 *
 *  Calibrate ADC1 code
 *
 * @para  obj address of MeasDataInternalType
 * @para  code  ADC1 code to be calibrated
 * @return  calibrated code
 */
GSHORT ADC1CalibrateCode(GSHORT code)
{
  GINT64 i64Code;
  GINT32 i32Code;
  
  GINT32 iDeltaITCode;
  GINT32 gain;
  GINT32 offset;

  iDeltaITCode = (GINT32)(GOTP->aveIT80) - (GINT32)(GOTP->aveIT25);

  /// [AT-PM] : Pre-operation to avoid 64-bit division ; 01/23/2013
  gain   = GADC->adc1Gain;
  offset = GADC->adc1Offset;
  while(1)
  {
    i64Code = (GINT64)code;
    i64Code = i64Code * iDeltaITCode - offset;
    i64Code = i64Code * ADC1_IDEAL_CODE_DELTA;
    /// i64Code < 0x7FFFFFFF
    /// i64Code > 0x80000000 ()
    if((i64Code < 2147483647) && (i64Code > -2147483647))
    {
      break;
    }
    code = code/2;
    iDeltaITCode = iDeltaITCode/2;
    gain = gain/4;
    offset = offset/4;
  }

  i32Code = (GINT32)i64Code;
  if(gain != 0)
  {
    i32Code = i32Code/gain;
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_GAIN_FAIL;
  }
  else
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_ADC1_GAIN_FAIL;
    return 0;
  }

  return (GSHORT)(i32Code);
}


/**
 * @brief ADCCalibrateChargeCode
 *
 *   Calibrate Charge Code 
 *
 * @return  calibrated code
 */
GSHORT ADCCalibrateCapCode(GSHORT sCodeCC)
{
  GINT32 i32Code;
  
  GINT64 tmp64;
  GINT32 tmp32;
  GINT32 gain;
  GINT32 offset;


  /// [AT-PM] : Calibrate charge code ; 01/23/2013
  i32Code = ADC1CalibrateCode((sCodeCC*2));

  /// [AT-PM] : Remove the offset in calibrated charge code ; 01/23/2013
  gain   = GADC->adc1Gain;
  offset = GADC->adc1Offset;
  
  while(1)
  {
    tmp64 = (GINT64)offset;
    tmp64 = tmp64 * ADC1_IDEAL_CODE_DELTA;
    if((tmp64 < 2147483647) && (tmp64 > -2147483647))
    {
      tmp32 = (GINT32)tmp64;
      break;
    }
    gain = gain/2;
    offset = offset/2;
  }
  if(gain!= 0)
  {
    tmp32 = tmp32/gain;
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC1_CAP_GAIN_FAIL;
  }
  else
  {   
    UpiGauge.wErrorStatus |= GAUGE_ERR_ADC1_CAP_GAIN_FAIL;
    return 0;
  }

  i32Code = i32Code + tmp32;
  
  return (GSHORT)i32Code;
}

/**
 * @brief ADC2ConvertData
 *
 *  Convert ADC2 data from OTP
 *
 * @return  _UPI_NULL_
 */
GVOID ADC2ConvertData(GVOID)
{
  GWORD  wDeltaCode;
  GINT32 dwDeltaCode;
  GBYTE  bAdcCodeIndex;

  ///----------------------------------------------------------------///
  ///
  /// 1. Get ADC2_CODE_T25_100MV
  ///
  ///----------------------------------------------------------------///
  wDeltaCode = GOTP->adc2DeltaCodeT25V100;

  ///---------------------------------------///
  /// Check signed bit
  ///---------------------------------------///
  if(wDeltaCode & ADC2_DELTA_CODE_25_100MV_SIGN_BIT)
  {
    wDeltaCode = wDeltaCode & (~ADC2_DELTA_CODE_25_100MV_SIGN_BIT);
    wDeltaCode = wDeltaCode + ADC2_CODE_100MV_NEGATIVE;
  }
  wDeltaCode = wDeltaCode + ADC2_CP_CODE_25_100MV;
  dwDeltaCode = (GINT32)(GINT16)wDeltaCode;

  ///---------------------------------------///
  /// Get index and map the real code value  
  /// from the mapping table
  ///---------------------------------------///
  bAdcCodeIndex = GOTP->indexAdc2V100T25;
  dwDeltaCode = dwDeltaCode + AdcDeltaCodeMapping[bAdcCodeIndex].Adc2V100;

  ///---------------------------------------///
  /// Assign the Adc1CodeT25V100mV value
  ///---------------------------------------///
  GADC->adc2CodeT25V100 = (GINT16)dwDeltaCode;  

  ///----------------------------------------------------------------///
  ///
  /// 2. Get ADC2_CODE_T25_200MV
  ///
  ///----------------------------------------------------------------///
  wDeltaCode = GOTP->adc2DeltaCodeT25V200;

  ///---------------------------------------///
  /// Check signed bit
  ///---------------------------------------///
  if(wDeltaCode & ADC2_DELTA_CODE_25_200MV_SIGN_BIT)
  {
    wDeltaCode = wDeltaCode & (~ADC2_DELTA_CODE_25_200MV_SIGN_BIT);
    wDeltaCode = wDeltaCode + ADC2_CODE_200MV_NEGATIVE;
  }
  wDeltaCode = wDeltaCode + ADC2_CP_CODE_25_200MV;
  dwDeltaCode = (GINT32)(GINT16)wDeltaCode;
  
  ///---------------------------------------///
  /// Get index and map the real code value  
  /// from the mapping table
  ///---------------------------------------///
  bAdcCodeIndex = GOTP->indexAdc2V200T25;
  dwDeltaCode = dwDeltaCode+ AdcDeltaCodeMapping[bAdcCodeIndex].Adc2V200;
  
  ///---------------------------------------///
  /// Assign the Adc1CodeT25V100mV value
  ///---------------------------------------///
  GADC->adc2CodeT25V200 = (GINT16)dwDeltaCode;


  ///----------------------------------------------------------------///
  ///
  /// 3. Get Code for ADC2_CODE_T80_100mV and ADC2_CODE_T80_100mV
  ///
  ///----------------------------------------------------------------///
  dwDeltaCode = (GINT32) (GOTP->aveIT80 - GOTP->aveIT25) * ADC_TEMPERATURE_GAIN_CONST;

  /// [AT-PM] : Get code T80 100mV ; 01/23/2013
  GADC->adc2CodeT80V100 = (GINT16)(dwDeltaCode/ADC2_TEMPERATURE_GAIN_100MV + GADC->adc2CodeT25V100);

  /// [AT-PM] : Get code T80 200mV ; 01/23/2013
  GADC->adc2CodeT80V200 = (GINT16)(dwDeltaCode/ADC2_TEMPERATURE_GAIN_200MV + GADC->adc2CodeT25V200);
}

/**
 * @brief ADC2FactorCal
 *
 *  Calculate ADC2's Gain and Offset
 *
 * @return NULL
 *
 */
GVOID ADC2FactorCal(GVOID)
{
  GINT32 iDeltaVoltCode25;
  GINT32 iDeltaVoltCode80;

  /// [AT-PM] : Calculate gain slope and factor B ; 01/23/2013
  iDeltaVoltCode25 = (GINT32)GADC->adc2CodeT25V200;
  iDeltaVoltCode25 = iDeltaVoltCode25 - GADC->adc2CodeT25V100;
  iDeltaVoltCode80 = (GINT32)GADC->adc2CodeT80V200;
  iDeltaVoltCode80 = iDeltaVoltCode80 - GADC->adc2CodeT80V100;

  GADC->adc2GainSlope = iDeltaVoltCode80 - iDeltaVoltCode25;
  GADC->adc2GainFactorB = iDeltaVoltCode25*(GOTP->aveIT80) - iDeltaVoltCode80*(GOTP->aveIT25);

  /// [AT-PM] : Calculate offset slope and factor O ; 01/23/2013
  iDeltaVoltCode25 = (GINT32)GADC->adc2CodeT25V100;
  iDeltaVoltCode25 = iDeltaVoltCode25*2 - GADC->adc2CodeT25V200;
  iDeltaVoltCode80 = (GINT32)GADC->adc2CodeT80V100;
  iDeltaVoltCode80 = iDeltaVoltCode80*2 - GADC->adc2CodeT80V200;

  GADC->adc2OffsetSlope = iDeltaVoltCode80 - iDeltaVoltCode25;
  GADC->adc2OffsetFactorO = iDeltaVoltCode25*(GOTP->aveIT80) - iDeltaVoltCode80*(GOTP->aveIT25);
}

/**
 * @brief ADC2FactorGet
 *
 *  Get instant gain and offset for ADC2
 *
 * @return NULL
 *
 */
GVOID ADC2FactorGet(GWORD wCodeIntTemp)
{
  GINT64 i64Gain;
  GINT64 i64Offset;  
  
 /// [AT-PM] : Calculate current ADC1 gain ; 01/23/2013
  i64Gain = (GINT64)GADC->adc2GainSlope;
  i64Gain = i64Gain*(wCodeIntTemp) + GADC->adc2GainFactorB;
  GADC->adc2Gain = (GINT32)i64Gain;

  /// [AT-PM] : Calculate current ADC1 offset ; 01/23/2013
  i64Offset = (GINT64)GADC->adc2OffsetSlope;
  i64Offset = i64Offset*(wCodeIntTemp) + GADC->adc2OffsetFactorO;
  GADC->adc2Offset = (GINT32)i64Offset;
}

/**
 * @brief ADC1CalibrateCode
 *
 *  Calibrate ADC2 code
 *
 * @para  obj address of MeasDataInternalType
 * @para  code  ADC1 code to be calibrated
 * @return  calibrated code
 */
GWORD ADC2CalibrateCode(GWORD code)
{
  GINT64 i64Code;
  GINT32 i32Code;
  
  GINT32 iDeltaITCode;
  GINT32 gain;
  GINT32 offset;

  iDeltaITCode = (GINT32)(GOTP->aveIT80) - (GINT32)(GOTP->aveIT25);

  /// [AT-PM] : Pre-operation to avoid 64-bit division ; 01/23/2013
  gain   = GADC->adc2Gain;
  offset = GADC->adc2Offset;
  while(1)
  {
    i64Code = (GINT64)code;
    i64Code = i64Code * iDeltaITCode - offset;
    i64Code = i64Code * ADC2_IDEAL_CODE_DELTA;
    /// i64Code < 0x7FFFFFFF
    /// i64Code > 0x80000000 ()
    if((i64Code < 2147483647) && (i64Code > -2147483647))
    {
      break;
    }
    code = code/2;
    iDeltaITCode = iDeltaITCode/2;
    gain = gain/4;
    offset = offset/4;
  }

  i32Code = (GINT32)i64Code;
  if(gain!= 0)
  {
    UpiGauge.wErrorStatus &= ~GAUGE_ERR_ADC2_GAIN_FAIL;
    i32Code = i32Code/gain;
  }
  else
  {
    UpiGauge.wErrorStatus |= GAUGE_ERR_ADC2_GAIN_FAIL;
    return 0;
  }

  return (GWORD)(i32Code);
}

/**
 * @brief ADC2RevertCalibrateCode
 *
 *  Revert Calibrate ADC2 code 
 *
 * @para  obj address of MeasDataInternalType
 * @para  code  ADC1 code to be calibrated
 * @return  calibrated code
 */

GWORD ADC2RevertCalibrateCode(GWORD code)
{
  GINT64 i64Code1;
  GINT64 i64Code2;
  GINT64 i64IdealDelta;
  
	GINT32 iDeltaITCode;
  GINT32 gain;
  GINT32 offset;
  GINT32 i32Code1;
  GINT32 i32Code2;

	int    iCnt;
  ///------------------------------------------------///
  /// Calculate Rawcode
  ///------------------------------------------------///
  
	iDeltaITCode = (GINT32)(GOTP->aveIT80) - (GINT32)(GOTP->aveIT25);
  gain   = GADC->adc2Gain;
  offset = GADC->adc2Offset;
	iCnt = 1;
  while(1)
  {
    i64Code1 = (GINT64)code;

    i64IdealDelta = (GINT64)ADC2_IDEAL_CODE_DELTA;
    i64Code1 = i64Code1 * gain;
    i64Code2 = offset * i64IdealDelta;
    i64Code1 = i64Code1 + i64Code2;
    
    if((i64Code1 < 2147483647) && (i64Code1 > -2147483647))
    { 
      break; 
    }

    gain = gain / 2;
    offset = offset / 2;
	  iCnt = iCnt * 2;
  }
  i32Code1 = (GINT64)code;
  i32Code1 = i32Code1 * gain;
  i32Code2 = (GINT64)offset;
  i32Code2 = i32Code2 * ADC2_IDEAL_CODE_DELTA;
  i32Code1 = i32Code1 + i32Code2;
  i32Code2 = (GINT32)iDeltaITCode;
  i32Code2 = i32Code2 * ADC2_IDEAL_CODE_DELTA;
  i32Code2 = i32Code2 / iCnt;                  ///> [YL] : iDeltaITCode 32 bit calculate ; 20150514
  i32Code1 = i32Code1 / i32Code2;
 
  return (GWORD)(i32Code1);
}

/**
 * @brief ADCSetCellEn
 *
 * Set Cell Enable
 *
 * @return NULL
 *
 */
GVOID ADCSetCellEn(GVOID)
{
  GBYTE bReg;

  bReg = READ_REG(REG_CELL_EN);
  
  ///--------------------------------------------------///
  /// CELL_EN1 : Global Enable for RID, and ET
  /// CELL_EN0 : Global Enable for OSC
  ///--------------------------------------------------///  
  bReg = bReg | (CELL_EN1 | CELL_EN0);
  
  WRITE_REG(REG_CELL_EN, bReg);
}

/**
 * @brief RevertCurrent
 *
 *  Revert current to adc code
 *
 * @para  obj address of MeasDataInternalType
 * @para  curr  current to be reverted
 * @return  adc code
 */
GINT16 RevertCurrent(GINT32 curr)
{
  GINT32 tmp32;

  tmp32 = curr*(GGB->cGasGauging.scCurrentThresholds.bRsense) - ADC1_VOLTAGE_100MV;
  tmp32 = tmp32 * ADC1_IDEAL_CODE_DELTA/ADC1_VOLTAGE_DELTA;
  tmp32 = tmp32 + ADC1_IDEAL_CODE_100MV;
  return ((GINT16)tmp32);
}

/**
 * @brief ADCSetCCOffset
 *
 *  Calculate coulomb counter offset
 *
 * @return  NULL
 */
GVOID ADCSetCCOffset(GVOID)
{
  GINT32 offset;
  GINT32 tmp32;
  GBYTE bProductType;
  GWORD wData;

  GINT32 iDeltaITCode;

  /// [AT-PM] : Offset from FT ; 09/24/2013
  iDeltaITCode = (GOTP->aveIT80 - GOTP->aveIT25);
  if(iDeltaITCode != 0)
  {
    offset = GADC->adc1Offset /iDeltaITCode ;
  }
  else
  {
    offset = GADC->adc1Offset;
  }
 
  /// [AT-PM] : Offset from board factor and calibration factor ; 09/24/2013
  tmp32 = (GINT32)  GGB->cCalibration.scCaliData.sAdc1PosOffset;

  bProductType = GET_PRODUCT_TYPE(GGB->cConfiguration.scRegisters.dwOPCfg);  
  tmp32 = tmp32 * BoardFactor[bProductType].currentGain/BOARD_FACTOR_CONST;
  tmp32 = tmp32 + BoardFactor[bProductType].currentOffset;
 
  /// [AT-PM] : Revert board factor and calibration factor  09/24/2013  
  tmp32 = RevertCurrent(tmp32);
  
  /// [AT-PM] : Update offset ; 09/24/2013
  offset = (offset - tmp32)*(-1);
  GADC->sCCOffset = (GINT16)offset;

  wData = *(GWORD *)&GADC->sCCOffset;

  /// [AT-PM] : Set coulomb counter offset ; 01/27/2013
  WRITE_REG16(REG16_ADC1_OFFSET, wData);
}


/**
 * @brief ADCInit
 *
 *  Init ADC
 *
 * @return NULL
 *
 */
GVOID ADCInit(GVOID)
{
  GBYTE bReg = 0x00;

  GADC->dwAdcITCodeSum = 0;
  GADC->bAdcITCodeCnt = 0;
  GADC->wAdcITCode = 0xFFFF;

  GADC->dwAdcETCodeSum = 0;
  GADC->bAdcETCodeCnt = 0;
  GADC->wAdcETCode = 0xFFFF; 
  
  GADC->dwAdcRidCodeSum = 0;
  GADC->bAdcRidCodeCnt = 0;
  GADC->wAdcRidCode = 0xFFFF; 

  GADC->dwAdcVCell1CodeSum = 0;
  GADC->bAdcVCell1CodeCnt  = 0;
  GADC->wAdcVCell1Code     = 0xFFFF;

  GADC->sCCOffset = GGB->cCalibration.scCaliData.sAdc1PosOffset;

  ///------------------------------------------------------------///
  /// 1. Init Filter
  ///------------------------------------------------------------///  
  ADCFilterInit();  

  ///------------------------------------------------------------///
  /// 1. Init Chopper
  ///------------------------------------------------------------///  
  ADCChopperSet();

  ///------------------------------------------------------------///
  /// 2. Init ADC1 Queue
  ///------------------------------------------------------------///
  if(UpiGauge.wSysMode & SYS_MODE_RESTORED)
  {
    ADC1QueueSet();
  }
  else
  {
    ADC1QueueInitSet();
  }
  
  ///------------------------------------------------------------///
  /// 3. Init ADC2 Queue
  ///------------------------------------------------------------///
  ADC2QueueSet();

  ///------------------------------------------------------------///
  /// 4. Enable ADC
  ///------------------------------------------------------------///
  ADCEnable();

  ///------------------------------------------------------------///
  /// 5. Decimation Reset
  ///------------------------------------------------------------///
  DecimateRst();

  ///------------------------------------------------------------///
  /// 6.Set Internal Control B
  ///------------------------------------------------------------///
  bReg = READ_REG(REG_INTR_CTRL_B); 
  
  /// Set ET, IT, RID
  bReg |= (INTR_CTRL_B_ET_EN | INTR_CTRL_B_IT_EN | INTR_CTRL_B_RID_EN); 
  
  /// Set CBC
  bReg &= ~(INTR_CTRL_B_CBC_32_EN | INTR_CTRL_B_CBC_21_EN);   
  bReg |= (GGB->cConfiguration.scAFE.bCbcEnable << INTR_CTRL_B_CBC_BIT );
  WRITE_REG(REG_INTR_CTRL_B, bReg); 

  ///-------------------------------------------------------------------------------///
  /// 7. Get ADC Factors
  ///-------------------------------------------------------------------------------///
  ADC1ConvertData();
  ADC1FactorCal();
  ADC2ConvertData();  
  ADC2FactorCal();  

  ///-------------------------------------------------------------------------------///
  /// Global Enable  ET, RID and internal OSC
  ///-------------------------------------------------------------------------------///
  ADCSetCellEn(); 
}

