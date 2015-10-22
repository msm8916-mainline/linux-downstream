/**
 * @brief VCapacity.c
 *
 * Voltage gauge algorithm
 *
 * @author Allen Teng <allen.kuoliang.teng@gmail.com>
 * @note
 */

#ifdef  __cplusplus
extern "C" {
#endif  ///< end of __cplusplus

#include "UpiDef.h"

/**
 * @breif VCapGenVoltTable
 *
 * Generate voltage table according to taper voltage and EDVF
 *
 * @return NULL
 */
STATIC GVOID VCapGenVoltTable(GVOID)
{
  GU4 wTmp;
  GU2 wStep;
  GU2 wIdx;

  /// [AT-PM] : Calculate voltage step per percentage ; 07/04/2014
  wTmp = (GU4)(GGB->cChargeControl.scTerminationCfg.wTPVoltage);
  wTmp = wTmp - (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
  wTmp = wTmp * VCAP_CONST_PERCENTAGE;
  wTmp = wTmp / VCAP_CONST_RSOC_UNIT;
  wStep = (GU2)wTmp;

  /// [AT-PM] : Calculate voltage table ; 07/04/2014
  wIdx = 0;
  while(1)
  {
    /// [AT-PM] : Assign discharging voltage table ; 07/06/2014
    wTmp = (GU4)(CapTableOcvSocPtr[wIdx]);
    wTmp = wTmp * wStep;
    wTmp = wTmp / VCAP_CONST_PERCENTAGE;
    wTmp = wTmp + (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
    VCAP->wDsgVoltTable[wIdx] = (GU2)wTmp;

    /// [AT-PM] : Assign charging voltage table ; 07/06/2014
    if(wIdx < (VCAP_OCV_TABLE_ELEMENT_CNT - 1))
    {
      wTmp = wTmp + (GGB->cChargeControl.scTerminationCfg.wTPCurrent);
      if(wTmp >= (GGB->cChargeControl.scTerminationCfg.wTPVoltage))
      {
        wTmp = (GGB->cChargeControl.scTerminationCfg.wTPVoltage) - wIdx;
      }
    }
    VCAP->wChgVoltTable[wIdx] = (GU2)wTmp;

    GLOGD("%s[%s]: VoltTable[%d] = (%d,%d)\n", VCAP_BATT_LOG_HEADER, __func__,
          wIdx,
          VCAP->wChgVoltTable[wIdx],
          VCAP->wDsgVoltTable[wIdx]);

    wIdx = wIdx + 1;
    if(wIdx >= VCAP_OCV_TABLE_ELEMENT_CNT)
    {
      break;
    }
  }
}

/**
 * @brief VCapGetTableSoc
 *
 * Get SOC from table
 *
 * @return NULL
 */
STATIC GVOID VCapGetTableSoc(GVOID)
{
  GU2 *ptrOcv;
  GU4 dwTmp;

  /// [AT-PM] : Select OCV table with battery state ; 07/06/2014
  if(UpiGauge.bState == STATE_CHARGING)
  {
    ptrOcv = (GU2 *)(&(VCAP->wChgVoltTable[0]));
  }
  else
  {
    ptrOcv = (GU2 *)(&(VCAP->wDsgVoltTable[0]));
  }

  /// [AT-PM] : Find the SOC ; 04/17/2014
  VCAP->bSocIdx = 0;
  while(1)
  {
    if(CurMeas->wMinVCell > (ptrOcv[VCAP->bSocIdx]))
    {
      GLOGD("%s[%s]: VCAP->bSocIdx = %d (%d > %d)\n", VCAP_BATT_LOG_HEADER, __func__,
            VCAP->bSocIdx,
            CurMeas->wMinVCell,
            ptrOcv[VCAP->bSocIdx]);
      break;
    }

    VCAP->bSocIdx = VCAP->bSocIdx + 1;
    if(VCAP->bSocIdx >= VCAP_OCV_TABLE_ELEMENT_CNT)
    {
      GLOGD("%s[%s]: End of table (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
            VCAP->bSocIdx);
      break;
    }		
  }

  /// [AT-PM] : Get RSOC ; 04/17/2014
  if(VCAP->bSocIdx == 0)
  {
    VCAP->wRsoc10x = CapTableOcvSocPtr[VCAP->bSocIdx];
  }
  else if(VCAP->bSocIdx == VCAP_OCV_TABLE_ELEMENT_CNT)
  {
    VCAP->wRsoc10x = CapTableOcvSocPtr[VCAP_OCV_TABLE_ELEMENT_CNT - 1];
  }
  else
  {
    /// [AT-PM] : Interpolate RSOC ; 04/17/2014
    dwTmp = (GU4)(CurMeas->wMinVCell);
    dwTmp = dwTmp - (ptrOcv[VCAP->bSocIdx]);
    dwTmp = dwTmp * (CapTableOcvSocPtr[VCAP->bSocIdx - 1] - CapTableOcvSocPtr[VCAP->bSocIdx]);
    dwTmp = dwTmp / (((ptrOcv[VCAP->bSocIdx -1])) - (ptrOcv[VCAP->bSocIdx]));
    dwTmp = dwTmp + CapTableOcvSocPtr[VCAP->bSocIdx];
    VCAP->wRsoc10x = (GU2)dwTmp;
  }
  GLOGD("%s[%s]: RSOC = %d (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wRsoc10x, 
        VCAP->bSocIdx);
}

/**
 * @brief ConvertUnitMahTime
 *
 * Convert capacity unit from mAh to time
 *
 * @para wMah capacity unit in mAh to be converted
 * @return capacity unit in time
 */
GU2 ConvertUnitMahTime(GWORD wMah)
{
  GU4 dwTmp;

  dwTmp = (GU4)wMah;
  dwTmp = dwTmp * TIME_MIN_TO_HOUR;
  dwTmp = dwTmp / VCAP_CONST_CURRENT;
  return ((GU2)dwTmp);
}

/**
 * @brief VCapInitRMFcc
 *
 * Initialize RM and FCC
 *
 * @return NULL
 */
STATIC GVOID VCapInitRMFcc(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Set FCC ; 07/18/2014
  VCAP->wFcc = ConvertUnitMahTime(GGB->cSBSConfiguration.scData.wILMD);

  /// [AT-PM] : Calculate RM ; 04/17/2014
  dwTmp = (GU4)VCAP->wFcc;
  dwTmp = dwTmp * (VCAP->wRsoc10x);
  dwTmp = dwTmp / VCAP_CONST_RSOC_UNIT;
  VCAP->wRM = (GU2)dwTmp;
  VCAP->wRMLast = VCAP->wRM;
}

/**
 * @brief VCapInit
 *
 * Initialize voltage gauge algorithm
 *
 * @return NULL
 */
GVOID VCapInit(GVOID)
{
  GLOGD("%s[%s]: Reset capacity algorithm\n", VCAP_BATT_LOG_HEADER, __func__);

  /// [AT-PM] : Generate voltage table ; 07/04/2014
  VCapGenVoltTable();

  /// [AT-PM] : Get SOC from voltage table ; 04/17/2014
  VCapGetTableSoc();
   
  /// [AT-PM] : Initialize RM and FCC ; 04/17/2014
  VCapInitRMFcc();
}

/**
 * @brief VCapInitQD
 *
 * Initialize QD buffer
 *
 * @return NULL
 */
GVOID VCapInitQD(GVOID)
{
  GU4 dwTmp;
  GU1 idx;
  GU2 soc;

  idx = 0;
  while(1)
  {
    if(idx < (VCAP_UPDATE_PTR_CNT - 1))
    {
      soc = CapTableOcvSocPtr[CapTableUpdateIdxPtr[idx + 1]] - 
            CapTableOcvSocPtr[CapTableUpdateIdxPtr[idx]];
    }
    else
    {
      soc = CapTableOcvSocPtr[0] - 
            CapTableOcvSocPtr[CapTableUpdateIdxPtr[idx]];
    }

    dwTmp = (GU4)soc;
    dwTmp = dwTmp * (VCAP->wFcc);
    dwTmp = dwTmp / VCAP_CONST_RSOC_UNIT;
    VCAP->wQD[idx] = (GU2)dwTmp;
    VCAP->wQDBuf[idx] = 0;
    GLOGD("%s[%s]: QD[%d] = (%d - %d)%% x %d = %d\n", VCAP_BATT_LOG_HEADER, __func__,
          idx,
          VCAP_CONST_RSOC_UNIT,
          soc,
          VCAP->wFcc,
          VCAP->wQD[idx]);

    idx = idx + 1;
    if(idx >= VCAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief VCapCalRMFromTime
 *
 * Calculate RM from time
 *
 * @para bIncrease GTRUE if in charging
 * @return NULL
 */
STATIC GVOID VCapCalRMFromTime(GBOOL bIncrease)
{
  GI4 lTmp;

  lTmp = (GI4)(VCAP->wRM);
  lTmp = (bIncrease == GTRUE) ? (lTmp + VCAP->wDeltaMin) : (lTmp - VCAP->wDeltaMin);
  if(lTmp < 0)
  {
    lTmp = 0;
  }
  if(lTmp > VCAP->wFcc)
  {
    lTmp = (GI4)VCAP->wFcc;
  }

  VCAP->wRM = (GU2)lTmp;
}

/**
 * @brief VCapStepFull
 *
 * Step RSOC to 100%
 *
 * @return NULL
 */
STATIC GVOID VCapStepFull(GVOID)
{
  GU2 wTmp;

  if(SBS->wVRSOC >= VCAP_CONST_PERCENTAGE)
  {
    return;
  }

  /// [AT-PM] : Step to 100% if full charge reached ; 04/18/2014
  wTmp = (GU2)VCAP->wFcc;
  wTmp = wTmp / VCAP_CONST_PERCENTAGE;
  if(wTmp < 1)
  {
    wTmp = 1;
  }
  wTmp = wTmp + VCAP->wRM;

  GLOGD("%s[%s]: Step RM = %d after full charged from %d\n", VCAP_BATT_LOG_HEADER, __func__,
        wTmp,
        VCAP->wRM);

  if(wTmp > VCAP->wFcc)
  {
    wTmp = VCAP->wFcc;
  }
  VCAP->wRM = wTmp;
}

/**
 * @brief VCapUpdateQD
 *
 * Self-learning mechanism
 *
 * @return NULL
 */
STATIC GVOID VCapUpdateQD(GVOID)
{
  GU4 dwTmp;
  GU2 wOldFcc;
  GU1 bIdx;
  GU1 bTargetIdx;

  if(!(VCAP->wDsgState & VCAP_DSG_STATE_QD))
  {
    GLOGD("%s[%s]: No QD update because VCAP_DSG_STATE_QD is not set\n", VCAP_BATT_LOG_HEADER, __func__);
    return;
  }

  bTargetIdx = 0;
  while(1)
  {
    if((!(VCAP->wDsgState & (VCAP_DSG_STATE_CROSS_EDV0 << bTargetIdx))) &&
       (VCAP->wDsgState & (VCAP_DSG_STATE_REACH_EDV0 << bTargetIdx)))
    {
      break;
    }

    bTargetIdx = bTargetIdx + 1;

    if(bTargetIdx >= VCAP_UPDATE_PTR_CNT)
    {
      GLOGD("%s[%s]: No QD update because no voltage point just reached.\n", VCAP_BATT_LOG_HEADER, __func__);
      return;
    }
  }

  VCAP->wQD[bTargetIdx] = VCAP->wQDBuf[bTargetIdx];
  VCAP->wDsgState = VCAP->wDsgState | (VCAP_DSG_STATE_CROSS_EDV0 << bTargetIdx);
  GLOGD("%s[%s]: QD[%d] = QDBuf[%d] = %d\n", VCAP_BATT_LOG_HEADER, __func__,
        bTargetIdx,
        bTargetIdx,
        VCAP->wQD[bTargetIdx]);

  /// [AT-PM] : Update FCC ; 05/13/2014
  wOldFcc = VCAP->wFcc;
  bIdx = 0;
  VCAP->wFcc = 0;
  while(1)
  {
    VCAP->wFcc = VCAP->wFcc + VCAP->wQD[bIdx];

    bIdx = bIdx + 1;
    if(bIdx >= VCAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
  GLOGD("%s[%s]: FCC = %d\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wFcc);

  /// [AT-PM] : Keep RSOC if FCC is updated ; 05/13/2014
  dwTmp = (GU4)VCAP->wRM;
  dwTmp = dwTmp * (VCAP->wFcc);
  dwTmp = dwTmp / wOldFcc;
  VCAP->wRM = (GU2)dwTmp;
  GLOGD("%s[%s]: RM = %d\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wRM);
}

/**
 * @brief VCapStepDsgRM
 *
 * Step RM to target value with 1% step
 *
 * @para wRMThrd target RM value
 * @return NULL
 */
STATIC GVOID VCapStepDsgRM(GU2 wRMThrd)
{
  GU2 wStep;

  if(VCAP->wRM <= wRMThrd)
  {
    GLOGD("%s[%s]: %d <= %d -> no needs to step\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->wRM,
          wRMThrd);
    return;
  }

  wStep = VCAP->wFcc / VCAP_CONST_RM_STEP_SIZE;

  if(VCAP->wRM < wStep)
  {
    GLOGD("%s[%s]: %d < %d -> set to 0\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->wRM,
          wStep);
    VCAP->wRM = 0;
    return;
  }

  VCAP->wRM = VCAP->wRM - wStep;
  if(VCAP->wRM < wRMThrd)
  {
    GLOGD("%s[%s]: %d < %d -> set to wRMThrd\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->wRM,
          wRMThrd);
    VCAP->wRM = wRMThrd;
    return;
  }
}

/**
 * @brief VCapResetQD
 *
 * Reset QD when self-learning is disqualified
 *
 * @return NULL
 */
STATIC GVOID VCapResetQD(GVOID)
{
  GU1 bIdx;

  /// [AT-PM] : Reset discharge state ; 04/22/2014
  VCAP->wDsgState = VCAP->wDsgState & (~(VCAP_DSG_STATE_QD |
                                         VCAP_DSG_STATE_CROSS_EDV0 |
                                         VCAP_DSG_STATE_CROSS_EDV1 |
                                         VCAP_DSG_STATE_CROSS_EDV2 |
                                         VCAP_DSG_STATE_REACH_EDV0 |
                                         VCAP_DSG_STATE_REACH_EDV1 |
                                         VCAP_DSG_STATE_REACH_EDV2)); 

  /// [AT-PM] : Reset qualified discharge counter buffer ; 05/13/2014
  bIdx = 0;
  while(1)
  {
    VCAP->wQDBuf[bIdx] = 0;

    bIdx = bIdx + 1;
    if(bIdx >= VCAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief VCapNowChg
 *
 * Operation in charging
 *
 * @return NULL
 */
STATIC GVOID VCapNowChg(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Handle capacity information at full charge ; 04/18/2014
  if(UpiGauge.bState == STATE_FULL_CHARGED)
  {
    /// [AT-PM] : Step to 100% if full charge reached ; 04/18/2014
    VCapStepFull();

    VCAP->wNearFullMin = 0;
  }
  else if(SBS->wVRSOC == VCAP_CONST_PERCENTAGE)
  {
    /// [AT-PM] : Keep current RM ; 05/26/2014
    VCAP->wRM = VCAP->wRMLast;

    VCAP->wNearFullMin = 0;
  }
  else
  {
    dwTmp = (GU4)VCAP->wRM;
    dwTmp = dwTmp * VCAP_CONST_PERCENTAGE;
    dwTmp = dwTmp / (VCAP->wFcc);

    if(dwTmp >= VCAP_CONST_PERCENTAGE)
    {
      /// [AT-PM] : Make maximum RSOC = 99% before full charge reached ; 04/18/2014
      dwTmp = VCAP_CONST_RSOC_UNIT - 5;
      dwTmp = dwTmp * (VCAP->wFcc);
      dwTmp = dwTmp / VCAP_CONST_RSOC_UNIT;

      GLOGD("%s[%s]: Lock RM = %d before full charged from %d\n", VCAP_BATT_LOG_HEADER, __func__,
            dwTmp,
            VCAP->wRM);
      VCAP->wRM = (GU2)dwTmp;
    }
    else
    {
      /// [AT-PM] : Convert to 0.1% for operation ; 10/30/2014
      dwTmp = dwTmp * VCAP_CONST_RSOC_UNIT;
      dwTmp = dwTmp / VCAP_CONST_PERCENTAGE;

      if(VCAP->wRsoc10x == VCAP_CONST_RSOC_UNIT)
      {
        if(VCAP->wNearFullMin >= VCAP_CONST_NEAR_FULL_TIME)
        {
          dwTmp = (GU4)VCAP->wRsoc10x;

          VCAP->wNearFullMin = VCAP_CONST_NEAR_FULL_TIME;
        }
        else
        {
          /// [AT-PM] : Average with table soc ; 10/28/2014
          dwTmp = dwTmp + VCAP->wRsoc10x;
          dwTmp = dwTmp / 2;
        }
      }
      else if(VCAP->wRsoc10x > VCAP_CHG_TABLE_MAX)
      {
        /// [AT-PM] : Step by 1% if larger than VCAP_CHG_TABLE_MAX ; 10/28/2014
        dwTmp = dwTmp + 1;

        /// [AT-PM] : Additional step if longer than 1 minutes ; 10/31/2014
        dwTmp = dwTmp + VCAP->wNearFullMin;
        if(dwTmp > VCAP_CONST_NEAR_FULL_RSOC)
        {
          dwTmp = VCAP_CONST_NEAR_FULL_RSOC;
        }

        VCAP->wNearFullMin = 0;
      }
      else
      {
        /// [AT-PM] : Average with table soc ; 10/28/2014
        dwTmp = dwTmp + VCAP->wRsoc10x;
        dwTmp = dwTmp / 2;

        VCAP->wNearFullMin = 0;
      }

      /// [AT-PM] : Get new RM ; 07/21/2014
      dwTmp = dwTmp * VCAP->wFcc;
      dwTmp = dwTmp / VCAP_CONST_RSOC_UNIT;
      VCAP->wRM = (dwTmp < VCAP->wRMLast) ? VCAP->wRMLast : (GU2)dwTmp;
      GLOGD("%s[%s]: Refer to table RM = %d (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
            VCAP->wRM,
            VCAP->wRMLast);
    }
  }
}

/**
 * @brief VCapLockDsgRM
 *
 * Lock RM at specified value in discharging
 *
 * @para wRMThrd target RM
 * @return NULL
 */
STATIC GVOID VCapLockDsgRM(GU2 wRMThrd)
{
  if(VCAP->wRM >= wRMThrd)
  {
    GLOGD("%s[%s]: %d >= %d -> no needs to lock\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->wRM,
          wRMThrd);
    return;
  }

  VCAP->wRM = VCAP->wRMLast; 
  return;
}

/**
 * @brief VCapDsgVoltChk
 *
 * Check voltage check point reached or not
 *
 * @return bUpdate GTRUE for update QD
 * @return NULL
 */
STATIC GVOID VCapDsgVoltChk(GBOOL bUpdate)
{
  GI1 bTargetIdx;
  GU1 bSocIdx;
  GU4 wRMThrd;

  bTargetIdx = 0;

  while(1)
  {
    /// [AT-PM] : Get remaining capacity from QD ; 05/13/2014
    if(bTargetIdx > 0)
    {
      wRMThrd = wRMThrd + (VCAP->wQD[bTargetIdx - 1]);
    }
    else
    {
      wRMThrd = 1;
    }

    /// [AT-PM] : Mapping soc idx ; 05/13/2014
    bSocIdx = CapTableUpdateIdxPtr[(GU1)bTargetIdx];

    /// [AT-PM] : Voltage is higher than check point ; 05/13/2014
    if(VCAP->bSocIdx <= bSocIdx)
    {
      GLOGD("%s[%s]: Voltage check point %d has not been reached (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
            bSocIdx,
            VCAP->bSocIdx);

      /// [AT-PM] : Reset delay ADC count ; 04/22/2014
      VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] = 0;

      /// [AT-PM] : Lock RM at specified value ; 04/22/2014
      VCapLockDsgRM((GU2)wRMThrd);
    }
    else
    {
      /// [AT-PM] : Count voltage check delay ADC count ; 05/13/2014
      VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] = VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] + CurMeas->iDeltaTime;
      GLOGD("%s[%s]: VCAP->lUpdateTimeMSec[%d] = %d\n", VCAP_BATT_LOG_HEADER, __func__,
            bTargetIdx,
            VCAP->lUpdateTimeMSec[(GU1)bTargetIdx]);

      /// [AT-PM] : Check delay ADC count ; 05/13/2014
      if((VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] / TIME_MSEC_TO_SEC) < VCAP_CONST_EDV_DELAY_SEC)
      {
        GLOGD("%s[%s]: Delay time for voltage check point %d has not been reached (%d < %d)\n", VCAP_BATT_LOG_HEADER, __func__,
				  		bSocIdx,
					  	(VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] / TIME_MSEC_TO_SEC),
              VCAP_CONST_EDV_DELAY_SEC);

        /// [AT-PM] : Lock RM at specified value ; 04/22/2014
        VCapLockDsgRM((GU2)wRMThrd);
      }
      else
      {
        VCAP->lUpdateTimeMSec[(GU1)bTargetIdx] = VCAP_CONST_EDV_DELAY_SEC * TIME_MSEC_TO_SEC;

        /// [AT-PM] : Step RM to specified value ; 04/22/2014
        VCapStepDsgRM((GU2)wRMThrd);

        if(bUpdate == GTRUE)
        {
          VCAP->wDsgState = VCAP->wDsgState | (VCAP_DSG_STATE_REACH_EDV0 << bTargetIdx);
        }
      }
    }

    bTargetIdx = bTargetIdx + 1;
    if(bTargetIdx >= VCAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief VCapDsgSocChk
 *
 * Check SOC with table
 *
 * @return NULL
 */
STATIC GVOID VCapDsgSocChk(GVOID)
{
  GU1 bSocIdx;
  GU4 wRMThrd;

  bSocIdx = CapTableUpdateIdxPtr[VCAP_UPDATE_PTR_CNT - 1];

  /// [AT-PM] : Active only if no voltage check point reached ; 10/13/2014
  if(VCAP->bSocIdx > bSocIdx)
  {
    GLOGD("%s[%s]: Any voltage check point is reached (%d > %d)\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->bSocIdx, bSocIdx);
    return;
  }

  /// [AT-PM] : Get target RM with interpolaration ; 10/13/2014
  wRMThrd = (GU4)VCAP->wQD[VCAP_UPDATE_PTR_CNT - 1];
  wRMThrd = wRMThrd * (CurMeas->wMinVCell - VCAP->wDsgVoltTable[bSocIdx]);
  wRMThrd = wRMThrd / (VCAP->wDsgVoltTable[0] - VCAP->wDsgVoltTable[bSocIdx]);

  bSocIdx = 0;
  while(bSocIdx < (VCAP_UPDATE_PTR_CNT - 1))
  {
    wRMThrd = wRMThrd + VCAP->wQD[bSocIdx];
    bSocIdx = bSocIdx + 1;
  }
  GLOGD("%s[%s]: Target RM = %d\n", VCAP_BATT_LOG_HEADER, __func__,
        wRMThrd);

  VCapLockDsgRM((GU2)wRMThrd);
  VCapStepDsgRM((GU2)wRMThrd);
}

/**
 * @brief VCapCumuQD
 *
 * Cumulative QD buffer
 *
 * @return NULL
 */
STATIC GVOID VCapCumuQD(GVOID)
{
  GU1 bIdx;

  if(!(VCAP->wDsgState & VCAP_DSG_STATE_QD))
  {
    GLOGD("%s[%s]: VCAP_DSG_STATE_QD is not set (%x)\n", VCAP_BATT_LOG_HEADER, __func__,
          VCAP->wDsgState);
    return;
  }

  bIdx = 0;
  while(1)
  {
    if(VCAP->wDsgState & (VCAP_DSG_STATE_CROSS_EDV0 << bIdx))
    {
      if(bIdx == 0)
      {
        GLOGD("%s[%s]: Below EDV0\n", VCAP_BATT_LOG_HEADER, __func__);
        break;
      }

      bIdx = bIdx - 1;
      VCAP->wQDBuf[bIdx] = VCAP->wQDBuf[bIdx] + VCAP->wQDMin; 
      GLOGD("%s[%s]: Update QDBuf[%d] = %d (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
            bIdx,
            VCAP->wQDBuf[bIdx],
            VCAP->wQDMin);
      break;
    }

    bIdx = bIdx + 1;
    if(bIdx >= VCAP_UPDATE_PTR_CNT)
    {
      bIdx = VCAP_UPDATE_PTR_CNT - 1;
      VCAP->wQDBuf[bIdx] = VCAP->wQDBuf[bIdx] + VCAP->wQDMin;
      GLOGD("%s[%s]: Update last QDBuf[%d] = %d (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
            bIdx,
            VCAP->wQDBuf[bIdx],
            VCAP->wQDMin);
      break;
    }
  }
}

/**
 * @brief VCapNowDsg
 *
 * Operation in discharging
 *
 * @return NULL
 */
STATIC GVOID VCapNowDsg(GVOID)
{
  /// [AT-PM] : Cumulate qualified discharge counter ; 04/22/2014
  VCapCumuQD();

  /// [AT-PM] : Check RM with table ; 05/13/2014
  VCapDsgVoltChk(GTRUE);

  /// [AT-PM] : Check SOC with table ; 10/13/2014
  VCapDsgSocChk();

  /// [AT-PM] : Update QD if necessary ; 06/03/2014
  VCapUpdateQD();
}

/**
 * @brief VCapNowStandby
 *
 * Operation in standby
 *
 * @return NULL
 */
STATIC GVOID VCapNowStandby(GVOID)
{
  /// [AT-PM] : Check RM with table ; 05/13/2014
  VCapDsgVoltChk(GFALSE);

  /// [AT-PM] : Check SOC with table ; 10/13/2014
  VCapDsgSocChk();
}

/**
 * @brief VCapUpdateDeltaTime
 *
 * Calculate delta time
 * 
 * @return NULL
 */
STATIC GVOID VCapUpdateDeltaTime(GVOID)
{
  VCAP->lBufTimeMSec  = VCAP->lBufTimeMSec  + CurMeas->iDeltaTime;
  VCAP->wDeltaSec     = VCAP->lBufTimeMSec  / TIME_MSEC_TO_SEC;
  VCAP->lBufTimeMSec  = VCAP->lBufTimeMSec  % TIME_MSEC_TO_SEC;

  VCAP->wBufTimeSec   = VCAP->wBufTimeSec   + VCAP->wDeltaSec;
  VCAP->wDeltaMin     = VCAP->wBufTimeSec   / TIME_SEC_TO_MIN;
  VCAP->wBufTimeSec   = VCAP->wBufTimeSec   % TIME_SEC_TO_MIN;

  VCAP->wBufQDTimeSec = VCAP->wBufQDTimeSec + VCAP->wDeltaSec;
  VCAP->wQDMin        = VCAP->wBufQDTimeSec / TIME_SEC_TO_MIN;
  VCAP->wBufQDTimeSec = VCAP->wBufQDTimeSec % TIME_SEC_TO_MIN;

  VCAP->wNearFullMin      = VCAP->wNearFullMin      + VCAP->wDeltaMin;
}
 
/**
 * @brief VCapUpdate
 *
 * Update capacity information
 *
 * @return NULL
 */
GVOID VCapUpdate(GVOID)
{
  /// [AT-PM] : Service SET_FULL command ; 06/17/2015
  if(VCAP->bCapCntl & VCAP_CNTL_SET_FULL)
  {
    VCAP->bCapCntl = VCAP->bCapCntl & (~VCAP_CNTL_SET_FULL);

    VCAP->bChgState = VCAP->bChgState | VCAP_CHG_STATE_FULL_CHARGE;
    VCAP->wDsgState = VCAP->wDsgState | VCAP_DSG_STATE_QD;
    VCAP->wNearFullMin  = 0;
  
    VCAP->wRM = VCAP->wFcc;
    GLOGE("%s[%s]: Set capacity to full\n", VCAP_BATT_LOG_HEADER, __func__);
    return;
  }

  /// [AT-PM] : Calculate delta time ; 07/18/2014
  VCapUpdateDeltaTime();
  GLOGD("%s[%s]: Delta time = %d.%d min or %d.%d sec\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wDeltaMin,
        VCAP->wBufTimeSec,
        VCAP->wDeltaSec,
        VCAP->lBufTimeMSec);

  /// [AT-PM] : Get SOC from table ; 07/18/2014
  VCapGetTableSoc();

  VCAP->wRM  = SBS->wVRT;
  VCAP->wRMLast = VCAP->wRM;
  VCAP->wFcc = SBS->wVFCT; 
  GLOGD("%s[%s]: Synchronize RM and FCC from SBS data (%d/%d)\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wRM,
        VCAP->wFcc);

  /// [AT-PM] : Update RM from coulomb counter ; 04/18/2014
  if(UpiGauge.bState == STATE_CHARGING)
  {
    VCapCalRMFromTime(GTRUE);
  }
  if(UpiGauge.bState == STATE_DISCHARGING)
  {
    VCapCalRMFromTime(GFALSE);
  }
  GLOGD("%s[%s]: Update RM and FCC from time (%d/%d)\n", VCAP_BATT_LOG_HEADER, __func__,
        VCAP->wRM,
        VCAP->wFcc);

  if(UpiGauge.bState == STATE_FULL_CHARGED)
  {
    VCAP->bChgState = VCAP->bChgState | VCAP_CHG_STATE_FULL_CHARGE;
    VCAP->wDsgState = VCAP->wDsgState | VCAP_DSG_STATE_QD;
    VCAP->wNearFullMin  = 0;

    VCapStepFull();
  }
  else if(UpiGauge.bState == STATE_FULL_DISCHARGED)
  {
    VCAP->wDsgState = VCAP->wDsgState | VCAP_DSG_STATE_REACH_EDV0;
    VCAP->wNearFullMin  = 0;

    /// [AT-PM] : Update QD if necessary ; 06/03/2014
    VCapUpdateQD();

    VCAP->bChgState = VCAP->bChgState & (~VCAP_CHG_STATE_FULL_CHARGE);
    VCAP->wDsgState = VCAP->wDsgState & (~VCAP_DSG_STATE_QD);

    VCapStepDsgRM(0);
  }
  else
  {
    VCAP->bChgState = VCAP->bChgState & (~VCAP_CHG_STATE_FULL_CHARGE);

    if(UpiGauge.bState == STATE_CHARGING)
    {
      /// [AT-PM] : Reset discharge state ; 04/22/2014
      VCapResetQD();

      VCapNowChg();
    }
    else if(UpiGauge.bState == STATE_DISCHARGING)
    {
      VCAP->wNearFullMin  = 0;

      VCapNowDsg();
    }
    else
    {
      VCAP->wNearFullMin  = 0;

      /// [AT-PM] : Reset discharge state ; 04/22/2014
      VCapResetQD();

      VCapNowStandby();
    }
  }
  GLOGD("%s[%s]: S(%x-%x-%x), IDX(%d), SOC(%d=%d/%d), QD(%d-%d-%d), QDB(%d-%d-%d), VDT(%d-%d-%d)\n", 
        VCAP_BATT_LOG_HEADER, __func__,
        VCAP->bCapCntl, VCAP->bChgState, VCAP->wDsgState,
        VCAP->bSocIdx,
        VCAP->wRsoc10x, VCAP->wRM, VCAP->wFcc,
        VCAP->wQD[2], VCAP->wQD[1], VCAP->wQD[0],
        VCAP->wQDBuf[2], VCAP->wQDBuf[1], VCAP->wQDBuf[0],
        VCAP->lUpdateTimeMSec[2], VCAP->lUpdateTimeMSec[1], VCAP->lUpdateTimeMSec[0]);
}

/**
 * @brief ConvertUnitTimeMah
 *
 * Convert capacity unit from time to mAh
 *
 * @para wTime capacity unit in time to be converted
 * @return capacity unit in mAh
 */
GWORD ConvertUnitTimeMah(GU2 wTime)
{
  GU4 dwTmp;

  dwTmp = (GU4)wTime;
  dwTmp = dwTmp * VCAP_CONST_CURRENT;
  dwTmp = dwTmp / TIME_MIN_TO_HOUR;
  return ((GWORD)dwTmp );
}

/**
 * @brief VCapUpdateSbs
 *
 * Update SBS data
 *
 * @return NULL
 */
GVOID VCapUpdateSbs(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Update RSOC ; 07/21/2014
  dwTmp = (GU4)VCAP->wRM;
  dwTmp = dwTmp * VCAP_CONST_PERCENTAGE;
  dwTmp = dwTmp / (VCAP->wFcc);
  if((dwTmp == 0) && (VCAP->wRM != 0))
  {
    dwTmp = 1;
  }
  SBS->wVRSOC = (GWORD)dwTmp;

  /// [AT-PM] : Update FCC ; 04/17/2014
  SBS->wVFCT = (GWORD)VCAP->wFcc; 
  SBS->wVFCC = ConvertUnitTimeMah(VCAP->wFcc);

  /// [AT-PM] : Update RM ; 04/17/2014
  SBS->wVRT = (GWORD)VCAP->wRM;
  dwTmp = (GU4)SBS->wVFCC;
  dwTmp = dwTmp * (SBS->wVRSOC);
  dwTmp = dwTmp / VCAP_CONST_PERCENTAGE;
  SBS->wVRM = (GWORD)dwTmp;

  /// [AT-PM] : Update ASOC ; 07/18/2014
  dwTmp = (GU4)SBS->wVRM;
  dwTmp = dwTmp * VCAP_CONST_PERCENTAGE;
  dwTmp = dwTmp / (GGB->cSBSConfiguration.scData.wILMD);
  if((dwTmp == 0) && (SBS->wVRM != 0))
  {
    dwTmp = 1;
  }
  SBS->wVASOC = (GWORD)dwTmp;

  GLOGD("%s[%s]: %d (%d) / %d (%d) = %d (%d)\n", VCAP_BATT_LOG_HEADER, __func__,
        SBS->wVRM,
        VCAP->wRM,
        SBS->wVFCC,
        VCAP->wFcc,
        SBS->wVRSOC,
        SBS->wVASOC);
}

/**
 * @brief VCapRestore
 *
 * Restore capacity information from mAh to time
 *
 * @return NULL
 */
GVOID VCapRestore(GVOID)
{
  /// [AT-PM] : Restore RT ; 07/18/2014
  SBS->wVRT = ConvertUnitMahTime((GU2)SBS->wVRM);

  /// [AT-PM] : Restore FCT ; 07/18/2014
  SBS->wVFCT = ConvertUnitMahTime((GU2)SBS->wVFCC);
}

#ifdef  __cplusplus
}
#endif  ///< end of __cplusplus

