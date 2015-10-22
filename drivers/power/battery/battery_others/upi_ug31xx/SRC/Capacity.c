/**
 * @file Capacity.c
 *
 * Capacity algorithm
 * - Fixed EDV
 *
 * @version $Revision$
 * @author AllenTeng <allen_teng@upi-semi.com>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
 */

#ifdef  __cplusplus
extern "C" {
#endif  ///< end of __cplusplus

#include "UpiDef.h"

/**
 * @brief CapGetTempIndex
 *
 * Get temperature index
 *
 * @return NULL
 */
STATIC GVOID CapGetTempIndex(GVOID)
{
  GU2 thrd;
  GWORD wTemp;

  if((GGB->cConfiguration.scRegisters.dwOPCfg & OPCFG_SBS_TEMP_SOURCE) == OPCFG_SBS_TEMP_SOURCE_INT)
  {
    wTemp = (CurMeas->sIntTemp + SBS_TEMP_C_2_K);
  }
  else
  {
    wTemp = (CurMeas->sExtTemp + SBS_TEMP_C_2_K);
  }

  CAP->bTempIdx = 0;
  while(1)
  {
    thrd = CapTableTempPtr[CAP->bTempIdx] + CapTableTempPtr[CAP->bTempIdx + 1];
    thrd = thrd / 2;
    thrd = thrd + SBS_TEMP_C_2_K;
    if(wTemp > thrd) 
    {
      GLOGD("%s[%s]: CAP->bTempIdx = %d (%d > %d)\n", CAP_BATT_LOG_HEADER, __func__,
            CAP->bTempIdx,
            wTemp,
            thrd);
      break;
    }

    CAP->bTempIdx =  CAP->bTempIdx + 1;
    if(CapTableTempPtr[CAP->bTempIdx] == 0)
    {
      GLOGD("%s[%s]: End of CapTableTemp table = %d (%d)\n", CAP_BATT_LOG_HEADER, __func__,
            CAP->bTempIdx,
            CapTableTempPtr[CAP->bTempIdx]);
      break;
    }
  }
}

/**
 * @brief CapGetOcvSoc
 *
 * Get SOC from OCV table
 *
 * @return NULL
 */
STATIC GVOID CapGetOcvSoc(GVOID)
{
  GU2 *ptrOcv;
  GU4 dwTmp;

  /// [AT-PM] : Select OCV table with battery state ; 07/06/2014
  if(UpiGauge.bState == STATE_CHARGING)
  {
    ptrOcv = (GU2 *)(&(CAP->wChgVoltTable[0]));
  }
  else if(UpiGauge.bState == STATE_STANDBY)
  {
    ptrOcv = (GU2 *)(&(CAP->wStandbyVoltTable[0]));
  }
  else
  {
    ptrOcv = (GU2 *)(&(CAP->wDsgVoltTable[0]));
  }

  /// [AT-PM] : Find the SOC ; 04/17/2014
  CAP->bSocIdx = 0;
  while(1)
  {
    if(CurMeas->wMinVCell > (ptrOcv[CAP->bSocIdx]))
    {
      GLOGD("%s[%s]: CAP->bSocIdx = %d (%d > %d)\n", CAP_BATT_LOG_HEADER, __func__,
            CAP->bSocIdx,
            CurMeas->wMinVCell,
            ptrOcv[CAP->bSocIdx]);
      break;
    }

    CAP->bSocIdx = CAP->bSocIdx + 1;
    if(CAP->bSocIdx >= CAP_OCV_TABLE_ELEMENT_CNT)
    {
      GLOGD("%s[%s]: End of table (%d)\n", CAP_BATT_LOG_HEADER, __func__,
            CAP->bSocIdx);
      break;
    }		
  }

  /// [AT-PM] : Get RSOC ; 04/17/2014
  if(CAP->bSocIdx == 0)
  {
    CAP->wRsoc10x = CapTableOcvSocPtr[CAP->bSocIdx];
  }
  else if(CAP->bSocIdx == CAP_OCV_TABLE_ELEMENT_CNT)
  {
    CAP->wRsoc10x = CapTableOcvSocPtr[CAP_OCV_TABLE_ELEMENT_CNT - 1];
  }
  else
  {
    /// [AT-PM] : Interpolate RSOC ; 04/17/2014
    dwTmp = (GU4)(CurMeas->wMinVCell);
    dwTmp = dwTmp - (ptrOcv[CAP->bSocIdx]);
    dwTmp = dwTmp * (CapTableOcvSocPtr[CAP->bSocIdx - 1] - CapTableOcvSocPtr[CAP->bSocIdx]);
    dwTmp = dwTmp / (((ptrOcv[CAP->bSocIdx -1])) - (ptrOcv[CAP->bSocIdx]));
    dwTmp = dwTmp + CapTableOcvSocPtr[CAP->bSocIdx];
    CAP->wRsoc10x = (GU2)dwTmp;
  }
  GLOGD("%s[%s]: RSOC = %d (%d)\n", CAP_BATT_LOG_HEADER, __func__,
        CAP->wRsoc10x, 
        CAP->bSocIdx);
}

/**
 * @brief CapInitRMFcc
 *
 * Initialize RM and FCC
 * - FCC = ILMD
 * - RM = FCC x RSOC
 *
 * @return NULL
 */
STATIC GVOID CapInitRMFcc(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Set FCC ; 04/17/2014
  CAP->wFcc = (GU2)GGB->cSBSConfiguration.scData.wILMD;

  /// [AT-PM] : Calculate RM ; 04/17/2014
  dwTmp = (GU4)CAP->wFcc;
  dwTmp = dwTmp * (CAP->wRsoc10x);
  dwTmp = dwTmp / CAP_CONST_RSOC_UNIT;
  CAP->wRM = (GU2)dwTmp;
  CAP->wRMLast = CAP->wRM;
}

/**
 * @brief CapCalRMFromCC
 *
 * Update RM from coulomb counter
 *
 * @return NULL
 */
STATIC GVOID CapCalRMFromCC(GVOID)
{
  GI4 lTmp;

  lTmp = (GI4)(CAP->wRM);
  lTmp = lTmp + CurMeas->sDeltaQ;
  if(lTmp < 0)
  {
    lTmp = 0;
  }
  if(lTmp > CAP->wFcc)
  {
    lTmp = (GI4)CAP->wFcc;
  }

  CAP->wRM = (GU2)lTmp;
}

/**
 * @brief CapCumuQD
 *
 * Cumulate qualified discharge counter from coulomb counter
 *
 * @return NULL
 */
STATIC GVOID CapCumuQD(GVOID)
{
  GU1 bIdx;

  if(!(CAP->wDsgState & CAP_DSG_STATE_QD))
  {
    GLOGD("%s[%s]: CAP_DSG_STATE_QD is not set (%x)\n", CAP_BATT_LOG_HEADER, __func__,
          CAP->wDsgState);
    return;
  }

  bIdx = 0;
  while(1)
  {
    if(CAP->wDsgState & (CAP_DSG_STATE_CROSS_EDV0 << bIdx))
    {
      if(bIdx == 0)
      {
        GLOGD("%s[%s]: Below EDV0\n", CAP_BATT_LOG_HEADER, __func__);
        break;
      }

      bIdx = bIdx - 1;
      CAP->wQDBuf[bIdx] = CAP->wQDBuf[bIdx] - CurMeas->sDeltaQ;
      GLOGD("%s[%s]: Update QDBuf[%d] = %d (%d)\n", CAP_BATT_LOG_HEADER, __func__,
            bIdx,
            CAP->wQDBuf[bIdx],
            CurMeas->sDeltaQ);
      break;
    }

    bIdx = bIdx + 1;
    if(bIdx >= CAP_UPDATE_PTR_CNT)
    {
      bIdx = CAP_UPDATE_PTR_CNT - 1;
      CAP->wQDBuf[bIdx] = CAP->wQDBuf[bIdx] - CurMeas->sDeltaQ;
      GLOGD("%s[%s]: Update last QDBuf[%d] = %d (%d)\n", CAP_BATT_LOG_HEADER, __func__,
            bIdx,
            CAP->wQDBuf[bIdx],
            CurMeas->sDeltaQ);
      break;
    }
  }
}

/**
 * @brief CapResetQD
 *
 * Reset QD state
 *
 * @return NULL
 */
STATIC GVOID CapResetQD(GVOID)
{
  GU1 bIdx;

  /// [AT-PM] : Reset discharge state ; 04/22/2014
  CAP->wDsgState = CAP->wDsgState & (~(CAP_DSG_STATE_QD |
                                       CAP_DSG_STATE_CROSS_EDV0 |
                                       CAP_DSG_STATE_CROSS_EDV1 |
                                       CAP_DSG_STATE_CROSS_EDV2 |
                                       CAP_DSG_STATE_CROSS_EDV3 |
                                       CAP_DSG_STATE_REACH_EDV0 |
                                       CAP_DSG_STATE_REACH_EDV1 |
                                       CAP_DSG_STATE_REACH_EDV2 |
                                       CAP_DSG_STATE_REACH_EDV3));

  /// [AT-PM] : Reset qualified discharge counter buffer ; 05/13/2014
  bIdx = 0;
  while(1)
  {
    CAP->wQDBuf[bIdx] = 0;

    bIdx = bIdx + 1;
    if(bIdx >= CAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapLockDsgRM
 *
 * Lock RM at discharging
 *
 * @para wRMThrd target RM to be locked
 * @return NULL
 */
STATIC GVOID CapLockDsgRM(GU2 wRMThrd)
{
  if(CAP->wRM >= wRMThrd)
  {
    GLOGD("%s[%s]: %d >= %d -> no needs to lock\n", CAP_BATT_LOG_HEADER, __func__,
          CAP->wRM,
          wRMThrd);
    return;
  }

  CAP->wRM = CAP->wRMLast; 
  return;
}

/**
 * @brief CapStepDsgRM
 *
 * Step RM at discharging with 0.5% FCC
 *
 * @para wRMThrd target RM to be stepped
 * @return NULL
 */
STATIC GVOID CapStepDsgRM(GU2 wRMThrd)
{
  GU2 wStep;

  if(CAP->wRM <= wRMThrd)
  {
    GLOGD("%s[%s]: %d <= %d -> no needs to step\n", CAP_BATT_LOG_HEADER, __func__,
          CAP->wRM,
          wRMThrd);
    return;
  }

  wStep = CAP->wFcc / CAP_CONST_RM_STEP_SIZE;

  if(CAP->wRM < wStep)
  {
    GLOGD("%s[%s]: %d < %d -> set to 0\n", CAP_BATT_LOG_HEADER, __func__,
          CAP->wRM,
          wStep);
    CAP->wRM = 0;
    return;
  }

  CAP->wRM = CAP->wRM - wStep;
  if(CAP->wRM < wRMThrd)
  {
    GLOGD("%s[%s]: %d < %d -> set to wRMThrd\n", CAP_BATT_LOG_HEADER, __func__,
          CAP->wRM,
          wRMThrd);
    CAP->wRM = wRMThrd;
    return;
  }
}

/**
 * @brief CapUpdateQD
 *
 * Update QD and FCC procedure
 *
 * @return NULL
 */
STATIC GVOID CapUpdateQD(GVOID)
{
  GU4 dwTmp;
  GU2 wOldFcc;
  GU1 bIdx;
  GU1 bTargetIdx;

  if(!(CAP->wDsgState & CAP_DSG_STATE_QD))
  {
    GLOGD("%s[%s]: No QD update because CAP_DSG_STATE_QD is not set\n", CAP_BATT_LOG_HEADER, __func__);
    return;
  }

  bTargetIdx = 0;
  while(1)
  {
    if((!(CAP->wDsgState & (CAP_DSG_STATE_CROSS_EDV0 << bTargetIdx))) &&
       (CAP->wDsgState & (CAP_DSG_STATE_REACH_EDV0 << bTargetIdx)))
    {
      break;
    }

    bTargetIdx = bTargetIdx + 1;

    if(bTargetIdx >= CAP_UPDATE_PTR_CNT)
    {
      GLOGD("%s[%s]: No QD update because no voltage point just reached.\n", CAP_BATT_LOG_HEADER, __func__);
      return;
    }
  }

  CAP->wQD[bTargetIdx] = CAP->wQDBuf[bTargetIdx];
  CAP->wDsgState = CAP->wDsgState | (CAP_DSG_STATE_CROSS_EDV0 << bTargetIdx);
  GLOGD("%s[%s]: QD[%d] = QDBuf[%d] = %d\n", CAP_BATT_LOG_HEADER, __func__,
        bTargetIdx,
        bTargetIdx,
        CAP->wQD[bTargetIdx]);

  /// [AT-PM] : Update FCC ; 05/13/2014
  wOldFcc = CAP->wFcc;
  bIdx = 0;
  CAP->wFcc = 0;
  while(1)
  {
    CAP->wFcc = CAP->wFcc + CAP->wQD[bIdx];

    bIdx = bIdx + 1;
    if(bIdx >= CAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
  GLOGD("%s[%s]: FCC = %d\n", CAP_BATT_LOG_HEADER, __func__,
        CAP->wFcc);

  /// [AT-PM] : Keep RSOC if FCC is updated ; 05/13/2014
  dwTmp = (GU4)CAP->wRM;
  dwTmp = dwTmp * (CAP->wFcc);
  dwTmp = dwTmp / wOldFcc;
  CAP->wRM = (GU2)dwTmp;
  GLOGD("%s[%s]: RM = %d\n", CAP_BATT_LOG_HEADER, __func__,
        CAP->wRM);
}

/**
 * @brief CapDsgVoltChk
 *
 * Discharge operation for voltage check point
 *
 * @para bUpdate set GTRUE for update QD
 * @return NULL
 */
STATIC GVOID CapDsgVoltChk(GBOOL bUpdate)
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
      wRMThrd = wRMThrd + (CAP->wQD[bTargetIdx - 1]);
    }
    else
    {
      wRMThrd = 1;
    }

    /// [AT-PM] : Mapping soc idx ; 05/13/2014
    bSocIdx = CapTableUpdateIdxPtr[(GU1)bTargetIdx];

    /// [AT-PM] : Voltage is higher than check point ; 05/13/2014
    if(CAP->bSocIdx <= bSocIdx)
    {
      GLOGD("%s[%s]: Voltage check point %d has not been reached (%d)\n", CAP_BATT_LOG_HEADER, __func__,
            bSocIdx,
            CAP->bSocIdx);

      /// [AT-PM] : Reset delay ADC count ; 04/22/2014
      CAP->lUpdateTimeMSec[(GU1)bTargetIdx] = 0;

      /// [AT-PM] : Lock RM at specified value ; 04/22/2014
      CapLockDsgRM((GU2)wRMThrd);
    }
    else
    {
      /// [AT-PM] : Count voltage check delay ADC count ; 05/13/2014
      CAP->lUpdateTimeMSec[(GU1)bTargetIdx] = CAP->lUpdateTimeMSec[(GU1)bTargetIdx] + CurMeas->iDeltaTime;
      GLOGD("%s[%s]: CAP->lUpdateTimeMSec[%d] = %d\n", CAP_BATT_LOG_HEADER, __func__,
            bTargetIdx,
            CAP->lUpdateTimeMSec[(GU1)bTargetIdx]);

      /// [AT-PM] : Check delay ADC count ; 05/13/2014
      if((CAP->lUpdateTimeMSec[(GU1)bTargetIdx] / TIME_MSEC_TO_SEC) < CAP_CONST_EDV_DELAY_SEC)
      {
        GLOGD("%s[%s]: Delay time for voltage check point %d has not been reached (%d < %d)\n", CAP_BATT_LOG_HEADER, __func__,
				  		bSocIdx,
					  	(CAP->lUpdateTimeMSec[(GU1)bTargetIdx] / TIME_MSEC_TO_SEC),
              CAP_CONST_EDV_DELAY_SEC);

        /// [AT-PM] : Lock RM at specified value ; 04/22/2014
        CapLockDsgRM((GU2)wRMThrd);
      }
      else
      {
        CAP->lUpdateTimeMSec[(GU1)bTargetIdx] = CAP_CONST_EDV_DELAY_SEC * TIME_MSEC_TO_SEC;

        /// [AT-PM] : Step RM to specified value ; 04/22/2014
        CapStepDsgRM((GU2)wRMThrd);

        if(bUpdate == GTRUE)
        {
          CAP->wDsgState = CAP->wDsgState | (CAP_DSG_STATE_REACH_EDV0 << bTargetIdx);
        }
      }
    }

    bTargetIdx = bTargetIdx + 1;
    if(bTargetIdx >= CAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapStepFull
 *
 * Step RM to 100%
 *
 * @return NULL
 */
STATIC GVOID CapStepFull(GVOID)
{
  GU2 wTmp;

  if(SBS->wRSOC >= CAP_CONST_PERCENTAGE)
  {
    return;
  }

  /// [AT-PM] : Step to 100% if full charge reached ; 04/18/2014
  wTmp = (GU2)CAP->wFcc;
  wTmp = wTmp / CAP_CONST_PERCENTAGE;
  if(wTmp < 1)
  {
    wTmp = 1;
  }
  wTmp = wTmp + CAP->wRM;

  GLOGD("%s[%s]: Step RM = %d after full charged from %d\n", CAP_BATT_LOG_HEADER, __func__,
        wTmp,
        CAP->wRM);

  if(wTmp > CAP->wFcc)
  {
    wTmp = CAP->wFcc;
  }
  CAP->wRM = wTmp;
}

/**
 * @brief CapCheckTable
 *
 * check capacity from OCV table
 *
 * @return NULL
 */
STATIC GVOID CapCheckTable(GVOID)
{
  GU2  bSRoc;
  GU4  dwTmp;

  bSRoc = CapTableOcvSocPtr[CAP->bSocIdx];
  dwTmp = bSRoc;
  dwTmp = dwTmp * CAP->wFcc;
  dwTmp = dwTmp / CAP_CONST_RSOC_UNIT; 
  CapLockDsgRM(dwTmp);

}

/**
 * @brief CapNowStandby
 *
 * Standby operation
 *
 * @return NULL
 */
STATIC GVOID CapNowStandby(GVOID)
{
  /// [YL] : Check Capacity with table ; 2015/07/15
  CapCheckTable();
  /// [AT-PM] : Check RM with table ; 05/13/2014
  CapDsgVoltChk(GFALSE);
	
}

/**
 * @brief CapNowChg
 *
 * Charge operation
 *
 * @return NULL
 */
STATIC GVOID CapNowChg(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Handle capacity information at full charge ; 04/18/2014
  if(UpiGauge.bState == STATE_FULL_CHARGED)
  {
    /// [AT-PM] : Step to 100% if full charge reached ; 04/18/2014
    CapStepFull();
  }
  else if(SBS->wRSOC == CAP_CONST_PERCENTAGE)
  {
    /// [AT-PM] : Keep current RM ; 05/26/2014
    CAP->wRM = CAP->wRMLast;
  }
  else
  {
    /// [AT-PM] : Make maximum RSOC = 99% before full charge reached ; 04/18/2014
    if(SBSCalSoc(CAP->wRM, CAP->wFcc) >= CAP_CONST_PERCENTAGE)
    {
      dwTmp = CAP_CONST_RSOC_UNIT - 5;
      dwTmp = dwTmp * (CAP->wFcc);
      dwTmp = dwTmp / CAP_CONST_RSOC_UNIT;

      GLOGD("%s[%s]: Lock RM = %d before full charged from %d\n", CAP_BATT_LOG_HEADER, __func__,
            dwTmp,
            CAP->wRM);
      CAP->wRM = (GU2)dwTmp;
    }

    /// [AT-PM] : Force RSOC = 0% before voltage > EDVF ; 06/01/2015
    if(CurMeas->wMinVCell <= (GGB->cGasGauging.scEDVCfg.wEdv1Voltage))
    {
      CAP->wRM = 0;
    }
  }
}

/**
 * @brief CapNowDsg
 *
 * Discharge operation
 *
 * @return NULL
 */
STATIC GVOID CapNowDsg(GVOID)
{
  /// [AT-PM] : Cumulate qualified discharge counter ; 04/22/2014
  CapCumuQD();

  /// [AT-PM] : Check RM with table ; 05/13/2014
  CapDsgVoltChk(GTRUE);

  /// [AT-PM] : Update QD if necessary ; 06/03/2014
  CapUpdateQD();
}

/**
 * @brief CapAverageGgb
 *
 * Average the table from ggb file
 *
 * @para pwOcvTable pointer of OCV table array
 * @para pbOcvCnt pointer of OCV table count
 * @para pwRTable pointer of R table array
 * @para pbRTable pointer of R table count
 * @return NULL
 */
STATIC GVOID CapAverageGgb(GU2 *pwOcvTable, GU1 *pbOcvCnt, GU2 *pwRTable, GU1 *pbRCnt)
{
  GU2 wIdx;
  GU2 *ptrOcv45;
  GU1 *ptrR45;
  GU2 *ptrOcv25;
  GU1 *ptrR25;
  GU2 *ptrOcv15;
  GU1 *ptrR15;
  GU2 *ptrOcv05;
  GU1 *ptrR05;

  wIdx = 0;
  ptrOcv45 = (GU2 *) &GGB->cCapacity.scOcvTableT45.wOcv100;
  ptrR45   = (GU1 *) &GGB->cCapacity.scRTableT45.bR100;
  ptrOcv25 = (GU2 *) &GGB->cCapacity.scOcvTableT25.wOcv100;
  ptrR25   = (GU1 *) &GGB->cCapacity.scRTableT25.bR100;
  ptrOcv15 = (GU2 *) &GGB->cCapacity.scOcvTableT15.wOcv100;
  ptrR15   = (GU1 *) &GGB->cCapacity.scRTableT15.bR100;
  ptrOcv05 = (GU2 *) &GGB->cCapacity.scOcvTableT05.wOcv100;
  ptrR05   = (GU1 *) &GGB->cCapacity.scRTableT05.bR100;
  while(wIdx < CAP_OCV_TABLE_ELEMENT_CNT)
  {
    if(ptrOcv45[wIdx] != 0)
    {
      pwOcvTable[wIdx] = pwOcvTable[wIdx] + ptrOcv45[wIdx];
      pbOcvCnt[wIdx]   = pbOcvCnt[wIdx] + 1;
      pwRTable[wIdx]   = pwRTable[wIdx] + ptrR45[wIdx];
      pbRCnt[wIdx]     = pbRCnt[wIdx] + 1;
    }
    if(ptrOcv25[wIdx] != 0)
    {
      pwOcvTable[wIdx] = pwOcvTable[wIdx] + ptrOcv25[wIdx];
      pbOcvCnt[wIdx]   = pbOcvCnt[wIdx] + 1;
      pwRTable[wIdx]   = pwRTable[wIdx] + ptrR25[wIdx];
      pbRCnt[wIdx]     = pbRCnt[wIdx] + 1;
    }
    if(ptrOcv15[wIdx] != 0)
    {
      pwOcvTable[wIdx] = pwOcvTable[wIdx] + ptrOcv15[wIdx];
      pbOcvCnt[wIdx]   = pbOcvCnt[wIdx] + 1;
      pwRTable[wIdx]   = pwRTable[wIdx] + ptrR15[wIdx];
      pbRCnt[wIdx]     = pbRCnt[wIdx] + 1;
    }
    if(ptrOcv05[wIdx] != 0)
    {
      pwOcvTable[wIdx] = pwOcvTable[wIdx] + ptrOcv05[wIdx];
      pbOcvCnt[wIdx]   = pbOcvCnt[wIdx] + 1;
      pwRTable[wIdx]   = pwRTable[wIdx] + ptrR05[wIdx];
      pbRCnt[wIdx]     = pbRCnt[wIdx] + 1;
    }

    wIdx = wIdx + 1;
  }

  wIdx = 0;
  while(wIdx < CAP_OCV_TABLE_ELEMENT_CNT)
  {
    pwOcvTable[wIdx] = (pbOcvCnt[wIdx] == 0) ? 0 : (pwOcvTable[wIdx] / pbOcvCnt[wIdx]);
    pwRTable[wIdx]   = (pbRCnt[wIdx] == 0)   ? 0 : (pwRTable[wIdx] / pbRCnt[wIdx]);
    GLOGD("[%s]: OCV,R Table[%d] = %d,%d / %d,%d\n", __func__, wIdx, pwOcvTable[wIdx], pwRTable[wIdx], pbOcvCnt[wIdx], pbRCnt[wIdx]);

    wIdx = wIdx + 1;
  }
}

/**
 * @brief CapAssignDsgTable
 *
 * Assign discharging table
 *
 * @para pwOcvTable pointer of OCV table
 * @para pwRTable pointer of R table
 * @return NULL
 */
STATIC GVOID CapAssignDsgTable(GU2 *pwOcvTable, GU2 *pwRTable)
{
  GU4 wTmp;
  GU2 wStep;
  GU2 wIdx;

  /// [AT-PM] : Calculate voltage step per percentage ; 07/04/2014
  wTmp = (GU4)(GGB->cChargeControl.scTerminationCfg.wTPVoltage);
  wTmp = wTmp - (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
  wTmp = wTmp * CAP_CONST_PERCENTAGE;
  wTmp = wTmp / CAP_CONST_RSOC_UNIT;
  wStep = (GU2)wTmp;

  /// [AT-PM] : Assign discharging table ; 08/12/2015
  wIdx = 0;
  while(1)
  {
    if(wIdx < (CAP_OCV_TABLE_ELEMENT_CNT - 1))
    {
      if(pwOcvTable[wIdx] == 0)
      {
        wTmp = (GU4)(CapTableOcvSocPtr[wIdx]);
        wTmp = wTmp * wStep;
        wTmp = wTmp / CAP_CONST_PERCENTAGE;
        wTmp = wTmp + (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
      }
      else
      {
        wTmp = (GU4)pwRTable[wIdx];
        wTmp = wTmp * (GGB->cSBSConfiguration.scData.wILMD);
        wTmp = wTmp / 10;
        wTmp = wTmp / 1000;
        wTmp = pwOcvTable[wIdx] - wTmp;
      }
      CAP->wDsgVoltTable[wIdx] = (GU2)wTmp;
    }
    else
    {
      CAP->wDsgVoltTable[wIdx] = (GU2)(GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
    }

    /// [AT-PM] : Limit the voltage smaller than TP voltage ; 08/28/2015
    if(CAP->wDsgVoltTable[wIdx] >= (GGB->cChargeControl.scTerminationCfg.wTPVoltage))
    {
      CAP->wDsgVoltTable[wIdx] = (GGB->cChargeControl.scTerminationCfg.wTPVoltage) - 
                                 wIdx;
    }

    wIdx = wIdx + 1;
    if(wIdx >= CAP_OCV_TABLE_ELEMENT_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapAssignChgTable
 *
 * Assign charging table
 *
 * @para pwOcvTable pointer of OCV table
 * @para pwRTable pointer of R table
 * @return NULL
 */
STATIC GVOID CapAssignChgTable(GU2 *pwOcvTable, GU2 *pwRTable)
{
  GU2 wIdx;
  GU4 wTmp;

  wIdx = 0;
  while(1)
  {
    if(pwOcvTable[wIdx] == 0)
    {
      wTmp = (GU4)CAP->wDsgVoltTable[wIdx];
      wTmp = wTmp + (GGB->cChargeControl.scTerminationCfg.wTPCurrent);
    }
    else
    {
      wTmp = (GU4)pwRTable[wIdx];
      wTmp = wTmp * (GGB->cChargeControl.scTerminationCfg.wTPCurrent);
      wTmp = wTmp / 1000;
      wTmp = pwOcvTable[wIdx] + wTmp;
    }

    if(wTmp >= (GGB->cChargeControl.scTerminationCfg.wTPVoltage))
    {
      wTmp = (GGB->cChargeControl.scTerminationCfg.wTPVoltage) - wIdx;
    }
    CAP->wChgVoltTable[wIdx] = (GU2)wTmp;

    wIdx = wIdx + 1;
    if(wIdx >= CAP_OCV_TABLE_ELEMENT_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapAssignStandbyTable
 *
 * Assign standby voltage table
 *
 * @para pwOcvTable pointer of OCV table
 * @return NULL
 */
STATIC GVOID CapAssignStandbyTable(GU2 *pwOcvTable)
{
  GU2 wIdx;
  GU4 wTmp;
  GU2 wStep;

  /// [AT-PM] : Calculate voltage step per percentage ; 07/04/2014
  wTmp = (GU4)(GGB->cChargeControl.scTerminationCfg.wTPVoltage);
  wTmp = wTmp - (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
  wTmp = wTmp * CAP_CONST_PERCENTAGE;
  wTmp = wTmp / CAP_CONST_RSOC_UNIT;
  wStep = (GU2)wTmp;

  wIdx = 0;
  while(1)
  {
    if(pwOcvTable[wIdx] == 0)
    {
      wTmp = (GU4)(CapTableOcvSocPtr[wIdx]);
      wTmp = wTmp * wStep;
      wTmp = wTmp / CAP_CONST_PERCENTAGE;
      wTmp = wTmp + (GGB->cGasGauging.scEDVCfg.wEdv1Voltage);
      CAP->wStandbyVoltTable[wIdx] = (GU2)wTmp;
    }
    else
    {
      CAP->wStandbyVoltTable[wIdx] = pwOcvTable[wIdx];
    }

    /// [AT-PM] : Limit the voltage smaller than TP voltage ; 08/28/2015
    if(CAP->wStandbyVoltTable[wIdx] >= 
       (GGB->cChargeControl.scTerminationCfg.wTPVoltage))
    {
      CAP->wStandbyVoltTable[wIdx] = (GGB->cChargeControl.scTerminationCfg.wTPVoltage) -
                                     wIdx;
    }

    wIdx = wIdx + 1;
    if(wIdx >= CAP_OCV_TABLE_ELEMENT_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapDumpVoltTable
 * 
 * Dump calculated voltage table
 *
 * @return NULL
 */
STATIC GVOID CapDumpVoltTable(GVOID)
{
  GU2 wIdx;

  wIdx = 0;
  while(1)
  {
    GLOGD("[%s]: VOLTAGE TABLE[%02d] at %3d = %d,%d,%d\n", __func__, 
          wIdx,
          CapTableOcvSocPtr[wIdx],
          CAP->wChgVoltTable[wIdx],
          CAP->wStandbyVoltTable[wIdx],
          CAP->wDsgVoltTable[wIdx]);

    wIdx = wIdx + 1;
    if(wIdx >= CAP_OCV_TABLE_ELEMENT_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapGenVoltTable
 *
 * Generate voltage table
 *
 * @return NULL
 */
GVOID CapGenVoltTable(GVOID)
{
  GU2 wIdx;
  GU2 wOcvTable[CAP_OCV_TABLE_ELEMENT_CNT];
  GU1 bOcvCnt[CAP_OCV_TABLE_ELEMENT_CNT];
  GU2 wRTable[CAP_OCV_TABLE_ELEMENT_CNT];
  GU1 bRCnt[CAP_OCV_TABLE_ELEMENT_CNT];

  /// [AT-PM] : Initialize variable ; 05/04/2015
  wIdx = 0;
  while(wIdx < CAP_OCV_TABLE_ELEMENT_CNT)
  {
    wOcvTable[wIdx] = 0;
    bOcvCnt[wIdx]   = 0;
    wRTable[wIdx]   = 0;
    bRCnt[wIdx]     = 0;

    wIdx = wIdx + 1;
  }

  /// [AT-PM] : Assign OCV and R from the average of GGB ; 05/04/2015
  CapAverageGgb(&wOcvTable[0], &bOcvCnt[0], &wRTable[0], &bRCnt[0]);

  /// [AT-PM] : Assign discharging table ; 08/12/2015
  CapAssignDsgTable(&wOcvTable[0], &wRTable[0]);

  /// [AT-PM] : Assign charging table ; 08/12/2015
  CapAssignChgTable(&wOcvTable[0], &wRTable[0]);

  /// [AT-PM] : Assign standby table ; 08/12/2015
  CapAssignStandbyTable(&wOcvTable[0]);

  CapDumpVoltTable();
}

/**
 * @brief CapInit
 *
 * Initialize capacity algorithm
 *
 * @return NULL
 */
GVOID CapInit(GVOID)
{
  GLOGD("%s[%s]: Reset capacity algorithm\n", CAP_BATT_LOG_HEADER, __func__);

  /// [AT-PM] : Generate voltage tablew ; 07/04/2014
  CapGenVoltTable();

  /// [AT-PM] : Get temperature index ; 04/17/2014
  CapGetTempIndex();

  /// [AT-PM] : Get SOC from OCV table ; 04/17/2014
  CapGetOcvSoc();
   
  /// [AT-PM] : Initialize RM and FCC ; 04/17/2014
  CapInitRMFcc();
}

/**
 * @brief CapInitQD
 *
 * Initialize QD buffer
 *
 * @return NULL
 */
GVOID CapInitQD(GVOID)
{
  GU4 dwTmp;
  GU1 idx;
  GU2 soc;

  idx = 0;
  while(1)
  {
    soc = CapTableOcvSocPtr[CapTableUpdateIdxPtr[idx + 1]] - 
          CapTableOcvSocPtr[CapTableUpdateIdxPtr[idx]];

    dwTmp = (GU4)soc;
    dwTmp = dwTmp * (CAP->wFcc);
    dwTmp = dwTmp / CAP_CONST_RSOC_UNIT;
    CAP->wQD[idx] = (GU2)dwTmp;
    CAP->wQDBuf[idx] = 0;
    GLOGD("%s[%s]: QD[%d] = (%d - %d)%% x %d = %d\n", CAP_BATT_LOG_HEADER, __func__,
          idx,
          CAP_CONST_RSOC_UNIT,
          soc,
          CAP->wFcc,
          CAP->wQD[idx]);

    idx = idx + 1;
    if(idx >= CAP_UPDATE_PTR_CNT)
    {
      break;
    }
  }
}

/**
 * @brief CapUpdate
 *
 * Update capacity information
 *
 * @return NULL
 */
GVOID CapUpdate(GVOID)
{
  GU4 dwTmp;

  /// [AT-PM] : Service RESET command ; 06/17/2015
  if(CAP->bCapCntl & CAP_CNTL_RESET)
  {
    CAP->bCapCntl = CAP->bCapCntl & (~CAP_CNTL_RESET);

    CapInit();
    CapInitQD();
    GLOGE("%s[%s]: Reset capacity\n", CAP_BATT_LOG_HEADER, __func__);
    return;
  }

  /// [AT-PM] : Service SET_FULL command ; 06/17/2015
  if(CAP->bCapCntl & CAP_CNTL_SET_FULL)
  {
    CAP->bCapCntl = CAP->bCapCntl & (~CAP_CNTL_SET_FULL);

    CAP->bChgState = CAP->bChgState | CAP_CHG_STATE_FULL_CHARGE;
    CAP->wDsgState = CAP->wDsgState | CAP_DSG_STATE_QD;
  
    CAP->wRM = CAP->wFcc;
    GLOGE("%s[%s]: Set capacity to full\n", CAP_BATT_LOG_HEADER, __func__);
    return;
  }

  /// [AT-PM] : Service SET_RSOC command ; 08/26/2015
  if(CAP->bCapCntl & CAP_CNTL_SET_RSOC)
  {
    CAP->bCapCntl = CAP->bCapCntl & (~CAP_CNTL_SET_RSOC);

    dwTmp = (GU4)CAP->wExtRsoc;
    dwTmp = dwTmp * CAP->wFcc;
    dwTmp = dwTmp / CAP_CONST_PERCENTAGE;
    CAP->wRM = (GWORD)dwTmp;
    GLOGE("%s[%s]: Set capacity to %d (%d/%d)\n", CAP_BATT_LOG_HEADER, __func__,
           CAP->wExtRsoc, CAP->wRM, CAP->wFcc);
    return;
  }

  /// [AT-PM] : Get temperature index ; 04/17/2014
  CapGetTempIndex();

  /// [AT-PM] : Get SOC from OCV table ; 04/17/2014
  CapGetOcvSoc();

  CAP->wRM  = (GU2)SBS->wRM;
  CAP->wRMLast = CAP->wRM;
  CAP->wFcc = (GU2)SBS->wFCC;
  GLOGD("%s[%s]: Synchronize RM and FCC from SBS data (%d/%d)\n", CAP_BATT_LOG_HEADER, __func__,
        CAP->wRM,
        CAP->wFcc);

  /// [AT-PM] : Update RM from coulomb counter ; 04/18/2014
  CapCalRMFromCC();
  GLOGD("%s[%s]: Update RM and FCC from coulomb counter (%d/%d)\n", CAP_BATT_LOG_HEADER, __func__,
        CAP->wRM,
        CAP->wFcc);

  if(UpiGauge.bState == STATE_FULL_CHARGED)
  {
    CAP->bChgState = CAP->bChgState | CAP_CHG_STATE_FULL_CHARGE;
    CAP->wDsgState = CAP->wDsgState | CAP_DSG_STATE_QD;

    CapStepFull();
  }
  else if(UpiGauge.bState == STATE_FULL_DISCHARGED)
  {
    CAP->wDsgState = CAP->wDsgState | CAP_DSG_STATE_REACH_EDV0;

    /// [AT-PM] : Update QD if necessary ; 06/03/2014
    CapUpdateQD();

    CAP->bChgState = CAP->bChgState & (~CAP_CHG_STATE_FULL_CHARGE);
    CAP->wDsgState = CAP->wDsgState & (~CAP_DSG_STATE_QD);

    CapStepDsgRM(0);
  }
  else
  {
    CAP->bChgState = CAP->bChgState & (~CAP_CHG_STATE_FULL_CHARGE);

    if(UpiGauge.bState == STATE_CHARGING)
    {
      /// [AT-PM] : Reset discharge state ; 04/22/2014
      CapResetQD();

      CapNowChg();
    }
    else if(UpiGauge.bState == STATE_DISCHARGING)
    {
      CapNowDsg();
    }
    else
    {
      /// [AT-PM] : Reset discharge state ; 04/22/2014
      CapResetQD();

      CapNowStandby();
    }
  }
  GLOGD("%s[%s]: S(%x-%x-%x), IDX(%d-%d), SOC(%d=%d/%d), QD(%d-%d-%d-%d), QDB(%d-%d-%d-%d), VDT(%d-%d-%d-%d)\n", 
        CAP_BATT_LOG_HEADER, __func__,
        CAP->bCapCntl, CAP->bChgState, CAP->wDsgState,
        CAP->bTempIdx, CAP->bSocIdx,
        CAP->wRsoc10x, CAP->wRM, CAP->wFcc,
        CAP->wQD[3], CAP->wQD[2], CAP->wQD[1], CAP->wQD[0],
        CAP->wQDBuf[3], CAP->wQDBuf[2], CAP->wQDBuf[1], CAP->wQDBuf[0],
        CAP->lUpdateTimeMSec[3], CAP->lUpdateTimeMSec[2], CAP->lUpdateTimeMSec[1], CAP->lUpdateTimeMSec[0]);
}

/**
 * @brief CapUpdateSbs
 *
 * Update capacity data to SBS
 *
 * @return NULL
 */
GVOID CapUpdateSbs(GVOID)
{
  /// [AT-PM] : Update RM ; 04/17/2014
  SBS->wRM = (GWORD)CAP->wRM;

  /// [AT-PM] : Update FCC ; 04/17/2014
  SBS->wFCC = (GWORD)CAP->wFcc;

  /// [AT-PM] : Update RSOC ; 04/17/2014
  SBS->wRSOC = SBSCalSoc(SBS->wRM, SBS->wFCC);

  /// [AT-PM] : Update ASOS ; 04/17/2014
  SBS->wASOC = SBSCalSoc(SBS->wRM, GGB->cSBSConfiguration.scData.wILMD);

  GLOGD("%s[%s]: %d (%d) / %d (%d) = %d (%d)\n", CAP_BATT_LOG_HEADER, __func__,
        SBS->wRM,
        CAP->wRM,
        SBS->wFCC,
        CAP->wFcc,
        SBS->wRSOC,
        SBS->wASOC);
}

#ifdef  __cplusplus
}
#endif  ///< end of __cplusplus

