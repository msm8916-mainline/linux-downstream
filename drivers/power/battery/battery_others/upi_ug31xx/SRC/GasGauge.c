/**
 * @file ChargeCtl.c
 *
 *  Charge Control
 *
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
 * @brief GGCheckFD
 *
 *  Check Full Discharged
 *
 * @return NULL
 *
 */
GBOOL GGCheckFD(GVOID)
{
  ///-----------------------------------------------------------///
  /// 1. Leave when under discharging
  ///-----------------------------------------------------------///
  if(GGIsDischarging() == GFALSE)
  {
    return GFALSE;
  }
  
  ///-----------------------------------------------------------///
  ///  Check FD set
  ///-----------------------------------------------------------///
  if(SBS->wBS & FD_BS)
  {
    return GTRUE;
  }   

  ///-----------------------------------------------------------///
  /// 2. Leave when under discharging
  ///-----------------------------------------------------------/// 
  if(CurMeas->wVCell1 >= GGB->cGasGauging.scEDVCfg.wEdv1Voltage)
  {
    return GFALSE;
  } 

  SBS->wBS |= FD_BS;
  
  return GTRUE; 
}

/**
 * @brief GGCheckFDClear
 *
 * Clear FD if voltage is higher than EDVF
 *
 * @return NULL
 */
GVOID GGCheckFDClear(GVOID)
{
  if(CurMeas->wVCell1 > GGB->cGasGauging.scEDVCfg.wEdv1Voltage)
  {
    SBS->wBS &= ~FD_BS;
  }
}

/**
 * @brief GGIsDischarging
 *
 *  Check if discahrging
 *
 * @return NULL
 *
 */
GBOOL GGIsDischarging(GVOID)
{
  if(CurMeas->sCurr > 0)
  {
    return GFALSE;
  }
  return GTRUE; 
}

/**
 * @brief GGGetState
 *
 *  Get Current State
 *
 * @return NULL
 *
 */
GVOID GGGetState(GVOID)
{ 
  if(SBS->wRM == SBS->wFCC)
  {
    UpiGauge.bState = STATE_FULL_CHARGED;
  }

  if(UPI_ABS(CurMeas->sCurr) < GGB->cGasGauging.scCurrentThresholds.wStandbyCurrent)
  {
    UpiGauge.chgCtrl.iTaperTimer = GGB->cChargeControl.scTerminationCfg.wTPTime * 1000;
    if(UpiGauge.bPreState  == STATE_FULL_CHARGED)
    {
      UpiGauge.bState = STATE_FULL_CHARGED;
    }
    else if (UpiGauge.bPreState  == STATE_FULL_DISCHARGED)
    {
      UpiGauge.bState = STATE_FULL_DISCHARGED;
    }
    else
    {
      UpiGauge.bState = STATE_STANDBY;
    }
  }
  else if(CurMeas->sCurr <= (GGB->cGasGauging.scCurrentThresholds.wStandbyCurrent * -1))
  {
    UpiGauge.chgCtrl.iTaperTimer = GGB->cChargeControl.scTerminationCfg.wTPTime * 1000;
    if (UpiGauge.bPreState == STATE_FULL_DISCHARGED)
    {
      UpiGauge.bState = STATE_FULL_DISCHARGED;
    }
    else
    {
      if(GGCheckFD()==GTRUE)
      {
        UpiGauge.bState = STATE_FULL_DISCHARGED;
      }
      else
      {
        UpiGauge.bState = STATE_DISCHARGING;
      }
    }
  }
  else if(CurMeas->sCurr >= (GGB->cGasGauging.scCurrentThresholds.wStandbyCurrent))
  {
    if (UpiGauge.bPreState == STATE_FULL_CHARGED)
    {
      UpiGauge.bState = STATE_FULL_CHARGED;
    }
    else
    {
      if(CHGCheckFC()==GTRUE)
      {
        UpiGauge.bState = STATE_FULL_CHARGED;   
      }
      else
      {
        UpiGauge.bState = STATE_CHARGING;
      }
    }
  }
}

/**
 * @brief GGStateChgToChg
 *
 *  From Charge to Charge State
 *
 * @return NULL
 *
 */
GVOID GGStateFullChgToDsg(GVOID)
{
  ///----------------------------------------------------------///
  ///
  ///----------------------------------------------------------///
  CHGCheckFCClear();

  ///----------------------------------------------------------///
  ///
  ///----------------------------------------------------------///
  
}

/**
 * @brief GGStateChgToChg
 *
 *  From Charge to Charge State
 *
 * @return NULL
 *
 */
GVOID GGStateDsgToDsg(GVOID)
{
  CHGCheckFCClear();
}

/**
 * @brief GGStateDsgToFullDsg
 *
 *  From Charge to Charge State
 *
 * @return NULL
 *
 */
GVOID GGStateDsgToFullDsg(GVOID)
{

}

/**
 * @brief GGStateChgToChg
 *
 *  From Charge to Charge State
 *
 * @return NULL
 *
 */
GVOID GGStateChgToChg(GVOID)
{
  GGCheckFDClear();
}

/**
 * @brief GGFullCharged
 *
 * From Full Charge to Full Charge state
 *
 * @return NULL
 */
GVOID GGFullCharged(GVOID)
{
  CHGCheckFCClear();
}

/**
 * @brief GGGetState
 *
 *  Get Current State
 *
 * @return NULL
 *
 */
GVOID GGDispatchState(GVOID)
{
  ///---------------------------------------------///
  /// 1. FULL_CHARGED --> DISCHARGING
  ///---------------------------------------------///
  if((UpiGauge.bPreState == STATE_FULL_CHARGED) &&
     (UpiGauge.bState    == STATE_DISCHARGING))
  {
    GGStateFullChgToDsg();
  }
  
  ///---------------------------------------------///
  /// 2. DISCHARGING --> DISCHARGING
  ///---------------------------------------------/// 
  else if((UpiGauge.bPreState == STATE_DISCHARGING) &&
         (UpiGauge.bState     == STATE_DISCHARGING))
  {
    GGStateDsgToDsg();
  }

  ///---------------------------------------------///
  /// 3. DISCHARGING --> FULL_DISCHARGED
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_DISCHARGING) &&
         (UpiGauge.bState     == STATE_FULL_DISCHARGED))
  {
    GGStateDsgToFullDsg();
  }

  ///---------------------------------------------///
  /// 4. DISCHARGING --> CHARGING
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_DISCHARGING) &&
         (UpiGauge.bState     == STATE_CHARGING))
  {
//    GGTurnToCharging()
  } 

  ///---------------------------------------------///
  /// 5. CHARGING --> DISCHARGING
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_CHARGING) &&
         (UpiGauge.bState     == STATE_DISCHARGING))
  {
//    GGTurnToDischarging()
  }   

  ///---------------------------------------------///
  /// 6. CHARGING --> CHARGING
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_CHARGING) &&
         (UpiGauge.bState     == STATE_CHARGING))
  {
    GGStateChgToChg();
  }     

  ///---------------------------------------------///
  /// 7. CHARGING --> FULL_CHARGED
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_CHARGING) && 
         (UpiGauge.bState     == STATE_FULL_CHARGED))
  {
//    GGFullCharged()
  }   
  ///---------------------------------------------///
  /// 8. FULL_CHARGED --> FULL_CHARGED
  ///---------------------------------------------/// 
  else if((UpiGauge.bPreState == STATE_FULL_CHARGED) && 
         (UpiGauge.bState     == STATE_FULL_CHARGED))
  {
    GGFullCharged();
  }   

  ///---------------------------------------------///
  /// 3. FULL_DISCHARGED --> FULL_DISCHARGED
  ///---------------------------------------------///
  else if((UpiGauge.bPreState == STATE_FULL_DISCHARGED) &&
         (UpiGauge.bState     == STATE_FULL_DISCHARGED))
  {
//    GGEndDischarging()
  } 

  ///---------------------------------------------///
  /// 9. STANDBY
  ///---------------------------------------------/// 
  else if(UpiGauge.bState  == STATE_STANDBY)
  {
    GGFullCharged();
  }     
  
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

