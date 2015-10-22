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
 * @brief CHGCheckFullChargedIn
 *
 *  Check Full Charged
 *
 * @return NULL
 *
 */
GVOID CHGCheckFCClear(GVOID)
{
	///------------------------------///
	/// 1. Check if FC is set
	///------------------------------///	
	if(!(SBS->wBS & FC_BS))
	{
		return;
	}

	///------------------------------///
	/// 2. Leave when under charging
	///------------------------------///		
	if(GGIsDischarging() == GFALSE)
	{
		return;
	}

	///------------------------------///
	/// 3. If RSOC < FC_CLEAR_RSOC, 
	///    then clear FC
	///------------------------------///		
	if(SBS->wBS & FC_BS)
	{
		if(SBS->wRSOC < FC_CLEAR_RSOC)
		{
			SBS->wBS &=~ (FC_BS | TCA_BS);
		}
	}
}

/**
 * @brief CHGCheckFC
 *
 *  Check Primary Charge Termination
 *
 * @return NULL
 *
 */
GBOOL CHGCheckFC(GVOID)
{
  ///-----------------------------------------------------------///
  /// 1. Leave when battery voltage less than taper voltage 
  ///-----------------------------------------------------------/// 
  if(CurMeas->wVCell1 < GGB->cChargeControl.scTerminationCfg.wTPVoltage)
  {
    UpiGauge.chgCtrl.iTaperTimer = GGB->cChargeControl.scTerminationCfg.wTPTime * 1000;
    return GFALSE;
  }

  ///-----------------------------------------------------------///
  /// 2. Leave when battery current more than taper current 
  ///-----------------------------------------------------------/// 
  if(CurMeas->sCurr > GGB->cChargeControl.scTerminationCfg.wTPCurrent )
  {
    UpiGauge.chgCtrl.iTaperTimer = GGB->cChargeControl.scTerminationCfg.wTPTime * 1000;
    return GFALSE;
  }

  ///-----------------------------------------------------------///
  /// 3. Leave when under discharging
  ///-----------------------------------------------------------/// 
  UpiGauge.chgCtrl.iTaperTimer = UpiGauge.chgCtrl.iTaperTimer - (GSHORT)(CurMeas->iDeltaTime);
  if(UpiGauge.chgCtrl.iTaperTimer > 0)
  {
    return GFALSE;
  }

  return GTRUE;
    
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

