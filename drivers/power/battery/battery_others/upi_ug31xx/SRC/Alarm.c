/**
 * @file Alarm.c
 *
 * Alarm Related Function
 *
 * @version $Revision$
 * @author AllenTeng <allen.kuoliang.teng@gmail.com>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#include "UpiDef.h"

/**
 * @brief AlarmUV
 *
 * UV alarm procedure
 *
 * @return NULL
 */
GVOID AlarmUV(GVOID)
{
  GBYTE bReg;
  GWORD wUVThrd;
  GWORD wOVThrd;

  if(!(SBS->wAlarmSts & ALARM_STS_UV_EN))
  {
    GLOGD("UV alarm is disabled\n");
    return;
  }

  /// [AT-PM] : Set threshold according to alarm status ; 03/30/2015
  switch(SBS->wAlarmSts & ALARM_STS_MASK_VOLT)
  {
    case (ALARM_STS_UV | ALARM_STS_OV):
      wOVThrd = ALARM_THRD_MAX_VALUE;
      wUVThrd = MeasRevertVCell1((GWORD)GGB->c1stLevelSafetyClass.scVoltage.sUvAlarm);
      GLOGD("[%s] : Set OV = %d and UV = %d for abnormal status (%04x)\n", __func__, wOVThrd, wUVThrd, SBS->wAlarmSts);
      break;
    case (ALARM_STS_UV):
      wOVThrd = MeasRevertVCell1((GWORD)GGB->c1stLevelSafetyClass.scVoltage.sUvRelease);
      wUVThrd = ALARM_THRD_MIN_VALUE;
      GLOGD("[%s] : Set OV = %d and UV = %d for UV status (%04x)\n", __func__, wOVThrd, wUVThrd, SBS->wAlarmSts);
      break;
    case (ALARM_STS_OV):
      wOVThrd = ALARM_THRD_MAX_VALUE;
      wUVThrd = MeasRevertVCell1((GWORD)GGB->c1stLevelSafetyClass.scVoltage.sUvAlarm);
      GLOGD("[%s] : Set OV = %d and UV = %d for OV status (%04x)\n", __func__, wOVThrd, wUVThrd, SBS->wAlarmSts);
      break;
    default:
      wOVThrd = ALARM_THRD_MAX_VALUE;
      wUVThrd = MeasRevertVCell1((GWORD)GGB->c1stLevelSafetyClass.scVoltage.sUvAlarm);
      GLOGD("[%s] : Set OV = %d and UV = %d for normal status (%04x)\n", __func__, wOVThrd, wUVThrd, SBS->wAlarmSts);
      break;
  }
  WRITE_REG16(REG16_OV1, wOVThrd);
  WRITE_REG16(REG16_UV1, wUVThrd);

  /// [AT-PM] : Reset alarm function ; 03/30/2015
  bReg = READ_REG(REG_ALARM_EN);
  bReg = bReg & (~ALARM_EN_V1_ALARM_EN);
  WRITE_REG(REG_ALARM_EN, bReg);
  bReg = READ_REG(REG_ALARM_EN);
  bReg = bReg | ALARM_EN_V1_ALARM_EN;
  WRITE_REG(REG_ALARM_EN, bReg);
}

/**
 * @brief AlarmSts
 *
 * Get alarm status from register
 *
 * @return NULL
 */
GVOID AlarmSts(GVOID)
{
  GBYTE bReg;

  SBS->wAlarmSts = SBS->wAlarmSts & (~ALARM_STS_OV);
  
  bReg = READ_REG(REG_ALARM2_STATUS);
  if(bReg & ALARM2_STATUS_UV1_ALARM)
  {
    SBS->wAlarmSts = SBS->wAlarmSts & (~ALARM_STS_MASK_VOLT);
    SBS->wAlarmSts = SBS->wAlarmSts | ALARM_STS_UV;    
  }
  if(bReg & ALARM2_STATUS_OV1_ALARM)
  {
    SBS->wAlarmSts = SBS->wAlarmSts & (~ALARM_STS_MASK_VOLT);
    SBS->wAlarmSts = SBS->wAlarmSts | ALARM_STS_OV; 
  }

 }


#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

