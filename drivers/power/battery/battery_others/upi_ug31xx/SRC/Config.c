/**
 * @file Config.c
 *
 *  Config Save, Load
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
 * @brief CFGSaveToFile
 *
 * Save data to file
 *
 * @para buf    address of data buffer
 * @para offset start address
 * @para size   data size
 * @return GTRUE if success
 */
GBOOL CFGSaveToFile(GCHAR *buf, GWORD offset, GWORD size)
{
  GWORD idx;
  GCHAR readBuf;
  GBOOL rtn;

  idx = 0;
  while(1)
  {
    /// [AT-PM] : Check data needs to be updated ; 04/21/2015
    rtn = _CFGReadByte(offset + idx, &readBuf);
    if(rtn != GTRUE)
    {
      GLOGE("[%s]: Read data at offset = %02x from file failed\n", __func__, offset + idx);
    }

    if((readBuf != buf[idx]) || (rtn == GFALSE))
    {
      rtn = _CFGWriteByte(offset + idx, buf[idx]);
      if(rtn != GTRUE)
      {
        GLOGE("[%s]: Write %02x to offset = %02x to file failed\n", __func__, buf[idx], offset + idx);
        return (GFALSE);
      }
    }

    idx = idx + 1;
    if(idx >= size)
    {
      break;
    }
  }
  return (GTRUE);
}

/**
 * @brief CFGLoadFromFile 
 *
 * Load data from file
 *
 * @para buf    address of data buffer
 * @para offset start address
 * @para size   data size
 * @return GTRUE if success
 */
GBOOL CFGLoadFromFile(GCHAR *buf, GWORD offset, GWORD size)
{
  GWORD idx;
  GBOOL rtn;
  GCHAR readBuf;

  idx = 0;
  while(1)
  {
    rtn = _CFGReadByte(offset + idx, &readBuf);
    if(rtn != GTRUE)
    {
      GLOGE("[%s]: Read data from offset = %02x failed\n", __func__, idx + offset);
      return (GFALSE);
    }

    buf[idx] = readBuf;

    idx = idx + 1;
    if(idx >= size)
    {
      break;
    }
  }
  return (GTRUE);
}

/**
 * @brief CFGCalculateChecksum
 *
 * Calculate checksum of CFG data
 *
 * @return checksum
 */
STATIC GWORD CFGCalculateChecksum(GVOID)
{
  GBYTE *pPtr;
  GU2    wSize;
  GWORD  wChecksum;

  pPtr = (GBYTE *)CFG;
  wSize = 0;
  wChecksum = 0;

  while(1)
  {
    GLOGD("[%s]: wChecksum = %04x + %02x\n", __func__, wChecksum, (*pPtr));
    wChecksum = wChecksum + (*pPtr);
    pPtr = pPtr + 1;
    wSize = wSize + 1;

    if(wSize >= (CFG->wDataSize - 2))
    {
      break;
    }
  }

  wChecksum = wChecksum - (GU1)CFG->cBoardOffsetSign;
  wChecksum = wChecksum - (GU1)(CFG->sBoardOffsetValue % 256);
  wChecksum = wChecksum - (GU1)(CFG->sBoardOffsetValue / 256);
  wChecksum = wChecksum - CFG->bBoardOffsetSum;
  wChecksum = wChecksum - (GU1)CFG->sBoardOffsetComp;
  GLOGE("[%s]: wChecksum = %04x\n", __func__, wChecksum);
  return (wChecksum);
}

/**
 * @brief CFGCheckData
 *
 * Check data load from file is correct
 *
 * @return GTRUE if success
 */
STATIC GBOOL CFGCheckData(GVOID)
{
  GWORD  wChecksum;

  wChecksum = CFGCalculateChecksum();
  if(wChecksum != CFG->wChecksum)
  {
    GLOGE("[%s]: Checksum from EEPROM = %04x != %04x\n", __func__, CFG->wChecksum, wChecksum);
    return (GFALSE);
  }

  if(CFG->wFcc == 0)
  {
    GLOGE("[%s]: FCC from EEPROM = %d == 0\n", __func__, CFG->wFcc);
    return (GFALSE);
  }

  if((CFG->wQD0 == 0) || (CFG->wQD1 == 0) || (CFG->wQD2 == 0) || (CFG->wQD3 == 0))
  {
    GLOGE("[%s]: QD from EEPROM = %d,%d,%d,%d == 0\n", __func__, CFG->wQD0, CFG->wQD1, CFG->wQD2, CFG->wQD3);
    return (GFALSE);
  }
  return (GTRUE);
}

/**
 * @brief CFGLoadBoardOffset
 *
 * Load board offset
 *
 * @return NULL
 */
STATIC GVOID CFGLoadBoardOffset(GVOID)
{
  GU1 bChecksum;

  GLOGE("[%s]: Board offset = %c%d,%02x+%02x\n", __func__, 
        CFG->cBoardOffsetSign, 
        CFG->sBoardOffsetValue, 
        CFG->bBoardOffsetSum,
        CFG->sBoardOffsetComp);

  bChecksum = 0;

  /// [AT-PM] : Check signature ; 04/20/2015
  if((CFG->cBoardOffsetSign != CONFIG_BOARD_OFFSET_SIGN) &&
     (CFG->cBoardOffsetSign != CONFIG_BOARD_OFFSET_LSIGN))
  {
    GLOGE("[%s]: Incorrect signature = %c\n", __func__, CFG->cBoardOffsetSign);
    return;
  }

  /// [AT-PM] : Check checksum ; 04/20/2015
  bChecksum = bChecksum + (GU1)CFG->cBoardOffsetSign;
  bChecksum = bChecksum + (GU1)(CFG->sBoardOffsetValue % 256);
  bChecksum = bChecksum + (GU1)(CFG->sBoardOffsetValue / 256);
  if(bChecksum != CFG->bBoardOffsetSum)
  {
    GLOGE("[%s]: Checksum from EEPROM = %02x != %02x\n", __func__, CFG->bBoardOffsetSum, bChecksum);
    return;
  }

  if(CFG->cBoardOffsetSign == CONFIG_BOARD_OFFSET_LSIGN)
  {
    CFG->sBoardOffsetValue = CFG->sBoardOffsetValue + CFG->sBoardOffsetComp;
  }
  GGB->cCalibration.scCaliData.sAdc1PosOffset = (GSHORT)CFG->sBoardOffsetValue;
}

/**
 * @brief CFGRecoverCapData
 *
 * Recover capacity data
 *
 * @return NULL
 */
STATIC GVOID CFGRecoverCapData(GVOID)
{
  SBS->wRM  = (GWORD)CFG->wRM;
  SBS->wFCC = (GWORD)CFG->wFcc;
  SBS->wRSOC = SBSCalSoc(SBS->wRM, SBS->wFCC);

  CurMeas->iCurCap = (GINT64)CFG->iCurCap;

  UpiGauge.tmRes.year = (GSHORT)CFG->wYear;
  UpiGauge.tmRes.mon  = (GSHORT)CFG->bMonth;
  UpiGauge.tmRes.day  = (GSHORT)CFG->bDay;
  UpiGauge.tmRes.hour = (GSHORT)CFG->bHour;
  UpiGauge.tmRes.min  = (GSHORT)CFG->bMin;

  CAP->wQD[0] = (GU2)CFG->wQD0;
  CAP->wQD[1] = (GU2)CFG->wQD1;
  CAP->wQD[2] = (GU2)CFG->wQD2;
  CAP->wQD[3] = (GU2)CFG->wQD3;
}

/**
 * @brief CFGCalDataSize
 *
 * Calculate config data size
 *
 * @return NULL
 */
STATIC GVOID CFGCalDataSize(GVOID)
{
  GU1 idx;

  CFG->wDataSize = 0;
  idx = 0;
  while(1)
  {
    CFG->wDataSize = CFG->wDataSize + CfgMapTablePtr[idx].size;

    idx = idx + 1;
    if(CfgMapTablePtr[idx].size == 0)
    {
      break;
    }
  }
  GLOGE("[%s]: Total config data size = %d\n", __func__, CFG->wDataSize);
}

/**
 * @brief CFGInit
 *
 *  Config Variable init
 *
 * @return NULL
 *
 */
GVOID CFGInit(GVOID)
{
  UPI_MEMSET(CFG, 0, sizeof(GConfigDataType));

  UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus & (~GAUGE_STATUS_CONFIG_LOADED);

  CFGCalDataSize();

  if(CFGLoadFromFile((GCHAR *)CFG, CfgMapTablePtr[0].addr, (GWORD)CFG->wDataSize) == GTRUE)
  {
    GLOGE("[%s]: Config Data -> R:%d,F:%d,LC:%d,Y:%d,M:%d,D:%d,HR:%d,MIN:%d,QD:%d-%d-%d-%d,CS:%04x\n", __func__,
          CFG->wRM, CFG->wFcc, CFG->iCurCap,
          CFG->wYear, CFG->bMonth, CFG->bDay, CFG->bHour, CFG->bMin,
          CFG->wQD0, CFG->wQD1, CFG->wQD2, CFG->wQD3,
          CFG->wChecksum);

    /// [AT-PM] : Load board offset ; 04/20/2015
    CFGLoadBoardOffset();

    /// [AT-PM] : Check checksum ; 04/20/2015
    if(CFGCheckData() == GFALSE)
    {
      return;
    }

    /// [AT-PM] : Recover capacity data if IC data is not available ; 04/20/2015
    CFGRecoverCapData();

    UpiGauge.wGaugeStatus = UpiGauge.wGaugeStatus | GAUGE_STATUS_CONFIG_LOADED;
  }
}

/**
 * @brief CFGPrepareData
 *
 * Prepare data for CONFIG file
 *
 * @return NULL
 */
GVOID CFGPrepareData(GVOID)
{
  /// [AT-PM] : Fetch data ; 04/20/2015
  CFG->wRM     = (GU2)SBS->wRM;
  CFG->wFcc    = (GU2)SBS->wFCC;
  CFG->iCurCap = (GSHORT)CurMeas->iCurCap;

  CFG->wYear   = (GU2)UpiGauge.tmConfig.year;
  CFG->bMonth  = (GU1)UpiGauge.tmConfig.mon;
  CFG->bDay    = (GU1)UpiGauge.tmConfig.day;
  CFG->bHour   = (GU1)UpiGauge.tmConfig.hour;
  CFG->bMin    = (GU1)UpiGauge.tmConfig.min;

  CFG->wQD0    = (GU2)CAP->wQD[0];
  CFG->wQD1    = (GU2)CAP->wQD[1];
  CFG->wQD2    = (GU2)CAP->wQD[2];
  CFG->wQD3    = (GU2)CAP->wQD[3];

  /// [AT-PM] : Calcualte checksum ; 04/20/2015
  CFG->wChecksum = (GWORD)CFGCalculateChecksum();
}

/**
 * @brief CFGUpdate
 *
 * Update config file
 *
 * @return NULL
 */
GVOID CFGUpdate(GVOID)
{
  GBOOL rtn;

  UpiGauge.wWarningStatus = UpiGauge.wWarningStatus & (~GAUGE_WARN_CONFIG_SAVE_FAIL);

  /// [AT-PM] : Save RM ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->wRM, CfgMapTablePtr[CONFIG_MAP_IDX_RM].addr, CfgMapTablePtr[CONFIG_MAP_IDX_RM].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wRM = %d fail\n", __func__, CFG->wRM);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }

  /// [AT-PM] : Save FCC ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->wFcc, CfgMapTablePtr[CONFIG_MAP_IDX_FCC].addr, CfgMapTablePtr[CONFIG_MAP_IDX_FCC].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wFcc = %d fail\n", __func__, CFG->wFcc);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }

  /// [AT-PM] : Save CurCap ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->iCurCap, CfgMapTablePtr[CONFIG_MAP_IDX_CUR_CAP].addr, CfgMapTablePtr[CONFIG_MAP_IDX_CUR_CAP].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->iCurCap = %d fail\n", __func__, CFG->iCurCap);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }

  /// [AT-PM] : Save time tag ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->wYear, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_YEAR].addr, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_YEAR].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wYear = %d fail\n", __func__, CFG->wYear);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->bMonth, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_MONTH].addr, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_MONTH].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->bMonth = %d fail\n", __func__, CFG->bMonth);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->bDay, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_DAY].addr, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_DAY].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->bDay = %d fail\n", __func__, CFG->bDay);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->bHour, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_HOUR].addr, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_HOUR].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->bHour = %d fail\n", __func__, CFG->bHour);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->bMin, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_MIN].addr, CfgMapTablePtr[CONFIG_MAP_IDX_TIME_MIN].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->bMin = %d fail\n", __func__, CFG->bMin);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }

  /// [AT-PM] : Save QD ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->wQD0, CfgMapTablePtr[CONFIG_MAP_IDX_QD_0].addr, CfgMapTablePtr[CONFIG_MAP_IDX_QD_0].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wQD0 = %d fail\n", __func__, CFG->wQD0);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->wQD1, CfgMapTablePtr[CONFIG_MAP_IDX_QD_1].addr, CfgMapTablePtr[CONFIG_MAP_IDX_QD_1].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wQD1 = %d fail\n", __func__, CFG->wQD1);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->wQD2, CfgMapTablePtr[CONFIG_MAP_IDX_QD_2].addr, CfgMapTablePtr[CONFIG_MAP_IDX_QD_2].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wQD2 = %d fail\n", __func__, CFG->wQD2);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
  rtn = CFGSaveToFile((GCHAR *)&CFG->wQD3, CfgMapTablePtr[CONFIG_MAP_IDX_QD_3].addr, CfgMapTablePtr[CONFIG_MAP_IDX_QD_3].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wQD3 = %d fail\n", __func__, CFG->wQD3);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }

  /// [AT-PM] : Save checksum ; 04/20/2015
  rtn = CFGSaveToFile((GCHAR *)&CFG->wChecksum, CfgMapTablePtr[CONFIG_MAP_IDX_CHECKSUM].addr, CfgMapTablePtr[CONFIG_MAP_IDX_CHECKSUM].size);
  if(rtn != GTRUE)
  {
    GLOGE("[%s]: Save CFG->wChecksum = %d fail\n", __func__, CFG->wChecksum);
    UpiGauge.wWarningStatus = UpiGauge.wWarningStatus | GAUGE_WARN_CONFIG_SAVE_FAIL;
    return;
  }
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

