/**
 * @file I2C.c
 *
 * I2C Related Function
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
 * @brief  I2CInit()
 *
 *  Init the I2C module
 *
 * @return NULL
 *
 */
GVOID I2CInit(GVOID)
{

}


/**
 * @brief  I2CExeCmd()
 *
 *  Execute I2C Command
 *
 * @return NULL
 *
 */
GBOOL I2CExeCmd(GI2CCmdType *pI2CCmd)
{
  GBOOL bRet = GTRUE;
  GBYTE bRetry = 0;

  /// [AT-PM] : Initialize the status to no error ; 08/14/2015
  pI2CCmd->bStatus = I2C_STS_NO_ERR;

  while(1)
  {
  ///--------------------------------///
  /// Execute I2C Command
  ///--------------------------------///
		
    
    ///-------------------------------------------------------------------------------///
    /// 1. Mutex Lock
    ///-------------------------------------------------------------------------------/// 
    UpiMutexLock((GVOID *)&UpiI2CMutex);
    bRet = _I2cExeCmd(pI2CCmd);
	  	
    ///-------------------------------------------------------------------------------///
    /// 3. Mutex UnLock
    ///-------------------------------------------------------------------------------/// 
    UpiMutexUnLock((GVOID *)&UpiI2CMutex);
	  	
    if(bRet == GTRUE)
    {
      break;
    }

    /// [AT-PM] : No retry if slave is not existed ; 08/14/2015
    if(pI2CCmd->bStatus & I2C_STS_SLAVE_NOT_EXIST)
    {
      GLOGE("[%s]: %02x - Slave is not existed\n", __func__, pI2CCmd->bStatus);
      UpiGauge.wErrorStatus |= GAUGE_ERR_I2C_FAIL;
      return (GFALSE);
    }

    ///--------------------------------///
    /// Retry
    ///--------------------------------///
    #ifdef PLATFORM_ASUS_Z370KL_COVER
    msleep(50);
    #endif ///< end of PLATFORM_ASUS_Z370KL_COVER
    bRetry++;
    if(bRetry == I2C_MAX_RETRY_CNT)
    {
      UpiGauge.wErrorStatus|=GAUGE_ERR_I2C_FAIL;
      UpiGauge.wI2cErrCnt++;
      return bRet;
     }
  }
	
  UpiGauge.wI2cErrCnt = 0;
  UpiGauge.wErrorStatus&=~GAUGE_ERR_I2C_FAIL;
  return bRet;
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

