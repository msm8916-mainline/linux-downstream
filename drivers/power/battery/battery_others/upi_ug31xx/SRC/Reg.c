/**
 * @file RegCtrl.c
 *
 * Register Related Function
 *
 * @version $Revision: 88 $
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#ifdef __cplusplus
extern "C" {
#endif ///< for __cplusplus

#include "UpiDef.h"


/**
 * @brief  I2CWriteByte()
 *
 *  Write 1 byte the I2C module
 *
 * @return NULL
 *
 */
GBYTE ReadRegByte(GBYTE bRegAddr)
{
	GI2CCmdType i2cCmd;
	GBYTE bReg;

	///-----------------------------///
	///  Prepare
	///-----------------------------///	
	i2cCmd.bSlaveAddr = I2C_SLAVE_ADDR;
	i2cCmd.bWtBuf[0] = bRegAddr;
	i2cCmd.bWtBuf[1] = I2C_SECURE_CODE;
	
	if(bRegAddr >= I2C_SECURE_ADDR)
	{
		i2cCmd.bWtCnt = 2;
	}
	else
	{
		i2cCmd.bWtCnt = 1;
	}
	
	i2cCmd.bRdCnt = 1;
	
	///-----------------------------///
	///  Do I2C Command
	///-----------------------------///
	if(I2CExeCmd(&i2cCmd) == GFALSE)
  {
    SBSCmd->errNo = ERR_NPP_RB_FAIL; 
  }

	///-----------------------------///
	///  Read Register
	///-----------------------------///	
	bReg = i2cCmd.bRdBuf[0];	

  UpiGauge.bRegArray[bRegAddr] = bReg;
	return bReg;
}

/**
 * @brief  ReadRegWord()
 *
 *  Read register word
 *
 * @return NULL
 *
 */
GWORD ReadRegWord(GBYTE bRegAddr)
{
	GI2CCmdType i2cCmd;
	GWORD wReg;
	GWORD wRegPre;
	GBYTE i;

	///-----------------------------///
	///  Prepare
	///-----------------------------///	
	i2cCmd.bSlaveAddr = I2C_SLAVE_ADDR;
	i2cCmd.bWtBuf[0] = bRegAddr;
	i2cCmd.bWtBuf[1] = I2C_SECURE_CODE;
	
	if(bRegAddr >= I2C_SECURE_ADDR)
	{
		i2cCmd.bWtCnt = 2;
	}
	else
	{
		i2cCmd.bWtCnt = 1;
	}	
	i2cCmd.bRdCnt = 2;
	
	///-------------------------------------///
	///  Do double buffer check
	///-------------------------------------///	
  wRegPre = 0xee;
	for(i = 0 ; i < I2C_MAX_RETRY_CNT; i++)
	{
		///-----------------------------///
		///  Do I2C Command
		///-----------------------------///
		if(I2CExeCmd(&i2cCmd) == GFALSE)
	  {
	    SBSCmd->errNo = ERR_NPP_RW_FAIL; 
	  }

		///-----------------------------///
		///  Read Register
		///-----------------------------///
		wReg = i2cCmd.bRdBuf[0]|i2cCmd.bRdBuf[1]<<8;
		
		if(i == 0)
		{
			wRegPre = wReg;
			continue;
		}
		if(wReg == wRegPre)
		{
		  SBSCmd->errNo = NO_ERR_PASS;
			break;
		}
		wRegPre = wReg;
	}
	
  UpiGauge.bRegArray[bRegAddr]     = (GBYTE)(wReg & 0x00ff);
  UpiGauge.bRegArray[bRegAddr + 1] = (GBYTE)(wReg >> 8);
	return wReg;
}

/**
 * @brief  ReadRegWord()
 *
 *  Read register word
 *
 * @return NULL
 *
 */
GVOID ReadRegPage(GBYTE bRegAddr, GBYTE *pData, GBYTE bCnt)
{
	GI2CCmdType i2cCmd;
	GBYTE bReg[4];
	GBYTE bRegPre[4];

	GBOOL bSame = GTRUE;
	
	GBYTE i;
	GBYTE j;

	///-----------------------------///
	///  Prepare
	///-----------------------------///	
	i2cCmd.bSlaveAddr = I2C_SLAVE_ADDR;
	i2cCmd.bWtBuf[0] = bRegAddr;
	i2cCmd.bWtBuf[1] = I2C_SECURE_CODE;
	
	if(bRegAddr >= I2C_SECURE_ADDR)
	{
		i2cCmd.bWtCnt = 2;
	}
	else
	{
		i2cCmd.bWtCnt = 1;
	}	
	i2cCmd.bRdCnt = bCnt;
	
	///-------------------------------------///
	///  Do double buffer check
	///-------------------------------------///	
	for(i = 0 ; i < I2C_MAX_RETRY_CNT; i++)
	{
		///-----------------------------///
		///  Do I2C Command
		///-----------------------------///
		if(I2CExeCmd(&i2cCmd) == GFALSE)
	  {
	    SBSCmd->errNo = ERR_NPP_RP_FAIL; 
	  }
		
		///-----------------------------///
		///  Read Register
		///-----------------------------///
		for(j = 0 ; j < bCnt ; j++)
		{
			bReg[j] = i2cCmd.bRdBuf[j];
		}
		
		if(i == 0)
		{
			for(j = 0 ; j < bCnt ; j++)
			{
				bRegPre[j] = bReg[j];
			}
			continue;
		}
		bSame = GTRUE;
		for(j = 0 ; j < bCnt ; j++)
		{
			if(bRegPre[j] != bReg[j])
			{
				bSame = GFALSE;				
			}
			bRegPre[j] = bReg[j];
		}
		if(bSame == GTRUE)
		{
   	  SBSCmd->errNo = NO_ERR_PASS;
			break;
		}
	}

	///-----------------------------///
	///  Return Data
	///-----------------------------///
	for(j = 0 ; j < bCnt ; j++)
	{
		pData[j] = bReg[j];

    UpiGauge.bRegArray[bRegAddr + j] = bReg[j];
	}
	
	return;
}

/**
 * @brief  I2CWriteByte()
 *
 *  Write 1 byte the I2C module
 *
 * @return NULL
 *
 */
GBOOL WriteRegByte(GBYTE bRegAddr, GBYTE bData)
{
	GBOOL bRet = GTRUE;

  GI2CCmdType i2cCmd;

  ///-----------------------------///
  ///  Prepare
  ///-----------------------------///	
  i2cCmd.bSlaveAddr = I2C_SLAVE_ADDR;
  i2cCmd.bWtBuf[0] = bRegAddr;

  if(bRegAddr >= I2C_SECURE_ADDR)
  {
  	 i2cCmd.bWtBuf[1] = I2C_SECURE_CODE;
  	 i2cCmd.bWtBuf[2] = bData;
  	 i2cCmd.bWtCnt = 3;
  }
  else
  {
  	 i2cCmd.bWtBuf[1] = bData;
  	 i2cCmd.bWtCnt = 2;
  }	
  i2cCmd.bRdCnt = 0;
  ///-----------------------------///
  ///  Do I2C Command
  ///-----------------------------///
  bRet = I2CExeCmd(&i2cCmd);
  if(bRet == GFALSE)
  {
		SBSCmd->errNo = ERR_NPP_WB_FAIL; 
  }

	return bRet;
}

/**
 * @brief  WriteRegWord()
 *
 *  Write 2 byte the I2C module
 *
 * @return NULL
 *
 */
GBOOL WriteRegWord(GBYTE bRegAddr, GWORD bData)
{
	 GBOOL bRet = GTRUE;
	 bRet = WRITE_REG(bRegAddr, GLOBYTE(bData));
	 if(bRet == GFALSE)
	 {
		 return bRet;
	 }
 	 bRet = WRITE_REG(bRegAddr + 1, GHIBYTE(bData));
	 return bRet;
}

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

