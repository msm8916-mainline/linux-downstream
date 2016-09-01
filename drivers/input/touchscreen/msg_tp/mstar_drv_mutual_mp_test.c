////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_mutual_mp_test.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.2.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_mutual_mp_test.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_mutual_fw_control.h"
#include "mstar_drv_platform_porting_layer.h"

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
#ifdef CONFIG_ENABLE_ITO_MP_TEST

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

static u32 _gIsInMpTest = 0;
static u32 _gTestRetryCount = CTP_MP_TEST_RETRY_COUNT;
static ItoTestMode_e _gItoTestMode = 0;

static s32 _gCtpMpTestStatus = ITO_TEST_UNDER_TESTING;
//zxz add rawdata
static u16 _gSenseLineNum = 0;
static u16 _gDriveLineNum = 0;

static struct work_struct _gCtpItoTestWork;
static struct workqueue_struct *_gCtpMpTestWorkQueue = NULL;
 
static s32 _gDeltaC[MAX_MUTUAL_NUM] = {0};
static s32 _gResult[MAX_MUTUAL_NUM] = {0};
static u8 _gMode[MAX_MUTUAL_NUM] = {0};
//zxz add rawdata
static u8 _gTestFailChannel[MAX_MUTUAL_NUM] = {0};
static u32 _gTestFailChannelCount = 0;

TestScopeInfo_t g_TestScopeInfo = {0};

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

static u16 _DrvMpTestItoOpenTestFirmwareGetState(void)
{
    u16 nCheckState = 0;

    DBG("*** %s() ***\n", __func__);

    nCheckState = RegGet16BitValue(0x3CDE); //bank:reg_PIU_MISC_0, addr:h006f

    return nCheckState;
}

static void _DrvMpTestItoOpenTestMcuStop(void)
{
//zxz add rawdata
    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x0FE6, 0x0001); //bank:mheg5, addr:h0073
}

static void _DrvMpTestItoOpenTestAnaSwitchToMutual(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x114A); //bank:ana, addr:h0025
    nTemp |= BIT0;
    RegSet16BitValue(0x114A, nTemp);
    nTemp = RegGet16BitValue(0x1116); //bank:ana, addr:h000b
    nTemp |= (BIT2 | BIT0);
    RegSet16BitValue(0x1116, nTemp);
}

static u16 _DrvMpTestItoOpenTestAnaGetMutualChannelNum(void)
{
    u16 nSenseLineNum = 0;
    u16 nRegData = 0;

    DBG("*** %s() ***\n", __func__);

    nRegData = RegGet16BitValue(0x102E); //bank:ana3, addr:h0017
    nSenseLineNum = nRegData & 0x000F;

    DBG("nSenseLineNum = %d\n", nSenseLineNum);

    return nSenseLineNum;
}

static u16 _DrvMpTestItoOpenTestAnaGetMutualSubFrameNum(void)
{
    u16 nDriveLineNum = 0;
    u16 nRegData = 0;

    DBG("*** %s() ***\n", __func__);

    nRegData = RegGet16BitValue(0x1216); //bank:ana2, addr:h000b
    nDriveLineNum = ((nRegData & 0xFF00) >> 8) + 1; //Since we only retrieve 8th~12th bit of reg_m_sf_num, 0xFF00 shall be changed to 0x1F00. 

    DBG("nDriveLineNum = %d\n", nDriveLineNum);

    return nDriveLineNum;
}

static void _DrvMpTestItoOpenTestAnaGetMutualCSub(u8 *pMode)
{
    u16 i, j;
    u16 nSenseLineNum;
    u16 nDriveLineNum;
    u16 nTotalNum;
    u8 szDataAna4[3];    
    u8 szDataAna3[3];    
    u8 szDataAna41[ANA4_MUTUAL_CSUB_NUMBER]; //200 = 392 - 192  
    u8 szDataAna31[ANA3_MUTUAL_CSUB_NUMBER]; //192 = 14 * 13 + 10   
    u8 szModeTemp[MAX_MUTUAL_NUM];

    DBG("*** %s() ***\n", __func__);

    nTotalNum = MAX_MUTUAL_NUM;
    nSenseLineNum = _DrvMpTestItoOpenTestAnaGetMutualChannelNum();
    nDriveLineNum = _DrvMpTestItoOpenTestAnaGetMutualSubFrameNum();

    if (ANA4_MUTUAL_CSUB_NUMBER > 0)
    {
        mdelay(100);
        for (i = 0; i < ANA4_MUTUAL_CSUB_NUMBER; i ++)
        {	
            szDataAna41[i] = 0;
        }

        szDataAna4[0] = 0x10;
        szDataAna4[1] = 0x15; //bank:ana4, addr:h0000
        szDataAna4[2] = 0x00;
        
        IicWriteData(SLAVE_I2C_ID_DBBUS, &szDataAna4[0], 3);
        IicReadData(SLAVE_I2C_ID_DBBUS, &szDataAna41[0], ANA4_MUTUAL_CSUB_NUMBER); //200

        nTotalNum -= (u16)ANA4_MUTUAL_CSUB_NUMBER;
    }

    for (i = 0; i < nTotalNum; i ++)
    {
        szDataAna31[i] = 0;
    }

    mdelay(100);

    szDataAna3[0] = 0x10;
    szDataAna3[1] = 0x10; //bank:ana3, addr:h0020
    szDataAna3[2] = 0x40;

    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDataAna3[0], 3);
    IicReadData(SLAVE_I2C_ID_DBBUS, &szDataAna31[0], ANA3_MUTUAL_CSUB_NUMBER); //192

    for (i = 0; i < ANA3_MUTUAL_CSUB_NUMBER; i ++)
    {
        szModeTemp[i] = szDataAna31[i];
    }

    for (i = ANA3_MUTUAL_CSUB_NUMBER; i < (ANA3_MUTUAL_CSUB_NUMBER + ANA4_MUTUAL_CSUB_NUMBER); i ++)
    {
        szModeTemp[i] = szDataAna41[i - ANA3_MUTUAL_CSUB_NUMBER];
    }
    
    for (i = 0; i < nDriveLineNum; i ++)
    {
        for (j = 0; j < nSenseLineNum; j ++)
        {
            _gMode[j * nDriveLineNum + i] = szModeTemp[i * MAX_CHANNEL_SEN + j];

//            DBG("_gMode[%d] = %d\n", j * nDriveLineNum + i, _gMode[j * nDriveLineNum + i]);
        }
    }
}

static void _DrvMpTestItoOpenTestDisableFilterNoiseDetect(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x1302); //bank:fir, addr:h0001
    nTemp &= (~(BIT2 | BIT1 | BIT0));
    RegSet16BitValue(0x1302, nTemp);
}

static void _DrvMpTestItoOpenTestAnaSwReset(void)
{
    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x1100, 0xFFFF); //bank:ana, addr:h0000
    RegSet16BitValue(0x1100, 0x0000);
    mdelay(100);
}

static void _DrvMpTestItoOpenTestEnableAdcOneShot(void)
{
    u16 nTemp = 0;

    DBG("*** %s() ***\n", __func__);

    RegSet16BitValue(0x130C, BIT15); //bank:fir, addr:h0006
    nTemp = RegGet16BitValue(0x1214); //bank:ana2, addr:h000a
    nTemp |= BIT0;
    RegSet16BitValue(0x1214, nTemp);
}

static void _DrvMpTestItoOpenTestGetMutualOneShotRawIir(u16 wszResultData[][MAX_CHANNEL_DRV], u16 nDriveLineNum, u16 nSenseLineNum)
{
    u16 nRegData;
    u16 i, j;
    u16 nTemp;
    u8 szDbBusTxData[3];
    u8 szShotData1[FILTER1_MUTUAL_DELTA_C_NUMBER]; //190 = (6 * 14 + 11) * 2
    u8 szShotData2[FILTER2_MUTUAL_DELTA_C_NUMBER]; //594 = (MAX_MUTUAL_NUM - (6 * 14 + 11)) * 2

    DBG("*** %s() ***\n", __func__);

    nTemp = RegGet16BitValue(0x3D08); //bank:intr_ctrl, addr:h0004
    nTemp &= (~(BIT8 | BIT4));
    RegSet16BitValue(0x3D08, nTemp);

    _DrvMpTestItoOpenTestEnableAdcOneShot();
    
    nRegData = 0;
    while (0x0000 == (nRegData & BIT8))
    {
        nRegData = RegGet16BitValue(0x3D18); //bank:intr_ctrl, addr:h000c
    }

    for (i = 0; i < FILTER1_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        szShotData1[i] = 0;
    }
    
    for (i = 0; i < FILTER2_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        szShotData2[i] = 0;
    }

    mdelay(100);
    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x13; //bank:fir, addr:h0021
    szDbBusTxData[2] = 0x42;
    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3);
    IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData1[0], FILTER1_MUTUAL_DELTA_C_NUMBER); //190

    mdelay(100);
    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x20; //bank:fir2, addr:h0000
    szDbBusTxData[2] = 0x00;
    IicWriteData(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3);
 //   IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData2[0], FILTER2_MUTUAL_DELTA_C_NUMBER); //594
 //zxz modify for i2c read data size
 //zxz add rawdata
    i = 0;
    j = 0;
    while (i < FILTER2_MUTUAL_DELTA_C_NUMBER) {
        if ((FILTER2_MUTUAL_DELTA_C_NUMBER - i ) < 128 )
            j = IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData2[i], (FILTER2_MUTUAL_DELTA_C_NUMBER - i));
         else
            j = IicReadData(SLAVE_I2C_ID_DBBUS, &szShotData2[i], 128);

        if (j > 0)    //transimit ok
            i += j;
    }

    for (j = 0; j < nDriveLineNum; j ++)
    {
        for (i = 0; i < nSenseLineNum; i ++)
        {
            // FILTER1 : SF0~SF5, AFE0~AFE13; SF6, AFE0~AFE10
            if ((j <= 5) || ((j == 6) && (i <= 10)))
            {
                nRegData = (u16)(szShotData1[(j * 14 + i) * 2] | szShotData1[(j * 14 + i) * 2 + 1] << 8);
                wszResultData[i][ j] = (short)nRegData;
            }
            else
            {
                // FILTER2 : SF6, AFE11~AFE13
                if ((j == 6) && (i > 10))
                {
                    nRegData = (uint16_t)(szShotData2[((j - 6) * 14 + (i - 11)) * 2] | szShotData2[((j - 6) * 14 + (i - 11)) * 2 + 1] << 8);
                    wszResultData[i][j] = (short)nRegData;
                }
                else
                {
                    nRegData = (uint16_t)(szShotData2[6 + ((j - 7) * 14 + i) * 2] | szShotData2[6 + ((j - 7) * 14 + i) * 2 + 1] << 8);
                    wszResultData[i][j] = (short)nRegData;
                }
            }
        }
    }

    nTemp = RegGet16BitValue(0x3D08); //bank:intr_ctrl, addr:h0004
    nTemp |= (BIT8 | BIT4);
    RegSet16BitValue(0x3D08, nTemp);
}

static void _DrvMpTestItoOpenTestGetDeltaC(s32 *pTarget)
{
    s16 nTemp;
    u16 wszRawData[MAX_CHANNEL_SEN][MAX_CHANNEL_DRV];
    u16 i, j;
    u16 nDriveLineNum = 0, nSenseLineNum = 0, nShift = 0;
//zxz add rawdata
    DBG("*** %s() ***\n", __func__);

    nSenseLineNum = _DrvMpTestItoOpenTestAnaGetMutualChannelNum();
    nDriveLineNum = _DrvMpTestItoOpenTestAnaGetMutualSubFrameNum();
    
    _DrvMpTestItoOpenTestGetMutualOneShotRawIir(wszRawData, nDriveLineNum, nSenseLineNum);

    for (i = 0; i < nSenseLineNum; i ++)
    {
        for (j = 0; j < nDriveLineNum; j ++)
        {
            nShift = (u16)(i * nDriveLineNum + j);
            nTemp = (s16)wszRawData[i][j];
            pTarget[nShift] = nTemp;

//            DBG("wszRawData[%d][%d] = %d\n", i, j, nTemp);
        }
    }
}

s32 _DrvMpTestItoOpenTest(void)
{
    s32 nRetVal = 0;
    s32 nPrev = 0, nDelta = 0;
    u16 i = 0, j = 0;
    u16 nCheckState = 0;

    DBG("*** %s() ***\n", __func__);


    DrvPlatformLyrDisableFingerTouchReport();
    DrvPlatformLyrTouchDeviceResetHw();

    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(300);
    
    /*
      0 : SYS_STATE_NULL
      1 : SYS_STATE_INIT
      4 : SYS_STATE_STRIKE
    */
    while ((nCheckState == 0 || nCheckState == 1 || nCheckState == 4) && (i < 10))
    {
        nCheckState = _DrvMpTestItoOpenTestFirmwareGetState();
        mdelay(100);
        i ++;
    }
    
    if (i >= 10)
    {
        DbBusIICNotUseBus();
        DbBusNotStopMCU();
        DbBusExitSerialDebugMode();
        DrvPlatformLyrTouchDeviceResetHw();
        mdelay(300);
        DrvPlatformLyrEnableFingerTouchReport();

        return -2;
    }
    _DrvMpTestItoOpenTestMcuStop();
    mdelay(10);
//zxz add rawdata
    for (i = 0; i < MAX_MUTUAL_NUM; i ++)
    {
        _gTestFailChannel[i] = 0;
    }

    _gTestFailChannelCount = 0; // Reset _gTestFailChannelCount to 0 before test start

    _gSenseLineNum = _DrvMpTestItoOpenTestAnaGetMutualChannelNum();
    _gDriveLineNum = _DrvMpTestItoOpenTestAnaGetMutualSubFrameNum();
    
    _DrvMpTestItoOpenTestAnaSwitchToMutual();
    _DrvMpTestItoOpenTestAnaGetMutualCSub(_gMode);
    _DrvMpTestItoOpenTestDisableFilterNoiseDetect();
    _DrvMpTestItoOpenTestAnaSwReset();
    _DrvMpTestItoOpenTestGetDeltaC(_gDeltaC);
  //zxz add rawdata
    for (i = 0; i < _gSenseLineNum; i ++)
    {
        DBG("\nSense[%02d]\t", i);
        
        for (j = 0; j < _gDriveLineNum; j ++)
        {
            _gResult[i * _gDriveLineNum + j] = (4464*_gMode[i * _gDriveLineNum + j] - _gDeltaC[i * _gDriveLineNum + j]);
            DBG("%d\t", _gResult[i * _gDriveLineNum + j]);
//            DBG("%d  %d  %d\t", _gResult[i * _gDriveLineNum + j], 4464*_gMode[i * _gDriveLineNum + j], _gDeltaC[i * _gDriveLineNum + j]);
        }
    }
    
    DBG("\n\n\n");

    for (j = 0; j < (_gDriveLineNum-1); j ++)
    {
        for (i = 0; i < _gSenseLineNum; i ++)
        {
            if (_gResult[i * _gDriveLineNum + j] < FIR_THRESHOLD)
            {
                _gTestFailChannel[i * _gDriveLineNum + j] = 1;
                _gTestFailChannelCount ++;
                nRetVal = -1;
                DBG("\nSense%d, Drive%d, MIN_Threshold = %d\t", i, j, _gResult[i * _gDriveLineNum + j]);
            }

            if (i > 0)
            {
                nDelta = _gResult[i * _gDriveLineNum + j] > nPrev ? (_gResult[i * _gDriveLineNum + j] - nPrev) : (nPrev - _gResult[i * _gDriveLineNum + j]);
                if (nDelta > nPrev*FIR_RATIO/100)
                {
                    if (0 == _gTestFailChannel[i * _gDriveLineNum + j]) // for avoid _gTestFailChannelCount to be added twice
                    {
                        _gTestFailChannel[i * _gDriveLineNum + j] = 1;
                        _gTestFailChannelCount ++;
                    }
                    nRetVal = -1;
                    DBG("\nSense%d, Drive%d, MAX_Ratio = %d,%d\t", i, j, nDelta, nPrev);
                }
            }
            nPrev = _gResult[i * _gDriveLineNum + j];
        }
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();
    DrvPlatformLyrTouchDeviceResetHw();
    mdelay(300);

    DrvPlatformLyrEnableFingerTouchReport();

    return nRetVal;
}

static void _DrvMpTestItoTestDoWork(struct work_struct *pWork)
{
    s32 nRetVal = 0;
    
    DBG("*** %s() _gIsInMpTest = %d, _gTestRetryCount = %d ***\n", __func__, _gIsInMpTest, _gTestRetryCount);

    if (_gItoTestMode == ITO_TEST_MODE_OPEN_TEST)
    {
        nRetVal = _DrvMpTestItoOpenTest();
    }
    else
    {
        DBG("*** Undefined Mp Test Mode = %d ***\n", _gItoTestMode);
        return;
    }

    DBG("*** ctp mp test result = %d ***\n", nRetVal);
    
    if (nRetVal == 0)
    {
//        _gCtpMpTestStatus = 0; //PASS
        _gCtpMpTestStatus = ITO_TEST_OK; //PASS
        _gIsInMpTest = 0;
        DBG("mp test success\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
        DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
    }
    else
    {
        _gTestRetryCount --;
        if (_gTestRetryCount > 0)
        {
            DBG("_gTestRetryCount = %d\n", _gTestRetryCount);
            queue_work(_gCtpMpTestWorkQueue, &_gCtpItoTestWork);
        }
        else
        {
//            _gCtpMpTestStatus = -1; //FAIL

            if (nRetVal == -1)
            {
                _gCtpMpTestStatus = ITO_TEST_FAIL;
            }
            else
            {
                _gCtpMpTestStatus = ITO_TEST_UNDEFINED_ERROR;
            }

            _gIsInMpTest = 0;
            DBG("mp test failed\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
            DrvFwCtrlRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
        }
    }
}

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

s32 DrvMpTestGetTestResult(void)
{
    DBG("*** %s() ***\n", __func__);
    DBG("_gCtpMpTestStatus = %d\n", _gCtpMpTestStatus);

    return _gCtpMpTestStatus;
}

void DrvMpTestGetTestFailChannel(ItoTestMode_e eItoTestMode, u8 *pFailChannel, u32 *pFailChannelCount)
{
    u32 i;
    //zxz add rawdata
    DBG("*** %s() ***\n", __func__);
    DBG("_gTestFailChannelCount = %d\n", _gTestFailChannelCount);

    for (i = 0; i < MAX_MUTUAL_NUM; i ++)
    {
		pFailChannel[i] = _gTestFailChannel[i];
    }

    *pFailChannelCount = MAX_MUTUAL_NUM; // Return the test result of all channels, APK will filter out the fail channels.
}

void DrvMpTestGetTestDataLog(ItoTestMode_e eItoTestMode, u8 *pDataLog, u32 *pLength)
{
    u32 i, j, k;
    //zxz add rawdata
    DBG("*** %s() ***\n", __func__);

    if (eItoTestMode == ITO_TEST_MODE_OPEN_TEST)
    {
        k = 0;

        for (j = 0; j < (_gDriveLineNum-1); j ++)
        {
            for (i = 0; i < _gSenseLineNum; i ++)
            {
//                DBG("\nDrive%d, Sense%d, Value = %d\t", j, i, _gResult[i * _gDriveLineNum + j]); // add for debug

                if (_gResult[i * _gDriveLineNum + j] >= 0)
                {
                    pDataLog[k*5] = 0; // + : a positive number
                }
                else
                {
					pDataLog[k*5] = 1; // - : a negative number
                }

                pDataLog[k*5+1] = (_gResult[i * _gDriveLineNum + j] >> 24) & 0xFF;
                pDataLog[k*5+2] = (_gResult[i * _gDriveLineNum + j] >> 16) & 0xFF;
                pDataLog[k*5+3] = (_gResult[i * _gDriveLineNum + j] >> 8) & 0xFF;
                pDataLog[k*5+4] = (_gResult[i * _gDriveLineNum + j]) & 0xFF;

                k ++;
            }
        }

        DBG("\nk = %d\n", k);

        *pLength = k*5;
    }
    else
    {
        DBG("*** Undefined MP Test Mode ***\n");
    }
}

void DrvMpTestGetTestScope(TestScopeInfo_t *pInfo)
{
    DBG("*** %s() ***\n", __func__);

    pInfo->nMy = _gDriveLineNum;
    pInfo->nMx = _gSenseLineNum;

    DBG("*** My = %d ***\n", pInfo->nMy);
    DBG("*** Mx = %d ***\n", pInfo->nMx);
}

void DrvMpTestScheduleMpTestWork(ItoTestMode_e eItoTestMode)
{
    DBG("*** %s() ***\n", __func__);

    if (_gIsInMpTest == 0)
    {
        DBG("ctp mp test start\n");
        
        _gItoTestMode = eItoTestMode;
        _gIsInMpTest = 1;
        _gTestRetryCount = CTP_MP_TEST_RETRY_COUNT;
        _gCtpMpTestStatus = ITO_TEST_UNDER_TESTING;
        
        queue_work(_gCtpMpTestWorkQueue, &_gCtpItoTestWork);
    }
}

void DrvMpTestCreateMpTestWorkQueue(void)
{
    DBG("*** %s() ***\n", __func__);

    _gCtpMpTestWorkQueue = create_singlethread_workqueue("ctp_mp_test");
    INIT_WORK(&_gCtpItoTestWork, _DrvMpTestItoTestDoWork);
}

#endif //CONFIG_ENABLE_ITO_MP_TEST
#endif //CONFIG_ENABLE_CHIP_MSG26XXM