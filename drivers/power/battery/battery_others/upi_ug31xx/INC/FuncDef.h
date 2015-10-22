/**
 * @file FuncDef.h
 *
 * Define global functions
 *
 * @version $Revision$
 * @author JLJuang <juangjl@csie.nctu.edu.tw>
 * @note Copyright (c) 2010, JSoftLab, all rights reserved.
 * @note
*/
#ifndef __FUNC_DEF__
#define __FUNC_DEF__

/// =============================================================///
///      START OF BOOT  FUNCTION DEFINITION                      ///
/// =============================================================///

///-------------------------------------------------------------------------------///
/// 1. Main 
///-------------------------------------------------------------------------------///
EXTERN GVOID GMainLoop(GVOID);
EXTERN GVOID GMainInit(GVOID);
EXTERN GVOID GMainTask(GVOID);
EXTERN GVOID GMainExit(GVOID);
EXTERN GVOID GGgbInit(GVOID);

///-------------------------------------------------------------------------------///
/// 2. Task
///-------------------------------------------------------------------------------///
EXTERN GVOID TaskRoundInit(GVOID);
EXTERN GVOID TaskADC(GVOID);
EXTERN GVOID TaskMeasurement(GVOID);
EXTERN GVOID TaskFakeMeasurement(GVOID);
EXTERN GVOID TaskGasGauge(GVOID);
EXTERN GVOID TaskCapacity(GVOID);
EXTERN GVOID TaskVCapacity(GVOID);
EXTERN GVOID TaskSBSData(GVOID);
EXTERN GVOID TaskSystem(GVOID);
EXTERN GVOID TaskAlarm(GVOID);
EXTERN GVOID TaskConfig(GVOID);

EXTERN GVOID TaskDebugPrint(GVOID);


///-------------------------------------------------------------------------------///
/// 3. Debug Function
///-------------------------------------------------------------------------------///
#ifdef  FEATURE_PRINT_SBS_DATA
EXTERN GVOID SbsDataLog(GVOID);
#endif  ///< end of FEATURE_PRINT_SBS_DATA

#ifdef FEATURE_PLAT_WINDOWS
EXTERN GVOID InfoToFile(GVOID);
#endif
///-------------------------------------------------------------------------------///
/// 4. I2C Function
///-------------------------------------------------------------------------------///
EXTERN GVOID I2CInit(GVOID);
EXTERN GBOOL _I2cExeCmd(GI2CCmdType *pI2CCmd);
EXTERN GBOOL I2CExeCmd(GI2CCmdType *pI2CCmd);

#ifdef FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION
EXTERN GBOOL EepromExeCmd(GI2CCmdType *pI2CCmd);
#endif ///< end of FEATURE_ASUS_TINY_BOARD_MASE_PRODUCTION

///-------------------------------------------------------------------------------///
/// 5. Reg read/write Function
///-------------------------------------------------------------------------------///
EXTERN GWORD ReadRegWord(GBYTE bRegAddr);
EXTERN GBYTE ReadRegByte(GBYTE bRegAddr);
EXTERN GVOID ReadRegPage(GBYTE bRegAddr, GBYTE *pData, GBYTE bCnt);
EXTERN GBOOL WriteRegByte(GBYTE bRegAddr, GBYTE bData);
EXTERN GBOOL WriteRegWord(GBYTE bRegAddr, GWORD bData);

///-------------------------------------------------------------------------------///
/// 5. Measurement
///-------------------------------------------------------------------------------///
EXTERN GVOID MeasPrepare(GVOID);

#ifdef  FEATURE_FAKE_MEASUREMENT
EXTERN GVOID FakeMeasPrepare(GVOID);
#endif  ///< end of FEATURE_FAKE_MEASUREMENT

EXTERN GSHORT MeasCurr(GVOID);
EXTERN GWORD MeasVCell1(GVOID);
EXTERN GWORD MeasVCell2(GVOID);
EXTERN GWORD MeasVCell3(GVOID);

#ifdef  FEATURE_FAKE_MEASUREMENT
EXTERN GSHORT FakeMeasCurr(GVOID);
EXTERN GWORD FakeMeasVCell1(GVOID);
EXTERN GWORD FakeMeasVCell2(GVOID);
EXTERN GWORD FakeMeasVCell3(GVOID);
#endif  ///< end of FEATURE_FAKE_MEASUREMENT

EXTERN GSHORT MeasExtTemp(GVOID);
EXTERN GSHORT MeasIntTemp(GVOID);
EXTERN GWORD MeasRid(GVOID);

#ifdef  FEATURE_FAKE_MEASUREMENT
EXTERN GSHORT FakeMeasExtTemp(GVOID);
EXTERN GSHORT FakeMeasIntTemp(GVOID);
EXTERN GWORD FakeMeasRid(GVOID);
#endif  ///< end of FEATURE_FAKE_MEASUREMENT

EXTERN GVOID  MeasConvTime(GVOID);

#ifdef  FEATURE_FAKE_MEASUREMENT
EXTERN GINT32 FakeMeasDeltaTime(GVOID);
#endif  ///< end of FEATURE_FAKE_MEASUREMENT

EXTERN GSHORT MeasCapacity(GVOID);

EXTERN GSHORT MeasCapFilter(GVOID);
EXTERN GSHORT MeasDeltaQ(GVOID);

#ifdef  FEATURE_FAKE_MEASUREMENT
EXTERN GSHORT FakeMeasDeltaQ(GVOID);
#endif  ///< end of FEATURE_FAKE_MEASUREMENT

EXTERN GWORD MeasMaxVCell(GVOID);
EXTERN GWORD MeasMinVCell(GVOID);

EXTERN GWORD MeasRevertVCell1(GWORD wVolt);

///-------------------------------------------------------------------------------///
/// 5. Gas Gauging Function
///-------------------------------------------------------------------------------///
EXTERN GVOID GGGetState(GVOID);
EXTERN GBOOL GGIsDischarging(GVOID);
EXTERN GVOID GGDispatchState(GVOID);

///-------------------------------------------------------------------------------///
/// 5. Charging Function
///-------------------------------------------------------------------------------///
EXTERN GBOOL CHGCheckFC(GVOID);
EXTERN GVOID CHGCheckFCClear(GVOID);

///-------------------------------------------------------------------------------///
/// 6. System Control
///-------------------------------------------------------------------------------///
EXTERN GVOID SYSReadOTP(GVOID);
EXTERN GBOOL SYSCheckLowVoltage(GVOID);
EXTERN GVOID SYSCheckSysMode(GVOID);
EXTERN GVOID SYSReset(GVOID);
EXTERN GVOID SYSInit(GVOID);
EXTERN GVOID SYSSaveData(GVOID);
EXTERN GVOID SYSWaitNextRound(GVOID);
EXTERN GVOID SYSCheckGGRun(GVOID);
EXTERN GVOID SYSCheckOtp(GVOID);

EXTERN GVOID SYSDelay(GDWORD msec);
EXTERN GDWORD SYSRawTime(GVOID);
EXTERN GVOID McuInit(GVOID);
EXTERN GBOOL ChkBridgeConnect(GVOID);

///-------------------------------------------------------------------------------///
/// 6. ADC Control
///-------------------------------------------------------------------------------///
EXTERN GVOID  ADCInit(GVOID);
EXTERN GVOID  ADCFetchCode(GVOID);
EXTERN GVOID  ADCConvStatus(GVOID);
EXTERN GVOID  ADCUpdateQueue1(GVOID);

EXTERN GVOID  ADC1FactorGet(GWORD wCodeIntTemp);
EXTERN GVOID  ADC2FactorGet(GWORD wCodeIntTemp);

EXTERN GWORD  ADCCalibrateITCode(GWORD wITcode);
EXTERN GWORD  ADCCalibrateETCode(GWORD wETcode);
EXTERN GWORD  ADCCalibrateRidCode(GWORD wRidCode);
EXTERN GSHORT ADCCalibrateCapCode(GSHORT sCodeCC);

EXTERN GSHORT ADC1CalibrateCode(GSHORT code);
EXTERN GWORD  ADC2CalibrateCode(GWORD code);
EXTERN GWORD  ADC2RevertCalibrateCode(GWORD code);

EXTERN GVOID DecimateRst(GVOID);

///----------------------------------------------///
/// ADC Filter Function
///----------------------------------------------///
EXTERN GWORD ADCFilterIntTemp(GWORD wITcode);
EXTERN GWORD ADCFilterExtTemp(GWORD wETcode);
EXTERN GWORD ADCFilterRid(GWORD wRidCode);
EXTERN GSHORT ADCFilterCurr(GSHORT sCurrCode);

EXTERN GWORD ADCFilterVCell(GBYTE bCell, GWORD wRawCode);

EXTERN GVOID ADCSetCCOffset(GVOID);

///-------------------------------------------------------------------------------///
/// 6. IOCTL
///-------------------------------------------------------------------------------///


///-------------------------------------------------------------------------------///
///  7. Calibration
///-------------------------------------------------------------------------------///


///-------------------------------------------------------------------------------///
/// 10. Algorithm
///-------------------------------------------------------------------------------///
EXTERN GVOID CapInit(GVOID);
EXTERN GVOID CapInitQD(GVOID);
EXTERN GVOID CapUpdate(GVOID);
EXTERN GVOID CapUpdateSbs(GVOID);

EXTERN GVOID VCapInit(GVOID);
EXTERN GVOID VCapInitQD(GVOID);
EXTERN GVOID VCapUpdate(GVOID);
EXTERN GVOID VCapUpdateSbs(GVOID);
EXTERN GVOID VCapRestore(GVOID);

///-------------------------------------------------------------------------------///
///  11. Data Format Function
///-------------------------------------------------------------------------------///

///-------------------------------------------------------------------------------///
/// 12. Memory Management Function
///-------------------------------------------------------------------------------///

///-------------------------------------------------------------------------------///
/// 13. Config Function
///-------------------------------------------------------------------------------///
EXTERN GVOID CFGInit(GVOID);
EXTERN GVOID CFGPrepareData(GVOID);
EXTERN GVOID CFGUpdate(GVOID);

EXTERN GBOOL _CFGWriteByte(GWORD addr, GCHAR buf);
EXTERN GBOOL _CFGReadByte(GWORD addr, GCHAR *buf);

///-------------------------------------------------------------------------------///
/// Alarm Function
///-------------------------------------------------------------------------------///
EXTERN GVOID AlarmSts(GVOID);
EXTERN GVOID AlarmUV(GVOID);

///-------------------------------------------------------------------------------///
/// Test Main Loop
///-------------------------------------------------------------------------------///
EXTERN GVOID GTestMainLoop(GVOID);

///-------------------------------------------------------------------------------///
/// Platform API
///-------------------------------------------------------------------------------///
EXTERN GVOID  UpiSleep(GDWORD msec);

EXTERN GVOID  UpiGetLocalTime(GTimeDataType *pTimeData);
EXTERN GDWORD UpiGetTickCount(GVOID);

EXTERN GWORD  UpiAbs(GSHORT val);

EXTERN GVOID  UpiMemset(GVOID *dst, GCHAR c, GINT32 cnt);
EXTERN GVOID  UpiMemcpy(GVOID *dst, GVOID *src, GINT32 cnt);

EXTERN GVOID UpiMutexInit(GVOID **pObj);
EXTERN GVOID UpiMutexExit(GVOID **pObj);
EXTERN GVOID UpiMutexLock(GVOID **pObj);
EXTERN GVOID UpiMutexUnLock(GVOID **pObj);
EXTERN GVOID UpiPrintDbg(char *format, ...);
EXTERN GVOID UpiPrintDbgE(char *format, ...);
///-------------------------------------------------------------------------------///
/// SBS Status Update Function
///-------------------------------------------------------------------------------///
EXTERN GVOID SBSStatusCheck(GVOID);
EXTERN GVOID SBSCheckMA(GVOID);
EXTERN GVOID SBSCheckAlarmSts(GVOID);
EXTERN GVOID SBSCheckRsocTimer(GVOID);
EXTERN GWORD SBSCalSoc(GWORD wRM, GWORD wFcc);

///-------------------------------------------------------------------------------///
/// SBS Command
///-------------------------------------------------------------------------------///
EXTERN GBYTE SmbReadCommand(GBYTE bSmbCmd);
EXTERN GBOOL SmbWriteCommand(GBYTE bSmbCmd, GBYTE *pbData, GWORD wSize);

EXTERN GWORD SbsManuAccess(GVOID);    									///< 0x00 Manufacture Access
EXTERN GWORD SbsRemainCapacityAlarm(GVOID);      	///< 0x01 Remaining Capacity Alarm
EXTERN GWORD SbsRemainTimeAlarm(GVOID);          		///< 0x02 Remaining Time Alarm
EXTERN GWORD SbsBattMode(GVOID);                   			///< 0x03 Battery Mode
EXTERN GWORD SbsAtRate(GVOID);                  				///< 0x04 At Rate
EXTERN GWORD SbsAtRateTimeToFull(GVOID);           	///< 0x05 At Rate Time To Full
EXTERN GWORD SbsAtRateTimeToEmpty(GVOID);        		///< 0x06 At Rate Time To Empty
EXTERN GWORD SbsAtRateOK(GVOID);                				///< 0x07 At Rate OK
EXTERN GWORD SbsTemp(GVOID);                 						///< 0x08 Temperature
EXTERN GWORD SbsVolt(GVOID);                 						///< 0x09 Total Battery Voltage	
EXTERN GWORD SbsCurr(GVOID);                							///< 0x0a Current
EXTERN GWORD SbsAvgCurr(GVOID);             						///< 0x0b Average Current
EXTERN GWORD SbsMaxErr(GVOID);               						///< 0x0c Max Error
EXTERN GWORD SbsRelativeStateOfCharge(GVOID);    ///< 0x0d Relative State Of Charge
EXTERN GWORD SbsAbsoluteStateOfCharge(GVOID);    ///< 0x0e Absolute State Of Charge
EXTERN GWORD SbsRemainingCapacity(GVOID);          ///< 0x0f Remaining Capacity		
EXTERN GWORD SbsFullChargeCapacity(GVOID);        ///< 0x10 Full Charge Capacity
EXTERN GWORD SbsRunTimeToEmpty(GVOID);            	 ///< 0x11 Run Time To Empty
EXTERN GWORD SbsAbsoluteTimeToEmpty(GVOID);      ///< 0x12 Absolute Time To Empty
EXTERN GWORD SbsAbsoluteTimeToFull(GVOID);        ///< 0x13 Absolute Time To Full
EXTERN GWORD SbsChargeCurrent(GVOID);               ///< 0x14 Charge Current
EXTERN GWORD SbsChargeVoltage(GVOID);               ///< 0x15 Charge Voltage
EXTERN GWORD SbsBatteryStatus(GVOID);               ///< 0x16 Battery Status
EXTERN GWORD SbsCycleCount(GVOID);               		 ///< 0x17 Cycle Count
EXTERN GWORD SbsDesignCapacity(GVOID);             ///< 0x18 Design Capacity
EXTERN GWORD SbsDesignVoltage(GVOID);               ///< 0x19 Design Voltage
EXTERN GWORD SbsSpecInfo(GVOID);											 ///< 0x1a  Specification Info shall be access by Read Page
EXTERN GWORD SbsManufactureDate(GVOID);          	 ///< 0x1b Manufacture Date : ReadPage
EXTERN GWORD SbsSerialNumber(GVOID);               	 ///< 0x1c Serial Number

EXTERN GWORD SbsMFName(GVOID);                      ///< 0x20 Manufacturer Name
EXTERN GWORD SbsDevName(GVOID);                     ///< 0x21 Device Name
EXTERN GWORD SbsDevChem(GVOID);                     ///< 0x22 Device Chemistry

EXTERN GWORD SbsRid(GVOID);                          ///< 0x39 RID
EXTERN GWORD SbsMaxCellVoltage(GVOID);               ///< 0x3a Maximum Cell Voltage
EXTERN GWORD SbsMinCellVoltage(GVOID);               ///< 0x3b Minimum Cell Voltage
EXTERN GWORD SbsCellVoltage3(GVOID);                 ///< 0x3c Cell Voltage 3
EXTERN GWORD SbsCellVoltage2(GVOID);                 ///< 0x3d Cell Voltage 2
EXTERN GWORD SbsCellVoltage1(GVOID);                 ///< 0x3e Cell Voltage 1
EXTERN GWORD SbsCellVoltage0(GVOID);                 ///< 0x3f Cell Voltage 0

///----------------------------------------------///
/// sbs page 2
///----------------------------------------------///
EXTERN GWORD SbsCapData(GVOID);                     ///< 0x40 Capacity data
EXTERN GVOID SbsCapCmd(GWORD wData);                ///< 0x40 Capacity command
EXTERN GWORD SbsVCapData(GVOID);                    ///< 0x41 Capacity data of Voltage Gauge
EXTERN GVOID SbsVCapCmd(GWORD wData);               ///< 0x41 Capacity command of Voltage Gauge
EXTERN GWORD SbsAlarm(GVOID);                       ///< 0x42 Alarm function
EXTERN GVOID SbsAlarmWt(GWORD wData);               ///< 0x42 Alarm function control
EXTERN GWORD SbsOffTime(GVOID);                     ///< 0x43 Power Off Time
EXTERN GWORD SbsVRT(GVOID);                         ///< 0x4b RT of Voltage Gauge
EXTERN GWORD SbsVFct(GVOID);                        ///< 0x4c FCT of Voltage Gauge
EXTERN GWORD SbsVRsoc(GVOID);                       ///< 0x4d RSOC of Voltage Gauge
EXTERN GWORD SbsVAsoc(GVOID);                       ///< 0x4e ASOC of Voltage Gauge
EXTERN GWORD SbsVRM(GVOID);                         ///< 0x4f RM of Voltage Gauge
EXTERN GWORD SbsVFcc(GVOID);                        ///< 0x50 FCC of Voltage Gauge
EXTERN GWORD SbsPrevRsoc(GVOID);                    ///< 0x5D Previous RSOC
EXTERN GWORD SbsPrevAsoc(GVOID);                    ///< 0x5E Previous ASOC
EXTERN GWORD SbsPrevRM(GVOID);                      ///< 0x5F Previous RM
EXTERN GWORD SbsPrevFcc(GVOID);                     ///< 0x60 Previous FCC
EXTERN GWORD SbsRsocTimer(GVOID);                   ///< 0x61 RSOC Timer

///----------------------------------------------///
/// sbs page 3
///----------------------------------------------///
EXTERN GWORD SbsSysMode(GVOID);												///< 0xC0, SYSTEM MODE
EXTERN GWORD SbsAdcEOC(GVOID);												///< 0xC1, ADC End of conversion status	 
EXTERN GWORD SbsMainLoopIdx(GVOID);									///< 0xC2, MAIN LOOP INDEX
EXTERN GWORD SbsRegAddr(GVOID);												///< 0xC3, Register read/write addr
EXTERN GWORD SbsRegData(GVOID);												///< 0xC4, Register read/write data LOW-BYTE only
EXTERN GWORD SbsDFAddr(GVOID);                        ///< 0xC5, DF address
EXTERN GWORD SbsDFData(GVOID);                        ///< 0xC6, DF data
EXTERN GWORD SbsCurMeas(GVOID);                       ///< 0xC7, Dump current measurement data
EXTERN GWORD SbsPreMeas(GVOID);                       ///< 0xC8, Dump previous measurement data
EXTERN GWORD SbsSuspendTime(GVOID);                   ///< 0xC9, Suspend Time


EXTERN GWORD SbsCodeCurr(GVOID);											///< 0xD0, code OF Current
EXTERN GWORD SbsCodeIT(GVOID);												///< 0xD1, code OF IT
EXTERN GWORD SbsCodeET(GVOID);												///< 0xD2, code of ET
EXTERN GWORD SbsCodeCC(GVOID);												///< 0xD3, code of CC
EXTERN GWORD SbsCodeCnt(GVOID);												///< 0xD4, count of the adc1 conversion
EXTERN GWORD SbsCodeVBat1(GVOID);										///< 0xD5, code of V1
EXTERN GWORD SbsCodeVBat2(GVOID);										///< 0xD6, code of V2
EXTERN GWORD SbsCodeVBat3(GVOID);										///< 0xD7, code of v3

EXTERN GWORD SbsCoulombCounter(GVOID);							///< 0xE0, Coulomb counter
EXTERN GWORD SbsDeltaQ(GVOID);                      ///< 0xE1, DeltaQ
EXTERN GWORD SbsVPack2(GVOID);                      ///< 0xE2, VCell1 + VCell2
EXTERN GWORD SbsVPack3(GVOID);                      ///< 0xE3, VCell1 + VCell2 + VCell3

EXTERN GWORD SbsCodeCurrCali(GVOID);								///< 0xF0,  calibrated code of current 
EXTERN GWORD SbsRawCurr(GVOID);												///< 0xF1, raw current
EXTERN GWORD SbsCodeVbat1Cali(GVOID);							///< 0xF2, calibrated code of v1
EXTERN GWORD SbsRawVbat1(GVOID);											///< 0XF3, raw v1
EXTERN GWORD SbsCodeVbat2Cali(GVOID);							///< 0xF4, calibrated code of v2
EXTERN GWORD SbsRawVbat2(GVOID);											///< 0XF5, raw v2
EXTERN GWORD SbsCodeVbat3Cali(GVOID);							///< 0xF6, calibrated code of v3
EXTERN GWORD SbsRawVbat3(GVOID);											///< 0XF7, raw v3
EXTERN GWORD SbsErrorStatus(GVOID);											///< 0XF8, Error status
EXTERN GWORD SbsWarningStatus(GVOID);											///< 0XF9, Warning status
EXTERN GWORD SbsGaugeStatus(GVOID);                   ///< 0XFA, Gauge Status
EXTERN GWORD SbsIIRVoltRatio(GVOID);                    ///< 0xFD,
EXTERN GWORD SbsFWVersion(GVOID);											///< 0xFE , FW Version
EXTERN GWORD SbsDFVersion(GVOID);											///< 0xFF , Data Flash version	
EXTERN GWORD SbsDummy(GVOID);             							///<	0x?? Dummy

///-------------------------------------------------------------------------------///
/// SBS Write Command
///-------------------------------------------------------------------------------///
EXTERN GVOID SbsSysModeWt(GWORD wData);							///< 0xC0, SYSTEM MODE Write
EXTERN GVOID SbsRegAddrWt(GWORD wData);							///< 0xC3, SYSTEM MODE Write
EXTERN GVOID SbsRegDataWt(GWORD wData);							///< 0xC4, Register read = 0x0000, write = 0x0001, data will be stored in SBS->wRegData
EXTERN GVOID SbsIIRVoltRatioWt(GWORD wData);         ///< 0xFD  IIR Filter ratio
EXTERN GVOID SbsDFAddrWt(GWORD wData);                ///< 0xC5, DF address
EXTERN GVOID SbsDFDataWt(GBYTE *pbData, GWORD bSize); ///< 0xC6, DF Data
EXTERN GVOID SbsSuspendTimeWt(GWORD wData);         ///< 0xC9, Suspend Time


#endif ///< __FUNC_DEF__

