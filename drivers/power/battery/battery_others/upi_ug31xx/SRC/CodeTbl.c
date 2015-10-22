/**
 * @file CodeTbl.c
 *
 *  Code Table
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

STATIC CONST GU2 CapTableTemp[] = {
  450, 		///< 45 oC  
  250, 		///< 25 oC  
  150, 		///< 15 oC  
   50, 		///<  5 oC
    0,    ///< End of Temperature Table
};

GU2 *CapTableTempPtr = (GU2 *) &CapTableTemp[0];

STATIC CONST GU2 CapTableOcvSoc[] = {
  1000, 950,  900,  850,  800,  750,  700,  650,  600,  550,  500,  
  450,  400,  350,  300,  250,  200,  150,  100,  50,   0,
};

GU2 *CapTableOcvSocPtr = (GU2 *) &CapTableOcvSoc[0];

STATIC CONST GU1 CapTableUpdateIdx[] = {
  20,     ///< 0%
  19,     ///< 5% 
  17,     ///< 15%
  16,     ///< 20%
  0,      ///< End of Update Index Table
};

GU1 *CapTableUpdateIdxPtr = (GU1 *) &CapTableUpdateIdx[0];

STATIC CONST GConfigMapType CfgMapTable[] = {
  { "RM",           0x40, 2,  },
  { "FCC",          0x42, 2,  },
  { "CUR_CAP",      0x44, 2,  },
  { "TIME_YEAR",    0x46, 2,  },
  { "TIME_MONTH",   0x48, 1,  },
  { "TIME_DAY",     0x49, 1,  },
  { "TIME_HOUR",    0x4A, 1,  },
  { "TIME_MIN",     0x4B, 1,  },
  { "BOARD_OFFSET", 0x4C, 4,  },
  { "TABLE_SIGN",   0x50, 1,  },
  { "TABLE_OCV",    0x51, 42, },
  { "TABLE_R",      0x7B, 21, },
  { "QD_0",         0x90, 2,  },
  { "QD_1",         0x92, 2,  },
  { "QD_2",         0x94, 2,  },
  { "QD_3",         0x96, 2,  },
  { "CELL_ID",      0x98, 8,  },
  { "DRV_VERSION",  0xA0, 11, },
  { "COMP_BO",      0xAB, 1,  },
  { "GGB_VERSION",  0xAC, 2,  },
  { "CHECKSUM",     0xAE, 2,  },
  { "",             0xFF, 0,  },
};

GConfigMapType *CfgMapTablePtr = (GConfigMapType *) &CfgMapTable[0];

STATIC CONST PollingTimeType PollingTimeTable[] = {
  { 50, 30, },
  { 20, 15, },
  { 10, 10, },
  { 5,  5,  },
  { 0,  1,  },
};

PollingTimeType *PollTime = (PollingTimeType *) &PollingTimeTable[0];

///----------------------------------------------///
/// SBS Standard Read Command Table
///----------------------------------------------///
GWORD (* const StdCmdTable[])(GVOID) = 
{
         /// Standard SBS Read Command
         SbsManuAccess,                  	///< 0x00 Manufacture Access
         SbsRemainCapacityAlarm,         	///< 0x01 Remaining Capacity Alarm
         SbsRemainTimeAlarm,          		///< 0x02 Remaining Time Alarm
         SbsBattMode,                   	///< 0x03 Battery Mode
         SbsAtRate,                  			///< 0x04 At Rate
         SbsAtRateTimeToFull,             ///< 0x05 At Rate Time To Full
         SbsAtRateTimeToEmpty,            ///< 0x06 At Rate Time To Empty
         SbsAtRateOK,                			///< 0x07 At Rate OK
         SbsTemp,                 				///< 0x08 Temperature
         SbsVolt,                 				///< 0x09 Total Battery Voltage	
         SbsCurr,                 				///< 0x0a Current
         SbsAvgCurr,              				///< 0x0b Average Current
         SbsMaxErr,               				///< 0x0c Max Error
         SbsRelativeStateOfCharge,        ///< 0x0d Relative State Of Charge
         SbsAbsoluteStateOfCharge,        ///< 0x0e Absolute State Of Charge
         SbsRemainingCapacity,            ///< 0x0f Remaining Capacity		
         SbsFullChargeCapacity,           ///< 0x10 Full Charge Capacity
         SbsRunTimeToEmpty,               ///< 0x11 Run Time To Empty
         SbsAbsoluteTimeToEmpty,          ///< 0x12 Absolute Time To Empty
         SbsAbsoluteTimeToFull,           ///< 0x13 Absolute Time To Full
         SbsChargeCurrent,                ///< 0x14 Charge Current
         SbsChargeVoltage,                ///< 0x15 Charge Voltage
         SbsBatteryStatus,                ///< 0x16 Battery Status
         SbsCycleCount,               		///< 0x17 Cycle Count
         SbsDesignCapacity,               ///< 0x18 Design Capacity
         SbsDesignVoltage,                ///< 0x19 Design Voltage
         SbsDummy,             						///< 0x1a  Specification Info shall be access by Read Page
         SbsManufactureDate,          		///< 0x1b Manufacture Date : ReadPage
         SbsSerialNumber,              		///< 0x1c Serial Number
         SbsDummy,                        ///< 0x1d Dummy
         SbsDummy,                        ///< 0x1e Dummy
         SbsDummy,                        ///< 0x1f Dummy
         SbsMFName,                       ///< 0x20 Manufacturer Name
         SbsDevName,                      ///< 0x21 Device Name
         SbsDevChem,                      ///< 0x22 Device Chemistry
         SbsDummy,                        ///< 0x23 Manufacturer Data
         SbsDummy,                        ///< 0x24 Dummy
         SbsDummy,                        ///< 0x25 Dummy
         SbsDummy,                        ///< 0x26 Dummy
         SbsDummy,                        ///< 0x27 Dummy
         SbsDummy,                        ///< 0x28 Dummy
         SbsDummy,                        ///< 0x29 Dummy
         SbsDummy,                        ///< 0x2a Dummy
         SbsDummy,                        ///< 0x2b Dummy
         SbsDummy,                        ///< 0x2c Dummy
         SbsDummy,                        ///< 0x2d Dummy
         SbsDummy,                        ///< 0x2e Dummy
         SbsDummy,                        ///< 0x2f Authentication and Manufacturer Input
         SbsDummy,                        ///< 0x30 Dummy
         SbsDummy,                        ///< 0x31 Dummy
         SbsDummy,                        ///< 0x32 Dummy
         SbsDummy,                        ///< 0x33 Dummy
         SbsDummy,                        ///< 0x34 Dummy
         SbsDummy,                        ///< 0x35 Dummy
         SbsDummy,                        ///< 0x36 Dummy
         SbsDummy,                        ///< 0x37 Dummy
         SbsDummy,                        ///< 0x38 Dummy
         SbsRid,                          ///< 0x39 RID
         SbsMaxCellVoltage,               ///< 0x3a Maximum Cell Voltage
         SbsMinCellVoltage,               ///< 0x3b Minimum Cell Voltage
         SbsCellVoltage3,                 ///< 0x3c Cell Voltage 3
         SbsCellVoltage2,                 ///< 0x3d Cell Voltage 2
         SbsCellVoltage1,                 ///< 0x3e Cell Voltage 1
         SbsCellVoltage0,                 ///< 0x3f Cell Voltage 0
};

GWORD (* const StdCmdTablePage2[])(GVOID) = {
         SbsCapData,                      ///< 0x40 Dummy
         SbsVCapData,                     ///< 0x41 Dummy
         SbsAlarm,                        ///< 0x42 Alarm function
         SbsOffTime,                      ///< 0x43 Power Off Time
         SbsDummy,                        ///< 0x44 Dummy
         SbsDummy,                        ///< 0x45 Dummy
         SbsDummy,                        ///< 0x46 Dummy
         SbsDummy,                        ///< 0x47 Dummy
         SbsDummy,                        ///< 0x48 Dummy
         SbsDummy,                        ///< 0x49 Dummy
         SbsDummy,                        ///< 0x4a Dummy
         SbsVRT,                          ///< 0x4b RT of Voltage Gauge
         SbsVFct,                         ///< 0x4c FCT of Voltage Gauge
         SbsVRsoc,                        ///< 0x4d RSOC of Voltage Gauge
         SbsVAsoc,                        ///< 0x4e ASOC of Voltage Gauge
         SbsVRM,                          ///< 0x4f RM of Voltage Gauge
         SbsVFcc,                         ///< 0x50 FCC of Voltage Gauge
         SbsDummy,                        ///< 0x51 Dummy
         SbsDummy,                        ///< 0x52 Dummy
         SbsDummy,                        ///< 0x53 Dummy
         SbsDummy,                        ///< 0x54 Dummy
         SbsDummy,                        ///< 0x55 Dummy
         SbsDummy,                        ///< 0x56 Dummy
         SbsDummy,                        ///< 0x57 Dummy
         SbsDummy,                        ///< 0x58 Dummy
         SbsDummy,                        ///< 0x59 Dummy
         SbsDummy,                        ///< 0x5a Dummy
         SbsDummy,                        ///< 0x5b Dummy
         SbsDummy,                        ///< 0x5c Dummy
         SbsPrevRsoc,                     ///< 0x5d Previous RSOC
         SbsPrevAsoc,                     ///< 0x5e Previous ASOC
         SbsPrevRM,                       ///< 0x5f Previous RM
         SbsPrevFcc,                      ///< 0x60 Previous FCC
         SbsRsocTimer,                    ///< 0x61 RSOC Timer
         SbsDummy,                        ///< 0x62 Dummy
         SbsDummy,                        ///< 0x63 Dummy
         SbsDummy,                        ///< 0x64 Dummy
         SbsDummy,                        ///< 0x65 Dummy
         SbsDummy,                        ///< 0x66 Dummy
         SbsDummy,                        ///< 0x67 Dummy
         SbsDummy,                        ///< 0x68 Dummy
         SbsDummy,                        ///< 0x69 Dummy
         SbsDummy,                        ///< 0x6a Dummy
         SbsDummy,                        ///< 0x6b Dummy
         SbsDummy,                        ///< 0x6c Dummy
         SbsDummy,                        ///< 0x6d Dummy
         SbsDummy,                        ///< 0x6e Dummy
         SbsDummy,                        ///< 0x6f Dummy
         SbsDummy,                        ///< 0x70 Dummy
         SbsDummy,                        ///< 0x71 Dummy
         SbsDummy,                        ///< 0x72 Dummy
         SbsDummy,                        ///< 0x73 Dummy
         SbsDummy,                        ///< 0x74 Dummy
         SbsDummy,                        ///< 0x75 Dummy
         SbsDummy,                        ///< 0x76 Dummy
         SbsDummy,                        ///< 0x77 Dummy
         SbsDummy,                        ///< 0x78 Dummy
         SbsDummy,                        ///< 0x79 Dummy
         SbsDummy,                        ///< 0x7a Dummy
         SbsDummy,                        ///< 0x7b Dummy
         SbsDummy,                        ///< 0x7c Dummy
         SbsDummy,                        ///< 0x7d Dummy
         SbsDummy,                        ///< 0x7e Dummy
         SbsDummy,                        ///< 0x7f Dummy
};

GWORD (* const StdCmdTablePage3[])(GVOID) = {
         /// vendor SBS Read Command
				SbsSysMode,			             			///< 0xc0  SysMode,			
				SbsAdcEOC,				             		///< 0xC1  AdcEOC,			
				SbsMainLoopIdx,             			///< 0xC2  MainLoopIdx       
				SbsRegAddr,             					///< 0xC3  Register Read / Writr Addr     
				SbsRegData,             					///< 0xC4  Register Read / Writr data        
				SbsDFAddr,             						///< 0xC5  DF Addr       
				SbsDFData,             						///< 0xC6  DF page data     
				SbsCurMeas,           						///< 0xC7  Dummy       
				SbsPreMeas,           						///< 0xC8  Dummy       
				SbsSuspendTime,        						///< 0xC9  Dummy       
				SbsDummy,             						///< 0xCA  Dummy       
				SbsDummy,             						///< 0xCB  Dummy       
				SbsDummy,             						///< 0xCC  Dummy       
				SbsDummy,             						///< 0xCD  Dummy       
				SbsDummy,             						///< 0xCE  Dummy       
				SbsDummy,             						///< 0xCF  Dummy       
				SbsCodeCurr,			             		///< 0xD0  CodeCurr,       
				SbsCodeIT,				             		///< 0xD1  CodeIT,			  
				SbsCodeET,				             		///< 0xD2  CodeET,  
				SbsCodeCC,				             		///< 0xD3  CodeCC,  
				SbsCodeCnt,			          	   		///< 0xD4  CodeCnt,  
				SbsCodeVBat1,		        	     		///< 0xD5  CodeVBat1,  
				SbsCodeVBat2,		      	       		///< 0xD6  CodeVBat2,  
				SbsCodeVBat3,		    	         		///< 0xD7  CodeVBat3,  
				SbsDummy,												  ///< 0xD8	Dummy 	 
				SbsDummy,             						///< 0xD9  Dummy					
				SbsDummy,												  ///< 0xDA 	Dummy 				 
				SbsDummy,             						///< 0xDB  Dummy				
				SbsDummy,												  ///< 0xDC	Dummy 				 
				SbsDummy,             						///< 0xDD  Dummy				
				SbsDummy,												  ///< 0xDE	Dummy 			 
				SbsDummy,             						///< 0xDF Dummy			
				SbsCoulombCounter,     						///< 0xE0  CurMeas->iCurCap
				SbsDeltaQ,             						///< 0xE1  CurMeas->sDeltaQ
				SbsVPack2,             						///< 0xE2  VCell1 + VCell2
				SbsVPack3,             						///< 0xE3  VCell1 + VCell2 + VCell3
				SbsDummy,             						///< 0xE4  Dummy       
				SbsDummy,             						///< 0xE5  Dummy       
				SbsDummy,             						///< 0xE6  Dummy       
				SbsDummy,             						///< 0xE7  Dummy       
				SbsDummy,             						///< 0xE8  Dummy       
				SbsDummy,             						///< 0xE9  Dummy       
				SbsDummy,             						///< 0xEA  Dummy       
				SbsDummy,             						///< 0xEB  Dummy       
				SbsDummy,             						///< 0xEC  Dummy       
				SbsDummy,             						///< 0xED  Dummy       
				SbsDummy,             						///< 0xEE  Dummy       
				SbsDummy,             						///< 0xEF  Dummy   	
				SbsCodeCurrCali,	         	    	///< 0xF0  CodeTmpCurr  
				SbsRawCurr,          				     	///< 0xF1  RawCurr			
				SbsCodeVbat1Cali,	         	    	///< 0xF2  CodeTmpVBat1
				SbsRawVbat1,          				    ///< 0xF3  RawVbat1       
				SbsCodeVbat2Cali,	         	    	///< 0xF4  CodeTmpVBat2
				SbsRawVbat2,          				    ///< 0xF5  RawVbat2   
				SbsCodeVbat3Cali,	         	    	///< 0xF6  CodeTmpVBat3
				SbsRawVbat3,          				    ///< 0xF7  RawVbat3
				SbsErrorStatus,             			///< 0xF8  ErrorStatus       
				SbsWarningStatus,             		///< 0xF9  WarningStatus         
				SbsGaugeStatus,        						///< 0xFA  GaugeStatus
				SbsDummy,             						///< 0xFB  Dummy       
				SbsDummy,             						///< 0xFC  Dummy       
				SbsIIRVoltRatio,       						///< 0xFD  Dummy       
				SbsFWVersion,             				///< 0xFE  FW SVN Version      
				SbsDFVersion,             				///< 0xFF  DF version
				
};

#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

