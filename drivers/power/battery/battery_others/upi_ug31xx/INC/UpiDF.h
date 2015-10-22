#ifdef FEATURE_PLAT_WINDOWS
  #pragma pack(push)  ///< push current alignment to stack  
  #pragma pack(1)     ///< set alignment to 1 byte boundary  
#endif ///< for FEATURE_PLAT_WINDOWS

///-------------------------------------------------------------------------------///
///  Data flash SubClass #0 => Control
///-------------------------------------------------------------------------------///
typedef struct SubClassControlSt
{
	///  1. Abbrev: CTLALA; Default: 0(-); DF Addr: 0000h; SRAM Offset: 0000h ; SubClass Offset : 0
	///     Alarm control
	GBYTE bCntl;                                               
} PACK SubClassControlType;	///< Total = 1 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #1 => Voltage
///-------------------------------------------------------------------------------///
typedef struct SubClassVoltageSt
{
	///  1. Abbrev: UVALA; Default: 3000(-); DF Addr: 0001h; SRAM Offset: 0001h ; SubClass Offset : 0
	///     uV Alarm
	GSHORT sUvAlarm;                                           
	///  2. Abbrev: UVREL; Default: 3200(-); DF Addr: 0003h; SRAM Offset: 0003h ; SubClass Offset : 2
	///     uV Release
	GSHORT sUvRelease;                                         
} PACK SubClassVoltageType;	///< Total = 4 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #2 => Temperature
///-------------------------------------------------------------------------------///
typedef struct SubClassTemperatureSt
{
	///  1. Abbrev: UETVALA; Default: 0(-); DF Addr: 0005h; SRAM Offset: 0005h ; SubClass Offset : 0
	///     under temperature threadshold Alarm
	GSHORT sUetAlarm;                                          
	///  2. Abbrev: UETREL; Default: 50(-); DF Addr: 0007h; SRAM Offset: 0007h ; SubClass Offset : 2
	///     under temperature threadshold Release
	GSHORT sUetRelease;                                        
	///  3. Abbrev: OETVALA; Default: 500(-); DF Addr: 0009h; SRAM Offset: 0009h ; SubClass Offset : 4
	///     Over temperature threadshold Alarm
	GSHORT sOetAlarm;                                          
	///  4. Abbrev: OETREL; Default: 400(-); DF Addr: 000Bh; SRAM Offset: 000Bh ; SubClass Offset : 6
	///     Over temperature threadshold Release
	GSHORT sOetRelease;                                        
} PACK SubClassTemperatureType;	///< Total = 8 Bytes

/// =============================================================///
/// Data flash Class #0 => 1stLevelSafetyClass
/// =============================================================///
typedef struct Class1stLevelSafetyClassSt
{
	SubClassControlType scControl;
	SubClassVoltageType scVoltage;
	SubClassTemperatureType scTemperature;
} PACK Class1stLevelSafetyClassType;	///< Total = 13 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #3 => TerminationCfg
///-------------------------------------------------------------------------------///
typedef struct SubClassTerminationCfgSt
{
	///  1. Abbrev: TPCURR; Default: 216(mA); DF Addr: 000Dh; SRAM Offset: 000Dh ; SubClass Offset : 0
	///     TP Current
	GWORD wTPCurrent;                                          
	///  2. Abbrev: TPVOLT; Default: 4100(mV); DF Addr: 000Fh; SRAM Offset: 000Fh ; SubClass Offset : 2
	///     TP Voltage
	GWORD wTPVoltage;                                          
	///  3. Abbrev: TPTIME; Default: 60(Sec); DF Addr: 0011h; SRAM Offset: 0011h ; SubClass Offset : 4
	///     TP Time
	GWORD wTPTime;                                             
	///  4. Abbrev: TPBYPSCURR; Default: 50(-); DF Addr: 0013h; SRAM Offset: 0013h ; SubClass Offset : 6
	///     Tp Bypass Current
	GWORD wTpBypassCurrent;                                    
} PACK SubClassTerminationCfgType;	///< Total = 8 Bytes

/// =============================================================///
/// Data flash Class #1 => ChargeControl
/// =============================================================///
typedef struct ClassChargeControlSt
{
	SubClassTerminationCfgType scTerminationCfg;
} PACK ClassChargeControlType;	///< Total = 8 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #4 => Data
///-------------------------------------------------------------------------------///
typedef struct SubClassDataSt
{
	///  1. Abbrev: CUST; Default: ASUS(-); DF Addr: 0015h; SRAM Offset: 0015h ; SubClass Offset : 0
	///     Customer name defined by uPI
	GCHAR strCustomer[16];                                     
	///  2. Abbrev: PROJ; Default: Z380KL_COVER(-); DF Addr: 0025h; SRAM Offset: 0025h ; SubClass Offset : 16
	///     Project name defined by uPI
	GCHAR strProject[16];                                      
	///  3. Abbrev: CUSTNAME; Default: Co(-); DF Addr: 0035h; SRAM Offset: 0035h ; SubClass Offset : 32
	///     Customer name record by customer
	GCHAR strCustomerSelfDef[16];                              
	///  4. Abbrev: CUSTPROJ; Default: A027(-); DF Addr: 0045h; SRAM Offset: 0045h ; SubClass Offset : 48
	///     Project name record by customer
	GCHAR strProjectCustomerSelfDef[16];                       
	///  5. Abbrev: CELLTYPECODE; Default: 0(-); DF Addr: 0055h; SRAM Offset: 0055h ; SubClass Offset : 64
	///     Cell Type Code
	GWORD wCellTypeCode;                                       
	///  6. Abbrev: ILMD; Default: 4330(mAh); DF Addr: 0057h; SRAM Offset: 0057h ; SubClass Offset : 66
	///     initial last measured discharge (ILMD)
	GWORD wILMD;                                               
	///  7. Abbrev: CYCTHRED; Default: 4330(-); DF Addr: 0059h; SRAM Offset: 0059h ; SubClass Offset : 68
	///     CycleCount Threadshold
	GWORD wCycleCountThrd;                                     
} PACK SubClassDataType;	///< Total = 70 Bytes

/// =============================================================///
/// Data flash Class #2 => SBSConfiguration
/// =============================================================///
typedef struct ClassSBSConfigurationSt
{
	SubClassDataType scData;
} PACK ClassSBSConfigurationType;	///< Total = 70 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #5 => ManufacturerInfo
///-------------------------------------------------------------------------------///
typedef struct SubClassManufacturerInfoSt
{
	///  1. Abbrev: TSIZE; Default: 0(-); DF Addr: 005Bh; SRAM Offset: 005Bh ; SubClass Offset : 0
	///     Total struct size
	GWORD wTotalSize;                                          
	///  2. Abbrev: FWVER; Default: 0(-); DF Addr: 005Dh; SRAM Offset: 005Dh ; SubClass Offset : 2
	///     CellParameter struct version
	GWORD wFwVersion;                                          
	///  3. Abbrev: GGBVER; Default: 0(-); DF Addr: 005Fh; SRAM Offset: 005Fh ; SubClass Offset : 4
	///     GGB version : 0x0102 = 2.1 Version
	GWORD wGGBVersion;                                         
} PACK SubClassManufacturerInfoType;	///< Total = 6 Bytes

/// =============================================================///
/// Data flash Class #3 => SystemData
/// =============================================================///
typedef struct ClassSystemDataSt
{
	SubClassManufacturerInfoType scManufacturerInfo;
} PACK ClassSystemDataType;	///< Total = 6 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #6 => Registers
///-------------------------------------------------------------------------------///
typedef struct SubClassRegistersSt
{
	///  1. Abbrev: TIMEINT; Default: 5(-); DF Addr: 0061h; SRAM Offset: 0061h ; SubClass Offset : 0
	///     Time Interval
	GBYTE bTimeInterval;                                       
	///  2. Abbrev: OPCFG; Default: 1851011105(-); DF Addr: 0062h; SRAM Offset: 0062h ; SubClass Offset : 1
	///     Nac Lmd Adjust Config
	GDWORD dwOPCfg;                                            
} PACK SubClassRegistersType;	///< Total = 5 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #7 => AFE
///-------------------------------------------------------------------------------///
typedef struct SubClassAFESt
{
	///  1. Abbrev: ICTYPE; Default: 1(-); DF Addr: 0066h; SRAM Offset: 0066h ; SubClass Offset : 0
	///     000 : uG3100 1-cell, 001 : uG3101 1-cell, 010 : uG3102 2-cell, 100 : uG3103_2 2-cell, 101 : uG3103_3 3-cell
	GBYTE bICType;                                             
	///  2. Abbrev: GPIO1; Default: 1(-); DF Addr: 0067h; SRAM Offset: 0067h ; SubClass Offset : 1
	///     bit[4] cbc_en32,bit[3] cbc_en21,bit[2] pwm, bit[1] alarm, bit[0] gpio
	GBYTE bGpio1;                                              
	///  3. Abbrev: GPIO2; Default: 1(-); DF Addr: 0068h; SRAM Offset: 0068h ; SubClass Offset : 2
	///     bit[4] cbc_en32,bit[3] cbc_en21,bit[2] pwm, bit[1] alarm, bit[0] gpio
	GBYTE bGpio2;                                              
	///  4. Abbrev: GPIO34; Default: 0(-); DF Addr: 0069h; SRAM Offset: 0069h ; SubClass Offset : 3
	///     bit[4] cbc_en32,bit[3] cbc_en21,bit[2] pwm, bit[1] alarm, bit[0] gpio
	GBYTE bGpio34;                                             
	///  5. Abbrev: CELLNUM; Default: 1(-); DF Addr: 006Ah; SRAM Offset: 006Ah ; SubClass Offset : 4
	///     Project used Cell number
	GBYTE bCellNumber;                                         
	///  6. Abbrev: ASSCELLONE; Default: 1(-); DF Addr: 006Bh; SRAM Offset: 006Bh ; SubClass Offset : 5
	///     AssignCellOneTo
	GBYTE bAssignCellOneTo;                                    
	///  7. Abbrev: ASSCELLTWO; Default: 0(-); DF Addr: 006Ch; SRAM Offset: 006Ch ; SubClass Offset : 6
	///     AssignCellTwoTo
	GBYTE bAssignCellTwoTo;                                    
	///  8. Abbrev: ASSCELLTHREE; Default: 0(-); DF Addr: 006Dh; SRAM Offset: 006Dh ; SubClass Offset : 7
	///     AssignCellThreeTo
	GBYTE bAssignCellThreeTo;                                  
	///  9. Abbrev: SLAVEADDR; Default: 112(-); DF Addr: 006Eh; SRAM Offset: 006Eh ; SubClass Offset : 8
	///     I2C Address(Hex)
	GWORD bI2cSlaveAddress;                                    
	/// 10. Abbrev: I2CCLOCK; Default: 100(-); DF Addr: 0070h; SRAM Offset: 0070h ; SubClass Offset : 10
	///     I2C colock
	GWORD bI2cClock;                                           
	/// 11. Abbrev: TENADDRMODE; Default: 0(-); DF Addr: 0072h; SRAM Offset: 0072h ; SubClass Offset : 12
	///     Ten Bit Address Mode
	GBYTE bTenBitAddressMode;                                  
	/// 12. Abbrev: HIGHSPEED; Default: 0(-); DF Addr: 0073h; SRAM Offset: 0073h ; SubClass Offset : 13
	///     High Speed Mode
	GBYTE bHighSpeedMode;                                      
	/// 13. Abbrev: CHOPCTL; Default: 0(-); DF Addr: 0074h; SRAM Offset: 0074h ; SubClass Offset : 14
	///     CHOP control reg(0xC1)
	GBYTE bChopCtrl;                                           
	/// 14. Abbrev: OSCTJ; Default: -164(-); DF Addr: 0075h; SRAM Offset: 0075h ; SubClass Offset : 15
	///     Osc Tune J
	GSHORT sOscTuneJ;                                          
	/// 15. Abbrev: OSCTK; Default: 375(-); DF Addr: 0077h; SRAM Offset: 0077h ; SubClass Offset : 17
	///     Osc Tune K
	GSHORT sOscTuneK;                                          
	/// 16. Abbrev: ALATIME; Default: 0(-); DF Addr: 0079h; SRAM Offset: 0079h ; SubClass Offset : 19
	///     Alarm Timer
	GBYTE bAlarmTimer;                                         
	/// 17. Abbrev: PWMTIME; Default: 0(-); DF Addr: 007Ah; SRAM Offset: 007Ah ; SubClass Offset : 20
	///     [1:0] 00:32k , 01:16k, 10:8k , 11: 4k
	GBYTE bPwmTimer;                                           
	/// 18. Abbrev: CLKDIVA; Default: 0(-); DF Addr: 007Bh; SRAM Offset: 007Bh ; SubClass Offset : 21
	///     Clk Div A
	GBYTE bClkDivA;                                            
	/// 19. Abbrev: CLKDIVB; Default: 0(-); DF Addr: 007Ch; SRAM Offset: 007Ch ; SubClass Offset : 22
	///     Clk Div B
	GBYTE bClkDivB;                                            
	/// 20. Abbrev: ALAEN; Default: 4(-); DF Addr: 007Dh; SRAM Offset: 007Dh ; SubClass Offset : 23
	///     [7]:COC,[6]:DOC,[5]:IT,[4]:ET,[3]:VP,[2]:V3,[1]:V2,[0]:V1
	GBYTE bAlarmEnable;                                        
	/// 21. Abbrev: CBCEN; Default: 0(-); DF Addr: 007Eh; SRAM Offset: 007Eh ; SubClass Offset : 24
	///     [1]:CBC_EN32, [0]:CBC_EN21
	GBYTE bCbcEnable;                                          
} PACK SubClassAFEType;	///< Total = 25 Bytes

/// =============================================================///
/// Data flash Class #4 => Configuration
/// =============================================================///
typedef struct ClassConfigurationSt
{
	SubClassRegistersType scRegisters;
	SubClassAFEType scAFE;
} PACK ClassConfigurationType;	///< Total = 30 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #8 => EDVCfg
///-------------------------------------------------------------------------------///
typedef struct SubClassEDVCfgSt
{
	///  1. Abbrev: EDV1VOLT; Default: 3000(mV); DF Addr: 007Fh; SRAM Offset: 007Fh ; SubClass Offset : 0
	///     Edv 1 Voltage
	GWORD wEdv1Voltage;                                        
	///  2. Abbrev: OFFTIME; Default: 1(-); DF Addr: 0081h; SRAM Offset: 0081h ; SubClass Offset : 2
	///     Max power off time for re-looking up table at initial
	GBYTE bOffTime;                                            
} PACK SubClassEDVCfgType;	///< Total = 3 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #9 => CurrentThresholds
///-------------------------------------------------------------------------------///
typedef struct SubClassCurrentThresholdsSt
{
	///  1. Abbrev: RSEN; Default: 5(-); DF Addr: 0082h; SRAM Offset: 0082h ; SubClass Offset : 0
	///     Current R-sense
	GBYTE bRsense;                                             
	///  2. Abbrev: STBYCURR; Default: 20(-); DF Addr: 0083h; SRAM Offset: 0083h ; SubClass Offset : 1
	///     Stand by Current
	GWORD wStandbyCurrent;                                     
} PACK SubClassCurrentThresholdsType;	///< Total = 3 Bytes

/// =============================================================///
/// Data flash Class #5 => GasGauging
/// =============================================================///
typedef struct ClassGasGaugingSt
{
	SubClassEDVCfgType scEDVCfg;
	SubClassCurrentThresholdsType scCurrentThresholds;
} PACK ClassGasGaugingType;	///< Total = 6 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #10 => CaliData
///-------------------------------------------------------------------------------///
typedef struct SubClassCaliDataSt
{
	///  1. Abbrev: ADC1OFF; Default: 0(code); DF Addr: 0085h; SRAM Offset: 0085h ; SubClass Offset : 0
	///     adc1 Offset
	GSHORT sAdc1Offset;                                        
	///  2. Abbrev: OFFSETR; Default: 5(mOhm); DF Addr: 0087h; SRAM Offset: 0087h ; SubClass Offset : 2
	///     OffsetR
	GWORD wOffsetR;                                            
	///  3. Abbrev: DELTAR; Default: 4(-); DF Addr: 0089h; SRAM Offset: 0089h ; SubClass Offset : 4
	///     delta R
	GWORD wDeltaR;                                             
	///  4. Abbrev: V2GAIN; Default: 1000(-); DF Addr: 008Bh; SRAM Offset: 008Bh ; SubClass Offset : 6
	///     Vbat2 Gain
	GWORD wVbat2Gain;                                          
	///  5. Abbrev: V2OFF; Default: 0(-); DF Addr: 008Dh; SRAM Offset: 008Dh ; SubClass Offset : 8
	///     Vbat2 Offset
	GSHORT sVbat2Offset;                                       
	///  6. Abbrev: V3GAIN; Default: 1000(-); DF Addr: 008Fh; SRAM Offset: 008Fh ; SubClass Offset : 10
	///     Vbat3 Gain
	GWORD wVbat3Gain;                                          
	///  7. Abbrev: V3OFF; Default: 0(-); DF Addr: 0091h; SRAM Offset: 0091h ; SubClass Offset : 12
	///     Vbat3 Offset
	GSHORT sVbat3Offset;                                       
	///  8. Abbrev: ADC1PGAIN; Default: 991(-); DF Addr: 0093h; SRAM Offset: 0093h ; SubClass Offset : 14
	///     Adc1 P Gain(Current)
	GSHORT sAdc1PGain;                                         
	///  9. Abbrev: ADC1NGAIN; Default: 977(-); DF Addr: 0095h; SRAM Offset: 0095h ; SubClass Offset : 16
	///     Adc1 N Gain(Current)
	GSHORT sAdc1NGain;                                         
	/// 10. Abbrev: ADC1POSOFF; Default: 16(-); DF Addr: 0097h; SRAM Offset: 0097h ; SubClass Offset : 18
	///     Adc1 Pos Offset(Current)
	GSHORT sAdc1PosOffset;                                     
	/// 11. Abbrev: ADC2GAIN; Default: 1000(-); DF Addr: 0099h; SRAM Offset: 0099h ; SubClass Offset : 20
	///     Adc 2 Gain (Vbat1)
	GSHORT sAdc2Gain;                                          
	/// 12. Abbrev: ADC2OFF; Default: 0(-); DF Addr: 009Bh; SRAM Offset: 009Bh ; SubClass Offset : 22
	///     Adc1 Offset
	GSHORT sAdc2Offset;                                        
} PACK SubClassCaliDataType;	///< Total = 24 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #11 => Config
///-------------------------------------------------------------------------------///
typedef struct SubClassConfigSt
{
	///  1. Abbrev: ADCD1; Default: 30(-); DF Addr: 009Dh; SRAM Offset: 009Dh ; SubClass Offset : 0
	///     adc d1 update for IT25
	GSHORT sAdcD1;                                             
	///  2. Abbrev: ADCD2; Default: 1000(-); DF Addr: 009Fh; SRAM Offset: 009Fh ; SubClass Offset : 2
	///     adc d2 update for IT80
	GSHORT sAdcD2;                                             
	///  3. Abbrev: ADCD3; Default: 30719(-); DF Addr: 00A1h; SRAM Offset: 00A1h ; SubClass Offset : 4
	///     adc d3 Used for ADC calibration IT code
	GSHORT sAdcD3;                                             
	///  4. Abbrev: ADCD4; Default: 0(-); DF Addr: 00A3h; SRAM Offset: 00A3h ; SubClass Offset : 6
	///     adc d4 Used for ADC calibration IT code
	GSHORT sAdcD4;                                             
	///  5. Abbrev: ADCD5; Default: 0(-); DF Addr: 00A5h; SRAM Offset: 00A5h ; SubClass Offset : 8
	///     adc d5 Used for ADC calibration IT code
	GSHORT sAdcD5;                                             
} PACK SubClassConfigType;	///< Total = 10 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #12 => TempModel
///-------------------------------------------------------------------------------///
typedef struct SubClassTempModelSt
{
	///  1. Abbrev: R; Default: 10000(-); DF Addr: 00A7h; SRAM Offset: 00A7h ; SubClass Offset : 0
	///     R
	GWORD wR;                                                  
	///  2. Abbrev: RTTAB0; Default: 44604(-); DF Addr: 00A9h; SRAM Offset: 00A9h ; SubClass Offset : 2
	///     rtTable0 : -10(ET)
	GWORD wRtTable0;                                           
	///  3. Abbrev: RTTAB1; Default: 35360(-); DF Addr: 00ABh; SRAM Offset: 00ABh ; SubClass Offset : 4
	///     rtTable1 : -5(ET)
	GWORD wRtTable1;                                           
	///  4. Abbrev: RTTAB2; Default: 28234(-); DF Addr: 00ADh; SRAM Offset: 00ADh ; SubClass Offset : 6
	///     rtTable2 : 0 (ET)
	GWORD wRtTable2;                                           
	///  5. Abbrev: RTTAB3; Default: 22699(-); DF Addr: 00AFh; SRAM Offset: 00AFh ; SubClass Offset : 8
	///     rtTable3 : 5 (ET)
	GWORD wRtTable3;                                           
	///  6. Abbrev: RTTAB4; Default: 18370(-); DF Addr: 00B1h; SRAM Offset: 00B1h ; SubClass Offset : 10
	///     rtTable4 : 10 (ET)
	GWORD wRtTable4;                                           
	///  7. Abbrev: RTTAB5; Default: 14960(-); DF Addr: 00B3h; SRAM Offset: 00B3h ; SubClass Offset : 12
	///     rtTable5 : 15 (ET)
	GWORD wRtTable5;                                           
	///  8. Abbrev: RTTAB6; Default: 12257(-); DF Addr: 00B5h; SRAM Offset: 00B5h ; SubClass Offset : 14
	///     rtTable6 : 20 (ET)
	GWORD wRtTable6;                                           
	///  9. Abbrev: RTTAB7; Default: 10100(-); DF Addr: 00B7h; SRAM Offset: 00B7h ; SubClass Offset : 16
	///     rtTable7 : 25 (ET)
	GWORD wRtTable7;                                           
	/// 10. Abbrev: RTTAB8; Default: 8400(-); DF Addr: 00B9h; SRAM Offset: 00B9h ; SubClass Offset : 18
	///     rtTable8 : 30 (ET)
	GWORD wRtTable8;                                           
	/// 11. Abbrev: RTTAB9; Default: 7022(-); DF Addr: 00BBh; SRAM Offset: 00BBh ; SubClass Offset : 20
	///     rtTable9 : 35 (ET)
	GWORD wRtTable9;                                           
	/// 12. Abbrev: RTTAB10; Default: 5900(-); DF Addr: 00BDh; SRAM Offset: 00BDh ; SubClass Offset : 22
	///     rtTable10 : 40 (ET)
	GWORD wRtTable10;                                          
	/// 13. Abbrev: RTTAB11; Default: 4981(-); DF Addr: 00BFh; SRAM Offset: 00BFh ; SubClass Offset : 24
	///     rtTable11 : 45 (ET)
	GWORD wRtTable11;                                          
	/// 14. Abbrev: RTTAB12; Default: 4224(-); DF Addr: 00C1h; SRAM Offset: 00C1h ; SubClass Offset : 26
	///     rtTable12 : 50 (ET)
	GWORD wRtTable12;                                          
	/// 15. Abbrev: RTTAB13; Default: 3598(-); DF Addr: 00C3h; SRAM Offset: 00C3h ; SubClass Offset : 28
	///     rtTable13 : 55 (ET)
	GWORD wRtTable13;                                          
	/// 16. Abbrev: RTTAB14; Default: 3078(-); DF Addr: 00C5h; SRAM Offset: 00C5h ; SubClass Offset : 30
	///     rtTable14 : 60 (ET)
	GWORD wRtTable14;                                          
	/// 17. Abbrev: RTTAB15; Default: 2644(-); DF Addr: 00C7h; SRAM Offset: 00C7h ; SubClass Offset : 32
	///     rtTable15 : 65 (ET)
	GWORD wRtTable15;                                          
	/// 18. Abbrev: RTTAB16; Default: 2280(-); DF Addr: 00C9h; SRAM Offset: 00C9h ; SubClass Offset : 34
	///     rtTable16 : 70 (ET)
	GWORD wRtTable16;                                          
	/// 19. Abbrev: RTTAB17; Default: 1973(-); DF Addr: 00CBh; SRAM Offset: 00CBh ; SubClass Offset : 36
	///     rtTable17 : 75 (ET)
	GWORD wRtTable17;                                          
	/// 20. Abbrev: RTTAB18; Default: 1714(-); DF Addr: 00CDh; SRAM Offset: 00CDh ; SubClass Offset : 38
	///     rtTable18 : 80 (ET)
	GWORD wRtTable18;                                          
} PACK SubClassTempModelType;	///< Total = 40 Bytes

/// =============================================================///
/// Data flash Class #6 => Calibration
/// =============================================================///
typedef struct ClassCalibrationSt
{
	SubClassCaliDataType scCaliData;
	SubClassConfigType scConfig;
	SubClassTempModelType scTempModel;
} PACK ClassCalibrationType;	///< Total = 74 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #13 => OcvTableT45
///-------------------------------------------------------------------------------///
typedef struct SubClassOcvTableT45St
{
	///  1. Abbrev: T45OCV100; Default: 4188(mV); DF Addr: 00CFh; SRAM Offset: 00CFh ; SubClass Offset : 0
	///     OCV at 45oC 100% voltage
	GWORD wOcv100;                                             
	///  2. Abbrev: T45OCV95; Default: 4126(mV); DF Addr: 00D1h; SRAM Offset: 00D1h ; SubClass Offset : 2
	///     OCV at 45oC 95% voltage
	GWORD wOcv95;                                              
	///  3. Abbrev: T45OCV90; Default: 4075(mV); DF Addr: 00D3h; SRAM Offset: 00D3h ; SubClass Offset : 4
	///     OCV at 45oC 90% voltage
	GWORD wOcv90;                                              
	///  4. Abbrev: T45OCV85; Default: 4027(mV); DF Addr: 00D5h; SRAM Offset: 00D5h ; SubClass Offset : 6
	///     OCV at 45oC 85% voltage
	GWORD wOcv85;                                              
	///  5. Abbrev: T45OCV80; Default: 3984(mV); DF Addr: 00D7h; SRAM Offset: 00D7h ; SubClass Offset : 8
	///     OCV at 45oC 80% voltage
	GWORD wOcv80;                                              
	///  6. Abbrev: T45OCV75; Default: 3943(mV); DF Addr: 00D9h; SRAM Offset: 00D9h ; SubClass Offset : 10
	///     OCV at 45oC 75% voltage
	GWORD wOcv75;                                              
	///  7. Abbrev: T45OCV70; Default: 3905(mV); DF Addr: 00DBh; SRAM Offset: 00DBh ; SubClass Offset : 12
	///     OCV at 45oC 70% voltage
	GWORD wOcv70;                                              
	///  8. Abbrev: T45OCV65; Default: 3868(mV); DF Addr: 00DDh; SRAM Offset: 00DDh ; SubClass Offset : 14
	///     OCV at 45oC 65% voltage
	GWORD wOcv65;                                              
	///  9. Abbrev: T45OCV60; Default: 3824(mV); DF Addr: 00DFh; SRAM Offset: 00DFh ; SubClass Offset : 16
	///     OCV at 45oC 60% voltage
	GWORD wOcv60;                                              
	/// 10. Abbrev: T45OCV55; Default: 3797(mV); DF Addr: 00E1h; SRAM Offset: 00E1h ; SubClass Offset : 18
	///     OCV at 45oC 55% voltage
	GWORD wOcv55;                                              
	/// 11. Abbrev: T45OCV50; Default: 3781(mV); DF Addr: 00E3h; SRAM Offset: 00E3h ; SubClass Offset : 20
	///     OCV at 45oC 50% voltage
	GWORD wOcv50;                                              
	/// 12. Abbrev: T45OCV45; Default: 3772(mV); DF Addr: 00E5h; SRAM Offset: 00E5h ; SubClass Offset : 22
	///     OCV at 45oC 45% voltage
	GWORD wOcv45;                                              
	/// 13. Abbrev: T45OCV40; Default: 3766(mV); DF Addr: 00E7h; SRAM Offset: 00E7h ; SubClass Offset : 24
	///     OCV at 45oC 40% voltage
	GWORD wOcv40;                                              
	/// 14. Abbrev: T45OCV35; Default: 3760(mV); DF Addr: 00E9h; SRAM Offset: 00E9h ; SubClass Offset : 26
	///     OCV at 45oC 35% voltage
	GWORD wOcv35;                                              
	/// 15. Abbrev: T45OCV30; Default: 3744(mV); DF Addr: 00EBh; SRAM Offset: 00EBh ; SubClass Offset : 28
	///     OCV at 45oC 30% voltage
	GWORD wOcv30;                                              
	/// 16. Abbrev: T45OCV25; Default: 3711(mV); DF Addr: 00EDh; SRAM Offset: 00EDh ; SubClass Offset : 30
	///     OCV at 45oC 25% voltage
	GWORD wOcv25;                                              
	/// 17. Abbrev: T45OCV20; Default: 3647(mV); DF Addr: 00EFh; SRAM Offset: 00EFh ; SubClass Offset : 32
	///     OCV at 45oC 20% voltage
	GWORD wOcv20;                                              
	/// 18. Abbrev: T45OCV15; Default: 3576(mV); DF Addr: 00F1h; SRAM Offset: 00F1h ; SubClass Offset : 34
	///     OCV at 45oC 15% voltage
	GWORD wOcv15;                                              
	/// 19. Abbrev: T45OCV10; Default: 3525(mV); DF Addr: 00F3h; SRAM Offset: 00F3h ; SubClass Offset : 36
	///     OCV at 45oC 10% voltage
	GWORD wOcv10;                                              
	/// 20. Abbrev: T45OCV5; Default: 3475(mV); DF Addr: 00F5h; SRAM Offset: 00F5h ; SubClass Offset : 38
	///     OCV at 45oC 5% voltage
	GWORD wOcv5;                                               
	/// 21. Abbrev: T45OCV0; Default: 2998(mV); DF Addr: 00F7h; SRAM Offset: 00F7h ; SubClass Offset : 40
	///     OCV at 45oC 0% voltage
	GWORD wOcv0;                                               
} PACK SubClassOcvTableT45Type;	///< Total = 42 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #14 => RTableT45
///-------------------------------------------------------------------------------///
typedef struct SubClassRTableT45St
{
	///  1. Abbrev: T45R100; Default: 65(0.1mOhm); DF Addr: 00F9h; SRAM Offset: 00F9h ; SubClass Offset : 0
	///     Impedence at 45oC at 100%
	GBYTE bR100;                                               
	///  2. Abbrev: T45R95; Default: 77(0.1mOhm); DF Addr: 00FAh; SRAM Offset: 00FAh ; SubClass Offset : 1
	///     Impedence at 45oC at 95%
	GBYTE bR95;                                                
	///  3. Abbrev: T45R90; Default: 78(0.1mOhm); DF Addr: 00FBh; SRAM Offset: 00FBh ; SubClass Offset : 2
	///     Impedence at 45oC at 90%
	GBYTE bR90;                                                
	///  4. Abbrev: T45R85; Default: 80(0.1mOhm); DF Addr: 00FCh; SRAM Offset: 00FCh ; SubClass Offset : 3
	///     Impedence at 45oC at 85%
	GBYTE bR85;                                                
	///  5. Abbrev: T45R80; Default: 82(0.1mOhm); DF Addr: 00FDh; SRAM Offset: 00FDh ; SubClass Offset : 4
	///     Impedence at 45oC at 80%
	GBYTE bR80;                                                
	///  6. Abbrev: T45R75; Default: 84(0.1mOhm); DF Addr: 00FEh; SRAM Offset: 00FEh ; SubClass Offset : 5
	///     Impedence at 45oC at 75%
	GBYTE bR75;                                                
	///  7. Abbrev: T45R70; Default: 85(0.1mOhm); DF Addr: 00FFh; SRAM Offset: 00FFh ; SubClass Offset : 6
	///     Impedence at 45oC at 70%
	GBYTE bR70;                                                
	///  8. Abbrev: T45R65; Default: 85(0.1mOhm); DF Addr: 0100h; SRAM Offset: 0100h ; SubClass Offset : 7
	///     Impedence at 45oC at 65%
	GBYTE bR65;                                                
	///  9. Abbrev: T45R60; Default: 77(0.1mOhm); DF Addr: 0101h; SRAM Offset: 0101h ; SubClass Offset : 8
	///     Impedence at 45oC at 60%
	GBYTE bR60;                                                
	/// 10. Abbrev: T45R55; Default: 77(0.1mOhm); DF Addr: 0102h; SRAM Offset: 0102h ; SubClass Offset : 9
	///     Impedence at 45oC at 55%
	GBYTE bR55;                                                
	/// 11. Abbrev: T45R50; Default: 79(0.1mOhm); DF Addr: 0103h; SRAM Offset: 0103h ; SubClass Offset : 10
	///     Impedence at 45oC at 50%
	GBYTE bR50;                                                
	/// 12. Abbrev: T45R45; Default: 82(0.1mOhm); DF Addr: 0104h; SRAM Offset: 0104h ; SubClass Offset : 11
	///     Impedence at 45oC at 45%
	GBYTE bR45;                                                
	/// 13. Abbrev: T45R40; Default: 87(0.1mOhm); DF Addr: 0105h; SRAM Offset: 0105h ; SubClass Offset : 12
	///     Impedence at 45oC at 40%
	GBYTE bR40;                                                
	/// 14. Abbrev: T45R35; Default: 91(0.1mOhm); DF Addr: 0106h; SRAM Offset: 0106h ; SubClass Offset : 13
	///     Impedence at 45oC at 35%
	GBYTE bR35;                                                
	/// 15. Abbrev: T45R30; Default: 93(0.1mOhm); DF Addr: 0107h; SRAM Offset: 0107h ; SubClass Offset : 14
	///     Impedence at 45oC at 30%
	GBYTE bR30;                                                
	/// 16. Abbrev: T45R25; Default: 92(0.1mOhm); DF Addr: 0108h; SRAM Offset: 0108h ; SubClass Offset : 15
	///     Impedence at 45oC at 25%
	GBYTE bR25;                                                
	/// 17. Abbrev: T45R20; Default: 87(0.1mOhm); DF Addr: 0109h; SRAM Offset: 0109h ; SubClass Offset : 16
	///     Impedence at 45oC at 20%
	GBYTE bR20;                                                
	/// 18. Abbrev: T45R15; Default: 82(0.1mOhm); DF Addr: 010Ah; SRAM Offset: 010Ah ; SubClass Offset : 17
	///     Impedence at 45oC at 15%
	GBYTE bR15;                                                
	/// 19. Abbrev: T45R10; Default: 83(0.1mOhm); DF Addr: 010Bh; SRAM Offset: 010Bh ; SubClass Offset : 18
	///     Impedence at 45oC at 10%
	GBYTE bR10;                                                
	/// 20. Abbrev: T45R5; Default: 90(0.1mOhm); DF Addr: 010Ch; SRAM Offset: 010Ch ; SubClass Offset : 19
	///     Impedence at 45oC at 5%
	GBYTE bR5;                                                 
	/// 21. Abbrev: T45R0; Default: 1(0.1mOhm); DF Addr: 010Dh; SRAM Offset: 010Dh ; SubClass Offset : 20
	///     Impedence at 45oC at 0%
	GBYTE bR0;                                                 
} PACK SubClassRTableT45Type;	///< Total = 21 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #15 => OcvTableT25
///-------------------------------------------------------------------------------///
typedef struct SubClassOcvTableT25St
{
	///  1. Abbrev: T25OCV100; Default: 4141(mV); DF Addr: 010Eh; SRAM Offset: 010Eh ; SubClass Offset : 0
	///     OCV at 25oC 100% voltage
	GWORD wOcv100;                                             
	///  2. Abbrev: T25OCV95; Default: 4121(mV); DF Addr: 0110h; SRAM Offset: 0110h ; SubClass Offset : 2
	///     OCV at 25oC 95% voltage
	GWORD wOcv95;                                              
	///  3. Abbrev: T25OCV90; Default: 4071(mV); DF Addr: 0112h; SRAM Offset: 0112h ; SubClass Offset : 4
	///     OCV at 25oC 90% voltage
	GWORD wOcv90;                                              
	///  4. Abbrev: T25OCV85; Default: 4021(mV); DF Addr: 0114h; SRAM Offset: 0114h ; SubClass Offset : 6
	///     OCV at 25oC 85% voltage
	GWORD wOcv85;                                              
	///  5. Abbrev: T25OCV80; Default: 3977(mV); DF Addr: 0116h; SRAM Offset: 0116h ; SubClass Offset : 8
	///     OCV at 25oC 80% voltage
	GWORD wOcv80;                                              
	///  6. Abbrev: T25OCV75; Default: 3937(mV); DF Addr: 0118h; SRAM Offset: 0118h ; SubClass Offset : 10
	///     OCV at 25oC 75% voltage
	GWORD wOcv75;                                              
	///  7. Abbrev: T25OCV70; Default: 3897(mV); DF Addr: 011Ah; SRAM Offset: 011Ah ; SubClass Offset : 12
	///     OCV at 25oC 70% voltage
	GWORD wOcv70;                                              
	///  8. Abbrev: T25OCV65; Default: 3858(mV); DF Addr: 011Ch; SRAM Offset: 011Ch ; SubClass Offset : 14
	///     OCV at 25oC 65% voltage
	GWORD wOcv65;                                              
	///  9. Abbrev: T25OCV60; Default: 3823(mV); DF Addr: 011Eh; SRAM Offset: 011Eh ; SubClass Offset : 16
	///     OCV at 25oC 60% voltage
	GWORD wOcv60;                                              
	/// 10. Abbrev: T25OCV55; Default: 3801(mV); DF Addr: 0120h; SRAM Offset: 0120h ; SubClass Offset : 18
	///     OCV at 25oC 55% voltage
	GWORD wOcv55;                                              
	/// 11. Abbrev: T25OCV50; Default: 3788(mV); DF Addr: 0122h; SRAM Offset: 0122h ; SubClass Offset : 20
	///     OCV at 25oC 50% voltage
	GWORD wOcv50;                                              
	/// 12. Abbrev: T25OCV45; Default: 3781(mV); DF Addr: 0124h; SRAM Offset: 0124h ; SubClass Offset : 22
	///     OCV at 25oC 45% voltage
	GWORD wOcv45;                                              
	/// 13. Abbrev: T25OCV40; Default: 3775(mV); DF Addr: 0126h; SRAM Offset: 0126h ; SubClass Offset : 24
	///     OCV at 25oC 40% voltage
	GWORD wOcv40;                                              
	/// 14. Abbrev: T25OCV35; Default: 3766(mV); DF Addr: 0128h; SRAM Offset: 0128h ; SubClass Offset : 26
	///     OCV at 25oC 35% voltage
	GWORD wOcv35;                                              
	/// 15. Abbrev: T25OCV30; Default: 3745(mV); DF Addr: 012Ah; SRAM Offset: 012Ah ; SubClass Offset : 28
	///     OCV at 25oC 30% voltage
	GWORD wOcv30;                                              
	/// 16. Abbrev: T25OCV25; Default: 3704(mV); DF Addr: 012Ch; SRAM Offset: 012Ch ; SubClass Offset : 30
	///     OCV at 25oC 25% voltage
	GWORD wOcv25;                                              
	/// 17. Abbrev: T25OCV20; Default: 3638(mV); DF Addr: 012Eh; SRAM Offset: 012Eh ; SubClass Offset : 32
	///     OCV at 25oC 20% voltage
	GWORD wOcv20;                                              
	/// 18. Abbrev: T25OCV15; Default: 3570(mV); DF Addr: 0130h; SRAM Offset: 0130h ; SubClass Offset : 34
	///     OCV at 25oC 15% voltage
	GWORD wOcv15;                                              
	/// 19. Abbrev: T25OCV10; Default: 3523(mV); DF Addr: 0132h; SRAM Offset: 0132h ; SubClass Offset : 36
	///     OCV at 25oC 10% voltage
	GWORD wOcv10;                                              
	/// 20. Abbrev: T25OCV5; Default: 3464(mV); DF Addr: 0134h; SRAM Offset: 0134h ; SubClass Offset : 38
	///     OCV at 25oC 5% voltage
	GWORD wOcv5;                                               
	/// 21. Abbrev: T25OCV0; Default: 2998(mV); DF Addr: 0136h; SRAM Offset: 0136h ; SubClass Offset : 40
	///     OCV at 25oC 0% voltage
	GWORD wOcv0;                                               
} PACK SubClassOcvTableT25Type;	///< Total = 42 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #16 => RTableT25
///-------------------------------------------------------------------------------///
typedef struct SubClassRTableT25St
{
	///  1. Abbrev: T25R100; Default: 0(0.1mOhm); DF Addr: 0138h; SRAM Offset: 0138h ; SubClass Offset : 0
	///     Impedence at 25oC at 100%
	GBYTE bR100;                                               
	///  2. Abbrev: T25R95; Default: 111(0.1mOhm); DF Addr: 0139h; SRAM Offset: 0139h ; SubClass Offset : 1
	///     Impedence at 25oC at 95%
	GBYTE bR95;                                                
	///  3. Abbrev: T25R90; Default: 111(0.1mOhm); DF Addr: 013Ah; SRAM Offset: 013Ah ; SubClass Offset : 2
	///     Impedence at 25oC at 90%
	GBYTE bR90;                                                
	///  4. Abbrev: T25R85; Default: 110(0.1mOhm); DF Addr: 013Bh; SRAM Offset: 013Bh ; SubClass Offset : 3
	///     Impedence at 25oC at 85%
	GBYTE bR85;                                                
	///  5. Abbrev: T25R80; Default: 111(0.1mOhm); DF Addr: 013Ch; SRAM Offset: 013Ch ; SubClass Offset : 4
	///     Impedence at 25oC at 80%
	GBYTE bR80;                                                
	///  6. Abbrev: T25R75; Default: 111(0.1mOhm); DF Addr: 013Dh; SRAM Offset: 013Dh ; SubClass Offset : 5
	///     Impedence at 25oC at 75%
	GBYTE bR75;                                                
	///  7. Abbrev: T25R70; Default: 111(0.1mOhm); DF Addr: 013Eh; SRAM Offset: 013Eh ; SubClass Offset : 6
	///     Impedence at 25oC at 70%
	GBYTE bR70;                                                
	///  8. Abbrev: T25R65; Default: 107(0.1mOhm); DF Addr: 013Fh; SRAM Offset: 013Fh ; SubClass Offset : 7
	///     Impedence at 25oC at 65%
	GBYTE bR65;                                                
	///  9. Abbrev: T25R60; Default: 104(0.1mOhm); DF Addr: 0140h; SRAM Offset: 0140h ; SubClass Offset : 8
	///     Impedence at 25oC at 60%
	GBYTE bR60;                                                
	/// 10. Abbrev: T25R55; Default: 105(0.1mOhm); DF Addr: 0141h; SRAM Offset: 0141h ; SubClass Offset : 9
	///     Impedence at 25oC at 55%
	GBYTE bR55;                                                
	/// 11. Abbrev: T25R50; Default: 109(0.1mOhm); DF Addr: 0142h; SRAM Offset: 0142h ; SubClass Offset : 10
	///     Impedence at 25oC at 50%
	GBYTE bR50;                                                
	/// 12. Abbrev: T25R45; Default: 114(0.1mOhm); DF Addr: 0143h; SRAM Offset: 0143h ; SubClass Offset : 11
	///     Impedence at 25oC at 45%
	GBYTE bR45;                                                
	/// 13. Abbrev: T25R40; Default: 120(0.1mOhm); DF Addr: 0144h; SRAM Offset: 0144h ; SubClass Offset : 12
	///     Impedence at 25oC at 40%
	GBYTE bR40;                                                
	/// 14. Abbrev: T25R35; Default: 126(0.1mOhm); DF Addr: 0145h; SRAM Offset: 0145h ; SubClass Offset : 13
	///     Impedence at 25oC at 35%
	GBYTE bR35;                                                
	/// 15. Abbrev: T25R30; Default: 129(0.1mOhm); DF Addr: 0146h; SRAM Offset: 0146h ; SubClass Offset : 14
	///     Impedence at 25oC at 30%
	GBYTE bR30;                                                
	/// 16. Abbrev: T25R25; Default: 130(0.1mOhm); DF Addr: 0147h; SRAM Offset: 0147h ; SubClass Offset : 15
	///     Impedence at 25oC at 25%
	GBYTE bR25;                                                
	/// 17. Abbrev: T25R20; Default: 121(0.1mOhm); DF Addr: 0148h; SRAM Offset: 0148h ; SubClass Offset : 16
	///     Impedence at 25oC at 20%
	GBYTE bR20;                                                
	/// 18. Abbrev: T25R15; Default: 113(0.1mOhm); DF Addr: 0149h; SRAM Offset: 0149h ; SubClass Offset : 17
	///     Impedence at 25oC at 15%
	GBYTE bR15;                                                
	/// 19. Abbrev: T25R10; Default: 118(0.1mOhm); DF Addr: 014Ah; SRAM Offset: 014Ah ; SubClass Offset : 18
	///     Impedence at 25oC at 10%
	GBYTE bR10;                                                
	/// 20. Abbrev: T25R5; Default: 130(0.1mOhm); DF Addr: 014Bh; SRAM Offset: 014Bh ; SubClass Offset : 19
	///     Impedence at 25oC at 5%
	GBYTE bR5;                                                 
	/// 21. Abbrev: T25R0; Default: 0(0.1mOhm); DF Addr: 014Ch; SRAM Offset: 014Ch ; SubClass Offset : 20
	///     Impedence at 25oC at 0%
	GBYTE bR0;                                                 
} PACK SubClassRTableT25Type;	///< Total = 21 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #17 => OcvTableT15
///-------------------------------------------------------------------------------///
typedef struct SubClassOcvTableT15St
{
	///  1. Abbrev: T15OCV100; Default: 4176(mV); DF Addr: 014Dh; SRAM Offset: 014Dh ; SubClass Offset : 0
	///     OCV at 15oC 100% voltage
	GWORD wOcv100;                                             
	///  2. Abbrev: T15OCV95; Default: 4062(mV); DF Addr: 014Fh; SRAM Offset: 014Fh ; SubClass Offset : 2
	///     OCV at 15oC 95% voltage
	GWORD wOcv95;                                              
	///  3. Abbrev: T15OCV90; Default: 4010(mV); DF Addr: 0151h; SRAM Offset: 0151h ; SubClass Offset : 4
	///     OCV at 15oC 90% voltage
	GWORD wOcv90;                                              
	///  4. Abbrev: T15OCV85; Default: 3969(mV); DF Addr: 0153h; SRAM Offset: 0153h ; SubClass Offset : 6
	///     OCV at 15oC 85% voltage
	GWORD wOcv85;                                              
	///  5. Abbrev: T15OCV80; Default: 3931(mV); DF Addr: 0155h; SRAM Offset: 0155h ; SubClass Offset : 8
	///     OCV at 15oC 80% voltage
	GWORD wOcv80;                                              
	///  6. Abbrev: T15OCV75; Default: 3892(mV); DF Addr: 0157h; SRAM Offset: 0157h ; SubClass Offset : 10
	///     OCV at 15oC 75% voltage
	GWORD wOcv75;                                              
	///  7. Abbrev: T15OCV70; Default: 3855(mV); DF Addr: 0159h; SRAM Offset: 0159h ; SubClass Offset : 12
	///     OCV at 15oC 70% voltage
	GWORD wOcv70;                                              
	///  8. Abbrev: T15OCV65; Default: 3824(mV); DF Addr: 015Bh; SRAM Offset: 015Bh ; SubClass Offset : 14
	///     OCV at 15oC 65% voltage
	GWORD wOcv65;                                              
	///  9. Abbrev: T15OCV60; Default: 3801(mV); DF Addr: 015Dh; SRAM Offset: 015Dh ; SubClass Offset : 16
	///     OCV at 15oC 60% voltage
	GWORD wOcv60;                                              
	/// 10. Abbrev: T15OCV55; Default: 3786(mV); DF Addr: 015Fh; SRAM Offset: 015Fh ; SubClass Offset : 18
	///     OCV at 15oC 55% voltage
	GWORD wOcv55;                                              
	/// 11. Abbrev: T15OCV50; Default: 3777(mV); DF Addr: 0161h; SRAM Offset: 0161h ; SubClass Offset : 20
	///     OCV at 15oC 50% voltage
	GWORD wOcv50;                                              
	/// 12. Abbrev: T15OCV45; Default: 3770(mV); DF Addr: 0163h; SRAM Offset: 0163h ; SubClass Offset : 22
	///     OCV at 15oC 45% voltage
	GWORD wOcv45;                                              
	/// 13. Abbrev: T15OCV40; Default: 3763(mV); DF Addr: 0165h; SRAM Offset: 0165h ; SubClass Offset : 24
	///     OCV at 15oC 40% voltage
	GWORD wOcv40;                                              
	/// 14. Abbrev: T15OCV35; Default: 3752(mV); DF Addr: 0167h; SRAM Offset: 0167h ; SubClass Offset : 26
	///     OCV at 15oC 35% voltage
	GWORD wOcv35;                                              
	/// 15. Abbrev: T15OCV30; Default: 3724(mV); DF Addr: 0169h; SRAM Offset: 0169h ; SubClass Offset : 28
	///     OCV at 15oC 30% voltage
	GWORD wOcv30;                                              
	/// 16. Abbrev: T15OCV25; Default: 3675(mV); DF Addr: 016Bh; SRAM Offset: 016Bh ; SubClass Offset : 30
	///     OCV at 15oC 25% voltage
	GWORD wOcv25;                                              
	/// 17. Abbrev: T15OCV20; Default: 3610(mV); DF Addr: 016Dh; SRAM Offset: 016Dh ; SubClass Offset : 32
	///     OCV at 15oC 20% voltage
	GWORD wOcv20;                                              
	/// 18. Abbrev: T15OCV15; Default: 3550(mV); DF Addr: 016Fh; SRAM Offset: 016Fh ; SubClass Offset : 34
	///     OCV at 15oC 15% voltage
	GWORD wOcv15;                                              
	/// 19. Abbrev: T15OCV10; Default: 3498(mV); DF Addr: 0171h; SRAM Offset: 0171h ; SubClass Offset : 36
	///     OCV at 15oC 10% voltage
	GWORD wOcv10;                                              
	/// 20. Abbrev: T15OCV5; Default: 3411(mV); DF Addr: 0173h; SRAM Offset: 0173h ; SubClass Offset : 38
	///     OCV at 15oC 5% voltage
	GWORD wOcv5;                                               
	/// 21. Abbrev: T15OCV0; Default: 2999(mV); DF Addr: 0175h; SRAM Offset: 0175h ; SubClass Offset : 40
	///     OCV at 15oC 0% voltage
	GWORD wOcv0;                                               
} PACK SubClassOcvTableT15Type;	///< Total = 42 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #18 => RTableT15
///-------------------------------------------------------------------------------///
typedef struct SubClassRTableT15St
{
	///  1. Abbrev: T15R100; Default: 88(0.1mOhm); DF Addr: 0177h; SRAM Offset: 0177h ; SubClass Offset : 0
	///     Impedence at 15oC at 100%
	GBYTE bR100;                                               
	///  2. Abbrev: T15R95; Default: 138(0.1mOhm); DF Addr: 0178h; SRAM Offset: 0178h ; SubClass Offset : 1
	///     Impedence at 15oC at 95%
	GBYTE bR95;                                                
	///  3. Abbrev: T15R90; Default: 155(0.1mOhm); DF Addr: 0179h; SRAM Offset: 0179h ; SubClass Offset : 2
	///     Impedence at 15oC at 90%
	GBYTE bR90;                                                
	///  4. Abbrev: T15R85; Default: 156(0.1mOhm); DF Addr: 017Ah; SRAM Offset: 017Ah ; SubClass Offset : 3
	///     Impedence at 15oC at 85%
	GBYTE bR85;                                                
	///  5. Abbrev: T15R80; Default: 157(0.1mOhm); DF Addr: 017Bh; SRAM Offset: 017Bh ; SubClass Offset : 4
	///     Impedence at 15oC at 80%
	GBYTE bR80;                                                
	///  6. Abbrev: T15R75; Default: 156(0.1mOhm); DF Addr: 017Ch; SRAM Offset: 017Ch ; SubClass Offset : 5
	///     Impedence at 15oC at 75%
	GBYTE bR75;                                                
	///  7. Abbrev: T15R70; Default: 154(0.1mOhm); DF Addr: 017Dh; SRAM Offset: 017Dh ; SubClass Offset : 6
	///     Impedence at 15oC at 70%
	GBYTE bR70;                                                
	///  8. Abbrev: T15R65; Default: 155(0.1mOhm); DF Addr: 017Eh; SRAM Offset: 017Eh ; SubClass Offset : 7
	///     Impedence at 15oC at 65%
	GBYTE bR65;                                                
	///  9. Abbrev: T15R60; Default: 159(0.1mOhm); DF Addr: 017Fh; SRAM Offset: 017Fh ; SubClass Offset : 8
	///     Impedence at 15oC at 60%
	GBYTE bR60;                                                
	/// 10. Abbrev: T15R55; Default: 164(0.1mOhm); DF Addr: 0180h; SRAM Offset: 0180h ; SubClass Offset : 9
	///     Impedence at 15oC at 55%
	GBYTE bR55;                                                
	/// 11. Abbrev: T15R50; Default: 173(0.1mOhm); DF Addr: 0181h; SRAM Offset: 0181h ; SubClass Offset : 10
	///     Impedence at 15oC at 50%
	GBYTE bR50;                                                
	/// 12. Abbrev: T15R45; Default: 183(0.1mOhm); DF Addr: 0182h; SRAM Offset: 0182h ; SubClass Offset : 11
	///     Impedence at 15oC at 45%
	GBYTE bR45;                                                
	/// 13. Abbrev: T15R40; Default: 192(0.1mOhm); DF Addr: 0183h; SRAM Offset: 0183h ; SubClass Offset : 12
	///     Impedence at 15oC at 40%
	GBYTE bR40;                                                
	/// 14. Abbrev: T15R35; Default: 202(0.1mOhm); DF Addr: 0184h; SRAM Offset: 0184h ; SubClass Offset : 13
	///     Impedence at 15oC at 35%
	GBYTE bR35;                                                
	/// 15. Abbrev: T15R30; Default: 205(0.1mOhm); DF Addr: 0185h; SRAM Offset: 0185h ; SubClass Offset : 14
	///     Impedence at 15oC at 30%
	GBYTE bR30;                                                
	/// 16. Abbrev: T15R25; Default: 197(0.1mOhm); DF Addr: 0186h; SRAM Offset: 0186h ; SubClass Offset : 15
	///     Impedence at 15oC at 25%
	GBYTE bR25;                                                
	/// 17. Abbrev: T15R20; Default: 181(0.1mOhm); DF Addr: 0187h; SRAM Offset: 0187h ; SubClass Offset : 16
	///     Impedence at 15oC at 20%
	GBYTE bR20;                                                
	/// 18. Abbrev: T15R15; Default: 172(0.1mOhm); DF Addr: 0188h; SRAM Offset: 0188h ; SubClass Offset : 17
	///     Impedence at 15oC at 15%
	GBYTE bR15;                                                
	/// 19. Abbrev: T15R10; Default: 177(0.1mOhm); DF Addr: 0189h; SRAM Offset: 0189h ; SubClass Offset : 18
	///     Impedence at 15oC at 10%
	GBYTE bR10;                                                
	/// 20. Abbrev: T15R5; Default: 175(0.1mOhm); DF Addr: 018Ah; SRAM Offset: 018Ah ; SubClass Offset : 19
	///     Impedence at 15oC at 5%
	GBYTE bR5;                                                 
	/// 21. Abbrev: T15R0; Default: 1(0.1mOhm); DF Addr: 018Bh; SRAM Offset: 018Bh ; SubClass Offset : 20
	///     Impedence at 15oC at 0%
	GBYTE bR0;                                                 
} PACK SubClassRTableT15Type;	///< Total = 21 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #19 => OcvTableT05
///-------------------------------------------------------------------------------///
typedef struct SubClassOcvTableT05St
{
	///  1. Abbrev: T05OCV100; Default: 4171(mV); DF Addr: 018Ch; SRAM Offset: 018Ch ; SubClass Offset : 0
	///     OCV at 5oC 100% voltage
	GWORD wOcv100;                                             
	///  2. Abbrev: T05OCV95; Default: 4117(mV); DF Addr: 018Eh; SRAM Offset: 018Eh ; SubClass Offset : 2
	///     OCV at 5oC 95% voltage
	GWORD wOcv95;                                              
	///  3. Abbrev: T05OCV90; Default: 4077(mV); DF Addr: 0190h; SRAM Offset: 0190h ; SubClass Offset : 4
	///     OCV at 5oC 90% voltage
	GWORD wOcv90;                                              
	///  4. Abbrev: T05OCV85; Default: 4037(mV); DF Addr: 0192h; SRAM Offset: 0192h ; SubClass Offset : 6
	///     OCV at 5oC 85% voltage
	GWORD wOcv85;                                              
	///  5. Abbrev: T05OCV80; Default: 3898(mV); DF Addr: 0194h; SRAM Offset: 0194h ; SubClass Offset : 8
	///     OCV at 5oC 80% voltage
	GWORD wOcv80;                                              
	///  6. Abbrev: T05OCV75; Default: 3790(mV); DF Addr: 0196h; SRAM Offset: 0196h ; SubClass Offset : 10
	///     OCV at 5oC 75% voltage
	GWORD wOcv75;                                              
	///  7. Abbrev: T05OCV70; Default: 3756(mV); DF Addr: 0198h; SRAM Offset: 0198h ; SubClass Offset : 12
	///     OCV at 5oC 70% voltage
	GWORD wOcv70;                                              
	///  8. Abbrev: T05OCV65; Default: 3729(mV); DF Addr: 019Ah; SRAM Offset: 019Ah ; SubClass Offset : 14
	///     OCV at 5oC 65% voltage
	GWORD wOcv65;                                              
	///  9. Abbrev: T05OCV60; Default: 3707(mV); DF Addr: 019Ch; SRAM Offset: 019Ch ; SubClass Offset : 16
	///     OCV at 5oC 60% voltage
	GWORD wOcv60;                                              
	/// 10. Abbrev: T05OCV55; Default: 3705(mV); DF Addr: 019Eh; SRAM Offset: 019Eh ; SubClass Offset : 18
	///     OCV at 5oC 55% voltage
	GWORD wOcv55;                                              
	/// 11. Abbrev: T05OCV50; Default: 3741(mV); DF Addr: 01A0h; SRAM Offset: 01A0h ; SubClass Offset : 20
	///     OCV at 5oC 50% voltage
	GWORD wOcv50;                                              
	/// 12. Abbrev: T05OCV45; Default: 3740(mV); DF Addr: 01A2h; SRAM Offset: 01A2h ; SubClass Offset : 22
	///     OCV at 5oC 45% voltage
	GWORD wOcv45;                                              
	/// 13. Abbrev: T05OCV40; Default: 3728(mV); DF Addr: 01A4h; SRAM Offset: 01A4h ; SubClass Offset : 24
	///     OCV at 5oC 40% voltage
	GWORD wOcv40;                                              
	/// 14. Abbrev: T05OCV35; Default: 3715(mV); DF Addr: 01A6h; SRAM Offset: 01A6h ; SubClass Offset : 26
	///     OCV at 5oC 35% voltage
	GWORD wOcv35;                                              
	/// 15. Abbrev: T05OCV30; Default: 3690(mV); DF Addr: 01A8h; SRAM Offset: 01A8h ; SubClass Offset : 28
	///     OCV at 5oC 30% voltage
	GWORD wOcv30;                                              
	/// 16. Abbrev: T05OCV25; Default: 3654(mV); DF Addr: 01AAh; SRAM Offset: 01AAh ; SubClass Offset : 30
	///     OCV at 5oC 25% voltage
	GWORD wOcv25;                                              
	/// 17. Abbrev: T05OCV20; Default: 3603(mV); DF Addr: 01ACh; SRAM Offset: 01ACh ; SubClass Offset : 32
	///     OCV at 5oC 20% voltage
	GWORD wOcv20;                                              
	/// 18. Abbrev: T05OCV15; Default: 3536(mV); DF Addr: 01AEh; SRAM Offset: 01AEh ; SubClass Offset : 34
	///     OCV at 5oC 15% voltage
	GWORD wOcv15;                                              
	/// 19. Abbrev: T05OCV10; Default: 3460(mV); DF Addr: 01B0h; SRAM Offset: 01B0h ; SubClass Offset : 36
	///     OCV at 5oC 10% voltage
	GWORD wOcv10;                                              
	/// 20. Abbrev: T05OCV5; Default: 3349(mV); DF Addr: 01B2h; SRAM Offset: 01B2h ; SubClass Offset : 38
	///     OCV at 5oC 5% voltage
	GWORD wOcv5;                                               
	/// 21. Abbrev: T05OCV0; Default: 3000(mV); DF Addr: 01B4h; SRAM Offset: 01B4h ; SubClass Offset : 40
	///     OCV at 5oC 0% voltage
	GWORD wOcv0;                                               
} PACK SubClassOcvTableT05Type;	///< Total = 42 Bytes

///-------------------------------------------------------------------------------///
///  Data flash SubClass #20 => RTableT05
///-------------------------------------------------------------------------------///
typedef struct SubClassRTableT05St
{
	///  1. Abbrev: T05R100; Default: 111(0.1mOhm); DF Addr: 01B6h; SRAM Offset: 01B6h ; SubClass Offset : 0
	///     Impedence at 5oC at 100%
	GBYTE bR100;                                               
	///  2. Abbrev: T05R95; Default: 130(0.1mOhm); DF Addr: 01B7h; SRAM Offset: 01B7h ; SubClass Offset : 1
	///     Impedence at 5oC at 95%
	GBYTE bR95;                                                
	///  3. Abbrev: T05R90; Default: 133(0.1mOhm); DF Addr: 01B8h; SRAM Offset: 01B8h ; SubClass Offset : 2
	///     Impedence at 5oC at 90%
	GBYTE bR90;                                                
	///  4. Abbrev: T05R85; Default: 133(0.1mOhm); DF Addr: 01B9h; SRAM Offset: 01B9h ; SubClass Offset : 3
	///     Impedence at 5oC at 85%
	GBYTE bR85;                                                
	///  5. Abbrev: T05R80; Default: 73(0.1mOhm); DF Addr: 01BAh; SRAM Offset: 01BAh ; SubClass Offset : 4
	///     Impedence at 5oC at 80%
	GBYTE bR80;                                                
	///  6. Abbrev: T05R75; Default: 97(0.1mOhm); DF Addr: 01BBh; SRAM Offset: 01BBh ; SubClass Offset : 5
	///     Impedence at 5oC at 75%
	GBYTE bR75;                                                
	///  7. Abbrev: T05R70; Default: 98(0.1mOhm); DF Addr: 01BCh; SRAM Offset: 01BCh ; SubClass Offset : 6
	///     Impedence at 5oC at 70%
	GBYTE bR70;                                                
	///  8. Abbrev: T05R65; Default: 102(0.1mOhm); DF Addr: 01BDh; SRAM Offset: 01BDh ; SubClass Offset : 7
	///     Impedence at 5oC at 65%
	GBYTE bR65;                                                
	///  9. Abbrev: T05R60; Default: 108(0.1mOhm); DF Addr: 01BEh; SRAM Offset: 01BEh ; SubClass Offset : 8
	///     Impedence at 5oC at 60%
	GBYTE bR60;                                                
	/// 10. Abbrev: T05R55; Default: 137(0.1mOhm); DF Addr: 01BFh; SRAM Offset: 01BFh ; SubClass Offset : 9
	///     Impedence at 5oC at 55%
	GBYTE bR55;                                                
	/// 11. Abbrev: T05R50; Default: 216(0.1mOhm); DF Addr: 01C0h; SRAM Offset: 01C0h ; SubClass Offset : 10
	///     Impedence at 5oC at 50%
	GBYTE bR50;                                                
	/// 12. Abbrev: T05R45; Default: 245(0.1mOhm); DF Addr: 01C1h; SRAM Offset: 01C1h ; SubClass Offset : 11
	///     Impedence at 5oC at 45%
	GBYTE bR45;                                                
	/// 13. Abbrev: T05R40; Default: 258(0.1mOhm); DF Addr: 01C2h; SRAM Offset: 01C2h ; SubClass Offset : 12
	///     Impedence at 5oC at 40%
	GBYTE bR40;                                                
	/// 14. Abbrev: T05R35; Default: 270(0.1mOhm); DF Addr: 01C3h; SRAM Offset: 01C3h ; SubClass Offset : 13
	///     Impedence at 5oC at 35%
	GBYTE bR35;                                                
	/// 15. Abbrev: T05R30; Default: 273(0.1mOhm); DF Addr: 01C4h; SRAM Offset: 01C4h ; SubClass Offset : 14
	///     Impedence at 5oC at 30%
	GBYTE bR30;                                                
	/// 16. Abbrev: T05R25; Default: 271(0.1mOhm); DF Addr: 01C5h; SRAM Offset: 01C5h ; SubClass Offset : 15
	///     Impedence at 5oC at 25%
	GBYTE bR25;                                                
	/// 17. Abbrev: T05R20; Default: 260(0.1mOhm); DF Addr: 01C6h; SRAM Offset: 01C6h ; SubClass Offset : 16
	///     Impedence at 5oC at 20%
	GBYTE bR20;                                                
	/// 18. Abbrev: T05R15; Default: 241(0.1mOhm); DF Addr: 01C7h; SRAM Offset: 01C7h ; SubClass Offset : 17
	///     Impedence at 5oC at 15%
	GBYTE bR15;                                                
	/// 19. Abbrev: T05R10; Default: 226(0.1mOhm); DF Addr: 01C8h; SRAM Offset: 01C8h ; SubClass Offset : 18
	///     Impedence at 5oC at 10%
	GBYTE bR10;                                                
	/// 20. Abbrev: T05R5; Default: 196(0.1mOhm); DF Addr: 01C9h; SRAM Offset: 01C9h ; SubClass Offset : 19
	///     Impedence at 5oC at 5%
	GBYTE bR5;                                                 
	/// 21. Abbrev: T05R0; Default: 1(0.1mOhm); DF Addr: 01CAh; SRAM Offset: 01CAh ; SubClass Offset : 20
	///     Impedence at 5oC at 0%
	GBYTE bR0;                                                 
} PACK SubClassRTableT05Type;	///< Total = 21 Bytes

/// =============================================================///
/// Data flash Class #7 => Capacity
/// =============================================================///
typedef struct ClassCapacitySt
{
	SubClassOcvTableT45Type scOcvTableT45;
	SubClassRTableT45Type scRTableT45;
	SubClassOcvTableT25Type scOcvTableT25;
	SubClassRTableT25Type scRTableT25;
	SubClassOcvTableT15Type scOcvTableT15;
	SubClassRTableT15Type scRTableT15;
	SubClassOcvTableT05Type scOcvTableT05;
	SubClassRTableT05Type scRTableT05;
} PACK ClassCapacityType;	///< Total = 252 Bytes

/// =============================================================///
/// Data Flash Structure
/// =============================================================////
typedef struct GlobalDFVarSt
{
	///-------------------------------------------------------------------------------///
	/// Data flash PAGE#00 (459 Bytes)
	///-------------------------------------------------------------------------------///
	Class1stLevelSafetyClassType  c1stLevelSafetyClass;
	ClassChargeControlType  cChargeControl;
	ClassSBSConfigurationType  cSBSConfiguration;
	ClassSystemDataType  cSystemData;
	ClassConfigurationType  cConfiguration;
	ClassGasGaugingType  cGasGauging;
	ClassCalibrationType  cCalibration;
	ClassCapacityType  cCapacity;
} PACK GlobalDFVarType;

#ifdef FEATURE_PLAT_WINDOWS
  #pragma pack(pop)   ///< restore original alignment from stack 
#endif ///< for FEATURE_PLAT_WINDOWS

