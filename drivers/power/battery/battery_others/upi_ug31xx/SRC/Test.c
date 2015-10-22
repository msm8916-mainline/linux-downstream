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

#ifdef FEATURE_TEST_PRINT_LOG
	

#endif ///< for FEATURE_TEST_PRINT_LOG

#ifdef FEATURE_TEST_MAINLOOP
/**
 * @brief Test OTP
 *
 *
 * @return NULL
 *
 */
GVOID GTestOTPRead()
{
	GBYTE bReg;
	///-----------------------------------------------------------------///
	/// 1. Read IC internal Registers
	///-----------------------------------------------------------------///
	bReg = READ_REG(REG_MODE);
	printf("REG_MODE = %02X\n", bReg);	

	UpiGauge.bRegIntrStatusPre = UpiGauge.bRegIntrStatus;
	UpiGauge.bRegIntrStatus    = READ_REG(REG_INTR_STATUS);

	///--------------------------------///
	/// Check if I2C OK
	///--------------------------------///
	if(UpiGauge.wErrorStatus & GAUGE_ERR_I2C_FAIL)
	{
		SYSReset();
		UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
		GLOGD("ERROR: I2C FAIL!!!\n");			
	}

	///--------------------------------///
	/// Read internal status
	///--------------------------------///
	GLOGD("REG_INTR_STATUS = %02x\n", UpiGauge.bRegIntrStatus);		
	if((UpiGauge.bRegIntrStatus & INTR_STATUS_LVD_STS) &&
			 (!(UpiGauge.bRegIntrStatusPre & INTR_STATUS_LVD_STS)))
	{
		SYSReset();
		UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
		GLOGD("NOTE : LEAVE LOW VOLTAGE\n");				
	}

	///--------------------------------///
	/// Check low voltage
	///--------------------------------///
	if(!(UpiGauge.bRegIntrStatus & INTR_STATUS_LVD_STS))
	{			
		UpiGauge.wGaugeStatus |= GAUGE_STATUS_SKIP_ROUND;
		UpiGauge.wErrorStatus |= GAUGE_ERR_LV_FAIL;
		GLOGD("ERROR: LOW VOLTAGE FAIL!!!\n");			
	}
	else
	{
		UpiGauge.wErrorStatus &= ~GAUGE_ERR_LV_FAIL;
	}	

	SYSReadOTP();
	GLOGD("OTP1 : %02X %02X %02X %02X \n", UpiGauge.otp.otp1[0], UpiGauge.otp.otp1[1], UpiGauge.otp.otp1[2], UpiGauge.otp.otp1[3])	;
	GLOGD("OTP2 : %02X %02X %02X %02X \n", UpiGauge.otp.otp2[0], UpiGauge.otp.otp2[1], UpiGauge.otp.otp2[2], UpiGauge.otp.otp1[3])	;
	GLOGD("OTP3 : %02X %02X %02X %02X \n", UpiGauge.otp.otp3[0], UpiGauge.otp.otp3[1], UpiGauge.otp.otp3[2], UpiGauge.otp.otp1[3])	;
	GLOGD("OTP4 : %02X %02X %02X %02X \n", UpiGauge.otp.otp4[0], UpiGauge.otp.otp4[1], UpiGauge.otp.otp4[2], UpiGauge.otp.otp1[3])	;
	GLOGD("OTP5 : %02X %02X %02X %02X \n", UpiGauge.otp.otp5[0], UpiGauge.otp.otp5[1], UpiGauge.otp.otp5[2], UpiGauge.otp.otp1[3])	;
	GLOGD("OTP6 : %02X %02X %02X %02X \n", UpiGauge.otp.otp6[0], UpiGauge.otp.otp6[1], UpiGauge.otp.otp6[2], UpiGauge.otp.otp1[3])	;
}

/**
 * @brief Test Mainloop
 *
 *  MainLoop of Test
 *  
 * @return NULL
 *
 */
GVOID GTestMainLoop(GVOID)
{
  while(CHK_AP_EXIT)
	{	
		///-----------------------------------------------------------------///
		/// Get time
		///-----------------------------------------------------------------///
		UPI_MEMCPY(&UpiGauge.tmPre, &UpiGauge.tmNow, sizeof(UpiGauge.tmNow));
		UPI_LOCALTIME(&UpiGauge.tmNow);
		GLOGD("===========================================\n");	
		GLOGD("%02d:%02d:%02d (%ld ms) : dT= %d ms\n", UpiGauge.tmNow.hour, 
														 						UpiGauge.tmNow.min, 
														 						UpiGauge.tmNow.sec,
																				UpiGauge.tmNow.absTime, 
														 						(UpiGauge.tmNow.absTime - UpiGauge.tmPre.absTime));	
		GLOGD("===========================================\n");	
		
		///-------------------------///
		/// Test#1 : OTP Read
		///-------------------------///
		GTestOTPRead();

		///-------------------------///
		/// Do Delay
		///-------------------------///		
		UpiSleep(1000);
	}
}
#endif ///< for  FEATURE_TEST_MAINLOOP


#ifdef __cplusplus
} 
#endif  ///< for __cplusplus

