/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 Copyright ?2012 Synaptics Incorporated. All rights reserved.

 The information in this file is confidential under the terms
 of a non-disclosure agreement with Synaptics and is provided
 AS IS.

 The information in this file shall remain the exclusive property
 of Synaptics and may be the subject of Synaptics?patents, in
 whole or part. Synaptics?intellectual property rights in the
 information in this file are not expressly or implicitly licensed
 or otherwise transferred to you as a result of such information
 being made available to you.
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* FullRawCapacitance Support 0D button */

#include "RefCode_F54.h"
#include "TestLimits.h"
#define LGTP_MODULE "[TD4191]"

#define TRX_mapping_max 54
#define LOWER_ABS_ADC_RANGE_LIMIT 60
#define UPPER_ABS_ADC_RANGE_LIMIT 190
#define LOWER_ABS_RAW_CAP_LIMIT 1000 /*fF*/
#define UPPER_ABS_RAW_CAP_LIMIT 14000 /*fF*/
#define REPORT_DATA_OFFEST 3
#define VERSION "1.0"


unsigned int count;
const int DefaultTimeout = 10; /* In counts*/

int pageNum;
int scanMaxPageCount = 5;
int input;

bool bHaveF01;
bool bHaveF11;
bool bHaveF1A;
bool bHaveF12;
bool bHaveF34;
bool bHaveF54;
bool bHaveF55;
bool SignalClarityOn;
bool bHaveF54Ctrl07;
bool bHaveF54Ctrl41;
bool bHaveF54Ctrl57;
bool bHavePixelTouchThresholdTuning;
bool bHaveInterferenceMetric;
bool bHaveCtrl11;
bool bHaveRelaxationControl;
bool bHaveSensorAssignment;
bool bHaveSenseFrequencyControl;
bool bHaveFirmwareNoiseMitigation;
bool bHaveIIRFilter;
bool bHaveCmnRemoval;
bool bHaveCmnMaximum;
bool bHaveTouchHysteresis;
bool bHaveEdgeCompensation;
bool bHavePerFrequencyNoiseControl;
bool bHaveSignalClarity;
bool bHaveMultiMetricStateMachine;
bool bHaveVarianceMetric;
bool bHave0DRelaxationControl;
bool bHave0DAcquisitionControl;
bool bHaveSlewMetric;
bool bHaveHBlank;
bool bHaveVBlank;
bool bHaveLongHBlank;
bool bHaveNoiseMitigation2;
bool bHaveSlewOption;
bool bHaveEnhancedStretch;
bool bHaveStartupFastRelaxation;
bool bHaveESDControl;
bool bHaveEnergyRatioRelaxation;
bool bHaveCtrl86;
bool bHaveCtrl87;
bool bHaveCtrl88;
bool bHaveCtrl89;
bool bHaveCtrl90;
bool bHaveCtrl91;
bool bHaveCtrl92;
bool bHaveCtrl93;
bool bHaveCtrl94;
bool bHaveCtrl95;
bool bHaveCtrl96;
bool bHaveCtrl97;
bool bHaveCtrl98;
bool bHaveCtrl99;
bool bHaveCtrl100;
bool bHaveCtrl101;
bool bHaveCtrl102;
bool bHaveF54Query13;
bool bHaveF54Query15;
bool bHaveF54Query16;
bool bHaveF54Query17;
bool bHaveF54Query18;
bool bHaveF54Query19;
bool bHaveF54Query22;
bool bHaveF54Query23;
bool bHaveF54Query25;
bool ButtonShared;

unsigned char F54DataBase;
unsigned char F54QueryBase;
unsigned char F54CommandBase;
unsigned char F54ControlBase;
unsigned char F55QueryBase;
unsigned char F55ControlBase;
unsigned char F01ControlBase;
unsigned char F01CommandBase;
unsigned char RxChannelCount;
unsigned char TxChannelCount;
unsigned char TouchControllerFamily;
unsigned char CurveCompensationMode;
unsigned char NumberOfSensingFrequencies;
unsigned char F54Ctrl07Offset;
unsigned char F54Ctrl41Offset;
unsigned char F54Ctrl57Offset;
unsigned char F54Ctrl88Offset;
unsigned char F54Ctrl89Offset;
unsigned char F54Ctrl98Offset;
unsigned char F1AControlBase;
unsigned char F12ControlBase;
unsigned char F12QueryBase;
unsigned char F12_2DTxCount;
unsigned char F12_2DRxCount;
unsigned char ButtonTx[8];
unsigned char ButtonRx[8];
unsigned char ButtonCount;
unsigned char F12Support;
unsigned char F12ControlRegisterPresence;
unsigned char mask;

/* Assuming Tx = 32 & Rx = 32 to accommodate any configuration*/
short Image1[TRX_max][TRX_max];
int ImagepF[TRX_max][TRX_max];
int AbsSigned32Data[TRX_mapping_max];
unsigned char AbsADCRangeData[TRX_mapping_max];
unsigned char Data[TRX_max * TRX_max * 4];
unsigned char TRxPhysical[TRX_mapping_max];

int MaxArrayLength;

unsigned char TREX_mapped[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3f};
unsigned char TRX_Open[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char TRX_Gnd[7] = {0xff, 0xff, 0xff, 0xff, 0x3, 0xff, 0xfc};
unsigned char TRX_Short[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int HighResistanceLowerLimit[3] = {-1000, -1000, -400};
int HighResistanceUpperLimit[3] = {450, 450, 200};
unsigned int AbsShort[TRX_max*2] = {0};
unsigned int AbsOpen[TRX_max*2] = {0};
int AbsTxShortLimit;
int AbsRxShortLimit;
int AbsTxOpenLimit;
int AbsRxOpenLimit = 1000;
int AbsRawRef[16] = {77, 11919, 14023, 15163, 16192, 18319, 19337, 21491,
	22633, 24692, 26853, 27993, 30147, 32253, 34411, 37605};

short NoiseLimitLow = -16;
short NoiseLimitHigh = 16;
unsigned short RT78Data[TRX_max][TRX_max];

enum {
	STARTTIME,
	ENDTIME,
	TIME_PROFILE_MAX
};

#define get_time_interval(a, b) (a >= b ? a - b : 1000000 + a - b)
struct timeval t_interval[TIME_PROFILE_MAX];
static int outbuf;
static int out_buf;
char f54_wlog_buf[8000] = {0};
char wlog_buf[8000] = {0};



/* Function to switch beteen register pages.*/
bool switchPage(int page)
{
	unsigned char values[1] = {0};
	unsigned char data = 0;

	pageNum = values[0] = page;

	count = 0;
	do {
		Write8BitRegisters(0xFF, values, 1);
		msleep(1);
		Read8BitRegisters(0xFF, &data, 1);
		count++;
	} while ((int)data != page && (count < DefaultTimeout));
	if (count >= DefaultTimeout) {
		TOUCH_LOG("Timeout -- Page switch fail !\n");
		return -EAGAIN;
	}
	return true;
}

void Reset(void)
{
	unsigned char data;

	switchPage(0x00);

	data = 0x01;
	Write8BitRegisters(F01CommandBase, &data, 1);

	msleep(10);
}
int CompareElectodeShortRT78(char *buf)
{
	bool result = true;
	int i, j = 0;

	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%s","      ");
	for (i = 0; i < F12_2DTxCount; i++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d ",i);
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"\n-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");

	for (i = 0; i < (unsigned int)F12_2DRxCount; i++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d: ",i);

		for (j = 0; j < (unsigned int)F12_2DTxCount; j++) {
			if ((RT78Data[i][j] < RT78ElectrodeShortUpper[i][j])
				&&(RT78Data[i][j] > RT78ElectrodeShortLower[i][j])) {
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%4s ", " ,");
			} else {
				TOUCH_LOG("TX[%d]RX[%d] failed value:  %d\n",
						i, j, RT78Data[i][j]);
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%4s ", "X,");
				result = false;
			}
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n");
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");
	if (result) {
		TOUCH_LOG("Electode Short Test Passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Electode Short Test passed.\n\n");
	} else {
		TOUCH_LOG("Electode Short Test Failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Electode Short Test failed.\n\n");
	}
	write_log(NULL, f54_wlog_buf);
	return (result) ? 1 : 0;
}
int ReadElectodeShortRT78(char *buf)
{
	int i,j,k= 0;
	int ret = 0;
	unsigned short temp = 0;

	TOUCH_LOG("@@@ %s : F12_TxCount = %d F12_RxCount = %d @@@",__func__,
				F12_2DTxCount, F12_2DRxCount);
	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount * F12_2DTxCount * 2));

	for (i = 0; i < F12_2DTxCount; i++) {
	}

	for (i = 0; i < F12_2DRxCount; i++) {
		for (j = 0; j < F12_2DTxCount; j++, k += 2) {
			temp = Data[k] | (Data[k+1] << 8);
			RT78Data[i][j] = temp;
		}
	}
	ret = CompareElectodeShortRT78(buf);

	return ret;
}

/* Compare Report type #23 data against test limits*/
int CompareRT78(char *buf)
{
	bool result = true;
	int i, j = 0;

	//us10_porting
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%s","      ");
	for (i = 0; i < F12_2DTxCount; i++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d ",i);
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"\n-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");

	for (i = 0; i < (unsigned int)F12_2DRxCount; i++) {
		//us10_porting
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d: ",i);

		for (j = 0; j < (unsigned int)F12_2DTxCount; j++) {
			if ((RT78Data[i][j] < RT78FullRawCapUpperLimit[i][j])
				&&(RT78Data[i][j] > RT78FullRawCapLowerLimit[i][j])) {
				//TOUCH_INFO_MSG("TX[%d]RX[%d] passed value:  %d\n",
				//		i, j, RT78Data[i][j]);
				//result = true;
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%4s ", " ,");//us10_porting
			} else {
				TOUCH_LOG("TX[%d]RX[%d] failed value:  %d\n",
						i, j, RT78Data[i][j]);
				outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%4s ", "X,");//us10_porting
				result = false;
			}
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n"); //us10_porting
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");
	if (result) {
		TOUCH_LOG("RT78 Test Passed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"RT78 Test passed.\n\n");
	} else {
		TOUCH_LOG("RT78 Test Failed.\n");
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"RT78 Test failed.\n\n");
	}
	write_log(NULL, f54_wlog_buf); //us10_porting
	//outbuf += snprintf(buf+outbuf,PAGE_SIZE-outbuf ,f54_wlog_buf);
	return (result) ? 1 : 0;
}

int ReadRT78(char *buf)
{
	int i,j,k= 0;
	int ret = 0;
	unsigned short temp = 0;

	TOUCH_LOG("@@@ %s : F12_TxCount = %d F12_RxCount = %d @@@",__func__,
				F12_2DTxCount, F12_2DRxCount);
	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),
			&Data[0], (F12_2DRxCount * F12_2DTxCount * 2));


	//us10_porting
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%s","      ");
	for (i = 0; i < F12_2DTxCount; i++) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d ",i);
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"\n-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");

	for (i = 0; i < F12_2DRxCount; i++) {
		//us10_porting
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,"%4d: ",i);
		for (j = 0; j < F12_2DTxCount; j++, k += 2) {
			temp = Data[k] | (Data[k+1] << 8);
			RT78Data[i][j] = temp;
			outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "%4d ", RT78Data[i][j]);//us10_porting
		}
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf, "\n"); //us10_porting
	}
	outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
						"-----------------------------------------"
						"-------------------------------------------------"
						"-------------------------------------------------"
						"--------------------------\n");
	TOUCH_LOG("\n");

	ret = CompareRT78(buf);

	return ret;
}

int pow_func(int x, int y)
{
	int result = 1;
	int i = 0;
	for (i = 0; i < y; i++)
		result *= x;
	return result;
}
int GetImageRT78(char *buf)
{
	int i, j, k= 0;
	int ret = 0;
	unsigned short temp = 0;

	Read8BitRegisters((F54DataBase + REPORT_DATA_OFFEST),

			&Data[0], (F12_2DRxCount * F12_2DTxCount * 2));
	*buf = 0;
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n\nInfo: Tx = %d Rx = %d\n\n",
			(int)F12_2DTxCount, (int)F12_2DRxCount);
	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"========================================="
			"================================================="
			"================================================="
			"============================\n      :");
	//us10_porting
	for (i = 0; i < (int)F12_2DTxCount; i++)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%4d ", i);

	ret += snprintf(buf+ret, PAGE_SIZE-ret,
			"\n---------------------------------------"
			"-------------------------------------------------"
			"-------------------------------------------------"
			"------------------------------\n");

	for (i = 0; i < F12_2DRxCount; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "   %2d : ", i);
		for (j = 0; j < F12_2DTxCount; j++, k += 2) {
			temp = Data[k] | (Data[k+1] << 8);
			RT78Data[i][j] = temp;
			ret += snprintf(buf + ret, PAGE_SIZE-ret, "%4d ", RT78Data[i][j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE-ret, "\n");
	}
	ret += snprintf(buf + ret, PAGE_SIZE-ret,
			"-----------------------------------------"
			"-------------------------------------------------"
			"-------------------------------------------------"
			"----------------------------\n");

	//ret = CompareRT78(buf);

	return ret;

}

/* Function to handle report reads based on user input*/
int ReadReport(unsigned char input, char *buf)
{
	int ret = 0;
	unsigned char data;

	/*Set the GetReport bit to run the AutoScan*/
	data = 0x01;
	DO_SAFE(Write8BitRegisters(F54CommandBase, &data, 1), error);
	
	count = 0;
	do {
		DO_SAFE(Read8BitRegisters(F54CommandBase, &data, 1), error);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));
	if (count >= DefaultTimeout) {
		TOUCH_LOG("Timeout - Not supported Report Type in FW\n");
		Reset();
		return -EAGAIN;
	}

	do_gettimeofday(&t_interval[ENDTIME]);

	TOUCH_LOG("Takes %lu ticks\n",
			get_time_interval(t_interval[ENDTIME].tv_sec,
				t_interval[STARTTIME].tv_sec));

	switch (input) {
	case 'p':
		ret = ReadRT78(buf);
		break;
    case 'q':
		ret = GetImageRT78(buf);
		break;
    case 'r':
		ret = ReadElectodeShortRT78(buf);
		break;
	default:
		break;
	}

	return ret;

error:
	TOUCH_ERR("[%s] ReadReport fail\n", __func__);
	return -EAGAIN;
}


/* Examples of reading query registers.
 Real applications often do not need to read query registers at all.
 */
void RunQueries(void)
{
	unsigned short cAddr = 0xEE;
	unsigned char cFunc = 0;
	int rxCount = 0;
	int txCount = 0;
	int offset = 0;
	int query_offset = 0;
	int i, j = 0;

	/* Scan Page Description Table (PDT)
	   to find all RMI functions presented by this device.
	   The Table starts at $00EE. This and every sixth register
	   (decrementing) is a function number
	   except when this "function number" is $00, meaning end of PDT.
	   In an actual use case this scan might be done only once
	   on first run or before compile.
	   */
	do {
		Read8BitRegisters(cAddr, &cFunc, 1);
		if (cFunc == 0)
			break;

		switch (cFunc) {
		case 0x01:
			if (!bHaveF01) {
				Read8BitRegisters((cAddr - 3),
						&F01ControlBase, 1);
				Read8BitRegisters((cAddr - 4),
						&F01CommandBase, 1);
			}
			break;

		case 0x12:
			if (!bHaveF12) {
				Read8BitRegisters((cAddr - 3),
						&F12ControlBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F12QueryBase, 1);
				Read8BitRegisters((F12QueryBase),
						&F12Support, 1);

				if ((F12Support | 0x00) == 0) {
					TOUCH_LOG(
						"Device not support F12.\n");
					break;
				}
				Read8BitRegisters((F12QueryBase + 5),
						Data, 2);
				mask = 0x01;
				for (j = 0; j < 8; j++) {
					if ((Data[1] & mask) == 1)
						offset++;
					Data[1] >>= 1;
				}
				Read8BitRegisters((F12ControlBase + offset),
						Data, 14);

				F12_2DRxCount = Data[12];
				F12_2DTxCount = Data[13];

				if (TRX_max <= F12_2DRxCount)
					F12_2DRxCount = TRX_max;
				if (TRX_max <= F12_2DTxCount)
					F12_2DTxCount = TRX_max;

				offset = 0;
			}
			break;
		case 0x54:
			if (!bHaveF54) {
				Read8BitRegisters((cAddr - 2),
						&F54DataBase, 1);
				Read8BitRegisters((cAddr - 3),
						&F54ControlBase, 1);
				Read8BitRegisters((cAddr - 4),
						&F54CommandBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F54QueryBase, 1);
				Read8BitRegisters(F54QueryBase,
						&RxChannelCount, 1);
				Read8BitRegisters((F54QueryBase + 1),
						&TxChannelCount, 1);

				if (TRX_max <= RxChannelCount)
					RxChannelCount = TRX_max;
				if (TRX_max <= TxChannelCount)
					TxChannelCount = 16;

				MaxArrayLength = (int)RxChannelCount
					* (int)TxChannelCount * 2;

				Read8BitRegisters(F54QueryBase,
						Data, 24);
				TouchControllerFamily = Data[5];
				offset++; /*Ctrl 00*/

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset++; /*Ctrl 01*/
				offset += 2; /*Ctrl 02*/
				bHavePixelTouchThresholdTuning =
					((Data[6] & 0x01) == 0x01);

				if (bHavePixelTouchThresholdTuning)
					offset++; /*Ctrl 03;*/

				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 3; /*Ctrl 04/05/06*/

				if (TouchControllerFamily == 0x01) {
					F54Ctrl07Offset = offset;
					offset++; /*Ctrl 07;*/
					bHaveF54Ctrl07 = true;
				}

				/*Ctrl 08*/
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily == 0x01)
					offset += 2;
				/*Ctrl 09*/
				if (TouchControllerFamily == 0x0 ||
						TouchControllerFamily
						== 0x01)
					offset++;
				bHaveInterferenceMetric =
					((Data[7] & 0x02) == 0x02);
				/* Ctrl 10*/
				if (bHaveInterferenceMetric)
					offset++;
				bHaveCtrl11 =
					((Data[7] & 0x10) == 0x10);
				/*Ctrl 11*/
				if (bHaveCtrl11)
					offset += 2;
				bHaveRelaxationControl =
					((Data[7] & 0x80) == 0x80);
				/*Ctrl 12/13*/
				if (bHaveRelaxationControl)
					offset += 2;
				bHaveSensorAssignment =
					((Data[7] & 0x01) == 0x01);
				/*Ctrl 14*/
				if (bHaveSensorAssignment)
					offset++;
				/*Ctrl 15*/
				if (bHaveSensorAssignment)
					offset += RxChannelCount;
				/*Ctrl 16*/
				if (bHaveSensorAssignment)
					offset += TxChannelCount;
				bHaveSenseFrequencyControl =
					((Data[7] & 0x04) == 0x04);
				NumberOfSensingFrequencies =
					(Data[13] & 0x0F);
				/*Ctrl 17/18/19*/
				if (bHaveSenseFrequencyControl)
					offset +=
					(3 * (int)NumberOfSensingFrequencies);
				offset++; /*Ctrl 20*/
				if (bHaveSenseFrequencyControl)
					offset += 2; /*Ctrl 21*/
				bHaveFirmwareNoiseMitigation =
					((Data[7] & 0x08) == 0x08);
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 22*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 23*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 24*/
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 25*/
				if (bHaveFirmwareNoiseMitigation)
					offset++; /*Ctrl 26*/
				bHaveIIRFilter = ((Data[9] & 0x02)
						== 0x02);
				if (bHaveIIRFilter)
					offset++; /*Ctrl 27*/
				if (bHaveFirmwareNoiseMitigation)
					offset += 2; /*Ctrl 28*/
				bHaveCmnRemoval = ((Data[9] & 0x04)
						== 0x04);
				bHaveCmnMaximum = ((Data[9] & 0x08)
						== 0x08);
				if (bHaveCmnRemoval)
					offset++; /*Ctrl 29*/
				if (bHaveCmnMaximum)
					offset++; /*Ctrl 30*/
				bHaveTouchHysteresis =
					((Data[9] & 0x10) == 0x10);
				if (bHaveTouchHysteresis)
					offset++; /*Ctrl 31*/
				bHaveEdgeCompensation =
					((Data[9] & 0x20) == 0x20);
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 32*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 33*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 34*/
				if (bHaveEdgeCompensation)
					offset += 2; /*Ctrl 35*/
				CurveCompensationMode =
					(Data[8] & 0x03);
				if (CurveCompensationMode == 0x02) {
					offset += (int)RxChannelCount;
				} else if (CurveCompensationMode == 0x01) {
					offset +=
					((int)RxChannelCount
					 > (int)TxChannelCount) ?
					(int)RxChannelCount
					: (int)TxChannelCount;
				} /*Ctrl 36*/

				if (CurveCompensationMode == 0x02) {
					/*Ctrl 37*/
					offset += (int)TxChannelCount;
				}

				bHavePerFrequencyNoiseControl =
					((Data[9] & 0x40) == 0x40);

				/*Ctrl 38/39/40*/
				if (bHavePerFrequencyNoiseControl)
					offset +=
					(3 * (int)NumberOfSensingFrequencies);

				bHaveSignalClarity =
					((Data[10] & 0x04) == 0x04);

				if (bHaveSignalClarity) {
					F54Ctrl41Offset = offset;
					offset++; /*Ctrl 41*/
					bHaveF54Ctrl41 = true;
				}

				bHaveMultiMetricStateMachine =
					((Data[10] & 0x02) == 0x02);
				bHaveVarianceMetric =
					((Data[10] & 0x08) == 0x08);
				if (bHaveVarianceMetric)
					offset += 2; /*Ctr 42*/
				if (bHaveMultiMetricStateMachine)
					offset += 2; /*Ctrl 43*/
				/*Ctrl 44/45/46/47/48/49/50/51/52/53/54*/
				if (bHaveMultiMetricStateMachine)
					offset += 11;

				bHave0DRelaxationControl =
					((Data[10] & 0x10) == 0x10);
				bHave0DAcquisitionControl =
					((Data[10] & 0x20) == 0x20);
				if (bHave0DRelaxationControl)
					offset += 2; /*Ctrl 55/56*/
				if (bHave0DAcquisitionControl) {
					F54Ctrl57Offset = offset;
					offset++; /*Ctrl 57;*/
					bHaveF54Ctrl57 = true;
				}
				if (bHave0DAcquisitionControl)
					offset += 1; /*Ctrl 58*/

				bHaveSlewMetric =
					((Data[10] & 0x80) == 0x80);
				bHaveHBlank = ((Data[11] & 0x01) == 0x01);
				bHaveVBlank = ((Data[11] & 0x02) == 0x02);
				bHaveLongHBlank = ((Data[11] & 0x04) == 0x04);
				bHaveNoiseMitigation2 =
					((Data[11] & 0x20) == 0x20);
				bHaveSlewOption = ((Data[12] & 0x02) == 0x02);

				if (bHaveHBlank)
					offset += 1; /*Ctrl 59*/

				if (bHaveHBlank || bHaveVBlank
						|| bHaveLongHBlank)
					offset += 3; /*Ctrl 60/61/62*/

				if (bHaveSlewMetric || bHaveHBlank
						|| bHaveVBlank
						|| bHaveLongHBlank
						|| bHaveNoiseMitigation2
						|| bHaveSlewOption)
					offset += 1; /*Ctrl 63*/

				if (bHaveHBlank)
					offset += 28; /*Ctrl 64/65/66/67*/
				else if (bHaveVBlank || bHaveLongHBlank)
					offset += 4; /*Ctrl 64/65/66/67*/

				if (bHaveHBlank || bHaveVBlank
						|| bHaveLongHBlank)
					offset += 8; /*Ctrl 68/69/70/71/72/73*/

				if (bHaveSlewMetric)
					offset += 2; /*Ctrl 74*/

				bHaveEnhancedStretch =
					((Data[9] & 0x80) == 0x80);
				/*Ctrl 75*/
				if (bHaveEnhancedStretch)
					offset +=
					(int)NumberOfSensingFrequencies;

				bHaveStartupFastRelaxation =
					((Data[11] & 0x08) == 0x08);
				if (bHaveStartupFastRelaxation)
					offset += 1; /*Ctrl 76*/

				bHaveESDControl =
					((Data[11] & 0x10) == 0x10);
				if (bHaveESDControl)
					offset += 2; /*Ctrl 77/78*/

				if (bHaveNoiseMitigation2)
					offset += 5; /*Ctrl 79/80/81/82/83*/

				bHaveEnergyRatioRelaxation =
					((Data[11] & 0x80) == 0x80);
				if (bHaveEnergyRatioRelaxation)
					offset += 2; /*Ctrl 84/85*/

				bHaveF54Query13 = ((Data[12] & 0x08) == 0x08);
				if (bHaveSenseFrequencyControl) {
					query_offset = 13;
					NumberOfSensingFrequencies =
						(Data[13] & 0x0F);
				} else
					query_offset = 12;
				if (bHaveF54Query13)
					query_offset++;
				bHaveCtrl86 = (bHaveF54Query13 &&
						((Data[13] & 0x01) == 0x01));
				bHaveCtrl87 = (bHaveF54Query13 &&
						((Data[13] & 0x02) == 0x02));
				bHaveCtrl88 = ((Data[12] & 0x40) == 0x40);

				if (bHaveCtrl86)
					offset += 1; /*Ctrl 86*/
				if (bHaveCtrl87)
					offset += 1; /*Ctrl 87*/
				if (bHaveCtrl88) {
					F54Ctrl88Offset = offset;
					offset++; /*Ctrl 88;*/
				}
				bHaveCtrl89 = ((Data[query_offset]
							& 0x20) == 0x20);
				if (bHaveCtrl89)
					offset++;
				bHaveF54Query15 = ((Data[12] & 0x80)
						== 0x80);
				if (bHaveF54Query15)
					query_offset++;  /*query_offset = 14*/
				bHaveCtrl90 = (bHaveF54Query15 &&
						((Data[query_offset]
						  & 0x01) == 0x01));
				if (bHaveCtrl90)
					offset++;
				bHaveF54Query16 = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveF54Query22 = ((Data[query_offset]
							& 0x40) == 0x40);
				bHaveF54Query25 = ((Data[query_offset]
							& 0x80) == 0x80);
				if (bHaveF54Query16)
					query_offset++; /*query_offset = 15*/
				bHaveF54Query17 = ((Data[query_offset]
							& 0x1) == 0x1);
				bHaveCtrl92 = ((Data[query_offset]
							& 0x4) == 0x4);
				bHaveCtrl93 = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveCtrl94 = ((Data[query_offset]
							& 0x10) == 0x10);
				bHaveF54Query18 = bHaveCtrl94;
				bHaveCtrl95 = ((Data[query_offset]
							& 0x20) == 0x20);
				bHaveF54Query19 = bHaveCtrl95;
				bHaveCtrl99 = ((Data[query_offset]
							& 0x40) == 0x40);
				bHaveCtrl100 = ((Data[query_offset]
							& 0x80) == 0x80);
				if (bHaveF54Query17)
					query_offset++; /*query_offset = 16*/
				if (bHaveF54Query18)
					query_offset++; /*query_offset = 17*/
				if (bHaveF54Query19)
					query_offset++; /*query_offset = 18*/

				/*query 20, 21 query_offset = 20*/
				query_offset = query_offset + 2;
				bHaveCtrl91 = ((Data[query_offset]
							& 0x4) == 0x4);
				bHaveCtrl96  = ((Data[query_offset]
							& 0x8) == 0x8);
				bHaveCtrl97  = ((Data[query_offset]
							& 0x10) == 0x10);
				bHaveCtrl98  = ((Data[query_offset]
							& 0x20) == 0x20);
				if (bHaveF54Query22)
					query_offset++; /*query_offset = 21*/
				bHaveCtrl101 = ((Data[query_offset]
							& 0x2) == 0x2);
				bHaveF54Query23 = ((Data[query_offset]
							& 0x8) == 0x8);
				if (bHaveF54Query23) {
					query_offset++; /*query_offset = 22*/
					bHaveCtrl102 = ((Data[query_offset]
							& 0x01) == 0x01);
				} else
					bHaveCtrl102 = false;
				if (bHaveCtrl91)
					offset++;
				if (bHaveCtrl92)
					offset++;
				if (bHaveCtrl93)
					offset++;
				if (bHaveCtrl94)
					offset++;
				if (bHaveCtrl95)
					offset++;
				if (bHaveCtrl96)
					offset++;
				if (bHaveCtrl97)
					offset++;
				if (bHaveCtrl98) {
					F54Ctrl98Offset = offset;
					offset++;
				}
				if (bHaveCtrl99)
					offset++;
				if (bHaveCtrl100)
					offset++;
				if (bHaveCtrl101)
					offset++;
			}
			break;
		case 0x55:
			if (!bHaveF55) {
				Read8BitRegisters((cAddr - 3),
						&F55ControlBase, 1);
				Read8BitRegisters((cAddr - 5),
						&F55QueryBase, 1);

				Read8BitRegisters(F55QueryBase,
						&RxChannelCount, 1);
				Read8BitRegisters((F55QueryBase+1),
						&TxChannelCount, 1);

				rxCount = 0;
				txCount = 0;
				/*Read Sensor Mapping*/
				Read8BitRegisters((F55ControlBase + 1), Data,
						(int)RxChannelCount);
				for (i = 0; i < (int)RxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						rxCount++;
						TRxPhysical[i] = Data[i];
					} else
						break;
				}
				Read8BitRegisters((F55ControlBase + 2), Data,
						(int)TxChannelCount);
				for (i = 0; i < (int)TxChannelCount; i++) {
					if (Data[i] != 0xFF) {
						TRxPhysical[rxCount + i]
							= Data[i];
						txCount++;
					} else
						break;
				}
				for (i = (rxCount + txCount);
						i < (TRX_mapping_max); i++) {
					TRxPhysical[i] = 0xFF;
				}

				RxChannelCount = rxCount;
				TxChannelCount = txCount;
				if (TRX_max <= RxChannelCount)
					RxChannelCount = TRX_max;
				if (TRX_max <= TxChannelCount)
					TxChannelCount = 16;

				MaxArrayLength = (int)RxChannelCount
					* (int)TxChannelCount * 2;
				if (((int)TxChannelCount - F12_2DTxCount == 0)
						&& ButtonCount > 0) {
					ButtonShared = true;
				}

			}
			break;
		default: /* Any other function*/
			break;
		}
		cAddr -= 6;
	} while (true);
}

/*
The following function is necessary to setup the Function $54 tests.i
The setup needs to be done once
after entering into page 0x01. As long as the touch controller stays in page1
the setup does not
need to be repeated.
*/
bool TestPreparation(void)
{
	unsigned char data = 0;
	unsigned char addr = 0;

	/* Turn off CBC.*/
	if (bHaveF54Ctrl07) {
		addr = F54ControlBase + F54Ctrl07Offset;
		Read8BitRegisters(addr, &data, 1);
		/* data = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	} else if (bHaveCtrl88) {
		addr = F54ControlBase + F54Ctrl88Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data & 0xDF;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Turn off 0D CBC.*/
	if (bHaveF54Ctrl57) {
		addr = F54ControlBase + F54Ctrl57Offset;
		Read8BitRegisters(addr, &data, 1);
		/*ata = data & 0xEF;*/
		data = 0;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Turn off SignalClarity.
	   ForceUpdate is required for the change to be effective
	   */
	if (bHaveF54Ctrl41) {
		addr = F54ControlBase + F54Ctrl41Offset;
		Read8BitRegisters(addr, &data, 1);
		data = data | 0x01;
		Write8BitRegisters(addr, &data, 1);
	}

	/* Apply ForceUpdate.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x04;
	Write8BitRegisters(F54CommandBase, &data, 1);
	/* Wait complete*/
	count = 0;
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceUpdate can not complete\n");
		TOUCH_LOG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	/* Apply ForceCal.*/
	Read8BitRegisters(F54CommandBase, &data, 1);
	data = data | 0x02;
	Write8BitRegisters(F54CommandBase, &data, 1);

	/* Wait complete*/
	count = 0;
	do {
		Read8BitRegisters(F54CommandBase, &data, 1);
		msleep(1);
		count++;
	} while (data != 0x00 && (count < DefaultTimeout));

	if (count >= DefaultTimeout) {
		outbuf += snprintf(f54_wlog_buf+outbuf, sizeof(f54_wlog_buf)-outbuf,
				"Timeout -- ForceCal can not complete\n");
		TOUCH_LOG("Timeout -- ForceUpdate can not complete\n");
		Reset();
		return -EAGAIN;
	}

	return true;
}

int diffnode(unsigned short *ImagepTest)
{

	int i = 0;
	int k = 0;
	unsigned char data;

	if (!bHaveF54) {
		TOUCH_LOG("not bHaveF54\n");
		return -EINVAL;
	}
	if (!switchPage(0x01)) {
		TOUCH_LOG("not switchPage(0x01)\n");
		return -EINVAL;
	}

	if (TestPreparation()) {

		/*memcpy(LowerImageLimit, LowerImage, sizeof(LowerImageLimit));
		  memcpy(UpperImageLimit, UpperImage, sizeof(UpperImageLimit));
		  */
		data = 20;/*rawdata mode*/
		Write8BitRegisters(F54DataBase, &data, 1);
		data = 0x01;
		Write8BitRegisters(F54CommandBase, &data, 1);
		count = 0;
		do {
			Read8BitRegisters(F54CommandBase, &data, 1);
			msleep(20);
			count++;
		} while (data != 0x00 && (count < DefaultTimeout));
		if (count >= DefaultTimeout) {
			TOUCH_LOG("Timeout -- Not supported Report Type in FW\n");
			Reset();
			return -EAGAIN;
		}

		Read8BitRegisters((F54DataBase+REPORT_DATA_OFFEST), &Data[0],
				MaxArrayLength);

		for (i = 0; i < (int)TxChannelCount * (int)RxChannelCount;
				i++) {
			ImagepTest[i] = ((short)Data[k]
					| (short)Data[k + 1] << 8);
			k = k + 2;
		}
		/*Reset Device*/
		Reset();
		TOUCH_LOG("diff_node success\n");
		return 0;

	} else {
		return -ECANCELED;
	}

}

/* Report type 78 */
int RT78_test(char *buf)
{
	unsigned char data;
	//unsigned char temp;
    
	memcpy(RT78FullRawCapUpperLimit, RT78FullRawCapUpper, sizeof(RT78FullRawCapUpperLimit));
	memcpy(RT78FullRawCapLowerLimit, RT78FullRawCapLower, sizeof(RT78FullRawCapLowerLimit));
	/* set report type*/
	data = 78;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);
	/*startTime = GetTickCount();
	 */

	data = 'p';
	return ReadReport(data, buf);
}

void SCAN_PDT(void)
{
	int i;

	for (i = 0; i < scanMaxPageCount; i++) {
		if (switchPage(i))
			RunQueries();
	}
}

int RT78_ElectodeShort_test(char *buf)
{
	unsigned char data;
	//unsigned char temp;
	memcpy(RT78ElectrodeShortUpperLimit, RT78ElectrodeShortUpper, sizeof(RT78ElectrodeShortUpperLimit));
	memcpy(RT78ElectrodeShortLowerLimit, RT78ElectrodeShortLower, sizeof(RT78ElectrodeShortLowerLimit));

	/* set report type*/
	data = 78;
	Write8BitRegisters(F54DataBase, &data, 1);

	do_gettimeofday(&t_interval[STARTTIME]);
	/*startTime = GetTickCount();
	 */

		data = 'r';

	return ReadReport(data, buf);
}

/* Main entry point for the application
 */
int F54Test(int input, int mode, char *buf)
{

	int ret = 0;
	unsigned char data;
	int retry_cnt1 = 0;
	int retry_cnt2 = 0;
	unsigned char temp;

	/*
	if (PowerOnSensor())
	{
	  fatal("Error powering on sensor.\n");
	}

	These 4 function calls are to scan the Page Description Table (PDT)
	Function $01, $11 and $34 are on page 0
	Function $54 is on page 1
	Function $55 is on Page ?
	Scan up to Page 4
	for (int i = 0; i < scanMaxPageCount ;i++)
	{
	  if (switchPage(i))
	   RunQueries();
	}
	*/

	/* Application should exit with the absence of Function $54
	 */
	/*if (!bHaveF54)
		return -ECANCELED;*/
	/*
	   while (1) {
	   printf("\nPress these keys for tests:\n");
	   printf(" a ) - Full Raw Capacitance Test\n");
	   printf(" b ) - ADC Range Test\n");
	   printf(" c ) - Sensor Speed Test\n");
	   printf(" d ) - TRex Open Test\n");
	   printf(" e ) - TRex Gnd Test\n");
	   printf(" f ) - TRx-to-TRx and TRx-to-Vdd shorts\n");
	   printf(" g ) - High Resistance Test\n");
	   printf(" h ) - Full Raw Capacitance Max/Min Test\n");
	   printf(" i ) - Abs Sensing ADC Range Test\n");
	   printf(" j ) - Abs Sensing Delta Capacitance\n");
	   printf(" k ) - Abs Sensing Raw Capcitance Test\n");
	   printk(" p ) - TD4191 Report 78 Test\n");
	   printf("---------------------------------------------------------");
	   printf("\n z ) - Version\n");
	   printf("\nPress any other key to exit.\n");
	   input = _getch();
	 */
retry:
	ret = switchPage(0x01);
	if (ret == -EAGAIN && ++retry_cnt1 <= 3) {
		TOUCH_LOG("retry switchPage, count=%d\n", retry_cnt1);
		goto retry;
	} else if (ret == 0) {
		return ret;
	}

	data = 0x00;
	Write8BitRegisters(F54DataBase+1, &data, 1);
	Write8BitRegisters(F54DataBase+2, &data, 1);

	outbuf = 0;
	out_buf = 0;
	memset(f54_wlog_buf, 0, sizeof(f54_wlog_buf));
	memset(wlog_buf, 0, sizeof(wlog_buf));

	switch (input) {
	case 'p':
		strlcpy(f54_wlog_buf,
				"p - TD4191 Report Type 78 Test\n",
				sizeof(f54_wlog_buf));

		/* Set dispaly freq to 60Hz */
		Read8BitRegisters(F54ControlBase + 0x20, &temp, 1);
		msleep(80);

		ret = RT78_test(buf);

		Write8BitRegisters(F54ControlBase + 0x20, &temp, 1);
		break;
    case 'q':
		strlcpy(f54_wlog_buf,
				"q - TD4191 Electode Short Test\n",
				sizeof(f54_wlog_buf));

		/* Set dispaly freq to 60Hz */
		Read8BitRegisters(F54ControlBase + 0x20, &temp, 1);
		msleep(80);

		ret = RT78_ElectodeShort_test(buf);

		Write8BitRegisters(F54ControlBase + 0x20, &temp, 1);
		break;
	default:
		return -EINVAL;
	}

	if (switchPage(0x00) != true) {
		TOUCH_LOG("switchPage failed\n");
		/*Reset Device*/
		Reset();
	}

	if (ret == -EAGAIN && ++retry_cnt2 <= 3) {
		TOUCH_LOG("retry Test, count=%d\n", retry_cnt2);
		goto retry;
	} else
		return ret;
		
}
