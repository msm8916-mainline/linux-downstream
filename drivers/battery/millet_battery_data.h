#define CAPACITY_MAX      1000
#define CAPACITY_MAX_MARGIN 50
#define CAPACITY_MIN      0

int Temperature_fn(void)
{
	return 0;
}

static struct battery_data_t stc3117_battery_data[] = {
	{
		.Vmode= 0,		 /*REG_MODE, BIT_VMODE 1=Voltage mode, 0=mixed mode */
		.Alm_SOC = 1,		/* SOC alm level %*/
		.Alm_Vbat = 3400,	/* Vbat alm level mV*/
		.CC_cnf = 823,		/* nominal CC_cnf, coming from battery characterisation*/
		.VM_cnf = 632,		/* nominal VM cnf , coming from battery characterisation*/
		.Rint = 155,		/* nominal internal impedance*/
		.Cnom = 4000,		/* nominal capacity in mAh, coming from battery characterisation*/
		.Rsense = 10,		/* sense resistor mOhms*/
		.RelaxCurrent = 150, /* current for relaxation in mA (< C/20) */
		.Adaptive = 1,	   /* 1=Adaptive mode enabled, 0=Adaptive mode disabled */

		.CapDerating[6] = 600,				/* capacity derating in 0.1%, for temp = -20C */
		.CapDerating[5] = 300,				/* capacity derating in 0.1%, for temp = -10C */
		.CapDerating[4] = 80,			   /* capacity derating in 0.1%, for temp = 0C */
		.CapDerating[3] = 20,			   /* capacity derating in 0.1%, for temp = 10C */
		.CapDerating[2] = 0,			  /* capacity derating in 0.1%, for temp = 25C */
		.CapDerating[1] = 0,			  /* capacity derating in 0.1%, for temp = 40C */
		.CapDerating[0] = 0,			  /* capacity derating in 0.1%, for temp = 60C */

		.OCVValue[15] = 4312,			 /* OCV curve adjustment */
		.OCVValue[14] = 4187,			 /* OCV curve adjustment */
		.OCVValue[13] = 4089,			 /* OCV curve adjustment */
		.OCVValue[12] = 3990,			 /* OCV curve adjustment */
		.OCVValue[11] = 3959,			 /* OCV curve adjustment */
		.OCVValue[10] = 3906,			 /* OCV curve adjustment */
		.OCVValue[9] = 3839,			 /* OCV curve adjustment */
		.OCVValue[8] = 3801,			 /* OCV curve adjustment */
		.OCVValue[7] = 3772,			 /* OCV curve adjustment */
		.OCVValue[6] = 3756,			 /* OCV curve adjustment */
		.OCVValue[5] = 3738,			 /* OCV curve adjustment */
		.OCVValue[4] = 3707,			 /* OCV curve adjustment */
		.OCVValue[3] = 3685,			 /* OCV curve adjustment */
		.OCVValue[2] = 3681,			 /* OCV curve adjustment */
		.OCVValue[1] = 3629,			 /* OCV curve adjustment */
		.OCVValue[0] = 3400,			 /* OCV curve adjustment */

		/*if the application temperature data is preferred than the STC3117 temperature*/
		.ExternalTemperature = Temperature_fn, /*External temperature fonction, return C*/
		.ForceExternalTemperature = 0, /* 1=External temperature, 0=STC3117 temperature */
	}
};

static sec_bat_adc_table_data_t temp_table[] = {
  {26437, 900},
  {27225, 800},
  {28019, 700},
  {28378, 650},
  {28738, 620},
  {28917, 600},
  {29126, 580},
  {29576, 550},
  {30027, 500},
  {30420, 470},
  {30675, 450},
  {30953, 430},
  {31375, 400},
  {32149, 350},
  {32924, 300},
  {33849, 250},
  {34775, 200},
  {35694, 150},
  {36613, 100},
  {37216, 50},
  {37820, 20},
  {38170, 0},
  {38727, -30},
  {38878, -50},
  {39589, -100},
  {40303, -150},
  {40584, -200},
};

#define TEMP_HIGH_THRESHOLD_EVENT  520
#define TEMP_HIGH_RECOVERY_EVENT   460
#define TEMP_LOW_THRESHOLD_EVENT   (-50)
#define TEMP_LOW_RECOVERY_EVENT    0
#define TEMP_HIGH_THRESHOLD_NORMAL 520
#define TEMP_HIGH_RECOVERY_NORMAL  460
#define TEMP_LOW_THRESHOLD_NORMAL  (-50)
#define TEMP_LOW_RECOVERY_NORMAL   0
#define TEMP_HIGH_THRESHOLD_LPM    520
#define TEMP_HIGH_RECOVERY_LPM     460
#define TEMP_LOW_THRESHOLD_LPM     (-50)
#define TEMP_LOW_RECOVERY_LPM      0

#if defined(CONFIG_BATTERY_SWELLING)
#define BATT_SWELLING_HIGH_TEMP_BLOCK		550
#define BATT_SWELLING_HIGH_TEMP_RECOV		450
#define BATT_SWELLING_LOW_TEMP_BLOCK		100
#define BATT_SWELLING_LOW_TEMP_RECOV		150
#define BATT_SWELLING_RECHG_VOLTAGE		4150
#define BATT_SWELLING_BLOCK_TIME	10 * 60 /* 10 min */
#endif
