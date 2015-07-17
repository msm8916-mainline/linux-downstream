#ifndef __SX9500_H
#define __SX9500_H

/*
 *  I2C Registers
 */
#define SX9500_IRQSTAT_REG    0x00
#define SX9500_TCHCMPSTAT_REG    0x01
#define SX9500_IRQ_ENABLE_REG       0x03
#define SX9500_CPS_CTRL0_REG      0x06
#define SX9500_CPS_CTRL1_REG      0x07
#define SX9500_CPS_CTRL2_REG      0x08
#define SX9500_CPS_CTRL3_REG      0x09
#define SX9500_CPS_CTRL4_REG      0x0A
#define SX9500_CPS_CTRL5_REG      0x0B
#define SX9500_CPS_CTRL6_REG      0x0C
#define SX9500_CPS_CTRL7_REG      0x0D
#define SX9500_CPS_CTRL8_REG      0x0E
#define SX9500_SOFTRESET_REG  0x7F


/*      Sensor Readback */
#define SX9500_CPSRD          0x20

#define SX9500_USEMSB         0x21
#define SX9500_USELSB         0x22

#define SX9500_AVGMSB         0x23
#define SX9500_AVGLSB         0x24

#define SX9500_DIFFMSB        0x25
#define SX9500_DIFFLSB        0x26

/*      IrqStat 0:Inactive 1:Active     */
#define SX9500_IRQSTAT_RESET_FLAG      0x80
#define SX9500_IRQSTAT_TOUCH_FLAG      0x40
#define SX9500_IRQSTAT_RELEASE_FLAG    0x20
#define SX9500_IRQSTAT_COMPDONE_FLAG   0x10
#define SX9500_IRQSTAT_CONV_FLAG       0x08
#define SX9500_IRQSTAT_TXENSTAT_FLAG   0x01


/* CpsStat  */
#define SX9500_TCHCMPSTAT_TCHSTAT3_FLAG   0x80
#define SX9500_TCHCMPSTAT_TCHSTAT2_FLAG   0x40
#define SX9500_TCHCMPSTAT_TCHSTAT1_FLAG   0x20
#define SX9500_TCHCMPSTAT_TCHSTAT0_FLAG   0x10

/*      SoftReset */
#define SX9500_SOFTRESET  0xDE


struct _buttonInfo {
  /*! The Key to send to the input */
  int keycode;
  /*! Mask to look for on Touch Status */
  int mask;
  /*! Current state of button. */
  int state;
};

struct smtc_reg_data {
  unsigned char reg;
  unsigned char val;
};

typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;

struct sx9500_platform_data {
	int i2c_reg_num;
	struct i2c_client *client;
	struct smtc_reg_data *pi2c_reg;
	int (*get_is_nirq_low)(void);
	int     (*init_platform_hw)(struct device *dev);
	void    (*exit_platform_hw)(void);
};

typedef struct sx9500_platform_data sx9500_platform_data_t;
typedef struct sx9500_platform_data *psx9500_platform_data_t;

#if 0
enum PSENSOR_STATUS{
	PSENSOR_STATUS_NEAR=0,
	PSENSOR_STATUS_FAR=1,
};


struct psensor_driver_t{
	char *name;
	int (*init)(void);
};
#endif


/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
  {
    .reg = SX9500_IRQ_ENABLE_REG,
    .val = 0xf0,
  },
  {
    .reg = SX9500_CPS_CTRL1_REG,
    .val = 0x43,
  },
  {
    .reg = SX9500_CPS_CTRL2_REG,
    .val = 0x57,
  },
  {
    .reg = SX9500_CPS_CTRL3_REG,
    .val = 0x01,//0x02
  },
  {
    .reg = SX9500_CPS_CTRL4_REG,
    .val = 0x30,
  },
  {
    .reg = SX9500_CPS_CTRL5_REG,
    .val = 0x0f,
  },
  {
    .reg = SX9500_CPS_CTRL6_REG,
    .val = 0x05,
  },
  {
    .reg = SX9500_CPS_CTRL7_REG,
    .val = 0x00,
  },
  {
    .reg = SX9500_CPS_CTRL8_REG,
    .val = 0x00,
  },
  {
    .reg = SX9500_CPS_CTRL0_REG,
    .val = 0x21,
  },
};
#endif

