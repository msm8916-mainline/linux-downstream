/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c_if.h
 * FUNCTION     : Interface header for RPR0521, Ambient Light Sensor(ALS) IC
 * PROGRAMMED   : Sensor application development group
 * AUTHOR       : Masafumi Seike
 * REMARKS    :
 * COPYRIGHT    : Copyright (C) 2014 - ROHM CO.,LTD.

 *****************************************************************************/
#ifndef _ROHM_RPR0521_I2C_IF_H_
#define _ROHM_RPR0521_I2C_IF_H_

/* ssoon.lee@lge.com */
#define LGE_PROXIMITY_NAME "lge_proximity"
#define RPR0521_PROX_DRV_NAME "rpr0521_proximity_drv"
#define SENSOR_TYPE_PROXIMITY "proximity"

#define LGE_LIGHT_NAME "lge_light"
#define RPR0521_LIGHT_DRV_NAME "rpr0521_als_drv"
#define SENSOR_TYPE_LIGHT "light"

/************ definition to dependent on sensor IC ************/
#define RPR0521_I2C_NAME          "rpr0521"
#define RPR0521_I2C_ADDRESS       (0x38)  //7 bits slave address 011 1000

/************ command definition of ioctl ************/
#define IOCTL_APP_SET_TIMER       (10)
#define IOCTL_APP_SET_PWRSET_ALS  (11)
#define IOCTL_APP_SET_PWRSET_PS   (12)
#define IOCTL_APP_SET_INTR_MODE   (13)
#define IOCTL_APP_SET_PS_TH_HIGH  (14)
#define IOCTL_APP_SET_ALS_TH_UP   (15)
#define IOCTL_APP_SET_ALS_TH_LOW  (16)
#define IOCTL_APP_SET_PERSISTENCE (18)
#define IOCTL_APP_SET_PS_TH_LOW   (19)
#define IOCTL_APP_SET_MEASUR_TIME (20)
#define IOCTL_APP_SET_GENERAL     (253)
#define IOCTL_APP_READ_GENERAL    (254)
#define IOCTL_APP_READ_DRIVER_VER (255)

/************ define register for IC ************/
/* RPR0521 REGSTER */
#define REG_SYSTEMCONTROL         (0x40)
#define REG_MODECONTROL           (0x41)
#define REG_ALSPSCONTROL          (0x42)
#define REG_PERSISTENCE           (0x43)
#define REG_PSDATA                (0x44)
    #define REG_PSDATA_LSB            (0x44)
    #define REG_PSDATA_MBS            (0x45)
#define REG_ALSDATA0              (0x46)
    #define REG_ALSDATA0_LSB          (0x46)
    #define REG_ALSDATA0_MBS          (0x47)
#define REG_ALSDATA1              (0x48)
    #define REG_ALSDATA1_LSB          (0x48)
    #define REG_ALSDATA1_MBS          (0x49)
#define REG_INTERRUPT             (0x4A)
#define REG_PSTH                  (0x4B)
    #define REG_PSTH_LSB              (0x4B)
    #define REG_PSTH_MBS              (0x4C)
#define REG_PSTL                  (0x4D)
    #define REG_PSTL_LSB              (0x4D)
    #define REG_PSTL_MBS              (0x4E)
#define REG_ALSDATA0TH            (0x4F)
    #define REG_ALSDATA0TH_LSB       (0x4F)
    #define REG_ALSDATA0TH_MBS       (0x50)
#define REG_ALSDATA0TL            (0x51)
    #define REG_ALSDATA0TL_LSB        (0x51)
    #define REG_ALSDATA0TL_MBS        (0x52)

/* for power_on and interrupt */
#define ALS_DECIMAL_BIT           (15)
#define PS_VALUE_BIT              (16)
#define PS_ALS_DISABLE            (0)
#define PS_ALS_ENABLE             (1)

/* structure to activate sensor */
typedef struct {
    unsigned char power_als; /* value of whether to start ALS or not */
    unsigned char intr;      /* value of whether to use interrupt or not */
} POWERON_ALS_ARG;

/* structure to activate sensor */
typedef struct {
    unsigned char power_ps;  /* value of whether to start PS or not */
    unsigned char intr;      /* value of whether to use interrupt or not */
} POWERON_PS_ARG;

/* structure to read register value from sensor */
typedef struct {
    unsigned char adr_reg;   /* start register value */
    unsigned char *addr;     /* address to save value which read from sensor */
    unsigned char size;      /* length to read */
} GENREAD_ARG;

#endif /* _ROHM_RPR0521_I2C_IF_H_ */

