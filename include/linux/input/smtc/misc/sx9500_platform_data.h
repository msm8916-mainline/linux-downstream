/*
 * include/linux/input/sx9500_platform_data.h
 *
 * SX9500 Platform Data
 * 2 cap differential  
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */
 /* Riven add SAR Driver */

#ifndef _SX9500_PLATFORM_DATA_H_
#define _SX9500_PLATFORM_DATA_H_
#define DIFFERENTIAL
#define USE_KERNEL_SUSPEND  //Riven fix unused function



struct smtc_reg_data {
    unsigned char reg;
    unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
    /*! The Key to send to the input */
    int keycode;
    /*! Mask to look for on Touch Status */
    int mask;
    /*! Current state of button. */
    int state;
};

struct _totalButtonInformation {
    struct _buttonInfo *buttons;
    int buttonSize;
//    struct input_dev *input;
};

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;


struct sx9500_platform_data {
    int i2c_reg_num;
    struct smtc_reg_data *pi2c_reg;
    
    pbuttonInformation_t pbuttonInformation;
    
/* Riven Add OF support START */
#ifdef CONFIG_OF
    int (*get_is_nirq_low)(int nirq);
#else
    int (*get_is_nirq_low)(void);
#endif
/* Riven Add OF support END */
    
    int     (*init_platform_hw)(void);
    void    (*exit_platform_hw)(void);
};
typedef struct sx9500_platform_data sx9500_platform_data_t;
typedef struct sx9500_platform_data *psx9500_platform_data_t;

#endif
