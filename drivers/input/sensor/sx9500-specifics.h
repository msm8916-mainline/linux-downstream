/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

/* IO Used for NIRQ */
#define GPIO_SX9500_NIRQ 54//114

#define SX9500_NIRQ OMAP_GPIO_IRQ(GPIO_SX9500_NIRQ)
#include <sx9500_platform_data.h>
#include <sx9500_i2c_reg.h>

static int sx9500_get_nirq_state(void)
{
    return !gpio_get_value(GPIO_SX9500_NIRQ);
}

static inline void __init sx9500_init(void)
{
    if ((gpio_request(GPIO_SX9500_NIRQ, "SX9500_NIRQ") == 0) && (gpio_direction_input(GPIO_SX9500_NIRQ) == 0)) {
        gpio_export(GPIO_SX9500_NIRQ, 0);
        printk(KERN_ERR "obtained gpio for SX9500_NIRQ\n");
    } 
    else {
        printk(KERN_ERR "could not obtain gpio for SX9500_NIRQ\n");
        return;
    }
}

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9500_i2c_reg_setup[] = {
    { .reg = SX9500_IRQ_ENABLE_REG,    .val = 0xFF, },
    { .reg = SX9500_CPS_CTRL1_REG,    .val = 0x43,  },
    { .reg = SX9500_CPS_CTRL2_REG,    .val = 0x77,  },
    { .reg = SX9500_CPS_CTRL3_REG,    .val = 0x01,  },
    { .reg = SX9500_CPS_CTRL4_REG,    .val = 0x20,  },
    { .reg = SX9500_CPS_CTRL5_REG,    .val = 0x16,  },
    { .reg = SX9500_CPS_CTRL6_REG,    .val = 0x04,  },
    { .reg = SX9500_CPS_CTRL7_REG,    .val = 0x40,  },
    { .reg = SX9500_CPS_CTRL8_REG,    .val = 0x00,  },
    { .reg = SX9500_CPS_CTRL0_REG,    .val = /*0x2C*/0x26,  },/* SCANPERIOD : 010(90ms), SENSOREN : 0110(CS1,CS2 pins enable) */
};


static struct _buttonInfo psmtcButtons[] = {
    { .keycode = KEY_0,    .mask = SX9500_TCHCMPSTAT_TCHSTAT0_FLAG,  },
    { .keycode = KEY_1,    .mask = SX9500_TCHCMPSTAT_TCHSTAT1_FLAG,  },
    { .keycode = KEY_2,    .mask = SX9500_TCHCMPSTAT_TCHSTAT2_FLAG,  },
    { .keycode = KEY_3,    .mask = SX9500_TCHCMPSTAT_TCHSTAT3_FLAG,  },
};


static struct _totalButtonInformation smtcButtonInformation = {
    .buttons = psmtcButtons,
    .buttonSize = ARRAY_SIZE(psmtcButtons),
};

static struct sx9500_platform_data sx9500_config = {
    /* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
    .get_is_nirq_low = sx9500_get_nirq_state,
    /*  pointer to an initializer function. Here in case needed in the future */
    //.init_platform_hw = sx9500_init_ts,
    .init_platform_hw = NULL,
    /*  pointer to an exit function. Here in case needed in the future */
    //.exit_platform_hw = sx9500_exit_ts,
    .exit_platform_hw = NULL,

    .pi2c_reg = sx9500_i2c_reg_setup,
    .i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup),

    .pbuttonInformation = &smtcButtonInformation,
};



/********************************************************************/
/* Use this define in the board specific array initializer */
#define SX9500_BOARD_INFO \
    I2C_BOARD_INFO("sx9500", 0x2B), \
        .flags         = I2C_CLIENT_WAKE, \
        .irq           = SX9500_NIRQ, \
        .platform_data = &sx9500_config,
/********************************************************************/

/* Below is an example of how to initialize this */
#if 0 
static struct i2c_board_info __initdata smtc_i2c_boardinfo[] = {
    {
        SX9500_BOARD_INFO
    },
};

static int __init overo_i2c_init(void)
{
    sx9500_init();
    omap_register_i2c_bus(3, 400, smtc_i2c_boardinfo, 
        ARRAY_SIZE(smtc_i2c2_boardinfo));
    return 0;
}
#endif



