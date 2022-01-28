//leichenji add for QCOM Platform
#ifndef _ES9028_H_
#define _ES9028_H_

//regsiters map
#define ES9028_CONTROL_REG00	0x00 //system settings
#define ES9028_CONTROL_REG01	0x01 //input selection and deemphasis
#define ES9028_CONTROL_REG02	0x02 
#define ES9028_CONTROL_REG03	0x03
#define ES9028_CONTROL_REG04	0x04 //volume control1 by software
#define ES9028_CONTROL_REG05	0x05 //volume control2 by software
#define ES9028_CONTROL_REG06	0x06 //volume control3 by software
#define	ES9028_CONTROL_REG07	0x07 //general settings
#define ES9028_CONTROL_REG08	0x08 //GPIO 1 Configuration
#define ES9028_CONTROL_REG09	0x09 
#define ES9028_CONTROL_REG10	0x0A 
#define ES9028_CONTROL_REG11	0x0B //channel mapping
#define ES9028_CONTROL_REG12	0x0C //DPLL/ASRC Settings
#define ES9028_CONTROL_REG13	0x0D //Moudlator Settings
#define ES9028_CONTROL_REG14	0x0E //Soft Start Settings
#define ES9028_CONTROL_REG15	0x0F //volume1
#define ES9028_CONTROL_REG16	0x10 //volume2
#define ES9028_CONTROL_REG17	0x11
#define ES9028_CONTROL_REG18	0x12
#define ES9028_CONTROL_REG19	0x13
#define ES9028_CONTROL_REG20	0x14
#define ES9028_CONTROL_REG21	0x15
#define ES9028_CONTROL_REG22	0x16 
#define ES9028_CONTROL_REG23	0x17 
#define ES9028_CONTROL_REG24	0x18 
#define ES9028_CONTROL_REG25	0x19
#define ES9028_CONTROL_REG34	0x22
#define ES9028_CONTROL_REG35	0x23
#define ES9028_CONTROL_REG36	0x24
#define ES9028_CONTROL_REG37	0x25
#define ES9028_CONTROL_REG38	0x26

#define ES9028_CONTROL_REG42   0x2a

#define ES9028_CONTROL_REG64	0x40 // 4:2 chip id
#define ES9028_CONTROL_REG65	0x41 //gpio status

#define BBK_VIVO_AUDIO_DEBUG 1

#define ES9028_CHIP_ID 0x10
#define ES9028_ID_REG 0x40

struct es9028_regulator {
	const char *name;
};

struct es9028_dev_platform_data{
	char *driver_name;
	int rst_gpio;
	int pm_conv_gpio;
	struct es9028_regulator *vd_regulator, *va_regulator;
};

struct es9028_private_params {
	u8 regval_20;
	u8 regval_22;
	u8 regval_23;
	u8 regval_24;
	u8 regval_25;
	u8 regval_34;
	u8 regval_35;
	u8 regval_36;
	u8 regval_37;
	u8 regval_38;

};
#endif
