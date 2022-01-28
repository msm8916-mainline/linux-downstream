#ifndef __BBK_ALSPS_PARA_H__
#define __BBK_ALSPS_PARA_H__
struct ps_step{
    int min;
    int max;

    int in_coefficient;   //ps adc 8 bit: +   10bit: *
    int out_coefficient;  
};

struct ps_para{
    int base_value;         //default
    int base_value_min;
    int base_value_max;
        
    int step_num;
    struct ps_step step[5];
};

struct als_para{
    int base_value;         //default
    int base_value_min;
    int base_value_max;
};
//extern void set_ps_para_num(int para);
///////////////////////////////////////////////////

#define bbk_ps_para()	\
{	\
	{ \
		200, 0,   801,	\
		4,	\
		{	\
			{  0,  200,  60,  35},	\
			{200,  400,   55,  30},	\
			{400,  600,   50,  25},	\
			{600,  800,   45,  20},	\
		},	\
	},	\
}
#define bbk_tmp_ps_para()	\
{	\
	{ \
		200, 0,   900,	\
		5,	\
		{	\
			{  0,  200,  60,  35},	\
			{200,  400,   55,  30},	\
			{400,  600,   50,  25},	\
			{600,  800,   45,  20},	\
			{800,  900,   40,  20},	\
		},	\
	},	\
}

//0: "*"   1: "+"
#define PS_OPERATION 1

#define bbk_als_para()	\
{	\
	{ \
		118,   50,   1501,	\
	},	\
}




#endif //end of __BBK_ALSPS_PARA_H__
