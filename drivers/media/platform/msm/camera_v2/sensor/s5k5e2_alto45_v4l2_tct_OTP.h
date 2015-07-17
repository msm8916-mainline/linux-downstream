/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
* 14/03/07      Hu Jin       add OTP for s5k5e2_rio5_ctcc(Ewelly)
* 14/05/06      YU Qijiang   add OTP for s5k5e2_alto45_cmcc
* 14/05/26      Hu Jin       /4 for 1st format version.
* 14/08/06      Yu Qijiang   don't check normal data for PR-704625
* ------------------------------------------------------------------
*/

#ifndef S5K5E2_ALTO45_CMCC_V4L2_TCT_OTP
#define S5K5E2_ALTO45_CMCC_V4L2_TCT_OTP
#include "msm_camera_i2c.h"

//Debug log
#define S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON 0

#define PERSO_DIS 0
#define s5k5e2_alto45_cmcc_Gain_Default 0x0400//0x0100
#ifdef CONFIG_TCT_8X16_ALTO45_LATAM_B28
#define s5k5e2_alto45_cmcc_RG_Gold  870//202//0.792760725 * 256;// sync with GS
#define s5k5e2_alto45_cmcc_BG_Gold  640//161//0.627300368 * 256;// sync with GS
#else
#define s5k5e2_alto45_cmcc_RG_Gold  911//202//0.792760725 * 256;// sync with GS
#define s5k5e2_alto45_cmcc_BG_Gold  744//161//0.627300368 * 256;// sync with GS
#endif
enum s5k5e2_alto45_cmcc_SN_type{
	s5k5e2_alto45_cmcc_FLAG_AF,
	s5k5e2_alto45_cmcc_FLAG_WB,
	s5k5e2_alto45_cmcc_FLAG_LSC,
	s5k5e2_alto45_cmcc_FLAG_NORMAL,
};

struct s5k5e2_alto45_cmcc_data_format{
	uint8_t  s5k5e2_alto45_cmcc_page_num; //[0x0, 0xD]
	uint16_t s5k5e2_alto45_cmcc_data_start;
    uint8_t  s5k5e2_alto45_cmcc_data_size;
	uint8_t  s5k5e2_alto45_cmcc_is_flag; //[NOTE:]if s5k5e2_alto45_cmcc_is_flag==1, s5k5e2_alto45_cmcc_data_size must be 1;
    enum s5k5e2_alto45_cmcc_SN_type type;
};
static uint16_t s5k5e2_alto45_cmcc_RG_BG_ratio[5];/* R/G H, R/G L, + B/G H, B/G L + G_ave*/
static uint16_t s5k5e2_alto45_cmcc_normal_data[7];
static int need_check = 0;

static struct s5k5e2_alto45_cmcc_data_format s5k5e2_alto45_cmcc_format_WB[]={
	{3, 0x0A43, 1, 1, s5k5e2_alto45_cmcc_FLAG_WB},//flag
	{3, 0x0A0F, 5, 0, s5k5e2_alto45_cmcc_FLAG_WB},
	{2, 0x0A43, 1, 1, s5k5e2_alto45_cmcc_FLAG_WB},//flag
	{2, 0x0A0F, 5, 0, s5k5e2_alto45_cmcc_FLAG_WB},
};

static struct s5k5e2_alto45_cmcc_data_format s5k5e2_alto45_cmcc_format_NORMAL[]={
	{3, 0x0A43, 1, 1, s5k5e2_alto45_cmcc_FLAG_NORMAL},//flag
	{3, 0x0A04, 7, 0, s5k5e2_alto45_cmcc_FLAG_NORMAL},
	{2, 0x0A43, 1, 1, s5k5e2_alto45_cmcc_FLAG_NORMAL},//flag
	{2, 0x0A04, 7, 0, s5k5e2_alto45_cmcc_FLAG_NORMAL},
};

static struct msm_camera_i2c_client *g_client;

static int32_t s5k5e2_alto45_cmcc_i2c_write_byte(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!g_client)
        printk("OTP: s5k5e2_alto45_cmcc_TCT_client null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_write(g_client, 
            									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: write error\n");
    }
    return rc;
}

static int16_t s5k5e2_alto45_cmcc_i2c_read_byte(uint32_t addr, uint16_t *data)
{
    int32_t rc = -EFAULT;
    if (!g_client)
        printk("OTP: s5k5e2_alto45_cmcc_TCT_client null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_read(g_client,
            									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: read error\n");
    }
    return rc;
}

//page select //(0x00~0x0d)
static int32_t s5k5e2_alto45_cmcc_select_page(struct msm_camera_i2c_client *client, uint8_t page_no)
{
    uint32_t rc;

	rc = s5k5e2_alto45_cmcc_i2c_write_byte(0x0a00, 0x04);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);
	rc = s5k5e2_alto45_cmcc_i2c_write_byte(0x0a02, page_no);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);

	return rc;
}
static void s5k5e2_alto45_cmcc_otp_read_enable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_alto45_cmcc_i2c_write_byte(0x0A00, 0x01) < 0)
        printk("%s error %d\n", __func__, __LINE__);

	mdelay(1);// > 1ms  ...HJ ensure 0x0A01=01h
	return ;
}
static void s5k5e2_alto45_cmcc_otp_read_disable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_alto45_cmcc_i2c_write_byte(0x0A00, 0x04) < 0)
        printk("%s error %d\n", __func__, __LINE__);
    if (s5k5e2_alto45_cmcc_i2c_write_byte(0x0A00, 0x00) < 0)
        printk("%s error %d\n", __func__, __LINE__);
}

static uint32_t s5k5e2_alto45_cmcc_real_check_valid(uint16_t data)
{
    int32_t rc = -1;
	uint8_t flag=0;

	flag = (uint8_t)(data&0xff);//keep 1 byte
	//flag >>= 6;
	if(flag == 0x01){
#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
		printk("valid group\n");
#endif
		rc = 0;
	}else if(flag == 0x00){
#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
		printk("empty\n");
#endif
	}else{
#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
		printk("invalid group\n");
#endif
	}
	return rc;
}

static uint32_t s5k5e2_alto45_cmcc_check_valid(enum s5k5e2_alto45_cmcc_SN_type type, uint16_t data)
{
    int32_t rc = -1;
	switch (type){
		case s5k5e2_alto45_cmcc_FLAG_WB:
            #if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
			printk("check WB flag.....\n");
			#endif
            break;
		case s5k5e2_alto45_cmcc_FLAG_NORMAL:
            #if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
			printk("check NORMAL flag.....\n");
			#endif
            break;
        default:
            printk("=====> [error]%s unknown type.....\n", __func__);
            break;
    }
	#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
		printk("flag data = [%d]\n", data);
    #endif
	rc = s5k5e2_alto45_cmcc_real_check_valid(data);

    return rc;
}

//HJ: pls note, same data should keep same flag/data order:
//must: flag+data; flag always come first;
//eg. flag+data(page 0) and flag+data(page 1) is valid;
//eg. flag+data(page 0) and data+flag(page 1) is not valid!!!
static void s5k5e2_alto45_cmcc_read_OTP_data(struct msm_camera_i2c_client *sensor_i2c_client,
    struct s5k5e2_alto45_cmcc_data_format *s5k5e2_alto45_cmcc_data_format, uint16_t size, uint16_t *dest)
{
	uint8_t s5k5e2_alto45_cmcc_data_size, s5k5e2_alto45_cmcc_is_flag, s5k5e2_alto45_cmcc_page_num;
    uint16_t s5k5e2_alto45_cmcc_data_start;
    uint32_t i, j, rc, read_line=0;
    uint16_t tmp_flag=0;
    uint8_t flag_count=0;
    uint8_t invalid_times=0;

	for(i=0; i<size; i++){
	    s5k5e2_alto45_cmcc_data_size = s5k5e2_alto45_cmcc_data_format->s5k5e2_alto45_cmcc_data_size;
		s5k5e2_alto45_cmcc_data_start = s5k5e2_alto45_cmcc_data_format->s5k5e2_alto45_cmcc_data_start;
		s5k5e2_alto45_cmcc_is_flag = s5k5e2_alto45_cmcc_data_format->s5k5e2_alto45_cmcc_is_flag;
        s5k5e2_alto45_cmcc_page_num = s5k5e2_alto45_cmcc_data_format->s5k5e2_alto45_cmcc_page_num;
        s5k5e2_alto45_cmcc_select_page(sensor_i2c_client, s5k5e2_alto45_cmcc_page_num);
	    s5k5e2_alto45_cmcc_otp_read_enable(sensor_i2c_client);
	    if(s5k5e2_alto45_cmcc_is_flag){// flag-s
            if(flag_count == 1){//have already read before
				#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
				printk("don't need read any more, have read line=%d\n", read_line);
                printk("read end: line=%d\n", i-1);
				#endif
                return ;
            }else{
            #if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
                printk("=====> [flag]read addr:[0x%02x]\n", s5k5e2_alto45_cmcc_data_start);
            #endif
	            if(s5k5e2_alto45_cmcc_data_size != 1)
	                printk("=====> %s it should be keep 1\n", __func__);
				s5k5e2_alto45_cmcc_i2c_read_byte(s5k5e2_alto45_cmcc_data_start, &tmp_flag);
                #if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
                printk("=====> i read flag = [0x%02x]\n", tmp_flag);
                #endif
	            rc = s5k5e2_alto45_cmcc_check_valid(s5k5e2_alto45_cmcc_data_format->type, tmp_flag);
				if(!rc){
					#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
	                printk("=====> flag: valid data\n");
                    #endif
	                flag_count++;// -> 1
					#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
                    printk("read begin: line=%d\n", i+1);
					#endif
                }else{
	                #if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
	                printk("=====> flag: invalid data\n");
					#endif
                    invalid_times++;
				}
	            //s5k5e2_alto45_cmcc_data_format++; //read next line now
				//s5k5e2_alto45_cmcc_otp_read_disable(sensor_i2c_client);
			}
		}
		else{// data-s
            if(flag_count == 1){
                read_line++;
                for(j=0; j<s5k5e2_alto45_cmcc_data_size; j++){
					#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
                    printk("=====> [data]read addr:[0x%02x]\n", s5k5e2_alto45_cmcc_data_start+j);
					#endif
                    s5k5e2_alto45_cmcc_i2c_read_byte(s5k5e2_alto45_cmcc_data_start+j, dest+j);
                }
				#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
	            printk("=====> Total read line[%d]\n", i);
                for(j=0; j<s5k5e2_alto45_cmcc_data_size; j++)
					printk("[0x%02x],", dest[j]);
                printk("\n");
				dest += s5k5e2_alto45_cmcc_data_size;
                #endif
                //s5k5e2_alto45_cmcc_data_format++;// next line
			}else{//don't need read;
				#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
                printk("=====> Total skip line[%d]\n", i);
				#endif
                //s5k5e2_alto45_cmcc_data_format++;
			}
	    }
        s5k5e2_alto45_cmcc_data_format++; //read next line now
        s5k5e2_alto45_cmcc_otp_read_disable(sensor_i2c_client);
    }
    return ;
}

static int s5k5e2_alto45_cmcc_update_awb_gain(int R_gain, int G_gain, int B_gain)
{
       //add for awb OTP calibration, factory forget to down scale gain
       R_gain = R_gain>>2;
       G_gain = G_gain>>2;
       B_gain = B_gain>>2;
//R Gain
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0210, (R_gain & 0xFF00) >> 8);
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0211, R_gain & 0xff);
//Gr gain
	s5k5e2_alto45_cmcc_i2c_write_byte(0x020E, (G_gain & 0xFF00) >> 8);
	s5k5e2_alto45_cmcc_i2c_write_byte(0x020F, G_gain & 0xff);
//Gb gain
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0214, (G_gain & 0xFF00) >> 8);
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0215, G_gain & 0xff);
//B gain
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0212, (B_gain & 0xFF00) >> 8);
	s5k5e2_alto45_cmcc_i2c_write_byte(0x0213, B_gain & 0xff);
	return 0;
}
/*
s5k5e2_alto45_cmcc_RG_BG_ratio[0]: R
s5k5e2_alto45_cmcc_RG_BG_ratio[1]: B
s5k5e2_alto45_cmcc_RG_BG_ratio[2]: G
*/
#if PERSO_DIS
void s5k5e2_alto45_cmcc_cal_R_G_B_gain_old(uint16_t *p)
{
#if 0
	#define s5k5e2_alto45_cmcc_Gain_Default 0x0100
#if 0
    //float s5k5e2_alto45_cmcc_RG_Gold = 0.792760725;
	//float s5k5e2_alto45_cmcc_BG_Gold = 0.627300368;
#else//*1024
	#define s5k5e2_alto45_cmcc_RG_Gold  812//0.792760725 * 1024;
	#define s5k5e2_alto45_cmcc_BG_Gold  642//0.627300368 * 1024;
#endif
    int RG_Current, BG_Current;
    
    int R_ration, B_ration;
    int R_gain, G_gain, B_gain;
#if 0
	//RG_Current = ((float)p[0]) / ((float)p[2]);
	//BG_Current = ((float)p[1]) / ((float)p[2]);
#else
	RG_Current = (p[0]<<10)/p[2];
	BG_Current = (p[1]<<10)/p[2];
#endif    
	R_ration = s5k5e2_alto45_cmcc_RG_Gold / RG_Current;
	B_ration = s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
    printk("OTP:s5k5e2_alto45_cmcc_RG_Gold/RG_Current=[%d:%d],s5k5e2_alto45_cmcc_RG_Gold/RG_Current=[%d:%d]\n", s5k5e2_alto45_cmcc_RG_Gold, RG_Current, s5k5e2_alto45_cmcc_BG_Gold, BG_Current);
    if (R_ration>=1)
    {
        if (B_ration>=1){
            G_gain=s5k5e2_alto45_cmcc_Gain_Default ;
            R_gain=s5k5e2_alto45_cmcc_Gain_Default*R_ration;
            B_gain=s5k5e2_alto45_cmcc_Gain_Default*B_ration;
        }else{
            B_gain=s5k5e2_alto45_cmcc_Gain_Default;
            G_gain=s5k5e2_alto45_cmcc_Gain_Default/B_ration;
            R_gain=s5k5e2_alto45_cmcc_Gain_Default*R_ration/B_ration;
        }
    }else{
        if (B_ration>=1){
            R_gain=s5k5e2_alto45_cmcc_Gain_Default;
            G_gain=s5k5e2_alto45_cmcc_Gain_Default/R_ration;
            B_gain=s5k5e2_alto45_cmcc_Gain_Default*B_ration/R_ration;
        }else{
            if (R_ration>=B_ration && B_ration<1)
            {
                B_gain=s5k5e2_alto45_cmcc_Gain_Default;
                G_gain=s5k5e2_alto45_cmcc_Gain_Default/B_ration;
                R_gain=s5k5e2_alto45_cmcc_Gain_Default*R_ration/B_ration;
            }else{
                R_gain=s5k5e2_alto45_cmcc_Gain_Default;
                G_gain=s5k5e2_alto45_cmcc_Gain_Default/R_ration;
                B_gain=s5k5e2_alto45_cmcc_Gain_Default*B_ration/R_ration;
            }
        }
    }
#else
#if 0
        //float s5k5e2_alto45_cmcc_RG_Gold = 0.792760725;
        //float s5k5e2_alto45_cmcc_BG_Gold = 0.627300368;
#else//*1024
	//#define s5k5e2_alto45_cmcc_Gain_Default 0x0100
	//#define s5k5e2_alto45_cmcc_RG_Gold  202//0.792760725 * 256;
	//#define s5k5e2_alto45_cmcc_BG_Gold  161//0.627300368 * 256;
	
    int RG_Current, BG_Current;
    int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;


	RG_Current = (p[0]<<8)/p[2];
	BG_Current = (p[1]<<8)/p[2];
    printk("OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", s5k5e2_alto45_cmcc_RG_Gold, RG_Current, s5k5e2_alto45_cmcc_BG_Gold, BG_Current);
#endif
	if(BG_Current < s5k5e2_alto45_cmcc_BG_Gold) {
	    if (RG_Current<s5k5e2_alto45_cmcc_RG_Gold){
	        G_gain = 0x100;
	        B_gain = 0x100 * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
	        R_gain = 0x100 * s5k5e2_alto45_cmcc_RG_Gold / RG_Current;
	    }else {
	        R_gain = 0x100;
	        G_gain = 0x100 * RG_Current / s5k5e2_alto45_cmcc_RG_Gold;
	        B_gain = G_gain * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
	    }
	}else {
	    if (RG_Current < s5k5e2_alto45_cmcc_RG_Gold) {
	        B_gain = 0x100;
	        G_gain = 0x100 * BG_Current / s5k5e2_alto45_cmcc_BG_Gold;
	        R_gain = G_gain * s5k5e2_alto45_cmcc_RG_Gold / RG_Current;
	    }else {
	        G_gain_B = 0x100 * BG_Current / s5k5e2_alto45_cmcc_BG_Gold;
	        G_gain_R = 0x100 * RG_Current / s5k5e2_alto45_cmcc_RG_Gold;
	        if(G_gain_B > G_gain_R ) {
	            B_gain = 0x100;
	            G_gain = G_gain_B;
	            R_gain = G_gain * s5k5e2_alto45_cmcc_RG_Gold /RG_Current;
	        }
	        else {
	            R_gain = 0x100;
	            G_gain = G_gain_R;
	            B_gain= G_gain * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
	        }
	    }
	}

#endif
    s5k5e2_alto45_cmcc_update_awb_gain(R_gain, G_gain, B_gain);
	return ;
}
#endif

void s5k5e2_alto45_cmcc_cal_R_G_B_gain_new(uint16_t *p)
{
    int RG_Current, BG_Current;
    int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;

    RG_Current = (p[0]<<8) | p[1];
    BG_Current = (p[2]<<8) | p[3];
/* [PLATFORM]-Add-BEGIN by TCTNB.qijiang.yu, 2014/05/12, OTP open panic */
	if(RG_Current == 0 || BG_Current == 0)
		return;
/* [PLATFORM]-Add-END by TCTNB.qijiang.yu*/
#if PERSO_DIS
    RG_Current /= 4;//1024->256
    BG_Current /= 4;
#endif
    printk("OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", s5k5e2_alto45_cmcc_RG_Gold, RG_Current, s5k5e2_alto45_cmcc_BG_Gold, BG_Current);
    if(BG_Current < s5k5e2_alto45_cmcc_BG_Gold) {
        if (RG_Current<s5k5e2_alto45_cmcc_RG_Gold){
            G_gain = 0x400;
            B_gain = 0x400 * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
            R_gain = 0x400 * s5k5e2_alto45_cmcc_RG_Gold / RG_Current;
        }else {
            R_gain = 0x400;
            G_gain = 0x400 * RG_Current / s5k5e2_alto45_cmcc_RG_Gold;
            B_gain = G_gain * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
        }
    }else {
        if (RG_Current < s5k5e2_alto45_cmcc_RG_Gold) {
            B_gain = 0x400;
            G_gain = 0x400 * BG_Current / s5k5e2_alto45_cmcc_BG_Gold;
            R_gain = G_gain * s5k5e2_alto45_cmcc_RG_Gold / RG_Current;
        }else {
            G_gain_B = 0x400 * BG_Current / s5k5e2_alto45_cmcc_BG_Gold;
            G_gain_R = 0x400 * RG_Current / s5k5e2_alto45_cmcc_RG_Gold;
            if(G_gain_B > G_gain_R ) {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * s5k5e2_alto45_cmcc_RG_Gold /RG_Current;
            }
            else {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain= G_gain * s5k5e2_alto45_cmcc_BG_Gold / BG_Current;
            }
        }
    }

    s5k5e2_alto45_cmcc_update_awb_gain(R_gain, G_gain, B_gain);
	return ;
#if 0
	rg = 512*s5k5e2_alto45_cmcc_RG_BG_ratio[0]/s5k5e2_alto45_cmcc_RG_BG_ratio[2];
	bg = 512*s5k5e2_alto45_cmcc_RG_BG_ratio[1]/s5k5e2_alto45_cmcc_RG_BG_ratio[2];
	printk("rg=[0x%04x]/[0x%04x], bg=[0x%04x]/[0x%04x]\n", rg, bg, \
    S5K5E2_ALTO45_CMCC_RG_Ratio_Typical, S5K5E2_ALTO45_CMCC_BG_Ratio_Typical);
/*
r_ratio = 512 * (golden_rg) /(current_rg);
b_ratio = 512 * (golden_bg) /(current_bg);
*/

#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
	printk("rg=[0x%04x], bg=[0x%04x]\n", rg, bg);
#endif
    //calculate G gain
	//0x100 = 1x gain  : sansung: 512
	if(bg < S5K5E2_ALTO45_CMCC_BG_Ratio_Typical) {
		if (rg<S5K5E2_ALTO45_CMCC_RG_Ratio_Typical){
			G_gain = 0x400;
			B_gain = 0x400 * S5K5E2_ALTO45_CMCC_BG_Ratio_Typical / bg;
			R_gain = 0x400 * S5K5E2_ALTO45_CMCC_RG_Ratio_Typical / rg;
		}else {
			R_gain = 0x400;
			G_gain = 0x400 * rg / S5K5E2_ALTO45_CMCC_RG_Ratio_Typical;
			B_gain = G_gain * S5K5E2_ALTO45_CMCC_BG_Ratio_Typical /bg;
		}
	}else {
		if (rg < S5K5E2_ALTO45_CMCC_RG_Ratio_Typical) {
			B_gain = 0x400;
			G_gain = 0x400 * bg / S5K5E2_ALTO45_CMCC_BG_Ratio_Typical;
			R_gain = G_gain * S5K5E2_ALTO45_CMCC_RG_Ratio_Typical / rg;
		}else {
			G_gain_B = 0x400 * bg / S5K5E2_ALTO45_CMCC_BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / S5K5E2_ALTO45_CMCC_RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * S5K5E2_ALTO45_CMCC_RG_Ratio_Typical /rg;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain= G_gain * S5K5E2_ALTO45_CMCC_BG_Ratio_Typical/bg;
			}
		}
	}
#endif
	return ;
}


// call this function after S5K5E2_alto45_cmcc initialization
// return value:  0 update success
// 1, no OTP
static int s5k5e2_alto45_cmcc_update_otp_wb(struct msm_camera_i2c_client *i2c_client)
{
	//int R_gain, G_gain, B_gain;//, G_gain_R, G_gain_B;
	//float R_ration, B_ration;// rg,bg;
    
	s5k5e2_alto45_cmcc_read_OTP_data(i2c_client, s5k5e2_alto45_cmcc_format_WB, ARRAY_SIZE(s5k5e2_alto45_cmcc_format_WB), s5k5e2_alto45_cmcc_RG_BG_ratio);
/*
version 1:
    0x3A22  R_ave
    0x3A23  B_ave
    0x3A24  G_ave
    0x3A25  Reserve
version 2:
    0x3A22  R/G_H
    0x3A23  R/G_L
    0x3A24  B/G_H
    0x3A25  B/G_L
    0x3A26  G_ave

	s5k5e2_alto45_cmcc_RG_BG_ratio[0]: R
	s5k5e2_alto45_cmcc_RG_BG_ratio[1]: B
	s5k5e2_alto45_cmcc_RG_BG_ratio[2]: G
*/
#if 0
	rg = s5k5e2_alto45_cmcc_RG_BG_ratio[0]<<8 | s5k5e2_alto45_cmcc_RG_BG_ratio[1];
	bg = s5k5e2_alto45_cmcc_RG_BG_ratio[2]<<8 | s5k5e2_alto45_cmcc_RG_BG_ratio[3];
#endif
#if PERSO_DIS
	if (s5k5e2_alto45_cmcc_RG_BG_ratio[4] == 0){//old
		s5k5e2_alto45_cmcc_cal_R_G_B_gain_old(s5k5e2_alto45_cmcc_RG_BG_ratio);
		printk("OTP: old\n");
        
	} else {
		s5k5e2_alto45_cmcc_cal_R_G_B_gain_new(s5k5e2_alto45_cmcc_RG_BG_ratio);
        printk("OTP: new\n");
    }
#else
		s5k5e2_alto45_cmcc_cal_R_G_B_gain_new(s5k5e2_alto45_cmcc_RG_BG_ratio);
        printk("OTP: new\n");
#endif
	return 0;
}

static void s5k5e2_alto45_cmcc_check_normal_data(struct msm_camera_i2c_client *i2c_client)
{   
	s5k5e2_alto45_cmcc_read_OTP_data(i2c_client, s5k5e2_alto45_cmcc_format_NORMAL, ARRAY_SIZE(s5k5e2_alto45_cmcc_format_NORMAL), s5k5e2_alto45_cmcc_normal_data);
#if S5K5E2_ALTO45_CMCC_OTP_DEBUG_ON
    printk(" \n****************************\n");
    printk(" module_id=[0x%x] \
         production_time=[20%d-%d-%d]\n\
         lens_id=[0x%x]\n, vcm_id=[0x%0x], driver_id=[0x%0x]\n", \
        s5k5e2_alto45_cmcc_normal_data[0], \
        s5k5e2_alto45_cmcc_normal_data[1], s5k5e2_alto45_cmcc_normal_data[2], s5k5e2_alto45_cmcc_normal_data[3],\
        s5k5e2_alto45_cmcc_normal_data[4], s5k5e2_alto45_cmcc_normal_data[5], s5k5e2_alto45_cmcc_normal_data[6]);
    printk(" \n****************************\n");
#endif
#if PERSO_DIS
	if(s5k5e2_alto45_cmcc_normal_data[0] == 0x06){
		printk("OTP: Qtech module\n");
    }else{
        printk("OTP: Unknow module\n");
    }
#endif
	return ;
}
static void s5k5e2_alto45_cmcc_OTP_all_calibration(struct msm_camera_i2c_client *i2c_client)
{
    //int ret1;
    if(need_check)
	s5k5e2_alto45_cmcc_check_normal_data(i2c_client);
	s5k5e2_alto45_cmcc_update_otp_wb(i2c_client);
}

void S5K5E2_ALTO45_CMCC_TCT_OTP_calibration(struct msm_camera_i2c_client *i2c_client)
{
    g_client = i2c_client;
    s5k5e2_alto45_cmcc_OTP_all_calibration(i2c_client);
	//...
}
void S5K5E2_ALTO5_CMCC_TCT_OTP_calibration(struct msm_camera_i2c_client *i2c_client)
{
    g_client = i2c_client;
    s5k5e2_alto45_cmcc_OTP_all_calibration(i2c_client);
	//...
}
#endif

