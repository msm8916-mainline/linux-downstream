/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
* 14/03/07      Hu Jin       add OTP for s5k5e2_rio5_cmcc(Ewelly)
* 14/05/26      Hu Jin       /4
* ------------------------------------------------------------------
*/

#ifndef S5K5E2_M823_ORANGE_OTP
#define S5K5E2_M823_ORANGE_OTP
#include "msm_camera_i2c.h"


typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
} otp_date_t;

typedef struct {
    uint8_t channel_h;
    uint8_t channel_l;
} channel_t;

typedef struct otp_base_info_setting {
    uint8_t module_id;
    otp_date_t date;
    uint8_t lens_id;
    uint8_t vcm_id;
    uint8_t driver_id;
    uint8_t  color_temperature_id;
} otp_base_info_setting_t;


typedef struct  otp_wb_setting{
    uint8_t ob_data;
    channel_t rg;
    channel_t bg;
    channel_t g_ave;
     uint8_t reserved1;
   channel_t rg_golden;
    channel_t bg_golden;
    channel_t g_golden_ave;
    uint8_t reserved2;
} otp_wb_info_setting_t;

typedef struct otp_info_setting {
    otp_base_info_setting_t base_info;
    otp_wb_info_setting_t wb_info;
} otp_info_setting_t; 


static otp_info_setting_t s5k5e2_otp_data ;


//Debug log
#define OTP_DEBUG_ON  0

#define OTP_DEBUG(fmt,arg...)          do{\
                                         if(OTP_DEBUG_ON)\
                                         printk("[OTP]"fmt"\n", ##arg);\
                                       }while(0)

#define PRINK_OTP_INFO()		 do{\
							if(OTP_DEBUG_ON)\
								OTP_log_debug_info(&s5k5e2_otp_data); \
							 }while (0)


#define s5k5e2_RG_Gold  927/*tianhongwei modfiy for new IR camera*/
#define s5k5e2_BG_Gold  643/*tianhongwei modfiy for new IR camera*/

enum s5k5e2_SN_type{
	s5k5e2_FLAG_AF,
	s5k5e2_FLAG_WB,
	s5k5e2_FLAG_LSC,
	s5k5e2_FLAG_NORMAL,
};

struct s5k5e2_data_format{
	uint8_t  page; //[0x0, 0xD]
	uint16_t flag_addr;
	uint16_t date_addr;
    uint8_t  date_len;
    enum s5k5e2_SN_type type;
};

static struct s5k5e2_data_format s5k5e2_format_WB[]={
	{2, 0x0A43, 0x0A0E, 15, s5k5e2_FLAG_WB},//flag
	{3, 0x0A43, 0x0A0E, 15, s5k5e2_FLAG_WB},//flag
	{4, 0x0A43, 0x0A0E, 15, s5k5e2_FLAG_WB},//flag
};

static struct s5k5e2_data_format s5k5e2_format_NORMAL[]={
	{2, 0x0A43, 0x0A04, 8, s5k5e2_FLAG_NORMAL},//flag
	{3, 0x0A43, 0x0A04, 8, s5k5e2_FLAG_NORMAL},//flag
	{4, 0x0A43, 0x0A04, 8, s5k5e2_FLAG_NORMAL},//flag
};
static struct s5k5e2_data_format s5k5e2_format_LSC[]={
	{5, 0x0A43, 0x0A43, 1, s5k5e2_FLAG_LSC},//flag
};
static struct msm_camera_i2c_client *g_client;


static void OTP_log_debug_info(otp_info_setting_t *otp_data)
{
    printk("OTP page begin ~~~~~ \n");


    printk("OTP module_id = 0x%02x \n", otp_data->base_info.module_id);
    printk("OTP year = %d, month = %d, day = %d \n", otp_data->base_info.date.year, otp_data->base_info.date.month, otp_data->base_info.date.day);
    printk("OTP lens_id = 0x%02x \n", otp_data->base_info.lens_id);
    printk("OTP vcm_id = 0x%02x \n", otp_data->base_info.vcm_id);
    printk("OTP driver_id = 0x%02x \n", otp_data->base_info.driver_id);
    printk("OTP color_temperature_id = 0x%02x \n", otp_data->base_info.color_temperature_id);
	
    printk("OTP r_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.rg.channel_h, otp_data->wb_info.rg.channel_l);
    printk("OTP b_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.bg.channel_h, otp_data->wb_info.bg.channel_l);
    printk("OTP g_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.g_ave.channel_h, otp_data->wb_info.g_ave.channel_l);

    printk("OTP r_golden_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.rg_golden.channel_h, otp_data->wb_info.rg_golden.channel_l);
    printk("OTP b_golden_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.bg_golden.channel_h, otp_data->wb_info.bg_golden.channel_l);
    printk("OTP g_golden_ave.channel_h = 0x%02x, channel_l = 0x%02x \n", otp_data->wb_info.g_golden_ave.channel_h, otp_data->wb_info.g_golden_ave.channel_l);
    printk("OTP reserved1 = 0x%02x \n", otp_data->wb_info.reserved1);
    printk("OTP reserved2 = 0x%02x \n", otp_data->wb_info.reserved2);


    printk("OTP page end ~~~~~ \n");
}



static int32_t s5k5e2_i2c_write_byte(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!g_client)
        printk("OTP: s5k5e2_TCT_client null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_write(g_client, 
            									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: write error\n");
    }
    return rc;
}

static int16_t s5k5e2_i2c_read_byte(uint32_t addr, uint16_t *data)
{
    int32_t rc = -EFAULT;
    if (!g_client)
        printk("OTP: s5k5e2_TCT_client null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_read(g_client,
            									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: read error\n");
    }
    return rc;
}

//page select //(0x00~0x0d)
static int32_t s5k5e2_select_page(struct msm_camera_i2c_client *client, uint8_t page_no)
{
    uint32_t rc;

	rc = s5k5e2_i2c_write_byte(0x0a00, 0x04);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);
	rc = s5k5e2_i2c_write_byte(0x0a02, page_no);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);

	return rc;
}
static void s5k5e2_otp_read_enable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_i2c_write_byte(0x0A00, 0x01) < 0)
        printk("%s error %d\n", __func__, __LINE__);

	mdelay(1);// > 1ms  ...HJ ensure 0x0A01=01h
	return ;
}
static void s5k5e2_otp_read_disable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_i2c_write_byte(0x0A00, 0x04) < 0)
        printk("%s error %d\n", __func__, __LINE__);
    if (s5k5e2_i2c_write_byte(0x0A00, 0x00) < 0)
        printk("%s error %d\n", __func__, __LINE__);
}

static int8_t s5k5e2_real_check_valid(uint16_t data)
{
	int8_t rc = -1;
	uint8_t flag=0;

	flag = (uint8_t)(data&0xff);//keep 1 byte
	switch (flag){
		case 0x00:
			OTP_DEBUG("empty");
		break;
		case 0x01:
			OTP_DEBUG("valid");
		rc = 0;
		break;
		case 0xff:
			OTP_DEBUG("invalid");
		break;
		default:
			printk("OTP : error flag \n");
		break;
		}
	return rc;
}

static int8_t s5k5e2_check_valid(enum s5k5e2_SN_type type, uint16_t data)
{
	int8_t rc = -1;
	switch (type){
		case s5k5e2_FLAG_WB:
			OTP_DEBUG("check WB flag");
		break;
		case s5k5e2_FLAG_NORMAL:
			OTP_DEBUG("check NORMAL flag");
		break;
		case s5k5e2_FLAG_LSC:
			OTP_DEBUG("check LSC flag");
		break;
		default:
		printk(" [error]%s unknown type.....\n", __func__);
		break;
	}
	OTP_DEBUG("flag data = [%d]", data);
	rc = s5k5e2_real_check_valid(data);

	return rc;
}

static void s5k5e2_read_OTP_data(struct msm_camera_i2c_client *sensor_i2c_client,
    struct s5k5e2_data_format * data_format, uint16_t size,uint8_t *dest)
{
	uint8_t  otp_data_info[32];
	uint8_t data_size;
	uint16_t start_addr;
	uint32_t i,j;
	uint16_t tmp_flag=0,tmp_date;
	int8_t rc;
	
	for(i=0; i<size; i++)
	{
		OTP_DEBUG("page = %d ",data_format->page);
		
		s5k5e2_select_page(sensor_i2c_client, data_format->page);
		s5k5e2_otp_read_enable(sensor_i2c_client);
		
		start_addr = data_format->flag_addr;
		s5k5e2_i2c_read_byte(start_addr, &tmp_flag);
		rc = s5k5e2_check_valid(data_format->type, tmp_flag);
		if(rc>=0)
		{
			OTP_DEBUG("data_size = %d ",data_format->date_len);
			OTP_DEBUG("start_addr = %d ",data_format->date_addr);
			OTP_DEBUG("type = %d ",data_format->type);

			data_size = data_format->date_len;
			start_addr = data_format->date_addr;
			for(j=0; j<data_size; j++){
				s5k5e2_i2c_read_byte(start_addr+j, &tmp_date);
				otp_data_info[j] = tmp_date&0xff;

				OTP_DEBUG("[%x :%x ]  %x",start_addr+j,otp_data_info[j],tmp_date);
			}
			memcpy(dest,otp_data_info,data_size);
			s5k5e2_otp_read_disable(sensor_i2c_client);
			return;
		}
		data_format++; //read next line now
		s5k5e2_otp_read_disable(sensor_i2c_client);
    }
    return ;
}

static int s5k5e2_update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	//add for awb OTP calibration, factory forget to down scale gain
	R_gain = R_gain>>2;
	G_gain = G_gain>>2;
	B_gain = B_gain>>2;
//R Gain
	s5k5e2_i2c_write_byte(0x0210, (R_gain & 0xFF00) >> 8);
	s5k5e2_i2c_write_byte(0x0211, R_gain & 0xff);
//Gr gain
	s5k5e2_i2c_write_byte(0x020E, (G_gain & 0xFF00) >> 8);
	s5k5e2_i2c_write_byte(0x020F, G_gain & 0xff);
//Gb gain
	s5k5e2_i2c_write_byte(0x0214, (G_gain & 0xFF00) >> 8);
	s5k5e2_i2c_write_byte(0x0215, G_gain & 0xff);
//B gain
	s5k5e2_i2c_write_byte(0x0212, (B_gain & 0xFF00) >> 8);
	s5k5e2_i2c_write_byte(0x0213, B_gain & 0xff);
	return 0;
}


void s5k5e2_cal_R_G_B_gain(otp_wb_info_setting_t  *wb_info)
{
    int RG_Current, BG_Current;
    int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;

	
    RG_Current = (((wb_info->rg.channel_h&0x00ff)<<8 ) | wb_info->rg.channel_l);
    BG_Current =(((wb_info->bg.channel_h&0x00ff)<<8 ) | wb_info->bg.channel_l);
     OTP_DEBUG("rg = %02x:%02x ",wb_info->rg.channel_h,wb_info->rg.channel_l);
     OTP_DEBUG("bg = %02x:%02x ",wb_info->bg.channel_h,wb_info->bg.channel_l);
     OTP_DEBUG("g = %02x:%02x ",wb_info->g_ave.channel_h,wb_info->g_ave.channel_l);

     OTP_DEBUG("rg G= %02x:%02x ",wb_info->rg_golden.channel_h,wb_info->rg_golden.channel_l);
     OTP_DEBUG("bg G= %02x:%02x ",wb_info->bg_golden.channel_h,wb_info->bg_golden.channel_l);
     OTP_DEBUG("g G= %02x:%02x ",wb_info->g_golden_ave.channel_h,wb_info->g_golden_ave.channel_l);

    OTP_DEBUG("OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", s5k5e2_RG_Gold, RG_Current, s5k5e2_BG_Gold, BG_Current);
	if( RG_Current == 0 || BG_Current == 0 )
		return;
	if(BG_Current < s5k5e2_BG_Gold) {
        if (RG_Current<s5k5e2_RG_Gold){
            G_gain = 0x400;
            B_gain = 0x400 * s5k5e2_BG_Gold / BG_Current;
            R_gain = 0x400 * s5k5e2_RG_Gold / RG_Current;
        }else {
            R_gain = 0x400;
            G_gain = 0x400 * RG_Current / s5k5e2_RG_Gold;
            B_gain = G_gain * s5k5e2_BG_Gold / BG_Current;
        }
    }else {
        if (RG_Current < s5k5e2_RG_Gold) {
            B_gain = 0x400;
            G_gain = 0x400 * BG_Current / s5k5e2_BG_Gold;
            R_gain = G_gain * s5k5e2_RG_Gold / RG_Current;
        }else {
            G_gain_B = 0x400 * BG_Current / s5k5e2_BG_Gold;
            G_gain_R = 0x400 * RG_Current / s5k5e2_RG_Gold;
            if(G_gain_B > G_gain_R ) {
                B_gain = 0x400;
                G_gain = G_gain_B;
                R_gain = G_gain * s5k5e2_RG_Gold /RG_Current;
            }
            else {
                R_gain = 0x400;
                G_gain = G_gain_R;
                B_gain= G_gain * s5k5e2_BG_Gold / BG_Current;
            }
        }
    }

    s5k5e2_update_awb_gain(R_gain, G_gain, B_gain);
	return ;
}


// call this function after S5K5E2 initialization
// return value:  0 update success
// 1, no OTP
static int s5k5e2_update_otp_wb(struct msm_camera_i2c_client *i2c_client)
{
    
	s5k5e2_read_OTP_data(i2c_client, s5k5e2_format_WB, ARRAY_SIZE(s5k5e2_format_WB),(uint8_t *)&s5k5e2_otp_data.wb_info);
	s5k5e2_cal_R_G_B_gain(&s5k5e2_otp_data.wb_info);
	return 0;
}


static void s5k5e2_check_normal_data(struct msm_camera_i2c_client *i2c_client)
{   
	s5k5e2_read_OTP_data(i2c_client, s5k5e2_format_NORMAL, ARRAY_SIZE(s5k5e2_format_NORMAL),(uint8_t *)&s5k5e2_otp_data.base_info);

	return ;
}

static void s5k5e2_check_lsc_data(struct msm_camera_i2c_client *i2c_client)
{   
	uint8_t lsc_status = 0;
	s5k5e2_read_OTP_data(i2c_client, s5k5e2_format_LSC, ARRAY_SIZE(s5k5e2_format_LSC),(uint8_t *)&lsc_status);
	OTP_DEBUG("lsc_status = %d ",lsc_status);
	if(lsc_status == 1)
	{
		s5k5e2_i2c_write_byte(0x3400, 0x00);
	}
	return ;
}



static void s5k5e2_OTP_all_config(struct msm_camera_i2c_client *i2c_client)
{
    //int ret1;
	s5k5e2_check_normal_data(i2c_client);
	s5k5e2_update_otp_wb(i2c_client);
	s5k5e2_check_lsc_data(i2c_client);
	PRINK_OTP_INFO();
}

void s5k5e2_m823_orange_otp_config(struct msm_camera_i2c_client *i2c_client)
{
    g_client = i2c_client;
    s5k5e2_OTP_all_config(i2c_client);
	//...
}

#endif

