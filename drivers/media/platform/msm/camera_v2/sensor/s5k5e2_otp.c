/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
* 14/03/07      Hu Jin       add OTP for s5k5e2_rio5_ctcc(Ewelly)
* 14/05/06      YU Qijiang   add OTP for s5k5e2_cmcc
* 14/05/26      Hu Jin       /4 for 1st format version.
* 14/08/06      Yu Qijiang   don't check normal data for PR-704625
* 14/08/26      Zkx  		add otp check for pop10
* ------------------------------------------------------------------
*/

//#include "msm_camera_i2c.h"
/* Header file declaration */
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_dt_util.h"

//Debug log
#define S5K5E2_CMCC_OTP_DEBUG_ON 0

enum s5k5e2_cmcc_SN_type{
	s5k5e2_cmcc_FLAG_AF,
	s5k5e2_cmcc_FLAG_WB,
	s5k5e2_cmcc_FLAG_LSC,
	s5k5e2_cmcc_FLAG_NORMAL,
};

struct s5k5e2_cmcc_data_format{
	uint8_t  s5k5e2_cmcc_page_num; //[0x0, 0xD]
	uint16_t s5k5e2_cmcc_data_start;
    uint8_t  s5k5e2_cmcc_data_size;
	uint8_t  s5k5e2_cmcc_is_flag; //[NOTE:]if s5k5e2_cmcc_is_flag==1, s5k5e2_cmcc_data_size must be 1;
    enum s5k5e2_cmcc_SN_type type;
};

struct cmcc_time{
	uint16_t year;
	uint16_t month;
	uint16_t day;
};

struct cmcc_normal_data{
	uint16_t module_id;
	struct cmcc_time date;
	uint16_t lens_id;
	uint16_t vcm_id;
	uint16_t driver_id;
};

static struct cmcc_normal_data otp_data;

static struct s5k5e2_cmcc_data_format s5k5e2_cmcc_format_NORMAL[]={
	{3, 0x0A43, 1, 1, s5k5e2_cmcc_FLAG_NORMAL},//flag
	{3, 0x0A04, 7, 0, s5k5e2_cmcc_FLAG_NORMAL},
	{2, 0x0A43, 1, 1, s5k5e2_cmcc_FLAG_NORMAL},//flag
	{2, 0x0A04, 7, 0, s5k5e2_cmcc_FLAG_NORMAL},
};

static int32_t s5k5e2_cmcc_i2c_write_byte(struct msm_camera_i2c_client *client, uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!client)
        printk("OTP: s5k5e2_cmcc_TCT_client null\n");
    else{
        rc = client->i2c_func_tbl->i2c_write(client,
									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: write error\n");
    }
    return rc;
}

static int16_t s5k5e2_cmcc_i2c_read_byte(struct msm_camera_i2c_client *client, uint32_t addr, uint16_t *data)
{
    int32_t rc = -EFAULT;
    if (!client)
        printk("OTP: s5k5e2_cmcc_TCT_client null\n");
    else{
        rc = client->i2c_func_tbl->i2c_read(client,
									addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("OTP: read error\n");
    }
    return rc;
}

//page select //(0x00~0x0d)
static int32_t s5k5e2_cmcc_select_page(struct msm_camera_i2c_client *client, uint8_t page_no)
{
    uint32_t rc;

	rc = s5k5e2_cmcc_i2c_write_byte(client,0x0a00, 0x04);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);
	rc = s5k5e2_cmcc_i2c_write_byte(client,0x0a02, page_no);
	if (rc<0)
        printk("[OTP]%s %d error\n", __func__, __LINE__);

	return rc;
}
static void s5k5e2_cmcc_otp_read_enable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_cmcc_i2c_write_byte(client,0x0A00, 0x01) < 0)
        printk("%s error %d\n", __func__, __LINE__);

	mdelay(1);// > 1ms  ...HJ ensure 0x0A01=01h
	return ;
}
static void s5k5e2_cmcc_otp_read_disable(struct msm_camera_i2c_client *client)
{
    if (s5k5e2_cmcc_i2c_write_byte(client,0x0A00, 0x04) < 0)
        printk("%s error %d\n", __func__, __LINE__);
    if (s5k5e2_cmcc_i2c_write_byte(client,0x0A00, 0x00) < 0)
        printk("%s error %d\n", __func__, __LINE__);
}

static uint32_t s5k5e2_cmcc_real_check_valid(uint16_t data)
{
    int32_t rc = -1;
	uint8_t flag=0;

	flag = (uint8_t)(data&0xff);//keep 1 byte
	//flag >>= 6;
	if(flag == 0x01){
#if S5K5E2_CMCC_OTP_DEBUG_ON
		printk("valid group\n");
#endif
		rc = 0;
	}else if(flag == 0x00){
#if S5K5E2_CMCC_OTP_DEBUG_ON
		printk("empty\n");
#endif
	}else{
#if S5K5E2_CMCC_OTP_DEBUG_ON
		printk("invalid group\n");
#endif
	}
	return rc;
}

static uint32_t s5k5e2_cmcc_check_valid(enum s5k5e2_cmcc_SN_type type, uint16_t data)
{
    int32_t rc = -1;
	switch (type){
		case s5k5e2_cmcc_FLAG_WB:
            #if S5K5E2_CMCC_OTP_DEBUG_ON
			printk("check WB flag.....\n");
			#endif
            break;
		case s5k5e2_cmcc_FLAG_NORMAL:
            #if S5K5E2_CMCC_OTP_DEBUG_ON
			printk("check NORMAL flag.....\n");
			#endif
            break;
        default:
            printk("=====> [error]%s unknown type.....\n", __func__);
            break;
    }
	#if S5K5E2_CMCC_OTP_DEBUG_ON
		printk("flag data = [%d]\n", data);
    #endif
	rc = s5k5e2_cmcc_real_check_valid(data);

    return rc;
}

//HJ: pls note, same data should keep same flag/data order:
//must: flag+data; flag always come first;
//eg. flag+data(page 0) and flag+data(page 1) is valid;
//eg. flag+data(page 0) and data+flag(page 1) is not valid!!!
static void s5k5e2_cmcc_read_OTP_data(struct msm_camera_i2c_client *sensor_i2c_client,
    struct s5k5e2_cmcc_data_format *s5k5e2_cmcc_data_format, uint16_t size, uint16_t *dest)
{
	uint8_t s5k5e2_cmcc_data_size, s5k5e2_cmcc_is_flag, s5k5e2_cmcc_page_num;
    uint16_t s5k5e2_cmcc_data_start;
    uint32_t i, j, rc, read_line=0;
    uint16_t tmp_flag=0;
    uint8_t flag_count=0;
    uint8_t invalid_times=0;

	for(i=0; i<size; i++){
	    s5k5e2_cmcc_data_size = s5k5e2_cmcc_data_format->s5k5e2_cmcc_data_size;
		s5k5e2_cmcc_data_start = s5k5e2_cmcc_data_format->s5k5e2_cmcc_data_start;
		s5k5e2_cmcc_is_flag = s5k5e2_cmcc_data_format->s5k5e2_cmcc_is_flag;
        s5k5e2_cmcc_page_num = s5k5e2_cmcc_data_format->s5k5e2_cmcc_page_num;
        s5k5e2_cmcc_select_page(sensor_i2c_client, s5k5e2_cmcc_page_num);
	    s5k5e2_cmcc_otp_read_enable(sensor_i2c_client);
	    if(s5k5e2_cmcc_is_flag){// flag-s
            if(flag_count == 1){//have already read before
				#if S5K5E2_CMCC_OTP_DEBUG_ON
				printk("don't need read any more, have read line=%d\n", read_line);
                printk("read end: line=%d\n", i-1);
				#endif
                return ;
            }else{
            #if S5K5E2_CMCC_OTP_DEBUG_ON
                printk("=====> [flag]read addr:[0x%02x]\n", s5k5e2_cmcc_data_start);
            #endif
	            if(s5k5e2_cmcc_data_size != 1)
	                printk("=====> %s it should be keep 1\n", __func__);
				s5k5e2_cmcc_i2c_read_byte(sensor_i2c_client,s5k5e2_cmcc_data_start, &tmp_flag);
                #if S5K5E2_CMCC_OTP_DEBUG_ON
                printk("=====> i read flag = [0x%02x]\n", tmp_flag);
                #endif
	            rc = s5k5e2_cmcc_check_valid(s5k5e2_cmcc_data_format->type, tmp_flag);
				if(!rc){
					#if S5K5E2_CMCC_OTP_DEBUG_ON
	                printk("=====> flag: valid data\n");
                    #endif
	                flag_count++;// -> 1
					#if S5K5E2_CMCC_OTP_DEBUG_ON
                    printk("read begin: line=%d\n", i+1);
					#endif
                }else{
	                #if S5K5E2_CMCC_OTP_DEBUG_ON
	                printk("=====> flag: invalid data\n");
					#endif
                    invalid_times++;
				}
	            //s5k5e2_cmcc_data_format++; //read next line now
				//s5k5e2_cmcc_otp_read_disable(sensor_i2c_client);
			}
		}
		else{// data-s
            if(flag_count == 1){
                read_line++;
                for(j=0; j<s5k5e2_cmcc_data_size; j++){
					#if S5K5E2_CMCC_OTP_DEBUG_ON
                    printk("=====> [data]read addr:[0x%02x]\n", s5k5e2_cmcc_data_start+j);
					#endif
                    s5k5e2_cmcc_i2c_read_byte(sensor_i2c_client,s5k5e2_cmcc_data_start+j, dest+j);
                }
				#if S5K5E2_CMCC_OTP_DEBUG_ON
	            printk("=====> Total read line[%d]\n", i);
                for(j=0; j<s5k5e2_cmcc_data_size; j++)
					printk("[0x%02x],", dest[j]);
                printk("\n");
				dest += s5k5e2_cmcc_data_size;
                #endif
                //s5k5e2_cmcc_data_format++;// next line
			}else{//don't need read;
				#if S5K5E2_CMCC_OTP_DEBUG_ON
                printk("=====> Total skip line[%d]\n", i);
				#endif
                //s5k5e2_cmcc_data_format++;
			}
	    }
        s5k5e2_cmcc_data_format++; //read next line now
        s5k5e2_cmcc_otp_read_disable(sensor_i2c_client);
    }
    return ;
}

static void s5k5e2_cmcc_check_normal_data(struct msm_camera_i2c_client *i2c_client)
{
	s5k5e2_cmcc_read_OTP_data(i2c_client, s5k5e2_cmcc_format_NORMAL, ARRAY_SIZE(s5k5e2_cmcc_format_NORMAL), (uint16_t *)&otp_data);

#if S5K5E2_CMCC_OTP_DEBUG_ON
	printk(" \n****************************\n");
	//printk("otp_data len %d\n",sizeof(otp_data));
	printk(" module_id=[0x%x] production_time=[20%d-%d-%d]\n lens_id=[0x%x]\n, vcm_id=[0x%0x], driver_id=[0x%0x]\n", \
	 otp_data.module_id, \
	 otp_data.date.year, otp_data.date.month, otp_data.date.day,\
	 otp_data.lens_id, otp_data.vcm_id, otp_data.driver_id);
	printk(" \n****************************\n");
#endif

	return ;
}

uint16_t s5k5e2_check_module(struct msm_camera_i2c_client *i2c_client)
{
	s5k5e2_cmcc_check_normal_data(i2c_client);

	return otp_data.vcm_id;
}

EXPORT_SYMBOL(s5k5e2_check_module);


