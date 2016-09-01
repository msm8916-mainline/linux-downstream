#ifndef S5K3M2_IDOL347_OTP
#define S5K3M2_IDOL347_OTP
#include "msm_camera_i2c.h"

#define S5K3M2_IDOL347_OTP_PAGE_FULL_SIZE             32
#define S5K3M2_IDOL347_OTP_BASE_ADD                    0x0A04
#define S5K3M2_IDOL347_OTP_WB_FLAG_ADD                0x0A04
#define S5K3M2_IDOL347_OTP_WB_DATA_ADD                0x0A0F
#define S5K3M2_IDOL347_OTP_WB_GOLDEN_DATA_ADD        0x0A15
#define S5K3M2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM        24
#define S5K3M2_IDOL347_OTP_GAIN_DEFAULT                0x100

#define S5K3M2_IDOL347_OTP_AF_DATA_ADD            0x0A34

struct S5K3M2MIPI_otp_struct {
    uint32_t mid;
    uint32_t lens_id;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t golden_rg_ratio;
    uint32_t golden_bg_ratio;
};

static int32_t s5k3m2_otp_i2c_write(struct msm_sensor_ctrl_t * s_ctrl,
    uint32_t addr, uint16_t data,enum msm_camera_i2c_data_type data_type)
{
    int32_t rc = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
            addr, data, data_type);
    return rc;
}

static uint16_t s5k3m2_otp_i2c_read(struct msm_sensor_ctrl_t * s_ctrl,
    uint32_t addr)
{
    int rc = 0;
    uint16_t data = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
        addr, &data, MSM_CAMERA_I2C_BYTE_DATA);

    if (rc != 0)
    {
        pr_err("%s,rc = %d\n", __func__, rc); 
    }
    return data;
}

static int otp_LSC_on(struct msm_sensor_ctrl_t * s_ctrl)
{
    int rc = 0;
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6028,0x2000,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x602A,0x14FA,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6F12,0x0F,  MSM_CAMERA_I2C_BYTE_DATA);

    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6028,0x4000,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x602A,0x306A,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6F12,0x0068,MSM_CAMERA_I2C_WORD_DATA);

    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x602A,0x0B00,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6F12,0x01,  MSM_CAMERA_I2C_BYTE_DATA);

    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x602A,0x3058,MSM_CAMERA_I2C_WORD_DATA);
    rc |= s5k3m2_otp_i2c_write(s_ctrl,0x6F12,0x0900,MSM_CAMERA_I2C_WORD_DATA);

    return rc;
}

static int otp_stream_on(struct msm_sensor_ctrl_t * s_ctrl)
{
    int rc = 0;
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0136,0x1800, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0304,0x0006, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0306,0x0073, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x030C,0x0004, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x030E,0x0064, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0302,0x0001, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0300,0x0004, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x030A,0x0001, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0308,0x0008, MSM_CAMERA_I2C_WORD_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x0100,0x0100, MSM_CAMERA_I2C_WORD_DATA);

    return rc;
}

#define AF_OFFSET_L2 0x36
#define AF_OFFSET_L1 0x30

void s5k3m2_otp_AWB_on(struct msm_sensor_ctrl_t * s_ctrl,uint8_t *buffer, uint16_t size)
{
    uint16_t R_test;
    uint16_t B_test;
    uint16_t Gr_test;
    uint16_t Gb_test;
    uint16_t R_1x;
    uint16_t B_1x;
    uint16_t Gr_1x;
    uint16_t Gb_1x;
    uint16_t RoverGr_dec;
    uint16_t BoverGr_dec;
    uint16_t GboverGr_dec;
    uint16_t RoverGr_dec_base;
    uint16_t BoverGr_dec_base;
    uint16_t GboverGr_dec_base;
    uint16_t valid = 1;
    uint16_t offset = 0;

    if (!buffer)
    {
        return;
    }
    //pr_err("%s,buffer[0] = 0x%x,buffer[24]= 0x%x",__func__,buffer[0],buffer[24]);

    if (buffer[0] >> 6== valid)
    {
        offset = 11;
    }
    else if (buffer[24] >> 6 == valid)
    {
        offset = 35;
    }
    RoverGr_dec = buffer[offset] << 8 | buffer[offset + 1];
    BoverGr_dec = buffer[offset + 2] << 8 | buffer[offset + 3];
    GboverGr_dec = buffer[offset + 4] << 8 | buffer[offset + 5];
    RoverGr_dec_base = buffer[offset + 6] << 8 | buffer[offset + 7];
    BoverGr_dec_base = buffer[offset + 8] << 8 | buffer[offset + 9];
    GboverGr_dec_base = buffer[offset + 10] << 8 | buffer[offset + 11];


    //pr_err("RoverGr_dec=0x%x,BoverGr_dec=0x%x,GboverGr_dec=0x%x\n",RoverGr_dec,BoverGr_dec,GboverGr_dec);   

    if (!RoverGr_dec || !BoverGr_dec || !GboverGr_dec)
    {
        return;
    }
    R_1x = 0x0100; 
    B_1x = 0x0100;
	Gr_1x = 0x0100;
	Gb_1x = 0x0100;

    if (RoverGr_dec_base == 0)
    {
        RoverGr_dec_base = 518;
    }
    if (BoverGr_dec_base == 0)
    {
        BoverGr_dec_base = 536;
    }
    if (GboverGr_dec_base == 0)
    {
        GboverGr_dec_base = 0x0100;
    }

    R_test = RoverGr_dec_base*R_1x/RoverGr_dec;
    B_test = BoverGr_dec_base*B_1x/BoverGr_dec;
	Gr_test = Gr_1x;
	Gb_test = Gb_1x;

    //pr_err("R_test=0x%x,B_test=0x%x,Gr_test=0x%x,Gb_test=0x%x\n",R_test,B_test,Gr_test,Gb_test);   


	s5k3m2_otp_i2c_write(s_ctrl,0x6028,0x4000,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x602a,0x3056,MSM_CAMERA_I2C_WORD_DATA);
    s5k3m2_otp_i2c_write(s_ctrl,0x6f12,0x01,MSM_CAMERA_I2C_BYTE_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x602a,0x020e,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x6f12,Gr_test,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x602a,0x0210,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x6f12,R_test,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x602a,0x0212,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x6f12,B_test,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x602a,0x0214,MSM_CAMERA_I2C_WORD_DATA);
	s5k3m2_otp_i2c_write(s_ctrl,0x6f12,Gb_test,MSM_CAMERA_I2C_WORD_DATA);
	
}

static int s5k3m2_idol347_read_otp(struct msm_sensor_ctrl_t * s_ctrl,uint8_t page, uint16_t address, uint8_t *buffer, uint16_t size)
{
    uint8_t  reVal = 0;
    uint16_t i = 0;

    pr_err("s5k3m2_idol347 otp page:%d address:0x%x read_size:%d,buffer = %p\n ", page, address, size,buffer);

    if ((buffer==NULL)){
        pr_err("[JRD_CAM][S5K3M2OTP]error s5k3m2_idol347 read otp size=%d\n", size);
        return -1;
    }

    otp_LSC_on(s_ctrl);
	otp_stream_on(s_ctrl);
    //pr_err("%s,LSC on result = %d\n",__func__,); //stream on

    //pr_err("%s,stream_on result = %d\n",__func__,); //stream on

    msleep(10);

    //s5k3m2_otp_i2c_write(s_ctrl,0x0A00, 0x04);//make initial
    s5k3m2_otp_i2c_write(s_ctrl,0x0A02, page,MSM_CAMERA_I2C_BYTE_DATA);//select page
    s5k3m2_otp_i2c_write(s_ctrl,0x0A00, 0x01,MSM_CAMERA_I2C_BYTE_DATA);//read mode
    mdelay(1); 

    //pr_err("%s,0x0A00 data 0x%x\n",__func__,s5k3m2_otp_i2c_read(s_ctrl,0x0A00));

    while (i < size){
        int addr = address + i;
        reVal = s5k3m2_otp_i2c_read(s_ctrl,addr);
        buffer[i] = (uint8_t)reVal;
		
        if(i==0)
		i=11;
	 else if(i == 25)
	 	i=35;
	 else if(i == 46)
	 	i=size;
	 else
        	i++; 
    }


    s5k3m2_otp_i2c_write(s_ctrl,0x0A00, 0x00,MSM_CAMERA_I2C_BYTE_DATA);//disable NVM controller

    s5k3m2_otp_AWB_on(s_ctrl,buffer,size);


    return true;
}
static int s5k3m2_update_otp(struct msm_sensor_ctrl_t * s_ctrl)
{	uint8_t buf[64];
	int rc;
	rc = s5k3m2_idol347_read_otp(s_ctrl,31, S5K3M2_IDOL347_OTP_BASE_ADD, buf,64);
	return rc;

}

static uint32_t s5k3m2_get_otp_af(struct msm_sensor_ctrl_t* s_ctrl)
{
    uint32_t af_otp = 0;
    uint8_t value1, value2;

    s5k3m2_otp_i2c_write(s_ctrl,0x0A02, 31/*page*/,MSM_CAMERA_I2C_BYTE_DATA);//select page
    s5k3m2_otp_i2c_write(s_ctrl,0x0A00, 0x01,MSM_CAMERA_I2C_BYTE_DATA);//read mode
    mdelay(1); 
    value1 = s5k3m2_otp_i2c_read(s_ctrl, 0x0A34);
    value2 = s5k3m2_otp_i2c_read(s_ctrl, 0x0A3A);

    if ((value2 & 0xc0)>>6 == 1 || value2 == 1)
    {
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A3B) << 16);
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A3C) << 24);
        af_otp |= s5k3m2_otp_i2c_read(s_ctrl, 0x0A3D);
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A3E) <<  8);
    }
    else if ((value1 & 0xc0)>>6 == 1 || value1 == 1)
    {
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A35) << 16);
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A36) << 24);
        af_otp |= s5k3m2_otp_i2c_read(s_ctrl, 0x0A37);
        af_otp |= ((uint32_t) s5k3m2_otp_i2c_read(s_ctrl, 0x0A38) <<  8);
    }
    s5k3m2_otp_i2c_write(s_ctrl,0x0A00, 0x00,MSM_CAMERA_I2C_BYTE_DATA);//disable NVM controller

    return af_otp;
}

static uint32_t s5k3m2_temperature(struct msm_sensor_ctrl_t* s_ctrl, uint32_t * params)
{
#define FLAG_SET_TEMPERATURE    (0x01)
#define FLAG_ENABLE_TEMPERATURE (0x02)
    int32_t ret = 0;
    
    if( ( (*params) >>16) & FLAG_SET_TEMPERATURE)
    {   //enable/disable temperature reading
        pr_err("FF_FN: enabling/disabling temperature:%d not implemented!\n",(((*params) >>16) & FLAG_ENABLE_TEMPERATURE)>>1);
        //temperature sensor not implemented on S5K3M2 sensor, according to Samsung's FAE
        ret = -ENOSYS;
    }
    else
    {
/*
        //the value read from this register is constant        
        //according to Samsung's FAE, temperature sensor is not implemented on S5K3M2
        int16_t temperature;
        temp2 = s5k3m2_i2c_read_2(s_ctrl, 0x4000000A, MSM_CAMERA_I2C_WORD_DATA);
        pr_err("FF_FN: kernel read temperature:%d\n", temperature);
        (*params) = (*params & 0xffff0000) | temperature;
*/
        ret = -ENOSYS;
    }
    return ret;
}



#endif

