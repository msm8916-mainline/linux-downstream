#ifndef S5K5E2_IDOL347_OTP
#define S5K5E2_IDOL347_OTP
#include "msm_camera_i2c.h"
#define S5K5E2_IDOL347_OTP_PAGE_FULL_SIZE             64
#define S5K5E2_IDOL347_OTP_BASE_ADD                    0x0A04
#define S5K5E2_IDOL347_OTP_WB_FLAG_ADD                0x0A43
#define S5K5E2_IDOL347_OTP_WB_DATA_ADD                0x0A0F
#define S5K5E2_IDOL347_OTP_WB_GOLDEN_DATA_ADD        0x0A16
#define S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_ADD      0x0A42
#define S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM        30
#define S5K5E2_IDOL347_OTP_GAIN_DEFAULT                0x100

struct S5K5E2MIPI_otp_struct {
    uint32_t mid;
    uint32_t lens_id;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t golden_rg_ratio;
    uint32_t golden_bg_ratio;
};

static bool awb_checksum_ready = false;
static int s5k5e2_idol347_read_otp(struct msm_sensor_ctrl_t * s_ctrl,uint8_t page, uint16_t address, uint8_t *buffer, uint16_t size)
{
    uint8_t  reVal = 0;
    uint16_t i = 0;

    //pr_err("s5k5e2_idol347 otp page:%d address:0x%x read_size:%d\n ", page, address, size);

    if ((buffer==NULL) || (size > S5K5E2_IDOL347_OTP_PAGE_FULL_SIZE)){
        pr_err("[JRD_CAM][S5K5E2OTP]error s5k5e2_idol347 read otp size=%d\n", size);
        return -1;
    }

    otp_i2c_write(s_ctrl,0x0A00, 0x04);//make initial
    otp_i2c_write(s_ctrl,0x0A02, page);//select page
    otp_i2c_write(s_ctrl,0x0A00, 0x01);//read mode
    mdelay(1);

    while (i < size){
        reVal = otp_i2c_read(s_ctrl,address + i);
        *(buffer+i) =(uint8_t)reVal;
        i++;
    }

    otp_i2c_write(s_ctrl,0x0A00, 0x04);//make initial
    otp_i2c_write(s_ctrl,0x0A00, 0x00);//disable NVM controller

    return true;
}

static uint32_t s5k5e2_idol347_otp_mid_awb_checksum(struct msm_sensor_ctrl_t * s_ctrl,uint8_t Page)
{
    uint8_t otpChecksumBuffer[S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM], otpChecksumValue=0;
    int i4RetValue = 0;
    uint32_t i = 0, temp = 0;

    i4RetValue = s5k5e2_idol347_read_otp(s_ctrl,Page, S5K5E2_IDOL347_OTP_BASE_ADD, otpChecksumBuffer, S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM);
    if (i4RetValue < 0){
        pr_err("fail to read s5k5e2_idol347 otp checksum data\n");
    }
    for (i = 0; i < S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM - 1; i++){
        temp += otpChecksumBuffer[i];
    }

    //pr_err("MID AWB checksum temp=%d\n", temp);
    /*Add for read checksum value*/
    i4RetValue = s5k5e2_idol347_read_otp(s_ctrl, Page, S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_ADD, &otpChecksumValue, 1);
    if (i4RetValue < 0)
    {
        pr_err("fail to read s5k5e2_idol347 otp checksum value.\n");
    }
    else
    {
        pr_err("LJTAO: ochecksumValue=0x%X, tempChecksum=0x%X.\n", otpChecksumValue, ((temp % 255) + 1));
    }

    if (((temp % 255) + 1) == otpChecksumValue)//otpChecksumBuffer[S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM -1])
    {
        pr_err("MID AWB checksum successfully\n");
        /*for (i = 0; i < S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM; i++)
        {
            pr_err("otpChecksumBuffer[0x%x]=%d\n", S5K5E2_IDOL347_OTP_BASE_ADD + i, otpChecksumBuffer[i]);
        }*/
        return true;
    }else{
        pr_err("fail to MID AWB checksum\n");
        for (i = 0; i < S5K5E2_IDOL347_OTP_MID_AWB_CHECKSUM_NUM; i++)
        {
            pr_err("otpChecksumBuffer[0x%x]=%d\n", S5K5E2_IDOL347_OTP_BASE_ADD + i, otpChecksumBuffer[i]);
        }
        return false;
    }
}

static int32_t s5k5e2_idol347_update_awb_gain(struct msm_sensor_ctrl_t * s_ctrl,uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    pr_err("[JRD_CAM][S5K5E2_IDOL347_OTP]R_gain=0x%x G_gain=0x%x, B_gain=0x%x\n", R_gain, G_gain, B_gain);

    otp_i2c_write(s_ctrl,0x020E, G_gain>>8);
    otp_i2c_write(s_ctrl,0x020F, G_gain& 0xFF);
    otp_i2c_write(s_ctrl,0x0210, R_gain >>8);
    otp_i2c_write(s_ctrl,0x0211, R_gain & 0xFF);
    otp_i2c_write(s_ctrl,0x0212, B_gain >>8);
    otp_i2c_write(s_ctrl,0x0213, B_gain & 0xFF);
    otp_i2c_write(s_ctrl,0x0214, G_gain>>8);
    otp_i2c_write(s_ctrl,0x0215, G_gain& 0xFF);
    return 0;
}

static int32_t s5k5e2_idol347_otp_read_update_wb(struct msm_sensor_ctrl_t * s_ctrl)
{
    struct S5K5E2MIPI_otp_struct otp_awb;
    uint32_t i = 0, page_index = 0, i4RetValue = 0;
    uint32_t R_gain, B_gain, G_gain, GR_gain, GB_gain;
    uint8_t flag[2];
    uint8_t otp_awb_data[6];
    uint8_t otp_awb_golden_data[6];
    uint32_t S5K5E2_RG_Ratio_Typical = 0x00;
    uint32_t S5K5E2_BG_Ratio_Typical = 0x00;

    //check otp awb flag
    for( i = 2; i < 5; i++ ){
        s5k5e2_idol347_read_otp(s_ctrl,i, S5K5E2_IDOL347_OTP_WB_FLAG_ADD, flag, 1);//check page 2 3 4
        if ( 0x01 == flag[0]){
            page_index = i;
            pr_err("[JRD_CAM][S5K5E2_IDOL347_OTP]find otp awb page=%d\n", page_index);
            break;
        }
    }

    if ( i == 5){
        pr_err("[JRD_CAM][S5K5E2_IDOL347_OTP]fail ot check otp awb page error\n");
        return -1;
    }

    //pr_err("check otp awb successfully page=%d\n", page_index);

    if(awb_checksum_ready == false){
       i4RetValue = s5k5e2_idol347_otp_mid_awb_checksum(s_ctrl,page_index);
       if(i4RetValue != true){
             return -2;
       }
       awb_checksum_ready = true;
    }

    //read otp awb data
    s5k5e2_idol347_read_otp(s_ctrl,page_index, S5K5E2_IDOL347_OTP_WB_DATA_ADD, otp_awb_data, 6);
    s5k5e2_idol347_read_otp(s_ctrl,page_index, S5K5E2_IDOL347_OTP_WB_GOLDEN_DATA_ADD, otp_awb_golden_data, 6);

    for (i = 0; i < 6; i++){
        pr_err("otp awb data[0x%x]=0x%x\n",S5K5E2_IDOL347_OTP_WB_DATA_ADD + i, otp_awb_data[i]);
    }

    for (i = 0; i < 6; i++){
        pr_err("otp golden awb data[0x%x]=0x%x\n",S5K5E2_IDOL347_OTP_WB_GOLDEN_DATA_ADD + i, otp_awb_golden_data[i]);
    }

    otp_awb.rg_ratio = (otp_awb_data[0]<<8) | otp_awb_data[1];
    otp_awb.bg_ratio = (otp_awb_data[2]<<8) | otp_awb_data[3];
    otp_awb.golden_rg_ratio = (otp_awb_golden_data[0]<<8) | otp_awb_golden_data[1];
    otp_awb.golden_bg_ratio = (otp_awb_golden_data[2]<<8) | otp_awb_golden_data[3];
    S5K5E2_RG_Ratio_Typical = otp_awb.golden_rg_ratio;
    S5K5E2_BG_Ratio_Typical = otp_awb.golden_bg_ratio;

    if (((S5K5E2_RG_Ratio_Typical * 7) < (otp_awb.rg_ratio * 10)) && ((S5K5E2_RG_Ratio_Typical*13) > (otp_awb.rg_ratio * 10)) &&
        ((S5K5E2_BG_Ratio_Typical * 7) < (otp_awb.bg_ratio * 10)) && ((S5K5E2_BG_Ratio_Typical*13) > (otp_awb.bg_ratio * 10))
       )
    {
       // pr_err("rg_ratio=0x%x bg_ratio=0x%x golden_rg_ratio=0x%x golden_bg_ratio=0x%x\n",
       //             otp_awb.rg_ratio, otp_awb.bg_ratio, otp_awb.golden_rg_ratio, otp_awb.golden_bg_ratio);

        if( otp_awb.bg_ratio < S5K5E2_BG_Ratio_Typical ){
            if( otp_awb.rg_ratio < S5K5E2_RG_Ratio_Typical){
                pr_err("current_opt.bg_ratio < S5K5E2_BG_Ratio_Typical && cuttent_otp.rg_ratio < S5K5E2_RG_Ratio_Typical\n");
                G_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT;
                B_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * S5K5E2_BG_Ratio_Typical / otp_awb.bg_ratio;
                R_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * S5K5E2_RG_Ratio_Typical / otp_awb.rg_ratio;
            }else{
                pr_err("current_opt.bg_ratio < S5K5E2_BG_Ratio_Typical && cuttent_otp.rg_ratio > S5K5E2_RG_Ratio_Typical\n");
                R_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT;
                G_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * otp_awb.rg_ratio / S5K5E2_RG_Ratio_Typical;
                B_gain = G_gain * S5K5E2_BG_Ratio_Typical / otp_awb.bg_ratio;
            }
        }else{
            if(otp_awb.rg_ratio < S5K5E2_RG_Ratio_Typical){
                pr_err("current_opt.bg_ratio > S5K5E2_BG_Ratio_Typical && cuttent_otp.rg_ratio < S5K5E2_RG_Ratio_Typical\n");
                B_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT;
                G_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * otp_awb.bg_ratio / S5K5E2_BG_Ratio_Typical;
                R_gain = G_gain * S5K5E2_RG_Ratio_Typical / otp_awb.rg_ratio;
            }else{
                pr_err("current_opt.bg_ratio > S5K5E2_BG_Ratio_Typical && cuttent_otp.rg_ratio > S5K5E2_RG_Ratio_Typical\n");
                GB_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * otp_awb.bg_ratio / S5K5E2_BG_Ratio_Typical;
                GR_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT * otp_awb.rg_ratio / S5K5E2_RG_Ratio_Typical;

                if(GB_gain > GR_gain){
                    B_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT;
                    G_gain = GB_gain;
                    R_gain = G_gain * S5K5E2_RG_Ratio_Typical / otp_awb.rg_ratio;
                }else{
                    R_gain = S5K5E2_IDOL347_OTP_GAIN_DEFAULT;
                    G_gain = GR_gain;
                    B_gain = G_gain * S5K5E2_BG_Ratio_Typical / otp_awb.bg_ratio;
                }
            }
        }

        s5k5e2_idol347_update_awb_gain(s_ctrl,R_gain, G_gain, B_gain);
    }else{
        pr_err("[JRD_CAM][S5K5E2_IDOL347_OTP]fail to check awb rg_ratio and bg_ratio error\n");
    }
    //pr_err("end to s5k5e2_idol347 otp read update_wb\n");

    return 0;
}
#endif
