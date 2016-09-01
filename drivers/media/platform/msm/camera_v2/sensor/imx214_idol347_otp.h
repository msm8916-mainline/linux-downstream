#ifndef IMX214_IDOL347_OTP
#define IMX214_IDOL347_OTP
#include "msm_camera_i2c.h"

#define IMX214OTP_PAGE_FULL_SIZE     64
#define IMX214OTP_BASE_ADDR           0x0A04
#define IMX214OTP_WB_FLAG_ADD         0x0A34
#define IMX214OTP_WB_DATA_ADD         0x0A35
#define IMX214_GAIN_DEFAULT            0x100

struct IMX214MIPI_otp_struct {
    uint32_t mid;
    uint32_t lens_id;
    uint32_t rg_ratio;
    uint32_t bg_ratio;
    uint32_t golden_rg_ratio;
    uint32_t golden_bg_ratio;
};

static int32_t otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t addr, uint16_t data)
{
    int32_t rc = 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
            addr, data, MSM_CAMERA_I2C_BYTE_DATA);
    return rc;
}

static uint16_t otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t addr)
{
    uint16_t data = 0;

    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
        addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
    return data;
}

static bool imx214semco_read_otp(struct msm_sensor_ctrl_t * s_ctrl,uint8_t page, uint16_t address, uint8_t *buffer, uint16_t size)
{
    uint8_t retry = 0, reVal = 0, rdrdy_status = 0;
    uint16_t i = 0;

    if ((buffer==NULL) || (size > IMX214OTP_PAGE_FULL_SIZE)){
        pr_err("[JRD_CAM][IMX214OTP]error imx214semco_read_otp size=%d\n", size);
        return -1;
    }

    otp_i2c_write(s_ctrl,0x0A02, page);//select page
    otp_i2c_write(s_ctrl,0x0A00, 0x01);//read mode
    mdelay(1);
    rdrdy_status = otp_i2c_read(s_ctrl,0x0A01);

    if( 0x01 & rdrdy_status)
    {
        pr_err("Ready to read OTP!\n");
    }
    else
    {
        for (retry = 0; retry < 3; retry++)
        {
            rdrdy_status = otp_i2c_read(s_ctrl,0x0A01);
            if(0x01 & rdrdy_status)
            {
                pr_err("Ready to read OTP! retry =%d \n", retry);
                break;
            }
            msleep(1);
        }
        if (retry == 3)
        {
            pr_err("Haven't been Ready to read OTP!\n");
            return false;
        }
    }

    while (i < size)
    {
        reVal = otp_i2c_read(s_ctrl,address + i);
        *(buffer+i) =(uint8_t)reVal;
        i++;
    }

    return true;
}

static int32_t imx214_update_awb_gain(struct msm_sensor_ctrl_t * s_ctrl,uint32_t R_gain, uint32_t G_gain, uint32_t B_gain)
{
    //printk("[JRD_CAM][IMX214OTP]R_gain=0x%x G_gain=0x%x, B_GAB_gain=0x%x\n", R_gain, G_gain, B_gain);

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

#define IMX214_ADDR(x) IMX214OTP_BASE_ADDR + x

static uint32_t imx214_get_otp_af(struct msm_sensor_ctrl_t* s_ctrl)
{

    int8_t j = 0;
    int8_t af_page = -1;
    uint32_t af_otp = 0;

    uint8_t buffOTP[16];

    #define OTPBUF_AF_G0        buffOTP[0]
    #define OTPBUF_AF_G1        buffOTP[1]
    #define OTPBUF_AF_G2        buffOTP[2]
    #define OTPBUF_AF_START     buffOTP[3]
    #define OTPBUF_AF_INF_HIGH  buffOTP[3]
    #define OTPBUF_AF_INF_LOW   buffOTP[4]
    #define OTPBUF_AF_10CM_HIGH buffOTP[5]
    #define OTPBUF_AF_10CM_LOW  buffOTP[6]

    //printk("FN_FF: Dumping OTP AF info!.\n");

    for( j =0; j < 16; j++ )
    {
        buffOTP[j] = 0;
    }

    // read page 0 AF
    if( false == imx214semco_read_otp(s_ctrl,0x00,IMX214_ADDR(0x20),&OTPBUF_AF_G0,1) )
    {
        printk("FN_FF: read OTP AF error!.\n");
        return af_otp;
    }

    // read page 1 AF
    if( false == imx214semco_read_otp(s_ctrl,0x01,IMX214_ADDR(0x20),&OTPBUF_AF_G1,1) )
    {
        printk("FN_FF: read OTP AF error!.\n");
        return af_otp;
    }

    // read page 2 AF
    if( false == imx214semco_read_otp(s_ctrl,0x02,IMX214_ADDR(0x20),&OTPBUF_AF_G2,1) )
    {
        printk("FN_FF: read OTP AF error!.\n");
        return af_otp;
    }

    //printk("FN_FF: OTPBUF_AF_G0: %d .\n", OTPBUF_AF_G0 );
    //printk("FN_FF: OTPBUF_AF_G1: %d .\n", OTPBUF_AF_G1 );
    //printk("FN_FF: OTPBUF_AF_G2: %d .\n", OTPBUF_AF_G2 );

    if( 0x01 == (OTPBUF_AF_G0 >> 6) )
        af_page = 0x00;
    else if( 0x01 == (OTPBUF_AF_G1 >> 6) )
        af_page = 0x01;
    else if( 0x01 == (OTPBUF_AF_G2 >> 6) )
        af_page = 0x02;

    if( -1 == af_page )
    {
        printk("FN_FF: no valid OTP AF page!\n");
        return af_otp;
    }

    // read 4 bytes OTP AF data
    if( false == imx214semco_read_otp(s_ctrl,af_page,IMX214_ADDR(0x21),&OTPBUF_AF_START,4) )
    {
        printk("FN_FF: read OTP AF error!\n");
        return af_otp;
    }

    /*printk("FN_FF: OTPBUF_AF_INF_HIGH :%d .\n", (uint16_t)OTPBUF_AF_INF_HIGH );
    printk("FN_FF: OTPBUF_AF_INF_LOW :%d .\n", (uint16_t)OTPBUF_AF_INF_LOW );
    printk("FN_FF: OTPBUF_AF_10CM_HIGH :%d .\n", (uint16_t)OTPBUF_AF_10CM_HIGH );
    printk("FN_FF: OTPBUF_AF_10CM_LOW :%d .\n", (uint16_t)OTPBUF_AF_10CM_LOW );
    printk("FN_FF: OTPBUF_AF_INF :%d .\n", (((uint16_t)OTPBUF_AF_INF_HIGH << 8) | OTPBUF_AF_INF_LOW)  );
    printk("FN_FF: OTPBUF_AF_10CM :%d .\n", (((uint16_t)OTPBUF_AF_10CM_HIGH << 8) | OTPBUF_AF_10CM_LOW) );*/

    af_otp |= OTPBUF_AF_INF_LOW;
    af_otp |= ((uint32_t)OTPBUF_AF_INF_HIGH << 8);
    af_otp |= ((uint32_t)OTPBUF_AF_10CM_LOW << 16);
    af_otp |= ((uint32_t)OTPBUF_AF_10CM_HIGH << 24);

    return af_otp;
}

static uint32_t imx214_temperature(struct msm_sensor_ctrl_t* s_ctrl, uint32_t * params)
{
#define FLAG_SET_TEMPERATURE    (0x01)
#define FLAG_ENABLE_TEMPERATURE (0x02)
    int32_t ret = 0;
    
    if( ( (*params) >>16) & FLAG_SET_TEMPERATURE)
    {   //enable/disable temperature reading
        pr_err("FF_FN: enabling/disabling temperature:%d\n",(((*params) >>16) & FLAG_ENABLE_TEMPERATURE)>>1);
        ret = otp_i2c_write(s_ctrl, 0x0138, (((*params) >>16) & FLAG_ENABLE_TEMPERATURE)>>1 );
    }
    else
    {
        uint8_t temperature;
        //pr_err("FF_FN: reading temperature\n");
        temperature = otp_i2c_read(s_ctrl, 0x013A);
        //pr_err("FF_FN: kernel read temperature:%d\n", temperature);
        (*params) = (*params & 0xffffff00) | temperature;
    }
    return ret;
}
//wenyuan.li@tcl 20150228 bug936683 get framelength from kernel+
static uint32_t imx214_framelength(struct msm_sensor_ctrl_t* s_ctrl, uint32_t * params)
{
    int32_t ret = 0;
        uint8_t framelength_hi,framelength_low;
        pr_err("FF_FN: reading temperature\n");
        framelength_hi = otp_i2c_read(s_ctrl, 0x0340);
        framelength_low = otp_i2c_read(s_ctrl, 0x0341);

        pr_err("FF_FN: kernel read framelength_hi:%x,framelength_low:%x\n", framelength_hi,framelength_low);
        (*params) = (*params & 0x00000000) | (framelength_hi<<8)|framelength_low;
    return ret;
}
//wenyuan.li@tcl 20150228 bug936683 get framelength from kernel -
static int imx214_get_otp(struct msm_sensor_ctrl_t * s_ctrl)
{
    uint32_t i = 0, page_index = 0;
    uint32_t R_gain, B_gain, G_gain, GR_gain, GB_gain;
    uint8_t flag[2];
    uint8_t otp_awb_data[12];
    struct IMX214MIPI_otp_struct otp_awb;
    uint32_t IMX214_RG_Ratio_Typical=0x00;
    uint32_t IMX214_BG_Ratio_Typical=0x00;

    //pr_err("%s,Start to read OTP\n",__func__);
    //check otp awb flag
    for( i = 0; i < 3; i++ ){
        imx214semco_read_otp(s_ctrl,i, IMX214OTP_WB_FLAG_ADD, flag, 1);//check page 0 1 2
        if ( 0x40 == flag[0]){
            page_index = i;
            pr_err("[JRD_CAM][IMX214OTP]find otp awb page=%d\n", page_index);
            break;
        }
    }

    if ( i == 3){
        printk("[JRD_CAM][IMX214OTP]fail ot check otp awb page error\n");
        return -1;
    }

    //pr_err("check otp awb successfully page\n");
    //read otp awb data
    imx214semco_read_otp(s_ctrl,page_index, IMX214OTP_WB_DATA_ADD, otp_awb_data, 12);

    for (i = 0; i < 12; i++){
        pr_err("otp awb data[0x%x]=0x%x\n",IMX214OTP_WB_DATA_ADD + i, otp_awb_data[i]);
    }

    otp_awb.rg_ratio = (otp_awb_data[0]<<8) | otp_awb_data[1];
    otp_awb.bg_ratio = (otp_awb_data[2]<<8) | otp_awb_data[3];
    otp_awb.golden_rg_ratio = (otp_awb_data[6]<<8) | otp_awb_data[7];
    otp_awb.golden_bg_ratio = (otp_awb_data[8]<<8) | otp_awb_data[9];
    IMX214_RG_Ratio_Typical = otp_awb.golden_rg_ratio;
    IMX214_BG_Ratio_Typical = otp_awb.golden_bg_ratio;

    if(
       ((IMX214_RG_Ratio_Typical * 7) < (otp_awb.rg_ratio * 10)) &&
       ((IMX214_RG_Ratio_Typical * 13) > (otp_awb.rg_ratio * 10)) &&
       ((IMX214_BG_Ratio_Typical * 7) < (otp_awb.bg_ratio * 10)) &&
       ((IMX214_BG_Ratio_Typical * 13) > (otp_awb.bg_ratio * 10))
        )
    {
        pr_err("rg_ratio=0x%x bg_ratio=0x%x golden_rg_ratio=0x%x golden_bg_ratio=0x%x\n",
                    otp_awb.rg_ratio, otp_awb.bg_ratio, otp_awb.golden_rg_ratio, otp_awb.golden_bg_ratio);

        if( otp_awb.bg_ratio < IMX214_BG_Ratio_Typical ){
            if( otp_awb.rg_ratio < IMX214_RG_Ratio_Typical){
                //current_opt.bg_ratio < IMX214_BG_Ratio_Typical && cuttent_otp.rg_ratio < IMX214_RG_Ratio_Typical
                G_gain = IMX214_GAIN_DEFAULT;
                B_gain = IMX214_GAIN_DEFAULT * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
                R_gain = IMX214_GAIN_DEFAULT * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
            }else{
                //current_otp.bg_ratio < IMX214_BG_Ratio_Typical &&current_otp.rg_ratio >= IMX214_RG_Ratio_Typical
                R_gain = IMX214_GAIN_DEFAULT;
                G_gain = IMX214_GAIN_DEFAULT * otp_awb.rg_ratio / IMX214_RG_Ratio_Typical;
                B_gain = G_gain * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
            }
        }
        else{
            if(otp_awb.rg_ratio < IMX214_RG_Ratio_Typical){
                //current_otp.bg_ratio >= IMX214_BG_Ratio_Typical && current_otp.rg_ratio < IMX214_RG_Ratio_Typical
                B_gain = IMX214_GAIN_DEFAULT;
                G_gain = IMX214_GAIN_DEFAULT * otp_awb.bg_ratio / IMX214_BG_Ratio_Typical;
                R_gain = G_gain * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
            }else{
                //current_otp.bg_ratio >= IMX214_BG_Ratio_Typical && current_otp.rg_ratio >= IMX214_RG_Ratio_Typical
                GB_gain = IMX214_GAIN_DEFAULT * otp_awb.bg_ratio / IMX214_BG_Ratio_Typical;
                GR_gain = IMX214_GAIN_DEFAULT * otp_awb.rg_ratio / IMX214_RG_Ratio_Typical;

                if(GB_gain > GR_gain){
                    B_gain = IMX214_GAIN_DEFAULT;
                    G_gain = GB_gain;
                    R_gain = G_gain * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
                }else{
                    R_gain = IMX214_GAIN_DEFAULT;
                    G_gain = GR_gain;
                    B_gain = G_gain * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
                }
            }
        }

        imx214_update_awb_gain(s_ctrl,R_gain, G_gain, B_gain);
    }

    return 0;
}

static int imx214_8916_otp_once;
static int imx214_8916_get_otp(struct msm_sensor_ctrl_t * s_ctrl)
{
    uint32_t i = 0, page_index = 0;
    static uint32_t R_gain, B_gain, G_gain;
    uint32_t GR_gain, GB_gain;
    uint8_t flag[2];
    uint8_t otp_awb_data[12];
    struct IMX214MIPI_otp_struct otp_awb;
    uint32_t IMX214_RG_Ratio_Typical=0x00;
    uint32_t IMX214_BG_Ratio_Typical=0x00;

	if (0 == imx214_8916_otp_once) {
	    pr_err("%s,Start to read OTP\n",__func__);
	    //check otp awb flag
	    for( i = 0; i < 3; i++ ){
	        imx214semco_read_otp(s_ctrl,i, IMX214OTP_WB_FLAG_ADD, flag, 1);//check page 0 1 2
	        if ( 0x40 == flag[0]){
	            page_index = i;
	            pr_err("[JRD_CAM][IMX214OTP]find otp awb page=%d\n", page_index);
	            break;
	        }
	    }

	    if ( i == 3){
	        printk("[JRD_CAM][IMX214OTP]fail ot check otp awb page error\n");
	        return -1;
	    }

	    //pr_err("check otp awb successfully page\n");
	    //read otp awb data
	    imx214semco_read_otp(s_ctrl,page_index, IMX214OTP_WB_DATA_ADD, otp_awb_data, 12);

	    for (i = 0; i < 12; i++){
	        pr_err("otp awb data[0x%x]=0x%x\n",IMX214OTP_WB_DATA_ADD + i, otp_awb_data[i]);
	    }

	    otp_awb.rg_ratio = (otp_awb_data[0]<<8) | otp_awb_data[1];
	    otp_awb.bg_ratio = (otp_awb_data[2]<<8) | otp_awb_data[3];
	    otp_awb.golden_rg_ratio = (otp_awb_data[6]<<8) | otp_awb_data[7];
	    otp_awb.golden_bg_ratio = (otp_awb_data[8]<<8) | otp_awb_data[9];
	    IMX214_RG_Ratio_Typical = otp_awb.golden_rg_ratio;
	    IMX214_BG_Ratio_Typical = otp_awb.golden_bg_ratio;

	    if(
	       ((IMX214_RG_Ratio_Typical * 7) < (otp_awb.rg_ratio * 10)) &&
	       ((IMX214_RG_Ratio_Typical * 13) > (otp_awb.rg_ratio * 10)) &&
	       ((IMX214_BG_Ratio_Typical * 7) < (otp_awb.bg_ratio * 10)) &&
	       ((IMX214_BG_Ratio_Typical * 13) > (otp_awb.bg_ratio * 10))
	        )
	    {
	        pr_err("rg_ratio=0x%x bg_ratio=0x%x golden_rg_ratio=0x%x golden_bg_ratio=0x%x\n",
	                    otp_awb.rg_ratio, otp_awb.bg_ratio, otp_awb.golden_rg_ratio, otp_awb.golden_bg_ratio);

	        if( otp_awb.bg_ratio < IMX214_BG_Ratio_Typical ){
	            if( otp_awb.rg_ratio < IMX214_RG_Ratio_Typical){
	                //current_opt.bg_ratio < IMX214_BG_Ratio_Typical && cuttent_otp.rg_ratio < IMX214_RG_Ratio_Typical
	                G_gain = IMX214_GAIN_DEFAULT;
	                B_gain = IMX214_GAIN_DEFAULT * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
	                R_gain = IMX214_GAIN_DEFAULT * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
	            }else{
	                //current_otp.bg_ratio < IMX214_BG_Ratio_Typical &&current_otp.rg_ratio >= IMX214_RG_Ratio_Typical
	                R_gain = IMX214_GAIN_DEFAULT;
	                G_gain = IMX214_GAIN_DEFAULT * otp_awb.rg_ratio / IMX214_RG_Ratio_Typical;
	                B_gain = G_gain * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
	            }
	        }
	        else{
	            if(otp_awb.rg_ratio < IMX214_RG_Ratio_Typical){
	                //current_otp.bg_ratio >= IMX214_BG_Ratio_Typical && current_otp.rg_ratio < IMX214_RG_Ratio_Typical
	                B_gain = IMX214_GAIN_DEFAULT;
	                G_gain = IMX214_GAIN_DEFAULT * otp_awb.bg_ratio / IMX214_BG_Ratio_Typical;
	                R_gain = G_gain * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
	            }else{
	                //current_otp.bg_ratio >= IMX214_BG_Ratio_Typical && current_otp.rg_ratio >= IMX214_RG_Ratio_Typical
	                GB_gain = IMX214_GAIN_DEFAULT * otp_awb.bg_ratio / IMX214_BG_Ratio_Typical;
	                GR_gain = IMX214_GAIN_DEFAULT * otp_awb.rg_ratio / IMX214_RG_Ratio_Typical;

	                if(GB_gain > GR_gain){
	                    B_gain = IMX214_GAIN_DEFAULT;
	                    G_gain = GB_gain;
	                    R_gain = G_gain * IMX214_RG_Ratio_Typical / otp_awb.rg_ratio;
	                }else{
	                    R_gain = IMX214_GAIN_DEFAULT;
	                    G_gain = GR_gain;
	                    B_gain = G_gain * IMX214_BG_Ratio_Typical / otp_awb.bg_ratio;
	                }
	            }
	        }
		imx214_8916_otp_once = 1;
	    }
	}

	if (1 == imx214_8916_otp_once)
	        imx214_update_awb_gain(s_ctrl,R_gain, G_gain, B_gain);
    return 0;
}


#endif
