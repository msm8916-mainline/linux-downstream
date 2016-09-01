/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
*
* ------------------------------------------------------------------
*/

#ifndef OV13850_idolc_TCT_OTP
#define OV13850_idolc_TCT_OTP
#include "msm_camera_i2c.h"

//Debug log
#define OV13850_idolc_TCT_OTP_DEBUG_ON 0

#define OV13850_idolc_RG_Ratio_Typical  612
#define OV13850_idolc_BG_Ratio_Typical  566

static struct msm_camera_i2c_client *OV13850_idolc_client;

struct OV13850_idolc_otp_struct {
    int module_integrator_id;
    int lens_id;
    int production_year;
    int production_month;
    int production_day;
    int rg_ratio;
    int bg_ratio;
    int rg_gold;
    int bg_gold;
    int R_gain;
    int B_gain;
    int G_gain;
    uint16_t flag;
    uint8_t lenc[186];
    uint8_t lenc_out[360];
    uint16_t checksumLSC;
    uint16_t checksumOTP;
    uint16_t checksumTotal;
};

static struct OV13850_idolc_otp_struct ov13850_otp;
static uint16_t g_checksumOTP_exist = 255;
static int g_gain_checksum = 255;

static int32_t OV13850_idolc_write_i2c(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
    if (!OV13850_idolc_client) pr_err("FFFF OTP: OV13850 client null\n");
    else
    {
        rc = OV13850_idolc_client->i2c_func_tbl->i2c_write(OV13850_idolc_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) pr_err("FFFF OTP: write error\n");
    }
    return rc;
}
static int16_t OV13850_idolc_read_i2c(uint32_t addr)
{
    uint16_t *data;
    uint16_t temp = 0;
    int32_t rc = -EFAULT;
    data = &temp;
    if (!OV13850_idolc_client) pr_err("FFFF OTP: OV13850 null\n");
    else
    {
        rc = OV13850_idolc_client->i2c_func_tbl->i2c_read(OV13850_idolc_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) pr_err("FFFF OTP: read error\n");
    }
    return temp;
}

// index: index of otp group. (1, 2, 3)
// return:	0, group index is empty
//		1, group index has invalid data
//		2, group index has valid data
int OV13850_idolc_check_otp_wb(int index)
{
    int flag;
    //select group
    flag = OV13850_idolc_read_i2c(0x7220);
    pr_err("SP_OTP,%s,OTP: flag = %d \n", __func__, flag);
    if (index == 0)
    {
        flag = (flag >> 6) & 0x03;
    }
    else if (index == 1)
    {
        flag = (flag >> 4) & 0x03;
    }

    if (flag == 0x00)
    {
        return 0;
    }
    else if (flag & 0x01)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of OV13850_idolc_otp_struct
// return:0,
int OV13850_idolc_read_otp_wb(int index)
{
    int i;
    int start_addr, end_addr;
    char buffer[7] = { 0 };
    //set 0x5002[1] to “0”

    if (index == 0)
    {
        start_addr = 0x7228;
        end_addr = 0x722E;
    }
    else if (index == 1)
    {
        start_addr = 0x7236;
        end_addr = 0x723C;
    }

    for (i = 0; i <= end_addr - start_addr; i++)
    {
        buffer[i] = OV13850_idolc_read_i2c(start_addr + i);
        pr_err("%s,buffer[%d] = 0x%x\n", __func__, i, buffer[i]);
    }

    ov13850_otp.rg_ratio = (buffer[0] << 2) + ((buffer[2] >> 6) & 0x03);
    ov13850_otp.bg_ratio = (buffer[1] << 2) + ((buffer[2] >> 4) & 0x03);
    pr_err("OTP:rg_ratio: %d   bg_ratio: %d \n", ov13850_otp.rg_ratio, ov13850_otp.bg_ratio);

    ov13850_otp.rg_gold = (buffer[3] << 2) + ((buffer[5] >> 6) & 0x03);
    ov13850_otp.bg_gold = (buffer[4] << 2) + ((buffer[5] >> 4) & 0x03);
    pr_err("OTP:rg_gld: %d   bg_gld: %d \n", ov13850_otp.rg_gold, ov13850_otp.bg_gold);

    // clear otp buffer
    for (i = start_addr; i <= end_addr; i++)
    {
        OV13850_idolc_write_i2c(i, 0x00);
    }

    return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int OV13850_idolc_update_awb_gain(int R_gain, int G_gain, int B_gain)
{
    int i = 0;
    OV13850_idolc_write_i2c(0x3d84, 0x40);
    pr_err("%s,R_gain = %d,G_gain = %d,B_gain = %d\n", __func__, R_gain, G_gain, B_gain);
    if (R_gain > 0x400)
    {
        OV13850_idolc_write_i2c(0x5056, R_gain >> 8);
        OV13850_idolc_write_i2c(0x5057, R_gain & 0x00ff);
    }
    if (G_gain > 0x400)
    {
        OV13850_idolc_write_i2c(0x5058, G_gain >> 8);
        OV13850_idolc_write_i2c(0x5059, G_gain & 0x00ff);
    }

    if (B_gain > 0x400)
    {
        OV13850_idolc_write_i2c(0x505A, B_gain >> 8);
        OV13850_idolc_write_i2c(0x505B, B_gain & 0x00ff);
    }
    OV13850_idolc_write_i2c(0x3d84, 0xC0);

    for (i = 0; i < 6; i++)
    {
        pr_err("%s,I2C = %d\n", __func__, OV13850_idolc_read_i2c(0x5056 + i));
    }

    return 0;
}

bool checkAWB(void)
{
    if (((ov13850_otp.R_gain + ov13850_otp.B_gain + ov13850_otp.G_gain) % 255) == g_gain_checksum)
    {
        return true;
    }

    return false;
}

bool checkLSC(void)
{
    uint32_t i = 0;
    uint32_t checksumOTP_exist = 0;
    for (i = 0; i < 10; i++)
    {
        checksumOTP_exist += ov13850_otp.lenc_out[i];
        checksumOTP_exist %= 255;
    }

    pr_err("%s,checksumOTP = %d,otp.checksumOTP = %d\n", __func__, checksumOTP_exist, g_checksumOTP_exist);

    if (checksumOTP_exist == g_checksumOTP_exist)
    {
        return true;
    }

    return false;
}

// call this function after OV13850_idolc initialization
// return value: 0 update success
//		 1, no OTP
int OV13850_idolc_update_otp_wb(void)
{
    int i;
    int otp_index;
    int nR_G_gain, nB_G_gain, nG_G_gain;
    int nBase_gain;

    if (!checkAWB())
    {
        // R/G and B/G of current camera module is read out from sensor OTP
        // check first OTP with valid data
        for (i = 0; i < 2; i++)
        {
            if (OV13850_idolc_check_otp_wb(i) == 1)
            {
                otp_index = i;
                break;
            }
        }

        if (i >= 2)
        {
            // no valid wb OTP data
            return 1;
        }
        pr_err("OTP:index = 0x%x\n", otp_index);
        OV13850_idolc_read_otp_wb(otp_index);

        //calculate G gain

        pr_err("OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", ov13850_otp.rg_gold, ov13850_otp.rg_ratio, ov13850_otp.bg_gold, ov13850_otp.bg_ratio);

        if (!ov13850_otp.rg_ratio || !ov13850_otp.bg_ratio)
        {
            pr_err("OTP:rg_ratio = 0x%x,bg_ratio = 0x%x\n", ov13850_otp.rg_ratio, ov13850_otp.bg_ratio);
            return 1;
        }

        if (!ov13850_otp.rg_gold)
        {
            ov13850_otp.rg_gold = 521;
        }
        if (!ov13850_otp.bg_gold)
        {
            ov13850_otp.bg_gold = 566;
        }

        nR_G_gain = (ov13850_otp.rg_gold * 1000) / ov13850_otp.rg_ratio;
        nB_G_gain = (ov13850_otp.bg_gold * 1000) / ov13850_otp.bg_ratio;
        nG_G_gain = 1000;

        if (nR_G_gain < 1000 && nB_G_gain < 1000)
        {
            if (nR_G_gain < nB_G_gain) nBase_gain = nR_G_gain;
            else nBase_gain = nB_G_gain;
        }
        else
        {
            nBase_gain = nG_G_gain;
        }

        if (!nBase_gain)
        {
            pr_err("OTP:nBase_gain = 0x%x\n", nBase_gain);
            return 1;
        }
        ov13850_otp.R_gain = 0x400 * nR_G_gain / (nBase_gain);
        ov13850_otp.B_gain = 0x400 * nB_G_gain / (nBase_gain);
        ov13850_otp.G_gain = 0x400 * nG_G_gain / (nBase_gain);

        g_gain_checksum = (ov13850_otp.R_gain + ov13850_otp.B_gain + ov13850_otp.G_gain) % 255;
    }
    if (checkAWB())
    {
        OV13850_idolc_update_awb_gain(ov13850_otp.R_gain, ov13850_otp.G_gain, ov13850_otp.B_gain);
    }
    return 0;
}


/***********Decode LENC Para Process Start*********************/
void LumaDecoder(uint8_t *pData, uint8_t *pPara)
{
    uint32_t Offset, Bit, Option;
    uint32_t i, k;
    uint8_t pCenter[16], pMiddle[32], pCorner[72];
    Offset = pData[0];
    Bit = pData[1] >> 4;
    Option = pData[1] & 0xf;
    if (Bit <= 5)
    {
        for (i = 0,k = 2; i < 120; i += 8, k += 5)
        {

            pPara[i] = pData[k] >> 3;
            // 7~3 (byte0)
            pPara[i + 1] = ((pData[k] & 0x7) << 2) | (pData[k + 1] >> 6); // 2~0 (byte0) and 7~6 (byte1)
            pPara[i + 2] = (pData[k + 1] & 0x3e) >> 1; // 5~1 (byte1)
            pPara[i + 3] = ((pData[k + 1] & 0x1) << 4) | (pData[k + 2] >> 4); // 0 (byte1) and 7~4 (byte2)
            pPara[i + 4] = ((pData[k + 2] & 0xf) << 1) | (pData[k + 3] >> 7); // 3~0 (byte2) and 7 (byte3)
            pPara[i + 5] = (pData[k + 3] & 0x7c) >> 2; // 6~2 (byte3)
            pPara[i + 6] = ((pData[k + 3] & 0x3) << 3) | (pData[k + 4] >> 5);
            pPara[i + 7] = pData[k + 4] & 0x1f;  // 1~0 (byte3) and 7~5 (byte4)
        }
    }
    else
    {
        for (i = 0,k = 2; i < 48; i += 8, k += 5)
        {
            // 4~0 (byte4)

            pPara[i] = pData[k] >> 3;

            pPara[i + 1] = ((pData[k] & 0x7) << 2) | (pData[k + 1] >> 6); // 7~3 (byte0)
            pPara[i + 2] = (pData[k + 1] & 0x3e) >> 1;
            pPara[i + 3] = ((pData[k + 1] & 0x1) << 4) | (pData[k + 2] >> 4); // 5~1 (byte1)
            pPara[i + 4] = ((pData[k + 2] & 0xf) << 1) | (pData[k + 3] >> 7);
            pPara[i + 5] = (pData[k + 3] & 0x7c) >> 2; // 6~2 (byte3)
            pPara[i + 6] = ((pData[k + 3] & 0x3) << 3) | (pData[k + 4] >> 5); // 1~0 (byte3) and 7~5
            pPara[i + 7] = pData[k + 4] & 0x1f;
        }
        // 4~0 (byte4)
        for (i = 48,k = 32; i < 120; i += 4, k += 3)
        {
            pPara[i] = pData[k] >> 2;
            pPara[i + 1] = ((pData[k] & 0x3) << 4) | (pData[k + 1] >> 4);
            pPara[i + 2] = ((pData[k + 1] & 0xf) << 2) | (pData[k + 2] >> 6); //3~0 (byte1) and
            pPara[i + 3] = pData[k + 2] & 0x3f; // 5~0 (byte2)
        }

        memcpy(pCenter, pPara, 16);
        memcpy(pMiddle, pPara + 16, 32);
        memcpy(pCorner, pPara + 48, 72);
        for (i = 0; i < 32; i++)
        {
            pMiddle[i] <<= (Bit - 6);
        }

        for (i = 0; i < 72; i++)
        {
            pCorner[i] <<= (Bit - 6);
        }
        if (Option == 0) // 10x12
        {

            memcpy(pPara, pCorner, 26);
            memcpy(pPara + 26, pMiddle, 8);
            memcpy(pPara + 34, pCorner + 26, 4);
            memcpy(pPara + 38, pMiddle + 8, 2);
            memcpy(pPara + 40,  pCenter, 4);
            memcpy(pPara + 44, pMiddle + 10, 2);
            memcpy(pPara + 46, pCorner + 30, 4);
            memcpy(pPara + 50, pMiddle + 12, 2);
            memcpy(pPara + 52,  pCenter + 4, 4);
            memcpy(pPara + 56, pMiddle + 14, 2);
            memcpy(pPara + 58, pCorner + 34, 4);
            memcpy(pPara + 62, pMiddle + 16, 2);
            memcpy(pPara + 64,  pCenter + 8, 4);
            memcpy(pPara + 68, pMiddle + 18, 2);
            memcpy(pPara + 70, pCorner + 38, 4);
            memcpy(pPara + 74, pMiddle + 20, 2);
            memcpy(pPara + 76,  pCenter + 12, 4);
            memcpy(pPara + 80, pMiddle + 22, 2);
            memcpy(pPara + 82, pCorner + 42, 4);
            memcpy(pPara + 86, pMiddle + 24, 8);
            memcpy(pPara + 94,  pCorner + 46, 26);

        }
        else // 12x10
        {
            memcpy(pPara,  pCorner, 22);

            memcpy(pPara + 22, pMiddle, 6);
            memcpy(pPara + 28, pCorner + 22, 4);
            memcpy(pPara + 32, pMiddle + 6, 6);
            memcpy(pPara + 38, pCorner + 26, 4);
            memcpy(pPara + 42, pMiddle + 12, 1);
            memcpy(pPara + 43, pCenter, 4);
            memcpy(pPara + 47, pMiddle + 13, 1);
            memcpy(pPara + 48, pCorner + 30, 4);
            memcpy(pPara + 52, pMiddle + 14, 1);
            memcpy(pPara + 53, pCenter + 4, 4);
            memcpy(pPara + 57, pMiddle + 15, 1);
            memcpy(pPara + 58, pCorner + 34, 4);
            memcpy(pPara + 62, pMiddle + 16, 1);
            memcpy(pPara + 63, pCenter + 8, 4);
            memcpy(pPara + 67, pMiddle + 17, 1);
            memcpy(pPara + 68, pCorner + 38, 4);
            memcpy(pPara + 72, pMiddle + 18, 1);
            memcpy(pPara + 73, pCenter + 12, 4);
            memcpy(pPara + 77, pMiddle + 19, 1);
            memcpy(pPara + 78, pCorner + 42, 4);
            memcpy(pPara + 82, pMiddle + 20, 6);
            memcpy(pPara + 88, pCorner + 46, 4);
            memcpy(pPara + 92, pMiddle + 26, 6);
            memcpy(pPara + 98, pCorner + 50, 22);
        }
    }
    for (i = 0;  i < 120; i++)
    {
        pPara[i] += Offset;
    }
}

void ColorDecoder(uint8_t *pData, uint8_t *pPara)
{
    uint32_t Offset, Bit, Option;
    uint32_t i, k;
    uint8_t pBase[30];
    Bit = pData[1] >> 7;
    Offset = pData[0];

    Option = (pData[1] & 0x40) >> 6;
    pPara[0] = (pData[1] & 0x3e) >> 1; // 5~1 (byte1)
    pPara[1] = ((pData[1] & 0x1) << 4) | (pData[2] >> 4); // 0 (byte1) and 7~4 (byte2)
    pPara[2] = ((pData[2] & 0xf) << 1) | (pData[3] >> 7); // 3~0 (byte2) and 7 (byte3)
    pPara[3] = (pData[3] & 0x7c) >> 2;
    pPara[4] = ((pData[3] & 0x3) << 3) | (pData[4] >> 5); // 1~0 (byte3) and 7~5 (byte4)
    pPara[5] = pData[4] & 0x1f; // 4~0 (byte4)
    for (i = 6,k = 5; i < 30; i += 8, k += 5)
    {
        pPara[i] = pData[k] >> 3;
        // 7~3 (byte0)
        pPara[i + 1] = ((pData[k] & 0x7) << 2) | (pData[k + 1] >> 6); // 2~0 (byte0) and 7~6 (byte1)
        pPara[i + 2] = (pData[k + 1] & 0x3e) >> 1; // 5~1 (byte1)
        pPara[i + 3] = ((pData[k + 1] & 0x1) << 4) | (pData[k + 2] >> 4); // 0 (byte1) and 7~4 (byte2)
        pPara[i + 4] = ((pData[k + 2] & 0xf) << 1) | (pData[k + 3] >> 7); // 3~0 (byte2) and 7 (byte3)
        pPara[i + 5] = (pData[k + 3] & 0x7c) >> 2; // 6~2 (byte3)
        pPara[i + 6] = ((pData[k + 3] & 0x3) << 3) | (pData[k + 4] >> 5); // 1~0 (byte3) and 7~5 (byte4)

        pPara[i + 7] = pData[k + 4] & 0x1f;
    }
    // 4~0 (byte4)
    memcpy(pBase, pPara, 30);
    for (i = 0,k = 20; i < 120; i += 4, k++)
    {
        pPara[i] = pData[k] >> 6;
        pPara[i + 1] = (pData[k] & 0x30) >> 4;
        pPara[i + 2] = (pData[k] & 0xc) >> 2;
        pPara[i + 3] = pData[k] & 0x3;
    }
    if (Option == 0) // 10x12
    {
        for (i = 0; i < 5; i++)
        {
            for (k = 0; k < 6; k++)
            {
                pPara[i * 24 + k * 2] += pBase[i * 6 + k];
                pPara[i * 24 + k * 2 + 1] += pBase[i * 6 + k];
                pPara[i * 24 + k * 2 + 12] += pBase[i * 6 + k];
                pPara[i * 24 + k * 2 + 13] += pBase[i * 6 + k];
            }
        }
    }
    else // 12x10
    {
        for (i = 0; i < 6; i++)
        {
            for (k = 0;  k < 5; k++)
            {

                pPara[i * 20 + k * 2] += pBase[i * 5 + k];
                pPara[i * 20 + k * 2 + 1] += pBase[i * 5 + k];
                pPara[i * 20 + k * 2 + 10] += pBase[i * 5 + k];
                pPara[i * 20 + k * 2 + 11] += pBase[i * 5 + k];
            }
        }
    }
    for (i = 0; i < 120; i++)
    {
        pPara[i] = (pPara[i] << Bit) + Offset;
    }
}

void OV13850_R2A_LENC_Decoder(uint8_t *pData, uint8_t *pPara)
{
    LumaDecoder(pData, pPara);
    ColorDecoder(pData + 86, pPara + 120);
    ColorDecoder(pData + 136, pPara + 240);
}

uint8_t Read_ov13850_LSC_OTP(void)
{
    uint32_t otp_flag, addr, i;
    uint8_t ret = 0;
    ov13850_otp.flag = 0x00; //
// OTP Lenc Calibration
    otp_flag = OV13850_idolc_read_i2c(0x723D);

    addr = 0;
    if ((otp_flag & 0xc0) == 0x40)
    {
        addr = 0x723E; // base address of Lenc Calibration group 1
    }
    else if ((otp_flag & 0x30) == 0x10) // base address of Lenc Calibration group 2
    {
        addr = 0x72FB;
    }

    pr_err("%s:otp_flag = 0x%x,addr = 0x%x\n", __func__, otp_flag, addr);

    if (addr != 0)
    {
        int checksumLSC = 0;
        for (i = 0; i < 186; i++)
        {
            ov13850_otp.lenc[i] = OV13850_idolc_read_i2c(addr + i);
            checksumLSC += ov13850_otp.lenc[i];
            checksumLSC %= 255;
        }

        checksumLSC++;
        ov13850_otp.flag = (ov13850_otp.flag | 0x10);
        ov13850_otp.checksumLSC = OV13850_idolc_read_i2c((addr + 186));
        ov13850_otp.checksumOTP  = OV13850_idolc_read_i2c((addr  + 187));
        ov13850_otp.checksumTotal = OV13850_idolc_read_i2c((addr + 188));

        pr_err("%s,checksumLsc = %d,ov13850_otp.checksumLSC = %d\n", __func__, checksumLSC, ov13850_otp.checksumLSC);
        if (checksumLSC == ov13850_otp.checksumLSC)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        for (i = 0; i < 186; i++)
        {
            ov13850_otp.lenc[i] = 0;
        }
        ret = 1;
        pr_err("Fail to read_otp_lenc_R2A\n");
    }
    return ret;
}

int Apply_ov13850_LSC_OTP(void)
{
    uint32_t i;
    uint8_t ret = 0;
    int checksumOTP = 0;

// apply OTP Lenc Calibration

    if (!checkLSC())
    {
        if (Read_ov13850_LSC_OTP())
        {
            return 1;
        }

        OV13850_R2A_LENC_Decoder(ov13850_otp.lenc, ov13850_otp.lenc_out);

        for (i = 0; i < 360; i++)
        {
            checksumOTP += ov13850_otp.lenc_out[i];
            checksumOTP %= 255;

            if (i < 10)
            {
                g_checksumOTP_exist += ov13850_otp.lenc_out[i];
                g_checksumOTP_exist %= 255;
            }
        }

        checksumOTP++;

        pr_err("%s,checksumOTP = %d,otp.checksumOTP = %d\n", __func__, checksumOTP, ov13850_otp.checksumOTP);

        if (checksumOTP != ov13850_otp.checksumOTP)
        {
            g_checksumOTP_exist = 255;
            return 1;
        }
    }

    if (checkLSC())
    {
        OV13850_idolc_write_i2c(0x3d84, 0x40);
        for (i = 0; i < 360; i++)
        {
            OV13850_idolc_write_i2c(0x5200 + i, ov13850_otp.lenc_out[i]);
        }
        OV13850_idolc_write_i2c(0x3d84, 0xC0);

        for (i = 0; i < 10; i++)
        {
            pr_err("%s,buffer = %d,I2C = %d\n", __func__, ov13850_otp.lenc_out[i], OV13850_idolc_read_i2c(0x5200 + i));
        }
    }
    else
    {
        ret = 1;
    }

    return ret;
}

int OV13850_idolc_update_otp(struct msm_camera_i2c_client *i2c_client)
{
    //set 0x5002[1] to “0”
    int temp1;
    OV13850_idolc_client = i2c_client;
    OV13850_idolc_write_i2c(0x0100, 0x01);
    temp1 = OV13850_idolc_read_i2c(0x5002);
    OV13850_idolc_write_i2c(0x5002, (0x00 & 0x08) | (temp1 & (~0x08)));
    OV13850_idolc_write_i2c(0x3d84, 0xC0);

    //partial mode OTP write start address
    OV13850_idolc_write_i2c(0x3d88, 0x72);
    OV13850_idolc_write_i2c(0x3d89, 0x20);
    // partial mode OTP write end address
    OV13850_idolc_write_i2c(0x3d8A, 0x73 & 0xff);
    OV13850_idolc_write_i2c(0x3d8B, 0xBE & 0xff);
    // read otp into buffer
    OV13850_idolc_write_i2c(0x3d81, 0x01);
    mdelay(5);

    OV13850_idolc_update_otp_wb();
    Apply_ov13850_LSC_OTP();

    // clear otp buffer
    OV13850_idolc_write_i2c(0x7026, 0x00);

    //set 0x5002[1] to “1”
    temp1 = OV13850_idolc_read_i2c(0x5002);
    OV13850_idolc_write_i2c(0x5002, (0x02 & 0x08) | (temp1 & (~0x08)));
    OV13850_idolc_write_i2c(0x0100, 0x00);
    return 1;
}

#endif

