/*         Modify History For This Module
* When           Who             What,Where,Why
* ------------------------------------------------------------------
* 14/03/07      Hu Jin       add OTP for s5k5e2_rio5_ctcc(Ewelly)
* 14/05/06      YU Qijiang   add OTP for s5k5e2_pop10_cmcc
* 14/05/26      Hu Jin       /4 for 1st format version.
* ------------------------------------------------------------------
*/

#ifndef OV5670_POP10_CMCC_V4L2_TCT_OTP
#define OV5670_POP10_CMCC_V4L2_TCT_OTP
#include "msm_camera_i2c.h"

//Debug log
#define OV5670_POP10_CMCC_OTP_DEBUG_ON 0


#define RG_Ratio_Typical  304//202//0.792760725 * 256;// sync with GS
#define BG_Ratio_Typical  300//161//0.627300368 * 256;// sync with GS

static struct msm_camera_i2c_client *g_client;

struct otp_struct {
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
};


static int32_t OV5670_write_i2c(uint32_t addr, uint16_t data)
{
    int32_t rc = -EFAULT;
	if (!g_client)
        printk("FFFF OTP: OV8865 client null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_write(g_client,addr, data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("FFFF OTP: write error\n");
    }
    return rc;
}
static int16_t OV5670_read_i2c(uint32_t addr)
{
	uint16_t *data;
	uint16_t temp=0;
    int32_t rc = -EFAULT;
	data=&temp;
    if (!g_client)
        printk("FFFF OTP: OV8865 null\n");
    else{
        rc = g_client->i2c_func_tbl->i2c_read(g_client,addr,data, MSM_CAMERA_I2C_BYTE_DATA);
		if(rc < 0)
            printk("FFFF OTP: read error\n");
    }
    return temp;
}


// index: index of otp group. (1, 2, 3)
// return:0, group index is empty
//1, group index has invalid data
//2, group index has valid data

int check_otp_info(int index)
{
int flag;
//set 0x5002[1] to “0”
int temp1;
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
OV5670_write_i2c(0x3d84, 0xC0);
//partial mode OTP write start address
OV5670_write_i2c(0x3d88, 0x70);
OV5670_write_i2c(0x3d89, 0x10);
// partial mode OTP write end address
OV5670_write_i2c(0x3d8A, 0x70);
OV5670_write_i2c(0x3d8B, 0x10);
// read otp into buffer
OV5670_write_i2c(0x3d81, 0x01);
mdelay(5);
flag = OV5670_read_i2c(0x7010);
//select group
if (index == 1)
{
	flag = (flag>>6) & 0x03;
}
else if (index == 2)
{
	flag = (flag>>4) & 0x03;
}
else if (index ==3)
{
	flag = (flag>>2) & 0x03;
}
// clear otp buffer
OV5670_write_i2c(0x7010, 0x00);
//set 0x5002[1] to “1”
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

if (flag == 0x00) {
	return 0;
}
else if (flag & 0x02) {
	return 1;
}
else {
	return 2;
}

}

// index: index of otp group. (1, 2, 3)
// return:	0, group index is empty
//		1, group index has invalid data
//		2, group index has valid data
int check_otp_wb(int index)
{
int flag;
//set 0x5002[1] to “0”
int temp1;
OV5670_write_i2c(0x0100, 0x01);
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
OV5670_write_i2c(0x3d84, 0xC0);
//partial mode OTP write start address
OV5670_write_i2c(0x3d88, 0x70);
OV5670_write_i2c(0x3d89, 0x26);
// partial mode OTP write end address
OV5670_write_i2c(0x3d8A, 0x70);
OV5670_write_i2c(0x3d8B, 0x26);
// read otp into buffer
OV5670_write_i2c(0x3d81, 0x01);
mdelay(5);

//select group
flag = OV5670_read_i2c(0x7026);
printk("OTP: flag = %d \n",flag);
if (index == 1)
{
	flag = (flag>>6) & 0x03;
}
else if (index == 2)
{
	flag = (flag>>4) & 0x03;
}
else if (index == 3)
{
	flag = (flag>>2) & 0x03;
}
// clear otp buffer
OV5670_write_i2c( 0x7026, 0x00);

//set 0x5002[1] to “1”
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

if (flag == 0x00) {
	return 0;
}
else if (flag & 0x02) {
	return 1;
}
else {
	return 2;
}
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_info(int index, struct otp_struct *otp_ptr)
{
int i;
int start_addr, end_addr;

//set 0x5002[1] to “0”
int temp1;
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));

if (index == 1) {
	start_addr = 0x7011;
	end_addr = 0x7015;
}
else if (index == 2) {
	start_addr = 0x7016;
	end_addr = 0x701a;
}
else if (index == 3) {
	start_addr = 0x701b;
	end_addr = 0x701f;
}



OV5670_write_i2c(0x3d84, 0xC0);
//partial mode OTP write start address
OV5670_write_i2c(0x3d88, (start_addr >> 8) & 0xff);
OV5670_write_i2c(0x3d89, start_addr & 0xff);
// partial mode OTP write end address
OV5670_write_i2c(0x3d8A, (end_addr >> 8) & 0xff);
OV5670_write_i2c(0x3d8B, end_addr & 0xff);

// read otp into buffer
OV5670_write_i2c(0x3d81, 0x01);

mdelay(5);
(*otp_ptr).module_integrator_id = OV5670_read_i2c(start_addr);
(*otp_ptr).lens_id = OV5670_read_i2c(start_addr + 1);
(*otp_ptr).production_year = OV5670_read_i2c(start_addr + 2);
(*otp_ptr).production_month = OV5670_read_i2c(start_addr + 3);
(*otp_ptr).production_day = OV5670_read_i2c(start_addr + 4);

// clear otp buffer
for (i=start_addr; i<=end_addr; i++) {
	OV5670_write_i2c(i, 0x00);
}

//set 0x5002[1] to “1”
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));
return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return:0,
int read_otp_wb(int index, struct otp_struct *otp_ptr)
{
int i;
int temp;
int start_addr, end_addr;
int rg_gld,bg_gld;
//set 0x5002[1] to “0”
int temp1;//,temp2,temp3,temp4,temp5,temp6;
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));

if (index == 1) {
	start_addr = 0x7028;
	end_addr = 0x702F;
}
else if (index == 2) {
	start_addr = 0x7031;
	end_addr = 0x7038;
}
else if (index == 3) {
	start_addr = 0x703A;
	end_addr = 0x7041;
}

OV5670_write_i2c(0x3d84, 0xC0);
//partial mode OTP write start address
OV5670_write_i2c(0x3d88, (start_addr >> 8) & 0xff);
OV5670_write_i2c(0x3d89, start_addr & 0xff);
// partial mode OTP write end address
OV5670_write_i2c(0x3d8A, (end_addr >> 8) & 0xff);
OV5670_write_i2c(0x3d8B, end_addr & 0xff);


// read otp into buffer
OV5670_write_i2c(0x3d81, 0x01);

mdelay(5);

temp = OV5670_read_i2c(start_addr + 3);
(*otp_ptr).rg_ratio = (OV5670_read_i2c(start_addr)<<2) + ((temp>>6) & 0x03);
(*otp_ptr).bg_ratio = (OV5670_read_i2c(start_addr + 1)<<2) + ((temp>>4) & 0x03);

temp = OV5670_read_i2c(start_addr + 7);
rg_gld = (OV5670_read_i2c(start_addr+4)<<2) + ((temp>>6) & 0x03);
bg_gld = (OV5670_read_i2c(start_addr + 5)<<2) + ((temp>>4) & 0x03);
printk("rg_gld: %d   bg_gld: %d \n",rg_gld,bg_gld);


// clear otp buffer
for (i=start_addr; i<=end_addr; i++) {
	OV5670_write_i2c(i, 0x00);
}
//set 0x5002[1] to “1”
temp1 = OV5670_read_i2c(0x5002);
OV5670_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));
return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
if (R_gain>0x400) {
	OV5670_write_i2c(0x5032, R_gain>>8);
	OV5670_write_i2c(0x5033, R_gain & 0x00ff);
}
if (G_gain>0x400) {
	OV5670_write_i2c(0x5034, G_gain>>8);
	OV5670_write_i2c(0x5035, G_gain & 0x00ff);
}

if (B_gain>0x400) {
	OV5670_write_i2c(0x5036, B_gain>>8);
	OV5670_write_i2c(0x5037, B_gain & 0x00ff);
}
return 0;
}

// call this function after OV5670 initialization
// return value: 0 update success
//		 1, no OTP
int update_otp_wb(struct msm_camera_i2c_client *i2c_client)
{
struct otp_struct current_otp;
int i;
int otp_index;
int temp;
int rg,bg,R_gain,B_gain,G_gain;
int nR_G_gain, nB_G_gain, nG_G_gain;
int nBase_gain;

g_client = i2c_client;
// R/G and B/G of current camera module is read out from sensor OTP
// check first OTP with valid data
for(i=1;i<=3;i++) {
	temp = check_otp_wb(i);
	if (temp == 2) {
	otp_index = i;
	break;
	}
}

if (i>3) {
	// no valid wb OTP data
	return 1;
}
printk("index = %d",otp_index);
read_otp_wb(otp_index, &current_otp);

rg = current_otp.rg_ratio ;
bg = current_otp.bg_ratio;

//calculate G gain

printk("OTP:RG_G/RG_C=[%d:%d],BG_G/BG_C=[%d:%d]\n", RG_Ratio_Typical, rg, BG_Ratio_Typical, bg);
nR_G_gain = (RG_Ratio_Typical*1000) / rg;
nB_G_gain = (BG_Ratio_Typical*1000) / bg;
nG_G_gain = 1000;

if (nR_G_gain < 1000 || nB_G_gain < 1000)
{
if (nR_G_gain < nB_G_gain)
	nBase_gain = nR_G_gain;
else
	nBase_gain = nB_G_gain;
}
else
{
	nBase_gain = nG_G_gain;
}

R_gain = 0x400 * nR_G_gain / (nBase_gain);
B_gain = 0x400 * nB_G_gain / (nBase_gain);
G_gain = 0x400 * nG_G_gain / (nBase_gain);

update_awb_gain(R_gain, G_gain, B_gain);
return 0;
}


#endif

