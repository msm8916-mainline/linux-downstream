
#define OTP_DRV_START_ADDR          0x0400
#define OTP_DRV_MID_ADDR          0x0401
#define OTP_DRV_END_ADDR			0x059E
#define OTP_DRV_LSC_REG_ADDR		0x5200
#define OTP_DRV_SIZE				415
#define OTP_DRV_LSC_SIZE            360

struct otp_struct_ov13850 {
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int light_rg;
	int light_bg;
	int lenc[OTP_DRV_LSC_SIZE];
	int VCM_start;
	int VCM_end;
	int VCM_dir;
};

