#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>

#include "tfa98xx-debug-common.h"

#include "tfa98xx-core.h"
#include "tfa98xx-regs.h"
#include "tfa_container.h"
#include "tfa_dsp.h"
#include "tfa98xx.h"

#define DEBUG

#define TFA98XX_DEBUG

#define I2C_RETRY_DELAY		5 /* ms */
#define I2C_RETRIES		5
#define I2C_ATTEMPTS_MAX  5

/* SNDRV_PCM_RATE_KNOT -> 12000, 24000 Hz, limit with constraint list */
#define TFA98XX_RATES (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define TFA98XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)
#define TFA98XX_STATUS_UP_MASK	(TFA98XX_STATUSREG_PLLS | \
				 TFA98XX_STATUSREG_CLKS | \
				 TFA98XX_STATUSREG_VDDS | \
				 TFA98XX_STATUSREG_AREFS)

extern int tfa98xx_unmute(struct tfa98xx *tfa98xx);

extern int tfa98xx_dsp_get_calibration_impedance(
			struct tfa98xx *tfa98xx, u32 *re25);

extern void tfa98xx_convert_bytes2data(int len, 
			const u8 *bytes, int *data);

extern int tfa98xx_dsp_get_param(struct tfa98xx *tfa98xx,
			u8 module_id, u8 param_id, int len, u8 *data);

extern char *tfa98xx_filename;

static struct tfa98xx *tfa98xx_priv;

static int smart_pa_switch_enable;
static int smart_pa_mode_select;

static int tfa98xx_i2c_check(void)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	int val,ret,tries = 0;

	if (!tfa98xx)
		return 0;

	mutex_lock(&tfa98xx->dsp_init_lock);
	for(tries = 0,ret = 1;(tries < I2C_ATTEMPTS_MAX && ret); tries++){
		ret = regmap_read(tfa98xx->regmap,TFA98XX_REVISIONNUMBER, &val);
		if (ret < 0)
			pr_err("%s:chip id 0x%02x ret %d \n",__func__,val,ret);
	}
	mutex_unlock(&tfa98xx->dsp_init_lock);

	return ((tries < I2C_ATTEMPTS_MAX)? 1:0);
}

static int tfa98xx_calibration_done(int valu)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;

	if ( !tfa98xx) {
		pr_err("%s: tfa98xx is NULL\n",
			__func__);
		return 0;
	}

	return !(valu<tfa98xx->para.imped_min || valu>tfa98xx->para.imped_max);

}

static int tfa98xx_check_Mtp(struct tfa98xx *tfa98xx)
{
	unsigned int mtp = 0;
	struct snd_soc_codec *codec = tfa98xx->codec;
	mtp = snd_soc_read(codec, TFA98XX_MTP);
	pr_info("%s mtp value 0x%x.\n", __func__, mtp);
	if (mtp & 2)
		return 1;
	else
		return 0;
}

int tfa98xx_check_mtp_dbg(void)
{
	return tfa98xx_check_Mtp(tfa98xx_priv);
}

static int tfa98xx_reset_Mtp(struct tfa98xx *tfa98xx)
{
	int status;
	int tries, ret;

	struct snd_soc_codec *codec = tfa98xx->codec;

	if(!tfa98xx_is_pwdn(tfa98xx)){
		tfa98xx_dsp_stop(tfa98xx);
	}
	tfaRunColdStartup(tfa98xx);
	msleep_interruptible(10);

	//tfa98xx_dsp_power_up(tfa98xx);
	//tfa98xx_unmute(tfa98xx);

	snd_soc_write(codec, 0x0b, 0x5A);
	snd_soc_write(codec, TFA98XX_MTP, 0x01);
	snd_soc_write(codec, 0x62, 0x800);

	//tfa98xx->imped_val = 0;
	tfa98xx->calibration = 0;

	pr_info("%s() wait for MTP \n", __func__);

	for (tries = 0; tries < 50; tries++) {
		msleep_interruptible(10);
		/* Check if MTP is still busy */
		status = snd_soc_read(codec, TFA98XX_STATUSREG);
		if (!( status & TFA98XX_STATUSREG_MTPB_MSK))
			break;
	}

	if (tries == 50) {
		pr_err("Wait for MTPB timedout\n");
		ret = -EINVAL;
	} else {
    	status = snd_soc_read(codec, TFA98XX_MTP);
		pr_info("%s() TFA98XX_MTP %02d \n",__func__, status);
		ret = 0;
	}

	if(!tfa98xx_is_pwdn(tfa98xx)){
		tfa98xx_dsp_stop(tfa98xx);
	}
	msleep_interruptible(10);

	//tfa98xx_dsp_stop(tfa98xx);
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;

	pr_info("%s() leave \n", __func__);

	return ret;
}

int tfa98xx_init(int force)
{	
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	u32 imped_valu = 0;
	int ret = -1;
	int retries = 3;
	
	if(!tfa98xx)
		return -EINVAL;

	msleep(20);

	if (!tfa98xx_i2c_check()){
		pr_err("%s() i2c error	\n", __func__);
		return 0;
	}

	pr_info("profile %d, vstep %d\n", tfa98xx->profile_ctl,
		 tfa98xx->vstep_ctl);

	mutex_lock(&tfa98xx->dsp_init_lock);

	if (force){
		tfa98xx_reset_Mtp(tfa98xx);
		msleep(5);
	}
	
	if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl,
			       tfa98xx->vstep_ctl))
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
	
	while(retries-- > 0 && ret != 0){
		ret = tfa98xx_dsp_get_calibration_impedance(tfa98xx, &imped_valu);
	}
	
//	tfa98xx_dsp_stop(tfa98xx);
	
	if ( !ret)
	{
		tfa98xx->calibration = tfa98xx_calibration_done(imped_valu);
		tfa98xx->imped_val = imped_valu;
		if (!tfa98xx->calibration)
			pr_err("%s:calibration impedence %02d.%02d is irrational\n",
					__func__,(int)imped_valu/100,(int)imped_valu%100);
		
	}else{
		tfa98xx->imped_val = 0;
		tfa98xx->calibration = 0;
	}
	
	if (tfa98xx->calibration)
	{
		pr_info("%s:calibration impedence %02d.%02d \n",
				__func__, (int)imped_valu/100, (int)imped_valu%100);
	}else{
		tfa98xx_reset_Mtp(tfa98xx);
		msleep(5);
		pr_err("%s:calibrate error \n", __func__);
	}

	tfa98xx_dsp_stop(tfa98xx);
	
	mutex_unlock(&tfa98xx->dsp_init_lock);

	return 0;
}

#ifdef TFA98XX_DEBUG

static struct snd_kcontrol *mi2s_clk_control;
static struct snd_kcontrol *mi2s_form_control;

#define PARAM_GET_LSMODEL  0x86 

static int find_control_set_valu(const char *ctl_name,
			int set_val, struct snd_kcontrol **tfa_control)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	struct snd_soc_codec *codec;
	struct snd_kcontrol *tmp_control;
	struct snd_ctl_elem_value control;

	if (!tfa98xx || !tfa98xx->codec) {
		pr_err("%s: tfa98xx is NULL\n",
			__func__);
		return -EINVAL;
	}
	
	if (!ctl_name) {
		pr_err("%s: control name is NULL\n",
			__func__);
		return -EINVAL;
	}

	tmp_control = *tfa_control;
	codec = tfa98xx->codec;

	if (!tmp_control) {
		list_for_each_entry(tmp_control,
				&(codec->card->snd_card->controls), list)
		if (strstr(tmp_control->id.name, ctl_name)) {
			break;
		}

		if (!strstr(tmp_control->id.name, ctl_name))
			tmp_control = NULL;

		pr_err("%s: %s %s\n", __func__, ctl_name,
			tmp_control ? "found" : "not found");

		if (!tmp_control)
			return -EINVAL;

		*tfa_control = tmp_control;
	}

	snd_power_lock(codec->card->snd_card);
	if (tmp_control->put && tmp_control->get) {
		control.value.integer.value[0] = set_val;
		tmp_control->put(tmp_control, &control);
	}
	snd_power_unlock(codec->card->snd_card);

	return 0;
}

static int tfa98xx_mi2s_clk_enable(void)
{	
	int ret;
	pr_info("%s():set QUAT_MI2S_RX Format ,MI2S Clock\n", __func__);
	ret = find_control_set_valu("QUAT_MI2S_RX Format", 0, &mi2s_form_control);
	//if (ret)
		//return ret;
	ret = find_control_set_valu("MI2S Clock", 1, &mi2s_clk_control);
	if(ret)
		return ret;
	return 0;
}

static int tfa98xx_mi2s_clk_disable(void)
{	
	int ret;
	pr_info("%s():set QUAT_MI2S_RX Format ,MI2S Clock\n", __func__);
	ret = find_control_set_valu("MI2S Clock", 0, &mi2s_clk_control);
	if(ret)
		return ret;
	return 0;
}

static int tfa98xx_dsp_get_speaker_freq(struct tfa98xx *tfa98xx)
{
	 unsigned char bytes[3 * 141];
	 int data[141];
	 int error = 0; 
	 int fRes,fResInit;
	 
	 mutex_lock(&tfa98xx->dsp_init_lock);
	 error = tfa98xx_dsp_get_param(tfa98xx, 1,
					PARAM_GET_LSMODEL, 423, bytes);
	 mutex_unlock(&tfa98xx->dsp_init_lock);
	 if( error)
	 	pr_err("%s: error\n", __func__);
	
	 tfa98xx_convert_bytes2data(sizeof(bytes), bytes, data);
	
	 fRes = data[135];
	 fResInit = data[136];
	 
	 pr_info("%s:f0 = %d fRes = %d\n",__func__,fRes,fResInit);

	 return fRes;
}
#endif 

#ifdef TFA98XX_DEBUG
static struct dentry *tfa98xx_debugfs_root;
static struct dentry *tfa98xx_debugfs_reg;
static struct dentry *tfa98xx_debugfs_calibrate;
//static struct dentry *tfa98xx_debugfs_status;
static struct dentry *tfa98xx_debugfs_freq;
static struct dentry *tfa98xx_debugfs_i2c;
static struct mutex debugfs_lock;

static u8 tfa98xx_regs[] = {
	0x00, //TFA98xx_STATUS
	0x01, //TFA98xx_BATTERYVOLTAGE (0x01)
	0x02, //TFA98xx_TEMPERATURE (0x02)
	0x03, //TFA98xx_REVISIONNUMBER (0x03)
	0x04, //TFA98xx_I2SREG (0x04)
	0x05, //TFA98xx_BAT_PROT (0x05)
	0x06, //TFA98xx_AUDIO_CTR (0x06)
	0x07, //TFA98xx_DCDCBOOST (0x07)
	0x08, //TFA98xx_SPKR_CALIBRATION (0x08)
	0x09, //TFA98xx_SYS_CTRL (0x09)
	0x0a, //TFA98xx_I2S_SEL_REG (0x0a)
	0x0b, //TFA98xx_MTP_KEY (0x0b)
	0x0f, //TFA98xx_INTERRUPT_CTL (0x0f)
	0x40, //TFA98xx_HIDE_UNHIDE_KEY (0x40)
	0x41, //TFA98xx_PWM_CONTROL (0x41)
	0x46, //TFA98xx_CURRENTSENSE1 (0x46)
	0x47, //TFA98xx_CURRENTSENSE2 (0x47)
	0x48, //TFA98xx_CURRENTSENSE3 (0x48)
	0x49, //TFA98xx_CURRENTSENSE4 (0x49)
	0x4c, //TFA98xx_ABISTTEST (0x4c)
	0x62, //TFA98xx_MTP_COPY (0x62)
	0x70, //TFA98xx_CF_CONTROLS (0x70)
	0x71, //TFA98xx_CF_MAD (0x71)
	0x72, //TFA98xx_CF_MEM (0x72)
	0x73, //TFA98xx_CF_STATUS (0x73)
	0x80, //TFA98xx_MTP
};

static int tfa98xx_debug_open(struct inode *inode,
		struct file *file)
{
	printk("%s enter\n",__func__);
	mutex_lock(&debugfs_lock);
	return 0;
}

static int tfa98xx_debug_release (struct inode *inode, struct file *filep)
{
	printk("%s exit\n",__func__);
	mutex_unlock(&debugfs_lock);
	return 0;
}

static ssize_t tfa98xx_debug_reg_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	
	printk("tfa98xx_reg_write(): cnt %d\n", (int)cnt);

	if (cnt > 2) {
		ret = sscanf(ubuf, "%x %x", &kbuf[0], &kbuf[1]);
		if (!ret)
			return -EFAULT;
		printk("%s() kbuf[0] = 0x%04x, kbuf[1] = 0x%x, cnt = %d\n",
				__func__, kbuf[0], kbuf[1], (int)cnt);
		regmap_write(tfa98xx_priv->regmap, kbuf[0], kbuf[1]);
	}
	return cnt;
}

static ssize_t tfa98xx_debug_reg_read(struct file *file,
		char __user *buf, size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	int data, n = 0;
	char buffer[size];

	for(i = 0; i < sizeof(tfa98xx_regs); i++) {
		regmap_read(tfa98xx_priv->regmap, tfa98xx_regs[i], &data);
		n += scnprintf(buffer+n, size - n,
				"reg{%04x}:%x \n",
				tfa98xx_regs[i], data);
	}
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations tfa98xx_debug_reg_fops = {
	.open	 = tfa98xx_debug_open,
	.read 	 = tfa98xx_debug_reg_read,
	.write	 = tfa98xx_debug_reg_write,
	.release = tfa98xx_debug_release,
};

static ssize_t tfa98xx_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	unsigned int kbuf[2];
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	int ret = 0;

	if (!tfa98xx) {
		pr_err("%s(): tfa98xx is NULL\n",__func__);
		return cnt;
	}

	ret = sscanf(ubuf, "%d", &kbuf[0]);

	pr_err("%s(): cnt %d, kbuf[0] %d\n",__func__,(int)cnt, kbuf[0]);

	switch(kbuf[0]) {
	case 1:
		mutex_lock(&tfa98xx->dsp_init_lock);

		if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl,
					   tfa98xx->vstep_ctl))
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;

		mutex_unlock(&tfa98xx->dsp_init_lock);
		break;
	case 2:
		mutex_lock(&tfa98xx->dsp_init_lock);
		tfa98xx_dsp_stop(tfa98xx);
		mutex_unlock(&tfa98xx->dsp_init_lock);
		break;
	case 3:
		ret= tfa98xx_mi2s_clk_enable();
		if (ret)
			return ret;
		ret = tfa98xx_init(1);
		if (ret)
			return ret;
		ret = tfa98xx_mi2s_clk_disable();
		if (ret)
			return ret;
		break;
	case 4:
		tfa98xx_mi2s_clk_enable();
		msleep(10000);
		tfa98xx_mi2s_clk_disable();
		break;
	case 5:
		tfa98xx_filename = "tfa98xx/test.cnt";
		ret = tfa98xx_cnt_loadfile(tfa98xx, 0);
		if (ret)
			return ret;
		ret= tfa98xx_mi2s_clk_enable();
		if (ret)
			return ret;
		ret = tfa98xx_init(1);
		if (ret)
			return ret;
		ret = tfa98xx_mi2s_clk_disable();
		if (ret)
			return ret;
		break;
	case 8:
		tfa98xx_mi2s_clk_enable();
		mutex_lock(&tfa98xx->dsp_init_lock);
		tfa98xx_reset_Mtp(tfa98xx);
		mutex_unlock(&tfa98xx->dsp_init_lock);
		tfa98xx_mi2s_clk_disable();
		pr_err("%s(): reset mtp \n", __func__);
		break;
	default:
		pr_err("%s:no supported cmd %d\n",__func__, kbuf[0]);
	}

	return cnt;
}

int tfa98xx_reset_mtp_dbg(void)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	
	pr_info("%s(): reset mtp Enter.\n", __func__);
	tfa98xx_mi2s_clk_enable();
	mutex_lock(&tfa98xx->dsp_init_lock);
	tfa98xx_reset_Mtp(tfa98xx);
	mutex_unlock(&tfa98xx->dsp_init_lock);
	tfa98xx_mi2s_clk_disable();
	pr_info("%s(): reset mtp complete.\n", __func__);
	return 0;
}

int tfa98xx_init_dbg(char *buffer, int size)
{
	int ret = 0, n = 0, valu = 0;
	int course = 0, fine = 0;
	ret= tfa98xx_mi2s_clk_enable();
	if (ret)
		return ret;
	ret = tfa98xx_init(1);
	if (ret)
		return ret;
	if(tfa98xx_calibration_done(tfa98xx_priv->imped_val))
		valu = tfa98xx_priv->imped_val;
	if (!ret) {
		course = tfa98xx_priv->imped_val / 100;
		fine = tfa98xx_priv->imped_val % 100;
	}
	n = scnprintf(buffer, size,
		"current status:\n mono: impedance %s %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n",
		    (!ret && valu) ? "ok" : "error", course, fine, tfa98xx_priv->para.imped_min / 100,
		    tfa98xx_priv->para.imped_min % 100, tfa98xx_priv->para.imped_max / 100, 
		    tfa98xx_priv->para.imped_max % 100);

	buffer[n] = 0;
	ret = tfa98xx_mi2s_clk_disable();
	if (ret)
		return ret;
	return 1;
}

static ssize_t tfa98xx_debug_read(struct file *file,
		char __user *buf, size_t count, loff_t *pos)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	const int size = 512;
	int n = 0;
	int ret = 0;
	char buffer[size];
	int valu;

	if (!tfa98xx) {
		pr_err("%s(): tfa98xx is NULL\n",__func__);
		return 0;
	}
	if(tfa98xx_calibration_done(tfa98xx->imped_val))
	ret = 1;
	
	valu = tfa98xx->imped_val;
	
	n = scnprintf(buffer, size,
		"current status:\n mono: impedance %s %02d.%02d ohm \n",
		    ret?"ok":"error",valu/100,valu%100);

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations tfa98xx_debugfs_fops = {
	.open    = tfa98xx_debug_open,
	.read 	 = tfa98xx_debug_read,
	.write 	 = tfa98xx_debug_write,
	.release = tfa98xx_debug_release,
};

static ssize_t tfa98xx_debug_freq_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	const int size = 512;
	char buffer[size];
	int n = 0;
	int fRes;
	int i = 0,j = 0;

	printk("%s\n",__func__);
	if(!tfa98xx)
		return -EINVAL;
	printk("%s:tfa98xx fRes min = %d fRes max = %d\n",__func__,
					tfa98xx->para.fres_min, tfa98xx->para.fres_max);
	while ((i++ < 15) && (j < 5)) {
		fRes = tfa98xx_dsp_get_speaker_freq(tfa98xx);
		if ((fRes < tfa98xx->para.fres_min) || (fRes > tfa98xx->para.fres_max))
			j = 0;
		else
			j++;
		n += scnprintf(buffer+n, size-n, "f0 = %d \n", fRes);
		printk("%s:tfa98xx fRes = %d i = %d j = %d\n",
			__func__, fRes, i, j);
		msleep(500);
	}
	if(j == 5)
		n += scnprintf(buffer+n, size-n, "PASS\n");
	else
		n += scnprintf(buffer+n, size-n, "FAIL\n");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations tfa98xx_debugfs_freq_fops = {
	.open  	 = tfa98xx_debug_open,
	.read	 = tfa98xx_debug_freq_read,
	.release = tfa98xx_debug_release,
};
static ssize_t tfa98xx_i2c_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	struct snd_soc_codec *codec = tfa98xx->codec;
	const int size = 512;
	char buffer[size];
	int n = 0;
	int val;

	tfa98xx_mi2s_clk_enable();
	tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl, tfa98xx->vstep_ctl);
	val = snd_soc_read(codec, TFA98XX_REVISIONNUMBER);
	val = val & 0xff;
	tfa98xx_dsp_stop(tfa98xx);
	tfa98xx_mi2s_clk_disable();

	if(val == REV_TFA9887 || val == REV_TFA9890 ||
			val == REV_TFA9895 || val == REV_TFA9897)
		printk(KERN_ERR "%s: read id successfully \n",__func__);
	else{
		printk(KERN_ERR "%s: read id error %d \n",__func__,val);
		val = 0;
	}

	n += scnprintf(buffer+n, size-n, "tfa98xx i2c read %s \n",
							val? "OK":"ERROR" );
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations tfa98xx_debugfs_i2c_fops = {
	.open = tfa98xx_debug_open,
	.read = tfa98xx_i2c_debug_read,
	.release = tfa98xx_debug_release,
};
static void tfa98xx_debugfs_init(void)
{
	mutex_init(&debugfs_lock);
	tfa98xx_debugfs_root = debugfs_create_dir("tfa98xx", NULL);
	if(!tfa98xx_debugfs_root)
		pr_err("%s debugfs create dir error\n", __func__);
	else if(IS_ERR(tfa98xx_debugfs_root)) {
		pr_err("%s Kernel not support debugfs \n", __func__);
		tfa98xx_debugfs_root = NULL;
	}
	tfa98xx_debugfs_reg = debugfs_create_file("codec_reg",
					0644, tfa98xx_debugfs_root,
					NULL, &tfa98xx_debug_reg_fops);
	if (!tfa98xx_debugfs_reg)
		pr_err("tfa98xx debugfs create file codec_reg fail \n");
	tfa98xx_debugfs_calibrate = debugfs_create_file("calibrate",
					0644, tfa98xx_debugfs_root,
					NULL, &tfa98xx_debugfs_fops);
	if (!tfa98xx_debugfs_calibrate)
		pr_err("tfa98xx debugfs create file calibrate fail \n");
/*
	####This debugfs function is same as calibrate , so delete it -----------change by xdmemory
	tfa98xx_debugfs_status = debugfs_create_file("current_status",
					0644, tfa98xx_debugfs_root,
					NULL, &tfa98xx_debugfs_fops);
	if (!tfa98xx_debugfs_status)
		pr_err("tfa98xx debugfs create file current_status fail \n");

*/
	tfa98xx_debugfs_freq = debugfs_create_file("fRes_Detect",
					0444, tfa98xx_debugfs_root,
					NULL, &tfa98xx_debugfs_freq_fops);
	if(!tfa98xx_debugfs_freq)
		pr_err("tfa98xx debugfs create file fRes_Detect fail \n");
	tfa98xx_debugfs_i2c = debugfs_create_file("i2c",
					0444, tfa98xx_debugfs_root,
					NULL, &tfa98xx_debugfs_i2c_fops);
	if(!tfa98xx_debugfs_i2c)
		pr_err("tfa98xx debugfs create file i2c fail \n");
	return ;
}

static void tfa98xx_debugfs_deinit(void)
{
	debugfs_remove(tfa98xx_debugfs_reg);
	debugfs_remove(tfa98xx_debugfs_calibrate);
	//debugfs_remove(tfa98xx_debugfs_status);
	debugfs_remove(tfa98xx_debugfs_freq);
	debugfs_remove(tfa98xx_debugfs_i2c);
	debugfs_remove(tfa98xx_debugfs_root);
	return ;
}

#endif

int tfa98xx_read_freq_dbg(char *buffer, int size)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	int n = 0;
	int fRes;
	int i = 0,j = 0;

	printk("%s",__func__);
	if(!tfa98xx)
		return -EINVAL;
	printk("%s:tfa98xx fRes min = %d fRes max = %d\n",__func__,
					tfa98xx->para.fres_min, tfa98xx->para.fres_max);
	while ((i++ < 15) && (j < 5)) {
		fRes = tfa98xx_dsp_get_speaker_freq(tfa98xx);
		if ((fRes < tfa98xx->para.fres_min) || (fRes > tfa98xx->para.fres_max))
			j = 0;
		else
			j++;
		n += scnprintf(buffer+n, size-n, "f0 = %d \n", fRes);
		printk("%s:tfa98xx fRes = %d i = %d j = %d\n",
			__func__, fRes, i, j);
		msleep(500);
	}
	if(j == 5)
		n += scnprintf(buffer+n, size-n, "PASS\n");
	else
		n += scnprintf(buffer+n, size-n, "FAIL\n");

	buffer[n] = 0;

	return 0;
}

/*
 * I2C Read/Write Functions
 */
int tfa98xx_i2c_read(struct i2c_client *tfa98xx_client,	u8 reg, u8 *value,
		     int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = tfa98xx_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	//pr_err("tfa98xx_i2c_read() reg 0x%x len %d\n", reg, len);

	do {
		err = i2c_transfer(tfa98xx_client->adapter, msgs,
							ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&tfa98xx_client->dev, "read transfer error %d\n" , err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

int tfa98xx_bulk_write_raw(struct snd_soc_codec *codec, const u8 *data,
				u8 count)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;

	//pr_err("tfa98xx_bulk_write_raw_bytes() data[0] 0x%x count %u\n", data[0], count);

	ret = i2c_master_send(tfa98xx->i2c, data, count);
	if (ret == count) {
		return 0;
	} else if (ret < 0) {
		pr_err("Error I2C send %d\n", ret);
		return ret;
	} else {
		pr_err("Error I2C send size mismatch %d\n", ret);
		return -EIO;
	}
}


static int tfa98xx_dsp_power_on(struct tfa98xx *tfa98xx)
{
	pr_info("%s() \n ",__func__);
	pr_info("profile %d, vstep %d\n", tfa98xx->profile_ctl,
		 tfa98xx->vstep_ctl);
	if(tfa98xx->calibration)
	{
		/* start the DSP using the latest profile / vstep */
	
		if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile_ctl,
				       tfa98xx->vstep_ctl))
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
		return 0;
	}else{
		pr_err("%s() not calibration \n ", __func__);
	}
	
	return -1;
}

static int tfa98xx_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	return 0;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	pr_info("%s()\n",__func__);
	pr_debug("Store rate: %d\n", params_rate(params));

	/* Store rate for further use during DSP init */
	tfa98xx->rate = params_rate(params);

	return 0;
}


static int tfa98xx_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s() state: %d\n",__func__, mute);
		
	if (!tfa98xx ) 
	{
		pr_err("%s:tfa98xx_prv is NULL\n", __func__);
		return -EINVAL;
	}
	
	if (stream == SNDRV_PCM_STREAM_PLAYBACK )
	{
		if (mute) {
			pr_debug("%s() mute \n", __func__);

			if (!tfa98xx_i2c_check()){
				pr_err("%s() i2c error \n", __func__);
				return 0;
			}
			mutex_lock(&tfa98xx->dsp_init_lock);
			if(tfa98xx->speaker_on)
				tfa98xx_dsp_stop(tfa98xx);
			tfa98xx->speaker_on = 0;
			mutex_unlock(&tfa98xx->dsp_init_lock);
			msleep(5);
		} else if(!mute){
			pr_debug("%s() ummute\n",__func__);
			if (!tfa98xx->speaker_on) { //qbq sync the modification from 8976 to avoid tfa98 was opened twice.
				msleep(20);
				if (!tfa98xx_i2c_check()){
					pr_err("%s() i2c error \n", __func__);
					return 0;
				}
				mutex_lock(&tfa98xx->dsp_init_lock);
				if (!tfa98xx_dsp_power_on(tfa98xx))
					tfa98xx->speaker_on = 1;
				mutex_unlock(&tfa98xx->dsp_init_lock);
			}
		}		
	}
	return 0;
}

static const struct snd_soc_dai_ops vivo_codec_ops = {
	.hw_params	 = tfa98xx_hw_params,
	.mute_stream = tfa98xx_mute_stream,
	.set_fmt	 = tfa98xx_set_dai_fmt,
};

static struct snd_soc_dai_driver vivo_codec_dai[] = {
	{
		.name = "SmartPA",
		.id = 0,
		.playback = {
			.stream_name = "SmartPA Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "SmartPA Capture",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &vivo_codec_ops,
	},
};

static const char *smart_pa_mode_text[] = {
		"Music", "Voice", "Off",};

static const struct soc_enum smart_pa_mode_enum =
	SOC_ENUM_SINGLE_EXT(3, smart_pa_mode_text);

static int smart_pa_mode_get(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	const char *pstring;
	int value;
	
	value = smart_pa_mode_select;

	switch(value)
	{
		case 0:
			pstring = "Music";
			break;
		case 1:
			pstring = "Voice";
			break;
		case 2:
			pstring = "Off";
			break;
		default:
			value = 0;
			pstring = "Music";
			break;
	}
	pr_info("%s:value %d selected MODE: %s\n",
			__func__, value, pstring);
//	type is int ,so change it ------change by XDmemory
//	ucontrol->value.enumerated.item[0] = value;
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int smart_pa_mode_put(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	const char *pstring;

	smart_pa_mode_select = value;

	if (!tfa98xx ) 
	{
		pr_err("%s:tfa98xx_prv is NULL\n",
			__func__);
		return 0;
	}
	
	switch(value)
	{
		case 0:
			pstring = "Music";
			break;
		case 1:
			pstring = "Voice";
			break;
		case 2:
			value = 0;
			pstring = "Off";
			break;
		default:
			value = 0;
			pstring = "Music";
			break;
	}
	pr_info("%s:value %d select MODE: %s\n",
			__func__, value, pstring);
	tfa98xx->profile_ctl = value;
	
	return 0;
}

static const struct snd_kcontrol_new tfa98xx_codec_controls[] = {
	SOC_ENUM_EXT("Smart PA Mode", smart_pa_mode_enum,
				smart_pa_mode_get, smart_pa_mode_put),
};

static int smart_pa_get_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = smart_pa_switch_enable;
	pr_info("%s: smart pa enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int smart_pa_put_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];

	pr_info("%s: smart pa enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
	else
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);

	smart_pa_switch_enable = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new smart_pa_ctl =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM, 0, 1, 0,
		smart_pa_get_switch_mixer, smart_pa_put_switch_mixer);

static const struct snd_soc_dapm_widget vivo_codec_dapm_widgets[] = {
			
	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_SWITCH("SmartPA", SND_SOC_NOPM, 0, 1, &smart_pa_ctl),

	SND_SOC_DAPM_AIF_IN_E("SmartPA I2S In", "SmartPA Playback", 0, SND_SOC_NOPM,
			0, 0, NULL, SND_SOC_DAPM_STREAM_START|SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SmartPA I2S Out", "SmartPA Capture", 0, SND_SOC_NOPM,
			0, 0, NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route vivo_codec_map[] = {
	{"Speaker", NULL, "SmartPA"},
	{"SmartPA", "Switch", "SmartPA I2S In"},
};

static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;
	u16 rev;
	printk(KERN_ERR "%s():enter!",__func__);
	codec->control_data = tfa98xx->regmap;
	tfa98xx->codec = codec;
	codec->cache_bypass = true;

	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	/*
	  * some device require a dummy read in order to generate
	  * i2c clocks when accessing the device for the first time
	  */
	snd_soc_read(codec, TFA98XX_REVISIONNUMBER);

	rev = snd_soc_read(codec, TFA98XX_REVISIONNUMBER);
	dev_info(codec->dev, "ID revision 0x%04x\n", rev);
	tfa98xx->rev = rev & 0xff;
	tfa98xx->subrev = (rev >> 8) & 0xff;

	ret = tfa98xx_cnt_loadfile(tfa98xx, 0);
	if (ret)
		return ret;


	tfa98xx->mtp_backup = 0;
	tfa98xx->profile_current = 0;
	tfa98xx->vstep_current = 0;
	tfa98xx->profile_ctl = 0;
	tfa98xx->vstep_ctl = 0;
	tfa98xx->imped_val = 0;
	tfa98xx->calibration = 0;
	tfa98xx->speaker_on = 0;
	mi2s_clk_control = NULL;
	mi2s_form_control = NULL;

	dev_info(codec->dev, "tfa98xx codec registered");
	printk(KERN_ERR "%s():leave!",__func__);
	return 0;
}

static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "tfa98xx codec removed");
	return 0;
}

static struct snd_soc_codec_driver soc_vivo_codec_driver = {
	.probe    = tfa98xx_probe,
	.remove   = tfa98xx_remove,
	.controls = tfa98xx_codec_controls,
    .num_controls = ARRAY_SIZE(tfa98xx_codec_controls),
	.dapm_widgets = vivo_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vivo_codec_dapm_widgets),
	.dapm_routes      = vivo_codec_map,
	.num_dapm_routes  = ARRAY_SIZE(vivo_codec_map),
};

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = TFA98XX_MAX_REGISTER,
	.cache_type   = REGCACHE_RBTREE,
};

static void tfa98xx_dts_phrase(struct i2c_client *dev, struct tfa98xx *tfa98xx)
{
	struct device_node *node = NULL;
	struct tfa98xx_para para;
	const char *name, *config;
	char *pstring;
	unsigned int temp;
	int ret;

	if ( dev->dev.of_node){
		node = dev->dev.of_node;
	}else{
		pr_err("%s:node is NULL \n", __func__);
		return;
	}
	
	ret = of_property_read_string(node, "vivo,tfa98xx-version", &name);
	if ((!!ret)||(!name))
		name = "tfa98xx";
	
	ret = of_property_read_string(node, "vivo,tfa98xx-config", &config);
	if ((!ret)&&(!!config))
		/* change to 30 because the name of .cnt will be bigger then 20 bytes */
		pstring  = kstrndup(config, 30, GFP_KERNEL);
   //qbq add to enable vdd regulator as it will shutdown by somewhere start
	tfa98xx->tfa98xx_vdd_regulator = regulator_get(&dev->dev, "vivo,tfa98xx-vdd");
	
	if(!tfa98xx->tfa98xx_vdd_regulator||IS_ERR(tfa98xx->tfa98xx_vdd_regulator)){		
		pr_err("%s:fail to get regulator of vdd.\n",__func__);
		tfa98xx->tfa98xx_vdd_regulator = NULL;
	}else{
		if ((regulator_count_voltages(tfa98xx->tfa98xx_vdd_regulator) > 0)
				&& regulator_can_change_voltage(tfa98xx->tfa98xx_vdd_regulator)){
            		ret = regulator_set_voltage(tfa98xx->tfa98xx_vdd_regulator, 1800000,
							1800000);
	              if (ret) {
	                pr_err("%s:fail to set voltage of vdd.\n",__func__);
	                regulator_put(tfa98xx->tfa98xx_vdd_regulator);
	              }
		}
		ret=regulator_enable(tfa98xx->tfa98xx_vdd_regulator);
		if(ret){
			pr_err("%s:fail to enable regulator of vdd.\n",__func__);
			tfa98xx->tfa98xx_vdd_regulator = NULL;
		}
		else
			pr_info("%s:Success to enable vdd 1.8V\n",__func__);
	}
	
  //qbq add to enable vdd regulator as it will shutdown by somewhere end 20160126	
	ret = of_property_read_u32(node, "vivo,tfa98xx-impedance-min", &temp);
	para.imped_min= ( !ret)? (int)temp:500;

	ret = of_property_read_u32(node, "vivo,tfa98xx-impedance-max", &temp);
	para.imped_max= ( !ret)? (int)temp:1100;

	ret = of_property_read_u32(node, "vivo,tfa98xx-frequency-min", &temp);
	para.fres_min= ( !ret)? (int)temp:600;

	ret = of_property_read_u32(node, "vivo,tfa98xx-frequency-max", &temp);
	para.fres_max= ( !ret)? (int)temp:1000;

	tfa98xx->tfa98xx_rst_gpio =
		of_get_named_gpio(node, "qcom,tfa98xx-rst-gpio", 0);
	ret = gpio_request(tfa98xx->tfa98xx_rst_gpio, "smart pa reset");
	if (ret) {
		tfa98xx->tfa98xx_rst_gpio = 0;
		pr_err("%s:gpio %d for smart pa reset gpio request failed...\n",
			__func__, tfa98xx->tfa98xx_rst_gpio);
	} else
		gpio_direction_output(tfa98xx->tfa98xx_rst_gpio, 0);

	tfa98xx->name = name;
	tfa98xx->para = para;

	if (pstring)
		tfa98xx_filename = pstring;// need to kfree?

	pr_info("%s: use the %s \n", __func__, tfa98xx_filename);
	pr_info("%s: frequence range from %d to %d Hz \n",
			__func__, para.fres_min, para.fres_max);
	pr_info("%s: calibration's value range from %02d.%02d to %02d.%02d Ohm \n",
			__func__, para.imped_min/100, para.imped_min%100,
			para.imped_max/100, para.imped_max%100);
}

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct tfa98xx *tfa98xx;
	int ret;
	printk(KERN_ERR "%s() enter\n", __func__);

#ifdef TFA98XX_DEBUG
	tfa98xx_debugfs_init();
#endif

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}
	ret = dev_set_name(&i2c->dev, "%s", "tfa98xx");
	if (ret||ERR_PTR(ret))
	return ret;

	tfa98xx = devm_kzalloc(&i2c->dev, sizeof(struct tfa98xx),
			       GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->i2c = i2c;
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_init_lock);
	
	tfa98xx_priv = tfa98xx;

	/* register codec */
	ret = snd_soc_register_codec(&i2c->dev, &soc_vivo_codec_driver,
				     vivo_codec_dai, ARRAY_SIZE(vivo_codec_dai));
	if (ret < 0) {
		pr_err("%s: Error registering tfa98xx codec", __func__);
		goto codec_fail;
	}

	tfa98xx_dts_phrase(i2c, tfa98xx);

	pr_info("tfa98xx probed successfully!");

	ret = tfa98xx_debug_probe(i2c);
	if (ret != 0) {
		printk(KERN_ERR"Failed to probe tfa98xx debug: %d\n", ret);
	}

	printk(KERN_ERR"%s() leave\n", __func__);
	return ret;

codec_fail:
	
	snd_soc_unregister_codec(&i2c->dev);
	return ret;
}

static int tfa98xx_i2c_remove(struct i2c_client *client)
{

	snd_soc_unregister_codec(&client->dev);

#ifdef TFA98XX_DEBUG
	tfa98xx_debugfs_deinit();
#endif
	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ "tfa98xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_match_tbl[] = {
	{ .compatible = "nxp,tfa98xx" },
	{ },
};
MODULE_DEVICE_TABLE(of, tfa98xx_match_tbl);
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_match_tbl),
	},
	.probe    = tfa98xx_i2c_probe,
	.remove   = tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

module_i2c_driver(tfa98xx_i2c_driver);

MODULE_DESCRIPTION("ASoC tfa98xx codec driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
