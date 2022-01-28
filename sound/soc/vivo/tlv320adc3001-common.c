#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#include "vivo-codec-common.h"
#include "tlv320adc3001-common.h"

#define TLV320_DEV_NAME "tlv320adc3001"
static bool tlv320_available = false;

#ifdef CONFIG_VIVO_REGDUMP
	#include <linux/vivo-regdump.h>

	static u8 tlv320_regs_dump[] = {
	TLV320_PAGE_CTL,
	TLV320_SW_RST,
	TLV320_CLK_IN_MUX,
	TLV320_PLL_P_R_VAL,
	TLV320_PLL_J_VAL,
	TLV320_PLL_D_VAL_MSB,
	TLV320_PLL_D_VAL_LSB,
	TLV320_NADC_VAL,
	TLV320_MADC_VAL,
	TLV320_AOSR_VAL,
	TLV320_IADC_VAL,
	TLV320_CLKOUT_MUX,
	TLV320_CLKOUT_M_VAL,
	TLV320_ADC_INF_CTL_1,
	TLV320_ADC_INF_CTL_2,
	TLV320_BDIV_N_VAL,
	TLV320_DOUT_CTL,
};
	static int tlv320_power_on;
	static int tlv320_rate;
#endif


#ifdef BBK_VIVO_AUDIO_DEBUG

static int pm_enable_gpio = 0;
static struct dentry *tlv320_debugfs_root;
static struct dentry *tlv320_debugfs_reg;

static u8 tlv320_regs[] = {
	TLV320_PAGE_CTL,
	TLV320_SW_RST,
	TLV320_CLK_IN_MUX,
	TLV320_PLL_P_R_VAL,
	TLV320_PLL_J_VAL,
	TLV320_PLL_D_VAL_MSB,
	TLV320_PLL_D_VAL_LSB,
	TLV320_NADC_VAL,
	TLV320_MADC_VAL,
	TLV320_AOSR_VAL,
	TLV320_IADC_VAL,
	TLV320_CLKOUT_MUX,
	TLV320_CLKOUT_M_VAL,
	TLV320_ADC_INF_CTL_1,
	TLV320_ADC_INF_CTL_2,
	TLV320_BDIV_N_VAL,
	TLV320_DOUT_CTL,
};

#endif

static u8 reg_settings[][2] = {

	{ 0x04, 0x03},
	{ 0x05, 0x11},
	{ 0x06, 0xff},
	{ 0x07, 0xff},
	{ 0x08, 0xff},
	{ 0x05, 0x91},
	{ 0xff, 0xff},
	{ 0x12, 0x84},
	{ 0x13, 0xff},
	{ 0x14, 0x20},
	{ 0x19, 0x03},
	{ 0x1d, 0x06},
	{ 0x35, 0x16},
	{ 0x1e, 0xff},
	{ 0x1a, 0x84},
	{ 0x1b, 0x3c},
};


struct tlv320_data {
    struct i2c_client *client;
	struct mutex lock;

	struct vivo_codec_function *fun;
};
static struct tlv320_data *tlv320_data;

static u8 tlv320_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(tlv320_data->client,&reg,1);
	if(ret < 0)
	{
		dev_err(&tlv320_data->client->dev, "%s i2c send cmd error reg=%d \n",__func__,reg);
		return ret;
	}
	ret = i2c_master_recv(tlv320_data->client,&buf,1);
	if(ret <0 )
	{
		dev_err(&tlv320_data->client->dev,"%s i2c recv error \n ",__func__);
		return ret;
	}

	return buf;

}
static u8 tlv320_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(tlv320_data->client,cmd,sizeof(cmd));
	if (ret < 1)
	{
		dev_err(&tlv320_data->client->dev,"%s i2c send error cmd[0]=%d,cmd[1]=%d\n",__func__,cmd[0],cmd[1]);
	}

	return ret;
}

static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = tlv320_i2c_read_byte(reg);
	while((val != reg_val)&&(retry > 0)){
		tlv320_i2c_write_byte(reg, val);
		reg_val = tlv320_i2c_read_byte(reg);
		retry --;
	}
	if(retry == 5)
		ret = 0;
	else if((retry >= 0)&&(val == reg_val)){
		sprintf(logbuf,"Write 0x%2x to 0x%2x success, retry %d times\n",
			val, reg, 5-retry);
			ret = 0;
	}else if(!retry){
		sprintf(logbuf,"Write 0x%2x to 0x%2x failed, retry 5 times\n",
			val, reg);
		ret = -1;
	}
	return ret;
}

int tlv320_enable(struct audio_params *params, bool enable)
{
	struct vivo_codec_function *function = tlv320_data->fun;
	int retry, ret, i =0;
	
	pr_err("%s: tlv enter!!!\n", __func__);

	if (tlv320_data->client == NULL) {
		dev_err(&tlv320_data->client->dev,
					"%s client is NULL \n ",
					__func__);
		return -EFAULT;
	}

	if (!function) {
		dev_err(&tlv320_data->client->dev,
					"%s function is NULL \n ",
					__func__);
		return -EINVAL;
	}

	if (!params) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}

	if(!tlv320_available)
		return 0;

	pr_err("%s: enable %d, rate %d\n", __func__, enable, params->rate);

	mutex_lock(&tlv320_data->lock);

	if(!enable) {
		if (function->mclk_disable)
			function->mclk_disable(VIVO_CODEC_HIFI_CLK);
		if (function->power_down)
			function->power_down(VIVO_CODEC_HIFI_CLK);
#ifdef CONFIG_VIVO_REGDUMP
			tlv320_power_on = 0;
#endif
		ret = 0;
		goto end;
	}

	switch (params->rate) {
	case 44100:
	case 88200:
	case 176400:
		reg_settings[2][1] = 0x04;
		reg_settings[3][1] = 0x1b;
		reg_settings[4][1] = 0x80;
		break;
	case 48000:
	case 96000:
	case 192000:
		reg_settings[2][1] = 0x05;
		reg_settings[3][1] = 0x04;
		reg_settings[4][1] = 0xb0;
		break;
	default:
		pr_err("%s: rate error %d\n", __func__, params->rate);
		return -EINVAL;
	}

	switch (params->rate) {
	case 44100:
	case 48000:
		reg_settings[8][1] = 0x90;
		reg_settings[13][1] = 0x88;
		break;
	case 88200:
	case 96000:
		reg_settings[8][1] = 0x88;
		reg_settings[13][1] = 0x84;
		break;
	case 176400:
	case 192000:
		reg_settings[8][1] = 0x84;
		reg_settings[13][1] = 0x82;
		break;
	default:
		pr_err("%s: rate error %d\n", __func__, params->rate);
		return -EINVAL;
	}

	if (function->mclk_enable)
		function->mclk_enable(VIVO_CODEC_HIFI_CLK, 19200000);
	if (function->power_up)
		function->power_up(VIVO_CODEC_HIFI_CLK);

	usleep(5000);
	if (function->hw_reset)
		function->hw_reset(VIVO_CODEC_HIFI_CLK);

	tlv320_i2c_write_byte(TLV320_PAGE_CTL,0x0);
	tlv320_i2c_write_byte(TLV320_SW_RST,0x01);
	msleep(10);
	for(i = 0; i < 16; i++){
		retry = 5;
		if(reg_settings[i][0] == 0xff){
			msleep(15);
			continue;
		}
		tlv320_i2c_write_byte(reg_settings[i][0],reg_settings[i][1]);
		pr_debug("%s:Write 0x%2x to reg 0x%2x\n",__func__, 
			reg_settings[i][1], reg_settings[i][0]);
		if(reg_settings[i][0] == 0x07)
			continue;
		ret = do_reg_write_check(reg_settings[i][0],reg_settings[i][1]);
		if(ret < 0)
			goto end;
	}
#ifdef CONFIG_VIVO_REGDUMP
	tlv320_power_on = 1;
	tlv320_rate = params->rate;
#endif
	ret = 0;
end:
	mutex_unlock(&tlv320_data->lock);
	pr_err("%s: tlv leave ret is  %d\n", __func__, ret);
	return ret;
}

#ifdef BBK_VIVO_AUDIO_DEBUG
static int tlv320_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t tlv320_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];

	ret = sscanf(ubuf,"%x %x",&kbuf[0],&kbuf[1]);
	if(!ret)
		return -EFAULT;

	printk(KERN_INFO "kbuf[0]=%x,kbuf[1]=%x cnt =%d %s\n",kbuf[0],kbuf[1],(int)cnt,__func__);

	if(kbuf[0]==0xff)
	{
		printk("gpio =0 disable voltage convert \n");
		gpio_direction_output(pm_enable_gpio,0);
	}else
	{

		printk("gpio =1 enable voltage convert \n");
		gpio_direction_output(pm_enable_gpio,1);
	}

	tlv320_i2c_write_byte(kbuf[0],kbuf[1]);

	return cnt;
}
static ssize_t tlv320_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	for(i = 0;i < sizeof(tlv320_regs);i++)
	{
		data = tlv320_i2c_read_byte(tlv320_regs[i]);
		n += scnprintf(buffer+n,size-n,"reg{%x}:%x \n",tlv320_regs[i],data);
	}

	buffer[n] = 0;
	pr_info("%s:==========catch adc3001 reg==========\n%s\n",
		__func__,buffer);
	pr_info("============caught adc3001 reg end =============\n");
	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations tlv320_debugfs_fops = {
	.open = tlv320_debug_open,
	.read = tlv320_debug_read,
	.write = tlv320_debug_write,
};
#endif

static int tlv320_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int tlv320_i2c_resume(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_VIVO_REGDUMP
void tlv320adc_regdump_callback(void)
{
	u8 data;
	int i;
	pr_info("%s() enter\n", __func__);
	if(tlv320_power_on)
	{
		pr_info("%s() rate %d Hz\n",__func__,tlv320_rate);
		for(i = 0;i < sizeof(tlv320_regs_dump);i++){
				data = tlv320_i2c_read_byte(tlv320_regs_dump[i]);
				pr_info("%s():reg[%02x]= %02x \n",__func__,tlv320_regs[i],data);
			}
	}else{
			pr_info("%s() power off \n",__func__);
		}
		
	pr_info("%s() leave\n", __func__);
}

static struct vivo_regdump_handler tlv320adc_regdump_handler = {
	.name = "tlv320adc",
	.callback = tlv320adc_regdump_callback,
};
#endif


static int tlv320_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *function;
	int ret = 0;

	printk("%s\n",__func__);

	function = get_vivo_codec_function();
	if (!function) {
		dev_err(&client->dev, "%s vivo_codec_function is NULL\n",
			__func__);
		return -EINVAL;
	}

	tlv320_data = kzalloc(sizeof(struct tlv320_data), GFP_KERNEL);
	if(!tlv320_data)
	{
		dev_err(&client->dev,"%s kzalloc failed\n",__func__);
		return -ENOMEM;
	}

	mutex_init(&tlv320_data->lock);

	tlv320_data->client = client;

	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
	{
		dev_err(&client->dev,"%s i2c check funtion error \n",__func__);
	}

	function->hifi_clk_enable = tlv320_enable;
	tlv320_data->fun = function;

	if (tlv320_data->fun && tlv320_data->fun->power_up)
		tlv320_data->fun->power_up(VIVO_CODEC_HIFI_CLK);

	if (tlv320_data->fun && tlv320_data->fun->hw_reset)
		tlv320_data->fun->hw_reset(VIVO_CODEC_HIFI_CLK);


	tlv320_available = true;

	msleep(10);

	if (function->power_down)
		function->power_down(VIVO_CODEC_HIFI_CLK);

#ifdef BBK_VIVO_AUDIO_DEBUG
	tlv320_debugfs_root = debugfs_create_dir("tlv320",NULL);
	if(!tlv320_debugfs_root)
	{
		dev_err(&client->dev,"%s debugfs create dir error\n",__func__);
	}
	else if(IS_ERR(tlv320_debugfs_root))
	{
		dev_err(&client->dev,"%s Kernel not support debugfs \n",__func__);
		tlv320_debugfs_root = NULL;
	}

	tlv320_debugfs_reg = debugfs_create_file("reg",0644,tlv320_debugfs_root,NULL,&tlv320_debugfs_fops);
	if(!tlv320_debugfs_reg)
	{
		dev_err(&client->dev,"tlv320 debugfs create fail \n");
	}
#endif

#ifdef CONFIG_VIVO_REGDUMP
	vivo_audio_regdump_register_callback(&tlv320adc_regdump_handler);
#endif

	return ret;

}

static int tlv320_i2c_remove(struct i2c_client *client)
{
	int ret = 0;

	debugfs_remove(tlv320_debugfs_reg);
	debugfs_remove(tlv320_debugfs_root);
	#ifdef CONFIG_VIVO_REGDUMP
		vivo_audio_regdump_deregister_callback(tlv320adc_regdump_handler.name);
	#endif


	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id device_tlv320_of_match[] = {
    {.compatible = "ti,tlv320adc3001",},
    {},
};
#else
#define device_tlv320_of_match 0
#endif

static const struct i2c_device_id tlv320_i2c_id[] = {
	{ TLV320_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver tlv320_i2c_driver = {
    .probe              = tlv320_i2c_probe,
	.suspend 			= tlv320_i2c_suspend,
	.resume 			= tlv320_i2c_resume,
    .remove             = tlv320_i2c_remove,
    .driver = {
        .name           = TLV320_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = device_tlv320_of_match,
    },
    .id_table = tlv320_i2c_id,
};

static int __init tlv320_clk_generator_init(void)
{
	 if(i2c_add_driver(&tlv320_i2c_driver))
	 {
	 	printk(KERN_ERR " %s add i2c driver error\n ",__func__);
	 }
     return 0;
}

static void __exit tlv320_clk_generator_exit(void)
{
    i2c_del_driver(&tlv320_i2c_driver);

	return;
}

module_init(tlv320_clk_generator_init);
module_exit(tlv320_clk_generator_exit);

MODULE_DESCRIPTION("tlv320 clock generator driver");
MODULE_AUTHOR("leichenji <leichenji@vivo.com>");
MODULE_LICENSE("GPL");
