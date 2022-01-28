//liuxudong C for cs4398 dac
#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
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
#include <linux/of_gpio.h>

#include "cs4398-common.h"
#include "vivo-codec-common.h"

#ifdef CONFIG_VIVO_REGDUMP
	#include <linux/vivo-regdump.h>
#endif

#define BBK_VIVO_AUDIO_DEBUG 1
//#define VIVO_CS4398_VA_GPIO 58
#define CS4398_I2C_NAME "cs4398"
#ifdef BBK_AUDIO_KERNEL
#define CS4398_RST_GPIO		(cs4398_data->rest_ctrl_gpio)
#else
#define CS4398_RST_GPIO		70
#endif

struct cs4398_data{
	struct i2c_client *client;
	char *driver_name;
	struct regulator *vd_regulator;
	struct regulator *va_regulator;
	struct regulator *v5v_regulator;
	int vd_type;
	int va_type;
	int v5v_type;
	int va_ctrl_gpio;
	int v5v_ctrl_gpio;
	int rest_ctrl_gpio;
	int vd_ctrl_gpio;
#ifdef BBK_VIVO_AUDIO_DEBUG
	struct dentry *cs4398_debugfs_root;
	struct dentry *cs4398_debugfs_reg;
	struct dentry *cs4398_debugfs_i2c;
#endif

};

/* Interface for HI-FI control */
struct vivo_codec_function *function_cs4398;

static struct cs4398_data *cs4398_data;
static bool cs4398_available = true;

#ifdef BBK_VIVO_AUDIO_DEBUG

static u8 cs4398_reg[] = {
	CS4398_CHIP_ID_REG,
	CS4398_MODE_CTL_REG,
	CS4398_VOL_MIX_INV_CTL_REG,
	CS4398_MUTE_CTL_REG,
	CS4398_A_VOL_CTL_REG,
	CS4398_B_VOL_CTL_REG,
	CS4398_RAMP_FILTER_CTL_REG,
	CS4398_MISC_CTL_REG,
	CS4398_MISC_CTL2_REG
};

#endif

#ifdef CONFIG_VIVO_REGDUMP
static u8 cs4398_reg_dump[] = {
	CS4398_CHIP_ID_REG,
	CS4398_MODE_CTL_REG,
	CS4398_VOL_MIX_INV_CTL_REG,
	CS4398_MUTE_CTL_REG,
	CS4398_A_VOL_CTL_REG,
	CS4398_B_VOL_CTL_REG,
	CS4398_RAMP_FILTER_CTL_REG,
	CS4398_MISC_CTL_REG,
	CS4398_MISC_CTL2_REG
};
static int cs4398_powered;
#endif

#if 0
static int cs4398_power_on(struct regulator *regulator,bool enable)
{
	int ret = 0;
	if(true == enable)
	{
		ret = regulator_enable(regulator);
	}else{
		ret = regulator_disable(regulator);
	}

	return ret;
}
static int cs4398_reset(int gpio_rst)
{
	gpio_direction_output(gpio_rst,0);
	mdelay(40);
	gpio_direction_output(gpio_rst,1);
	mdelay(5);
	return 0;
}
#endif
static u8 cs4398_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;
	struct i2c_client *client;

	if(!cs4398_data->client)
	{
		printk(KERN_ERR "%s i2c client is NULL\n",__func__);
		return -EINVAL;
	}

	client = cs4398_data->client;
	ret = i2c_master_send(client,&reg,1);
	if(ret < 0)
	{

		dev_err(&client->dev, "%s i2c send cmd error reg=%d \n",__func__,reg);
		return ret;

	}
	ret = i2c_master_recv(client,&buf,1);
	if(ret <0 )
	{
		dev_err(&client->dev,"%s i2c recv error \n ",__func__);
		return ret;
	}
	return buf;
}
static int cs4398_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	struct i2c_client *client;
	u8 cmd[2];

	cmd[0]=reg;
	cmd[1]=data;

	if(!cs4398_data->client)
	{
		printk(KERN_ERR "%s i2c client is NULL \n",__func__);
		return -EINVAL;
	}

	client = cs4398_data->client;
	ret = i2c_master_send(client,cmd,sizeof(cmd));
	if(ret < 0)
	{
		printk(KERN_ERR "%s i2c master send error\n",__func__);
	}

	return ret;
}

#ifdef BBK_VIVO_AUDIO_DEBUG

static int cs4398_debugfs_open(struct inode *inode, struct file *pfile)
{
	return 0;
}
static ssize_t cs4398_debugfs_read(struct file *pfile, char __user *buf, size_t count, loff_t *pos)
{
	int size = 512;
	char kbuf[size];
	u8 data;
	int i = 0;
	int n =0;
//	if (function_cs4398->power_up)
//		function_cs4398->power_up(VIVO_CODEC_HIFI_DAC);
//	if (function_cs4398->hw_reset)
//		function_cs4398->hw_reset(VIVO_CODEC_HIFI_DAC);
	for(i=0;i<sizeof(cs4398_reg);i++)
	{
		printk("i=%d %s\n",i,__func__);
		data = cs4398_i2c_read_byte(cs4398_reg[i]);
		n += scnprintf(kbuf+n,size-n,"reg[%x]=%x \n",cs4398_reg[i],data);

	}
	kbuf[n]='\0';

	return simple_read_from_buffer(buf,count,pos,kbuf,n);
}

static ssize_t cs4398_debugfs_write (struct file *pfile, const char __user *buf, size_t count, loff_t *pos)
{
	int rc;
	unsigned int kbuf[2];
	rc = sscanf(buf,"%x %x",&kbuf[0],&kbuf[1]);
	if(!rc)
		return -EFAULT;

	printk(KERN_INFO "kbuf[0]=%x, kbuf[1]=%x %s\n",kbuf[0],kbuf[1],__func__);

	(void)cs4398_i2c_write_byte(kbuf[0],kbuf[1]);

	return count;
}


struct file_operations cs4398_debugfs_fops = {
	.open = cs4398_debugfs_open,
	.read = cs4398_debugfs_read,
	.write = cs4398_debugfs_write,
};

static ssize_t cs4398_i2c_read(struct file *pfile,
			char __user *buf, size_t count, loff_t *pos)
{
	int size = 512;
	char kbuf[size];
	int n =0;
	u8 chipid =0;
	if (!function_cs4398) {
		printk(KERN_ERR "%s vivo_codec_function NULL\n",__func__);
		chipid = 0;
		goto exit;
	}
	/* we donn't consider the situation that music is playing */
	if (function_cs4398->power_up)
		function_cs4398->power_up(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->hw_reset)
		function_cs4398->hw_reset(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->mclk_enable)
		function_cs4398->mclk_enable(VIVO_CODEC_HIFI_CLK, 44100);

	chipid = cs4398_i2c_read_byte(CS4398_CHIP_ID_REG);
	if(CS4398_ID != chipid)
	{
		printk("cs4398 id is %x read error\n",chipid);
		chipid = 0;
	}else{
		printk("cs4398 read id successfully: 0x%d\n",chipid);
	}

	if (function_cs4398->power_down)
		function_cs4398->power_down(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->mclk_disable)
		function_cs4398->mclk_disable(VIVO_CODEC_HIFI_CLK);

exit:
	n += scnprintf(kbuf+n,size-n,"cs4398 i2c read %s \n",
						chipid?"OK":"ERROR");

	kbuf[n]= 0;

	return simple_read_from_buffer(buf,count,pos,kbuf,n);
}

struct file_operations cs4398_i2c_fops = {
	.open = cs4398_debugfs_open,
	.read = cs4398_i2c_read,
	.write = cs4398_debugfs_write,
};

static int cs4398_debugfs_exit(void)
{
	struct dentry *debugfs_i2c;
	struct dentry *debugfs_reg;
	struct dentry *debugfs_root;

	debugfs_i2c = cs4398_data->cs4398_debugfs_i2c;
	debugfs_reg = cs4398_data->cs4398_debugfs_reg;
	debugfs_root = cs4398_data->cs4398_debugfs_root;

	debugfs_remove(debugfs_i2c);
	debugfs_remove(debugfs_reg);
	debugfs_remove(debugfs_root);

	return 0;
}
static int cs4398_debugfs_init(void)
{
	struct dentry *debugfs_root;
	struct dentry *debugfs_reg;
	struct dentry *debugfs_i2c;

	printk(KERN_INFO "%s \n",__func__);

	debugfs_root = cs4398_data->cs4398_debugfs_root;
	debugfs_reg = cs4398_data->cs4398_debugfs_reg;
	debugfs_i2c = cs4398_data->cs4398_debugfs_i2c;

	debugfs_root = debugfs_create_dir("cs4398",NULL);
	if(NULL == debugfs_root){
		printk(KERN_ERR "%s create cs4398 debugfs root failed! \n",__func__);
	}else if(IS_ERR(debugfs_root)){
		printk(KERN_ERR "%s kernle not support debugfs\n",__func__);
		debugfs_root = NULL;
	}

	debugfs_reg = debugfs_create_file("reg",0644,debugfs_root,NULL,&cs4398_debugfs_fops);
	if(NULL == debugfs_reg){
		printk(KERN_ERR "%s create cs4398 debugfs file failed \n",__func__);
	}

	debugfs_i2c = debugfs_create_file("i2c",0444,debugfs_root,NULL,&cs4398_i2c_fops);
	if(NULL == debugfs_i2c){
		printk(KERN_ERR "%s create cs4398 i2c file failed \n",__func__);
	}

	return 0;
}
#endif
#if 0
void cs4398_digital_power(int on){
	printk("cs4398_digital_power with = %d\n",on);
	if(on){
		if (cs4398_data->vd_type)
		  cs4398_power_on(cs4398_data->vd_regulator,true);

		else
			gpio_direction_output(cs4398_data->vd_ctrl_gpio,1);

		if (cs4398_data->va_type)
		  cs4398_power_on(cs4398_data->va_regulator,true);

		else
			gpio_direction_output(cs4398_data->va_ctrl_gpio,1);

		if (cs4398_data->v5v_type)
		  cs4398_power_on(cs4398_data->v5v_regulator,true);

		else
			gpio_direction_output(cs4398_data->v5v_ctrl_gpio,1);

	}else{
		if (cs4398_data->vd_type)
		  cs4398_power_on(cs4398_data->vd_regulator,false);

		else
			gpio_direction_output(cs4398_data->vd_ctrl_gpio,0);

		if (cs4398_data->va_type)
		  cs4398_power_on(cs4398_data->va_regulator,false);

		else
			gpio_direction_output(cs4398_data->va_ctrl_gpio,0);

		if (cs4398_data->v5v_type)
		  cs4398_power_on(cs4398_data->v5v_regulator,false);

		else
			gpio_direction_output(cs4398_data->v5v_ctrl_gpio,0);
	}
}
#endif
int cs4398_mute(bool enable)
{
	if(!cs4398_available)
		return 0;
	if(enable)
	{
		cs4398_i2c_write_byte(CS4398_VOL_MIX_INV_CTL_REG, 0);
		cs4398_i2c_write_byte(CS4398_A_VOL_CTL_REG, 0xff);
		cs4398_i2c_write_byte(CS4398_B_VOL_CTL_REG, 0xff);
		cs4398_i2c_write_byte(CS4398_RAMP_FILTER_CTL_REG, 0x70);
		cs4398_i2c_write_byte(CS4398_MUTE_CTL_REG, 0xd8);
		return 1;
	}
	else
	{
		cs4398_i2c_write_byte(CS4398_VOL_MIX_INV_CTL_REG, 9);
		cs4398_i2c_write_byte(CS4398_A_VOL_CTL_REG, 0);
		cs4398_i2c_write_byte(CS4398_B_VOL_CTL_REG, 0);
		cs4398_i2c_write_byte(CS4398_MUTE_CTL_REG, 0xc0);
		cs4398_i2c_write_byte(CS4398_RAMP_FILTER_CTL_REG, 0xc0);
	}
	return 0;
}

int cs4398_enable(struct audio_params *params, int enable)
{
	if(!cs4398_available)
	{
		printk("cs4398_enable cs4398 is not available\n");
		return 0;
	}

	if (!params) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}
#ifdef CONFIG_VIVO_REGDUMP
	cs4398_powered = enable;
#endif
	if(enable)
	{
		if (function_cs4398->power_up)
			function_cs4398->power_up(VIVO_CODEC_HIFI_DAC);
		if (function_cs4398->hw_reset)
			function_cs4398->hw_reset(VIVO_CODEC_HIFI_DAC);

		msleep(20);

		if (params->rate < 100000)
			cs4398_i2c_write_byte(CS4398_MODE_CTL_REG, 0x10);
		else if (params->rate >= 100000 && params->rate < 200000)
			cs4398_i2c_write_byte(CS4398_MODE_CTL_REG, 0x12);
		else
			pr_err("%s:params->rate %d error", __func__, params->rate);

		cs4398_i2c_write_byte(CS4398_MISC_CTL_REG, 0x40);
		return 1;
	}
	else
	{
		cs4398_i2c_write_byte(CS4398_MISC_CTL_REG, 0xc0);
		if (function_cs4398->hw_reset)
			function_cs4398->hw_reset(VIVO_CODEC_HIFI_DAC_RST_DOWN);
		if (function_cs4398->power_down)
			function_cs4398->power_down(VIVO_CODEC_HIFI_DAC);
	}
	return 0;
}

int cs4398_dac_enable(struct audio_params *params, bool enable)
{
	/* Skip the format,and rate */
	printk("cs4398_dac_enable with enable = %d\n",enable);
	cs4398_enable(params, enable);

	return 0;
}

int cs4398_dac_mute(bool mute)
{
	cs4398_mute(mute);

	return 0;
}

#ifdef CONFIG_VIVO_REGDUMP
void cs4398_regdump_callback(void)
{
	u8 data;
	int i;
	pr_info("%s() enter\n", __func__);
	
	if(cs4398_powered){
		for(i=0;i<sizeof(cs4398_reg_dump);i++)
		{
			data = cs4398_i2c_read_byte(cs4398_reg_dump[i]);
			pr_info("%s():reg[%02x]= %02x \n",__func__,cs4398_reg_dump[i],data);
		}
	}else{
		pr_info("%s():cs4398 power off\n",__func__);
	}
	pr_info("%s() leave\n", __func__);
	return;
}

static struct vivo_regdump_handler cs4398_regdump_handler = {
	.name = "cs4398",
	.callback = cs4398_regdump_callback,
};
#endif


static int cs4398_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret = 0;
	u8 chipid =0;
	struct cs4398_data *dev_data = NULL;
	struct cs4398_device_platform_data *pdata = client->dev.platform_data;

#ifdef BBK_VIVO_AUDIO_DEBUG
	cs4398_debugfs_init(); 
#endif

	function_cs4398 = get_vivo_codec_function();
	if (!function_cs4398) {
		dev_err(&client->dev, "%s vivo_codec_function is NULL\n",
			__func__);
		return -EINVAL;
	}

	dev_info(&client->dev,"%s cs i2c probe \n",__func__);
	if(!pdata)
	{
		dev_err(&client->dev,"%s pdata is NULL \n",__func__);
	}
	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
	{
		dev_err(&client->dev,"%s i2c check functionality failed \n",__func__);
	}

	dev_data = kzalloc(sizeof(struct cs4398_data), GFP_KERNEL);
	if(!dev_data)
	{
		dev_err(&client->dev,"%s kzalloc failed\n",__func__);
		return -ENOMEM;
	}

	dev_data->driver_name = CS4398_I2C_NAME;
	dev_data->client = client;

	cs4398_data = dev_data;
#if 0
	if (client->dev.of_node) {
		of_property_read_u32(client->dev.of_node,"bbk,vd-type",&cs4398_data->vd_type);
		of_property_read_u32(client->dev.of_node,"bbk,va-type",&cs4398_data->va_type);
		of_property_read_u32(client->dev.of_node,"bbk,v5v-type",&cs4398_data->v5v_type);
		printk("%s:%d,%d,%d\n",__func__,cs4398_data->vd_type,cs4398_data->va_type,cs4398_data->v5v_type);
		/* Done with vd 1.8V */
		if (cs4398_data->vd_type) {
			/* Use regulator */
			cs4398_data->vd_regulator = regulator_get(&client->dev,"ext_dac_vd");
			if (IS_ERR(cs4398_data->vd_regulator)) {
				printk(KERN_ERR"Get ext_dac_vd for CS4398 error\n");
				kfree(dev_data);
				return PTR_ERR(cs4398_data->vd_regulator);
			}
			/* Set to 1.8V */
			if (regulator_count_voltages(cs4398_data->vd_regulator) > 0) {
                ret = regulator_set_voltage(cs4398_data->vd_regulator,1800000,1800000);
                if (ret) {
                    dev_err(&client->dev,"regulator set_vtg failed rc=%d\n", ret);
					kfree(dev_data);
                    regulator_put(cs4398_data->vd_regulator);
                    return ret;
                }
			}
		} else {
			/* Use gpio to control */
			cs4398_data->vd_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-vd-enable-gpio",0);
			gpio_request(cs4398_data->vd_ctrl_gpio, "hifi-vd-ctrl-gpio");
			pr_info("%s:vd_ctrl_gpio = %d\n", __func__, cs4398_data->vd_ctrl_gpio);
		}


		if (cs4398_data->va_type) {
			/* Use regulator */
			cs4398_data->va_regulator = regulator_get(&client->dev,"ext_dac_va");
			if (IS_ERR(cs4398_data->va_regulator)) {
				printk(KERN_ERR"Get ext_dac_va for CS4398 error\n");
				kfree(dev_data);
				return PTR_ERR(cs4398_data->va_regulator);
			}
			/* Set to 3.3V */
			if (regulator_count_voltages(cs4398_data->va_regulator) > 0) {
                ret = regulator_set_voltage(cs4398_data->va_regulator,3300000,3300000);
                if (ret) {
                    dev_err(&client->dev,"regulator set_vtg failed rc=%d\n", ret);
					kfree(dev_data);
                    regulator_put(cs4398_data->va_regulator);
                    return ret;
                }
			}
		} else {
			/* Use gpio to control */
			cs4398_data->va_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-va-enable-gpio",0);
			gpio_request(cs4398_data->va_ctrl_gpio, "hifi-va-ctrl-gpio");
			pr_info("%s:vd_ctrl_gpio = %d\n", __func__, cs4398_data->va_ctrl_gpio);
		}
		if (cs4398_data->v5v_type) {
			/* Use regulator */
			cs4398_data->v5v_regulator = regulator_get(&client->dev,"ext_dac_v5v");
			if (IS_ERR(cs4398_data->v5v_regulator)) {
				printk(KERN_ERR"Get ext_dac_v5v for CS4398 error\n");
				kfree(dev_data);
				return PTR_ERR(cs4398_data->v5v_regulator);
			}
			/* Set to 5V */
			if (regulator_count_voltages(cs4398_data->v5v_regulator) > 0) {
                ret = regulator_set_voltage(cs4398_data->v5v_regulator,5000000,5000000);
                if (ret) {
                    dev_err(&client->dev,"regulator set_vtg failed rc=%d\n", ret);
					kfree(dev_data);
                    regulator_put(cs4398_data->v5v_regulator);
                    return ret;
                }
			}
		} else {
			/* Use gpio to control */
			cs4398_data->v5v_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-v5v-enable-gpio",0);
			gpio_request(cs4398_data->v5v_ctrl_gpio, "hifi-v5v-ctrl-gpio");
			pr_info("%s:v5v_ctrl_gpio = %d\n", __func__, cs4398_data->v5v_ctrl_gpio);
		}

		cs4398_data->rest_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-rst-gpio",0);
		if (cs4398_data->rest_ctrl_gpio < 0) {
			dev_err(&client->dev,"Get gpio for dac reset error!!\n");
            return -EINVAL;
		}

		/* For reset control */
		ret = gpio_request(cs4398_data->rest_ctrl_gpio,"cs4398_reset_gpio");
		if (ret) {
			dev_err(&client->dev,"Request cs4398_reset_gpio error = %d!!\n",ret);
            return ret;
		}
	}else {
		dev_err(&client->dev,"Have no node!!\n");
	}
	/* Add interface for HI-FI control */
#endif

	if (function_cs4398->power_up)
		function_cs4398->power_up(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->hw_reset)
		function_cs4398->hw_reset(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->mclk_enable)
		function_cs4398->mclk_enable(VIVO_CODEC_HIFI_CLK, 44100);
    function_cs4398->hifi_dac_enable = cs4398_dac_enable;
    function_cs4398->hifi_dac_mute = cs4398_dac_mute;


	chipid = cs4398_i2c_read_byte(CS4398_CHIP_ID_REG);
	if(CS4398_ID != chipid)
	{
		dev_err(&client->dev,"%s cs4398 id is %x read error\n",__func__,chipid);
	}else{
		printk("cs4398_i2c_probe read id successfully: 0x%d\n",chipid);
	}
	cs4398_available=true;

	if (function_cs4398->power_down)
		function_cs4398->power_down(VIVO_CODEC_HIFI_DAC);
	if (function_cs4398->mclk_disable)
		function_cs4398->mclk_disable(VIVO_CODEC_HIFI_CLK);
#ifdef CONFIG_VIVO_REGDUMP
	vivo_audio_regdump_register_callback(&cs4398_regdump_handler);
#endif

	return ret;
}

static int cs4398_i2c_suspend(struct i2c_client *client,pm_message_t pmeg)
{
	return 0;

}
static int cs4398_i2c_resume(struct i2c_client *client)
{
#ifdef BBK_VIVO_AUDIO_DEBUG
	(void)cs4398_debugfs_exit();
#endif
	return 0;
}
static int cs4398_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_VIVO_REGDUMP
		vivo_audio_regdump_deregister_callback(cs4398_regdump_handler.name);
#endif
	return 0;
}

static const struct i2c_device_id cs4398_i2c_id_table[] = {
		{CS4398_I2C_NAME,0},
		{ }
};

#ifdef CONFIG_OF
static const struct of_device_id cs4398_match_table[] = {
    {.compatible = "cs,cs4398",},
    {},
};
#else
#define cs4398_match_table 0
#endif

static struct i2c_driver cs4398_i2c_driver = {
	.probe = cs4398_i2c_probe,
	.suspend = cs4398_i2c_suspend,
	.resume = cs4398_i2c_resume,
	.remove = cs4398_i2c_remove,
	.driver ={
		.name = CS4398_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cs4398_match_table,
	},
	.id_table = cs4398_i2c_id_table,
};
static int __init cs4398_dac_init(void)
{
	int ret  = 0;

	ret = i2c_add_driver(&cs4398_i2c_driver);
	if(ret < 0)
	{
		printk(KERN_ERR "%s i2c add driver error\n",__func__);
	}

	return ret;
}
static void __exit cs4398_dac_exit(void)
{
	i2c_del_driver(&cs4398_i2c_driver);
	return;
}

module_init(cs4398_dac_init);
module_exit(cs4398_dac_exit);
MODULE_DESCRIPTION("cs4398 DAC driver");
MODULE_AUTHOR("liuxudong <liuxudong@vivo.com>");
MODULE_LICENSE("GPL");