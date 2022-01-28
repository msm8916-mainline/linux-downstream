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
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/clk.h>

#include "vivo-codec-common.h"
#include "es9018-common.h"

#define ES9018_DEV_NAME "es9018"
#define ES9018_PHYS_NAME "es9018_phys_dev"
static bool es9018_available = false;

enum
{
    ES9018_VERSION_W,
	ES9018_VERSION_V,
	ES9018_VERSION_NONE
}ES9018_VERSION;

int es9018_version = ES9018_VERSION_NONE;

static unsigned char dac_parameters[][2] = {
	{0,0x00},
	{1,0xcc},
	{2,0x18},
	{3,0x10},
	{4,0x00},
	{5,0x68},
	{6,0x4a},
	{7,0x80},
	{8,0x70},
	{9,0x22},
	{10,0x2d},
	{11,0x02},
	{13,0x0},
	{14,0x8a},
	{20,0x78},
	{22,0x0},
	{23,0xfe},
	{24,0x0},
	{25,0xed},
};

#ifdef BBK_VIVO_AUDIO_DEBUG
static struct dentry *es9018_debugfs_root;
static struct dentry *es9018_debugfs_reg;

static u8 es9018_regs[] = {
	ES9018_CONTROL_REG00,
	ES9018_CONTROL_REG01,
	ES9018_CONTROL_REG02,
	ES9018_CONTROL_REG03,
	ES9018_CONTROL_REG04,
	ES9018_CONTROL_REG05,
	ES9018_CONTROL_REG06,
	ES9018_CONTROL_REG07,
	ES9018_CONTROL_REG08,
	ES9018_CONTROL_REG09,
	ES9018_CONTROL_REG10,
	ES9018_CONTROL_REG11,
	ES9018_CONTROL_REG12,
	ES9018_CONTROL_REG13,
	ES9018_CONTROL_REG14,
	ES9018_CONTROL_REG15,
	ES9018_CONTROL_REG16,
	ES9018_CONTROL_REG20,
	ES9018_CONTROL_REG22,
	ES9018_CONTROL_REG23,
	ES9018_CONTROL_REG24,
	ES9018_CONTROL_REG25,
	ES9018_CONTROL_REG64,
	ES9018_CONTROL_REG65,
};
#endif

struct es9018_data {
	struct i2c_client *client;
	char *driver_name;

	struct mutex lock;

	struct vivo_codec_function *fun;
};
static struct es9018_data *es9018_data;

static u8 es9018_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(es9018_data->client,&reg,1);
	if(ret < 0)
	{
		dev_err(&es9018_data->client->dev, "%s i2c send cmd error reg=%d \n",__func__,reg);
		return ret;
	}
	ret = i2c_master_recv(es9018_data->client,&buf,1);
	if(ret <0 )
	{
		dev_err(&es9018_data->client->dev,"%s i2c recv error \n ",__func__);
		return ret;
	}

	return buf;

}
static u8 es9018_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(es9018_data->client,cmd,sizeof(cmd));
	if (ret < 1)
	{
		dev_err(&es9018_data->client->dev,"%s i2c send error cmd[0]=%d,cmd[1]=%d\n",__func__,cmd[0],cmd[1]);
	}

	return ret;
}

static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = es9018_i2c_read_byte(reg);
	while((val != reg_val)&&(retry > 0)){
		es9018_i2c_write_byte(reg, val);
		reg_val = es9018_i2c_read_byte(reg);
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

int es9018_enable(struct audio_params *params, bool enable)
{
	struct es9018_private_params *es9018_params;
	u8 reg_val, regval_20, regval_23, regval_25;
	int para_num = sizeof(dac_parameters) / 2;
	int i, clk_div;
	int ret = 0;

	if (es9018_data->client == NULL)
	{
		dev_err(&es9018_data->client->dev,"%s client is NULL \n ",__func__);
		return -EFAULT;
	}
	if(!es9018_available)
		return 0;

	if (!params) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}

	pr_err("%s:enable %d, pcm_format 0x%x, params->i2s_format 0x%x\n", __func__,
		enable, params->pcm_format, params->i2s_format);

	mutex_lock(&es9018_data->lock);

	if(enable)
	{
		if (es9018_data->fun && es9018_data->fun->power_up)
			es9018_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
		usleep(5000);
		if (es9018_data->fun && es9018_data->fun->hw_reset)
			es9018_data->fun->hw_reset(VIVO_CODEC_HIFI_DAC);
		usleep(1000 * 20);

		for (i = 0; i < para_num; i++) {
			es9018_i2c_write_byte(dac_parameters[i][0], dac_parameters[i][1]);
			ret = do_reg_write_check(dac_parameters[i][0], dac_parameters[i][1]);
			if(ret < 0)
				goto end;
		}

		/* Set Volume */
		regval_20 = 0x7f;
		if (es9018_version == ES9018_VERSION_W) {
			regval_25 = 0x02;
		} else {
			regval_23 = 0x01;
			regval_25 = 0x01;
		}

		/* Get private params */
		if (params->private_params) {
			es9018_params = (struct es9018_private_params *)params->private_params;
			regval_20 = es9018_params->regval_20;
			regval_23 = es9018_params->regval_23;
			regval_25 = es9018_params->regval_25;

			pr_err("%s: private params 0x%x, 0x%x, 0x%x\n", __func__,
				regval_20, regval_23, regval_25);
		}

		es9018_i2c_write_byte(0x14, regval_20);
		ret = do_reg_write_check(0x14, regval_20);
		if (ret < 0) goto end;

		es9018_i2c_write_byte(0x17, regval_23);
		ret = do_reg_write_check(0x17, regval_23);
		if(ret < 0) goto end;
		
		es9018_i2c_write_byte(0x19, regval_25);
		ret = do_reg_write_check(0x19, regval_25);
		if(ret < 0) goto end;

		/* Set I2S add by ChenJinQuan*/
		reg_val = es9018_i2c_read_byte(1);
#if 0
		/* input select to i2s, i2s mode set to i2s.  */
		reg_val &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 4));

		reg_val &= ~(0x3 << 6);
		if ((params->i2s_format &
			SND_SOC_DAIFMT_MASTER_MASK) ==
			SND_SOC_DAIFMT_CBM_CFM) {
			/* In the master mode, i2s length should be 32bit */
			reg_val |= 3 << 6;
		} else {
			switch (params->pcm_format) {
			case SNDRV_PCM_FORMAT_S16_LE:
				reg_val |= 0 << 6;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				reg_val |= 1 << 6;
				break;
			default:
				pr_err("%s:format error %d\n", __func__,
					params->pcm_format);
				return -EINVAL;
			}
		}
#else
		/* I2S mode, 32bit */
		reg_val = 3 << 6;
#endif
		es9018_i2c_write_byte(1, reg_val);

		if (es9018_version == ES9018_VERSION_V) {
			reg_val = es9018_i2c_read_byte(10);
		} else if (es9018_version == ES9018_VERSION_W) {
			reg_val = es9018_i2c_read_byte(9);
		}

		reg_val &= ~(0x1 << 7);
		if ((params->i2s_format &
			SND_SOC_DAIFMT_MASTER_MASK) ==
			SND_SOC_DAIFMT_CBM_CFM) {

			/* Set master */
			reg_val |= (0x1 << 7);

			/* Set divider
			  * LRCK = BCLK / 64, BCLK = MCLK / n
			  * when divider is 0, n = 4.
			  * when divider is 1, n = 8.
			  * when divider is 2 or 3, n = 16.
			  */
			reg_val &= ~(0x3 << 5);
			clk_div = params->sys_clk / params->rate / 64;

			if ((params->sys_clk % params->rate) == 0 &&
				(params->sys_clk / params->rate % 64) == 0) {
				if (clk_div == 4) {
					reg_val |= 0x0 << 5;
				} else if (clk_div == 8) {
					reg_val |= 0x1 << 5;
				} else if (clk_div == 16) {
					reg_val |= 0x2 << 5;
				} else
					pr_err("%s:error div %d", __func__, clk_div);
			} else
				pr_err("%s: error sys_clk %lu and rate %d",
					__func__, params->sys_clk, params->rate);
		}

		if (es9018_version == ES9018_VERSION_V) {
			reg_val = es9018_i2c_write_byte(10, reg_val);
		} else if (es9018_version == ES9018_VERSION_W) {
			reg_val = es9018_i2c_write_byte(9, reg_val);
		}
		/* Set I2S end, add by ChenJinQuan */

		ret = 0;
	} else {
		//mute dac first
		es9018_i2c_write_byte(0x6, 0x44);
		es9018_i2c_write_byte(0x4, 0x10);

		ret = do_reg_write_check(0x4, 0x10);
		if (ret < 0) goto end;

		reg_val = es9018_i2c_read_byte(7);
		reg_val = reg_val | 0x3;
		es9018_i2c_write_byte(0x7, reg_val);
		ret = do_reg_write_check(0x7, reg_val);
		if (ret < 0) goto end;
		msleep(50);
		es9018_i2c_write_byte(0x6, 0x42);
		msleep(15);
		es9018_i2c_write_byte(0xe, 0x03);
		ret = do_reg_write_check(0xe, 0x03);
		if (ret < 0) goto end;

		msleep(5);

		if (es9018_data->fun && es9018_data->fun->power_down)
			es9018_data->fun->power_down(VIVO_CODEC_HIFI_DAC);

		ret = 0;
	}
end:
	mutex_unlock(&es9018_data->lock);
	return ret;
}

int es9018_mute(bool mute)
{
	int reg_val = 0;

	pr_info("%s:mute = %d\n",__func__, mute);

	reg_val = es9018_i2c_read_byte(7);
	if (mute) {
		es9018_i2c_write_byte(14, 0x03);
		reg_val = reg_val | 0x3;
		msleep(20);
	} else {
		reg_val = reg_val & 0xfc;
		es9018_i2c_write_byte(14, 0x89);
	}

	//es9018_i2c_write_byte(7, reg_val);

	return 0;
}

static int es9018_phys_open(struct inode *inode, struct file *filep)
{
	filep->private_data = inode->i_private;
	return 0;
}
int es9018_phys_release (struct inode *inode, struct file *filep)
{
	int ret =0;

	return ret;
}
long es9018_phys_ioctl (struct file *filep, unsigned int cmd, unsigned long args)
{
	int ret = 0;

	printk(KERN_ALERT "%s \n",__func__);

	return (long)ret;
}


static struct file_operations es9018_phys_fops = {
	.open = es9018_phys_open,
	.unlocked_ioctl = es9018_phys_ioctl,
	.release = es9018_phys_release,
};

static struct miscdevice es9018_phys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ES9018_PHYS_NAME,
	.fops = &es9018_phys_fops,
};
#ifdef BBK_VIVO_AUDIO_DEBUG
static int es9018_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t es9018_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];

	ret = sscanf(ubuf,"%x %x",&kbuf[0],&kbuf[1]);
	if(!ret)
		return -EFAULT;

#ifdef CONFIG_ARM64
	pr_err("kbuf[0] = 0x%x,kbuf[1]= 0x%x cnt = %lu %s\n",
		kbuf[0], kbuf[1], cnt, __func__);
#else
	pr_err("kbuf[0] = 0x%x,kbuf[1]= 0x%x cnt = %u %s\n",
		kbuf[0], kbuf[1], cnt, __func__);
#endif
	if (es9018_data->fun && es9018_data->fun->power_up)
		es9018_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
	msleep(5);

	es9018_i2c_write_byte(kbuf[0],kbuf[1]);
	do_reg_write_check(kbuf[0],kbuf[1]);

	return cnt;
}

static ssize_t es9018_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	if (es9018_data->fun && es9018_data->fun->power_up)
		es9018_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
	msleep(5);

	for(i = 0;i < sizeof(es9018_regs);i++)
	{
		data = es9018_i2c_read_byte(es9018_regs[i]);
		n += scnprintf(buffer+n,size-n,"reg{0x%x}:0x%x \n",es9018_regs[i],data);
	}

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations es9018_debugfs_fops = {
	.open = es9018_debug_open,
	.read = es9018_debug_read,
	.write = es9018_debug_write,
};
#endif

static int es9018_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *function;
	int ret = 0;
	u8 chipid = 0x0;
	int retry = 5;

	function = get_vivo_codec_function();
	if (!function) {
		dev_err(&client->dev, "%s vivo_codec_function is NULL\n",
			__func__);
		return -EINVAL;
	}

    es9018_data = kzalloc(sizeof(struct es9018_data), GFP_KERNEL);
	es9018_data->client = client;
	es9018_data->driver_name = ES9018_DEV_NAME;
	es9018_data->fun = function;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s i2c check funtion error \n",
			__func__);
	}

	if (function->power_up)
		function->power_up(VIVO_CODEC_HIFI_DAC);
	if (function->hw_reset)
		function->hw_reset(VIVO_CODEC_HIFI_DAC);

	chipid = es9018_i2c_read_byte(ES9018_ID_REG);
	pr_err("%s:chip id = 0x%x\n", __func__, chipid);

	while (retry--) {
		if (ES9018_CHIP_ID != (chipid & 0x1c)) {
			msleep(5);
			chipid = es9018_i2c_read_byte(ES9018_ID_REG);
		}
		else break;
	}

	if (ES9018_CHIP_ID != (chipid & 0x1c)) {
		dev_err(&client->dev, "%s chip id error 0x%x\n",
			__func__, chipid);
	} else {
		if (chipid & 0x20) {
			es9018_version = ES9018_VERSION_V;
			dev_err(&client->dev, "%s ES9018_VERSION_V %x\n",
				__func__,chipid);
		} else {
			es9018_version = ES9018_VERSION_W;
			dev_err(&client->dev, "%s ES9018_VERSION_W %x\n",
				__func__, chipid);
		}
        es9018_available = true;
		function->hifi_dac_enable = es9018_enable;
		function->hifi_dac_mute = es9018_mute;

		mutex_init(&es9018_data->lock);
	}

	es9018_i2c_write_byte(0x0e, 0x6a);

	msleep(10);

	if (function->power_down)
		function->power_down(VIVO_CODEC_HIFI_DAC);

	ret = misc_register(&es9018_phys_dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s es9018 phys misc device register error\n",
			__func__);
		goto err_gpio;
	}
err_gpio:

#ifdef BBK_VIVO_AUDIO_DEBUG
	es9018_debugfs_root = debugfs_create_dir("es9018", NULL);
	if (!es9018_debugfs_root) {
		dev_err(&client->dev,"%s debugfs create dir error\n",
			__func__);
	} else if(IS_ERR(es9018_debugfs_root)) {
		dev_err(&client->dev,"%s Kernel not support debugfs \n",
			__func__);
		es9018_debugfs_root = NULL;
	}

	es9018_debugfs_reg = debugfs_create_file(
			"reg", 0644, es9018_debugfs_root,
			NULL, &es9018_debugfs_fops);
	if (!es9018_debugfs_reg) {
		dev_err(&client->dev, "es9018 debugfs create fail \n");
	}
#endif

	return ret;

}
static int es9018_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int es9018_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static int es9018_i2c_remove(struct i2c_client *client)
{
	int ret = 0;

	debugfs_remove(es9018_debugfs_reg);
	debugfs_remove(es9018_debugfs_root);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id device_es9018_of_match[] = {
    {.compatible = "ess,es9018-2m",},
    {},
};
#else
#define device_es9018_of_match 0
#endif

static const struct i2c_device_id es9018_i2c_id[] = {
	{ ES9018_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver es9018_i2c_driver = {
    .probe              = es9018_i2c_probe,
	.suspend 			= es9018_i2c_suspend,
	.resume 			= es9018_i2c_resume,
    .remove             = es9018_i2c_remove,
    .driver = {
        .name           = ES9018_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = device_es9018_of_match,
    },
    .id_table = es9018_i2c_id,
};

static int __init es9018_dac_init(void)
{
	 if(i2c_add_driver(&es9018_i2c_driver))
	 {
	 	printk(KERN_ERR " %s add i2c driver error\n ",__func__);
	 }
     return 0;
}

static void __exit es9018_dac_exit(void)
{
    i2c_del_driver(&es9018_i2c_driver);

	return;
}

module_init(es9018_dac_init);
module_exit(es9018_dac_exit);

MODULE_DESCRIPTION("es9018 clock generator driver");
MODULE_AUTHOR("leichenji <leichenji@vivo.com>");
MODULE_LICENSE("GPL");
