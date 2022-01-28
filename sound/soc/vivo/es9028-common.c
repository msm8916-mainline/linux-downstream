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
#include "es9028-common.h"

#define ES9028_DEV_NAME "es9028"
#define ES9028_PHYS_NAME "es9028_phys_dev"
static bool es9028_available = false;

enum
{
    ES9028_VERSION_W,
	ES9028_VERSION_V,
	ES9028_VERSION_NONE
}ES9028_VERSION;

int es9028_version = ES9028_VERSION_NONE;

static unsigned char dac_parameters[][2] = {
	{0,0x00},
	{1,0x8C},
	{2,0x18},
	{3,0x10},
	{4,0x00},
	{5,0x68},
	{6,0x4a},
	{7,0x80},
	{8,0x10},
	{9,0x22},
	{10,0x02},
	{11,0x02}, //02
	{12,0x5A},
	{13,0x00},
	{14,0x8a},
	{15,0x00},
	{16,0x00},
	{17,0xff},
	{18,0xff},
	{19,0xff},
	{20,0x7f},
	{21,0x0},
	{22,0x0},
	{23,0x01},
	{24,0x0},
	{25,0xfa},
	{41, 0x07}, //add 7-3
	{42, 0x60}, //add 7-3
};

#ifdef BBK_VIVO_AUDIO_DEBUG
static struct dentry *es9028_debugfs_root;
static struct dentry *es9028_debugfs_reg;

static u8 es9028_regs[] = {
	ES9028_CONTROL_REG00,
	ES9028_CONTROL_REG01,
	ES9028_CONTROL_REG02,
	ES9028_CONTROL_REG03,
	ES9028_CONTROL_REG04,
	ES9028_CONTROL_REG05,
	ES9028_CONTROL_REG06,
	ES9028_CONTROL_REG07,
	ES9028_CONTROL_REG08,
	ES9028_CONTROL_REG09,
	ES9028_CONTROL_REG10,
	ES9028_CONTROL_REG11,
	ES9028_CONTROL_REG12,
	ES9028_CONTROL_REG13,
	ES9028_CONTROL_REG14,
	ES9028_CONTROL_REG15,
	ES9028_CONTROL_REG16,
	ES9028_CONTROL_REG17,
	ES9028_CONTROL_REG18,
	ES9028_CONTROL_REG19,
	ES9028_CONTROL_REG20,
	ES9028_CONTROL_REG21,
	ES9028_CONTROL_REG22,
	ES9028_CONTROL_REG23,
	ES9028_CONTROL_REG24,
	ES9028_CONTROL_REG25,
	ES9028_CONTROL_REG34,
	ES9028_CONTROL_REG35,
	ES9028_CONTROL_REG36,
	ES9028_CONTROL_REG37,
	ES9028_CONTROL_REG38,
	ES9028_CONTROL_REG64,
	ES9028_CONTROL_REG65,
	ES9028_CONTROL_REG42,
};
#endif

struct es9028_data {
	struct i2c_client *client;
	char *driver_name;

	struct mutex lock;

	struct vivo_codec_function *fun;
};
static struct es9028_data *es9028_data;

static u8 es9028_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(es9028_data->client,&reg,1);
	if(ret < 0)
	{
		dev_err(&es9028_data->client->dev, "%s i2c send cmd error reg=%d \n",__func__,reg);
		return ret;
	}
	ret = i2c_master_recv(es9028_data->client,&buf,1);
	if(ret <0 )
	{
		dev_err(&es9028_data->client->dev,"%s i2c recv error \n ",__func__);
		return ret;
	}

	return buf;

}
static u8 es9028_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(es9028_data->client,cmd,sizeof(cmd));
	if (ret < 1)
	{
		dev_err(&es9028_data->client->dev,"%s i2c send error cmd[0]=%d,cmd[1]=%d\n",__func__,cmd[0],cmd[1]);
	}

	return ret;
}

static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = es9028_i2c_read_byte(reg);
	while((val != reg_val)&&(retry > 0)){
		es9028_i2c_write_byte(reg, val);
		reg_val = es9028_i2c_read_byte(reg);
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

int es9028_enable(struct audio_params *params, bool enable)
{
	struct es9028_private_params *es9028_params;
	u8 reg_val, regval_20, regval_23, regval_25, regval_22, regval_24, regval_34, regval_35,regval_36,regval_37,regval_38;
	int para_num = sizeof(dac_parameters) / 2;
	int i, clk_div;
	int ret = 0;

	if (es9028_data->client == NULL)
	{
		dev_err(&es9028_data->client->dev,"%s client is NULL \n ",__func__);
		return -EFAULT;
	}
	if(!es9028_available)
		return 0;

	if (!params) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}

	pr_err("%s:enable %d, pcm_format 0x%x, params->i2s_format 0x%x\n", __func__,
		enable, params->pcm_format, params->i2s_format);

	mutex_lock(&es9028_data->lock);

	if(enable)
	{
		if (es9028_data->fun && es9028_data->fun->power_up)
			es9028_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
		usleep(5000);
		if (es9028_data->fun && es9028_data->fun->hw_reset)
			es9028_data->fun->hw_reset(VIVO_CODEC_HIFI_DAC);
		usleep(1000 * 20);

		for (i = 0; i < para_num; i++) {
			es9028_i2c_write_byte(dac_parameters[i][0], dac_parameters[i][1]);
			ret = do_reg_write_check(dac_parameters[i][0], dac_parameters[i][1]);
			if(ret < 0)
				goto end;
		}

		/* Set Volume */
		regval_20 = 0x7f;
		if (es9028_version == ES9028_VERSION_W) {
			regval_25 = 0x02;
		} else {
			regval_23 = 0x01;
			regval_25 = 0x01;
		}

		/* Get private params */
		if (params->private_params) {
			es9028_params = (struct es9028_private_params *)params->private_params;
			regval_20 = es9028_params->regval_20;
			regval_22 = es9028_params->regval_22;
			regval_23 = es9028_params->regval_23;
			regval_24 = es9028_params->regval_24;
			regval_25 = es9028_params->regval_25;

			regval_34 = es9028_params->regval_34;
			regval_35 = es9028_params->regval_35;
			regval_36 = es9028_params->regval_36;
			regval_37 = es9028_params->regval_37;
			regval_38 = es9028_params->regval_38;

			pr_err("legen-%s: private params 0x%x, 0x%x, 0x%x\n", __func__,
				regval_20, regval_23, regval_25);
		}

		es9028_i2c_write_byte(0x14, regval_20);
		ret = do_reg_write_check(0x14, regval_20);
		if (ret < 0) goto end;

		es9028_i2c_write_byte(0x16, regval_22);
		ret = do_reg_write_check(0x16, regval_22);
		if (ret < 0) goto end;

		es9028_i2c_write_byte(0x17, regval_23);
		ret = do_reg_write_check(0x17, regval_23);
		if(ret < 0) goto end;

		es9028_i2c_write_byte(0x18, regval_24);
		ret = do_reg_write_check(0x18, regval_24);
		if (ret < 0) goto end;
		
		es9028_i2c_write_byte(0x19, regval_25);
		ret = do_reg_write_check(0x19, regval_25);
		if(ret < 0) goto end;

		es9028_i2c_write_byte(0x22, regval_34);
		ret = do_reg_write_check(0x22, regval_34);
		if(ret < 0) goto end;
		
		es9028_i2c_write_byte(0x23, regval_35);
		ret = do_reg_write_check(0x23, regval_35);
		if(ret < 0) goto end;

		es9028_i2c_write_byte(0x24, regval_36);
		ret = do_reg_write_check(0x24, regval_36);
		if(ret < 0) goto end;

		es9028_i2c_write_byte(0x25, regval_37);
		ret = do_reg_write_check(0x25, regval_37);
		if(ret < 0) goto end;

		es9028_i2c_write_byte(0x26, regval_38);
		ret = do_reg_write_check(0x26, regval_38);
		if(ret < 0) goto end;


		/* Set I2S add by ChenJinQuan*/
		reg_val = es9028_i2c_read_byte(1);
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
		es9028_i2c_write_byte(1, reg_val);

		if (es9028_version == ES9028_VERSION_V) {
			reg_val = es9028_i2c_read_byte(10);
		} else if (es9028_version == ES9028_VERSION_W) {
			reg_val = es9028_i2c_read_byte(9);
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

		if (es9028_version == ES9028_VERSION_V) {
			reg_val = es9028_i2c_write_byte(10, reg_val);
		} else if (es9028_version == ES9028_VERSION_W) {
			reg_val = es9028_i2c_write_byte(9, reg_val);
		}

		//Bowen add for temp
		//es9028_i2c_write_byte(0x01,0x8c);
		//es9028_i2c_write_byte(0x0b,0x03);
		//es9028_i2c_write_byte(0x19,0xfa);
		//Bowen add end
		
		/* Set I2S end, add by ChenJinQuan */

		ret = 0;
	} else {
		//mute dac first
		es9028_i2c_write_byte(0x6, 0x44);
		es9028_i2c_write_byte(0x4, 0x10);

		ret = do_reg_write_check(0x4, 0x10);
		if (ret < 0) goto end;

		reg_val = es9028_i2c_read_byte(7);
		reg_val = reg_val | 0x3;
		es9028_i2c_write_byte(0x7, reg_val);
		ret = do_reg_write_check(0x7, reg_val);
		if (ret < 0) goto end;
		msleep(50);
		es9028_i2c_write_byte(0x6, 0x42);
		msleep(15);
		es9028_i2c_write_byte(0xe, 0x03);
		ret = do_reg_write_check(0xe, 0x03);
		if (ret < 0) goto end;

		msleep(5);

		if (es9028_data->fun && es9028_data->fun->power_down)
			es9028_data->fun->power_down(VIVO_CODEC_HIFI_DAC);

		ret = 0;
	}
end:
	mutex_unlock(&es9028_data->lock);
	return ret;
}

int es9028_mute(bool mute)
{
	int reg_val = 0;

	pr_info("%s:mute = %d\n",__func__, mute);

	reg_val = es9028_i2c_read_byte(7);
	if (mute) {
		es9028_i2c_write_byte(14, 0x03);
		reg_val = reg_val | 0x3;
		msleep(20);
	} else {
		reg_val = reg_val & 0xfc;
		es9028_i2c_write_byte(14, 0x89);
	}

	//es9028_i2c_write_byte(7, reg_val);

	return 0;
}

static int es9028_phys_open(struct inode *inode, struct file *filep)
{
	filep->private_data = inode->i_private;
	return 0;
}
int es9028_phys_release (struct inode *inode, struct file *filep)
{
	int ret =0;

	return ret;
}
long es9028_phys_ioctl (struct file *filep, unsigned int cmd, unsigned long args)
{
	int ret = 0;

	printk(KERN_ALERT "%s \n",__func__);

	return (long)ret;
}


static struct file_operations es9028_phys_fops = {
	.open = es9028_phys_open,
	.unlocked_ioctl = es9028_phys_ioctl,
	.release = es9028_phys_release,
};

static struct miscdevice es9028_phys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ES9028_PHYS_NAME,
	.fops = &es9028_phys_fops,
};
#ifdef BBK_VIVO_AUDIO_DEBUG
static int es9028_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t es9028_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];

	ret = sscanf(ubuf,"%x %x",&kbuf[0],&kbuf[1]);
	if(!ret)
		return -EFAULT;

	pr_err("kbuf[0] = 0x%x,kbuf[1]= 0x%x cnt = %lu %s\n",
		kbuf[0], kbuf[1], (long)cnt, __func__);

	if (es9028_data->fun && es9028_data->fun->power_up)
		es9028_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
	msleep(5);

	es9028_i2c_write_byte(kbuf[0],kbuf[1]);
	do_reg_write_check(kbuf[0],kbuf[1]);

	return cnt;
}

static ssize_t es9028_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	if (es9028_data->fun && es9028_data->fun->power_up)
		es9028_data->fun->power_up(VIVO_CODEC_HIFI_DAC);
	msleep(5);

	for(i = 0;i < sizeof(es9028_regs);i++)
	{
		data = es9028_i2c_read_byte(es9028_regs[i]);
		n += scnprintf(buffer+n,size-n,"reg{0x%x}:0x%x \n",es9028_regs[i],data);
	}

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations es9028_debugfs_fops = {
	.open = es9028_debug_open,
	.read = es9028_debug_read,
	.write = es9028_debug_write,
};
#endif

static int es9028_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
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

    es9028_data = kzalloc(sizeof(struct es9028_data), GFP_KERNEL);
	es9028_data->client = client;
	es9028_data->driver_name = ES9028_DEV_NAME;
	es9028_data->fun = function;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s i2c check funtion error \n",
			__func__);
	}

	if (function->power_up)
		function->power_up(VIVO_CODEC_HIFI_DAC);
	if (function->hw_reset)
		function->hw_reset(VIVO_CODEC_HIFI_DAC);

	chipid = es9028_i2c_read_byte(ES9028_ID_REG);
	pr_err("%s:chip id = 0x%x\n", __func__, chipid);

	while (retry--) {
		if (ES9028_CHIP_ID != (chipid & 0x1c)) {
			msleep(5);
			chipid = es9028_i2c_read_byte(ES9028_ID_REG);
		}
		else break;
	}

	//if (ES9028_CHIP_ID != (chipid & 0x1c)) {
		//dev_err(&client->dev, "%s chip id error 0x%x\n",
			//__func__, chipid);
	//} else {
		//if (chipid & 0x20) {
			es9028_version = ES9028_VERSION_V;
			//dev_err(&client->dev, "%s ES9028_VERSION_V %x\n",
				//__func__,chipid);
		//} else {
		//	es9028_version = ES9028_VERSION_W;
		//	dev_err(&client->dev, "%s ES9028_VERSION_W %x\n",
		//		__func__, chipid);
		//}
		//es9028_version = ES9028_VERSION_V; //legen
        	es9028_available = true;
		function->hifi_dac_enable = es9028_enable;
		function->hifi_dac_mute = es9028_mute;

		mutex_init(&es9028_data->lock);
	//}

	es9028_i2c_write_byte(0x0e, 0x6a);

	msleep(10);

	if (function->power_down)
		function->power_down(VIVO_CODEC_HIFI_DAC);

	ret = misc_register(&es9028_phys_dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s es9028 phys misc device register error\n",
			__func__);
		goto err_gpio;
	}
err_gpio:

#ifdef BBK_VIVO_AUDIO_DEBUG
	es9028_debugfs_root = debugfs_create_dir("es9028", NULL);
	if (!es9028_debugfs_root) {
		dev_err(&client->dev,"%s debugfs create dir error\n",
			__func__);
	} else if(IS_ERR(es9028_debugfs_root)) {
		dev_err(&client->dev,"%s Kernel not support debugfs \n",
			__func__);
		es9028_debugfs_root = NULL;
	}

	es9028_debugfs_reg = debugfs_create_file(
			"reg", 0644, es9028_debugfs_root,
			NULL, &es9028_debugfs_fops);
	if (!es9028_debugfs_reg) {
		dev_err(&client->dev, "es9028 debugfs create fail \n");
	}
#endif

	return ret;

}
static int es9028_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int es9028_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static int es9028_i2c_remove(struct i2c_client *client)
{
	int ret = 0;

	debugfs_remove(es9028_debugfs_reg);
	debugfs_remove(es9028_debugfs_root);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id device_es9028_of_match[] = {
    {.compatible = "ess,es9028",},
    {},
};
#else
#define device_es9028_of_match 0
#endif

static const struct i2c_device_id es9028_i2c_id[] = {
	{ ES9028_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver es9028_i2c_driver = {
    .probe              = es9028_i2c_probe,
	.suspend 			= es9028_i2c_suspend,
	.resume 			= es9028_i2c_resume,
    .remove             = es9028_i2c_remove,
    .driver = {
        .name           = ES9028_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = device_es9028_of_match,
    },
    .id_table = es9028_i2c_id,
};

static int __init es9028_dac_init(void)
{
	printk("%s\n",__func__);
	if(i2c_add_driver(&es9028_i2c_driver))
	{
	 	printk(KERN_ERR " %s add i2c driver error\n ",__func__);
	}
    return 0;
}

static void __exit es9028_dac_exit(void)
{
    i2c_del_driver(&es9028_i2c_driver);

	return;
}

module_init(es9028_dac_init);
module_exit(es9028_dac_exit);

MODULE_DESCRIPTION("es9028 clock generator driver");
MODULE_AUTHOR("leichenji <leichenji@vivo.com>");
MODULE_LICENSE("GPL");
