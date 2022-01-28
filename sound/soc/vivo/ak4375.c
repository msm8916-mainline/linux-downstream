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

#include "ak4375.h"
#define AK4375_DEV_NAME "ak4375"
#define AK4375_PHYS_NAME "ak4375_phys_dev"
static bool ak4375_available = false;
static struct vivo_codec_function *ak4375_function = NULL;

enum
{
    AK4375_VERSION_0,
	AK4375_VERSION_A,
	AK4375_VERSION_NONE
   	
}AK4375_VERSION;
int ak4375_version = AK4375_VERSION_NONE;

#ifndef BBK_VIVO_AUDIO_DEBUG
#define BBK_VIVO_AUDIO_DEBUG
#endif

#ifdef BBK_VIVO_AUDIO_DEBUG
static struct dentry *ak4375_debugfs_root;
static struct dentry *ak4375_debugfs_reg;
static struct dentry *ak4375_debugfs_i2c;

static u8 ak4375_regs[] = {

	0X00,
	0X01,
	0X02,
	0X03,
	0X04,
	0X05,
	0X06,
	0X07,
	0X08,
	0X09,
	0X0A,
	0X0B,
	0X0C,
	0X0D,
	0X0E,
	0X0F,
	0X10,
	0X11,
	0X12,
	0X13,
	0X14,
	0X15,
	0X24,
};

#endif

static u8 ak4375_reg_disable[6][2] = {
	{ 0x03 ,0x00 },
	{ 0x01 ,0x31 },
	{ 0x02 ,0x00 },
	{ 0x01 ,0x01 },
	{ 0xff ,0xff },
	{ 0x01 ,0x00 }
};
static u8 ak4375_reg_enable[34][2] = {
	{ 0x06 ,0x00 },
	{ 0x07 ,0x21 },
	{ 0x08 ,0x00 },
	{ 0x09 ,0x00 },
	{ 0x0A ,0x00 },
	{ 0x0B ,0x19 },
	{ 0x0C ,0x19 },
	{ 0x0D ,0x73 },
	{ 0x0E ,0x01 },
	{ 0x0F ,0x00 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0x20 },
	{ 0x24 ,0x00 },
	{ 0x13 ,0x01 },
	{ 0x00 ,0x01 },
	{ 0x05 ,0x0A },
	{ 0xff ,0xff },
	{ 0x01 ,0x01 },
	{ 0xff ,0xff },
	{ 0xff ,0xff },
	{ 0xff ,0xff },
	{ 0xff ,0xff },
	{ 0x01 ,0x31 },
	{ 0xff ,0xff },
	{ 0x02 ,0x01 },
	{ 0x01 ,0x33 },
	{ 0xff ,0xff },
	{ 0xff ,0xff },
	{ 0xff ,0xff },
	{ 0x03 ,0x03 },
	{ 0x04 ,0x00 }
};

int reg_addr_tbl[] =
{
	0x05,
	0x08,
	0x0e,
	0x10,
	0x12,
	0x14,
	0x15
};

int reg_value_tbl[11][7] = 
{
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x01}, //ak4375 48k 16bit
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x00}, //ak4375 48k 24bit
	{0x0e, 0x0e, 0x01, 0x00, 0x13, 0x09, 0x00}, //ak4375 96k 24bit
	{0x12, 0x12, 0x01, 0x00, 0x09, 0x09, 0x00}, //ak4375 192k 24bit
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x21}, //ak4375a 48k 16bit
	{0x09, 0x09, 0x00, 0x18, 0x92, 0x09, 0x30}, //ak4375a 44.1k 24bit
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x20}, //ak4375a 48k 24bit
	{0x0d, 0x0d, 0x00, 0x18, 0x92, 0x04, 0x30}, //ak4375a 88.2k 24bit
	{0x0e, 0x0e, 0x01, 0x01, 0x27, 0x04, 0x20}, //ak4375a 96k 24bit
	{0x71, 0x71, 0x00, 0x18, 0x92, 0x04, 0x30}, //ak4375a 176.4k 24bit
	{0x72, 0x72, 0x01, 0x03, 0x27, 0x04, 0x20}, //ak4375a 192k 24bit
};

struct ak4375_data {
	struct i2c_client *client;
	char *driver_name;
	struct regulator *vd_regulator;
	struct regulator *va_regulator;
	int vd_type;
	int va_type;
	int vd_ctrl_gpio;
	int va_ctrl_gpio;
	int rst_gpio;
	int volume;
	struct clk *hifi_mclk; 
	struct mutex lock;
};
static struct ak4375_data *ak4375_data;

static int ak4375_phys_open(struct inode *inode, struct file *filep)
{
	filep->private_data = inode->i_private;
	return 0;
}
int ak4375_phys_release (struct inode *inode, struct file *filep)
{
	int ret =0;
	return ret;
}
long ak4375_phys_ioctl (struct file *filep, unsigned int cmd, unsigned long args)
{
	int ret = 0;

	printk(KERN_ALERT "%s \n",__func__);
	return (long)ret;
}


static struct file_operations ak4375_phys_fops = {
	.open = ak4375_phys_open,
	.unlocked_ioctl = ak4375_phys_ioctl,
	.release = ak4375_phys_release,
};

static struct miscdevice ak4375_phys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = AK4375_PHYS_NAME,
	.fops = &ak4375_phys_fops,
};
static int ak4375_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(ak4375_data->client,&reg,1);
	if(ret < 0)
	{
		dev_err(&ak4375_data->client->dev, "%s i2c send cmd error reg=%d \n",__func__,reg);
		return ret;
	}
	ret = i2c_master_recv(ak4375_data->client,&buf,1);
	if(ret <0 )
	{
		dev_err(&ak4375_data->client->dev,"%s i2c recv error \n ",__func__);
		return ret;
	}
	//pr_err("%s: reg = 0x%x value = 0x%x\n", __func__, reg, buf);

	return buf;

}
static int ak4375_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(ak4375_data->client,cmd,sizeof(cmd));
	if (ret < 1)
	{
		dev_err(&ak4375_data->client->dev,"%s i2c send error cmd[0]=%d,cmd[1]=%d\n",__func__,cmd[0],cmd[1]);
	}
	pr_err("%s: reg = 0x%x value = 0x%x\n", __func__, reg, data);

	return ret;
}

int ak4375_reset(int reset_pin)
{
	gpio_direction_output(reset_pin,0);
	usleep(10000);
	gpio_direction_output(reset_pin,1);
	return 0;
}
int ak4375_mute(bool mute)
{

	return 0;
}
int ak4375_dpower_on(bool enable)
{
	int ret;
	if(enable){
	    if(ak4375_data->vd_type)
            ret = regulator_enable(ak4375_data->vd_regulator);
        else
			gpio_direction_output(ak4375_data->vd_ctrl_gpio,1);
	}else{
		if(ak4375_data->vd_type)
            ret = regulator_disable(ak4375_data->vd_regulator);
        else
			gpio_direction_output(ak4375_data->vd_ctrl_gpio,0);
	}
	return 0;
}

int ak4375_apower_on(bool enable)
{
	int ret;
	if(enable){
        if(ak4375_data->va_type)
            ret = regulator_enable(ak4375_data->va_regulator);
        else
        {
			gpio_direction_output(ak4375_data->va_ctrl_gpio,1);
        }
	}else{
        if(ak4375_data->va_type)
            ret = regulator_disable(ak4375_data->va_regulator);
        else
        {
			gpio_direction_output(ak4375_data->va_ctrl_gpio,0);
        }
	}
	return ret;
}


static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = ak4375_i2c_read_byte(reg);
	while((val != reg_val)&&(retry > 0)){
		ak4375_i2c_write_byte(reg, val);
		reg_val = ak4375_i2c_read_byte(reg);
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


int ak4375_enable(struct audio_params *params, bool enable)
{
	int i, j;
	int ret = 0;
	int format = params->pcm_format;
	int sample_rate = params->rate;
	int reg_value, reg_addr;
	int mode_sel = 0;

	if(ak4375_data->client == NULL)
	{
		dev_err(&ak4375_data->client->dev,"%s client is NULL \n ",__func__);
		return -EFAULT;
	}
	if(!ak4375_available)
		return 0;

	switch (params->rate)
	{
		case 44100:
			sample_rate = 0;
			break;
		case 48000:
			sample_rate = 1;
			break;
		case 88200:
			sample_rate = 2;
			break;
		case 96000:
			sample_rate = 3;
			break;
		case 176400:
			sample_rate = 4;
			break;
		case 192000:
			sample_rate = 5;
			break;
		default:
			sample_rate = 1;
			break;
	}
	pr_info("%s:enable %d,format = %d, sample_rate = %d params->rate = %d\n",__func__,(int)enable, format, sample_rate, params->rate);
	mutex_lock(&ak4375_data->lock);
	if(enable)
	{
		if (ak4375_data->hifi_mclk)
		clk_prepare_enable(ak4375_data->hifi_mclk);
		usleep(5000);
			ak4375_reset(ak4375_data->rst_gpio);
		usleep(5000);
        
		
		if (ak4375_version == AK4375_VERSION_A)
		{
			if (format == SNDRV_PCM_FORMAT_S24_LE)
			{
				if ((sample_rate >= 0) && (sample_rate <= 5))
					mode_sel = sample_rate + 5;
				else
					mode_sel = 6;
			}
			else
				mode_sel = 4;
		}
		else
		{
			if (format == SNDRV_PCM_FORMAT_S24_LE)
			{
				if (sample_rate == 1)
					mode_sel = 1;
				else if (sample_rate == 3)
					mode_sel = 2;
				else if (sample_rate == 5)
					mode_sel = 3;
					
			}
			else
				mode_sel = 0;
		}
		pr_info("%s:mode_sel = %d\n",__func__, mode_sel);
		
		for(i=0;i<34;i++){

			reg_addr = ak4375_reg_enable[i][0];
			reg_value = ak4375_reg_enable[i][1];

			for (j = 0; j < 7; j++)
			{
               if (reg_addr_tbl [j] == reg_addr)
               {
			   	 reg_value = reg_value_tbl[mode_sel][j];
				 break;   
			   }
			}

			if (reg_addr == 0x0d && ak4375_data->volume > 0){
				reg_value = ak4375_data->volume;
				pr_info("%s:volume = %0x02x\n",__func__, reg_value);
			}
			
			if (reg_addr == 0xff)
			{
				usleep(2000);
				continue;
			}

			ret = ak4375_i2c_write_byte(reg_addr,reg_value);
			if(ret < 0)
			{
				pr_err("%s:check i2c addr= 0x%x,value= 0x%x\n",__func__,reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr,reg_value);
			if(ret < 0)
			{
				pr_err("%s:check i2c addr= 0x%x,value= 0x%x\n",__func__,reg_addr, reg_value);
				goto end;
			}
		}
		ret = 0;
	}else
	{
		for(i=0;i<6;i++){
			
			reg_addr = ak4375_reg_disable[i][0];
			reg_value = ak4375_reg_disable[i][1];
			if (reg_addr == 0xff)
			{
				usleep(2000);
				continue;
			}
			ret = ak4375_i2c_write_byte(reg_addr,reg_value);
			if(ret < 0)
			{
				pr_err("%s:check i2c addr= 0x%x,value= 0x%x\n",__func__,reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr,reg_value);
			if(ret < 0)
			{
				pr_err("%s:check i2c addr= 0x%x,value= 0x%x\n",__func__,reg_addr, reg_value);
				goto end;
			}
		}
		gpio_direction_output(ak4375_data->rst_gpio,0);
		if (ak4375_data->hifi_mclk)
		clk_disable_unprepare(ak4375_data->hifi_mclk);
		ret = 0;
	}
end:
	mutex_unlock(&ak4375_data->lock);
	pr_info("%s:exist\n",__func__);
	return ret;
}

#ifdef BBK_VIVO_AUDIO_DEBUG
static int ak4375_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t ak4375_debug_write(struct file *filp,
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
	}else
	{

		printk("gpio =1 enable voltage convert \n");
	}

	ak4375_i2c_write_byte(kbuf[0],kbuf[1]);

	return cnt;
}
static ssize_t ak4375_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	for(i = 0;i < sizeof(ak4375_regs);i++)
	{
		data = ak4375_i2c_read_byte(ak4375_regs[i]);
		n += scnprintf(buffer+n,size-n,"reg{%x}:%x \n",ak4375_regs[i],data);
	}

	buffer[n] = 0;
	pr_info("%s:==========catch ak4375 reg==========\n%s\n",
		__func__,buffer);
	pr_info("============caught ak4375 reg end =============\n");
	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations ak4375_debugfs_fops = {
	.open = ak4375_debug_open,
	.read = ak4375_debug_read,
	.write = ak4375_debug_write,
};

static ssize_t ak4375_debug_i2c_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;
	int ret;

	if (ak4375_data->hifi_mclk)
		clk_prepare_enable(ak4375_data->hifi_mclk);
	usleep(5000);
	ak4375_reset(ak4375_data->rst_gpio);
	usleep(5000);
    ret = ak4375_i2c_read_byte(0x15);
	if (ret < 0){
		printk(KERN_ERR "%s: read i2c error \n",__func__);
		ret = 0;
	}

	printk(KERN_ERR "%s: read reg[0x15] val[0x%x] \n",__func__,ret);

    ret = ((ret & 0xe0) >> 5);
	if (ret != 1) ret = 0;
	gpio_direction_output(ak4375_data->rst_gpio,0);
	if (ak4375_data->hifi_mclk)
		clk_disable_unprepare(ak4375_data->hifi_mclk);

	n += scnprintf(buffer+n, size-n, "ak4375 i2c read %s \n",
							ret? "OK":"ERROR" );

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations ak4375_i2c_debugfs_fops = {
	.open = ak4375_debug_open,
	.read = ak4375_debug_i2c_read,
};

static void ak4375_debugfs_init(void)
{
	ak4375_debugfs_root = debugfs_create_dir("ak4375",NULL);
	if(!ak4375_debugfs_root)
	{
		pr_err("%s debugfs create dir error\n",__func__);
	}
	else if(IS_ERR(ak4375_debugfs_root))
	{
		pr_err("%s Kernel not support debugfs \n",__func__);
		ak4375_debugfs_root = NULL;
	}

	ak4375_debugfs_reg = debugfs_create_file("reg",0644,ak4375_debugfs_root,NULL,&ak4375_debugfs_fops);
	if(!ak4375_debugfs_reg)
	{
		pr_err("ak4375 debugfs create fail \n");
	}
	ak4375_debugfs_i2c = debugfs_create_file("i2c",0444,ak4375_debugfs_root,NULL,&ak4375_i2c_debugfs_fops);
	if(!ak4375_debugfs_i2c)
	{
		pr_err("ak4375 i2c create fail \n");
	}
	return ;
}

static void ak4375_debugfs_deinit(void)
{
	debugfs_remove(ak4375_debugfs_i2c);
	debugfs_remove(ak4375_debugfs_reg);
	debugfs_remove(ak4375_debugfs_root);
	return ;
}
#endif
static int ak4375_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
    ak4375_data = kzalloc(sizeof(struct ak4375_data), GFP_KERNEL);
	dev_err(&client->dev,"%s start\n",__func__);

#ifdef BBK_VIVO_AUDIO_DEBUG
	ak4375_debugfs_init();
#endif

	ak4375_function = get_vivo_codec_function();

	if (!ak4375_function)
	{
		dev_err(&client->dev,"%s get_vivo_codec_function(%p) is NULL point failed\n",
			__func__, &ak4375_function);
		return -EINVAL;
	}
	
	if(!ak4375_data)
	{
		dev_err(&client->dev,"%s kzalloc failed\n",__func__);
		return -ENOMEM;
	}
    if(client->dev.of_node)
    {
        of_property_read_u32(client->dev.of_node,"bbk,vd-type",&ak4375_data->vd_type);
        of_property_read_u32(client->dev.of_node,"bbk,va-type",&ak4375_data->va_type);
        of_property_read_u32(client->dev.of_node,"vivo,hp-volume-control",&ak4375_data->volume);

        printk("%s:%d,%d,hp volume 0x%2x\n",__func__,ak4375_data->vd_type,ak4375_data->va_type,ak4375_data->volume);
        if(ak4375_data->vd_type){
            ak4375_data->vd_regulator = regulator_get(&client->dev,"ext_dac_vd");
            if (IS_ERR(ak4375_data->vd_regulator)) {
                ret = PTR_ERR(ak4375_data->vd_regulator);
                dev_err(&client->dev,
                    "Regulator get failed vd ret=%d\n", ret);
            return ret;
            }
            if ((regulator_count_voltages(ak4375_data->vd_regulator) > 0)){
                ret = regulator_set_voltage(ak4375_data->vd_regulator, 1800000,
							1800000);
                if (ret) {
                    dev_err(&client->dev,
                            "regulator set_vtg failed rc=%d\n", ret);
                    regulator_put(ak4375_data->vd_regulator);
                    return ret;
                }
            }
        }
        else{
            ak4375_data->vd_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-vd-enable-gpio",0);
			gpio_request(ak4375_data->vd_ctrl_gpio, "ak4375-vd-ctrl-gpio");
			pr_err("%s:vd_ctrl_gpio = %d\n", __func__, ak4375_data->vd_ctrl_gpio);
        }

        if(ak4375_data->va_type){
            ak4375_data->va_regulator = regulator_get(&client->dev,"ext_dac_va");
            if (IS_ERR(ak4375_data->vd_regulator)) {
                ret = PTR_ERR(ak4375_data->vd_regulator);
                dev_err(&client->dev,
                    "Regulator get failed va ret=%d\n", ret);
            return ret;
            }
            if ((regulator_count_voltages(ak4375_data->vd_regulator) > 0)){
                ret = regulator_set_voltage(ak4375_data->vd_regulator, 1800000,
							1800000);
                if (ret) {
                    dev_err(&client->dev,
                            "regulator set_vtg failed ret=%d\n", ret);
                    regulator_put(ak4375_data->vd_regulator);
                    return ret;
                }
            }
        }
        else{
			
            ak4375_data->va_ctrl_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-va-enable-gpio",0);
			gpio_request(ak4375_data->va_ctrl_gpio, "ak4375-va-ctrl-gpio");
            pr_info("%s:va_ctrl_gpio = %d\n", __func__, ak4375_data->va_ctrl_gpio);
            
        }
        ak4375_data->rst_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-rst-gpio",0);
    }
    else{
        if(client->dev.platform_data){/*
            pdata = client->dev.platform_data;
            ak4375_data->vd_regulator = regulator_get(&client->dev,pdata->vd_regulator->name);
            ak4375_data->va_regulator = regulator_get(&client->dev,pdata->va_regulator->name);
            ak4375_data->rst_gpio = pdata->rst_gpio;
            ak4375_data->vd_type = 1;
            ak4375_data->va_type = 1;*/
            ;
        }
    }

	
	ak4375_data->client = client;
	ak4375_data->driver_name = AK4375_DEV_NAME;

	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
	{
		dev_err(&client->dev,"%s i2c check funtion error \n",__func__);
	}
    ak4375_data->hifi_mclk = clk_get(&client->dev,"ref_clk");
	 if (IS_ERR(ak4375_data->hifi_mclk)) {
		   printk(KERN_ERR"Get main 19.2M clock error = %d\n",(int)PTR_ERR(ak4375_data->hifi_mclk));
	   }
	 else
	 {
	 	pr_info("%s:get 19.2M clock suscess!!\n", __func__);
	 	clk_set_rate(ak4375_data->hifi_mclk, 19200000);
	 }

    ak4375_dpower_on(true);
	ak4375_apower_on(true);
	ret = gpio_request(ak4375_data->rst_gpio, "ak4375_rst");
	pr_err("%s:ak4375_rst = %d\n", __func__,ak4375_data->rst_gpio);
	if(ret <0)
	{
		dev_err(&client->dev,"'%s'(%d) gpio_request failed, rc=%d\n",
				"ak4375_rst", ak4375_data->rst_gpio, ret);
	}
	msleep(5);
	ak4375_reset(ak4375_data->rst_gpio);
	msleep(20);
    ret = ak4375_i2c_read_byte(0x15);
	if (ret < 0)
	{
		dev_err(&client->dev,"%s ak4375 device not exist\n",__func__);
		goto err_gpio;

	}
    ret = ((ret & 0xe0) >> 5);
	dev_err(&client->dev,"%s 0x15 = 0x%x\n",__func__, ret);
	switch (ret)
	{
		case 0:
			ak4375_version = AK4375_VERSION_0;
			dev_err(&client->dev,"%s AK4375_VERSION_0\n",__func__);
			break;
		case 1:
			ak4375_version = AK4375_VERSION_A;
			dev_err(&client->dev,"%s AK4375_VERSION_A\n",__func__);
			break;
		default:
			ak4375_version = AK4375_VERSION_0;
			dev_err(&client->dev,"%s default set AK4375_VERSION_0\n",__func__);
			break;
	}
    ak4375_available = true;
	ak4375_function->hifi_dac_enable = ak4375_enable;
	ak4375_function->hifi_dac_mute = ak4375_mute;
	mutex_init(&ak4375_data->lock);
	gpio_direction_output(ak4375_data->rst_gpio,0);
	//ak4375_apower_on(false);
	//ak4375_dpower_on(false);
	ret = misc_register(&ak4375_phys_dev);
	if(ret < 0)
	{
		dev_err(&client->dev,"%s ak4375 phys misc device register error\n",__func__);
		goto err_gpio;
	}
err_gpio:

	return ret;

}

static int ak4375_i2c_remove(struct i2c_client *client)
{
#ifdef BBK_VIVO_AUDIO_DEBUG
		ak4375_debugfs_deinit();
#endif

	return 0;
}

static int ak4375_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int ak4375_i2c_resume(struct i2c_client *client)
{
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id device_ak4375_of_match[] = {
    {.compatible = "ak,ak4375",},
    {},
};
#else
#define device_ak4375_of_match 0
#endif

static const struct i2c_device_id ak4375_i2c_id[] = {
	{ "ak4375", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4375_i2c_id);

static struct i2c_driver ak4375_i2c_driver = {
	.driver = {
		.name = "ak4375-codec",
		.owner = THIS_MODULE,
		.of_match_table = device_ak4375_of_match,
	},
	.probe 		= ak4375_i2c_probe,
	.remove 	= ak4375_i2c_remove,
	.suspend 	= ak4375_i2c_suspend,
	.resume 	= ak4375_i2c_resume,
	.id_table = ak4375_i2c_id,
};

module_i2c_driver(ak4375_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4375 codec driver");
MODULE_AUTHOR("Houzheng.Shen <shenhouzheng@vivo.com>");
MODULE_LICENSE("GPL");


