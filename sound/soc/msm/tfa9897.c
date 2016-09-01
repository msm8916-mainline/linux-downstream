#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#define TFA9897_DEBUG
#define AUDIO_NAME "#BC tfa9897"

static int g_tfa9897_init_fs = 0;
static int tfa9897_current_index = 0;

#undef TFA9897_DEBUG
#ifdef TFA9897_DEBUG
#define b_codec_dbg(format, arg...) \
    printk(KERN_INFO AUDIO_NAME ": " format, ## arg)

#define b_codec_err(format, arg...) \
    printk(KERN_ERR AUDIO_NAME ": " format, ## arg)

#define b_codec_trace() \
    printk("%s(%d)\n", __func__, __LINE__)
#else
#define b_codec_dbg(format, arg...) pr_debug(format, ##arg)
#define b_codec_err(format, arg...) pr_err(format, ##arg)

#define b_codec_trace() do {} while (0)
#endif


#define TFA9897_I2C_ADDR_TOP 0x38
#define TFA9897_I2C_ADDR_BTM 0x40

struct tfa9897_sys_data_s{
    u32 reset_pin;
    u32 switch_ctl_pin;
	
    int reset_pin_state;
    int switch_pin_state;

    struct regulator *vdd;
    struct i2c_client *client;
	
};

enum {
    TFA9897_TOP = 0,
    TFA9897_BTM,
};

struct tfa9897_sys_data_s  *g_tfa9897_sys[2];


static int hex2dec(u8 ch)
{
    if(ch >= '0' && ch <= '9')
        return ch - '0';
    else if(ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if(ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;

    return -1;
}


static int raw_i2c_read(struct i2c_client *client, char *buf, int count)
{
    if (count != i2c_master_recv(client, buf, count)){
        b_codec_err("i2c_master_recv error\n");
        return -1;
    }

    return 0;
}

static int raw_i2c_write(struct i2c_client *client, char *buf, int count)
{
    if(count != i2c_master_send(client, buf, count)){
        b_codec_err("i2c_master_send error\n");
        return -1;
    }

    return 0;
}

//
// 1. write addr
// 2. first msb
// 3, sec   lsb
//
static int tfa9897_read(struct i2c_client *client, int reg)
{
    int rc;
	__be16 tmp_data = 0;
	u16 reg_data  = 0;

    rc = raw_i2c_write(client, (char *)(&reg), 1);
    if (rc) {
        b_codec_err("tfa9897_read w 0x%02x\n", reg);
        return -1;
    }

    rc = raw_i2c_read(client, (char *)(&tmp_data), 2);
    if (rc) {
        b_codec_err("tfa9897_read r 0x%02x\n", reg);
        return -1;
    }
    reg_data = be16_to_cpu(tmp_data);

    b_codec_dbg("tmp_data=0x%04x, reg_data=0x%04x\n", tmp_data, reg_data);

    return reg_data;
}



//
// 1. write addr
// 2. write msb
// 3, write lsb
//

static int tfa9897_write_word(int pos ,int reg, int data)
{
	int rc;
    u8 buf[3];
    struct i2c_client *client;
    __be16 tmp_data = data;
	u16 reg_data = be16_to_cpu(tmp_data);
	
    client = g_tfa9897_sys[pos]->client;

    b_codec_dbg("%s chip addr=0x%02x reg=0x%02x human_data=0x%04x, reg_data=0x%04x\n", __func__, client->addr,reg , tmp_data, reg_data);
	
    buf[0] = reg&0xff;
    buf[1] = reg_data&0xff; //msb first
    buf[2] = (reg_data >> 8)&0xff; //lsb last

    if(client->addr == 0x38 )
		client->addr = 0x34;
    else if(client->addr == 0x40)
		client->addr = 0x36;
    b_codec_dbg("%s chip addr=0x%02x reg=0x%02x human_data=0x%04x, reg_data=0x%04x\n", __func__, client->addr,  reg, tmp_data, reg_data);
    rc = raw_i2c_write(client, (char *)buf, 3);
    if (rc < 0) {
        b_codec_err("%s chip addr=0x%02x err = %d\n", __func__, client->addr, rc);
    }	
	
    if(client->addr == 0x34 )
		client->addr = 0x38;
    else if(client->addr == 0x36)
		client->addr = 0x40;

    return 0;
}


void tfa9897_reset(int pos, int onoff)
{
    struct tfa9897_sys_data_s  *tfa9897_sys =  g_tfa9897_sys[pos];

    if (onoff)
    {
        gpio_direction_output(tfa9897_sys->reset_pin, 1);
    }
    else
    {
        gpio_direction_output(tfa9897_sys->reset_pin, 0);
    }

    tfa9897_sys->reset_pin_state = onoff;

    b_codec_dbg("tfa9897[%d] reset = %d\n",pos, tfa9897_sys->reset_pin_state);

    return;
}

enum {
    SPK_REV_L_SWITCH = 1,
    SPK_REV_R_SWITCH,
};


void tfa9897_switch(int pos, int onoff)
{
    u32 pin;
    struct tfa9897_sys_data_s  *tfa9897_sys = g_tfa9897_sys[pos];

    pin = tfa9897_sys->switch_ctl_pin;
    tfa9897_sys->switch_pin_state = onoff;
    b_codec_dbg("set SPK_REV_L_SWITCH = %d\n", tfa9897_sys->switch_pin_state);

    if (onoff)
    {
        gpio_direction_output(pin, 1);
    }
    else
    {
        gpio_direction_output(pin, 0);
    }

    return;
}


/*
 * platform data
 * */
static int tfa9897_parse_dt(struct tfa9897_sys_data_s *sys_data)
{
    struct device_node *np = sys_data->client->dev.of_node;
    enum of_gpio_flags flags;
	
	sys_data->reset_pin = of_get_named_gpio_flags(np, "tfa,rst-gpio", 0, &flags);
	if (sys_data->reset_pin < 0) {
		return sys_data->reset_pin;
    }

	sys_data->switch_ctl_pin = of_get_named_gpio_flags(np, "tfa,spk-rcv-switch", 0, &flags);
	if (sys_data->switch_ctl_pin < 0) {
		return sys_data->switch_ctl_pin;
    }

    return 0;
}


static int tfa9897_config_pins(struct tfa9897_sys_data_s *sys_data)
{
    if (gpio_request(sys_data->reset_pin, "tfa9897_reset") < 0) {
        b_codec_err("gpio err  %d\n", sys_data->reset_pin);
        return -1;
    }
    gpio_direction_output(sys_data->reset_pin, 0);


    if (gpio_request(sys_data->switch_ctl_pin, "tfa9897_switch") < 0) {
        b_codec_err("gpio err %d\n", sys_data->switch_ctl_pin);
        return -1;
    }
    gpio_direction_output(sys_data->switch_ctl_pin, 0);

    return  0;
}



static int tfa9897_regulator_init(struct tfa9897_sys_data_s *sys_data)
{
    int rc = -1;

    /*
     * codec V_L6_1P8 src VREG_L6
     * I2C pull up  V_L6_1P8 src VREG_L6
     * */
    sys_data->vdd = regulator_get(&sys_data->client->dev, "vdd");
    if (IS_ERR(sys_data->vdd)) {
        b_codec_err("%s %d get vdd error\n", __func__, __LINE__);
        goto err_get_vdd;
    }

    if (regulator_count_voltages(sys_data->vdd) > 0)
    {
        rc = regulator_set_voltage(sys_data->vdd, 1800000, 1800000);
        if (rc) {
            b_codec_err("%s %d set vdd error\n", __func__, __LINE__);
            goto err_set_vdd;
        }
    }

	rc = regulator_enable(sys_data->vdd);
	if (rc) {
		dev_err(&sys_data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

    return 0;

err_set_vdd:
    regulator_put(sys_data->vdd);

err_get_vdd:
    if (regulator_count_voltages(sys_data->vdd) > 0) {
        regulator_set_voltage(sys_data->vdd, 0, 1800000);
    }

    return rc;
}



/*
 *
 * */
static ssize_t tfa9897_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%u",  g_tfa9897_sys[tfa9897_current_index]->reset_pin_state);
}

static ssize_t tfa9897_reset_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	int err = 0;
    unsigned long value;

    if (count != 1) {
        b_codec_err("%s %d cnt=%d, buf=%d\n", __func__, __LINE__, (int)count, (int)sizeof(buf));
        return -EINVAL;
    }

	err = kstrtoul(buf, 10, &value);
	if (err != 0) {
        b_codec_err("%s %d err=%d\n", __func__, __LINE__, err);
		return err;
    }

    switch (value)
    {
        case 0:
            tfa9897_reset(TFA9897_TOP, 0);
            tfa9897_reset(TFA9897_BTM, 0);
           break;

        case 1:
            tfa9897_reset(TFA9897_TOP, 1);
            tfa9897_reset(TFA9897_BTM, 1);
            break;

        default:
            err = -EINVAL;
            b_codec_err("%s %d unknow value err=%d\n", __func__, __LINE__, (int)value);
            break;
    }

    return count;
}
static DEVICE_ATTR(tfa9897_reset, 0664, tfa9897_reset_show, tfa9897_reset_store);


/*
 *
 * */
static ssize_t tfa9897_switch1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%u",  g_tfa9897_sys[0]->switch_pin_state);
}

static ssize_t tfa9897_switch1_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	int err = 0;
	unsigned long value;

    if (count != 1) {
        b_codec_err("%s %d cnt=%d\n", __func__, __LINE__, (int)count);
        return -EINVAL;
    }

	err = kstrtoul(buf, 10, &value);
	if (err != 0) {
        b_codec_err("%s %d err=%d\n", __func__, __LINE__, err);
		return err;
    }

    switch (value)
    {
        case 0:
            tfa9897_switch(TFA9897_TOP, 0);
            break;

        case 1:
            tfa9897_switch(TFA9897_TOP, 1);
            break;

        default:
            err = -EINVAL;
            b_codec_err("%s %d unknow value err=%d\n", __func__, __LINE__, (int)value);
            break;
    }

    return count;
}
static DEVICE_ATTR(tfa9897_switch1, 0664, tfa9897_switch1_show, tfa9897_switch1_store);

atomic_t calibrate;
static ssize_t tfa9897_calibrate_show(
               struct device *dev, struct device_attribute *attr,
               char *buf)
{
       int value = atomic_read(&calibrate);
       return sprintf(buf, "%d\n", value);
}

static ssize_t tfa9897_calibrate_store(
               struct device *dev, struct device_attribute *attr,
               const char *buf, size_t size)
{
       int value;
       sscanf(buf, "%d", &value);
       atomic_set(&calibrate, value);
       if(value) {
        tfa9897_reset(TFA9897_TOP, 0);
        tfa9897_reset(TFA9897_BTM, 0);
       }
       else {
        tfa9897_reset(TFA9897_TOP, 1);
        tfa9897_reset(TFA9897_BTM, 1);
       }
       b_codec_dbg("[Liu]%s: calibrate=%d\n", __func__, value);
       return size;
}

static DEVICE_ATTR(calibrate, 0664, tfa9897_calibrate_show, tfa9897_calibrate_store);

/*
 * force set CLASS-D AMP
 * wish code is useless
 * */
void enable_amp(int pos ,int on)
{
    u16 old = 0;
    u16 new = 0;
    struct i2c_client *client;

    client = g_tfa9897_sys[pos]->client;

    if(client->addr == 0x38 )
		client->addr = 0x34;
    else if(client->addr == 0x40)
		client->addr = 0x36;

    old = tfa9897_read(client, 0x09);
    new = (old & ~(1 << 3)) | (on<<3);

    if(client->addr == 0x34 )
		client->addr = 0x38;
    else if(client->addr == 0x36)
		client->addr = 0x40;

    b_codec_dbg("[%d][%d]:old=0x%x new=0x%x\n", pos,on,old, new);

    tfa9897_write_word(pos, 0x09, new);
}

static int stero_mono_mode = 2;

static ssize_t tfa9897_sm_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", stero_mono_mode);
}

static ssize_t tfa9897_sm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	switch (value)
    {
        case 0:
            enable_amp(TFA9897_TOP,1);
            enable_amp(TFA9897_BTM,0);
            break;

        case 1:
            enable_amp(TFA9897_TOP,0);
            enable_amp(TFA9897_BTM,1);
            break;

        case 2:
            enable_amp(TFA9897_TOP,1);
            enable_amp(TFA9897_BTM,1);
            break;

        default:
            b_codec_err("%s: unknow stero_mono_mode=%d\n", __func__, value);
            break;
	}

    stero_mono_mode = value;

	b_codec_dbg("%s: stero_mono_mode=%d\n", __func__, stero_mono_mode);

	return size;
}

static DEVICE_ATTR(stereo_mono_mode, 0664, tfa9897_sm_show, tfa9897_sm_store);


/*
 *
 * */
static ssize_t tfa9897_switch2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%u",  g_tfa9897_sys[1]->switch_pin_state);
}

static ssize_t tfa9897_switch2_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	int err = 0;
	unsigned long value;

    if (count != 1) {
        b_codec_dbg("%s %d cnt=%d\n", __func__, __LINE__, (int)count);
        return -EINVAL;
    }

	err = kstrtoul(buf, 10, &value);
	if (err != 0) {
        b_codec_dbg("%s %d err=%d\n", __func__, __LINE__, err);
		return err;
    }

    switch (value)
    {
        case 0:
            tfa9897_switch(TFA9897_BTM, 0);
            break;

        case 1:
            tfa9897_switch(TFA9897_BTM, 1);
            break;

        default:
            err = -EINVAL;
            b_codec_err("%s %d unknow value err=%d\n", __func__, __LINE__, (int)value);
            break;
    }

    return count;
}
static DEVICE_ATTR(tfa9897_switch2, 0664, tfa9897_switch2_show, tfa9897_switch2_store);


static ssize_t tfa9897_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    g_tfa9897_sys[0]->client->addr = 0x34;
    tfa9897_read(g_tfa9897_sys[0]->client, 0x03);
    tfa9897_read(g_tfa9897_sys[0]->client, 0x04);
    tfa9897_read(g_tfa9897_sys[0]->client, 0x09);
    g_tfa9897_sys[0]->client->addr = 0x38;

     g_tfa9897_sys[1]->client->addr = 0x36;
   tfa9897_read(g_tfa9897_sys[1]->client, 0x03);
    tfa9897_read(g_tfa9897_sys[1]->client, 0x04);
    tfa9897_read(g_tfa9897_sys[1]->client, 0x09);
    g_tfa9897_sys[1]->client->addr = 0x40;

    return sprintf(buf, "%sregister read done.\r\n", buf);
}
static ssize_t tfa9897_reg_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t size)
{
    u8 reg = 0;
    u16 data = 0;
    int ret = 0;

    if (size != 6) {
        b_codec_err("echo -n 030000 > tfa9897_control\n");
        return -1;
    }

    /*
     * trans reg
     * */
    ret = hex2dec(buf[0]);
    if (ret == -1) {
        b_codec_err("store error.\n");
        return -1;
    }
    reg = ret << 4;

    ret = hex2dec(buf[1]);
    if (ret == -1) {
        b_codec_err("store error.\n");
        return -1;
    }
    reg |= (ret & 0xf);


    /*
     * trans data
     * */
    ret = hex2dec(buf[2]);
    if (ret == -1) {
        b_codec_err("data 0 error\n");
        return -1;
    }
    data= ret << 12;

    ret = hex2dec(buf[3]);
    if (ret == -1) {
        b_codec_err("data 1 error\n");
        return -1;
    }
    data |= (ret << 8);

    ret = hex2dec(buf[4]);
    if (ret == -1) {
        b_codec_err("data 2 error\n");
        return -1;
    }
    data |= ret << 4;

    ret = hex2dec(buf[5]);
    if (ret == -1) {
        b_codec_err("data 3 error\n");
        return -1;
    }
    data |= (ret & 0xf);
	
    tfa9897_write_word(TFA9897_TOP,reg,data);

    tfa9897_write_word(TFA9897_BTM,reg,data);
    return size;
}
static DEVICE_ATTR(tfa9897_reg, 0664, tfa9897_reg_show, tfa9897_reg_store);

atomic_t sclk_en;
extern int quat_mi2s_sclk_enable(int enable);
static ssize_t tfa9897_sclk_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int value = atomic_read(&sclk_en);
	return sprintf(buf, "%d\n", value);
}
static ssize_t tfa9897_sclk_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	atomic_set(&sclk_en, value);
	quat_mi2s_sclk_enable(value);
	b_codec_dbg("[Liu]%s: sclk_en=%d\n", __func__, value);
	return size;
}
static DEVICE_ATTR(sclk, 0664,
	tfa9897_sclk_show, tfa9897_sclk_store);


static struct class *nxp_class;
static struct device *nxp_dev;
static void tfa9897_control_attr_create(void)
{
	int rc = 0;
	
	nxp_class = class_create(THIS_MODULE, "tfa9897");
	if (IS_ERR(nxp_class))
		pr_err("Failed to create class(nxp_class)!\n");
	nxp_dev = device_create(nxp_class,
                                     NULL, 0, NULL, "control");
	if (IS_ERR(nxp_dev))
		pr_err("Failed to create device(nxp_dev)!\n");

	rc = device_create_file(nxp_dev, &dev_attr_tfa9897_reset);
	if (rc) {
		b_codec_err("device_create_file reset eror %d\n", rc);
	}
	rc = device_create_file(nxp_dev, &dev_attr_tfa9897_switch1);
	if (rc) {
		b_codec_err("device_create_file switch1 eror %d\n", rc);
	}

	rc = device_create_file(nxp_dev, &dev_attr_tfa9897_switch2);
	if (rc) {
	        b_codec_err("device_create_file switch2 eror %d\n", rc);
	}

	rc = device_create_file(nxp_dev, &dev_attr_tfa9897_reg);
	if (rc) {
	        b_codec_err("device_create_file reg eror %d\n", rc);
	}
	rc = device_create_file(nxp_dev, &dev_attr_calibrate);
	if (rc) {
	        b_codec_err("device_create_file calibrate eror %d\n", rc);
	}

	rc = device_create_file(nxp_dev, &dev_attr_stereo_mono_mode);
	if (rc) {
	        b_codec_err("device_create_file stereo_mono_mode eror %d\n", rc);
	}
	
	rc = device_create_file(nxp_dev, &dev_attr_sclk);
	if (rc) {
	        b_codec_err("device_create_file sclk eror %d\n", rc);
	}

}

static int g_nxp_top_spk_state = 0;
static int g_nxp_top_rcv_state = 0;
static int g_nxp_btm_spk_state = 0;
static int g_nxp_btm_rcv_state = 0;
static int g_nxp_reset_state = 0;


int tfa98xx_codec_get_speaker_amp_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  spk  state %d\n", __func__, g_nxp_top_spk_state);
    ucontrol->value.integer.value[0] = g_nxp_top_spk_state;
    return 0;
}

int tfa98xx_codec_put_speaker_amp_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9897_reset(TFA9897_TOP, 0);
        tfa9897_switch(TFA9897_TOP, 0);
    }
    else
    {
 //       tfa9897_reset(TFA9897_TOP, 1);
    }
    g_nxp_top_spk_state = state;
    return 0;
}

int tfa98xx_codec_get_rcv_amp_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_top_rcv_state);
    ucontrol->value.integer.value[0] = g_nxp_top_rcv_state;
    return 0;
}
int tfa98xx_codec_put_rcv_amp_top_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9897_reset(TFA9897_TOP, 0);
        tfa9897_switch(TFA9897_TOP, 1);
    }
    else
    {
 //       tfa9897_reset(TFA9897_TOP, 1);
    }

    g_nxp_top_rcv_state = state;

    return 0;
}

int tfa98xx_codec_get_speaker_amp_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  spk  state %d\n", __func__, g_nxp_btm_spk_state);
    ucontrol->value.integer.value[0] = g_nxp_btm_spk_state;
    return 0;
}

int tfa98xx_codec_put_speaker_amp_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9897_reset(TFA9897_BTM, 0);
        tfa9897_switch(TFA9897_BTM, 0);
    }
    else
    {
//        tfa9897_reset(TFA9897_BTM, 1);
    }
    g_nxp_btm_spk_state = state;
    return 0;
}

int tfa98xx_codec_get_rcv_amp_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_btm_rcv_state);
    ucontrol->value.integer.value[0] = g_nxp_btm_rcv_state;
    return 0;
}
int tfa98xx_codec_put_rcv_amp_btm_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.enumerated.item[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9897_reset(TFA9897_BTM, 0);
        tfa9897_switch(TFA9897_BTM, 1);
    }
    else
    {
//        tfa9897_reset(TFA9897_BTM, 1);
    }
    g_nxp_btm_rcv_state = state;
    return 0;
}

int tfa98xx_codec_get_reset_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_reset_state);
    ucontrol->value.integer.value[0] = g_nxp_reset_state;
    return 0;
}

int tfa98xx_codec_put_reset_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.integer.value[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    if (state)
    {
        tfa9897_reset(TFA9897_TOP, 1);
        tfa9897_reset(TFA9897_BTM, 1);
	 msleep(5);
        tfa9897_reset(TFA9897_TOP, 0);
        tfa9897_reset(TFA9897_BTM, 0);
    }
    else
    {
        tfa9897_reset(TFA9897_TOP, 1);
        tfa9897_reset(TFA9897_BTM, 1);
    }
    g_nxp_reset_state = state;
    return 0;
}

static int g_nxp_power_switch_state = 0;

int tfa98xx_codec_get_power_switch_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    b_codec_dbg("#B get %s  rcv state %d\n", __func__, g_nxp_power_switch_state);
    ucontrol->value.integer.value[0] = g_nxp_power_switch_state;
    return 0;
}

int tfa98xx_codec_put_power_switch_control(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int state = ucontrol->value.integer.value[0];
    b_codec_dbg("\n\n\n>>>>> %s  set  %d\n\n\n", __func__, state);

    switch (state)
    {
		case 0:
		/*TCTNB THW add for PR887970 2015.1.6 */
		tfa9897_reset(TFA9897_TOP, 1);
		tfa9897_reset(TFA9897_BTM, 1);
		/*TCTNB THW add for PR887970 end */
		break;
		case 1:
		/*TCTNB THW add for PR887970 2015.1.6 */
		tfa9897_reset(TFA9897_TOP, 1);
		tfa9897_reset(TFA9897_BTM, 1);
		msleep(5);
		tfa9897_reset(TFA9897_TOP, 0);
		tfa9897_reset(TFA9897_BTM, 0);
		/*TCTNB THW add for PR887970 end */
		tfa9897_write_word(TFA9897_TOP, 0x14, 0x0000);
		tfa9897_write_word(TFA9897_TOP, 0x04, 0x880b);
		tfa9897_write_word(TFA9897_TOP, 0x09, 0x8209);
		tfa9897_write_word(TFA9897_TOP, 0x09, 0x0608);
		enable_amp(TFA9897_TOP,1);
		enable_amp(TFA9897_BTM,0);
		break;
		case 2:
		/*TCTNB THW add for PR887970 2015.1.6 */
		tfa9897_reset(TFA9897_TOP, 1);
		tfa9897_reset(TFA9897_BTM, 1);
		msleep(5);
		tfa9897_reset(TFA9897_TOP, 0);
		tfa9897_reset(TFA9897_BTM, 0);
		/*TCTNB THW add for PR887970 end */
		tfa9897_write_word(TFA9897_BTM, 0x14, 0x0000);
		tfa9897_write_word(TFA9897_BTM, 0x04, 0x880b);
		tfa9897_write_word(TFA9897_BTM, 0x09, 0x8209);
		tfa9897_write_word(TFA9897_BTM, 0x09, 0x0608);
		enable_amp(TFA9897_TOP,0);
		enable_amp(TFA9897_BTM,1);
		break;
	default:
		b_codec_err("error ,tfa not support case !");
		break;
    }
    g_nxp_power_switch_state = state;
    return 0;
}

static const char *const amp_text[] = {"Off", "On"};
static const char *const amp_text_switch[] = {"Off", "Top","Button"};
static const struct soc_enum amp_enum = SOC_ENUM_SINGLE_EXT(2, amp_text);
static const struct soc_enum amp_enum_switch = SOC_ENUM_SINGLE_EXT(3, amp_text_switch);

static const struct snd_kcontrol_new tfa98xx_snd_controls[] = {
	SOC_ENUM_EXT("TFA98XX_SPK_AMP_TOP",
		amp_enum,
		tfa98xx_codec_get_speaker_amp_top_control,
		tfa98xx_codec_put_speaker_amp_top_control),

	SOC_ENUM_EXT("TFA98XX_RCV_AMP_TOP",
		amp_enum,
		tfa98xx_codec_get_rcv_amp_top_control,
		tfa98xx_codec_put_rcv_amp_top_control),

	SOC_ENUM_EXT("TFA98XX_SPK_AMP_BTM",
		amp_enum,
		tfa98xx_codec_get_speaker_amp_btm_control,
		tfa98xx_codec_put_speaker_amp_btm_control),

	SOC_ENUM_EXT("TFA98XX_RCV_AMP_BTM",
		amp_enum,
		tfa98xx_codec_get_rcv_amp_btm_control,
		tfa98xx_codec_put_rcv_amp_btm_control),

       SOC_ENUM_EXT("TFA98XX_RESET",
        amp_enum,
		tfa98xx_codec_get_reset_control,
		tfa98xx_codec_put_reset_control),

       SOC_ENUM_EXT("TFA98XX_POWER_SWITCH",
        amp_enum_switch,
		tfa98xx_codec_get_power_switch_control,
		tfa98xx_codec_put_power_switch_control),

};


static int tfa98xx_codec_probe(struct snd_soc_codec *codec)
{
    b_codec_trace();
    return 0;
}

static int tfa98xx_codec_remove(struct snd_soc_codec *codec)
{
    b_codec_trace();
    return 0;
}


static struct snd_soc_codec_driver soc_codec_dev_tfa98xx = {
	.probe	= tfa98xx_codec_probe,
	.remove	= tfa98xx_codec_remove,
	.controls = tfa98xx_snd_controls,
	.num_controls = ARRAY_SIZE(tfa98xx_snd_controls),
};


static struct snd_soc_dai_driver tfa98xx_dais[] = {
	{
		.name = "tfa98xx-rx",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};


static int tfa9897_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
    int rc = 0;
    struct tfa9897_sys_data_s  * sys_data = NULL;
    uint32_t cell_id;
    struct device_node *of_node = i2c->dev.of_node;

    b_codec_trace();

    b_codec_dbg("i2c addr = 0x%02x\n", i2c->addr);


	rc = of_property_read_u32(of_node, "cell-index", &cell_id);
	if (rc < 0) {
		b_codec_err("failed: cell-index rc %d", rc);
		goto finish;
	}

    if (i2c->addr != TFA9897_I2C_ADDR_TOP && i2c->addr != TFA9897_I2C_ADDR_BTM) {
        b_codec_err("unknow tfa9897 = 0x%02x\n", i2c->addr);
        return -1;
    }


      sys_data = kzalloc(sizeof(struct tfa9897_sys_data_s), GFP_KERNEL);
      if (sys_data == NULL) {
            return -ENOMEM;
        }

     sys_data->client = i2c;
     b_codec_trace();
    /*
     * get resource from OF!
     * */
    rc = tfa9897_parse_dt(sys_data);
    if (rc) {
        b_codec_err("tfa9897_parse_dt error %d\n", rc);
    }


    rc = tfa9897_regulator_init(sys_data);
    if (rc < 0) {
        b_codec_err("get regulator error %d\n", rc);
    }


    tfa9897_config_pins(sys_data);


    g_tfa9897_sys[cell_id] = sys_data;
 
    if(!g_tfa9897_init_fs)
    {
    	tfa9897_control_attr_create();
	dev_set_name(&i2c->dev, "%s", "tfa9897-codec");
	rc = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_tfa98xx,
		 tfa98xx_dais, ARRAY_SIZE(tfa98xx_dais));

	g_tfa9897_init_fs = 1;	
    }

finish:
    return rc;
}

static int tfa9897_i2c_remove(struct i2c_client *client)
{
	b_codec_dbg("%s(%d)\n",__FUNCTION__,__LINE__);
	return 0;
}

static const struct i2c_device_id tfa9897_i2c_id[] = {
	{ "tfa9897", 0 },
	{ }
};

static struct of_device_id tfa9897_match_table[] = {
	{ .compatible = "nxp,tfa9897", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tfa9897_i2c_id);

static struct i2c_driver tfa9897_i2c_driver = {
	.driver = {
		.name = "tfa-tfa9897",
		.owner = THIS_MODULE,
		.of_match_table = tfa9897_match_table,
	},
	.probe = tfa9897_i2c_probe,
	.remove = tfa9897_i2c_remove,
	.id_table = tfa9897_i2c_id,
};

module_i2c_driver(tfa9897_i2c_driver);

MODULE_DESCRIPTION("IDOL3 tfa9897 codec driver");
MODULE_LICENSE("GPL");

