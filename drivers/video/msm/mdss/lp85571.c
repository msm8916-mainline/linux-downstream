#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/HWVersion.h>
#include "mdss_dsi.h"

extern int lcm_id2;
static struct i2c_client *lp85571_client;

static int i2c_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
	};
	u8 rx_data[1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

static int i2c_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	int r;
	u8 tx_data[] = {
		reg & 0xff,
		value & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

static int is_support = -1;
int lp85571_support(void)
{
	if(is_support != -1)
		return is_support;

	if(false /*Read_HW_ID()==HW_ID_SR1*/) {
		printk("[DISPLAY] %s: Not support LP85571\n", __func__);
		is_support = 0;
	}else{
		printk("[DISPLAY] %s: Support LP85571\n", __func__);
		is_support = 1;
	}

	return is_support;
}
EXPORT_SYMBOL(lp85571_support);

void lp85571_suspend(void)
{
	u8 value0=0;
	int ret0;

	i2c_reg_write(lp85571_client, 0x00, 0x00);
	ret0 = i2c_reg_read(lp85571_client, 0x00, &value0);
	printk("[DISPLAY] %s: 00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0);
	return;
}
EXPORT_SYMBOL(lp85571_suspend);

void lp85571_resume(void)
{
	u8 value0, value1, value2, value3, value4, value5=0;
	int ret0, ret1, ret2, ret3 ,ret4 ,ret5=0;

	usleep_range(5000, 5000);

	i2c_reg_write(lp85571_client, 0x10, 0x84);
	if(lcm_id2)
		i2c_reg_write(lp85571_client, 0x11, 0x04);
	else
		i2c_reg_write(lp85571_client, 0x11, 0x05);
	i2c_reg_write(lp85571_client, 0x12, 0x2C);
	i2c_reg_write(lp85571_client, 0x13, 0x03);
	i2c_reg_write(lp85571_client, 0x14, 0x1F);
	i2c_reg_write(lp85571_client, 0x15, 0xC3);
	i2c_reg_write(lp85571_client, 0x16, 0x60);
	i2c_reg_write(lp85571_client, 0x00, 0x01);

	ret0 = i2c_reg_read(lp85571_client, 0x10, &value0);
	ret1 = i2c_reg_read(lp85571_client, 0x11, &value1);
	ret2 = i2c_reg_read(lp85571_client, 0x12, &value2);
	ret3 = i2c_reg_read(lp85571_client, 0x13, &value3);
	ret4 = i2c_reg_read(lp85571_client, 0x15, &value4);
	ret5 = i2c_reg_read(lp85571_client, 0x00, &value5);

	printk("[DISPLAY] %s: 10h=0x%x(ret=%d),11h=0x%x(ret=%d),12h=0x%x(ret=%d),13h=0x%x(ret=%d),15h=0x%x(ret=%d),00h=0x%x(ret=%d)\n",
			__func__,
			value0, ret0,
			value1, ret1,
			value2, ret2,
			value3, ret3,
			value4, ret4,
			value5, ret5);
	return;
}
EXPORT_SYMBOL(lp85571_resume);

static int lp85571_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int addr;

	if(!lp85571_support()) return 0;

	printk("[DISPLAY] %s: Enter\n",__func__);

	lp85571_client = client;
	addr = client->addr;

	printk("[DISPLAY] %s: slave address=0x%x\n", __func__, addr);

	return 0;
}

static struct of_device_id lp85571_i2c_table[] = {
	{ .compatible = "lp,85571"}, //Compatible node must match dts
	{ },
};

static const struct i2c_device_id lp85571_id[] = {
	{ "lp85571", 0 },
	{ },
};

static struct i2c_driver lp85571_driver = {
	.driver = {
		.name = "lp85571",
		.owner = THIS_MODULE,
		.of_match_table = lp85571_i2c_table,
	},
	.probe = lp85571_probe,
	.id_table = lp85571_id,
};


static int __init lp85571_I2C_init(void)
{
	int ret = 0;
	printk("[DISPLAY] %s: Enter\n",__func__);
	ret = i2c_add_driver(&lp85571_driver);

	return ret;
}

static void __exit lp85571_I2C_exit(void)
{
	return;
}

module_init(lp85571_I2C_init);
module_exit(lp85571_I2C_exit);

MODULE_DESCRIPTION("lp85571");
MODULE_LICENSE("GPL v2");
