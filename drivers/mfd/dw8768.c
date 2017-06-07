/*
 * DW8768 MFD Driver
 *
 * Copyright 2015 LG Electronics Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/mfd/dw8768.h>

static struct dw8768 *dw8768_base;
static struct mfd_cell dw8768_devs[] = {
	{ .name = "dw8768_dev" },
};

int dw8768_read_byte(struct dw8768 *dw8768, u8 reg, u8 *read)
{
	int ret;
	unsigned int val;

	ret = regmap_read(dw8768->regmap, reg, &val);
	if (ret < 0)
		return ret;

	*read = (u8)val;
	return 0;
}
EXPORT_SYMBOL_GPL(dw8768_read_byte);

int dw8768_write_byte(struct dw8768 *dw8768, u8 reg, u8 data)
{
	return regmap_write(dw8768->regmap, reg, data);
}
EXPORT_SYMBOL_GPL(dw8768_write_byte);

int dw8768_off_seq(void)
{
	int ret;

	ret = dw8768_write_byte(dw8768_base, DW8768_ENABLE_REG, DW8768_OFF_SEQ1);
	if (ret < 0)
		return ret;
	mdelay(12);

	ret = dw8768_write_byte(dw8768_base, DW8768_ENABLE_REG, DW8768_OFF_SEQ2);
	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(dw8768_off_seq);

static struct regmap_config dw8768_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = DW8768_MAX_REGISTERS,
};

static int dw8768_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct dw8768 *dw8768;
	struct device *dev = &cl->dev;
	struct dw8768_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	pr_err("%s start\n",__func__);

	dw8768 = devm_kzalloc(dev, sizeof(*dw8768), GFP_KERNEL);
	if (!dw8768)
		return -ENOMEM;

	dw8768->pdata = pdata;

	dw8768->regmap = devm_regmap_init_i2c(cl, &dw8768_regmap_config);
	if (IS_ERR(dw8768->regmap)){
		pr_err("Failed to allocate register map\n");
		devm_kfree(dev, dw8768);
		return PTR_ERR(dw8768->regmap);
	}

	dw8768->dev = &cl->dev;
	i2c_set_clientdata(cl,dw8768);
	dw8768_base = dw8768;

	rc = mfd_add_devices(dev, -1, dw8768_devs, ARRAY_SIZE(dw8768_devs),
			NULL, 0, NULL);
	if (rc)
		pr_err("Failed to add dw8768 subdevice ret=%d\n", rc);

	return rc;
}

static int dw8768_remove(struct i2c_client *cl)
{
	struct dw8768 *dw8768 = i2c_get_clientdata(cl);

	mfd_remove_devices(dw8768->dev);

	return 0;
}

static const struct i2c_device_id dw8768_ids[] = {
	{ "dw8768", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dw8768_ids);

#ifdef CONFIG_OF
static const struct of_device_id dw8768_of_match[] = {
	{ .compatible = "dw,dsv-dw8768", },
	{ }
};
MODULE_DEVICE_TABLE(of, dw8768_of_match);
#endif

static struct i2c_driver dw8768_driver = {
	.probe = dw8768_probe,
	.remove = dw8768_remove,
	.driver = {
		.name = "dw8768",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dw8768_of_match),
	},
	.id_table = dw8768_ids,
};
module_i2c_driver(dw8768_driver);

MODULE_DESCRIPTION("dw8768 MFD Core");
