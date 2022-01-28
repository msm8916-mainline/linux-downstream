#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#define MAX_TIMES		7

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vcc_fpc", 1800000UL, 1800000UL, 10, },
	{ "vcc_goodix", 2800000UL, 2800000UL, 10, },
};
static const char * const pctl_names[] = {
	"fp_gpio_pull_up",
	"fp_gpio_pull_down",
};

static int fp_id = -1;
const char * fp_project_name;
struct regulator *fp_vreg[ARRAY_SIZE(vreg_conf)];
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

int get_fp_id(void);

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

static int vreg_setup(struct device *dev, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	
	printk("vreg_setup start\n");
	for (i = 0; i < ARRAY_SIZE(fp_vreg); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fp_vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (!vreg) {
				dev_err(dev, "Unable to get  %s\n", name);
				return -ENODEV;
			}
		}
		printk("vreg_setup 111111\n");
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fp_vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fp_vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static int select_pin_ctl(struct device *dev, const char *name)
{
	size_t i;
	int rc;
	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fingerprint_pinctrl,pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	 return sprintf(buf, "%d\n",get_fp_id());
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	printk("fp_id cannot be writed.\n");
	return 0;
}

int get_fp_id(void)
{
	printk("get_fp_id enter, fp_id=%d\n", fp_id);
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};
static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

struct kobject kobj;
static int
fp_id_probe(struct platform_device *pdev)
{
	int ret;
	int fp_gpio,fp_up,fp_down;
	int i;
	fp_id = 0;

	ret = kobject_init_and_add(&kobj, &fp_id_object_type,NULL, "fp_id");
	if (ret)
	{
		printk("%s: Create fp_id error!\n", __func__);
		return -1;
	}

	fp_gpio = of_get_named_gpio(pdev->dev.of_node,"fp_id,gpios",0);
	if(fp_gpio < 0)
	{
		printk("%s: get fp_id gpio failed!\n", __func__);
		return -1;	
	}
	printk("%s:fp gpio: %d \n", __func__,fp_gpio);
	
	ret = devm_gpio_request(&pdev->dev,fp_gpio,"fp_id,gpios");
	if (ret)
	{
		printk("%s: request fp_id gpio failed!\n", __func__);
		return -1;
	}
	
	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		if (PTR_ERR(fingerprint_pinctrl) == -EPROBE_DEFER) {
			printk("%s: pinctrl not ready!\n", __func__);
			return -1;
		}
		printk("%s: Target does not use pinctrl\n", __func__);
		fingerprint_pinctrl = NULL;
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			printk("%s: cannot find '%s'\n", __func__, n);
			return -1;
		}
		printk("%s: found pin control %s\n", __func__, n);
		pinctrl_state[i] = state;
	}
	
	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name",&fp_project_name);
	if(ret) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
	}
	printk("%s:vivo,project-name = %s\n", __func__, fp_project_name);
	if(!strncmp(fp_project_name, "PD1523", 6))
	{
		printk("PD1523 directly return 1\n");
		fp_id = 1;
		ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
		if (ret)
			printk("%s: set fp-id pull up error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
		return 0;
	}

	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
	mdelay(5);
	if (ret)
		return -1;
	fp_up = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull up,get gpio value = %d\n", __func__, fp_up);
	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
	mdelay(5);
	if (ret)
		return -1;
	fp_down = gpio_get_value(fp_gpio);
	printk("%s: set fp-id pull down,get gpio value = %d\n", __func__, fp_down);
	
	if((fp_up==1)&&(fp_down==1))
	{
		fp_id = 1;
		ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
		if (ret)
			printk("%s: set fp-id pull up error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
	}
	if((fp_up==0)&&(fp_down==0))
	{
		fp_id = 2;
		ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		if (ret)
			printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
		ret = vreg_setup(&pdev->dev, "vcc_fpc", false);
		if (ret)
			return -1;
	}
	return 0;
}

static int
fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};

static struct platform_driver fp_id_driver = {
    .probe      = fp_id_probe,
    .remove     = fp_id_remove,
    .driver = {
        .name   = "fp_id",
        .owner  = THIS_MODULE,
        .of_match_table = fp_id_match_table,
    },
};

static int __init fp_id_init(void)
{

    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);

}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");
