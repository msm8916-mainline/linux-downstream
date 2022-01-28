/*
 * Copyright (C) 2014 VIVO Technologies inc.
 *   wangyuanliang created <wangyuanliang@vivo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/bbk_drivers_info.h>
//#include <mach/gpio.h>
//#include <mach/gpiomux.h>
#include <soc/qcom/socinfo.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/sndinfo_vivo.h>


static char bbk_board_version[16] = "version_invalid";
static char bbk_model_version[64] = "version_invalid";


int bbk_drivers_info_switch_register_callback(struct bbk_drivers_callback_handler *handler) 
{
	/*nothing to do */
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_drivers_info_switch_register_callback);

int set_vivo_snd_card_info(unsigned int mask, unsigned int value)
{
	/*nothing to do */
	return 0;
}

EXPORT_SYMBOL_GPL(set_vivo_snd_card_info);

int set_vivo_fm_info(unsigned int mask, unsigned int value)
{
	/*nothing to do */
	return 0;
}

EXPORT_SYMBOL_GPL(set_vivo_fm_info);



char* get_bbk_board_version(void)
{
	return bbk_board_version;
}
EXPORT_SYMBOL_GPL(get_bbk_board_version);
static __init int set_bbk_board_version(char *str)
{
	strcpy(bbk_board_version, str);
	printk(KERN_ERR "bbk board version is %s\n", bbk_board_version);
	return 0;
}
early_param("bbk_board_version", set_bbk_board_version);


static __init int set_bbk_model_version(char *str)
{
	strcpy(bbk_model_version, str);
	printk(KERN_ERR "bbk model version is %s\n", bbk_model_version);
	return 0;
}
early_param("bbk_model_version", set_bbk_model_version);


static char mpp_char_val[16] = "00";
static __init int set_mpp_char_val(char *str)
{
	strcpy(mpp_char_val, str);
	printk(KERN_ERR "mpp char is %s\n", mpp_char_val);
	return 0;
}
early_param("vol_mpp_char", set_mpp_char_val);

static char gpio_dig_val[16] = "00";
static __init int set_gpio_dig_val(char *str)
{
	strcpy(gpio_dig_val, str);
	printk(KERN_ERR "gpio digital is %s\n", gpio_dig_val);
	return 0;
}
early_param("vol_gpio_dig", set_gpio_dig_val);

static ssize_t devices_board_vsersion_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", bbk_board_version);
}


static ssize_t devices_model_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf, "%s\n", bbk_model_version);
}


 int get_hw_subtype(void) { 
    
	struct device_node *root;
	struct property *prop;
	int buf[16];
	int len;
	
	root = of_find_node_by_path("/");
	if (root) {
		prop = of_find_property(root, "qcom,board-id",&len);
		if (!prop || !prop->value) {
			printk(KERN_ERR "read qcom,board-id error!\n");
			return -1;
		}
		
		printk(KERN_ERR "hw_subtype_len = %d!\n",len);
		
		if(len == 8) {
		    memcpy(buf,prop->value,len);
			printk(KERN_ERR "hw_subtype = %d!\n",(buf[1]>>24));
			return (buf[1]>>24);
		} else {
			printk(KERN_ERR "qcom,board-id data error!\n");
		}
	} else {
		printk(KERN_ERR " Can't find root of device tree\n");
	}
		
	return -1;
}
EXPORT_SYMBOL_GPL(get_hw_subtype);


static ssize_t devices_mpps_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", bbk_model_version); 

}

static struct bbk_devices_info_list {
	struct list_head list;
	struct bbk_device_info dev_info;
}devices_list;


static struct debug_sysfs_entry board_version = 
	__ATTR(board_version, S_IRUGO, 
			devices_board_vsersion_show, NULL);

static struct debug_sysfs_entry model_value = 
	__ATTR(model_value, S_IRUGO, 
			devices_model_value_show, NULL);

static struct debug_sysfs_entry mpps_value = 
	__ATTR(mpps_value, S_IRUGO, 
			devices_mpps_value_show, NULL);


static struct attribute *sys_attrs[] = {
	&board_version.attr,
	&model_value.attr,
	&mpps_value.attr,
	
	NULL
};

static ssize_t debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops debug_object_sysfs_ops = {
	.show = debug_object_show,
	.store = debug_object_store,
};
static struct kobj_type debug_object_type = {
	.sysfs_ops	= &debug_object_sysfs_ops,
	.release	= debug_object_release,
	.default_attrs = sys_attrs,
};

static struct kobject kobject_debug;

static int creat_sys_files(void) 
{ 
    int ret; 
	
	ret = kobject_init_and_add(&kobject_debug, &debug_object_type,
					NULL, "devs_list");
    if (ret) 
    { 
        printk("%s: Create kobjetct error!\n", __func__); 
        return -1; 
    } 
	
    return 0; 
} 

int devs_create_sys_files(const struct attribute * attr)
{
	/* nothing to do temply */
	return 0;
}
EXPORT_SYMBOL_GPL(devs_create_sys_files);


static int __init bbk_drivers_switch_init(void)
{
	int error;

	printk("SYNA bbk_drivers_log_switch_init ");

	INIT_LIST_HEAD(&devices_list.list);
	error = creat_sys_files();
	if (error) {
		printk(KERN_ERR "%s: creat sysfs files failed\n", __func__);
		return error;
	}
	return 0;
}

static void __exit bbk_drivers_switch_exit(void)
{
	/*nothing to do */
}
 
arch_initcall(bbk_drivers_switch_init);
module_exit(bbk_drivers_switch_exit);
