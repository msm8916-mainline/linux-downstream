/*
 * =====================================================================================
 *
 *       Filename:  vivo_sensor_config.c
 *
 *    Description:  The motivation for editing this file is mainly for those projects 
					which  didn't have project macro in kernel, such as PDxxxx,cos we 
					wanna the boot is shared used. We filled alsps para relatived with 
					projects in dts and parse the device tree. In the foundation of canceling
					kernel macro. Any macro need to be used in kernel please inform author
					to add property in dts.
 *
 *        Version:  1.0
 *        Created:  03/31/2015 10:29:20 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  haosirong <haosirong@vivo.com.cn>, 
 *        Company:  
 *
 * =====================================================================================
 */
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/vivo_sensor_config.h>

#define VSC_TAG "[SENSOR_CONFIG>]"
#define VSC_LOG(fmt,args...) printk(KERN_INFO VSC_TAG fmt,##args)
#define VSC_ERR(fmt,args...) printk(KERN_ERR VSC_TAG fmt,##args)

//we referenced the global paras vars in alsps driver
struct ps_para *bbk_ps_paras = NULL;
struct als_para *bbk_als_paras = NULL;
int bbk_ps_para_count = 0,bbk_als_para_count = 0;
unsigned char bbk_product_model_for_alsps[20] = {0};
EXPORT_SYMBOL_GPL(bbk_ps_paras);
EXPORT_SYMBOL_GPL(bbk_als_paras);
EXPORT_SYMBOL_GPL(bbk_ps_para_count);
EXPORT_SYMBOL_GPL(bbk_als_para_count);
EXPORT_SYMBOL_GPL(bbk_product_model_for_alsps);

//preserve config property for replacing macro
static sensor_config config_obj;

static struct platform_driver vivo_sensor_config_driver;


/*********************************************/
/* parse alsps para in device tree*/
/*********************************************/

static void create_ps_para(struct ps_para* ac_ps_para,
int base_val,int low_thres,int high_thres,int step_num,struct ps_step *ac_steps){
	memset(ac_ps_para,0,sizeof(struct ps_para));
	
	ac_ps_para->base_value = base_val;
	ac_ps_para->base_value_min = low_thres;
	ac_ps_para->base_value_max = high_thres;
	ac_ps_para->step_num = step_num;
	
	memcpy(ac_ps_para->step,ac_steps,sizeof(struct ps_step)*step_num);
}

static void create_als_para(struct als_para* ac_als_para,int gain,int low_thres,int high_thres){
	memset(ac_als_para,0,sizeof(struct als_para));
	
	ac_als_para->base_value = gain;
	ac_als_para->base_value_min = low_thres;
	ac_als_para->base_value_max = high_thres;

}

static void parse_step(u32* step_data,struct ps_step *ac_step){
	//memset(ac_step,0,sizeof(struct ps_step));
	ac_step->min = step_data[0];
	ac_step->max = step_data[1];
	ac_step->in_coefficient = step_data[2];
	ac_step->out_coefficient = step_data[3];
}

static int parse_ps_para(struct device_node *ps_para_dn){
	int rc = -1;
	int i;
	char prop_step[10] = "";

	u32 base_val,low_thres,high_thres,step_num;
	u32 step_data[4];
	struct ps_step *steps = NULL;
	struct ps_para temp_ps_para;
	
	rc = of_property_read_u32(ps_para_dn,"base-val",&base_val);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read base-val\n");
		return rc;
	}	
	of_property_read_u32(ps_para_dn,"low-thres",&low_thres);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read low-thres\n");
		return rc;
	}	
	of_property_read_u32(ps_para_dn,"high-thres",&high_thres);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read high-thres\n");
		return rc;
	}	
	of_property_read_u32(ps_para_dn,"step-num",&step_num);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read step-num\n");
		return rc;
	}	
	
	steps = kzalloc(sizeof(struct ps_step)*step_num, GFP_KERNEL);
	
	for(i = 0;i < step_num;i++){
		snprintf(prop_step,sizeof(prop_step),"step%d",i);
		of_property_read_u32_array(ps_para_dn,prop_step,step_data,ARRAY_SIZE(step_data));
		if(rc && (rc != -EINVAL)){
			VSC_ERR("Unable to read %s\n",prop_step);
			return rc;
		}
		parse_step(step_data,&steps[i]);

	}
	
	bbk_ps_para_count++;
	bbk_ps_paras = krealloc(bbk_ps_paras,sizeof(struct ps_para)*bbk_ps_para_count,GFP_KERNEL);
	
	create_ps_para(&temp_ps_para,base_val,low_thres,high_thres,step_num,steps);
	
	memcpy(bbk_ps_paras + (bbk_ps_para_count-1),&temp_ps_para,sizeof(struct ps_para));
	
	kfree(steps);
	return rc;
}

static int parse_als_para(struct device_node *als_para_dn){
	int rc = -1;
	u32 gain,low_thres,high_thres;
	struct als_para temp_als_para;
	
	rc = of_property_read_u32(als_para_dn,"gain",&gain);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read gain\n");
		return rc;
	}	
	
	rc = of_property_read_u32(als_para_dn,"low-thres",&low_thres);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read base-val\n");
		return rc;
	}	
	
	rc = of_property_read_u32(als_para_dn,"high-thres",&high_thres);
	if(rc && (rc != -EINVAL)){
		VSC_ERR("Unable to read base-val\n");
		return rc;
	}	
	
	bbk_als_para_count++;
	bbk_als_paras = krealloc(bbk_als_paras,sizeof(struct als_para)*bbk_ps_para_count,GFP_KERNEL);
	
	create_als_para(&temp_als_para,gain,low_thres,high_thres);
	
	memcpy(bbk_als_paras + (bbk_als_para_count-1),&temp_als_para,sizeof(struct als_para));
	return rc;
}

/*================================================================================================*/
/*debug sys fs interfaces                                          */
/*================================================================================================*/

static ssize_t vivo_sensor_config_debug_show(struct device_driver *driver,char *buf)
{
	int i,j;
    int cnt;
	char step[30] = "";
	char als_info[30] ="";
	char ps_info[30] ="";
	
	VSC_LOG("debug show\n");
	snprintf(buf,PAGE_SIZE,"%s als and ps parameter:\n",config_obj.product_name);
	for(i = 0;i < bbk_ps_para_count;i++){
		snprintf(ps_info,30,"ps_info%d:%d,%d,%d,%d\n",i,bbk_ps_paras[i].base_value,bbk_ps_paras[i].base_value_min,bbk_ps_paras[i].base_value_max,bbk_ps_paras[i].step_num);
		snprintf(buf,PAGE_SIZE,"%s%s",buf,ps_info);
		for(j = 0;j < bbk_ps_paras[i].step_num;j++){
			snprintf(step,30,"%d,%d,%d,%d\n",bbk_ps_paras[i].step[j].min,bbk_ps_paras[i].step[j].max,bbk_ps_paras[i].step[j].in_coefficient,bbk_ps_paras[i].step[j].out_coefficient);
			snprintf(buf,PAGE_SIZE,"%s%s",buf,step);
		}
	}
	
	for(i = 0;i < bbk_als_para_count;i++){
		snprintf(als_info,30,"als_info%d:%d,%d,%d\n",i,bbk_als_paras[i].base_value,bbk_als_paras[i].base_value_min,bbk_als_paras[i].base_value_max);
		snprintf(buf,PAGE_SIZE,"%s%s",buf,als_info);
	}
	
	cnt = snprintf(buf, PAGE_SIZE, "%s",buf);
	
	return cnt;
}


static DRIVER_ATTR(debug, 0644,vivo_sensor_config_debug_show,NULL);

static struct driver_attribute * vivo_sensor_config_debug_attrs[] = {
     &driver_attr_debug,
};

static int  vivo_sensor_config_debug_create(struct device_driver *driver)
{
    int i; 
	int rc;
	int cnt = sizeof(vivo_sensor_config_debug_attrs)/sizeof(vivo_sensor_config_debug_attrs[0]);
	for (i=0; i<cnt; i++) {
	    rc = driver_create_file(driver,vivo_sensor_config_debug_attrs[i]);
		if (rc < 0) {
			VSC_ERR("%s: Failed to driver_create_file (%s) = %d\n",__func__,vivo_sensor_config_debug_attrs[i]->attr.name, rc);
			goto err_sysfs;
		}
	}
	
err_sysfs:
    return rc;
}


static int vivo_sensor_config_load_product_model(struct platform_device *pdev)
{
	const char *product_name = NULL;
    const char *product_model = "vivo,product-model";
	
	if(of_property_read_string(pdev->dev.of_node,product_model,&product_name) < 0){
		VSC_ERR("sensor-cfg: Property '%s' could not be read\n",product_model);
		return -EINVAL;
	}
		
	config_obj.product_name = product_name;
	snprintf(bbk_product_model_for_alsps,sizeof(bbk_product_model_for_alsps),"%s",config_obj.product_name);
	return 0;
}

static int vivo_sensor_config_load_paras_device_tree(struct platform_device *pdev){
	struct device_node *alsps_para_dn = NULL;
	for_each_child_of_node(pdev->dev.of_node,alsps_para_dn){
		if(strstr(alsps_para_dn->name,"ps-para")){
			if(parse_ps_para(alsps_para_dn) < 0){
				VSC_ERR("parse node %s error\n",alsps_para_dn->name);
				return -1;
			}	
		}
		else if(strstr(alsps_para_dn->name,"als-para")){
			if(parse_als_para(alsps_para_dn) < 0){
				VSC_ERR("parse node %s error\n",alsps_para_dn->name);
				return -1;
			}
		}
		else{
			VSC_ERR("parse sensor config error for node name %s\n",alsps_para_dn->name);
			return -1;
		}
	}
	return 0;
}
static int vivo_sensor_config_probe(struct platform_device *pdev)
{
	VSC_LOG("%s\n",__FUNCTION__);
    memset((void*)&config_obj,0,sizeof(config_obj));
	
	if(vivo_sensor_config_load_paras_device_tree(pdev) < 0)
		goto exit;
		
	if(vivo_sensor_config_load_product_model(pdev) < 0)
		goto exit;
	
	if(vivo_sensor_config_debug_create(&vivo_sensor_config_driver.driver) < 0)
		goto exit;
	return 0;

exit:
	VSC_LOG("%s probe but exit \n",__FUNCTION__);
	if(bbk_ps_paras != NULL)
		kfree(bbk_ps_paras);
	if(bbk_als_paras != NULL)
		kfree(bbk_als_paras);
	bbk_ps_para_count = bbk_als_para_count = 0;
	
	return -1;
}
 
static int vivo_sensor_config_remove(struct platform_device *pdev)
{
	VSC_LOG("%s\n",__FUNCTION__);
	return 0;
}
static const struct of_device_id vivo_sensor_config_of_match[]  = {
	{ .compatible = "vivo,sensor-config", },
	{},
};

static struct platform_driver vivo_sensor_config_driver = {
	.driver = {
		.name = "sensor-config",
		.owner = THIS_MODULE,
		.of_match_table = vivo_sensor_config_of_match,
	},
	.probe = vivo_sensor_config_probe,
	.remove = vivo_sensor_config_remove,
};


static int __init vivo_sensor_config_init(void)
{
	VSC_LOG("%s\n",__FUNCTION__); 
	if(platform_driver_register(&vivo_sensor_config_driver) < 0){
		VSC_ERR("failed to register sensor config driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit vivo_sensor_config_exit(void)
{
	VSC_LOG("%s\n",__FUNCTION__);
	platform_driver_unregister(&vivo_sensor_config_driver);
}

arch_initcall(vivo_sensor_config_init);
module_exit(vivo_sensor_config_exit);

MODULE_AUTHOR("haosirong <haosirong@vivo.com.cn>");
MODULE_DESCRIPTION("sensor config and parameter mainly for proximity and ambient light");
MODULE_LICENSE("GPL");
