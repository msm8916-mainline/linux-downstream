/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 #include <linux/init.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/cpuidle.h>
#include <linux/gpio.h>


#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

//*****************for debug *****************//
#define DEBUG_SWITCH_CAPSENSOR 1
#define CAPSENSOR_POWER_CONTROL 1

//open the debug log
#if    DEBUG_SWITCH_CAPSENSOR
#define CAPSENSOR_DEBUG(fmt,arg...)      printk("<<-CAP SENSOR->> "fmt"\n",##arg)
#else
#define CAPSENSOR_DEBUG(fmt,args...) /*do nothing */
#endif

#define CAP_PRINT(fmt,args...) printk("<<-CAP SENSOR->>"fmt"\n",##arg)

//****************Variables and define****************//
/*
 capsensor_enable TRUE
 capsensor_disable FALSE
*/
int capsensor_switch=1;

/*
    capsensor_near TRUE
    capsensor_away FALSE
*/
int capsensor_near=0;

#if CAPSENSOR_POWER_CONTROL
#define CAP_VDD_MIN_UV 	2950000
#define CAP_VDD_MAX_UV	2950000
#endif

struct capsensor_platform_data {
	struct work_struct	work;
	const char *name;
	int irq_gpio;
};

struct capsensor_data {

    struct capsensor_platform_data *pdata;
    struct regulator *vdd;
};

struct capsensor_data *g_capsensor_pdata;


#if CAPSENSOR_POWER_CONTROL
//*****************sys file system start*****************//
static ssize_t show_CapSensor_Switch(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(1==capsensor_switch)
	  return  sprintf(buf, "capsensor is enable\n");
	else
	  return  sprintf(buf, "capsensor is disable\n");
}
static ssize_t store_CapSensor_Switch(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	if((strcmp(buf,"1\n")==0)||(strcmp(buf,"enable\n")==0))
	  {
		CAPSENSOR_DEBUG("power on iqs128");
		
		if (regulator_set_voltage(g_capsensor_pdata->vdd, 0,CAP_VDD_MAX_UV)<0) {
			printk( "iqs128 set voltage failed! \n");
			return -1;    
		}
		if (regulator_enable(g_capsensor_pdata->vdd)< 0) {
			printk( "iqs128 power on failed! \n");
			return -1;
		}  
		capsensor_switch=1;
	  }
	else if((strcmp(buf,"0\n")==0)||(strcmp(buf,"disable\n")==0))
	  {   
	  	CAPSENSOR_DEBUG("power off iqs128");
          if (regulator_disable(g_capsensor_pdata->vdd)< 0) {
			printk("iqs128 power off failed! \n");
			return -1;
		}  
		 capsensor_switch=0;
	  } 
	     else
	     {
			printk("<<-CAP SENSOR->> your input capsensor_switch=%s data is error\n",buf);
		 }
		  
     return size;
}
static DEVICE_ATTR(CapSensor_Switch, 0664, show_CapSensor_Switch, store_CapSensor_Switch);
#endif

static ssize_t show_CapSensor_Data(struct device *dev,struct device_attribute *attr, char *buf)
{
	  
	if(1==capsensor_switch)
	  {
		 
		if(gpio_get_value(g_capsensor_pdata->pdata->irq_gpio)==1)
		   {  
			   CAPSENSOR_DEBUG("away");
			   capsensor_near=0;
			   return sprintf(buf,"removed\n");
			}
		  else
		   {   CAPSENSOR_DEBUG("near");
			  capsensor_near=1;
			  return sprintf(buf,"near\n");
		   } 
	  }
	else
	{
	CAPSENSOR_DEBUG("capsensor is disable please enable it first");
	  return  sprintf(buf, "capsensor is disable please enable it first\n");
    }
}


static DEVICE_ATTR(CapSensor_Data,  0664, show_CapSensor_Data, NULL);
//*****************sys file system end*****************//

static int  capsensor_parse_dt(struct device *dev,
			struct capsensor_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_property_read_string(np, "label", &pdata->name);
	if (rc) {
		dev_err(dev, "Failed to read label\n");
		return -EINVAL;
	}

	pdata->irq_gpio = of_get_named_gpio(np,"qcom,capsensor-out", 0);


	return 0;
}
#if CAPSENSOR_POWER_CONTROL
static int capsensor_power_init(struct device *dev, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(g_capsensor_pdata->vdd) > 0)
			regulator_set_voltage(g_capsensor_pdata->vdd, 0, CAP_VDD_MAX_UV);

		regulator_put(g_capsensor_pdata->vdd);

	} else {
		g_capsensor_pdata->vdd = regulator_get(dev, "vdd");
		if (IS_ERR(g_capsensor_pdata->vdd)) {
			rc = PTR_ERR(g_capsensor_pdata->vdd);
			printk(KERN_ERR"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(g_capsensor_pdata->vdd) > 0) {
			rc = regulator_set_voltage(g_capsensor_pdata->vdd, CAP_VDD_MIN_UV,
						   CAP_VDD_MAX_UV);
			if (rc) {
				printk(KERN_ERR"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(g_capsensor_pdata->vdd);
	return rc;
}
#endif

static int capsensor_probe(struct platform_device *pdev)
{	
	int ret_device_file=0;  
	int ret,error;
	struct capsensor_data *cap_sensor;
	CAPSENSOR_DEBUG("iqs128_prob"); 

	cap_sensor = kzalloc(sizeof(struct capsensor_data), GFP_KERNEL);
	if (!cap_sensor) {
		dev_err(&pdev->dev, "capsensor not enough memory for ts\n");
		return -ENOMEM;
	}
	if (pdev->dev.of_node) {
		cap_sensor->pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct capsensor_platform_data), GFP_KERNEL);
		if (!cap_sensor->pdata) {
			dev_err(&pdev->dev,
				"cap_sensor Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = capsensor_parse_dt(&pdev->dev, cap_sensor->pdata);
		if (ret)
			return ret;
	} else {
		cap_sensor->pdata = pdev->dev.platform_data;
	}
	g_capsensor_pdata=cap_sensor;
#if CAPSENSOR_POWER_CONTROL
	//init power
	ret = capsensor_power_init(&pdev->dev, true);
	if (ret < 0) {
		dev_err(&pdev->dev, "power init failed! err=%d", ret);
	}
	ret = regulator_enable(g_capsensor_pdata->vdd);
	if (ret) {
		dev_err(&pdev->dev,"Regulator vdd enable failed ret=%d\n", ret);
	}
#endif
	//init gpio
	error = gpio_request_one(cap_sensor->pdata->irq_gpio,
					 GPIOF_IN, "capsensor-irq");
	if (error) {
			dev_err(&pdev->dev,
				"Failed to request GPIO %d, error %d\n",
				cap_sensor->pdata->irq_gpio, error);
			goto failed;
		}

	// create device file
#if CAPSENSOR_POWER_CONTROL
	ret_device_file = device_create_file(&(pdev->dev), &dev_attr_CapSensor_Switch);
#endif
	ret_device_file = device_create_file(&(pdev->dev), &dev_attr_CapSensor_Data);

	CAPSENSOR_DEBUG("iqs128_prob end"); 
	return 0;

failed:
	gpio_free(cap_sensor->pdata->irq_gpio);
	kfree(cap_sensor);
	g_capsensor_pdata = NULL;
	return 0;

}
static int capsensor_remove(struct platform_device *pdev)
{
	return 0;
}
static int capsensor_suspend(struct platform_device *pdev,pm_message_t mesg)
{
	return 0;
}

static int capsensor_resume(struct platform_device *pdev)
{
	return 0;
}
static struct of_device_id capsensor_match_table[] = {
	{ .compatible = "qcom,capsensor",},
	{ },
};

static struct platform_driver capsensor_driver = {
	.probe = capsensor_probe,
	.remove = capsensor_remove,
	.suspend = capsensor_suspend,
	.resume	= capsensor_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "capsensor",
		.of_match_table = capsensor_match_table,
	},
};
module_platform_driver(capsensor_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("capsensor Driver");
