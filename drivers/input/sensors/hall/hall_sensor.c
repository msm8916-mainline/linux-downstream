#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#define DRIVER_NAME "hall_sensor"
#define DEV_NAME "HALL_SENSOR"
#define DEBOUNCE_TIME 1
static struct workqueue_struct *hall_sensor_wq;
static struct kobject *hall_sensor_kobj;
static struct platform_device *pdev;
struct wake_lock Wake_Lock;
static struct hall_sensor_str {
 	int irq;
	int status;
	int gpio;
	int enable; 
	spinlock_t mHallSensorLock;
	struct input_dev *lid_indev;
 	struct delayed_work hall_sensor_work;
}* hall_sensor_dev;

//this file node can show the hall sensor action status
static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (!hall_sensor_dev)
	    return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	char *MSG = NULL;
	if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        if (!request)
            hall_sensor_dev->status = 0;
	else
            hall_sensor_dev->status = 1;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->lid_indev);
        pr_info("[%s] SW_LID rewite value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
	return count;
}

//This file node can enable or disable hall sensor function
static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (!hall_sensor_dev)
            return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}
static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	char *MSG = NULL;
	if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
	sscanf(buf, "%du", &request);
	if (!!request==hall_sensor_dev->enable){
	    return count;
	}
	else {
	    unsigned long flags;
	    spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	    hall_sensor_dev->enable=!!request;
	    spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	return count;
}

//This file node can direct control the gpio high/low which used by hall sensor
static ssize_t show_gpio_status(struct device *dev, struct device_attribute *attr, char *buf)
{
        if (!hall_sensor_dev)
            return sprintf(buf, "Hall sensor does not exist!\n");
        return sprintf(buf, "GPIO:[%d],Status:[%d]\n",hall_sensor_dev->gpio,gpio_get_value(hall_sensor_dev->gpio));
}
static ssize_t store_gpio_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int request;
        char *MSG = NULL;
        if (!hall_sensor_dev)
            return sprintf(MSG, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        if (request > 1){
            return count;
        }
        else {
            unsigned long flags;
            spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
            gpio_direction_output(hall_sensor_dev->gpio,request);
            spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        }
        return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO|S_IWUSR, show_action_status, store_action_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);
static SENSOR_DEVICE_ATTR_2(gpio_status, S_IRUGO|S_IWUSR, show_gpio_status, store_gpio_status, 0, 0);

static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
        &sensor_dev_attr_gpio_status.dev_attr.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};

static int lid_input_device_create(void)
{
	int err = 0;

	hall_sensor_dev->lid_indev = input_allocate_device();     
	if (!hall_sensor_dev->lid_indev){
	    pr_info("[%s] lid_indev allocation fails\n", DRIVER_NAME);
	    err = -ENOMEM;
	    goto exit;
	}

	hall_sensor_dev->lid_indev->name = "lid_input";
	hall_sensor_dev->lid_indev->phys= "/dev/input/lid_indev";
	hall_sensor_dev->lid_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->lid_indev, EV_SW, SW_LID);

	err = input_register_device(hall_sensor_dev->lid_indev);
	if (err) {
	    pr_info("[%s] input registration fails\n", DRIVER_NAME);
	    err = -1;
	    goto exit_input_free;
	}
	return 0;
exit_input_free:
       input_free_device(hall_sensor_dev->lid_indev);
       hall_sensor_dev->lid_indev = NULL;
exit:
       return err;
}

static void lid_report_function(struct work_struct *dat)
{
        unsigned long flags;
        msleep(DEBOUNCE_TIME);
        if (!hall_sensor_dev->enable){
            pr_info("[%s] disable hall sensor becasue user!\n", DRIVER_NAME);
	    wake_unlock(&Wake_Lock);
	    return;
        }

        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        if (gpio_get_value(hall_sensor_dev->gpio) > 0)
            hall_sensor_dev->status = 1;
        else
            hall_sensor_dev->status = 0;
        spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);

        input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->lid_indev);
	wake_unlock(&Wake_Lock);
        pr_info("[%s] SW_LID report value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);

}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	pr_info("[%s] hall_sensor_interrupt = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	wake_lock(&Wake_Lock);
	return IRQ_HANDLED;
}

static int set_irq_hall_sensor(void)
{
	int rc = 0 ;

	hall_sensor_dev->irq = gpio_to_irq(hall_sensor_dev->gpio);
	pr_info("[%s] hall_sensor irq = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	rc = request_irq(hall_sensor_dev->irq,hall_sensor_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"hall_sensor_irq",hall_sensor_dev);
	if (rc<0) {
	    pr_info("[%s]Couldn't register for hall sensor interrupt,irq = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq,rc);
	    rc = -EIO;
	    goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(hall_sensor_dev->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}


//+++++++++++++for pm_ops callback+++++++++++++++

static int lid_suspend_noirq(struct device *dev){
	return 0;
}
static int lid_resume_noirq(struct device *dev){
	return 0;
}
static int lid_suspend_prepare(struct device *dev){
	return 0;
}

static void lid_resume_complete(struct device *dev){

}

static int lid_probe(struct platform_device *pdev){
	int ret = 0;
	pr_info("=====[%s]Probe Start=====\n",DEV_NAME);
        //set file node
        hall_sensor_kobj = kobject_create_and_add("hall_sensor_kobject", kernel_kobj);
        if (!hall_sensor_kobj){
            pr_info("[%s_ERROR] hall_sensor_kobject fails for hall sensor\n", DRIVER_NAME);
            platform_device_unregister(pdev);
            return -ENOMEM;
        }
        ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
        if (ret){
            goto fail_for_hall_sensor;
        }

        //Memory allocation
        hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
        if (!hall_sensor_dev){
            pr_info("[%s_ERROR] Memory allocation fails for hall sensor\n", DRIVER_NAME);
            ret = -ENOMEM;
            goto fail_for_hall_sensor;
        }
        spin_lock_init(&hall_sensor_dev->mHallSensorLock);
        wake_lock_init(&Wake_Lock, WAKE_LOCK_SUSPEND, "lid_suspend_blocker");
        hall_sensor_dev->enable = 1;

        //set gpio
        hall_sensor_dev->gpio = of_get_named_gpio_flags(pdev->dev.of_node,"YOBON,hall-intr-gpio",0,0);
	printk("hall_sensor GPIO=%d\n", hall_sensor_dev->gpio);
        if (!gpio_is_valid(hall_sensor_dev->gpio) || hall_sensor_dev->gpio == 0){
             pr_info("[%s_ERROR] GPIO:[%d] for hall sensor does not exist.\n", DRIVER_NAME,hall_sensor_dev->gpio);
             ret= -1;
             goto fail_for_set_gpio_hall_sensor;
        }
        gpio_request(hall_sensor_dev->gpio,"hall_sensor_gpio");
        gpio_direction_input(hall_sensor_dev->gpio);

        //init workqueue & start detect signal
        hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
        INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, lid_report_function);

        //create input_dev
        hall_sensor_dev->lid_indev = NULL;
        ret = lid_input_device_create();
        if (ret < 0)
            goto fail_for_create_input_dev;

        //set irq
        ret = set_irq_hall_sensor();
        if (ret < 0)
            goto fail_for_irq_hall_sensor;

        queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
        pr_info("=====[%s]Probe Success=====\n",DEV_NAME);
	return 0;

fail_for_create_input_dev:
        free_irq(hall_sensor_dev->irq, hall_sensor_dev);

fail_for_irq_hall_sensor:
        gpio_free(hall_sensor_dev->gpio);

fail_for_set_gpio_hall_sensor:
        kfree(hall_sensor_dev);
        hall_sensor_dev=NULL;

fail_for_hall_sensor:
        kobject_put(hall_sensor_kobj);
        return ret;
}

static const struct dev_pm_ops lid_dev_pm_ops = {
	.prepare	 = lid_suspend_prepare,
	.suspend_noirq	 = lid_suspend_noirq,
	.complete	 = lid_resume_complete,
	.resume_noirq	 = lid_resume_noirq,
};

static const struct platform_device_id lid_id_table[] = {
        {DRIVER_NAME, 1},
};

static const struct of_device_id hall_sensor_of_match[] = {
        { .compatible = "YOBON,YB8251ST23"},
	{}
};

static struct platform_driver lid_platform_driver = {
	.driver = {
	     .name  = DEV_NAME,
	     .owner = THIS_MODULE,
	     .pm    = &lid_dev_pm_ops,
	     .of_match_table = hall_sensor_of_match,
	},
	.probe          = lid_probe,
};

//----------------for pm_ops callback----------------

static int __init hall_sensor_init(void)
{	
	int ret = 0;

        pr_info("++++++[%s]initial!++++++\n",DRIVER_NAME);
	//insert pm_ops
	pdev= platform_device_alloc(DRIVER_NAME,-1);
	if (!pdev){
            pr_info("[%s_ERROR]platform device alloc fail\n",DRIVER_NAME);
	    return -1;
        }
	ret = platform_device_add(pdev);
        if (ret){
            pr_info("[%s_ERROR]platform add device fail\n",DRIVER_NAME);
            return -1;
        }
	ret = platform_driver_register(&lid_platform_driver);
	if (ret){
            pr_info("[%s_ERROR]platform register fail\n",DRIVER_NAME);
	    return ret;
        }
	return 0;
}

static void __exit hall_sensor_exit(void)
{
	gpio_free(hall_sensor_dev->gpio);
	free_irq(hall_sensor_dev->irq, hall_sensor_dev);
	input_free_device(hall_sensor_dev->lid_indev);
	hall_sensor_dev->lid_indev=NULL;
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	kobject_put(hall_sensor_kobj);
	platform_driver_unregister(&lid_platform_driver);
	platform_device_unregister(pdev);
	wake_lock_destroy(&Wake_Lock);
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_AUTHOR("David Shih <David2_Shih@asus.com>");
MODULE_AUTHOR("Brandon Kao <Brandon_Kao@asus.com>");
MODULE_DESCRIPTION("Qcom hall sensor Driver");
MODULE_LICENSE("GPL v2");
