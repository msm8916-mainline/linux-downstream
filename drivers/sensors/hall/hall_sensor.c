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
//#include <asm/intel-mid.h>

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef HALL_DEBUG
	#define dbg(fmt, args...) printk("[%s] "fmt,DRIVER_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk("[%s] "fmt,DRIVER_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)

/*****************************/
/* Hall Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"hall_sensor"
#define GPIO_NAME 		"hall_det#"
#define IRQ_Name			"ASUS_Hall_Sensor-irq"
#define INT_NAME			"HallSensor_INT"
#define KERNEL_OBJECT	"hall_sensor_kobject"
#define WAKE_LOCK_TIME	(800)

/**************************/
/* Driver Data Structure */
/*************************/
static struct hall_sensor_str { 	
	int status;	
	int enable; 
	int debounce;
	spinlock_t mHallSensorLock;
	struct wake_lock wake_lock;
	struct input_dev *hall_indev;	
 	struct delayed_work hall_sensor_work;
	
}* hall_sensor_dev;

/*******************************/
/* Hall Sensor Global Variables */
/******************************/
static int 							ASUS_HALL_SENSOR_GPIO;
static int 							ASUS_HALL_SENSOR_IRQ;
static struct workqueue_struct 	*hall_sensor_wq;
//static struct platform_device 	*pdev;
/*
static struct input_device_id mID[] = {
        { .driver_info = 1 },		//scan all device to match hall sensor
        { },
};
*/


/*===============================
 *|| Interrupt Service Routine part ||
 *===============================
 */
static void hall_sensor_report_function(struct work_struct *dat)
{
	  int GPIO_value; 

	msleep(50);
        if(!hall_sensor_dev->enable){
                log("[ISR] hall sensor is disable!\n");
		goto IST_RET;
        }
        
        if (gpio_get_value(ASUS_HALL_SENSOR_GPIO) > 0)
			GPIO_value = 1;
        else
                	GPIO_value = 0;		
	  
	  if(GPIO_value != hall_sensor_dev->status){
		hall_sensor_dev->status =GPIO_value;
	  }else{
		goto IST_RET;
	  }
        		
        input_report_switch(hall_sensor_dev->hall_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->hall_indev);
        log("[ISR] report value = %s\n", hall_sensor_dev->status?"open":"close");
IST_RET:
	//wake_unlock(&hall_sensor_dev->wake_lock);
	return;

}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	int GPIO_value; 
	
	
	cancel_delayed_work(&hall_sensor_dev->hall_sensor_work);

	/*get GPIO status*/
	if (gpio_get_value(ASUS_HALL_SENSOR_GPIO) > 0) GPIO_value = 1;
      else GPIO_value = 0;		
	log("[ISR] hall_sensor_interrupt = %s\n",GPIO_value?"open":"close");
	
	/*start work queue when status change*/
	if(GPIO_value != hall_sensor_dev->status){
		queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 
			msecs_to_jiffies(hall_sensor_dev->debounce));
		wake_lock_timeout(&hall_sensor_dev->wake_lock, msecs_to_jiffies(WAKE_LOCK_TIME));
	}	
	
	return IRQ_HANDLED;
}

/*===========================
 *|| sysfs DEVICE_ATTR part ||
 *===========================
 *
 */
static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	
	//if(!hall_sensor_dev)
       //         return sprintf(buf, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
		
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        	if (!request)
                	hall_sensor_dev->status = 0;
	 	else
        		hall_sensor_dev->status = 1;
	 spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	
        log("[ATTR] status rewite value = %d\n",!hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if(!hall_sensor_dev)
                return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	
	if(!hall_sensor_dev) {
		err("Hall sensor does not exist!\n");
		return 0;
	}
	
	sscanf(buf, "%du", &request);
	
	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		if(0 == request) {
			/* Turn off */
			log("[ATTR] Turn off.\n");
			
			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
			
		}else if(1 == request){
			/* Turn on */
			log("[ATTR] Turn on. \n");
			
			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
			
		}else{
			err("[ATTR] Enable/Disable Error, can not recognize (%d)", request);
		}
		
	}
	return count;
}

static ssize_t show_hall_sensor_debounce(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",hall_sensor_dev->debounce);
}

static ssize_t store_hall_sensor_debounce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int debounce;
	sscanf(buf, "%du", &debounce);
	hall_sensor_dev->debounce = debounce;
	
	return count;
}


static DEVICE_ATTR(status, 0666, show_action_status, store_action_status);
static DEVICE_ATTR(switch, 0666,show_hall_sensor_enable, store_hall_sensor_enable);
static DEVICE_ATTR(debounce, 0666,show_hall_sensor_debounce, store_hall_sensor_debounce);

static struct attribute *hall_sensor_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_switch.attr,
	&dev_attr_debounce.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};


/*====================
 *|| Initialization Part ||
 *====================
 * 
 */

static int init_input_event(void)
{
	int ret = 0;

	hall_sensor_dev->hall_indev = input_allocate_device();     
	if(!hall_sensor_dev->hall_indev){
		err("[Input] Failed to allocate input event device\n");
		return -ENOMEM;		
	}

	hall_sensor_dev->hall_indev->name = "hall_input";
	hall_sensor_dev->hall_indev->phys= "/dev/input/hall_indev";
	hall_sensor_dev->hall_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->hall_indev, EV_SW, SW_LID);

	ret = input_register_device(hall_sensor_dev->hall_indev);
	if (ret) {
		err("[Input] Failed to register input event device\n");
		return -1;		
	}
		
	log("[Input] Input Event registration Success!\n");
	return 0;
}

static int init_data(void)
{
	int ret = 0;
	
	/* Memory allocation for data structure */
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		err("Memory allocation fails for hall sensor\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	wake_lock_init(&hall_sensor_dev->wake_lock, WAKE_LOCK_SUSPEND, "HallSensor_wake_lock");
	
	hall_sensor_dev->enable = 1;

	/*ZE500KL*/
	if ((ZE500KL_EVB==(g_ASUS_hwID&0xF0)&&g_ASUS_hwID <= ZE500KL_ER2) ||
		(ZE500KG_EVB==(g_ASUS_hwID&0xF0)&&g_ASUS_hwID <= ZE500KG_ER2)){
		hall_sensor_dev->debounce = 500;
	}else if((ZE500KL_EVB==(g_ASUS_hwID&0xF0)&&g_ASUS_hwID <= ZE500KL_PR) ||
		(ZE500KG_EVB==(g_ASUS_hwID&0xF0)&&g_ASUS_hwID <= ZE500KG_PR)){
		hall_sensor_dev->debounce = 200;
	}else{
		hall_sensor_dev->debounce = 30;
	}

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "hall_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	log("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}

static int init_irq (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	ASUS_HALL_SENSOR_IRQ = gpio_to_irq(ASUS_HALL_SENSOR_GPIO);
	
	if (ASUS_HALL_SENSOR_IRQ < 0) {
		err("[IRQ] gpio_to_irq ERROR, irq=%d.\n", ASUS_HALL_SENSOR_IRQ);
	}else {
		log("[IRQ] gpio_to_irq IRQ %d successed on GPIO:%d\n", ASUS_HALL_SENSOR_IRQ, ASUS_HALL_SENSOR_GPIO);
	}

	/*Request IRQ */
	//ret = request_irq(ASUS_HALL_SENSOR_IRQ,
	//		hall_sensor_interrupt_handler, 
	//		IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, 
	//		INT_NAME, hall_sensor_dev);
	ret = request_threaded_irq(ASUS_HALL_SENSOR_IRQ, NULL, hall_sensor_interrupt_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				INT_NAME, hall_sensor_dev);

	if (ret < 0)
		err("[IRQ] request_irq() ERROR %d.\n", ret);
	else {
		dbg("[IRQ] Enable irq !! \n");
		enable_irq_wake(ASUS_HALL_SENSOR_IRQ);
	}
	
	return 0;
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

log("Probe +++\n");

	/* Initialization Data */
	ret = init_data();
	if (ret < 0)
		goto probe_err;

	set_pinctrl(&pdev->dev);
	
	/* GPIO */
	ASUS_HALL_SENSOR_GPIO = of_get_named_gpio(pdev->dev.of_node, "qcom,hall-gpio", 0);	
	log("[GPIO] GPIO =%d(%d)\n", ASUS_HALL_SENSOR_GPIO, gpio_get_value(ASUS_HALL_SENSOR_GPIO));	

	/* GPIO Request */
	ret = gpio_request(ASUS_HALL_SENSOR_GPIO, IRQ_Name);
	if (ret) {
		err("[GPIO] Unable to request gpio %s(%d)\n", IRQ_Name, ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_HALL_SENSOR_GPIO);
	if (ret < 0) {
		err("[GPIO] Unable to set the direction of gpio %d\n", ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	/* IRQ */
	ret = init_irq();
	if (ret < 0)
		goto probe_err;

	/* sysfs */
	ret = sysfs_create_group(&pdev->dev.kobj, &hall_sensor_group);
	if (ret) {
		err("Hall sensor sysfs_create_group ERROR.\n");
		goto probe_err;
	}	

	/* Work Queue */
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, hall_sensor_report_function);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);

	/* Input Device */
	ret = init_input_event();
	if (ret < 0)
		goto probe_err;

	device_rename(&pdev->dev, "hall_sensor");
	return 0;
	
log("Probe ---\n");
	return 0;

probe_err:
	err("Probe ERROR\n");
	return ret;
	
}

static int hall_sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	log("Suspend +++---\n");
	return 0;
}

static int hall_sensor_resume(struct platform_device *pdev)
{
	log("Resume +++---\n");
	return 0;
}

static const struct platform_device_id hall_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct of_device_id hallsensor_match_table[] = {
	{ .compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hallsensor_match_table,
	},
	.probe          = hall_sensor_probe,	
	.suspend	= hall_sensor_suspend,
	.resume	= hall_sensor_resume,
	.id_table	= hall_id_table,
};

//----------------for pm_ops callback----------------

static int __init hall_sensor_init(void)
{	
	int err = 0;
	log("Driver INIT +++\n");

	/************************************************* 
	 * SR device out of function ( L18 power source shut down by camera ) 
	 * without register HALL sensor in SR device
	 */
	if (g_ASUS_hwID > ZE500KL_SR2)	{
		/* Platform Driver Registeration */
		err = platform_driver_register(&hall_sensor_driver);
		if (err != 0)
			err("[platform] platform_driver_register fail, Error : %d\n", err);
	}
	else
		err("[platform] SR device(%d) bypass platform_driver_register HALL sensor\n", g_ASUS_hwID);
	
	log("Driver INIT ---\n");

	return err;
}

static void __exit hall_sensor_exit(void)
{
	log("Driver EXIT +++\n");

	free_irq(ASUS_HALL_SENSOR_IRQ, hall_sensor_dev);
	gpio_free(ASUS_HALL_SENSOR_GPIO);	
	//input_free_device(hall_sensor_dev->hall_indev);
	//hall_sensor_dev->hall_indev=NULL;
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	wake_lock_destroy(&hall_sensor_dev->wake_lock);
	platform_driver_unregister(&hall_sensor_driver);	
	
	
	log("Driver EXIT ---\n");
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Intel APX9131 Hall Sensor");
MODULE_LICENSE("GPL v2");
