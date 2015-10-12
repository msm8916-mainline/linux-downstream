#ifndef _HALL_AHXXX_SENSOR_H
#define _HALL_AHXXX_SENSOR_H
//add by yanfei for hall sensor 20140915
#include <linux/regulator/consumer.h>

struct hall_sensor_platform_data {
	struct work_struct	work;
/*zhouwentao add for  run in test 2015.5.12 begin*/	
	u32 delay_msc;
	u8 run_rpt_val;
	struct delayed_work	test_work;
/*zhouwentao add for  run in test 2015.5.12 end*/	
	struct input_dev  *input_dev;
	struct workqueue_struct *hall_wq;
	const char *name;
	int irq_gpio;
	int irq;
	u32 wake_key;
	u32 sleep_key;
	u32 debounce;
	int (*power_on)(bool);
//added by litao 20140610
	struct  wake_lock hall_lock;
};

struct hall_sensor_data {

    struct hall_sensor_platform_data *pdata;
    //add by yanfei for pinctrl 20140704 begin
    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_default;
     //add by yanfei for pinctrl 20140704 end

    #if defined(CONFIG_FB)
	struct notifier_block fb_notif;
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
    #endif
//add by yanfei for hall sensor power on 20140915
    struct regulator *vdd;
};


#endif//_HALL_AHXXX_SENSOR_H
