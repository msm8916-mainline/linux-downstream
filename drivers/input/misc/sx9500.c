
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/switch.h>
#include <linux/regulator/consumer.h>
#include "psensor.h"
#include "sx9500.h"
/*******add begin by lizhi.wu@tcl.com 2014-10-10***************/
#include <linux/interrupt.h>
/*******add end by lizhi.wu@tcl.com 2014-10-10***************/

struct sx9500_platform_data *pdata;
struct psensor_data *psensor_data;
extern int iqs128_init(void);
//#define DRIVER_NAME "sx9500"

//#define SX9500_CONTROL_BY_GPIO 1


typedef struct sx9500_st sx9500_t, *psx9500_t;
struct sx9500_st
{
	void *bus; /* either i2c_client or spi_client */
  /* Function Pointers */
	int (*init)(psx9500_t this); /* (re)initialize device */
  /* since we are trying to avoid knowing registers, create a pointer to a
   * common read register which would be to read what the interrupt source
   * is from 
   */
	int (*refreshStatus)(psx9500_t this); /* read register status */
	int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */
	struct delayed_work dworker; /* work struct for worker function */
};

static psx9500_t g_sx9500 = NULL;

/*! \fn static int write_register(psx86XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx9500_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;
	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;

		returnValue = i2c_master_send(i2c,buffer,2);
		dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
				address,value,returnValue);
	}
	return returnValue;
}

/*! \fn static int read_register(psx86XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx9500_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

/*! \fn static int read_regStat(psx86XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx9500_t this)
{
	u8 data = 0;
	int ret=0;

	if (!this)
		return -1;

	ret = read_register(this, SX9500_IRQSTAT_REG, &data);

	if(ret==0)
		return (data & 0x00FF);

	return ret;
}


static int psensor_gpio_config(struct device *dev)
{
	int ret = 0;

	struct device_node *node = dev->of_node;

	psensor_data->enable_gpio = of_get_named_gpio(node, "enable-gpios", 0);
	if (psensor_data->enable_gpio < 0)
	{
		dev_err(dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->enable_gpio);
		ret = -1;
	}
	else
		{
		ret = gpio_request(psensor_data->enable_gpio, "PSENSOR_ENABLE");
		if (ret)
		{
			dev_err(dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->enable_gpio, ret);
		
			ret = -1;
		}
		ret = gpio_direction_output(psensor_data->enable_gpio,1);
		if (ret)
		{
			dev_err(dev, "Unable to set direction for irq gpio [%d]\n",
				psensor_data->enable_gpio);
			gpio_free(psensor_data->enable_gpio);
			ret = -1;
		}
	
		}
	
	psensor_data->irq = of_get_named_gpio(node, "interrupt-gpios", 0);
	if (psensor_data->irq < 0)
	{
		dev_err(dev,
			"Looking up %s property in node %s failed. ret =  %d\n",
			"interrupt-gpios", node->full_name, psensor_data->irq);
		ret = -1;
	}
	else
	{
		ret = gpio_request(psensor_data->irq, "PSENSOR_INTERRUPT");
		if (ret)
		{
			dev_err(dev,
				"%s: Failed to request gpio %d,ret = %d\n",
				__func__, psensor_data->irq, ret);
		
			ret = -1;
		}
		ret = gpio_direction_input(psensor_data->irq);
		if (ret)
		{
			dev_err(dev, "Unable to set direction for irq gpio [%d]\n", psensor_data->irq);
			gpio_free(psensor_data->irq);
			ret = -1;
		}
	}

	ret = 0;

	return ret;
}

static int sx9500_nirq_is_low(void)
{
	return !gpio_get_value(psensor_data->irq);
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx9500_t this)
{
//	psx9500_platform_data_t pdata = 0;
//  	struct i2c_client *client = 0;
	int i = 0;
	u8 read=0;
	/* configure device */
	if(!this){
		printk("<2>""%s: i2c client is err!!!\n",__func__);
		return;
	}
#if 0
	client=this->bus;
	if(!client){
		printk("<2>""%s: platform data is err!!!\n",__func__);
		return;
	}

	pdata=client->dev.platform_data;
#endif
	if (pdata){
		while ( i < pdata->i2c_reg_num) {
		/* Write all registers/values contained in i2c_reg */
			printk("<2>""Going to Write Reg: 0x%x Value: 0x%x\n",pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			//      msleep(3);        
			write_register(this, pdata->pi2c_reg[i].reg, pdata->pi2c_reg[i].val);
			read_register(this, pdata->pi2c_reg[i].reg,&read);
			
			printk("<2>""read Reg: 0x%x Value: 0x%x\n",pdata->pi2c_reg[i].reg,read);
			i++;
		}
	} else
		printk("<2>""ERROR! platform data\n");
}
/*********************************************************************/

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx9500_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this,SX9500_IRQSTAT_REG,0xFF);
	return returnValue;
}

static int sx9500_initialize(psx9500_t this)
{
	printk("######enter %s\n",__func__);
	if (!this) {
		printk("<2>""%s: can't get data\n",__func__);
		return -1;
	}

	/* prepare reset by disabling any irq handling */
	disable_irq_nosync(gpio_to_irq(psensor_data->irq));	
	/* perform a reset */
	write_register(this,SX9500_SOFTRESET_REG,SX9500_SOFTRESET);
	/* wait until the reset has finished by monitoring NIRQ */
	/* just sleep for awhile instead of using a loop with reading irq status */
	msleep(300);
	hw_init(this);
	msleep(100); /* make sure everything is running */
	manual_offset_calibration(this);
	/* re-enable interrupt handling */
	enable_irq(gpio_to_irq(psensor_data->irq));	
	/* make sure no interrupts are pending since enabling irq will only
	 * work on next falling edge */
	read_regStat(this);

	return 0;
}

static int check_sx9500_i2c(psx9500_t this)
{
	u8 value;
	int ret;

	ret=read_register(this,0x06,&value);
	if(ret){
		printk("<2>""read addr 0x06 err(%d)\n",ret);
		return ret;
	}else
		printk("<2>""read addr 0x06,val=%x\n",value);
			
	ret=write_register(this,0x06,value);
	if(ret<0){
		printk("<2>""write addr 0x06 err(%d)\n",ret);
		return ret;
	}else
		printk("<2>""write addr 0x06,val=%x,ret =%d\n",0xa,ret);
	
	ret=read_register(this,0x06,&value);
	if(ret)
		printk("<2>""read addr 0x06 err(%d)\n",ret);
	else
		printk("<2>""read addr 0x06,val=%x\n",value);
	
	return ret;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
        return (regulator_count_voltages(reg) > 0) ?
                regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sx9500_power_on(void)	//struct sx9500_platform_data *pdata
{
	int ret;
	struct device *dev = &pdata->client->dev;

	if(psensor_data->power_on)
	{
		dev_info(dev,"Device already power on\n");
		return 0;
	}

	psensor_data->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(psensor_data->vdd)) {
		ret = PTR_ERR(psensor_data->vdd);
		dev_err(dev,
			"Regulator get failed vdd ret=%d\n", ret);
	}
	if(!IS_ERR(psensor_data->vdd))
	{
		ret = regulator_set_voltage(psensor_data->vdd, 1800000, 1800000);
		if (ret) {
			dev_err(dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto err_set_vtg_vdd;
		}
		ret = reg_set_optimum_mode_check(psensor_data->vdd, 10000);
		if (ret < 0) {
			dev_err(dev,
				"Regulator vdd set_opt failed ret=%d\n", ret);
			goto err_set_opt_vdd;
		}
		ret = regulator_enable(psensor_data->vdd);
		if (ret) {
			dev_err(dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}

	psensor_data->power_on = true;

	return 0;

err_enable_vdd:
err_set_opt_vdd:
	if (!IS_ERR(psensor_data->vdd))
		regulator_set_voltage(psensor_data->vdd, 0, 1800000);
err_set_vtg_vdd:
	psensor_data->power_on = false;

	return ret;
}

void send_psensor_uevent(enum PSENSOR_STATUS val)
{
	char *envp[2];
	char psensor[20];
	if(psensor_data == NULL)
		return;

	psensor_data->psensor_dev->state = val;
	snprintf(psensor, sizeof(psensor), "SWITCH_STATE=%d", psensor_data->psensor_dev->state);
	envp[0] = psensor;
	envp[1] = NULL;
	
	if(!psensor_data->psensor_dev->state)
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_ADD, envp);
	else
		kobject_uevent_env(&psensor_data->psensor_dev->dev->kobj, KOBJ_REMOVE, envp);
}

#if 1
static void sx9500_worker_func(struct work_struct *work)
{
	int status;
	u8 v;

	psx9500_t this = NULL;

	printk("<2>""======enter %s=======\n",__func__);
	if(!work){
		printk(KERN_ERR "sx86XX_worker_func, NULL work_struct\n");
		return;
	}

	this = container_of(work,sx9500_t,dworker.work);
	if (!this) {
      		printk(KERN_ERR "sx86XX_worker_func, NULL sx86XX_t\n");
		return;
	}
	
	status = this->refreshStatus(this);
	read_register(this, SX9500_TCHCMPSTAT_REG, &v);

	if(v&0x10){
		printk("<2>""---------------------psensor touched--------------------\n");
		send_psensor_uevent(PSENSOR_STATUS_NEAR);	
	}else{
		printk("<2>""------------------psensor realased----------------\n");
		send_psensor_uevent(PSENSOR_STATUS_FAR);	
	}

	printk("<2>""%s: status =%x, reg1=%x\n",__func__,status,v);
	
}


static irqreturn_t sx9500_int_handle(int irq, void *dev)
{
	printk("<2>""----------enter %s\n",__func__);

	if(g_sx9500 == NULL){
		printk("<2>""g_sx9500==NULL!!!!!\n");
		return -1;
	}
	schedule_delayed_work(&(g_sx9500->dworker),msecs_to_jiffies(1));
	return IRQ_HANDLED;
}
#endif

#if 0
static irqreturn_t interrupt_iqs128_irq(int irq, void *dev)
{
	printk("<2>""---%s,gpio_get_value(irq_gpio) = %d\n",__func__,gpio_get_value(pdata->irq));
	pdata->p_sensor_value = gpio_get_value(pdata->irq);

	if (!work_pending(&pdata->psensor_work)) {
		queue_work(pdata->psensor_wq, &pdata->psensor_work);
	}

	return IRQ_HANDLED;
}

static void do_psensor_work(struct work_struct *work)
{
	send_psensor_uevent(pdata->p_sensor_value);
}

int iqs128_init(void)
{
	int ret = 0;

	INIT_WORK(&pdata->psensor_work, do_psensor_work);
	pdata->psensor_wq = create_singlethread_workqueue("psensor_wq");
	if (!pdata->psensor_wq) {
		dev_err(&pdata->client->dev,"create thread error, line: %d\n", __LINE__);
		return -ENOMEM;
	}

	ret = request_irq(gpio_to_irq(pdata->irq), interrupt_iqs128_irq, IRQ_TYPE_EDGE_BOTH , "iqs128_work", NULL);
	if(ret < 0)
	{
		dev_err(&pdata->client->dev,"request_irq failed. irq = %d\n",gpio_to_irq(pdata->irq));
		goto exit_free_irq;
	}

	return 0;

exit_free_irq:
	free_irq(gpio_to_irq(pdata->irq),NULL);

	return ret;
}
#endif

static int sx9500_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
//	psx9500_platform_data_t pdata = client->dev.platform_data;
	struct sx9500_st *this;
	int ret;

	printk("#####enter %s\n",__func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct sx9500_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,
				"sx9500 Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		this = devm_kzalloc(&client->dev,sizeof(struct sx9500_st), GFP_KERNEL);
		if (!this) {
			dev_err(&client->dev,
				"sx9500 Failed to allocate memory for this\n");
			return -ENOMEM;
		}

		psensor_data = devm_kzalloc(&client->dev,
			sizeof(struct psensor_data), GFP_KERNEL);
		if (!psensor_data) {
			dev_err(&client->dev,
				"sx9500 Failed to allocate memory for psensor_data.\n");
			return -ENOMEM;
		}

		psensor_data->psensor_dev = devm_kzalloc(&client->dev, sizeof(struct switch_dev), GFP_KERNEL);
		 if (psensor_data->psensor_dev == NULL)
		{
				dev_err(&client->dev, "%s:%d Unable to allocate memory\n",
					__func__, __LINE__);
				return -ENOMEM;
		}
	}

	this->refreshStatus = read_regStat;	
	this->get_nirq_low = pdata->get_is_nirq_low;
    this->init = sx9500_initialize;
	this->bus = client;

	printk("<2>""%s: start config gpio\n",__func__);
	pdata->init_platform_hw = psensor_gpio_config;
	pdata->init_platform_hw(&client->dev);
	
	pdata->pi2c_reg = sx9500_i2c_reg_setup;
	pdata->i2c_reg_num = ARRAY_SIZE(sx9500_i2c_reg_setup);
	pdata->get_is_nirq_low = sx9500_nirq_is_low;
	pdata->client = client;

	i2c_set_clientdata(client, this);

	psensor_data->p_sensor_value = 1;
	psensor_data->psensor_dev->name = "psensor";
	psensor_data->psensor_dev->state = PSENSOR_STATUS_FAR;

	sx9500_power_on();

	ret = switch_dev_register(psensor_data->psensor_dev);
	if (ret < 0){
		dev_err(&client->dev,"register switch dev fail,ret=%d\n",ret);
		return -2;
	}
	send_psensor_uevent(PSENSOR_STATUS_FAR);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        dev_err(&client->dev,"I2C check functionality failed.");
        return -ENODEV;
    }

	if(check_sx9500_i2c(this)){
		printk("<2>""i2c read/write err,maybe sx9500 not exit\n");
		
		//load iqs128_init
		ret = iqs128_init();
		return ret;
	}

	/* initialize worker function */
	INIT_DELAYED_WORK(&this->dworker, sx9500_worker_func);

	g_sx9500 = this;

	ret = request_irq(gpio_to_irq(psensor_data->irq), sx9500_int_handle, IRQ_TYPE_EDGE_FALLING , "sx9500_work", &client->dev);
	if(ret < 0)
	{
		goto exit_free_irq;
	}

	if(this->init)
		this->init(this);

	read_regStat(this);
	
	return 0;
	
exit_free_irq:
	free_irq(gpio_to_irq(psensor_data->irq),pdata);
	
	return ret;
}

static int sx9500_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id sx9500_id[] = {
    { "sx9500", 0 },
    { }
};
static struct of_device_id sx9500_match_table[] = {
	{ .compatible = "semtech,sx9500", },
	{ },
};


static struct i2c_driver sx9500_driver = {
    .probe      = sx9500_probe,
    .remove     = sx9500_remove,
    .id_table   = sx9500_id,
    .driver = {
        .name     = "sx9500",
        .owner    = THIS_MODULE,
		.of_match_table = sx9500_match_table,
    },
};



static int __init sx9500_init(void)
{
    s32 ret;

    printk("enter %s\n",__func__);
	
    ret = i2c_add_driver(&sx9500_driver);
    return ret; 
}



static void __exit sx9500_exit(void)
{
    pr_info("GTP driver exited.");
    i2c_del_driver(&sx9500_driver);
#if 0
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
#endif
}

module_init(sx9500_init);
module_exit(sx9500_exit);

MODULE_DESCRIPTION("SX Series Driver");
MODULE_LICENSE("GPL");

