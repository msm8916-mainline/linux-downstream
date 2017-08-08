/*
* Copyright (C) 2014  Peel Technologies Inc
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>
#include "peelir.h"

#include <asm/uaccess.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

static int peelir_major;
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define TRANS_BUFF	40960
#define NUM_BUFF	14
#define MAX_BUFF	NUM_BUFF*TRANS_BUFF
#define LR_EN		73

u8 t_buff[MAX_BUFF], rx_snd_buff[MAX_BUFF];

struct peelir_data {
	dev_t			devt;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex		buf_lock;
	spinlock_t		spi_lock;
	unsigned		users;
	u8			*buffer;
	struct regulator * vdd_ana;
    u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 600 * 1024;  /* Default buffer size */ 
static int mode = 0, bpw = 8, spi_clk_freq = 960000 ;
static int prev_tx_status; /* Status of previous transaction */
static unsigned int field;
static void *p_kbuf;

static int peelir_regulator_enable(struct peelir_data *peelir);
static int peelir_regulator_disable(struct peelir_data *peelir);

static void peelir_complete(void *arg)
{
	complete(arg);
}

static ssize_t
peelir_sync(struct peelir_data *peelir, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = peelir_complete;
	message->context = &done;

	spin_lock_irq(&peelir->spi_lock);
	if (peelir->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(peelir->spi, message);

	spin_unlock_irq(&peelir->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static int peelir_read_message(struct peelir_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		total;
	u8			*buf;
	char *tx_data;
	int status = -EFAULT;
	int i;

	spi_message_init(&msg);
	k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Accept Pattern from the user and generate the SPI message */

	tx_data = (char *) kmalloc (MAX_BUFF,GFP_KERNEL);

	memset(tx_data,0, MAX_BUFF); // Transmit buffer
	memset(t_buff,0xff,MAX_BUFF);	//Receive Buffer

	u_xfers->tx_buf = (unsigned long )tx_data;

	/* Transmit 40K buffer filled with zero for 3 second to keep the SPI clk active for receiving IR input */

	buf = peelir->buffer;
	total = 0;
	k_tmp = k_xfers;
	u_tmp = u_xfers;
	k_tmp->len = u_tmp->len;
	total += k_tmp->len;
	if (total > bufsiz) {
		status = -EMSGSIZE;
		goto done;
	}

	if (u_tmp->rx_buf) {
		k_tmp->rx_buf = buf;
		if (!access_ok(VERIFY_WRITE, (u8 __user *)
					(uintptr_t) u_tmp->rx_buf,
					u_tmp->len))
			printk("\nERROR::::: In rcv");
	}
	if (u_tmp->tx_buf) {
		buf = tx_data;
		k_tmp->tx_buf = buf;
	}
	buf += k_tmp->len;

	k_tmp->cs_change = 0;
	k_tmp->bits_per_word = u_tmp->bits_per_word;
//	k_tmp->bits_per_word = 8;
	k_tmp->speed_hz = u_tmp->speed_hz;

	spi_message_add_tail(k_tmp, &msg);

	/* Receiving IR input for 3 seconds */

	printk("\n Waiting for IR data.... ");
	printk("\n Press the Key\n");

	status = peelir_sync(peelir, &msg);
	buf = peelir->buffer;
	for (i=0; i < MAX_BUFF; i++)
		t_buff[i] = buf[i];

	u_tmp = u_xfers;
	u_tmp->len = MAX_BUFF;
	buf = peelir->buffer;

	/* copy any rx data to user space */
	if (u_tmp->rx_buf) {
		printk("\nCopying data to user space");
		if (__copy_to_user((u8 __user *)
			(uintptr_t) u_tmp->rx_buf, k_tmp->rx_buf, k_tmp->len))
		{
			printk("\nCopy to user space failed !!!");
			status = -EFAULT;
			goto done;
		}
	}

	status = total;

	kfree(tx_data);

	done:
	kfree(k_xfers);

	return 0;
}


static int peelir_write_message(struct peelir_data *peelir,
		struct spi_ioc_transfer *u_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		total;
	u8 *buf;
	int status = -EFAULT;

	unsigned long start_time = jiffies;

	spi_message_init(&msg);
	k_xfers = kcalloc(1, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;


	buf = peelir->buffer;
	total = 0;

	k_tmp = k_xfers;
	u_tmp = u_xfers;
	k_tmp->len = u_tmp->len;

	total += k_tmp->len;
	if (total > bufsiz) {
		status = -EMSGSIZE;
		goto done;
	}

	if (u_tmp->rx_buf) {
		k_tmp->rx_buf = buf;
		if (!access_ok(VERIFY_WRITE, (u8 __user *)
					(uintptr_t) u_tmp->rx_buf,
					u_tmp->len))
			goto done;
	}
	if (u_tmp->tx_buf) {
		k_tmp->tx_buf = buf;
		if (copy_from_user(buf, (const u8 __user *)
					(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
			goto done;
	}
	buf += k_tmp->len;
	k_tmp->cs_change = 0;
	k_tmp->bits_per_word = u_tmp->bits_per_word;
	//k_tmp->bits_per_word = 32;
	k_tmp->speed_hz = u_tmp->speed_hz;
	peelir->spi->bits_per_word = k_tmp->bits_per_word;
	//printk("%s: Bits per word = %d\n", __func__, k_tmp->bits_per_word);
	peelir->spi->mode = 0;
	spi_message_add_tail(k_tmp, &msg);

	status = peelir_sync(peelir, &msg);
	if (status < 0)
		goto done;


	status = total;


done:
	kfree(k_xfers);
	IRRC_INFO_MSG("SPI write done duration %d\n", jiffies_to_msecs(jiffies - start_time));
	return status;
}

static long
peelir_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			retval = 0;
	struct peelir_data	*peelir;
	struct spi_device	*spi;
	struct spi_ioc_transfer	*ioc;

	IRRC_DEBUG_MSG("IOCTL \n");
	peelir = filp->private_data;

	spin_lock_irq(&peelir->spi_lock);

	spi = spi_dev_get(peelir->spi);

	spin_unlock_irq(&peelir->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	mutex_lock(&peelir->buf_lock);

	spi->mode = mode;
	spi->bits_per_word = bpw;
	spi->max_speed_hz = spi_clk_freq;

	retval = spi_setup(spi);
	IRRC_DEBUG_MSG("spi_setup %d \n", retval);
	if(retval < 0)
		dev_dbg(&spi->dev, "Error configuring SPI\n");


	IRRC_DEBUG_MSG("IOCTL cmd : %d, mode : %d, bpw : %d, clk : %d \n", cmd, mode, bpw, spi_clk_freq);
	switch (cmd) {
	/* read ioctls */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
	    IRRC_DEBUG_MSG("RD_MODE \n");
		break;

	case SPI_IOC_WR_MSG:
        //peelir_regulator_enable(peelir);
		//HMOON gpio_set_value(LR_EN, 0);	// LR Enable low for Tx
		/* copy into scratch area */
		ioc = kmalloc( sizeof(struct spi_ioc_transfer), GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, sizeof(struct spi_ioc_transfer))) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}


		retval = peelir_write_message(peelir, ioc);
		if(retval > 0)
			prev_tx_status = 1;
		else
			prev_tx_status = 0;

		kfree(ioc);
		//HMOON gpio_set_value(LR_EN, 0);	// LR Enable default state

	    IRRC_DEBUG_MSG("WR_MODE Done \n");
        //peelir_regulator_disable(peelir);
		break;

	case SPI_IOC_RD_MSG:
		//HMOON gpio_set_value(LR_EN, 1);	// LR Enable high for Rx
		/* copy into scratch area */
                ioc = kmalloc( sizeof(struct spi_ioc_transfer), GFP_KERNEL);
                if (!ioc) {
                        retval = -ENOMEM;
                        break;
                }
                if (__copy_from_user(ioc, (void __user *)arg, sizeof(struct spi_ioc_transfer))) {
                        kfree(ioc);
                        retval = -EFAULT;
                        break;
                }
		retval = peelir_read_message(peelir, ioc);
		//HMOON gpio_set_value(LR_EN, 0);	// LR Enable default state
	    IRRC_DEBUG_MSG("RD_MSG Done \n");
	default:
	    IRRC_DEBUG_MSG("Default \n");
		break;
	}

	mutex_unlock(&peelir->buf_lock);
	spi_dev_put(spi);

	return retval;
}

static int peelir_open(struct inode *inode, struct file *filp)
{
	struct peelir_data	*peelir;
	int			status = -ENXIO;

	IRRC_DEBUG_MSG("OPEN \n");
	mutex_lock(&device_list_lock);

	list_for_each_entry(peelir, &device_list, device_entry) {
		if (peelir->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!peelir->buffer) {
			peelir->buffer = p_kbuf;
			if (!peelir->buffer) {
				dev_dbg(&peelir->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			peelir->users++;
			filp->private_data = peelir;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("peelir: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);

    peelir_regulator_enable(peelir);

	IRRC_DEBUG_MSG("OPEN - done \n");

	return status;
}

static int peelir_release(struct inode *inode, struct file *filp)
{
	struct peelir_data	*peelir;
	int			status = 0;

	IRRC_DEBUG_MSG("RELEASE \n");

	mutex_lock(&device_list_lock);
	peelir = filp->private_data;
	filp->private_data = NULL;

	peelir->users--;

	mutex_unlock(&device_list_lock);

    peelir_regulator_disable(peelir);
	
	IRRC_DEBUG_MSG("RELEASE - done \n");

	return status;
}

/*
 * sysfs layer
 */

static ssize_t ir_tx_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%d\n", prev_tx_status );
}

static ssize_t field_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
        return sprintf( buf, "%x\n", field );
}

static ssize_t field_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf, size_t count)
{
        sscanf(buf, "%x", &field);
        return count;
}

static DEVICE_ATTR(txstat, S_IRUGO, ir_tx_status, NULL);
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field_show, field_store);

static struct attribute *peel_attributes[] = {
        &dev_attr_txstat.attr,
        &dev_attr_field.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = peel_attributes,
};

static const struct file_operations peelir_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = peelir_ioctl,
	.open =		peelir_open,
	.release =	peelir_release,
};

static struct class *peelir_class;
/*-------------------------------------------------------------------------*/
static int peelir_regulator_enable(struct peelir_data *peelir)
{
    int rc = 0;

	if(peelir == NULL)
		return 0;
	if(peelir->vdd_ana == NULL)
		return 0;
    if(IS_ERR(peelir->vdd_ana) && (PTR_ERR( peelir->vdd_ana) == -ENODEV))
		return 0;

	IRRC_DEBUG_MSG("Regulator enable \n");

    rc = regulator_enable(peelir->vdd_ana);
	if(rc)
    {
		IRRC_ERR_MSG("regulator_enable failed %d\n", rc);
		goto error_regulator_enable;
	}
	return 0;

error_regulator_enable:
	regulator_disable(peelir->vdd_ana);
	return rc;
}

static int peelir_regulator_disable(struct peelir_data *peelir)
{
    int rc = 0;
	
	if(peelir == NULL)
		return 0;
	if(peelir->vdd_ana == NULL)
		return 0;
    if(IS_ERR(peelir->vdd_ana) && (PTR_ERR( peelir->vdd_ana) == -ENODEV))
		return 0;

	IRRC_DEBUG_MSG("Regulator disable \n");
    rc = regulator_disable(peelir->vdd_ana);
	if(rc)
    {
		IRRC_ERR_MSG("regulator_disable failed %d\n", rc);
		goto error_regulator_disable;
	}
	return 0;

error_regulator_disable:
	regulator_disable(peelir->vdd_ana);
	return rc;
}

static int peelir_regulator_configure(struct peelir_data *peelir)
{
	int rc = 0;

	IRRC_DEBUG_MSG("Regulator configure  \n");

	if(peelir == NULL)
		return -ENOMEM;

	peelir->vdd_ana = regulator_get(&peelir->spi->dev, "Peel,vdd_ana");
	// Rev A has no value
	if(peelir->vdd_ana == NULL)
		return rc;

	if(IS_ERR(peelir->vdd_ana)){
		rc = PTR_ERR(peelir->vdd_ana);
		peelir->vdd_ana = NULL;
	    IRRC_ERR_MSG("Regulator get failed vdd_ana rc =%d \n", rc);
		return rc;
	}

	if(regulator_count_voltages(peelir->vdd_ana) > 0 ) {
		rc = regulator_set_voltage(peelir->vdd_ana,peelir->vdd_ana_supply_min,peelir->vdd_ana_supply_max);

	    if(rc) {
	        IRRC_ERR_MSG("requlator_set_voltage failed rc %d\n", rc);
            goto error_set_voltage;
        }

    }
	IRRC_INFO_MSG("Regulator configure - regulator_get done \n");
    return 0; 

error_set_voltage:
	regulator_put(peelir->vdd_ana);
	return rc;

}
/*-------------------------------------------------------------------------*/

static int peelir_parse_dt(struct device *dev, struct peelir_data *pdata)
{
	struct device_node *node = dev->of_node;
    int rc = 0;
	u32 temp_val = 0;

	if(node == NULL){
		IRRC_ERR_MSG("node is null. \n");
		return -ENODEV;
	}

	rc = of_property_read_u32(node, "vdd_ana_supply_min", &temp_val);
    if(rc){
		IRRC_ERR_MSG("DT : unable to read vdd_ana_supply_min \n");
	}else {
		pdata->vdd_ana_supply_min = temp_val;
		IRRC_DEBUG_MSG("DT : vdd_ana_supply_min : %d \n", pdata->vdd_ana_supply_min);
	}

	rc  = of_property_read_u32(node, "vdd_ana_supply_max", &temp_val);
    if(rc){
		IRRC_ERR_MSG("DT : unable to read vdd_ana_supply_max \n");
	}else{
		pdata->vdd_ana_supply_max = temp_val;
		IRRC_DEBUG_MSG("DT : vdd_ana_supply_max : %d \n", pdata->vdd_ana_supply_max);
	}

	rc  = of_property_read_u32(node, "vdd_ana_load_ua", &temp_val);
    if(rc){
		IRRC_ERR_MSG("DT : unable to read vdd_ana_load_ua \n");
	}else{
		pdata->vdd_ana_load_ua = temp_val;
		IRRC_DEBUG_MSG("DT : vdd_ana_load_ua : %d \n", pdata->vdd_ana_load_ua);
	}
    return rc;
}

static int __devinit peelir_probe(struct spi_device *spi)
{
	struct peelir_data	*peelir;
	int			status;
	unsigned long		minor;

	IRRC_DEBUG_MSG("Probe \n");
	/* Allocate driver data */
	peelir = kzalloc(sizeof(*peelir), GFP_KERNEL);
	if (!peelir)
		return -ENOMEM;

	/* Initialize the driver data */
	peelir->spi = spi;
	spin_lock_init(&peelir->spi_lock);
	mutex_init(&peelir->buf_lock);

	INIT_LIST_HEAD(&peelir->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		peelir->devt = MKDEV(peelir_major, minor);
		dev = device_create(peelir_class, &spi->dev, peelir->devt,
				    peelir, "peel_ir");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&peelir->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	peelir_parse_dt(&spi->dev, peelir);

	if (status == 0)
		spi_set_drvdata(spi, peelir);
	else
		kfree(peelir);

	if( peelir_regulator_configure(peelir) < 0)
	{
		dev_dbg(&spi->dev, "Error regulator failed \n" );
	}

/* HMOON */
#if 0 
	if (gpio_is_valid(LR_EN)) {
                /* configure LR enable gpio */
                status= gpio_request(LR_EN, "lr_enable");
                if (status) {
                        printk("unable to request gpio [73]: %d\n",status);
                }
                status = gpio_direction_output(LR_EN, 0);
                if (status) {
                        printk("unable to set direction for gpio [73]: %d\n", status);
                }
                //HMOON gpio_set_value(LR_EN, 0);
        }
       else
               printk("gpio 73 is not valid \n");
#endif

	/* sysfs entry */
	status = sysfs_create_group(&spi->dev.kobj, &attr_group);
	if( status )
		dev_dbg(&spi->dev, " Error creating sysfs entry \n" );

	IRRC_INFO_MSG("Probe - Done\n");

	return status;
}

static int peelir_remove(struct spi_device *spi)
{
	struct peelir_data	*peelir = spi_get_drvdata(spi);

	IRRC_DEBUG_MSG("Remove \n");
	
    if(peelir->vdd_ana)
        regulator_put(peelir->vdd_ana);

	sysfs_remove_group(&spi->dev.kobj, &attr_group);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&peelir->spi_lock);
	peelir->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&peelir->spi_lock);

	/* prevent opening a new instance of the device
	   during the removal of the device
	 */
	mutex_lock(&device_list_lock);
	list_del(&peelir->device_entry);
	device_destroy(peelir_class, peelir->devt);
	clear_bit(MINOR(peelir->devt), minors);
	if (peelir->users == 0)
		kfree(peelir);
	mutex_unlock(&device_list_lock);

	IRRC_DEBUG_MSG("Remove  - Done\n");
	return 0;
}

static struct spi_driver peelir_spi_driver = {
	.driver = {
		.name =		"peel_ir",
		.owner =	THIS_MODULE,
	},
	.probe =	peelir_probe,
	.remove =	peelir_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init peelir_init(void)
{
	int status = 0;

	IRRC_DEBUG_MSG("Init\n");

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	p_kbuf = kzalloc(bufsiz, GFP_KERNEL);
	if(IS_ERR_OR_NULL(p_kbuf))
		return -ENOMEM;
	peelir_major = register_chrdev(0, "peel_ir", &peelir_fops);
	if (peelir_major < 0)
		return peelir_major;

	peelir_class = class_create(THIS_MODULE, "peelir");
	if (IS_ERR(peelir_class)) {
		unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
		return PTR_ERR(peelir_class);
	}

	status = spi_register_driver(&peelir_spi_driver);
	if (status < 0) {
		class_destroy(peelir_class);
		unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
	}

	IRRC_DEBUG_MSG("Init - Done\n");
	return status;
}
module_init(peelir_init);

static void __exit peelir_exit(void)
{
	IRRC_DEBUG_MSG("Exit\n");
	spi_unregister_driver(&peelir_spi_driver);
	class_destroy(peelir_class);
	unregister_chrdev(peelir_major, peelir_spi_driver.driver.name);
}
module_exit(peelir_exit);

MODULE_DESCRIPTION("Peel IR SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("PEEL_IR");
