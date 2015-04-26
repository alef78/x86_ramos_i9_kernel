/******************************************************************************
 * MODULE     : rohm_bh1721_driver.c
 * FUNCTION   : drivers of BH1721GLI
 * PROGRAMMED : Tracy
 * DATE(ORG)  : Jun-30-2011(Jun-30-2011)
 * REMARKS    : 
 *            : digital illuminance sensor
 *            : supported device : BH1721FVC
 *            : slave address    : 0x23
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011- ROHM CO.,LTD.
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new file
 *****************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <asm/intel-mid.h>
//#include <unistd.h>
#include "rohm_bh1721_driver.h"
#include <asm/intel_scu_ipcutil.h>

//#define _SEI_DEBUG_
#define GPIO63_ALS_DVI_PIN  63

static struct workqueue_struct *rohm_wq;
static int data_count=0;

/* variable to set sleeping time */
int set_sleep;

struct rohm_ls_data 
{
	uint16_t addr;
	struct i2c_client *client;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint32_t flags;
	int (*power)(int on);
	struct input_dev *input_dev;
	struct delayed_work input_work;
	atomic_t enable_als;
};

/******************************************************************************
 * NAME       : _make_result_data
 * FUNCTION   : make the returned value of function which has data
 * PROCESS    : make the returned value of function which has data
 * INPUT      : data - the returned value of function
 *            : mask - mask data
 * RETURN     : result - the returned value of function which is called
 * REMARKS    : 
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new function
 *****************************************************************************/
static int _make_result_data(int *data,unsigned long mask)
{
	int result;
	
	if(0 <= *data){
		result = DRIVER_OK;
		*data  = *data & mask;
	}else{
		result = *data;
		*data = 0;
	}
	
	return(result);
}

/******************************************************************************
 * NAME       : _set_slave_address
 * FUNCTION   : set slave address
 * PROCESS    : set slave address
 * INPUT      : client  - structure  that OS provides as the standard
 * RETURN     : result - the returned value of function which is called
 * REMARKS    : 
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new function
 *****************************************************************************/
static void _set_slave_address(struct i2c_client *client)
{
	/* set I2C slave address */
	client->addr = BH1721_SLAVE_ADDR;
	
	return;
}

/******************************************************************************
 * NAME       : bh1721_driver_init
 * FUNCTION   : initiailze bh1721
 * PROCESS    : set the initalization value to registers
 * INPUT      : client - structure  that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : DRIVER_NG(-1) or return the error value in i2c function
 * REMARKS    : 
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new function
 *****************************************************************************/
static int bh1721_driver_init(struct i2c_client *client)
{
	int result = 0;
	
	/* set I2C slave address */
	_set_slave_address(client);

	result  = i2c_smbus_write_byte(client, CMD_POWER_UP);	//0000 0001,	Power on
	result |= i2c_smbus_write_byte(client, CMD_TRANS_100_H);	//0100 1001,	
	result |= i2c_smbus_write_byte(client, CMD_TRANS_100_L);	//0110 1100,	transimission rate is 100%
	result |= i2c_smbus_write_byte(client, CMD_H_RES_CONT);	//0001 0000,	1lx/step H-Res Continuously
	

	return(result);
}

/******************************************************************************
 * NAME       : bh1721_driver_close
 * FUNCTION   : close bh1721
 * PROCESS    : set the closing value to registers
 * INPUT      : client - structure  that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    : 
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new function
 *****************************************************************************/
int bh1721_driver_close(struct i2c_client *client)
{
	int result;
	
	/* set I2C slave address */
	_set_slave_address(client);
	
	/* write register to bh1721GLI via i2c */
	result = i2c_smbus_write_byte(client, CMD_POWER_DOWN);

	return(result);
}


/******************************************************************************
 * NAME       : bh1721_driver_read_illuminance
 * FUNCTION   : read the value of illuminance in bh1721
 * PROCESS    : read the illuminance value from registers
 * INPUT      : data   - the value of illuminance
 *            : client - structure  that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    : 
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new function
 *****************************************************************************/
int bh1721_driver_read_illuminance(unsigned short *data, struct i2c_client *client)
{
	int result;
	int get_data;

	struct i2c_msg msg[1];
	uint8_t buf[2];

	/* set I2C slave address */
	_set_slave_address(client);

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 2;
	msg[0].buf = buf;
	result = i2c_transfer(client->adapter, msg, 1);

	get_data = (buf[0]<<8) + buf[1];	
	
	/* check the return value */
	result = _make_result_data(&get_data,MASK_SHORT);
	
	/* set to the returned variable */
	*data = (unsigned short)get_data;
	
	return(result);
}

static int bh1721_als_enable(struct rohm_ls_data *stat)
{
	int err;
	if (!atomic_cmpxchg(&stat->enable_als, 0, 1)) {
		err = bh1721_driver_init(stat->client);
		if (err < 0) {
			printk(KERN_ERR "%s:bh1721 driver init failed\n",__func__);
			atomic_set(&stat->enable_als, 0);
			return err;
		}
		hrtimer_start(&stat->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	}

	return 0;
}

static int bh1721_als_disable(struct rohm_ls_data *stat)
{
	if (atomic_cmpxchg(&stat->enable_als, 1, 0)) {
		hrtimer_cancel(&stat->timer);
		bh1721_driver_close(stat->client);
	}

	return 0;
}

static ssize_t attr_get_enable_als(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct rohm_ls_data *stat = dev_get_drvdata(dev);
	int val = (int)atomic_read(&stat->enable_als);
	return sprintf(buf, "%d\n", val);
}



static ssize_t attr_set_enable_als(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct rohm_ls_data *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	printk("pjn===%s(): val:%d\n",__func__,val);
	if (val)
		bh1721_als_enable(stat);
	else
		bh1721_als_disable(stat);

	return size;
}


static ssize_t attr_get_als_value(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int            result;
	unsigned short illuminance_data;
	struct rohm_ls_data *stat = dev_get_drvdata(dev);
	
	result = bh1721_driver_read_illuminance(&illuminance_data, stat->client);
	return sprintf(buf, "%d\n", illuminance_data);
}


static struct device_attribute attributes[] = {
	__ATTR(enable_als, 0664, attr_get_enable_als,attr_set_enable_als),	
	__ATTR(value, 0444, attr_get_als_value,NULL),		
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


static void rohm_work_report(struct rohm_ls_data *ls,int *data)
{
#ifdef _SEI_DEBUG_
		printk("bh1721_work_report! data:%d\n",data[0]);
#endif

	//input_report_abs(ls->input_dev, ABS_X, data[0]);
	input_report_abs(ls->input_dev, ABS_MISC, data[0]);
	input_sync(ls->input_dev);
	
	return;
}

static void rohm_ls_work_func(struct work_struct *work)
{
	int            result;
	unsigned short illuminance_data;
	int            data_up[4];

	struct rohm_ls_data *ls = container_of(work, struct rohm_ls_data, work);
	data_count ++;
	

	result = bh1721_driver_read_illuminance(&illuminance_data, ls->client);
	#ifdef _SEI_DEBUG_
 	printk("illuminance result = %d\n",result);
	#endif

	data_up[0] = (int)illuminance_data;
	#ifdef _SEI_DEBUG_
	printk("%02d : illuminance_data(16) = %d\n", data_count, illuminance_data);
	#endif
	rohm_work_report(ls, data_up);
	
	msleep(set_sleep);
#ifdef _SEI_DEBUG_
//	ssleep(10);
	ssleep(1);
#endif

	hrtimer_start(&ls->timer, ktime_set(0, 125000000), HRTIMER_MODE_REL);			//each one 125ms(8Hz)
}

static enum hrtimer_restart rohm_ls_timer_func(struct hrtimer *timer)
{
	struct rohm_ls_data *ls = container_of(timer, struct rohm_ls_data, timer);
	queue_work(rohm_wq, &ls->work);
	//report rate 1/10000000=100Hz, tracy
	return HRTIMER_NORESTART;
}


static int rohm_sensor_input_init(struct rohm_ls_data *ls)
{
	int err;

#ifdef _SEI_DEBUG_	
		printk("bh1721_input_init!\n");	
#endif

	ls->input_dev = input_allocate_device();
	if (!ls->input_dev) {
		err = -ENOMEM;
		dev_err(&ls->client->dev, "input device allocate failed\n");
		goto err0;
	}

	input_set_drvdata(ls->input_dev, ls);

	set_bit(EV_ABS, ls->input_dev->evbit);
	set_bit(EV_SYN, ls->input_dev->evbit);

	input_set_abs_params(ls->input_dev, ABS_MISC, 0, 65535, 0, 0);
	ls->input_dev->name = "als_1721";

	err = input_register_device(ls->input_dev);
	if (err) {
		dev_err(&ls->client->dev,
			"unable to register input polled device %s: %d\n",
			ls->input_dev->name, err);
		goto err1;
	}
	return 0;
err1:
	input_free_device(ls->input_dev);
err0:
	return err;
}

static int rohm_ls_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rohm_ls_data *ls;
	int ret = 0;
	struct ROHM_I2C_platform_data *pdata;
	int als_gpio = 0;
	
	
	printk("called rohm_ls_probe for BH1721FVC!!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk(KERN_ERR "Rohm_ls_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ls = kzalloc(sizeof(*ls), GFP_KERNEL);
	if (ls == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	INIT_WORK(&ls->work, rohm_ls_work_func);
	ls->client = client;
	i2c_set_clientdata(client, ls);
	atomic_set(&ls->enable_als, 0);
	pdata = client->dev.platform_data;
	if (pdata)
		ls->power = pdata->power;
	if (ls->power)
	{
		ret = ls->power(1);
		if (ret < 0) 
		{
			printk(KERN_ERR "Rohm_ls_probe power on failed\n");
			goto err_power_failed;
		}
	}
	
	//gpio44
	/*
	als_gpio  = get_gpio_by_name("ALS_INT_N");
	if(als_gpio<0)
		printk("Faild to get  als gpio63\n");
	
	ret = gpio_request(als_gpio, "als_int");
	if(ret<0)
		printk("%s,gpio63 request fail,ret:%d\n",ret);
	*/

	/* Check the vprog2 be 2.8V, Otherwise set it, Change for P802_C000126 */
	if(!intel_scu_ipc_read_msic_vprog2_reg()) {
		intel_scu_ipc_msic_vprog2(1);
		printk("%s, enable vprog2 to light-sensor\n",__func__);
		msleep(1);
	}

	/* Refer to spec BH1721FVC.pdf */
	als_gpio = 44;
	gpio_direction_output(als_gpio, 0);
	gpio_set_value_cansleep(als_gpio, 0);
	printk("pjn= %s:gpio44:%d\n",__func__,gpio_get_value_cansleep(als_gpio));
	msleep(10);//zfming 1 to 100 in patch
	gpio_direction_output(als_gpio, 1);
	gpio_set_value_cansleep(als_gpio, 1);
	printk("pjn= %s:gpio44:%d\n",__func__,gpio_get_value_cansleep(als_gpio));

	
	rohm_sensor_input_init(ls);
	ret = bh1721_driver_init(ls->client);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:bh1721 driver init failed result:0x%x\n",__func__, ret);
	}
	hrtimer_init(&ls->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ls->timer.function = rohm_ls_timer_func;
	
	//printk("start timer in probe!");
	//hrtimer_start(&ls->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		
	ret = create_sysfs_interfaces(&client->dev);
	if (ret){
		printk("%s,Unable to create interface",__func__);
		goto error;
	}
	printk("BH1721FVC probe done!!\n");
	return 0;
	
error:
	remove_sysfs_interfaces(&client->dev);
err_power_failed:
	kfree(ls);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int rohm_ls_remove(struct i2c_client *client)
{
	struct rohm_ls_data *ls = i2c_get_clientdata(client);

	hrtimer_cancel(&ls->timer);
	kfree(ls);
	return 0;
}

static const struct i2c_device_id rohm_ls_id[] = 
{
	{ ROHM_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver rohm_ls_driver = 
{
	.driver = {
		.name	= ROHM_I2C_NAME,
	},
	.probe		= rohm_ls_probe,
	.remove		= rohm_ls_remove,
	.id_table	= rohm_ls_id,
};

static int __init rohm_ls_init(void)
{
	rohm_wq = create_singlethread_workqueue("rohm_wq");
	if (!rohm_wq)
	{
		printk(KERN_ERR "create_singlethread_workqueue ERROR!!\n");
		return -ENOMEM;
	}	
	
	printk("rohm_ls_init\n");
	return i2c_add_driver(&rohm_ls_driver);
}

static void __exit rohm_ls_exit(void)
{
	#ifdef _SEI_DEBUG_
		printk("rohm_ls_exit\n");
	#endif
	i2c_del_driver(&rohm_ls_driver);
	if (rohm_wq)
		destroy_workqueue(rohm_wq);
}

MODULE_DESCRIPTION("Rohm Ambient Lighit Sensor Driver");
MODULE_LICENSE("GPL");

module_init(rohm_ls_init);
module_exit(rohm_ls_exit);

