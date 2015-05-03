/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/ft5x0x_ts.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//#define I2C_FT5x26
#define FTS_CTL_IIC
#define FTS_APK_DEBUG
#define SYSFS_DEBUG

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#ifdef SYSFS_DEBUG
#include "ft5x0x_ex_fun.h"
#endif

#define FT_1664S_NAME		"FT1664"
#define FT_3432S_NAME 		"FT3432"

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_early_suspend(struct early_suspend *es);
static void ft5x0x_late_resume(struct early_suspend *es);
#endif

//Add by emdoor jim.kuang for ft5x0x report error
static void ft5x0x_touch_wakeup_reset(void);
struct ft5x0x_ts_data *g_ptsdata=NULL;
int touchflag = 0;
struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct mutex mutex;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft5x0x_platform_data *pdata;
	char phys[32];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

/*
*ft5x0x_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x0x_i2c_read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/* Read touch point information when the interrupt  is asserted. */
static int ft5x0x_read_touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft5x0x_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}

	event->pressure = FT_PRESS;

	return 0;
}

/*
*report the point information
*/
static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	//Add by emdoor jim.kuang 2014-05-07 for touch screen error report 
	if((event->touch_point == 1 || event->au8_touch_event[0] == 2) && touchflag == 0 )
	{
		touchflag = 1;
		ft5x0x_touch_wakeup_reset();
	}
	
	/*protocol B*/
	for (i = 0; i < event->touch_point; i++)
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == 0 ||
			event->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(data->input_dev,
					MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					/*data->x_max -*/ event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					/*data->y_max -*/ event->au16_y[i]);
		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev,
				MT_TOOL_FINGER, false);
		}
	}
	if (event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		touchflag=0;
	}	
	else{
		input_report_key(data->input_dev, BTN_TOUCH,
					event->touch_point > 0);
	}				
	input_sync(data->input_dev);


}

//Add by emdoor jim.kuang 2014-05-07

static void ft5x0x_touch_wakeup_reset(void)
{
	struct input_dev *input_dev = g_ptsdata->input_dev;
	int i = 0, ret = 0;

	ret = ft5x0x_read_touchdata(g_ptsdata);;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; ++i) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(input_dev);
	return;
}

static int ft5x0x_ts_reset(struct i2c_client *client, int reset_pin)
{
	int error;

	if (gpio_is_valid(reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = gpio_request_one(reset_pin, GPIOF_OUT_INIT_LOW,
					 "ft5x0x reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				reset_pin, error);
			return error;
		}

		msleep(10);
		gpio_set_value(reset_pin, 0);
		msleep(20);
		gpio_set_value(reset_pin, 1);
		msleep(20);
		gpio_free(reset_pin);
	}

	return 0;
}

/*The ft5x0x device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	int ret = 0;

	disable_irq_nosync(ft5x0x_ts->irq);

	ret = ft5x0x_read_touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(ft5x0x_ts);

	enable_irq(ft5x0x_ts->irq);

	return IRQ_HANDLED;
}

//Add by emdoor jim.kuang for Multi Tp Detect 2014-05-08

extern struct i2c_driver vtl_ts_driver;
extern int chip_get_chip_id(struct i2c_client *client,unsigned char *rx_buf);
extern struct i2c_driver goodix_ts_driver;

////////////////////////////////////////////////////////////////
//Add by emdoor jim.kuang 2014-05-09
//Register define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140

#define GTP_ADDR_LENGTH       2

static s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    ///msgs[0].scl_rate=200000;

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    ///msgs[0].scl_rate=200000;

    while(retries < 1)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    return ret;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    while(retry++ < 2)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        printk("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}
//extern int gt911_init(void);
////////////////////////////////////////////////////////////////

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x0x_platform_data *pdata =
	    (struct ft5x0x_platform_data *)client->dev.platform_data;
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	int ret = 0;
	unsigned char reg_value;
	unsigned char reg_addr;
	u8 chip_id = 0xff;
pr_info("ft5x0x_ts_probe\n");
	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}
	client->addr=0x38;		

	err = ft5x0x_ts_reset(client, pdata->reset);
	if (err)
		return err;
pr_info("ft5x0x_ts_probe reset ok\n");
	#ifdef I2C_FT5x26
	unsigned char i2c_cmd[6] = {0};
	#endif

	reg_addr = FT5X0X_REG_FW_VER;
	
	err=ft5x0x_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		printk(KERN_WARNING "Adding ft5x0x driver failed "
			"(errno = %d)\n", err);
	}		
	
	printk("################ %s reg_value = 0x%x err= %d ##################\r\n",__func__,reg_value,err);
	
	if(err < 0)
	{
	
		//client->addr=0x01;
		//chip_get_chip_id(client,&chip_id);
		client->addr=0x38;		
		
		//if(chip_id == 1 || chip_id == 2 || chip_id == 6 )	
		//{
		//	i2c_add_driver(&vtl_ts_driver);
		//}


		//Add for detect GT911 TP Driver 2014-05-09 start 
		/*client->addr=0x14;
		
		ret = gtp_i2c_test(client);
		printk("################## %s GT911 detect %d #############\r\n",__func__,ret);
		if (ret < 0)
		{
			client->addr = 0x5d;
			ret = gtp_i2c_test(client);
			if (ret >= 0)
			{
		//		gt911_init();
			}
			printk("GTP I2C new Address:0x%02x", client->addr);
		}else{
				gt911_init();
		}

		//Add for detect GT911 TP Driver 2014-05-09 end
		*/
		client->addr = 0x38;	
		
		return -ENODEV;
			
		}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ft5x0x_ts);
	mutex_init(&ft5x0x_ts->mutex);
	ft5x0x_ts->irq = pdata->irq;
	ft5x0x_ts->x_max = pdata->x_max - 1;
	ft5x0x_ts->y_max = pdata->y_max - 1;
	ft5x0x_ts->client = client;
	ft5x0x_ts->pdata = pdata;

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	input_set_drvdata(input_dev, ft5x0x_ts);
	ft5x0x_ts->input_dev = input_dev;
	input_dev->name = FT5X0X_NAME;
	
	g_ptsdata=ft5x0x_ts;
	if (gpio_is_valid(pdata->irq)) {
		err = gpio_request_one(pdata->irq,
					 GPIOF_IN, "ft5x06 irq");
		if (err) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				pdata->irq, err);
			goto exit_gpio_request_failed;
		}
	}
	client->irq = gpio_to_irq(ft5x0x_ts->irq);
	dev_info(&client->dev,"ft5x0x's irq = %d\n", client->irq);

	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
				   ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ft5x0x_ts->y_max, 0, 0);
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/* make sure CTP already finish startup process */
	msleep(150);

#ifdef SYSFS_DEBUG
	ft5x0x_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

#ifdef FTS_APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
#endif

#ifdef I2C_FT5x26
	i2c_cmd[0] = 0xeb;
	i2c_cmd[1] = 0xaa;
	i2c_cmd[2] = 0x09;
	err = ft5x0x_i2c_read(client, i2c_cmd, 3, i2c_cmd+3, 3);
	if (err < 0) {
		dev_err(&client->dev, "[FTS] switch to i2c standard protocol failure!\n");
		goto exit_input_register_device_failed;
	} else {
		if (0xeb == i2c_cmd[3] && 0xaa == i2c_cmd[4] && 0x08 == i2c_cmd[5]) {
			pr_info("[FTS] switch to i2c standard protocol!\n");
		} else {
			dev_err(&client->dev, "---[FTS] switch to i2c standard protocol failure!\
				0x%x 0x%x 0x%x\n", i2c_cmd[3], i2c_cmd[4], i2c_cmd[5]);
			//goto exit_input_register_device_failed;
		}
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
        ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        ft5x0x_ts->early_suspend.suspend = ft5x0x_early_suspend;
        ft5x0x_ts->early_suspend.resume = ft5x0x_late_resume;
        register_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	/* get some register information */
	reg_addr = FT5X0X_REG_FW_VER;
	ft5x0x_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	dev_info(&client->dev, "[FTS] Firmware version = 0x%x\n", reg_value);

	reg_addr = FT5X0X_REG_POINT_RATE;
	ft5x0x_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	dev_info(&client->dev,"[FTS] report rate is %dHz.\n", reg_value * 10);

	reg_addr = FT5X0X_REG_THGROUP;
	ft5x0x_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	dev_info(&client->dev,"[FTS] touch threshold is %d.\n", reg_value * 4);

	enable_irq(client->irq);
	return 0;

exit_input_register_device_failed:
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
	if (gpio_is_valid(pdata->irq))
		gpio_free(pdata->irq);
exit_gpio_request_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}


#ifdef CONFIG_PM_SLEEP
static int ft5x0x_ts_suspend(struct device *dev)
{
	unsigned char buf[2] = { 0 };
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *tsdata = i2c_get_clientdata(client);
	printk("############### %s ################\r\n",__func__);
	disable_irq(client->irq);
	#if 0 //Modify by emdoor jim.kuang 2014-03-03
	mutex_lock(&tsdata->mutex);
	/*set to hibernate mode. */
	buf[0] = FT5X0X_ID_G_PMODE;
	buf[1] = 0x03;
	ft5x0x_i2c_write(client, buf, sizeof(buf));
	mutex_unlock(&tsdata->mutex);
	#endif
	return 0;
}

static int ft5x0x_ts_resume(struct device *dev)
{
	unsigned char buf[3] = {0};
	struct ft5x0x_ts_data *tsdata;
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_platform_data *pdata = client->dev.platform_data;
	printk("############### %s ################\r\n",__func__);

	tsdata = (struct ft5x0x_ts_data *)i2c_get_clientdata(client);
	pdata = (struct ft5x0x_platform_data *)(client->dev.platform_data);
	ft5x0x_ts_reset(client, pdata->reset);
	/*set to hibernate mode. */
	buf[0] = 0xeb;
	buf[1] = 0xaa;
	buf[2] = 0x09;
	mutex_lock(&tsdata->mutex);
	ft5x0x_i2c_write(client, buf, sizeof(buf));
    mutex_unlock(&tsdata->mutex);
	printk("[ KENT ] ===== %s : write android cmd end\n", __func__ );  
    msleep(30);

	/*reset to enter active mode. */
	enable_irq(client->irq);

	return 0;
}
#else
#define ft5x0x_ts_suspend	NULL
#define ft5x0x_ts_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_early_suspend(struct early_suspend *es)
{
	unsigned char buf[2] = { 0 };
	struct ft5x0x_ts_data *tsdata;
	tsdata = (struct ft5x0x_ts_data *)container_of(es, struct ft5x0x_ts_data, early_suspend);
	printk("############### %s ################\r\n",__func__);

	disable_irq(tsdata->client->irq);
	#if 0 //Modify by emdoor jim.kuang 2014-03-03

	mutex_lock(&tsdata->mutex);
	/*set to hibernate mode. */
	buf[0] = FT5X0X_ID_G_PMODE;
	buf[1] = 0x03;
	ft5x0x_i2c_write(tsdata->client, buf, sizeof(buf));

	mutex_unlock(&tsdata->mutex);
	#endif
}

static void ft5x0x_late_resume(struct early_suspend *es)
{
	struct ft5x0x_ts_data *tsdata;
	struct ft5x0x_platform_data *pdata;
	printk("############### %s ################\r\n",__func__);
	int err = 0;
	unsigned char buf[3] = {0};
	int try_count = 3;
	int i;
	bool bRetry = false;
#ifdef I2C_FT5x26
		unsigned char i2c_cmd[6] = {0};
#endif

	tsdata = (struct ft5x0x_ts_data *)container_of(es, struct ft5x0x_ts_data, early_suspend);
	pdata = (struct ft5x0x_platform_data *)(tsdata->client->dev.platform_data);

	/*reset to enter active mode */
	//ft5x0x_ts_reset(tsdata->client, pdata->reset);

#ifdef I2C_FT5x26
			i2c_cmd[0] = 0xeb;
			i2c_cmd[1] = 0xaa;
			i2c_cmd[2] = 0x09;
			for( i = 0; i < try_count; i++ )
			{
				bRetry = false;
				err = ft5x0x_i2c_read(tsdata->client, i2c_cmd, 3, i2c_cmd+3, 3);
				if (err < 0) {
					bRetry = true;
					dev_err(&tsdata->client->dev, "[FTS] switch to i2c standard protocol failure!\n");
				} else {
					if (0xeb == i2c_cmd[3] && 0xaa == i2c_cmd[4] && 0x08 == i2c_cmd[5]) {
						ft5x0x_touch_wakeup_reset();
						pr_info("[FTS] switch to i2c standard protocol!\n");
						break;
					} else {
						bRetry = true;
						dev_err(&tsdata->client->dev, "---[FTS] switch to i2c standard protocol failure!\
							0x%x 0x%x 0x%x\n", i2c_cmd[3], i2c_cmd[4], i2c_cmd[5]);
					}
				}
				if( bRetry == true )
				{
					ft5x0x_ts_reset(tsdata->client, pdata->reset);
					msleep(20);
					/*set to hibernate mode. */
					buf[0] = 0xeb;
					buf[1] = 0xaa;
					buf[2] = 0x09;
					mutex_lock(&tsdata->mutex);
					ft5x0x_i2c_write(tsdata->client, buf, sizeof(buf));
				    mutex_unlock(&tsdata->mutex);
				    msleep(30);
				    dev_err(&tsdata->client->dev, "[FTS] Try to set android mode.(time = %d)!\n", i+1);							
				}
			}
#endif
	enable_irq(tsdata->client->irq);
}
#endif
//static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
static int ft5x0x_ts_remove(struct i2c_client *client)
{
	const struct ft5x0x_platform_data *pdata =
	    dev_get_platdata(&client->dev);
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts = i2c_get_clientdata(client);

	input_unregister_device(ft5x0x_ts->input_dev);
	#ifdef FTS_APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
	#ifdef SYSFS_DEBUG
	ft5x0x_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif

	free_irq(client->irq, ft5x0x_ts);
	if (gpio_is_valid(pdata->irq))
		gpio_free(pdata->irq);

	if (gpio_is_valid(pdata->reset))
		gpio_free(pdata->reset);

	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}



static SIMPLE_DEV_PM_OPS(ft5x0x_ts_pm_ops, ft5x0x_ts_suspend, ft5x0x_ts_resume);

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{FT5X0X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct acpi_device_id ft5x0x_acpi_match[] = {
	{FT_1664S_NAME, 0},
	{FT_3432S_NAME, 0},
	{},
};

static struct i2c_driver ft5x0x_ts_driver = {
	.driver = {
		.name = FT5X0X_NAME,
		.owner = THIS_MODULE,
		.pm = &ft5x0x_ts_pm_ops,
		.acpi_match_table = ACPI_PTR(ft5x0x_acpi_match),
		},
	.probe = ft5x0x_ts_probe,
	//.remove = __devexit_p(ft5x0x_ts_remove),
	.remove = (ft5x0x_ts_remove),
	.id_table = ft5x0x_ts_id,
};

//Add by emdoor jim.kuang for FT5826 TP Driver
///////////////////////////////////////////////////////////////////
static ssize_t ft_value_store(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{	
	struct ts_event *event = &g_ptsdata->event;
	u8 buf1[3]={0xeb,0xaa,0x09};
	int i;
	if(!strncmp(buf,"debug",count-1)){
		for(i=0;i<10;i++)
		{
			printk("point[ %d ]= [ %d ] [ %d ] [ %d ]\r\n",i,event->touch_point,event->au8_finger_id[i],event->au8_touch_event[i]);
		}
	}
	ft5x0x_i2c_write(g_ptsdata->client, buf1, sizeof(buf1));
	
	return count;
}

static ssize_t ft_value_show(struct class *class, 
			struct class_attribute *attr,	char *buf)
{
	unsigned char i2c_cmd[6] = {0};	

	i2c_cmd[0] = 0xeb;
	i2c_cmd[1] = 0xaa;
	i2c_cmd[2] = 0x09;

	ft5x0x_i2c_read(g_ptsdata->client, i2c_cmd, 3, i2c_cmd+3, 3);		
	
	sprintf(buf,"i2c_cmd[3] 0xeb 0xaa 0x08 ===> 0x%x 0x%x 0x%x \r\n",i2c_cmd[3],i2c_cmd[4],i2c_cmd[5]);
}

static struct class_attribute ft_class_attrs[] = { 
	__ATTR(ft,0644,ft_value_show,ft_value_store),
    __ATTR_NULL
};

static struct class ft_interface_class = {
        .name = "ft_interface",
        .class_attrs = ft_class_attrs,
};

/////////////////////////////////////////////////////////////

static int __init ft5x0x_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft5x0x driver failed "
			"(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft5x0x_ts_driver.driver.name);
	}

	ret=class_register(&ft_interface_class);
	
	if(ret)
		printk("%s failed!\r\n",__func__);
	
	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	class_unregister(&ft_interface_class);
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("focaltech");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
