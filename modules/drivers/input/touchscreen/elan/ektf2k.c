/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 */

/* #define DEBUG */
#define ELAN_BUTTON
#define BUTTON_ID_INDEX 7
#define ELAN_KEY_BACK	0x10 // Elan Key's define
#define ELAN_KEY_HOME	0x08
#define ELAN_KEY_MENU	0x04
#define ELAN_KEY_SEARCH 0x20
//#define SOFTKEY_AXIS_VER

//#define ELAN_X_MAX	960
//#define ELAN_Y_MAX  1664
#define LCM_X_MAX  799
#define LCM_Y_MAX  1279

//#define ELAN_BUFFER_MODE

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/ektf2k.h> 
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include <asm/intel_scu_ipcutil.h>
#include <linux/intel_mid_hwid.h>

#if 0
#define PACKET_SIZE		34	
#else
#define PACKET_SIZE		18
#endif
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define NORMAL_PKT			0x5D

#define TWO_FINGERS_PKT      0x5A
#define FIVE_FINGERS_PKT       0x6D
#define TEN_FINGERS_PKT	0x62

#define RESET_PKT			0x77
#define CALIB_PKT			0xA8

#define ELAN_POR    1//modify for ESD

// modify
#define SYSTEM_RESET_PIN_SR 26	

//Add these Define
#define IAP_IN_DRIVER_MODE 	1
#define IAP_PORTION           1 //update_firware change 
#define PAGERETRY  30
#define IAPRESTART 5
#define CMD_54001234	1

static struct i2c_client *this_client;
static struct elan_ktf2k_i2c_platform_data *elan_pdata;
extern void *ekthf2k_platform_data(void *info);


//****************** For adb--Tcard Firmware Update  *********************
#define ELAN_IOCTLID	0xD0
#define CUSTOMER_IOCTLID 0xA0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK  _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK  _IOR(ELAN_IOCTLID, 16, int)
#if IAP_PORTION
#define IOCTL_FW_UPDATE  _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID,  2, int)
#endif
//*******************************************************************
#define ELAN_TP_DEFAULT_X_RESOLUTION 1088
#define ELAN_TP_DEFAULT_Y_RESOLUTION 1792

int  tpd_down_flag=0;
int  is_inupgrade = 0;
uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
static int X_RESOLUTION=ELAN_TP_DEFAULT_X_RESOLUTION;
static int Y_RESOLUTION=ELAN_TP_DEFAULT_Y_RESOLUTION;
int FW_ID=0x00;
int BC_VERSION = 0x00;
int work_lock=0x00;
int power_lock=0x00;
int circuit_ver=0x01;
int button_state = 0;
int first66=0;
/*++++i2c transfer start+++++++*/
int file_fops_addr=0x10;
/*++++i2c transfer end+++++++*/


int tp_suspend_flag = 0;

//#ifdef ELAN_POR
static int touch_esd_handler(void *unused);
static wait_queue_head_t esd_wait_queue;
static int esdflag = 0;
static struct task_struct *esd_thread=NULL;
struct hrtimer timer;
//#endif

#ifdef SOFTKEY_AXIS_VER
static int key_pressed = -1;

struct osd_offset{
	int left_x;
	int right_x;
	unsigned int key_event;
};

static struct osd_offset OSD_mapping[] = {
	{70, 170,  KEY_HOME},	//menu_left_x, menu_right_x, KEY_MENU
	{340, 440, KEY_MENU},	//home_left_x, home_right_x, KEY_HOME
	{550, 650, KEY_BACK},	//back_left_x, back_right_x, KEY_BACK
	{790, 890, KEY_SEARCH},	//search_left_x, search_right_x, KEY_SEARCH
};
#endif 

#if IAP_PORTION
uint8_t ic_status=0x00;	//0:OK 1:master fail 2:slave fail
int update_progree=0;
uint8_t I2C_DATA[3] = {0x10,0x20, 0x21};/*I2C devices address*/  
int is_OldBootCode = 0; // 0:new 1:old

/*The newest firmware, if update must be changed here*/
static uint8_t *file_fw_data= NULL;

static  uint8_t LAIBAO_FW_0X2895[]=
{
	#include "Laibao_for_Dell7_V5511_for_20140617.i"
};
static  uint8_t WINTEK_FW_0X2894[]=
{
	#include "WTK_for_Dell7_V5508_0617.i"
};
enum
{
	PageSize		= 132,
	PageNum		        = 249,
	ACK_Fail		= 0x00,
	ACK_OK			= 0xAA,
	ACK_REWRITE		= 0x55,
};

enum
{
	E_FD			= -1,
};
#endif
struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct early_suspend early_suspend;
	int intr_gpio;
	int reset;
// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
// For Firmare Update 
	struct miscdevice firmware;
};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct i2c_client *client);

#if IAP_PORTION
int Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);
int IAPReset(struct i2c_client *client);
#endif



/************** For  adb--Tcard Firmware Update ******************************************/
static int elan_iap_open(struct inode *inode, struct file *filp)
{
	printk("[ELAN]into elan_iap_open\n");
	is_inupgrade =1;
	if (private_ts == NULL)
	{
		printk("private_ts is NULL~~~");
	}		
	return 0;
}

static int elan_iap_release(struct inode *inode, struct file *filp)
{
	printk("[ELAN]into elan_iap_release");
	is_inupgrade =0;
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
    int ret;
    char *tmp;
    printk("[ELAN]into elan_iap_write\n");

    /*++++i2c transfer start+++++++*/    	
    struct i2c_adapter *adap = private_ts->client->adapter;    	
    struct i2c_msg msg;
    /*++++i2c transfer end+++++++*/	

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }

/*++++i2c transfer start+++++++*/
	//down(&worklock);
	msg.addr = file_fops_addr;
	msg.flags = 0x00;// 0x00
	msg.len = count;
	msg.buf = (char *)tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
/*++++i2c transfer end+++++++*/

    //if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    //return ret;
    return (ret == 1) ? count : ret;
}

static ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    char *tmp;
    int ret;  
    long rc;
    printk("[ELAN]into elan_iap_read\n");
   /*++++i2c transfer start+++++++*/
    	struct i2c_adapter *adap = private_ts->client->adapter;
    	struct i2c_msg msg;
/*++++i2c transfer end+++++++*/
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;
/*++++i2c transfer start+++++++*/
#if 1
	//down(&worklock);
	msg.addr = file_fops_addr;
	//msg.flags |= I2C_M_RD;
	msg.flags = 0x00;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = tmp;
	//up(&worklock);
	ret = i2c_transfer(adap, &msg, 1);
#else
    ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
/*++++i2c transfer end+++++++*/
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;
}

static long int elan_iap_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;
        char *path;
 	printk("[ELAN]into elan_iap_ioctl\n");
 	printk("cmd value %x\n",cmd);

	switch (cmd) {
		case IOCTL_I2C_SLAVE:
			//private_ts->client->addr = (int __user)arg;
			file_fops_addr = 0x10;
			break;   
		case IOCTL_MAJOR_FW_VER:
			break;
		case IOCTL_MINOR_FW_VER:
			break;
		case IOCTL_RESET:
		    	gpio_set_value(elan_pdata->reset, 0);
			msleep(20);
			gpio_set_value(elan_pdata->reset, 1);
			printk("elan ioctl reset\n");
			break;
		case IOCTL_IAP_MODE_LOCK:
			printk("elan this_client->irq==%d  line=%d\n\n",this_client->irq,__LINE__);
			if(work_lock==0)
			{
				work_lock=1;
				disable_irq(this_client->irq); //0909 song modify
				cancel_work_sync(&private_ts->work);
				printk("private_ts->client->irq==%d  line=%d\n\n",private_ts->client->irq,__LINE__);
			}
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			if(work_lock==1)
			{
				work_lock=0;
				enable_irq(this_client->irq);//0909 song modify
				printk("private_ts->client->irq==%d  line=%d\n\n",private_ts->client->irq,__LINE__);
			}
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client);
			return FW_ID;
			break;
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(elan_pdata->intr_gpio), ip);
			break;	
		case IOCTL_RESUME:
			elan_ktf2k_ts_resume(private_ts->client);
			break;
		case IOCTL_POWER_LOCK:
			power_lock=1;
			break;
		case IOCTL_POWER_UNLOCK:
			power_lock=0;
			break;
#if IAP_PORTION
		case IOCTL_GET_UPDATE_PROGREE:
			update_progree=(int __user)arg;
			break;

		case IOCTL_FW_UPDATE:
			path=(char *)arg;
		//	printk("update_fw path:%s\n",path);
			Update_FW_One(private_ts->client, 1);
#endif
		default:
			break;
	}
	return 0;
}

static struct file_operations elan_touch_fops = {
	.owner	 = THIS_MODULE,
	.open	 = elan_iap_open,
	.write	 = elan_iap_write,
	.read    = elan_iap_read,
	.release = elan_iap_release,
	//.unlocked_ioctl = elan_iap_ioctl,
	.compat_ioctl= elan_iap_ioctl,
};
// **********************************************************



#if IAP_PORTION
int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
	int len = 0;
	
	len = i2c_master_send(private_ts->client, isp_cmd,  4);
	if (len != 4) {
		printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	}
	else
		printk("[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) 
	{
		printk("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage,  byte);
	if (len != byte) 
	{
		printk("[ELAN] ERROR: write page error, write error. len=%d\r\n", len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	uint8_t buff[2] = {0};
	
	len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len);
		return -1;
	}

	pr_info("[ELAN] GetAckData:%x,%x",buff[0],buff[1]);
	if (buff[0] == 0xaa/* && buff[1] == 0xaa*/) 
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i=0; i<((page)/10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}
	
	page_tatol=page+249*(ic_num-j);
	percent = ((100*page)/(249));
	percent_tatol = ((100*page_tatol)/(249*ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249*ic_num))
		percent_tatol = 100;		

	printk("\rprogress %s| %d%%", str, percent);
	
	if (page == (249))
		printk("\n");
}
/* 
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
int IAPReset(struct i2c_client *client)
{
	int res;
	gpio_set_value(elan_pdata->reset, 0);
	msleep(20);
	gpio_set_value(elan_pdata->reset, 1);
	msleep(100);
	printk("[ELAN] read Hello packet data!\n"); 	  
	res= __hello_packet_handler(client);
	return res;
}


int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0; // For IAP_RESTART
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	#if 0
	uint8_t isp_cmd[] = {0x54, 0x00, 0x12, 0x34};
	#else
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};
	#endif
	private_ts = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:	

	data=I2C_DATA[0];//Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

	//if(recovery != 0x80)
	//{
       //	printk("[ELAN] Firmware upgrade normal mode !\n");
       	gpio_set_value(elan_pdata->reset,1);
		msleep(20);
		gpio_set_value(elan_pdata->reset,0);
		msleep(20);
		gpio_set_value(elan_pdata->reset,1);
		msleep(20);
		
		res = EnterISPMode(private_ts->client, isp_cmd);	 //enter ISP mode
		msleep(20);
	//} else
      	//	printk("[ELAN] Firmware upgrade recovery mode !\n");
	
	// Send Dummy Byte	
	printk("[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
	res = i2c_master_send(private_ts->client, &data,  sizeof(data));
	if(res!=sizeof(data))
	{
		printk("[ELAN] dummy error code = %d\n",res);
	}	
	msleep(20);


	// Start IAP
	for( iPage = 1; iPage <= PageNum; iPage++ ) 
	{
PAGE_REWRITE:
#if 0 
		for(byte_count=1;byte_count<=17;byte_count++)
		{
			if(byte_count!=17)
			{		
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 8;
				res = WritePage(szBuff, 8);
			}
			else
			{
				szBuff = file_fw_data + curIndex;
				curIndex =  curIndex + 4;
				res = WritePage(szBuff, 4); 
			}
		} // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif 
#if 1// 132byte mode		
		szBuff = file_fw_data + curIndex;
		curIndex =  curIndex + PageSize;
		res = WritePage(szBuff, PageSize);
#endif
#if 1
		if(iPage==249 || iPage==1)
		{
			msleep(300); 			 
		}
		else
		{
			msleep(30); 			 
		}	
#endif
		res = GetAckData(private_ts->client);

		if (ACK_OK != res) 
		{
			msleep(50); 
			printk("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
			if ( res == ACK_REWRITE ) 
			{
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY)
				{
					printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
					return E_FD;
				}
				else
				{
					printk("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
					curIndex = 0;
					goto PAGE_REWRITE;
				}
			}
			else
			{
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5)
				{
					printk("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
					return E_FD;
				}
				else
				{
					printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
					curIndex = 0;
					goto IAP_RESTART;
				}
			}
		}
		else
		{       printk("  data : 0x%02x ",  data);  
			rewriteCnt=0;
			print_progress(iPage,ic_num,i);
		}

		//msleep(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	if (IAPReset(client) > 0)
		printk("[ELAN] Update ALL Firmware successfully!\n");
	return 0;
}

#endif
// End Firmware Update

// Start sysfs
#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf2k_ts_late_resume(struct early_suspend *h);
#endif


static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(this_client);
	int status = 0, retry = 30;

	status = gpio_get_value(elan_pdata->intr_gpio);

        printk("%s: init gpio%d status=0x%x\n", __func__, elan_pdata->intr_gpio, status);
	
	do {
		status = gpio_get_value(elan_pdata->intr_gpio);
		retry--;
		msleep(20);
	} while (status >= 1 && retry > 0);
	
	printk( "\n[elan]%s: poll gpio: %d for interrupt status %s\n",
			__func__, elan_pdata->intr_gpio, status == 0 ? "low" : "high");

	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(this_client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	dev_err(&this_client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(this_client, cmd, 4)) != 4) {
		dev_err(&this_client->dev,
			"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(this_client);

	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(this_client, buf, size) != size || buf[0] != CMD_S_PKT)
		{
		   // printk("buf[0]=0x%x, 0x%x, 0x%x, 0x%x\n", buf[0], buf[1], buf[2], buf[3]); 	
		    return -EINVAL;
		}
	       //printk("buf[0]=0x%x, 0x%x, 0x%x, 0x%x\n", buf[0], buf[1], buf[2], buf[3]); 	
	}
	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };
	
	msleep(200);

	rc = elan_ktf2k_ts_poll(client);
	
	if (rc < 0) {
		printk( "[elan] %s: Int poll failed!\n", __func__);
		RECOVERY=0x80;
		return -EINVAL;
	}
	
	rc = i2c_master_recv(client, buf_recv, 8);
	
	if(rc<0)
	{
                 printk("[elan] %s: read packet i2c  error  !\n", __func__);
		 return rc;
	}
	if(buf_recv[0]!=0x55)
        {
                 printk("[elan] %s: packet is wrong  !\n", __func__);
                  return -EINVAL;
        }

	printk("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);
	if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
	{
             RECOVERY=0x80;
	     return RECOVERY; 
	}
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
        if(this_client==NULL){
	    return 0;
        }
        
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(this_client);
	int rc;
	uint16_t major=0;
        uint16_t minor=0;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
        uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};

// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver =(major << 8 | minor)&0x0ffff;
	FW_VERSION = ts->fw_ver;
// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id =( major << 8 | minor)&0x0ffff;
	FW_ID = ts->fw_id;
// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution =(minor)&0x0ffff;
	X_RESOLUTION = ts->x_resolution > 0 ? ts->x_resolution : ELAN_TP_DEFAULT_X_RESOLUTION;
	
// Y Resolution	
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution =(minor&0x0ffff);
	Y_RESOLUTION = ts->y_resolution > 0 ? ts->y_resolution : ELAN_TP_DEFAULT_Y_RESOLUTION;

// Bootcode version
	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver =( major << 8 | minor)&0x0ffff;
	BC_VERSION = ts->bc_ver;
	
	printk(KERN_INFO "[elan] %s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	printk(KERN_INFO "[elan] %s: firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, X_RESOLUTION, Y_RESOLUTION);
	printk(KERN_INFO "[elan] %s: bootcode version: 0x%4.4x\n",
			__func__, ts->bc_ver);
	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc;
	
	rc = __hello_packet_handler(client);
	return rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	//dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1];
	dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
	power_state = (power_state & PWR_STATE_MASK) >> 3;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf)
{

	int rc, bytes_to_recv=PACKET_SIZE;

	if (buf == NULL)
	    return -EINVAL;

	memset(buf, 0, sizeof(buf)); //bytes_to_recv);
	
	rc = i2c_master_recv(client, buf, bytes_to_recv);

	if (rc != bytes_to_recv)
		printk("[elan_debug] The first package error.\n");
#if 1
	//printk("[elan_recv_0-17] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17]);
#else
	   printk("[elan_recv_0-9] %x %x %x %x %x %x %x %x %x %x\n[elan_recv_10-19] %x %x %x %x %x %x %x %x %x %x\n[elan_recv_20-29] %x %x %x %x %x %x %x %x %x %x\n[elan_recv_30-33] %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17],buf[18], buf[19],  buf[20], buf[21], buf[22], buf[23], buf[24], buf[25], buf[26], buf[27], buf[28], buf[29], buf[30], buf[31], buf[32], buf[33]);
 #endif
	  // mdelay(1);  //modify for report rate can't be raise upper  than 50

#ifdef ELAN_POR  
	if (  (buf[0] == 0x52) && ((buf[1]&0xf0) == 0xf0))
	{
		esdflag=1;
		//printk( "=============esd wakeup\n");
		wake_up_interruptible(&esd_wait_queue);
		return;
	 }
#endif

	if(buf[0] == 0x66){
		printk("elan calibration 66 66 66 66 sucesss\n");
	}

	 
	return rc;
}

#ifdef SOFTKEY_AXIS_VER //SOFTKEY is reported via AXI
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx,btn_idx;
	int finger_num;
	int limitY =1365 ;//1380;//1408;//1340;// = ELAN_Y_MAX -100; // limitY need define by Case!


    if ((buf[0] == NORMAL_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
/* for 5 fingers*/
	    	finger_num = 5;
	    	num = buf[1] & 0x07; 
        	fbits = buf[1] >>3;
	    	idx=2;
	}else{
/* for 2 fingers */      
		finger_num = 2;
		num = buf[7] & 0x03; 
		fbits = buf[7] & 0x03;
		idx=1;
	}
		
	switch (buf[0]) {
		case NORMAL_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:	
		case TEN_FINGERS_PKT:
			 input_report_key(idev, BTN_TOUCH, 1);
			if (num == 0)
			{
				dev_dbg(&client->dev, "no press\n");
				if(key_pressed < 0){
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
					input_report_key(idev, BTN_TOUCH, 0);	//gxh-add---
					input_mt_sync(idev);
					input_sync(idev);
				}
				else {
					dev_err(&client->dev, "[elan] KEY_RELEASE: key_code:%d\n",OSD_mapping[key_pressed].key_event);
					input_report_key(idev, OSD_mapping[key_pressed].key_event, 0);
					input_sync(idev);
					key_pressed = -1;
				}
			}
			else 
			{			
				dev_dbg(&client->dev, "[elan] %d fingers\n", num);                        
				 input_report_key(idev, BTN_TOUCH, 1);
				for (i = 0; i < finger_num; i++) 
				{
					if ((fbits & 0x01)) 
					{
						elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);  
						//elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x);  		 
						//printk("[elan_debug] %s, x=%d, y=%d\n",__func__, x , y);
						//x = X_RESOLUTION-x;	 
						//y = Y_RESOLUTION-y;			     
					if (!((x<0) || (y<0) || (x>=X_RESOLUTION) || (y>=Y_RESOLUTION))) 
						{   
							if ( y < limitY )
						     	{
								//x = x * 320 / X_RESOLUTION;
								//y = y * 480 / Y_RESOLUTION;
								//printk("[elan_debug]  x=%d, y=%d\n", x , y);
			    					//input_report_abs(idev, ABS_MT_TRACKING_ID, i);
								input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
								input_report_abs(idev, ABS_MT_POSITION_X, x);
								input_report_abs(idev, ABS_MT_POSITION_Y, y);
								input_report_key(idev, BTN_TOUCH, 1);
								input_mt_sync(idev);
								//input_sync(idev);//add_lq
								reported++;						
							}
							else
							{
								if(y>1365&&y<1536)//if( y > 1440)
								{
							    		int i=0;
							    		for(i=0;i<4;i++)
							    		{
							    			if((x > OSD_mapping[i].left_x) && (x < OSD_mapping[i].right_x))
							    			{
											dev_err(&client->dev, "[elan] KEY_PRESS: key_code:%d\n",OSD_mapping[i].key_event);
											//printk("[elan] %d KEY_PRESS: key_code:%d\n", i, OSD_mapping[i].key_event);
							    				input_report_key(idev, OSD_mapping[i].key_event, 1);
											//input_sync(idev);add_lq
							    				key_pressed = i;
							    			  }
							    		  }
							    	   }
							   }
							//reported++;//del_add
							
					  	} // end if border
					} // end if finger status
				  	fbits = fbits >> 1;
				  	idx += 3;
				} // end for
			           }

			//input_sync(idev);
			if (reported) 
		        	input_sync(idev);
			else  
			{
		   	  input_mt_sync(idev);
			  input_sync(idev);
			}

			break;
	   	default:
				dev_err(&client->dev,
					"[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
				break;
	} // end switch
	return;
}
#else //SOFTKEY is reported via BTN bit
static uint8_t  num_li=0;
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits=0;
	uint8_t i, num, reported = 0;
	uint8_t idx,btn_idx;
	int finger_num;
	int rc;
        //printk( "[elan] %s:  line==%d\n", __func__,__LINE__);

	 if (buf[0] == TEN_FINGERS_PKT){
/* for 10 fingers */
	    	finger_num = 10;
	    	num = buf[2] & 0x0f; 
	    	fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
	    	idx=3;
		    btn_idx=33;
        }
	else if ((buf[0] == NORMAL_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
/* for 5 fingers */
	    	finger_num = 5;
	    	num = buf[1] & 0x07; 
        	fbits = buf[1] >>3;
	    	idx=2;
		num_li=num;
	}else{
/* for 2 fingers */      
		finger_num = 2;
		num = buf[7] & 0x03; 
		fbits = buf[7] & 0x03;
		idx=1;
	}
		
	switch (buf[0]) {
		case NORMAL_PKT:
		case TWO_FINGERS_PKT:
		case FIVE_FINGERS_PKT:	
		case TEN_FINGERS_PKT:
			 
			if (num == 0)
			{
				//input_report_key(idev, BTN_TOUCH, 1);//
				dev_dbg(&client->dev, "no press\n");
			
				//printk("tp button_state0 = %x\n",button_state);
        		//printk("tp buf[btn_idx] = %x KEY_MENU=%x KEY_HOME=%x KEY_BACK=%x KEY_SEARCH =%x\n",buf[btn_idx], KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH);
			
				#ifdef ELAN_BUTTON
						
					switch (buf[btn_idx]) {
				    	case ELAN_KEY_BACK:
						printk("KEY back 1\n");
									#ifndef LCT_VIRTUAL_KEY
			                        input_report_key(idev, KEY_BACK, 1);
									#else
									input_report_key(idev, BTN_TOUCH, 1);
									input_report_abs(idev, ABS_MT_TRACKING_ID, 6);
									input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 16);
									input_report_abs(idev, ABS_MT_POSITION_X, 180);
									input_report_abs(idev, ABS_MT_POSITION_Y, 1360);
									#endif
			                        button_state = KEY_BACK;
							tpd_down_flag = 0;
						break;
						
					case ELAN_KEY_HOME:
						printk("KEY home 1\n");
									#ifndef LCT_VIRTUAL_KEY
			                        input_report_key(idev, KEY_HOMEPAGE, 1);
									#else
									input_report_key(idev, BTN_TOUCH, 1);
									input_report_abs(idev, ABS_MT_TRACKING_ID, 7);
									input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 16);
									input_report_abs(idev, ABS_MT_POSITION_X, 300);
									input_report_abs(idev, ABS_MT_POSITION_Y, 1360);
									#endif
			                        button_state = KEY_HOMEPAGE;
						tpd_down_flag = 0;
						break;
						
					case ELAN_KEY_MENU:
						printk("KEY menu 1\n");
						#ifndef LCT_VIRTUAL_KEY
                        			input_report_key(idev, KEY_MENU, 1);
						#else
						input_report_key(idev, BTN_TOUCH, 1);
						input_report_abs(idev, ABS_MT_TRACKING_ID, 8);
						input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 16);
						input_report_abs(idev, ABS_MT_POSITION_X, 60);
						input_report_abs(idev, ABS_MT_POSITION_Y, 1360);
						#endif
			                        button_state = KEY_MENU;
						   tpd_down_flag = 0;
						break;
				
			     // TOUCH release
					default: 	
						if(tpd_down_flag == 0)
						{
							//key up
							if(button_state != 0)
							{
								input_report_key(idev, button_state, 0);
								button_state = 0;
							}
						}	
						else
						{
					    		//printk("[ELAN ] test tpd up\n");
							input_report_key(idev, BTN_TOUCH, 0);
							//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
							input_mt_sync(idev);
		            				tpd_down_flag = 0;
						}
               				break;
				    }   
				#endif		      
		}
			else 
			{			
				input_report_key(idev, BTN_TOUCH, 1);
				for (i = 0; i < finger_num; i++) 
				{	
					if ((fbits & 0x01)) 
					{
						elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);  
						//elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x); 
						x = ( x *LCM_X_MAX )/X_RESOLUTION;
						y = ( y *LCM_Y_MAX )/Y_RESOLUTION;
									     
						if (!((x<0) || (y<0) || (y>LCM_Y_MAX) || (x>LCM_X_MAX))) //
						{   
							#if 0
							if(boot_mode == FACTORY_BOOT)
							{
								int tmp = x;
				 				x = LCM_Y_MAX - y -1;
								y = tmp; 
							}
							#endif
							input_report_key(idev, BTN_TOUCH, 1);
							input_report_abs(idev, ABS_MT_TRACKING_ID, i);
							input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 16);
							input_report_abs(idev, ABS_MT_POSITION_X, x);
							input_report_abs(idev, ABS_MT_POSITION_Y, y);
							input_mt_sync(idev);
							reported++;
							tpd_down_flag=1;

					  	} // end if border
					} // end if finger status
				  	fbits = fbits >> 1;
				  	idx += 3;
				} // end for
			}
			if (reported)
				input_sync(idev);
			else 
			{
				input_mt_sync(idev);
				input_sync(idev);
			}
			break;
	   	default:
				//printk("[elan] %s: unknown packet type: %02x, %02x, %02x, %02x\n", __func__, buf[0], buf[1], buf[2], buf[3]);
				break;
	} // end switch
	return;
}
#endif
static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[PACKET_SIZE] = { 0 };
#if 0
if (gpio_get_value(elan_pdata->intr_gpio))
		{
         printk( "[elan] %s:  int_pin_status==%d\n", __func__,ts->intr_gpio);
			enable_irq(this_client->client->irq);
			return;
		}
#endif	
		rc = elan_ktf2k_ts_recv_data(this_client, buf);
 
		if (rc < 0)
		{
			enable_irq(this_client->irq);
                           printk( "[elan] %s:  rc==%d\n", __func__,rc);
			return;
		}

		elan_ktf2k_ts_report_data(this_client, buf);

		enable_irq(this_client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;
	
	//printk( "int irq -------> [elan] %s\n", __func__);

	dev_dbg(&client->dev, "[elan] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}

#ifdef ELAN_POR
static int elan_esd_check(void)
{
	int version=0;
	int rc=0;
	int ret;
	uint8_t cmd[] = {CMD_R_PKT, 0xf0, 0x00, 0x01};
	uint8_t buf[4]={0};	
	       //  printk( "[elan] %s\n", __func__);

	if(tp_suspend_flag==1){
                  return false;
	}
	else
    	{
    		rc=i2c_master_send(private_ts->client, cmd, 4);
    		if (rc != sizeof(cmd)/sizeof(cmd[0])) 
    		{
			printk("elan send cmd fail\n");
			return true;
    		} 
    		else
    		{
    			long ret = wait_event_interruptible_timeout(esd_wait_queue, 
                                            esdflag!=0,
                                            1*HZ);
			 esdflag=0;
			if(ret==0){
				return true;
			}
    		}
	}
	return false;
		
}
#endif

/* Elan POR start*/
static int touch_esd_handler(void *unused)
{
	int version=0;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t buf[4]={0};
	int rc;
	//timer_flag = 1;
	int New_FW_ID;	
	int New_FW_VER;

	msleep(1000);

	disable_irq(this_client->irq);
	/*FW info start....*/
	if(RECOVERY != 0x80){
		
		rc = __fw_packet_handler(this_client);
		if (rc < 0)
			printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
		//enable_irq(this_client->irq);	
	}
       
	/*check update start...*/
#if IAP_PORTION
	if(1)
	{
		msleep(500);
		printk("[ELAN]start update.................!!\n");
		work_lock=1;
		//disable_irq(this_client->irq);
		//cancel_work_sync(&private_ts->work);
		power_lock = 1;
		if(FW_ID==0x2895)
			file_fw_data=LAIBAO_FW_0X2895;
		else if(FW_ID==0x2894)			
			file_fw_data=WINTEK_FW_0X2894;
			
		else{
			if(intel_mid_get_panel_id() == HW_PANEL_7_AUO)
				file_fw_data=WINTEK_FW_0X2894;
			else
				file_fw_data=LAIBAO_FW_0X2895;
			}
		printk(" [7b64]=0x%02x,  [7b65]=0x%02x, [7b66]=0x%02x, [7b67]=0x%02x\n",  
		file_fw_data[32100],file_fw_data[32101],file_fw_data[32102],file_fw_data[32103]);
		New_FW_ID = file_fw_data[32103]<<8  | file_fw_data[32102] ;	       
		New_FW_VER =0x55<<8  | file_fw_data[32100] ;
		printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);   	       
		printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);  
	#if 0
		Update_FW_One(client, RECOVERY);
	#else
		if (New_FW_ID   ==  FW_ID || FW_ID == 0){		      
			if (New_FW_VER > (FW_VERSION)) {
				Update_FW_One(this_client, RECOVERY);
				/*read new fw info*/
				if(RECOVERY == 0x80)
					rc = __fw_packet_handler(this_client);
			}	
			else
			printk(" FW is new, needn't to upgrade firmware!\n!");
		} else {                        
			printk("FW_ID is different!\n");
		}	
	#endif	
		power_lock = 0;
		work_lock=0;
		//enable_irq(this_client->irq);
	}
#endif
	/*check update end...*/

	enable_irq(this_client->irq);
	
#ifdef ELAN_POR	
	printk( "[elan] %s\n", __func__);
    
	do
	{
		msleep(2000);

		if(is_inupgrade ==1)
		{
			printk("[elan] ctp_esd_recovery_kthread TP IN UPGRADE,WE do not check esd\n");
			continue;
		}
		
		set_current_state(TASK_RUNNING);

		if(elan_esd_check()==true)
		{
			printk("[elan] ic will be POR\n");
			msleep(20);
			gpio_set_value(elan_pdata->reset, 1);
			msleep(10);
			gpio_set_value(elan_pdata->reset, 0);
			msleep(20);
			gpio_set_value(elan_pdata->reset, 1);
			msleep(50);
		}
		else
		{
			//printk("[elan]: The touch TP check OK!\n");
		}
	}while(!kthread_should_stop());
#endif
	
	return 0;
}


/*--------------------------- sys add ---------------------------*/

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;
        int i = 100;
	ret = gpio_get_value(ts->intr_gpio);
	printk(KERN_DEBUG "GPIO_TP_INT_N=%d\n", ts->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	for(i=100;i>0;i--){

            gpio_set_value(elan_pdata->reset, 0);
	    msleep(20);
            gpio_set_value(elan_pdata->reset, 1);
	    msleep(1000);
	}

	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);


/****************************2013.10.17 add*****************************/
#if 1
static ssize_t elan_ktf2k_diff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
        ssize_t ret = 0;
	uint8_t adcinfo_buff1[17]={0};
	uint8_t cmd_getinfo[] = {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	int x, y,i;
	char Len_H, Len_L;
	int xlenght=0;
	int len;
	int TPlength=0;
	int pagereceive=0;
	uint16_t rawdata;
	uint8_t hello_buff[4]={0};
	uint8_t rek_buff[4]={0};

	uint16_t rawdata2[1000]={0};
	int m=0;
	int q,p;
	int frame=0;
		
	//char rawdata;
	int receivecnt;
	int chip_num;
	int j=0;
	char *buf1=NULL;
	char adc_buff[6]={0};
	uint8_t raw_buff[63]={0};
	uint8_t cmd_test[] = {0x55, 0x55, 0x55, 0x55}; 
	uint8_t super_cmd_test[] = {0x66, 0x66, 0x66, 0x66};
	uint8_t cmd_close[] = {0xA5,0xA5,0xA5,0xA5}; 
	uint8_t cmd_stop[] = {0x9F,0x00,0x00,0x01};
	uint8_t prom_cmd[] = {0x54, 0x27, 0x01, 0x01};
        ssize_t  num_read_chars = 0;

	tp_suspend_flag = 1;		  
	disable_irq(private_ts->client->irq);
	cancel_work_sync(&private_ts->work);
        //add by aaron
        gpio_set_value(elan_pdata->reset, 1);
        msleep(40);
        gpio_set_value(elan_pdata->reset, 0);
        msleep(40);
        gpio_set_value(elan_pdata->reset, 1);
        msleep(400);                                           //the dylay time can be modify
        ret=elan_ktf2k_ts_poll(private_ts->client);
       if(ret<0)
       {
       printk("[elan]:diag_function int poll low fail/n");
       }
        i2c_master_recv(private_ts->client, hello_buff, sizeof(hello_buff));
        if(hello_buff[0]==0x55&&hello_buff[1]==0x55&&hello_buff[2]==0x55&&hello_buff[3]==0x55)
        printk("[elan]:diag_function hello_packet ok\n");
        else
        printk("[elan]:diag_function hello_packet fail\n");
        msleep(300);                                            //this can be modify, but the least is 200
        ret=elan_ktf2k_ts_poll(private_ts->client);
        i2c_master_recv(private_ts->client, hello_buff, sizeof(hello_buff));
        if(rek_buff[0]==0x66&&rek_buff[1]==0x66&&rek_buff[2]==0x66&&rek_buff[3]==0x66)
        printk("[elan]:diag_function rek_packet ok\n");
        else
        printk("[elan]:diag_function rek_packet fail\n");
        msleep(30);


#if 0
	ret=i2c_master_send(private_ts->client, super_cmd_test, 4);//0x66, 0x66, 0x66, 0x66
	if(ret!=4){
		printk("[elan]:send super_cmd_test fail\n");		
	}
#endif

	ret=i2c_master_send(private_ts->client, cmd_getinfo, 6);//0x5B, 0x00, 0x00, 0x00, 0x00, 0x00
	if(ret!=6){
		printk("[elan]:send cmd_getinfo fail ret=%d\n",ret);
		num_read_chars = sprintf(buf,"%s:send cmd_getinfo fai!\n",__func__);
		goto aaron_ffff;
		}
	ret = elan_ktf2k_ts_poll(private_ts->client);
		if (ret < 0){
			printk("[elan]:poll fail ret=%d\n",ret);

			goto aaron_ffff;}

		else {
			len = i2c_master_recv(private_ts->client, adcinfo_buff1, sizeof(adcinfo_buff1));
		}	
	if(len!=sizeof(adcinfo_buff1)){
		printk("[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n", len);
		num_read_chars = sprintf(buf,"%s:[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n",__func__,len);
		}
		 printk("[ELAN] ADC INFO : 0x%02x,  0x%02x,  0x%02x,  0x%02x, 0x%02x,  0x%02x,  0x%02x,  0x%02x,0x%02x,  0x%02x,  0x%02x,  0x%02x, 0x%02x,  0x%02x,  0x%02x,  0x%02x,  0x%02x\r\n", 
		 	adcinfo_buff1[0], adcinfo_buff1[1], adcinfo_buff1[2], adcinfo_buff1[3], adcinfo_buff1[4], adcinfo_buff1[5], 
		 	adcinfo_buff1[6], adcinfo_buff1[7], adcinfo_buff1[8], adcinfo_buff1[9], adcinfo_buff1[10], adcinfo_buff1[11],
		 	adcinfo_buff1[12], adcinfo_buff1[13], adcinfo_buff1[14], adcinfo_buff1[15], adcinfo_buff1[16]);
		 
	x = adcinfo_buff1[2]+adcinfo_buff1[6]+adcinfo_buff1[10]+adcinfo_buff1[14];
	y = adcinfo_buff1[3]+adcinfo_buff1[7]+adcinfo_buff1[11]+adcinfo_buff1[15];
	
	TPlength = x* y*2;
	xlenght=x;
	adc_buff[0] = 0x58;
	adc_buff[1] = 0x04;//0x03;
	adc_buff[2] = 0x00;
	adc_buff[3] = 0x00;
		
	Len_H = (TPlength & 0xff00) >> 8;
	Len_L= TPlength & 0x00ff;
	adc_buff[4] = Len_H;
	adc_buff[5] = Len_L;
       pagereceive = TPlength / 60 +1;	
	   
  	chip_num = (adcinfo_buff1[1]!=0) +(adcinfo_buff1[5]!=0)+(adcinfo_buff1[9]!=0)+(adcinfo_buff1[13]!=0) ;
  	printk("[Info]%d,%d,%d,%d,%d\n", chip_num, x, y,pagereceive,TPlength); 
	num_read_chars += sprintf(&(buf[num_read_chars]),"eKTH3244EWS  tp channel: %u * %u\n", x, y);
	num_read_chars += sprintf(&(buf[num_read_chars]),"Delta data:\n");


  	len = i2c_master_send(private_ts->client, cmd_test, sizeof(cmd_test));//0x55, 0x55, 0x55, 0x55
  	if(len!=4){
		printk("[elan]:send cmd_test fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send cmd_test fail!\n", __func__);
		goto aaron_ffff;

		}
	msleep(100); /*wait test mode 100ms*/
	 len = i2c_master_send(private_ts->client, prom_cmd, sizeof(prom_cmd));//0x54, 0x27, 0x01, 0x01
  	if(len!=4){
		printk("[elan]:send cmd_test fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send cmd_test fail!\n", __func__);
		goto aaron_ffff;

		}
	
	len = i2c_master_send(private_ts->client, adc_buff, sizeof(adc_buff));//0x58,0x04,0x00,0x00,XXXX,XXXX
	if(len!=sizeof(adc_buff))
	{
		printk("[elan]:send adc_buff fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send adc_buff fail!\n", __func__);
		goto aaron_ffff;

		}
	#if 1
	for(receivecnt=0;receivecnt< pagereceive; receivecnt++)
	{
		ret = elan_ktf2k_ts_poll(private_ts->client);
		if (ret < 0){
			goto aaron_ffff;}

			else {
					if (i2c_master_recv(private_ts->client, raw_buff, sizeof(raw_buff)) != sizeof(raw_buff))    
						{
		                                     num_read_chars += sprintf(&(buf[num_read_chars]),"%s:aaron get raw_buff is fail!\n", __func__);
					           printk("aaron get raw_buff is fail\n");	
	                	                             goto aaron_ffff;
						}
					else{
						printk(" raw_buff [0-9]:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",raw_buff[0],raw_buff[1],raw_buff[2],raw_buff[3],raw_buff[4],raw_buff[5],raw_buff[6],raw_buff[7],raw_buff[8],raw_buff[9]);
                                                        for(i=3;i<63;i=i+2)
                                                        {
                                                       	
                                                       	rawdata=raw_buff[i+1]| (raw_buff[i]<<8);
                                                       	if(rawdata==0xffff)
                                                       	{	
                                                       	goto aaron_ffff1;
                                                       	}	
														
								#if	1	//li				
								rawdata2[m]=rawdata;
								m++;
								printk(" %d ",rawdata);
								#else
								if(i==63)
                                                       	{printk("\n");	
                                                                num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
                                                       		}
                                                       	printk(" %d ",rawdata);
                                                       	num_read_chars += sprintf(&(buf[num_read_chars]), " %.5d ",rawdata);
								#endif		 		
                                                           }
					}		
				 //num_read_chars += sprintf(&(buf[num_read_chars]),"\n");//li
				 printk("\n");
				 
	       }
	}
#endif	

aaron_ffff1:
#if 1
m= 0;
	for (q= 1; q <= y; q++) {
  
		for (p= 1; p<= x; p++) {
                   num_read_chars += sprintf(&(buf[num_read_chars])," %4d",rawdata2[m]);
				   m++;
		}
	  num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
	}
 //num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
#endif

aaron_ffff:
      // printk("[elan]:----------4--num_read_chars=%d-------\n",num_read_chars);
	i2c_master_send(private_ts->client, cmd_close, sizeof(cmd_close));//0xA5,0xA5,0xA5,0xA5
	msleep(30);
	i2c_master_send(private_ts->client, cmd_stop, sizeof(cmd_stop));//0x9F,0x00,0x00,0x01
	enable_irq(private_ts->client->irq);
	tp_suspend_flag = 0;	
	return  num_read_chars ;
	
}

static DEVICE_ATTR(diff, S_IRUGO, elan_ktf2k_diff_show, NULL);
#endif

#if 1
static ssize_t elan_ktf2k_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	uint8_t adcinfo_buff1[17]={0};
	uint8_t adcinfo_buf11[4]={0};

	uint8_t cmd_getinfo[] = {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	int x, y,i;
	char Len_H, Len_L;
	int xlenght=0;
	int len;
	int TPlength=0;
	int pagereceive=0;
	uint16_t rawdata;
	
	uint16_t rawdata2[1000]={0};
	int m=0;
	int q,p;
		
	//char rawdata;
	int receivecnt;
	int chip_num;
	int j=0;
	char *buf1=NULL;
	char adc_buff[6]={0};
	uint8_t raw_buff[63]={0};
	uint8_t cmd_test[] = {0x55, 0x55, 0x55, 0x55};
	uint8_t cmd_test_buff[] = { 0x53, 0x90, 0x00, 0x01 };
	uint8_t diamond_test_buff[] = { 0x54, 0x8d, 0x01, 0x01 };
	uint8_t exit_diamond_test_buff[] = { 0x54, 0x8d, 0x00, 0x01 };
	uint8_t super_cmd_test[] = {0x66, 0x66, 0x66, 0x66};
	uint8_t cmd_close[] = {0xA5,0xA5,0xA5,0xA5}; 
	uint8_t cmd_stop[] = {0x9F,0x00,0x00,0x01};
	uint8_t partial_scan[] = { 0x54, 0x26, 0x01, 0x01 };
	uint8_t exit_partial_scan[] = { 0x54, 0x26, 0x00, 0x01 };
	uint8_t cal_cmd_1[] = { 0x54, 0xc0, 0xe1, 0x5a };
	uint8_t cal_cmd_2[] = { 0x54, 0x29, 0x00, 0x01 };
	uint8_t check_re_k_buf[] ={0};
	uint8_t prom_cmd[] = {0x54, 0x8d, 0x01, 0x01};
       ssize_t  num_read_chars = 0;
	   
	tp_suspend_flag = 1;	  
	disable_irq(private_ts->client->irq);
	cancel_work_sync(&private_ts->work);
	#if 0
	ret=i2c_master_send(private_ts->client, super_cmd_test, 4);//0x66, 0x66, 0x66, 0x66
	if(ret!=4){
		printk("[elan]:send super_cmd_test fail\n");		
	}
	msleep(50);
	#endif 
	
	ret=i2c_master_send(private_ts->client, cmd_getinfo, 6);//0x5B, 0x00, 0x00, 0x00, 0x00, 0x00
	if(ret!=6){
		printk("[elan]:send cmd_getinfo fail ret=%d\n",ret);
		num_read_chars = sprintf(buf,"send cmd_getinfo fai!\n");
		goto aaron_ffff;
		}
	ret = elan_ktf2k_ts_poll(private_ts->client);
		if (ret < 0){
			printk("[elan]:poll fail ret=%d\n",ret);

			goto aaron_ffff;}

		else {
			len = i2c_master_recv(private_ts->client, adcinfo_buff1, sizeof(adcinfo_buff1));
			if(len!=sizeof(adcinfo_buff1)){
			   printk("[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n", len);
			   num_read_chars = sprintf(buf,"%s:[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n",__func__,len);
			}
		}	
	
		 printk("[ELAN] ADC INFO : 0x%02x,  0x%02x,  0x%02x,  0x%02x, 0x%02x,  0x%02x,  0x%02x,  0x%02x,0x%02x,  0x%02x,  0x%02x,  0x%02x, 0x%02x,  0x%02x,  0x%02x,  0x%02x,  0x%02x\r\n", 
		 	adcinfo_buff1[0], adcinfo_buff1[1], adcinfo_buff1[2], adcinfo_buff1[3], adcinfo_buff1[4], adcinfo_buff1[5], 
		 	adcinfo_buff1[6], adcinfo_buff1[7], adcinfo_buff1[8], adcinfo_buff1[9], adcinfo_buff1[10], adcinfo_buff1[11],
		 	adcinfo_buff1[12], adcinfo_buff1[13], adcinfo_buff1[14], adcinfo_buff1[15], adcinfo_buff1[16]);
		 
	x = adcinfo_buff1[2]+adcinfo_buff1[6]+adcinfo_buff1[10]+adcinfo_buff1[14];
	y = adcinfo_buff1[3]+adcinfo_buff1[7]+adcinfo_buff1[11]+adcinfo_buff1[15];
	TPlength = x* y*2;
	xlenght=x;
	adc_buff[0] = 0x58;
	adc_buff[1] = 0x03;
	adc_buff[2] = 0x00;
	adc_buff[3] = 0x00;
		
	Len_H = (TPlength & 0xff00) >> 8;
	Len_L= TPlength & 0x00ff;
	adc_buff[4] = Len_H;
	adc_buff[5] = Len_L;
       pagereceive = TPlength / 60 +1;	
	   
  	chip_num = (adcinfo_buff1[1]!=0) +(adcinfo_buff1[5]!=0)+(adcinfo_buff1[9]!=0)+(adcinfo_buff1[13]!=0) ;
  	printk("[Info]%d,%d,%d,%d,%d\n", chip_num, x, y,pagereceive,TPlength); 
	num_read_chars += sprintf(&(buf[num_read_chars]),"eKTH3244EWS  tp channel: %u * %u\n", x, y);
	num_read_chars += sprintf(&(buf[num_read_chars]),"Reference data:\n");

	len = i2c_master_send(private_ts->client, partial_scan, sizeof(partial_scan));//0x54, 0x26, 0x01, 0x01
  	if(len!=sizeof(partial_scan)){
		printk("[elan]:send partial_scan  fail\n");
	        num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send partial_scan fail!\n", __func__);
		goto aaron_ffff;
	}
	len = i2c_master_send(private_ts->client, diamond_test_buff, sizeof(diamond_test_buff));//0x54, 0x8d, 0x01, 0x01
  	if(len!=sizeof(diamond_test_buff)){
		printk("[elan]:send diamond_test_buff  fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send diamond_test_buff fail!\n", __func__);
		goto aaron_ffff;
	}
	msleep(800);
	len =i2c_master_send(private_ts->client, cal_cmd_1, sizeof(cal_cmd_1));//0x54, 0xc0, 0xe1, 0x5a
	if (len != sizeof(cal_cmd_1)) {
		printk("[ELAN] ERROR: cal_cmd_1 fail(Write)! len=%d\r\n", len);
		goto aaron_ffff;
	}
	len =i2c_master_send(private_ts->client, cal_cmd_2, sizeof(cal_cmd_2));//0x54, 0x29, 0x00, 0x01
	if (len != sizeof(cal_cmd_2)) {
		printk("[ELAN] ERROR: cal_cmd_2 fail(Write)! len=%d\r\n", len);
		goto aaron_ffff;
	}
	msleep(800);
	ret = elan_ktf2k_ts_poll(private_ts->client);
       if (ret < 0){
		printk("[elan]:poll fail ret=%d\n",ret);
		goto aaron_ffff;}
      else {
		len = i2c_master_recv(private_ts->client, adcinfo_buf11, sizeof(adcinfo_buf11));
		if(len!=sizeof(adcinfo_buf11)){
		 printk("[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n", len);
		  num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[ELAN] ERROR: Read ADC INFO fail(Read )! len=%d\r\n",__func__,len);
		}
		else{
			//num_read_chars = sprintf(buf,"--1--check rek status %x:%x:%x:%x\n",check_re_k_buf[0],check_re_k_buf[1], check_re_k_buf[2], check_re_k_buf[3]);
			printk("[ELAN] --1--check rek status %x:%x:%x:%x\n",check_re_k_buf[0],check_re_k_buf[1], check_re_k_buf[2], check_re_k_buf[3]);
		}
	}


  	len = i2c_master_send(private_ts->client, cmd_test, sizeof(cmd_test));//0x55, 0x55, 0x55, 0x55
  	if(len!=4){
		printk("[elan]:send cmd_test fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send cmd_test fail!\n", __func__);
		goto aaron_ffff;

		}
	msleep(200);
	len = i2c_master_send(private_ts->client, adc_buff, sizeof(adc_buff));// 0x58 ,0x03,0x00,0x00,XXXX,XXXX
	if(len!=sizeof(adc_buff))
	{
		printk("[elan]:send adc_buff fail\n");
	         num_read_chars += sprintf(&(buf[num_read_chars]),"%s:[elan]:send adc_buff fail!\n", __func__);
		goto aaron_ffff;

		}
	#if 1
	for(receivecnt=0;receivecnt< pagereceive; receivecnt++)
	{
		ret = elan_ktf2k_ts_poll(private_ts->client);
		if (ret < 0){
			goto aaron_ffff;}

			else {
					if (i2c_master_recv(private_ts->client, raw_buff, sizeof(raw_buff)) != sizeof(raw_buff))    
						{
		                                     num_read_chars += sprintf(&(buf[num_read_chars]),"%s:aaron get raw_buff is fail!\n", __func__);
					           printk("aaron get raw_buff is fail\n");	
	                	                             goto aaron_ffff;
						}
					else{
					   printk(" raw_buff [0-9]:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",raw_buff[0],raw_buff[1],raw_buff[2],raw_buff[3],raw_buff[4],raw_buff[5],raw_buff[6],raw_buff[7],raw_buff[8],raw_buff[9]);
                                                        for(i=3;i<63;i=i+2)
                                                        {
                                                       	
                                                       	rawdata=raw_buff[i+1]| (raw_buff[i]<<8);
                                                       	if(rawdata==0xffff)
                                                       	{	
                                                       	goto aaron_ffff1;}	
														
								#if	1					
								rawdata2[m]=rawdata;
								m++;
								printk(" %d ",rawdata);
								#else
								if(i==63)
                                                       	{printk("\n");	
                                                                num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
                                                       		}
                                                       	printk(" %d ",rawdata);
                                                       	num_read_chars += sprintf(&(buf[num_read_chars]), " %.5d ",rawdata);
								#endif		 		
                                                           }
					}		
				 //num_read_chars += sprintf(&(buf[num_read_chars]),"\n");//li
				 printk("\n");
	       }
	}
#endif	

aaron_ffff1:
#if 1
//printk("_0m=%d ", m);
m= 0;
for (q= 1; q <= y; q++) {
  
	for (p= 1; p<= x; p++) {
                   num_read_chars += sprintf(&(buf[num_read_chars])," %d",rawdata2[m]);
				   m++;
		}
	  num_read_chars += sprintf(&(buf[num_read_chars]),"\n");
}
 //num_read_chars += sprintf(&(buf[num_read_chars]),"\n");

#endif
aaron_ffff:
	i2c_master_send(private_ts->client, cmd_close, sizeof(cmd_close));//0xA5,0xA5,0xA5,0xA5
	msleep(100);
	i2c_master_send(private_ts->client, exit_diamond_test_buff, sizeof(exit_diamond_test_buff));//0x54, 0x8d, 0x00, 0x01
	msleep(100);
	i2c_master_send(private_ts->client, cal_cmd_1, sizeof(cal_cmd_1));//0x54, 0xc0, 0xe1, 0x5a
	i2c_master_send(private_ts->client, cal_cmd_2, sizeof(cal_cmd_2));//0x54, 0x29, 0x00, 0x01
	msleep(800);
	ret = elan_ktf2k_ts_poll(private_ts->client);
       if (ret < 0){
		printk("[elan]:poll fail ret=%d\n",ret);
		}
      else {
		len = i2c_master_recv(private_ts->client, adcinfo_buf11, sizeof(adcinfo_buf11));
		if(len!=sizeof(adcinfo_buf11)){
		 printk("[ELAN] ERROR: Read rek status fail(Read )! len=%d\r\n", len);
		  num_read_chars = sprintf(buf,"%s:[ELAN] ERROR: Read rek status fail(Read )! len=%d\r\n",__func__,len);
		}
		else{
			//num_read_chars = sprintf(buf,"--2--check rek status %x:%x:%x:%x\n",check_re_k_buf[0],check_re_k_buf[1], check_re_k_buf[2], check_re_k_buf[3]);
                   printk("--2--check rek status %x:%x:%x:%x\n",check_re_k_buf[0],check_re_k_buf[1], check_re_k_buf[2], check_re_k_buf[3]);

		}
	}
	len =i2c_master_send(private_ts->client, exit_partial_scan, sizeof(exit_partial_scan));//0x54, 0x26, 0x00, 0x01
	if (len != sizeof(exit_partial_scan)) {
		printk("[ELAN] ERROR: exit_partial_scan fail(Write)! len=%d\r\n", len);
	}
	enable_irq(private_ts->client->irq);
	tp_suspend_flag = 0;	
	
	return  num_read_chars;
}
static DEVICE_ATTR(diag, S_IRUGO, elan_ktf2k_diag_show, NULL);

#endif

//add extra debug interface by intel
static ssize_t elan_ktf2k_devinfo(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

        ret = __fw_packet_handler(private_ts->client);
    
	return ret;
}
static DEVICE_ATTR(devinfo, S_IRUGO, elan_ktf2k_devinfo, NULL);
static ssize_t elan_ktf2k_tpinfo(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	if(FW_ID==0||FW_VERSION==0)
	num_read_chars = sprintf(&buf[num_read_chars], "%s  ","Get TP module ID fail! \n");
	else{
		if(FW_ID==0X2895)
			num_read_chars = sprintf(&buf[num_read_chars], "TP module is LAIBAO! VERSION=0x%x\n",FW_VERSION);
		else
			num_read_chars = sprintf(&buf[num_read_chars], "TP module is Wintek! VERSION=0x%x\n",FW_VERSION);
		}
	return num_read_chars;
}
static DEVICE_ATTR(tpinfo, S_IRUGO, elan_ktf2k_tpinfo, NULL);


static ssize_t elan_point_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	int point=5;
	if(point < 0)
	{
		num_read_chars = sprintf(&buf[num_read_chars], "%s","get TP point failed!\n");;
	}
	else 
	{
		num_read_chars = sprintf(&buf[num_read_chars], "%d\n",point);	
	}	

	return num_read_chars;
}

static ssize_t elan_point_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}
static DEVICE_ATTR(point, (S_IWUSR|S_IRUGO), elan_point_show,
			elan_point_store);

/*********************************************************************/
static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	/******************2013.10.17 add**************************/
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs create diag failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diff.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs create diff  failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_point.attr);
	if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs create point  failed\n", __func__);
		return ret;
	}

    // add extra debug interface by intel 
	    ret = sysfs_create_file(android_touch_kobj, &dev_attr_devinfo.attr);
	    if (ret) {
			printk(KERN_ERR "[elan]%s: sysfs create devinfo  failed\n", __func__);
			return ret;
		}
	    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tpinfo.attr);
	    if (ret) {
		printk(KERN_ERR "[elan]%s: sysfs create tpinfo  failed\n", __func__);
		return ret;
		}

	/*********************************************************/
	return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
	//sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	/*******2013.10.17  add***********/
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_diff.attr);
	/********************************/
	kobject_del(android_touch_kobj);
}	

/*---------------------------  sys end ----------------------------*/ 

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	int rc = 0;
	int value=0;
	//struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;
	
	
        printk( "[elan] %s begin\n", __func__);

	elan_pdata = client->dev.platform_data;

	if ((elan_pdata == NULL)) {
            printk( "[elan] %s platform_data is NULL\n", __func__);
	    return -1;
	}

	printk("reset pin=%d, intr pin=%d\n", elan_pdata->reset, elan_pdata->intr_gpio);
 
        rc = gpio_request(elan_pdata->reset,"reset");

        if(rc < 0){
                pr_err("Error request gpio%s\n","reset");
                return -1;
        }

	rc = gpio_direction_output(elan_pdata->reset, 1);

	if (err)
	{
		printk(KERN_ERR "Failed to set reset direction, error=%d\n", err);
		gpio_free(elan_pdata->reset);
	}

	   /*init INTERRUPT pin*/
	err = gpio_request(elan_pdata->intr_gpio, "touch_intr");
	if(err < 0)
	{
		dev_err(&client->dev,"Failed to request GPIO%d ,error=%d\n",elan_pdata->intr_gpio,err);	
	}

	err = gpio_direction_input(elan_pdata->intr_gpio);

	if (err)
	{
		dev_err(&client->dev,"Failed to set interrupt direction, error=%d\n",err);		
		gpio_free(elan_pdata->intr_gpio);
	}

	intel_scu_ipc_msic_vprog2(1);
	
	msleep(10);//1500
	
	gpio_set_value(elan_pdata->reset, 1);
	msleep(10);// 40
	gpio_set_value(elan_pdata->reset, 0);
	msleep(10);// 40
	gpio_set_value(elan_pdata->reset, 1);
	msleep(10);//  1000
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "[elan] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->intr_gpio = elan_pdata->intr_gpio;
        ts->reset = elan_pdata->reset;

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}
	
	client->irq=gpio_to_irq(elan_pdata->intr_gpio);
        this_client = client;
	
	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	fw_err = elan_ktf2k_ts_setup(client);

	if (fw_err < 0) {
		printk(KERN_INFO "No Elan chip inside\n");
//		fw_err = -ENODEV;  
                goto no_elan_chip;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";   // for andorid2.2 Froyo  

	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev,ABS_MT_POSITION_X, 0, LCM_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev,ABS_MT_POSITION_Y, 0, LCM_Y_MAX, 0, 0);
	input_set_abs_params(ts->input_dev,ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
	input_set_abs_params(ts->input_dev,ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	
	__set_bit(KEY_HOME, ts->input_dev->keybit);
	__set_bit(KEY_MENU, ts->input_dev->keybit);
	__set_bit(KEY_BACK, ts->input_dev->keybit);
	__set_bit(KEY_SEARCH, ts->input_dev->keybit);

        // james: check input_register_devices & input_set_abs_params sequence
	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

#if 0	
	err = request_irq(gpio_to_irq(elan_pdata->intr_gpio),
		              elan_ktf2k_ts_irq_handler, 
		              IRQF_TRIGGER_FALLING,
		              "elan_touch",
		              ts);
#endif 

	if (err < 0) 
	{
		dev_err(&client->dev, "elan_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

#if 0
	if (gpio_get_value(ts->intr_gpio) == 0) {
		printk(KERN_INFO "[elan]%s: handle missed interrupt\n", __func__);
		elan_ktf2k_ts_irq_handler(client->irq, ts);
	}
#endif

// james: check earlysuspend
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = elan_ktf2k_ts_early_suspend;
	ts->early_suspend.resume = elan_ktf2k_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	private_ts = ts;

	err = elan_ktf2k_touch_sysfs_init();
	if (err < 0) {
		printk("%s: create sysfs failed: %d", __func__, err);
	}

	dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
		ts->input_dev->name);

// Firmware Update
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&ts->firmware) < 0)
  		printk("[ELAN]misc_register failed!\n!");
  	else
		printk("[ELAN]misc_register finished!!\n");
// End Firmware Update

	int retval;
	init_waitqueue_head(&esd_wait_queue);
	esd_thread= kthread_run(touch_esd_handler, 0, "elan_tpd");
	if(IS_ERR(esd_thread))
	{
		retval = PTR_ERR(esd_thread);
		printk( "[elan]  failed to create kernel esd_thread: %d\n", retval);
	}

	elan_ktf2k_ts_register_interrupt(ts->client);

	printk( "[elan] %s end\n", __func__);

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed: 
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
exit_irq_request_failed:
	free_irq(client->irq, ts);
err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
no_elan_chip:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	i2c_set_clientdata(client, NULL);
	gpio_free(elan_pdata->reset);
	gpio_free(elan_pdata->intr_gpio);
	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;

	#ifdef ELAN_POR
	tp_suspend_flag = 1;
	#endif

		if(is_inupgrade == 1)
	{
		printk("[elan] tpd_suspend TP IN UPGRADE,WE CAN NOT SLEEP\n");
		return 0;
	}

	if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{

		disable_irq(client->irq);

		rc = cancel_work_sync(&ts->work);
		if (rc)
			enable_irq(client->irq);

		rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
	}
	return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;

	int rc = 0, retry = 5;

		if(is_inupgrade == 1)
	{
		printk("[elan] tpd_resume TP IN UPGRADE,WE DO NOT NEED RESUME\n");
		return 0;
	}
	
	if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
	{
	 gpio_set_value(elan_pdata->reset, 1);
         msleep(20);
         gpio_set_value(elan_pdata->reset, 0);
      	  msleep(20);
         gpio_set_value(elan_pdata->reset, 1);
		 
	     #ifdef ELAN_POR
              tp_suspend_flag = 0;
             #endif 
		enable_irq(client->irq);
	}
       if(num_li){
                 input_report_key(idev, BTN_TOUCH, 0);
                 //input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
         	 input_mt_sync(idev);
         	 input_sync(idev);
       	//printk("[elan_debug] %s,num_li=%d\n",__func__,num_li);
       }
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf2k_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
         printk( "[elan] %s :  power_lock==%d\n", __func__,power_lock);
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void elan_ktf2k_ts_late_resume(struct early_suspend *h)
{
	struct elan_ktf2k_ts_data *ts;
       printk( "[elan] %s :  power_lock==%d\n", __func__,power_lock);
	ts = container_of(h, struct elan_ktf2k_ts_data, early_suspend);
	elan_ktf2k_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, elan_ktf2k_ts_id);
static struct i2c_driver ektf2k_ts_driver = {
	.probe		= elan_ktf2k_ts_probe,
	.remove		= elan_ktf2k_ts_remove,
	//.remove		= __devexit_p(elan_ktf2k_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf2k_ts_suspend,
	.resume		= elan_ktf2k_ts_resume,
#endif
	.id_table	= elan_ktf2k_ts_id,
	.driver		= {
		.name = ELAN_KTF2K_NAME,
	},
};

//static int __devinit elan_ktf2k_ts_init(void)
static int __init elan_ktf2k_ts_init(void)
{
if(intel_mid_get_board_id() & HW_BOARD_7_LTE)
	return i2c_add_driver(&ektf2k_ts_driver);
else
	return 0;
}

static void __exit elan_ktf2k_ts_exit(void)
{
	i2c_del_driver(&ektf2k_ts_driver);
	return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
