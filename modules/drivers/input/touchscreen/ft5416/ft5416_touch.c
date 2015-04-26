/* 
 * drivers/input/touchscreen/ft5416_touch.c
 *
 * FocalTech ft5416 TouchScreen driver. 
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
 *	VERSION		DATE			AUTHOR			NOTE
 *	1.0		2010-01-05		WenFS		only support mulititouch
  *	2.0		2012-03-22		David.Wang	add factory mode 
 * 
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5416_touch.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
//#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/module.h>
#include <asm/intel_scu_ipcutil.h>

#define __FT5416_FT__

static struct i2c_client *this_client;
static struct ft5416_touch_platform_data *ft5416_touch_pdata;
unsigned char fts_ctpm_get_upg_ver(void);
static u8 ft5416_tp_type_id; //add for save tp id by liukai

extern void *ft5416_platform_data(void *info) __attribute__((weak));

#define CONFIG_FT5416_MULTITOUCH  1
#define    FTS_PACKET_LENGTH        128

#define CFG_SUPPORT_AUTO_UPG 1

#if CFG_SUPPORT_AUTO_UPG
static unsigned char CTPM_FW_0X51[]=
{
	""
	//#include "BYD_LAVA_ver0x12_app.i"
};

static unsigned char CTPM_FW_0X53[]=
{
""
	//#include "BYD_LAVA_ver0x12_app.i"
};
static unsigned char CTPM_FW_0X79[]=
{
	""
	//#include "BYD_LAVA_ver0x12_app.i" };
};
#endif

#define DBUG(x) x
#define DBUG2(x) //x
#define DBUG3(x) //x //about reset suspend resume
#define DBUG4(x) //x //about update fw

struct ts_event {
    u16 au16_x[MAX_TOUCH_POINTS];              //x coordinate
    u16 au16_y[MAX_TOUCH_POINTS];              //y coordinate
    u8  au8_touch_event[MAX_TOUCH_POINTS];     //touch event:  0 -- down; 1-- contact; 2 -- contact
    u8  au8_finger_id[MAX_TOUCH_POINTS];       //touch ID
	u16	pressure;
    u8  touch_point;
};

struct ft5416_touch_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
	struct mutex 		device_mode_mutex;   /* Ensures that only one function can specify the Device Mode at a time. */
};
//register address
#define FT5416_REG_FW_VER 0xA6

static void ft5416_chip_reset(void)
{

	/* FT5416 wants 40mSec minimum after reset to get organized */
	gpio_set_value(ft5416_touch_pdata->reset, 1);
	msleep(40);
	gpio_set_value(ft5416_touch_pdata->reset, 0);
	printk("%s,0x%x\n", __func__, gpio_get_value(ft5416_touch_pdata->reset));
	msleep(40);
	gpio_set_value(ft5416_touch_pdata->reset, 1);
	printk("%s,0x%x\n", __func__, gpio_get_value(ft5416_touch_pdata->reset));
	msleep(40);

}

static int ft5416_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};
   //  pr_err("touch msg %s i2c read error: %d\n", __func__, this_client->addr);
	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("touch msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5416_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	//msleep(1);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5416_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5416_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}
    
	return 0;
}

static int ft5416_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

	msleep(50);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}
/*get the tp's id*/
static void ft5416_tp_id(void)
{
	if(ft5416_read_reg(0xa8, &ft5416_tp_type_id) < 0)
	{
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	}
	else 
	{
		if (ft5416_tp_type_id == 0xA8)
		{
			printk("[%s]: Read TP MODULE ID error!\n", __func__);
		}
		else
		{
			printk("[%s]: TP MODULE ID is 0x%02X !\n", __func__, ft5416_tp_type_id);
		}
	}
}

/*read the version of firmware*/
static unsigned char ft5416_read_fw_ver(void)
{
	unsigned char ver;
	ft5416_read_reg(FT5416_REG_FIRMID, &ver);
	return(ver);
}

typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL			0x0
#define FTS_TRUE			0x01
#define FTS_FALSE			0x0

#define I2C_CTPM_ADDRESS	0xFF //no reference!


void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			udelay(1);
		}
	}
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
    
	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret<=0)
	{
		DBUG(printk("[TSP]i2c_read_interface error\n");)
		return FTS_FALSE;
	}
  
	return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret<=0)
	{
		DBUG(printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);)
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/
/* update firmware partition*/

//#define    FTS_PACKET_LENGTH        128

/*firmware update interface*/
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_DWRD i = 0;

	FTS_DWRD  packet_number;
	FTS_DWRD  j;
	FTS_DWRD  temp;
	FTS_DWRD  lenght;
	FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_BYTE bt_ecc;
	int      i_ret;

	/*********Step 1:Reset  CTPM *****/
	/*write 0xaa to register 0xfc*/
	/*chip internal reset*/
	ft5416_write_reg(0xfc,0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5416_write_reg(0xfc,0x55);
	printk("[TSP] Step 1: Reset CTPM test\n");
   
	delay_qt_ms(30);   


	/*********Step 2:Enter upgrade mode *****/
	/*switch to bootloader*/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do
	{
		i ++;
		i_ret = ft5416_i2c_txdata(auc_i2c_write_buf, 2);
		delay_qt_ms(5);
	}while(i_ret <= 0 && i < 5 );

	/*********Step 3:check READ-ID***********************/
	/*read chip id*/        
	cmd_write(0x90,0x00,0x00,0x00,4);
	byte_read(reg_val,2);
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		
		printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
		return ERR_READID;
	}

	/*********Step 4:erase app*******************************/
	cmd_write(0x61,0x00,0x00,0x00,1);
   
	delay_qt_ms(1500);
	printk("[TSP] Step 4: erase. \n");

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	printk("[TSP] Step 5: start upgrade. \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}
        
		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i=0;i<temp;i++)
		{
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);    
		delay_qt_ms(20);
	}

	//send the last six byte
	for (i = 0; i<6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp =1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);  
		delay_qt_ms(20);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);

	return ERR_OK;
}

/*  touch panel calibration  */
int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i ;

	printk("[FTS] start auto CLB.\n");
	msleep(200);
	ft5416_write_reg(0, 0x40);  
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5416_write_reg(2, 0x4);  //write command to start calibration
	delay_qt_ms(300);
	for(i=0;i<100;i++)
	{
		ft5416_read_reg(0, &uc_temp);
		if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
		{
			break;
		}
		delay_qt_ms(200);
		printk("[FTS] waiting calibration %d\n",i);
        
	}
	printk("[FTS] calibration OK.\n");
    
	msleep(300);
	ft5416_write_reg(0, 0x40);  //goto factory mode
	delay_qt_ms(100);   //make sure already enter factory mode
	ft5416_write_reg(2, 0x5);  //store CLB result
	delay_qt_ms(300);
	ft5416_write_reg(0, 0x0); //return to normal mode 
	msleep(300);
	printk("[FTS] store CLB result OK.\n");
	return 0;
}


/***********************************************
	read the version of current firmware
***********************************************/
static ssize_t ft5416_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ft5416_touch_data *data = NULL;
	DBUG4(printk("[%s]: Enter!\n", __func__);)
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5416_touch_data *) i2c_get_clientdata( client );
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;
	mutex_lock(&data->device_mode_mutex);
	if(ft5416_read_reg(FT5416_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "0x%02X\n", fwver);
	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5416_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	/* place holder for future use */
	return -EPERM;
}
/***********************************
  set or read the report rate 
************************************/
static ssize_t ft5416_tprwreg_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5416_tprwreg_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ft5416_touch_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5416_touch_data *) i2c_get_clientdata( client );
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0;
	u8 regaddr=0x88, regvalue=0xff;
	u8 valbuf[5];
	
	DBUG(printk("[%s]: Enter!\n", __func__);)
	
	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&data->device_mode_mutex);
	
	num_read_chars = count - 1;
	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}
	
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval)
    	{
        	pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
        	goto error_return;
    	}
	DBUG4(printk("[%s]:valbuf=%s wmreg=%x\n", __func__, valbuf, wmreg);)
	
	if(2 == num_read_chars)
	{
		/*read the register at regaddr, report rate*/
		regaddr = wmreg;
		if(ft5416_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ft5416_write_reg(regaddr, regvalue) < 0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

/*get the size of firmware to update */
static int ft5416_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize = 0; 
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
	DBUG(printk("[%s]: Enter!\n", __func__);)
	
	sprintf(filepath, "/mnt/sdcard/%s", firmware_name);
	
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	filp_close(pfile, NULL);
	DBUG4(printk("[%s]: fsize = %d!\n", __func__, fsize);)
	return fsize;
}
/*read the new firmware*/
static int ft5416_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
	loff_t pos;
	mm_segment_t old_fs;
	
	DBUG(printk("[%s]: Enter!\n", __func__);)
	
	sprintf(filepath, "/mnt/sdcard/%s", firmware_name);
	//pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		printk("[%s]: Can not open the app!\n", __func__);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	DBUG4(printk("[%s]: fsize = %d!\n", __func__, fsize);)
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/*the interface to update firmware with app.bin*/
int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret; 
	u8 fwver;
	DBUG(printk("[%s]: Enter!\n", __func__);)
	int fwsize = ft5416_GetFirmwareSize(firmware_name);
	if(fwsize <= 0)
	{
		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
	}
	//=========FW upgrade========================*/
	pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5416_ReadFirmware(firmware_name, pbt_buf))
	{
		pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -1;
	}
   	/*call the upgrade function*/
   	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
   	if (i_ret != 0)
   	{
		pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
		//error handling ...
		//TBD
   	}
	else
   	{
		pr_info("[FTS] upgrade successfully.\n");
		if(ft5416_read_reg(FT5416_REG_FW_VER, &fwver) >= 0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
		fts_ctpm_auto_clb();  //start auto CLB
	}
	kfree(pbt_buf);
	return i_ret;
}

static ssize_t ft5416_fwupgradeapp_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5416_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, 
					size_t count)
{
	struct ft5416_touch_data *data = NULL;
	DBUG(printk("[%s]: Enter!\n", __func__);)
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5416_touch_data *) i2c_get_clientdata( client );
	ssize_t num_read_chars = 0;
	char fwname[128];
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	DBUG4(printk("[%s]: fwname is %s\n", __func__, fwname);)
	fwname[count-1] = '\0';

	mutex_lock(&data->device_mode_mutex);

	disable_irq(client->irq);
	num_read_chars = fts_ctpm_fw_upgrade_with_app_file(fwname);
	if(num_read_chars < 0)
		goto error_return;

	enable_irq(client->irq);

error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}


#if CFG_SUPPORT_AUTO_UPG
static int ft5416_fwupgrade_with_ifile(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   FTS_DWRD dw_lenth = 0;
   int i_ret;
    
	//=========FW upgrade========================*/
	if (ft5416_tp_type_id == 0x51)
	{
		pbt_buf = CTPM_FW_0X51;
		dw_lenth =sizeof(CTPM_FW_0X51);
	}
	else if (ft5416_tp_type_id == 0x53)
	{
		pbt_buf = CTPM_FW_0X53;
		dw_lenth =sizeof(CTPM_FW_0X51);
	}
	else if (ft5416_tp_type_id == 0x79)
	{
		pbt_buf = CTPM_FW_0X79;
		dw_lenth =sizeof(CTPM_FW_0X79);
	}
	else
	{
		printk(KERN_ERR"ft5416_fwupgrade_ifile(),not support this tp.\n");
		return 1;
	}
	
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(dw_lenth));
	if (i_ret != 0)
	{
		printk(KERN_ERR"ft5416_fwupgrade_ifile() = %d.\n", i_ret);
		//error handling ...
		//TBD
	}
	else
	{
		printk(KERN_ERR"ft5416_fwupgrade_ifile()upgrade successfully.\n");
		fts_ctpm_auto_clb();  //start auto CLB
	}

	return i_ret;
}

unsigned char ft5416_get_ifile_ver(void)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	FTS_DWRD dw_lenth = 0;
	
	if (ft5416_tp_type_id == 0x51)
	{
		pbt_buf = CTPM_FW_0X51;
		dw_lenth =sizeof(CTPM_FW_0X51);
	}
	else if (ft5416_tp_type_id == 0x53)
	{
		pbt_buf = CTPM_FW_0X53;
		dw_lenth =sizeof(CTPM_FW_0X51);
	}
	else if (ft5416_tp_type_id == 0x79)
	{
		pbt_buf = CTPM_FW_0X79;
		dw_lenth =sizeof(CTPM_FW_0X79);
	}
	else
	{
		printk(KERN_ERR"ft5416_get_ifile_ver(),not support this tp.\n");
		return 0xff;
	}	
	
    if (dw_lenth > 2)
    {
        return pbt_buf[dw_lenth - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
	
}

int ft5416_fwupgrate_ifile(void)
{
    unsigned char old_ver;
    unsigned char new_ver;
    int           i_ret;

    old_ver = ft5416_read_fw_ver();
    new_ver = ft5416_get_ifile_ver();

	printk(KERN_ERR"ft5416_fwupgrate_ifile():old_ver = 0x%x, new_ver = 0x%x\n",
		old_ver, 
		new_ver);
	
	if (new_ver == 0xFF)
	{
		return 0;
	}
	
	//if (old_ver < new_ver)  
	{
		msleep(100);

		i_ret = ft5416_fwupgrade_with_ifile();   

		if (i_ret == 0)
		{
			msleep(300);
			new_ver = ft5416_read_fw_ver();
			printk(KERN_ERR"ft5416_fwupgrate_ifilenew_ver = 0x%x\n", new_ver);
			return 1;
		}
		else
		{
			printk(KERN_ERR"[FTS] upgrade failed ret=%d.\n", i_ret);
		}
	}

    return 0;
}

static ssize_t ft5416_fwupgradeifile_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	int  irt = 0;
	struct ft5416_touch_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5416_touch_data *) i2c_get_clientdata( client );	

	disable_irq(client->irq);
	irt = ft5416_fwupgrate_ifile();
	enable_irq(client->irq);		
	return -EPERM;
}

static ssize_t ft5416_fwupgradeifile_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, 
					size_t count)
{
	int  irt = 0;
	struct ft5416_touch_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5416_touch_data *) i2c_get_clientdata( client );	

	disable_irq(client->irq);
	irt = ft5416_fwupgrate_ifile();
	enable_irq(client->irq);
	return count;		
}

#endif

/*************************
release touch panel	
**************************/
static void ft5416_touch_release(void)
{
	struct ft5416_touch_data *data = i2c_get_clientdata(this_client);
	input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0); 
	input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
}
/*read the data transfered by i2c*/
static int ft5416_read_data(void)
{
	struct ft5416_touch_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;
	
	ret = ft5416_i2c_rxdata(buf, CFG_POINT_READ_BUF);
    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07; 

    if (event->touch_point > MAX_TOUCH_POINTS)
    {
        event->touch_point = MAX_TOUCH_POINTS;
    }

    for (i = 0; i < event->touch_point; i++)
    {
        event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
        event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
        event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
        event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
    }

    event->pressure = 200;

    return 0;
}

static void ft5416_report_value(void)
{
	struct ft5416_touch_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;

	for (i  = 0; i < event->touch_point; i++)
	{
		if ((event->au16_x[i] <= SCREEN_MAX_X)
			&& (event->au16_y[i] <= SCREEN_MAX_Y))
		{
			/* LCD view area. */
			input_report_abs(data->input_dev,
				ABS_MT_POSITION_X,
				event->au16_x[i]);
		#if 0	
			if ((ft5416_tp_type_id == 0x79)&&(event->au16_y[i] >= SCREEN_MAX_Y -1 ))
			{
			input_report_abs(data->input_dev,
				ABS_MT_POSITION_Y,
				event->au16_y[i] + 10);
			   
			   DBUG(printk("id[%d] = %d, x[%d] = %d, y[%d] = %d \n", i, event->au8_finger_id[i], i, event->au16_x[i], i, event->au16_y[i]);)
			}
			else
		#endif	
			{
			input_report_abs(data->input_dev,
				ABS_MT_POSITION_Y,
				event->au16_y[i]);
			}
			
			input_report_abs(data->input_dev,
				ABS_MT_WIDTH_MAJOR,
				1);
			input_report_abs(data->input_dev,
				ABS_MT_TRACKING_ID,
				event->au8_finger_id[i]);
			if ((event->au8_touch_event[i]== 0)
				|| (event->au8_touch_event[i] == 2))
			{
				input_report_abs(data->input_dev,
					ABS_MT_PRESSURE,
					event->pressure);
			}
			else
			{
				input_report_abs(data->input_dev,
					ABS_MT_PRESSURE,
					0);
			}
			input_report_key(data->input_dev, BTN_TOUCH, 1); 
			DBUG(printk("id[%d] = %d, x[%d] = %d, y[%d] = %d \n", i, event->au8_finger_id[i], i, event->au16_x[i], i, event->au16_y[i]);)
		}
		else //maybe the touch key area
		{
#if CFG_SUPPORT_TOUCH_KEY
			if (event->au16_y[i] >= SCREEN_VIRTUAL_Y)
			{
				ft5x06_touch_key_process(data->input_dev,
					event->au16_x[i],
					event->au16_y[i],
					event->au8_touch_event[i]);
			}
#endif
		}	    
		input_mt_sync(data->input_dev);
	}
	
	input_sync(data->input_dev);

	if (event->touch_point == 0)
		ft5416_touch_release();
}

static void ft5416_touch_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	if (ft5416_read_data()== 0)	
	{	
		ft5416_report_value();
	}
	else
	{
		printk("data package read error\n");
	}
 	
 	//enable_irq(this_client->irq);
}

static irqreturn_t ft5416_touch_interrupt(int irq, void *dev_id)
{
	struct ft5416_touch_data *ft5416_touch = dev_id;
	DBUG(printk("[%s]: Enter!\n",__func__);)

	printk("%s\n", __func__);

	if (!work_pending(&ft5416_touch->pen_event_work)) {
		queue_work(ft5416_touch->ts_workqueue, &ft5416_touch->pen_event_work);
	}

	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND

static void ft5416_touch_suspend(struct early_suspend *handler)
{
	struct ft5416_touch_data *ts;
	ts =  container_of(handler, struct ft5416_touch_data, early_suspend);
	//struct ts_event *event = &ts->event;
	printk("[%s]: Enter!\n", __func__);
	disable_irq(this_client->irq);
	cancel_work_sync(&ts->pen_event_work);
	flush_workqueue(ts->ts_workqueue);
	#if 0
	/*added by liukai if find the touch point when suspend release it*/
	if( event->touch_point != 0) 
	{
		printk("[%s]: 1139 event->touch_point==%d\n", __func__,event->touch_point);
		printk("[%s]:Find the touchpoint when ft5416 suspend!\n", __func__);
		event->touch_point = 0;
		ft5416_touch_release();
	}
	#endif
	/*add end*/
	/* ==switch to deep sleep mode ==*/ 
   	ft5416_write_reg(FT5416_REG_PMODE, PMODE_HIBERNATE);
	printk("[%s]: Exit!\n", __func__);
}

static void ft5416_touch_resume(struct early_suspend *handler)
{
	printk("[%s]: Enter!\n", __func__);
	struct ft5416_touch_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;


	if( event->touch_point != 0) 
	{
		printk("[%s]:Find the touchpoint when ft5416 resume!\n", __func__);
		event->touch_point = 0;
		ft5416_touch_release();
	}
		/*reset touch*/
	ft5416_chip_reset();
	msleep(200);
	enable_irq(this_client->irq);
	printk("[%s]: exist!\n", __func__);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

/* sysfs */
/*read the version of firmware*/
static DEVICE_ATTR(fwversion, S_IRUGO|S_IWUSR, ft5416_tpfwver_show, ft5416_tpfwver_store);
/*read and set the report rate*/
static DEVICE_ATTR(fwrate, S_IRUGO|S_IWUSR, ft5416_tprwreg_show, ft5416_tprwreg_store);
/*upgrade the tp firmware with app.bin*/ 
static DEVICE_ATTR(fwupdate, S_IRUGO|S_IWUSR, ft5416_fwupgradeapp_show, ft5416_fwupgradeapp_store);

#if CFG_SUPPORT_AUTO_UPG
static DEVICE_ATTR(fwupdateifile, S_IRUGO|S_IWUSR, ft5416_fwupgradeifile_show, ft5416_fwupgradeifile_store);
#endif
static struct attribute *ft5416_attributes[] = {
	&dev_attr_fwversion.attr,
	&dev_attr_fwrate.attr,
	&dev_attr_fwupdate.attr,
#if CFG_SUPPORT_AUTO_UPG	
	&dev_attr_fwupdateifile.attr,
#endif	
	NULL
};

static struct attribute_group ft5416_attribute_group = {
	.attrs = ft5416_attributes
};

#ifdef __FT5416_FT__


/*------------------------------------------------------------------------
*For FT5416 factory Test
-------------------------------------------------------------------------*/
	
//#define FTS_PACKET_LENGTH        128
#define FTS_SETTING_BUF_LEN        128
	
#define FTS_TX_MAX				40
#define FTS_RX_MAX				40
#define FTS_DEVICE_MODE_REG	0x00
#define FTS_TXNUM_REG			0x03
#define FTS_RXNUM_REG			0x04
#define FTS_RAW_READ_REG		0x01
#define FTS_RAW_BEGIN_REG		0x10
#define FTS_VOLTAGE_REG		0x05
	
#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

/*open & short param*/
#define VERYSMALL_TX_RX	1
#define SMALL_TX_RX			2
#define NORMAL_TX_RX		0
#define RAWDATA_BEYOND_VALUE		10000
#define DIFFERDATA_ABS_OPEN		10
#define DIFFERDATA_ABS_ABNORMAL	100
#define RAWDATA_SMALL_VALUE		5500 /*cross short*/
static u16 g_min_rawdata = 7000;
static u16 g_max_rawdata = 9500;
static u16 g_min_diffdata = 50;
static u16 g_max_diffdata = 550;
static u8 g_voltage_level = 2;	/*default*/

int ft5416_ft_i2c_Read(char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = this_client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = this_client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(this_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&this_client->dev, "f%s: i2c read error.\n",__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = this_client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(this_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&this_client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5416_ft_i2c_Write(char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&this_client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

int ft5416_ft_write_reg(u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft5416_ft_i2c_Write(buf, sizeof(buf));
}


int ft5416_ft_read_reg(u8 regaddr, u8 *regvalue)
{
	return ft5416_ft_i2c_Read(&regaddr, 1, regvalue, 1);
}
static ssize_t ft5416_vendor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u8 id_type = 0;
	u8 fwver = 0x0;
	/*fw version*/
	if(ft5416_read_reg(FT5416_REG_FW_VER, &fwver) < 0)
	{
		num_read_chars = sprintf(buf, "%s","get tp fw version fail!\n");
	}
	else
	{
		num_read_chars = sprintf(buf, "FW Version:0x%02X\n", fwver);
	}

	/*chip id*/
	if(ft5416_read_reg(0xa8, &id_type) < 0)
	{
		num_read_chars += sprintf(&buf[num_read_chars], "%s","get chip id fail!\n");;
	}
	else 
	{
		num_read_chars += sprintf(&buf[num_read_chars], "Chip ID = 0x%02X\n", id_type);	
	}	

	return num_read_chars;
}

static ssize_t ft5416_vendor_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft5416_ft_tprwreg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft5416_ft_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 regaddr = 0xff, regvalue = 0xff;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	//mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&this_client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = wmreg;
		if (ft5416_ft_read_reg(regaddr, &regvalue) < 0)
			dev_err(&this_client->dev, "Could not read the register(0x%02x)\n",
						regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n",
					regaddr, regvalue);
	} else {
		regaddr = wmreg >> 8;
		regvalue = wmreg;
		if (ft5416_ft_write_reg(regaddr, regvalue) < 0)
			dev_err(&this_client->dev, "Could not write the register(0x%02x)\n",
							regaddr);
		else
			dev_err(&this_client->dev, "Write 0x%02x into register(0x%02x) successful\n",
							regvalue, regaddr);
	}

error_return:
	//mutex_unlock(&g_device_mutex);

	return count;
}

static int ft5416_ft_read_rawdata(u16 rawdata[][FTS_RX_MAX],
			u8 tx, u8 rx)
{
	u8 i = 0, j = 0, k = 0;
	int err = 0;
	u8 regvalue = 0x00;
	u8 regaddr = 0x00;
	u16 dataval = 0x0000;
	u8 writebuf[2] = {0};
	u8 read_buffer[FTS_RX_MAX * 2];
	/*scan*/
	err = ft5416_ft_read_reg(FTS_DEVICE_MODE_REG, &regvalue);
	if (err < 0) {
		return err;
	} else {
		regvalue |= 0x80;
		err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, regvalue);
		if (err < 0) {
			return err;
		} else {
			for(i=0; i<20; i++)
			{
				msleep(8);
				err = ft5416_ft_read_reg(FTS_DEVICE_MODE_REG, 
							&regvalue);
				if (err < 0) {
					return err;
				} else {
					if (0 == (regvalue >> 7))
						break;
				}
			}
		}
	}

	/*get rawdata*/
	dev_dbg(&this_client->dev, "%s() - Reading raw data...\n", __func__);
	for(i=0; i<tx; i++)
	{
		memset(read_buffer, 0x00, (FTS_RX_MAX * 2));
		writebuf[0] = FTS_RAW_READ_REG;
		writebuf[1] = i;
		err = ft5416_ft_i2c_Write(writebuf, 2);
		if (err < 0) {
			return err;
		}
		/* Read the data for this row */
		regaddr = FTS_RAW_BEGIN_REG;
		err = ft5416_ft_i2c_Read( &regaddr, 1, read_buffer, rx*2);
		if (err < 0) {
			return err;
		}
		k = 0;
		for (j = 0; j < rx*2; j += 2)
        	{
			dataval  = read_buffer[j];
			dataval  = (dataval << 8);
			dataval |= read_buffer[j+1];
			rawdata[i][k] = dataval;
			k++;
        	}
	}

	return 0;
}
/*raw data show*/
static ssize_t ft5416_ft_rawdata_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u16 before_rawdata[FTS_TX_MAX][FTS_RX_MAX];
	u8 rx = 0, tx = 0;
	u8 i = 0, j = 0;
	int err = 0;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	//mutex_lock(&g_device_mutex);
	/*entry factory*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_FACTORYMODE_VALUE);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:rawdata show error!\n", __func__);
		goto RAW_ERROR;
	}

	/*get rx and tx num*/
	err = ft5416_ft_read_reg(FTS_TXNUM_REG, &tx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get tx error!\n", __func__);
		goto RAW_ERROR;
	}
	err = ft5416_ft_read_reg(FTS_RXNUM_REG, &rx);
	if (err < 0)
	{
		num_read_chars = sprintf(buf,
			"%s:get rx error!\n", __func__);
		goto RAW_ERROR;
	}
	num_read_chars += sprintf(&(buf[num_read_chars]),"FT5416 tp channel: %u * %u\n", tx, rx);
	num_read_chars += sprintf(&(buf[num_read_chars]),"Reference data:\n");

	/*get rawdata*/
	err = ft5416_ft_read_rawdata(before_rawdata, tx, rx);
	if (err < 0) {
		num_read_chars = sprintf(buf,
				"%s:rawdata show error!\n", __func__);
		goto RAW_ERROR;
	} else {
		for (i=0; i<tx; i++) {
			for (j=0; j<rx; j++) {
				num_read_chars += sprintf(&(buf[num_read_chars]),
						"%5d"/*"%u "*/, before_rawdata[i][j]);
			}
			//buf[num_read_chars-1] = '\n';
			num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
		}
	}
RAW_ERROR:
	/*enter work mode*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_WORKMODE_VALUE);
	if (err < 0)
		dev_err(&this_client->dev,"%s:enter work error!\n", __func__);
	msleep(100);
	//mutex_unlock(&g_device_mutex);
	return num_read_chars;
}
static ssize_t ft5416_ft_rawdata_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

/*diff data show. default voltage level is 2.*/
static ssize_t ft5416_ft_diffdata_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u16 before_rawdata[FTS_TX_MAX][FTS_RX_MAX];
	u16 after_rawdata[FTS_TX_MAX][FTS_RX_MAX];
	u8 orig_vol = 0x00;
	u8 rx = 0, tx = 0;
	u8 i = 0, j = 0;
	int err = 0;
	u8 regvalue = 0x00;
	u16 tmpdata = 0x0000;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	//mutex_lock(&g_device_mutex);
	/*entry factory*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_FACTORYMODE_VALUE);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:rawdata show error!\n", __func__);
		goto RAW_ERROR;
	}

	/*get rx and tx num*/
	err = ft5416_ft_read_reg(FTS_TXNUM_REG, &tx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get tx error!\n", __func__);
		goto RAW_ERROR;
	}
	err = ft5416_ft_read_reg(FTS_RXNUM_REG, &rx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get rx error!\n", __func__);
		goto RAW_ERROR;
	}
	//num_read_chars += sprintf(&(buf[num_read_chars]), "tp channel: %u * %u\n", tx, rx);
	num_read_chars += sprintf(&(buf[num_read_chars]),"Delta data:\n");
	/*get rawdata*/
	err = ft5416_ft_read_rawdata(before_rawdata, tx, rx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:diffdata show error!\n", __func__);
		goto RAW_ERROR;
	} 

	/*get original voltage and change it to get new frame rawdata*/
	err = ft5416_ft_read_reg(FTS_VOLTAGE_REG, &regvalue);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get original voltage error!\n", __func__);
		goto RAW_ERROR;
	} 
	else 
	{
		orig_vol = regvalue;
	}
	
	if (orig_vol <= 1)
	{
    		regvalue = orig_vol + 2;
	}
	else 
	{			
		if(orig_vol >= 4)
		{
			regvalue = 1;
		}
		else
		{
			regvalue = orig_vol - 2;
		}
    }
	if (regvalue > 7)
		regvalue = 7;
	if (regvalue <= 0)
		regvalue = 0;
	#if 0	
	num_read_chars += sprintf(&(buf[num_read_chars]),
		"original voltage: %u changed voltage:%u\n",
		orig_vol, regvalue);
	#endif
	err = ft5416_ft_write_reg(FTS_VOLTAGE_REG, regvalue);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:set original voltage error!\n", __func__);
		goto RAW_ERROR;
	}
	
	/*get rawdata*/
	for (i=0; i<3; i++)
		err = ft5416_ft_read_rawdata(after_rawdata, tx, rx);
	if (err < 0) {
		num_read_chars = sprintf(buf,"%s:diffdata show error!\n", __func__);
		goto RETURN_ORIG_VOLTAGE;
	} else {
		for (i=0; i<tx; i++) {
			for (j=0; j<rx; j++) {
				if (after_rawdata[i][j] > before_rawdata[i][j])
					tmpdata = after_rawdata[i][j] - before_rawdata[i][j];
				else
					tmpdata = before_rawdata[i][j] - after_rawdata[i][j];
				num_read_chars += sprintf(&(buf[num_read_chars]),
						"%5d", tmpdata);
			}
			num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
			//buf[num_read_chars-1] = '\n';
		}
	}
	
RETURN_ORIG_VOLTAGE:
	err = ft5416_ft_write_reg(FTS_VOLTAGE_REG, orig_vol);
	if (err < 0)
		dev_err(&this_client->dev,
			"%s:return original voltage error!\n", __func__);
RAW_ERROR:
	/*enter work mode*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_WORKMODE_VALUE);
	if (err < 0)
		dev_err(&this_client->dev,"%s:enter work error!\n", __func__);
	msleep(100);
	//mutex_unlock(&g_device_mutex);
	
	return num_read_chars;
}

static ssize_t ft5416_ft_diffdata_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}


static ssize_t ft5416_ft_diag_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = ft5416_ft_rawdata_show(dev,attr,buf);
	return (num_read_chars);
}
static ssize_t ft5416_ft_diff_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = ft5416_ft_diffdata_show(dev,attr,buf);
	return (num_read_chars);
}

static ssize_t ft5416_ft_diag_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static void ft5416_ft_OnAnalyseRXOpenShort(u8 tx, u8 rx,
			u16 diffdata[][FTS_RX_MAX], u16 rawdata[][FTS_RX_MAX],
			char *buf, ssize_t *pnum_read_chars)
{
	u16 i = 0, j = 0, k = 0;
	u16 abnormalRX[FTS_RX_MAX];
	int sumrx = 0, avgrx = 0;
	u16 sumabnormal = 0;
	u16 index = 0;
	int sumrawbeyond = 0;
	int sumsmallraw = 0;
	int sumbeyond = 0;
	int iadd = 1;
	int smallindex = 0, smallrawvalue = RAWDATA_BEYOND_VALUE;
	for (i = 0; i < rx; i++) {
		sumrx = 0;
		for (j=0; j<tx; j++)
		{
			sumrx += diffdata[j][i];
		}
		avgrx = sumrx/tx;
		if(avgrx <= DIFFERDATA_ABS_OPEN)
			abnormalRX[i] = VERYSMALL_TX_RX;
		else if(avgrx <= DIFFERDATA_ABS_ABNORMAL)
			abnormalRX[i] = SMALL_TX_RX;
		else
			abnormalRX[i] = NORMAL_TX_RX;
	}
	for (i = 0; i < rx;) {
		sumabnormal = 0;
		if (NORMAL_TX_RX == abnormalRX[i])
			i++;
		else {
			
			if(i == (rx-1))
			{
				index = 0;
			}
			else
				index = i+1;
			for (j = index; j < rx; j++) {
				if (abnormalRX[j] > 0)
					sumabnormal++;
				else
					break;
			}
			
			if (VERYSMALL_TX_RX == abnormalRX[i])
			{				
				if(1 == sumabnormal)
				{
					for (j = 0; j < tx; j++) {
						if(diffdata[j][i] < DIFFERDATA_ABS_OPEN)
							sumbeyond++;
					}
					if (sumbeyond > (tx*4/5)) {
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							"short circuit: Rx%d & GND\n", (i + 1));
						i += 2;
					}	
					else
						i++;
				}
				else if (sumabnormal > 1) {
					for (k=0; k<sumabnormal; k++) {
						if (abnormalRX[i] != VERYSMALL_TX_RX)
							break;
						for (j=0; j<tx; j++) {
    							if (rawdata[j][i] >= RAWDATA_BEYOND_VALUE)
    								sumbeyond++;
    						}
    						if (sumbeyond > (rx*4/5)) {
							*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
								"Open circuit: RX%d\n", (i + 1));
    						}	
    						i++;
					}
				}
				else	
				{
					sumbeyond = 0;
					for(j=0; j<tx; j++)
					{
						if(rawdata[j][i] >= RAWDATA_BEYOND_VALUE)
							sumbeyond++;
						if (rawdata[j][i] <= RAWDATA_SMALL_VALUE) {
							sumsmallraw++;
							if (rawdata[j][i] < smallrawvalue) {
								smallindex = j;
								smallrawvalue = rawdata[j][i];
							}
						}
					}
					if (sumbeyond > (tx*4/5))
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
								"Open circuit: RX%d\n", (i + 1));
					if (sumsmallraw > 0) {
						//ÅÐ¶Ïcross short
						sumbeyond = 0;
						for (j=0; j<rx; j++) {
							if(diffdata[smallindex][j] < DIFFERDATA_ABS_OPEN)
								sumbeyond++;
						}
						if (sumbeyond > (rx*4/5))
							*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
								"cross short circuit: Rx%d & Tx%d\n", (i + 1),
								(smallindex+1));
					}
					i++;
				}					
			}
			else
			{
				for(j=i+1; j<(i+1+sumabnormal); j++) {
					
					for(k=0; k<tx; k++) {
						if (diffdata[k][j] > DIFFERDATA_ABS_OPEN)
							sumbeyond++;
						if (rawdata[k][j] > RAWDATA_BEYOND_VALUE)
							sumrawbeyond++;
					}
					if (sumbeyond > (tx*4/5))
						iadd++;
					else {
						if (sumrawbeyond > (tx*4/5))
							break;
						else
							iadd++;
					}
				}
				if (iadd > 1) {
					*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
						"Short circuit:RX%d", (i+1));
					for(j=0; j < (iadd-1); j++)
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							 " & RX%d", (i + 2 + j));
					buf[*pnum_read_chars-1] ='\n';
				}
				i += iadd;
			}
		}			
	}

}

static  void ft5416_ft_OnAnalyseTXOpenShort(u8 tx, u8 rx,
			u16 diffdata[][FTS_RX_MAX],
			u16 rawdata[][FTS_RX_MAX],
			char *buf, ssize_t *pnum_read_chars)
{
	u16 i = 0, j = 0, k = 0;
	u16 abnormalTX[FTS_TX_MAX];
	int sumtx = 0, avgtx = 0;
	u16 sumabnormal = 0;
	u16 index = 0;
	int sumrawbeyond = 0;
	int sumbeyond = 0;
	int iadd = 1;

	for (i=0; i < tx; i++) {
		sumtx = 0;
		for (j=0; j<rx; j++)
			sumtx += diffdata[i][j];
		avgtx = sumtx/rx;
		if(avgtx <= DIFFERDATA_ABS_OPEN)
			abnormalTX[i] = VERYSMALL_TX_RX;
		else if(avgtx <= DIFFERDATA_ABS_ABNORMAL)
			abnormalTX[i] = SMALL_TX_RX;
		else
			abnormalTX[i] = NORMAL_TX_RX;
	}
	for (i=0; i<tx; ) {
		sumabnormal = 0;
		if (NORMAL_TX_RX == abnormalTX[i])
			i++;
		else
		{
			if (i==(tx-1))
				index = 0;
			else
				index = i+1;
			for (j=index; j<tx; j++) {
				if(abnormalTX[j] > 0)
					sumabnormal++;
				else
					break;
			}
			
			if (VERYSMALL_TX_RX == abnormalTX[i]) {				
				if(1 == sumabnormal) {
					for (j=0; j < rx; j++) {
						if (diffdata[index][j] > DIFFERDATA_ABS_OPEN)
							sumbeyond++;
					}
					if (sumbeyond > (rx*4/5)) {
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							 "short circuit: Tx%d & GND\n", (i + 2 + j));
						i += 2;
					}	
					else
						i++;
				}
				else if (sumabnormal > 1) {
					for (k=0; k<sumabnormal; k++) {
						for(j=0; j<rx; j++) {
    							if(rawdata[i][j] >= RAWDATA_BEYOND_VALUE)
    								sumbeyond++;
    						}
    						if (sumbeyond > (rx*4/5))
							*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							 	"Open circuit: Tx%d\n", (i + 1));
    						i++;
					}
				}
				else {
					for(j=0; j<rx; j++)
					{
						if(rawdata[i][j] >= RAWDATA_BEYOND_VALUE)
							sumbeyond++;
					}
					if(sumbeyond > (rx*4/5))
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							 	"Open circuit: Tx%d\n", (i + 1));
					i++;
				}					
			}
			else
			{
				for (j=i+1; j < (i+1+sumabnormal); j++) {
					for(k=0; k<rx; k++) {
						if(diffdata[j][k] > DIFFERDATA_ABS_OPEN)
							sumbeyond++;
						if(rawdata[j][k] > RAWDATA_BEYOND_VALUE)
							sumrawbeyond++;
					}
					if(sumbeyond > (rx*4/5))					
						break;
					else {
						if(sumrawbeyond > (rx*4/5))
							break;
						else
							iadd++;
					}
				}
				if (iadd > 1) {
					*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
						"Short circuit:TX%d", (i+1));
					for(j=0; j < (iadd-1); j++)
						*pnum_read_chars += sprintf(&(buf[*pnum_read_chars]),
							 " & TX%d", (i + 2 + j));
					buf[*pnum_read_chars-1] = '\n';
				}
				i += iadd;
			}
		}			
	}
}
/*open short show*/

/*open short show*/
static ssize_t ft5416_ft_openshort_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	u16 rawdata[FTS_TX_MAX][FTS_RX_MAX];
	u16 diffdata[FTS_TX_MAX][FTS_RX_MAX];
	u8 orig_vol = 0x00;
	u8 rx = 0, tx = 0;
	u8 i = 0, j = 0;
	int err = 0;
	u8 regvalue = 0x00;
	u8 bpass = 1;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	//mutex_lock(&g_device_mutex);
	/*entry factory*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_FACTORYMODE_VALUE);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:rawdata show error!\n", __func__);
		goto RAW_ERROR;
	}

	/*get rx and tx num*/
	err = ft5416_ft_read_reg(FTS_TXNUM_REG, &tx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get tx error!\n", __func__);
		goto RAW_ERROR;
	}
	err = ft5416_ft_read_reg(FTS_RXNUM_REG, &rx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get rx error!\n", __func__);
		goto RAW_ERROR;
	}
	num_read_chars += sprintf(&(buf[num_read_chars]),"tp channel: %u * %u\n", tx, rx);
	/*get rawdata*/
	err = ft5416_ft_read_rawdata(rawdata, tx, rx);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:diffdata show error!\n", __func__);
		goto RAW_ERROR;
	} 

	/*get original voltage and change it to get new frame rawdata*/
	err = ft5416_ft_read_reg(FTS_VOLTAGE_REG, &regvalue);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,"%s:get original voltage error!\n", __func__);
		goto RAW_ERROR;
	} else 
		orig_vol = regvalue;

	if (g_voltage_level <= 2) {
		if (orig_vol <= 1)
	    		regvalue = orig_vol + g_voltage_level;
	    	else {			
	    		if(orig_vol >= 4)
	    			regvalue = 1;
	    		else
	    			regvalue = orig_vol - g_voltage_level;
	    	}
	} else if (3 == g_voltage_level) {
		if (orig_vol <= 2)
			regvalue = orig_vol + g_voltage_level;
		else
			regvalue = orig_vol - g_voltage_level;
	} else {
		if(orig_vol <= 3)
			regvalue = orig_vol + g_voltage_level;
		else
			regvalue = orig_vol - g_voltage_level;
	}
    	if (regvalue > 7)
    		regvalue = 7;
    	if (regvalue <= 0)
    		regvalue = 0;
		
	num_read_chars += sprintf(&(buf[num_read_chars]), 
			"original voltage: %u changed voltage:%u\n",
			orig_vol, regvalue);
	err = ft5416_ft_write_reg(FTS_VOLTAGE_REG, regvalue);
	if (err < 0) 
	{
		num_read_chars = sprintf(buf,
				"%s:set original voltage error!\n", __func__);
		goto RAW_ERROR;
	}
	
	/*get rawdata*/
	for (i=0; i<3; i++)
		err = ft5416_ft_read_rawdata(diffdata, tx, rx);
	if (err < 0) {
		num_read_chars = sprintf(buf,"%s:diffdata show error!\n", __func__);
		goto RETURN_ORIG_VOLTAGE;
	} else {
		for (i=0; i<tx; i++) {
			for (j=0; j<rx; j++) {
				if (rawdata[i][j] > diffdata[i][j])
					diffdata[i][j] = rawdata[i][j] - diffdata[i][j];
				else
					diffdata[i][j] = diffdata[i][j] - rawdata[i][j];
				if (rawdata[i][j] > g_max_rawdata ||
					rawdata[i][j] < g_min_rawdata)
					bpass = 0;
				if (diffdata[i][j] > g_max_diffdata ||
					diffdata[i][j] < g_min_diffdata)
					bpass = 0;
			}
		}
	}

	if (0 == bpass) 
	{
		ft5416_ft_OnAnalyseRXOpenShort(tx, rx, diffdata, rawdata, buf, &num_read_chars);
		ft5416_ft_OnAnalyseTXOpenShort(tx, rx, diffdata, rawdata, buf, &num_read_chars);
	} else
		num_read_chars = sprintf(buf,
				"Good. None open and short.\n");
RETURN_ORIG_VOLTAGE:
	err = ft5416_ft_write_reg(FTS_VOLTAGE_REG, orig_vol);
	if (err < 0)
		dev_err(&this_client->dev,"%s:return original voltage error!\n", __func__);
RAW_ERROR:
	/*enter work mode*/
	err = ft5416_ft_write_reg(FTS_DEVICE_MODE_REG, FTS_WORKMODE_VALUE);
	if (err < 0)
		dev_err(&this_client->dev,"%s:enter work error!\n", __func__);
	msleep(100);
	//mutex_unlock(&g_device_mutex);
	
	return num_read_chars;
}

static ssize_t ft5416_ft_openshort_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft5416_ft_setrawrange_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;

	//mutex_lock(&g_device_mutex);
	num_read_chars = sprintf(buf,
				"min rawdata:%d max rawdata:%d\n",
				g_min_rawdata, g_max_rawdata);
	
	//mutex_unlock(&g_device_mutex);
	
	return num_read_chars;
}

static ssize_t ft5416_ft_setrawrange_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 valbuf[12] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	//mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 8) {
		dev_err(&this_client->dev, "%s:please input 1 character\n",
				__func__);
		goto error_return;
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&this_client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}
	g_min_rawdata = wmreg >> 16;
	g_max_rawdata = wmreg;
error_return:
	//mutex_unlock(&g_device_mutex);

	return count;
}
static ssize_t ft5416_ft_setdiffrange_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//mutex_lock(&g_device_mutex);
	num_read_chars = sprintf(buf,
				"min diffdata:%d max diffdata:%d\n",
				g_min_diffdata, g_max_diffdata);
	//mutex_unlock(&g_device_mutex);
	
	return num_read_chars;
}

static ssize_t ft5416_ft_setdiffrange_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 valbuf[12] = {0};

	memset(valbuf, 0, sizeof(valbuf));
//	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 8) {
		dev_err(&this_client->dev, "%s:please input 1 character\n",
				__func__);
		goto error_return;
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&this_client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}
	g_min_diffdata = wmreg >> 16;
	g_max_diffdata = wmreg;
error_return:
	//mutex_unlock(&g_device_mutex);

	return count;

}

static ssize_t ft5416_ft_setvoltagelevel_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t num_read_chars = 0;
	//mutex_lock(&g_device_mutex);
	num_read_chars = sprintf(buf, "voltage level:%d\n", g_voltage_level);
	//mutex_unlock(&g_device_mutex);
	
	return num_read_chars;
}

static ssize_t ft5416_ft_setvoltagelevel_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u8 valbuf[5] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	//mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 1) {
		dev_err(&this_client->dev, "%s:please input 1 character\n",
				__func__);
		goto error_return;
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval) {
		dev_err(&this_client->dev, "%s() - ERROR: Could not convert the "\
						"given input to a number." \
						"The given input was: \"%s\"\n",
						__func__, buf);
		goto error_return;
	}
	g_voltage_level = wmreg;
	
error_return:
	//mutex_unlock(&g_device_mutex);

	return count;
}

/*------------------------------------------------------------------------
*get the fw version
*example:cat version
-------------------------------------------------------------------------*/
//static DEVICE_ATTR(version, (S_IWUSR|S_IRUGO), ft5416_version_show,
//			ft5416_version_store);


/*------------------------------------------------------------------------
*get the id version
*example:cat id
-------------------------------------------------------------------------*/
static DEVICE_ATTR(vendor, (S_IWUSR|S_IRUGO), ft5416_vendor_show,
			ft5416_vendor_store);

/*------------------------------------------------------------------------
*read and write register
*read example: echo 88 > rwreg ---read register 0x88
*write example:echo 8807 > rwreg ---write 0x07 into register 0x88
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
-------------------------------------------------------------------------*/
static DEVICE_ATTR(rwreg, S_IRUGO | S_IWUSR, ft5416_ft_tprwreg_show,
			ft5416_ft_tprwreg_store);

/*------------------------------------------------------------------------
*show a frame rawdata
*example:example:cat rawdata
-------------------------------------------------------------------------*/
//static DEVICE_ATTR(rawdata, S_IRUGO | S_IWUSR, ft5416_ft_rawdata_show,
//			ft5416_ft_rawdata_store);

/*------------------------------------------------------------------------
*show a frame diffdata
*example:cat diffdata
-------------------------------------------------------------------------*/
//static DEVICE_ATTR(diffdata, S_IRUGO | S_IWUSR, ft5416_ft_diffdata_show,
//			ft5416_ft_diffdata_store);


/*------------------------------------------------------------------------
*show a frame diffdata
*example:cat diag
-------------------------------------------------------------------------*/
static DEVICE_ATTR(diag, S_IRUGO | S_IWUSR, ft5416_ft_diag_show,
			ft5416_ft_diag_store);

/*------------------------------------------------------------------------
*show a frame diffdata
*example:cat diff
-------------------------------------------------------------------------*/
static DEVICE_ATTR(diff, S_IRUGO | S_IWUSR, ft5416_ft_diff_show,
			NULL);
/*------------------------------------------------------------------------
*if tp is bad, then show where is opened or shorted.
*example:cat openshort
-------------------------------------------------------------------------*/
static DEVICE_ATTR(openshort, S_IRUGO | S_IWUSR, ft5416_ft_openshort_show,
			ft5416_ft_openshort_store);


/*------------------------------------------------------------------------
*set range of rawdata
*example:echo 60009999 > setraw(range is 6000-9999)
*		cat setraw
-------------------------------------------------------------------------*/
static DEVICE_ATTR(setraw, S_IRUGO | S_IWUSR, ft5416_ft_setrawrange_show,
			ft5416_ft_setrawrange_store);
/*------------------------------------------------------------------------
*set range of diffdata
*example:echo 00501000 > setdiffdata(range is 50-1000)
*		cat setdiffdata
-------------------------------------------------------------------------*/

static DEVICE_ATTR(setdiffdata, S_IRUGO | S_IWUSR, ft5416_ft_setdiffrange_show,
			ft5416_ft_setdiffrange_store);
/*------------------------------------------------------------------------
*set range of diffdata
*example:echo 2 > setvol
*		cat setvol
-------------------------------------------------------------------------*/
static DEVICE_ATTR(setvol, S_IRUGO | S_IWUSR,
			ft5416_ft_setvoltagelevel_show,
			ft5416_ft_setvoltagelevel_store);




static struct kobject *android_touch_ftobj;
static int ft5416_touch_sysfs_init(void)
{
	int ret;
	android_touch_ftobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_ftobj == NULL)
	{
		printk(KERN_ERR "TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
#if 0	
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_version.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file version failed\n");
		return ret;
	}
#endif
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_vendor.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file chip vendor failed\n");
		return ret;
	}
#if 0	
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_rawdata.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file rawdata failed\n");
		return ret;
	}

	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_diffdata.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file diffdata failed\n");
		return ret;
	}
#endif	
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_diag.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file diag failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_diff.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file diff failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_openshort.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file openshort failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_setraw.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file setraw failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_setdiffdata.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file setdiffdata failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_setvol.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file setvol failed\n");
		return ret;
	}
	
	ret = sysfs_create_file(android_touch_ftobj, &dev_attr_rwreg.attr);
	if (ret) 
	{
		printk(KERN_ERR "TOUCH_ERR: create_file rwreg failed\n");
		return ret;
	}	
	return 0;
}

static void ft5416_touch_sysfs_deinit(void)
{
	//sysfs_remove_file(android_touch_ftobj, &dev_attr_fwversion.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_vendor.attr);
	//sysfs_remove_file(android_touch_ftobj, &dev_attr_rawdata.attr);
	//sysfs_remove_file(android_touch_ftobj, &dev_attr_diffdata.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_openshort.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_setraw.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_setdiffdata.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_setvol.attr);
	sysfs_remove_file(android_touch_ftobj, &dev_attr_rwreg.attr);
	kobject_del(android_touch_ftobj);
}

/*-----------------------------------------------------------------------------------
* tp factory test end
-----------------------------------------------------------------------------------*/
#endif
/***********************************************************************************************
	add by liukai for virtualkeys 
***********************************************************************************************/
static struct kobject *properties_kobj;

static ssize_t ft5416_virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (ft5416_tp_type_id == 0x51)
	{
		return sprintf(buf, 
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":124:1000:50:50"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":286:1000:50:50"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":398:1000:50:50"
	"\n");
	}
	else if(ft5416_tp_type_id == 0x79)
	{
		return sprintf(buf, 
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":70:970:50:20"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":230:970:50:20"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":400:970:50:20"
	"\n");	
	}
	//start [Du Yichun, 20120912, for acer tp compatible]
	else if(ft5416_tp_type_id == 0x53) 
	{
		return sprintf(buf, 
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":90:1010:50:20"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":270:1010:50:20"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":450:1010:50:20"
	"\n");	
	}
	else
	{
		return sprintf(buf, "TP matching failed! type: %02x\n", ft5416_tp_type_id);
	}
	//end
}

static struct kobj_attribute ft5416_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5416",
		.mode = S_IRUGO,
	},
	.show = &ft5416_virtual_keys_show,
};

static struct attribute *ft5416_properties_attrs[] = {
	&ft5416_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group ft5416_properties_attr_group = {
	.attrs = ft5416_properties_attrs
};

extern struct kobject * intel_get_properties(void);


static int ft5416_virtual_keys_init(void) 
{
	
	int ret;
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (!properties_kobj) {
		pr_err("failed to create /sys/board_properties\n");
		ret = -EINVAL;
	}

	if (properties_kobj)
	{
		ret = sysfs_create_group(properties_kobj,&ft5416_properties_attr_group);
	}
	if (!properties_kobj || ret)
	{
		printk(KERN_ERR "failed to create board_properties\n");
	}
	return 0;
}
 

static int ft5416_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5416_touch_data *ft5416_touch;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value; 

	
	ft5416_touch_pdata = client->dev.platform_data;

	if (ft5416_touch_pdata == NULL) 
	{
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}

	printk("ft5416_touch_probe ENTER\n");
        
	printk("rest pin=%d, int pin=%d\n", ft5416_touch_pdata->reset, ft5416_touch_pdata->irq);
	
	/*init RESET pin*/
	err = gpio_request(ft5416_touch_pdata->reset, "ft5416_chip_reset");

	if (err < 0)
	{
		printk(KERN_ERR "Failed to request GPIO%d (ft5416_chip_reset) error=%d\n",
		ft5416_touch_pdata->reset, err);
	}

	err = gpio_direction_output(ft5416_touch_pdata->reset, 1);

	if (err)
	{
		printk(KERN_ERR "Failed to set reset direction, error=%d\n", err);
		gpio_free(ft5416_touch_pdata->reset);
		return err;
	}

	intel_scu_ipc_msic_vprog2(1);

	msleep(1500);

	ft5416_chip_reset();
     
 
	printk("touch_reset_pin ==%d\n", gpio_get_value(ft5416_touch_pdata->reset));
        
        /* Check if the I2C bus supports BYTE transfer */
	err = i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE);
	
	if (!err) 
	{
		dev_err(&client->dev,
			    "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		goto exit_check_functionality_failed;
	}
		
	ft5416_touch = kzalloc(sizeof(*ft5416_touch), GFP_KERNEL);
	
	if (!ft5416_touch)	
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	
	printk("client=0x%x, nr=%d, addr=0x%x\n", client, client->adapter->nr, client->addr);

	i2c_set_clientdata(client, ft5416_touch);

	mutex_init(&ft5416_touch->device_mode_mutex);
	
	INIT_WORK(&ft5416_touch->pen_event_work, ft5416_touch_pen_irq_work);

	ft5416_touch->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	
	if (!ft5416_touch->ts_workqueue) 
	{
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/*init INTERRUPT pin*/
	err = gpio_request(ft5416_touch_pdata->irq, "touch_int");

	if(err < 0)
	{
		dev_err(&client->dev,
			    "Failed to request GPIO%d ,error=%d\n",
			    ft5416_touch_pdata->irq,
			    err);	
	}

	err = gpio_direction_input(ft5416_touch_pdata->irq);

	if (err)
	{
		dev_err(&client->dev,
			    "Failed to set interrupt direction, error=%d\n",
			    err);		
		gpio_free(ft5416_touch_pdata->irq);
	}

	printk("touch_irq_pin ==%d\n", gpio_get_value(ft5416_touch_pdata->irq));

	err = request_irq(gpio_to_irq(ft5416_touch_pdata->irq),
		              ft5416_touch_interrupt, 
		              IRQF_TRIGGER_FALLING | IRQF_DISABLED,
		              "ft5416_touch",
		              ft5416_touch);

	if (err < 0) 
	{
		dev_err(&client->dev, "ft5416_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	input_dev = input_allocate_device();
	
	if (!input_dev) 
	{
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5416_touch->input_dev = input_dev;
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TRACKING_ID, 0, 5, 0, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_HOME, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);	

	msleep(20);
	/*get the tp module id*/
	ft5416_tp_id();
	
	//ft5416_fwupgrate_ifile();

	/*tp's device name*/
	input_dev->name	= "ft5416";		//dev_name(&client->dev)
	input_dev->phys = "ft5416";
	
	/* Initialize virtualkeys */
	//ft5416_virtual_keys_init();

	input_set_capability(input_dev, EV_KEY, KEY_PROG1);

	/*register the input device*/
	err = input_register_device(input_dev);
	if (err) 
	{
		dev_err(&client->dev,
		"ft5416_touch_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	//ft5416_touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5416_touch->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ft5416_touch->early_suspend.suspend = ft5416_touch_suspend;
	ft5416_touch->early_suspend.resume	= ft5416_touch_resume;
	register_early_suspend(&ft5416_touch->early_suspend);
#endif

	uc_reg_value = ft5416_read_fw_ver();
	printk("[%s]: Firmware version is 0x%x\n", __func__, uc_reg_value);
	
	ft5416_read_reg(FT5416_REG_PERIODACTIVE, &uc_reg_value);
	printk("[%s]: report rate is %dHz.\n", __func__, uc_reg_value * 10);

	if(uc_reg_value != 0xb) 
	{
		ft5416_write_reg(FT5416_REG_PERIODACTIVE, 0xb);
		ft5416_read_reg(FT5416_REG_PERIODACTIVE, &uc_reg_value);
		printk("[%s]: set report rate to %dHz.\n", __func__, uc_reg_value * 10);
	}
	
	ft5416_read_reg(FT5416_REG_THGROUP, &uc_reg_value);
	printk("[%s]: touch threshold is %d.\n", __func__, uc_reg_value * 4);
	
	enable_irq(client->irq);

	//create sysfs
	err = sysfs_create_group(&client->dev.kobj, &ft5416_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5416_attribute_group);
	}
	else
	{
		printk("[%s] - sysfs_create_group() succeeded.\n", __func__);
	}
#ifdef __FT5416_FT__	
	ft5416_touch_sysfs_init();
#endif
	DBUG(printk("[%s]: End of probe !\n", __func__);)
	printk("ft5416_touch_probe END\n");
	
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5416_touch);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&ft5416_touch->pen_event_work);
	destroy_workqueue(ft5416_touch->ts_workqueue);
exit_create_singlethread:
	DBUG(printk("[%s]: singlethread error !\n", __func__);)
	i2c_set_clientdata(client, NULL);
	kfree(ft5416_touch);
exit_alloc_data_failed:
exit_check_functionality_failed:
	gpio_free(ft5416_touch_pdata->reset);
	gpio_free(ft5416_touch_pdata->irq);

	return err;
}

/***********************************************************************************************
remove touch
***********************************************************************************************/
static int ft5416_touch_remove(struct i2c_client *client)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	struct ft5416_touch_data *ft5416_touch = i2c_get_clientdata(client);
#ifdef __FT5416_FT__		
	ft5416_touch_sysfs_deinit();
#endif	
	unregister_early_suspend(&ft5416_touch->early_suspend);
	free_irq(client->irq, ft5416_touch);
	input_unregister_device(ft5416_touch->input_dev);
	kfree(ft5416_touch);
	cancel_work_sync(&ft5416_touch->pen_event_work);
	destroy_workqueue(ft5416_touch->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5416_touch_id[] = {
	{ FT5416_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5416_touch_id);
/***********************************************************************************************
touch driver
***********************************************************************************************/
static struct i2c_driver ft5416_touch_driver = {
	.probe		= ft5416_touch_probe,
	.remove		= ft5416_touch_remove,
	.id_table	= ft5416_touch_id,
	.driver	= {
		.name	= FT5416_NAME,
		.owner	= THIS_MODULE,
	},
};


static int __init ft5416_platform_init(void)
{
    int i2c_busnum = 7;
    struct i2c_board_info i2c_info;
    void *pdata = NULL;

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy( i2c_info.type, FT5416_NAME , strlen(FT5416_NAME) );
    //i2c_info.irq = get_gpio_by_name("TOUCH_INT_N");
 
    i2c_info.addr = 0x38;
 
    pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    pdata = ft5416_platform_data(&i2c_info);


    if(pdata!=NULL)
        i2c_info.platform_data = pdata;
    else
        printk("%s, pdata is NULL\n", __func__);

    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
}

fs_initcall(ft5416_platform_init);


/***********************************************************************************************
initialize the touch
***********************************************************************************************/
static int __init ft5416_touch_init(void)
{
	int ret;
	DBUG(printk("[%s]: Enter!\n", __func__);)
	ret = i2c_add_driver(&ft5416_touch_driver);
	DBUG(printk("ret=%d\n",ret);)
	return ret;
}

/***********************************************************************************************
exit the touch
***********************************************************************************************/
static void __exit ft5416_touch_exit(void)
{
	DBUG(printk("[%s]: Enter!\n", __func__);)
	i2c_del_driver(&ft5416_touch_driver);
}

module_init(ft5416_touch_init);
module_exit(ft5416_touch_exit);



MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5416 TouchScreen driver");
MODULE_LICENSE("GPL");
