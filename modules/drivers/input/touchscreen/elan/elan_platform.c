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

extern void *ekthf2k_platform_data(void *info);

static int __init elan_platform_init(void)
{
    int i2c_busnum = 7;
    struct i2c_board_info i2c_info;
    void *pdata = NULL;

    memset(&i2c_info, 0, sizeof(i2c_info));
    strncpy(i2c_info.type, ELAN_KTF2K_NAME, strlen(ELAN_KTF2K_NAME));

    i2c_info.addr = 0x10;

    pr_info("I2C bus = %d, name = %16.16s, irq = 0x%2x, addr = 0x%x\n",
                i2c_busnum,
                i2c_info.type,
                i2c_info.irq,
                i2c_info.addr);

    pdata = ekthf2k_platform_data(&i2c_info);

    if(pdata != NULL)
        i2c_info.platform_data = pdata;
    else
        printk("%s, pdata is NULL\n", __func__);

    return i2c_register_board_info(i2c_busnum, &i2c_info, 1);
}

fs_initcall(elan_platform_init);

