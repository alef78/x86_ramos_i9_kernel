/******************************************************************************
 * MODULE     : rohm_bh1721_driver.h
 * FUNCTION   : drivers of bh1721GLI of header
 * PROGRAMMED : SEI
 * DATE(ORG)  : FEB-10-2011(FEB-10-2011)
 * REMARKS    : 
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011-2015 ROHM CO.,LTD.
 * HISTORY    :
 * 1.00A Jun-30-2011  Tracy   Made a new file
 *****************************************************************************/
#ifndef _LINUX_I2C_ROHM_bh1721_DRIVER_H
#define _LINUX_I2C_ROHM_bh1721_DRIVER_H

#define BH1721_SLAVE_ADDR (0x23)
#define ROHM_I2C_NAME "bh1721als"

#define DRIVER_OK  (0)
#define DRIVER_NG  (-1)
#define MASK_SHORT (0xFFFF)
#define MASK_CHAR  (0x00FF)
#define MASK_7BIT  (0x007F)
#define MASK_6BIT  (0x003F)
#define MASK_5BIT  (0x001F)
#define MASK_4BIT  (0x000F)
#define MASK_3BIT  (0x0007)
#define MASK_2BIT  (0x0003)
#define MASK_1BIT  (0x0001)

/* BH1721 COMMAND */
#define CMD_TRANS_100_H     (0x49)
#define CMD_TRANS_100_L     (0x6C)//0110 1100,	transimission rate is 100%
#define CMD_H_RES_CONT      (0x10)//Auto-Res continuously

//REG_ALSCONTROL(0x80)
#define CMD_POWER_DOWN         (0)
#define CMD_POWER_UP           (1)

/* cmd definition of ioctl */
#define IOCTL_APP_SET_TIMER (0)

struct ROHM_I2C_platform_data {
	int (*power)(int on);	/* Only valid in first array entry */
	uint32_t flags;
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
};

#endif /* _LINUX_ROHM_I2C */

