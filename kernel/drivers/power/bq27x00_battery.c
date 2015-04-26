/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27425-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/intel-mid.h>

#include <linux/power/bq27x00_battery.h>
#include <linux/power/bq24261_charger.h>

#include <linux/delay.h>
#define DRIVER_VERSION			"1.2.0"
#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26
#define BQ27x00_REG_NAC			0x0C /* Nominal available capacity */
#define BQ27x00_REG_LMD			0x12 /* Last measured discharge */
#define BQ27x00_REG_CYCT		0x2A /* Cycle count total */
#define BQ27x00_REG_AE			0x22 /* Available energy */
#define BQ27x00_POWER_AVG		0x24

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76 /* Initial last measured discharge */
#define BQ27000_FLAG_EDVF		BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_EDV1		BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_CI			BIT(4) /* Capacity Inaccurate flag */
#define BQ27000_FLAG_FC			BIT(5)
#define BQ27000_FLAG_CHGS		BIT(7) /* Charge state flag */

#define BQ27500_REG_SOC			0x2C
#define BQ27500_REG_DCAP		0x3C /* Design capacity */
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27500_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27500_FLAG_BATDET		BIT(3) /* Battery insertion detected */
#define BQ27500_FLAG_ITPOR		BIT(5) /* Indicates a Power On Reset or RESET subcommand has occurred */
#define BQ27500_FLAG_FC			BIT(9)
#define BQ27500_FLAG_OTC		BIT(15)

/* bq27425 register addresses are same as bq27x00 addresses minus 4 */
#define BQ27425_REG_OFFSET		0x04
#define BQ27425_REG_SOC			0x20 /* Register address plus offset */
#define BQ27425_REG_DCAP        0x40 /* Register address plus offset */

#define BQ27000_RS			20 /* Resistor sense */
#define BQ27x00_POWER_CONSTANT		(256 * 29200 / 1000)

#ifdef CONFIG_BATTERY_MODEL_WD3870127P
	#define BATTERY_MODEL "WD3870127P"
#else
	#ifdef CONFIG_BATTERY_MODEL_SL3770125
		#define BATTERY_MODEL "SL3770125"
	#else
		#error "BATTERY_MODEL IS NOT DEFINED." 
	#endif
#endif

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
};
static int bq27x00_reboot_callback(struct notifier_block *nfb,
					unsigned long event, void *data);
static struct notifier_block bq27x00_reboot_notifier_block = {
	.notifier_call = bq27x00_reboot_callback,
	.priority = 0,
};
static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, u8 value);
static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single);

enum bq27x00_chip { BQ27200, BQ27500, BQ27425};

#define VBATT_MAX 4360
#define VBATT_MIN 3400

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	enum bq27x00_chip	chip;

	struct bq27x00_reg_cache cache;
	int status;
	struct bq27x00_platform_data *pdata;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;
#if defined(DEBUG)	
	struct delayed_work dump_register_worker;
#endif
	struct power_supply	bat;

	struct bq27x00_access_methods bus;
	bool plat_rebooting;

	struct mutex lock;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property bq27425_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static unsigned int poll_interval = 360;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

static struct bq27x00_device_info *bq27x00_di;
static int probe_passed = 0;
/* Sysfs Entry to show fuel gague dffs list in software */
static ssize_t get_dffs_list(struct device *device,
					struct device_attribute *attr, char *buf);
static DEVICE_ATTR(dffs_list, S_IRUGO, get_dffs_list, NULL);


#if defined(DEBUG)
static char* register_dump_buffer;
#define DUMP_BUFFER_SIZE 1024
#endif


/*
 * Common code for BQ27x00 devices
 */

static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	if (di->chip == BQ27425)
		return di->bus.read(di, reg - BQ27425_REG_OFFSET, single);
	return di->bus.read(di, reg, single);
}

static inline int bq27x00_write(struct bq27x00_device_info *di, u8 reg, u8 value)
{
	if (di->chip == BQ27425)
		return bq27x00_write_i2c(di, reg - BQ27425_REG_OFFSET, value);
	else
		return bq27x00_write_i2c(di, reg, value);
}

/*
 * Higher versions of the chip like BQ27425 and BQ27500
 * differ from BQ27000 and BQ27200 in calculation of certain
 * parameters. Hence we need to check for the chip type.
 */
static bool bq27xxx_is_chip_version_higher(struct bq27x00_device_info *di)
{
	if (di->chip == BQ27425 || di->chip == BQ27500)
		return true;
	return false;
}

static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, u8 value)
{       
	struct i2c_client *client = to_i2c_client(di->dev);
	int ret;
	u8 buffer[2];

	if (!client->adapter){
		dev_err(di->dev, "client->adapter is null");
		return -ENODEV;
	}

	/* if the shutdown or reboot sequence started
	 * then block the access to maxim registers as chip
	 * cannot be recovered from broken i2c transactions
	 */
	if (di->plat_rebooting) {
		dev_warn(di->dev, "rebooting is in progress\n");
		return -EINVAL;
	}
	
	buffer[0] = reg;
	buffer[1] = value;

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.buf	= buffer,
		.len	= 2,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(di->dev, "Error write i2c, address = 0x%x, value = 0x%x. retcode = %d", reg, value, ret);
		return ret;
	}
	return 0;
}
static int bq27x00_write_i2c_rommode(struct bq27x00_device_info *di, u8 reg, u8 value)
{       
	struct i2c_client *client = to_i2c_client(di->dev);
	int ret;
	u8 buffer[2];

	if (!client->adapter){
		dev_err(di->dev, "client->adapter is null");
		return -ENODEV;
	}

	/* if the shutdown or reboot sequence started
	 * then block the access to maxim registers as chip
	 * cannot be recovered from broken i2c transactions
	 */
	if (di->plat_rebooting) {
		dev_warn(di->dev, "rebooting is in progress\n");
		return -EINVAL;
	}
	
	buffer[0] = reg;
	buffer[1] = value;

	struct i2c_msg msg = {
		.addr	= 0x0B,
		.flags	= 0,
		.buf	= buffer,
		.len	= 2,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(di->dev, "Error write i2c in rom mode, address = 0x%x, value = 0x%x\n. retcode = %d", reg, value, ret);
		return ret;
	}
	return 0;
}
static int bq27x00_read_i2c_rommode(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	/* if the shutdown or reboot sequence started
	 * then block the access to maxim registers as chip
	 * cannot be recovered from broken i2c transactions
	 */
	if (di->plat_rebooting) {
		dev_warn(di->dev, "rebooting is in progress\n");
		return -EINVAL;
	}
	
	msg[0].addr = 0x0B;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = 0x0B;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0){
		dev_err(di->dev, "Error read i2c in rom mode, address = 0x%x, retcode: %d", reg, ret);
		return ret;
	}

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27425_init(struct bq27x00_device_info *di, int (*dffs)[10])
{
	int i = 0;
	int j = 0;
	int ret = 0;
	unsigned char address = 0;
        
	dev_info(di->dev, "%s is called.", __func__);
	for(i = 0; dffs[i][0] != 0; i++) {
		switch(dffs[i][0])
		{
			case 'W': 
				address = dffs[i][2];
				if(dffs[i][1]==0xaa){
					for(j = 3; dffs[i][j]!= -1; j++)
					{     			
						bq27x00_write_i2c(di, address, dffs[i][j]);
						address++;
					}
				}
				else{
					for(j = 3; dffs[i][j]!= -1; j++)
						{     			
							bq27x00_write_i2c_rommode(di, address, dffs[i][j]);
							address++;
						}
				}
				break;
			case 'R':
			    break;
			case 'C':
			    address = dffs[i][2];
			    for(j = 3; dffs[i][j]!= -1; j++)
					{     	
						if(bq27x00_read_i2c_rommode(di, address, 1)!=dffs[i][j])
							dev_warn(di->dev, "===test bq27x00_read error address= 0x%x, dffs[i][j]=0x%x, i=%d, j=%d\n", address, dffs[i][j],i,j);
						address++;
					}
				
				break;
			case 'X': 
				msleep(dffs[i][1]);
				break;
			default:
			   break;
		}
	}
	return 0;	

}	

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc;

	if (di == NULL) {
		printk(KERN_ERR "%s: NULL pointer(di) detected. ", __func__);
		return -1;
	}

	if (di->chip == BQ27500)
		rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	else if (di->chip == BQ27425)
		rsoc = bq27x00_read(di, BQ27425_REG_SOC, false);
	else
		rsoc = bq27x00_read(di, BQ27000_REG_RSOC, true);

	if (rsoc < 0)
		dev_err(di->dev, "error reading relative State-of-Charge\n");

	return rsoc;
}
/*
 * Update battery temperature from PMU/CHARGER to FG 
 * Or < 0 if something fails.
 */
static int bq27x00_battery_update_temperature(struct bq27x00_device_info *di)
{
    u8 address = BQ27x00_REG_TEMP;
    int battery_temp = 0;
    u8 temp1 = 0;
    u8 temp2 = 0;
    int rc;

	if (di->pdata->battery_pack_temp != NULL) {
		rc = di->pdata->battery_pack_temp(&battery_temp);
		if (rc < 0 ) {
			battery_temp = 25;
			dev_err(di->dev, "error reading temperature from charger.\n");
		}
		battery_temp = battery_temp * 10 + 2731;
		dev_info(di->dev,"battery_temp:%d.\n",battery_temp);
		temp1= battery_temp & 0x00FF;   
		temp2 = battery_temp >> 8;
		bq27x00_write(di, address, temp1);

		address++;
		bq27x00_write(di, address, temp2);	
	}
    return battery_temp;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_charge(struct bq27x00_device_info *di, u8 reg)
{
	int charge;

	charge = bq27x00_read(di, reg, false);
	if (charge < 0) {
		dev_err(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (bq27xxx_is_chip_version_higher(di))
		charge *= 1000;
	else
		charge = charge * 3570 / BQ27000_RS;

	return charge;
}

/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_nac(struct bq27x00_device_info *di)
{
	int flags;
	bool is_bq27500 = di->chip == BQ27500;
	bool is_higher = bq27xxx_is_chip_version_higher(di);

	flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !is_bq27500);
	if (flags >= 0 && !is_higher && (flags & BQ27000_FLAG_CI))
		return -ENODATA;

	return bq27x00_battery_read_charge(di, BQ27x00_REG_NAC);
}

/*
 * Return the battery Last measured discharge in µAh
 * Or < 0 if something fails.
 */
static inline int bq27x00_battery_read_lmd(struct bq27x00_device_info *di)
{
	return bq27x00_battery_read_charge(di, BQ27x00_REG_LMD);
}

/*
 * Return the battery Initial last measured discharge in µAh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_ilmd(struct bq27x00_device_info *di)
{
	int ilmd;

	if (di->chip == BQ27500)
		ilmd = bq27x00_read(di, BQ27500_REG_DCAP, false);
	else if (di->chip == BQ27425)
		ilmd = bq27x00_read(di, BQ27425_REG_DCAP, false);
	else
		ilmd = bq27x00_read(di, BQ27000_REG_ILMD, true);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading initial last measured discharge\n");
		return ilmd;
	}

	if (bq27xxx_is_chip_version_higher(di))
		ilmd *= 1000;
	else
		ilmd = ilmd * 256 * 3570 / BQ27000_RS;

	return ilmd;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_energy(struct bq27x00_device_info *di)
{
	int ae;

	ae = bq27x00_read(di, BQ27x00_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27500)
		ae *= 1000;
	else
		ae = ae * 29200 / BQ27000_RS;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_temperature(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27x00_battery_update_temperature(di);

	temp = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}
	dev_info(di->dev,"bat_temp:%d.\n",temp);

	if (bq27xxx_is_chip_version_higher(di))
		temp -= 2731;
	else
		temp = 5 * temp / 2;
	
	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_time(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_read_pwr_avg(struct bq27x00_device_info *di, u8 reg)
{
	int tval;

	tval = bq27x00_read(di, reg, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (di->chip == BQ27500)
		return tval;
	else
		return (tval * BQ27x00_POWER_CONSTANT) / BQ27000_RS;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_health(struct bq27x00_device_info *di)
{
	int tval;

	if (di->pdata->battery_health) {
		tval = di->pdata->battery_health();
	} else {
	tval = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

		if (bq27xxx_is_chip_version_higher(di)) {
		if (tval & BQ27500_FLAG_SOCF)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else if (tval & BQ27500_FLAG_OTC)
			tval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	} else {
		if (tval & BQ27000_FLAG_EDV1)
			tval = POWER_SUPPLY_HEALTH_DEAD;
		else
			tval = POWER_SUPPLY_HEALTH_GOOD;
		return tval;
	}
	}

	return tval;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di)
{
	int status = 0;
	if (di->pdata->battery_status) {
		status = di->pdata->battery_status();
		if (status < 0) {
			status = POWER_SUPPLY_STATUS_UNKNOWN;	
		} else if ((status == POWER_SUPPLY_STATUS_CHARGING) && 
					(di->cache.capacity == 100) ) {
			status = POWER_SUPPLY_STATUS_FULL;
		}
#if 1 
	} else {
		status = di->status;
	}
#else
	} else {
		if (bq27xxx_is_chip_version_higher(di)) {
			if (flags & BQ27500_FLAG_FC)
				status = POWER_SUPPLY_STATUS_FULL;
			else if (flags & BQ27500_FLAG_DSC)
				status = POWER_SUPPLY_STATUS_DISCHARGING;
			else 
				if (capacity == 100) 
					status = POWER_SUPPLY_STATUS_FULL;
				else
					status = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			if (flags & BQ27000_FLAG_FC)
				status = POWER_SUPPLY_STATUS_FULL;
			else if (flags & BQ27000_FLAG_CHGS)
				if  (capacity == 100) 
					status = POWER_SUPPLY_STATUS_FULL;
				else
					status = POWER_SUPPLY_STATUS_CHARGING;
			else if (power_supply_am_i_supplied(&di->bat))
				status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}
#endif
	return status;
}

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };
	bool is_bq27500;
	bool is_bq27425;

	is_bq27500 = di->chip == BQ27500;
	is_bq27425 = di->chip == BQ27425;

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !(is_bq27500 || is_bq27425));
	if (cache.flags >= 0) {
		if (!is_bq27500 && !is_bq27425
				&& (cache.flags & BQ27000_FLAG_CI)) {
			dev_info(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			cache.capacity = bq27x00_battery_read_rsoc(di);
			if (!is_bq27425) {
				cache.energy = bq27x00_battery_read_energy(di);
				cache.time_to_empty =
					bq27x00_battery_read_time(di,
							BQ27x00_REG_TTE);
				cache.time_to_empty_avg =
					bq27x00_battery_read_time(di,
							BQ27x00_REG_TTECP);
				cache.time_to_full =
					bq27x00_battery_read_time(di,
							BQ27x00_REG_TTF);
			}
			cache.charge_full = bq27x00_battery_read_lmd(di);
			cache.health = bq27x00_battery_health(di);
		cache.temperature = bq27x00_battery_read_temperature(di);
		if (!is_bq27425)
			cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.power_avg =
			bq27x00_battery_read_pwr_avg(di, BQ27x00_POWER_AVG);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27x00_battery_read_ilmd(di);
	}
	}

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0) {
		mutex_lock(&di->lock);
		di->cache = cache;
		di->status = bq27x00_battery_status(di);
		mutex_unlock(&di->lock);
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);

	if (di == NULL) {
		printk(KERN_ERR "bq27425: error in %s, for di is NULL.\n", __func__);
		return;
	}

	bq27x00_update(di);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}
#if defined(DEBUG)
static void bq27425_dump_all_register(struct bq27x00_device_info *di)
{
	u8 addr;
	u16 value;
	
	for (addr = 0x00; ; addr+=2)
	{
		value = bq27x00_read_i2c(di, addr, false);
		if (value < 0)
			dev_err(di->dev, "Error reading data from register 0x%x, ret: %d", addr, value);
		else 
			dev_info(di->dev, "address: 0x%02x%02x, value: 0x%04x", addr, addr+1, value);

		if(addr >= 0x20)
			break;
	}
}

static void bq27425_dump_register(struct bq27x00_device_info *di)
{
	u8 addr;
	u16 value;
	char *p;

	if (!register_dump_buffer)
		return;

	register_dump_buffer[0] = '\t';
	p = register_dump_buffer + 1;

	for (addr = 0x00; ; addr+=2)
	{
		value = bq27x00_read_i2c(di, addr, false);
		if (value < 0){
			dev_err(di->dev, "Error reading data from register 0x%02x%02x, ret: %d", addr, addr+1, value);
			sprintf(p, "0x%s\t", "ERR!");
		} else {
			//sprintf(p, "0x%04x\t", value);
			sprintf(p, "%d\t", value);
			p += strlen(p);
		}

		if(addr >= 0x20)
			break;
	}

	sprintf(p, "%s", "\n");
	dev_info(di->dev, register_dump_buffer);
}

static void bq27425_dump_register_worker(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, dump_register_worker.work);

	bq27425_dump_register(di);

	schedule_delayed_work(&di->dump_register_worker, 5*(HZ));
}
#endif


/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (bq27xxx_is_chip_version_higher(di)) {
		/* bq27500 returns signed value */
		val->intval = (int)((s16)curr) * 1000;
	} else {
		flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
		if (flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * 3570 / BQ27000_RS;
	}
	dev_info(di->dev, "%s:current:%d\n",__func__,val->intval);	
	return 0;
}

static int bq27x00_battery_capacity_level(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int level;

	if (bq27xxx_is_chip_version_higher(di)) {
		if (di->cache.flags & BQ27500_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27500_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27500_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27000_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27000_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27000_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
	union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;
	dev_info(di->dev, "%s:voltage:%d\n",__func__,val->intval);
	return 0;
}

static int bq27x00_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);
static int bq27x00_battery_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di;

	if (probe_passed == 0)
		return -ENODEV;

	di = to_bq27x00_device_info(psy);
	if (di == NULL) {
		printk(KERN_ERR "%s: NULL pointer(di) detected.\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&di->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		di->status = val->intval;
		dev_info(di->dev,"%s:battery_status:%d\n",__func__,di->status);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&di->lock);

	return ret;
}
static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di;

	if (probe_passed == 0)
		return -ENODEV;

	di = to_bq27x00_device_info(psy);
	if (di == NULL) {
		printk(KERN_ERR "%s: NULL pointer(di) detected.\n", __func__);
		return -ENODEV;
	}

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	if(time_is_before_jiffies(di->last_update + 2*HZ)){
		cancel_delayed_work_sync(&di->work);
		bq27x00_battery_poll(&di->work.work);
	}

	mutex_lock(&di->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->status;
		dev_info(di->dev,"%s:battery_status:%d\n",__func__,di->status);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = bq27x00_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = bq27x00_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27x00_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27x00_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->cache.temperature; 
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = di->pdata->technology;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27x00_simple_value(bq27x00_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27x00_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27x00_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27x00_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27x00_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27x00_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27x00_simple_value(di->cache.health, val);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (!strncmp(di->pdata->battid, "UNKNOWNB", 8))
			val->strval = di->pdata->battid;
		else
			val->strval = di->pdata->model_name;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (di->pdata->get_vmin_threshold == NULL)
		    val->intval = VBATT_MIN * 1000;
		else
		    val->intval = di->pdata->get_vmin_threshold();
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (di->pdata->get_vmax_threshold == NULL)
		    val->intval = VBATT_MAX * 1000;
		else
		    val->intval = di->pdata->get_vmax_threshold();
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = di->pdata->serial_num;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&di->lock);

	return ret;
}

static void bq27x00_external_power_changed(struct power_supply *psy)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;

	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	if (di->chip == BQ27425) {
		di->bat.properties = bq27425_battery_props;
		di->bat.num_properties = ARRAY_SIZE(bq27425_battery_props);
	} else {
		di->bat.properties = bq27x00_battery_props;
		di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	}
	di->bat.set_property = bq27x00_battery_set_property;
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = bq27x00_external_power_changed;

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	mutex_init(&di->lock);

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	/*
	 * power_supply_unregister call bq27x00_battery_get_property which
	 * call bq27x00_battery_poll.
	 * Make sure that bq27x00_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);
#if defined(DEBUG)
	if (register_dump_buffer) {
		cancel_delayed_work_sync(&di->dump_register_worker);
		kfree(register_dump_buffer);
	}
#endif
	power_supply_unregister(&di->bat);

	mutex_destroy(&di->lock);
}


/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27X00_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	/* if the shutdown or reboot sequence started
	 * then block the access to maxim registers as chip
	 * cannot be recovered from broken i2c transactions
	 */
	if (di->plat_rebooting) {
		dev_warn(di->dev, "rebooting is in progress\n");
		return -EINVAL;
	}
	
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static irqreturn_t bq27x00_battery_det_irq(int irq, void *_host)
{
    struct bq27x00_device_info *di = _host;
	dev_info(di->dev, "bq27x00_battery_det_irq soc=%d \n",di->cache.capacity);	
//	schedule_delayed_work(&di->work, 0);
	return IRQ_HANDLED;
}

static void bq27x00_battery_seal(struct bq27x00_device_info *di)
{
    int result;

	bq27x00_write_i2c(di, 0x00, 0x20);
    bq27x00_write_i2c(di, 0x01, 0x00);	
    msleep(20);
	result = bq27x00_read_i2c(di, 0x00, 0);
	dev_info(di->dev, "sealed FG, result: 0x%04x", result);
}

static int bq27x00_battery_unseal(struct bq27x00_device_info *di)
{
    int result;

    bq27x00_write_i2c(di, 0x00, 0x14);
    bq27x00_write_i2c(di, 0x01, 0x04);
    bq27x00_write_i2c(di, 0x00, 0x72);
    bq27x00_write_i2c(di, 0x01, 0x36);

    msleep(1000);
    bq27x00_write_i2c(di, 0x00, 0x00);
	bq27x00_write_i2c(di, 0x01, 0x00);
	result = bq27x00_read_i2c(di, 0x00, 0);
	dev_info(di->dev, "unsealed FG, result: 0x%04x", result);
	return !(result & 0x2000);
}

static int bq27x00_get_oemstr(struct bq27x00_device_info *di, u8 *oem_str)
{
	u8  addr, i;
	int result;
	int address;

    bq27x00_write_i2c(di, 0x00, 0x00);
	bq27x00_write_i2c(di, 0x01, 0x00);
	result = bq27x00_read_i2c(di, 0x00, 0);
	dev_info(di->dev, "%s--result: 0x%04x", __func__, result);
	if (result & 0x2000) {
		bq27x00_write_i2c(di, 0x3F, 0x01);
		address = bq27x00_read_i2c(di, 0x3F, 0);
		dev_info(di->dev, "%s--0x3F: 0x%04x", __func__, address);
	} else {
		bq27x00_write_i2c(di, 0x00, 0x13);
		address = bq27x00_read_i2c(di, 0x00, 0);
		dev_info(di->dev, "%s--0x00: 0x%04x", __func__, address);
		bq27x00_write_i2c(di, 0x3E, 0x3A);
		address = bq27x00_read_i2c(di, 0x3E, 0);
		dev_info(di->dev, "%s--0x3E: 0x%04x", __func__, address);	
		bq27x00_write_i2c(di, 0x3F, 0x00);
		address = bq27x00_read_i2c(di, 0x3F, 0);
		dev_info(di->dev, "%s--0x3F: 0x%04x", __func__, address);
	}
	msleep(20);

	addr = 0x40;
	for (i=0; i<8; i++) 
		oem_str[i] = bq27x00_read_i2c(di, addr+i, 1);
	oem_str[i] = 0;
	dev_info(di->dev, "got oem string: %s.\n", oem_str);
	
	return 0;
}

static int bq27x00_get_firmware_version(struct bq27x00_device_info *di, u16 *fw_ver )
{
	bq27x00_write_i2c(di, 0x00, 0x02);
	bq27x00_write_i2c(di, 0x01, 0x00);
	msleep(20);
	*fw_ver = bq27x00_read_i2c(di, 0x00, 0);
	dev_info(di->dev, "got firmware version: 0x%04x\n", *fw_ver);

	return 0;
}

static int bq27x00_insert_battery(struct bq27x00_device_info *di)
{
	int flags;
	bool is_bq27500;
	bool is_bq27425;
	bool is_changed = false;

	is_bq27500 = di->chip == BQ27500;
	is_bq27425 = di->chip == BQ27425;

	flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !(is_bq27500 || is_bq27425));
	if(flags & BQ27500_FLAG_BATDET)
	{
		dev_info(di->dev, "flags: 0x%04x, battery is inserted already.", flags);
	} else {
		bq27x00_write_i2c(di, 0x00, 0x0c);
		bq27x00_write_i2c(di, 0x01, 0x00);
		is_changed = true;
		msleep(20);
	}

	if(flags & BQ27500_FLAG_ITPOR)
	{
		dev_info(di->dev, "flags: 0x%04x, ITPOR flag isn't cleared.", flags);
		is_changed = true;
		msleep(20);
	} else {
		dev_info(di->dev, "flags: 0x%04x, ITPOR flag is cleared already.", flags);
	}

	if(is_changed)
	{
		flags = bq27x00_read(di, BQ27x00_REG_FLAGS, !(is_bq27500 || is_bq27425));
		dev_info(di->dev, "flags is changed to 0x%04x.", flags);
	}

	return 0;
}

static ssize_t get_dffs_list(struct device *device,
						struct device_attribute *attr, char *buf)
{
	bq27x00_di->pdata->get_dffs_list(buf);
	return strlen(buf);
}

bool bq27x00_recover_link(struct bq27x00_device_info *di)
{
	int    result;
	int    i;
	bool   ret = true;
/* No of times we should reset I2C lines */
#define NR_I2C_RESET_CNT	8
	
	result = bq27x00_write_i2c(di, 0x00, 0x00);
	if (result < 0) {
		dev_warn(di->dev, "reset i2c device:%d\n", result);
		for (i = 0; i < NR_I2C_RESET_CNT; i++) {
			di->pdata->reset_i2c_lines();
			result = bq27x00_write_i2c(di, 0x00, 0x00);
			if (result < 0) 
				dev_warn(di->dev,
						"reset i2c device:%d\n", result);
			else 
				break;
		}
		ret = (i<NR_I2C_RESET_CNT);
	}

	if (ret) {
		bq27x00_write_i2c(di, 0x01, 0x00);
		result = bq27x00_read_i2c(di, 0x00, 0);
		dev_info(di->dev, "Status code: %04x", result);
	}

	return ret;
}

//md: initialize the FG
static bool bq27441_init(struct bq27x00_device_info *di)
{
	bool ret = false;
	int err;
	int result;
	int flags;
	int descap_msb, descap_lsb;
	int deseng_msb, deseng_lsb;
	int tervol_msb, tervol_lsb;
	int tapcur_msb, tapcur_lsb;
	int dsgcur_msb,	dsgcur_lsb;
	int chgcur_msb, chgcur_lsb;
	int qutcur_msb, qutcur_lsb;
	int sub64_msb, sub64_lsb;
	u8 csum0, csum1, csum2;
	int csum00,csum11,csum22;
	int csum0_temp,csum1_temp,csum2_temp;
	result = bq27x00_read_i2c(di,0x06,0);
	if(!(result & 0x0020)){
		dev_info(di->dev, "the FG isn't in confige mode, Flags(): 0x%04x", result);
		return ret;
	}
	bq27x00_write_i2c(di, 0x00, 0x00);
	bq27x00_write_i2c(di, 0x01, 0x00);
	result = bq27x00_read_i2c(di, 0x00, 0);
	dev_info(di->dev, "%s--status: 0x%04x", __func__, result);
	if (result & 0x2000) {
		dev_info(di->dev, "FG be sealed, result: 0x%04x", result);
		//unseal the FG
		bq27x00_write_i2c(di, 0x00, 0x00);
		bq27x00_write_i2c(di, 0x01, 0x80);
		bq27x00_write_i2c(di, 0x00, 0x00);
		bq27x00_write_i2c(di, 0x01, 0x80);
		bq27x00_write_i2c(di, 0x00, 0x00);
		bq27x00_write_i2c(di, 0x01, 0x00);
		result = bq27x00_read_i2c(di, 0x00, 0);
		if (result & 0x2000) {
			dev_info(di->dev, "fail unsealed FG, result: 0x%04x", result);
			return	ret;
		}
		dev_info(di->dev, "have unsealed FG, result: 0x%04x", result);
		
	}
	dev_info(di->dev, "FG in unsealed.\n");
	
	// set the configue mode
	bq27x00_write_i2c(di, 0x00, 0x13);
	bq27x00_write_i2c(di, 0x01, 0x00);
	
	msleep(1000);
	flags = bq27x00_read_i2c(di, 0x06, 0);
	if(!(flags & 0x0010)){
		dev_info(di->dev, "FG haven't be in configue mode: 0x%04x", flags);
		return ret;
	}
	dev_info(di->dev, "FG be in configue mode and have been reset: 0x%04x", flags);
	
	//modify the subclass ID 82 parameters
	bq27x00_write_i2c(di, 0x61, 0x00);
	bq27x00_write_i2c(di, 0x3E, 0x52);
	bq27x00_write_i2c(di, 0x3F, 0x00);
	//read the subclass ID 82 defult checksum
	csum0 = bq27x00_read_i2c(di, 0x60, 1);	
	dev_info(di->dev, "the defult subclass ID 82 checksum is :0x%04x", csum0);
	
	//initialize the Design Capacity to 4550mAh
	descap_msb = bq27x00_read_i2c(di, 0x4A, 1);
	descap_lsb = bq27x00_read_i2c(di, 0x4B, 1);
	dev_info(di->dev, "the defult design capacity is :0x%02x,0x%02x.\n", descap_msb,descap_lsb);
	bq27x00_write_i2c(di, 0x4A, 0x11);
	bq27x00_write_i2c(di, 0x4B, 0xC6);
	
	//initialize the Design Energy
	deseng_msb = bq27x00_read_i2c(di, 0x4C, 1);
	deseng_lsb = bq27x00_read_i2c(di, 0x4D, 1);
	dev_info(di->dev, "the defult design engery is :0x%02x,0x%02x.\n", deseng_msb,deseng_lsb);
	bq27x00_write_i2c(di, 0x4C, 0x43);
	bq27x00_write_i2c(di, 0x4D, 0x8A);
	
	//initialize the Terminate Voltage	3400mV
	tervol_msb = bq27x00_read_i2c(di, 0x50, 1);
	tervol_lsb = bq27x00_read_i2c(di, 0x51, 1);
	dev_info(di->dev, "the defult Terminate Voltage is :0x%02x,0x%02x.\n", tervol_msb,tervol_lsb);
	bq27x00_write_i2c(di, 0x50, 0x0D);
	bq27x00_write_i2c(di, 0x51, 0x48);
	
	//initialize the Taper Rate 
	tapcur_msb = bq27x00_read_i2c(di, 0x5B, 1);
	tapcur_lsb = bq27x00_read_i2c(di, 0x5C, 1);
	dev_info(di->dev, "the defult Taper Rate is :0x%02x,0x%02x.\n", tapcur_msb,tapcur_lsb);
	bq27x00_write_i2c(di, 0x5B, 0x00);
	bq27x00_write_i2c(di, 0x5C, 0xE3);
	//write the subclass ID 82 checksum
	csum00 = csum0;
	csum0_temp = (255 - csum00 - descap_msb - descap_lsb - \
							deseng_msb - deseng_lsb - \
							tervol_msb - tervol_lsb - \
							tapcur_msb - tapcur_lsb ) % 256;
	if(csum0_temp < 0)
		csum0_temp += 256;
	csum0 = 255 - (csum0_temp + 17 + 198 + 67 + 138 + 13 + 72 + 227) % 256;
	dev_info(di->dev, "FG:BQ27441_82:%x.\n",csum0);
	err = bq27x00_write_i2c(di, 0x60, csum0);
	if(err){
		dev_info(di->dev, "error write the subclass ID 82.\n");
		return ret;
	}
	dev_info(di->dev, "have reset the subclass ID 82");
	
	
	//modify the subclass ID 81 parameters
	bq27x00_write_i2c(di, 0x61, 0x00);
	bq27x00_write_i2c(di, 0x3E, 0x51);
	bq27x00_write_i2c(di, 0x3F, 0x00);
	
	//read the subclass ID 81 checksum
	csum1 = bq27x00_read_i2c(di, 0x60, 1);	
	dev_info(di->dev, "the defult subclass ID 81 checksum is :0x%04x", csum1);
	
	//initialize Dsg Current Threshold
	dsgcur_msb = bq27x00_read_i2c(di, 0x40, 1);
	dsgcur_lsb = bq27x00_read_i2c(di, 0x41, 1);
	dev_info(di->dev, "the defult Dsg Current is :0x%02x,0x%02x", dsgcur_msb,dsgcur_lsb);
	bq27x00_write_i2c(di, 0x40, 0x02);
	bq27x00_write_i2c(di, 0x41, 0x58);
	
	//initialize Chg Current Threshold
	chgcur_msb = bq27x00_read_i2c(di, 0x42, 1);
	chgcur_lsb = bq27x00_read_i2c(di, 0x43, 1);
	dev_info(di->dev, "the defult Chg Current is :0x%02x,0x%02x", chgcur_msb,chgcur_lsb);
	bq27x00_write_i2c(di, 0x42, 0x02);
	bq27x00_write_i2c(di, 0x43, 0x58);

	//initialize Quit Current 
	qutcur_msb = bq27x00_read_i2c(di, 0x44, 1);
	qutcur_lsb = bq27x00_read_i2c(di, 0x45, 1);
	dev_info(di->dev, "the defult Quit Current is :0x%02x,0x%02x", qutcur_msb,qutcur_lsb);
	bq27x00_write_i2c(di, 0x44, 0x03);
	bq27x00_write_i2c(di, 0x45, 0x20);
	
	//write the subclass ID 81 checksum
	csum11 = csum1;
	csum1_temp = (255 - csum11 - dsgcur_msb - dsgcur_lsb - \
						chgcur_msb - chgcur_lsb - \
						qutcur_msb - qutcur_lsb) % 256;
	if(csum1_temp < 0)
		csum1_temp += 256;
	csum1 = 255 - (csum1_temp + 2 + 88 + 2 + 88 + 3 + 32)%256;
	dev_info(di->dev, "FG:BQ27441_81:%x.\n",csum1);
	err = bq27x00_write_i2c(di, 0x60, csum1);
	if(err){
		dev_info(di->dev, "error write the subclass ID 81.\n");
		return ret;
	}
	dev_info(di->dev, "have reset the subclass ID 81.\n");
	
	
	//modify the subclass ID 64 parameters
	bq27x00_write_i2c(di, 0x61, 0x00);
	bq27x00_write_i2c(di, 0x3E, 0x40);
	bq27x00_write_i2c(di, 0x3F, 0x00);
	//read the subclass ID 64 checksum
	csum2 = bq27x00_read_i2c(di, 0x60, 1);	
	dev_info(di->dev, "the defult subclass ID 64 checksum is :0x%04x", csum2);
	//set the external temperature source
	sub64_msb = bq27x00_read_i2c(di, 0x40, 1);
	sub64_lsb = bq27x00_read_i2c(di, 0x41, 1);
	dev_info(di->dev, "the defult subclass 64:0x%02x,0x%02x.\n",sub64_msb,sub64_lsb); 
	bq27x00_write_i2c(di, 0x40, 0x25);
	bq27x00_write_i2c(di, 0x41, 0xF9);
	//write the subclass ID 64 checksuma
	csum22 = csum2;
	csum2_temp = (255 - csum22 - sub64_msb - sub64_lsb)%256;
	if(csum2_temp < 0)
		csum2_temp += 256;
	csum2 = 255 - (csum2_temp + 37 + 249)%256;
	dev_info(di->dev, "FG:BQ27441_64:%x.\n",csum2);
	err = bq27x00_write_i2c(di, 0x60, csum2);
	if(err){
		dev_err(di->dev, "error write the subclass ID 64.\n");
		return ret;
	}
	dev_info(di->dev, "have reset the subclass ID 64.\n");
	//exit configue mode
	bq27x00_write_i2c(di, 0x00, 0x42);
	bq27x00_write_i2c(di, 0x01, 0x00);
	msleep(1000);
	
	flags = bq27x00_read_i2c(di, 0x06, 0);
	if((flags & 0x0010) &&(flags & 0x0020)){
		dev_info(di->dev, "FG haven't exit configue mode: 0x%04x", flags);
		return ret;
	}
	dev_info(di->dev, "FG have exit configue mod: 0x%04x", flags);
	
	//seal the FG
	bq27x00_write_i2c(di, 0x00, 0x20);
	bq27x00_write_i2c(di, 0x01, 0x00);
	return true;
}
	
	

static void bq27x00_init_dffs(struct bq27x00_device_info *di, char *model_str)
{
	int rc;
	u16 fw_ver   = 0xBDBD;
	u8  oem_str[16];
	char curr_dffs_info[256];
	int (*dffs)[10];
	
	dev_info(di->dev, "start to initailize dffs.");
//Get OEM string 
	rc = bq27x00_get_oemstr(di, oem_str);
	if (rc < 0) {
		return;
	}
//Check FG dffs initialization state 
	if (strlen(oem_str) != 0) {
		di->pdata->get_curr_dffs_by_oemstr(oem_str, curr_dffs_info);
		dev_info(di->dev, "fg dffs info is:");
		dev_info(di->dev, curr_dffs_info);

		if(strstr(curr_dffs_info, model_str) != NULL) {
			dev_info(di->dev, "fg dffs is not need to be initialized.");
			return;
		} else {
			dev_info(di->dev, "fg dffs will be switched to %s.", model_str);
		}
	} else {
		dev_info(di->dev, "fg dffs is not initialized!");
	}
	return;
//Get FG firmware version
	rc = bq27x00_get_firmware_version(di, &fw_ver);
	if (rc < 0) {
		return;
	}

//Get FG dffs 
	dffs = di->pdata->get_dffs_data(model_str, fw_ver);
	if (dffs == NULL) {
		dev_info(di->dev, "have not found dffs of model: %s, firmware:%04x", model_str, fw_ver);
		return;
	}

//Write dffs to FG 
	dev_info(di->dev, "initializing dffs, model: %s, firmware:%04x", model_str, fw_ver);
	bq27425_init(di, dffs);
	bq27x00_battery_seal(di);

//Release the access right of i2c 
	dev_info(di->dev, "dffs is initialized.\n");
}
 
static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	int num;
	bool ret;
	int retval = 0;
	printk(KERN_ERR "%s:++++++\n",__func__);
	dev_err(&client->dev, "%s:1\n",__func__);
	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	dev_err(&client->dev, "%s:2\n",__func__);
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;
	dev_err(&client->dev, "%s:3\n",__func__);
	name = kasprintf(GFP_KERNEL, "%s_battery", id->name);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	dev_err(&client->dev, "%s:4\n",__func__);
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	dev_err(&client->dev, "%s:5\n",__func__);
#if defined(DEBUG)
	register_dump_buffer = kzalloc(DUMP_BUFFER_SIZE, GFP_KERNEL);
	if (!register_dump_buffer) 
		dev_err(&client->dev, "Failed to alloc register dump buffer.\n");
	else
		INIT_DELAYED_WORK(&di->dump_register_worker, bq27425_dump_register_worker);
#endif	
	//return 0;
	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bat.name = name;
	di->bus.read = &bq27x00_read_i2c;
	//return 0;
	//Recover i2c line
	i2c_set_clientdata(client, di);
	di->pdata = client->dev.platform_data;
	bq27x00_di = di;
	//return 0;
	//Recover i2c line
	if (!bq27x00_recover_link(di))
		goto batt_failed_2;
	//initailize bq27x00 dffs
	ret = bq27441_init(di);
	if(ret)
		dev_info(&client->dev, "have initialize the FG.\n");
	else
		dev_info(&client->dev, "fail initialize the FG.\n");
	//Notify bq27x00, battery is inserted.
	bq27x00_insert_battery(di);

	retval = bq27x00_powersupply_init(di);
	if (retval)
		goto batt_failed_3;
	dev_err(&client->dev, "%s:6\n",__func__);
	if(request_irq(client->irq, bq27x00_battery_det_irq, 0, "bq27425_battery", di))
		 dev_err(&client->dev, "===bq27425 request_irq fail \n");

	register_reboot_notifier(&bq27x00_reboot_notifier_block);

#if defined(DEBUG)
	if (register_dump_buffer) 
		schedule_delayed_work(&di->dump_register_worker, 60 * (HZ));
#endif

	/* No used.
	retval = device_create_file(di->dev, &dev_attr_dffs_list);
	if (retval)
		goto batt_failed_3;
	*/
	probe_passed = 1;

	return 0;

batt_failed_3:
	kfree(di);
#if defined(DEBUG)
	if (register_dump_buffer) 
		kfree(register_dump_buffer);	
#endif	
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);
	unregister_reboot_notifier(&bq27x00_reboot_notifier_block);
	free_irq(client->irq, di);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27441", BQ27425 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
	{ "bq27425", BQ27200 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00_battery",
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");
	else
		printk(KERN_ERR "register BQ27x00 i2c driver\n");
	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

#else

static inline int bq27x00_battery_i2c_init(void) { return 0; }
static inline void bq27x00_battery_i2c_exit(void) {};

#endif

static int bq27x00_reboot_callback(struct notifier_block *nfb,
					unsigned long event, void *data)
{
	/* if the shutdown or reboot sequence started
	 * then block the access to maxim registers as chip
	 * cannot be recovered from broken i2c transactions
	 */
	mutex_lock(&bq27x00_di->lock);
	bq27x00_di->plat_rebooting = true;
	mutex_unlock(&bq27x00_di->lock);

	return NOTIFY_OK;
}

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;

	probe_passed = 0;
	ret = bq27x00_battery_i2c_init();
	if (ret)
		return ret;

	return ret;
}

static void __exit bq27x00_battery_exit(void)
{
	probe_passed = 0;
	bq27x00_battery_i2c_exit();
}

module_init(bq27x00_battery_init);
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
