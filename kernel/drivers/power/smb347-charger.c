/*
 * Summit Microelectronics SMB347 Battery Charger Driver
 *
 * Copyright (C) 2011, Intel Corporation
 *
 * Authors: Bruce E. Robertson <bruce.e.robertson@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/power/smb347-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/usb/penwell_otg.h>
#include <linux/kernel.h>	/* Needed for KERN_DEBUG */
#include <linux/wakelock.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel-mid.h>

/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS_FUNCS			0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB		BIT(2)
#define CFG_VARIOUS_FUNCS_SETTING		0x83
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_CHARGER_CONTROL			0x04
#define CFG_APSD_ENABLED				BIT(2)
#define CFG_NC_CHARGER_VIA_APSD_ENABLED	BIT(1)
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_USB_HC_MODE_CONTROL_BY_PIN	BIT(4)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_DISABLED_OTG_I2C		0x00
#define CFG_OTHER_RID_DISABLED_OTG_PIN		0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C		0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW		BIT(5)
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK	0x03
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_FAULT_IRQ_DCIN_OV			BIT(3)
#define CFG_FAULT_IRQ_TEMP_HARD			BIT(7)
#define CFG_FAULT_IRQ_OTG_UV			BIT(5)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT		BIT(7)
#define CFG_STATUS_IRQ_BATTERY_OVP			BIT(5)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_OTG_ENABLED			BIT(4)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_B_USB500_MODE			BIT(1)
#define CMD_B_HC_MODE				BIT(0)
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_A_HOT_TEMP_HARD_IRQ			BIT(7)
#define IRQSTAT_A_HOT_TEMP_HARD_STAT		BIT(6)
#define IRQSTAT_A_COLD_TEMP_HARD_IRQ		BIT(5)
#define IRQSTAT_A_COLD_TEMP_HARD_STAT		BIT(4)
#define IRQSTAT_B				0x36
#define IRQSTAT_B_BATT_OVP_IRQ				BIT(7)
#define IRQSTAT_B_BATT_OVP_STAT				BIT(6)
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_D				0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT		BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ		BIT(3)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
#define IRQSTAT_E_DCIN_OV_STAT			BIT(6)
#define IRQSTAT_E_DCIN_OV_IRQ			BIT(7)
#define IRQSTAT_F				0x3a
#define IRQSTAT_F_OTG_UV_IRQ			BIT(5)
#define IRQSTAT_F_OTG_UV_STAT			BIT(4)

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_HOLDOFF_STAT			BIT(3)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHG_TERM				BIT(5)
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_E					0x3f

#define STATUS_UPDATE_INTERVAL			(HZ * 60)	/*60 sec */
#define CURR_THRES_FULL_COND			200
#define CAP_FULL_VALUE					100

/* ADC Channel Numbers for BPTHERM */
#define CLT_BATT_NUM_GPADC_SENSORS	1
#define CLT_GPADC_BPTHERM_CHNUM	0x9
#define CLT_GPADC_BPTHERM_SAMPLE_COUNT	1

/*CLT battery temperature  attributes*/
#define CLT_BTP_ADC_MIN	107
#define CLT_BTP_ADC_MAX	977
#define CLT_BPTHERM_CURVE_MAX_VALUES	4
#define CLT_BPTHERM_CURVE_MAX_SAMPLES	23

#define CHARGER_USB_NAME		"smb347-usb"
#define CHARGER_MAINS_NAME		"smb347-mains"

struct smb347_otg_event {
	struct list_head	node;
	bool			param;
};

enum smb347_usb_charge_mode {
	SMB347_CHARGE_MODE_USB100 = 0,
	SMB347_CHARGE_MODE_USB500,
	SMB347_CHARGE_MODE_HC,
};

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @running: the driver is up and running
 * @dentry: for debugfs
 * @otg: pointer to OTG transceiver if any
 * @otg_nb: notifier for OTG notifications
 * @otg_work: work struct for OTG notifications
 * @otg_queue: queue holding received OTG notifications
 * @otg_queue_lock: protects concurrent access to @otg_queue
 * @otg_last_param: last parameter value from OTG notification
 * @otg_enabled: OTG VBUS is enabled
 * @otg_battery_uv: OTG battery undervoltage condition is on
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	struct power_supply_charger_cap cap;
	struct delayed_work chrg_evt_worker;
	bool			mains_online;
	bool			usb_online;
	bool			ac_online;
	bool			usb_is_suspend;
	bool			charging_enabled;
	bool			running;
	struct dentry		*dentry;
	struct usb_phy	*otg;
	struct notifier_block	otg_nb;
	struct work_struct	otg_work;
	struct list_head	otg_queue;
	spinlock_t		otg_queue_lock;
	bool			otg_enabled;
	bool			otg_battery_uv;
	const struct smb347_charger_platform_data	*pdata;
	struct delayed_work	smb347_statmon_worker;
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;

	/* Handle for gpadc requests BPTHERM */
	void *gpadc_handle;
};

static struct smb347_charger *smb347_dev;
static char *smb347_power_supplied_to[] = {
			"max170xx_battery",
			"max17042_battery",
			"max17047_battery",
			"max17050_battery",
};

/* Fast charge current in uA */
static const unsigned int fcc_tbl[] = {
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Pre-charge current in uA */
static const unsigned int pcc_tbl[] = {
	100000,
	150000,
	200000,
	250000,
};

/* Termination current in uA */
static const unsigned int tc_tbl[] = {
	37500,
	50000,
	100000,
	150000,
	200000,
	250000,
	500000,
	600000,
};

/* Input current limit in uA */
static const unsigned int icl_tbl[] = {
	300000,
	500000,
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Charge current compensation in uA */
static const unsigned int ccc_tbl[] = {
	250000,
	700000,
	900000,
	1200000,
};

/*
 * temperature v/s ADC value table to interpolate and calculate temp
 */
static int const ctp_bptherm_curve_data[CLT_BPTHERM_CURVE_MAX_SAMPLES]
	[CLT_BPTHERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 977, 961},
	{-10, -15, 961, 941},
	{-5, -10, 941, 917},
	{0, -5, 917, 887},
	{5, 0, 887, 853},
	{10, 5, 853, 813},
	{15, 10, 813, 769},
	{20, 15, 769, 720},
	{25, 20, 720, 669},
	{30, 25, 669, 615},
	{35, 30, 615, 561},
	{40, 35, 561, 508},
	{45, 40, 508, 456},
	{50, 45, 456, 407},
	{55, 50, 407, 357},
	{60, 55, 357, 315},
	{65, 60, 315, 277},
	{70, 65, 277, 243},
	{75, 70, 243, 212},
	{80, 75, 212, 186},
	{85, 80, 186, 162},
	{90, 85, 162, 140},
	{100, 90, 140, 107},
};

/***********************************************************************
 * charger and battery specific interfaces exposed to external modules
 */
static int smb347_read(struct smb347_charger *smb, u8 reg);

/* Temperature conversion Macros */
static int ctp_conv_adc_temp(int adc_val,
	int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

/* Check for valid Temp ADC range */
static bool ctp_is_valid_temp_adc(int adc_val)
{
	bool ret = false;

	if (adc_val >= CLT_BTP_ADC_MIN && adc_val <= CLT_BTP_ADC_MAX)
		ret = true;

	return ret;
}

/* Check if the adc value is in the curve sample range */
static bool ctp_is_valid_temp_adc_range(int val, int min, int max)
{
	bool ret = false;
	if (val >= min && val <= max)
		ret = true;
	return ret;
}

/**
 * ctp_adc_to_temp - convert ADC code to temperature
 * @adc_val : ADC sensor reading
 * @tmp : finally read temperature
 *
 * Returns 0 on success or -ERANGE in error case
 */
static int ctp_adc_to_temp(uint16_t adc_val, int *tmp)
{
	int temp = 0;
	int i;

	if (!ctp_is_valid_temp_adc(adc_val)) {
		dev_warn(&smb347_dev->client->dev,
			"Temperature out of Range: %u\n", adc_val);
		/*
		 * If the value returned as an ERANGE the battery icon shows an
		 * exclaimation mark in the COS.In order to fix the issue, if
		 * the ADC returns a value which is not in range specified, we
		 * update the value within the bound.
		 */
		if (adc_val > CLT_BTP_ADC_MAX)
			adc_val = CLT_BTP_ADC_MAX;
		else if (adc_val < CLT_BTP_ADC_MIN)
			adc_val = CLT_BTP_ADC_MIN;
	}

	for (i = 0; i < CLT_BPTHERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (ctp_is_valid_temp_adc_range(
			adc_val, ctp_bptherm_curve_data[i][3],
			ctp_bptherm_curve_data[i][2])) {

			temp = ctp_conv_adc_temp(adc_val,
				  ctp_bptherm_curve_data[i][2],
				  ctp_bptherm_curve_data[i][2] -
				  ctp_bptherm_curve_data[i][3],
				  ctp_bptherm_curve_data[i][0] -
				  ctp_bptherm_curve_data[i][1]);

			temp += ctp_bptherm_curve_data[i][1];
			break;
		}
	}

	if (i >= CLT_BPTHERM_CURVE_MAX_SAMPLES) {
		dev_warn(&smb347_dev->client->dev, "Invalid temp adc range\n");
		return -EINVAL;
	}
	*tmp = temp;

	return 0;
}

/**
 * ctp_read_adc_temp - read ADC sensor to get the temperature
 * @tmp: op parameter where temperature get's read
 *
 * Returns 0 if success else -1 or -ERANGE
 */
static int ctp_read_adc_temp(int *tmp)
{
	int ret;
	int gpadc_sensor_val = 0;

	if (!smb347_dev->gpadc_handle) {
		ret = -ENODEV;
		dev_err(&smb347_dev->client->dev, "get BPTHERM pin failed!\n");
		goto read_adc_exit;
	}

	ret = intel_mid_gpadc_sample(smb347_dev->gpadc_handle,
				CLT_GPADC_BPTHERM_SAMPLE_COUNT,
				&gpadc_sensor_val);
	if (ret) {
		dev_err(&smb347_dev->client->dev,
			"adc driver api returned error(%d)\n", ret);
		goto read_adc_exit;
	}

	ret = ctp_adc_to_temp(gpadc_sensor_val, tmp);
read_adc_exit:
	return ret;
}

/* returns the battery pack temperature read from adc */
int ctp_get_battery_pack_temp(int *temp)
{
	if (!smb347_dev)
		return -ENODEV;

	/* check if charger is ready */
	if (!power_supply_get_by_name(CHARGER_MAINS_NAME)
			|| !power_supply_get_by_name(CHARGER_USB_NAME)) {
		printk("-- %s --Error:The charger driver interface is not ready.\n",__func__);
		return -EAGAIN;
	}
	return ctp_read_adc_temp(temp);
}
EXPORT_SYMBOL(ctp_get_battery_pack_temp);

/**
 * ctp_get_battery_health - to get the battery health status
 *
 * Returns battery health status
 */
int ctp_get_battery_health(void)
{
	int batt_temp, ret;
	if (!smb347_dev) {
		printk("-- %s --Error:The charger dev data is not ready.\n", __func__);
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	dev_dbg(&smb347_dev->client->dev, "+%s\n", __func__);
	/* Get the battery pack temperature */
	ret = ctp_get_battery_pack_temp(&batt_temp);
	if (ret < 0) {
		dev_err(&smb347_dev->client->dev,
			"battery pack temp read fail:%d", ret);
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	if ((batt_temp >smb347_dev->pdata->battery_overheat_high) ||
			(batt_temp < smb347_dev->pdata->battery_overheat_low)) {
		dev_err(&smb347_dev->client->dev, "Battery over heat.\n");
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	/* Check if battery OVP condition occured */
	ret = smb347_read(smb347_dev, IRQSTAT_B);
	if (ret < 0) {
		dev_warn(&smb347_dev->client->dev,
			"read reg failed %s\n", __func__);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (ret & IRQSTAT_B_BATT_OVP_STAT)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	dev_dbg(&smb347_dev->client->dev, "-%s\n", __func__);
	return POWER_SUPPLY_HEALTH_GOOD;
}
EXPORT_SYMBOL(ctp_get_battery_health);

bool ctp_is_volt_shutdown_enabled(void)
{
	/* FPO1 is reserved in case of CTP so we are returning true */
	return true;
}
EXPORT_SYMBOL(ctp_is_volt_shutdown_enabled);

/* When batt level is >0%, but volt_ocv < vsys_min,
 * 0% batt level should be reported,
 * and shut down the DUT.
 */
int ctp_get_vsys_min(void)
{
	return 3550 * 1000;
}
EXPORT_SYMBOL(ctp_get_vsys_min);

/* When batt level is 0%, but volt_ocv >> 0level_batt_volt,
 * 1% batt level should be reported,
 * and the DUT continues to work.
 * When 0level_batt_volt is set as 4.3V, it means this feature is disabled.
 */
int ctp_get_0level_batt_volt(void)
{
	return 4300 * 1000;
}
EXPORT_SYMBOL(ctp_get_0level_batt_volt);

#if MONITOR_BATT_STATUS
/**
 * check_batt_psy -check for whether power supply type is battery
 * @dev : Power Supply dev structure
 * @data : Power Supply Driver Data
 * Context: can sleep
 *
 * Return true if power supply type is battery
 *
 */
static struct power_supply *fg_psy;
static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	/* check for whether power supply type is battery */
	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		dev_info(&smb347_dev->client->dev, "fg chip found:%s\n", psy->name);
		fg_psy = psy;
		return true;
	}

	return false;
}

/**
 * get_fg_chip_psy - identify the Fuel Gauge Power Supply device
 * Context: can sleep
 *
 * Return Fuel Gauge power supply structure
 */
static struct power_supply *get_fg_chip_psy(void)
{
	if (fg_psy)
		return fg_psy;

	/* loop through power supply class */
	class_for_each_device(power_supply_class, NULL, NULL,
		check_batt_psy);
	return fg_psy;
}

/**
 * fg_chip_get_property - read a power supply property from Fuel Gauge driver
 * @psp : Power Supply property
 *
 * Return power supply property value
 *
 */
static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = get_fg_chip_psy();
	if (fg_psy) {
		ret = fg_psy->get_property(fg_psy, psp, &val);
		if (!ret)
		return val.intval;
	}

	return ret;
}

#endif


/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return -EINVAL;
	return tbl[val];
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

static int smb347_read(struct smb347_charger *smb, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to read reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		dev_warn(&smb->client->dev, "failed to write reg 0x%x: %d\n",
			 reg, ret);
	return ret;
}

/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

/**
 * smb347_update_status - updates the charging status
 * @smb: pointer to smb347 charger instance
 *
 * Function checks status of the charging and updates internal state
 * accordingly. Returns %0 if there is no change in status, %1 if the
 * status has changed and negative errno in case of failure.
 */
static int smb347_update_status(struct smb347_charger *smb)
{
	bool usb = false;
	bool dc = false;
	int ret;

	ret = smb347_read(smb, IRQSTAT_E);
	if (ret < 0)
		return ret;

	/*
	 * Dc and usb are set depending on whether they are enabled in
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if (!smb->otg_enabled) {
		if (smb->pdata->use_mains)
			dc = !(ret & IRQSTAT_E_DCIN_UV_STAT);
		if (smb->pdata->use_usb)
			usb = !(ret & IRQSTAT_E_USBIN_UV_STAT);
			dc |= smb->ac_online;
	}

	mutex_lock(&smb->lock);
	ret = smb->mains_online != dc || smb->usb_online != usb;
	smb->mains_online = dc;
	smb->usb_online = usb;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_status_monitor - worker function to monitor status
 * @work: delayed work handler structure
 * Context: Can sleep
 *
 * Monitors status of the charger and updates the charging status.
 * Note: This worker is manily added to notify the user space about
 * capacity, health and status chnages.
 */
static void smb347_status_monitor(struct work_struct *work)
{
	struct smb347_charger *smb = container_of(work,
			struct smb347_charger, smb347_statmon_worker.work);
	int ret;

	pm_runtime_get_sync(&smb->client->dev);

	ret = smb347_update_status(smb);
	if (ret < 0)
		dev_err(&smb->client->dev, "error in updating smb347 status\n");
	
#if MONITOR_BATT_STATUS
	int charge, chr_full, temp, volt, ocv, level, curr;
	int batt_temp = 0;
	ocv = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_OCV);
	if (ocv == -ENODEV || ocv == -EINVAL) {
		dev_warn(&smb->client->dev, "Can't read ocv from FG\n");
		return;
	}

	volt = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (volt == -ENODEV || volt == -EINVAL) {
		dev_warn(&smb->client->dev, "Can't read voltage from FG\n");
		return;
	}

	temp = fg_chip_get_property(POWER_SUPPLY_PROP_TEMP);
	if (temp == -ENODEV || temp == -EINVAL) {
		dev_warn(&smb->client->dev, "Can't read temp from FG\n");
		return;
	}

	/* convert to milli volts */
	volt /= 1000;
	ocv /= 1000;

	/*convert to degree Celcius from tenths of degree Celsius */
	temp = temp / 10;

	charge = fg_chip_get_property(POWER_SUPPLY_PROP_CHARGE_NOW);
	if (charge == -ENODEV || charge == -EINVAL) {
		dev_warn(&smb->client->dev, "Can't read charge from FG\n");
		return;
	}

	chr_full = fg_chip_get_property(POWER_SUPPLY_PROP_CHARGE_FULL);
	if (chr_full == -ENODEV || chr_full == -EINVAL) {
		dev_warn(&smb->client->dev, "Can't read charge_full from FG\n");
		return;
	}

	level = fg_chip_get_property(POWER_SUPPLY_PROP_CAPACITY);
	if (level == -ENODEV || level == -EINVAL) {
		dev_warn(&smb347_dev->client->dev, "Can't read level from FG\n");
		return;
	}

	curr = fg_chip_get_property(POWER_SUPPLY_PROP_CURRENT_NOW);
	if (curr == -ENODEV || curr == -EINVAL) {
		dev_warn(&smb347_dev->client->dev, "Can't read curr from FG\n");
		return;
	}
	curr /= 1000;

	/* Get the battery pack temperature */
	ret = ctp_get_battery_pack_temp(&batt_temp);
	if (ret < 0) {
		dev_err(&smb347_dev->client->dev,
			"battery pack temp read fail:%d", ret);
	}

	dev_info(&smb->client->dev,
			"\n%s: temp=%d vbatt=%d ocv=%d chrg_now=%d chrg_full=%d\nbatt_level=%d, current_now=%d(mA)\nBPTHERM= %d[%d,%d]\n",
			__func__, temp, volt, ocv, charge, chr_full, level, curr, batt_temp,
			smb347_dev->pdata->battery_overheat_low,
			smb347_dev->pdata->battery_overheat_high);

#endif	/* MONITOR_BATT_STATUS */

	if (smb->pdata->use_mains)
		power_supply_changed(&smb->mains);
	if (smb->pdata->use_usb)
		power_supply_changed(&smb->usb);
	schedule_delayed_work(&smb->smb347_statmon_worker,
						STATUS_UPDATE_INTERVAL);
	pm_runtime_put_sync(&smb->client->dev);
}

/*
 * smb347_is_online - returns whether input power source is connected
 * @smb: pointer to smb347 charger instance
 *
 * Returns %true if input power source is connected. Note that this is
 * dependent on what platform has configured for usable power sources. For
 * example if USB is disabled, this will return %false even if the USB
 * cable is connected.
 */
static bool smb347_is_online(struct smb347_charger *smb)
{
	bool ret;

	mutex_lock(&smb->lock);
	ret = (smb->usb_online || smb->mains_online) && !smb->otg_enabled;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_charging_status - returns status of charging
 * @smb: pointer to smb347 charger instance
 *
 * Function returns charging status. %0 means no charging is in progress,
 * %1 means pre-charging, %2 fast-charging and %3 taper-charging.
 */
static int smb347_charging_status(struct smb347_charger *smb)
{
	int ret;

	if (!smb347_is_online(smb))
		return 0;

	ret = smb347_read(smb, STAT_C);
	if (ret < 0)
		return 0;

	return (ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
}

static int smb347_charging_set(struct smb347_charger *smb, bool enable)
{
	int ret = 0;

	if (smb->pdata->enable_control != SMB347_CHG_ENABLE_SW) {
		dev_dbg(&smb->client->dev,
			"charging enable/disable in SW disabled\n");
		return 0;
	}

	mutex_lock(&smb->lock);
	if (smb->charging_enabled != enable) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb->charging_enabled = enable;

		if (enable)
			ret |= CMD_A_CHG_ENABLED;
		else
			ret &= ~CMD_A_CHG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
	}
out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_charging_enable(struct smb347_charger *smb)
{
	/* prevent system from entering s3 while charger is connected */
	if (!wake_lock_active(&smb->wakelock))
		wake_lock(&smb->wakelock);
	return smb347_charging_set(smb, true);
}

static inline int smb347_charging_disable(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_charging_set(smb, false);
	/* release the wake lock when charger is unplugged */
	if (wake_lock_active(&smb->wakelock))
		wake_unlock(&smb->wakelock);

	return ret;
}

static int smb347_update_online(struct smb347_charger *smb)
{
	int ret;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (smb347_is_online(smb)) {
		ret = smb347_charging_enable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to enable charging\n");
	} else {
		ret = smb347_charging_disable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to disable charging\n");
	}

	return ret;
}

static int smb347_otg_set(struct smb347_charger *smb, bool enable)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	mutex_lock(&smb->lock);

	if (pdata->otg_control == SMB347_OTG_CONTROL_SW) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb347_set_writable(smb, true);
		if (enable)
			ret |= CMD_A_OTG_ENABLED;
		else
			ret &= ~CMD_A_OTG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		smb347_set_writable(smb, false);

		if (ret < 0)
			goto out;
	} else {
		/*
		 * Switch to pin control or auto-OTG depending on how
		 * platform has configured.
		 */
		smb347_set_writable(smb, true);

		ret = smb347_read(smb, CFG_OTHER);
		if (ret < 0) {
			smb347_set_writable(smb, false);
			goto out;
		}

		ret &= ~CFG_OTHER_RID_MASK;

		switch (pdata->otg_control) {
		case SMB347_OTG_CONTROL_SW_PIN:
			if (enable) {
				ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
				ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			} else {
				ret &= ~CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			}
			break;

		case SMB347_OTG_CONTROL_SW_AUTO:
			if (enable)
				ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
			break;

		default:
			dev_err(&smb->client->dev,
				"impossible OTG control configuration: %d\n",
				pdata->otg_control);
			break;
		}

		ret = smb347_write(smb, CFG_OTHER, ret);
		smb347_set_writable(smb, false);
		if (ret < 0)
			goto out;
	}

	smb->otg_enabled = enable;

out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_otg_enable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, true);
}

static inline int smb347_otg_disable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, false);
}

static void smb347_otg_drive_vbus(struct smb347_charger *smb, bool enable)
{
	int ret = 0;
	if (enable == smb->otg_enabled)
		return;

	if (enable) {
		if (smb->otg_battery_uv) {
			dev_dbg(&smb->client->dev,
				"battery low voltage, won't enable OTG VBUS\n");
			return;
		}

		/*
		 * Normal charging must be disabled first before we try to
		 * enable OTG VBUS.
		 */
		smb347_charging_disable(smb);
		smb347_otg_enable(smb);
		smb347_update_status(smb);
		ret = smb347_read(smb, CMD_A);
		dev_info(&smb->client->dev, "OTG VBUS on, CMD_A= 0x%x\n", ret);
	} else {
		smb347_otg_disable(smb);
		smb347_update_status(smb);
		/*
		 * Only re-enable charging if we have some power supply
		 * connected.
		 */
		if (smb347_is_online(smb)) {
			smb347_charging_enable(smb);
			smb->otg_battery_uv = false;
			/* Small delay-interval(10-11ms) for
			 * STAT-C to be updated
			 */
			usleep_range(10000, 11000);
		}
		ret = smb347_read(smb, CMD_A);
		dev_info(&smb->client->dev, "OTG VBUS off, CMD_A= 0x%x\n", ret);
	}

	if (smb->pdata->use_mains)
		power_supply_changed(&smb->mains);
	if (smb->pdata->use_usb)
		power_supply_changed(&smb->usb);
}

static void smb347_otg_work(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, otg_work);
	struct smb347_otg_event *evt, *tmp;
	unsigned long flags;

	/* Process the whole event list in one go. */
	spin_lock_irqsave(&smb->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&smb->otg_queue_lock, flags);

		/* For now we only support set vbus events */
		smb347_otg_drive_vbus(smb, evt->param);
		kfree(evt);

		spin_lock_irqsave(&smb->otg_queue_lock, flags);
	}
	spin_unlock_irqrestore(&smb->otg_queue_lock, flags);
}

static int smb347_otg_notifier(struct notifier_block *nb, unsigned long event,
			       void *param)
{
	struct smb347_charger *smb =
		container_of(nb, struct smb347_charger, otg_nb);
	struct smb347_otg_event *evt;

	dev_dbg(&smb->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS || !smb->running)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&smb->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->param = *(bool *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&smb->otg_queue_lock);
	list_add_tail(&evt->node, &smb->otg_queue);
	spin_unlock(&smb->otg_queue_lock);

	queue_work(system_nrt_wq, &smb->otg_work);
	return NOTIFY_OK;
}

static int smb347_set_charge_current(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CHARGE_CURRENT);
	if (ret < 0)
		return ret;

	if (smb->pdata->max_charge_current) {
		val = current_to_hw(fcc_tbl, ARRAY_SIZE(fcc_tbl),
				    smb->pdata->max_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_FCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_FCC_SHIFT;
	}

	if (smb->pdata->pre_charge_current) {
		val = current_to_hw(pcc_tbl, ARRAY_SIZE(pcc_tbl),
				    smb->pdata->pre_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_PCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_PCC_SHIFT;
	}

	if (smb->pdata->termination_current) {
		val = current_to_hw(tc_tbl, ARRAY_SIZE(tc_tbl),
				    smb->pdata->termination_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_TC_MASK;
		ret |= val;
	}

	return smb347_write(smb, CFG_CHARGE_CURRENT, ret);
}

static int smb347_set_current_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->mains_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->mains_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_DC_MASK;
		ret |= val << CFG_CURRENT_LIMIT_DC_SHIFT;
	}

	if (smb->pdata->usb_hc_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->usb_hc_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_USB_MASK;
		ret |= val;
	}

	return smb347_write(smb, CFG_CURRENT_LIMIT, ret);
}

static int smb347_set_voltage_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		return ret;

	if (smb->pdata->pre_to_fast_voltage) {
		val = smb->pdata->pre_to_fast_voltage;

		/* uV */
		val = clamp_val(val, 2400000, 3000000) - 2400000;
		val /= 200000;

		ret &= ~CFG_FLOAT_VOLTAGE_THRESHOLD_MASK;
		ret |= val << CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT;
	}

	if (smb->pdata->max_charge_voltage) {
		val = smb->pdata->max_charge_voltage;

		/* uV */
		val = clamp_val(val, 3500000, 4500000) - 3500000;
		val /= 20000;

		ret |= val;
	}

	ret = smb347_write(smb, CFG_FLOAT_VOLTAGE, ret);
	if (ret < 0)
		return ret;

	if (smb->pdata->otg_uvlo_voltage) {
		val = smb->pdata->otg_uvlo_voltage;

		val = clamp_val(val, 2700000, 3300000) - 2700000;
		val /= 200000;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK;
		ret |= val & 0x3;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int smb347_set_temp_limits(struct smb347_charger *smb)
{
	bool enable_therm_monitor = false;
	int ret, val;

	if (smb->pdata->chip_temp_threshold) {
		val = smb->pdata->chip_temp_threshold;

		/* degree C */
		val = clamp_val(val, 100, 130) - 100;
		val /= 10;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_TEMP_THRESHOLD_MASK;
		ret |= val << CFG_OTG_TEMP_THRESHOLD_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	ret = smb347_read(smb, CFG_TEMP_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->soft_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_cold_temp_limit;

		val = clamp_val(val, 0, 15);
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_SOFT_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->soft_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_hot_temp_limit;

		val = clamp_val(val, 40, 55) - 40;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_SOFT_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_cold_temp_limit;

		val = clamp_val(val, -5, 10) + 5;
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_HARD_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_hot_temp_limit;

		val = clamp_val(val, 50, 65) - 50;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_HARD_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	ret = smb347_write(smb, CFG_TEMP_LIMIT, ret);
	if (ret < 0)
		return ret;

	/*
	 * If any of the temperature limits are set, we also enable the
	 * thermistor monitoring.
	 *
	 * When soft limits are hit, the device will start to compensate
	 * current and/or voltage depending on the configuration.
	 *
	 * When hard limit is hit, the device will suspend charging
	 * depending on the configuration.
	 */
	if (enable_therm_monitor) {
		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_MONITOR_DISABLED;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->suspend_on_hard_temp_limit) {
		ret = smb347_read(smb, CFG_SYSOK);
		if (ret < 0)
			return ret;

		ret &= ~CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED;

		ret = smb347_write(smb, CFG_SYSOK, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->soft_temp_limit_compensation !=
	    SMB347_SOFT_TEMP_COMPENSATE_DEFAULT) {
		val = smb->pdata->soft_temp_limit_compensation & 0x3;

		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT;

		ret &= ~CFG_THERM_SOFT_COLD_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->charge_current_compensation) {
		val = current_to_hw(ccc_tbl, ARRAY_SIZE(ccc_tbl),
				    smb->pdata->charge_current_compensation);
		if (val < 0)
			return val;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_CC_COMPENSATION_MASK;
		ret |= (val & 0x3) << CFG_OTG_CC_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int smb347_set_usb_charge_mode(struct smb347_charger *smb,
							enum smb347_usb_charge_mode charge_mode)
{
	int ret;

	ret = smb347_read(smb, CMD_B);
	switch(charge_mode)
	{
		case SMB347_CHARGE_MODE_USB100:
			ret &= ~CMD_B_HC_MODE;
			ret &= ~CMD_B_USB500_MODE;
			dev_info(&smb->client->dev, "Set charge mode as USB100.\n");
			break;
		case SMB347_CHARGE_MODE_USB500:
			ret &= ~CMD_B_HC_MODE;
			ret |= CMD_B_USB500_MODE;
			dev_info(&smb->client->dev, "Set charge mode as USB500.\n");
			break;
		case SMB347_CHARGE_MODE_HC:
			ret |= CMD_B_HC_MODE;
			dev_info(&smb->client->dev, "Set charge mode as HC.\n");
			break;
		default:
			dev_warn(&smb->client->dev, "Unknown charge mode.Nothing to be done.\n");
	}
	ret = smb347_write(smb, CMD_B, ret);

	return ret;
}

static int smb347_init_usb_charge(struct smb347_charger *smb)
{
	int ret;

	/* disable Automatic Power Source Detection(APSD)
	 * According to the D+(pin_DPLUS) and D-(pin_DMUNS) connecting way
	 * of smb347 on different HW
	 */
	ret = smb347_read(smb, CFG_CHARGER_CONTROL);
	ret &= ~CFG_APSD_ENABLED;
	ret &= ~CFG_NC_CHARGER_VIA_APSD_ENABLED;

	ret = smb347_write(smb, CFG_CHARGER_CONTROL, ret);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "Disable APSD failed.\n");
		return ret;
	}

	/* USB/HC charge mode controled by register */
	ret = smb347_read(smb, CFG_PIN);
	ret &= ~CFG_USB_HC_MODE_CONTROL_BY_PIN;
	ret = smb347_write(smb, CFG_PIN, ret);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "Charge mode controled by reg failed.\n");
		return ret;
	}

	/* USB100 as the default charge mode */
	ret = smb347_set_usb_charge_mode(smb, SMB347_CHARGE_MODE_USB100);

	return ret;
}

static int smb347_set_usb_suspend(struct smb347_charger *smb, bool enable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0) {
		return ret;
	}

	if (enable) {
		ret |= CMD_A_SUSPEND_ENABLED;
		smb->usb_is_suspend = true;
		dev_info(&smb->client->dev, "To set USB suspend.\n");
	} else {
		ret &= ~CMD_A_SUSPEND_ENABLED;
		smb->usb_is_suspend = false;
		dev_info(&smb->client->dev, "To set USB enabled.\n");
	}
	ret = smb347_write(smb, CMD_A, ret);

	return ret;
}

static int smb347_hw_init(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */
	ret = smb347_set_charge_current(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_current_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_voltage_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_temp_limits(smb);
	if (ret < 0)
		goto fail;

	/* If USB charging is disabled we should put the USB in suspend mode */
	if (!smb->pdata->use_usb) {
		ret = smb347_set_usb_suspend(smb, true);
		if (ret < 0)
			goto fail;
	} else {
		ret = smb347_set_usb_suspend(smb, false);
		if (ret < 0)
			goto fail;

		ret = smb347_init_usb_charge(smb);
		if (ret < 0)
			goto fail;
	}

	/* Setup OTG VBUS control depending on the platform data. */
	ret = smb347_read(smb, CFG_OTHER);
	if (ret < 0)
		goto fail;

	ret &= ~CFG_OTHER_RID_MASK;

	switch (smb->pdata->otg_control) {
	case SMB347_OTG_CONTROL_DISABLED:
		break;

	case SMB347_OTG_CONTROL_SW:
	case SMB347_OTG_CONTROL_SW_PIN:
	case SMB347_OTG_CONTROL_SW_AUTO:
		//smb->otg = usb_get_transceiver();
		smb->otg = usb_get_phy(USB_PHY_TYPE_USB2);
		if (smb->otg) {
			INIT_WORK(&smb->otg_work, smb347_otg_work);
			INIT_LIST_HEAD(&smb->otg_queue);
			spin_lock_init(&smb->otg_queue_lock);

			smb->otg_nb.notifier_call = smb347_otg_notifier;
			ret = usb_register_notifier(smb->otg, &smb->otg_nb);
			if (ret < 0) {
				usb_put_phy(smb->otg);
				//usb_put_transceiver(smb->otg);
				smb->otg = NULL;
				goto fail;
			}

			dev_info(&smb->client->dev,
				"registered to OTG notifications\n");
		}
		break;

	case SMB347_OTG_CONTROL_PIN:
		ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
		ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
		break;

	case SMB347_OTG_CONTROL_AUTO:
		ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
		break;
	}

	ret = smb347_write(smb, CFG_OTHER, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_read(smb, CFG_PIN);
	if (ret < 0)
		goto fail;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;

	switch (smb->pdata->enable_control) {
	case SMB347_CHG_ENABLE_SW:
		/* Do nothing, 0 means i2c control */
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_LOW:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_HIGH:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_HIGH;
		break;
	}

	/* Disable Automatic Power Source Detection (APSD) interrupt. */
	ret &= ~CFG_PIN_EN_APSD_IRQ;

	ret = smb347_write(smb, CFG_PIN, ret);
	if (ret < 0)
		goto fail;

	/*
	 * Summit recommends that register 0x02 (CFG_VARIOUS_FUNCS) is
	 * programmed last. This has something to do with the fact that
	 * current limits are correctly updated. We really don't have
	 * anything else to configure there except that we update the input
	 * source priority.
	 */
	ret = smb347_read(smb, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	ret &= ~CFG_VARIOUS_FUNCS_PRIORITY_USB;

	ret = smb347_write(smb, CFG_VARIOUS_FUNCS, CFG_VARIOUS_FUNCS_SETTING);
	if (ret < 0)
		goto fail;

	ret = smb347_update_status(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_update_online(smb);

	smb347_set_writable(smb, false);
	return ret;

fail:
	if (smb->otg) {
		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		//usb_put_transceiver(smb->otg);
		usb_put_phy(smb->otg);
		smb->otg = NULL;
	}
	smb347_set_writable(smb, false);
	return ret;
}

static irqreturn_t smb347_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;
	int stat_c, irqstat_a, irqstat_b, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;

	pm_runtime_get_sync(&smb->client->dev);

	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_a = smb347_read(smb, IRQSTAT_A);
	if (irqstat_a < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_A failed\n");
        return IRQ_NONE;
	}
	if (irqstat_a & IRQSTAT_A_HOT_TEMP_HARD_IRQ) {
		if (irqstat_a & IRQSTAT_A_HOT_TEMP_HARD_STAT) {
			dev_err(&smb->client->dev, "charging stopped due to hot temp\n");
		}
	}
	if (irqstat_a & IRQSTAT_A_COLD_TEMP_HARD_IRQ) {
		if (irqstat_a & IRQSTAT_A_COLD_TEMP_HARD_STAT) {
			dev_err(&smb->client->dev, "charging stopped due to cold temp\n");
		}
	}

	irqstat_b = smb347_read(smb, IRQSTAT_B);
	if (irqstat_b < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_B failed\n");
		return IRQ_NONE;
	}
	
	irqstat_c = smb347_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_d = smb347_read(smb, IRQSTAT_D);
	if (irqstat_d < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_D failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	if (irqstat_e & (IRQSTAT_E_DCIN_OV_IRQ)) {
		if (irqstat_e & (IRQSTAT_E_DCIN_OV_STAT)) {
			dev_err(&smb->client->dev, "charging stopped due to DC over voltage\n");
		}
	}
	if (irqstat_e & (IRQSTAT_E_DCIN_UV_IRQ)) {
		if (irqstat_e & (IRQSTAT_E_DCIN_UV_STAT)) {
			dev_err(&smb->client->dev, "charging stopped due to DC under voltage\n");
		}
	}

	irqstat_f = smb347_read(smb, IRQSTAT_F);
	if (irqstat_f < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_F failed\n");
		return IRQ_NONE;
	}

	/*
	 * If we get charger error we report the error back to user.
	 * If the error is recovered charging will resume again.
	 */
	if (stat_c & STAT_C_CHARGER_ERROR) {
		dev_err(&smb->client->dev,
			"charging stopped due to charger error\n");

		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);

		ret = IRQ_HANDLED;
	}

	if (irqstat_b & (IRQSTAT_B_BATT_OVP_IRQ)) {
		if (irqstat_b & (IRQSTAT_B_BATT_OVP_STAT)) {
			dev_err(&smb->client->dev, "Battery is OVP!\n");
		}
	}

	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 *
	 * Turn off it because of so many interrupts when in maintance.
	 */
#if 0
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ)) {
		if ((irqstat_c & IRQSTAT_C_TERMINATION_STAT) &&
						smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		dev_info(&smb->client->dev,
			"[Charge Terminated] Going to HW Maintenance mode\n");
		ret = IRQ_HANDLED;
	}
#endif

	/*
	 * If we got a complete charger timeout int that means the charge
	 * full is not detected with in charge timeout value.
	 */
	if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_IRQ) {
		dev_info(&smb->client->dev,
			"[Charge Timeout]:Total Charge Timeout INT recieved\n");
		if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_STAT)
			dev_info(&smb->client->dev,
				"[Charge Timeout]:charging stopped\n");
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (irqstat_e &  IRQSTAT_E_DCIN_UV_IRQ) {
		if (smb347_update_status(smb) > 0) {
			smb347_update_online(smb);
			if (smb->pdata->use_mains)
				power_supply_changed(&smb->mains);
		}

		if (smb->mains_online)
			dev_info(&smb->client->dev, "Charger connected\n");
		else
			dev_info(&smb->client->dev, "Charger disconnected\n");
		ret = IRQ_HANDLED;
	} 

	if (irqstat_e & IRQSTAT_E_USBIN_UV_IRQ) {
		if (smb347_update_status(smb) > 0) {
			smb347_update_online(smb);
		}
		
		if (smb->usb_online)
			dev_info(&smb->client->dev, "USB connected\n");
		else
			dev_info(&smb->client->dev, "USB disconnected\n");
		ret = IRQ_HANDLED;
	}

	/*
	 * If the battery voltage falls below OTG UVLO the VBUS is
	 * automatically turned off but we must not enable it again unless
	 * UVLO is cleared. It will be cleared when external power supply
	 * is connected and the battery voltage goes over the UVLO
	 * threshold.
	 */
	if (irqstat_f & IRQSTAT_F_OTG_UV_IRQ) {
		smb->otg_battery_uv = !!(irqstat_f & IRQSTAT_F_OTG_UV_STAT);
		dev_info(&smb->client->dev, "Vbatt is below OTG UVLO\n");
		ret = IRQ_HANDLED;
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}

static int smb347_irq_set(struct smb347_charger *smb, bool enable)
{
	int ret,val;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Enable/disable interrupts for:
	 *	- under voltage
	 *	- termination current reached
	 *	- charger error
	 */
	if (enable) {
		val = CFG_FAULT_IRQ_DCIN_UV | CFG_FAULT_IRQ_DCIN_OV | CFG_FAULT_IRQ_TEMP_HARD;

		if (smb->otg)
			val |= CFG_FAULT_IRQ_OTG_UV;

		ret = smb347_write(smb, CFG_FAULT_IRQ, val);
		if (ret < 0)
			goto fail;

		val = CFG_STATUS_IRQ_CHARGE_TIMEOUT | CFG_STATUS_IRQ_BATTERY_OVP;
		ret = smb347_write(smb, CFG_STATUS_IRQ, val);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	} else {
		ret = smb347_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	}

fail:
	smb347_set_writable(smb, false);
	return ret;
}

static inline int smb347_irq_enable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, true);
}

static inline int smb347_irq_disable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, false);
}

static int smb347_irq_init(struct smb347_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, smb->client->name,
				   smb);
	if (ret < 0)
		goto fail_gpio;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto fail_irq;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb347_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb347_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb347_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

	smb347_set_writable(smb, false);
	smb->client->irq = irq;
	enable_irq_wake(smb->client->irq);
	return 0;

fail_readonly:
	smb347_set_writable(smb, false);
fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb347_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, mains);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->mains_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->usb_online;
		return 0;
	}
	return -EINVAL;
}

/* This function is for FG getting battery status 
 * by setting the FG platform data
 */
int smb347_get_charging_status(void)
{
	int ret, status, level, curr;

	if (!smb347_dev)
		return -EINVAL;

	if (!smb347_is_online(smb347_dev))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return ret;

	dev_info(&smb347_dev->client->dev,
			"Charging Status: STAT_C:0x%x\n", ret);

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		level = fg_chip_get_property(POWER_SUPPLY_PROP_CAPACITY);
		if (level == -ENODEV || level == -EINVAL) {
			dev_warn(&smb347_dev->client->dev, "Can't read level from FG\n");
			return -EINVAL;
		}
		curr = fg_chip_get_property(POWER_SUPPLY_PROP_CURRENT_NOW);
		if (curr == -ENODEV || curr == -EINVAL) {
			dev_warn(&smb347_dev->client->dev, "Can't read curr from FG\n");
			return -EINVAL;;
		}
		curr /= 1000;
		if (level == CAP_FULL_VALUE) {
			status = POWER_SUPPLY_STATUS_FULL;
		} else if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return status;
}
EXPORT_SYMBOL(smb347_get_charging_status);

static enum power_supply_property smb347_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb347_charger *smb =
			container_of(psy, struct smb347_charger, battery);
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	ret = smb347_update_status(smb);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = smb347_get_charging_status();
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!smb347_is_online(smb))
			return -ENODATA;

		/*
		 * We handle trickle and pre-charging the same, and taper
		 * and none the same.
		 */
		switch (smb347_charging_status(smb)) {
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->battery_info.voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->battery_info.voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb347_debugfs_show(struct seq_file *s, void *data)
{
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	ret = smb347_read(smb, CMD_A);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_A, ret);
	ret = smb347_read(smb, CMD_B);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_B, ret);
	ret = smb347_read(smb, CMD_C);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_C, ret);
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

#define DBGFS_CMD_LEN			10
#define DBGFS_EACH_PARAM_LEN	3
static ssize_t smb347_debugfs_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	unsigned long reg_addr;
	unsigned long reg_value;
	char cmd_buf[DBGFS_CMD_LEN];
	char str_addr[DBGFS_EACH_PARAM_LEN];
	char str_value[DBGFS_EACH_PARAM_LEN];
	struct smb347_charger *smb;

	struct seq_file *seq = file->private_data;
	if (NULL != seq) {
		smb = seq->private;
	} else {
		return -EFAULT;
	}

	if (!smb) {
		return -EFAULT;
	}
	/* get the reg addr and value to be write from the cmd */
	if (copy_from_user(cmd_buf, user_buf, DBGFS_CMD_LEN)) {
		return -EFAULT;
	}
	sscanf(cmd_buf, "%s %s", (char *)&str_addr, (char *)&str_value);
	str_addr[DBGFS_EACH_PARAM_LEN - 1] = '\0';
	str_value[DBGFS_EACH_PARAM_LEN - 1] = '\0';
	if (kstrtoul(str_addr, 16, &reg_addr)) {
		return -EINVAL;
	}
	if (kstrtoul(str_value, 16, &reg_value)) {
		return -EINVAL;
	}

	/* write the reg */
	dev_info(&smb->client->dev,"[dbgfs write] reg_addr:0x%x reg_value:0x%x\n",
			(u32)reg_addr, (u32)reg_value);
	ret = smb347_set_writable(smb, true);
	ret = smb347_write(smb, reg_addr, reg_value);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "I2C write failed\n");
	}
	smb347_set_writable(smb, false);

	return count;
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.write		= smb347_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void smb347_event_worker(struct work_struct *work)
{
	struct smb347_charger *smb = container_of(work,
				struct smb347_charger, chrg_evt_worker.work);
	int ret = 0;

	dev_info(&smb->client->dev, "%s\n", __func__);

	smb347_set_writable(smb, true);
	switch (smb->cap.chrg_evt) {
	case POWER_SUPPLY_CHARGER_EVENT_CONNECT:
		pm_runtime_get_sync(&smb->client->dev);
	case POWER_SUPPLY_CHARGER_EVENT_UPDATE:
	case POWER_SUPPLY_CHARGER_EVENT_RESUME:
		if (smb->cap.chrg_type != POWER_SUPPLY_TYPE_USB_HOST) {
			if (smb->usb_is_suspend) {
				smb347_set_usb_suspend(smb, false);
			}

			mutex_lock(&smb->lock);
			switch(smb->cap.chrg_type)
			{
				case POWER_SUPPLY_TYPE_USB:
					dev_info(&smb->client->dev, "SDP charger connected.\n");
					smb->usb.type = POWER_SUPPLY_TYPE_USB;
					smb->usb_online = true;
					switch(smb->cap.ma)
					{
						case 100:
							smb347_set_usb_charge_mode(smb, SMB347_CHARGE_MODE_USB100);
							break;
						case 500:
							smb347_set_usb_charge_mode(smb, SMB347_CHARGE_MODE_USB500);
							break;
						default:
							dev_warn(&smb->client->dev,
										"SDP Charge current not supported: %d(mA)\n",
										smb->cap.ma);
					}
					break;
				case POWER_SUPPLY_TYPE_USB_DCP:
					dev_info(&smb->client->dev, "DCP charger connected.\n");
					smb->usb.type = POWER_SUPPLY_TYPE_USB_DCP;
					smb->ac_online = true;
					smb347_set_usb_charge_mode(smb, SMB347_CHARGE_MODE_HC);
					break;
				default:
					dev_info(&smb->client->dev, "No care charger type:%d\n",
									smb->cap.chrg_type);
					smb->usb.type = smb->cap.chrg_type;
			}
			mutex_unlock(&smb->lock);

		}
#if 0
		/*
		 * We update the battery charging status as per the type of
		 * charger connected. If it is host mode cable connected then
		 * battery status should be discharging
		 */
		if (chip->chrg_type != POWER_SUPPLY_TYPE_USB_HOST) {
			/* if current level is 100% but need to be charged, show FULL status icon */
			int batt_level;
			batt_level = fg_chip_get_property(POWER_SUPPLY_PROP_CAPACITY);
			if (100 == batt_level) {
				dev_info(&chip->client->dev, "01-Last level 100%, charging and show FULL icon.\n");
				chip->batt_status = POWER_SUPPLY_STATUS_FULL;
			} else {
				chip->batt_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		}
#endif

		/*
		 * Prevent system from entering s3 while charger is connected
		 * or if any OTG device (mouse/keyboard) is connected.
		 */
		if (!wake_lock_active(&smb->wakelock))
			wake_lock(&smb->wakelock);

		break;
	case POWER_SUPPLY_CHARGER_EVENT_DISCONNECT:
		pm_runtime_put_sync(&smb->client->dev);
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
	case POWER_SUPPLY_CHARGER_EVENT_SUSPEND:
		dev_info(&smb->client->dev, "USB suspend/disconnect evt. To disable charging\n");
		/* set the charge mode to USB100 */
		smb347_set_usb_charge_mode(smb, SMB347_CHARGE_MODE_USB100);
		if (!smb->usb_is_suspend) {
				ret = smb347_set_usb_suspend(smb, true);
			}
		if (ret < 0) {
			dev_err(&smb->client->dev,
				"charge disabling failed\n");
			goto i2c_write_fail;
		}
		mutex_lock(&smb->lock);

		smb->usb_online = false;
		smb->ac_online = false;
		/* release the wake lock when charger is unplugged */
		if (wake_lock_active(&smb->wakelock))
			wake_unlock(&smb->wakelock);
		mutex_unlock(&smb->lock);
		break;
	default:
		dev_err(&smb->client->dev,
			"invalid charger event:%d\n", smb->cap.chrg_evt);
		goto i2c_write_fail;
	}

	smb347_set_writable(smb, false);

	smb347_update_status(smb);
	msleep(120);
	power_supply_changed(&smb->usb);
	power_supply_changed(&smb->mains);

i2c_write_fail:
	return ;
}

static void smb347_charging_port_changed(struct power_supply *psy,
				struct power_supply_charger_cap *cap)
{
	struct smb347_charger *smb = container_of(psy,
				struct smb347_charger, usb);

	mutex_lock(&smb->lock);
	smb->cap.chrg_evt = cap->chrg_evt;
	smb->cap.chrg_type = cap->chrg_type;
	smb->cap.ma = cap->ma;
	mutex_unlock(&smb->lock);

	dev_info(&smb->client->dev, "[chrg] evt:%d type:%d cur:%d\n",
				cap->chrg_evt, cap->chrg_type, cap->ma);

	/* 1.the default charge mode is USB100
	 * 2.the event time between USB100 to USB500 is so short,
	 *	 so there may be too works are running at the same
	 *	 time, which will interact each other for getting the
	 *	 battery status.
	 */
	if (100 != smb->cap.ma) {
		schedule_delayed_work(&smb->chrg_evt_worker, 0);
	}
}

static int smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	smb->client = client;
	smb->pdata = pdata;
	i2c_set_clientdata(client, smb);
	smb347_dev = smb;

	mutex_init(&smb->lock);
	/* init wake lock */
	wake_lock_init(&smb->wakelock,
		WAKE_LOCK_SUSPEND, "smb347_wakelock");

	/* Allocate ADC Channels for BPTHERM to get battery thermal */
	smb->gpadc_handle =
		intel_mid_gpadc_alloc(CLT_BATT_NUM_GPADC_SENSORS,
				  CLT_GPADC_BPTHERM_CHNUM | CH_NEED_VCALIB |
				  CH_NEED_VREF);
	if (NULL == smb->gpadc_handle) {
		dev_err(&client->dev,
		 "ADC allocation failed: Check if ADC driver came up\n");
	}

	if ((pdata->susp_gpio>0) && gpio_request(pdata->susp_gpio, "smb347_susp") )
	{
		printk("Error request smb347_susp gpio\n");
		return -EINVAL;
	}
	if (pdata->susp_gpio>0) {
		gpio_direction_output(pdata->susp_gpio,1);
		msleep(20);
	}

	ret = smb347_hw_init(smb);
	if (ret < 0){
		dev_warn(&smb->client->dev, "smb347 suspend\n");
		wake_lock_destroy(&smb->wakelock);
		devm_kfree(dev, smb);

		/* Because FG will use smb347_dev for gettng status, free it */
		smb347_dev = NULL;
		return ret;
	}

	if (smb->pdata->use_mains) {
		smb->mains.name = CHARGER_MAINS_NAME;
		smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
		smb->mains.get_property = smb347_mains_get_property;
		smb->mains.properties = smb347_mains_properties;
		smb->mains.num_properties = ARRAY_SIZE(smb347_mains_properties);
		smb->mains.supplied_to = smb347_power_supplied_to;
		smb->mains.num_supplicants =
				ARRAY_SIZE(smb347_power_supplied_to);
		ret = power_supply_register(dev, &smb->mains);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->use_usb) {
		smb->usb.name = CHARGER_USB_NAME;
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
		smb->usb.get_property = smb347_usb_get_property;
		smb->usb.properties = smb347_usb_properties;
		smb->usb.num_properties = ARRAY_SIZE(smb347_usb_properties);
		smb->usb.supplied_to = smb347_power_supplied_to;
		smb->usb.num_supplicants = ARRAY_SIZE(smb347_power_supplied_to);
		smb->usb.charging_port_changed = smb347_charging_port_changed;
		ret = power_supply_register(dev, &smb->usb);
		if (ret < 0) {
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	if (smb->pdata->show_battery) {
		smb->battery.name = "smb347-battery";
		smb->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		smb->battery.get_property = smb347_battery_get_property;
		smb->battery.properties = smb347_battery_properties;
		smb->battery.num_properties =
				ARRAY_SIZE(smb347_battery_properties);
		ret = power_supply_register(dev, &smb->battery);
		if (ret < 0) {
			if (smb->pdata->use_usb)
				power_supply_unregister(&smb->usb);
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb347_irq_init(smb);
		if (ret < 0) {
			dev_warn(&smb->client->dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(&smb->client->dev, "disabling IRQ support\n");
		}
	}
	INIT_DELAYED_WORK(&smb->chrg_evt_worker, smb347_event_worker);
	INIT_DELAYED_WORK(&smb->smb347_statmon_worker,
						smb347_status_monitor);
	//INIT_DELAYED_WORK_DEFERRABLE(&smb->smb347_statmon_worker,
	//					smb347_status_monitor);

	smb->running = true;
	smb->dentry = debugfs_create_file("smb347-regs", S_IRUSR, NULL, smb,
					  &smb347_debugfs_fops);

	/*
	 * Query the OTG driver to check if it has already sent the charger
	 * event. If yes, then we should start the charging. This would ensure
	 * charger driver doesn't miss any usb event.
	 */
	ret = penwell_otg_query_power_supply_cap(&smb->cap);
	if (ret < 0) {
		dev_err(&smb->client->dev,
					"OTG Query failed. OTGD not loaded\n");
	} else {
		dev_info(&smb->client->dev, "Schedule the charge event worker.\n");
		schedule_delayed_work(&smb->chrg_evt_worker, 0);
	}

	/* Start the status monitoring worker */
	schedule_delayed_work(&smb->smb347_statmon_worker, 0);

	return 0;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

	if (client->irq) {
		smb347_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}

	if (smb->otg) {
		struct smb347_otg_event *evt, *tmp;

		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		smb347_otg_disable(smb);
		//usb_put_transceiver(smb->otg);
		usb_put_phy(smb->otg);

		/* Clear all the queued events. */
		flush_work_sync(&smb->otg_work);
		list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
			list_del(&evt->node);
			kfree(evt);
		}
	}

	if (smb->gpadc_handle) {
		intel_mid_gpadc_free(smb->gpadc_handle);
	}

	wake_lock_destroy(&smb->wakelock);

	if (smb->pdata->show_battery)
		power_supply_unregister(&smb->battery);
	if (smb->pdata->use_usb)
		power_supply_unregister(&smb->usb);
	if (smb->pdata->use_mains)
		power_supply_unregister(&smb->mains);

	pm_runtime_get_noresume(&smb->client->dev);

	return 0;
}

/*
 * Befor shutdown, must make sure the OTG will be turned off.
 * If not, because of the non-volatile regs of smb347,
 * there will be always 5V on the USB-vbus, and then
 * the charge only mode will always run up.
 */
static void smb347_shutdown(struct i2c_client *client)
{
	int ret;
	struct smb347_charger *smb = i2c_get_clientdata(client);

	/* Disable OTG during shutdown */
	mutex_lock(&smb->lock);
	if (smb) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0) {
			dev_warn(&smb->client->dev, "RD failed to turn off OTG.\n");
		} else {
			smb347_set_writable(smb, true);
			ret &= ~CMD_A_OTG_ENABLED;
			ret = smb347_write(smb, CMD_A, ret);
			if (ret < 0) {
				dev_warn(&smb->client->dev, "WR failed to turn off OTG.\n");
			} else {
				dev_info(&smb->client->dev, "Turn off OTG for shutdown.\n");
			}

			smb347_set_writable(smb, false);
		}
	}
	mutex_unlock(&smb->lock);
}

#ifdef CONFIG_PM
static int smb347_prepare(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb347_is_online(smb))
		return -EBUSY;

	/*
	 * disable irq here doesn't mean smb347 interrupt
	 * can't wake up system. smb347 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

	cancel_delayed_work_sync(&smb->smb347_statmon_worker);

	dev_dbg(&smb->client->dev, "smb347 suspend\n");

	return 0;
}

static void smb347_complete(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb->client->irq > 0)
		enable_irq(smb->client->irq);

	/* check if the wakeup is due to charger connect */
	if (smb347_update_status(smb) > 0) {
		smb347_update_online(smb);
		if (smb->mains_online || smb->usb_online)
			dev_info(&smb->client->dev,
				"wakeup due to charger connect\n");
		else	/* most unlkely to happen */
			dev_info(&smb->client->dev,
				"wake up due to charger disconnect\n");
	}

	/* Start the status monitoring worker */
	schedule_delayed_work(&smb->smb347_statmon_worker,
					msecs_to_jiffies(500));

	dev_dbg(&smb->client->dev, "smb347 resume\n");

}
#else
#define smb347_prepare NULL
#define smb347_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb347_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int smb347_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb347_runtime_suspend	NULL
#define smb347_runtime_resume	NULL
#define smb347_runtime_idle	NULL
#endif

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0},
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static const struct dev_pm_ops smb347_pm_ops = {
	.prepare		= smb347_prepare,
	.complete		= smb347_complete,
	.runtime_suspend	= smb347_runtime_suspend,
	.runtime_resume		= smb347_runtime_resume,
	.runtime_idle		= smb347_runtime_idle,
};

static struct i2c_driver smb347_driver = {
	.driver = {
		.name	= "smb347",
		.owner	= THIS_MODULE,
		.pm	= &smb347_pm_ops,
	},
	.probe		= smb347_probe,
	.shutdown	= smb347_shutdown,
	.remove		= smb347_remove,
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	printk(KERN_DEBUG "Inside smb347_init\n");
	return i2c_add_driver(&smb347_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
