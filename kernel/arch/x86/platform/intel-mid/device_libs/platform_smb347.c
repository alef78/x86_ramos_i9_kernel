/*
 * platform_smb347.c: smb347 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include "platform_smb347.h"

/* For  CLVT_TABLET_BQS_RHB0 */
static struct smb347_charger_platform_data smb347_pdata_rhb0 = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.max_charge_current		= 1800000,	/* Fast charge current(uA) */
	.pre_charge_current		= 250000,	/* Pre-charge current(uA) */
	.termination_current	= 200000,	/* Terminated-charge current(uA) */
	.mains_current_limit	= 1800000,	/* DCIN input current limit(uA) */
	.usb_hc_current_limit	= 1800000,	/* USBIN HC input current limit(uA) */
	.max_charge_voltage		= 4200000,	/* Float Voltage(uV) */
	.otg_uvlo_voltage		= 3300000,	/* OTG battery UVLO threshold(uA) */
	.battery_overheat_low	= 0,
	.battery_overheat_high	=45,
	.chip_temp_threshold		= 120,
	.soft_cold_temp_limit		= 5,
	.soft_hot_temp_limit		= 50,
	.hard_cold_temp_limit		= 5,
	.hard_hot_temp_limit		= 55,
	.suspend_on_hard_temp_limit	= true,
	.soft_temp_limit_compensation	= SMB347_SOFT_TEMP_COMPENSATE_CURRENT
					| SMB347_SOFT_TEMP_COMPENSATE_VOLTAGE,
	.charge_current_compensation	= 900000,
	.use_mains			= true,
	.use_usb			= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.susp_gpio			= SMB347_SUSP_GPIO,
};

static void *get_platform_data(void)
{
	return &smb347_pdata_rhb0;
}

void *smb347_platform_data(void *info)
{
	return get_platform_data();
}
