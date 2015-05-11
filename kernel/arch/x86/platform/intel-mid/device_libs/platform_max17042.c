/*
 * platform_max17042.c: max17042 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/export.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
//#include <linux/power/intel_mdf_battery.h>
#include <linux/power/smb347-a500cg-charger.h>
//#include <linux/power/bq24192_charger.h>
//#include <linux/power/bq24261_charger.h>
#include <linux/power/battery_id.h>
//#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include "platform_max17042.h"
//#include "platform_bq24192.h"
#include "platform_smb347.h"
//#include <asm/intel_em_config.h>

#define MRFL_SMIP_SRAM_ADDR		0xFFFCE000
#define MOFD_SMIP_SRAM_ADDR		0xFFFC5C00
#define MRFL_PLATFORM_CONFIG_OFFSET	0x3B3
#define MRFL_SMIP_SHUTDOWN_OFFSET	1
#define MRFL_SMIP_RESV_CAP_OFFSET	3

#define MRFL_VOLT_SHUTDOWN_MASK (1 << 1)
#define MRFL_NFC_RESV_MASK	(1 << 3)

#define BYT_FFRD8_TEMP_MIN_LIM	0	/* 0degC */
#define BYT_FFRD8_TEMP_MAX_LIM	55	/* 55degC */
#define BYT_FFRD8_BATT_MIN_VOLT	3400	/* 3400mV */
#define BYT_FFRD8_BATT_MAX_VOLT	4350	/* 4350mV */

#define BYT_CRV2_TEMP_MIN_LIM	0	/* 0degC */
#define BYT_CRV2_TEMP_MAX_LIM	45	/* 45degC */
#define BYT_CRV2_BATT_MIN_VOLT	3400	/* 3400mV */
#define BYT_CRV2_BATT_MAX_VOLT	4350	/* 4350mV */

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C to recover devices from abnormal status.
 * currently, only max17042 on I2C needs such workaround */
#define I2C_GPIO_PIN 29
	int i2c_gpio_pin = I2C_GPIO_PIN;
	lnw_gpio_set_alt(i2c_gpio_pin, LNW_GPIO);
	gpio_direction_output(i2c_gpio_pin, 0);
	gpio_set_value(i2c_gpio_pin, 1);
	udelay(10);
	gpio_set_value(i2c_gpio_pin, 0);
	udelay(10);
	lnw_gpio_set_alt(i2c_gpio_pin, LNW_ALT_1);
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

int mrfl_get_bat_health(void)
{
	return -ENODEV;
}

#define UMIP_REF_FG_TBL			0x806	/* 2 bytes */
#define BATT_FG_TBL_BODY		14	/* 144 bytes */
/**
 * mfld_fg_restore_config_data - restore config data
 * @name : Power Supply name
 * @data : config data output pointer
 * @len : length of config data
 *
 */
int mfld_fg_restore_config_data(const char *name, void *data, int len)
{
	int ret = 0;
#ifdef CONFIG_X86_MDFLD
	int mip_offset;
	/* Read the fuel gauge config data from umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_read_mip((u8 *)data, len, mip_offset, 0);
#endif
	return ret;
}
EXPORT_SYMBOL(mfld_fg_restore_config_data);

/**
 * mfld_fg_save_config_data - save config data
 * @name : Power Supply name
 * @data : config data input pointer
 * @len : length of config data
 *
 */
int mfld_fg_save_config_data(const char *name, void *data, int len)
{
	int ret = 0;
#ifdef CONFIG_X86_MDFLD
	int mip_offset;
	/* write the fuel gauge config data to umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_write_umip((u8 *)data, len, mip_offset);
#endif
	return ret;
}
EXPORT_SYMBOL(mfld_fg_save_config_data);

static uint16_t ctp_cell_char_tbl[] = {
	/* Data to be written from 0x80h */
	0x9d70, 0xb720, 0xb940, 0xba50, 0xbba0, 0xbc70, 0xbce0, 0xbd40,
	0xbe60, 0xbf60, 0xc1e0, 0xc470, 0xc700, 0xc970, 0xcce0, 0xd070,

	/* Data to be written from 0x90h */
	0x0090, 0x1020, 0x04A0, 0x0D10, 0x1DC0, 0x22E0, 0x2940, 0x0F60,
	0x11B0, 0x08E0, 0x08A0, 0x07D0, 0x0820, 0x0590, 0x0570, 0x0570,

	/* Data to be written from 0xA0h */
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
};

static void ctp_fg_restore_config_data(const char *name, void *data, int len)
{
	struct max17042_config_data *fg_cfg_data =
				(struct max17042_config_data *)data;
	fg_cfg_data->cfg = 0x2210;
	fg_cfg_data->learn_cfg = 0x0076;
	fg_cfg_data->filter_cfg = 0x87a4;
	fg_cfg_data->relax_cfg = 0x506b;
	memcpy(&fg_cfg_data->cell_char_tbl, ctp_cell_char_tbl,
					sizeof(ctp_cell_char_tbl));
	fg_cfg_data->rcomp0 = 0x0047;
	fg_cfg_data->tempCo = 0x1920;
	fg_cfg_data->etc = 0x00e0;
	fg_cfg_data->kempty0 = 0x0100;
	fg_cfg_data->ichgt_term = 0x0240;
	fg_cfg_data->full_cap = 3408;
	fg_cfg_data->design_cap = 3408;
	fg_cfg_data->full_capnom = 3408;
	fg_cfg_data->soc_empty = 0x0060;
	fg_cfg_data->rsense = 1;
	fg_cfg_data->cycles = 0x160;
}
EXPORT_SYMBOL(ctp_fg_restore_config_data);

static int ctp_fg_save_config_data(const char *name, void *data, int len)
{
	return 0;
}
EXPORT_SYMBOL(ctp_fg_save_config_data);

static bool ctp_is_volt_shutdown_enabled(void)
{
	return true;
}

static int ctp_get_vsys_min(void)
{
	return 3400000;//ok - disasm
}

static int ctp_get_0level_batt_volt(void)
{
	return 4300000;//ok - disasm
}

void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data pdata;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	struct sfi_table_simple *sb;
	char *mrfl_batt_str = "INTN0001";
	
	int intr = get_gpio_by_name("max_fg_alert");
	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	sb = (struct sfi_table_simple *)get_oem0_table();
	if (sb == NULL) {
		pr_info("invalid battery detected\n");
		snprintf(pdata.battid, BATTID_LEN + 1, "UNKNOWNB");
		snprintf(pdata.serial_num, SERIAL_NUM_LEN + 1, "000000");
		pdata.valid_battery = false;
	} else {
		pr_info("valid battery detected\n");
		/* First entry in OEM0 table is the BATTID. Read battid
		 * if pentry is not NULL and header length is greater
		 * than BATTID length*/
		if (sb->pentry && sb->header.len >= BATTID_LEN) {
			if (!((INTEL_MID_BOARD(1, TABLET, MRFL)) ||
				(INTEL_MID_BOARD(1, PHONE, MRFL)))) {
				snprintf(pdata.battid, BATTID_LEN + 1, "%s",
						(char *)sb->pentry);
			} else {
				if (strncmp((char *)sb->pentry,
					"PG000001", (BATTID_LEN)) == 0) {
					snprintf(pdata.battid,
						(BATTID_LEN + 1),
						"%s", mrfl_batt_str);
				} else {
					snprintf(pdata.battid,
						(BATTID_LEN + 1),
						"%s", (char *)sb->pentry);
				}
			}

			/* First 2 bytes represent the model name
			 * and the remaining 6 bytes represent the
			 * serial number. */
			if (pdata.battid[0] == 'I' &&
				pdata.battid[1] >= '0'
					&& pdata.battid[1] <= '9') {
				unsigned char tmp[SERIAL_NUM_LEN + 2];
				int i;
				snprintf(pdata.model_name,
					(MODEL_NAME_LEN) + 1,
						"%s", pdata.battid);
				memcpy(tmp, sb->pentry, BATTID_LEN);
				for (i = 0; i < SERIAL_NUM_LEN; i++) {
					sprintf(pdata.serial_num + i*2,
					"%02x", tmp[i + MODEL_NAME_LEN]);
				}
				if ((2 * SERIAL_NUM_LEN) <
					ARRAY_SIZE(pdata.serial_num))
					pdata.serial_num[2 * SERIAL_NUM_LEN]
								 = '\0';
			} else {
				snprintf(pdata.model_name,
						(MODEL_NAME_LEN + 1),
						"%s", pdata.battid);
				snprintf(pdata.serial_num,
						(SERIAL_NUM_LEN + 1), "%s",
				pdata.battid + MODEL_NAME_LEN);
			}
		}
		pdata.valid_battery = false;
	}
	
	pdata.en_vmax_intr = 0;
	pdata.reset_i2c_lines = max17042_i2c_reset_workaround;
	//pdata.fg_algo_model = 2;
	pdata.technology = 2;
	pdata.is_init_done = 1;
	pdata.battery_health = ctp_get_battery_health;
	//pdata.battery_pack_temp = ctp_get_battery_pack_temp;// not is ramos binary

	pdata.is_volt_shutdown_enabled = ctp_is_volt_shutdown_enabled;
	pdata.get_vmin_threshold = ctp_get_vsys_min;
	pdata.enable_current_sense = 1;
	pdata.get_vmax_threshold = ctp_get_0level_batt_volt;
	pdata.battery_status = smb347_get_charging_status;

	return &pdata;
}
