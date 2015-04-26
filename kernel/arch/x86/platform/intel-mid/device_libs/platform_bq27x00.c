/*
 * platform_bq27x00.c: bq27xxx platform data initilization file
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
#include <linux/power/bq27x00_battery.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/power/smb347-charger.h>
#include <linux/power/bq24192_charger.h>
#include <linux/power/bq24261_charger.h>
#include <linux/power/battery_id.h>
#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include "platform_bq27x00.h"
#include "platform_bq24192.h"
#include "platform_bq24261.h"
#include "bq27x00_dffs_SL3770125.h"
#include "bq27x00_dffs_WD3870127P.h"

#define VBATT_MAX 4350
#define VBATT_MIN 3400
#define MAX_PROD_NAME_LEN 4

#define MRFL_SMIP_SRAM_ADDR		0xFFFCE000
#define MRFL_PLATFORM_CONFIG_OFFSET	0x3B3
#define MRFL_SMIP_SHUTDOWN_OFFSET	1
#define MRFL_SMIP_RESV_CAP_OFFSET	3

#define MRFL_VOLT_SHUTDOWN_MASK (1 << 1)
#define MRFL_NFC_RESV_MASK	(1 << 3)

//MD:add SFI_TABLE_OEM0
#define SFI_TABLE_OEM0_SIZE 120

void bq27x00_i2c_reset_workaround(void)
{
	int ret;
//The following code is from P702T. For I2C line is changed, and connected 2 devices
//bq27425 and bq24192, the following code need to be changed later.
#if 0
#define I2C_1_DAT_GPIO_PIN 26
#define I2C_1_CLK_GPIO_PIN 27
/* toggle clock pin of I2C to recover devices from abnormal status. */
	ret = gpio_request(I2C_1_CLK_GPIO_PIN, "i2c-1-clock");
	if (ret) {
		pr_info("Unable to request i2c-1-clock pin\n");	
		return;
	}

	ret = gpio_request(I2C_1_DAT_GPIO_PIN, "i2c-1-data");
	if (ret) {
		pr_info("Unable to request i2c-1-data pin\n");	
		return;
	}

 	lnw_gpio_set_alt(I2C_1_CLK_GPIO_PIN, LNW_GPIO);
 	lnw_gpio_set_alt(I2C_1_DAT_GPIO_PIN, LNW_GPIO);

	gpio_direction_output(I2C_1_CLK_GPIO_PIN, 0);
	gpio_direction_output(I2C_1_DAT_GPIO_PIN, 0);

//As TI engineer talked, if I2C is held by FG, pull data/clock line to low for 2 seconds,
//FG will release I2C line.
	gpio_set_value(I2C_1_CLK_GPIO_PIN, 0);
	gpio_set_value(I2C_1_DAT_GPIO_PIN, 0);
	mdelay(2 * 1000);

	lnw_gpio_set_alt(I2C_1_CLK_GPIO_PIN, LNW_ALT_1);
	lnw_gpio_set_alt(I2C_1_DAT_GPIO_PIN, LNW_ALT_1);
	mdelay(50);
#endif	
}
EXPORT_SYMBOL(bq27x00_i2c_reset_workaround);

extern void sfi_table_oem0_init(struct sfi_table_simple *sb);
static bool msic_battery_check(struct bq27x00_platform_data *pdata)
{
	struct sfi_table_simple *sb;
	char *mrfl_batt_str = "INTN0001";

	sb = (struct sfi_table_simple *)get_oem0_table();
//MD:add for charger prof
	if(sb == NULL){
		sb = kzalloc( SFI_TABLE_OEM0_SIZE,GFP_KERNEL);
		if(!sb){
			pr_err("%s(): Error in kzalloc_sb\n", __func__);
			return -ENOMEM;
		}
		sfi_table_oem0_init(sb);
	}
	if (sb == NULL) {
		pr_info("invalid battery detected\n");
		snprintf(pdata->battid, BATTID_LEN + 1, "UNKNOWNB");
		snprintf(pdata->serial_num, SERIAL_NUM_LEN + 1, "000000");
		return false;
	} else {
		pr_info("valid battery detected\n");
		/* First entry in OEM0 table is the BATTID. Read battid
		 * if pentry is not NULL and header length is greater
		 * than BATTID length*/
		if (sb->pentry && sb->header.len >= BATTID_LEN) {
			if (!((INTEL_MID_BOARD(1, TABLET, MRFL)) ||
				(INTEL_MID_BOARD(1, PHONE, MRFL)))) {
				snprintf(pdata->battid, BATTID_LEN + 1, "%s",
						(char *)sb->pentry);
			} else {
				if (strncmp((char *)sb->pentry,
					"PG000001", (BATTID_LEN)) == 0) {
					snprintf(pdata->battid,
						(BATTID_LEN + 1),
							"%s", mrfl_batt_str);
				} else {
					snprintf(pdata->battid,
						(BATTID_LEN + 1),
						"%s", (char *)sb->pentry);
				}
			}

			/* First 2 bytes represent the model name
			 * and the remaining 6 bytes represent the
			 * serial number. */
			if (pdata->battid[0] == 'I' &&
				pdata->battid[1] >= '0'
					&& pdata->battid[1] <= '9') {
				unsigned char tmp[SERIAL_NUM_LEN + 2];
				int i;
				snprintf(pdata->model_name,
					(MODEL_NAME_LEN) + 1,
						"%s", pdata->battid);
				memcpy(tmp, sb->pentry, BATTID_LEN);
				for (i = 0; i < SERIAL_NUM_LEN; i++) {
					sprintf(pdata->serial_num + i*2,
					"%02x", tmp[i + MODEL_NAME_LEN]);
				}
				if ((2 * SERIAL_NUM_LEN) <
					ARRAY_SIZE(pdata->serial_num))
					pdata->serial_num[2 * SERIAL_NUM_LEN]
								 = '\0';
			} else {
				snprintf(pdata->model_name,
						(MODEL_NAME_LEN + 1),
						"%s", pdata->battid);
				snprintf(pdata->serial_num,
						(SERIAL_NUM_LEN + 1), "%s",
				pdata->battid + MODEL_NAME_LEN);
			}
		}
		pr_info("battid is :%s\n",pdata->battid);
		return true;
	}
	return false;
}

static const struct bq27x00_dffs_list dffs_list[] = {
	{
		.name = "SL3770125",
		.oemstr = SL3770125_DFFS_OEM_STR_G1,
		.fuel_gauge_type = 0x0202,
		.dffs = SL3770125_dffs_G1, 
	},
	{
		.name = "SL3770125",
		.oemstr = SL3770125_DFFS_OEM_STR_G2A,
		.fuel_gauge_type = 0x0205,
		.dffs = SL3770125_dffs_G2A, 
	},
	{
		.name = "WD3870127P",
		.oemstr = WD3870127P_DFFS_OEM_STR_G1,
		.fuel_gauge_type = 0x0202,
		.dffs = WD3870127P_dffs_G1, 
	},

	{
		.name = "WD3870127P",
		.oemstr = WD3870127P_DFFS_OEM_STR_G2A,
		.fuel_gauge_type = 0x0205,
		.dffs = WD3870127P_dffs_G2A, 
	},
};

void get_dffs_list(char* output_str)
{
	int i,n;
	int v, y, m, d;
	char prod_name[MAX_PROD_NAME_LEN+1];
	char *p;

	p = output_str;
	n = sizeof(dffs_list)/sizeof(struct bq27x00_dffs_list);
	for (i=0; i<n; i++) {
		v = dffs_list[i].oemstr[0];
		
		memcpy(prod_name, &dffs_list[i].oemstr[1], MAX_PROD_NAME_LEN);
		prod_name[MAX_PROD_NAME_LEN] = 0;

		y = dffs_list[i].oemstr[5];
		m = dffs_list[i].oemstr[6];
		d = dffs_list[i].oemstr[7];

		sprintf(p, "%s, \t0x%04x, \t%s, \t%d, %d-%d-%d\n", 
				dffs_list[i].name, 
				dffs_list[i].fuel_gauge_type, 
				prod_name, 
				v, 
				y, m, d);
		p += strlen(p);
	}
}

void* get_dffs_data(char* model, unsigned short fuel_gauge_type)
{
	int i,n;
	
	n = sizeof(dffs_list)/sizeof(struct bq27x00_dffs_list);
	for (i=0; i<n; i++) {
		if (!strncmp(model, dffs_list[i].name, strlen(dffs_list[i].name)) && 
				(dffs_list[i].fuel_gauge_type == fuel_gauge_type)) {
			break;
		}
	}

	if (i < n) 
		return dffs_list[i].dffs;
	else
		return (void*)0; 
}

void get_curr_dffs_by_oemstr(char* oemstr, char* output_str)
{
	int i,n;
	int v, y, m, d;
	char prod_name[MAX_PROD_NAME_LEN+1];

	n = sizeof(dffs_list)/sizeof(struct bq27x00_dffs_list);
	for (i=0; i<n; i++) {
		if (memcmp(oemstr, dffs_list[i].oemstr, MAX_OEM_STRING_LEN) == 0) {
			v = dffs_list[i].oemstr[0];
		
			memcpy(prod_name, &dffs_list[i].oemstr[1], MAX_PROD_NAME_LEN);
			prod_name[MAX_PROD_NAME_LEN] = 0;

			y = dffs_list[i].oemstr[5];
			m = dffs_list[i].oemstr[6];
			d = dffs_list[i].oemstr[7];

			sprintf(output_str, "%s, \t%04x, \t%s, \t%d, %d-%d-%d\n", 
					dffs_list[i].name, 
					dffs_list[i].fuel_gauge_type, 
					prod_name, 
					v, 
					y, m, d);
			break;
		}
	}
}

int get_vmax_threshold(void)
{
	return VBATT_MAX * 1000;
}

int get_vmin_threshold(void)
{
	return VBATT_MIN * 1000;
}

static bool ctp_is_volt_shutdown_enabled(void)
{
	 /* FPO1 is reserved in case of CTP so we are returning true */
	return true;
}

static int ctp_get_vsys_min(void)
{
	struct ps_batt_chg_prof batt_profile;
	int ret;
	ret = get_batt_prop(&batt_profile);
	if (!ret)
		return ((struct ps_pse_mod_prof *)batt_profile.batt_prof)
					->low_batt_mV * 1000;

	return BATT_VMIN_THRESHOLD_DEF * 1000;
}

static int ctp_get_battery_temp(int *temp)
{
	return platform_get_battery_pack_temp(temp);
}

int mrfl_get_bat_health(void)
{

	int pbat_health = -ENODEV;
	int bqbat_health = -ENODEV;
#ifdef CONFIG_BQ24261_CHARGER
	 bqbat_health = bq24261_get_bat_health();
#endif
#ifdef CONFIG_PMIC_CCSM
	pbat_health = pmic_get_health();
#endif

	/*Battery temperature exceptions are reported to PMIC. ALl other
	* exceptions are reported to bq24261 charger. Need to read the
	* battery health reported by both drivers, before reporting
	* the actual battery health
	*/

	/* FIXME: need to have a time stamp based implementation to
	* report battery health
	*/

	if (pbat_health < 0 && bqbat_health < 0)
		return pbat_health;
	if (pbat_health > 0 && pbat_health != POWER_SUPPLY_HEALTH_GOOD)
		return pbat_health;
	else
		return bqbat_health;
}

#define DEFAULT_VMIN	3400000
int mrfl_get_vsys_min(void)
{
	struct ps_batt_chg_prof batt_profile;
	int ret;
	ret = get_batt_prop(&batt_profile);
	if (!ret)
		return ((struct ps_pse_mod_prof *)batt_profile.batt_prof)
					->low_batt_mV * 1000;
	return DEFAULT_VMIN;
}
#define DEFAULT_VMAX_LIM	4360000
int mrfl_get_volt_max(void)
{
	struct ps_batt_chg_prof batt_profile;
	int ret;
	ret = get_batt_prop(&batt_profile);
	if (!ret)
		return ((struct ps_pse_mod_prof *)batt_profile.batt_prof)
					->voltage_max * 1000;
	return DEFAULT_VMAX_LIM;
}

int byt_get_vsys_min(void)
{
	return DEFAULT_VMIN;
}

static bool is_mapped;
static void __iomem *smip;
int get_smip_plat_config(int offset)
{
	if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
		INTEL_MID_BOARD(1, TABLET, MRFL)) {
		if (!is_mapped) {
			smip = ioremap_nocache(MRFL_SMIP_SRAM_ADDR +
				MRFL_PLATFORM_CONFIG_OFFSET, 8);
			is_mapped = true;
		}
		return ioread8(smip + offset);
	}
	return -EINVAL;
}

static void init_callbacks(struct bq27x00_platform_data *pdata)
{
	if (INTEL_MID_BOARD(1, PHONE, MFLD) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, YKB, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, YKB, PRO)) {
		/* MFLD Phones and Yukka beach Tablet */
		pdata->battery_present =
					intel_msic_check_battery_present;
		pdata->battery_health = intel_msic_check_battery_health;
		pdata->battery_status = intel_msic_check_battery_status;
		pdata->battery_pack_temp =
					intel_msic_get_battery_pack_temp;
		pdata->get_dffs_list = get_dffs_list;
		pdata->get_dffs_data = get_dffs_data;
		pdata->get_curr_dffs_by_oemstr = get_curr_dffs_by_oemstr;
		pdata->get_vmin_threshold = get_vmin_threshold;
		pdata->get_vmax_threshold = get_vmax_threshold;
		pdata->is_cap_shutdown_enabled =
					intel_msic_is_capacity_shutdown_en;
		pdata->is_volt_shutdown_enabled =
					intel_msic_is_volt_shutdown_en;
		pdata->is_lowbatt_shutdown_enabled =
					intel_msic_is_lowbatt_shutdown_en;
		pdata->get_vmin_threshold = intel_msic_get_vsys_min;
		pr_info("%s:__1__\n",__func__);
	} else if (INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO)) {
		/* MFLD  Redridge and Salitpa Tablets */
		pdata->get_dffs_list = get_dffs_list;
		pdata->get_dffs_data = get_dffs_data;
		pdata->get_curr_dffs_by_oemstr = get_curr_dffs_by_oemstr;
		pdata->get_vmin_threshold = get_vmin_threshold;
		pdata->get_vmax_threshold = get_vmax_threshold;
		pdata->battery_status = smb347_get_charging_status;
		pr_info("%s:__2__\n",__func__);
	} else if (INTEL_MID_BOARD(1, PHONE, CLVTP)
			|| INTEL_MID_BOARD(1, TABLET, CLVT)) {
		/* CLTP Phones and tablets */
		pdata->get_dffs_list = get_dffs_list;
		pdata->get_dffs_data = get_dffs_data;
		pdata->get_curr_dffs_by_oemstr = get_curr_dffs_by_oemstr;
		pdata->get_vmin_threshold = get_vmin_threshold;
		pdata->get_vmax_threshold = get_vmax_threshold;
		pdata->battery_health = bq24192_get_battery_health;
		pdata->reset_chip = true;
		pdata->battery_pack_temp = ctp_get_battery_temp;
		pdata->is_volt_shutdown_enabled = ctp_is_volt_shutdown_enabled;
		pr_info("%s:__3__\n",__func__);
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL)
			|| INTEL_MID_BOARD(1, TABLET, MRFL)) {
		/* MRFL Phones and tablets*/
		pdata->battery_health = mrfl_get_bat_health;
		pdata->battery_pack_temp = pmic_get_battery_pack_temp;
		pdata->get_vmin_threshold = mrfl_get_vsys_min;
		pdata->get_vmax_threshold = mrfl_get_volt_max;
		pr_info("%s:__4__\n",__func__);
	} else if (intel_mid_identify_cpu() ==
				INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		pdata->battery_status = smb347_get_charging_status;
		pdata->get_vmin_threshold = byt_get_vsys_min;
		pdata->reset_chip = true;
		pdata->temp_min_lim = 0;
		pdata->temp_max_lim = 55;
		pdata->volt_min_lim = 3400;
		pdata->volt_max_lim = 4350;
		pr_info("%s:__5__\n",__func__);
	}

	pdata->reset_i2c_lines = bq27x00_i2c_reset_workaround;
}

static void init_platform_params(struct bq27x00_platform_data *pdata)
{
	if (INTEL_MID_BOARD(1, PHONE, MFLD)) {
		/* MFLD phones */
		if (!(INTEL_MID_BOARD(2, PHONE, MFLD, LEX, ENG)) ||
			!(INTEL_MID_BOARD(2, PHONE, MFLD, LEX, PRO)))
		if (msic_battery_check(pdata)) {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		} else {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		}
		pr_info("%s:__1__\n",__func__);
	} else if (INTEL_MID_BOARD(2, TABLET, MFLD, YKB, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, YKB, PRO)) {
		/* Yukka beach Tablet */
		if (msic_battery_check(pdata)) {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		} else {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		}
		pr_info("%s:__2__\n",__func__);
	} else if (INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG) ||
			INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO)) {
		pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		pr_info("%s:__3__\n",__func__);
	} else if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
				INTEL_MID_BOARD(1, TABLET, CLVT)) {
		if (msic_battery_check(pdata)) {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
			pdata->file_sys_storage_enabled = 1;
			pdata->soc_intr_mode_enabled = true;
		}
		pr_info("%s:__4__\n",__func__);
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
				INTEL_MID_BOARD(1, TABLET, MRFL)) {
		if (msic_battery_check(pdata)) {
			pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
			pdata->file_sys_storage_enabled = 1;
			pdata->soc_intr_mode_enabled = true;
		}
		pr_info("%s:__5__\n",__func__);
	} else if (intel_mid_identify_cpu() ==
				INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		char byt_t_ffrd8_batt_str[] = "INTN0001";
		pdata->technology = POWER_SUPPLY_TECHNOLOGY_LION;
		pdata->file_sys_storage_enabled = 1;
		pdata->soc_intr_mode_enabled = true;
		snprintf(pdata->battid, (BATTID_LEN + 1),
					"%s", byt_t_ffrd8_batt_str);
		snprintf(pdata->model_name, (MODEL_NAME_LEN + 1),
					"%s", pdata->battid);
		snprintf(pdata->serial_num, (SERIAL_NUM_LEN + 1), "%s",
				pdata->battid + MODEL_NAME_LEN);
		pr_info("%s:__6__\n",__func__);
	}

	pdata->is_init_done = 0;
}

static void init_platform_thresholds(struct bq27x00_platform_data *pdata)
{
	u8 shutdown_method;
	if (INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO)) {
		pdata->temp_min_lim = 0;
		pdata->temp_max_lim = 60;
		pdata->volt_min_lim = 3200;
		pdata->volt_max_lim = 4300;
		pr_info("%s:__1__\n",__func__);
	} else if (INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO)) {
		pdata->temp_min_lim = 0;
		pdata->temp_max_lim = 45;
		pdata->volt_min_lim = 3200;
		pdata->volt_max_lim = 4350;
		pr_info("%s:__2__\n",__func__);
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
			INTEL_MID_BOARD(1, TABLET, MRFL)) {
		/* Bit 1 of shutdown method determines if voltage based
		 * shutdown in enabled.
		 * Bit 3 specifies if capacity for NFC should be reserved.
		 * Reserve capacity only if Bit 3 of shutdown method
		 * is enabled.
		 */
		shutdown_method =
			get_smip_plat_config(MRFL_SMIP_SHUTDOWN_OFFSET);
		if (shutdown_method & MRFL_NFC_RESV_MASK)
			pdata->resv_cap =
				get_smip_plat_config
					(MRFL_SMIP_RESV_CAP_OFFSET);

		pdata->is_volt_shutdown = (shutdown_method &
			MRFL_VOLT_SHUTDOWN_MASK) ? 1 : 0;
		pr_info("%s:__3__\n",__func__);
	}
}

void *bq27x00_platform_data(void *info)
{
	static struct bq27x00_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	//int intr = get_gpio_by_name("fg_alert");
	int ret,intr;
	ret = gpio_request(52,"fg_alert");
	if(ret){
		sprintf("%s:can't get fg_alert_irq_gpio:52",__func__);
		gpio_free(52);
		//return -EINVAL;
	}
	intr = 52;
	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	if (intel_mid_identify_cpu() ==
				INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		intr = acpi_get_gpio("\\_SB.GPO2", 0x12);
		intr =  52; /* GPIO_S5_18  = SUS0_18. SUS0_0 = 130 */
		i2c_info->irq = gpio_to_irq(intr);
	}

	init_callbacks(&platform_data);
	init_platform_params(&platform_data);
	init_platform_thresholds(&platform_data);

	if (smip)
		iounmap(smip);
	return &platform_data;
}
