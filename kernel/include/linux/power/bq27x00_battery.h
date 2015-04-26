/*
 * bq27x00_battery.h - Fuel gauge driver for TI bq27xxx
 *
 * Copyright (C) 2013 BYD Electronics
 * Meng Dong <meng.dong@byd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __BQ27x00_BATTERY_H_
#define __BQ27x00_BATTERY_H_

#define BATTID_LEN		8
#define MODEL_NAME_LEN		2
#define SERIAL_NUM_LEN		6

#define MAX_NAME_LEN  20
#define MAX_OEM_STRING_LEN 8

struct bq27x00_dffs_list {
	char name[MAX_NAME_LEN+1];
	char oemstr[MAX_OEM_STRING_LEN+1];
	unsigned short fuel_gauge_type;
	int (*dffs)[10];
} __packed;

struct bq27x00_platform_data {
	bool is_init_done;
	bool is_volt_shutdown;
	bool is_capacity_shutdown;
	bool is_lowbatt_shutdown;
	bool file_sys_storage_enabled;
	bool soc_intr_mode_enabled;
	bool reset_chip;
	int technology;
	char battid[BATTID_LEN + 1];
	char model_name[MODEL_NAME_LEN + 1];
	char serial_num[2*SERIAL_NUM_LEN + 1];

	/* battery safety thresholds */
	int temp_min_lim;	/* in degrees centi grade */
	int temp_max_lim;	/* in degrees centigrade */
	int volt_min_lim;	/* milli volts */
	int volt_max_lim;	/* milli volts */
	int resv_cap;

	int (*battery_present)(void);
	int (*battery_health)(void);
	int (*battery_status)(void);
	int (*battery_pack_temp)(int *);
	void (*get_dffs_list)(char* output_str);
	void* (*get_dffs_data)(char* model, unsigned short fuel_gauge_type);
	void (*get_curr_dffs_by_oemstr)(char* oemstr, char* output_str);
	
	void (*reset_i2c_lines)(void);

	bool (*is_cap_shutdown_enabled)(void);
	bool (*is_volt_shutdown_enabled)(void);
	bool (*is_lowbatt_shutdown_enabled)(void);
	int (*get_vmin_threshold)(void);
	int (*get_vmax_threshold)(void);
};

#endif
