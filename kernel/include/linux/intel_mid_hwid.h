/*
 *  intel_mid_hwid.h - Intel MID Hardware ID Detect
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Author: Wang Xiaojun <wang.xiaojun3@byd.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 */
#ifndef __INTEL_MID_HWID_H__
#define __INTEL_MID_HWID_H__

enum intel_mid_board_id {
	HW_BOARD_8_LTE = 0x0,
	HW_BOARD_8_WIFI = 0x1,
	HW_BOARD_7_LTE = 0x2,
	HW_BOARD_7_WIFI = 0x3,
};

enum intel_mid_7in_panel_id {
	HW_PANEL_7_AUO = 0x1,
	HW_PANEL_7_3rd = 0x2,
	HW_PANEL_7_CPT = 0x3,
};

enum intel_mid_8in_panel_id {
	HW_PANEL_8_AUO_BYD = 0x0,
	HW_PANEL_8_3rd = 0x1,
	HW_PANEL_8_INOX = 0x2,
	HW_PANEL_8_AUO = 0x3,
};

struct hardware_id_table{
	int hardware_id_num;
	int panel_id_num;
};

extern int intel_mid_get_board_id(void);
extern int intel_mid_get_panel_id(void);
extern void intel_mid_set_board_id(int board_id);
extern void intel_mid_set_panel_id(int panel_id);

#endif