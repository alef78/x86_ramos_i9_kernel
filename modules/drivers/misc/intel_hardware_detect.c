/*
 * drivers/misc/intel_hardware_detect.c
 *
 * Copyright (C) 2011 Intel Corp
 * Author: winson.w.yung@intel.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/intel_mid_hwid.h>

#define HARDWARE_ID1_GPIO157 157
#define HARDWARE_ID2_GPIO158 158
#define HARDWARE_ID3_GPIO159 159
#define HARDWARE_ID4_GPIO160 160
#define PANEL_ID1_GP191 191
#define PANEL_ID2_GP178 178
#define PANEL_ID3_GP58 58

//static struct kobject *hardware_properties;

static void intel_mid_hardware_id_detect(void)
{
	int ret = 0;
	int board_id = 0;
	int panel_id = 0;
	
	ret = gpio_request(HARDWARE_ID1_GPIO157, "hardware_id_detect");
	if (ret) {
		printk(" Faild to request HARDWARE_ID_GPIO157\n");
	}
	ret = gpio_request(HARDWARE_ID2_GPIO158, "hardware_id_detect");
	if (ret) {
		printk(" Faild to request HARDWARE_ID_GPIO158\n");
	}
	ret = gpio_request(HARDWARE_ID3_GPIO159, "hardware_id_detect");
	if (ret) {
		printk(" Faild to request HARDWARE_ID_GPIO159\n");
	}
	ret = gpio_request(HARDWARE_ID4_GPIO160, "hardware_id_detect");
	if (ret) {
		printk(" Faild to request HARDWARE_ID_GPIO160\n");
	}
	ret = gpio_request(PANEL_ID2_GP178, "panel_id_detect");
	if (ret) {
		printk(" Faild to request PANEL_ID_GP178\n");
	}
	ret = gpio_request(PANEL_ID1_GP191, "panel_id_detect");
	if (ret) {
		printk(" Faild to request PANEL_ID_GP191\n");
	}
	ret = gpio_request(PANEL_ID3_GP58, "panel_id_detect");
	if (ret) {
		printk(" Faild to request PANEL_ID_GP58\n");
	}
	gpio_direction_input(HARDWARE_ID1_GPIO157);
	gpio_direction_input(HARDWARE_ID2_GPIO158);
	gpio_direction_input(HARDWARE_ID3_GPIO159);
	gpio_direction_input(HARDWARE_ID4_GPIO160);
	gpio_direction_input(PANEL_ID2_GP178);
	gpio_direction_input(PANEL_ID1_GP191);
	gpio_direction_input(PANEL_ID3_GP58);
	board_id = ((gpio_get_value(HARDWARE_ID2_GPIO158)?1:0) << 1) | \
		(gpio_get_value(HARDWARE_ID1_GPIO157)?1:0);
	panel_id=	((gpio_get_value(PANEL_ID2_GP178)?1:0) << 1) | (gpio_get_value(PANEL_ID1_GP191)?1:0);

	intel_mid_set_board_id(board_id);
	intel_mid_set_panel_id(panel_id);
	printk("hardware_id_str = 0x%x, panel_id_str = 0x%x\n", board_id, panel_id);

	gpio_free(HARDWARE_ID1_GPIO157);
	gpio_free(HARDWARE_ID2_GPIO158);
	gpio_free(HARDWARE_ID3_GPIO159);
	gpio_free(HARDWARE_ID4_GPIO160);
	gpio_free(PANEL_ID2_GP178);
	gpio_free(PANEL_ID1_GP191);
	gpio_free(PANEL_ID3_GP58);
}

#if 0
static ssize_t intel_mid_hardware_id_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	  return sprintf(buf, "%01x\n", hardware_id_detect.hardware_id_num);
}

static ssize_t intel_mid_panel_id_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	  return sprintf(buf, "%01X\n", hardware_id_detect.panel_id_num);
}

static struct kobj_attribute hardware_id_attr = {
	.attr = {
		.name = "board_id",
		.mode = S_IRUGO,
		},
	.show = intel_mid_hardware_id_show,
};

static struct kobj_attribute panel_id_attr = {
	.attr = {
		.name = "panel_id",
		.mode = S_IRUGO,
		},
	.show = intel_mid_panel_id_show,
};

static int __init intel_mid_hardware_detect_init(void)
{
	int ret = 0;

	hardware_properties = kobject_create_and_add("hardware_id", NULL);
	if (!hardware_properties) {
		pr_err("failed to create /sys/hardware_id\n");
		ret = -EINVAL;
	}

	sysfs_create_file(hardware_properties, &hardware_id_attr.attr);
	sysfs_create_file(hardware_properties, &panel_id_attr.attr);
	intel_mid_hardware_id_detect();
	
	return ret;
}
fs_initcall(intel_mid_hardware_detect_init);
#else
fs_initcall(intel_mid_hardware_id_detect);
#endif
