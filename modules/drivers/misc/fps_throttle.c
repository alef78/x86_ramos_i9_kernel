/*
 * Support for camera module fps thermal throttling.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>

#define STATE_NUM 4

static int cur_state = 0;
static int state_list[STATE_NUM] = {100, 75, 50, 25};
static struct kobject *adapter_kobj;

struct thermal_cooling_device *tcd_fps;
struct adapter_attr {
	struct attribute attr;
	int value;
};

static struct adapter_attr notify = {
	.attr.name="notify",
	.attr.mode = 0664,
	.value = 100,
};

static struct adapter_attr handshake = {
	.attr.name="handshake",
	.attr.mode = 0664,
	.value = 0,
};

static struct attribute * throttle_attr[] = {
	&notify.attr,
	&handshake.attr,
	NULL
};

static void set_fps_scaling(int fs) {
	notify.value = fs;
	handshake.value = 0;
	sysfs_notify(adapter_kobj, NULL, "notify");
}

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct adapter_attr *a = container_of(attr, struct adapter_attr, attr);
	return scnprintf(buf, PAGE_SIZE, "%d\n", a->value);
}

static ssize_t store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	struct adapter_attr *a = container_of(attr, struct adapter_attr, attr);
	sscanf(buf, "%d", &a->value);

	if(strcmp(a->attr.name, notify.attr.name) == 0)
		set_fps_scaling(a->value);
	else
		handshake.value = a->value;

	return len;
}

static struct sysfs_ops throttle_ops = {
	.show = show,
	.store = store,
};

static struct kobj_type throttle_type = {
	.sysfs_ops = &throttle_ops,
	.default_attrs = throttle_attr,
};

static int thermal_get_max_fps(struct thermal_cooling_device *tcd, unsigned
		long *pms)
{
	*pms = STATE_NUM;
	return 0;
}

static int thermal_get_cur_fps(struct thermal_cooling_device *tcd, unsigned
		long *pcs)
{
	*pcs = cur_state;
	return 0;
}

static int thermal_set_cur_fps(struct thermal_cooling_device *tcd, unsigned
		long pcs)
{
	int ret;
	if (pcs >= STATE_NUM || pcs < 0)
		return -EINVAL;
	set_fps_scaling(state_list[(int)pcs]);
	//Wait the fps seting by HAL.
	wait_queue_head_t wait;
	init_waitqueue_head (&wait);
	ret = wait_event_interruptible_timeout(wait, (handshake.value == 1), 500);
	if (ret > 0 && handshake.value == 1) {
		cur_state = (int)pcs;
		handshake.value = 0;
	} else {
		//fps change request is not replied by camera HAL, remain the value as
		//the previous one.
		notify.value = state_list[cur_state];
	}

	return ret;
}

static int thermal_set_force_state_override(struct thermal_cooling_device *tcd,
		char *fps_state)
{
	sscanf(fps_state, "%d %d %d %d\n", &state_list[0],
			 &state_list[1],
			 &state_list[2],
			 &state_list[3]);
	return 0;
}

static int thermal_get_force_state_override(struct thermal_cooling_device *tcd,
		char *fps_state)
{
	return sprintf(fps_state, "%d %d %d %d\n", state_list[0], state_list[1],
			state_list[2], state_list[3]);
}

static int thermal_get_available_states(struct thermal_cooling_device *tcd,
		char *fps_state)
{
	return sprintf(fps_state, "%d %d %d %d\n", state_list[0], state_list[1],
			state_list[2], state_list[3]);
}

static const struct thermal_cooling_device_ops thermal_fps_ops = {
	.get_max_state = thermal_get_max_fps,
	.get_cur_state = thermal_get_cur_fps,
	.set_cur_state = thermal_set_cur_fps,
	.set_force_state_override = thermal_set_force_state_override,
	.get_force_state_override = thermal_get_force_state_override,
	.get_available_states = thermal_get_available_states,
};

static int thermal_adapter_init(void)
{
	int err = 0;

	adapter_kobj = kzalloc(sizeof(*adapter_kobj), GFP_KERNEL);
	if (!adapter_kobj)
		return -ENOMEM;
	kobject_init(adapter_kobj, &throttle_type);
	if (kobject_add(adapter_kobj, NULL, "%s", "fps_throttle")) {
		err = -1;
		goto adapter_failed;
	}

	tcd_fps = thermal_cooling_device_register("CameraFps", NULL,
			&thermal_fps_ops);
	if (IS_ERR(tcd_fps)) {
		err = PTR_ERR(tcd_fps);
		goto thermal_failed;
	}
	return err;

thermal_failed:
	kobject_del(adapter_kobj);
adapter_failed:
	kfree(adapter_kobj);
	adapter_kobj = NULL;
	return err;
}

static void thermal_adapter_exit(void)
{
	kobject_del(adapter_kobj);
	kfree(adapter_kobj);
	adapter_kobj = NULL;

	thermal_cooling_device_unregister(tcd_fps);
	tcd_fps = NULL;
}

static int __init fps_thermal_adapter_init(void)
{
	return thermal_adapter_init();
}

static void __exit fps_thermal_adapter_exit(void)
{
	thermal_adapter_exit();
}

module_init(fps_thermal_adapter_init);
module_exit(fps_thermal_adapter_exit);

MODULE_AUTHOR("Zhu,Shaoping <shaopingx.zhu@intel.com>");
MODULE_DESCRIPTION("FPS throttling thermal adapter device driver");
MODULE_LICENSE("GPL");
