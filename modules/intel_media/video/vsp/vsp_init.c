/**
 * file vsp_init.c
 * VSP initialization and firmware upload
 * Author: Binglin Chen <binglin.chen@intel.com>
 *
 */

/**************************************************************************
 *
 * Copyright (c) 2007 Intel Corporation, Hillsboro, OR, USA
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#include <linux/firmware.h>

#include <drm/drmP.h>
#include <drm/drm.h>

#include "vsp.h"

#define FW_SZ (2 * 1024 * 1024)

#define FW_NAME "vsp_vpp_vp8.bin"
#define FW_NAME_B0 "vsp_vpp_vp8_b0.bin"
#define FW_NAME_ANN "ann_signed_vsp_a0key0.bin"

#ifdef CONFIG_DX_SEP54
extern int sepapp_image_verify(u8 *addr, ssize_t size, u32 key_index, u32 magic_num);
#endif

static ssize_t psb_vsp_pmstate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	int ret = -EINVAL;

	if (drm_dev == NULL)
		return 0;

	ret = snprintf(buf, 64, "VSP Power state 0x%s\n",
			ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND)
			? "ON" : "OFF");

	return ret;
}
static DEVICE_ATTR(vsp_pmstate, 0444, psb_vsp_pmstate_show, NULL);

int vsp_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ttm_bo_device *bdev = &dev_priv->bdev;
	struct vsp_private *vsp_priv;
	bool is_iomem;
	int ret;
	unsigned int context_size;

	VSP_DEBUG("init vsp private data structure\n");
	vsp_priv = kmalloc(sizeof(struct vsp_private), GFP_KERNEL);
	if (vsp_priv == NULL)
		return -1;

	memset(vsp_priv, 0, sizeof(*vsp_priv));

	/* get device --> drm_device --> drm_psb_private --> vsp_priv
	 * for psb_vsp_pmstate_show: vsp_pmpolicy
	 * if not pci_set_drvdata, can't get drm_device from device
	 */
	/* pci_set_drvdata(dev->pdev, dev); */
	if (device_create_file(&dev->pdev->dev,
			       &dev_attr_vsp_pmstate))
		DRM_ERROR("TOPAZ: could not create sysfs file\n");

	vsp_priv->sysfs_pmstate = sysfs_get_dirent(
		dev->pdev->dev.kobj.sd, NULL,
		"vsp_pmstate");

	vsp_priv->vsp_cmd_num = 0;
	vsp_priv->fw_loaded = VSP_FW_NONE;
	vsp_priv->current_sequence = 0;
	vsp_priv->vsp_state = VSP_STATE_DOWN;
	vsp_priv->dev = dev;
	vsp_priv->coded_buf = NULL;

	vsp_priv->context_num = 0;
	atomic_set(&dev_priv->vsp_mmu_invaldc, 0);

	dev_priv->vsp_private = vsp_priv;

	vsp_priv->cmd_queue_sz = VSP_CMD_QUEUE_SIZE *
		sizeof(struct vss_command_t);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->cmd_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->cmd_queue_bo);
#else
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->cmd_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, NULL, &vsp_priv->cmd_queue_bo);
#endif
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP cmd queue\n");
		goto out_clean;
	}

	vsp_priv->ack_queue_sz = VSP_ACK_QUEUE_SIZE *
		sizeof(struct vss_response_t);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->ack_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->ack_queue_bo);
#else
	ret = ttm_buffer_object_create(bdev,
				       vsp_priv->ack_queue_sz,
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, NULL, &vsp_priv->ack_queue_bo);
#endif
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP cmd ack queue\n");
		goto out_clean;
	}

	/* Create setting buffer */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	ret =  ttm_buffer_object_create(bdev,
				       sizeof(struct vsp_settings_t),
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, 0, NULL, &vsp_priv->setting_bo);
#else
	ret =  ttm_buffer_object_create(bdev,
				       sizeof(struct vsp_settings_t),
				       ttm_bo_type_kernel,
				       DRM_PSB_FLAG_MEM_MMU |
				       TTM_PL_FLAG_NO_EVICT,
				       0, 0, NULL, &vsp_priv->setting_bo);
#endif
	if (ret != 0) {
		DRM_ERROR("VSP: failed to allocate VSP setting buffer\n");
		goto out_clean;
	}

	/* map cmd queue */
	ret = ttm_bo_kmap(vsp_priv->cmd_queue_bo, 0,
			  vsp_priv->cmd_queue_bo->num_pages,
			  &vsp_priv->cmd_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->cmd_queue_bo);
		ttm_bo_kunmap(&vsp_priv->cmd_kmap);
		goto out_clean;
	}

	vsp_priv->cmd_queue = ttm_kmap_obj_virtual(&vsp_priv->cmd_kmap,
						   &is_iomem);


	/* map ack queue */
	ret = ttm_bo_kmap(vsp_priv->ack_queue_bo, 0,
			  vsp_priv->ack_queue_bo->num_pages,
			  &vsp_priv->ack_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->ack_queue_bo);
		ttm_bo_kunmap(&vsp_priv->ack_kmap);
		goto out_clean;
	}

	vsp_priv->ack_queue = ttm_kmap_obj_virtual(&vsp_priv->ack_kmap,
						   &is_iomem);

	/* map vsp setting */
	ret = ttm_bo_kmap(vsp_priv->setting_bo, 0,
			  vsp_priv->setting_bo->num_pages,
			  &vsp_priv->setting_kmap);
	if (ret) {
		DRM_ERROR("drm_bo_kmap setting_bo failed: %d\n", ret);
		ttm_bo_unref(&vsp_priv->setting_bo);
		ttm_bo_kunmap(&vsp_priv->setting_kmap);
		goto out_clean;
	}
	vsp_priv->setting = ttm_kmap_obj_virtual(&vsp_priv->setting_kmap,
						 &is_iomem);

	vsp_priv->vp8_filp[0] = NULL;
	vsp_priv->vp8_filp[1] = NULL;
	vsp_priv->context_vp8_num = 0;

	vsp_priv->vp8_cmd_num = 0;

	spin_lock_init(&vsp_priv->lock);
	mutex_init(&vsp_priv->vsp_mutex);

	INIT_DELAYED_WORK(&vsp_priv->vsp_suspend_wq,
			&psb_powerdown_vsp);
	INIT_DELAYED_WORK(&vsp_priv->vsp_irq_wq,
			&vsp_irq_task);

	return 0;
out_clean:
	vsp_deinit(dev);
	return -1;
}

int vsp_deinit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	VSP_DEBUG("free VSP firmware/context buffer\n");

	if (vsp_priv->cmd_queue) {
		ttm_bo_kunmap(&vsp_priv->cmd_kmap);
		vsp_priv->cmd_queue = NULL;
	}

	if (vsp_priv->ack_queue) {
		ttm_bo_kunmap(&vsp_priv->ack_kmap);
		vsp_priv->ack_queue = NULL;
	}
	if (vsp_priv->setting) {
		ttm_bo_kunmap(&vsp_priv->setting_kmap);
		vsp_priv->setting = NULL;
	}

	if (vsp_priv->ack_queue_bo)
		ttm_bo_unref(&vsp_priv->ack_queue_bo);
	if (vsp_priv->cmd_queue_bo)
		ttm_bo_unref(&vsp_priv->cmd_queue_bo);
	if (vsp_priv->setting_bo)
		ttm_bo_unref(&vsp_priv->setting_bo);

	device_remove_file(&dev->pdev->dev, &dev_attr_vsp_pmstate);
	sysfs_put(vsp_priv->sysfs_pmstate);

	VSP_DEBUG("free VSP private structure\n");
	kfree(dev_priv->vsp_private);

	return 0;
}

void vsp_enableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned int mask;
	unsigned int enable;
	unsigned int clear;

	VSP_DEBUG("will enable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);
	clear = 0;

	VSP_SET_FLAG(mask, VSP_SP0_IRQ_SHIFT);
	VSP_SET_FLAG(enable, VSP_SP0_IRQ_SHIFT);
	VSP_SET_FLAG(clear, VSP_SP0_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_EDGE, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_CLR, clear);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
	/* use the Level type interrupt */
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_LEVEL_PULSE, 0x80);
}

void vsp_disableirq(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	unsigned int mask, enable;

	VSP_DEBUG("will disable irq\n");

	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_MASK, &mask);
	IRQ_REG_READ32(VSP_IRQ_CTRL_IRQ_ENB, &enable);

	VSP_CLEAR_FLAG(mask, VSP_SP0_IRQ_SHIFT);
	VSP_CLEAR_FLAG(enable, VSP_SP0_IRQ_SHIFT);

	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_MASK, mask);
	IRQ_REG_WRITE32(VSP_IRQ_CTRL_IRQ_ENB, enable);
	return;
}

extern struct soft_platform_id spid;

int vsp_init_fw(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int ret = 0;
	const struct firmware *raw;
	unsigned char *ptr, *ma_ptr;
	struct vsp_secure_boot_header *boot_header;
	struct vsp_multi_app_blob_data *ma_header;
	unsigned long imr_addr;
	int imr_size;
	unsigned char *imr_ptr;
	unsigned int vrl_header_size = 736;
	const unsigned long vsp_magic_num = 0x50535624;
	const int FW_NAME_LEN = 30;
	char fw_name[FW_NAME_LEN];
	int name_ret;

	PSB_DEBUG_GENERAL("read firmware into buffer\n");

	name_ret = snprintf(fw_name, FW_NAME_LEN, "vsp.bin.%04x.%04x",
		 (int)spid.platform_family_id, (int)spid.hardware_id);
	if (name_ret > FW_NAME_LEN) {
		DRM_ERROR("failed to get fw name, ret %d vs expect %d\n",
			  name_ret, FW_NAME_LEN);
		/* no way */
		return -1;
	}

	/* try load with spid first */
	ret = request_firmware(&raw, fw_name, &dev->pdev->dev);
	if (ret < 0 || raw == NULL) {
		VSP_DEBUG("failed to load fw: %s, try to load different fw\n",
			fw_name);

		/* read firmware img */
		VSP_DEBUG("load vsp fw\n");
		if (IS_ANN_A0(dev))
			ret = request_firmware(&raw, FW_NAME_ANN,
					       &dev->pdev->dev);
		else if (IS_TNG_B0(dev))
			ret = request_firmware(&raw, FW_NAME_B0,
					       &dev->pdev->dev);
		else {
			DRM_ERROR("VSP secure fw: bad platform\n");
			raw = NULL;
		}
	}

	if (ret < 0 || raw == NULL) {
		DRM_ERROR("VSP: request_firmware failed: reason %d\n", ret);
		return -1;
	}

	if (raw->size < sizeof(struct vsp_secure_boot_header)) {
		DRM_ERROR("VSP: %s is not a correct firmware (size %d)\n",
				FW_NAME, raw->size);
		ret = -1;
		goto out;
	}

	ptr = (void *)raw->data;
	ma_ptr = (void *) raw->data + vrl_header_size;
	boot_header = (struct vsp_secure_boot_header *)
		(ptr + vrl_header_size);
	ma_header = (struct vsp_multi_app_blob_data *)
		(ma_ptr + boot_header->ma_header_offset);

	/* get firmware header */
	memcpy(&vsp_priv->boot_header, boot_header, sizeof(vsp_priv->boot_header));

	if (vsp_priv->boot_header.magic_number != VSP_SECURE_BOOT_MAGIC_NR) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->boot_header.magic_number,
			  VSP_SECURE_BOOT_MAGIC_NR);
		ret = -1;
		goto out;
	}

	/* read application firmware image data (for state-buffer size, etc) */
	/* load the multi-app blob header */
	memcpy(&vsp_priv->ma_header, ma_header, sizeof(vsp_priv->ma_header));
	if (vsp_priv->ma_header.magic_number != VSP_MULTI_APP_MAGIC_NR) {
		DRM_ERROR("VSP: failed to load correct vsp firmware\n"
			  "FW magic number is wrong %x (should be %x)\n",
			  vsp_priv->ma_header.magic_number,
			  VSP_MULTI_APP_MAGIC_NR);
		ret = -1;
		goto out;
	}

	VSP_DEBUG("firmware secure header:\n");
	VSP_DEBUG("boot_header magic number %x\n", boot_header->magic_number);
	VSP_DEBUG("boot_text_offset %x\n", boot_header->boot_text_offset);
	VSP_DEBUG("boot_text_reg %x\n", boot_header->boot_text_reg);
	VSP_DEBUG("boot_icache_value %x\n", boot_header->boot_icache_value);
	VSP_DEBUG("boot_icache_reg %x\n", boot_header->boot_icache_reg);
	VSP_DEBUG("boot_pc_value %x\n", boot_header->boot_pc_value);
	VSP_DEBUG("boot_pc_reg %x\n", boot_header->boot_pc_reg);
	VSP_DEBUG("ma_header_offset %x\n", boot_header->ma_header_offset);
	VSP_DEBUG("ma_header_reg %x\n", boot_header->ma_header_reg);
	VSP_DEBUG("boot_start_value %x\n", boot_header->boot_start_value);
	VSP_DEBUG("boot_start_reg %x\n", boot_header->boot_start_reg);
	VSP_DEBUG("firmware ma_blob header:\n");
	VSP_DEBUG("ma_header magic number %x\n", ma_header->magic_number);
	VSP_DEBUG("offset_from_start %x\n", ma_header->offset_from_start);
	VSP_DEBUG("imr_state_buffer_addr %x\n", ma_header->imr_state_buffer_addr);
	VSP_DEBUG("imr_state_buffer_size %x\n", ma_header->imr_state_buffer_size);
	VSP_DEBUG("apps_default_context_buffer_size %x\n",
			ma_header->apps_default_context_buffer_size);

	/* get imr 11 region start address and size */
	imr_addr = intel_mid_msgbus_read32(PNW_IMR_MSG_PORT,
					   TNG_IMR11L_MSG_REGADDR);
	imr_addr <<= 10;
	VSP_DEBUG("IMR11 base address %p\n", imr_addr);

	imr_size = raw->size;

	/* map imr 11 */
	/* check if raw size is smaller than */
	/* ioremap the region */
	imr_ptr = ioremap(imr_addr, imr_size);
	if (!imr_ptr) {
		DRM_ERROR("failed to map imr_addr\n");
		ret = -1;
		goto out;
	}

	/* copy the firmware to imr 11 */
	memcpy(imr_ptr, ptr, raw->size);

	/* unmap the region */
	iounmap(imr_ptr);
#ifdef CONFIG_DX_SEP54
	ret = sepapp_image_verify(imr_addr, imr_size, 15, vsp_magic_num);
	if (ret) {
		DRM_ERROR("failed to verify vsp firmware, ret %x\n", ret);
		ret = -1;
		goto out;
	}
#endif

	vsp_priv->fw_loaded = VSP_FW_LOADED;
	vsp_priv->vsp_state = VSP_STATE_DOWN;

	vsp_priv->ctrl = (struct vsp_ctrl_reg *) (dev_priv->vsp_reg +
						  VSP_CONFIG_REG_SDRAM_BASE +
						  VSP_CONFIG_REG_START);
out:
	release_firmware(raw);

	return ret;
}

int vsp_reset(struct drm_psb_private *dev_priv)
{
	int ret;

	ret = vsp_setup_fw(dev_priv);

	return ret;
}

int vsp_setup_fw(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	uint32_t pd_addr;

	/* set MMU */
	pd_addr = psb_get_default_pd_addr(dev_priv->vsp_mmu);
	SET_MMU_PTD(pd_addr >> PAGE_TABLE_SHIFT);
	SET_MMU_PTD(pd_addr >> PAGE_SHIFT);

	/* vsp setting */
	vsp_priv->setting->command_queue_size = VSP_CMD_QUEUE_SIZE;
	vsp_priv->setting->command_queue_addr = vsp_priv->cmd_queue_bo->offset;
	vsp_priv->setting->response_queue_size = VSP_ACK_QUEUE_SIZE;
	vsp_priv->setting->response_queue_addr = vsp_priv->ack_queue_bo->offset;

	vsp_priv->ctrl->setting_addr = vsp_priv->setting_bo->offset;
	vsp_priv->ctrl->mmu_tlb_soft_invalidate = 0;
	vsp_priv->ctrl->cmd_rd = 0;
	vsp_priv->ctrl->cmd_wr = 0;
	vsp_priv->ctrl->ack_rd = 0;
	vsp_priv->ctrl->ack_wr = 0;

	VSP_DEBUG("setup firmware\n");

	/* Set power-saving mode */
	if (drm_vsp_pmpolicy == PSB_PMPOLICY_NOPM)
		vsp_priv->ctrl->power_saving_mode = vsp_always_on;
	else if (drm_vsp_pmpolicy == PSB_PMPOLICY_POWERDOWN ||
			drm_vsp_pmpolicy == PSB_PMPOLICY_CLOCKGATING)
		vsp_priv->ctrl->power_saving_mode = vsp_suspend_on_empty_queue;
	else
		vsp_priv->ctrl->power_saving_mode =
			vsp_suspend_and_hw_idle_on_empty_queue;

	/* communicate the type of init
	 * this is the last value to write
	 * it will cause the VSP to read all other settings as wll
	 */
	vsp_priv->ctrl->entry_kind = vsp_entry_init;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;

	/* enable irq */
	psb_irq_preinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);
	psb_irq_postinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);

	return 0;
}

void vsp_start_function(struct drm_psb_private *dev_priv, unsigned int pc,
		    unsigned int processor)
{
	unsigned int reg;

	/* set the start addr */
	SP_REG_WRITE32(VSP_START_PC_REG_OFFSET, pc, processor);

	/* set start command */
	SP_REG_READ32(SP_STAT_AND_CTRL_REG, &reg, processor);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_RUN_FLAG);
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_START_FLAG);
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);
	return;
}
#if 0
unsigned int vsp_set_firmware(struct drm_psb_private *dev_priv,
			      unsigned int processor)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	unsigned int reg = 0;

	/* config icache */
	VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_INVALID_FLAG);
	/* disable ICACHE_PREFETCH_FLAG from v2.3 */
	/* VSP_SET_FLAG(reg, SP_STAT_AND_CTRL_REG_ICACHE_PREFETCH_FLAG); */
	SP_REG_WRITE32(SP_STAT_AND_CTRL_REG, reg, processor);

	/* set icache base address: point to instructions in DDR */
	SP_REG_WRITE32(VSP_ICACHE_BASE_REG_OFFSET,
		       vsp_priv->firmware->offset +
		       vsp_priv->boot_header.boot_text_offset,
		       processor);

	/* write ma_header_address to the variable allocated for it*/
	MM_WRITE32(vsp_priv->boot_header.ma_header_reg,
		   0,
		   vsp_priv->firmware->offset +
		   vsp_priv->boot_header.ma_header_offset);

	/* start the secure boot program */
	vsp_start_function(dev_priv,
			   vsp_priv->boot_header.boot_pc_value,
			   processor);

	return 0;
}
#endif

void vsp_init_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	vsp_priv->ctrl->entry_kind = vsp_entry_init;
}

void vsp_continue_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	vsp_priv->ctrl->entry_kind = vsp_entry_booted;

	vsp_priv->ctrl->entry_kind = vsp_entry_resume;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;
}

int vsp_resume_function(struct drm_psb_private *dev_priv)
{
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	struct pci_dev *pdev = vsp_priv->dev->pdev;
	uint32_t pd_addr, mmadr;

	/* FIXME, change should be removed once bz 120324 is fixed */
	pci_read_config_dword(pdev, 0x10, &mmadr);
	if (mmadr == 0) {
		DRM_ERROR("Bad PCI config!\n");
		return -1;
	}

	vsp_priv->ctrl = (struct vsp_ctrl_reg *) (dev_priv->vsp_reg +
						  VSP_CONFIG_REG_SDRAM_BASE +
						  VSP_CONFIG_REG_START);

	/* Set MMU */
	pd_addr = psb_get_default_pd_addr(dev_priv->vsp_mmu);
	SET_MMU_PTD(pd_addr >> PAGE_TABLE_SHIFT);
	SET_MMU_PTD(pd_addr >> PAGE_SHIFT);

	/* enable irq */
	psb_irq_preinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);
	psb_irq_postinstall_islands(dev_priv->dev, OSPM_VIDEO_VPP_ISLAND);

	/* restore the config regs */
	CONFIG_REG_WRITE32(VSP_SETTING_ADDR_REG,
			vsp_priv->saved_config_regs[VSP_SETTING_ADDR_REG]);
	CONFIG_REG_WRITE32(VSP_POWER_SAVING_MODE_REG,
			vsp_priv->saved_config_regs[VSP_POWER_SAVING_MODE_REG]);
	CONFIG_REG_WRITE32(VSP_CMD_QUEUE_RD_REG,
			vsp_priv->saved_config_regs[VSP_CMD_QUEUE_RD_REG]);
	CONFIG_REG_WRITE32(VSP_CMD_QUEUE_WR_REG,
			vsp_priv->saved_config_regs[VSP_CMD_QUEUE_WR_REG]);
	CONFIG_REG_WRITE32(VSP_ACK_QUEUE_RD_REG,
			vsp_priv->saved_config_regs[VSP_ACK_QUEUE_RD_REG]);
	CONFIG_REG_WRITE32(VSP_ACK_QUEUE_WR_REG,
			vsp_priv->saved_config_regs[VSP_ACK_QUEUE_WR_REG]);

	vsp_priv->ctrl->entry_kind = vsp_entry_resume;

	vsp_priv->vsp_state = VSP_STATE_ACTIVE;

	return 0;
}

