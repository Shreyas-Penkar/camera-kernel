/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_COMPAT_H_
#define _CAM_COMPAT_H_

#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/iommu.h>
#include <linux/qcom_scm.h>
#include <linux/list_sort.h>
#include <soc/qcom/of_common.h>
#include <linux/spi/spi.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#include <linux/dma-iommu.h>
#endif

#include "cam_csiphy_dev.h"
#include "cam_cpastop_hw.h"
#include "cam_smmu_api.h"
#include "cam_sync_synx.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
#include <linux/ion.h>
#include <linux/msm_ion.h>
#define VFL_TYPE_VIDEO VFL_TYPE_GRABBER
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
#include <linux/qcom-dma-mapping.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
#ifdef CONFIG_SECURE_CAMERA_V3
#undef CONFIG_SECURE_CAMERA_V3
#endif
#endif

#ifdef CONFIG_SECURE_CAMERA_V3
#include <soc/qcom/smci_clientenv.h>
#include <soc/qcom/smci_opener.h>
#include <soc/qcom/ctrusted_camera_driver.h>
#include <soc/qcom/trusted_camera_driver.h>
#endif

#ifdef CONFIG_TZ_DCP_API_VER_2
#include <linux/smci_clientenv.h>
#include <smci/interface/smci_opener.h>
#include <linux/ctrusted_camera_driver.h>
#include <linux/trusted_camera_driver.h>
#endif

#ifdef CONFIG_SECURE_CAMERA_25
#include <smmu-proxy/linux/qti-smmu-proxy.h>
#endif

#if KERNEL_VERSION(6, 0, 0) <= LINUX_VERSION_CODE
#include <linux/qcom-dma-mapping.h>
#endif

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
MODULE_IMPORT_NS(DMA_BUF);
#endif

#define IS_CSF25(x, y) ((((x) == 2) && ((y) == 5)) ? 1 : 0)

struct cam_fw_alloc_info {
	struct device *fw_dev;
	void          *fw_kva;
	uint64_t       fw_hdl;
};

/* Unblock compilation if target kernel does not support camnoc reg update through HYP */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#define QCOM_SCM_CAMERA_MAX_QOS_CNT 20
struct qcom_scm_camera_qos {
	u32 offset;
	u32 val;
};
#endif

int cam_reserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length);
void cam_unreserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length);
void cam_cpastop_scm_write(struct cam_cpas_hw_errata_wa *errata_wa);
int cam_update_camnoc_qos_settings(uint32_t use_case_id,
	uint32_t qos_cnt, struct qcom_scm_camera_qos *scm_buf);
int cam_ife_notify_safe_lut_scm(bool safe_trigger);
int camera_component_match_add_drivers(struct device *master_dev,
	struct component_match **match_list);
int cam_csiphy_notify_secure_mode(struct csiphy_device *csiphy_dev,
	bool protect, int32_t offset, bool is_shutdown);
void cam_free_clear(const void *);
void cam_check_iommu_faults(struct iommu_domain *domain,
	struct cam_smmu_pf_info *pf_info);
static inline int cam_get_ddr_type(void) { return of_fdt_get_ddrtype(); }
int cam_compat_util_get_dmabuf_va(struct dma_buf *dmabuf, uintptr_t *vaddr);
void cam_compat_util_put_dmabuf_va(struct dma_buf *dmabuf, void *vaddr);
void cam_smmu_util_iommu_custom(struct device *dev,
	dma_addr_t discard_start, size_t discard_length);

#if defined CONFIG_SECURE_CAMERA_V3 || defined CONFIG_TZ_DCP_API_VER_2
int cam_isp_notify_secure_unsecure_port(struct port_info *sec_unsec_port_info);
#endif

#ifdef CONFIG_SECURE_CAMERA_V3
int32_t cam_convert_hw_id_to_secure_hw_type(uint32_t hw_id);
#endif

#ifdef CONFIG_TZ_DCP_API_VER_2
int cam_convert_hw_idx_to_ife_hw_type(int hw_idx,
	uint32_t num_ife, uint32_t num_ife_lite);
int32_t cam_convert_hw_id_to_secure_cam_hw_type(uint32_t hw_id);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
int cam_req_mgr_ordered_list_cmp(void *priv,
	const struct list_head *head_1, const struct list_head *head_2);
#else
int cam_req_mgr_ordered_list_cmp(void *priv,
	struct list_head *head_1, struct list_head *head_2);
#endif

int cam_compat_util_get_irq(struct cam_hw_soc_info *soc_info);
struct file *cam_fcheck_files(struct files_struct *files, uint32_t fd);
void cam_close_fd(struct files_struct *files, uint32_t fd);
int cam_atomic_add_unless (struct file *file);

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
void cam_eeprom_spi_driver_remove(struct spi_device *sdev);
#else
int cam_eeprom_spi_driver_remove(struct spi_device *sdev);
#endif

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
void cam_actuator_driver_i2c_remove(struct i2c_client *client);
void cam_eeprom_i2c_driver_remove(struct i2c_client *client);
void cam_flash_i2c_driver_remove(struct i2c_client *client);
void cam_ois_i2c_driver_remove(struct i2c_client *client);
void cam_sensor_i2c_driver_remove(struct i2c_client *client);
#else
int cam_actuator_driver_i2c_remove(struct i2c_client *client);
int cam_eeprom_i2c_driver_remove(struct i2c_client *client);
int cam_flash_i2c_driver_remove(struct i2c_client *client);
int cam_ois_i2c_driver_remove(struct i2c_client *client);
int cam_sensor_i2c_driver_remove(struct i2c_client *client);
#endif

int cam_smmu_fetch_csf_version(struct cam_csf_version *csf_version);

unsigned long cam_update_dma_map_attributes(unsigned long attr);

size_t cam_align_dma_buf_size(size_t len);

int cam_mem_buf_dma_buf_get_memparcel_hdl(struct dma_buf *dmabuf,
	uint32_t *smmu_proxy_buf_hdl, struct cam_csf_version *csf_version);

int cam_synx_enable_resources(uint32_t client_idx, uint32_t signal_id, bool enable);

#endif /* _CAM_COMPAT_H_ */
