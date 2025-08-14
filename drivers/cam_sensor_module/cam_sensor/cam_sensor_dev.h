/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, 2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_SENSOR_DEV_H_
#define _CAM_SENSOR_DEV_H_

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include <cam_sensor_io.h>
#include "cam_debug_util.h"
#include "cam_context.h"

#define NUM_MASTERS 2
#define NUM_QUEUES 2

#undef CDBG
#ifdef CAM_SENSOR_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define SENSOR_DRIVER_I2C "cam-i2c-sensor"
#define CAMX_SENSOR_DEV_NAME "cam-sensor-driver"

enum cam_sensor_state_t {
	CAM_SENSOR_INIT,
	CAM_SENSOR_ACQUIRE,
	CAM_SENSOR_CONFIG,
	CAM_SENSOR_START,
};

enum cam_sensor_frame_state {
	CAM_SENSOR_FRAME_APPLY_PENDING,
	CAM_SENSOR_FRAME_APPLY,
	CAM_SENSOR_FRAME_STATE_MAX
};

/**
 * struct sensor_intf_params
 * @device_hdl: Device Handle
 * @session_hdl: Session Handle
 * @link_hdl: Link Handle
 * @ops: KMD operations
 * @crm_cb: Callback API pointers
 */
struct sensor_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_no_crm_kmd_ops no_crm_ops;
	struct cam_req_mgr_crm_cb *crm_cb;
	cam_req_mgr_no_crm_frame_skip_notify frame_skip_cb;
	uint32_t enable_crm;
};

/**
 * struct cam_sensor_dev_res_info
 *
 * @vc: Virtaul channel
 * @dt: Data type
 * @frame_duration: Frame duraton
 * @req_id: Request Id
 * @feature_mask: Feature mask
 */
struct cam_sensor_dev_res_info {
	uint16_t   vc;
	uint16_t   dt;
	uint64_t   frame_duration;
	uint64_t   req_id;
	uint16_t   feature_mask;
};

/**
 * struct cam_sensor_ctrl_t: Camera control structure
 * @device_name: Sensor device name
 * @pdev: Platform device
 * @cam_sensor_mutex: Sensor mutex
 * @sensordata: Sensor board Information
 * @sensor_res: Sensor resolution information
 * @cci_i2c_master: I2C structure
 * @io_master_info: Information about the communication master
 * @sensor_state: Sensor states
 * @is_probe_succeed: Probe succeeded or not
 * @id: Cell Index
 * @of_node: Of node ptr
 * @v4l2_dev_str: V4L2 device structure
 * @sensor_probe_addr_type: Sensor probe address type
 * @sensor_probe_data_type: Sensor probe data type
 * @i2c_data: Sensor I2C register settings
 * @sensor_info: Sensor query cap structure
 * @bridge_intf: Bridge interface structure
 * @streamon_count: Count to hold the number of times stream on called
 * @streamoff_count: Count to hold the number of times stream off called
 * @bob_reg_index: Hold to BoB regulator index
 * @bob_pwm_switch: Boolean flag to switch into PWM mode for BoB regulator
 * @last_flush_req: Last request to flush
 * @pipeline_delay: Sensor pipeline delay
 * @trigger_sensor_pipeline_delay: Trigger Sensor pipeline delay
 * @sensor_name: Sensor name
 * @aon_camera_id: AON Camera ID associated with this sensor
 * @hw_no_ops: To determine whether HW operations need to be disabled
 * @sof_notify_handler: handler for sof notification
 * @en_perframe_reg_dump: enable perframe register dump flag
 * @last_applied_req: last applied request to detech skip in apply
 * @vc: vc
 * @dt: dt
 * @frame_duration: frame_duration
 * @root_dentry: root directory entry
 * @gpio_mask: Timer number in a single cci core
 * @frame_state: frame settings applied or not
 * @cci_contextId:  sensor ID to be synchronized to the I2C command
 * @is_trigger_mode: sensor is running in trigger mode or not
 */
struct cam_sensor_ctrl_t {
	char                           device_name[
		CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device        *pdev;
	struct cam_hw_soc_info         soc_info;
	struct mutex                   cam_sensor_mutex;
	struct cam_sensor_board_info  *sensordata;
	struct cam_sensor_dev_res_info sensor_res;
	enum cci_i2c_master_t          cci_i2c_master;
	enum cci_device_num            cci_num;
	struct camera_io_master        io_master_info;
	enum cam_sensor_state_t        sensor_state;
	uint8_t                        is_probe_succeed;
	uint32_t                       id;
	struct device_node            *of_node;
	struct cam_subdev              v4l2_dev_str;
	uint8_t                        sensor_probe_addr_type;
	uint8_t                        sensor_probe_data_type;
	struct i2c_data_settings       i2c_data;
	struct  cam_sensor_query_cap   sensor_info;
	struct sensor_intf_params      bridge_intf;
	uint32_t                       streamon_count;
	uint32_t                       streamoff_count;
	int                            bob_reg_index;
	bool                           bob_pwm_switch;
	uint32_t                       last_flush_req;
	uint16_t                       pipeline_delay;
	uint16_t                       trigger_sensor_pipeline_delay;
	char                           sensor_name[
		CAM_SENSOR_NAME_MAX_SIZE];
	uint8_t                        aon_camera_id;
	bool                           hw_no_io_ops;
	bool                           hw_no_ops;
	bool                           hw_no_probe_pw_ops;
	int                            anchor_pd;
	bool                           en_perframe_reg_dump;
	bool                           is_setting_id_valid;
	uint64_t                       last_applied_req;
	uint16_t                       vc;
	uint16_t                       dt;
	uint64_t                       frame_duration;

	struct dentry                  *root_dentry;
	bool                           pause_state;
	uint32_t                       gpio_mask;
	enum cam_sensor_frame_state    frame_state;
	uint32_t                       cci_contextId;
	bool                           is_trigger_mode;
};

struct sensor_userdata {
	struct cam_sensor_ctrl_t *sensor_ctrl;
	uint64_t reqid;
	struct cam_sensor_per_frame_event_data *event_data;
	void *userdata;
};

/**
 * @brief : API to register SENSOR hw to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_sensor_driver_init(void);

/**
 * @brief : API to remove SENSOR Hw from platform framework.
 */
void cam_sensor_driver_exit(void);

/**
 * @brief : API to remove SENSOR component.
 */
void cam_sensor_i2c_component_del_wrapper(struct i2c_client *client);
#endif /* _CAM_SENSOR_DEV_H_ */
