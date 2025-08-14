/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __CAM_SYNC_API_H__
#define __CAM_SYNC_API_H__

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/videodev2.h>
#include <media/cam_sync.h>

/* Sync object types */
#define CAM_SYNC_TYPE_UMD         0
#define CAM_SYNC_TYPE_KMD         1

#define SYNC_DEBUG_NAME_LEN 63
#define CAM_GENERIC_FENCE_BATCH_MAX     10
#define CAM_GENERIC_FENCE_TYPE_HW_FENCE 0x4

typedef void (*sync_callback)(int32_t sync_obj, int status, void *data);

/**
 * Enumeration for clients capable of doing HW fencing using
 * synx or fence protocol
 */
enum cam_sync_fencing_client_cores {
	/* Synx clients */
	CAM_SYNC_SYNX_CLIENT_ICP_0 = 1,
	CAM_SYNC_SYNX_CLIENT_ICP_1,

	/* HW fence clients */
	CAM_SYNC_HW_FENCE_CLIENT_IFE0_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE1_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE2_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE3_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE4_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE5_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE6_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE7_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE8_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE9_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE10_CTX0,
	CAM_SYNC_HW_FENCE_CLIENT_IFE11_CTX0,
	CAM_SYNC_FENCING_CLIENTS_MAX,
};

/**
 * @brief: HW fence session initialization params
 *
 * @param name: Name of the client
 * @param client_core: Client core ID
 * @param signal_id: Signal ID
 * @param session_hdl: Session hdl
 * @param fenceq_dev_addr: Fence tx/rx queue device address
 * @param len: Length of the fence tx/rx queue
 * @param fencing_protocol: Indicates fencing protocol is used
 */
struct cam_sync_hwfence_session_initialize_params {
	char name[100];
	enum cam_sync_fencing_client_cores client_core;
	uint32_t signal_id;

	int32_t session_cookie;
	phys_addr_t fenceq_dev_addr;
	int32_t offset;
	size_t len;

	bool fencing_protocol;
};

/**
 * @brief: HW fence tx queue wr pointer info
 *
 * @client_core: Client core ID pertaining to the HW fence
 * @signal_id: Signal ID associated with the HW fence
 * @wr_pntr: write pointer index for this hw fence for this client
 */
struct cam_sync_hwfence_tx_q_wr_pntr_info {
	enum cam_sync_fencing_client_cores client_core;
	int32_t signal_id;
	int32_t wr_pntr;
};

/**
 * @brief: Update HW fence info in the tx queue for the given sync object
 *
 * @sync_object: Sync object backing associated HW fences
 * @is_hwfence: Set if the sync object is a HW fence
 * @num_hwfences: Number of HW fences associated with the given sync object
 * @wr_pntr_arr: Write pointer array for the associated HW fences
 */
struct cam_sync_hwfence_info {
	int32_t sync_object;
	int32_t is_hwfence;
	int32_t num_hwfences;
	uint32_t reserved;
	struct cam_sync_hwfence_tx_q_wr_pntr_info wr_pntr_arr[CAM_GENERIC_FENCE_BATCH_MAX];
};

/* Kernel APIs */

/**
 * struct cam_sync_signal_param - Cam sync signal parameter information
 *
 * @sync_obj                Int referencing the sync object.
 * @status                  Status of the signaling. Can be either SYNC_SIGNAL_ERROR or
 *                          SYNC_SIGNAL_SUCCESS.
 * @event_cause	            Event parameter
 * @request_id              Request id. This is valid only for IFE fences,
 *                          for other drivers it should be zero.
 * @applied_crop_req_id     Applied crop request id. This is valid only for IFE fences,
 *                          for other drivers it should be zero.
 * @fh                      File handle
 */
struct cam_sync_signal_param {
	uint32_t   sync_obj;
	uint32_t   status;
	uint32_t   event_cause;
	uint64_t   request_id;
	uint64_t   applied_crop_req_id;
	void       *fh;
};

/**
 * @brief: Creates a sync object
 *
 *  The newly created sync obj is assigned to sync_obj.
 *  sync object.
 *
 * @param sync_manager_idx : Index of sync manager
 * @param sync_obj   : Pointer to int referencing the sync object.
 * @param name : Optional parameter associating a name with the sync object for
 * debug purposes. Only first SYNC_DEBUG_NAME_LEN bytes are accepted,
 * rest will be ignored.
 * @param type : Type of the sync object
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if sync_obj is an invalid pointer.
 * -ENOMEM will be returned if the kernel can't allocate space for
 * sync object.
 */
int cam_sync_create(uint32_t sync_manager_idx, int32_t *sync_obj, const char *name,
	uint32_t type);

/**
 * @brief: Registers a callback with a sync object
 *
 * @param cb_func:  Pointer to callback to be registered
 * @param userdata: Opaque pointer which will be passed back with callback.
 * @param sync_obj: int referencing the sync object.
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if userdata is invalid.
 * -ENOMEM will be returned if cb_func is invalid.
 *
 */
int cam_sync_register_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj);

/**
 * @brief: De-registers a callback with a sync object
 *
 * @param cb_func:  Pointer to callback to be de-registered
 * @param userdata: Opaque pointer which will be passed back with callback.
 * @param sync_obj: int referencing the sync object.
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if userdata is invalid.
 * -ENOMEM will be returned if cb_func is invalid.
 */
int cam_sync_deregister_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj);

/**
 * @brief: Signals a sync object with the status argument.
 *
 * This function will signal the sync object referenced by the sync_obj
 * parameter and when doing so, will trigger callbacks in both user space and
 * kernel. Callbacks will triggered asynchronously and their order of execution
 * is not guaranteed. The status parameter will indicate whether the entity
 * performing the signaling wants to convey an error case or a success case.
 *
 * @param param: Pointer to cam_sync_signal_param object.
 * @param timestamp: Pointer to cam_sync_timestamp object.
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_signal(struct cam_sync_signal_param *param,
	struct cam_sync_timestamp *timestamp);

/**
 * @brief: To get sync device version
 *
 * @return .Status of operation Sync device version support.
 */

int cam_sync_get_version(void);

/**
 * @brief: Merges multiple sync objects
 *
 * This function will merge multiple sync objects into a sync group.
 *
 * @param sync_obj: pointer to a block of ints to be merged
 * @param num_objs: Number of ints in the block
 * @param type    : Type of the merged sync object
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_merge(int32_t *sync_obj, uint32_t num_objs, int32_t *merged_obj,
	uint32_t type);

/**
 * @brief: get ref count of sync obj
 *
 * This function will increment ref count for the sync object, and the ref
 * count will be decremented when this sync object is signaled.
 *
 * @param sync_obj: sync object
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_get_obj_ref(int32_t sync_obj);

/**
 * @brief: put ref count of sync obj
 *
 * This function will decrement ref count for the sync object.
 *
 * @param sync_obj: sync object
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_put_obj_ref(int32_t sync_obj);

/**
 * @brief: Destroys a sync object
 *
 * @param sync_obj: int referencing the sync object to be destroyed
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_destroy(int32_t sync_obj);

/**
 * @brief: Waits for a sync object synchronously
 *
 * Does a wait on the sync object identified by sync_obj for a maximum
 * of timeout_ms milliseconds. Must not be called from interrupt context as
 * this API can sleep. Should be called from process context only.
 *
 * @param sync_obj: int referencing the sync object to be waited upon
 * @timeout_ms sync_obj: Timeout in ms.
 *
 * @return 0 upon success, -EINVAL if sync object is in bad state or arguments
 * are invalid, -ETIMEDOUT if wait times out.
 */
int cam_sync_wait(int32_t sync_obj, uint64_t timeout_ms);

/**
 * @brief: Check if sync object is valid
 *
 * @param sync_obj: int referencing the sync object to be checked
 *
 * @return 0 upon success, -EINVAL if sync object is in bad state or arguments
 * are invalid
 */
int cam_sync_check_valid(int32_t sync_obj);

/**
 * @brief: Synx recovery for a given core
 *
 * @param core_id: Core ID we want to recover for
 *
 * @return Status of operation. Zero in case of success
 * -EINVAL if core_id is invalid
 */
int cam_sync_synx_core_recovery(
	enum cam_sync_fencing_client_cores core_id);

int cam_sync_hw_fence_session_cleanup(void);

/**
 * @brief: Initialize HW fence session for a given client
 *
 * @param name: Name of the HW fence session
 * @param client_id: Client ID pertainting to this session ICP0/IFE0_CTX0/IFE0_CTX1....
 * @output init_params: Initialization output parameters if session is created successfully
 *
 * @return status of operation zero in case of success
 */
int cam_sync_initialize_hw_fence_session(
	struct cam_sync_hwfence_session_initialize_params *init_params);

/**
 * @brief: Deinitialize HW fence session for a given client
 *
 * @param session_hdl: Handle pertaining to the session to be deinitialized
 *
 * @return status of operation zero in case of success
 */
int cam_sync_deinitialize_hw_fence_session(int32_t session_hdl);

/**
 * @brief: Validates sync obj and updates hw fence queue pntrs
 *
 * @param hwfence_info: pointer to structure having sync_object
 *
 * @return status of operation zero in case of success
 */
int cam_sync_check_and_update_hw_fence_queue_pntrs(
	struct cam_sync_hwfence_info *hwfence_info);

/**
 * @brief : API to register SYNC to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_sync_init(void);

/**
 * @brief : API to remove SYNC from platform framework.
 */
void cam_sync_exit(void);
#endif /* __CAM_SYNC_API_H__ */
