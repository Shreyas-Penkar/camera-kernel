// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/irqflags.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/remoteproc/qcom_rproc.h>
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX) || IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
#include <synx_api.h>
#endif
#include "cam_sync_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_compat.h"
#include "camera_main.h"
#include "cam_req_mgr_worker_wrapper.h"

struct sync_device *sync_dev;
struct sync_uid_info sync_uid_access;

#define CAM_SYNC_HW_FENCE_MAX_CLIENTS  CAM_SYNC_FENCING_CLIENTS_MAX
#define CAM_SYNC_HW_FENCE_MAX_SUB_GRPS 20

/*
 * BITS[0  : 7]  --> client core ID [refers to the HW fence tbl row]
 * BITS[8  : 15] --> IPCC ID within the core [refers to the HW fence tbl column]
 * BITS[16 : 31] --> Random magic number
 */
#define CAM_SYNC_GENERATE_SYNX_CLIENT_SESSION_HDL(client_core, col_index)         \
({                                                                                \
	int32_t cookie;                                                               \
	uint32_t rand;                                                                \
	cookie = ((client_core) & 0xFF);                                             \
	cookie |= ((col_index) << 8) & 0xFF00;                                       \
	get_random_bytes(&rand, 2);                                                   \
	cookie |= ((rand) << 16);                                                     \
	cookie;                                                                       \
})                                                                                \

#define CAM_SYNC_GET_CLIENT_INFO_FROM_SESSION_HDL(session_hdl, client_core,       \
	col_index)                                                                    \
({                                                                                \
	*client_core = ((session_hdl) & 0xFF);                                         \
	*col_index = (((session_hdl) >> 8) & 0xFF);                                    \
})                                                                                \

#define CAM_SYNC_VALIDATE_HW_FENCE_CLIENT_INFO(client_core, col_index)            \
({                                                                                \
	bool is_valid = false;                                                        \
	if (((*client_core) * (*col_index)) <=                                          \
		(CAM_SYNC_HW_FENCE_MAX_CLIENTS * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS))         \
		is_valid = true;                                                          \
	is_valid;                                                                     \
})                                                                                \


struct cam_sync_hw_fence_client_entries {
	enum cam_sync_fencing_client_cores client_core;
	uint32_t signal_id;

	/* cookie will be the session_hdl in the client's scope */
	int32_t cookie;
	struct synx_session *session_hdl;

	/* kernel va to get tx queue SW write ptr */
	void *txq_wr_ptr;

	bool active;
	bool fence_protocol;
};

/*
 * Placeholder for all HW fence related info
 */
struct cam_sync_hw_fence_info {
/*
 * Ptr to a 2D table
 * row --> indexed for different client core
 * column --> indexed for different IPCC IDs/grps within the same core
 *
 *
 *        +-------+------+------+------+------+
 * ICP0   |  ID0  |      |      |      |      |
 *        +-------+------+------+------+------+
 * ICP1   |  ID0  |      |      |      |      |
 *        +-------+------+------+------+------+
 * IFE0   |  ID0  | ID1  | ID2  |  ID3 |      |
 *        +-------+------+------+------+------+
 * IFE1   |  ID0  | ID1  |      |      |      |
 *        +-------+------+------+------+------+
 *        .
 *        .
 *        +-------+------+------+------+------+
 * IFEn   |  ID0  | ID1  | ID2  | ID3  | ID4  |
 *        +-------+------+------+------+------+
 *
 * Table to provide constant time access to relevant HW fence info
 */
	struct cam_sync_hw_fence_client_entries *hw_fence_tbl;

	/* Per client bitmap to determine free column */
	size_t num_bits;
	void *client_bitmaps[CAM_SYNC_HW_FENCE_MAX_CLIENTS];

	/* spinlocks for each entry in the table */
	spinlock_t *hw_fence_locks[CAM_SYNC_HW_FENCE_MAX_CLIENTS * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS];
};

static struct cam_sync_hw_fence_info hw_fence_info;

/*
 * Flag to determine whether to enqueue cb of a
 * signaled fence onto the worker or invoke it
 * directly in the same context
 */
static bool trigger_cb_without_switch;
#define CAM_SYNC_IS_HW_FENCE_SESSION(val) ((val) ? true : false)

static int cam_sync_signal_validate_util(
	int32_t sync_obj, int32_t status)
{
	struct sync_table_row *row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name, sync_obj);
		return -EINVAL;
	}

	if (row->state != CAM_SYNC_STATE_ACTIVE) {
		CAM_ERR(CAM_SYNC,
			"Error: Sync object already signaled sync_obj = %s[%d]",
			row->name, sync_obj);
		return -EALREADY;
	}

	if ((status != CAM_SYNC_STATE_SIGNALED_SUCCESS) &&
		(status != CAM_SYNC_STATE_SIGNALED_ERROR) &&
		(status != CAM_SYNC_STATE_SIGNALED_CANCEL)) {
		CAM_ERR(CAM_SYNC,
			"Error: signaling with undefined status = %d", status);
		return -EINVAL;
	}

	return 0;
}

static void cam_sync_signal_parent_util(
	struct cam_sync_signal_param *param, struct list_head *parents_list,
	struct cam_sync_timestamp *time_stamp)
{
	int rc;
	struct sync_table_row *parent_row = NULL;
	struct sync_parent_info *parent_info, *temp_parent_info;
	uint32_t psync_obj;

	/*
	 * Now iterate over all parents of this object and if they too need to
	 * be signaled dispatch cb's
	 */
	 list_for_each_entry_safe(parent_info, temp_parent_info,
		parents_list, list) {
		psync_obj = parent_info->sync_id & sync_uid_access.fenceIdMask;
		parent_row = sync_dev->sync_table + psync_obj;
		spin_lock_bh(&sync_dev->row_spinlocks[psync_obj]);
		parent_row->remaining--;

		rc = cam_sync_util_update_parent_state(
			parent_row,
			param->status);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Invalid parent state %d",
				parent_row->state);
			spin_unlock_bh(
				&sync_dev->row_spinlocks[psync_obj]);
			kfree(parent_info);
			continue;
		}

		param->sync_obj = parent_info->sync_id;
		if (!parent_row->remaining)
			cam_sync_util_dispatch_signaled_cb(param, time_stamp);


		spin_unlock_bh(&sync_dev->row_spinlocks[psync_obj]);
		list_del_init(&parent_info->list);
		kfree(parent_info);
	}
}

static int cam_generic_fence_alloc_validate_input_info_util(
	struct cam_generic_fence_cmd_args    *fence_cmd_args,
	struct cam_generic_fence_input_info **fence_input_info)
{
	int rc = 0;
	struct cam_generic_fence_input_info *fence_input = NULL;
	uint32_t num_fences;
	size_t expected_size;

	*fence_input_info = NULL;

	if (fence_cmd_args->input_data_size !=
		sizeof(struct cam_generic_fence_input_info)) {
		CAM_ERR(CAM_SYNC, "Size is invalid expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_generic_fence_input_info),
			fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	fence_input = memdup_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_cmd_args->input_data_size);
	if (IS_ERR_OR_NULL(fence_input)) {
		CAM_ERR(CAM_SYNC, "memdup failed for hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		return -ENOMEM;
	}

	/* Validate num fences */
	num_fences = fence_input->num_fences_requested;
	if ((num_fences == 0) || (num_fences > CAM_GENERIC_FENCE_BATCH_MAX)) {
		CAM_ERR(CAM_SYNC, "Invalid number of fences: %u for batching",
			num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	/* Validate sizes */
	expected_size = sizeof(struct cam_generic_fence_input_info) +
		((num_fences - 1) * sizeof(struct cam_generic_fence_config));
	if ((uint32_t)expected_size != fence_cmd_args->input_data_size) {
		CAM_ERR(CAM_SYNC, "Invalid input size expected: 0x%x actual: 0x%x for fences: %u",
			expected_size, fence_cmd_args->input_data_size, num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	*fence_input_info = fence_input;
	return rc;

free_mem:
	kfree(fence_input);
	return rc;
}

static void cam_generic_fence_free_input_info_util(
	void **fence_input_info)
{
	void *fence_input = *fence_input_info;

	kfree(fence_input);
	*fence_input_info = NULL;
}

static void cam_sync_print_fence_table(void)
{
	int idx;

	for (idx = 0; idx < CAM_SYNC_MAX_OBJS; idx++) {
		spin_lock(&sync_dev->row_spinlocks[idx]);
		CAM_INFO(CAM_SYNC,
			"index[%u]: sync_id=%d, name=%s, type=%d, state=%d, ref_cnt=%d",
			idx,
			sync_dev->sync_table[idx].sync_id,
			sync_dev->sync_table[idx].name,
			sync_dev->sync_table[idx].type,
			sync_dev->sync_table[idx].state,
			atomic_read(&sync_dev->sync_table[idx].ref_cnt));
		spin_unlock(&sync_dev->row_spinlocks[idx]);
	}
}

static inline int __add_sync_manager_idx(int32_t sync_id, uint32_t sync_manager_idx)
{
	return (sync_manager_idx << sync_dev->sync_manager_id_shift) | sync_id;
}

static inline int get_sync_manager_idx(uint32_t sync_obj)
{
	return (sync_obj >> sync_dev->sync_manager_id_shift) & sync_dev->sync_manager_id_mask;
}

static int cam_sync_create_util(
	uint32_t sync_manager_idx, int32_t *sync_obj, const char *name,
	struct cam_dma_fence_create_sync_obj_payload *dma_sync_create_info,
	struct sync_synx_obj_info *synx_obj_sync_create_info, uint32_t type)
{
	struct sync_table_row *row = NULL;
	struct sync_ext_fence_info *ext_fence_info = NULL;
	long idx;
	int rc;
	bool bit;

	do {
		idx = find_first_zero_bit(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);
		if (idx >= CAM_SYNC_MAX_OBJS) {
			CAM_ERR(CAM_SYNC,
				"Error: Unable to create sync idx = %d sync name = %s reached max!",
				idx, name);
			cam_sync_print_fence_table();
			return -ENOMEM;
		}
		CAM_DBG(CAM_SYNC, "Index location available at idx: %ld", idx);
		bit = test_and_set_bit(idx, sync_dev->bitmap);
	} while (bit);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	rc = cam_sync_init_row(sync_dev->sync_table, idx, name,
		type, sync_manager_idx);
	if (rc) {
		CAM_ERR(CAM_SYNC, "Error: Unable to init row at idx = %ld",
			idx);
		clear_bit(idx, sync_dev->bitmap);
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		return -EINVAL;
	}
	*sync_obj = __add_sync_manager_idx(idx, sync_manager_idx);
	row = sync_dev->sync_table + idx;

	if (synx_obj_sync_create_info || dma_sync_create_info) {
		rc = cam_sync_get_ext_fence_payload(&ext_fence_info);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Failed to get external fence info");
			clear_bit(idx, sync_dev->bitmap);
			spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
			return rc;
		}
		list_add_tail(&ext_fence_info->list, &row->ext_fences);
	}

	/* Associate sync obj with synx if any holding sync lock */
	if (synx_obj_sync_create_info) {
		ext_fence_info->synx_obj_info.synx_obj_row_idx =
			synx_obj_sync_create_info->synx_obj_row_idx;
		ext_fence_info->synx_obj_info.sync_created_with_synx =
			synx_obj_sync_create_info->sync_created_with_synx;
		ext_fence_info->synx_obj_info.synx_obj = synx_obj_sync_create_info->synx_obj;
		ext_fence_info->synx_obj_info.is_valid = true;

		set_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask);

		CAM_DBG(CAM_SYNC, "sync_obj: %s[%d] associated with synx_obj: %d",
			name, *sync_obj, ext_fence_info->synx_obj_info.synx_obj);
	}

	/* Associate sync obj with dma fence if any holding sync lock */
	if (dma_sync_create_info) {
		ext_fence_info->dma_fence_info.dma_fence_fd = dma_sync_create_info->fd;
		ext_fence_info->dma_fence_info.dma_fence_row_idx =
			dma_sync_create_info->dma_fence_row_idx;
		ext_fence_info->dma_fence_info.sync_created_with_dma =
			dma_sync_create_info->sync_created_with_dma;
		ext_fence_info->dma_fence_info.is_valid = true;

		set_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask);

		/* Association refcnt for non-import cases */
		if (dma_sync_create_info->sync_created_with_dma) {
			rc = cam_dma_fence_get_put_ref(true,
				ext_fence_info->dma_fence_info.dma_fence_row_idx);
			if (rc)
				CAM_ERR(CAM_SYNC,
					"Failed to getref on dma fence idx: %u fd: %d sync_obj: %d rc: %d",
					ext_fence_info->dma_fence_info.dma_fence_row_idx,
					ext_fence_info->dma_fence_info.dma_fence_fd,
					*sync_obj, rc);
			goto end;
		}

		CAM_DBG(CAM_SYNC, "sync_obj: %s[%d] associated with dma fence fd: %d",
			name, *sync_obj, dma_sync_create_info->fd);
		goto end;
	}
	CAM_DBG(CAM_SYNC, "sync_obj: %s[%i]", name, *sync_obj);
end:
	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	return rc;
}
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
static int cam_generic_fence_config_parse_params(
	struct cam_generic_fence_config *fence_cfg,
	int32_t requested_param_mask, int32_t *result)
{
	uint32_t index = 0, num_entries;

	if (!result) {
		CAM_ERR(CAM_SYNC, "Invalid result hdl : %p", result);
		return -EINVAL;
	}

	/* Assign to 0 by default */
	*result = 0;

	if (!fence_cfg->num_valid_params || !requested_param_mask) {
		CAM_DBG(CAM_SYNC,
			"No params configured num_valid = %d requested_mask = 0x%x",
			fence_cfg->num_valid_params, requested_param_mask);
		return 0;
	}

	if (!(fence_cfg->valid_param_mask & requested_param_mask)) {
		CAM_DBG(CAM_SYNC,
			"Requested parameter not set in additional param mask expecting: 0x%x actual: 0x%x",
			requested_param_mask, fence_cfg->valid_param_mask);
		return 0;
	}

	index = ffs(requested_param_mask) - 1;
	num_entries = ARRAY_SIZE(fence_cfg->params);
	if (index >= num_entries) {
		CAM_DBG(CAM_SYNC,
			"Obtained index %u from mask: 0x%x num_param_entries: %u, index exceeding max",
			index, requested_param_mask, num_entries);
		return 0;
	}

	*result = fence_cfg->params[index];
	return 0;
}

static int cam_sync_synx_obj_cb(int32_t sync_obj,
	struct cam_synx_obj_signal_sync_obj *signal_sync_obj)
{
	struct sync_table_row *row = NULL;
	struct list_head parents_list;
	struct sync_ext_fence_info *ext_fence_info, *tmp;
	struct cam_sync_signal_param param;
	int32_t rc = 0;
	bool found = false;
	uint32_t sync_object, uid_validity;
	uint16_t sync_uid;

	sync_object = sync_obj & sync_uid_access.fenceIdMask;
	sync_uid = sync_obj >> sync_uid_access.uidShift;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_object]);
	row = sync_dev->sync_table + sync_object;

	/* Check if it is a valid fence, i.e., either current fence uid or newer */
	uid_validity = cam_sync_check_uid_valid(sync_obj);
	if (uid_validity == SYNC_UID_NEW) {
		rc = cam_sync_reinit_object(sync_dev->sync_table, sync_obj);
	} else if (uid_validity == SYNC_UID_OLD) {
		CAM_ERR(CAM_SYNC, "Invalid fence, sync obj: %d, sync_uid: %u",
			sync_object, sync_uid);
		rc = -EINVAL;
		goto end;
	}

	if (!signal_sync_obj) {
		CAM_ERR(CAM_SYNC, "Invalid signal info args");
		rc = -EINVAL;
		goto end;
	}

	/* Validate sync object range */
	if (cam_sync_check_valid(sync_obj)) {
		CAM_ERR(CAM_SYNC, "Invalid sync obj: %d", sync_obj);
		rc = -EINVAL;
		goto end;
	}

	/* Validate if sync obj has a synx obj association */
	if (!test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask)) {
		CAM_ERR(CAM_SYNC,
			"sync obj = %d[%s] has no associated synx obj ext_fence_mask = 0x%x",
			sync_obj, row->name, row->ext_fence_mask);
		rc = -EINVAL;
		goto end;
	}

	/* Validate if we are signaling the right sync obj based on synx handle */
	list_for_each_entry_safe(ext_fence_info, tmp, &row->ext_fences, list) {
		if (ext_fence_info->synx_obj_info.synx_obj == signal_sync_obj->synx_obj) {
			found = true;
			break;
		}
	}
	if (!found) {
		CAM_ERR(CAM_SYNC,
			"sync obj: %d[%s] is not associated with synx obj: %d",
			sync_obj, row->name, signal_sync_obj->synx_obj);
		rc = -EINVAL;
		goto end;
	}

	rc = cam_sync_signal_validate_util(sync_object, signal_sync_obj->status);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"Error: Failed to validate signal info for sync_obj = %d[%s] with status = %d rc = %d",
			sync_object, row->name, signal_sync_obj->status, rc);
		goto end;
	}

	/* Adding synx reference on sync */
	atomic_inc(&row->ref_cnt);
	if (!atomic_dec_and_test(&row->ref_cnt)) {
		CAM_DBG(CAM_SYNC, "Sync = %d[%s] fence still has references, synx_hdl = %d",
			sync_obj, row->name, signal_sync_obj->synx_obj);
		goto end;
	}

	row->state = signal_sync_obj->status;
	param.status = signal_sync_obj->status;
	param.sync_obj = sync_obj;
	param.fh = sync_dev->cam_sync_eventq[0];
	cam_sync_util_dispatch_signaled_cb(&param, NULL);

	INIT_LIST_HEAD(&parents_list);
	list_splice_init(&row->parents_list, &parents_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_object]);

	if (list_empty(&parents_list))
		return 0;

	cam_sync_signal_parent_util(&param, &parents_list, NULL);
	CAM_DBG(CAM_SYNC,
		"Successfully signaled sync obj = %d with status = %d via synx obj = %d signal callback",
		sync_obj, signal_sync_obj->status, signal_sync_obj->synx_obj);

	return 0;

end:
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_object]);
	return rc;
}
#endif

static int cam_sync_dma_fence_cb(
	int32_t sync_obj,
	struct cam_dma_fence_signal_sync_obj *signal_sync_obj)
{
	struct sync_ext_fence_info *ext_fence_info, *tmp;
	struct sync_table_row *row = NULL;
	struct cam_sync_signal_param param;
	struct list_head parents_list;
	int32_t rc = 0;
	int32_t status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
	bool found = false;

	if (!signal_sync_obj) {
		CAM_ERR(CAM_SYNC, "Invalid signal info args");
		return -EINVAL;
	}

	/* Validate sync object range */
	if (!(sync_obj > 0 && sync_obj < CAM_SYNC_MAX_OBJS)) {
		CAM_ERR(CAM_SYNC, "Invalid sync obj: %d", sync_obj);
		return -EINVAL;
	}

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	/* Validate if sync obj has a dma fence association */
	if (!test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask)) {
		CAM_ERR(CAM_SYNC,
			"sync obj = %d[%s] has no associated dma fence ext_fence_mask = 0x%x",
			sync_obj, row->name, row->ext_fence_mask);
		rc = -EINVAL;
		goto end;
	}

	/* Validate if we are signaling the right sync obj based on synx handle */
	list_for_each_entry_safe(ext_fence_info, tmp, &row->ext_fences, list) {
		if (ext_fence_info->dma_fence_info.dma_fence_fd == signal_sync_obj->fd) {
			found = true;
			break;
		}
	}
	/* Validate if we are signaling the right sync obj based on dma fence fd */
	if (!found) {
		CAM_ERR(CAM_SYNC,
			"sync obj: %d[%s] is not associated with dma fence fd: %d",
			sync_obj, row->name, signal_sync_obj->fd);
		rc = -EINVAL;
		goto end;
	}

	/* Check for error status */
	if (signal_sync_obj->status < 0) {
		if (signal_sync_obj->status == -ECANCELED)
			status = CAM_SYNC_STATE_SIGNALED_CANCEL;
		else
			status = CAM_SYNC_STATE_SIGNALED_ERROR;
	}

	rc = cam_sync_signal_validate_util(sync_obj, status);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"Error: Failed to validate signal info for sync_obj = %d[%s] with status = %d rc = %d",
			sync_obj, row->name, status, rc);
		goto end;
	}

	/* Adding dma fence reference on sync */
	atomic_inc(&row->ref_cnt);

	if (!atomic_dec_and_test(&row->ref_cnt))
		goto end;

	row->state = status;
	param.status = status;
	param.sync_obj = sync_obj;
	param.fh = sync_dev->cam_sync_eventq[0];
	cam_sync_util_dispatch_signaled_cb(&param, NULL);

	INIT_LIST_HEAD(&parents_list);
	list_splice_init(&row->parents_list, &parents_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	if (list_empty(&parents_list))
		return 0;

	cam_sync_signal_parent_util(&param, &parents_list, NULL);
	return 0;

end:
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return rc;
}

static int cam_generic_fence_handle_sync_synx_dma_create(
	uint32_t sync_manager_idx, struct cam_generic_fence_config *fence_cfg,
	uint32_t num_fences_requested)
{
	int rc, dma_fence_row_idx;
	bool dma_fence_created;
	unsigned long fence_sel_mask;
	struct cam_dma_fence_release_params release_params;
	struct cam_dma_fence_create_sync_obj_payload dma_sync_create;
	bool synx_obj_created = false;
	struct sync_synx_obj_info synx_obj_create;
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	int32_t fence_flag;
	int32_t synx_obj_row_idx = 0;
	struct cam_synx_obj_release_params synx_release_params;
	struct dma_fence *dma_fence_ptr;
#endif

	/* Reset flag */
	dma_fence_created = false;
	synx_obj_created = false;

	fence_sel_mask = fence_cfg->fence_sel_mask;
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
		rc = cam_dma_fence_create_fd(&fence_cfg->dma_fence_fd,
				&dma_fence_row_idx, fence_cfg->name);
		if (rc) {
			CAM_ERR(CAM_SYNC,
					"Failed to create dma fence rc: %d num_fences:%u",
					rc, num_fences_requested);
			fence_cfg->reason_code = rc;
			goto end;
		}

		dma_sync_create.dma_fence_row_idx = dma_fence_row_idx;
		dma_sync_create.fd = fence_cfg->dma_fence_fd;
		dma_sync_create.sync_created_with_dma = true;
		dma_fence_created = true;
	}

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	/* Create a synx object */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &fence_sel_mask)) {
		if (dma_fence_created) {
			dma_fence_ptr = cam_dma_fence_get_fence_from_fd(
					dma_sync_create.fd, &dma_fence_row_idx);
			rc = cam_synx_obj_import_dma_fence(fence_cfg->name,
					fence_cfg->params[0], dma_fence_ptr,
					&fence_cfg->synx_obj, &synx_obj_row_idx);
		} else {
			cam_generic_fence_config_parse_params(fence_cfg,
					CAM_GENERIC_FENCE_CONFIG_FLAG_PARAM_INDEX,
					&fence_flag);
			rc = cam_synx_obj_create(fence_cfg->name, NULL,
					fence_flag, &fence_cfg->synx_obj,
					&synx_obj_row_idx);
		}

		if (rc) {
			CAM_ERR(CAM_SYNC,
					"Failed to create/import synx obj rc: %d num_fences: %u",
					rc, num_fences_requested);

			/* Release dma fence */
			if (dma_fence_created) {
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx =
					dma_fence_row_idx;

				cam_dma_fence_release(&release_params);
			}
			/* Release synx obj */
			if (synx_obj_created) {
				synx_release_params.use_row_idx = true;
				synx_release_params.u.synx_row_idx =
					synx_obj_row_idx;

				cam_synx_obj_release(&synx_release_params);
			}
			goto end;
		}

		synx_obj_create.sync_created_with_synx = true;
		synx_obj_create.synx_obj = fence_cfg->synx_obj;
		synx_obj_create.synx_obj_row_idx = synx_obj_row_idx;
		synx_obj_created = true;
	}
#endif
	rc = cam_sync_create_util(sync_manager_idx, &fence_cfg->sync_obj, fence_cfg->name,
			(dma_fence_created ? &dma_sync_create : NULL),
			(synx_obj_created ? &synx_obj_create : NULL), CAM_SYNC_TYPE_UMD);
	if (rc) {
		fence_cfg->reason_code = rc;

		CAM_ERR(CAM_SYNC,
				"Failed to create sync obj rc: %d num_fences: %u",
				rc, num_fences_requested);
		/* Release dma fence */
		if (dma_fence_created) {
			release_params.use_row_idx = true;
			release_params.u.dma_row_idx = dma_fence_row_idx;

			cam_dma_fence_release(&release_params);
		}
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
		/* Release synx obj */
		if (synx_obj_created) {
			synx_release_params.use_row_idx = true;
			synx_release_params.u.synx_row_idx = synx_obj_row_idx;

			cam_synx_obj_release(&synx_release_params);
		}
#endif
		goto end;
	}

	/* Register dma fence cb */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
		rc = cam_dma_fence_register_cb(&fence_cfg->sync_obj,
				&dma_fence_row_idx, cam_sync_dma_fence_cb);
		if (rc) {
			CAM_ERR(CAM_SYNC,
					"Failed to register cb for dma fence fd: %d sync_obj: %d rc: %d",
					fence_cfg->dma_fence_fd, fence_cfg->sync_obj,
					rc);

			fence_cfg->reason_code = rc;
			/* Destroy sync obj */
			cam_sync_deinit_object(sync_dev->sync_table,
					fence_cfg->sync_obj);
			/* Release dma fence */
			if (dma_fence_created) {
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx =
					dma_fence_row_idx;

				cam_dma_fence_release(&release_params);
			}
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
			/* Release synx obj */
			if (synx_obj_created) {
				synx_release_params.use_row_idx = true;
				synx_release_params.u.synx_row_idx =
					synx_obj_row_idx;

				cam_synx_obj_release(&synx_release_params);
			}
#endif
			goto end;
		}
	}
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	/* Register synx object callback */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &fence_sel_mask)) {
		rc = cam_synx_obj_register_cb(&fence_cfg->sync_obj,
				synx_obj_row_idx, cam_sync_synx_obj_cb);
		if (rc) {
			CAM_ERR(CAM_SYNC,
					"Failed to register cb for synx_obj: %d	sync_obj: %d rc: %d",
					fence_cfg->synx_obj, fence_cfg->sync_obj, rc);

			fence_cfg->reason_code = rc;
			/* Destroy sync obj */
			cam_sync_deinit_object(sync_dev->sync_table,
					fence_cfg->sync_obj);
			/* Release dma fence */
			if (dma_fence_created) {
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx =
					dma_fence_row_idx;

				cam_dma_fence_release(&release_params);
			}
			/* Release synx obj */
			if (synx_obj_created) {
				synx_release_params.use_row_idx = true;
				synx_release_params.u.synx_row_idx =
					synx_obj_row_idx;

				cam_synx_obj_release(&synx_release_params);
			}
		}
	}
#endif

end:
	return rc;
}

int cam_sync_create(uint32_t sync_manager_idx, int32_t *sync_obj, const char *name, uint32_t type)
{
	return cam_sync_create_util(sync_manager_idx, sync_obj, name, NULL, NULL, type);
}

int cam_sync_register_callback(sync_callback cb_func,
	void *userdata, int32_t sync_var)
{
	struct sync_callback_info *sync_cb;
	struct sync_table_row *row = NULL;
	int status = 0;
	uint32_t sync_obj, uid_validity;
	uint16_t sync_uid;
	int rc, sync_manager_idx;
	struct crm_worker_task *task = NULL;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(sync_var);

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0 || !cb_func)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;
	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, sync_var);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}
	uid_validity = cam_sync_check_uid_valid(sync_var);

	if (uid_validity == SYNC_UID_NEW) {
		rc = cam_sync_reinit_object(sync_dev->sync_table, sync_var);
	} else if (uid_validity == SYNC_UID_OLD) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "Called for invalid fence, sync obj: %d, uid: %d",
			sync_obj,
			sync_uid);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	sync_cb = kzalloc(sizeof(*sync_cb), GFP_ATOMIC);
	if (!sync_cb) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -ENOMEM;
	}

	/* Trigger callback if sync object is already in SIGNALED state */
	if (((row->state == CAM_SYNC_STATE_SIGNALED_SUCCESS) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) &&
		(!row->remaining)) {
		if (trigger_cb_without_switch) {
			CAM_DBG(CAM_SYNC, "Invoke callback for sync object:%s[%d], sync_uid: %d",
				row->name,
				sync_obj,
				sync_uid);
			status = row->state;
			kfree(sync_cb);
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			cb_func(sync_var, status, userdata);
		} else {
			sync_cb->callback_func = cb_func;
			sync_cb->cb_data = userdata;
			sync_cb->sync_obj = sync_var;
			sync_cb->status = row->state;
			CAM_DBG(CAM_SYNC, "Enqueue callback for sync object:%s[%d], sync_uid: %d",
				row->name,
				sync_cb->sync_obj,
				sync_uid);
			task = cam_req_mgr_worker_get_task(sync_dev->worker);
			if (IS_ERR_OR_NULL(task)) {
				CAM_ERR(CAM_SYNC, "Failed to get task = %d", PTR_ERR(task));
				kfree(sync_cb);
			} else {
				task->payload = sync_cb;
				task->process_cb = cam_sync_util_cb_dispatch;
				cam_req_mgr_worker_enqueue_task(task, NULL, CRM_TASK_PRIORITY_0);
			}
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		}

		return 0;
	}

	sync_cb->callback_func = cb_func;
	sync_cb->cb_data = userdata;
	sync_cb->sync_obj = sync_var;
	list_add_tail(&sync_cb->list, &row->callback_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	return 0;
}

int cam_sync_deregister_callback(sync_callback cb_func,
	void *userdata, int32_t sync_var)
{
	struct sync_table_row *row = NULL;
	struct sync_callback_info *sync_cb, *temp;
	bool found = false;
	uint32_t sync_obj, sync_manager_idx;
	uint16_t sync_uid;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(sync_var);

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	if (row->uid != sync_uid) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "deregister called for invalid fence, sync obj: %d, uid: %d",
			sync_obj, sync_uid);
		return -EINVAL;
	}
	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, sync_var);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	CAM_DBG(CAM_SYNC, "deregistered callback for sync object:%s[%d], uid: %d",
		row->name,
		sync_obj,
		sync_uid);
	list_for_each_entry_safe(sync_cb, temp, &row->callback_list, list) {
		if (sync_cb->callback_func == cb_func &&
			sync_cb->cb_data == userdata) {
			list_del_init(&sync_cb->list);
			kfree(sync_cb);
			found = true;
		}
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return found ? 0 : -ENOENT;
}

static inline int cam_sync_signal_dma_fence_util(
	struct sync_dma_fence_info *dma_fence_info, uint32_t status)
{
	struct cam_dma_fence_signal signal_dma_fence;

	signal_dma_fence.dma_fence_fd = dma_fence_info->dma_fence_fd;

	switch (status) {
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		signal_dma_fence.status = 0;
		break;
	case CAM_SYNC_STATE_SIGNALED_ERROR:
		/* Advertise error */
		signal_dma_fence.status = -EADV;
		break;
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
		signal_dma_fence.status = -ECANCELED;
		break;
	default:
		CAM_ERR(CAM_SYNC,
			"Signaling undefined status: %d", status);
		return -EINVAL;
	}

	return cam_dma_fence_internal_signal(dma_fence_info->dma_fence_row_idx,
		&signal_dma_fence);
}

int cam_sync_merge(int32_t *sync_var, uint32_t num_objs, int32_t *merged_obj,
	uint32_t type)
{
	int rc;
	long idx = 0;
	bool bit;
	int i = 0;
	uint32_t sync_manager_idx;

	if (!sync_var || !merged_obj) {
		CAM_ERR(CAM_SYNC, "Invalid pointer(s)");
		return -EINVAL;
	}

	if (num_objs <= 1) {
		CAM_ERR(CAM_SYNC, "Single object merge is not allowed");
		return -EINVAL;
	}

	if (cam_common_util_remove_duplicate_arr(sync_var, num_objs)
		!= num_objs) {
		CAM_ERR(CAM_SYNC, "The obj list has duplicate fence");
		return -EINVAL;
	}

	sync_manager_idx = get_sync_manager_idx(sync_var[0]);
	for (i = 0; i < num_objs; i++) {
		rc = cam_sync_check_valid(sync_var[i]);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Sync_obj[%d] %d valid check fail",
				i, sync_var[i]);
			return rc;
		}
		if (get_sync_manager_idx(sync_var[i]) != sync_manager_idx) {
			CAM_ERR(CAM_SYNC,
				"Sync objects %d %d belongs to different sync manager %d %d",
				sync_var[0], sync_var[i], sync_manager_idx,
				get_sync_manager_idx(sync_var[i]));
			return -EINVAL;
		}
	}
	do {
		idx = find_first_zero_bit(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);
		if (idx >= CAM_SYNC_MAX_OBJS)
			return -ENOMEM;
		bit = test_and_set_bit(idx, sync_dev->bitmap);
	} while (bit);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	rc = cam_sync_init_group_object(sync_dev->sync_table,
		idx, sync_var,
		num_objs,
		type);
	if (rc < 0) {
		CAM_ERR(CAM_SYNC, "Error: Unable to init row at idx = %ld",
			idx);
		clear_bit(idx, sync_dev->bitmap);
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		return -EINVAL;
	}
	sync_dev->sync_table[idx].sync_manager_idx = sync_manager_idx;
	CAM_DBG(CAM_SYNC, "Init row at idx:%ld to merge objects", idx);
	*merged_obj = __add_sync_manager_idx(idx, sync_manager_idx);
	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	return 0;
}

int cam_sync_get_obj_ref(int32_t sync_var)
{
	struct sync_table_row *row = NULL;
	uint32_t sync_obj, sync_manager_idx, uid_validity;
	uint16_t sync_uid;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(sync_var);

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	spin_lock(&sync_dev->row_spinlocks[sync_obj]);

	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, sync_var);
		spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	uid_validity = cam_sync_check_uid_valid(sync_var);
	if (uid_validity == SYNC_UID_NEW) {
		cam_sync_reinit_object(sync_dev->sync_table, sync_var);
	} else if (uid_validity == SYNC_UID_OLD) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, " Called for invalid fence, sync obj: %d, uid: %d",
			sync_obj,
			sync_uid);
		return -EINVAL;
	}

	if (row->state != CAM_SYNC_STATE_ACTIVE) {
		spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}

	atomic_inc(&row->ref_cnt);
	spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
	CAM_DBG(CAM_SYNC, "get ref for obj %d, uid: %d", sync_obj, sync_uid);

	return 0;
}

int cam_sync_put_obj_ref(int32_t sync_var)
{
	struct sync_table_row *row = NULL;
	uint32_t sync_obj, sync_manager_idx;
	uint16_t sync_uid;
	sync_manager_idx = get_sync_manager_idx(sync_var);

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with synx 0x%x",
			row->sync_manager_idx, sync_var);
		return -EINVAL;
	}
	atomic_dec(&row->ref_cnt);
	CAM_DBG(CAM_SYNC, "put ref for obj %d, row_state:%u, uid: %d",
		sync_obj,
		row->state,
		sync_uid);

	return 0;
}

int cam_sync_destroy(int32_t sync_var)
{
	return cam_sync_deinit_object(sync_dev->sync_table, sync_var);
}

int cam_sync_check_valid(int32_t sync_var)
{
	struct sync_table_row *row = NULL;
	uint32_t sync_obj;
	uint16_t sync_uid;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	if (!test_bit(sync_obj, sync_dev->bitmap)) {
		CAM_ERR(CAM_SYNC, "Error: Released sync obj received %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}
	return 0;
}

int cam_sync_wait(int32_t sync_var, uint64_t timeout_ms)
{
	unsigned long timeleft;
	int rc = -EINVAL;
	struct sync_table_row *row = NULL;
	uint32_t sync_obj, sync_manager_idx;
	uint16_t sync_uid;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(sync_var);

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d], uid = %d",
			row->name,
			sync_obj,
			sync_uid);
		return -EINVAL;
	}
	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, sync_var);
		return -EINVAL;
	}
	timeleft = cam_common_wait_for_completion_timeout(&row->signaled,
		msecs_to_jiffies(timeout_ms));

	if (!timeleft) {
		CAM_ERR(CAM_SYNC,
			"Error: timed out for sync obj = %s[%d], uid = %d",
			row->name,
			sync_obj,
			sync_uid);
		rc = -ETIMEDOUT;
	} else {
		switch (row->state) {
		case CAM_SYNC_STATE_INVALID:
		case CAM_SYNC_STATE_ACTIVE:
		case CAM_SYNC_STATE_SIGNALED_ERROR:
		case CAM_SYNC_STATE_SIGNALED_CANCEL:
			CAM_ERR(CAM_SYNC,
				"Error: Wait on invalid state = %d, obj = %d, name = %s, uid = %d",
				row->state, sync_obj, row->name, sync_uid);
			rc = -EINVAL;
			break;
		case CAM_SYNC_STATE_SIGNALED_SUCCESS:
			rc = 0;
			break;
		default:
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

static int cam_sync_handle_create(int sync_manager_idx, struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_info sync_create;
	int result;

	if (k_ioctl->size != sizeof(struct cam_sync_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_create,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;
	sync_create.name[SYNC_DEBUG_NAME_LEN] = '\0';

	result = cam_sync_create(sync_manager_idx, &sync_create.sync_obj,
		sync_create.name, CAM_SYNC_TYPE_UMD);

	if (!result)
		if (copy_to_user(
			u64_to_user_ptr(k_ioctl->ioctl_ptr),
			&sync_create,
			k_ioctl->size))
			return -EFAULT;

	return result;
}

static int cam_sync_handle_signal(struct cam_private_ioctl_arg *k_ioctl)
{
	int rc = 0;
	struct cam_sync_signal sync_signal;
	struct cam_sync_signal_param param;
	uint32_t uid_validity, sync_obj, sync_manager_idx;

	if (k_ioctl->size != sizeof(struct cam_sync_signal))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_signal,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	sync_obj = (uint32_t)sync_signal.sync_obj & sync_uid_access.fenceIdMask;
	sync_manager_idx = get_sync_manager_idx(sync_signal.sync_obj);

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;
	if (sync_manager_idx != sync_dev->sync_table[sync_obj].sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			sync_dev->sync_table[sync_obj].sync_manager_idx, sync_signal.sync_obj);
		return -EINVAL;
	}

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	uid_validity = cam_sync_check_uid_valid(sync_signal.sync_obj);
	if (uid_validity == SYNC_UID_NEW) {
		rc = cam_sync_reinit_object(sync_dev->sync_table, sync_signal.sync_obj);
	} else if (uid_validity == SYNC_UID_OLD) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "Signaling an old fence, sync : %d (0x%x)",
			sync_signal.sync_obj, sync_signal.sync_obj);
		return -EINVAL;
	}
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	/* need to get ref for UMD signaled fences */
	rc = cam_sync_get_obj_ref(sync_signal.sync_obj);
	if (rc) {
		CAM_DBG(CAM_SYNC,
			"Error: cannot signal an uninitialized sync obj = %d",
			sync_signal.sync_obj);
		return rc;
	}
	memset(&param, 0, sizeof(param));
	param.sync_obj = sync_signal.sync_obj;
	param.status = sync_signal.sync_state;
	param.event_cause = CAM_SYNC_COMMON_SYNC_SIGNAL_EVENT;
	return cam_sync_signal(&param, NULL);
}

static int cam_sync_handle_merge(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_merge sync_merge;
	uint32_t *sync_objs;
	uint32_t num_objs;
	uint32_t size;
	int result;

	if (k_ioctl->size != sizeof(struct cam_sync_merge))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_merge,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	if (sync_merge.num_objs >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	size = sizeof(uint32_t) * sync_merge.num_objs;
	sync_objs = kzalloc(size, GFP_ATOMIC);

	if (!sync_objs)
		return -ENOMEM;

	if (copy_from_user(sync_objs,
		u64_to_user_ptr(sync_merge.sync_objs),
		sizeof(uint32_t) * sync_merge.num_objs)) {
		kfree(sync_objs);
		return -EFAULT;
	}

	num_objs = sync_merge.num_objs;

	result = cam_sync_merge(sync_objs,
		num_objs,
		&sync_merge.merged,
		CAM_SYNC_TYPE_UMD);

	if (!result)
		if (copy_to_user(
			u64_to_user_ptr(k_ioctl->ioctl_ptr),
			&sync_merge,
			k_ioctl->size)) {
			kfree(sync_objs);
			return -EFAULT;
	}

	kfree(sync_objs);

	return result;
}

static int cam_sync_handle_wait(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_wait sync_wait;

	if (k_ioctl->size != sizeof(struct cam_sync_wait))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_wait,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	k_ioctl->result = cam_sync_wait(sync_wait.sync_obj,
		sync_wait.timeout_ms);

	return 0;
}

static inline int cam_sync_handle_exit_poll(void *fh)
{
	return cam_sync_util_send_exit_poll_event(fh);
}

static int cam_sync_handle_destroy(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_info sync_create;

	if (k_ioctl->size != sizeof(struct cam_sync_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_create,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	return cam_sync_destroy(sync_create.sync_obj);
}

static int cam_sync_handle_register_user_payload(
	struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_userpayload_info userpayload_info;
	struct sync_user_payload *user_payload_kernel;
	struct sync_user_payload *user_payload_iter;
	struct sync_user_payload *temp_upayload_kernel;
	uint32_t sync_obj;
	struct sync_table_row *row = NULL;
	uint16_t sync_uid;
	uint32_t rc, uid_validity, sync_manager_idx;

	if (k_ioctl->size != sizeof(struct cam_sync_userpayload_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&userpayload_info,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	sync_obj = (uint32_t)userpayload_info.sync_obj & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)userpayload_info.sync_obj >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(userpayload_info.sync_obj);
	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;
	if (sync_manager_idx != sync_dev->sync_table[sync_obj].sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			sync_dev->sync_table[sync_obj].sync_manager_idx, userpayload_info.sync_obj);
		return -EINVAL;
	}
	user_payload_kernel = kzalloc(sizeof(*user_payload_kernel), GFP_KERNEL);
	if (!user_payload_kernel)
		return -ENOMEM;

	memcpy(user_payload_kernel->payload_data,
		userpayload_info.payload,
		CAM_SYNC_PAYLOAD_WORDS * sizeof(__u64));

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row =  sync_dev->sync_table + sync_obj;

	uid_validity = cam_sync_check_uid_valid(userpayload_info.sync_obj);

	if (uid_validity == SYNC_UID_NEW) {
		rc = cam_sync_reinit_object(sync_dev->sync_table, userpayload_info.sync_obj);
	} else if (uid_validity == SYNC_UID_OLD) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "Called for invalid fence, sync obj: %d, uid: %d",
			sync_obj,
			sync_uid);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		kfree(user_payload_kernel);
		return -EINVAL;
	}

	if ((row->state == CAM_SYNC_STATE_SIGNALED_SUCCESS) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) {

		cam_sync_util_send_v4l2_event(CAM_SYNC_V4L_EVENT_ID_CB_TRIG,
			sync_obj,
			row->state, 0, 0,
			user_payload_kernel->payload_data,
			CAM_SYNC_USER_PAYLOAD_SIZE * sizeof(__u64),
			CAM_SYNC_COMMON_REG_PAYLOAD_EVENT, NULL,
			sync_dev->cam_sync_eventq[sync_manager_idx]);

		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		kfree(user_payload_kernel);
		return 0;
	}

	list_for_each_entry_safe(user_payload_iter,
		temp_upayload_kernel,
		&row->user_payload_list,
		list) {
		if (user_payload_iter->payload_data[0] ==
				user_payload_kernel->payload_data[0] &&
			user_payload_iter->payload_data[1] ==
				user_payload_kernel->payload_data[1]) {

			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			kfree(user_payload_kernel);
			return -EALREADY;
		}
	}

	list_add_tail(&user_payload_kernel->list, &row->user_payload_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return 0;
}

static int cam_sync_handle_deregister_user_payload(
	struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_userpayload_info userpayload_info;
	struct sync_user_payload *user_payload_kernel, *temp;
	uint32_t sync_obj, sync_manager_idx;
	struct sync_table_row *row = NULL;
	uint16_t sync_uid;

	if (k_ioctl->size != sizeof(struct cam_sync_userpayload_info)) {
		CAM_ERR(CAM_SYNC, "Incorrect ioctl size");
		return -EINVAL;
	}

	if (!k_ioctl->ioctl_ptr) {
		CAM_ERR(CAM_SYNC, "Invalid embedded ioctl ptr");
		return -EINVAL;
	}

	if (copy_from_user(&userpayload_info,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	sync_obj = (uint32_t)userpayload_info.sync_obj & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)userpayload_info.sync_obj >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(userpayload_info.sync_obj);
	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	if (sync_manager_idx != row->sync_manager_idx) {
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, userpayload_info.sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}
	if (row->uid != sync_uid) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "deregister called for invalid fence, sync obj: %d, uid: %d",
			sync_obj,
			sync_uid);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	list_for_each_entry_safe(user_payload_kernel, temp,
				&row->user_payload_list, list) {
		if (user_payload_kernel->payload_data[0] ==
				userpayload_info.payload[0] &&
				user_payload_kernel->payload_data[1] ==
				userpayload_info.payload[1]) {
			list_del_init(&user_payload_kernel->list);
			kfree(user_payload_kernel);
		}
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return 0;
}

static inline int get_syncmanager_index(void *fh)
{
	int i;

	for (i = 0; i < CAM_SYNC_MAX_SYNC_MANAGER; i++) {
		if (sync_dev->cam_sync_eventq[i] == fh)
			return i;
	}
	return -EINVAL;
}

static int cam_generic_fence_handle_dma_create(
	uint32_t sync_manager_idx, struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i, dma_fence_row_idx;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		rc = cam_dma_fence_create_fd(&fence_cfg->dma_fence_fd,
			&dma_fence_row_idx, fence_cfg->name);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to create dma fence at index: %d rc: %d num fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			goto out_copy;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"Created dma_fence @ i: %d fence fd: %d[%s] num fences [requested: %u processed: %u] ",
			i, fence_cfg->dma_fence_fd, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_release(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i;
	bool failed = false;
	struct cam_dma_fence_release_params release_params;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		release_params.use_row_idx = false;
		release_params.u.dma_fence_fd = fence_cfg->dma_fence_fd;
		rc = cam_dma_fence_release(&release_params);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to destroy dma fence at index: %d fd %d rc: %d num fences [requested: %u processed: %u]",
				i, fence_cfg->dma_fence_fd, rc,
				fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			/* Continue to release other fences, but mark the call as failed */
			failed = true;
			continue;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"Released dma_fence @ i: %d fd: %d num fences [requested: %u processed: %u]",
			i, fence_cfg->dma_fence_fd,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

	if (failed)
		rc = -ENOMSG;

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_import(
	uint32_t sync_manager_idx, struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int32_t rc = 0, i, dma_fence_row_idx;
	struct dma_fence *fence = NULL;
	struct cam_dma_fence_create_sync_obj_payload dma_sync_create;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		/* Check if fd is for a valid dma fence */
		fence = cam_dma_fence_get_fence_from_fd(fence_cfg->dma_fence_fd,
			&dma_fence_row_idx);
		if (IS_ERR_OR_NULL(fence)) {
			CAM_ERR(CAM_DMA_FENCE,
				"Invalid dma fence for fd: %d", fence_cfg->dma_fence_fd);
			fence_cfg->reason_code = -EINVAL;
			goto out_copy;
		}

		dma_sync_create.dma_fence_row_idx = dma_fence_row_idx;
		dma_sync_create.fd = fence_cfg->dma_fence_fd;
		dma_sync_create.sync_created_with_dma = false;

		/* Create new sync object and associate dma fence */
		rc = cam_sync_create_util(sync_manager_idx, &fence_cfg->sync_obj, fence_cfg->name,
			&dma_sync_create, NULL, CAM_SYNC_TYPE_UMD);
		if (rc) {
			fence_cfg->reason_code = rc;

			/* put on the import refcnt */
			cam_dma_fence_get_put_ref(false, dma_fence_row_idx);
			goto out_copy;
		}

		/* Register a cb for dma fence */
		rc = cam_dma_fence_register_cb(&fence_cfg->sync_obj,
			&dma_fence_row_idx, cam_sync_dma_fence_cb);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to register cb for dma fence fd: %d sync_obj: %d rc: %d",
				fence_cfg->dma_fence_fd, fence_cfg->sync_obj, rc);
			cam_sync_deinit_object(sync_dev->sync_table, fence_cfg->sync_obj);
			fence_cfg->reason_code = rc;
			goto out_copy;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"dma fence fd = %d imported for sync_obj = %d[%s] num fences [requested: %u processed: %u]",
			fence_cfg->dma_fence_fd, fence_cfg->sync_obj, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_signal(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	struct cam_dma_fence_signal signal_dma_fence;

	if (fence_cmd_args->input_data_size != sizeof(struct cam_dma_fence_signal)) {
		CAM_ERR(CAM_DMA_FENCE, "Size is invalid expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_dma_fence_signal),
			fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	if (copy_from_user(&signal_dma_fence, (void __user *)fence_cmd_args->input_handle,
		fence_cmd_args->input_data_size))
		return -EFAULT;

	return cam_dma_fence_signal_fd(&signal_dma_fence);
}

static int cam_generic_fence_process_dma_fence_cmd(
	uint32_t sync_manager_idx, uint32_t id, int32_t fence_cmd_args_flag,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	if (unlikely(fence_cmd_args_flag & CAM_GENERIC_FENCE_CMD_IS_ONE_TO_MANY_MANY))
		return rc;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
		rc = cam_generic_fence_handle_dma_create(sync_manager_idx, fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_RELEASE:
		rc = cam_generic_fence_handle_dma_release(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_IMPORT:
		rc = cam_generic_fence_handle_dma_import(sync_manager_idx, fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_SIGNAL:
		rc = cam_generic_fence_handle_dma_signal(fence_cmd_args);
		break;
	default:
		CAM_ERR(CAM_DMA_FENCE, "IOCTL cmd: %u not supported for dma fence", id);
		break;
	}

	return rc;
}

static int cam_generic_fence_cmd_parse_params(
	struct cam_generic_fence_cmd_args *cmd_args,
	int32_t requested_param_mask)
{
	uint32_t index = 0, num_entries, result = 0;

	if (!cmd_args->num_valid_params || !requested_param_mask) {
		CAM_DBG(CAM_SYNC,
			"No params configured num_valid = %d requested_mask = 0x%x",
			cmd_args->num_valid_params, requested_param_mask);
		return 0;
	}

	if (!(cmd_args->valid_param_mask & requested_param_mask)) {
		CAM_DBG(CAM_SYNC,
			"Requested parameter not set in additional param mask expecting: 0x%x actual: 0x%x",
			requested_param_mask, cmd_args->valid_param_mask);
		return 0;
	}

	index = ffs(requested_param_mask) - 1;
	num_entries = ARRAY_SIZE(cmd_args->params);
	if (index >= num_entries) {
		CAM_DBG(CAM_SYNC,
			"Obtained index %u from mask: 0x%x num_param_entries: %u, index exceeding max",
			index, requested_param_mask, num_entries);
		return 0;
	}

	result = cmd_args->params[index];
	return result;
}

int cam_sync_synx_core_recovery(
	enum cam_sync_fencing_client_cores core_id)
{
	int rc = -EOPNOTSUPP;

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	  rc = cam_synx_core_recovery(core_id);
#endif

	return rc;
}

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
static inline int cam_sync_signal_synx_fence_util(
	struct sync_synx_obj_info *synx_obj_info, uint32_t status)
{
	struct cam_synx_obj_signal signal_synx_obj;
	uint32_t synx_row_idx;

	signal_synx_obj.status = status;
	signal_synx_obj.synx_obj = synx_obj_info->synx_obj;
	synx_row_idx = synx_obj_info->synx_obj_row_idx;
	return cam_synx_obj_internal_signal(synx_row_idx, &signal_synx_obj);
}

static int cam_generic_fence_validate_signal_input_info_util(
	int32_t fence_type,
	struct cam_generic_fence_cmd_args *fence_cmd_args,
	struct cam_generic_fence_signal_info **fence_signal_info,
	void **fence_signal_data)
{
	int rc = 0;
	struct cam_generic_fence_signal_info *signal_info = NULL;
	void *signal_data;
	uint32_t num_fences;
	size_t expected_size;

	*fence_signal_info = NULL;
	*fence_signal_data = NULL;

	if (fence_cmd_args->input_data_size !=
		sizeof(struct cam_generic_fence_signal_info)) {
		CAM_ERR(CAM_SYNC, "Size is invalid expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_generic_fence_signal_info),
			fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	signal_info = memdup_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_cmd_args->input_data_size);
	if (IS_ERR_OR_NULL(signal_info)) {
		CAM_ERR(CAM_SYNC, "memdup failed for hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		return -ENOMEM;
	}

	/* Validate num fences */
	num_fences = signal_info->num_fences_requested;
	if ((num_fences == 0) || (num_fences > CAM_GENERIC_FENCE_BATCH_MAX)) {
		CAM_ERR(CAM_SYNC, "Invalid number of fences: %u for batching",
			num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	if (signal_info->fence_handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SYNC, "Invalid signal handle type: %d",
			signal_info->fence_handle_type);
		rc = -EINVAL;
		goto free_mem;
	}

	/* Validate sizes */
	switch (fence_type) {
	case CAM_GENERIC_FENCE_TYPE_SYNC_OBJ:
		expected_size = sizeof(struct cam_sync_signal);
		break;
	case CAM_GENERIC_FENCE_TYPE_SYNX_OBJ:
		expected_size = sizeof(struct cam_synx_obj_signal);
		break;
	case CAM_GENERIC_FENCE_TYPE_DMA_FENCE:
		expected_size = sizeof(struct cam_dma_fence_signal);
		break;
	default:
		CAM_ERR(CAM_SYNC, "Unsupported fence type: %u", fence_type);
		rc = -EINVAL;
		goto free_mem;
	}

	if ((signal_info->fence_data_size) != (expected_size * num_fences)) {
		CAM_ERR(CAM_SYNC, "Invalid input size expected: 0x%x actual: 0x%x for fences: %u",
			(expected_size * num_fences), signal_info->fence_data_size, num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	signal_data = memdup_user(u64_to_user_ptr(signal_info->fence_info_hdl),
		signal_info->fence_data_size);
	if (IS_ERR_OR_NULL(signal_data)) {
		CAM_ERR(CAM_SYNC, "memdup failed for hdl: %d size: 0x%x",
			signal_info->fence_info_hdl, signal_info->fence_data_size);
		rc = -ENOMEM;
		goto free_mem;
	}

	*fence_signal_info = signal_info;
	*fence_signal_data = signal_data;
	return rc;

free_mem:
	kfree(signal_info);
	return rc;
}

static void cam_generic_fence_free_signal_input_info_util(
	struct cam_generic_fence_signal_info **fence_signal_info,
	void **fence_signal_data)
{
	void *signal_data = *fence_signal_data;
	struct cam_generic_fence_signal_info *fence_input = *fence_signal_info;

	kfree(signal_data);
	kfree(fence_input);

	*fence_signal_info = NULL;
	*fence_signal_data = NULL;
}

static int cam_generic_fence_handle_synx_create(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i;
	int32_t row_idx, fence_flag;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNX,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;
		fence_flag = 0;

		cam_generic_fence_config_parse_params(fence_cfg,
			CAM_GENERIC_FENCE_CONFIG_FLAG_PARAM_INDEX, &fence_flag);

		rc = cam_synx_obj_create(fence_cfg->name, NULL,
			fence_flag, &fence_cfg->synx_obj, &row_idx);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Failed to create synx fence at index: %d rc: %d num fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			goto out_copy;
		}

		CAM_DBG(CAM_SYNX,
			"Created synx fence @ i: %d synx_obj: %d[%s] num fences [requested: %u processed: %u] ",
			i, fence_cfg->synx_obj, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_SYNX, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_synx_release(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i;
	bool failed = false;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;
	struct cam_synx_obj_release_params synx_release_params;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNX,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		synx_release_params.use_row_idx = false;
		synx_release_params.u.synx_obj = fence_cfg->synx_obj;

		rc = cam_synx_obj_release(&synx_release_params);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Failed to release synx object at index: %d rc: %d num fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			/* Continue to release other fences, but mark the call as failed */
			failed = true;
			continue;
		}

		CAM_DBG(CAM_SYNX,
			"Released synx object @ i: %d handle: %d num fences [requested: %u processed: %u]",
			i, fence_cfg->synx_obj,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

	if (failed)
		rc = -ENOMSG;

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_SYNX, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_sync_synx_associate_obj(int32_t sync_obj, uint32_t synx_obj,
	int32_t synx_obj_row_idx, bool *is_sync_obj_signaled)
{
	struct sync_table_row *row = NULL;
	struct cam_synx_obj_signal signal_synx_obj;
	struct sync_ext_fence_info *ext_fence_info;
	int rc = 0;
	uint32_t sync_object;

	sync_object = sync_obj & sync_uid_access.fenceIdMask;

	row = sync_dev->sync_table + sync_object;
	spin_lock(&sync_dev->row_spinlocks[sync_object]);
	if (unlikely(test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask))) {
		CAM_ERR(CAM_SYNC,
			"sync_obj: %s[%d] has already been associated with a synx_hdl",
			row->name, sync_obj);
		rc = -EINVAL;
	} else if (row->state != CAM_SYNC_STATE_ACTIVE) {
		signal_synx_obj.status = row->state;
		signal_synx_obj.synx_obj = synx_obj;
		*is_sync_obj_signaled = true;
		goto signal_synx;
	} else {
		rc = cam_sync_get_ext_fence_payload(&ext_fence_info);
		if (rc) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_object]);
			return rc;
		}

		ext_fence_info->synx_obj_info.synx_obj_row_idx = synx_obj_row_idx;
		ext_fence_info->synx_obj_info.sync_created_with_synx = false;
		ext_fence_info->synx_obj_info.synx_obj = synx_obj;
		ext_fence_info->synx_obj_info.is_valid = true;
		set_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask);
		list_add_tail(&ext_fence_info->list, &row->ext_fences);

		CAM_DBG(CAM_SYNX, "sync_obj: %s[%d] associated with synx_obj: %d",
			row->name, sync_obj, ext_fence_info->synx_obj_info.synx_obj);
	}

	spin_unlock(&sync_dev->row_spinlocks[sync_object]);
	return rc;

signal_synx:
	spin_unlock(&sync_dev->row_spinlocks[sync_object]);
	return cam_synx_obj_signal_obj(&signal_synx_obj);
}

static int cam_generic_fence_handle_synx_import(
	uint32_t sync_manager_idx,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int32_t rc = 0, i, synx_obj_row_idx;
	struct sync_synx_obj_info synx_sync_create;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;
	bool is_sync_obj_signaled = false;
	bool is_sync_obj_created = false;
	uint32_t uid_validity, sync_obj;
	uint16_t sync_uid;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNX,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;
		is_sync_obj_signaled = false;
		is_sync_obj_created = false;

		sync_obj = fence_cfg->sync_obj & sync_uid_access.fenceIdMask;
		sync_uid = fence_cfg->sync_obj >> sync_uid_access.uidShift;

		spin_lock(&sync_dev->row_spinlocks[sync_obj]);
		/* Check if it is a valid fence, i.e., either current fence uid or newer */
		uid_validity = cam_sync_check_uid_valid(fence_cfg->sync_obj);
		if (uid_validity == SYNC_UID_NEW) {
			rc = cam_sync_reinit_object(sync_dev->sync_table, fence_cfg->sync_obj);
		} else if (uid_validity == SYNC_UID_OLD) {
			CAM_ERR(CAM_SYNC, "Invalid fence, sync obj: %d, sync_uid: %u",
				sync_obj, sync_uid);
			spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
			return -EINVAL;
		}
		spin_unlock(&sync_dev->row_spinlocks[sync_obj]);

		/* Check if synx handle is for a valid synx obj */
		rc = cam_synx_obj_find_obj_in_table(fence_cfg->synx_obj,
			&synx_obj_row_idx);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Invalid synx obj for handle: %d", fence_cfg->synx_obj);
			fence_cfg->reason_code = -EINVAL;
			goto out_copy;
		}


		if (!cam_sync_check_valid(fence_cfg->sync_obj)) {
			/* Associate synx object with existing sync object */
			CAM_DBG(CAM_SYNX, "cam_sync_synx_associate_obj for sync: %u, synx: %u",
				fence_cfg->sync_obj, fence_cfg->synx_obj);
			rc = cam_sync_synx_associate_obj(fence_cfg->sync_obj,
				fence_cfg->synx_obj, synx_obj_row_idx,
				&is_sync_obj_signaled);
		} else {
			/* Create new sync object and associate synx object */
			synx_sync_create.sync_created_with_synx = false;
			synx_sync_create.synx_obj = fence_cfg->synx_obj;
			synx_sync_create.synx_obj_row_idx = synx_obj_row_idx;

			rc = cam_sync_create_util(sync_manager_idx, &fence_cfg->sync_obj,
				fence_cfg->name, NULL, &synx_sync_create, CAM_SYNC_TYPE_UMD);
			is_sync_obj_created = true;
		}

		if (rc) {
			fence_cfg->reason_code = rc;
			CAM_ERR(CAM_SYNX, "synx associate failed for synx: %u sync: %u",
				fence_cfg->synx_obj, fence_cfg->sync_obj);
			goto out_copy;
		}

		if (!is_sync_obj_signaled) {
			/* Register a cb for synx_obj */
			rc = cam_synx_obj_register_cb(&fence_cfg->sync_obj,
				synx_obj_row_idx, cam_sync_synx_obj_cb);
			if (rc) {
				CAM_ERR(CAM_SYNX,
					"Failed to register cb for synx_obj: %d sync_obj: %d rc: %d",
					fence_cfg->synx_obj, fence_cfg->sync_obj, rc);
				if (is_sync_obj_created)
					cam_sync_deinit_object(sync_dev->sync_table,
						fence_cfg->sync_obj);
				fence_cfg->reason_code = rc;
				goto out_copy;
			}
		}

		CAM_DBG(CAM_SYNX,
			"synx_obj handle = %d imported for dma fence fd: %d sync_obj = %d[%s] num fences [requested: %u processed: %u]",
			fence_cfg->synx_obj, fence_cfg->dma_fence_fd,
			fence_cfg->sync_obj, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNX, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_synx_signal(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int32_t rc = 0, i;
	struct cam_generic_fence_signal_info *fence_signal_info;
	struct cam_synx_obj_signal *synx_signal_info;

	rc = cam_generic_fence_validate_signal_input_info_util(
		CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, fence_cmd_args,
		&fence_signal_info, (void **)&synx_signal_info);
	if (rc || !fence_signal_info || !synx_signal_info) {
		CAM_ERR(CAM_SYNX,
			"Fence input signal info validation failed rc: %d fence_input_info: %pK synx_signal_info: %pK",
			rc, fence_signal_info, synx_signal_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_signal_info->num_fences_requested; i++) {
		fence_signal_info->num_fences_processed++;

		rc = cam_synx_obj_signal_obj(&synx_signal_info[i]);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Failed to signal for synx_obj: %d, rc: %d, status : %d",
				synx_signal_info[i].synx_obj, rc,
				synx_signal_info[i].status);
		}

		synx_signal_info[i].reason_code = rc;
	}

	if (copy_to_user(u64_to_user_ptr(fence_signal_info->fence_info_hdl), synx_signal_info,
		fence_signal_info->fence_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNX, "copy to user for signal data failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle,
			(sizeof(struct cam_synx_obj_signal) *
			fence_signal_info->num_fences_requested));
		goto end;
	}

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_signal_info, sizeof(struct cam_generic_fence_signal_info))) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNX, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle,
			sizeof(struct cam_generic_fence_signal_info));
}

end:
	cam_generic_fence_free_signal_input_info_util(&fence_signal_info,
		(void **)&synx_signal_info);
	return rc;
}

static int cam_generic_fence_process_synx_obj_cmd(
	uint32_t sync_manager_idx, uint32_t id, int32_t fence_cmd_args_flag,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	if (unlikely(fence_cmd_args_flag & CAM_GENERIC_FENCE_CMD_IS_ONE_TO_MANY_MANY))
		return rc;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
		rc = cam_generic_fence_handle_synx_create(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_RELEASE:
		rc = cam_generic_fence_handle_synx_release(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_IMPORT:
		rc = cam_generic_fence_handle_synx_import(sync_manager_idx, fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_SIGNAL:
		rc = cam_generic_fence_handle_synx_signal(fence_cmd_args);
		break;
	default:
		CAM_ERR(CAM_SYNX, "IOCTL cmd: %u not supported for synx object", id);
		break;
	}

	return rc;
}
#endif

#if IS_REACHABLE(CONFIG_CAM_ENABLE_SOCCP)
static int cam_sync_find_free_bit_util(
	void *bitmap, uint32_t num_bits, long *free_idx)
{
	long idx;
	bool bit;

	do {
		idx = find_first_zero_bit(bitmap, num_bits);
		if (idx >= num_bits) {
			CAM_ERR(CAM_SYNC, "No free idx available");
			return -ENOMEM;
		}
		bit = test_and_set_bit(idx, bitmap);
	} while (bit);

	*free_idx = idx;
	return 0;
}

static bool cam_sync_validate_and_get_hw_fence_client_info(
	int32_t session_hdl, uint32_t *client_core, uint32_t *col_index)
{
	CAM_SYNC_GET_CLIENT_INFO_FROM_SESSION_HDL(session_hdl, client_core, col_index);

	return CAM_SYNC_VALIDATE_HW_FENCE_CLIENT_INFO(client_core, col_index);
}

static int __cam_sync_validate_hw_fence_client_entry(int32_t session_hdl,
	struct cam_sync_hw_fence_client_entries *entry)
{
	int rc = 0;

	if (entry->cookie != session_hdl) {
		CAM_ERR(CAM_SYNC,
				"Cookie [0x%x] mismatch for session_hdl: 0x%x",
				entry->cookie, session_hdl);
		rc = -EINVAL;
		goto end;
	}

	if (!entry->active) {
		CAM_ERR(CAM_SYNC,
				"Session with hdl: 0x%x is not active", session_hdl);
		rc = -EINVAL;
	}

end:
	return rc;
}

static inline int cam_synx_update_hw_fence_queue_util(
	struct sync_table_row *row,
	struct cam_sync_hwfence_info *hwfence_info,
	int32_t *wr_pnt_arr_idx)
{
	int rc = 0;
	int32_t client_entry_idx;
	struct cam_sync_hw_fence_client_entries *client_entry;
	struct synx_session *session_hdl;
	struct sync_ext_fence_info *ext_fence_info, *ext_fence_tmp;
	int32_t wr_ptr_idx;

	wr_ptr_idx = *wr_pnt_arr_idx;
	client_entry_idx = row->hw_fence_client_idx;
	spin_lock(hw_fence_info.hw_fence_locks[client_entry_idx]);
	client_entry = &hw_fence_info.hw_fence_tbl[client_entry_idx];
	session_hdl = client_entry->session_hdl;
	spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);

	list_for_each_entry_safe(ext_fence_info, ext_fence_tmp, &row->ext_fences,
		list) {
		if (ext_fence_info->synx_obj_info.is_valid) {
			rc = cam_synx_update_hw_fence_queue(session_hdl,
				ext_fence_info->synx_obj_info.synx_obj);
			if (rc) {
				CAM_ERR(CAM_SYNC,
					"Failed to update hw fence queue for synx: 0x%x",
					ext_fence_info->synx_obj_info.synx_obj);
				goto end;
			}
			hwfence_info->wr_pntr_arr[wr_ptr_idx].client_core =
				client_entry->client_core;
			hwfence_info->wr_pntr_arr[wr_ptr_idx].signal_id =
				client_entry->signal_id;
			hwfence_info->wr_pntr_arr[wr_ptr_idx].wr_pntr =
				*(int32_t *)client_entry->txq_wr_ptr;
			wr_ptr_idx++;
		}
	}
	*wr_pnt_arr_idx = wr_ptr_idx;
end:
	return rc;
}

static inline int cam_sync_merged_primary_update_hw_fence_queue(
	struct cam_sync_hwfence_info *hwfence_info)
{
	int rc = 0;
	int32_t wr_pnt_arr_idx = 0;
	struct sync_table_row *row, *child_row = NULL;
	struct sync_child_info *child_info, *child_tmp;

	row = sync_dev->sync_table + hwfence_info->sync_object;

	list_for_each_entry_safe(child_info, child_tmp, &row->children_list, list) {
		child_row = sync_dev->sync_table + child_info->sync_id;
		rc = cam_synx_update_hw_fence_queue_util(child_row, hwfence_info,
			&wr_pnt_arr_idx);
		if (rc) {
			CAM_ERR(CAM_SYNC,
				"Failed to update hw fence queue for child_sync: 0x%x",
				child_info->sync_id);
			goto end;
		}
	}
	hwfence_info->num_hwfences = wr_pnt_arr_idx;

end:
	return rc;
}

int __cam_sync_update_hw_fence_queue_pntr_util_locked(
	struct cam_sync_hwfence_info *hwfence_info)
{
	int rc = 0;
	int32_t wr_pnt_arr_idx = 0;
	struct sync_table_row *row = NULL;

	row = sync_dev->sync_table + hwfence_info->sync_object;

	/* If the sync object is a merged parent with one to many */
	if (row->is_merged_primary) {
		rc =
			cam_sync_merged_primary_update_hw_fence_queue(hwfence_info);
		if (rc) {
			CAM_ERR(CAM_SYNC,
					"Failed to update hw fence queue for merged_primary_sync: 0x%x",
					hwfence_info->sync_object);
			goto end;
		}
	} else {
		rc = cam_synx_update_hw_fence_queue_util(row, hwfence_info, &wr_pnt_arr_idx);
		if (rc) {
			CAM_ERR(CAM_SYNC,
				"Failed to update hw fence queue for child_sync: 0x%x",
				hwfence_info->sync_object);
			goto end;
		}
		hwfence_info->num_hwfences = wr_pnt_arr_idx;
	}
end:
	return rc;
}

static int cam_generic_fence_alloc_validate_one_to_many_util(
	struct cam_generic_fence_cmd_args    *fence_cmd_args,
	struct cam_generic_fence_one_to_many_input_info **fence_input_info)
{
	int rc = 0, i;
	struct cam_generic_fence_one_to_many_input_info *fence_input = NULL;
	uint32_t num_fences;
	uint32_t expected_size;
	uint8_t *payload = NULL;
	struct cam_generic_fence_create_associated_array *associated_arr = NULL;
	uint32_t associated_arr_totalfences = 0, associated_arrtotalsize = 0, current_size = 0;
	*fence_input_info = NULL;

	if (unlikely(fence_cmd_args->input_data_size == 0xffffffff)) {
		CAM_ERR(CAM_SYNC, "Size is invalid: 0x%llx",
				fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	if (fence_cmd_args->input_data_size <
			sizeof(struct cam_generic_fence_one_to_many_input_info)) {
		CAM_ERR(CAM_SYNC, "Size is invalid min expected: 0x%llx actual: 0x%llx",
				sizeof(struct cam_generic_fence_one_to_many_input_info),
				fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	fence_input = memdup_user(u64_to_user_ptr(fence_cmd_args->input_handle),
			fence_cmd_args->input_data_size);
	if (IS_ERR_OR_NULL(fence_input)) {
		CAM_ERR(CAM_SYNC, "memdup failed for hdl: %d size: 0x%x",
				fence_cmd_args->input_handle,
				fence_cmd_args->input_data_size);
		return -ENOMEM;
	}

	/* Validate num fences */
	num_fences = fence_input->num_fences_requested;
	if ((num_fences == 0) || (num_fences > CAM_GENERIC_FENCE_BATCH_MAX)) {
		CAM_ERR(CAM_SYNC, "Invalid number of fences: %u for batching",
				num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	/* Validate sizes */
	payload = (uint8_t *)fence_input->payload;
	associated_arr = (struct cam_generic_fence_create_associated_array *)payload;
	for (i = 0; i < fence_input->num_fences_requested; i++) {
		associated_arr = (struct cam_generic_fence_create_associated_array *)
			((uint8_t *)associated_arr + current_size);
		associated_arr_totalfences += associated_arr->num_fences_requested;
		current_size = sizeof(struct cam_generic_fence_create_associated_array) +
			(associated_arr->num_fences_requested *
			sizeof(struct cam_generic_fence_config));
	}

	associated_arrtotalsize = (fence_input->num_fences_requested *
		sizeof(struct cam_generic_fence_create_associated_array)) +
		(associated_arr_totalfences * sizeof(struct cam_generic_fence_config));

	expected_size = sizeof(struct cam_generic_fence_one_to_many_input_info) +
		associated_arrtotalsize;

	if ((uint32_t)expected_size != fence_cmd_args->input_data_size) {
		CAM_ERR(CAM_SYNC, "Invalid input size expected: 0x%x actual: 0x%x for fences: %u",
				expected_size, fence_cmd_args->input_data_size,
				num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	*fence_input_info = fence_input;
	return rc;

free_mem:
	kfree(fence_input);
	return rc;
}

static int cam_generic_fence_create_hw_fence(
	uint32_t session_cookie, struct cam_generic_fence_config *fence_cfg,
	int32_t sync_obj)
{
	int rc = -EINVAL, synx_obj_row_idx, dma_fence_row_idx;
	unsigned long fence_sel_mask = 0;
	struct sync_table_row *row = NULL;
	bool is_valid = false;
	struct sync_ext_fence_info *ext_fence_info;
	void *native_fence = NULL;
	uint32_t client_core = 0, col_idx = 0, client_entry_idx = 0, fence_flag = 0;
	struct synx_session *session_hdl;
	struct cam_sync_hw_fence_client_entries *client_entry;

	is_valid = cam_sync_validate_and_get_hw_fence_client_info(session_cookie,
				&client_core, &col_idx);
	if (!is_valid)
		goto end;

	rc = cam_sync_get_ext_fence_payload(&ext_fence_info);
	if (rc)
		goto end;

	fence_sel_mask = (unsigned long)fence_cfg->fence_sel_mask;
	client_entry_idx = (client_core * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS) + col_idx;

	spin_lock(hw_fence_info.hw_fence_locks[client_entry_idx]);
	client_entry = &hw_fence_info.hw_fence_tbl[client_entry_idx];
	rc = __cam_sync_validate_hw_fence_client_entry(session_cookie,
			client_entry);
	if (rc) {
		spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);
		goto free_fence_payload;
	}
	session_hdl = client_entry->session_hdl;
	spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);

	cam_generic_fence_config_parse_params(fence_cfg,
			CAM_GENERIC_FENCE_CONFIG_FLAG_PARAM_INDEX, &fence_flag);

	rc = cam_synx_obj_create(fence_cfg->name, session_hdl,
			fence_flag, &fence_cfg->synx_obj, &synx_obj_row_idx);
	if (rc)
		goto free_fence_payload;

	ext_fence_info->synx_obj_info.synx_obj = fence_cfg->synx_obj;
	ext_fence_info->synx_obj_info.synx_obj_row_idx = synx_obj_row_idx;
	ext_fence_info->synx_obj_info.is_valid = true;

	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
		native_fence = cam_synx_obj_get_native_fence(session_hdl,
				fence_cfg->synx_obj);
		if (IS_ERR_OR_NULL(native_fence)) {
			CAM_ERR(CAM_SYNC,
					"Invalid native fence for synx: 0x%x",
					fence_cfg->synx_obj);
			rc =  PTR_ERR(native_fence);
			goto end;
		}

		rc = cam_dma_fence_get_fd(fence_cfg->name, native_fence,
				&fence_cfg->dma_fence_fd, &dma_fence_row_idx, true);
		if (rc)
			goto free_synx;

		ext_fence_info->dma_fence_info.dma_fence_fd =
			fence_cfg->dma_fence_fd;
		ext_fence_info->dma_fence_info.dma_fence_row_idx =
			dma_fence_row_idx;
		ext_fence_info->dma_fence_info.is_valid = true;
	}

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;
	set_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask);
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask))
		set_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask);

	list_add_tail(&ext_fence_info->list, &row->ext_fences);
	set_bit(CAM_GENERIC_FENCE_TYPE_HW_FENCE, &row->ext_fence_mask);
	row->hw_fence_client_idx = client_entry_idx;
	row->signaling_en = false;
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	fence_cfg->reason_code = 0;
	CAM_DBG(CAM_SYNC, "HW Fence created for session cookie: %u, sync obj: %u, synx_obj: %u",
		session_cookie, sync_obj, fence_cfg->synx_obj);
	return rc;

free_synx: {
		struct cam_synx_obj_release_params release_params;

		release_params.use_row_idx = true;
		release_params.u.synx_row_idx = synx_obj_row_idx;

		cam_synx_obj_release(&release_params);
		fence_cfg->synx_obj = -1;
	}
free_fence_payload:
	cam_sync_put_ext_fence_payload(&ext_fence_info);
end:
	fence_cfg->reason_code = rc;
	return rc;
}

static void cam_generic_fence_free_one_to_many_util(
	struct cam_generic_fence_one_to_many_input_info **fence_input_info)
{
	struct cam_generic_fence_one_to_many_input_info *fence_input =
		*fence_input_info;

	kfree(fence_input);
	*fence_input_info = NULL;
}
static int cam_generic_fence_create_associated_fences_util(
	uint32_t sync_manager_idx,
	struct cam_generic_fence_create_associated_array *associated_array)
{
	int i, rc;
	int32_t sync_obj = 0;
	uint8_t *fence_cfg = NULL;
	bool for_hw_fence = false;

	fence_cfg = (uint8_t *)associated_array;
	fence_cfg += offsetof(struct cam_generic_fence_create_associated_array,
			fence_cfg);

	for_hw_fence =
		CAM_SYNC_IS_HW_FENCE_SESSION(associated_array->session_cookie);

	if (for_hw_fence) {
		rc = cam_sync_create_util(sync_manager_idx, &sync_obj,
				associated_array->sync_obj_name, NULL, NULL, CAM_SYNC_TYPE_UMD);
		if (rc)
			goto end;
	}

	for (i = 0; i < associated_array->num_fences_requested; i++) {
		associated_array->num_fences_processed++;
		if (for_hw_fence) {
			rc = cam_generic_fence_create_hw_fence(associated_array->session_cookie,
				(struct cam_generic_fence_config *)fence_cfg, sync_obj);
			if (rc)
				goto sync_destroy;
		} else {
			/* For sw fence, creating fence same as generic_fence_handle_sync_create */
			rc = cam_generic_fence_handle_sync_synx_dma_create(sync_manager_idx,
					(struct cam_generic_fence_config *)fence_cfg,
					associated_array->num_fences_requested);
			if (rc) {
				CAM_ERR(CAM_SYNC, "SW sync/synx/dma create failed, rc: %d", rc);
				goto end;
			}
		}
		fence_cfg += sizeof(struct cam_generic_fence_config);
	}

	if (for_hw_fence)
		associated_array->sync_obj = sync_obj;
	return rc;

sync_destroy:
	cam_sync_deinit_object(sync_dev->sync_table, sync_obj);
end:
	return rc;
}

static int cam_generic_fence_handle_one_to_many_create(
	uint32_t sync_manager_idx, struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc, i, merged_primary;
	struct sync_child_info *child_info;
	size_t total_computed_size = 0, current_size = 0;
	size_t fence_cfg_size, associated_arr_size;
	struct sync_table_row *row = NULL;
	uint8_t *payload = NULL;
	struct cam_generic_fence_one_to_many_input_info *fence_input_info = NULL;
	struct cam_generic_fence_create_associated_array *associated_arr = NULL;

	rc = cam_generic_fence_alloc_validate_one_to_many_util(fence_cmd_args,
			&fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNC,
				"Fence input info validation failed rc: %d fence_input_info: %pK",
				rc, fence_input_info);
		return -EINVAL;
	}

	fence_cfg_size = sizeof(struct cam_generic_fence_config);
	associated_arr_size = sizeof(struct cam_generic_fence_create_associated_array);
	total_computed_size = sizeof(struct cam_generic_fence_one_to_many_input_info);

	if (fence_input_info->flags & CAM_GENERIC_FENCE_CMD_CREATE_PRIMARY_SYNC) {
		rc = cam_sync_create_util(sync_manager_idx, &fence_input_info->merged_primary_sync,
					fence_input_info->merged_primary_name, NULL, NULL,
					CAM_SYNC_TYPE_UMD);
		if (rc)
			goto end;

		merged_primary = fence_input_info->merged_primary_sync;
		spin_lock_bh(&sync_dev->row_spinlocks[merged_primary]);
		row = sync_dev->sync_table + merged_primary;
		row->is_merged_primary = true;
		set_bit(CAM_GENERIC_FENCE_TYPE_HW_FENCE, &row->ext_fence_mask);
		spin_unlock_bh(&sync_dev->row_spinlocks[merged_primary]);
	}

	payload = (uint8_t *)fence_input_info->payload;
	associated_arr = (struct cam_generic_fence_create_associated_array *)payload;
	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		total_computed_size += associated_arr_size;

		if (unlikely(fence_cmd_args->input_data_size < total_computed_size)) {
			rc = -EINVAL;
			goto end;
		}

		associated_arr = (struct cam_generic_fence_create_associated_array *)
			((uint8_t *)associated_arr + current_size);
		total_computed_size += ((associated_arr->num_fences_requested) *
			sizeof(struct cam_generic_fence_config));

		if (unlikely(fence_cmd_args->input_data_size < total_computed_size)) {
			rc = -EINVAL;
			goto end;
		}

		rc = cam_generic_fence_create_associated_fences_util(sync_manager_idx,
			associated_arr);
		if (rc)
			goto end;

		if (merged_primary) {
			spin_lock_bh(&sync_dev->row_spinlocks[merged_primary]);
			child_info = kzalloc(sizeof(*child_info), GFP_ATOMIC);
			if (!child_info) {
				spin_unlock_bh(&sync_dev->row_spinlocks[merged_primary]);
				rc = -ENOMEM;
				goto end;
			}
			child_info->sync_id = associated_arr->sync_obj;
			list_add_tail(&child_info->list, &row->children_list);
			spin_unlock_bh(&sync_dev->row_spinlocks[merged_primary]);
		}

		current_size = associated_arr_size +
			(associated_arr->num_fences_requested *
			sizeof(struct cam_generic_fence_config));

	}

end:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
				fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNC, "copy to user failed hdl: %d size: 0x%x",
				fence_cmd_args->input_handle,
				fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_one_to_many_util(&fence_input_info);
	return rc;
}

int cam_sync_check_and_update_hw_fence_queue_pntrs(
	struct cam_sync_hwfence_info *hwfence_info)
{
	int rc = 0, sync_object;
	struct sync_table_row *row = NULL;

	if (!hwfence_info)
		return -EINVAL;

	rc = cam_sync_check_valid(hwfence_info->sync_object);
	if (rc)
		goto end;

	sync_object = (uint32_t)hwfence_info->sync_object & sync_uid_access.fenceIdMask;
	row = sync_dev->sync_table + sync_object;

	spin_lock(&sync_dev->row_spinlocks[sync_object]);
	/*
	 * If the sync object is an object, but is a HW fence and
	 * If the sync object is a merged parent with one to many.
	 */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_HW_FENCE, &row->ext_fence_mask)) {
		hwfence_info->is_hwfence = true;
		rc =
			__cam_sync_update_hw_fence_queue_pntr_util_locked(hwfence_info);
		if (rc) {
			spin_unlock(&sync_dev->row_spinlocks[sync_object]);
			goto end;
		}
	}
	/* SW fence */
	else
		CAM_DBG(CAM_SYNC, "sync_object: %d is an object but is a SW fence", sync_object);

	spin_unlock(&sync_dev->row_spinlocks[sync_object]);

end:
	return rc;
}
void cam_sync_send_ssr_error_v4l2_event(uint32_t id, uint32_t event_cause)
{
	struct v4l2_event event;

	if (sync_dev->version == CAM_SYNC_V4L_EVENT_V2) {
		struct cam_sync_ev_header_v2 *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V2;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V2(event);
		ev_header->version = sync_dev->version;
		ev_header->evt_param[CAM_SYNC_EVENT_REASON_CODE_INDEX] =
			event_cause;
	} else {
		struct cam_sync_ev_header *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT;

		ev_header = CAM_SYNC_GET_HEADER_PTR(event);
		ev_header->status = event_cause;
	}

	v4l2_event_queue(sync_dev->vdev, &event);
	CAM_ERR(CAM_SYNC, "FATAL error: send v4l2 event for SocCP SSR, sync_dev version %d",
			sync_dev->version);
}

static int cam_sync_soccp_ssr_notify(struct notifier_block *nb, unsigned long action,
	void *data)
{
	struct qcom_ssr_notify_data *notify_data = data;

	if (!notify_data) {
		CAM_ERR(CAM_SYNC, "ssr notify data is NULL");
		return NOTIFY_BAD;
	}

	switch (action) {
	case QCOM_SSR_BEFORE_POWERUP:
		CAM_INFO(CAM_SYNC, "received SocCP starting event");
		break;
	case QCOM_SSR_AFTER_POWERUP:
		CAM_INFO(CAM_SYNC, "received SocCP running event");
		break;
	case QCOM_SSR_BEFORE_SHUTDOWN:
		CAM_ERR(CAM_SYNC, "received SocCP %s event",
			notify_data->crashed ? "crashed" : "stopping");
		cam_sync_send_ssr_error_v4l2_event(CAM_SYNC_V4L_EVENT_ID_SOCCP_SSR_ERROR,
				CAM_SYNC_COMMON_SOCCP_SSR_EVENT);
		break;
	case QCOM_SSR_AFTER_SHUTDOWN:
		CAM_INFO(CAM_SYNC, "received SocCP offline event");
		break;
	default:
		CAM_ERR(CAM_SYNC, "received unrecognized event %lu", action);
		break;
	}

	return NOTIFY_OK;
}
int cam_sync_initialize_hw_fence_session(
	struct cam_sync_hwfence_session_initialize_params *init_params)
{
	int rc = 0;
	uint32_t client_entry_idx;
	long idx;
	void *txq_wr_ptr;
	struct cam_sync_hw_fence_client_entries *client_entry;
	struct synx_session *synx_session = NULL;
	void *notifier;

	if (!sync_dev->hw_fencing_en)
		return -EOPNOTSUPP;

	synx_session = cam_synx_initialize_hw_fence_session(init_params, &txq_wr_ptr);
	if (IS_ERR_OR_NULL(synx_session))
		return -EINVAL;

	rc = cam_sync_find_free_bit_util(hw_fence_info.client_bitmaps[init_params->client_core],
		hw_fence_info.num_bits, &idx);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"Failed to find free idx for client: %d", init_params->client_core);
		rc = EINVAL;
		goto deinitialize_session;
	}

	client_entry_idx = (init_params->client_core * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS) +
		(uint32_t)idx;
	spin_lock(hw_fence_info.hw_fence_locks[client_entry_idx]);
	client_entry = &hw_fence_info.hw_fence_tbl[client_entry_idx];
	if (client_entry->active) {
		CAM_ERR(CAM_SYNC, "Entry for client: %u at index: %u is still active",
			init_params->client_core, idx);
		clear_bit(idx, hw_fence_info.client_bitmaps[init_params->client_core]);
		spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);
		goto deinitialize_session;
	}

	client_entry->active = true;
	client_entry->client_core = init_params->client_core;
	client_entry->signal_id = init_params->signal_id;
	client_entry->txq_wr_ptr = txq_wr_ptr;
	client_entry->fence_protocol = init_params->fencing_protocol;
	client_entry->session_hdl = synx_session;

	client_entry->cookie = CAM_SYNC_GENERATE_SYNX_CLIENT_SESSION_HDL(
		init_params->client_core, idx);
	init_params->session_cookie = client_entry->cookie;
	spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);

	if (!sync_dev->ref_cnt) {
		sync_dev->nb.notifier_call = cam_sync_soccp_ssr_notify;
		notifier = qcom_register_ssr_notifier("soccp", &sync_dev->nb);
		if (IS_ERR(notifier)) {
			CAM_ERR(CAM_SYNC, "Failed to register for soccp ssr notifier: %d",
				PTR_ERR(notifier));
			return PTR_ERR(notifier);
		}
		sync_dev->notifier = notifier;
	}
	sync_dev->ref_cnt++;

	return rc;

deinitialize_session:
	cam_sync_deinitialize_hw_fence_session(init_params->session_cookie);
	return rc;
}

int cam_sync_deinitialize_hw_fence_session(int32_t session_hdl)
{
	int rc = 0;
	bool is_valid = false;
	uint32_t client_core = 0, col_idx = 0, client_entry_idx = 0;
	struct synx_session *synx_session;
	struct cam_sync_hw_fence_client_entries *client_entry;

	if (!sync_dev->hw_fencing_en)
		return -EOPNOTSUPP;

	is_valid = cam_sync_validate_and_get_hw_fence_client_info(session_hdl,
		&client_core, &col_idx);
	if (!is_valid) {
		CAM_ERR(CAM_SYNC, "Invalid session_hdl: 0x%x client_core: %u col_idx: %u",
			session_hdl, client_core, col_idx);
		return -EINVAL;
	}

	client_entry_idx = (client_core * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS) + col_idx;
	spin_lock(hw_fence_info.hw_fence_locks[client_entry_idx]);
	client_entry = &hw_fence_info.hw_fence_tbl[client_entry_idx];

	rc = __cam_sync_validate_hw_fence_client_entry(session_hdl, client_entry);
	if (rc) {
		spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);
		goto end;
	}

	synx_session = client_entry->session_hdl;

	memset(client_entry, 0x0, sizeof(*client_entry));
	spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);

	rc = cam_synx_uninitialize_hw_fence_session(synx_session);
	if (rc) {
		CAM_ERR(CAM_SYNC, "Synx session uninitialize failed rc: %d", rc);
		goto end;
	}

	if (sync_dev->ref_cnt)
		sync_dev->ref_cnt--;

	if (!sync_dev->ref_cnt) {
		rc = qcom_unregister_ssr_notifier(sync_dev->notifier, &sync_dev->nb);
		if (rc)
			CAM_ERR(CAM_SYNC, "error %d unregistering soccp ssr notifier", rc);

		sync_dev->notifier = NULL;
		memset(&sync_dev->nb, 0, sizeof(sync_dev->nb));
	}
end:
	return rc;
}

int cam_sync_hw_fence_session_cleanup(void)
{
	int i, j, rc = 0;
	uint32_t client_entry_idx;
	struct cam_sync_hw_fence_client_entries *client_entry;

	for (i = 0; i < CAM_SYNC_HW_FENCE_MAX_CLIENTS; i++) {
		for (j = 0; j < CAM_SYNC_HW_FENCE_MAX_SUB_GRPS; j++) {
			client_entry_idx = (i * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS) + j;
			spin_lock(hw_fence_info.hw_fence_locks[client_entry_idx]);
			client_entry = &hw_fence_info.hw_fence_tbl[client_entry_idx];
			if (client_entry->active) {
				clear_bit(j, hw_fence_info.client_bitmaps[i]);
				spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);
				rc = cam_sync_deinitialize_hw_fence_session(
					client_entry->cookie);
				if (rc) {
					return rc;
				}
			} else {
				spin_unlock(hw_fence_info.hw_fence_locks[client_entry_idx]);
			}
		}
	}
	return rc;
}
#else
int cam_sync_check_and_update_hw_fence_queue_pntrs(
	struct cam_sync_hwfence_info *hwfence_info)
{
	return -EOPNOTSUPP;
}

int cam_sync_initialize_hw_fence_session(
	struct cam_sync_hwfence_session_initialize_params *init_params)
{
	return -EOPNOTSUPP;
}

int cam_sync_deinitialize_hw_fence_session(int32_t session_hdl)
{
	return -EOPNOTSUPP;
}

int cam_sync_hw_fence_session_cleanup(void)
{
	return -EOPNOTSUPP;
}
#endif

static int cam_sync_signal_synx_dma_fence_util_locked(int32_t sync_obj,
	struct cam_sync_signal_param *param, struct cam_sync_timestamp *time_stamp)
{
	uint32_t status, event_cause;
	int rc = 0;
	struct sync_table_row *row = NULL;
	struct list_head parents_list;
	struct sync_ext_fence_info *ext_fence_info, *tmp;

	status = param->status;
	event_cause = param->event_cause;

	row = sync_dev->sync_table + sync_obj;
	row->state = param->status;

	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask) ||
		test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask)) {
		list_for_each_entry_safe(ext_fence_info, tmp, &row->ext_fences, list) {
			if ((ext_fence_info->dma_fence_info.is_valid) &&
				((row->hw_fence_client_idx == -1) ||
				(row->state != CAM_SYNC_STATE_SIGNALED_SUCCESS))) {
				rc = cam_sync_signal_dma_fence_util(&ext_fence_info->dma_fence_info,
					row->state);
				if (rc) {
					CAM_ERR(CAM_SYNC,
						"Error: Failed to signal associated dma fencefd = %d for sync_obj = %s[%d]",
						ext_fence_info->dma_fence_info.dma_fence_fd,
						row->name, sync_obj);
				}
			}

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
			/* signaling synx incase of SW fence only */
			if ((row->hw_fence_client_idx == -1) &&
				ext_fence_info->synx_obj_info.is_valid) {
				spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
				rc = cam_sync_signal_synx_fence_util(&ext_fence_info->synx_obj_info,
					row->state);
				spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
				if (rc) {
					CAM_ERR(CAM_SYNC,
						"Error: Failed to signal associated synx obj = %d for sync_obj = %d",
						ext_fence_info->synx_obj_info.synx_obj, sync_obj);
				}
			}
#endif
		}
	}

	cam_sync_util_dispatch_signaled_cb(param, time_stamp);

	/* copy parent list to local and release child lock */
	INIT_LIST_HEAD(&parents_list);
	list_splice_init(&row->parents_list, &parents_list);

	if (list_empty(&parents_list))
		return 0;

	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	cam_sync_signal_parent_util(param, &parents_list, time_stamp);
	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);

	return rc;
}

int cam_sync_signal(struct cam_sync_signal_param *param, struct cam_sync_timestamp *time_stamp)
{
	struct sync_table_row *row = NULL;
	struct sync_table_row *child_row = NULL;
	struct sync_child_info *child_info, *child_tmp;
	int rc = 0;
	uint32_t sync_obj, uid_validity, sync_manager_idx, sync_var;
	int16_t sync_uid;

	sync_var = param->sync_obj;
	sync_obj = param->sync_obj & sync_uid_access.fenceIdMask;
	sync_uid = param->sync_obj >> sync_uid_access.uidShift;
	sync_manager_idx = get_sync_manager_idx(param->sync_obj);
	param->fh = sync_dev->cam_sync_eventq[sync_manager_idx];

	if ((sync_obj >= CAM_SYNC_MAX_OBJS) || (sync_obj <= 0)) {
		CAM_ERR(CAM_SYNC, "Error: Out of range sync obj (0 <= %d < %d)",
			sync_obj, CAM_SYNC_MAX_OBJS);
		return -EINVAL;
	}

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	/* Check if it is a valid fence, i.e., either current fence uid or newer */
	uid_validity = cam_sync_check_uid_valid(param->sync_obj);
	if (uid_validity == SYNC_UID_NEW) {
		rc = cam_sync_reinit_object(sync_dev->sync_table, param->sync_obj);
	} else if (uid_validity == SYNC_UID_OLD) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "Signaling an old fence, sync obj: %d, uid: %d, state: %d",
			sync_obj,
			sync_uid,
			param->status);
		return -EINVAL;
	}

	if (sync_manager_idx != row->sync_manager_idx) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC, "sync manager idx for row %d don't match with sync 0x%x",
			row->sync_manager_idx, param->sync_obj);
		return -EINVAL;
	}

	rc = cam_sync_signal_validate_util(sync_obj, param->status);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"Error: Failed to validate signal info for sync_obj = %s[%d] with status = %d rc = %d",
			row->name, sync_obj, param->status, rc);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return rc;
	}

	if (!atomic_dec_and_test(&row->ref_cnt)) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return 0;
	}

	row->state = param->status;

	list_for_each_entry_safe(child_info, child_tmp, &row->children_list, list) {
		spin_lock_bh(&sync_dev->row_spinlocks[child_info->sync_id]);
		child_row = sync_dev->sync_table + child_info->sync_id;
		param->sync_obj = child_info->sync_id;
		rc = cam_sync_signal_synx_dma_fence_util_locked(child_info->sync_id, param,
			time_stamp);
		if (rc)
			CAM_ERR(CAM_SYNC,
				"Error: Failed to signal child_sync %d in merged_primay_sync_obj = %s[%d]",
				child_row->name, child_info->sync_id, sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[child_info->sync_id]);
	}

	param->sync_obj = sync_var;
	rc = cam_sync_signal_synx_dma_fence_util_locked(sync_obj, param, time_stamp);
	if (rc)
		CAM_ERR(CAM_SYNC,
			"Error: Failed to signal sync_obj = %s[%d]",
			row->name, sync_obj);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	return 0;
}

static int cam_generic_fence_handle_sync_create(
	uint32_t sync_manager_idx, struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc, i;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNC,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;
		rc = cam_generic_fence_handle_sync_synx_dma_create(sync_manager_idx,
			fence_cfg, fence_input_info->num_fences_requested);
		if (rc) {
			CAM_ERR(CAM_SYNC, "sync/synx/dma create failed at index:%d, rc: %d",
				i, rc);
			goto out_copy;
		}

		CAM_DBG(CAM_SYNC,
			"Created sync_obj = %d[%s] with fence_sel_mask: 0x%x dma_fence_fd: %d num fences [requested: %u processed: %u]",
			fence_cfg->sync_obj, fence_cfg->name,
			fence_cfg->fence_sel_mask, fence_cfg->dma_fence_fd,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNC, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}
	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_sync_release(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	bool failed = false;
	int rc, i;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNC,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		/* Reset fields */
		fence_cfg->reason_code = 0;
		rc = cam_sync_deinit_object(sync_dev->sync_table, fence_cfg->sync_obj);
		if (rc) {
			fence_cfg->reason_code = rc;
			failed = true;
			CAM_ERR(CAM_SYNC,
				"Failed to release sync obj at index: %d rc: %d num_fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
		}

		CAM_DBG(CAM_SYNC,
			"Released sync_obj = %d[%s] num fences [requested: %u processed: %u]",
			fence_cfg->sync_obj, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

	if (failed)
		rc = -ENOMSG;

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNC, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util((void **)&fence_input_info);
	return rc;
}


static int cam_generic_fence_process_one_to_many_cmds(
	uint32_t sync_manager_idx, uint32_t id,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
#if IS_REACHABLE(CONFIG_CAM_ENABLE_SOCCP)
		rc = cam_generic_fence_handle_one_to_many_create(sync_manager_idx, fence_cmd_args);
		break;
#endif
	default:
		CAM_ERR(CAM_SYNC, "IOCTL cmd: %u not supported for sync object", id);
		break;
	}

	return rc;
}

static int cam_generic_fence_process_one_to_one_cmds(uint32_t sync_manager_idx,
	uint32_t id, struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
		rc = cam_generic_fence_handle_sync_create(sync_manager_idx,
			fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_RELEASE:
		rc = cam_generic_fence_handle_sync_release(fence_cmd_args);
		break;
	default:
		CAM_ERR(CAM_SYNC, "IOCTL cmd: %u not supported for sync object",
			id);
		break;
	}

	return rc;
}

static int cam_generic_fence_process_sync_obj_cmd(
	uint32_t sync_manager_idx, uint32_t id, int32_t fence_cmd_args_flag,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	if (fence_cmd_args_flag & CAM_GENERIC_FENCE_CMD_IS_ONE_TO_MANY_MANY)
		rc = cam_generic_fence_process_one_to_many_cmds(sync_manager_idx, id,
			fence_cmd_args);
	else
		rc = cam_generic_fence_process_one_to_one_cmds(sync_manager_idx, id,
			fence_cmd_args);


	return rc;
}


static int cam_generic_fence_parser(
	uint32_t sync_manager_idx, struct cam_private_ioctl_arg *k_ioctl)
{
	int rc;
	uint32_t flag = 0;
	struct cam_generic_fence_cmd_args fence_cmd_args;

	if (!k_ioctl->ioctl_ptr) {
		CAM_ERR(CAM_SYNC, "Invalid args input ptr: %p",
			k_ioctl->ioctl_ptr);
		return -EINVAL;
	}

	if (k_ioctl->size != sizeof(struct cam_generic_fence_cmd_args)) {
		CAM_ERR(CAM_SYNC, "Size mismatch expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_generic_fence_cmd_args), k_ioctl->size);
		return -EINVAL;
	}

	if (copy_from_user(&fence_cmd_args, u64_to_user_ptr(k_ioctl->ioctl_ptr),
		sizeof(fence_cmd_args))) {
		CAM_ERR(CAM_SYNC, "copy from user failed for input ptr: %pK",
			k_ioctl->ioctl_ptr);
		return -EFAULT;
	}

	if (fence_cmd_args.input_handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SYNC, "Invalid handle type: %u",
			fence_cmd_args.input_handle_type);
		return -EINVAL;
	}

	flag = cam_generic_fence_cmd_parse_params(&fence_cmd_args,
		CAM_GENERIC_FENCE_CMD_FLAG_PARAM_INDEX);

	switch (fence_cmd_args.fence_type) {
	case CAM_GENERIC_FENCE_TYPE_SYNC_OBJ:
		rc = cam_generic_fence_process_sync_obj_cmd(sync_manager_idx, k_ioctl->id,
			flag, &fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_TYPE_DMA_FENCE:
		rc = cam_generic_fence_process_dma_fence_cmd(sync_manager_idx, k_ioctl->id,
			flag, &fence_cmd_args);
		break;
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	case CAM_GENERIC_FENCE_TYPE_SYNX_OBJ:
		rc = cam_generic_fence_process_synx_obj_cmd(sync_manager_idx, k_ioctl->id,
			flag, &fence_cmd_args);
		break;
#endif
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_SYNC, "fence type: 0x%x handling not supported",
			fence_cmd_args.fence_type);
		break;
	}

	return rc;
}

static long cam_sync_dev_ioctl(struct file *filep, void *fh,
		bool valid_prio, unsigned int cmd, void *arg)
{
	int32_t rc, sync_manager_idx;
	struct sync_device *sync_dev = video_drvdata(filep);
	struct cam_private_ioctl_arg k_ioctl;

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "sync_dev NULL");
		return -EINVAL;
	}

	if (!arg)
		return -EINVAL;

	if (cmd != CAM_PRIVATE_IOCTL_CMD)
		return -ENOIOCTLCMD;

	sync_manager_idx = get_syncmanager_index(fh);
	if (sync_manager_idx < 0) {
		CAM_ERR(CAM_SYNC, "sync manager not found");
		return -EINVAL;
	}
	k_ioctl = *(struct cam_private_ioctl_arg *)arg;

	switch (k_ioctl.id) {
	case CAM_SYNC_CREATE:
		rc = cam_sync_handle_create(sync_manager_idx, &k_ioctl);
		break;
	case CAM_SYNC_DESTROY:
		rc = cam_sync_handle_destroy(&k_ioctl);
		break;
	case CAM_SYNC_REGISTER_PAYLOAD:
		rc = cam_sync_handle_register_user_payload(
			&k_ioctl);
		break;
	case CAM_SYNC_DEREGISTER_PAYLOAD:
		rc = cam_sync_handle_deregister_user_payload(
			&k_ioctl);
		break;
	case CAM_SYNC_SIGNAL:
		rc = cam_sync_handle_signal(&k_ioctl);
		break;
	case CAM_SYNC_MERGE:
		rc = cam_sync_handle_merge(&k_ioctl);
		break;
	case CAM_SYNC_WAIT:
		rc = cam_sync_handle_wait(&k_ioctl);
		((struct cam_private_ioctl_arg *)arg)->result =
			k_ioctl.result;
		break;
	case CAM_GENERIC_FENCE_CREATE:
	case CAM_GENERIC_FENCE_RELEASE:
	case CAM_GENERIC_FENCE_IMPORT:
	case CAM_GENERIC_FENCE_SIGNAL:
		rc = cam_generic_fence_parser(sync_manager_idx, &k_ioctl);
		break;
	case CAM_SYNC_EXIT_DQ_THREAD:
		rc = cam_sync_handle_exit_poll(fh);
		break;
	default:
		rc = -ENOIOCTLCMD;
	}

	return rc;
}

static unsigned int cam_sync_poll(struct file *f,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	struct v4l2_fh *eventq = f->private_data;

	if (!eventq)
		return -EINVAL;

	poll_wait(f, &eventq->wait, pll_table);

	if (v4l2_event_pending(eventq))
		rc = POLLPRI;

	return rc;
}

static int cam_sync_open(struct file *filep)
{
	int rc, idx, bit;
	struct sync_device *sync_dev = video_drvdata(filep);

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "Sync device NULL");
		return -ENODEV;
	}

	do {
		idx = find_first_zero_bit(sync_dev->bitmap_syncmanager, CAM_SYNC_MAX_SYNC_MANAGER);
		if (idx >= CAM_SYNC_MAX_SYNC_MANAGER) {
			CAM_ERR(CAM_SYNC,
				"Error: Unable to create syncmanager reached max!");
			return -EALREADY;
		}
		CAM_INFO(CAM_SYNC, "Sync manager Index location available at idx: %ld", idx);
		bit = test_and_set_bit(idx, sync_dev->bitmap_syncmanager);
	} while (bit);

	rc = v4l2_fh_open(filep);
	if (!rc) {
		if (!sync_dev->open_cnt)
			cam_dma_fence_open();
		sync_dev->open_cnt++;
		spin_lock_bh(&sync_dev->cam_sync_eventq_lock[idx]);
		sync_dev->cam_sync_eventq[idx] = filep->private_data;
		spin_unlock_bh(&sync_dev->cam_sync_eventq_lock[idx]);
	} else {
		CAM_ERR(CAM_SYNC, "v4l2_fh_open failed : %d", rc);
	}

	return rc;
}

static int cam_sync_close(struct file *filep)
{
	int rc = 0;
	int i;
	struct sync_device *sync_dev = video_drvdata(filep);
	struct cam_sync_signal_param param;
	void *fh = filep->private_data;
	uint32_t sync_manager_idx = get_syncmanager_index(fh);

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "Sync device NULL");
		rc = -ENODEV;
		return rc;
	}
	for (i = 1; i < CAM_SYNC_MAX_OBJS; i++) {
		struct sync_table_row *row =
		sync_dev->sync_table + i;
		/*
		 * Signal all ACTIVE objects as ERR, but we don't
		 * care about the return status here apart from logging
		 * it.
		 */
		if ((row->sync_manager_idx == sync_manager_idx) &&
				(row->state == CAM_SYNC_STATE_ACTIVE)) {
			memset(&param, 0, sizeof(param));
			param.sync_obj = __add_sync_manager_idx(i, sync_manager_idx);
			param.status = CAM_SYNC_STATE_SIGNALED_ERROR;
			param.event_cause = CAM_SYNC_COMMON_RELEASE_EVENT;
			rc = cam_sync_signal(&param, NULL);
			if (rc < 0)
				CAM_ERR(CAM_SYNC,
				  "Cleanup signal fail idx:%d\n",
				  i);
		}
	}

	/*
	 * Flush the work queue to wait for pending signal callbacks to
	 * finish
	 */
	cam_req_mgr_worker_flush(sync_dev->worker);

	/* Clean dma fence table */
	cam_dma_fence_close();
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	/* Clean synx obj table */
	cam_synx_obj_close();
#endif

	/*
	 * Now that all callbacks worker threads have finished,
	 * destroy the sync objects
	 */
	for (i = 1; i < CAM_SYNC_MAX_OBJS; i++) {
		struct sync_table_row *row =
		sync_dev->sync_table + i;

		if ((row->sync_manager_idx == sync_manager_idx) &&
				(row->state != CAM_SYNC_STATE_INVALID)) {
			rc = cam_sync_destroy(__add_sync_manager_idx(i, sync_manager_idx));
			if (rc < 0)
				CAM_ERR(CAM_SYNC,
				  "Cleanup destroy fail:idx:%d\n",
				  i);
		}
	}
	sync_dev->open_cnt--;
	if (!sync_dev->open_cnt)
		cam_dma_fence_close();
	spin_lock_bh(&sync_dev->cam_sync_eventq_lock[sync_manager_idx]);
	sync_dev->cam_sync_eventq[sync_manager_idx] = NULL;
	clear_bit(sync_manager_idx, sync_dev->bitmap_syncmanager);
	spin_unlock_bh(&sync_dev->cam_sync_eventq_lock[sync_manager_idx]);
	v4l2_fh_release(filep);

	return rc;
}

static void cam_sync_event_queue_notify_error(const struct v4l2_event *old,
	struct v4l2_event *new)
{
	if (sync_dev->version == CAM_SYNC_V4L_EVENT_V2) {
		struct cam_sync_ev_header_v2 *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V2((*old));
		CAM_ERR(CAM_CRM,
			"Failed to notify event id %d fence %d statue %d reason %u %u %u %u",
			old->id, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param[0], ev_header->evt_param[1],
			ev_header->evt_param[2], ev_header->evt_param[3]);

	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V3) {
		struct cam_sync_ev_header_v3 *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V3((*old));
		CAM_ERR(CAM_CRM,
			"Fail to notify event id %d fence %d status %d reason %u",
			old->id, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param[0]);
	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V4) {
		struct cam_sync_ev_header_v4 *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V4((*old));
		CAM_ERR(CAM_CRM,
			"Fail to notify event id %d fence %d status %d reason %u",
			old->id, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param.event_cause);
	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V5) {
		struct cam_sync_ev_header_v5 *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V5((*old));
		CAM_ERR(CAM_CRM,
			"Fail to notify event id %d fence %d status %d",
			old->id, ev_header->sync_obj, ev_header->status);
	} else {
		struct cam_sync_ev_header *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR((*old));
		CAM_ERR(CAM_CRM,
			"Failed to notify event id %d fence %d statue %d",
			old->id, ev_header->sync_obj, ev_header->status);
	}
}

static struct v4l2_subscribed_event_ops cam_sync_v4l2_ops = {
	.merge = cam_sync_event_queue_notify_error,
};

int cam_sync_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case CAM_SYNC_V4L_EVENT:
	case CAM_SYNC_V4L_EVENT_V2:
	case CAM_SYNC_V4L_EVENT_V3:
	case CAM_SYNC_V4L_EVENT_V4:
	case CAM_SYNC_V4L_EVENT_V5:
		break;
	default:
		CAM_ERR(CAM_SYNC, "Non supported event type 0x%x", sub->type);
		return -EINVAL;
	}

	switch (sub->id) {
	case CAM_SYNC_V4L_EVENT_ID_CB_TRIG:
	case CAM_SYNC_V4L_EVENT_ID_EXIT:
		break;
	default:
		CAM_ERR(CAM_SYNC, "Non supported event id 0x%x", sub->id);
		return -EINVAL;
	}

	sync_dev->version = sub->type;
	CAM_DBG(CAM_SYNC, "Sync event verion type 0x%x", sync_dev->version);
	return v4l2_event_subscribe(fh, sub, CAM_SYNC_MAX_V4L2_EVENTS,
		&cam_sync_v4l2_ops);
}

int cam_sync_unsubscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case CAM_SYNC_V4L_EVENT:
	case CAM_SYNC_V4L_EVENT_V2:
	case CAM_SYNC_V4L_EVENT_V3:
	case CAM_SYNC_V4L_EVENT_V4:
	case CAM_SYNC_V4L_EVENT_V5:
		break;
	default:
		CAM_ERR(CAM_SYNC, "Non supported event type 0x%x", sub->type);
		return -EINVAL;
	}

	return v4l2_event_unsubscribe(fh, sub);
}

static const struct v4l2_ioctl_ops g_cam_sync_ioctl_ops = {
	.vidioc_subscribe_event = cam_sync_subscribe_event,
	.vidioc_unsubscribe_event = cam_sync_unsubscribe_event,
	.vidioc_default = cam_sync_dev_ioctl,
};

static struct v4l2_file_operations cam_sync_v4l2_fops = {
	.owner = THIS_MODULE,
	.open  = cam_sync_open,
	.release = cam_sync_close,
	.poll = cam_sync_poll,
	.unlocked_ioctl   = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
};

#if IS_REACHABLE(CONFIG_MEDIA_CONTROLLER)
static int cam_sync_media_controller_init(struct sync_device *sync_dev,
	struct platform_device *pdev)
{
	int rc;

	sync_dev->v4l2_dev.mdev = kzalloc(sizeof(struct media_device),
		GFP_KERNEL);
	if (!sync_dev->v4l2_dev.mdev)
		return -ENOMEM;

	media_device_init(sync_dev->v4l2_dev.mdev);
	strlcpy(sync_dev->v4l2_dev.mdev->model, CAM_SYNC_DEVICE_NAME,
			sizeof(sync_dev->v4l2_dev.mdev->model));
	sync_dev->v4l2_dev.mdev->dev = &(pdev->dev);

	rc = media_device_register(sync_dev->v4l2_dev.mdev);
	if (rc < 0)
		goto register_fail;

	rc = media_entity_pads_init(&sync_dev->vdev->entity, 0, NULL);
	if (rc < 0)
		goto entity_fail;

	return 0;

entity_fail:
	media_device_unregister(sync_dev->v4l2_dev.mdev);
register_fail:
	media_device_cleanup(sync_dev->v4l2_dev.mdev);
	return rc;
}

static void cam_sync_media_controller_cleanup(struct sync_device *sync_dev)
{
	media_entity_cleanup(&sync_dev->vdev->entity);
	media_device_unregister(sync_dev->v4l2_dev.mdev);
	media_device_cleanup(sync_dev->v4l2_dev.mdev);
	kfree(sync_dev->v4l2_dev.mdev);
}

static void cam_sync_init_entity(struct sync_device *sync_dev)
{
	sync_dev->vdev->entity.function = CAM_SYNC_DEVICE_TYPE;
	sync_dev->vdev->entity.name =
				video_device_node_name(sync_dev->vdev);
}
#else
static int cam_sync_media_controller_init(struct sync_device *sync_dev,
	struct platform_device *pdev)
{
	return 0;
}

static void cam_sync_media_controller_cleanup(struct sync_device *sync_dev)
{
}

static void cam_sync_init_entity(struct sync_device *sync_dev)
{
}
#endif

static int cam_sync_create_debugfs(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	dbgfileptr = debugfs_create_dir("camera_sync", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_SYNC,"DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	sync_dev->dentry = dbgfileptr;

	debugfs_create_bool("trigger_cb_without_switch", 0644,
		sync_dev->dentry, &trigger_cb_without_switch);

end:
	return rc;
}

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
int cam_synx_sync_signal(int32_t sync_obj, uint32_t synx_status)
{
	int rc = 0;
	uint32_t sync_status = synx_status;
	struct cam_sync_signal_param param;

	switch (synx_status) {
	case SYNX_STATE_ACTIVE:
		sync_status = CAM_SYNC_STATE_ACTIVE;
		break;
	case SYNX_STATE_SIGNALED_SUCCESS:
		sync_status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
		break;
	case SYNX_STATE_SIGNALED_ERROR:
		sync_status = CAM_SYNC_STATE_SIGNALED_ERROR;
		break;
	case 4: /* SYNX_STATE_SIGNALED_CANCEL: */
		sync_status = CAM_SYNC_STATE_SIGNALED_CANCEL;
		break;
	default:
		CAM_ERR(CAM_SYNC, "Invalid synx status %d for obj %d",
			synx_status, sync_obj);
		sync_status = CAM_SYNC_STATE_SIGNALED_ERROR;
		break;
	}
	memset(&param, 0, sizeof(param));
	param.sync_obj = sync_obj;
	param.status = sync_status;
	param.event_cause = CAM_SYNC_COMMON_EVENT_SYNX;
	rc = cam_sync_signal(&param, NULL);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"synx signal failed with %d, sync_obj=%d, synx_status=%d, sync_status=%d",
			sync_obj, synx_status, sync_status, rc);
	}

	return rc;
}

static int cam_sync_register_synx_bind_ops(
	struct synx_register_params *object)
{
	int rc = 0;

	rc = synx_register_ops(object);
	if (rc)
		CAM_ERR(CAM_SYNC, "synx registration fail with rc=%d", rc);

	return rc;
}

static void cam_sync_unregister_synx_bind_ops(
	struct synx_register_params *object)
{
	int rc = 0;

	rc = synx_deregister_ops(object);
	if (rc)
		CAM_ERR(CAM_SYNC, "sync unregistration fail with %d", rc);
}

static void cam_sync_configure_synx_obj(struct synx_register_params *object)
{
	struct synx_register_params *params = object;

	params->name = CAM_SYNC_NAME;
	params->type = SYNX_TYPE_CSL;
	params->ops.register_callback = cam_sync_register_callback;
	params->ops.deregister_callback = cam_sync_deregister_callback;
	params->ops.enable_signaling = cam_sync_get_obj_ref;
	params->ops.signal = cam_synx_sync_signal;
}
#endif

static int cam_sync_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc, idx, i, bitmap_iter = 0, fence_lock_iter = 0;
	struct platform_device *pdev = to_platform_device(dev);
	uint32_t num_uid_bits, num_sync_manager_bits;
	const unsigned long max_uid = CAM_SYNC_MAX_UID;
	const unsigned long max_sync_manager_id = CAM_SYNC_MAX_SYNC_MANAGER - 1;

	sync_dev = kvzalloc(sizeof(*sync_dev), GFP_KERNEL);
	if (!sync_dev)
		return -ENOMEM;

	for (idx = 0; idx < CAM_SYNC_MAX_SYNC_MANAGER; idx++)
		spin_lock_init(&sync_dev->cam_sync_eventq_lock[idx]);
	sync_dev->sync_table = kvzalloc(sizeof(struct sync_table_row) * CAM_SYNC_MAX_OBJS,
							GFP_KERNEL);
	if (!sync_dev->sync_table)
		return -ENOMEM;

	for (idx = 0; idx < CAM_SYNC_MAX_OBJS; idx++)
		spin_lock_init(&sync_dev->row_spinlocks[idx]);

	/*
	 * We treat zero as invalid handle, so we will keep the 0th bit set
	 * always
	 */
	set_bit(0, sync_dev->bitmap);

	sync_dev->vdev = video_device_alloc();
	if (!sync_dev->vdev) {
		rc = -ENOMEM;
		goto vdev_fail;
	}

	rc = cam_sync_media_controller_init(sync_dev, pdev);
	if (rc < 0)
		goto mcinit_fail;

	sync_dev->vdev->v4l2_dev = &sync_dev->v4l2_dev;

	rc = v4l2_device_register(&(pdev->dev), sync_dev->vdev->v4l2_dev);
	if (rc < 0)
		goto register_fail;

	strlcpy(sync_dev->vdev->name, CAM_SYNC_NAME,
				sizeof(sync_dev->vdev->name));
	sync_dev->vdev->release  = video_device_release_empty;
	sync_dev->vdev->fops     = &cam_sync_v4l2_fops;
	sync_dev->vdev->ioctl_ops = &g_cam_sync_ioctl_ops;
	sync_dev->vdev->minor     = -1;
	sync_dev->vdev->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
	sync_dev->vdev->vfl_type  = VFL_TYPE_VIDEO;
	rc = video_register_device(sync_dev->vdev, VFL_TYPE_VIDEO, -1);
	if (rc < 0) {
		CAM_ERR(CAM_SYNC,
			"video device registration failure rc = %d, name = %s, device_caps = %d",
			rc, sync_dev->vdev->name, sync_dev->vdev->device_caps);
		goto v4l2_fail;
	}

	cam_sync_init_entity(sync_dev);
	video_set_drvdata(sync_dev->vdev, sync_dev);

	cam_req_mgr_worker_create("sync_worker", 20,
		&sync_dev->worker, CRM_WORKER_USAGE_IRQ, 0);

	if (!sync_dev->worker) {
		CAM_ERR(CAM_SYNC,
			"Error: high priority worker creation failed");
		rc = -ENOMEM;
		goto workqueue_create_fail;
	}
	/* Initialize dma fence driver */
	rc = cam_dma_fence_driver_init();
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"DMA fence driver initialization failed rc: %d", rc);
		goto workqueue_create_fail;

	}

	trigger_cb_without_switch = false;
	cam_sync_create_debugfs();
	num_uid_bits = bitmap_weight(&max_uid, sizeof(sync_dev->sync_table->sync_id));

	/* Setup for HW fencing */
	sync_dev->hw_fencing_en = of_property_read_bool(pdev->dev.of_node, "hw_fencing_en");
	if (sync_dev->hw_fencing_en) {
		uint32_t total_entries, total_size;
		size_t map_size = 0;

		total_entries = (CAM_SYNC_HW_FENCE_MAX_CLIENTS * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS);
		total_size = total_entries * sizeof(struct cam_sync_hw_fence_info);
		hw_fence_info.hw_fence_tbl = kvzalloc(total_size, GFP_KERNEL);
		if (!hw_fence_info.hw_fence_tbl) {
			rc = -ENOMEM;
			goto workqueue_create_fail;
		}

		map_size = BITS_TO_LONGS(CAM_SYNC_HW_FENCE_MAX_SUB_GRPS) * sizeof(long);
		for (bitmap_iter = 0; bitmap_iter < CAM_SYNC_HW_FENCE_MAX_CLIENTS; bitmap_iter++) {
			hw_fence_info.client_bitmaps[bitmap_iter] = kvzalloc(map_size, GFP_KERNEL);
			if (!hw_fence_info.client_bitmaps[bitmap_iter]) {
				rc = -ENOMEM;
				goto hw_fence_info_destroy;
			}
			bitmap_zero(hw_fence_info.client_bitmaps[bitmap_iter],
				CAM_SYNC_HW_FENCE_MAX_SUB_GRPS);
		}
		hw_fence_info.num_bits = map_size * BITS_PER_BYTE;
		for (fence_lock_iter = 0; fence_lock_iter < total_entries; fence_lock_iter++) {
			hw_fence_info.hw_fence_locks[fence_lock_iter] =
				kvzalloc(sizeof(spinlock_t), GFP_KERNEL);
			if (!hw_fence_info.hw_fence_locks[fence_lock_iter]) {
				rc = -ENOMEM;
				goto hw_fence_info_destroy;
			}
			spin_lock_init(hw_fence_info.hw_fence_locks[fence_lock_iter]);
		}
	}

	num_sync_manager_bits = cam_common_get_num_bits_required(max_sync_manager_id);
	sync_uid_access.fenceIdMask =
		GENMASK((sizeof(sync_dev->sync_table->sync_id) * 8) -
			num_uid_bits - num_sync_manager_bits - 1, 0);
	sync_uid_access.uidShift = (sizeof(sync_dev->sync_table->sync_id) * 8) - num_uid_bits;
	sync_uid_access.init_uid_val = max_uid << sync_uid_access.uidShift;
	sync_dev->sync_manager_id_shift =  sync_uid_access.uidShift - num_sync_manager_bits;
	sync_dev->sync_manager_id_mask = GENMASK(num_sync_manager_bits - 1, 0);
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	/* Initialize synx obj driver */
	rc = cam_synx_obj_driver_init();
	if (rc) {
		CAM_ERR(CAM_SYNC,
		"Synx obj driver initialization failed rc: %d", rc);
		goto dma_driver_deinit;
	}
#elif IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
	CAM_DBG(CAM_SYNC, "Registering with synx driver");
	cam_sync_configure_synx_obj(&sync_dev->params);
	rc = cam_sync_register_synx_bind_ops(&sync_dev->params);
	if (rc) {
		CAM_ERR(CAM_SYNC,
		"Synx obj driver initialization failed rc: %d", rc);
		goto dma_driver_deinit;
	}
#endif
	INIT_LIST_HEAD(&sync_dev->free_ext_fence_list);
	for (i = 0; i < CAM_SYNC_MAX_EXT_FENCE_PAYLOADS; i++) {
		INIT_LIST_HEAD(&sync_dev->ext_fence_info[i].list);
		list_add_tail(&sync_dev->ext_fence_info[i].list,
			&sync_dev->free_ext_fence_list);
	}
	spin_lock_init(&sync_dev->payload_lock);

	CAM_DBG(CAM_SYNC, "Component bound successfully");
	return rc;

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX) || IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
dma_driver_deinit:
	cam_dma_fence_driver_deinit();
#endif
hw_fence_info_destroy:
	kvfree(hw_fence_info.hw_fence_tbl);
	hw_fence_info.hw_fence_tbl = NULL;

	for (--bitmap_iter; bitmap_iter >= 0; bitmap_iter--) {
		kvfree(hw_fence_info.client_bitmaps[bitmap_iter]);
		hw_fence_info.client_bitmaps[bitmap_iter] = NULL;
	}

	for (--fence_lock_iter; fence_lock_iter >= 0; fence_lock_iter--) {
		kvfree(hw_fence_info.hw_fence_locks[fence_lock_iter]);
		hw_fence_info.hw_fence_locks[fence_lock_iter] = NULL;
	}
workqueue_create_fail:
	cam_req_mgr_worker_destroy(&sync_dev->worker);
v4l2_fail:
	v4l2_device_unregister(sync_dev->vdev->v4l2_dev);
register_fail:
	cam_sync_media_controller_cleanup(sync_dev);
mcinit_fail:
	video_unregister_device(sync_dev->vdev);
	video_device_release(sync_dev->vdev);
vdev_fail:
	kvfree(sync_dev);
	return rc;
}

static void cam_sync_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int j;

	v4l2_device_unregister(sync_dev->vdev->v4l2_dev);
	cam_sync_media_controller_cleanup(sync_dev);
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	cam_synx_obj_driver_deinit();
#elif IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
	cam_sync_unregister_synx_bind_ops(&sync_dev->params);
#endif
	video_unregister_device(sync_dev->vdev);
	video_device_release(sync_dev->vdev);
	debugfs_remove_recursive(sync_dev->dentry);
	sync_dev->dentry = NULL;

	cam_dma_fence_driver_deinit();

	for (j = 0; j < CAM_SYNC_MAX_OBJS; j++)
		spin_lock_init(&sync_dev->row_spinlocks[j]);

	cam_req_mgr_worker_destroy(&sync_dev->worker);

	if (sync_dev->hw_fencing_en) {
		uint32_t total_entries;

		kvfree(hw_fence_info.hw_fence_tbl);
		hw_fence_info.hw_fence_tbl = NULL;

		for (j = 0; j < CAM_SYNC_HW_FENCE_MAX_CLIENTS; j++) {
			kvfree(hw_fence_info.client_bitmaps[j]);
			hw_fence_info.client_bitmaps[j] = NULL;
		}

		total_entries = CAM_SYNC_HW_FENCE_MAX_CLIENTS * CAM_SYNC_HW_FENCE_MAX_SUB_GRPS;
		for (j = 0; j < total_entries; j++) {
			kvfree(hw_fence_info.hw_fence_locks[j]);
			hw_fence_info.hw_fence_locks[j] = NULL;
		}
	}
	kvfree(sync_dev->sync_table);
}

const static struct component_ops cam_sync_component_ops = {
	.bind = cam_sync_component_bind,
	.unbind = cam_sync_component_unbind,
};

static int cam_sync_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_SYNC, "Adding Sync component");
	rc = component_add(&pdev->dev, &cam_sync_component_ops);
	if (rc)
		CAM_ERR(CAM_SYNC, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_sync_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_sync_component_ops);
	return 0;
}

static const struct of_device_id cam_sync_dt_match[] = {
	{.compatible = "qcom,cam-sync"},
	{}
};

MODULE_DEVICE_TABLE(of, cam_sync_dt_match);

struct platform_driver cam_sync_driver = {
	.probe = cam_sync_probe,
	.remove = cam_sync_remove,
	.driver = {
		.name = "cam_sync",
		.owner = THIS_MODULE,
		.of_match_table = cam_sync_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_sync_init(void)
{
	return platform_driver_register(&cam_sync_driver);
}

void cam_sync_exit(void)
{
	platform_driver_unregister(&cam_sync_driver);
}

MODULE_DESCRIPTION("Camera sync driver");
MODULE_LICENSE("GPL v2");
