// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/soc/qcom/msm_hw_fence.h>

#include "cam_sync_synx.h"
#if IS_REACHABLE(CONFIG_CAM_ENABLE_SOCCP)
#include <synx_extension_api.h>
#endif

/* Max IPCC signals are 64 per client, starting from 0 */
#define HW_FENCE_MAX_IPCC_SIGNALS_PER_CLIENT 63

/**
 * struct cam_synx_obj_row - Synx obj row
 */
struct cam_synx_obj_row {
	char                            name[CAM_SYNX_OBJ_NAME_LEN];
	uint32_t                        synx_obj;
	enum cam_synx_obj_state         state;
	cam_sync_callback_for_synx_obj  sync_cb;
	struct synx_session            *session_hdl;
	bool                            cb_registered_for_sync;
	bool                            sync_signal_synx;
	int32_t                         sync_obj;
};

/**
 * struct cam_synx_obj_device - Synx obj device
 */
struct cam_synx_obj_device {
	struct cam_synx_obj_row rows[CAM_SYNX_MAX_OBJS];
	spinlock_t row_spinlocks[CAM_SYNX_MAX_OBJS];
	struct synx_session *session_handle;
	struct mutex dev_lock;
	DECLARE_BITMAP(bitmap, CAM_SYNX_MAX_OBJS);
};

static struct cam_synx_obj_device *g_cam_synx_obj_dev;
static char cam_synx_session_name[64] = "Camera_Generic_Synx_Session";

static void __cam_synx_obj_signal_cb(u32 h_synx, int status, void *data)
{
	struct cam_synx_obj_signal_sync_obj signal_sync_obj;
	struct cam_synx_obj_row *synx_obj_row = NULL;

	if (!data) {
		CAM_ERR(CAM_SYNX,
			"Invalid data passed to synx obj : %d callback function.",
			synx_obj_row->synx_obj);
		return;
	}

	synx_obj_row = (struct cam_synx_obj_row *)data;

	/* If this synx obj is signaled by sync obj, skip cb */
	if (synx_obj_row->sync_signal_synx)
		return;

	if (synx_obj_row->synx_obj != h_synx) {
		CAM_ERR(CAM_SYNX,
			"Synx obj: %d callback does not match synx obj: %d in sync table.",
			h_synx, synx_obj_row->synx_obj);
		return;
	}

	if (synx_obj_row->state == CAM_SYNX_OBJ_STATE_INVALID) {
		CAM_ERR(CAM_SYNX,
			"Synx obj :%d is in invalid state: %d",
			synx_obj_row->synx_obj, synx_obj_row->state);
		return;
	}

	CAM_DBG(CAM_SYNX, "Synx obj: %d signaled, signal sync obj: %d",
		 synx_obj_row->synx_obj, synx_obj_row->sync_obj);

	if ((synx_obj_row->cb_registered_for_sync) && (synx_obj_row->sync_cb)) {
		signal_sync_obj.synx_obj = synx_obj_row->synx_obj;
		switch (status) {
		case SYNX_STATE_SIGNALED_SUCCESS:
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
			break;
		case SYNX_STATE_SIGNALED_CANCEL:
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_CANCEL;
			break;
		default:
			CAM_WARN(CAM_SYNX,
				"Synx signal status %d is neither SUCCESS nor CANCEL, custom code?",
				status);
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_ERROR;
			break;
		}
		synx_obj_row->state = CAM_SYNX_OBJ_STATE_SIGNALED;
		synx_obj_row->sync_cb(synx_obj_row->sync_obj, &signal_sync_obj);
	}

}

/*
 * Synx APIs need to be invoked in non atomic context,
 * all these utils invoke synx driver
 */
static inline int __cam_synx_signal_util(
	struct synx_session *session_hdl,
	uint32_t synx_hdl, uint32_t signal_status)
{
	return synx_signal((session_hdl ? session_hdl :
		g_cam_synx_obj_dev->session_handle), synx_hdl, signal_status);
}

static inline int __cam_synx_create_hdl_util(
	struct synx_session *session_hdl, struct synx_create_params *params)
{
	return synx_create((session_hdl ? session_hdl :
		g_cam_synx_obj_dev->session_handle), params);
}

static inline int __cam_synx_deregister_cb_util(
	uint32_t synx_hdl, void *data)
{
	struct synx_callback_params cb_params;

	cb_params.userdata = data;
	cb_params.cancel_cb_func = NULL;
	cb_params.h_synx = synx_hdl;
	cb_params.cb_func = __cam_synx_obj_signal_cb;

	return synx_cancel_async_wait(g_cam_synx_obj_dev->session_handle, &cb_params);
}

static void *__cam_synx_get_fence(
	struct synx_session *session_hdl, int32_t synx_hdl)
{
	return synx_get_fence((session_hdl ? session_hdl :
		g_cam_synx_obj_dev->session_handle), synx_hdl);
}

static int __cam_synx_obj_map_sync_status_util(uint32_t sync_status,
	uint32_t *out_synx_status)
{
	if (!out_synx_status)
		return -EINVAL;

	switch (sync_status) {
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		*out_synx_status = SYNX_STATE_SIGNALED_SUCCESS;
		break;
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
	default:
		*out_synx_status = SYNX_STATE_SIGNALED_CANCEL;
		break;
	}

	return 0;
}

static int __cam_synx_obj_release(int32_t row_idx)
{
	struct cam_synx_obj_row *row = NULL;

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];

	if (row->state == CAM_SYNX_OBJ_STATE_ACTIVE && (row->session_hdl == NULL)) {
		CAM_WARN(CAM_SYNX,
			"Unsignaled synx obj being released name: %s synx_obj:%d",
			row->name, row->synx_obj);
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
		synx_signal(g_cam_synx_obj_dev->session_handle,
			row->synx_obj,
			SYNX_STATE_SIGNALED_CANCEL);
	} else {
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	}
	CAM_DBG(CAM_SYNX,
		"Releasing synx_obj: %d[%s] row_idx: %u",
		row->synx_obj, row->name, row_idx);
	synx_release((row->session_hdl ? row->session_hdl : g_cam_synx_obj_dev->session_handle),
		row->synx_obj);
	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	/* deinit row */
	memset(row, 0, sizeof(struct cam_synx_obj_row));
	clear_bit(row_idx, g_cam_synx_obj_dev->bitmap);
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	return 0;
}

static int __cam_synx_obj_find_free_idx(uint32_t *idx)
{
	int rc = 0;
	bool bit;

	do {
		*idx = find_first_zero_bit(g_cam_synx_obj_dev->bitmap, CAM_SYNX_MAX_OBJS);
		if (*idx >= CAM_SYNX_MAX_OBJS) {
			CAM_ERR(CAM_SYNC,
				"Error: Unable to find free synx idx = %d reached max!",
				*idx);
			return -ENOMEM;
		}
		CAM_DBG(CAM_SYNC, "Index location available at idx: %ld", *idx);
		bit = test_and_set_bit(*idx, g_cam_synx_obj_dev->bitmap);
	} while (bit);

	return rc;
}

static void __cam_synx_obj_init_row(uint32_t idx, const char *name,
	uint32_t synx_obj, struct synx_session *session_hdl)
{
	struct cam_synx_obj_row *row;

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
	row = &g_cam_synx_obj_dev->rows[idx];
	memset(row, 0, sizeof(*row));
	row->synx_obj = synx_obj;
	row->state = CAM_SYNX_OBJ_STATE_ACTIVE;
	row->session_hdl = session_hdl;
	strscpy(row->name, name, CAM_SYNX_OBJ_NAME_LEN);
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
}

static int __cam_synx_obj_release_row(int32_t row_idx)
{
	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx row idx: %d is invalid",
			row_idx);
		return -EINVAL;
	}

	return __cam_synx_obj_release(row_idx);
}

int cam_synx_obj_find_obj_in_table(uint32_t synx_obj, int32_t *idx)
{
	int i, rc = -EINVAL;
	struct cam_synx_obj_row *row = NULL;

	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++) {
		spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
		row = &g_cam_synx_obj_dev->rows[i];
		if ((row->state != CAM_SYNX_OBJ_STATE_INVALID) &&
			(row->synx_obj == synx_obj)) {
			*idx = i;
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
			rc = 0;
			break;
		}
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
	}

	return rc;
}

static int __cam_synx_obj_release_obj(uint32_t synx_obj)
{
	int32_t idx;

	if (cam_synx_obj_find_obj_in_table(synx_obj, &idx)) {
		CAM_ERR(CAM_SYNX, "Failed to find synx obj: %d", synx_obj);
		return -EINVAL;
	}

	return __cam_synx_obj_release(idx);
}

static int __cam_synx_obj_import(const char *name,
	struct synx_import_params *params, int32_t *row_idx)
{
	int rc = -1;
	uint32_t idx;

	if (__cam_synx_obj_find_free_idx(&idx))
		goto end;

	rc = synx_import(g_cam_synx_obj_dev->session_handle, params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Synx import failed for fence : %p",
			params->indv.fence);
		goto free_idx;
	}

	*row_idx = idx;
	__cam_synx_obj_init_row(idx, name, *params->indv.new_h_synx, NULL);

	CAM_DBG(CAM_SYNX, "Imported synx obj handle: %d[%s] row_idx: %u",
		*params->indv.new_h_synx, name, idx);

	return rc;

free_idx:
	clear_bit(idx, g_cam_synx_obj_dev->bitmap);
end:
	return rc;
}

static int __cam_synx_map_generic_flags_to_create(uint32_t generic_flags,
	struct synx_create_params *params)
{
	if (!params) {
		CAM_ERR(CAM_SYNX, "Create parameters missing");
		return -EINVAL;
	}

	if (CAM_GENERIC_FENCE_FLAG_IS_GLOBAL_SYNX_OBJ & generic_flags)
		params->flags |= SYNX_CREATE_GLOBAL_FENCE;
	else
		params->flags |= SYNX_CREATE_LOCAL_FENCE;

	return 0;
}

static int __cam_synx_map_generic_flags_to_import(uint32_t generic_flags,
	struct synx_import_indv_params *params)
{
	if (!params) {
		CAM_ERR(CAM_SYNX, "Import parameters missing");
		return -EINVAL;
	}

	if (CAM_GENERIC_FENCE_FLAG_IS_GLOBAL_SYNX_OBJ & generic_flags)
		params->flags |= SYNX_IMPORT_GLOBAL_FENCE;
	else
		params->flags |= SYNX_IMPORT_LOCAL_FENCE;

	return 0;
}

int cam_synx_obj_create(const char *name, struct synx_session *session_hdl,
	uint32_t flags, uint32_t *synx_obj, int32_t *row_idx)
{
	int rc = -1;
	uint32_t idx;
	struct synx_create_params params;

	if (__cam_synx_obj_find_free_idx(&idx))
		goto end;

	params.fence = NULL;
	params.name = name;
	params.flags = 0;
	params.h_synx = synx_obj;

	rc = __cam_synx_map_generic_flags_to_create(flags, &params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to generate create flags");
		goto free_idx;
	}

	/*
	 * Create Global Always - remove after userspace optimizes and
	 * determines when global Vs local is needed
	 */
	params.flags |= SYNX_CREATE_GLOBAL_FENCE;

	rc = __cam_synx_create_hdl_util(session_hdl, &params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to create synx obj");
		goto free_idx;
	}

	*row_idx = idx;
	__cam_synx_obj_init_row(idx, name, *synx_obj, session_hdl);

	CAM_DBG(CAM_SYNX, "Created synx obj handle: %d[%s] row_idx: %u",
		*synx_obj, name, idx);

	return rc;

free_idx:
	clear_bit(idx, g_cam_synx_obj_dev->bitmap);
end:
	return rc;
}

int cam_synx_update_hw_fence_queue(void *session_hdl, int32_t synx_hdl)
{
	return __cam_synx_signal_util(session_hdl, synx_hdl, SYNX_STATE_SIGNALED_SUCCESS);
}

void *cam_synx_obj_get_native_fence(struct synx_session *session_hdl,
	int32_t synx_hdl)
{

	return __cam_synx_get_fence(session_hdl, synx_hdl);
}

int cam_synx_obj_import_dma_fence(const char *name, uint32_t flags, void *fence,
	uint32_t *synx_obj, int32_t *row_idx)
{
	struct synx_import_params params;

	if (!fence) {
		CAM_ERR(CAM_SYNX,
			"Importing DMA fence failed - fence pointer is NULL");
		return -EINVAL;
	}

	params.indv.flags = 0;
	params.indv.fence = fence;
	params.indv.new_h_synx = synx_obj;
	params.type = SYNX_IMPORT_INDV_PARAMS;
	params.indv.flags |= SYNX_IMPORT_DMA_FENCE;

	if (__cam_synx_map_generic_flags_to_import(flags, &params.indv)) {
		CAM_ERR(CAM_SYNX,
			"Importing DMA fence failed - invalid synx import flags");
		return -EINVAL;
	}

	return __cam_synx_obj_import(name, &params, row_idx);
}

int cam_synx_obj_internal_signal(int32_t row_idx,
	struct cam_synx_obj_signal *signal_synx_obj)
{
	int rc = 0;
	uint32_t signal_status;
	struct cam_synx_obj_row *row = NULL;

	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx obj row idx: %d is invalid",
			row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];

	/* Ensures sync obj cb is not invoked */
	row->sync_signal_synx = true;

	if (row->state != CAM_SYNX_OBJ_STATE_ACTIVE) {
		CAM_ERR(CAM_SYNX, "synx obj: %u not in right state: %d to signal",
			signal_synx_obj->synx_obj, row->state);
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
		return -EINVAL;
	}

	if (row->synx_obj != signal_synx_obj->synx_obj) {
		CAM_WARN(CAM_SYNX,
			"Trying to signal synx obj: %u in row: %u having a different synx obj: %u",
			signal_synx_obj->synx_obj, row_idx, row->synx_obj);
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
		return 0;
	}

	rc = __cam_synx_obj_map_sync_status_util(signal_synx_obj->status,
		&signal_status);
	if (rc) {
		CAM_WARN(CAM_SYNX,
			"Signaling undefined status: %d for synx obj: %d",
			signal_synx_obj->status,
			signal_synx_obj->synx_obj);
	}
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);

	if (row->cb_registered_for_sync) {
		rc = __cam_synx_deregister_cb_util(signal_synx_obj->synx_obj, row);
		if (rc) {
			CAM_ERR(CAM_SYNX, "Failed to deregister cb for synx: %u rc: %d",
				signal_synx_obj->synx_obj, rc);
			goto end;
		}
	}

	rc = __cam_synx_signal_util(NULL, signal_synx_obj->synx_obj, signal_status);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to signal synx hdl: %u with status: %u rc: %d",
			signal_synx_obj->synx_obj, signal_status, rc);
		goto end;
	}

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row->state = CAM_SYNX_OBJ_STATE_SIGNALED;
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);

	CAM_DBG(CAM_SYNX, "synx obj: %d signaled with status: %d rc: %d",
		signal_synx_obj->synx_obj, signal_status, rc);

end:
	return rc;
}

int cam_synx_obj_release(struct cam_synx_obj_release_params *release_params)
{
	if (release_params->use_row_idx)
		return __cam_synx_obj_release_row(release_params->u.synx_row_idx);
	else
		return __cam_synx_obj_release_obj(release_params->u.synx_obj);
}

int cam_synx_obj_signal_obj(struct cam_synx_obj_signal *signal_synx_obj)
{
	int rc = 0;
	uint32_t idx, signal_status = 0;
	struct cam_synx_obj_row *row = NULL;

	rc = cam_synx_obj_find_obj_in_table(
		signal_synx_obj->synx_obj, &idx);

	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to find synx obj: %d",
			signal_synx_obj->synx_obj);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
	row = &g_cam_synx_obj_dev->rows[idx];
	if (row->state == CAM_SYNX_OBJ_STATE_SIGNALED) {
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
		CAM_WARN(CAM_SYNX, "synx obj: %d already in signaled state",
			signal_synx_obj->synx_obj);
		return 0;
	}

	rc = __cam_synx_obj_map_sync_status_util(signal_synx_obj->status,
		&signal_status);
	if (rc) {
		CAM_WARN(CAM_SYNX,
			"Signaling undefined sync status: %d for synx obj: %d",
			signal_synx_obj->status,
			signal_synx_obj->synx_obj);
	}
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);

	rc = synx_signal(g_cam_synx_obj_dev->session_handle,
		signal_synx_obj->synx_obj, signal_status);
	if (rc)
		CAM_WARN(CAM_SYNX, "synx obj: %d already signaled rc: %d",
			row->synx_obj, rc);

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
	row->state = CAM_SYNX_OBJ_STATE_SIGNALED;
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
	CAM_DBG(CAM_SYNX, "synx obj: %d signaled with status: %d rc: %d",
		signal_synx_obj->synx_obj, signal_status, rc);

	return rc;
}

int cam_synx_obj_register_cb(int32_t *sync_obj, int32_t row_idx,
	cam_sync_callback_for_synx_obj sync_cb)
{
	int rc = 0;
	struct cam_synx_obj_row *row = NULL;
	struct synx_callback_params cb_params;

	if (!sync_obj || !sync_cb) {
		CAM_ERR(CAM_SYNX, "Invalid args sync_obj: %p sync_cb: %p",
			sync_obj, sync_cb);
		return -EINVAL;
	}

	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx obj idx: %d is invalid",
			row_idx);
		return -EINVAL;
	}
	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];

	if (row->state != CAM_SYNX_OBJ_STATE_ACTIVE) {
		CAM_ERR(CAM_SYNX,
			"synx obj at idx: %d handle: %d is not active, current state: %d",
			row_idx, row->synx_obj, row->state);
		rc = -EINVAL;
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
		goto end;
	}

	/**
	 * If the cb is already registered, return
	 */
	if (row->cb_registered_for_sync) {
		CAM_WARN(CAM_SYNX,
			"synx obj at idx: %d handle: %d has already registered a cb for sync: %d",
			row_idx, row->synx_obj, row->sync_obj);
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
		goto end;
	}

	row->sync_cb = sync_cb;
	row->sync_obj = *sync_obj;
	row->cb_registered_for_sync = true;

	cb_params.userdata = row;
	cb_params.cancel_cb_func = NULL;
	cb_params.h_synx = row->synx_obj;
	cb_params.timeout_ms = SYNX_NO_TIMEOUT;
	cb_params.cb_func = __cam_synx_obj_signal_cb;

	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	rc = synx_async_wait(g_cam_synx_obj_dev->session_handle, &cb_params);
	if (rc) {
		CAM_ERR(CAM_SYNX,
			"Failed to register cb for synx obj: %d rc: %d",
			row->synx_obj, rc);
		goto end;
	}

	CAM_DBG(CAM_SYNX,
		"CB successfully registered for synx obj: %d for sync_obj: %d",
		row->synx_obj, *sync_obj);

end:
	return rc;
}
#if IS_REACHABLE(CONFIG_CAM_ENABLE_SOCCP)
enum synx_client_id cam_synx_map_camera_client_id_for_synx(
	enum cam_sync_fencing_client_cores cam_client_id,
	uint32_t signal_id)
{
	switch (cam_client_id) {
	case CAM_SYNC_SYNX_CLIENT_ICP_0:
		return SYNX_CLIENT_ICP_CTX0;
	case CAM_SYNC_SYNX_CLIENT_ICP_1:
		return SYNX_CLIENT_NATIVE;
	case CAM_SYNC_HW_FENCE_CLIENT_IFE0_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE0_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE1_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE1_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE2_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE2_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE3_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE3_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE4_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE4_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE5_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE5_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE6_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE6_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE7_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE7_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE8_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE8_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE9_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE9_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE10_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE10_CTX0 + signal_id);
	case CAM_SYNC_HW_FENCE_CLIENT_IFE11_CTX0:
		return (SYNX_CLIENT_HW_FENCE_IFE11_CTX0 + signal_id);
	default:
		return SYNX_CLIENT_MAX;
	}
}

struct synx_session *cam_synx_initialize_hw_fence_session(
	struct cam_sync_hwfence_session_initialize_params *init_params,
	void **sw_tx_wm_vaddr)
{
	enum synx_client_id synx_client_id;
	struct synx_queue_desc queue_desc;
	struct synx_initialization_params params;
	struct synx_session *session_handle = NULL;

	if (!init_params || !sw_tx_wm_vaddr) {
		CAM_ERR(CAM_SYNX, "Invalid arguments");
		return ERR_PTR(-EINVAL);
	}

	if (unlikely(init_params->signal_id >= HW_FENCE_MAX_IPCC_SIGNALS_PER_CLIENT)) {
		CAM_ERR(CAM_SYNX, "Invalid signal_id: %u max_supported: %u",
			init_params->signal_id, HW_FENCE_MAX_IPCC_SIGNALS_PER_CLIENT);
		return ERR_PTR(-EINVAL);
	}

	synx_client_id = cam_synx_map_camera_client_id_for_synx(init_params->client_core,
		init_params->signal_id);

	if (unlikely(synx_client_id >= SYNX_CLIENT_MAX)) {
		CAM_ERR(CAM_SYNX, "Invalid client_core: %u signal_id: %u",
			init_params->client_core, init_params->signal_id);
		return ERR_PTR(-EINVAL);
	}

	params.name = init_params->name;
	params.ptr = &queue_desc;
	params.flags = SYNX_INIT_MAX;
	params.id = synx_client_id;

	session_handle = synx_initialize(&params);
	if (IS_ERR_OR_NULL(session_handle)) {
		CAM_ERR(CAM_SYNX,
			"Failed to initialize session for client: %u %u [%s]",
			init_params->client_core, synx_client_id, init_params->name);
		return NULL;
	}

	if (init_params->fencing_protocol) {
		struct msm_hw_fence_hfi_queue_table_header *queue_table;
		uint32_t sw_tx_wm_offset;

		queue_table = (struct msm_hw_fence_hfi_queue_table_header *)queue_desc.vaddr;
		/* Gets queue start for camera fencing clients */
		init_params->offset = queue_table->qhdr0_offset +
			offsetof(struct synx_hw_fence_hfi_queue_header, write_index);
		/* Gets the SW tx queue watermark */
		sw_tx_wm_offset = queue_table->qhdr0_offset +
			offsetof(struct synx_hw_fence_hfi_queue_header, tx_wm);

		init_params->fenceq_dev_addr = queue_desc.dev_addr;
		*sw_tx_wm_vaddr = queue_desc.vaddr + sw_tx_wm_offset;

		/* Gets queue_desc.size */
		init_params->len = queue_desc.size;
	}

	CAM_DBG(CAM_SYNX,
		"Client [id: %u signal_id: %u name: %s] session: %p initialized",
		init_params->client_core, init_params->signal_id,
		init_params->name, session_handle);

	return session_handle;
}

int cam_synx_uninitialize_hw_fence_session(
	struct synx_session *session_handle)
{
	return synx_uninitialize(session_handle);
}

int cam_synx_core_recovery(
	enum cam_sync_fencing_client_cores cam_core_id)
{
	int rc;
	enum synx_client_id client_id = SYNX_CLIENT_MAX;

	switch (cam_core_id) {
	case CAM_SYNC_SYNX_CLIENT_ICP_0:
		client_id = SYNX_CLIENT_ICP_CTX0;
		break;
	default:
		rc = -EINVAL;
		goto err;
	}

	rc = synx_recover(client_id);
	if (rc)
		goto err;

	CAM_DBG(CAM_SYNX, "Synx recovery for synx_client: %d[%d] success",
		client_id, cam_core_id);

	return rc;

err:
	CAM_ERR(CAM_SYNX, "Failed to recover for synx_client: %d rc: %d",
			client_id, rc);
	return rc;
}
#else
enum synx_client_id cam_synx_map_camera_client_id_for_synx(
	enum cam_sync_fencing_client_cores cam_client_id,
	uint32_t signal_id)
{
	return -EOPNOTSUPP;
}

int cam_synx_core_recovery(
	enum cam_sync_fencing_client_cores cam_core_id)
{
	return -EOPNOTSUPP;
}

int cam_synx_uninitialize_hw_fence_session(
	struct synx_session *session_handle)
{
	return -EOPNOTSUPP;
}

struct synx_session *cam_synx_initialize_hw_fence_session(
	struct cam_sync_hwfence_session_initialize_params *init_params,
	void **sw_tx_wm_vaddr)
{
	return ERR_PTR(-EOPNOTSUPP);
}
#endif

int __cam_synx_init_session(void)
{
	struct synx_queue_desc queue_desc;
	struct synx_initialization_params params;

	params.name = cam_synx_session_name;
	params.ptr = &queue_desc;
	params.flags = SYNX_INIT_MAX;
	params.id = SYNX_CLIENT_NATIVE;
	g_cam_synx_obj_dev->session_handle = synx_initialize(&params);

	if (!g_cam_synx_obj_dev->session_handle) {
		CAM_ERR(CAM_SYNX, "Synx session initialization failed");
		return -EINVAL;
	}

	CAM_DBG(CAM_SYNX, "Synx session initialized: %p",
		g_cam_synx_obj_dev->session_handle);

	return 0;
}

void cam_synx_obj_close(void)
{
	int i, rc = 0;
	struct cam_synx_obj_row *row = NULL;
	struct synx_callback_params cb_params;

	mutex_lock(&g_cam_synx_obj_dev->dev_lock);
	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++) {
		spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);

		row = &g_cam_synx_obj_dev->rows[i];
		if (row->state == CAM_SYNX_OBJ_STATE_INVALID) {
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
			continue;
		}
		CAM_DBG(CAM_SYNX, "Releasing synx_obj: %d[%s]",
			row->synx_obj, row->name);

		/* If registered for cb, remove cb */
		if (row->cb_registered_for_sync) {
			cb_params.userdata = row;
			cb_params.cancel_cb_func = NULL;
			cb_params.h_synx = row->synx_obj;
			cb_params.cb_func = __cam_synx_obj_signal_cb;
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);

			rc = synx_cancel_async_wait(
				g_cam_synx_obj_dev->session_handle,
				&cb_params);
			if (rc) {
				CAM_WARN(CAM_SYNX,
				"Registered callback could not be canceled for synx obj : %d",
				cb_params.h_synx);
			}
		} else {
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
		}
		/* Signal and release the synx obj */
		if (row->state != CAM_SYNX_OBJ_STATE_SIGNALED)
			__cam_synx_signal_util(NULL, row->synx_obj, SYNX_STATE_SIGNALED_CANCEL);
		synx_release((row->session_hdl ? row->session_hdl :
			g_cam_synx_obj_dev->session_handle), row->synx_obj);

		spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
		memset(row, 0, sizeof(struct cam_synx_obj_row));
		clear_bit(i, g_cam_synx_obj_dev->bitmap);
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
	}

	mutex_unlock(&g_cam_synx_obj_dev->dev_lock);
	CAM_DBG(CAM_SYNX, "Close on Camera SYNX driver");
}

int cam_synx_obj_driver_init(void)
{
	int i;

	g_cam_synx_obj_dev = kzalloc(sizeof(struct cam_synx_obj_device), GFP_KERNEL);
	if (!g_cam_synx_obj_dev)
		return -ENOMEM;

	if (__cam_synx_init_session())
		goto deinit_driver;

	mutex_init(&g_cam_synx_obj_dev->dev_lock);
	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++)
		spin_lock_init(&g_cam_synx_obj_dev->row_spinlocks[i]);

	memset(&g_cam_synx_obj_dev->rows, 0, sizeof(g_cam_synx_obj_dev->rows));
	memset(&g_cam_synx_obj_dev->bitmap, 0, sizeof(g_cam_synx_obj_dev->bitmap));
	bitmap_zero(g_cam_synx_obj_dev->bitmap, CAM_SYNX_MAX_OBJS);

	CAM_DBG(CAM_SYNX, "Camera synx obj driver initialized");
	return 0;

deinit_driver:
	CAM_ERR(CAM_SYNX, "Camera synx obj driver initialization failed");
	kfree(g_cam_synx_obj_dev);
	g_cam_synx_obj_dev = NULL;
	return -EINVAL;
}

void cam_synx_obj_driver_deinit(void)
{
	int rc;

	if (g_cam_synx_obj_dev->session_handle) {
		rc = synx_uninitialize(g_cam_synx_obj_dev->session_handle);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Synx failed to uninitialize session: %p, rc: %d",
				g_cam_synx_obj_dev->session_handle, rc);
		}
	}

	kfree(g_cam_synx_obj_dev);
	g_cam_synx_obj_dev = NULL;
	CAM_DBG(CAM_SYNX, "Camera synx obj driver deinitialized");
}
