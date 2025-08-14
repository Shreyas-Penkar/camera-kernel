// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "cam_sync_util.h"
#include "cam_req_mgr_worker_wrapper.h"
#include "cam_common_util.h"

extern struct sync_uid_info sync_uid_access;

int cam_sync_get_ext_fence_payload(
	struct sync_ext_fence_info **ext_fence_payload)
{
	int rc = 0;
	struct sync_ext_fence_info *tmp_payload = NULL;

	*ext_fence_payload = NULL;

	spin_lock_bh(&sync_dev->payload_lock);
	if (list_empty(&sync_dev->free_ext_fence_list)) {
		CAM_ERR(CAM_SYNC, "No free payload for associated fences");
		rc = -EINVAL;
		goto unlock_end;
	}

	tmp_payload = list_first_entry(&sync_dev->free_ext_fence_list,
		struct sync_ext_fence_info, list);
	list_del_init(&tmp_payload->list);
	*ext_fence_payload = tmp_payload;
	memset(&tmp_payload->synx_obj_info, 0, sizeof(struct sync_synx_obj_info));
	memset(&tmp_payload->dma_fence_info, 0, sizeof(struct sync_dma_fence_info));

unlock_end:
	spin_unlock_bh(&sync_dev->payload_lock);
	return rc;
}

int cam_sync_put_ext_fence_payload(
	struct sync_ext_fence_info **ext_fence_payload)
{
	struct sync_ext_fence_info *tmp_payload = *ext_fence_payload;

	spin_lock_bh(&sync_dev->payload_lock);
	list_del_init(&tmp_payload->list);
	list_add_tail(&tmp_payload->list, &sync_dev->free_ext_fence_list);
	spin_unlock_bh(&sync_dev->payload_lock);

	return 0;
}

int cam_sync_util_send_exit_poll_event(void *fh)
{
	struct v4l2_event event;

	memset(&event, 0, sizeof(struct v4l2_event));

	switch (sync_dev->version) {
	case CAM_SYNC_V4L_EVENT_V2:
		event.type = CAM_SYNC_V4L_EVENT_V2;
		break;
	case CAM_SYNC_V4L_EVENT_V3:
		event.type = CAM_SYNC_V4L_EVENT_V3;
		break;
	case CAM_SYNC_V4L_EVENT_V4:
		event.type = CAM_SYNC_V4L_EVENT_V4;
		break;
	case CAM_SYNC_V4L_EVENT_V5:
		event.type = CAM_SYNC_V4L_EVENT_V5;
		break;
	default:
		event.type = CAM_SYNC_V4L_EVENT;
		break;
	}

	event.id = CAM_SYNC_V4L_EVENT_ID_EXIT;

	v4l2_event_queue_fh(fh, &event);
	return 0;
}

int cam_sync_init_row(struct sync_table_row *table,
	uint32_t idx, const char *name, uint32_t type, uint32_t sync_manager_idx)
{
	struct sync_table_row *row = table + idx;

	if (!table || idx <= 0 || idx >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	memset(row, 0, sizeof(*row));

	strlcpy(row->name, name, SYNC_DEBUG_NAME_LEN);
	INIT_LIST_HEAD(&row->parents_list);
	INIT_LIST_HEAD(&row->children_list);
	row->type = type;
	row->sync_id = idx;
	if (row->state != CAM_SYNC_STATE_INVALID) {
		reinit_completion(&row->signaled);
	} else {
		init_completion(&row->signaled);
		row->state = CAM_SYNC_STATE_ACTIVE;
	}
	row->remaining = 0;
	row->uid = 0;
	row->sync_manager_idx = sync_manager_idx;
	row->hw_fence_client_idx = -1;
	row->signaling_en = true;
	atomic_set(&row->ref_cnt, 0);
	INIT_LIST_HEAD(&row->callback_list);
	INIT_LIST_HEAD(&row->user_payload_list);
	INIT_LIST_HEAD(&row->ext_fences);
	CAM_DBG(CAM_SYNC,
		"row name:%s sync_id:%i [idx:%u] row_state:%u ",
		row->name, row->sync_id, idx, row->state);

	return 0;
}

int cam_sync_init_group_object(struct sync_table_row *table,
	uint32_t sync_var,
	uint32_t *sync_objs,
	uint32_t num_objs,
	uint32_t type)
{
	int i, rc = 0;
	struct sync_child_info *child_info;
	struct sync_parent_info *parent_info;
	uint32_t idx, sync_obj, sync_uid, sync_manager_idx;
	struct sync_table_row *row;
	struct sync_table_row *child_row = NULL;

	idx = sync_var & sync_uid_access.fenceIdMask;
	row = table + idx;
	sync_manager_idx = row->sync_manager_idx;
	cam_sync_init_row(table, idx, "merged_fence", type, sync_manager_idx);

	/*
	 * While traversing for children, parent's row list is updated with
	 * child info and each child's row is updated with parent info.
	 * If any child state is ERROR or SUCCESS, it will not be added to list.
	 */
	for (i = 0; i < num_objs; i++) {
		sync_obj = (uint32_t)(sync_objs[i]) & sync_uid_access.fenceIdMask;
		sync_uid = (uint32_t)(sync_objs[i]) >> sync_uid_access.uidShift;
		child_row = table + sync_obj;
		spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);

		if (SYNC_UID_OLD == cam_sync_check_uid_valid(sync_objs[i])) {
			CAM_DBG(CAM_SYNC, "sync_var: %d is old, don't add to parent list",
				sync_objs[i]);
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			continue;
		}
		/* validate child */
		if ((!list_empty(&child_row->children_list)) ||
			(child_row->state == CAM_SYNC_STATE_INVALID)) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			CAM_ERR(CAM_SYNC,
				"Invalid child fence:%i state:%u type:%u",
				child_row->sync_id, child_row->state,
				child_row->type);
			rc = -EINVAL;
			goto clean_children_info;
		}

		/* check for child's state */
		if ((child_row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
			(child_row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) {
			row->state = child_row->state;
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			continue;
		}
		if (child_row->state != CAM_SYNC_STATE_ACTIVE) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			continue;
		}

		row->remaining++;

		/* Add child info */
		child_info = kzalloc(sizeof(*child_info), GFP_ATOMIC);
		if (!child_info) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			rc = -ENOMEM;
			goto clean_children_info;
		}
		child_info->sync_id = sync_obj;
		list_add_tail(&child_info->list, &row->children_list);

		/* Add parent info */
		parent_info = kzalloc(sizeof(*parent_info), GFP_ATOMIC);
		if (!parent_info) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			rc = -ENOMEM;
			goto clean_children_info;
		}
		parent_info->sync_id = idx;
		list_add_tail(&parent_info->list, &child_row->parents_list);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	}

	if (!row->remaining) {
		if ((row->state != CAM_SYNC_STATE_SIGNALED_ERROR) &&
			(row->state != CAM_SYNC_STATE_SIGNALED_CANCEL))
			row->state = CAM_SYNC_STATE_SIGNALED_SUCCESS;
		complete_all(&row->signaled);
	}

	return 0;

clean_children_info:
	row->state = CAM_SYNC_STATE_INVALID;
	for (i = i-1; i >= 0; i--) {
		sync_obj = sync_objs[i] & sync_uid_access.fenceIdMask;
		spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
		child_row = table + sync_obj;
		cam_sync_util_cleanup_parents_list(child_row,
			SYNC_LIST_CLEAN_ONE, idx);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	}

	cam_sync_util_cleanup_children_list(row, SYNC_LIST_CLEAN_ALL, 0);
	return rc;
}

static int cam_sync_util_release_synx_dma(int32_t sync_obj)
{
	int rc = 0;
	struct sync_table_row *row;
	struct cam_dma_fence_release_params release_params;
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
	struct cam_synx_obj_release_params synx_release_params;
#endif
	struct sync_ext_fence_info *ext_fence_info, *tmp;

	row = sync_dev->sync_table + sync_obj;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	/* Decrement ref cnt for imported dma fence */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask) ||
		test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask)) {

		list_for_each_entry_safe(ext_fence_info, tmp, &row->ext_fences, list) {
			if (ext_fence_info->dma_fence_info.is_valid) {
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx =
					ext_fence_info->dma_fence_info.dma_fence_row_idx;
				rc = cam_dma_fence_release(&release_params);
				if (rc)
					CAM_ERR(CAM_SYNC,
						"Failed to destroy dma fence fd: %d rc: %d associated with sync: %d",
						ext_fence_info->dma_fence_info.dma_fence_fd,
						rc, sync_obj);
			}
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX_V2)
			if (ext_fence_info->synx_obj_info.is_valid) {
				synx_release_params.use_row_idx = true;
				synx_release_params.u.synx_row_idx =
					ext_fence_info->synx_obj_info.synx_obj_row_idx;
				/* Release & obtain the row lock after synx release */
				spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
				rc = cam_synx_obj_release(&synx_release_params);
				spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
				if (rc)
					CAM_ERR(CAM_SYNC,
						"Failed to destroy synx_obj: %d rc: %d associated with sync: %d",
						ext_fence_info->synx_obj_info.synx_obj, rc,
						sync_obj);
			}
#endif
			cam_sync_put_ext_fence_payload(&ext_fence_info);
		}
	}

	memset(row, 0, sizeof(*row));
	clear_bit(sync_obj, sync_dev->bitmap);
	INIT_LIST_HEAD(&row->callback_list);
	INIT_LIST_HEAD(&row->parents_list);
	INIT_LIST_HEAD(&row->children_list);
	INIT_LIST_HEAD(&row->user_payload_list);
	INIT_LIST_HEAD(&row->ext_fences);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	return rc;
}

int cam_sync_deinit_object(struct sync_table_row *table, uint32_t sync_var)
{
	struct sync_table_row      *row;
	struct sync_child_info     *child_info, *temp_child;
	struct sync_callback_info  *sync_cb, *temp_cb;
	struct sync_parent_info    *parent_info, *temp_parent;
	struct sync_user_payload   *upayload_info, *temp_upayload;
	struct sync_table_row      *child_row = NULL, *parent_row = NULL;
	struct list_head            temp_child_list, temp_parent_list;
	uint32_t idx, sync_id;
	uint16_t sync_uid;
	int rc;

	idx = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	row = table + idx;

	if (!table || idx <= 0 || idx >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	CAM_DBG(CAM_SYNC,
		"row name:%s sync_obj:%i [idx:%u] ] uid:%d",
		row->name, idx, row->sync_manager_idx, sync_uid);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	if (row->state == CAM_SYNC_STATE_INVALID) {
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj: idx = %d name = %s",
			idx,
			row->name);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_ACTIVE)
		CAM_DBG(CAM_SYNC,
			"Destroying an active sync object name:%s id:%i",
			row->name, row->sync_id);

	row->state = CAM_SYNC_STATE_INVALID;

	if (test_bit(CAM_GENERIC_FENCE_TYPE_HW_FENCE, &row->ext_fence_mask)) {
		list_for_each_entry_safe(child_info, temp_child, &row->children_list, list) {
			rc = cam_sync_util_release_synx_dma(child_info->sync_id);
			if (rc)
				CAM_ERR(CAM_SYNC,
					"Failed to release synx/dma associated with child_sync_obj: %d parent_sync_obj: %d rc: %d",
					child_info->sync_id, idx, rc);
		}
	}

	list_for_each_entry_safe(parent_info, temp_parent, &row->parents_list, list) {
		rc = cam_sync_util_release_synx_dma(parent_info->sync_id);
		if (rc)
			CAM_ERR(CAM_SYNC,
				"Failed to release synx/dma associated with parent_sync_obj: %d child_sync_obj: %d rc: %d",
				parent_info->sync_id, idx, rc);
	}

	/* Object's child and parent objects will be added into this list */
	INIT_LIST_HEAD(&temp_child_list);
	INIT_LIST_HEAD(&temp_parent_list);

	list_for_each_entry_safe(child_info, temp_child, &row->children_list,
		list) {
		if (child_info->sync_id <= 0)
			continue;

		list_del_init(&child_info->list);
		list_add_tail(&child_info->list, &temp_child_list);
	}

	list_for_each_entry_safe(parent_info, temp_parent, &row->parents_list,
		list) {
		if (parent_info->sync_id <= 0)
			continue;

		list_del_init(&parent_info->list);
		list_add_tail(&parent_info->list, &temp_parent_list);
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	/* Cleanup the child to parent link from child list */
	while (!list_empty(&temp_child_list)) {
		child_info = list_first_entry(&temp_child_list,
			struct sync_child_info, list);
		sync_id = (uint32_t)(child_info->sync_id) & sync_uid_access.fenceIdMask;
		child_row = table + sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[sync_id]);

		if (child_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&child_info->list);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				sync_id]);
			kfree(child_info);
			continue;
		}

		if (child_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active child sync obj = %s[%d]",
				child_row->name,
				child_info->sync_id);

		cam_sync_util_cleanup_parents_list(child_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&child_info->list);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_id]);
		kfree(child_info);
	}

	/* Cleanup the parent to child link */
	while (!list_empty(&temp_parent_list)) {
		parent_info = list_first_entry(&temp_parent_list,
			struct sync_parent_info, list);
		sync_id = (uint32_t)(parent_info->sync_id) & sync_uid_access.fenceIdMask;
		parent_row = table + sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[sync_id]);

		if (parent_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&parent_info->list);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				sync_id]);
			kfree(parent_info);
			continue;
		}

		if (parent_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active parent sync obj = %s[%d]",
				parent_row->name,
				parent_info->sync_id);

		cam_sync_util_cleanup_children_list(parent_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&parent_info->list);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_id]);
		kfree(parent_info);
	}

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	list_for_each_entry_safe(upayload_info, temp_upayload,
			&row->user_payload_list, list) {
		list_del_init(&upayload_info->list);
		kfree(upayload_info);
	}

	list_for_each_entry_safe(sync_cb, temp_cb,
			&row->callback_list, list) {
		list_del_init(&sync_cb->list);
		kfree(sync_cb);
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
	rc = cam_sync_util_release_synx_dma(idx);
	if (rc)
		CAM_ERR(CAM_SYNC, "Failed to release synx/dma associated with sync_obj: %d rc: %d",
			idx, rc);

	return rc;
}

int cam_sync_reinit_object(struct sync_table_row *table, uint32_t sync_var)
{
	struct sync_table_row      *row;
	struct sync_child_info     *child_info, *temp_child;
	struct sync_callback_info  *sync_cb, *temp_cb;
	struct sync_parent_info    *parent_info, *temp_parent;
	struct sync_user_payload   *upayload_info, *temp_upayload;
	struct sync_table_row      *child_row = NULL, *parent_row = NULL;
	struct list_head            temp_child_list, temp_parent_list;
	uint32_t                    idx, sync_id;
	uint16_t                    sync_uid;

	idx = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;
	if (!table || idx <= 0 || idx >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	row = table + idx;

	CAM_DBG(CAM_SYNC,
		"row name:%s sync_id:%i [idx:%u] row_state:%u, uid:%d",
		row->name, row->sync_id, idx, row->state, sync_uid);

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj: idx = %d name = %s",
			idx,
			row->name);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_ACTIVE)
		CAM_DBG(CAM_SYNC,
			"Destroying an active sync object name:%s id:%i",
			row->name, row->sync_id);

	/* Object's child and parent objects will be added into this list */
	INIT_LIST_HEAD(&temp_child_list);
	INIT_LIST_HEAD(&temp_parent_list);

	list_for_each_entry_safe(child_info, temp_child, &row->children_list,
		list) {
		if (child_info->sync_id <= 0)
			continue;

		list_del_init(&child_info->list);
		list_add_tail(&child_info->list, &temp_child_list);
	}

	list_for_each_entry_safe(parent_info, temp_parent, &row->parents_list,
		list) {
		if (parent_info->sync_id <= 0)
			continue;

		list_del_init(&parent_info->list);
		list_add_tail(&parent_info->list, &temp_parent_list);
	}

	/* Cleanup the child to parent link from child list */
	while (!list_empty(&temp_child_list)) {
		child_info = list_first_entry(&temp_child_list,
			struct sync_child_info, list);
		sync_id = (uint32_t)(child_info->sync_id) & sync_uid_access.fenceIdMask;
		child_row = table + sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[sync_id]);

		if (child_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&child_info->list);
			kfree(child_info);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				sync_id]);
			continue;
		}

		if (child_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active child sync obj = %s[%d]",
				child_row->name,
				child_info->sync_id);

		cam_sync_util_cleanup_parents_list(child_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&child_info->list);
		kfree(child_info);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_id]);
	}

	/* Cleanup the parent to child link */
	while (!list_empty(&temp_parent_list)) {
		parent_info = list_first_entry(&temp_parent_list,
			struct sync_parent_info, list);
		sync_id = (uint32_t)(parent_info->sync_id) & sync_uid_access.fenceIdMask;
		parent_row = table + sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[sync_id]);

		if (parent_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&parent_info->list);
			kfree(parent_info);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				sync_id]);
			continue;
		}

		if (parent_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active parent sync obj = %s[%d]",
				parent_row->name,
				parent_info->sync_id);

		cam_sync_util_cleanup_children_list(parent_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&parent_info->list);
		kfree(parent_info);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_id]);
	}

	list_for_each_entry_safe(upayload_info, temp_upayload,
			&row->user_payload_list, list) {
		list_del_init(&upayload_info->list);
		kfree(upayload_info);
	}

	list_for_each_entry_safe(sync_cb, temp_cb,
			&row->callback_list, list) {
		list_del_init(&sync_cb->list);
		kfree(sync_cb);
	}

	row->state = CAM_SYNC_STATE_ACTIVE;
	row->uid = sync_uid;
	reinit_completion(&row->signaled);
	atomic_set(&row->ref_cnt, 0);
	INIT_LIST_HEAD(&row->callback_list);
	INIT_LIST_HEAD(&row->parents_list);
	INIT_LIST_HEAD(&row->children_list);
	INIT_LIST_HEAD(&row->user_payload_list);

	return 0;
}

int cam_sync_util_cb_dispatch(void *priv, void *data)
{
	struct sync_callback_info *cb_info = (struct sync_callback_info *) data;
	sync_callback sync_data = cb_info->callback_func;

	sync_data(cb_info->sync_obj, cb_info->status, cb_info->cb_data);

	kfree(cb_info);
	return 0;
}

void cam_sync_util_dispatch_signaled_cb(struct cam_sync_signal_param *param,
	struct cam_sync_timestamp *time_stamp)
{
	struct sync_callback_info  *sync_cb;
	struct sync_user_payload   *payload_info;
	struct sync_callback_info  *temp_sync_cb;
	struct sync_table_row      *signalable_row;
	struct sync_user_payload   *temp_payload_info;
	uint32_t                    sync_obj, sync_manager_idx;
	uint16_t                    sync_uid;
	struct crm_worker_task     *task = NULL;

	sync_obj = (uint32_t)(param->sync_obj) & sync_uid_access.fenceIdMask;
	sync_uid = (uint32_t)(param->sync_obj) >> sync_uid_access.uidShift;
	signalable_row = sync_dev->sync_table + sync_obj;
	sync_manager_idx = signalable_row->sync_manager_idx;
	if (signalable_row->state == CAM_SYNC_STATE_INVALID) {
		CAM_DBG(CAM_SYNC,
			"Accessing invalid sync object:%s[%i], uid:%d", signalable_row->name,
			sync_obj,
			sync_uid);
		return;
	}

	/* Dispatch kernel callbacks if any were registered earlier */
	list_for_each_entry_safe(sync_cb,
		temp_sync_cb, &signalable_row->callback_list, list) {
		sync_cb->status = param->status;
		list_del_init(&sync_cb->list);
		task = cam_req_mgr_worker_get_task(sync_dev->worker);
		if (IS_ERR_OR_NULL(task)) {
			CAM_ERR(CAM_SYNC, "Failed to get task = %d", PTR_ERR(task));
			kfree(sync_cb);
		} else {
			task->payload = sync_cb;
			task->process_cb = cam_sync_util_cb_dispatch;
			cam_req_mgr_worker_enqueue_task(task, NULL, CRM_TASK_PRIORITY_0);
		}
	}

	/* Dispatch user payloads if any were registered earlier */
	list_for_each_entry_safe(payload_info, temp_payload_info,
		&signalable_row->user_payload_list, list) {
		spin_lock_bh(&sync_dev->cam_sync_eventq_lock[sync_manager_idx]);
		if (!sync_dev->cam_sync_eventq[sync_manager_idx]) {
			spin_unlock_bh(
				&sync_dev->cam_sync_eventq_lock[sync_manager_idx]);
			break;
		}
		spin_unlock_bh(&sync_dev->cam_sync_eventq_lock[sync_manager_idx]);
		if (signalable_row->signaling_en) {
			cam_sync_util_send_v4l2_event(
				CAM_SYNC_V4L_EVENT_ID_CB_TRIG,
				param->sync_obj,
				param->status,
				param->request_id,
				0,
				payload_info->payload_data,
				CAM_SYNC_PAYLOAD_WORDS * sizeof(__u64),
				param->event_cause, time_stamp, param->fh);

			list_del_init(&payload_info->list);
			/*
			 * We can free the list node here because
			 * sending V4L event will make a deep copy
			 * anyway
			 */
			kfree(payload_info);
		}
	}

	/* Send the event without payload in version 5*/
	if ((sync_dev->version == CAM_SYNC_V4L_EVENT_V5) &&
		(signalable_row->type == CAM_SYNC_TYPE_UMD)) {
		if (signalable_row->signaling_en) {
			cam_sync_util_send_v4l2_event(
				CAM_SYNC_V4L_EVENT_ID_CB_TRIG,
				param->sync_obj,
				param->status,
				param->request_id,
				param->applied_crop_req_id,
				NULL,
				0,
				param->event_cause, time_stamp, param->fh);
		}
	}

	/*
	 * This needs to be done because we want to unblock anyone
	 * who might be blocked and waiting on this sync object
	 */
	complete_all(&signalable_row->signaled);
}

void cam_sync_util_send_v4l2_event(uint32_t id,
	uint32_t sync_obj,
	int status,
	uint64_t req_id,
	uint64_t applied_crop_req_id,
	void *payload, int len,
	uint32_t event_cause, struct cam_sync_timestamp *time_stamp, void *fh)
{
	struct v4l2_event event;
	__u64 *payload_data = NULL;

	if (sync_dev->version == CAM_SYNC_V4L_EVENT_V2) {
		struct cam_sync_ev_header_v2 *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V2;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V2(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		ev_header->version = sync_dev->version;
		ev_header->evt_param[CAM_SYNC_EVENT_REASON_CODE_INDEX] =
			event_cause;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR_V2(event, __u64);

	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V3) {

		struct cam_sync_ev_header_v3 *ev_header = NULL;
		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V3;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V3(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		ev_header->version = sync_dev->version;
		ev_header->evt_param[CAM_SYNC_EVENT_REASON_CODE_INDEX] =
			event_cause;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR_V3(event, __u64);

		if (time_stamp) {
			ev_header->evt_param[CAM_SYNC_EVENT_TRACKER_ID] =
				time_stamp->tracker_id;
			ev_header->evt_param[CAM_SYNC_EVENT_SOF_TIMESTAMP] =
				time_stamp->sof_timestamp & 0xFFFFFFFF;
			ev_header->evt_param[CAM_SYNC_EVENT_SOF_TIMESTAMP + 1] =
				(time_stamp->sof_timestamp >> 32) & 0xFFFFFFFF;
			ev_header->evt_param[CAM_SYNC_EVENT_BOOT_TIMESTAMP] =
				time_stamp->boot_timestamp & 0xFFFFFFFF;
			ev_header->evt_param[CAM_SYNC_EVENT_BOOT_TIMESTAMP + 1] =
				(time_stamp->boot_timestamp >> 32) & 0xFFFFFFFF;
			ev_header->evt_param[CAM_SYNC_EVENT_SLAVE_TIMESTAMP] =
				time_stamp->slave_timestamp & 0xFFFFFFFF;
			ev_header->evt_param[CAM_SYNC_EVENT_SLAVE_TIMESTAMP + 1] =
				(time_stamp->slave_timestamp >> 32) & 0xFFFFFFFF;
		}
	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V4) {
		struct cam_sync_ev_header_v4 *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V4;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V4(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		ev_header->version = sync_dev->version;
		ev_header->evt_param.event_cause =
			event_cause;

		if (time_stamp) {
			ev_header->evt_param.tracker_id =
				time_stamp->tracker_id;
			ev_header->evt_param.sof_timestamp =
				time_stamp->sof_timestamp;
			ev_header->evt_param.boot_timestamp =
				time_stamp->boot_timestamp;
			ev_header->evt_param.slave_timestamp =
				time_stamp->slave_timestamp;
		}
		ev_header->evt_param.request_id = req_id;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR_V4(event, __u64);
		CAM_DBG(CAM_SYNC,
			"sizeof(struct cam_sync_ev_header_v4) %d payload 0 0x%llx payload 0x%llx",
			sizeof(struct cam_sync_ev_header_v4),
			*((__u64 *)payload + 0), *((__u64 *)payload + 1));
		CAM_DBG(CAM_SYNC,
			"send v4l2 event version %d sync_obj %d status %d, event_cause %d req_id %lld tracker_id %d sof_timestamp %lld ",
			sync_dev->version, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param.event_cause, ev_header->evt_param.request_id,
			ev_header->evt_param.tracker_id, ev_header->evt_param.sof_timestamp);
	} else if (sync_dev->version == CAM_SYNC_V4L_EVENT_V5) {
		struct cam_sync_ev_header_v5 *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V5;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V5(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		ev_header->version = sync_dev->version;
		ev_header->evt_param.event_cause =
			event_cause;

		if (time_stamp) {
			ev_header->evt_param.tracker_id =
				time_stamp->tracker_id;
			ev_header->evt_param.sof_timestamp =
				time_stamp->sof_timestamp;
			ev_header->evt_param.boot_timestamp =
				time_stamp->boot_timestamp;
			ev_header->evt_param.slave_timestamp =
				time_stamp->slave_timestamp;
		}
		ev_header->evt_param.sensor_req_id = req_id;
		ev_header->evt_param.applied_crop_req_id = applied_crop_req_id;

		CAM_DBG(CAM_SYNC,
			"send v4l2 event version %d sync_obj %d status %d, event_cause %d sensor_req_id %lld applied_crop_req_id %lld tracker_id %d sof_timestamp %lld ",
			sync_dev->version, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param.event_cause, ev_header->evt_param.sensor_req_id,
			ev_header->evt_param.applied_crop_req_id, ev_header->evt_param.tracker_id,
			ev_header->evt_param.sof_timestamp);
	} else {
		struct cam_sync_ev_header *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT;

		ev_header = CAM_SYNC_GET_HEADER_PTR(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR(event, __u64);
	}

	if (sync_dev->version != CAM_SYNC_V4L_EVENT_V5)
		memcpy(payload_data, payload, len);

	v4l2_event_queue_fh(fh, &event);
	CAM_DBG(CAM_SYNC, "send v4l2 event version %d for sync_obj :%d",
		sync_dev->version,
		sync_obj);
}

int cam_sync_util_update_parent_state(struct sync_table_row *parent_row,
	int new_state)
{
	int rc = 0;

	switch (parent_row->state) {
	case CAM_SYNC_STATE_ACTIVE:
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		parent_row->state = new_state;
		break;

	case CAM_SYNC_STATE_SIGNALED_ERROR:
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
		break;

	case CAM_SYNC_STATE_INVALID:
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

void cam_sync_util_cleanup_children_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_var)
{
	struct sync_child_info *child_info = NULL;
	struct sync_child_info *temp_child_info = NULL;
	uint32_t                curr_sync_obj;
	uint32_t                sync_obj = sync_var & sync_uid_access.fenceIdMask;

	list_for_each_entry_safe(child_info,
			temp_child_info, &row->children_list, list) {
		curr_sync_obj = (uint32_t)(child_info->sync_id) & sync_uid_access.fenceIdMask;
		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj != sync_obj))
			continue;

		list_del_init(&child_info->list);
		kfree(child_info);

		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj == sync_obj))
			break;
	}
}

void cam_sync_util_cleanup_parents_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_var)
{
	struct sync_parent_info *parent_info = NULL;
	struct sync_parent_info *temp_parent_info = NULL;
	uint32_t                 curr_sync_obj, sync_obj;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	list_for_each_entry_safe(parent_info,
			temp_parent_info, &row->parents_list, list) {
		curr_sync_obj = (uint32_t)(parent_info->sync_id) & sync_uid_access.fenceIdMask;
		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj != sync_obj))
			continue;

		list_del_init(&parent_info->list);
		kfree(parent_info);

		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj == sync_obj))
			break;
	}
}

enum sync_is_uid_valid cam_sync_check_uid_valid(uint32_t sync_var)
{
	struct sync_table_row *row = NULL;
	uint32_t sync_obj;
	uint16_t requested_uid;

	sync_obj = (uint32_t)sync_var & sync_uid_access.fenceIdMask;
	requested_uid = (uint32_t)sync_var >> sync_uid_access.uidShift;

	row = sync_dev->sync_table + sync_obj;

	if ((requested_uid < row->uid) &&
		((row->uid - requested_uid) > (CAM_SYNC_MAX_UID/2)))
		return SYNC_UID_NEW;

	if ((requested_uid > row->uid) &&
		((requested_uid - row->uid) < (CAM_SYNC_MAX_UID/2)))
		return SYNC_UID_NEW;

	if (requested_uid == row->uid)
		return SYNC_UID_CURRENT;

	return SYNC_UID_OLD;
}
