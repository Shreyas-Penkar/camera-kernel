/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_IFE_CSID_LITE_634_H_
#define _CAM_IFE_CSID_LITE_634_H_

#include "cam_ife_csid_common.h"
#include "cam_ife_csid_dev.h"
#include "cam_ife_csid_hw_ver2.h"
#include "cam_irq_controller.h"
#include "cam_isp_hw_mgr_intf.h"

static struct cam_ife_csid_ver2_common_reg_info
			cam_ife_csid_lite_634_cmn_reg_info = {
	.hw_version_addr                         = 0x0,
	.cfg0_addr                               = 0x4,
	.global_cmd_addr                         = 0x8,
	.reset_cfg_addr                          = 0xc,
	.reset_cmd_addr                          = 0x10,
	.irq_cmd_addr                            = 0x14,
	.rup_aup_cmd_addr                        = 0x18,
	.offline_cmd_addr                        = 0x1C,
	.shdr_master_slave_cfg_addr              = 0x20,
	.top_irq_status_addr                     = 0x7C,
	.top_irq_mask_addr                       = 0x80,
	.top_irq_clear_addr                      = 0x84,
	.top_irq_set_addr                        = 0x88,
	.buf_done_irq_status_addr                = 0x8C,
	.buf_done_irq_mask_addr                  = 0x90,
	.buf_done_irq_clear_addr                 = 0x94,
	.buf_done_irq_set_addr                   = 0x98,

	/*configurations */
	.major_version                                = 6,
	.minor_version                                = 8,
	.version_incr                                 = 0,
	.num_rdis                                     = 4,
	.num_pix                                      = 1,
	.num_ppp                                      = 0,
	.rst_done_shift_val                           = 1,
	.path_en_shift_val                            = 31,
	.dt_id_shift_val                              = 27,
	.vc_shift_val                                 = 22,
	.dt_shift_val                                 = 16,
	.crop_shift_val                               = 16,
	.decode_format_shift_val                      = 12,
	.decode_format1_supported                     = false,
	.frame_id_decode_en_shift_val                 = 1,
	.multi_vcdt_vc1_shift_val                     = 2,
	.multi_vcdt_dt1_shift_val                     = 7,
	.multi_vcdt_en_shift_val                      = 0,
	.timestamp_stb_sel_shift_val                  = 0,
	.vfr_en_shift_val                             = 0,
	.early_eof_supported                          = 1,
	.vfr_supported                                = 1,
	.multi_vcdt_supported                         = 1,
	.frame_id_dec_supported                       = 1,
	.measure_en_hbi_vbi_cnt_mask                  = 0xc,
	.measure_pixel_line_en_mask                   = 0x3,
	.crop_pix_start_mask                          = 0x3fff,
	.crop_pix_end_mask                            = 0xffff,
	.crop_line_start_mask                         = 0x3fff,
	.crop_line_end_mask                           = 0xffff,
	.drop_supported                               = 1,
	.ipp_irq_mask_all                             = 0x7FFF,
	.rdi_irq_mask_all                             = 0x7FFF,
	.rst_loc_path_only_val                        = 0x0,
	.rst_location_shift_val                       = 4,
	.rst_loc_complete_csid_val                    = 0x1,
	.rst_mode_frame_boundary_val                  = 0x0,
	.rst_mode_immediate_val                       = 0x1,
	.rst_cmd_hw_reset_complete_val                = 0x1,
	.rst_cmd_sw_reset_complete_val                = 0x2,
	.rst_cmd_irq_ctrl_only_val                    = 0x4,
	.timestamp_strobe_val                         = 0x2,
	.top_reset_irq_mask                           = 0x1,
	.top_buf_done_irq_mask                        = 0x2000,
	.global_reset                                 = 1,
	.rup_supported                                = 1,
	.only_master_rup                              = 1,
	.format_measure_height_mask_val               = 0xFFFF,
	.format_measure_height_shift_val              = 0x10,
	.format_measure_width_mask_val                = 0xFFFF,
	.format_measure_width_shift_val               = 0x0,
	.camif_irq_support                            = true,
	.is_core_clk_gate_enable                      = true,
};

static struct cam_ife_csid_ver2_reg_info cam_ife_csid_lite_634_reg_info = {
	.irq_reg_info                         = &cam_ife_csid_lite_650_irq_reg_info,
	.cmn_reg                              = &cam_ife_csid_lite_634_cmn_reg_info,
	.csi2_reg                             = &cam_ife_csid_lite_650_csi2_reg_info,
	.buf_done_irq_reg_info                =
		&cam_ife_csid_lite_650_buf_done_irq_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_IPP]   = &cam_ife_csid_lite_650_ipp_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_PPP]   = NULL,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_0] = &cam_ife_csid_lite_650_rdi_0_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_1] = &cam_ife_csid_lite_650_rdi_1_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_2] = &cam_ife_csid_lite_650_rdi_2_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_3] = &cam_ife_csid_lite_650_rdi_3_reg_info,
	.need_top_cfg = 0,
	.rx_irq_desc        = cam_ife_csid_lite_650_rx_irq_desc,
	.path_irq_desc      = cam_ife_csid_lite_650_path_irq_desc,
};
#endif /* _CAM_IFE_CSID_LITE_634_H_ */
