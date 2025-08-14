/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_LX7_REG_H_
#define _CAM_LX7_REG_H_

struct cam_lx7_hw_info {
	int32_t ob_irq_status;
	uint32_t ob_irq_mask;
	uint32_t ob_irq_clear;
	uint32_t ob_irq_set;
	uint32_t ob_irq_cmd;
	uint32_t host2icpint;
	uint32_t pfault_info;
};

/* ICP_SYS - Protected reg space defined in AC policy */
#define ICP_LX7_SYS_RESET      0x0
#define ICP_LX7_SYS_CONTROL    0x4
#define ICP_LX7_SYS_STATUS     0xC
#define ICP_LX7_SYS_ACCESS     0x10

#define ICP_LX7_STANDBYWFI     (1 << 7)
#define ICP_LX7_EN_CPU         (1 << 9)
#define ICP_LX7_FUNC_RESET     (1 << 4)

/* ICP WD reg space */
#define ICP_LX7_WD_CTRL        0x8
#define ICP_LX7_WD_INTCLR      0xC

/* These bitfields are shared by OB_MASK, OB_CLEAR, OB_STATUS */
#define LX7_WDT_BITE_WS1       (1 << 6)
#define LX7_WDT_BARK_WS1       (1 << 5)
#define LX7_WDT_BITE_WS0       (1 << 4)
#define LX7_WDT_BARK_WS0       (1 << 3)
#define LX7_ICP2HOSTINT        (1 << 2)

#define LX7_IRQ_CLEAR_CMD       (1 << 1)
#define LX7_IRQ_SET_CMD         (1 << 0)
#define LX7_HOST2ICPINT          (1 << 0)

#endif /* _CAM_LX7_REG_H_ */
