ifeq ($(call is-board-platform-in-list, $(TARGET_BOARD_PLATFORM)),true)

# Make target to specify building the camera.ko from within Android build system.
LOCAL_PATH := $(call my-dir)
# Path to DLKM make scripts
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

SMCINVOKE_DLKM_BOARDS := niobe

ifeq ($(TARGET_BOARD_PLATFORM), niobe)
    LOCAL_MODULE_DDK_BUILD := true
endif

LOCAL_MODULE_DDK_BUILD := true

ifneq ($(TARGET_BOARD_PLATFORM),)
LOCAL_MODULE_DDK_EXTRA_ARGS := "--//vendor/qcom/opensource/camera-kernel:project_name=$(TARGET_BOARD_PLATFORM)"
endif

# List of board platforms for which SMMU_PROXY_DLKM driver API should be enabled
SMMU_PROXY_DLKM_BOARDS := niobe

CAMERA_SRC_FILES := \
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.h,drivers dt-bindings include))\
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.mk,config))\
                    $(addprefix $(LOCAL_PATH)/, $(call all-named-files-under,*.c,drivers))\
                    $(LOCAL_PATH)/dependency.mk \
                    $(LOCAL_PATH)/board.mk      \
                    $(LOCAL_PATH)/product.mk    \
                    $(LOCAL_PATH)/Kbuild

# Target for pre-sil symbols
ifeq ($(CONFIG_CAM_PRESIL), y)
$(warning camera-kernel: Enabling Pre-Sil Kbuild Options!)
KBUILD_OPTIONS := CONFIG_CAM_PRESIL=y
include $(CLEAR_VARS)
$(warning camera-kernel: Enabling Pre-Sil build, exporting symbols!)
LOCAL_SRC_FILES           := $(CAMERA_SRC_FILES)
LOCAL_MODULE              := camera-kernel-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
# Check build for optional dependencies
include $(LOCAL_PATH)/dependency.mk

include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

# Kbuild options
KBUILD_OPTIONS := CAMERA_KERNEL_ROOT=$(TOP)/$(LOCAL_PATH)
KBUILD_OPTIONS += KERNEL_ROOT=$(TOP)/kernel/msm-$(TARGET_KERNEL_VERSION)/
KBUILD_OPTIONS += MODNAME=camera
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)

ifeq ($(TARGET_BOARD_PLATFORM), taro)
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(TOP)/$(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
endif

ifeq ($(TARGET_BOARD_PLATFORM), parrot)
	KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(TOP)/$(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
endif

# Clear shell environment variables from previous android module during build
include $(CLEAR_VARS)

# For incremental compilation support.
LOCAL_SRC_FILES             := $(CAMERA_SRC_FILES)
LOCAL_MODULE_PATH           := $(KERNEL_MODULES_OUT)
LOCAL_MODULE                := camera.ko
LOCAL_MODULE_TAGS           := optional
#LOCAL_MODULE_KBUILD_NAME   := camera.ko
#LOCAL_MODULE_DEBUG_ENABLE  := true

BOARD_VENDOR_KERNEL_MODULES += $(LOCAL_MODULE_PATH)/$(LOCAL_MODULE)

ifeq ($(TARGET_BOARD_PLATFORM), taro)
	LOCAL_REQUIRED_MODULES        := mmrm-module-symvers
	LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
endif

ifeq ($(TARGET_BOARD_PLATFORM), parrot)
	LOCAL_REQUIRED_MODULES        := mmrm-module-symvers
	LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,mmrm-module-symvers)/Module.symvers
endif

# Check build for optional dependencies
include $(LOCAL_PATH)/dependency.mk

# $(info LOCAL_SRC_FILES = $(LOCAL_SRC_FILES))
# $(info intermediates mmrm symvers path = $(call
# intermediates-dir-for,DLKM,mmrm-module-symvers))
# $(info CAMERA_EXTRA_SYMBOLS = $(CAMERA_EXTRA_SYMBOLS))
# $(info CAMERA_EXTRA_CONFIGS = $(CAMERA_EXTRA_CONFIGS))
# $(info LOCAL_ADDITIONAL_DEPENDENCIES = $(LOCAL_ADDITIONAL_DEPENDENCIES))
# $(info LOCAL_REQUIRED_MODULES = $(LOCAL_REQUIRED_MODULES))
# $(info DLKM_DIR = $(DLKM_DIR))
$(info KBUILD_OPTIONS = $(KBUILD_OPTIONS))

ifeq ($(TARGET_BOARD_PLATFORM), lahaina)
# Include Kernel DLKM Android.mk target to place generated .ko file in image
include $(DLKM_DIR)/AndroidKernelModule.mk
# Include Camera UAPI Android.mk target to copy headers
include $(LOCAL_PATH)/include/uapi/Android.mk
else
include $(DLKM_DIR)/Build_external_kernelmodule.mk
endif

endif # End of check for board platform
