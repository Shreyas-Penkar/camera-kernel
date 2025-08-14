load("@bazel_skylib//rules:write_file.bzl", "write_file")

common_configs = [
    "CONFIG_SPECTRA_ISP=y",
    "CONFIG_SPECTRA_ICP=y",
    "CONFIG_SPECTRA_JPEG=y",
    "CONFIG_SPECTRA_SENSOR=y",
]

dependency_config = [
    "CONFIG_INTERCONNECT_QCOM=y",
    "CONFIG_TZ_DCP_API_VER_2=y",
]

project_configs = select({
    # Project-specific configs
    ":no_project": [],
    ":niobe": dependency_config + [
        "CONFIG_SPECTRA_SECURE_CAMNOC_REG_UPDATE=y",
    ],
    ":seraph": [],
    ":neo61": [],
})

def get_project_defconfig(target, variant):
    rule_name = "{}_{}_project_defconfig".format(target, variant)
    write_file(
        name = rule_name,
	out = "{}.generated".format(rule_name),
	content = common_configs + project_configs + [""],
    )

    return rule_name
