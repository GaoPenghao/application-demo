load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_jerk_problem",
    srcs = ["piecewise_jerk_problem.cc"],
    hdrs = ["piecewise_jerk_problem.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/common:planning_gflags",
        "@osqp",
    ],
)

cc_library(
    name = "piecewise_jerk_path_problem",
    srcs = ["piecewise_jerk_path_problem.cc"],
    hdrs = ["piecewise_jerk_path_problem.h"],
    deps = [
        ":piecewise_jerk_problem",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_problem",
    srcs = ["piecewise_jerk_speed_problem.cc"],
    hdrs = ["piecewise_jerk_speed_problem.h"],
    deps = [
        ":piecewise_jerk_problem",
        "//modules/planning/common/speed:speed_data"
    ],
)

cpplint()
