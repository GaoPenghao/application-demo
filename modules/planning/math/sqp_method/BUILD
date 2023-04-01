load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bfgs",
    hdrs = ["bfgs.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "qp",
    srcs = ["qp.cc"],
    hdrs = ["qp.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "sqp",
    srcs = ["sqp.cc"],
    hdrs = ["sqp.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":qp",
        ":bfgs",
        "//cyber",
    ],
)

cc_test(
    name = "sqp_test",
    size = "small",
    srcs = ["sqp_test.cc"],
    deps = [
        ":sqp",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
