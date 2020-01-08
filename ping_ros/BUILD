
load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_app(
    name = "ping_ros",
    app_json_file = "ping_ros.app.json",
    modules = ["//packages/ping_ros:ping_components"]
)

isaac_cc_module(
    name = "ping_components",
    srcs = [
    	"Ping.cpp"
    ],
    hdrs = [
    	"Ping.hpp"
    ],
    deps = [
        "//packages/ros_bridge/gems",
    	"//third_party:ros"
    ],
)
