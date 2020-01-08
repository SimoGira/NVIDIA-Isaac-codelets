load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_app(
    name = "ping_ros_pub",
    app_json_file = "ping_ros_pub.app.json",
    modules = ["//packages/ping_ros_pub:ping_components"]
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
