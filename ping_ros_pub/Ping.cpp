#include "Ping.hpp"

#include "packages/ros_bridge/gems/include_before_ros.hpp"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"


// Internal struct for holding the ROS node handle and the publisher and subscriber channels
// Note the callback queue. To avoid spinning other codelets it is necessary to generate
// a separate callback queue per codelet.
struct Ping::RosPingData {
  ros::NodeHandle node;
  ros::Publisher pub;
  ros::CallbackQueue callbackQueue;
};


void Ping::start() {
	// Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // roscore needs to be started externally.
  if (!ros::master::check()) {
    reportFailure("ROS is not running. Please start roscore first.");
    return;
  }

  ping_data_ = std::make_unique<RosPingData>();
  ping_data_->node.setCallbackQueue(&(ping_data_->callbackQueue));
  ping_data_->pub = ping_data_->node.advertise<std_msgs::String>(
      get_publisher_channel_name(), get_publisher_queue_size());

  // Because ROS requires that the message queues be manually pumped we need to tick.
  tickPeriodically();
}

void Ping::tick() {
	if (ros::ok()) {
		std_msgs::String msg_to_ros;
		msg_to_ros.data = get_message_simo();
		ping_data_->pub.publish(msg_to_ros);

		// Pump the queue for this codelet.
		ping_data_->callbackQueue.callAvailable();
	} else {
		LOG_ERROR("An error has occurred within ROS.");
	}
}

void Ping::stop() {
  if (ping_data_) {
    ping_data_->pub.shutdown();
    ping_data_ = nullptr;
  }
}
