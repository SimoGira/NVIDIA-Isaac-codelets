#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"

class Ping : public isaac::alice::Codelet {
public:
  void start() override;
  void tick()  override;
  void stop()  override;
  ISAAC_PARAM(std::string, message_simo, "Nothing message yet..");
  ISAAC_PROTO_TX(PingProto, ping_channel)


  // ROS publisher queue depth
  ISAAC_PARAM(int, publisher_queue_size, 1000);
  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, publisher_channel_name, "isaac_string_message_status");
  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, subscriber_channel_name, "isaac_string_message_request");

  std::string msg_converted;
  bool msg_converted_rdy;

private:
  // Hide the ROS implementation details
  struct RosPingData;
  std::unique_ptr<RosPingData> ping_data_;
};

ISAAC_ALICE_REGISTER_CODELET(Ping);
