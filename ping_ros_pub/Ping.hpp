#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"

class Ping : public isaac::alice::Codelet {
public:
  void start() override;
  void tick()  override;
  void stop()  override;

  ISAAC_PARAM(std::string, message_simo, "message from isaac..");

  // ROS publisher queue depth
  ISAAC_PARAM(int, publisher_queue_size, 1000);
  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, publisher_channel_name, "isaac_string_message_status");

private:
  // Hide the ROS implementation details
  struct RosPingData;
  std::unique_ptr<RosPingData> ping_data_;
};

ISAAC_ALICE_REGISTER_CODELET(Ping);
