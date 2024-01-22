// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utility"

#include "bumpstop/BumpStopNode.coo"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interface/msg/bumper_event.hpp"

using namespace std::chrono_literals; //usar msg
using std::placeholders::_1;

class BumpStopNode : public rclcpp::Node
{
public:
  BumpStopNode()
  : Node("bump_stop_node")
  {
    bumpstop_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(
      100ms, std::bind(&BumpStopNode::timer_callback, this));
  }

  void timer_callback()
  {
    message_.data += 1;
    bumpstop_->publish(message_);
  }

private:
  rclcpp::BumpStop<std_msgs::msg::Int32>::SharedPtr bumpstop_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BumpStopNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
