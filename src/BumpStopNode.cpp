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

#include "rclcpp/rclcpp.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <stdlib.h> 


using namespace std::chrono_literals; // capacidad de usar 'ms'
using std::placeholders::_1; 

class BumpStopNode : public rclcpp::Node
{
public:
  BumpStopNode()
  : Node("bumstop_node")
  {
    bumpstop_sub = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>("/events/bumper", 10,
    std::bind(&BumpStopNode::bumper_callback, this, _1));
    
    bumpstop_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      100ms, std::bind(&BumpStopNode::timer_callback, this));
      stop = false;
      bump_side = -1;
      r = rand() % 10 + 1;
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist msg;
    if (stop) { 
      msg.linear.x = 0.0;
      // if (bump_side == 0)
      //   msg.angular.z = -1.0;
      // else if (bump_side == 1)
      //   msg.angular.z = 1.0;
      // else if (bump_side == 2)
      //   msg.angular.z = 1.0;
      // usleep(r);
      // msg.angular.z = 0;  
    }
    else {
      msg.linear.x = 0.2;
    }

    bumpstop_pub->publish(msg);
    // tiene que saber si se ha presionado el bumper o no, creamos un subscriptor a 
    //  /events/Bumper. Analizamos el mensaje y según y eso (presionado o liberado) 
    // tomamos decisiones. Si esta presionado (definimos un atributo presionado = True) 
    // se para y si no pues sigue.

  }

void bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
stop = msg->state;
bump_side = msg->bumper;
}


private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bumpstop_pub;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumpstop_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  int bump_side;
  bool stop;
  int r;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BumpStopNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
