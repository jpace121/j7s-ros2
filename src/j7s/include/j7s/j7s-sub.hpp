// Copyright 2020 James Pace
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
#pragma once
#include "blinkt_interface/blinkt.hpp"
#include "blinkt_interface/color.hpp"

#include "j7s_msgs/msg/color.hpp"
#include "j7s_msgs/msg/led_state.hpp"

#include "rclcpp/rclcpp.hpp"

class J7sSub : public rclcpp::Node
{
public:
  J7sSub();

private:
  blinkt_interface::Blinkt _blinkt;
  rclcpp::Subscription<j7s_msgs::msg::LedState>::SharedPtr _stateSub;
  rclcpp::TimerBase::SharedPtr _timer;
  const double _disp_freq;

  void led_callback(j7s_msgs::msg::LedState::SharedPtr msg);
  void timer_callback();
  blinkt_interface::Pixel msg_to_pixel(const j7s_msgs::msg::Color & color, double brightness) const;
};
