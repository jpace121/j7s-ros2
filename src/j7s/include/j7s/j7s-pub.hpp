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
#include <string>
#include "j7s_msgs/msg/color.hpp"
#include "j7s_msgs/msg/led_state.hpp"
#include "rclcpp/rclcpp.hpp"

class J7sPub : public rclcpp::Node
{
public:
    J7sPub();

private:
    const double _freq;
    const long int _led_index;
    const j7s_msgs::msg::Color _color;
    double _time;
    const double _pub_freq;
    const double _brightness;

    rclcpp::Publisher<j7s_msgs::msg::LedState>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    void timer_callback();
    j7s_msgs::msg::Color string_to_color(const std::string & st) const;
};
