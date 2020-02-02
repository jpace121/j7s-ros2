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
#include "j7s/j7s-pub.hpp"
#include <string>
#include <chrono>
#include <cmath>

J7sPub::J7sPub()
: Node("j7s_publisher"),
  _freq{declare_parameter("freq").get<double>()},
  _led_index{declare_parameter("led_index").get<long int>()},
  _color{string_to_color(declare_parameter("color").get<std::string>())},
  _pub_freq{declare_parameter("pub_freq").get<double>()},
  _publisher{create_publisher<j7s_msgs::msg::LedState>("led_state", 1)},
  _timer{}
{
  _timer = create_wall_timer(
    std::chrono::microseconds(static_cast<int>((1.0 / _pub_freq) * 1e6)),
    std::bind(&J7sPub::timer_callback, this));
}

void J7sPub::timer_callback()
{
  // Increment time.
  _time = _time + (1.0 / _pub_freq);

  j7s_msgs::msg::LedState state;
  state.color = _color;
  state.index = _led_index;
  state.brightness = 0.5 * sin(_freq * _time) + 0.5;

  _publisher->publish(state);
}


j7s_msgs::msg::Color J7sPub::string_to_color(const std::string & st) const
{
  j7s_msgs::msg::Color color;
  if (st == "aqua") {
    color.color = j7s_msgs::msg::Color::AQUA;
  } else if (st == "red") {
    color.color = j7s_msgs::msg::Color::RED;
  } else if (st == "lime") {
    color.color = j7s_msgs::msg::Color::LIME;
  } else if (st == "green") {
    color.color = j7s_msgs::msg::Color::GREEN;
  } else if (st == "blue") {
    color.color = j7s_msgs::msg::Color::BLUE;
  } else if (st == "white") {
    color.color = j7s_msgs::msg::Color::WHITE;
  } else if (st == "off") {
    color.color = j7s_msgs::msg::Color::OFF;
  } else {
    throw std::string("Invalid parameter.");
  }

  return color;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<J7sPub>());
  rclcpp::shutdown();

  return 0;
}
