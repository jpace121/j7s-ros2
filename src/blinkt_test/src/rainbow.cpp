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
#include "blinkt_test/rainbow.hpp"
#include <chrono>

Rainbow::Rainbow()
: Node("blinkt_rainbow"),
  _blinkt{}
{
  auto brightness{declare_parameter("brightness").get<double>()};

  _pixels.emplace_back(255, 0, 0, brightness);    // red
  _pixels.emplace_back(0, 255, 0, brightness);    // green
  _pixels.emplace_back(0, 0, 255, brightness);    // blue
  _pixels.emplace_back(255, 255, 0, brightness);  // red+green = yellow
  _pixels.emplace_back(255, 0, 255, brightness);  // red+blue=purple
  _pixels.emplace_back(0, 255, 255, brightness);  // blue+green = light blue

  _iter = _pixels.begin();
  _timer = create_wall_timer(std::chrono::seconds(1), std::bind(&Rainbow::timer_callback, this));
}

void Rainbow::timer_callback()
{
  for (unsigned int i = 0; i < _blinkt.number_of_pixels(); i++) {
    _blinkt.setPixel(i, *_iter);
  }

  _iter++;
  if (_iter == _pixels.end()) {
    _iter = _pixels.begin();
  }

  RCLCPP_INFO(get_logger(), "Switching.");

  _blinkt.display();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rainbow>());
  rclcpp::shutdown();

  return 0;
}
