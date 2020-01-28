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
#include "rclcpp/rclcpp.hpp"

class Rainbow : public rclcpp::Node
{
public:
  Rainbow();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr _timer;

  blinkt_interface::Blinkt _blinkt;
  std::vector<blinkt_interface::Pixel> _pixels;
  std::vector<blinkt_interface::Pixel>::iterator _iter;
};
