#pragma once
#include "blinkt_interface/blinkt.hpp"
#include "rclcpp/rclcpp.hpp"

class Rainbow : public rclcpp::Node {
 public:
  Rainbow();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer;

  blinkt_interface::Blinkt blinkt;
  blinkt_interface::Pixel on_pixel;
  blinkt_interface::Pixel off_pixel;
  bool state;
};
