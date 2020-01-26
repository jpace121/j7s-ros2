#pragma once
#include "blinkt_interface/blinkt.hpp"
#include "rclcpp/rclcpp.hpp"

class Rainbow : public rclcpp::Node {
 public:
  Rainbow();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr _timer;

  blinkt_interface::Blinkt _blinkt;
  blinkt_interface::Pixel _on_pixel;
  blinkt_interface::Pixel _off_pixel;
  bool _state;
};
