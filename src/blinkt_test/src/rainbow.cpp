#include "blinkt_test/rainbow.hpp"
#include <chrono>

Rainbow::Rainbow()
: Node("blinkt_rainbow"),
  _blinkt{},
  _on_pixel{255, 255, 255, 1.0},
  _off_pixel{0, 0, 0, 1.0},
  _state{false}
{
  _timer = create_wall_timer(std::chrono::milliseconds(10),
      std::bind(&Rainbow::timer_callback, this));
}

void Rainbow::timer_callback()
{
  if (_state) {
    RCLCPP_DEBUG(get_logger(), "Turning on.");
    for (unsigned int i = 0; i < _blinkt.number_of_pixels(); i++) {
      _blinkt.setPixel(i, _on_pixel);
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Turning off.");
    for (unsigned int i = 0; i < _blinkt.number_of_pixels(); i++) {
      _blinkt.setPixel(i, _off_pixel);
    }
  }

  _blinkt.display();
  _state = not _state;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rainbow>());
  rclcpp::shutdown();

  return 0;
}
