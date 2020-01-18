#include "blinkt_test/rainbow.hpp"
#include <chrono>

Rainbow::Rainbow()
    : Node("blinkt_rainbow"),
      blinkt{},
      on_pixel{255, 255, 255, 1.0},
      off_pixel{0, 0, 0, 1.0},
      state{false} {
  timer = create_wall_timer(std::chrono::milliseconds(10),
                            std::bind(&Rainbow::timer_callback, this));
}

void Rainbow::timer_callback() {
  if (state) {
    for (unsigned int i = 0; i < blinkt.number_of_pixels(); i++) {
      blinkt.setPixel(i, on_pixel);
    }
  } else {
    for (unsigned int i = 0; i < blinkt.number_of_pixels(); i++) {
      blinkt.setPixel(i, off_pixel);
    }
  }

  blinkt.display();
  state = not state;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rainbow>());
  rclcpp::shutdown();

  return 0;
}
