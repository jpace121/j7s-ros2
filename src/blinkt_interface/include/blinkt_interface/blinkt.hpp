#pragma once
#include <cstdint>
#include <array>
#include <gpiod.hpp>
#include "blinkt_interface/visibility_control.h"

namespace blinkt_interface {

// Pixel as seen by a user of the class.
struct Pixel {
  // rgb bytes between 0-255
  uint8_t red{255};
  uint8_t green{255};
  uint8_t blue{255};
  // brightness float between 0 and 1.0
  double brightness{0.2};
};

class Blinkt {
 public:
  Blinkt();
  void setPixel(uint8_t pixel_number, const Pixel& pixel);
  void display();
  unsigned int number_of_pixels() const;

 private:
  // Pixel as packed when sent over bus.
  struct BusPixel {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness;

    BusPixel(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
        : red{red}, green{green}, blue{blue}, brightness{brightness}{}

      BusPixel() : red{255}, green{255}, blue{255}, brightness{255}{}
  };

  void write_byte(uint8_t byte);
  void start_frame();
  void end_frame();

  const unsigned int data_pin_number{23};
  const unsigned int clk_pin_number{24};
  const unsigned long sleep_time_ms{0};

  std::array<BusPixel, 8> pixel_array;

  gpiod::chip rpi_chip;
  gpiod::line data_line;
  gpiod::line clk_line;
};

}  // namespace blinkt_interface
