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
#include <cstdint>
#include <array>
#include <gpiod.hpp>
#include "blinkt_interface/visibility_control.h"

namespace blinkt_interface
{

// Pixel as packed when sent over bus.
struct BusPixel
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t brightness;

  BusPixel(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
  : red{red}, green{green}, blue{blue}, brightness{brightness} {}

  BusPixel()
  : red{0}, green{0}, blue{0}, brightness{0} {}
};


typedef std::array<BusPixel, 8> BusPixelArray;

// Pixel as seen by a user of the class.
struct Pixel
{
  // rgb bytes between 0-255
  uint8_t red{0};
  uint8_t green{0};
  uint8_t blue{0};
  // brightness float between 0 and 1.0
  double brightness{0.0};

  Pixel(uint8_t red, uint8_t green, uint8_t blue, double brightness)
  : red{red}, green{green}, blue{blue}, brightness{brightness}
  {
  }

  BusPixel toBusPixel() const
  {
    return BusPixel(
      red, green, blue,
      static_cast<uint8_t>(
        0b11100000 | (static_cast<uint8_t>(brightness * 31.0) & 0x1F)) );
  }
};

class Blinkt
{
public:
  Blinkt();
  void setPixel(uint8_t pixel_number, const Pixel & pixel);
  BusPixelArray & getBusPixelArray();
  void display();
  void clear();
  unsigned int number_of_pixels() const;

private:
  void write_byte(uint8_t byte);
  void start_frame();
  void end_frame();

  const unsigned int _data_pin_number{23};
  const unsigned int _clk_pin_number{24};
  const unsigned long _sleep_time_us{0};

  BusPixelArray _pixel_array;

  gpiod::chip _rpi_chip;
  gpiod::line _data_line;
  gpiod::line _clk_line;
};

}  // namespace blinkt_interface
