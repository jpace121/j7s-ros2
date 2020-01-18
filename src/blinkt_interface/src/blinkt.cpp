#include "blinkt_interface/blinkt.hpp"
#include <chrono>
#include <thread>

namespace blinkt_interface {

Blinkt::Blinkt()
    : pixel_array{},
      rpi_chip{"gpiochip0"},
      data_line{rpi_chip.get_line(data_pin_number)},
      clk_line{rpi_chip.get_line(clk_pin_number)} {
  data_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
  clk_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
}

unsigned int Blinkt::number_of_pixels() const
{
    return pixel_array.size();
}

void Blinkt::setPixel(uint8_t pixel_number, const Pixel& pixel) {
  BusPixel bus_pixel(
      pixel.red, pixel.green, pixel.blue,
      static_cast<uint8_t>(
          0b11100000 | static_cast<uint8_t>((pixel.brightness * 31.0) / 0x1F)));

  pixel_array[pixel_number] = bus_pixel;
}

void Blinkt::display()
{
    start_frame();
    for(const auto pixel : pixel_array)
    {
        write_byte(pixel.brightness);
        write_byte(pixel.blue);
        write_byte(pixel.green);
        write_byte(pixel.red);
    }
    end_frame();
}

void Blinkt::write_byte(uint8_t byte) {
  for (unsigned int cnt = 0; cnt < 8; cnt++) {
    data_line.set_value(byte & 0x80);
    clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
    byte = byte << 1;
    clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
  }
}

void Blinkt::start_frame() {
  data_line.set_value(0);
  for (unsigned int cnt = 0; cnt < 32; cnt++) {
    clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
    clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
  }
}

void Blinkt::end_frame() {
  data_line.set_value(0);
  for (unsigned int cnt = 0; cnt < 36; cnt++) {
    clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
    clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time_ms));
  }
}

}  // namespace blinkt_interface
