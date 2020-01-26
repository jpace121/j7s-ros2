#include "blinkt_interface/blinkt.hpp"
#include <chrono>
#include <thread>

namespace blinkt_interface {

Blinkt::Blinkt()
    : _pixel_array{},
      _rpi_chip{"gpiochip0"},
      _data_line{_rpi_chip.get_line(_data_pin_number)},
      _clk_line{_rpi_chip.get_line(_clk_pin_number)} {
  _data_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
  _clk_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
}

unsigned int Blinkt::number_of_pixels() const
{
    return _pixel_array.size();
}

void Blinkt::setPixel(uint8_t pixel_number, const Pixel& pixel) {
  BusPixel bus_pixel(
      pixel.red, pixel.green, pixel.blue,
      static_cast<uint8_t>(
          0b11100000 | static_cast<uint8_t>((pixel.brightness * 31.0) / 0x1F)));

  _pixel_array[pixel_number] = bus_pixel;
}

void Blinkt::display()
{
    start_frame();
    for(const auto pixel : _pixel_array)
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
    _data_line.set_value(byte & 0x80);
    _clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
    byte = byte << 1;
    _clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
  }
}

void Blinkt::start_frame() {
  _data_line.set_value(0);
  for (unsigned int cnt = 0; cnt < 32; cnt++) {
    _clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
    _clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
  }
}

void Blinkt::end_frame() {
  _data_line.set_value(0);
  for (unsigned int cnt = 0; cnt < 36; cnt++) {
    _clk_line.set_value(1);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
    _clk_line.set_value(0);
    std::this_thread::sleep_for(std::chrono::seconds(_sleep_time_ms));
  }
}

}  // namespace blinkt_interface
