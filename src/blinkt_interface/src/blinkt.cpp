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
#include "blinkt_interface/blinkt.hpp"
#include <chrono>
#include <thread>

namespace blinkt_interface
{
Blinkt::Blinkt() :
    _pixel_array{},
    _rpi_chip{"gpiochip0"},
    _data_line{_rpi_chip.get_line(_data_pin_number)},
    _clk_line{_rpi_chip.get_line(_clk_pin_number)}
{
    _data_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
    _clk_line.request({"blinkt", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
}

unsigned int Blinkt::number_of_pixels() const
{
    return _pixel_array.size();
}

PixelArray & Blinkt::getPixelArray()
{
    return _pixel_array;
}

std::vector<BusPixel> Blinkt::getBusPixels() const
{
    std::vector<BusPixel> bus_pixels;
    for (const auto & pixel : _pixel_array)
    {
        bus_pixels.emplace_back(pixel);
    }
    return bus_pixels;
}

void Blinkt::clear()
{
    const Pixel offPixel(0, 0, 0, 0);
    for (auto & pixel : _pixel_array)
    {
        pixel = offPixel;
    }
    display();
}

void Blinkt::setPixel(uint8_t pixel_number, const Pixel & pixel)
{
    if (pixel_number < _pixel_array.size())
    {
        _pixel_array[pixel_number] = pixel;
    }
}

void Blinkt::display()
{
    // Write bus pixels to bus.
    start_frame();
    for (const auto pixel : getBusPixels())
    {
        write_byte(pixel.brightness);
        write_byte(pixel.blue);
        write_byte(pixel.green);
        write_byte(pixel.red);
    }
    end_frame();
}

void Blinkt::write_byte(uint8_t byte)
{
    for (unsigned int cnt = 0; cnt < 8; cnt++)
    {
        _data_line.set_value(byte & 0x80);
        _clk_line.set_value(1);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
        byte = byte << 1;
        _clk_line.set_value(0);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
    }
}

void Blinkt::start_frame()
{
    _data_line.set_value(0);
    for (unsigned int cnt = 0; cnt < 32; cnt++)
    {
        _clk_line.set_value(1);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
        _clk_line.set_value(0);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
    }
}

void Blinkt::end_frame()
{
    _data_line.set_value(0);
    for (unsigned int cnt = 0; cnt < 36; cnt++)
    {
        _clk_line.set_value(1);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
        _clk_line.set_value(0);
        std::this_thread::sleep_for(std::chrono::microseconds(_sleep_time_us));
    }
}

}  // namespace blinkt_interface
