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

#include <blinkt_interface/blinkt.hpp>

// Commonly used colors with configureable brightness levels.

namespace blinkt_interface
{
namespace color
{
Pixel red(double brightness)
{
    return Pixel(255, 0, 0, brightness);
}

Pixel lime(double brightness)
{
    return Pixel(0, 255, 0, brightness);
}

Pixel green(double brightness)
{
    return Pixel(0, 125, 0, brightness);
}

Pixel blue(double brightness)
{
    return Pixel(0, 0, 255, brightness);
}

Pixel white(double brightness)
{
    return Pixel(255, 255, 255, brightness);
}

Pixel aqua(double brightness)
{
    return Pixel(0, 255, 255, brightness);
}

Pixel off(double brightness)
{
    return Pixel(0, 0, 0, brightness);
}
}  // namespace color
}  // namespace blinkt_interface
