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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <blinkt_interface/blinkt.hpp>
#include <blinkt_interface/color.hpp>

namespace py = pybind11;
using namespace blinkt_interface;

PYBIND11_MODULE(blinkt_interface, m)
{
    py::class_<Pixel>(m, "Pixel")
        .def_readwrite("red", &Pixel::red)
        .def_readwrite("green", &Pixel::green)
        .def_readwrite("blue", &Pixel::blue)
        .def_readwrite("brightness", &Pixel::brightness)
        .def(py::init<uint8_t, uint8_t, uint8_t, double>())
        .def(py::init<>())
        .def("set", [](Pixel & orig_value, const Pixel & new_value) { orig_value = new_value; });

    py::class_<Blinkt>(m, "Blinkt")
        .def(py::init<>())
        .def("setPixel", &Blinkt::setPixel)
        .def("getPixelArray", &Blinkt::getPixelArray, py::return_value_policy::reference_internal)
        .def("display", &Blinkt::display)
        .def("clear", &Blinkt::clear)
        .def("number_of_pixels", &Blinkt::number_of_pixels);

    m.def_submodule("color")
        .def("red", &color::red)
        .def("lime", &color::lime)
        .def("green", &color::green)
        .def("blue", &color::blue)
        .def("white", &color::white)
        .def("aqua", &color::aqua)
        .def("off", &color::off);
}
