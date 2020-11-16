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
#include "j7s/j7s-sub.hpp"

J7sSub::J7sSub() :
    Node("j7s_subscriber"),
    _blinkt{},
    _stateSub{}
{
    _blinkt.clear();

    _stateSub = create_subscription<j7s_msgs::msg::LedState>(
        "led_state", rclcpp::QoS(rclcpp::KeepAll()),
        std::bind(&J7sSub::led_callback, this, std::placeholders::_1));
}

void J7sSub::led_callback(j7s_msgs::msg::LedState::SharedPtr msg)
{
    if (msg->index < _blinkt.number_of_pixels())
    {
        _blinkt.setPixel(msg->index, msg_to_pixel(msg->color, msg->brightness));
        _blinkt.display();
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Sent invalid index. What?");
    }
}

blinkt_interface::Pixel J7sSub::msg_to_pixel(
    const j7s_msgs::msg::Color & color, double brightness) const
{
    switch (color.color)
    {
        case j7s_msgs::msg::Color::RED:
            return blinkt_interface::color::red(brightness);
        case j7s_msgs::msg::Color::LIME:
            return blinkt_interface::color::lime(brightness);
        case j7s_msgs::msg::Color::GREEN:
            return blinkt_interface::color::green(brightness);
        case j7s_msgs::msg::Color::BLUE:
            return blinkt_interface::color::blue(brightness);
        case j7s_msgs::msg::Color::WHITE:
            return blinkt_interface::color::white(brightness);
        case j7s_msgs::msg::Color::AQUA:
            return blinkt_interface::color::aqua(brightness);
        case j7s_msgs::msg::Color::OFF:
        default:
            return blinkt_interface::color::off(brightness);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<J7sSub>());
    rclcpp::shutdown();

    return 0;
}
