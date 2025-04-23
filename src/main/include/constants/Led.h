#pragma once

#include <units/length.h>


namespace Constants
{

namespace Led
{
    constexpr auto PwmPort     =   9;

    constexpr auto Length      = 400;              // The length of the LED string
    constexpr auto Brightness  =   0.5;

    constexpr auto LedSpacing  =   0.008333333_m;  // The spacing between LEDs in meters

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  20;              // The delay between strobe flashes
    constexpr auto HvaDelay    =  20;              // The delay between HVA color changes
}

}