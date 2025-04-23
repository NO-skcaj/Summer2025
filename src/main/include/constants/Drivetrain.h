#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>


namespace Constants
{

namespace Drivetrain
{
    // Chassis configuration
    constexpr auto TrackWidth               = 25_in;  // Distance between centers of right and left wheels on robot
    constexpr auto WheelBase                = 25_in;  // Distance between centers of front and back wheels on robot

    constexpr auto MaxSpeed                 = 4.8_mps;
    constexpr auto MaxAngularSpeed          = std::numbers::pi * 2_rad_per_s;
}

namespace ChassisPose
{
    constexpr auto   MaxSpeed               = 2_mps;
    constexpr auto   MaxAcceleration        = 2_mps_sq;
    constexpr auto   MaxAngularSpeed        = 3.142_rad_per_s;
    constexpr auto   MaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double PXController           = 4.0;
    constexpr double PYController           = 4.0;
    constexpr double PProfileController     = 5.0;
}

}