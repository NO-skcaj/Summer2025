#pragma once

#include <units/length.h>
#include <units/voltage.h>
#include <units/current.h>
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

    constexpr auto MaxSpeed                 = 4.572_mps; // fastest theoretical speed of the swerve modules
    constexpr auto MaxAngularSpeed          = std::numbers::pi * 2_rad_per_s;

    // Define the absolute encoder value for forward
    constexpr auto FrontRightForwardAngle          = -0.193604 * 2.0_rad * std::numbers::pi;
    constexpr auto FrontLeftForwardAngle           = -0.422119 * 2.0_rad * std::numbers::pi;
    constexpr auto RearRightForwardAngle           = -0.174561 * 2.0_rad * std::numbers::pi;
    constexpr auto RearLeftForwardAngle            =  0.268555 * 2.0_rad * std::numbers::pi;

    // Drive motor parameters
    constexpr auto DriveMaximumAmperage            = 30_A;
    constexpr auto DriveMotorReduction             = 6.75;
    constexpr auto WheelDiameter                   = 0.098022_m;
    constexpr auto WheelCircumference              = WheelDiameter * std::numbers::pi;
    constexpr auto DriveMotorConversion            = WheelCircumference / DriveMotorReduction;

    constexpr auto DriveP                          = 0.10;
    constexpr auto DriveI                          = 0.02;
    constexpr auto DriveD                          = 0.00;
    constexpr auto DriveV                          = 0.10;
    constexpr auto DriveA                          = 0.10;

    // Angle motor parameters
    constexpr auto AngleMaximumAmperage            = 30_A;
    constexpr auto AngleMotorRevolutions           = 21.5;  // The number of motor revolutions per wheel revolutions
    constexpr auto AngleRadiansToMotorRevolutions  = (2.0_rad * std::numbers::pi) / AngleMotorRevolutions;  // Radians to motor revolutions	

    constexpr auto AngleP                          = 1.00;
    constexpr auto AngleI                          = 0.00;
    constexpr auto AngleD                          = 0.20;
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