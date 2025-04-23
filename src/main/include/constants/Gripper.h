#pragma once

#include <numbers>

#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/current.h>


namespace Constants
{

namespace Elevator
{
    constexpr auto P                               = 2.00;             // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.00;             // Integral:        No output for integrated error
    constexpr auto D                               = 0.10;             // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.25;             // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.12;             // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.01;             // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 180_tps;          // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 360_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = units::turns_per_second_cubed_t{3600}; // Jerk

    constexpr auto PositionToTurnsConversionFactor = 64.0 / (0.06378 * 3.0 * std::numbers::pi); // The number of motor rotations per meter

    constexpr auto MaximumAmperage                 = 60_A;

    constexpr auto MinimumPosition                 = 0_m;
    constexpr auto MaximumPosition                 = 1.7_m;

    constexpr auto HeightOffset                    = 0.025_m;
}

namespace Arm
{
    constexpr auto P                               = 2.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.0;              // Integral:        No output for integrated error
    constexpr auto D                               = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.0;              // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.0;              // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.0;              // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 160_tps;          // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 500_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 500_tr_per_s_cu;  // Jerk

    constexpr auto AngleToTurnsConversionFactor    = 360_deg / 36;     // 36 to 1 gear box

    constexpr auto MaximumAmperage                 = 40_A;

    constexpr auto MinimumPosition                 = -20_deg;
    constexpr auto PastElevatorPosition            =  25_deg;
    constexpr auto MaximumPosition                 = 180_deg;

    constexpr auto AngleOffset                     = 5.0_deg;
}

namespace Wrist
{
    constexpr auto P                             = 0.8;              // Proportional
    constexpr auto I                             = 0.0;              // Integral
    constexpr auto D                             = 1.0;              // Differential

    constexpr auto MaximumVelocity               = 2000.0;           // Rotations per minute (RPM)
    constexpr auto MaximumAcceleration           = 1000.0;           // Acceleration
    constexpr auto AllowedError                  =   0.04;           // Allowed error for the pid controller (smaller values are more accurate)

    constexpr auto MaximumAmperage               =  20;
    constexpr auto AngleToTurnsConversionFactor  = 360_deg / 20;     // 20 to 1 gear box

    constexpr auto MinimumPosition               = -10_deg;          // Note: Need to calibrate angle to motor rotations
    constexpr auto MaximumPosition               = 200_deg;

    constexpr auto AngleOffset                   = 1.0_deg;
}

namespace Gripper
{
    constexpr auto MaximumAmperage      =  20;

    constexpr auto MeanAnalogInput      = 0.11811 / 2.0;

    constexpr auto GripperWheelDeadZone = 0.01;
    constexpr auto AnalogConversion     = 75.0;
}

}