#pragma once

#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/Gripper.h"

namespace Constants
{

namespace GripperPoseAlgae
{
    constexpr auto AlgeaGripVoltage           = 3.0_V;
    constexpr auto AlgaePickupVoltage         = 5.0_V;
    constexpr auto AlgaeReefGetVoltage        = 6.5_V;

    constexpr auto GroundElevator             = 0.421899_m;
    constexpr auto GroundArmAngle             = 126.918945_deg;
    constexpr auto GroundWristAngle           = 0_deg;
    constexpr auto GroundGripperBothWheels    = true;
    constexpr auto GroundGripperVoltage       = AlgaePickupVoltage;

    constexpr auto OnCoralElevator            = 0.3_m;
    constexpr auto OnCoralArmAngle            = 90.0_deg;
    constexpr auto OnCoralWristAngle          = 0.0_deg;
    constexpr auto OnCoralGripperBothWheels   = true;
    constexpr auto OnCoralGripperVoltage      = AlgaePickupVoltage;

    constexpr auto LowElevator                = 0.411439_m;
    constexpr auto LowArmAngle                = 90.0_deg;
    constexpr auto LowWristAngle              = 0.0_deg;
    constexpr auto LowGripperBothWheels       = true;
    constexpr auto LowGripperVoltage          = AlgaeReefGetVoltage;

    constexpr auto HighElevator               = 0.866149_m;
    constexpr auto HighArmAngle               = 90.0_deg;
    constexpr auto HighWristAngle             = 0.0_deg;
    constexpr auto HighGripperBothWheels      = true;
    constexpr auto HighGripperVoltage         = AlgaeReefGetVoltage;

    constexpr auto ProcessorElevator          = 0.139206_m;
    constexpr auto ProcessorArmAngle          = 100.0_deg;
    constexpr auto ProcessorWristAngle        = 0.0_deg;
    constexpr auto ProcessorGripperBothWheels = true;
    constexpr auto ProcessorGripperVoltage    = AlgeaGripVoltage;

    constexpr auto BargeElevator              = 1.7_m;
    constexpr auto BargeArmAngle              = 63.154297_deg;
    constexpr auto BargeWristAngle            = 0.0_deg;
    constexpr auto BargeGripperBothWheels     = true;
    constexpr auto BargeGripperVoltage        = AlgeaGripVoltage;
}

}