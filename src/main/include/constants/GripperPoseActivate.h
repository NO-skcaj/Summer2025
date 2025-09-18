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

#include "commands/SimpleCommands.h"

#include "subsystems/Gripper.h"


namespace Constants
{

namespace GripperPoseActivate
{
    struct GripperActivationData
    {
        units::length::meter_t   ElevatorOffset       = 0.0_m;
        units::time::second_t    ElevatorWait         = 0.0_s;
        units::angle::degree_t   ArmOffset            = 0.0_deg;
        units::time::second_t    ArmMoveWait          = 0.0_s;
        bool                     BothWheels           = true;
        units::voltage::volt_t   GripperVoltage       = 0.0_V;
        units::time::second_t    GripperPlaceWait     = 0.0_s;
        units::length::meter_t   ElevatorFinish       = 0.0_m;
        units::angle::degree_t   ArmFinish            = 0.0_deg;
    };

    // Coral1
    constexpr GripperActivationData Coral1
    {
        0.0_m,
        0.0_s,
        0.0_deg,
        0.0_s,
        true,
        2.0_V,
        1.0_s,
        0.0_m,
        0.0_deg
    };

    // Coral23
    constexpr GripperActivationData Coral23
    {
        -6_in,
        0.25_s,
        20.0_deg,
        1.0_s,
        true,
        -1.0_V,
        5.0_s,
        0.0_m,
        0.0_deg
    };
    // Coral4
    constexpr GripperActivationData Coral4
    {
        -11.0_in,
        1.0_s,
        0.0_deg,
        0.0_s,
        true,
        -2.0_V,
        3.0_s,
        0.0_m,
        0.0_deg
    };

    // Algae Processor
    constexpr GripperActivationData AlgaeProcessor
    {
        0.0_m,
        0.0_s,
        0.0_deg,
        0.0_s,
        true,
        -5.0_V,
        5.0_s,
        0.0_m,
        0.0_deg
    };

    // Algae Barge
    constexpr GripperActivationData AlgaeBarge
    {
        0.0_m,
        0.0_s,
        0.0_deg,
        0.0_s,
        true,
        -12.0_V,
        5.0_s,
        0.0_m,
        0.0_deg
    };
}

}