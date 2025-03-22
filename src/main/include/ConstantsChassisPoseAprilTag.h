#pragma once

#pragma region Includes
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

#include "subsystems/Gripper.h"
#pragma endregion

#pragma region ConstantsChassisAprilTagToPose
namespace ConstantsChassisAprilTagToPose
{
    constexpr auto ChassisSpeed                  = 1.0_mps;
    constexpr auto TimeoutTime                   = 10_s;

    constexpr auto CoralStationDistanceOffsetX   = 0.0_m;
    constexpr auto CoralStationDistanceOffsetY   = 0.0_m;
    constexpr auto CoralStationAngleOffset       = 0.0_deg;

    constexpr auto CoralReefRightDistanceOffsetX = 0.6_m;
    constexpr auto CoralReefRightDistanceOffsetY = 0.3_m;
    constexpr auto CoralReefRightAngleOffset     = 0.0_deg;

    constexpr auto CoralReefLeftDistanceOffsetX  = 0.6_m;
    constexpr auto CoralReefLeftDistanceOffsetY  = 0.3_m;
    constexpr auto CoralReefLeftAngleOffset      = 0.0_deg;

    constexpr auto AlgaeReefDistanceOffsetX      = 0.0_m;
    constexpr auto AlgaeReefDistanceOffsetY      = 0.0_m;
    constexpr auto AlgaeReefAngleOffset          = 0.0_deg;

    constexpr auto AlgaeProcessorDistanceOffsetX = 0.0_m;
    constexpr auto AlgaeProcessorDistanceOffsetY = 0.0_m;
    constexpr auto AlgaeProcessorAngleOffset     = 0.0_deg;

    constexpr auto AlgaelBargeDistanceOffsetX    = 0.0_m;
    constexpr auto AlgaelBargeDistanceOffsetY    = 0.0_m;
    constexpr auto AlgaelBargeAngleOffset        = 0.0_deg;
}
#pragma endregion
