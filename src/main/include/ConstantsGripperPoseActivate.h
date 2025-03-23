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
#include <units/voltage.h>

#include "subsystems/Gripper.h"
#pragma endregion

#pragma region ConstantsGripperPoseActivate
namespace ConstantsGripperPoseActivate
{
    // Coral1
    constexpr auto Coral1ElevatorOffset           =  0.0_m;
    constexpr auto Coral1ElevatorWait             =  0.0_s;
    constexpr auto Coral1ArmOffset                =  0.0_deg;
    constexpr auto Coral1ArmMoveWait              =  0.0_s;
    constexpr auto Coral1BothWheels               =  true;
    constexpr auto Coral1GripperVoltage           = -3.0_V;
    constexpr auto Coral1GripperPlaceWait         =  1.0_s;
    constexpr auto Coral1ElevatorFinish           =  0.0_m;
    constexpr auto Coral1ArmFinish                =  0.0_deg;

    // Coral23
    constexpr auto Coral123ElevatorOffset         = -0.2_m;
    constexpr auto Coral123ElevatorWait           =  1.0_s;
    constexpr auto Coral123ArmOffset              =  0.0_deg;
    constexpr auto Coral123ArmMoveWait            =  0.1_s;
    constexpr auto Coral123BothWheels             =  true;
    constexpr auto Coral123GripperVoltage         = -3.0_V;
    constexpr auto Coral123GripperPlaceWait       =  5.0_s;
    constexpr auto Coral123ElevatorFinish         =  0.0_m;
    constexpr auto Coral123ArmFinish              =  0.0_deg;

    // Coral4
    constexpr auto Coral4ElevatorOffset           = -0.26_m;
    constexpr auto Coral4ElevatorWait             =  1.0_s;
    constexpr auto Coral4ArmOffset                =  0.0_deg;
    constexpr auto Coral4ArmMoveWait              =  0.0_s;
    constexpr auto Coral4BothWheels               =  true;
    constexpr auto Coral4GripperVoltage           = -3.0_V;
    constexpr auto Coral4GripperPlaceWait         =  5.0_s;
    constexpr auto Coral4ElevatorFinish           =  0.0_m;
    constexpr auto Coral4ArmFinish                =  0.0_deg;

    // Algae Processor
    constexpr auto AlgaeProcessorElevatorOffset   =  0.0_m;
    constexpr auto AlgaeProcessorElevatorWait     =  0.0_s;
    constexpr auto AlgaeProcessorArmOffset        =  0.0_deg;
    constexpr auto AlgaeProcessorArmMoveWait      =  0.0_s;
    constexpr auto AlgaeProcessorBothWheels       =  true;
    constexpr auto AlgaeProcessorGripperVoltage   = -5.0_V;
    constexpr auto AlgaeProcessorGripperPlaceWait =  5.0_s;
    constexpr auto AlgaeProcessorElevatorFinish   =  0.0_m;
    constexpr auto AlgaeProcessorArmFinish        =  0.0_deg;

    // Algae Barge
    constexpr auto AlgaeBargeElevatorOffset       =  0.0_m;
    constexpr auto AlgaeBargeElevatorWait         =  0.0_s;
    constexpr auto AlgaeBargeArmOffset            =  0.0_deg;
    constexpr auto AlgaeBargeArmMoveWait          =  0.0_s;
    constexpr auto AlgaeBargeBothWheels           =  true;
    constexpr auto AlgaeBargeGripperVoltage       = -12.0_V;
    constexpr auto AlgaeBargeGripperPlaceWait     =  5.0_s;
    constexpr auto AlgaeBargeElevatorFinish       =  0.0_m;
    constexpr auto AlgaeBargeArmFinish            =  0.0_deg;
}
#pragma endregion
