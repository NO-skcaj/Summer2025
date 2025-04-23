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

namespace GripperPose
{

    struct GripperPoseState
    {
        units::meter_t  ElevatorHeight;
        units::degree_t ArmAngle;
        units::degree_t WristAngle;
        bool            GripperBothWheels;
        units::volt_t   GripperVoltage;
    };

    namespace Coral
    {
        constexpr auto CoralGroundPickupVoltage      = 4.0_V;
        constexpr auto CoralGripVoltage              = 0.5_V;

        constexpr GripperPoseState HomeState
        {
            /*ElevatorHeight   */ 0.05_m,
            /*ArmAngle         */ 25_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };

        constexpr GripperPoseState GroundState
        {
            /*ElevatorHeight   */ 0.114432_m,
            /*ArmAngle         */ 138.765_deg,
            /*WristAngle       */ 180.0_deg,
            /*GripperBothWheels*/ false,
            /*GripperVoltage   */ CoralGroundPickupVoltage
        };

        constexpr GripperPoseState StationState
        {
            /*ElevatorHeight   */ 0.053983_m,
            /*ArmAngle         */ 5.439453_deg,
            /*WristAngle       */ 180.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ 5.0_V
        };

        constexpr GripperPoseState L1State
        {
            /*ElevatorHeight   */ 0.450036_m,
            /*ArmAngle         */ 97.778320_deg,
            /*WristAngle       */ 180.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };

        constexpr GripperPoseState AutonomousL1State
        {
            /*ElevatorHeight   */ 0.450036_m,
            /*ArmAngle         */ 97.778320_deg,
            /*WristAngle       */ 180.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };

        constexpr GripperPoseState L2State
        {
            /*ElevatorHeight   */ 0.369155_m,
            /*ArmAngle         */ 45.0_deg,
            /*WristAngle       */ 90.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };

        constexpr GripperPoseState L3State
        {
            /*ElevatorHeight   */ 0.789707_m,
            /*ArmAngle         */ 45.0_deg,
            /*WristAngle       */ 90.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };

        constexpr GripperPoseState L4State
        {
            /*ElevatorHeight   */ 1.68_m,
            /*ArmAngle         */ 93.579102_deg,
            /*WristAngle       */ 90.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ CoralGripVoltage
        };
    }

    namespace Algae
    {
        constexpr auto AlgaeGripVoltage           = 3.0_V;
        constexpr auto AlgaePickupVoltage         = 5.0_V;
        constexpr auto AlgaeReefGetVoltage        = 6.5_V;

        constexpr GripperPoseState GroundState
        {
            /*ElevatorHeight   */ 0.421899_m,
            /*ArmAngle         */ 126.918945_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaePickupVoltage
        };

        constexpr GripperPoseState OnCoralState
        {
            /*ElevatorHeight   */ 0.3_m,
            /*ArmAngle         */ 90.0_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaePickupVoltage
        };

        constexpr GripperPoseState LowState
        {
            /*ElevatorHeight   */ 0.411439_m,
            /*ArmAngle         */ 90.0_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaeReefGetVoltage
        };

        constexpr GripperPoseState HighState
        {
            /*ElevatorHeight   */ 0.866149_m,
            /*ArmAngle         */ 90.0_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaeReefGetVoltage
        };

        constexpr GripperPoseState ProcessorState
        {
            /*ElevatorHeight   */ 0.139206_m,
            /*ArmAngle         */ 100.0_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaeGripVoltage
        };

        constexpr GripperPoseState BargeState
        {
            /*ElevatorHeight   */ 1.7_m,
            /*ArmAngle         */ 63.154297_deg,
            /*WristAngle       */ 0.0_deg,
            /*GripperBothWheels*/ true,
            /*GripperVoltage   */ AlgaeGripVoltage
        };
    }

}

}
