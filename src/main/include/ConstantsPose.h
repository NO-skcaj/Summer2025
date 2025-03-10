#pragma once

#pragma region Includes
#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/Gripper.h"
#pragma endregion

//#define USE_APRILTAGS

#pragma region AutonomousConstants
namespace AutonomousConstants
{
    constexpr auto OneCoralSpeed            =  2_mps;

#ifdef USE_APRILTAGS    
    // Center starting position for one coral placement
    constexpr auto OneCoralCenterXDistance  =  0.5_m;
    constexpr auto OneCoralCenterYDistance  =  0_m;
    constexpr auto OneCoralAngleChange      =  0_deg;

    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistance    =  1.0_m;
    constexpr auto OneCoralLeftYDistance    = -0.5_m;
    constexpr auto OneCoralLeftAngleChange  = -60_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistance   = 1.0_m;
    constexpr auto OneCoralRightYDistance   = 0.5_m;
    constexpr auto OneCoralRightAngleChange = 60_deg;
#else
    // Center starting position for one coral placement
    constexpr auto OneCoralCenterXDistance  =  0.5_m;
    constexpr auto OneCoralCenterYDistance  =  0_m;
    constexpr auto OneCoralAngleChange      =  0_deg;

    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistance    =  1.0_m;
    constexpr auto OneCoralLeftYDistance    = -0.5_m;
    constexpr auto OneCoralLeftAngleChange  = -60_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistance   = 1.0_m;
    constexpr auto OneCoralRightYDistance   = 0.5_m;
    constexpr auto OneCoralRightAngleChange = 60_deg;
#endif
    constexpr auto OneCoralTimeOut          = 15_s;

    // Algael/Barge starting position for one coral placement
    constexpr auto AlgaeAndCoralSpeed       =  1_mps;
    constexpr auto AlgaeAndCoralXDistance   = -1_m;
    constexpr auto AlgaeAndCoralYDistance   =  0_m;
    constexpr auto AlgaeAndCoralAngleChange = 180_deg;
    constexpr auto AlgaeAndCoralTimeOut     = 15_s;
};
#pragma endregion

#pragma region AprilTagToPoseConstants
namespace AprilTagToPoseConstants
{
    constexpr auto ChassisSpeed                  = 1.0_mps;
    constexpr auto TimeoutTime                   = 10_s;

    constexpr auto CoralStationDistanceOffsetX   = 0.0_m;
    constexpr auto CoralStationDistanceOffsetY   = 0.0_m;
    constexpr auto CoralStationAngleOffset       = 0.0_deg;

    constexpr auto CoralReefDistanceOffsetX      = 0.5_m;
    constexpr auto CoralReefDistanceOffsetY      = 0.5_m;
    constexpr auto CoralReefAngleOffset          = 0.0_deg;

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

#pragma region CoralPoseConstants
namespace CoralPoseConstants
{
    constexpr auto CoralGroundPickupVoltage      = 4.0_V;  // TODO: Set the voltage
    constexpr auto CoralGripVoltage              = 0.5_V;  // TODO: Set the voltage

    constexpr auto HomeElevator                  = 0.0_m;
    constexpr auto HomeArmAngle                  = 25_deg;
    constexpr auto HomeWristAngle                = 0.0_deg;
    constexpr auto HomeGripperBothWheels         = true;
    constexpr auto HomeGripperVoltage            = 0.0_V;

    constexpr auto GroundElevator                = 0.164432_m;
    constexpr auto GroundArmAngle                = 138.765_deg;
    constexpr auto GroundWristAngle              = 180.0_deg;
    constexpr auto GroundGripperBothWheels       = false;
    constexpr auto GroundGripperVoltage          = CoralGroundPickupVoltage;

    constexpr auto StationElevator               = 0.5_m;
    constexpr auto StationArmAngle               = 45.0_deg;
    constexpr auto StationWristAngle             = 180.0_deg;
    constexpr auto StationGripperBothWheels      = true;
    constexpr auto StationGripperVoltage         = CoralGripVoltage;  // Verify the voltage

    constexpr auto L1Elevator                    = 0.347767_m;
    constexpr auto L1ArmAngle                    = 80.0_deg;
    constexpr auto L1WristAngle                  = 180.0_deg;
    constexpr auto L1GripperBothWheels           = true;
    constexpr auto L1GripperVoltage              = CoralGripVoltage;

    constexpr auto L2Elevator                    = 0.452952_m;
    constexpr auto L2ArmAngle                    = 45.0_deg;
    constexpr auto L2WristAngle                  = 90.0_deg;
    constexpr auto L2GripperBothWheels           = true;
    constexpr auto L2GripperVoltage              = CoralGripVoltage;

    constexpr auto L3Elevator                    = 0.801765_m;
    constexpr auto L3ArmAngle                    = 45.0_deg;
    constexpr auto L3WristAngle                  = 90.0_deg;
    constexpr auto L3GripperBothWheels           = true;
    constexpr auto L3GripperVoltage              = CoralGripVoltage;

    constexpr auto L4Elevator                    = 1.697126_m;
    constexpr auto L4ArmAngle                    = 73.0_deg;
    constexpr auto L4WristAngle                  = 90.0_deg;
    constexpr auto L4GripperBothWheels           = true;
    constexpr auto L4GripperVoltage              = CoralGripVoltage;

    constexpr auto AutonomousL1Elevator          = L1Elevator;
    constexpr auto AutonomousL1ArmAngle          = L1ArmAngle;
    constexpr auto AutonomousL1WristAngle        = 0.0_deg;
    constexpr auto AutonomousL1GripperBothWheels = true;
    constexpr auto AutonomousL1GripperVoltage    = CoralGripVoltage;  // TODO: Verify the voltage
}
#pragma endregion

#pragma region AlgaePoseConstants
namespace AlgaePoseConstants
{
    constexpr auto AlgeaGripVoltage           = 1.0_V;  // TODO: Verify the voltage
    constexpr auto AlgaePickupVoltage         = 3.0_V;  // TODO: Verify the voltage
    constexpr auto AlgaeReefGetVoltage        = 0.0_V;  // TODO: Verify the voltage

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

    constexpr auto LowElevator                = 0.576956_m;
    constexpr auto LowArmAngle                = 90.0_deg;
    constexpr auto LowWristAngle              = 0.0_deg;
    constexpr auto LowGripperBothWheels       = true;
    constexpr auto LowGripperVoltage          = AlgaeReefGetVoltage;

    constexpr auto HighElevator               = 0.973151_m;
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
#pragma endregion

#pragma region ActivateConstants
namespace ActivateConstants
{
    // Coral Ground:
    constexpr auto CoralGroundElevatorOffset    = 0.0_m;
    constexpr auto CoralGroundWait1             = 0.0_s;
    constexpr auto CoralGroundArmOffset         = 0.0_deg;
    constexpr auto CoralGroundWait2             = 0.0_s;
    constexpr auto CoralGroundBothWheels        = true;
    constexpr auto CoralGroundGripperVoltage    = 0.0_V;
    constexpr auto CoralGroundWait3             = 0.0_s;
    constexpr auto CoralGroundElevatorFinish    = 0.0_m;
    constexpr auto CoralGroundArmFinish         = 0.0_deg;

    // Coral Station:
    constexpr auto CoralStationElevatorOffset   = 0.0_m;
    constexpr auto CoralStationWait1            = 0.0_s;
    constexpr auto CoralStationArmOffset        = 0.0_deg;
    constexpr auto CoralStationWait2            = 0.0_s;
    constexpr auto CoralStationBothWheels       = true;
    constexpr auto CoralStationGripperVoltage   = 0.0_V;
    constexpr auto CoralStationWait3            = 0.0_s;
    constexpr auto CoralStationElevatorFinish   = 0.0_m;
    constexpr auto CoralStationArmFinish        = 0.0_deg;

    // Coral1:
    constexpr auto Coral1ElevatorOffset         =  0.0_m;
    constexpr auto Coral1Wait1                  =  0.0_s;
    constexpr auto Coral1ArmOffset              =  0.0_deg;
    constexpr auto Coral1Wait2                  =  0.1_s;
    constexpr auto Coral1BothWheels             =  true;
    constexpr auto Coral1GripperVoltage         = -3.0_V;
    constexpr auto Coral1Wait3                  =  1.0_s;
    constexpr auto Coral1ElevatorFinish         =  0.0_m;
    constexpr auto Coral1ArmFinish              =  0.0_deg;

    // Coral23:
    constexpr auto Coral123ElevatorOffset       = -0.2_m;
    constexpr auto Coral123Wait1                =  1.0_s;
    constexpr auto Coral123ArmOffset            =  0.0_deg;
    constexpr auto Coral123Wait2                =  0.1_s;
    constexpr auto Coral123BothWheels           =  true;
    constexpr auto Coral123GripperVoltage       = -3.0_V;
    constexpr auto Coral123Wait3                =  1.0_s;
    constexpr auto Coral123ElevatorFinish       = -0.2_m;
    constexpr auto Coral123ArmFinish            =  0.0_deg;

    // Coral4:
    constexpr auto Coral4ElevatorOffset         = -0.26_m;
    constexpr auto Coral4Wait1                  = 1.0_s;
    constexpr auto Coral4ArmOffset              = 0.0_deg;
    constexpr auto Coral4Wait2                  = 0.0_s;
    constexpr auto Coral4BothWheels             = true;
    constexpr auto Coral4GripperVoltage         = -3.0_V;
    constexpr auto Coral4Wait3                  = 1.0_s;
    constexpr auto Coral4ElevatorFinish         = -0.26_m;
    constexpr auto Coral4ArmFinish              = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeGroundElevatorOffset    = 0.0_m;
    constexpr auto AlgaeGroundWait1             = 0.0_s;
    constexpr auto AlgaeGroundArmOffset         = 0.0_deg;
    constexpr auto AlgaeGroundWait2             = 0.0_s;
    constexpr auto AlgaeGroundBothWheels        = true;
    constexpr auto AlgaeGroundGripperVoltage    = 0.0_V;
    constexpr auto AlgaeGroundWait3             = 0.0_s;
    constexpr auto AlgaeGroundElevatorFinish    = 0.0_m;
    constexpr auto AlgaeGroundArmFinish         = 0.0_deg;

    // Algae On Coral:
    constexpr auto AlgaeOnCoralElevatorOffset   = 0.0_m;
    constexpr auto AlgaeOnCoralWait1            = 0.0_s;
    constexpr auto AlgaeOnCoralArmOffset        = 0.0_deg;
    constexpr auto AlgaeOnCoralWait2            = 0.0_s;
    constexpr auto AlgaeOnCoralBothWheels       = true;
    constexpr auto AlgaeOnCoralGripperVoltage   = 0.0_V;
    constexpr auto AlgaeOnCoralWait3            = 0.0_s;
    constexpr auto AlgaeOnCoralElevatorFinish   = 0.0_m;
    constexpr auto AlgaeOnCoralArmFinish        = 0.0_deg;

    // Algae Lo:
    constexpr auto AlgaeLoElevatorOffset        = 0.0_m;
    constexpr auto AlgaeLoWait1                 = 0.0_s;
    constexpr auto AlgaeLoArmOffset             = 0.0_deg;
    constexpr auto AlgaeLoWait2                 = 0.0_s;
    constexpr auto AlgaeLoBothWheels            = true;
    constexpr auto AlgaeLoGripperVoltage        = 0.0_V;
    constexpr auto AlgaeLoWait3                 = 0.0_s;
    constexpr auto AlgaeLoElevatorFinish        = 0.0_m;
    constexpr auto AlgaeLoArmFinish             = 0.0_deg;

    // Algae High:
    constexpr auto AlgaeHighElevatorOffset      = 0.0_m;
    constexpr auto AlgaeHighWait1               = 0.0_s;
    constexpr auto AlgaeHighArmOffset           = 0.0_deg;
    constexpr auto AlgaeHighWait2               = 0.0_s;
    constexpr auto AlgaeHighBothWheels          = true;
    constexpr auto AlgaeHighGripperVoltage      = 0.0_V;
    constexpr auto AlgaeHighWait3               = 0.0_s;
    constexpr auto AlgaeHighElevatorFinish      = 0.0_m;
    constexpr auto AlgaeHighArmFinish           = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeProcessorElevatorOffset = 0.0_m;
    constexpr auto AlgaeProcessorWait1          = 0.0_s;
    constexpr auto AlgaeProcessorArmOffset      = 0.0_deg;
    constexpr auto AlgaeProcessorWait2          = 0.0_s;
    constexpr auto AlgaeProcessorBothWheels     = true;
    constexpr auto AlgaeProcessorGripperVoltage = 0.0_V;
    constexpr auto AlgaeProcessorWait3          = 0.0_s;
    constexpr auto AlgaeProcessorElevatorFinish = 0.0_m;
    constexpr auto AlgaeProcessorArmFinish      = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeBargeElevatorOffset     = 0.0_m;
    constexpr auto AlgaeBargeWait1              = 0.0_s;
    constexpr auto AlgaeBargeArmOffset          = 0.0_deg;
    constexpr auto AlgaeBargeWait2              = 0.0_s;
    constexpr auto AlgaeBargeBothWheels         = true;
    constexpr auto AlgaeBargeGripperVoltage     = 0.0_V;
    constexpr auto AlgaeBargeWait3              = 0.0_s;
    constexpr auto AlgaeBargeElevatorFinish     = 0.0_m;
    constexpr auto AlgaeBargeArmFinish          = 0.0_deg;
}
#pragma endregion
