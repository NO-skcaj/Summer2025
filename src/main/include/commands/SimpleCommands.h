#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "subsystems/Gripper.h"
#include "subsystems/Climb.h"
#include "subsystems/Leds.h"
#include "subsystems/Drivetrain/Drivetrain.h"
#include "subsystems/Drivetrain/Odometry.h"

#include "constants/Climb.h"
#include "constants/GripperPose.h"
#include "constants/GripperPoseActivate.h"


using namespace pathplanner;

// Offsetting commands

frc2::CommandPtr OffsetElevator(bool isUp, units::meter_t offset = Constants::Elevator::HeightOffset);

frc2::CommandPtr OffsetArm(bool isUp, units::degree_t offset = Constants::Arm::AngleOffset);

frc2::CommandPtr OffsetWrist(bool isUp, units::degree_t offset = Constants::Wrist::AngleOffset);

// Climb Commands

frc2::CommandPtr ClimbSetVoltage(double isUp, units::volt_t voltage = Constants::Climb::ClimbVoltage);

// Drive Commands

frc2::CommandPtr ChassisSetSwerveWheelAnglesToZero();

frc2::CommandPtr ChassisDrive(std::function<frc::ChassisSpeeds()>  speedsGetter);

frc2::CommandPtr ChassisZeroHeading();

frc2::CommandPtr FlipFieldCentricity();

// Path following commands

frc2::CommandPtr ChassisDrivePose(std::string CommandName);

frc2::CommandPtr ChassisDrivePose(frc::Pose2d targetPose);

frc2::CommandPtr ChassisDrivePoseFlipped(frc::Pose2d targetPose);

// This command will align the robot to the nearest AprilTag
// It will use the AprilTag's pose to determine the target position and rotation
// The robot will drive towards the target position and rotate to face the target rotation
frc2::CommandPtr AlignToNearestTag(frc::Transform2d targetOffset = {0_in, -18_in, 0_deg});

// Gripper Commands

frc2::CommandPtr GripperPose(Constants::GripperPose::GripperPoseEnum gripperPose);

frc2::CommandPtr SetGripperWheels(std::function<Constants::GripperPose::GripperWheelState()> state);

frc2::CommandPtr SetGripperWheels(Constants::GripperPose::GripperWheelState state);

frc2::CommandPtr StopGripperWheels();

// LEDs
frc2::CommandPtr SetLeds(LedMode ledMode, units::second_t howLong);

// Sequentials

frc2::CommandPtr PrepareAndAlign(frc::Transform2d prepState1, Constants::GripperPose::GripperPoseEnum prepState2);

frc2::CommandPtr AutoScore(frc::Transform2d scoreState1, Constants::GripperPose::GripperPoseEnum scoreState2);

frc2::CommandPtr GripperActivate();

// Odometry periodic

frc2::CommandPtr UpdateOdometry();