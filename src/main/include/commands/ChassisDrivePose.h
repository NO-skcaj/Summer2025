#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/Command.h>


using namespace pathplanner;

namespace ChassisDrivePose
{
    inline frc2::CommandPtr ChassisDrivePose(auto CommandName)
    {
        return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));
    }

    inline frc2::CommandPtr ChassisDrivePose(frc::Rotation2d currentRot, // Current robot rotation used as starting state 
                                      frc::Pose2d targetPose)     // End goal state relative to the origin, blue alliance side
    {
        // Create the path using the waypoints created above
        // We make a shared pointer here since the path following commands require a shared pointer
        auto path = std::make_shared<PathPlannerPath>(
            PathPlannerPath::waypointsFromPoses({targetPose}), // The end pose for the path. You can also use a vector of waypoints
            Constants::PathPlanner::Constraints, // The constraints for the path.
            IdealStartingState(0.0_mps, currentRot), // The starting state of the path. You can also use a vector of starting states
            GoalEndState(0.0_mps, targetPose.Rotation()), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            false
        );

        return AutoBuilder::followPath(path);
    }

    inline frc2::CommandPtr ChassisDriveTranslation(frc::Rotation2d currentRot,           // Current robot rotation used as starting state
                                             frc::Translation2d targetTranslation) // End goal state relative to the anchor, by default, the current robot position
    {
        // PathConstraints constraints = PathConstraints::unlimitedConstraints(12_V); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        // We make a shared pointer here since the path following commands require a shared pointer
        auto path = PathPlannerPath(
            {Waypoint(std::nullopt, frc::Translation2d(), std::optional<frc::Translation2d>{targetTranslation})}, // The end pose for the path. You can also use a vector of waypoints
            Constants::PathPlanner::Constraints, // The constraints for the path.
            std::nullopt, // The starting state of the path. You can also use a vector of starting states
            GoalEndState(0.0_mps, targetTranslation.Angle()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder::followPath(std::make_shared<PathPlannerPath>(path));
    }

    inline frc2::CommandPtr ChassisDriveTranslation(frc::Rotation2d currentRot,            // Current robot rotation used as starting state
                                             frc::Translation2d targetTranslation,  // End goal state relative to the anchor, by default, the current robot position
                                             frc::Translation2d anchorTranslation)  // Anchor point for the path. By default, this is the current robot position;
    {
        // Create the path using the waypoints created above
        // We make a shared pointer here since the path following commands require a shared pointer
        auto path = PathPlannerPath(
            {Waypoint(std::nullopt, anchorTranslation, std::optional<frc::Translation2d>{targetTranslation})}, // The end pose for the path. You can also use a vector of waypoints
            Constants::PathPlanner::Constraints, // The constraints for the path.
            std::nullopt, // The starting state of the path. You can also use a vector of starting states
            GoalEndState(0.0_mps, targetTranslation.Angle()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder::followPath(std::make_shared<PathPlannerPath>(path));
    }
};