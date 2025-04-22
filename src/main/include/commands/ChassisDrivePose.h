#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/Command.h>


using namespace pathplanner;

namespace ChassisDrivePose
{
    const pathplanner::PathConstraints Constraints{3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq}; // The constraints for this path.

    inline frc2::CommandPtr ChassisDrivePose(auto CommandName)
    {
        return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));
    }

    inline frc2::CommandPtr ChassisDrivePose(std::function<frc::Pose2d()> currentPoseSupplier, // Current robot pose used as starting state 
                                             frc::Pose2d targetPose)     // End goal state relative to the origin, blue alliance side
    {
        frc::Pose2d currentPose = currentPoseSupplier();

        // Create the path using the waypoints created above
        // We make a shared pointer here since the path following commands require a shared pointer
        auto path = std::make_shared<PathPlannerPath>(
            PathPlannerPath::waypointsFromPoses(std::vector<frc::Pose2d>{currentPose, targetPose}), // The end pose for the path. You can also use a vector of waypoints
            Constraints,                                       // The constraints for the path.
            std::nullopt,
            GoalEndState(0.0_mps, targetPose.Rotation())      // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder::followPath(path);
    }
};