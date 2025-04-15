#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/Command.h>


using namespace pathplanner;

namespace ChassisDrivePose
{
    frc2::CommandPtr ChassisDrivePose(auto CommandName)
    {
        return PathPlannerPath::fromPathFile(CommandName);
    };

    frc2::CommandPtr ChassisDrivePose(frc::Pose2d targetPose)
    {

        PathConstraints constraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq); // The constraints for this path.
        // PathConstraints constraints = PathConstraints::unlimitedConstraints(12_V); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        // We make a shared pointer here since the path following commands require a shared pointer
        auto path = std::make_shared<PathPlannerPath>(
            constraints,
            GoalEndState(0.0_mps, frc::Rotation2d(-90_deg)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return AutoBuilder::followPath(path);
    };
};