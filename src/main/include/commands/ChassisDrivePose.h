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

    inline frc2::CommandPtr ChassisDrivePose(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side
    {
        return AutoBuilder::pathfindToPose(targetPose, Constraints);
    }

    inline frc2::CommandPtr ChassisDrivePoseFlipped(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side then flipped to red
    {
        return AutoBuilder::pathfindToPoseFlipped(targetPose, Constraints);
    }
};
