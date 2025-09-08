#pragma once

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/Command.h>

#include "constants/Drivetrain.h"


using namespace pathplanner;

namespace ChassisDrivePose
{
    inline frc2::CommandPtr ChassisDrivePose(std::string CommandName)
    {
        return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));
    }

    inline frc2::CommandPtr ChassisDrivePose(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side
    {
        return AutoBuilder::pathfindToPose(targetPose, Constants::PathPlanner::Constraints);
    }

    inline frc2::CommandPtr ChassisDrivePoseFlipped(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side then flipped to red
    {
        return AutoBuilder::pathfindToPoseFlipped(targetPose, Constants::PathPlanner::Constraints);
    }
};
