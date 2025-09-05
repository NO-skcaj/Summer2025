#pragma once

#include <cmath>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>

#include "subsystems/Drivetrain.h"

#include "commands/ChassisDrivePose.h"


namespace Constants
{

    namespace AlignConstraints
    {
        constexpr pathplanner::PathConstraints Constraints{3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq};
    }
}

namespace AlignToNearestTag
{

    // This command will align the robot to the nearest AprilTag
    // It will use the AprilTag's pose to determine the target position and rotation
    // The robot will drive towards the target position and rotate to face the target rotation
    inline frc2::CommandPtr AlignToNearestTag(Drivetrain *drivetrain, frc::Transform2d targetOffset = {0_in, -18_in, 0_deg})
    {
        // This doesn't need to be a variable. When I wrote this, I just really liked using lambdas.
        std::function<frc::Pose2d(frc::Pose2d, frc::Transform2d)> getTargetWithOffset = 
            [] (frc::Pose2d targetPosition, frc::Transform2d targetOffset)
            {
                // Rotate offset
                return frc::Pose2d{
                    targetPosition.X() +                  targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value()) - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),
                    targetPosition.Y() +                  targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value()) + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),
                    targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()};
            };

        return ChassisDrivePose::ChassisDrivePose(getTargetWithOffset(drivetrain->GetNearestTag(), targetOffset));
    }
};
