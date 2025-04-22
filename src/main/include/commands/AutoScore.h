#pragma once

#include <frc2/command/WaitCommand.h>

#include "commands/AlignToNearestTag.h"
#include "commands/GripperActivate.h"


namespace AutoScore
{
inline frc2::CommandPtr AutoScore(Drivetrain* drivetrain, Gripper* gripper, std::pair<frc::Pose2d, GripperPoseEnum> scoreState)
{
return PrepareAndAlign::PrepareAndAlign(drivetrain, gripper, scoreState.second)
       .AndThen(std::move(frc2::WaitCommand(2.0_s).ToPtr()))
       .AndThen(std::move(GripperActivate(gripper)), {gripper});
}
}
