#pragma once

#include <frc2/command/WaitCommand.h>

#include "commands/PrepareAndAlign.h"
#include "commands/GripperActivate.h"


namespace AutoScore
{
       inline frc2::CommandPtr AutoScore(Drivetrain* drivetrain, Gripper* gripper, std::pair<frc::Transform2d, GripperPoseEnum> scoreState)
       {
              return PrepareAndAlign::PrepareAndAlign(drivetrain, gripper, scoreState)
                     .AndThen(std::move(frc2::WaitCommand(2.0_s).ToPtr()))
                     .AndThen(std::move(GripperActivate(gripper).ToPtr()));
       }
}
