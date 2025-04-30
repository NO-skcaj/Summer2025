#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "commands/GripperPose.h"
#include "commands/AlignToNearestTag.h"


namespace PrepareAndAlign
{
    inline frc2::CommandPtr PrepareAndAlign(Drivetrain *drivetrain, Gripper *gripper, std::pair<frc::Transform2d, GripperPoseEnum> scoreState)
    {
        return AlignToNearestTag::AlignToNearestTag(drivetrain, scoreState.first)
               .AlongWith(GripperPose::GripperPose(scoreState.second, gripper));
    }
};