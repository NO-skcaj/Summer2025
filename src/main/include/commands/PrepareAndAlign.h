#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "commands/GripperPose.h"
#include "commands/AlignToNearestTag.h"


namespace PrepareAndAlign
{
    frc2::CommandPtr PrepareAndAlign(Drivetrain *drivetrain, Gripper *gripper, GripperPoseEnum gripperPose)
    {
        return AlignToNearestTag::AlignToNearestTag(drivetrain).AlongWith(GripperPose::GripperPose(gripperPose, gripper));
    }
};