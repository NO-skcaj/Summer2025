#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

#include "Constants/Controller.h"


namespace GripperPose
{
    inline frc2::CommandPtr GripperPose(GripperPoseEnum gripperPose, Gripper *gripper)
    {
        return frc2::cmd::RunOnce( [gripperPose, gripper] { gripper->SetPose(gripperPose); } );
    }
}