#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

#include "Constants/Controller.h"

class GripperPose : public frc2::CommandHelper<frc2::Command, GripperPose>
{
    public:

        explicit GripperPose(GripperPoseEnum gripperPose, Gripper *gripper);

        void     Execute()             override;
        bool     IsFinished()          override;

    private:

        GripperPoseEnum m_gripperPose;  // The gripper pose
        Gripper        *m_gripper;      // The Gripper subsystem
};
