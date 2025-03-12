#pragma once

#include <functional>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperPose.h"

#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "ConstantsChassisPoseAprilTag.h"

class AprilTagScoreCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup, AprilTagScoreCoral>
{
    public:

        explicit AprilTagScoreCoral(GripperPoseEnum               gripperPose,
                                    const std::function<bool ()> &GetJoystickToggle,
                                    AprilTags                    *aprilTags,
                                    Gripper                      *gripper,
                                    Drivetrain                   *drivetrain);
};
