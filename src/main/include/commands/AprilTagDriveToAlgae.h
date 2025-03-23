#pragma once

#include <functional>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperPose.h"
#include "commands/GripperActivate.h"

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

class AprilTagDriveToAlgae : public frc2::CommandHelper<frc2::SequentialCommandGroup, AprilTagDriveToAlgae>
{
    public:

        explicit AprilTagDriveToAlgae(GripperPoseEnum  gripperPose,
                                      Gripper         *gripper,
                                      Drivetrain      *drivetrain);
};
