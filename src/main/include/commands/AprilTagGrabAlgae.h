#pragma once

#include <functional>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperPose.h"
#include "commands/GripperActivate.h"

#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

class AprilTagGrabAlgae : public frc2::CommandHelper<frc2::SequentialCommandGroup, AprilTagGrabAlgae>
{
    public:

        explicit AprilTagGrabAlgae(GripperPoseEnum  gripperPose,
                                   AprilTags       *aprilTags,
                                   Gripper         *gripper,
                                   Drivetrain      *drivetrain);
};
