#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/AprilTags.h"

#include "commands/AutonomousOneCoral.h"
#include "commands/AprilTagGrabAlgae.h"
#include "commands/GripperActivate.h"
#include "commands/ChassisDrivePose.h"
#include "commands/GripperPose.h"

#include "Constants.h"
#include "ConstantsGripperPoseCoral.h"

class AutonomousScoreAlgaeAndCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousScoreAlgaeAndCoral>
{
    public:

        explicit AutonomousScoreAlgaeAndCoral(Drivetrain *drivetrain, Gripper *gripper, AprilTags* aprilTags);
};
