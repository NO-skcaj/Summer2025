#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/AprilTags.h"

#include "commands/AutonomousOneCoralAprilTAg.h"
#include "commands/AprilTagDriveToAlgae.h"
#include "commands/GripperActivate.h"
#include "commands/ChassisDrivePose.h"
#include "commands/GripperPose.h"

#include "Constants.h"
#include "ConstantsGripperPoseCoral.h"
#include "ConstantsChassisPoseAutonomous.h"

class AutonomousCoralAndAlgae : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousCoralAndAlgae>
{
    public:

        explicit AutonomousCoralAndAlgae(GripperPoseEnum gripperPoseEnum,
                                         std::function<std::string()>              getStartingPosition,
                                         std::function<ChassDrivePoseParameters()> getParameters,
                                         Drivetrain *drivetrain, Gripper *gripper, AprilTags* aprilTags);
};
