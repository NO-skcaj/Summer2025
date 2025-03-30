#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "commands/ChassisDrivePose.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperActivate.h"
#include "commands/GripperPose.h"

#include "Constants.h"

class AutonomousTwoCoralAprilTag : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousTwoCoralAprilTag> 
{
    public:
    
        explicit AutonomousTwoCoralAprilTag(GripperPoseEnum                               gripperPoseEnum,
                                            std::function<std::string()>                  getStartingPosition,
                                            std::function<ChassDrivePoseParameters ()>    getOneCoralParemters,
                                            std::function<ChassDriveAprilTagParameters()> getSecondCoralParemters,
                                            std::function<ChassDriveAprilTagParameters()> getAprilTagParameters,
                                            std::function<ChassDriveAprilTagParameters()> getStationParemters,
                                            Drivetrain *drivetrain, Gripper *gripper);
};
