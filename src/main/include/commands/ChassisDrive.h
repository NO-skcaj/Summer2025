#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"


namespace ChassisDrive
{
    inline frc2::CommandPtr ChassisDrive(std::function<frc::ChassisSpeeds()>  speedsGetter,
                                  Drivetrain                                  *drivetrain)
    {
        return frc2::cmd::Run( [speedsGetter, drivetrain] { drivetrain->Drive(speedsGetter(), std::nullopt); }, {drivetrain});
    }
}