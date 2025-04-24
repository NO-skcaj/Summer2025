#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"


namespace ChassisSetSwerveWheelAnglesToZero
{
    inline frc2::CommandPtr ChassisSetSwerveWheelAnglesToZero(Drivetrain *drivetrain)
    {
        return frc2::cmd::RunOnce( [drivetrain] { drivetrain->SetWheelAnglesToZero(); }, {drivetrain} );
    };
}