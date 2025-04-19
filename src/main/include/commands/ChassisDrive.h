#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

namespace ChassisDrive
{
    inline frc2::CommandPtr ChassisDrive(std::function<units::meters_per_second_t()>  forward,
                                  std::function<units::meters_per_second_t()>  strafe,
                                  std::function<units::radians_per_second_t()> angle,
                                  Drivetrain                                  *drivetrain)
    {
        return frc2::cmd::Run( [forward, strafe, angle, drivetrain] { drivetrain->Drive(forward(), strafe(), angle()); }, {drivetrain});
    }
}