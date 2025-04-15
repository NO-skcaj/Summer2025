#include "commands/AutonomousParallel.h"

/// @brief Comand to test running commands in parallel .
/// @param leds The LED subsystem.
/// @param drivetrain The Drivetrain subsystem.
AutonomousParallel::AutonomousParallel(Leds *leds, Drivetrain *drivetrain)
{
    // Set the command name
    SetName("AutonomousParallel");

    AddCommands(ChassisDriveTime(2_s, 0.5_mps, drivetrain), SetLeds(LedMode::Rainbow, 5_s, leds));
}
