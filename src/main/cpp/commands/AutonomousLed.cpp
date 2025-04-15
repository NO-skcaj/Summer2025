#include "commands/AutonomousLed.h"

/// @brief Automonous command to test sequential commands.
/// @param leds The LED subsystem
AutonomousLed::AutonomousLed(Leds *leds)
{
    // Set the command name
    SetName("AutonomusLed");

    // Add the LED commands
    AddCommands(SetLeds(LedMode::Rainbow,   5_s, leds),
                SetLeds(LedMode::Strobe,    5_s, leds),
                SetLeds(LedMode::HvaColors, 5_s, leds));
}
