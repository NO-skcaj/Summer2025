#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Leds.h"

class SetLeds : public frc2::CommandHelper<frc2::Command, SetLeds>
{
    public:

        explicit SetLeds(int Mode,                       Leds *leds);
        explicit SetLeds(int Mode, units::second_t time, Leds *leds);

        void     Initialize()          override;
        bool     IsFinished()          override;
        bool     RunsWhenDisabled()    const override;

    private:

        int             m_mode;           // The LED mode for the command
        units::second_t m_time;           // The length of time that the LEDS will be set to the given mode. Infinite by default.
        Leds           *m_leds;           // Pointer to the LED subsystem class

        bool            m_timed = false;  // Determines if the LED sequence is timed
        units::second_t m_startTime;      // The start of the LED sequence
};