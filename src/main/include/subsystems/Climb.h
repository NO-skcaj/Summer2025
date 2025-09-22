#pragma once

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/DigitalInput.h"
#include "lib/hardware/TalonFX.h"

#include "Constants/CanIds.h"
#include "Constants/Climb.h"


class Climb : public frc2::SubsystemBase
{
    public:

        static Climb*        GetInstance();

        void     SetVoltage(units::volt_t voltage);

    private:

        Climb(); 

        void ConfigureClimbMotor(int motorCanId);

        static Climb*            m_climb;

        hardware::TalonFX       m_climbMotor;

        hardware::DigitalInput  m_climbLimit;
        hardware::DigitalInput  m_captureLimit;
};
