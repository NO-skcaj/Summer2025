#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants/Controller.h"
#include "Constants/CanIds.h"

namespace ClimbConstants
{
    constexpr auto ClimbVoltage       = 12_V;

    constexpr auto ClimbLimitSwtich   = 0;
    constexpr auto CaptureLimitSwitch = 1;
}

class Climb : public frc2::SubsystemBase
{
    public:

        explicit Climb();

        void     SetVoltage(units::volt_t voltage);

    private:

        void ConfigureClimbMotor(int motorCanId);

        ctre::phoenix6::hardware::TalonFX *m_climbMotor;

        frc::DigitalInput                  m_climbLimit  {ClimbConstants::ClimbLimitSwtich};
        frc::DigitalInput                  m_captureLimit{ClimbConstants::CaptureLimitSwitch};
};
