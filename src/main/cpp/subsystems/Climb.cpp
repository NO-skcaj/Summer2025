#include "subsystems/Climb.h"


using namespace Constants::CanIds;

Climb* Climb::GetInstance()
{
    static Climb climb;
    return &climb;
}

/// @brief Class to support the Climb subsystem.
Climb::Climb() : m_climbMotor    {ClimbMotorCanId},
                 m_climbLimit    {Constants::Climb::ClimbLimitSwtich},
                 m_captureLimit  {Constants::Climb::CaptureLimitSwitch}
{
    // Configure the climb motor
    ConfigureClimbMotor(ClimbMotorCanId);
}

/// @brief Method to configure the climb motor using MotionMagic.
/// @param motorCanId The CAN identifier for the climb motor.
void Climb::ConfigureClimbMotor(int motorCanId)
{
    // Create the climb motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration climbMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = climbMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    m_climbMotor.ConfigureMotor(climbMotorConfiguration);
}

/// @brief Method to set the climb angle.
/// @param position The setpoint for the climb angle.
void Climb::SetVoltage(units::volt_t voltage)
{
    // Determine the motor diretion (up or down)
    if ((voltage > 0_V && Constants::Climb::ClimbLimitSwtich   == 1) ||
        (voltage < 0_V && Constants::Climb::CaptureLimitSwitch == 1))
    {
        // Stop the motor
        m_climbMotor.SetSpeed(0_V);
        return;
    }
    // Set the motor voltage
    m_climbMotor.SetSpeed(voltage);
}
