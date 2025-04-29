#include "subsystems/Climb.h"


using namespace Constants::CanIds;

/// @brief Class to support the Climb subsystem.
Climb::Climb() : m_climbMotor    {ClimbMotorCanId},
                 m_climbLimit    {Constants::Climb::ClimbLimitSwtich},
                 m_captureLimit  {Constants::Climb::CaptureLimitSwitch},

                 // Logging
                 m_loggingManager{LoggingManager::GetInstance()},

                 m_loggedClimbTargetVoltage{0.0}
{
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Climb Target Voltage", &m_loggedClimbTargetVoltage));

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
    if ((voltage > 0_V && m_climbLimit   == 1) ||
        (voltage < 0_V && m_captureLimit == 1))
    {
        // Stop the motor
        m_climbMotor.SetSpeed(0_V);
        return;
    }

    // Log the target climb voltage

    // Set the motor voltage
    m_climbMotor.SetSpeed(voltage);
}
