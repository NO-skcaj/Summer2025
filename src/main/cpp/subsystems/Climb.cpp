#include "subsystems/Climb.h"

using namespace ConstantsCanIds;

#pragma region Climb (constructor)
/// @brief Class to support the Climb subsystem.
Climb::Climb()
{
    // Configure the climb motor
    ConfigureClimbMotor(ClimbMotorCanId);
}
#pragma endregion

#pragma region ConfigureClimbMotor
/// @brief Method to configure the climb motor using MotionMagic.
/// @param motorCanId The CAN identifier for the climb motor.
void Climb::ConfigureClimbMotor(int motorCanId)
{
    // Instantiate the climb motor
    m_climbMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanBus};

    // Create the climb motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration climbMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = climbMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_climbMotor->GetConfigurator().Apply(climbMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure climb motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region SetAngle
/// @brief Method to set the climb angle.
/// @param position The setpoint for the climb angle.
void Climb::SetVoltage(units::volt_t voltage)
{
    // Determine the motor diretion (up or down)
    if ((voltage > 0_V && m_climbLimit.Get()   == 1) ||
        (voltage < 0_V && m_captureLimit.Get() == 1))
    {
        // Stop the motor
        m_climbMotor->SetVoltage(0_V);
        return;
    }

    // Set the motor voltage
    m_climbMotor->SetVoltage(voltage);
}
#pragma endregion
