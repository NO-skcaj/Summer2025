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
                 m_captureLimit  {Constants::Climb::CaptureLimitSwitch},

                 m_climbMechanism   {20, 20}
{
    // Configure the climb motor
    ConfigureClimbMotor(ClimbMotorCanId);

    m_climbLigament = m_climbMechanism.GetRoot("root", 0, 0)->Append<frc::MechanismLigament2d>("Climb", 15, 90_deg);
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
    if ((voltage > 0_V && m_climbLimit))
    {
        // Stop the motor
        m_climbMotor.SetSpeed(0_V);
        m_climbMotor.OffsetEncoder(m_climbMotor.GetPosition()); // reset the encoder to 0 degrees
        m_climbLigament->SetAngle(-60_deg);
        return;
    }
    else if (voltage < 0_V && m_captureLimit)
    {
        // Stop the motor
        m_climbMotor.SetSpeed(0_V);
        m_climbMotor.OffsetEncoder(m_climbMotor.GetPosition()); // reset the encoder to 0 degrees
        m_climbLigament->SetAngle(10_deg);
        return;
    }

    m_climbLigament->SetAngle((m_climbMotor.GetPosition().value() / 125) * 360_deg); // afaik the motor is geared 125:1
    frc::SmartDashboard::PutData("Climb Mechanism", &m_climbMechanism);

    // Set the motor voltage
    m_climbMotor.SetSpeed(voltage);
}
