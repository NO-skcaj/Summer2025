#include "lib/modules/SwerveModule.h"

using namespace Constants::CanIds;

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) :
                           m_driveMotor(hardware::TalonFX{driveMotorCanId}),
                           m_angleMotor(hardware::TalonFX{angleMotorCanId}),
                           m_angleAbsoluteEncoder(angleEncoderCanId)

{
    // Configure the drive and angle motors
    ConfigureDriveMotor();
    ConfigureAngleMotor();

    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);
}

/// @brief Method to configure the drive motor.
void SwerveModule::ConfigureDriveMotor()
{
    // Configure the drive motor
    hardware::TalonMotorConfiguration talonFXConfiguration
    {
        hardware::TalonMotorConfiguration::NeutralMode::Brake,
        Constants::Drivetrain::DriveMaximumAmperage,
        true,
        Constants::Drivetrain::DriveP,
        Constants::Drivetrain::DriveI,
        Constants::Drivetrain::DriveD,
        Constants::Drivetrain::DriveV,
        Constants::Drivetrain::DriveA,
        0.0_tps,
        0.0_tr_per_s_sq,
        0.0_tr_per_s_cu
    };

    m_driveMotor.ConfigureMotor(talonFXConfiguration);
}

/// @brief Method to configure the angle motor and encoder.
void SwerveModule::ConfigureAngleMotor()
{
    hardware::TalonMotorConfiguration sparkMaxConfig
    {
        hardware::MotorConfiguration::NeutralMode::Brake,
        units::ampere_t{Constants::Drivetrain::AngleMaximumAmperage},
        true,
        Constants::Drivetrain::AngleP,
        Constants::Drivetrain::AngleI,
        Constants::Drivetrain::AngleD,
        0.0,
        0.0,
        0.0_tps,
        0.0_tr_per_s_sq,
        0.0_tr_per_s_cu
    };

    // Write the configuration to the motor controller
    m_angleMotor.ConfigureMotor(sparkMaxConfig);
}

/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState)
{
    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(GetPosition().angle);
    // desiredState.speed *= (desiredState.angle - GetPosition().angle).Cos();

    // Set the motor speed and angle
    m_driveMotor.SetSpeed(units::turns_per_second_t(desiredState.speed.value() / Constants::Drivetrain::DriveMotorConversion.value()));

    m_angleMotor.SetPosition(units::turn_t(desiredState.angle.Radians().value() * Constants::Drivetrain::AngleRadiansToMotorRevolutions));

}

/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    // Determine the module wheel velocity
    auto driveVelocity = units::meters_per_second_t{m_driveMotor.GetVelocity().value() * Constants::Drivetrain::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleMotor.GetPosition().value() / Constants::Drivetrain::AngleRadiansToMotorRevolutions.value()}; // ????

    // Return the swerve module state
    return {driveVelocity, anglePosition};
}

/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Determine the module wheel position
    auto drivePosition = m_driveMotor.GetPosition().value() * Constants::Drivetrain::DriveMotorConversion;

    auto anglePosition = units::radian_t{m_angleMotor.GetPosition().value() / Constants::Drivetrain::AngleRadiansToMotorRevolutions.value()};

    // Return the swerve module position
    return {drivePosition, anglePosition};
}

// Reset the drive encoder position.
void SwerveModule::ResetDriveEncoder()
{
    m_driveMotor.SetPosition(0_tr);
}

/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param forwardAngle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::radian_t forwardAngle)
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);

    // Set the motor angle encoder position to the forward direction
    m_angleMotor.OffsetEncoder(units::turn_t((GetAbsoluteEncoderAngle().value() - forwardAngle.value()) * Constants::Drivetrain::AngleRadiansToMotorRevolutions.value()));

    // Set the motor angle to the forward direction
    m_angleMotor.SetPosition(0.0_tr);
}

/// @brief Method to read the absolute encode in radians.
/// @return The absolute angle value in radians.
units::angle::radian_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = m_angleAbsoluteEncoder.GetAbsoluteValue().value();

    // To convert to radians
    return encoderValue * 2.0_rad * std::numbers::pi;
}

void SwerveModule::SimPeriodic()
{
    m_driveMotor.SimPeriodic();
    m_angleMotor.SimPeriodic();
}