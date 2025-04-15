#include "SwerveModule.h"

using namespace Constants::CanIds;

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) :
                           m_driveMotor(driveMotorCanId, CanBus),
                           m_angleMotor(angleMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                           m_angleAbsoluteEncoder(angleEncoderCanId, CanBus),
                           m_angleEncoder(m_angleMotor.GetEncoder()),
                           m_turnClosedLoopController(m_angleMotor.GetClosedLoopController())

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
    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = Constants::Swerve::DriveMaximumAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = Constants::Swerve::DriveP;
    slot0Configs.kI = Constants::Swerve::DriveI;
    slot0Configs.kD = Constants::Swerve::DriveD;
    slot0Configs.kV = Constants::Swerve::DriveV;
    slot0Configs.kA = Constants::Swerve::DriveA;

    // Add the "Feedback" section settings
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = Constants::Swerve::WheelCircumference.value() / Constants::Swerve::DriveMotorReduction;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < Constants::CanIds::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_driveMotor.GetConfigurator().Apply(talonFXConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
}

/// @brief Method to configure the angle motor and encoder.
void SwerveModule::ConfigureAngleMotor()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .Inverted(true)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(Constants::Swerve::AngleMaximumAmperage);

    sparkMaxConfig.encoder
        .PositionConversionFactor(Constants::Swerve::AngleRadiansToMotorRevolutions)
        .VelocityConversionFactor(Constants::Swerve::AngleRadiansToMotorRevolutions / 60.0);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(Constants::Swerve::AngleP, Constants::Swerve::AngleI, Constants::Swerve::AngleD)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_angleMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState, std::string description)
{
    frc::SmartDashboard::PutNumber(description + "Drive", (double) desiredState.speed);
    frc::SmartDashboard::PutNumber(description + "Angle", (double) desiredState.angle.Degrees().value());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(frc::Rotation2d(units::radian_t{m_angleEncoder.GetPosition()}));

    // Set the motor speed and angle
    m_driveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage((units::angular_velocity::turns_per_second_t)
                           (desiredState.speed.value() / Constants::Swerve::DriveMotorConversion.value())));
    m_turnClosedLoopController.SetReference(desiredState.angle.Radians().value(), rev::spark::SparkMax::ControlType::kPosition);
}

/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    // Determine the module wheel velocity
    auto driveVelocity = units::meters_per_second_t {
        (double) m_driveMotor.GetVelocity().GetValue() * Constants::Swerve::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleEncoder.GetPosition()};

    // Return the swerve module state
    return {driveVelocity, anglePosition};
}

/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Determine the module wheel position
    auto drivePosition = units::meter_t{
        ((double) m_driveMotor.GetPosition().GetValue()) * Constants::Swerve::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleEncoder.GetPosition()};

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
    m_angleMotor.GetEncoder().SetPosition(GetAbsoluteEncoderAngle().value() - forwardAngle.value());

    // Set the motor angle to the forward direction
    m_turnClosedLoopController.SetReference(0.0, rev::spark::SparkMax::ControlType::kPosition);
}

/// @brief Method to read the absolute encode in radians.
/// @return The absolute angle value in radians.
units::angle::radian_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();

    // To convert to radians
    return encoderValue * 2.0_rad * std::numbers::pi;
}

/// @brief Method to retrieve the drive encoder rate (velocity in meters/s).
/// @return The wheel drive encoder rate.
units::meters_per_second_t SwerveModule::GetDriveEncoderRate()
{
    // Get the motor velocity
    double rotationsPerSecond = (double) m_driveMotor.GetVelocity().GetValue();

    // Return the wheel drive encoder rate
    return (units::meters_per_second_t) rotationsPerSecond;
}
