#include "subsystems/Gripper.h"

#pragma region Gripper
/// @brief The Constructor for the Gripper class.
Gripper::Gripper() : m_wristMotor(CanConstants::WristMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                     m_wristEncoder(m_wristMotor.GetEncoder()),
                     m_wristTurnClosedLoopController(m_wristMotor.GetClosedLoopController()),

                     m_gripperMotorFixed(CanConstants::GripperMotorCanIdFixed, rev::spark::SparkMax::MotorType::kBrushless),
                     m_gripperMotorFree(CanConstants::GripperMotorCanIdFree,   rev::spark::SparkMax::MotorType::kBrushless)
{
    // Configure the elevator motor
    ConfigureElevatorMotor(CanConstants::ElevatorMotorCanId);

    // Configure the Arm motor
    ConfigureArmMotor(CanConstants::ArmMotorCanId);

    // Configure the wrist motor
    ConfigureWristMotor();

    // Configure the gripper wheel motors
    ConfigureGripperMotorRight();
    ConfigureGripperMotorLeft();

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(GripperWheelState{true, 0_V});
}
#pragma endregion

#pragma region ConfigureElevatorMotor
/// @brief Method to configure the elevator motor using MotionMagic.
/// @param motorCanId The CAN identifier for the elevator motor.
void Gripper::ConfigureElevatorMotor(int motorCanId)
{
    // Instantiate the elevator motor
    m_elevatorMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the elevator motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration elevatorMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = elevatorMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    motorOutputConfigs.Inverted    = true;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = elevatorMotorConfiguration.Slot0;
    slot0Configs.kS = ElevatorConstants::S;
    slot0Configs.kV = ElevatorConstants::V;
    slot0Configs.kA = ElevatorConstants::A;
    slot0Configs.kP = ElevatorConstants::P;
    slot0Configs.kI = ElevatorConstants::I;
    slot0Configs.kD = ElevatorConstants::D;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ElevatorConstants::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ElevatorConstants::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_elevatorMotor->GetConfigurator().Apply(elevatorMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure elevator motor. Error: " << status.GetName() << std::endl;

    // Set the elevator motor control to the default
    SetElevatorHeight(0_m);
}
#pragma endregion

#pragma region ConfigureArmMotor
/// @brief Method to configure the Arm motor using MotionMagic.
/// @param motorCanId The CAN identifier for the Arm motor.
void Gripper::ConfigureArmMotor(int motorCanId)
{
    // Instantiate the Arm motor
    m_armMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the Arm motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration armMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = armMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    motorOutputConfigs.Inverted = true;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = armMotorConfiguration.Slot0;
    slot0Configs.kS = ArmConstants::S;
    slot0Configs.kV = ArmConstants::V;
    slot0Configs.kA = ArmConstants::A;
    slot0Configs.kP = ArmConstants::P;
    slot0Configs.kI = ArmConstants::I;
    slot0Configs.kD = ArmConstants::D;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = armMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ArmConstants::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ArmConstants::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_armMotor->GetConfigurator().Apply(armMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure arm motor. Error: " << status.GetName() << std::endl;

    // Start the control at zero degrees
    SetArmAngle(0_deg);
}
#pragma endregion

#pragma region ConfigureWristMotor
/// @brief Method to configure the Wrist motor using MotionMagic.
void Gripper::ConfigureWristMotor()
{
    // Configure the wrist motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(WristConstants::MaxAmperage);
    // sparkMaxConfig.encoder
    //     .Inverted(false);
    sparkMaxConfig.closedLoop.maxMotion
        .MaxVelocity(WristConstants::MaximumVelocity)
        .MaxAcceleration(WristConstants::MaximumAcceleration)
        .AllowedClosedLoopError(WristConstants::AllowedError);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(WristConstants::P, WristConstants::I, WristConstants::D)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_wristMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    // Start the control at zero degrees
    SetWristAngle(0_deg);
}
#pragma endregion

#pragma region ConfigureGripperMotorRight
/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorRight()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GripperConstants::MaximumAmperage);

    // Write the configuration to the motor controller
    m_gripperMotorFixed.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region ConfigureGripperMotorLeft
/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorLeft()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GripperConstants::MaximumAmperage);

    // Write the configuration to the motor controller
    m_gripperMotorFree.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region SetPose
/// @brief Method to set the pose of the gripper.
/// @param pose The pose to set the gripper.
void Gripper::SetPose(GripperPoseEnum pose)
{
    auto elevatorHeight  = 0_m;
    auto armAngle        = 0_deg;
    auto wristAngle      = 0_deg;
    bool bothwheels      = true;
    auto gripperVoltage  = 0.0_V;

    // Determine the pose
    switch (pose)
    {
        case GripperPoseEnum::Home:
        {
            elevatorHeight  = CoralPoseConstants::HomeElevator;
            armAngle        = CoralPoseConstants::HomeArmAngle;
            wristAngle      = CoralPoseConstants::HomeWristAngle;
            bothwheels      = CoralPoseConstants::HomeGripperBothWheels;
            gripperVoltage  = CoralPoseConstants::HomeGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralGround:
        {
            elevatorHeight  = CoralPoseConstants::GroundElevator;  // TODO: Set elevator a little higher to allow the operator to set the gripper wheel height
            armAngle        = CoralPoseConstants::GroundArmAngle;
            wristAngle      = CoralPoseConstants::GroundWristAngle;
            bothwheels      = CoralPoseConstants::GroundGripperBothWheels;
            gripperVoltage  = CoralPoseConstants::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralStation:
        {
            elevatorHeight  = CoralPoseConstants::StationElevator;
            armAngle        = CoralPoseConstants::StationArmAngle;
            wristAngle      = CoralPoseConstants::StationWristAngle;
            bothwheels      = CoralPoseConstants::StationGripperBothWheels;
            gripperVoltage  = CoralPoseConstants::StationGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL1:
        {
            elevatorHeight  = CoralPoseConstants::L1Elevator;
            armAngle        = CoralPoseConstants::L1ArmAngle;
            wristAngle      = CoralPoseConstants::L1WristAngle;
            bothwheels      = CoralPoseConstants::L1GripperBothWheels;
            gripperVoltage  = CoralPoseConstants::L1GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL2:
        {
            elevatorHeight  = CoralPoseConstants::L2Elevator;
            armAngle        = CoralPoseConstants::L2ArmAngle;
            wristAngle      = CoralPoseConstants::L2WristAngle;
            bothwheels      = CoralPoseConstants::L2GripperBothWheels;
            gripperVoltage  = CoralPoseConstants::L2GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL3:
        {
            elevatorHeight  = CoralPoseConstants::L3Elevator;
            armAngle        = CoralPoseConstants::L3ArmAngle;
            wristAngle      = CoralPoseConstants::L3WristAngle;
            bothwheels      = CoralPoseConstants::L3GripperBothWheels;
            gripperVoltage  = CoralPoseConstants::L3GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL4:
        {
            elevatorHeight  = CoralPoseConstants::L4Elevator;
            armAngle        = CoralPoseConstants::L4ArmAngle;
            wristAngle      = CoralPoseConstants::L4WristAngle;
            bothwheels      = CoralPoseConstants::L4GripperBothWheels;
            gripperVoltage  = CoralPoseConstants::L4GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralAutonomousL1:
        {
            elevatorHeight  = CoralPoseConstants::AutonomousL1Elevator;
            armAngle        = CoralPoseConstants::AutonomousL1ArmAngle;
            wristAngle      = CoralPoseConstants::AutonomousL1WristAngle;
            bothwheels      = CoralPoseConstants::AutonomousL1GripperBothWheels;
            gripperVoltage  = CoralPoseConstants::AutonomousL1GripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeGround:
        {
            elevatorHeight  = AlgaePoseConstants::GroundElevator;
            armAngle        = AlgaePoseConstants::GroundArmAngle;
            wristAngle      = AlgaePoseConstants::GroundWristAngle;
            bothwheels      = AlgaePoseConstants::GroundGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeOnCoral:
        {
            elevatorHeight  = AlgaePoseConstants::OnCoralElevator;
            armAngle        = AlgaePoseConstants::OnCoralArmAngle;
            wristAngle      = AlgaePoseConstants::OnCoralWristAngle;
            bothwheels      = AlgaePoseConstants::OnCoralGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::OnCoralGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeLow:
        {
            elevatorHeight  = AlgaePoseConstants::LowElevator;
            armAngle        = AlgaePoseConstants::LowArmAngle;
            wristAngle      = AlgaePoseConstants::LowWristAngle;
            bothwheels      = AlgaePoseConstants::LowGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::LowGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeHigh:
        {
            elevatorHeight  = AlgaePoseConstants::HighElevator;
            armAngle        = AlgaePoseConstants::HighArmAngle;
            wristAngle      = AlgaePoseConstants::HighWristAngle;
            bothwheels      = AlgaePoseConstants::HighGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::HighGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeProcessor:
        {
            elevatorHeight  = AlgaePoseConstants::ProcessorElevator;
            armAngle        = AlgaePoseConstants::ProcessorArmAngle;
            wristAngle      = AlgaePoseConstants::ProcessorWristAngle;
            bothwheels      = AlgaePoseConstants::ProcessorGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::ProcessorGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeBarge:
        {
            elevatorHeight  = AlgaePoseConstants::BargeElevator;
            armAngle        = AlgaePoseConstants::BargeArmAngle;
            wristAngle      = AlgaePoseConstants::BargeWristAngle;
            bothwheels      = AlgaePoseConstants::BargeGripperBothWheels;
            gripperVoltage  = AlgaePoseConstants::BargeGripperVoltage;
            break;
        }
    }

    // Remember the pose
    m_gripperPose = pose;

    // Set the elevator height
    SetElevatorHeight(elevatorHeight);

    // Set the arm angle
    SetArmAngle(armAngle);

    // Set the wrist angle
    SetWristAngle(wristAngle);

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(GripperWheelState{bothwheels, gripperVoltage});
}
#pragma endregion

#pragma region SetElevatorHeight
/// @brief Method to set the elevator height.
/// @param position The setpoint for the elevator height.
void Gripper::SetElevatorHeight(units::length::meter_t position)
{
    // Limit the elevator height
    if (position < ElevatorConstants::MinimumPosition)
        position = ElevatorConstants::MinimumPosition;
    else if (position > ElevatorConstants::MaximumPosition)
        position = ElevatorConstants::MaximumPosition;

    // Show the target elevator height
    frc::SmartDashboard::PutNumber("Elevator Target", position.value());

    // Compute the number of turns based on the specficied position
    units::angle::turn_t newPosition = (units::angle::turn_t) (position.value() * ElevatorConstants::PositionToTurnsConversionFactor);

    // Set the elevator set position
    m_elevatorMotor->SetControl(m_elevatorMotionMagicVoltage.WithPosition(newPosition).WithSlot(0));
}
#pragma endregion

#pragma region SetElevatorOffset
/// @brief Moves the elevator by the given offset
/// @param offset The Given offset
void Gripper::SetElevatorOffset(units::length::meter_t offset)
{
    // Set the elevator height based on the offset
    SetElevatorHeight(GetElevatorHeight() + offset);
}
#pragma endregion

#pragma region GetElevatorHeight
/// @brief Method to get the elevator height.
/// @return The elevator height.
units::length::meter_t Gripper::GetElevatorHeight()
{
    // Get the current elevator motor position
    auto currentPosition = m_elevatorMotor->GetPosition().GetValueAsDouble();

    // Return the elevator height
    return (units::length::meter_t) (currentPosition / ElevatorConstants::PositionToTurnsConversionFactor);
}
#pragma endregion

#pragma region SetArmAngle
/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngle(units::angle::degree_t angle)
{
    // Making sure that the climb doesn't try to go through the robot
    if (angle < ArmConstants::MinimumPosition)
       angle = ArmConstants::MinimumPosition;

    if (angle > ArmConstants::MaximumPosition)
        angle = ArmConstants::MaximumPosition;

    // Show the target arm angle
    frc::SmartDashboard::PutNumber("Arm Target", angle.value());

    // Compute the number of turns based on the specficied angle
    units::angle::turn_t position = (units::angle::turn_t) (angle.value() / ArmConstants::AngleToTurnsConversionFactor.value());

    // Set the arm set position
    m_armMotor->SetControl(m_motionMagicVoltage.WithPosition(position).WithSlot(0));
}
#pragma endregion

#pragma region SetArmAngleOffset
/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngleOffset(units::angle::degree_t angleOffset)
{
    // Set the arm angle based on the offset
    SetArmAngle(GetArmAngle() + angleOffset);
}
#pragma endregion

#pragma region GetArmAngle
/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetArmAngle()
{
    // Get the current arm motor angle
    auto currentAngle = m_armMotor->GetPosition().GetValueAsDouble();

    // Return the arm angle
    return (units::angle::degree_t) (currentAngle * ArmConstants::AngleToTurnsConversionFactor.value());
}
#pragma endregion

#pragma region SetWristAngle
/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetWristAngle(units::angle::degree_t angle)
{
    // Making sure that the climb doesn't try to go through the robot
    if (angle < WristConstants::MinimumPosition)
       angle = WristConstants::MinimumPosition;

    if (angle > WristConstants::MaximumPosition)
        angle = WristConstants::MaximumPosition;

    // Remember the wrist angle
    m_wristAngle = angle;

    // Show the target wrist angle
    frc::SmartDashboard::PutNumber("Wrist Target", angle.value());
	frc::SmartDashboard::PutNumber("Wrist Offset", m_wristAngleOffset.value());
	
    // Converting angle to motor rotations
    double position = (angle.value() + m_wristAngleOffset.value()) / WristConstants::AngleToTurnsConversionFactor.value();

    // Set the Wrist set position
    m_wristTurnClosedLoopController.SetReference(position , rev::spark::SparkMax::ControlType::kMAXMotionPositionControl);
}
#pragma endregion

#pragma region SetWristAngleOffset
/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetWristAngleOffset(units::angle::degree_t angleOffset)
{
    // Increase the wrist angle offset
    m_wristAngleOffset += angleOffset;

    // Set the wrist angle
    SetWristAngle(GetWristAngle());
}
#pragma endregion

#pragma region GetWristAngle
/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetWristAngle()
{
    // Return the latest set wrist angle
    return units::angle::degree_t{m_wristAngle};
}
#pragma endregion

#pragma region SetGripperWheelsVoltage (GripperWheelState)
/// @brief Method to set the Gripper wheels voltage.
/// @param voltage The setpoint for the Gripper wheels voltage.
void Gripper::SetGripperWheelsVoltage(GripperWheelState gripperWheelState)
{
    // Show the target gripper wheels voltage
    frc::SmartDashboard::PutNumber("Wheels Target", gripperWheelState.voltage.value());

    // Remember the gripper voltage
    m_gripperVoltage = gripperWheelState.voltage;

    // Set the voltage of the fixed gripper wheel
    m_gripperMotorFixed.SetVoltage(m_gripperVoltage);

    // Set the voltage of the free gripper wheel
    if (gripperWheelState.bothWheels)
        m_gripperMotorFree.SetVoltage(m_gripperVoltage);
    else
        m_gripperMotorFree.SetVoltage(0_V);
}
#pragma endregion

#pragma region SetGripperWheelsVoltage (functions GripperWheelState())
/// @brief Method to set the Gripper wheels voltage.
/// @param voltage The setpoint for the Gripper wheels voltage.
void Gripper::SetGripperWheelsVoltage(std::function<GripperWheelState()> gripperWheelState)
{
    // Set the voltage of the Gripper wheels
    SetGripperWheelsVoltage(gripperWheelState());
}
#pragma endregion

#pragma region GetGripperWheelsVoltage
/// @brief Method to get the Gripper wheels voltage.
/// @return The Gripper wheels voltage.
units::voltage::volt_t Gripper::GetGripperWheelsVoltage()
{
    return m_gripperVoltage;
}
#pragma endregion
