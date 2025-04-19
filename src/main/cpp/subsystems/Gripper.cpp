#include "subsystems/Gripper.h"

using namespace Constants::CanIds;

/// @brief The Constructor for the Gripper class.
Gripper::Gripper() : m_elevatorMotor(ElevatorMotorCanId),

                     m_armMotor(ArmMotorCanId),

                     m_wristMotor(WristMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                     m_wristEncoder(m_wristMotor.GetEncoder()),
                     m_wristTurnClosedLoopController(m_wristMotor.GetClosedLoopController()),

                     m_gripperMotorFixed(GripperMotorCanIdFixed, rev::spark::SparkMax::MotorType::kBrushless),
                     m_gripperMotorFree(GripperMotorCanIdFree,   rev::spark::SparkMax::MotorType::kBrushless)
{
    // Configure the elevator motor
    ConfigureElevatorMotor();

    // Configure the Arm motor
    ConfigureArmMotor();

    // Configure the wrist motor
    ConfigureWristMotor();

    // Configure the gripper wheel motors
    ConfigureGripperMotorRight();
    ConfigureGripperMotorLeft();

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(GripperWheelState{true, 0_V});
}

/// @brief Method to configure the elevator motor using MotionMagic.
/// @param motorCanId The CAN identifier for the elevator motor.
void Gripper::ConfigureElevatorMotor()
{

    // Create the elevator motor configuration
    hardware::TalonMotorConfiguration elevatorMotorConfiguration
    {
        hardware::TalonMotorConfiguration::NeutralMode::Brake,
        Constants::Elevator::MaximumAmperage,
        true,
        Constants::Elevator::P,
        Constants::Elevator::I,
        Constants::Elevator::D,
        Constants::Elevator::V,
        Constants::Elevator::A,
        Constants::Elevator::MotionMagicCruiseVelocity,
        Constants::Elevator::MotionMagicAcceleration,
        Constants::Elevator::MotionMagicJerk
    };

    // Apply the configuration to the elevator motor
    m_elevatorMotor.ConfigureMotor(elevatorMotorConfiguration);

    // Set the elevator motor control to the default
    SetElevatorHeight(0_m);
}

/// @brief Method to configure the Arm motor using MotionMagic.
/// @param motorCanId The CAN identifier for the Arm motor.
void Gripper::ConfigureArmMotor()
{
    // Create the arm motor configuration
    hardware::TalonMotorConfiguration armMotorConfiguration
    {
        hardware::TalonMotorConfiguration::NeutralMode::Brake,
        Constants::Arm::MaximumAmperage,
        true,
        Constants::Arm::P,
        Constants::Arm::I,
        Constants::Arm::D,
        Constants::Arm::V,
        Constants::Arm::A,
        Constants::Arm::MotionMagicCruiseVelocity,
        Constants::Arm::MotionMagicAcceleration,
        Constants::Arm::MotionMagicJerk
    };

    // Apply the configuration to the arm motor
    m_armMotor.ConfigureMotor(armMotorConfiguration);

    // Start the control at zero degrees
    SetArmAngle(0_deg);
}

/// @brief Method to configure the Wrist motor using MotionMagic.
void Gripper::ConfigureWristMotor()
{
    // Configure the wrist motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(Constants::Wrist::MaximumAmperage);
    sparkMaxConfig.closedLoop.maxMotion
        .MaxVelocity(Constants::Wrist::MaximumVelocity)
        .MaxAcceleration(Constants::Wrist::MaximumAcceleration)
        .AllowedClosedLoopError(Constants::Wrist::AllowedError);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(Constants::Wrist::P, Constants::Wrist::I, Constants::Wrist::D)
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_wristMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    // Start the control at zero degrees
    SetWristAngle(0_deg);
}

/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorRight()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(Constants::Gripper::MaximumAmperage);

    // Write the configuration to the motor controller
    m_gripperMotorFixed.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorLeft()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(Constants::Gripper::MaximumAmperage);

    // Write the configuration to the motor controller
    m_gripperMotorFree.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

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
            elevatorHeight  = Constants::GripperPoseCoral::HomeElevator;
            armAngle        = Constants::GripperPoseCoral::HomeArmAngle;
            wristAngle      = Constants::GripperPoseCoral::HomeWristAngle;
            bothwheels      = Constants::GripperPoseCoral::HomeGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::HomeGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralGround:
        {
            elevatorHeight  = Constants::GripperPoseCoral::GroundElevator;
            armAngle        = Constants::GripperPoseCoral::GroundArmAngle;
            wristAngle      = Constants::GripperPoseCoral::GroundWristAngle;
            bothwheels      = Constants::GripperPoseCoral::GroundGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralStation:
        {
            elevatorHeight  = Constants::GripperPoseCoral::StationElevator;
            armAngle        = Constants::GripperPoseCoral::StationArmAngle;
            wristAngle      = Constants::GripperPoseCoral::StationWristAngle;
            bothwheels      = Constants::GripperPoseCoral::StationGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::StationGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralAutonomousL1:
        {
            elevatorHeight  = Constants::GripperPoseCoral::AutonomousL1Elevator;
            armAngle        = Constants::GripperPoseCoral::AutonomousL1ArmAngle;
            wristAngle      = Constants::GripperPoseCoral::AutonomousL1WristAngle;
            bothwheels      = Constants::GripperPoseCoral::AutonomousL1GripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::AutonomousL1GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL1:
        {
            elevatorHeight  = Constants::GripperPoseCoral::L1Elevator;
            armAngle        = Constants::GripperPoseCoral::L1ArmAngle;
            wristAngle      = Constants::GripperPoseCoral::L1WristAngle;
            bothwheels      = Constants::GripperPoseCoral::L1GripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::L1GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL2:
        {
            elevatorHeight  = Constants::GripperPoseCoral::L2Elevator;
            armAngle        = Constants::GripperPoseCoral::L2ArmAngle;
            wristAngle      = Constants::GripperPoseCoral::L2WristAngle;
            bothwheels      = Constants::GripperPoseCoral::L2GripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::L2GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL3:
        {
            elevatorHeight  = Constants::GripperPoseCoral::L3Elevator;
            armAngle        = Constants::GripperPoseCoral::L3ArmAngle;
            wristAngle      = Constants::GripperPoseCoral::L3WristAngle;
            bothwheels      = Constants::GripperPoseCoral::L3GripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::L3GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL4:
        {
            elevatorHeight  = Constants::GripperPoseCoral::L4Elevator;
            armAngle        = Constants::GripperPoseCoral::L4ArmAngle;
            wristAngle      = Constants::GripperPoseCoral::L4WristAngle;
            bothwheels      = Constants::GripperPoseCoral::L4GripperBothWheels;
            gripperVoltage  = Constants::GripperPoseCoral::L4GripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeGround:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::GroundElevator;
            armAngle        = Constants::GripperPoseAlgae::GroundArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::GroundWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::GroundGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeOnCoral:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::OnCoralElevator;
            armAngle        = Constants::GripperPoseAlgae::OnCoralArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::OnCoralWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::OnCoralGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::OnCoralGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeLow:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::LowElevator;
            armAngle        = Constants::GripperPoseAlgae::LowArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::LowWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::LowGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::LowGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeHigh:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::HighElevator;
            armAngle        = Constants::GripperPoseAlgae::HighArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::HighWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::HighGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::HighGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeProcessor:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::ProcessorElevator;
            armAngle        = Constants::GripperPoseAlgae::ProcessorArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::ProcessorWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::ProcessorGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::ProcessorGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeBarge:
        {
            elevatorHeight  = Constants::GripperPoseAlgae::BargeElevator;
            armAngle        = Constants::GripperPoseAlgae::BargeArmAngle;
            wristAngle      = Constants::GripperPoseAlgae::BargeWristAngle;
            bothwheels      = Constants::GripperPoseAlgae::BargeGripperBothWheels;
            gripperVoltage  = Constants::GripperPoseAlgae::BargeGripperVoltage;
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

/// @brief Method to set the elevator height.
/// @param position The setpoint for the elevator height.
void Gripper::SetElevatorHeight(units::length::meter_t position)
{
    // Limit the elevator height
    if (position < Constants::Elevator::MinimumPosition)
        position = Constants::Elevator::MinimumPosition;
    else if (position > Constants::Elevator::MaximumPosition)
        position = Constants::Elevator::MaximumPosition;

    // Show the target elevator height
    frc::SmartDashboard::PutNumber("Elevator Target", position.value());

    // Compute the number of turns based on the specficied position
    units::angle::turn_t newPosition = (units::angle::turn_t) (position.value() * Constants::Elevator::PositionToTurnsConversionFactor);

    // Set the elevator set position
    m_elevatorMotor.SetPosition(newPosition);
}

/// @brief Moves the elevator by the given offset
/// @param offset The Given offset
void Gripper::SetElevatorOffset(units::length::meter_t offset)
{
    // Set the elevator height based on the offset
    SetElevatorHeight(GetElevatorHeight() + offset);
}

/// @brief Method to get the elevator height.
/// @return The elevator height.
units::length::meter_t Gripper::GetElevatorHeight()
{
    // Return the elevator height
    return (units::length::meter_t) (m_elevatorMotor.GetPosition().value() / Constants::Elevator::PositionToTurnsConversionFactor);
}

/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngle(units::angle::degree_t angle)
{
    // Making sure that the climb doesn't try to go through the robot
    if (angle < Constants::Arm::MinimumPosition)
       angle = Constants::Arm::MinimumPosition;

    if (angle > Constants::Arm::MaximumPosition)
        angle = Constants::Arm::MaximumPosition;

    // Show the target arm angle
    frc::SmartDashboard::PutNumber("Arm Target", angle.value());

    // Compute the number of turns based on the specficied angle
    units::angle::turn_t position = (units::angle::turn_t) (angle.value() / Constants::Arm::AngleToTurnsConversionFactor.value());

    // Set the arm set position
    m_armMotor.SetPosition(position);
}

/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngleOffset(units::angle::degree_t angleOffset)
{
    // Set the arm angle based on the offset
    SetArmAngle(GetArmAngle() + angleOffset);
}

/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetArmAngle()
{
    // Return the arm angle
    return (units::angle::degree_t) (m_armMotor.GetPosition().value() * Constants::Arm::AngleToTurnsConversionFactor.value());
}

/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetWristAngle(units::angle::degree_t angle)
{
    // Making sure that the climb doesn't try to go through the robot
    if (angle < Constants::Wrist::MinimumPosition)
       angle = Constants::Wrist::MinimumPosition;

    if (angle > Constants::Wrist::MaximumPosition)
        angle = Constants::Wrist::MaximumPosition;

    // Remember the wrist angle
    m_wristAngle = angle;

    // Show the target wrist angle
    frc::SmartDashboard::PutNumber("Wrist Target", angle.value());
	frc::SmartDashboard::PutNumber("Wrist Offset", m_wristAngleOffset.value());
	
    // Converting angle to motor rotations
    double position = (angle.value() + m_wristAngleOffset.value()) / Constants::Wrist::AngleToTurnsConversionFactor.value();

    // Set the Wrist set position
    m_wristTurnClosedLoopController.SetReference(position , rev::spark::SparkMax::ControlType::kMAXMotionPositionControl);
}

/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetWristAngleOffset(units::angle::degree_t angleOffset)
{
    // Increase the wrist angle offset
    m_wristAngleOffset += angleOffset;

    // Set the wrist angle
    SetWristAngle(GetWristAngle());
}

/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetWristAngle()
{
    // Return the latest set wrist angle
    return units::angle::degree_t{m_wristAngle};
}

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

/// @brief Method to set the Gripper wheels voltage.
/// @param voltage The setpoint for the Gripper wheels voltage.
void Gripper::SetGripperWheelsVoltage(std::function<GripperWheelState()> gripperWheelState)
{
    // Set the voltage of the Gripper wheels
    SetGripperWheelsVoltage(gripperWheelState());
}

/// @brief Method to get the Gripper wheels voltage.
/// @return The Gripper wheels voltage.
units::voltage::volt_t Gripper::GetGripperWheelsVoltage()
{
    return m_gripperVoltage;
}
