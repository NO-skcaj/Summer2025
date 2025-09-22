#include "subsystems/Gripper.h"


using namespace Constants::CanIds;

Gripper* Gripper::GetInstance()
{
    static Gripper gripper;
    return &gripper;
}

/// @brief The Constructor for the Gripper class.
Gripper::Gripper() : m_elevatorMotor    {ElevatorMotorCanId},
                     m_armMotor         {ArmMotorCanId},
                     m_wristMotor       {WristMotorCanId, true},
                     m_gripperMotorFixed{GripperMotorCanIdFixed, true},
                     m_gripperMotorFree {GripperMotorCanIdFree,   true},

                     // Logging
                     m_loggingManager            {LoggingManager::GetInstance()},
                    //  m_loggedElevatorHeightTarget{0.0, "Elevator Height Target"},
                     m_loggedElevatorHeight      {0.0},
                     m_loggedElevatorHeightTarget{0.0},
                     m_loggedArmAngle            {0.0},
                     m_loggedArmAngleTarget      {0.0},
                     m_loggedWristRotation       {0.0},
                     m_loggedWristRotationTarget {0.0},
                     m_loggedWristRotationOffset {0.0},
                     m_loggedGripperVoltage      {0.0},
                     m_loggedGripperVoltageTarget{0.0}
{ 
    // Add logs
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Elevator Height",        &m_loggedElevatorHeight      ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Elevator Height Target", &m_loggedElevatorHeightTarget));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Arm Angle",              &m_loggedArmAngle            ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Arm Angle Target",       &m_loggedArmAngleTarget      ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Wrist Rotation",         &m_loggedWristRotation       ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Wrist Rotation Target",  &m_loggedWristRotationTarget ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Wrist Rotation Offset",  &m_loggedWristRotationOffset ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Gripper Wheels Voltage", &m_loggedGripperVoltage      ));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Gripper Voltage Target", &m_loggedGripperVoltageTarget));

    // Configure the elevator motor
    ConfigureElevatorMotor();

    // Configure the Arm motor
    ConfigureArmMotor();

    // Configure the wrist motor
    ConfigureWristMotor();

    // Configure the gripper wheel motors
    ConfigureGripperMotors();

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(GripperWheelState{true, 0_V});
}

/// @brief Method to update the logged values periodically.
void Gripper::UpdateLoggedValues()
{
    m_loggedElevatorHeight = GetElevatorHeight().value();
    m_loggedArmAngle       = GetArmAngle().value();
    m_loggedWristRotation  = GetWristAngle().value();
    m_loggedGripperVoltage = GetGripperWheelsVoltage().value();
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
    hardware::SparkMaxConfiguration config = 
    {
        hardware::MotorConfiguration::NeutralMode::Brake,
        units::ampere_t{Constants::Wrist::MaximumAmperage},
        true,
        Constants::Wrist::P, 
        Constants::Wrist::I, 
        Constants::Wrist::D,
        false,
        true,
        0,
        2 * std::numbers::pi
    };

    // Write the configuration to the motor controller
    m_wristMotor.ConfigureMotor(config);

    // Start the control at zero degrees
    SetWristAngle(0_deg);
}

/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotors()
{
    hardware::SparkMaxConfiguration config =
    {
        
        hardware::MotorConfiguration::Brake,
        units::ampere_t{Constants::Wrist::MaximumAmperage},
        true,
        0.0,
        0.0,
        0.0,
        false,
        false,
        0,
        0
    };

    // Write the configuration to the motor controller
    m_gripperMotorFixed.ConfigureMotor(config);
    m_gripperMotorFree.ConfigureMotor(config);
}

/// @brief Method to set the pose of the gripper.
/// @param pose The pose to set the gripper.
void Gripper::SetPose(GripperPoseEnum pose)
{

    GripperPoseState gripperPoseState{};

    // Determine the pose
    switch (pose)
    {
        case GripperPoseEnum::Home:
            gripperPoseState = Constants::GripperPose::Coral::HomeState;
            break;

        case GripperPoseEnum::CoralGround:
            gripperPoseState = Constants::GripperPose::Coral::GroundState;
            break;

        case GripperPoseEnum::CoralStation:
            gripperPoseState = Constants::GripperPose::Coral::StationState;
            break;

        case GripperPoseEnum::CoralAutonomousL1:
            gripperPoseState = Constants::GripperPose::Coral::AutonomousL1State;
            break;

        case GripperPoseEnum::CoralL1:
            gripperPoseState = Constants::GripperPose::Coral::L1State;
            break;

        case GripperPoseEnum::CoralL2:
            gripperPoseState = Constants::GripperPose::Coral::L2State;
            break;

        case GripperPoseEnum::CoralL3:
            gripperPoseState = Constants::GripperPose::Coral::L3State;
            break;

        case GripperPoseEnum::CoralL4:
            gripperPoseState = Constants::GripperPose::Coral::L4State;
            break;

        case GripperPoseEnum::AlgaeGround:
            gripperPoseState = Constants::GripperPose::Algae::GroundState;
            break;

        case GripperPoseEnum::AlgaeOnCoral:
            gripperPoseState = Constants::GripperPose::Algae::OnCoralState;
            break;

        case GripperPoseEnum::AlgaeLow:
            gripperPoseState = Constants::GripperPose::Algae::LowState;
            break;

        case GripperPoseEnum::AlgaeHigh:
            gripperPoseState = Constants::GripperPose::Algae::HighState;
            break;

        case GripperPoseEnum::AlgaeProcessor:
            gripperPoseState = Constants::GripperPose::Algae::ProcessorState;
            break;

        case GripperPoseEnum::AlgaeBarge:
            gripperPoseState = Constants::GripperPose::Algae::BargeState;
            break;
    }

    // Remember the pose
    m_gripperPose = pose;

    // Set the elevator height
    SetElevatorHeight(gripperPoseState.ElevatorHeight);

    // Set the arm angle
    SetArmAngle(gripperPoseState.ArmAngle);

    // Set the wrist angle
    SetWristAngle(gripperPoseState.WristAngle);

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(GripperWheelState{gripperPoseState.GripperBothWheels, gripperPoseState.GripperVoltage});
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

    // Log target elevator height
    // m_loggedElevatorHeightTarget.Set(position.value());

    // Compute the number of turns based on the specficied position
    units::angle::turn_t newPosition = (units::angle::turn_t) (position.value() * Constants::Elevator::PositionToTurnsConversionFactor);

    // Set the elevator set position
    m_elevatorMotor.SetPosition(newPosition);
}

/// @brief Moves the elevator by the given offset
/// @param offset The Given offset
void Gripper::AddElevatorOffset(units::length::meter_t offset)
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

    // Log the target arm angle
    m_loggedArmAngleTarget = angle.value();

    // Compute the number of turns based on the specficied angle
    units::angle::turn_t position = (units::angle::turn_t) (angle.value() / Constants::Arm::AngleToTurnsConversionFactor.value());

    // Set the arm set position
    m_armMotor.SetPosition(position);
}

/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::AddArmAngleOffset(units::angle::degree_t angleOffset)
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

    // Log the target wrist angle and offset
    m_loggedWristRotationTarget = angle.value();
    m_loggedWristRotationOffset = m_wristAngleOffset.value();
	
    // Converting angle to motor rotations
    units::turn_t position{(angle.value() + m_wristAngleOffset.value()) / Constants::Wrist::AngleToTurnsConversionFactor.value()};

    // Set the Wrist set position
    m_wristMotor.SetPosition(position); // this may need to get changed as to use kMAXMotionMagicControl
}

/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::AddWristAngleOffset(units::angle::degree_t angleOffset)
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
    // Log the target gripper wheels voltage
    m_loggedGripperVoltageTarget = gripperWheelState.voltage.value();

    // Remember the gripper voltage
    m_gripperVoltage = gripperWheelState.voltage;

    // Set the voltage of the fixed gripper wheel
    m_gripperMotorFixed.SetSpeed(m_gripperVoltage);

    // Set the voltage of the free gripper wheel
    if (gripperWheelState.bothWheels)
        m_gripperMotorFree.SetSpeed(m_gripperVoltage);
    else
        m_gripperMotorFree.SetSpeed(0_V);
}

/// @brief Method to set the Gripper wheels voltage.
/// @param voltage The setpoint for the Gripper wheels voltage.
void Gripper::SetGripperWheelsVoltage(std::function<GripperWheelState()> gripperWheelState)
{
    // Set the voltage of the Gripper wheels
    SetGripperWheelsVoltage(gripperWheelState());
}

GripperActivationData Gripper::GetActivationState()
{
    switch (this->GetState())
    {
        case GripperPoseEnum::CoralL1:
        case GripperPoseEnum::CoralAutonomousL1:
            return Constants::GripperPoseActivate::Coral1;
            break;

        case GripperPoseEnum::CoralL2:
        case GripperPoseEnum::CoralL3:
            return Constants::GripperPoseActivate::Coral23;
            break;

        case GripperPoseEnum::CoralL4:
            return Constants::GripperPoseActivate::Coral4;
            break;

        case GripperPoseEnum::AlgaeProcessor:
            return Constants::GripperPoseActivate::AlgaeProcessor;
            break;

        case GripperPoseEnum::AlgaeBarge:
            return Constants::GripperPoseActivate::AlgaeBarge;
            break;

        default:
            break;
    }
    return GripperActivationData{};
}

/// @brief Method to get the Gripper wheels voltage.
/// @return The Gripper wheels voltage.
units::voltage::volt_t Gripper::GetGripperWheelsVoltage()
{
    return m_gripperVoltage;
}
