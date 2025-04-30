#include "OperatorController.h"

/// @brief Constructor for the OperatorController class.
/// @param gripper reference to the gripper subsystem.
OperatorController::OperatorController(Gripper* gripper, Climb* climb) : m_operatorController {Constants::Controller::JoystickOperatorUsbPort},
                                                                         m_gripper            {gripper},
                                                                         m_climb              {climb}
{
    // Configure the operator controller
    Configure();
}

/// @brief Method to get the voltage for the gripper wheels based on the potentiometer value.
void OperatorController::Configure()
{
    // Configure the operator control panel gripper controls
    ConfigureGripperControls();

    // Configure the operator control panel scoring/intaking positioning
    ConfigureCoralPoseControls();
    ConfigureAlgaePoseControls();

    // Configure the operator control panel climb controls
    ConfigureClimberControls();
}

/// @brief Method to bind the operator control panel gripper controls.
void OperatorController::ConfigureGripperControls()
{
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::OperatorWheels)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetGripperWheelsVoltage([this]() { return GetWheelInput(); }); }, {m_gripper}).ToPtr());

    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::Home)
        .OnTrue(GripperPose::GripperPose(GripperPoseEnum::Home, m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Manually offsets elevator upwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ElevatorUp)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetElevatorOffset(Constants::Elevator::HeightOffset);}).ToPtr());

    // Manually offsets elevator downwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ElevatorDown)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetElevatorOffset(-Constants::Elevator::HeightOffset);}).ToPtr());
}

/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: L1Score then activate).
void OperatorController::ConfigureCoralPoseControls()
{
    std::pair<int, GripperPoseEnum> coralPoses[] =
    {
        {Constants::ControlPanel::CoralGnd, GripperPoseEnum::CoralGround},
        {Constants::ControlPanel::CoralStn, GripperPoseEnum::CoralStation},
        {Constants::ControlPanel::CoralL1,  GripperPoseEnum::CoralL1},
        {Constants::ControlPanel::CoralL2,  GripperPoseEnum::CoralL2},
        {Constants::ControlPanel::CoralL3,  GripperPoseEnum::CoralL3},
        {Constants::ControlPanel::CoralL4,  GripperPoseEnum::CoralL4}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : coralPoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.first)
            .OnTrue(GripperPose::GripperPose(mapping.second, m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}

/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: AlgaeBarge then activate).
void OperatorController::ConfigureAlgaePoseControls()
{
    std::pair<int, GripperPoseEnum> algaePoses[] =
    {
        {Constants::ControlPanel::AlgaeGnd,       GripperPoseEnum::AlgaeGround},
        {Constants::ControlPanel::AlgaeCoral,     GripperPoseEnum::AlgaeOnCoral},
        {Constants::ControlPanel::AlgaeLow,       GripperPoseEnum::AlgaeLow},
        {Constants::ControlPanel::AlgaeHigh,      GripperPoseEnum::AlgaeHigh},
        {Constants::ControlPanel::AlgaeProcessor, GripperPoseEnum::AlgaeProcessor},
        {Constants::ControlPanel::AlgaeBarge,     GripperPoseEnum::AlgaeBarge}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : algaePoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.first)
            .OnTrue(GripperPose::GripperPose(mapping.second, m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}

/// @brief Method to bind the operator control panel climb controls.
void OperatorController::ConfigureClimberControls()
{
    // Manually offsets climb upwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ClimbUp)
        .WhileTrue(frc2::RunCommand(  [this] { m_climb->SetVoltage(Constants::Climb::ClimbVoltage); }, {m_climb}).ToPtr())
        .OnFalse(frc2::InstantCommand([this] { m_climb->SetVoltage(0_V); },                            {m_climb}).ToPtr());

    // Manually offsets climb downwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ClimbDown)
        .WhileTrue(frc2::RunCommand(  [this] { m_climb->SetVoltage(-Constants::Climb::ClimbVoltage); }, {m_climb}).ToPtr())
        .OnFalse(frc2::InstantCommand([this] { m_climb->SetVoltage(0_V); },                             {m_climb}).ToPtr());
}

/// @brief Method to get the potentiometer wheel voltage.
/// @return The potentiometer wheel voltage.
GripperWheelState OperatorController::GetWheelInput()
{
    // Read the wheel voltage potentiometer
    auto potentiometer = -(m_operatorController.GetRawAxis(Constants::ControlPanel::GripperMotor) - Constants::Gripper::MeanAnalogInput);

    // Apply a deadband to the potentiometer
    if (potentiometer < Constants::Gripper::GripperWheelDeadZone && potentiometer > -Constants::Gripper::GripperWheelDeadZone)
        potentiometer = 0.0;

    frc::SmartDashboard::PutNumber("Potentiometer", potentiometer);
 
    // Convert to a voltage
    auto voltage = units::voltage::volt_t{potentiometer * Constants::Gripper::AnalogConversion};

    // Determine if both wheels are active
    bool bothWheels = !m_operatorController.GetRawButton(Constants::ControlPanel::BothWheelsActive);

    // Return the gripper wheel state
    return {bothWheels, voltage};
}