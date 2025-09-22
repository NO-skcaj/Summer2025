#include "Controller.h"


Controller* Controller::GetInstance()
{
    static Controller controller;
    return &controller;
}

/// @brief Constructor for the DriveController class.
/// @param gripper reference to the gripper subsystem.
Controller::Controller() 
    : m_driveController   {Constants::Controller::DriverControllerUsbPort},
      m_operatorController{Constants::Controller::JoystickOperatorUsbPort},
      
      m_xspeedLimiter     {1 / 1_s},
      m_yspeedLimiter     {1 / 1_s},
      m_rotLimiter        {1 / 1_s}
{
    // Configure the operator controller
    std::pair<int, frc2::CommandPtr> controls[] = {
        {Constants::Extreme3D::Handle12,         OffsetElevator(true)},
        {Constants::Extreme3D::Handle11,         OffsetElevator(false)},
        {Constants::Extreme3D::Handle10,         OffsetArm(true)},
        {Constants::Extreme3D::Handle9,          OffsetArm(false)},
        {Constants::Extreme3D::Handle8,          OffsetWrist(true)},
        {Constants::Extreme3D::Handle7,          OffsetWrist(false)},
        {Constants::Extreme3D::HandleSide,       AlignToNearestTag()},
        {Constants::Extreme3D::HandleTrigger,    frc2::CommandPtr(GripperActivate())},
        {Constants::Extreme3D::HandleLowerRight, ChassisZeroHeading()},
        {Constants::Extreme3D::HandleLowerLeft,  FlipFieldCentricity()}
    };

    for (auto& [button, command] : controls)
    {
        frc2::JoystickButton (&m_driveController, button)
            .WhileTrue(std::move(command));
    }
}

std::function<frc::ChassisSpeeds()> Controller::GetChassisSpeedsGetter()
{
    return [this]() -> frc::ChassisSpeeds
    {
        // Use exponential function to calculate the forward value for better slow speed control
        auto joystickForward = GetExponentialValue(
            GetThrottleRange() * frc::ApplyDeadband(m_driveController.GetLeftY(), Constants::Controller::JoystickDeadZone), Constants::Controller::ExponentForward);

        auto joystickStrafe = GetExponentialValue(
            GetThrottleRange() * frc::ApplyDeadband(m_driveController.GetLeftX(),  Constants::Controller::JoystickDeadZone), Constants::Controller::ExponentStrafe);

        auto joystickAngle = GetExponentialValue(
            GetThrottleRange() * frc::ApplyDeadband(m_driveController.GetRightX(), Constants::Controller::JoystickRotateDeadZone), Constants::Controller::ExponentAngle);

        // Return the x speed
        return {Constants::Drivetrain::MaxSpeed        * -m_xspeedLimiter.Calculate(joystickForward),
                Constants::Drivetrain::MaxSpeed        * -m_yspeedLimiter.Calculate(joystickStrafe),
                Constants::Drivetrain::MaxAngularSpeed *  m_rotLimiter.   Calculate(joystickAngle)
        };
    };
}

/// @brief Method to convert the throttle range to a value between ThrottleMinimum and 1.0.
/// @return The throttle value.
double Controller::GetThrottleRange()
{
    auto throttle = -m_driveController.GetRawAxis(Constants::Controller::JoystickThrottleIndex);

    // Convert the throttle value from -1.0 to 1.0 to 0.0 to 1.0
    throttle = (throttle + 1.0) / 2.0; // START IT IN THE MIDDLE

    // Return the throttle value
    return throttle;
}

/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double Controller::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output = 1.0;

    // Return the output value
    return output;
}

/// @brief Method to get the voltage for the gripper wheels based on the potentiometer value.
void Controller::ConfigureOperator()
{
    // Configure the operator control panel controls
    std::pair<int, frc2::CommandPtr> Controls[] =
    {
        {Constants::ControlPanel::OperatorWheels, SetGripperWheels(std::function<GripperWheelState()>{ [this] {return GetWheelInput();} })},
        {Constants::ControlPanel::ElevatorUp,     OffsetElevator(true)},
        {Constants::ControlPanel::ElevatorDown,   OffsetElevator(false)},
        {Constants::ControlPanel::ClimbUp,        ClimbSetVoltage(true)},
        {Constants::ControlPanel::ClimbDown,      ClimbSetVoltage(false) }
    };

    for (auto& [button, function] : Controls)
    {
        frc2::JoystickButton (&m_operatorController, button)
            .WhileTrue(std::move(function));
    }

    // Configure the operator control panel scoring/intaking positioning
    std::pair<int, GripperPoseEnum> GripperPoses[] =
    {
        {Constants::ControlPanel::Home,           GripperPoseEnum::Home},
        {Constants::ControlPanel::CoralGnd,       GripperPoseEnum::CoralGround},
        {Constants::ControlPanel::CoralStn,       GripperPoseEnum::CoralStation},
        {Constants::ControlPanel::CoralL1,        GripperPoseEnum::CoralL1},
        {Constants::ControlPanel::CoralL2,        GripperPoseEnum::CoralL2},
        {Constants::ControlPanel::CoralL3,        GripperPoseEnum::CoralL3},
        {Constants::ControlPanel::CoralL4,        GripperPoseEnum::CoralL4},
        {Constants::ControlPanel::AlgaeGnd,       GripperPoseEnum::AlgaeGround},
        {Constants::ControlPanel::AlgaeCoral,     GripperPoseEnum::AlgaeOnCoral},
        {Constants::ControlPanel::AlgaeLow,       GripperPoseEnum::AlgaeLow},
        {Constants::ControlPanel::AlgaeHigh,      GripperPoseEnum::AlgaeHigh},
        {Constants::ControlPanel::AlgaeProcessor, GripperPoseEnum::AlgaeProcessor},
        {Constants::ControlPanel::AlgaeBarge,     GripperPoseEnum::AlgaeBarge}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (auto& [button, pose] : GripperPoses)
    {
        frc2::JoystickButton (&m_operatorController, button)
            .OnTrue(GripperPose(pose).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}

/// @brief Method to get the potentiometer wheel voltage.
/// @return The potentiometer wheel voltage.
GripperWheelState Controller::GetWheelInput()
{
    // Read the wheel voltage potentiometer
    auto potentiometer = -(m_operatorController.GetRawAxis(Constants::ControlPanel::GripperMotor) - Constants::Gripper::MeanAnalogInput);

    // Apply a deadband to the potentiometer
    if (potentiometer < Constants::Gripper::GripperWheelDeadZone && potentiometer > -Constants::Gripper::GripperWheelDeadZone)
        potentiometer = 0.0;

    // Return the gripper wheel state
    return {!m_operatorController.GetRawButton(Constants::ControlPanel::BothWheelsActive), // check for both wheels
            units::voltage::volt_t{potentiometer * Constants::Gripper::AnalogConversion}}; // convert to voltage
}