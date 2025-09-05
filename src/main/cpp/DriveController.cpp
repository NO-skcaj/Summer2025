#include "DriveController.h"


/// @brief Constructor for the DriveController class.
/// @param gripper reference to the gripper subsystem.
DriveController::DriveController(Drivetrain* drivetrain, Gripper* gripper) 
    : m_driveController {Constants::Controller::DriverControllerUsbPort},
      
      m_xspeedLimiter   {3 / 1_s},
      m_yspeedLimiter   {3 / 1_s},
      m_rotLimiter      {3 / 1_s},
      
      m_drivetrain      {drivetrain},
      m_gripper         {gripper}
{
    // Configure the operator controller
    Configure();
}

/// @brief Method to get the voltage for the gripper wheels based on the potentiometer value.
void DriveController::Configure()
{
    ConfigureDriverControls();
    ConfigureJogControls();
}

/// @brief Method to bind the driver joystick controls to the robot commands.
void DriveController::ConfigureDriverControls()
{
    // Drive to position using the AprilTag
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleSide)
        .WhileTrue(AlignToNearestTag::AlignToNearestTag(m_drivetrain));

    // Use the trigger to activate the operation (Scores/Intakes Algae/Coral)
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleTrigger)
        .WhileTrue(GripperActivate(m_gripper).ToPtr());

    // Reset the gyro angle
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleUpperLeft)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain->ZeroHeading(); }, {m_drivetrain}).ToPtr());

    // Reset the gyro angle
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleUpperRight)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain->ZeroHeadingReverse(); }, {m_drivetrain}).ToPtr());

    // Set field centricity on
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleLowerLeft)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain->SetFieldCentricity(true); }, {m_drivetrain}).ToPtr());

    // Set field centricity off
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::HandleLowerRight)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain->SetFieldCentricity(false); }, {m_drivetrain}).ToPtr());

    // Toggle X mode
    frc2::JoystickButton (&m_driveController, frc::XboxController::Button::kX)
        .WhileTrue(frc2::RunCommand([this] { m_drivetrain->BECOMEDEFENSE(); }, {m_drivetrain}).ToPtr());

    
}

void DriveController::ConfigureJogControls()
{
    // *** Jog Controls ***
    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle12)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetElevatorOffset(Constants::Elevator::HeightOffset);}).ToPtr());

    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle11)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetElevatorOffset(-Constants::Elevator::HeightOffset);}).ToPtr());

    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle10)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetArmAngleOffset(-Constants::Arm::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle9)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetArmAngleOffset(Constants::Arm::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle8)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetWristAngleOffset(Constants::Wrist::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driveController, Constants::Extreme3D::Handle7)
        .WhileTrue(frc2::RunCommand([this] { m_gripper->SetWristAngleOffset(-Constants::Wrist::AngleOffset);}).ToPtr());
}

frc::ChassisSpeeds DriveController::GetChassisSpeeds()
{
    // Get the forward joystick setting
    double joystickForward = m_driveController.GetRawAxis(Constants::Controller::JoystickForwardIndex);
    double joystickStrafe  = m_driveController.GetRawAxis(Constants::Controller::JoystickStrafeIndex);
    double joystickAngle   = m_driveController.GetRawAxis(Constants::Controller::JoystickAngleIndex);

    // Apply smoothing between frames to reduce jerky movement (inline implementation)
    // Smoothing factor: 0.0-1.0 (higher = more smoothing, 0.3 is a good starting point)
    constexpr double kSmoothingFactor   = 0.3;
    static double    previousAngleInput = 0.0; // Static variable persists between function calls

    // Calculate smoothed value using previous output and current input
    double smoothedAngle = kSmoothingFactor * previousAngleInput + (1.0 - kSmoothingFactor) * joystickAngle;
    previousAngleInput   = smoothedAngle; // Store for next cycle

    // Modify the joystick value by the "throttle" setting
    joystickForward *= GetThrottleRange();
    joystickStrafe  *= GetThrottleRange();
    smoothedAngle   *= GetThrottleRange();

    // Use exponential function to calculate the forward value for better slow speed control
    joystickForward = GetExponentialValue(joystickForward, Constants::Controller::ExponentForward);
    joystickStrafe  = GetExponentialValue(joystickStrafe, Constants::Controller::ExponentStrafe);
    smoothedAngle   = GetExponentialValue(smoothedAngle, Constants::Controller::ExponentStrafe);

    // Return the x speed
    return {Constants::Drivetrain::MaxSpeed 
                * -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, Constants::Controller::JoystickDeadZone)),

            Constants::Drivetrain::MaxSpeed 
                * -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrafe,  Constants::Controller::JoystickDeadZone)),

            Constants::Drivetrain::MaxAngularSpeed * 0.5 
                * -m_rotLimiter.Calculate(frc::ApplyDeadband(smoothedAngle, Constants::Controller::JoystickRotateDeadZone))
    };
}

/// @brief Method to convert the throttle range to a value between ThrottleMinimum and 1.0.
/// @return The throttle value.
double DriveController::GetThrottleRange()
{
    auto throttle = -m_driveController.GetRawAxis(Constants::Controller::JoystickThrottleIndex);

    // Convert the throttle value from -1.0 to 1.0 to 0.0 to 1.0
    throttle = (throttle + 1.0) / 2.0;

    // Set the throttle to the minimum to maximum range
    throttle = (1 - Constants::Controller::ThrottleMinimum) * throttle + Constants::Controller::ThrottleMinimum;

    // Return the throttle value
    return throttle;
}

/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double DriveController::GetExponentialValue(double joystickValue, double exponent)
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