#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = NULL;

/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == NULL)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}

/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer() : m_setSwerveWheelAnglesToZero{ChassisSetSwerveWheelAnglesToZero::ChassisSetSwerveWheelAnglesToZero(&m_drivetrain)},
                                   m_driverController  {Constants::Controller::DriverControllerUsbPort},
                                   m_operatorController{Constants::Controller::JoystickOperatorUsbPort}

{
    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // frc::SmartDashboard::PutData("Chassis: AprilTag ",      ChassisDriveToAprilTag([this] { return GetChassisDriveToAprilTagParameters(); }, &m_drivetrain));

    // frc::SmartDashboard::PutData("Elevator Jog Up",          frc2::InstantCommand([this] { m_gripper.SetElevatorOffset( Constants::Elevator::HeightOffset); }));
    // frc::SmartDashboard::PutData("Elevator Jog Down",        frc2::InstantCommand([this] { m_gripper.SetElevatorOffset(-Constants::Elevator::HeightOffset); }));

    // frc::SmartDashboard::PutData("Arm Jog Positive",         frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset( Constants::Arm::AngleOffset);}));
    // frc::SmartDashboard::PutData("Arm Jog Negative",         frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset(-Constants::Arm::AngleOffset);}));

    // frc::SmartDashboard::PutData("Wrist Jog Positive",       frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset( Constants::Wrist::AngleOffset);}));
    // frc::SmartDashboard::PutData("Wrist Jog Negative",       frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset(-Constants::Wrist::AngleOffset);}));

    // Build an auto chooser. This will use frc2::cmd::None() as the default option.
    m_autoChooser = AutoBuilder::buildAutoChooser();

    // m_autonomousChooser.AddOption("Place Coral L1",          AutonomousOneCoral(GripperPoseEnum::CoralAutonomousL1,
    //                                                                 [this] { return GetAutonomousOneCoralParameters(-5_in, 0_in); },
    //                                                                 &m_drivetrain, &m_gripper));

    // m_autonomousChooser.AddOption("Place Coral L4",          AutonomousOneCoral(GripperPoseEnum::CoralL4,
    //                                                                 [this] { return GetAutonomousOneCoralParameters(2_in, 4_in);  },
    //                                                                 &m_drivetrain, &m_gripper));

    // m_autonomousChooser.AddOption("Place Coral L1 AprilTag", AutonomousOneCoralAprilTag(GripperPoseEnum::CoralAutonomousL1,
    //                                                                 [this] { return GetStartPosition();                           },
    //                                                                 [this] { return GetAutonomousOneCoralAprilTagParameters();    },
    //                                                                 [this] { return GetChassisDriveToAprilTagParameters();        },
    //                                                                 &m_drivetrain, &m_gripper));
    // m_autonomousChooser.AddOption("Place Coral L4 AprilTag", AutonomousOneCoralAprilTag(GripperPoseEnum::CoralL4,
    //                                                                 [this] { return GetStartPosition();                           },
    //                                                                 [this] { return GetAutonomousOneCoralAprilTagParameters();    },
    //                                                                 [this] { return GetChassisDriveToAprilTagParameters();        },
    //                                                                 &m_drivetrain, &m_gripper));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autoChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive::ChassisDrive([this] { return Forward(); },
                                                              [this] { return Strafe();  },
                                                              [this] { return Angle();   },
                                                              &m_drivetrain));
     
    // Set the LED default command
    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));

    // Set the swerve wheels to zero
    ChassisSetSwerveWheelAnglesToZero::ChassisSetSwerveWheelAnglesToZero(&m_drivetrain).Unwrap()->Schedule();

    // Start capturing video from the USB camera
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    m_server.SetSource(m_usbCamera);

    // Set the resolution and frame rate of the camera
    camera.SetResolution(640, 480); // Set resolution to 640x480
    camera.SetFPS(30);             // Set frame rate to 30 FPS
}

/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Configure the driver controls
    ConfigureDriverControls();

    // Configure the operator controls
    ConfigureCoralPoseControls();
    ConfigureAlgaePoseControls();
    ConfigureGripperControls();
    ConfigureClimberControls();
}

/// @brief Method to bind the driver joystick controls to the robot commands.
void RobotContainer::ConfigureDriverControls()
{
    // Drive to position using the AprilTag
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleSide)
        .WhileTrue(AlignToNearestTag::AlignToNearestTag(&m_drivetrain));

    // Use the trigger to activate the operation (Scores/Intakes Algae/Coral)
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleTrigger)
        .WhileTrue(GripperActivate(&m_gripper).ToPtr());

    // Reset the gyro angle
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleUpperLeft)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain.ZeroHeading(); }, {&m_drivetrain}).ToPtr());

    // Reset the gyro angle
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleUpperRight)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain.ZeroHeadingReverse(); }, {&m_drivetrain}).ToPtr());

    // Set field centricity on
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleLowerLeft)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(true); }, {&m_drivetrain}).ToPtr());

    // Set field centricity off
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::HandleLowerRight)
        .OnTrue(frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(false); }, {&m_drivetrain}).ToPtr());

    // Toggle X mode
    frc2::JoystickButton (&m_driverController, frc::XboxController::Button::kX)
        .WhileTrue(frc2::RunCommand([this] { m_drivetrain.BECOMEDEFENSE(); }, {&m_drivetrain}).ToPtr());

    // *** Jog Controls
    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle12)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetElevatorOffset(Constants::Elevator::HeightOffset);}).ToPtr());

    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle11)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-Constants::Elevator::HeightOffset);}).ToPtr());

    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle10)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetArmAngleOffset(-Constants::Arm::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle9)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetArmAngleOffset(Constants::Arm::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle8)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetWristAngleOffset(Constants::Wrist::AngleOffset);}).ToPtr());

    frc2::JoystickButton (&m_driverController, Constants::Extreme3D::Handle7)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetWristAngleOffset(-Constants::Wrist::AngleOffset);}).ToPtr());
}

/// @brief Method to bind the operator control panel gripper controls.
void RobotContainer::ConfigureGripperControls()
{
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::OperatorWheels)
        .WhileTrue(frc2::RunCommand([this] { m_gripper
        .SetGripperWheelsVoltage([this] { return PotentiometerWheelVoltage(); }); }, {&m_gripper}).ToPtr());

    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::Home)
        .OnTrue(GripperPose::GripperPose(GripperPoseEnum::Home, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Manually offsets elevator upwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ElevatorUp)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetElevatorOffset(Constants::Elevator::HeightOffset);}).ToPtr());

    // Manually offsets elevator downwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ElevatorDown)
        .WhileTrue(frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-Constants::Elevator::HeightOffset);}).ToPtr());
}

/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: L1Score then activate).
void RobotContainer::ConfigureCoralPoseControls()
{
    // Define an array of button mappings for coral poses
    struct CoralPoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    CoralPoseMapping coralPoses[] =
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
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose::GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}

/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: AlgaeBarge then activate).
void RobotContainer::ConfigureAlgaePoseControls()
{
    // Define an array of button mappings for algae poses
    struct AlgaePoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    AlgaePoseMapping algaePoses[] =
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
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose::GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}

/// @brief Method to bind the operator control panel climb controls.
void RobotContainer::ConfigureClimberControls()
{
    // Manually offsets climb upwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ClimbUp)
        .WhileTrue(frc2::RunCommand([this] { m_climb.SetVoltage(Constants::Climb::ClimbVoltage); }, {&m_climb}).ToPtr())
        .OnFalse(frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}).ToPtr());

    // Manually offsets climb downwards
    frc2::JoystickButton (&m_operatorController, Constants::ControlPanel::ClimbDown)
        .WhileTrue(frc2::RunCommand([this] { m_climb.SetVoltage(-Constants::Climb::ClimbVoltage); }, {&m_climb}).ToPtr())
        .OnFalse(frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}).ToPtr());
}

/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetDriverController()
{
    // Return the pointer to the driver joystick
    return &m_driverController;
}

/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::XboxController *RobotContainer::GetOperatorController()
{
    // Return the pointer to the operator joystick
    return &m_operatorController;
}

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{

    // Get the selected autonomous command from the chooser
    // The selected command will be run in autonomous
    return frc2::CommandPtr(std::unique_ptr<frc2::Command>(m_autoChooser.GetSelected()));
}
/// @brief Method to return the forward joystick value.
/// @return The forward joystick meters per second value.
units::meters_per_second_t RobotContainer::Forward()
{
    // Get the forward joystick setting
    double joystickForward = GetDriverController()->GetRawAxis(Constants::Controller::JoystickForwardIndex);

    // Modify the joystick value by the "throttle" setting
    joystickForward *= GetThrottleRange();

    // Use exponential function to calculate the forward value for better slow speed control
    joystickForward = GetExponentialValue(joystickForward, Constants::Controller::ExponentForward);

    // Return the x speed
    return -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, Constants::Controller::JoystickDeadZone)) * Constants::Drivetrain::MaxSpeed;
}

/// @brief Method to return the strafe joystick value.
/// @return The strafe joystick meters per second value.
units::meters_per_second_t RobotContainer::Strafe()
{
    // Get the strafe joystick setting
    double joystickStrafe = GetDriverController()->GetRawAxis(Constants::Controller::JoystickStrafeIndex);

    // Modify the joystick value by the "throttle" setting
    joystickStrafe *= GetThrottleRange();

    // Use exponential function to calculate the forward value for better slow speed control
    joystickStrafe = GetExponentialValue(joystickStrafe, Constants::Controller::ExponentStrafe);

    // Return the y speed
    return -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrafe, Constants::Controller::JoystickDeadZone)) * Constants::Drivetrain::MaxSpeed;
}

/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
units::radians_per_second_t RobotContainer::Angle()
{
    // Get the angle joystick setting
    double joystickAngle = GetDriverController()->GetRawAxis(Constants::Controller::JoystickAngleIndex);

    // Apply deadband first
    double deadbandedAngle = frc::ApplyDeadband(joystickAngle, Constants::Controller::JoystickRotateDeadZone);


    // Use exponential function to calculate the angle value for better slow speed control
    double exponentialAngle = GetExponentialValue(deadbandedAngle, Constants::Controller::ExponentAngle);

    // Apply smoothing between frames to reduce jerky movement (inline implementation)
    // Smoothing factor: 0.0-1.0 (higher = more smoothing, 0.3 is a good starting point)
    constexpr double kSmoothingFactor   = 0.3;
    static double    previousAngleInput = 0.0; // Static variable persists between function calls

    // Calculate smoothed value using previous output and current input
    double smoothedAngle = kSmoothingFactor * previousAngleInput + (1.0 - kSmoothingFactor) * exponentialAngle;
    previousAngleInput   = smoothedAngle; // Store for next cycle

    // Modify the joystick value by the "throttle" setting
    smoothedAngle *= GetThrottleRange();

    // Return the rotation speed with rate limiter applied
    return units::radians_per_second_t(-m_rotLimiter.Calculate(smoothedAngle) * Constants::Drivetrain::MaxAngularSpeed * 0.5);
}

/// @brief Method to set the swerve wheel angles to zero.
/// @return returns the command to do so
frc2::CommandPtr RobotContainer::SetSwerveWheelAnglesToZero()
{
    return std::move(m_setSwerveWheelAnglesToZero);
}

/// @brief Method to convert the throttle range to a value between ThrottleMinimum and 1.0.
/// @return The throttle value.
double RobotContainer::GetThrottleRange()
{
    auto throttle = -GetDriverController()->GetRawAxis(Constants::Controller::JoystickThrottleIndex);

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
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
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

/// @brief Method to get the potentiometer wheel voltage.
/// @return The potentiometer wheel voltage.
GripperWheelState RobotContainer::PotentiometerWheelVoltage()
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
    GripperWheelState gripperWheelState;
    gripperWheelState.bothWheels = bothWheels;
    gripperWheelState.voltage    = voltage;

    // Return the gripper wheel state
    return gripperWheelState;
}

/// @brief Method to get the chassis Pose.
/// @return The chassis Pose.
frc::Pose2d RobotContainer::GetChassisPose()
{
    // Return the chassis pose
    return m_drivetrain.GetPose();
}

/// @brief Method to reverse the Chassis heading to account for field centric drive with the robot facing the driver.
void RobotContainer::ReverseChassisGryo()
{
    // Reverse the chassis gyro
    m_drivetrain.ReverseHeading();
}

/// @brief Method to return a pointer to the gripper subsystem.
/// @return Pointer to the gripper subsystem.
Gripper *RobotContainer::GetGripper()
{
    // Return the pointer to the gripper
    return &m_gripper;
}

/// @brief Method to return a pointer to the power distribution panel.
frc::PowerDistribution *RobotContainer::GetPowerDistribution()
{
    // Return the pointer to the power distribution panel
    return &m_powerDistribution;
}
