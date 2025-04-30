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
RobotContainer::RobotContainer() : m_climb     {},
                                   m_drivetrain{},
                                   m_gripper   {},
                                   m_leds      {},

                                   m_setSwerveWheelAnglesToZero{ChassisSetSwerveWheelAnglesToZero::ChassisSetSwerveWheelAnglesToZero(&m_drivetrain)},

                                   m_operatorController        {&m_gripper,    &m_climb},
                                   m_driverController          {&m_drivetrain, &m_gripper},

                                   // Logging       
                                   m_loggingManager            {LoggingManager::GetInstance()},
                                   m_loggedPotentiometer       {0.0}

{
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Drivetrain",    &m_drivetrain));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Potentiometer", &m_loggedPotentiometer));

    // frc::SmartDashboard::PutData("Chassis: AprilTag ",      ChassisDriveToAprilTag([this] { return GetChassisDriveToAprilTagParameters(); }, &m_drivetrain));

    // frc::SmartDashboard::PutData("Elevator Jog Up",          frc2::InstantCommand([this] { m_gripper.SetElevatorOffset( Constants::Elevator::HeightOffset); }));
    // frc::SmartDashboard::PutData("Elevator Jog Down",        frc2::InstantCommand([this] { m_gripper.SetElevatorOffset(-Constants::Elevator::HeightOffset); }));

    // frc::SmartDashboard::PutData("Arm Jog Positive",         frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset( Constants::Arm::AngleOffset);}));
    // frc::SmartDashboard::PutData("Arm Jog Negative",         frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset(-Constants::Arm::AngleOffset);}));

    // frc::SmartDashboard::PutData("Wrist Jog Positive",       frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset( Constants::Wrist::AngleOffset);}));
    // frc::SmartDashboard::PutData("Wrist Jog Negative",       frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset(-Constants::Wrist::AngleOffset);}));

    // Build an auto chooser. This will use frc2::cmd::None() as the default option.
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    // Command to go to the nearest AprilTag, default offset puts the bumpers slightly behind the tag
    pathplanner::NamedCommands::registerCommand("AlignToNearestTag", AlignToNearestTag::AlignToNearestTag(&m_drivetrain));
    
    // Commands to align with, ready the gripper, and score
    pathplanner::NamedCommands::registerCommand("ScoreL4", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL4ReefLeft,  GripperPoseEnum::CoralL4}));
    pathplanner::NamedCommands::registerCommand("ScoreR4", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL4ReefRight, GripperPoseEnum::CoralL4}));

    pathplanner::NamedCommands::registerCommand("ScoreL3", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL23ReefLeft,  GripperPoseEnum::CoralL3}));
    pathplanner::NamedCommands::registerCommand("ScoreR3", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL23ReefRight, GripperPoseEnum::CoralL3}));

    pathplanner::NamedCommands::registerCommand("ScoreL2", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL23ReefLeft,  GripperPoseEnum::CoralL2}));
    pathplanner::NamedCommands::registerCommand("ScoreR2", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL23ReefRight, GripperPoseEnum::CoralL2}));

    pathplanner::NamedCommands::registerCommand("ScoreL1", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL1ReefLeft,  GripperPoseEnum::CoralL1}));
    pathplanner::NamedCommands::registerCommand("ScoreR1", AutoScore::AutoScore(&m_drivetrain, &m_gripper, std::pair<frc::Transform2d, GripperPoseEnum>{Constants::ChassisAprilTagToPose::CoralL1ReefRight, GripperPoseEnum::CoralL1}));

    // Commands to intake


    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autoChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive::ChassisDrive([this] { return m_driverController.GetChassisSpeeds(); },
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

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Get the selected autonomous command from the chooser
    // The selected command will be run in autonomous
    return frc2::CommandPtr(std::unique_ptr<frc2::Command>(m_autoChooser.GetSelected()));
}

/// @brief Method to set the swerve wheel angles to zero.
/// @return returns the command to do so
frc2::CommandPtr RobotContainer::SetSwerveWheelAnglesToZero()
{
    return std::move(m_setSwerveWheelAnglesToZero);
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
