#include "RobotContainer.h"


using namespace pathplanner;

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
RobotContainer::RobotContainer() : m_climb     {Climb::GetInstance()},
                                   m_drivetrain{Drivetrain::GetInstance()},
                                   m_gripper   {Gripper::GetInstance()},
                                   m_odometry  {Odometry::GetInstance()},
                                   m_leds      {Leds::GetInstance()},

                                   m_controller        {},

                                   // Logging       
                                   m_loggingManager            {LoggingManager::GetInstance()},
                                   m_loggedPotentiometer       {0.0}

{
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Drivetrain",    m_drivetrain));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Potentiometer", &m_loggedPotentiometer));    

    // Command to go to the nearest AprilTag, default offset puts the bumpers slightly behind the tag
    NamedCommands::registerCommand("AlignToNearestTag", AlignToNearestTag());
    
    NamedCommands::registerCommand("ScoreL4", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL4ReefLeft,  Constants::GripperPose::GripperPoseEnum::CoralL4)));
    NamedCommands::registerCommand("ScoreR4", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL4ReefRight, Constants::GripperPose::GripperPoseEnum::CoralL4)));

    NamedCommands::registerCommand("ScoreL3", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL23ReefLeft,  Constants::GripperPose::GripperPoseEnum::CoralL3)));
    NamedCommands::registerCommand("ScoreR3", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL23ReefRight, Constants::GripperPose::GripperPoseEnum::CoralL3)));

    NamedCommands::registerCommand("ScoreL2", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL23ReefLeft,  Constants::GripperPose::GripperPoseEnum::CoralL2)));
    NamedCommands::registerCommand("ScoreR2", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL23ReefRight, Constants::GripperPose::GripperPoseEnum::CoralL2)));

    NamedCommands::registerCommand("ScoreL1", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL1ReefLeft,  Constants::GripperPose::GripperPoseEnum::CoralL1)));
    NamedCommands::registerCommand("ScoreR1", std::move(AutoScore(Constants::ChassisAprilTagToPose::CoralL1ReefRight, Constants::GripperPose::GripperPoseEnum::CoralL1)));

    NamedCommands::registerCommand("ScoreBarge", std::move(AutoScore(Constants::ChassisAprilTagToPose::AlgaeBarge, Constants::GripperPose::GripperPoseEnum::AlgaeBarge)));

    NamedCommands::registerCommand("IntakeCoralAuto", std::move(frc2::WaitCommand(6_s).ToPtr()));
    NamedCommands::registerCommand("HighAlgaeIntake", std::move(frc2::WaitCommand(6_s).ToPtr()));
    NamedCommands::registerCommand("LowAlgaeIntake", std::move(frc2::WaitCommand(6_s).ToPtr()));

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autoChooser);

    // Set the default commands for the subsystems
    m_drivetrain->SetDefaultCommand(ChassisDrive(m_controller.GetChassisSpeeds()));
     
    m_leds->SetDefaultCommand(SetLeds(LedMode::Off, 10_s));

    m_climb->SetDefaultCommand(ClimbSetVoltage(true, 0_V));

    m_odometry->SetDefaultCommand(UpdateOdometry());

    // Start capturing video from the USB camera
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    m_server.SetSource(m_usbCamera);

    // Set the resolution and frame rate of the camera
    camera.SetResolution(640, 480);
    camera.SetFPS(30);
}

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // Get the selected autonomous command from the chooser
    // The selected command will be run in autonomous
    return m_autoChooser.GetSelected();
}