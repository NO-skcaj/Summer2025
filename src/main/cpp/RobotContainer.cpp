#include "RobotContainer.h"


/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer* RobotContainer::GetInstance()
{
    static RobotContainer instance;
    return &instance;
}

/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer() : m_climb     {Climb::GetInstance()},
                                   m_drivetrain{Drivetrain::GetInstance()},
                                   m_gripper   {Gripper::GetInstance()},
                                   m_odometry  {Odometry::GetInstance()},
                                   m_leds      {Leds::GetInstance()}

{
    // Set the default commands for the subsystems
    m_drivetrain->SetDefaultCommand(ChassisDrive(Controller::GetInstance()->GetChassisSpeedsGetter()));
     
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

void RobotContainer::ScheduleTeleopCommands()
{
    // Schedule the default commands for the subsystems
    m_drivetrain->GetDefaultCommand()->Schedule();
    m_leds->GetDefaultCommand()->Schedule();
    m_climb->GetDefaultCommand()->Schedule();
    m_odometry->GetDefaultCommand()->Schedule();
}

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command* RobotContainer::GetAutonomousCommand()
{
    // Get the selected autonomous command from the chooser
    // The selected command will be run in autonomous
    return m_autoChooser.GetSelected();
}