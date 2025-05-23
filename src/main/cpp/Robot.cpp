#include "Robot.h"

/// @brief Method called when the robot class is instantiated.
void Robot::RobotInit()
{
    // Enable LiveWindow in test mode
    EnableLiveWindowInTest(true);

    // Report the robot framework usage
    HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);

    // Retrieve Singletons
    m_robotContainer = RobotContainer::GetInstance();
    m_loggingManager = LoggingManager::GetInstance();

    // Debug message
    m_loggedDebug = "RobotInit";

    // Reset the debug message
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Debug", &m_loggedDebug));
}

/// @brief Method is called every robot packet, no matter the mode.
void Robot::RobotPeriodic()
{
    // Run the command scheduler
    frc2::CommandScheduler::GetInstance().Run();
    
    // Show the Gripper pose positions
    m_robotContainer->GetGripper()->UpdateLoggedValues();

    // Log everything
    LoggingManager::GetInstance()->Log();
}

/// @brief Method is called when switching to autonomous mode.
void Robot::AutonomousInit()
{
    // Set the swerve wheels to zero
    m_robotContainer->SetSwerveWheelAnglesToZero();

    // Get the selected autonomous command
    m_autonomousCommand = m_robotContainer->GetAutonomousCommand().Unwrap();

    // Ensure the arm angle is past the elevator
    if (m_robotContainer->GetGripper()->GetArmAngle() < Constants::Arm::PastElevatorPosition)
    {
        // Set the arm to the past elevator position
        m_robotContainer->GetGripper()->SetArmAngle(Constants::Arm::PastElevatorPosition);
    }

    // Determine if the chooser returned a pointer to a command
    if (m_autonomousCommand.get() != nullptr)
    {
        // Schedule the autonomous command
        m_autonomousCommand.get()->Schedule();
    }
}

/// @brief Method is called periodically when the robot is in autonomous mode.
void Robot::AutonomousPeriodic()
{

}

/// @brief Method is called when switching to teleoperated mode.
void Robot::TeleopInit()
{
    // Determine if the gyro has been reversed
    if (m_chassisGyroReversed == false)
    {
        // Indicate that the gyro has been reversed
        m_chassisGyroReversed = true;

        // Reverse the chassis gyro
        m_robotContainer->ReverseChassisGryo();
    }

    // Set the swerve wheels to zero
    m_robotContainer->SetSwerveWheelAnglesToZero();

    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand.get() != nullptr)
    {
        // Cancel the autonomous command and set the pointer to null
        m_autonomousCommand.get()->Cancel();
    }

    // Ensure the arm angle is past the elevator
    if (m_robotContainer->GetGripper()->GetArmAngle() < Constants::Arm::PastElevatorPosition)
    {
        // Set the arm to the past elevator position
        m_robotContainer->GetGripper()->SetArmAngle(Constants::Arm::PastElevatorPosition);
    }
}

/// @brief Method is called periodically when the robot is in tleloperated mode.
void Robot::TeleopPeriodic()
{

}

/// @brief Method is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{

}

/// @brief Method is called periodically when the robot is disabled.
void Robot::DisabledPeriodic()
{

}

/// @brief Method is called when switching to test mode.
void Robot::TestInit()
{

}
// This function is called periodically during test mode.
void Robot::TestPeriodic()
{

}
/// @brief Method is called when starting in simulation mode.
void Robot::SimulationInit()
{

}
/// @brief Method is called periodically when in simulation mode.
void Robot::SimulationPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
