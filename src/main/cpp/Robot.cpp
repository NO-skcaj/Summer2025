#include "Robot.h"

#pragma region RobotInit
/// @brief Method called when the robot class is instantiated.
void Robot::RobotInit()
{
    // Enable LiveWindow in test mode
    EnableLiveWindowInTest(true);

    // Report the robot framework usage
    HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);

    // Reset the debug message
    frc::SmartDashboard::PutString("Debug", "RobotInit");
}
#pragma endregion

#pragma region RobotPeriodic
/// @brief Method is called every robot packet, no matter the mode.
void Robot::RobotPeriodic()
{
    // Run the command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Get the voltage going into the PDP, in Volts
    double voltage = m_robotContainer->GetPowerDistribution()->GetVoltage();
    frc::SmartDashboard::PutNumber("Voltage", voltage);

    // Show the present chassis pose
    frc::Pose2d pressentPose = m_robotContainer->GetChassisPose();
    frc::SmartDashboard::PutNumber("Present X", pressentPose.X().value() * 39.3701);
    frc::SmartDashboard::PutNumber("Present Y", pressentPose.Y().value() * 39.3701);
    frc::SmartDashboard::PutNumber("Present A", pressentPose.Rotation().Degrees().value());

    // Show the control panel Gripper wheels potentiometer value
    frc::SmartDashboard::PutNumber("Wheels Input", m_robotContainer->GetOperatorController()->GetRawAxis(ConstantsControlPanel::GripperMotor));

    // Show the Gripper pose positions
    frc::SmartDashboard::PutNumber("Elevator", m_robotContainer->GetGripper()->GetElevatorHeight().value());
    frc::SmartDashboard::PutNumber("Arm",      m_robotContainer->GetGripper()->GetArmAngle().to<double>());
    frc::SmartDashboard::PutNumber("Wrist",    m_robotContainer->GetGripper()->GetWristAngle().value());
    frc::SmartDashboard::PutNumber("Wheels",   m_robotContainer->GetGripper()->GetGripperWheelsVoltage().value());

    // Read Limelight information to Network Table
    auto targetPose   = LimelightHelpers::getTargetPose_CameraSpace("limelight");
    auto targetPose3d = LimelightHelpers::toPose3D(targetPose);

    frc::SmartDashboard::PutNumber("Limelight X",     targetPose3d.X().to<double>() * 39.3701);
    frc::SmartDashboard::PutNumber("Limelight Y",     targetPose3d.Y().to<double>() * 39.3701);
    frc::SmartDashboard::PutNumber("Limelight Z",     targetPose3d.Z().to<double>() * 39.3701);
    frc::SmartDashboard::PutNumber("Limelight Rot X", targetPose3d.Rotation().X().to<double>() * 180.0 / M_PI);
    frc::SmartDashboard::PutNumber("Limelight Rot Y", targetPose3d.Rotation().Y().to<double>() * 180.0 / M_PI);
    frc::SmartDashboard::PutNumber("Limelight Rot Z", targetPose3d.Rotation().Z().to<double>() * 180.0 / M_PI);

    auto pipelineLatency = LimelightHelpers::getLatency_Pipeline("limelight");
    frc::SmartDashboard::PutNumber("Limelight Pipeline Latency", pipelineLatency);
}
#pragma endregion

#pragma region AutonomousInit
/// @brief Method is called when switching to autonomous mode.
void Robot::AutonomousInit()
{
    // Set the swerve wheels to zero
    m_robotContainer->SetSwerveWheelAnglesToZero();

    // Get the selected autonomous command
    m_autonomousCommand = m_robotContainer->GetAutonomousCommand();

    // Ensure the arm angle is past the elevator
    if (m_robotContainer->GetGripper()->GetArmAngle() < ArmConstants::PastElevatorPosition)
    {
        // Set the arm to the past elevator position
        m_robotContainer->GetGripper()->SetArmAngle(ArmConstants::PastElevatorPosition);
    }

    // Determine if the chooser returned a pointer to a command
    if (m_autonomousCommand != nullptr)
    {
        // Schedule the autonomous command
        m_autonomousCommand->Schedule();
    }
}
#pragma endregion

#pragma region AutonomousPeriodic
/// @brief Method is called periodically when the robot is in autonomous mode.
void Robot::AutonomousPeriodic()
{

}
#pragma endregion

#pragma region TeleopInit
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
    if (m_autonomousCommand != nullptr)
    {
        // Cancel the autonomous command and set the pointer to null
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }

    // Ensure the arm angle is past the elevator
    if (m_robotContainer->GetGripper()->GetArmAngle() < ArmConstants::PastElevatorPosition)
    {
        // Set the arm to the past elevator position
        m_robotContainer->GetGripper()->SetArmAngle(ArmConstants::PastElevatorPosition);
    }
}
#pragma endregion

#pragma region TeleopPeriodic
/// @brief Method is called periodically when the robot is in tleloperated mode.
void Robot::TeleopPeriodic()
{

}
#pragma endregion

#pragma region DisabledInit
/// @brief Method is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{

}
#pragma endregion

#pragma region DisabledPeriodic
/// @brief Method is called periodically when the robot is disabled.
void Robot::DisabledPeriodic()
{

}
#pragma endregion

#pragma region TestInit
/// @brief Method is called when switching to test mode.
void Robot::TestInit()
{

}
#pragma endregion

#pragma region TestPeriodic
// This function is called periodically during test mode.
void Robot::TestPeriodic()
{

}
#pragma endregion

#pragma region SimulationInit
/// @brief Method is called when starting in simulation mode.
void Robot::SimulationInit()
{

}
#pragma endregion

#pragma region SimulationPeriodic
/// @brief Method is called periodically when in simulation mode.
void Robot::SimulationPeriodic()
{

}
#pragma endregion

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
