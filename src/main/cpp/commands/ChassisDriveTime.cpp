#include "commands/ChassisDriveTime.h"

#pragma region ChassisDriveTime (constructor)
/// @brief Command to drive the robot the specified time.
/// @param time The time to drive the robot.
/// @param speed The speed to perform the drive.
/// @param drivetrain The Drivetrains subsystem.
ChassisDriveTime::ChassisDriveTime(units::second_t time, units::meters_per_second_t speed, Drivetrain *drivetrain) :
                                   m_time(time), m_speed(speed), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveTime");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void ChassisDriveTime::Initialize()
{
    // Get the start time
    m_startTime = frc::GetTime();
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveTime::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0_mps, 0_rad_per_s);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveTime::IsFinished()
{
    // Determine if the sequence is complete
    if (frc::GetTime() - m_startTime > m_time)
        return true;

    // Still driving
    return false;
}
#pragma endregion

#pragma region End
/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveTime::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);
}
#pragma endregion
