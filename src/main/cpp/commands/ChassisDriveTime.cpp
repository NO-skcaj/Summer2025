#include "commands/ChassisDriveTime.h"

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

/// @brief Called just before this Command runs.
void ChassisDriveTime::Initialize()
{
    // Get the field centricity
    m_fieldCentricity = m_drivetrain->GetFieldCentricity();

    // Set the field to robot centric
    m_drivetrain->SetFieldCentricity(false);

    // Get the start time
    m_startTime = frc::GetTime();
}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveTime::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0_mps, 0_rad_per_s);
}

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

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveTime::End(bool interrupted)
{
    // Return the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);

    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);
}
