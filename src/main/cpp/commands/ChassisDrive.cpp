#include "commands/ChassisDrive.h"

#pragma region ChassisDrive (constructor)
/// @brief Command to support the driver chassis drive command.
/// @param forward The forward driver input.
/// @param strafe The strafe driver input.
/// @param angle The angle driver input.
/// @param m_drivetrain The drive train subsystem.
ChassisDrive::ChassisDrive(std::function<units::meters_per_second_t()>  forward,
                           std::function<units::meters_per_second_t()>  strafe,
                           std::function<units::radians_per_second_t()> angle,
                           std::function<bool()>                        ultraSonicEnabled,
                           Drivetrain *drivetrain) :
                           m_forward{std::move(forward)},
                           m_strafe{std::move(strafe)},
                           m_angle{std::move(angle)},
                           m_ultraSonicEnabled{std::move(ultraSonicEnabled)},
                           m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDrive");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDrive::Execute()
{
    // Perform the chassis drive
    m_drivetrain->DriveUltaSonic(m_forward(), m_strafe(), m_angle(), m_ultraSonicEnabled());
}
#pragma endregion
