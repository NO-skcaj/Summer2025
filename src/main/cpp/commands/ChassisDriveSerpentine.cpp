#include "commands/ChassisDriveSerpentine.h"

#pragma region ChassisDriveSerpentine
/// @brief Constructor for the ChassisDriveSerpentine class.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The time-out time for the command.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveSerpentine::ChassisDriveSerpentine(units::velocity::meters_per_second_t speed, units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                               m_speed(speed), m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
   SetName("ChassisDriveSerpentine");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void ChassisDriveSerpentine::Initialize()
{
    // Reset the position of the drivetrain to be (X: 0_m, Y: 0_m, Rotation: 0_deg)
    m_drivetrain->ResetPositionToOrgin();

    try
    {
        // Set up config for trajectory
        frc::TrajectoryConfig trajectoryConfig(m_speed, ChassisPoseConstants::MaxAcceleration);

        // Add kinematics to ensure maximum speed is actually obeyed
        trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // An example trajectory to follow
        auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
            // Start at the origin facing the +X direction
            frc::Pose2d{0_m, 0_m, 0_deg},
            // Pass through these two interior waypoints, making an 's' curve path
            {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
            // End 3 meters straight ahead of where we started, facing forward
            frc::Pose2d{3_m, 0_m, 0_deg},
            // Pass the config
            trajectoryConfig);

        frc::ProfiledPIDController<units::radians> profiledPIDController{ChassisPoseConstants::PProfileController, 0, 0,
                                                                         ChassisPoseConstants::ThetaControllerConstraints};

        // enable continuous input for the profile PID controller
        profiledPIDController.EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});

        // Create the swerve controller command
        m_swerveControllerCommand = new frc2::SwerveControllerCommand<4>(
            trajectory,
            [this]() { return m_drivetrain->GetPose(); },
            m_drivetrain->m_kinematics,
            frc::PIDController(ChassisPoseConstants::PXController, 0, 0),
            frc::PIDController(ChassisPoseConstants::PYController, 0, 0),
            profiledPIDController,
            [this](auto moduleStates) { m_drivetrain->SetModuleStates(moduleStates); },
            {m_drivetrain}
        );

        // Initialize the swerve controller command
        m_swerveControllerCommand->Initialize();

        // Get the start time
        m_startTime = frc::GetTime();
    }
    catch(const std::exception& exception)
    {
        frc::SmartDashboard::PutString("Debug", exception.what());
    }
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveSerpentine::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
        m_swerveControllerCommand->Execute();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveSerpentine::IsFinished()
{
    // Determine if the time-out time has expired
    if (frc::GetTime() - m_startTime > m_timeoutTime)
        return true;

    // Determine if the swerve controller command is finished
    return m_swerveControllerCommand && m_swerveControllerCommand->IsFinished();
}
#pragma endregion

#pragma region End
/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveSerpentine::End(bool interrupted)
{
    // If the swerve controller command is not nullptr, end the command
    if (m_swerveControllerCommand)
    {
        // End the swerve controller command
        m_swerveControllerCommand->End(interrupted);

        // Delete the swerve controller command and nullify the pointer
        delete m_swerveControllerCommand;
        m_swerveControllerCommand = nullptr;
    }

    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);
}
#pragma endregion