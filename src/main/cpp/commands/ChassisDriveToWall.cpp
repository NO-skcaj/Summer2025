#include "commands/ChassisDriveToWall.h"

#pragma region
/// @brief Command to drive the chassis to the specified distance from the wall.
/// @param speed The speed to move the chassis.
/// @param distanceToWall The distance to move the chassis from the wall.
/// @param timeoutTime The timeout time for the move.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveToWall::ChassisDriveToWall(units::meters_per_second_t speed, units::meter_t distanceToWall,
                                       units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                       m_speed(speed), m_distanceToWall(distanceToWall),
                                       m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToWall");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void ChassisDriveToWall::Initialize()
{
    // Reset the position of the drivetrain to be (X: 0_m, Y: 0_m, Rotation: 0_deg)
    m_drivetrain->ResetPositionToOrgin();

    try
    {
        // Set up config for trajectory
        frc::TrajectoryConfig trajectoryConfig(m_speed, ChassisPoseConstants::MaxAcceleration);

        // Add kinematics to ensure maximum speed is actually obeyed
        trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // Determine how far to move towards the wall
        auto distance = m_drivetrain->GetDistance() - m_distanceToWall;

        // Ensure the new pose requires an X or Y move
        // Note: GenerateTrajectory will throw an exception if the distance X and Y are zero
        if (fabs(distance.value()) < 0.001)
            distance = 0.01_m;

        // Get the robot starting pose
        auto startPose = m_drivetrain->GetPose();

        // Create the trajectory to follow
        frc::Pose2d endPose{startPose.X() + distance,
                            startPose.Y(),
                            startPose.Rotation().Degrees()};

        frc::SmartDashboard::PutNumber("Distance X", distance.value());
        frc::SmartDashboard::PutNumber("Distance Y", 0.0);
        frc::SmartDashboard::PutNumber("Angle",     0.0);

        frc::SmartDashboard::PutNumber("Start X", startPose.X().value());
        frc::SmartDashboard::PutNumber("Start Y", startPose.Y().value());
        frc::SmartDashboard::PutNumber("Start A", startPose.Rotation().Degrees().value());

        frc::SmartDashboard::PutNumber("End X", endPose.X().value());
        frc::SmartDashboard::PutNumber("End Y", endPose.Y().value());
        frc::SmartDashboard::PutNumber("End A", endPose.Rotation().Degrees().value());

        // Create the trajectory to follow
        auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, {}, endPose, trajectoryConfig);

        // Create a profile PID controller
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
void ChassisDriveToWall::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
        m_swerveControllerCommand->Execute();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveToWall::IsFinished()
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
void ChassisDriveToWall::End(bool interrupted)
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