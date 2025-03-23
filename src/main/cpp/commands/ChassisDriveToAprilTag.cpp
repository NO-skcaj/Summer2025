#include "commands/ChassisDriveToAprilTag.h"

#pragma region ChassisDriveToAprilTag
/// @brief Command to drive the chassis to an AprilTag.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The timeout time for the move.
/// @param aprilTags The AprilTag subsystem.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveToAprilTag::ChassisDriveToAprilTag(units::meters_per_second_t speed,
                                               units::meter_t             distanceOffsetX,
                                               units::meter_t             distanceOffsetY,
                                               units::degree_t            angleOffset,
                                               units::time::second_t      timeoutTime,
                                               Drivetrain                *drivetrain) :
                                               m_speed(speed),                     m_distanceOffsetX(distanceOffsetX),
                                               m_distanceOffsetY(distanceOffsetY), m_angleOffset(angleOffset),
                                               m_timeoutTime(timeoutTime),         m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToAprilTag");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Indicate that the parameters should not be read from the lambda function
    m_readParameters = false;
}
#pragma endregion

#pragma region ChassisDriveToAprilTag
/// @brief Construct a ChassisDriveToAprilTag command using a lambda function to get the parameters.
/// @param getParmeters The lambda function to get the parameters.
ChassisDriveToAprilTag::ChassisDriveToAprilTag(std::function<ChassDriveAprilTagParameters()> getParameters, Drivetrain *drivetrain) :
                                               m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToAprilTag");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Indicate that the parameters should be read from the lambda function
    m_readParameters = true;
    m_getParameters  = getParameters;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
/// @details Initializes the command.
/// Swerve module order for kinematics calculations
///
/// Apriltag Coordinates:
///
///    Z - The distance from the camera to the tag.
///    X - The distance from the camera to the tag in the X direction.
///    Y - The distance from the camera to the tag in the Y direction.
///
///             Camera           Translation2d Coordinates
///
///               |                         ^ X
///               |                         |
///         ------+------> X       Y <------+-------
///               |                         |
///               |                         |
///               V Y
///
/// Translation:  Pose X ->  Z
///               Pose Y -> -X
///               Pose A -> -Rotation Y

void ChassisDriveToAprilTag::Initialize()
{
    // Reset the position of the drivetrain to be (X: 0_m, Y: 0_m, Rotation: 0_deg)
    m_drivetrain->ResetPositionToOrgin();

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;

    // Assume the pose command will run
    m_finished = false;

    // Determine if the parameters should be read from the lambda function
    if (m_readParameters)
    {
        // Get the drive to april tag parameters
        auto parameters = m_getParameters();

        // Determine if the pose is valid
        if (parameters.ValidPose == true)
        {
            // Initialize the member variables using an initialization list
            m_speed           = parameters.PoseParameters.Speed;
            m_distanceOffsetX = parameters.PoseParameters.DistanceX;
            m_distanceOffsetY = parameters.PoseParameters.DistanceY;
            m_angleOffset     = parameters.PoseParameters.Angle;
            m_timeoutTime     = parameters.PoseParameters.TimeoutTime;
        }
        else
        {
            frc::SmartDashboard::PutString("Debug", "Not a valid pose");

            // End the command
            m_finished = true;
        }

        // Determine if the pose is not valid (do not continue)
        if (m_finished)
            return;
    }

    try
    {
        // Read Limelight information to Network Table
        auto targetPose   = LimelightHelpers::getTargetPose_CameraSpace("limelight");
        auto targetPose3d = LimelightHelpers::toPose3D(targetPose);

        // Get the AprilTag information
        auto aprilTagX   =  targetPose3d.Z();
        auto aprilTagY   = -targetPose3d.X();
        auto aprilTagRot = -targetPose3d.Rotation().Y();

        frc::SmartDashboard::PutNumber("aprilTagX",   aprilTagX.value());
        frc::SmartDashboard::PutNumber("aprilTagY",   aprilTagY.value());
        frc::SmartDashboard::PutNumber("aprilTagRot", aprilTagRot.value() * 180.0 / M_PI);

        // Determine if an AprilTag was found
        if (aprilTagX.value() <= 0.0)
        {
            frc::SmartDashboard::PutString("Debug", "No AprilTag Found");

            // End the command
            m_finished = true;
            return;
        }

        // Set up config for trajectory
        frc::TrajectoryConfig trajectoryConfig(m_speed, ChassisPoseConstants::MaxAcceleration);

        // Add kinematics to ensure maximum speed is actually obeyed
        trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // Ensure the new pose requires an X or Y move
        // Note: GenerateTrajectory will throw an exception if the distance X and Y are zero
        if (fabs(aprilTagX.value()) < 0.001 && fabs(aprilTagY.value()) < 0.001)
            aprilTagX = 0.01_m;

        // Get the robot starting pose
        auto startPose = m_drivetrain->GetPose();

        // Offset the position based on the specified distances and angle
        auto distanceX   = aprilTagX   - m_distanceOffsetX;
        auto distanceY   = aprilTagY   + m_distanceOffsetY;
        auto angleOffset = aprilTagRot + m_angleOffset;

        // Create the trajectory to follow
        // frc::Pose2d endPose{startPose.X()                  + distanceX,  // Note: The start pose is the orgin so X, Y, and Rotation are all zero (not needed)
        //                     startPose.Y()                  + distanceY,
        //                     startPose.Rotation().Degrees() + angleOffset};

        // Create the trajectory to follow
        // Note: The start pose is the orgin so X, Y, and Rotation are all zero (not needed)
        frc::Pose2d endPose{distanceX, distanceY, angleOffset};

        frc::SmartDashboard::PutNumber("Distance X",  distanceX.value());
        frc::SmartDashboard::PutNumber("Distance Y",  distanceY.value());
        frc::SmartDashboard::PutNumber("Angle",       angleOffset.value() * 180.0 / M_PI);

        frc::SmartDashboard::PutNumber("End X",       endPose.X().value());
        frc::SmartDashboard::PutNumber("End Y",       endPose.Y().value());
        frc::SmartDashboard::PutNumber("End A",       endPose.Rotation().Degrees().value());

        // Create the trajectory to follow
        auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, {}, endPose, trajectoryConfig);

        // Create a profile PID controller
        frc::ProfiledPIDController<units::radians> profiledPIDController{ChassisPoseConstants::PProfileController, 0, 0,
                                                                         ChassisPoseConstants::ThetaControllerConstraints};

        // Enable continuous input for the profile PID controller
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

        frc::SmartDashboard::PutString("Debug", "Move to AprilTag");

        // Initialize the swerve controller command
        m_swerveControllerCommand->Initialize();

        // Get the start time
        m_startTime = frc::GetTime();
    }
    catch(const std::exception& exception)
    {
        frc::SmartDashboard::PutString("Debug", exception.what());

        // Ensure the SwerveControllerCommand is set to nullptr
        m_swerveControllerCommand = nullptr;

        // Determine if the command was not able to start (do not continue)
        if (m_finished)
            return;
    }
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveToAprilTag::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
       m_swerveControllerCommand->Execute();
    else
	   m_finished = true;
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveToAprilTag::IsFinished()
{
    // Determine if the command is finished
    if (m_finished)
        return true;

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
void ChassisDriveToAprilTag::End(bool interrupted)
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