#include "subsystems/Drivetrain.h"


using namespace Constants::CanIds;
using namespace pathplanner;

/// @brief The Constructor for the Drivetrain class.
Drivetrain::Drivetrain()
    : m_gyro             {},
      m_vision           {},
      m_field            {},
      m_fieldCentricity  {true}, // Default to field centricity
      m_frontLeft        {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId },
      m_frontRight       {SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId},
      m_rearLeft         {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId  },
      m_rearRight        {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId },
      m_setpointGenerator{m_config, 10_rad_per_s}, // Initialize the setpoint generator with the config and max speed
      m_config           {RobotConfig::fromGUISettings()},
      m_estimator        {m_kinematics, GetRotation2d(),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                           m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, frc::Pose2d{}},

      // Logging
      m_loggingManager   {LoggingManager::GetInstance()},
      m_loggedGyro       {0.0}
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Field", &m_field));
    m_loggingManager->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Gyro Rotation", &m_loggedGyro));

    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this] (){ return GetPose(); },                // Robot pose supplier
        [this] (frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this] (){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this] (auto speeds, auto feedforwards){ Drive(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        m_config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Logging callback for current robot pose
    pathplanner::PathPlannerLogging::setLogCurrentPoseCallback( [this] (frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging::setLogTargetPoseCallback( [this] (frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    pathplanner::PathPlannerLogging::setLogActivePathCallback( [this] (std::vector<frc::Pose2d> poses) {
        // Do whatever you want with the pose here
        m_field.GetObject("path")->SetTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(poses, frc::TrajectoryConfig(1_mps, 1_mps_sq)));;
    });
}

/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    m_loggedGyro = GetRotation2d().Degrees().value(); 

    // Update the swerve drive odometry
    m_estimator.Update(GetRotation2d(),
                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_rearLeft.GetPosition(),  m_rearRight.GetPosition()});

    AddVisionMeasurements(); // resets position to coord system origin blue
}

/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::Drive(frc::ChassisSpeeds  speeds,
                       std::optional<bool> fieldCentric)
{
    m_previousSetpoint = pathplanner::SwerveSetpoint(this->GetRobotRelativeSpeeds(), this->GetSwerveModuleStates(), DriveFeedforwards::zeros(4));

    // Determine the swerve module states
    m_previousSetpoint = m_setpointGenerator.generateSetpoint(m_previousSetpoint,
                                                       fieldCentric.value_or(m_fieldCentricity) ?
                                                            frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetRotation2d()) :
                                                            frc::ChassisSpeeds{speeds},
                                                        0.02_s);

    // Set the module states
    SetModuleStates(m_previousSetpoint.moduleStates);
}

/// @brief X MODE MAXIMUM DEFENCE 100% SAFE NO ROBOTS GETTING THROUGH HERE, CHAT
void Drivetrain::BECOMEDEFENSE()
{
    frc::SwerveModuleState frontLeftState {0_mps, frc::Rotation2d{ 45_deg}};
    frc::SwerveModuleState frontRightState{0_mps, frc::Rotation2d{-45_deg}};
    frc::SwerveModuleState rearLeftState  {0_mps, frc::Rotation2d{-45_deg}};
    frc::SwerveModuleState rearRightState {0_mps, frc::Rotation2d{ 45_deg}};

    m_frontLeft. SetDesiredState(frontLeftState,  "Front Left " );
    m_frontRight.SetDesiredState(frontRightState, "Front Right ");
    m_rearLeft.  SetDesiredState(rearLeftState,   "Rear Left "  );
    m_rearRight. SetDesiredState(rearRightState,  "Rear Right " );
}

/// @brief Method to reset the drive encoders for each swerve module.
void Drivetrain::ResetDriveEncoders()
{
    // Reset the swerve motor encoders
    m_frontLeft. ResetDriveEncoder();
    m_frontRight.ResetDriveEncoder();
    m_rearLeft.  ResetDriveEncoder();
    m_rearRight. ResetDriveEncoder();
}

/// @brief Method to set the swerve drive states.
/// @param desiredStates
void Drivetrain::SetModuleStates(std::vector<frc::SwerveModuleState> desiredStates)
{
    // Set the swerve module states
    m_frontLeft. SetDesiredState(desiredStates[0], "Front Left ");
    m_frontRight.SetDesiredState(desiredStates[1], "Front Right ");
    m_rearLeft.  SetDesiredState(desiredStates[2], "Rear Left ");
    m_rearRight. SetDesiredState(desiredStates[3], "Rear Right ");
}

/// @brief Method to get the robot heading.
/// @return The robot heading.
frc::Rotation2d Drivetrain::GetRotation2d()
{
    // Return the robot rotation
    return m_gyro.GetRotation().ToRotation2d();
}

/// @brief Method to get the robot heading.
/// @return The robot heading.
units::degree_t Drivetrain::GetHeading()
{
    // Return the robot heading
    return m_gyro.GetRotation().ToRotation2d().Degrees();
}

/// @brief Method to zero the robot heading.
void Drivetrain::ZeroHeading()
{
    // Reset the gyro
    m_gyro.ResetYaw();
}

/// @brief Method to zero the robot heading.
void Drivetrain::ZeroHeadingReverse()
{
    // Reset the gyro
    m_gyro.ResetYaw();

    // Reverse the chassis heading
    ReverseHeading();
}

/// @brief Reverse the chassis heading.
void Drivetrain::ReverseHeading()
{
    // Reverse the gyro heading
    m_gyro.SetOffset(frc::Rotation3d{180_deg, 0_deg, 0_deg});
}

/// @brief Method to get the pose of the chassis.
/// @return The chassis pose.
frc::Pose2d Drivetrain::GetPose()
{
    // Return the chassis pose
    return m_estimator.GetEstimatedPosition();
}

/// @brief Method to reset the chassis position to the orgin.
void Drivetrain::ResetPositionToOrgin()
{
    // Create a pose at the origin
    frc::Pose2d poseOrgin = frc::Pose2d();

    // reset the odometry pose
    m_estimator.ResetPose(poseOrgin);

    // Reset the present odometry
    m_estimator.ResetPosition(GetRotation2d(), { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                                m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, poseOrgin);
}

/// @brief Method to reset the chassis position to the orgin.
/// @param pose The pose to reset the odometry.
void Drivetrain::ResetPose(frc::Pose2d pose)
{
    // reset the odometry pose
    m_estimator.ResetPose(pose);

    // Reset the present odometry
    m_estimator.ResetPosition(GetRotation2d(), { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                                m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, pose);
}

/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::SetFieldCentricity(bool fieldCentric)
{
    // Set the field centric member variable
    m_fieldCentricity = fieldCentric;
}

/// @brief Method to set the field centricity.
/// @return The field centricity setting.
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}

/// @brief Method to get the nearest tag given
/// @return Pose of the nearest tag
frc::Pose2d Drivetrain::GetNearestTag()
{
    return m_vision.GetNearestTag(frc::Pose3d{GetPose()}).ToPose2d();
}

/// @brief Method to get the relative chassis speeds.
/// @return The robot relative speeds.
frc::ChassisSpeeds Drivetrain::GetRobotRelativeSpeeds()
{
    // Return the robot relative speeds
    return m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(),
                                       m_rearLeft.GetState(),  m_rearRight.GetState()});

}

std::vector<frc::SwerveModuleState> Drivetrain::GetSwerveModuleStates()
{
    // Return the swerve module states
    return {m_frontLeft.GetState(), m_frontRight.GetState(),
            m_rearLeft.GetState(),  m_rearRight.GetState()};
}

std::span<double> Drivetrain::GetData()
{
    // Create a data array to hold the swerve module states
    double dataArray[] = {m_frontLeft. GetState().angle.Degrees().value(), m_frontLeft. GetState().speed.value(),
                          m_frontRight.GetState().angle.Degrees().value(), m_frontRight.GetState().speed.value(),
                          m_rearLeft.  GetState().angle.Degrees().value(), m_rearLeft.  GetState().speed.value(),
                          m_rearRight. GetState().angle.Degrees().value(), m_rearRight. GetState().speed.value(),
                          GetRotation2d().Degrees().value()};

    return std::span<double>{dataArray, std::size(dataArray)};
}


/// @brief adds vision measurement (if any) to the odometry mesurement
void Drivetrain::AddVisionMeasurements()
{
    // Get the vision estimate
    // This will be a nullopt if no vision measurement is available
    auto visionEst = m_vision.GetEstimatedGlobalPose();

    // Check if the vision estimate is valid
    if (visionEst.has_value()) {
        // Get the vision measurement pose
        auto estPose = visionEst.value().estimatedPose.ToPose2d();

        // Get the vision measurement standard deviations
        auto estStdDevs = m_vision.GetEstimationStdDevs(estPose);

        // Add the vision measurement to the odometry
        m_estimator.AddVisionMeasurement(estPose, visionEst.value().timestamp,
                                         {estStdDevs[0], estStdDevs[1], estStdDevs[2]});
    }
}

/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_frontLeft. SetWheelAngleToForward(Constants::Drivetrain::FrontLeftForwardAngle);
    m_frontRight.SetWheelAngleToForward(Constants::Drivetrain::FrontRightForwardAngle);
    m_rearLeft.  SetWheelAngleToForward(Constants::Drivetrain::RearLeftForwardAngle);
    m_rearRight. SetWheelAngleToForward(Constants::Drivetrain::RearRightForwardAngle);
}

/// @brief Method to get the kinematics of the robot.
/// @return the kinematics of the robot given the swerve module order and position.
frc::SwerveDriveKinematics<4> Drivetrain::GetKinematics() 
{ 
    return this->m_kinematics; 
}