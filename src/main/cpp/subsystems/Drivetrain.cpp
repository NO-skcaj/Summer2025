#include "subsystems/Drivetrain.h"

using namespace Constants::CanIds;
using namespace pathplanner;

/// @brief The Constructor for the Drivetrain class.
Drivetrain::Drivetrain()
    : m_gyro      {},
      m_frontLeft {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId },
      m_frontRight{SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId},
      m_rearLeft  {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId  },
      m_rearRight {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId },
      m_vision    {},
      m_estimator  {m_kinematics, GetRotation2d(),
                  {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                   m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, frc::Pose2d{}}
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    // 

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = RobotConfig::fromGUISettings();
    
    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ Drive(speeds.vx, speeds.vy, speeds.omega, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
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
}

/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    frc::SmartDashboard::PutNumber("Gyro Rotation", (double) GetRotation2d().Degrees());

    // Update the swerve drive odometry
    m_estimator.Update(GetRotation2d(),
                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_rearLeft.GetPosition(),  m_rearRight.GetPosition()});

    AddVisionMeasurements(); // resets position to coord system origin blue
}

/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param ySpeed The speed in the Y dirction.
/// @param rotation The rate of rotation.
void Drivetrain::Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation)
{
    // Determine the swerve module states
    auto states = m_kinematics.ToSwerveModuleStates(m_fieldCentricity ?
                  frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, GetRotation2d()) :
                  frc::ChassisSpeeds{xSpeed, ySpeed, rotation});

    // Set the module states
    SetModuleStates(states);
}

/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param ySpeed The speed in the Y dirction.
/// @param rotation The rate of rotation.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation,
                       bool                        fieldCentric)
{
    // Determine the swerve module states
    auto states = m_kinematics.ToSwerveModuleStates(fieldCentric ?
                  frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, GetRotation2d()) :
                  frc::ChassisSpeeds{xSpeed, ySpeed, rotation});

    // Set the module states
    SetModuleStates(states);
}

/// @brief X MODE MAXIMUM DEFENCE 100% SAFE NO ROBOTS GETTING THROUGH HERE, CHAT
void Drivetrain::SetX()
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
void Drivetrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    // Normalize the wheel speeds if any individual speed is above the specified maximum
    m_kinematics.DesaturateWheelSpeeds(&desiredStates, Constants::Drivetrain::MaxSpeed);

    auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

    // Set the swerve module states
    m_frontLeft. SetDesiredState(frontLeft,  "Front Left " );
    m_frontRight.SetDesiredState(frontRight, "Front Right ");
    m_rearLeft.  SetDesiredState(rearLeft,   "Rear Left "  );
    m_rearRight. SetDesiredState(rearRight,  "Rear Right " );
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
    // searches through a list of all tags and returns the closest one
    auto nearestTag = *std::min_element(Constants::Vision::AprilTagLocations::tagsSpan.begin(), Constants::Vision::AprilTagLocations::tagsSpan.end(),
                            [this](frc::Pose3d a, frc::Pose3d b) {
                                return GetPose().Translation().Distance(a.Translation().ToTranslation2d()) < 
                                       GetPose().Translation().Distance(b.Translation().ToTranslation2d());
                            });

    return nearestTag.ToPose2d();
}

/// @brief Method to get the relative chassis speeds.
/// @return The robot relative speeds.
frc::ChassisSpeeds Drivetrain::GetRobotRelativeSpeeds()
{
    // Return the robot relative speeds
    return m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(),
                                       m_rearLeft.GetState(),  m_rearRight.GetState()});

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
        auto estStdDevs = m_vision.GetEstimationStdDevs(estPose).array();

        // Add the vision measurement to the odometry
        m_estimator.AddVisionMeasurement(estPose, visionEst.value().timestamp,
                                         {estStdDevs[0], estStdDevs[1], estStdDevs[2]});
    }
}

/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_frontLeft. SetWheelAngleToForward(Constants::Swerve::FrontLeftForwardAngle);
    m_frontRight.SetWheelAngleToForward(Constants::Swerve::FrontRightForwardAngle);
    m_rearLeft.  SetWheelAngleToForward(Constants::Swerve::RearLeftForwardAngle);
    m_rearRight. SetWheelAngleToForward(Constants::Swerve::RearRightForwardAngle);
}

/// @brief Method to get the kinematics of the robot.
/// @return the kinematics of the robot given the swerve module order and position.
frc::SwerveDriveKinematics<4> Drivetrain::GetKinematics() 
{ 
    return this->m_kinematics; 
}