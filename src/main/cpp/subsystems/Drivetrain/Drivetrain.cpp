#include "subsystems/Drivetrain/Drivetrain.h"


Drivetrain* Drivetrain::m_drivetrain = nullptr;

using namespace Constants::CanIds;
using namespace pathplanner;

Drivetrain* Drivetrain::GetInstance()
{
    if (m_drivetrain == nullptr)
    {
        m_drivetrain = new Drivetrain();
    }
    return m_drivetrain;
}

/// @brief The Constructor for the Drivetrain class.
Drivetrain::Drivetrain()
    : m_gyro             {hardware::Navx::GetInstance()},

      m_field            {},
      m_fieldCentricity  {true}, // Default to field centricity
      m_frontLeft        {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId },
      m_frontRight       {SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId},
      m_rearLeft         {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId  },
      m_rearRight        {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId },
      m_config           {RobotConfig::fromGUISettings()},
      
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
        [this] (){ return Odometry::GetInstance()->GetPose(); },               // Robot pose supplier
        [this] (frc::Pose2d pose){Odometry::GetInstance()->ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this] (){ return Odometry::GetInstance()->GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this] (auto speeds, auto feedforwards){ Drive(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        m_config, // The robot configuration
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Logging callback for current robot pose
    PathPlannerLogging::setLogCurrentPoseCallback( [this] (frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.SetRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging::setLogTargetPoseCallback( [this] (frc::Pose2d pose) {
        // Do whatever you want with the pose here
        m_field.GetObject("target pose")->SetPose(pose);
    });

    // Logging callback for the active path, this is sent as a vector of poses
    PathPlannerLogging::setLogActivePathCallback( [this] (std::vector<frc::Pose2d> poses) {
        // Do whatever you want with the pose here
        m_field.GetObject("path")->SetTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(poses, frc::TrajectoryConfig(1_mps, 1_mps_sq)));;
    });
}

/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
}

void Drivetrain::Drive(frc::ChassisSpeeds  speeds)
{
    Drive(speeds, m_fieldCentricity);
}

/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param fieldCentric Boolean to indicate if the robot control should be field centric.
void Drivetrain::Drive(frc::ChassisSpeeds speeds, bool centricity)
{
    // // Set the module states
    SetModuleStates(Odometry::GetInstance()->GetKinematics().ToSwerveModuleStates(
        centricity ? 
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, m_gyro->GetRotation().ToRotation2d()) 
            : speeds));
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
void Drivetrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    // Set the swerve module states
    m_frontLeft. SetDesiredState(desiredStates[0], "Front Left ");
    m_frontRight.SetDesiredState(desiredStates[1], "Front Right ");
    m_rearLeft.  SetDesiredState(desiredStates[2], "Rear Left ");
    m_rearRight. SetDesiredState(desiredStates[3], "Rear Right ");
}
/// @brief Method to zero the robot heading.
void Drivetrain::ZeroHeading()
{
    // Reset the gyro
    m_gyro->ResetYaw();
}

/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::FlipFieldCentric()
{
    // Set the field centric member variable
    m_fieldCentricity = !m_fieldCentricity;
}

/// @brief Method to set the field centricity.
/// @return The field centricity setting.
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}

wpi::array<frc::SwerveModulePosition, 4> Drivetrain::GetModulePositions()
{
    return { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
             m_rearLeft.GetPosition(),  m_rearRight.GetPosition()};
}

wpi::array<frc::SwerveModuleState, 4> Drivetrain::GetSwerveModuleStates()
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
                          m_gyro->GetRotation().ToRotation2d().Degrees().value()};

    return std::span<double>{dataArray, std::size(dataArray)};
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