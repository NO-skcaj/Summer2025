#include "subsystems/Drivetrain/Drivetrain.h"


Drivetrain* Drivetrain::m_drivetrain = nullptr;

using namespace Constants::CanIds;

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
    : m_gyro                 {hardware::Navx::GetInstance()},

      m_field                {},
      m_fieldCentricity      {true}, // Default to field centricity
      m_frontLeft            {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId },
      m_frontRight           {SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId},
      m_rearLeft             {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId  },
      m_rearRight            {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId },

      // Logging
      m_loggedGyro           {0.0},
      m_loggedModulePublisher{nt::NetworkTableInstance::GetDefault()
                .GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStates").Publish()}
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    LoggingManager::GetInstance()->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Field", &m_field));
    LoggingManager::GetInstance()->AddLoggerFunction(LoggerFactory::CreateLoggedValue("Gyro Rotation", &m_loggedGyro));
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
    auto states = Odometry::GetInstance()->GetKinematics().ToSwerveModuleStates(
        centricity ? 
            frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, m_gyro->GetRotation().ToRotation2d()) 
            : speeds);
    SetModuleStates(states);

    m_loggedModulePublisher.Set(GetModuleStates());
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
    m_frontLeft. SetDesiredState(desiredStates[0]);
    m_frontRight.SetDesiredState(desiredStates[1]);
    m_rearLeft.  SetDesiredState(desiredStates[2]);
    m_rearRight. SetDesiredState(desiredStates[3]);
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
    return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
            m_rearLeft.GetPosition(),  m_rearRight.GetPosition()};
}

wpi::array<frc::SwerveModuleState, 4> Drivetrain::GetModuleStates()
{
    // Return the swerve module states
    return {m_frontLeft.GetState(), m_frontRight.GetState(),
            m_rearLeft.GetState(),  m_rearRight.GetState()};
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

void Drivetrain::SimPeriodic()
{
    m_frontLeft. SimPeriodic();
    m_frontRight.SimPeriodic();
    m_rearLeft.  SimPeriodic();
    m_rearRight. SimPeriodic();
}