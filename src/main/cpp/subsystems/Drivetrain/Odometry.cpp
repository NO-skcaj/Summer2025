#include "subsystems/Drivetrain/Odometry.h"


Odometry* Odometry::m_odometry = nullptr;

Odometry* Odometry::GetInstance()
{
    if (m_odometry == nullptr)
    {
        m_odometry = new Odometry();
    }
    return m_odometry;
}

Odometry::Odometry() : m_gyro      {hardware::Navx::GetInstance()},
                       m_vision    {Vision::GetInstance()},
                       m_estimator {m_kinematics, m_gyro->GetRotation().ToRotation2d(),
                           {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()}, 
                           frc::Pose2d{}},
                       m_kinematics{
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Front Left
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2},   // Front Right
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Rear Left
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2}}  // Rear Right
{
    m_estimator.SetVisionMeasurementStdDevs(m_vision->GetEstimationStdDevs(frc::Pose2d{}));
}

void Odometry::Update()
{
    if (!frc::RobotBase::IsSimulation())
    {
        // Add vision measurements to the odometry
        auto visionEst = m_vision->GetEstimatedGlobalPose();

        if (visionEst.has_value()) 
        {
            m_estimator.AddVisionMeasurement(visionEst.value().first, visionEst.value().second, 
                                                m_vision->GetEstimationStdDevs(visionEst.value().first));
        }
    } else
    {
        Drivetrain::GetInstance()->SimPeriodic();
        auto moduleStates = Drivetrain::GetInstance()->GetModuleStates();
        hardware::Navx::GetInstance()->SimUpdate(m_kinematics.ToChassisSpeeds(moduleStates).omega);
    }

    m_estimator.Update(m_gyro->GetRotation().ToRotation2d(), Drivetrain::GetInstance()->GetModulePositions());
}

frc::Pose2d Odometry::GetPose()
{
    return m_estimator.GetEstimatedPosition();
}

/// @brief Method to reset the chassis position to the orgin.
/// @param pose The pose to reset the odometry.
void Odometry::ResetPose(frc::Pose2d pose)
{
    // reset the odometry pose
    m_estimator.ResetPose(pose);

    // Reset the present odometry
    m_estimator.ResetPosition(m_gyro->GetRotation().ToRotation2d(), Drivetrain::GetInstance()->GetModulePositions(), pose);
}

/// @brief Method to get the nearest tag given
/// @return Pose of the nearest tag
frc::Pose2d Odometry::GetNearestTag()
{
    return m_vision->GetNearestTag(frc::Pose3d{GetPose()});
}

/// @brief Method to get the relative chassis speeds.
/// @return The robot relative speeds.
frc::ChassisSpeeds Odometry::GetRobotRelativeSpeeds()
{
    // Return the robot relative speeds
    return m_kinematics.ToChassisSpeeds(Drivetrain::GetInstance()->GetModuleStates());

}

frc::SwerveDriveKinematics<4> Odometry::GetKinematics()
{
    return m_kinematics;
}