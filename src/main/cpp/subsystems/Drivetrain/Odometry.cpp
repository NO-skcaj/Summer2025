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
                       m_kinematics{
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Front Left
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2},   // Front Right
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Rear Left
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2}}, // Rear Right
                       m_estimator {m_kinematics, m_gyro->GetRotation().ToRotation2d(),
                           {Drivetrain::GetInstance()->GetModulePositions()}, 
                           frc::Pose2d{}},
                       m_loggedModuleStatePublisher{nt::NetworkTableInstance::GetDefault()
                                   .GetStructArrayTopic<frc::SwerveModuleState>("/Data/SwerveStatesOdo").Publish()},
                       m_loggedModulePositionsPublisher{nt::NetworkTableInstance::GetDefault()
                                   .GetDoubleArrayTopic("/Data/SwervePositionsOdo").Publish()},
                       m_loggedPosePublisher{nt::NetworkTableInstance::GetDefault()
                                   .GetStructTopic<frc::Pose2d>("/Data/OdometryPose").Publish()}
{
    // m_estimator.SetVisionMeasurementStdDevs(m_vision->GetEstimationStdDevs(frc::Pose2d{}));
}

void Odometry::Update()
{
    auto moduleStates    = Drivetrain::GetInstance()->GetModuleStates();
    auto modulePositions = Drivetrain::GetInstance()->GetModulePositions();

    // // Add vision measurements to the odometry
    // auto visionEst = m_vision->GetEstimatedGlobalPose();

    // if (visionEst.has_value()) 
    // {
    //     m_estimator.AddVisionMeasurement(visionEst.value().first, visionEst.value().second, 
    //                                         m_vision->GetEstimationStdDevs(visionEst.value().first));
    // }

    Drivetrain::GetInstance()->SimPeriodic();
    hardware::Navx::GetInstance()->SimPeriodic(m_kinematics.ToChassisSpeeds(moduleStates).omega);

    m_estimator.Update(m_gyro->GetRotation().ToRotation2d(), modulePositions);

    m_loggedModulePositionsPublisher.Set(std::vector<double>{
        modulePositions[0].distance.value(), modulePositions[0].angle.Degrees().value(),
        modulePositions[1].distance.value(), modulePositions[1].angle.Degrees().value(),
        modulePositions[2].distance.value(), modulePositions[2].angle.Degrees().value(),
        modulePositions[3].distance.value(), modulePositions[3].angle.Degrees().value()});
    m_loggedModuleStatePublisher.Set(moduleStates);
    m_loggedPosePublisher.Set(GetPose());

    frc::SmartDashboard::PutNumber("time", frc::Timer::GetFPGATimestamp().value());
}

frc::Pose2d Odometry::GetPose()
{
    return m_estimator.GetPose();
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