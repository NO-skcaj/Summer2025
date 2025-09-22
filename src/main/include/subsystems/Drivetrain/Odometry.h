#pragma once

#include <frc/geometry/Pose2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/hardware/Navx.h"
#include "subsystems/drivetrain/Vision.h"
#include "subsystems/drivetrain/drivetrain.h"

#include "constants/drivetrain.h"
#include "constants/Vision.h"


class Odometry : public frc2::SubsystemBase
{
    public:
        static Odometry*              GetInstance();

        void                          Update();                    // Updates the odometry with the latest measurements

        frc::Pose2d                   GetNearestTag();

        frc::Pose2d                   GetPose();                   // Returns the currently-estimated pose of the robot
                  
        void                          ResetPositionToOrgin();      // Resets the odometry to the orgin
        void                          ResetPose(frc::Pose2d pose); // Resets the odometry to the specified pose

        frc::ChassisSpeeds            GetRobotRelativeSpeeds();

        frc::SwerveDriveKinematics<4> GetKinematics();

    private:
        Odometry();

        Vision*          m_vision;

        frc::SwerveDriveKinematics<4>        m_kinematics;

        frc::SwerveDrivePoseEstimator<4>     m_estimator;

        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;
        nt::DoubleArrayPublisher                         m_loggedModulePositionsPublisher;
        nt::StructPublisher<frc::Pose2d>                 m_loggedPosePublisher;
};