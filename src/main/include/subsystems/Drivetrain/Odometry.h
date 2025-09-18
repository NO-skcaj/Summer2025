#pragma once

#include <frc/geometry/Pose2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/hardware/Navx.h"
#include "subsystems/Drivetrain/Vision.h"
#include "subsystems/Drivetrain/Drivetrain.h"

#include "constants/Drivetrain.h"
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

        static Odometry* m_odometry;

        hardware::Navx*  m_gyro;
        Vision*          m_vision;

        // Odometry class for tracking robot pose for the swerve modules modules
        frc::SwerveDrivePoseEstimator<4>     m_estimator;

        frc::SwerveDriveKinematics<4>        m_kinematics;
};