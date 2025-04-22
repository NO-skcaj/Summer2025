#pragma once

#include <pathplanner/lib/util/swerve/SwerveSetpoint.h>
#include <pathplanner/lib/util/swerve/SwerveSetpointGenerator.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <frc/DriverStation.h>

#include <hal/FRCUsageReporting.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/AnalogPotentiometer.h>

#include <frc/filter/SlewRateLimiter.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc2/command/SubsystemBase.h>

#include <numbers>

#include "subsystems/Vision.h"

#include "lib/modules/SwerveModule.h"
#include "lib/hardware/Navx.h"

#include "Constants/Controller.h"
#include "Constants/CanIds.h"


namespace Constants
{

namespace Drivetrain
{
    // Chassis configuration
    constexpr auto TrackWidth               = 25_in;  // Distance between centers of right and left wheels on robot
    constexpr auto WheelBase                = 25_in;  // Distance between centers of front and back wheels on robot

    constexpr auto MaxSpeed                 = 4.8_mps;
    constexpr auto MaxAngularSpeed          = std::numbers::pi * 2_rad_per_s;
}

namespace ChassisPose
{
    constexpr auto   MaxSpeed               = 2_mps;
    constexpr auto   MaxAcceleration        = 2_mps_sq;
    constexpr auto   MaxAngularSpeed        = 3.142_rad_per_s;
    constexpr auto   MaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double PXController           = 4.0;
    constexpr double PYController           = 4.0;
    constexpr double PProfileController     = 5.0;
}

}

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit                            Drivetrain();
                 
        void                                Periodic() override;
           
        void                                Drive(units::meters_per_second_t  xSpeed,
                                                  units::meters_per_second_t  ySpeed,
                                                  units::radians_per_second_t rotation,
                                                  std::optional<bool>         fieldCentric);
                  
        void                                BECOMEDEFENSE();                                 // Sets the wheels into an X formation to prevent movement
                  
        void                                ResetDriveEncoders();                   // Resets the drive encoders to currently read a position of 0
                  
        void                                SetModuleStates(std::vector<frc::SwerveModuleState> desiredStates);
                  
        frc::Rotation2d                     GetRotation2d();                        // Returns the rotation of the robot
        units::degree_t                     GetHeading();                           // Returns the heading of the robot
                  
        void                                ZeroHeading();                          // Zeroes the heading of the robot
        void                                ZeroHeadingReverse();                   // Zeroes the heading and reverses the gyro (180 degrees)
        void                                ReverseHeading();                       // Reverses the gyro (180 degress)
                  
        frc::Pose2d                         GetPose();                              // Returns the currently-estimated pose of the robot
                  
        void                                ResetPositionToOrgin();                 // Resets the odometry to the orgin
        void                                ResetPose(frc::Pose2d pose);            // Resets the odometry to the specified pose
                  
        void                                SetFieldCentricity(bool fieldCentric);  // Sets the field centricity
        bool                                GetFieldCentricity();                   // Reads the field centricity
                  
        void                                SetWheelAnglesToZero();                 // Sets the wheels to forward based on the absolute encoder
                  
        frc::Pose2d                         GetNearestTag();
       
        frc::ChassisSpeeds                  GetRobotRelativeSpeeds();               // Reads the robot relative speeds
       
        frc::SwerveDriveKinematics<4>       GetKinematics();                        // Returns the kinematics of the robot

        std::vector<frc::SwerveModuleState> GetSwerveModuleStates();                // Returns the swerve module states



        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |

    private:

        void AddVisionMeasurements();

        Vision                               m_vision;
    
        hardware::Navx                       m_gyro;  // The gyro sensor
    
        bool                                 m_fieldCentricity;                      // Field centricity flag
    
        SwerveModule                         m_frontLeft;
        SwerveModule                         m_frontRight;
        SwerveModule                         m_rearLeft;
        SwerveModule                         m_rearRight;

        pathplanner::SwerveSetpointGenerator m_setpointGenerator;
        pathplanner::SwerveSetpoint          m_previousSetpoint;

        pathplanner::RobotConfig             m_config;

        // Odometry class for tracking robot pose for the swerve modules modules
        frc::SwerveDrivePoseEstimator<4>     m_estimator;

        frc::SwerveDriveKinematics<4>        m_kinematics{
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Front Left
            frc::Translation2d{ Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2},   // Front Right
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2,  Constants::Drivetrain::TrackWidth / 2},   // Rear Left
            frc::Translation2d{-Constants::Drivetrain::WheelBase / 2, -Constants::Drivetrain::TrackWidth / 2}};  // Rear Right
};
