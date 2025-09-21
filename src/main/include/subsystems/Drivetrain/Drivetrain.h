#pragma once

#include <new>
#include <numbers>

#include <frc/DriverStation.h>

#include <hal/FRCUsageReporting.h>

#include <units/angle.h>
#include <units/velocity.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>

#include <networktables/StructArrayTopic.h>

#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc2/command/SubsystemBase.h>

#include "subsystems/Drivetrain/Odometry.h"

#include "lib/modules/SwerveModule.h"
#include "lib/hardware/Navx.h"

#include "lib/logging/LoggingManager.h"
#include "lib/logging/LoggerFactory.h"

#include "Constants/Drivetrain.h"
#include "Constants/CanIds.h"

 
class Drivetrain : public frc2::SubsystemBase
{
    public:

        static Drivetrain*                       GetInstance();
                 
        void                                     Periodic() override;
           
        void                                     Drive(frc::ChassisSpeeds  speeds);
        void                                     Drive(frc::ChassisSpeeds  speeds, bool centricity);
                                    
        void                                     ResetDriveEncoders(); // Resets the drive encoders to currently read a position of 0
                  
        void                                     SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
                  
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions(); // Fix the odometry to a position
                  
        void                                     FlipFieldCentric();       // Sets the field centricity
        bool                                     GetFieldCentricity();        // Reads the field centricity
                  
        void                                     SetWheelAnglesToZero();      // Sets the wheels to forward based on the absolute encoder

        void                                     SimPeriodic();

        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();     // Returns the swerve module states

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
    
        explicit                 Drivetrain();

        static Drivetrain*       m_drivetrain;  // The drivetrain singleton class

        hardware::Navx*          m_gyro;  // The gyro sensor

        frc::Field2d             m_field;

        bool                     m_fieldCentricity;                      // Field centricity flag
    
        SwerveModule             m_frontLeft;
        SwerveModule             m_frontRight;
        SwerveModule             m_rearLeft;
        SwerveModule             m_rearRight;

        // Logging
        double                   m_loggedGyro;
        nt::StructArrayPublisher<frc::SwerveModuleState> m_loggedModuleStatePublisher;
};
