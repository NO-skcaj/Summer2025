#pragma once

#include <numbers>
#include <cmath>
#include "string.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include "TalonFX.h"
#include "SparkMax.h"

#include "Constants/Controller.h"
#include "Constants/CanIds.h"

namespace Constants
{

namespace Swerve
{
    // Define the absolute encoder value for forward
    constexpr auto FrontRightForwardAngle          = -0.193604 * 2.0_rad * std::numbers::pi;
    constexpr auto FrontLeftForwardAngle           = -0.422119 * 2.0_rad * std::numbers::pi;
    constexpr auto RearRightForwardAngle           = -0.174561 * 2.0_rad * std::numbers::pi;
    constexpr auto RearLeftForwardAngle            =  0.268555 * 2.0_rad * std::numbers::pi;

    // Drive motor parameters
    constexpr auto DriveMaximumAmperage            = 60_A;
    constexpr auto DriveMotorReduction             = 6.75;
    constexpr auto WheelDiameter                   = 0.098022_m;
    constexpr auto WheelCircumference              = WheelDiameter * std::numbers::pi;
    constexpr auto DriveMotorConversion            = WheelCircumference / DriveMotorReduction;

    constexpr auto DriveP                          = 0.10;
    constexpr auto DriveI                          = 0.02;
    constexpr auto DriveD                          = 0.00;
    constexpr auto DriveV                          = 0.10;
    constexpr auto DriveA                          = 0.10;

    // Angle motor parameters
    constexpr auto AngleMaximumAmperage            = 20;
    constexpr auto AngleMotorRevolutions           = 21.5;  // The number of motor revolutions per wheel revolutions
    constexpr auto AngleRadiansToMotorRevolutions  = (2.0 * std::numbers::pi) / AngleMotorRevolutions;  // Radians to motor revolutions	

    constexpr auto AngleP                          = 1.00;
    constexpr auto AngleI                          = 0.00;
    constexpr auto AngleD                          = 0.20;
}

}

class SwerveModule
{
    public:

        explicit                   SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId);

        void                       SetDesiredState(frc::SwerveModuleState& state, std::string description);  // Sets the desired state for the module

        frc::SwerveModuleState     GetState();                                            // Returns the current state of the module

        frc::SwerveModulePosition  GetPosition();                                         // Returns the current position of the module

        void                       ResetDriveEncoder();                                   // Zeroes all the  encoders

        void                       SetWheelAngleToForward(units::angle::radian_t desiredAngle);

    private:

        void                       ConfigureDriveMotor();
        void                       ConfigureAngleMotor();

        units::angle::radian_t     GetAbsoluteEncoderAngle();
        units::meters_per_second_t GetDriveEncoderRate();

        hardware::TalonFX                  m_driveMotor;
        hardware::SparkMax                 m_angleMotor;
        ctre::phoenix6::hardware::CANcoder m_angleAbsoluteEncoder;
};
