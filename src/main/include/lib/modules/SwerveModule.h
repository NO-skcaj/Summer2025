#pragma once

#include <numbers>
#include <cmath>
#include "string.h"

#include <frc/RobotBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <rev/SparkMax.h>

#include <rev/config/SparkMaxConfig.h>

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

#include "lib/hardware/CANCoder.h"
#include "lib/hardware/TalonFX.h"

#include "constants/Drivetrain.h"
#include "constants/Controller.h"
#include "constants/CanIds.h"


class SwerveModule
{
    public:

        explicit                   SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId);

        void                       SetDesiredState(frc::SwerveModuleState& state);  // Sets the desired state for the module

        frc::SwerveModuleState     GetState();                                            // Returns the current state of the module

        frc::SwerveModulePosition  GetPosition();                                         // Returns the current position of the module

        void                       ResetDriveEncoder();                                   // Zeroes all the  encoders

        void                       SetWheelAngleToForward(units::angle::radian_t desiredAngle);

        void                       SimPeriodic();

    private:

        void                       ConfigureDriveMotor();
        void                       ConfigureAngleMotor();

        units::angle::radian_t     GetAbsoluteEncoderAngle();

        hardware::TalonFX   m_driveMotor;
        hardware::TalonFX   m_angleMotor;
        hardware::CANCoder  m_angleAbsoluteEncoder;
};
