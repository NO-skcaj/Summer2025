#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants/Controller.h"
#include "Constants/CanIds.h"
#include "Constants/GripperPoseCoral.h"
#include "Constants/GripperPoseAlgae.h"

namespace Constants
{

namespace Elevator
{
    constexpr auto P                               = 2.00;             // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.00;             // Integral:        No output for integrated error
    constexpr auto D                               = 0.10;             // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.25;             // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.12;             // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.01;             // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 180_tps;           // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 360_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 3600_tr_per_s_cu; // Jerk

    constexpr auto PositionToTurnsConversionFactor = 64.0 / (0.06378 * 3.0 * std::numbers::pi); // The number of motor rotations per meter

    constexpr auto MaximumAmperage                 = 60_A;

    constexpr auto MinimumPosition                 = 0_m;
    constexpr auto MaximumPosition                 = 1.7_m;

    constexpr auto HeightOffset                    = 0.025_m;
}

namespace Arm
{
    constexpr auto P                               = 2.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.0;              // Integral:        No output for integrated error
    constexpr auto D                               = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.0;              // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.0;              // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.0;              // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 160_tps;          // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 500_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 500_tr_per_s_cu;  // Jerk

    constexpr auto AngleToTurnsConversionFactor    = 360_deg / 36;      // 36 to 1 gear box

    constexpr auto MaximumAmperage                 = 40_A;

    constexpr auto MinimumPosition                 = -20_deg;
    constexpr auto PastElevatorPosition            =  25_deg;
    constexpr auto MaximumPosition                 = 180_deg;

    constexpr auto AngleOffset                     = 5.0_deg;
}

namespace Wrist
{
    constexpr auto P                             = 0.8;              // Proportional
    constexpr auto I                             = 0.0;              // Integral
    constexpr auto D                             = 1.0;              // Differential

    constexpr auto MaximumVelocity               = 2000.0;           // Rotations per minute (RPM)
    constexpr auto MaximumAcceleration           = 1000.0;           // Acceleration
    constexpr auto AllowedError                  =   0.04;           // Allowed error for the pid controller (smaller values are more accurate)

    constexpr auto MaximumAmperage               =  20;
    constexpr auto AngleToTurnsConversionFactor  = 360_deg / 20;     // 20 to 1 gear box

    constexpr auto MinimumPosition               = -10_deg;          // Note: Need to calibrate angle to motor rotations
    constexpr auto MaximumPosition               = 200_deg;

    constexpr auto AngleOffset                   = 1.0_deg;
}

namespace Gripper
{
    constexpr auto MaximumAmperage      =  20;

    constexpr auto MeanAnalogInput      = 0.11811 / 2.0;

    constexpr auto GripperWheelDeadZone = 0.01;
    constexpr auto AnalogConversion     = 75.0;
}

}

enum GripperPoseEnum
{
    Home,

    CoralGround,
    CoralStation,
    CoralL1,
    CoralL2,
    CoralL3,
    CoralL4,
    CoralAutonomousL1,

    AlgaeGround,
    AlgaeOnCoral,
    AlgaeLow,
    AlgaeHigh,
    AlgaeProcessor,
    AlgaeBarge,
};

struct GripperWheelState
{
    bool                   bothWheels = true;
    units::voltage::volt_t voltage    = 0_V;
};

class Gripper : public frc2::SubsystemBase
{
    public:

        explicit               Gripper();

        void                   SetPose(GripperPoseEnum pose);

        void                   SetElevatorHeight(units::length::meter_t position);
        void                   SetElevatorOffset(units::length::meter_t offset);
        units::length::meter_t GetElevatorHeight();

        void                   SetArmAngle(units::angle::degree_t angle);
        void                   SetArmAngleOffset(units::angle::degree_t angleOffset);
        units::angle::degree_t GetArmAngle();

        void                   SetWristAngle(units::angle::degree_t angle);
        void                   SetWristAngleOffset(units::angle::degree_t angleOffset);
        units::angle::degree_t GetWristAngle();

        void                   SetGripperWheelsVoltage(GripperWheelState gripperWheelState);
        void                   SetGripperWheelsVoltage(std::function<GripperWheelState()> gripperWheelState);
        units::voltage::volt_t GetGripperWheelsVoltage();

        GripperPoseEnum        GetPose() { return m_gripperPose; }  // Get the Gripper Pose

    private:

        void ConfigureElevatorMotor(int driveMotorCanId);
        void ConfigureArmMotor(int driveMotorCanId);
        void ConfigureWristMotor();
        void ConfigureGripperMotorRight();
        void ConfigureGripperMotorLeft();

        ctre::phoenix6::hardware::TalonFX           *m_elevatorMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_elevatorMotionMagicVoltage{0_tr};

        ctre::phoenix6::hardware::TalonFX           *m_armMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};

        rev::spark::SparkMax                         m_wristMotor;
        rev::spark::SparkRelativeEncoder             m_wristEncoder;
        rev::spark::SparkClosedLoopController        m_wristTurnClosedLoopController;
        units::angle::degree_t                       m_wristAngle       = 0_deg;
        units::angle::degree_t                       m_wristAngleOffset = 0_deg;

        rev::spark::SparkMax                         m_gripperMotorFixed;
        rev::spark::SparkMax                         m_gripperMotorFree;

        GripperPoseEnum                              m_gripperPose;

        units::voltage::volt_t                       m_gripperVoltage = 0_V;
};
