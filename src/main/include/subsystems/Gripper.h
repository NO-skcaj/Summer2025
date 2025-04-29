#pragma once

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/TalonFX.h"
#include "lib/hardware/SparkMax.h"

#include "lib/logging/LoggingManager.h"
#include "lib/logging/BaseLoggedValue.h"
#include "lib/logging/LoggedValue.h"

#include "constants/CanIds.h"
#include "constants/Gripper.h"
#include "constants/GripperPose.h"
 

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

/// @brief Structure to hold a target Gripper wheel state.
/// @param bothWheels bool
/// @param voltage    units::voltage::volt_t
struct GripperWheelState
{
    bool                   bothWheels = true;
    units::voltage::volt_t voltage    = 0_V;
};

typedef Constants::GripperPose::GripperPoseState GripperPoseState;

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

        void                   UpdateLoggedValues();

    private:

        void ConfigureElevatorMotor();
        void ConfigureArmMotor();
        void ConfigureWristMotor();
        void ConfigureGripperMotors();

        hardware::TalonFX        m_elevatorMotor;
                    
        hardware::TalonFX        m_armMotor;

        hardware::SparkMax       m_wristMotor;
        
        units::angle::degree_t   m_wristAngle       = 0_deg;
        units::angle::degree_t   m_wristAngleOffset = 0_deg;

        hardware::SparkMax       m_gripperMotorFixed;
        hardware::SparkMax       m_gripperMotorFree;

        GripperPoseEnum          m_gripperPose;

        units::voltage::volt_t   m_gripperVoltage = 0_V;

        // Logging
        LoggingManager*          m_loggingManager;

        double  m_loggedElevatorHeight;
        double  m_loggedElevatorHeightTarget;

        double  m_loggedArmAngle;
        double  m_loggedArmAngleTarget;

        double  m_loggedWristRotation;
        double  m_loggedWristRotationTarget;
        double  m_loggedWristRotationOffset;

        double  m_loggedGripperVoltage;
        double  m_loggedGripperVoltageTarget;
};
