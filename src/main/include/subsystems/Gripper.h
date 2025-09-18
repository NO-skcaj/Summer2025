#pragma once

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/TalonFX.h"
#include "lib/hardware/SparkMax.h"

#include "lib/logging/LoggingManager.h"

#include "lib/logging/LoggerFactory.h"

#include "constants/CanIds.h"
#include "constants/Gripper.h"
#include "constants/GripperPose.h"
#include "Constants/GripperPoseActivate.h"


using namespace Constants::GripperPose;
using namespace Constants::GripperPoseActivate;

class Gripper : public frc2::SubsystemBase
{
    public:
        static Gripper*        GetInstance();

        void                   SetPose(GripperPoseEnum pose);

        void                   SetElevatorHeight(units::length::meter_t position);
        void                   AddElevatorOffset(units::length::meter_t offset);
        units::length::meter_t GetElevatorHeight();

        void                   SetArmAngle(units::angle::degree_t angle);
        void                   AddArmAngleOffset(units::angle::degree_t angleOffset);
        units::angle::degree_t GetArmAngle();

        void                   SetWristAngle(units::angle::degree_t angle);
        void                   AddWristAngleOffset(units::angle::degree_t angleOffset);
        units::angle::degree_t GetWristAngle();

        void                   SetGripperWheelsVoltage(GripperWheelState gripperWheelState);
        void                   SetGripperWheelsVoltage(std::function<GripperWheelState()> gripperWheelState);
        units::voltage::volt_t GetGripperWheelsVoltage();

        GripperPoseEnum        GetState() { return m_gripperPose; }  // Get the Gripper Pose
        GripperActivationData  GetActivationState();               // Get the Gripper Pose State

        void                   UpdateLoggedValues();

    private:
        Gripper();

        void ConfigureElevatorMotor();
        void ConfigureArmMotor();
        void ConfigureWristMotor();
        void ConfigureGripperMotors();

        static Gripper*          m_gripper;  // The gripper singleton class

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
