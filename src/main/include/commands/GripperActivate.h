#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

#include "Constants/Controller.h"
#include "Constants/GripperPoseActivate.h"


enum GripperState
{
    ElevatorMove,
    ArmMove,
    GripperWheelsMove,
    Finish,
    Complete
};

struct GripperStateData
{
    units::meter_t         ElevatorOffset   = 0_m;
    units::time::second_t  ElevatorMoveWait = 0_s;
    units::angle::degree_t ArmOffset        = 0_deg;
    units::time::second_t  ArmMoveWait      = 0_s;
    bool                   BothWheels       = true;
    units::voltage::volt_t GripperVoltage   = 0_V;
    units::time::second_t  GripperPlaceWait = 0_s;

    units::meter_t         ElevatorFinish   = 0_m;
    units::angle::degree_t ArmFinish        = 0_deg;
};

class GripperActivate : public frc2::CommandHelper<frc2::Command, GripperActivate>
{
    public:

        explicit GripperActivate(Gripper *gripper);

        void     Initialize() override;
        void     Execute()    override;
        bool     IsFinished() override;

    private:

        void CoralL1();
        void CoralL23();
        void CoralL4();
        void AlgaeProcessor();
        void AlgaeBarge();

        GripperState          m_state;
        GripperStateData      m_stateData;

        units::time::second_t m_timer;
        bool                  m_isFinished = false;

        GripperPoseEnum       m_gripperPose;  // The gripper pose
        Gripper              *m_gripper;      // The Gripper subsystem
};
