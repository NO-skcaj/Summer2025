#pragma once

#include <hal/FRCUsageReporting.h>
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "LimelightHelpers.h"

#include "RobotContainer.h"

#include "Constants.h"

class Robot : public frc::TimedRobot
{
    public:

        void RobotInit()          override;
        void RobotPeriodic()      override;
        void AutonomousInit()     override;
        void AutonomousPeriodic() override;
        void TeleopInit()         override;
        void TeleopPeriodic()     override;
        void DisabledInit()       override;
        void DisabledPeriodic()   override;
        void TestInit()           override;
        void TestPeriodic()       override;
        void SimulationInit()     override;
        void SimulationPeriodic() override;

    private:

        // Pointer to the autonomous command
        frc2::Command  *m_autonomousCommand   = nullptr;

        // Instantiate the Robot container and get a pointer to the class
        RobotContainer *m_robotContainer      = RobotContainer::GetInstance();

        bool            m_chassisGyroReversed = false;
};
