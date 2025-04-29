#pragma once

#include <hal/FRCUsageReporting.h>
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"

#include "lib/logging/LoggingManager.h"

#include "Constants/Controller.h"
#include "constants/Gripper.h"


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
        std::unique_ptr<frc2::Command>  m_autonomousCommand;

        // Instantiate the Robot container and get a pointer to the class
        RobotContainer *m_robotContainer;

        bool            m_chassisGyroReversed = false;

        // Logging
        LoggingManager* m_loggingManager;  // The logging manager
};
