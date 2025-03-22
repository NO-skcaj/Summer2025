#pragma once

#include <frc/MathUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class ChassisDriveSerpentine : public frc2::CommandHelper<frc2::Command, ChassisDriveSerpentine>
{
    public:

        explicit ChassisDriveSerpentine(units::velocity::meters_per_second_t speed,
                                        units::time::second_t                timeoutTime,
                                        Drivetrain                          *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::meters_per_second_t        m_speed;                    // The speed that the chassis will drive
        units::time::second_t             m_timeoutTime;              // The command time-out time
        Drivetrain                       *m_drivetrain;               // The drivetrain subsystem

        units::second_t                   m_startTime;                // The start of the drive time
        frc2::SwerveControllerCommand<4> *m_swerveControllerCommand;  // The swerve controller command
};
