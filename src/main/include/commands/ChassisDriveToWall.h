#pragma once

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class ChassisDriveToWall : public frc2::CommandHelper<frc2::Command, ChassisDriveToWall>
{
    public:

        explicit ChassisDriveToWall(units::meters_per_second_t speed,
                                    units::meter_t             distanceToWall,
                                    units::time::second_t      timeoutTime,
                                    Drivetrain                *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::meters_per_second_t        m_speed;                    // The speed of the chassis
        units::meter_t                    m_distanceToWall;           // The distance to the wall
        units::time::second_t             m_timeoutTime;              // The command time-out time
        Drivetrain                       *m_drivetrain;               // The drivetrain subsystem

        units::second_t                   m_startTime;                // The start of the drive time
        frc2::SwerveControllerCommand<4> *m_swerveControllerCommand;  // The swerve controller command
};
