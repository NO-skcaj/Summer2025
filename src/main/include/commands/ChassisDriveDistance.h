#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

#include "subsystems/Drivetrain.h"

class ChassisDriveDistance : public frc2::CommandHelper<frc2::Command, ChassisDriveDistance>
{
    public:

        explicit ChassisDriveDistance(units::meter_t distance, units::meters_per_second_t speed, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::meter_t             m_distance;         // The distance that the chassis will drive
        units::meters_per_second_t m_speed;            // The speed that the chassis will drive
        Drivetrain                *m_drivetrain;       // The drivetrain subsystem

        bool                       m_fieldCentricity;  // The field centricity setting (true = field centric, false = robot centric)
};
