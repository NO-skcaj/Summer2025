#pragma once

#include <utility>

#include <frc/MathUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"

#include "Constants.h"

struct ChassDrivePoseParameters
{
    units::meters_per_second_t Speed;
    units::meter_t             DistanceX;
    units::meter_t             DistanceY;
    units::degree_t            Angle;
    units::time::second_t      TimeoutTime;
};

class ChassisDrivePose : public frc2::CommandHelper<frc2::Command, ChassisDrivePose>
{
    public:

        explicit ChassisDrivePose(units::velocity::meters_per_second_t speed,
                                  units::meter_t                       distanceX,
                                  units::meter_t                       distanceY,
                                  units::angle::degree_t               angle,
                                  units::time::second_t                timeoutTime,
                                  Drivetrain                          *drivetrain);

        explicit ChassisDrivePose(std::function<ChassDrivePoseParameters()> getParameters, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        bool                              m_readParameters;           // Indicates the parameters should be read from the lambda function

        units::meters_per_second_t        m_speed;                    // The speed that the chassis will drive
        units::meter_t                    m_distanceX;                // The distance that the chassis will drive in the X direction
        units::meter_t                    m_distanceY;                // The distance that the chassis will drive in the Y direction
        units::time::second_t             m_timeoutTime;              // The command time-out time
        units::angle::degree_t            m_angle;                    // The angle that the chassis will drive
        Drivetrain                       *m_drivetrain;               // The drivetrain subsystem

        units::second_t                   m_startTime;                // The start of the drive time
        bool                              m_finished;                 // Indicates the command is finished
        frc2::SwerveControllerCommand<4> *m_swerveControllerCommand;  // The swerve controller command

        bool                              m_fieldCentricity;          // Variable to remember the field centricity

        std::function<ChassDrivePoseParameters()> m_getParameters;    // The lambda function to get the parameters
};
