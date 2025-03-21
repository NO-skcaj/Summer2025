#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<units::meters_per_second_t()>  forward,
                              std::function<units::meters_per_second_t()>  strafe,
                              std::function<units::radians_per_second_t()> angle,
                              std::function<bool()>                        ultraSonicEnabled,
                              Drivetrain                                  *drivetrain);

        void     Execute() override;

    private:

        std::function<units::meters_per_second_t()>  m_forward;            // The forward speed
        std::function<units::meters_per_second_t()>  m_strafe;             // The strafe speed
        std::function<units::radians_per_second_t()> m_angle;              // The angle speed
        std::function<bool()>                        m_ultraSonicEnabled;  // If the ultraSonic should do anything
        Drivetrain                                  *m_drivetrain;         // The drivetrain subsystem;
};
