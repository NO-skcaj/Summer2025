#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>

#include "subsystems/Drivetrain.h"

#include "commands/ChassisDrivePose.h"


namespace Constants
{

    namespace AlignPID
    {
        auto transP = 8;
        auto transI = 0;
        auto transD = 0;
        
        auto rotP = 1;
        auto rotI = 0;
        auto rotD = 0;
    }

}

class AlignToNearest : public frc2::CommandHelper<frc2::Command, AlignToNearest>
{
    public:

        explicit AlignToNearest(Drivetrain *drivetrain) : m_drivetrain    {drivetrain}, 
                                                          m_targetPosition{},

                                                          m_transXPID{Constants::AlignPID::transP, Constants::AlignPID::transI, Constants::AlignPID::transD},
                                                          m_transYPID{Constants::AlignPID::transP, Constants::AlignPID::transI, Constants::AlignPID::transD},
                                                          m_rotPID   {Constants::AlignPID::rotP,   Constants::AlignPID::rotI,   Constants::AlignPID::rotD}
        {}

        inline void     Initialize() override
        {
            m_targetPosition = m_drivetrain->GetNearestTag();

            m_rotPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

            m_transXPID.SetSetpoint(m_targetPosition.X().value());
            m_transYPID.SetSetpoint(m_targetPosition.Y().value());

            m_rotPID.SetSetpoint   (m_targetPosition.Rotation().Degrees().value());
        }
        
        inline void     Execute() override
        {
            m_drivetrain->Drive(
                units::meters_per_second_t {m_transXPID.Calculate(m_drivetrain->GetPose().X().value())},
                units::meters_per_second_t {m_transYPID.Calculate(m_drivetrain->GetPose().Y().value())},

                units::degrees_per_second_t{m_rotPID.Calculate   (m_drivetrain->GetPose().Rotation().Degrees().value())});
        }

        inline bool     IsFinished() override
        {
            if ((frc::Translation2d{m_drivetrain->GetPose().ToMatrix()}.Distance(frc::Translation2d{m_targetPosition.ToMatrix()}) < 2_in &&
          std::abs(frc::Translation2d{m_drivetrain->GetPose().ToMatrix()}.Angle().Degrees().value() - frc::Translation2d{m_targetPosition.ToMatrix()}.Angle().Degrees().value()) < 4))
                return true;
        }

        inline void     End(bool interrupted) override
        {
            m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);
            m_drivetrain->SetWheelAnglesToZero();
        }

    private:
        Drivetrain        *m_drivetrain; // The drivetrain subsystem

        frc::Pose2d        m_targetPosition;

        frc::PIDController m_transXPID;
        frc::PIDController m_transYPID;
        frc::PIDController m_rotPID;
};