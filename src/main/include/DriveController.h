#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "commands/GripperPose.h"
#include "commands/AlignToNearestTag.h"
#include "commands/GripperActivate.h"

#include "constants/GripperPose.h"
#include "constants/Climb.h"
#include "constants/Controller.h"


class DriveController
{
        public:
            DriveController(Drivetrain* drivetrain, Gripper* gripper);
        
            frc::ChassisSpeeds GetXChassisSpeeds();

        private:
            void   Configure();
            void   ConfigureXDriverControls();
            void   ConfigureXJogControls();

            void   ConfigureXBoxDriverControls();
            void   ConfigureXBoxJogControls();

            double GetThrottleRange();
            double GetExponentialValue(double joystickValue, double exponent);
            
            frc::Joystick m_driveController;

            // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
            frc::SlewRateLimiter<units::scalar>   m_xspeedLimiter;
            frc::SlewRateLimiter<units::scalar>   m_yspeedLimiter;
            frc::SlewRateLimiter<units::scalar>   m_rotLimiter;

            Drivetrain*   m_drivetrain;
            Gripper*      m_gripper;
};