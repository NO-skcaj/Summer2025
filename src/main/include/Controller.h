#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/Drivetrain/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/Climb.h"

#include "commands/SimpleCommands.h"

#include "constants/Controller.h"


class Controller
{
        public:
            Controller();
        
            std::function<frc::ChassisSpeeds()> GetChassisSpeedsGetter();

        private:
            void                                ConfigureOperator();

            GripperWheelState                   GetWheelInput();

            void                                ConfigureDriverControls();
            void                                ConfigureDriverJogControls();
        
            double                              GetThrottleRange();
            double                              GetExponentialValue(double joystickValue, double exponent);
            
            frc::XboxController                 m_driveController;
            frc::XboxController                 m_operatorController;

            // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
            frc::SlewRateLimiter<units::scalar> m_xspeedLimiter;
            frc::SlewRateLimiter<units::scalar> m_yspeedLimiter;
            frc::SlewRateLimiter<units::scalar> m_rotLimiter;
};