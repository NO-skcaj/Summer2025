#pragma once

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/XboxController.h>

#include "subsystems/Gripper.h"
#include "subsystems/Climb.h"

#include "commands/GripperPose.h"

#include "constants/GripperPose.h"
#include "constants/Climb.h"
#include "constants/Controller.h"


class OperatorController
{
        public:
            OperatorController(Gripper* gripper, Climb* climb);
        
        private:
            void Configure();
            
            void ConfigureGripperControls();

            void ConfigureCoralPoseControls();

            void ConfigureAlgaePoseControls();

            void ConfigureClimberControls();

            GripperWheelState GetWheelInput();

            frc::XboxController m_operatorController;

            Gripper*            m_gripper;
            Climb*              m_climb;
};