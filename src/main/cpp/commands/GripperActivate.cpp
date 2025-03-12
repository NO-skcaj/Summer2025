#include "commands/GripperActivate.h"

#pragma region GripperActivate
/// @brief CCommand to activate the gripper to a specific pose.
/// @param gripper The gripper subsystem.
GripperActivate::GripperActivate(Gripper *gripper) : m_gripper(gripper)
{
    // Set the command name
    SetName("GripperPose");

    // Declare subsystem dependencies
    AddRequirements(m_gripper);
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void GripperActivate::Initialize()
{
    m_state      = GripperState::ElevatorMove;
    m_isFinished = false;

    // Determine the action based on the present gripper pose
    switch (m_gripper->GetPose())
    {
        case GripperPoseEnum::CoralL1:
        case GripperPoseEnum::CoralAutonomousL1:
        {
            CoralL1();
            break;
        }

        case GripperPoseEnum::CoralL2:
        case GripperPoseEnum::CoralL3:
        {
            CoralL23();
            break;
        }

        case GripperPoseEnum::CoralL4:
        {
            CoralL4();
            break;
        }

        case GripperPoseEnum::AlgaeProcessor:
        {
            AlgaeProcessor();
            break;
        }

        case GripperPoseEnum::AlgaeBarge:
        {
            AlgaeBarge();
            break;
        }

        default:
        {
            m_state      = Complete;
            m_isFinished = true;
        }
    }

    frc::SmartDashboard::PutNumber("ElevatorOffset", m_stateData.ElevatorOffset.value());
    frc::SmartDashboard::PutNumber("ArmOffset",      m_stateData.ArmOffset.value());
    frc::SmartDashboard::PutNumber("GripperVoltage", m_stateData.GripperVoltage.value());
    frc::SmartDashboard::PutNumber("ElevatorFinish", m_stateData.ElevatorFinish.value());
    frc::SmartDashboard::PutNumber("ArmFinish",      m_stateData.ArmFinish.value());
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GripperActivate::Execute()
{
    // Execute the state machine
    switch (m_state)
    {
        case GripperState::ElevatorMove:
        {
            m_gripper->SetElevatorOffset(m_stateData.ElevatorOffset);  // Move the elevator
            m_timer = frc::GetTime() + m_stateData.ElevatorMoveWait;   // Time allowed to move the Elevator
            m_state = ArmMove;                                         // Next state is ArmMove
            break;
        }

        case GripperState::ArmMove:
        {
            if (frc::GetTime() > m_timer)
            {
                m_gripper->SetArmAngleOffset(m_stateData.ArmOffset);  // Move the Arm
                m_timer = frc::GetTime() + m_stateData.ArmMoveWait;   // Time allowed to move the Arm
                m_state = GripperWheelsMove;                          // Next state is GripperWheelsMove
            }
            break;
        }

        case GripperState::GripperWheelsMove:
        {
            if (frc::GetTime() > m_timer)
            {
                m_gripper->SetGripperWheelsVoltage(GripperWheelState{m_stateData.BothWheels, m_stateData.GripperVoltage});  // Move the Gripper Wheels
                m_timer = frc::GetTime() + m_stateData.GripperPlaceWait;                                                    // Time allowed to move the Gripper Wheels
                m_state = Finish;                                                                                           // Next state is finihed

            }
            break;
        }

        case GripperState::Finish:
        {
            if (frc::GetTime() > m_timer)
            {
                m_gripper->SetElevatorOffset(-m_stateData.ElevatorOffset);         // Return the Elevator to the start position
                m_gripper->SetArmAngleOffset(-m_stateData.ArmOffset);              // Return the Arm to the start position
                m_gripper->SetGripperWheelsVoltage(GripperWheelState{true, 0_V});  // Stop the Gripper Wheels
                m_isFinished = true;                                               // State machine is complete
            }
			break;
        }

        case GripperState::Complete:
        {
            // Stop the state machine
            m_isFinished = true;
        }

        default:
        {
            // Stop the state machine
            m_state      = Complete;
            m_isFinished = true;
        }
    }
}
#pragma endregion

#pragma region IsFinished
// Returns true when the command should end.
bool GripperActivate::IsFinished()
{
    return m_isFinished;
}
#pragma endregion

#pragma region CoralL1
/// @brief Method to set the gripper activate parameters for Coral at L1.
void GripperActivate::CoralL1()
{
    m_stateData.ElevatorOffset   = ActivateConstants::Coral1ElevatorOffset;
    m_stateData.ElevatorMoveWait = ActivateConstants::Coral1ElevatorWait;
    m_stateData.ArmOffset        = ActivateConstants::Coral1ArmOffset;
    m_stateData.ArmMoveWait      = ActivateConstants::Coral1ArmMoveWait;
    m_stateData.BothWheels       = ActivateConstants::Coral1BothWheels;
    m_stateData.GripperVoltage   = ActivateConstants::Coral1GripperVoltage;
    m_stateData.GripperPlaceWait = ActivateConstants::Coral1GripperPlaceWait;

    m_stateData.ElevatorFinish   = ActivateConstants::Coral1ElevatorFinish;
    m_stateData.ArmFinish        = ActivateConstants::Coral1ArmFinish;
}
#pragma endregion

#pragma region CoralL23
/// @brief Method to set the gripper activate parameters for Coral at L2 or L3.
void GripperActivate::CoralL23()
{
    m_stateData.ElevatorOffset   = ActivateConstants::Coral123ElevatorOffset;
    m_stateData.ElevatorMoveWait = ActivateConstants::Coral123ElevatorWait;
    m_stateData.ArmOffset        = ActivateConstants::Coral123ArmOffset;
    m_stateData.ArmMoveWait      = ActivateConstants::Coral123ArmMoveWait;
    m_stateData.BothWheels       = ActivateConstants::Coral123BothWheels;
    m_stateData.GripperVoltage   = ActivateConstants::Coral123GripperVoltage;
    m_stateData.GripperPlaceWait = ActivateConstants::Coral123GripperPlaceWait;

    m_stateData.ElevatorFinish   = ActivateConstants::Coral123ElevatorFinish;
    m_stateData.ArmFinish        = ActivateConstants::Coral123ArmFinish;
}
#pragma endregion

#pragma region CoralL4
/// @brief Method to set the gripper activate parameters for Coral at L4.
void GripperActivate::CoralL4()
{
    m_stateData.ElevatorOffset   = ActivateConstants::Coral4ElevatorOffset;
    m_stateData.ElevatorMoveWait = ActivateConstants::Coral4ElevatorWait;
    m_stateData.ArmOffset        = ActivateConstants::Coral4ArmOffset;
    m_stateData.ArmMoveWait      = ActivateConstants::Coral4ArmMoveWait;
    m_stateData.BothWheels       = ActivateConstants::Coral4BothWheels;
    m_stateData.GripperVoltage   = ActivateConstants::Coral4GripperVoltage;
    m_stateData.GripperPlaceWait = ActivateConstants::Coral4GripperPlaceWait;

    m_stateData.ElevatorFinish   = ActivateConstants::Coral4ElevatorFinish;
    m_stateData.ArmFinish        = ActivateConstants::Coral4ArmFinish;
}
#pragma endregion

#pragma region AlgaeProcessor
/// @brief Method to set the gripper activate parameters for Algae at the Processor.
void GripperActivate::AlgaeProcessor()
{
    m_stateData.ElevatorOffset   = ActivateConstants::AlgaeProcessorElevatorOffset;
    m_stateData.ElevatorMoveWait = ActivateConstants::AlgaeProcessorElevatorWait;
    m_stateData.ArmOffset        = ActivateConstants::AlgaeProcessorArmOffset;
    m_stateData.ArmMoveWait      = ActivateConstants::AlgaeProcessorArmMoveWait;
    m_stateData.BothWheels       = ActivateConstants::AlgaeProcessorBothWheels;
    m_stateData.GripperVoltage   = ActivateConstants::AlgaeProcessorGripperVoltage;
    m_stateData.GripperPlaceWait = ActivateConstants::AlgaeProcessorGripperPlaceWait;

    m_stateData.ElevatorFinish   = ActivateConstants::AlgaeProcessorElevatorFinish;
    m_stateData.ArmFinish        = ActivateConstants::AlgaeProcessorArmFinish;
}
#pragma endregion

#pragma region AlgaeBarge
/// @brief Method to set the gripper activate parameters for Algae at the Barge.
void GripperActivate::AlgaeBarge()
{
    m_stateData.ElevatorOffset   = ActivateConstants::AlgaeBargeElevatorOffset;
    m_stateData.ElevatorMoveWait = ActivateConstants::AlgaeBargeElevatorWait;
    m_stateData.ArmOffset        = ActivateConstants::AlgaeBargeArmOffset;
    m_stateData.ArmMoveWait      = ActivateConstants::AlgaeBargeArmMoveWait;
    m_stateData.BothWheels       = ActivateConstants::AlgaeBargeBothWheels;
    m_stateData.GripperVoltage   = ActivateConstants::AlgaeBargeGripperVoltage;
    m_stateData.GripperPlaceWait = ActivateConstants::AlgaeBargeGripperPlaceWait;

    m_stateData.ElevatorFinish   = ActivateConstants::AlgaeBargeElevatorFinish;
    m_stateData.ArmFinish        = ActivateConstants::AlgaeBargeArmFinish;
}
#pragma endregion
