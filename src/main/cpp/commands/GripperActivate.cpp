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
    m_startTime  = frc::GetTime();
    m_isFinished = false;

    frc::SmartDashboard::PutNumber("Gripper Pose", m_gripper->GetPose());

    // Determine the action based on the present gripper pose
    switch (m_gripper->GetPose())
    {
        case GripperPoseEnum::CoralGround:
        {
            CoralGround();
            break;
        }

        case GripperPoseEnum::CoralStation:
        {
            CoralStation();
            break;
        }

        case GripperPoseEnum::CoralL1:
        case GripperPoseEnum::CoralL2:
        case GripperPoseEnum::CoralL3:
        {
            CoralL123();
            break;
        }

        case GripperPoseEnum::CoralL4:
        {
            CoralL4();
            break;
        }

        case GripperPoseEnum::AlgaeGround:
        {
            AlgaeGround();
            break;
        }

        case GripperPoseEnum::AlgaeOnCoral:
        {
            AlgaeOnCoral();
            break;
        }

        case GripperPoseEnum::AlgaeLow:
        {
            AlgaeLow();
            break;
        }

        case GripperPoseEnum::AlgaeHigh:
        {
            AlgaeHigh();
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
            m_gripper->SetElevatorOffset(m_stateData.ElevatorOffset);
            m_state = ArmMove;
            break;
        }

        case GripperState::ArmMove:
        {
            if (frc::GetTime() > m_stateData.Wait1)
            {
                m_state = ArmMove;
                m_gripper->SetArmAngleOffset(m_stateData.ArmOffset);
                m_state = GripperWheelsMove;
            }
            break;
        }

        case GripperState::GripperWheelsMove:
        {
            if (frc::GetTime() > m_stateData.Wait2)
            {
                m_state = GripperWheelsMove;
                m_gripper->SetGripperWheelsVoltage(GripperWheelState{m_stateData.BothWheels, m_stateData.GripperVoltage});
                m_state = Finish;
            }
            break;
        }

        case GripperState::Finish:
        {
            if (frc::GetTime() > m_stateData.Wait3)
            {
                m_state = Finish;
                m_gripper->SetGripperWheelsVoltage(GripperWheelState{true, 0_V});
                m_gripper->SetArmAngleOffset(-m_stateData.ArmOffset);
                m_gripper->SetElevatorOffset(-m_stateData.ElevatorOffset);
                m_state      = Complete;
                m_isFinished = true;
            }
			break;
        }

        default:
        {
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

#pragma region SettingStateData
void GripperActivate::CoralGround()
{
    m_stateData.ElevatorOffset = ActivateConstants::CoralGroundElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::CoralGroundWait1;
    m_stateData.ArmOffset      = ActivateConstants::CoralGroundArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::CoralGroundWait2;
    m_stateData.BothWheels     = ActivateConstants::CoralGroundBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::CoralGroundGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::CoralGroundWait3;

    m_stateData.ElevatorFinish = ActivateConstants::CoralGroundElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::CoralGroundArmFinish;
}

void GripperActivate::CoralStation()
{
    m_stateData.ElevatorOffset = ActivateConstants::CoralStationElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::CoralStationWait1;
    m_stateData.ArmOffset      = ActivateConstants::CoralStationArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::CoralStationWait2;
    m_stateData.BothWheels     = ActivateConstants::CoralStationBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::CoralStationGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::CoralStationWait3;

    m_stateData.ElevatorFinish = ActivateConstants::CoralStationElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::CoralStationArmFinish;
}

void GripperActivate::CoralL123()
{
    m_stateData.ElevatorOffset = ActivateConstants::Coral123ElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::Coral123Wait1;
    m_stateData.ArmOffset      = ActivateConstants::Coral123ArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::Coral123Wait2;
    m_stateData.BothWheels     = ActivateConstants::Coral123BothWheels;
    m_stateData.GripperVoltage = ActivateConstants::Coral123GripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::Coral123Wait3;

    m_stateData.ElevatorFinish = ActivateConstants::Coral123ElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::Coral123ArmFinish;
}

void GripperActivate::CoralL4()
{
    m_stateData.ElevatorOffset = ActivateConstants::Coral4ElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::Coral4Wait1;
    m_stateData.ArmOffset      = ActivateConstants::Coral4ArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::Coral4Wait2;
    m_stateData.BothWheels     = ActivateConstants::Coral4BothWheels;
    m_stateData.GripperVoltage = ActivateConstants::Coral4GripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::Coral4Wait3;

    m_stateData.ElevatorFinish = ActivateConstants::Coral4ElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::Coral4ArmFinish;
}

void GripperActivate::AlgaeGround()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeGroundElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeGroundWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeGroundArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeGroundWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeGroundBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeGroundGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeGroundWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeGroundElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeGroundArmFinish;
}

void GripperActivate::AlgaeOnCoral()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeOnCoralElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeOnCoralWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeOnCoralArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeOnCoralWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeOnCoralBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeOnCoralGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeOnCoralWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeOnCoralElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeOnCoralArmFinish;
}

void GripperActivate::AlgaeLow()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeLoElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeLoWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeLoArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeLoWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeLoBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeLoGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeLoWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeLoElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeLoArmFinish;
}

void GripperActivate::AlgaeHigh()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeHighElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeHighWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeHighArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeHighWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeHighBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeHighGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeHighWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeHighElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeHighArmFinish;
}

void GripperActivate::AlgaeProcessor()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeProcessorElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeProcessorWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeProcessorArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeProcessorWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeProcessorBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeProcessorGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeProcessorWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeProcessorElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeProcessorArmFinish;
}

void GripperActivate::AlgaeBarge()
{
    m_stateData.ElevatorOffset = ActivateConstants::AlgaeBargeElevatorOffset;
    m_stateData.Wait1          = m_startTime + ActivateConstants::AlgaeBargeWait1;
    m_stateData.ArmOffset      = ActivateConstants::AlgaeBargeArmOffset;
    m_stateData.Wait2          = m_stateData.Wait1 + ActivateConstants::AlgaeBargeWait2;
    m_stateData.BothWheels     = ActivateConstants::AlgaeBargeBothWheels;
    m_stateData.GripperVoltage = ActivateConstants::AlgaeBargeGripperVoltage;
    m_stateData.Wait3          = m_stateData.Wait2 + ActivateConstants::AlgaeBargeWait3;

    m_stateData.ElevatorFinish = ActivateConstants::AlgaeBargeElevatorFinish;
    m_stateData.ArmFinish      = ActivateConstants::AlgaeBargeArmFinish;
}
#pragma endregion
