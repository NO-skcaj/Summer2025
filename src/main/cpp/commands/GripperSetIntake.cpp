#include "commands/GripperSetIntake.h"

#pragma region GripperSetIntake
/// @brief Constructor for the GripperSetIntake class.
/// @param angle The angle to set the arm.
/// @param velocity The velocity to set the gripper wheels.
/// @param gripper The gripper subsystem.
GripperSetIntake::GripperSetIntake(units::angle::degree_t angle, double velocity, Gripper *gripper) :
                                   m_angle(angle), m_velocity(velocity), m_gripper(gripper)
{
    // Set the command name
    SetName("GripperSetIntake");

    // Declare subsystem dependencies
    AddRequirements(gripper);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GripperSetIntake::Execute()
{
    // Set the gripper angle
    m_gripper->SetWristAngle(m_angle);

    // Set the gripper wheel velocity
    m_gripper->SetGripperWheelsVelocity(m_velocity);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool GripperSetIntake::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
