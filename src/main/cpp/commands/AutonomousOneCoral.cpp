#include "commands/AutonomousOneCoral.h"

using namespace AutonomousConstants;

#pragma region AutonomousOneCoral (constructor)
/// @brief Command to place one coral in autonomous mode.
/// @param drivetrain The drivetrain subsystem.
/// @param gripper The gripper subsystem.
/// @param aprilTags The AprilTags subsystem.
AutonomousOneCoral::AutonomousOneCoral(GripperPoseEnum gripperPoseEnum, std::function<ChassDrivePoseParameters ()> getParameters, Drivetrain *drivetrain, Gripper *gripper, AprilTags *aprilTags)
{
    // Run the command sequence
    // AddCommands(ChassisDrivePose(getParameters, drivetrain),
    //             AprilTagScoreCoral(gripperPoseEnum, []() { return true; }, aprilTags, gripper, drivetrain),
    //             GripperActivate(gripper));
    
    // Run the command sequence
    AddCommands(GripperPose(GripperPoseEnum::Home, gripper),
                ChassisDrivePose(getParameters, drivetrain),
                GripperPose(gripperPoseEnum, gripper));//,
                //GripperActivate(gripper));
}
#pragma endregion
