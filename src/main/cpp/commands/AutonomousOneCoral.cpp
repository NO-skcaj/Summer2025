#include "commands/AutonomousOneCoral.h"

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
    AddCommands(GripperPose(GripperPoseEnum::Home, gripper),  // Set the gripper pose to Home
                ChassisDrivePose(getParameters, drivetrain),  // Get the chassis drive pose parameters
                GripperPose(gripperPoseEnum, gripper),        // Set the gripper to the appropiate level
                frc2::WaitCommand(2_s),                       // Allow time for the gripper pose to complete
                GripperActivate(gripper)                      // Place the coral
    );
}
#pragma endregion
