#include "commands/AutonomousScoreAlgaeAndCoral.h"

#pragma region AutonomousScoreAlgaeAndCoral (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousScoreAlgaeAndCoral::AutonomousScoreAlgaeAndCoral(Drivetrain *drivetrain, Gripper *gripper, AprilTags *aprilTags)
{
    // AddCommands(AutonomousOneCoral(drivetrain, gripper, aprilTags),                                   // Do all the coral stuff
    //             AprilTagGrabAlgae(GripperPoseEnum::AlgaeHigh, aprilTags, gripper, drivetrain),        // Grab the Algae
    //             ChassisDrivePose(AlgaeAndCoralSpeed, AlgaeAndCoralXDistance, AlgaeAndCoralYDistance,
    //                              AlgaeAndCoralAngleChange, AlgaeAndCoralTimeOut, drivetrain),         // Drive To Barge
    //             GripperPose(GripperPoseEnum::AlgaeBarge, gripper),                                    // Get the robot in position to shoot algae
    //             GripperActivate(gripper)                                                              // Shoot the algae into the barge
    // );
}
#pragma endregion
