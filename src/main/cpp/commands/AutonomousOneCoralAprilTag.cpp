#include "commands/AutonomousOneCoralAprilTag.h"
#include <frc2/command/ConditionalCommand.h>

#pragma region AutonomousOneCoralAprilTag (constructor)
/// @brief Command to place one coral in autonomous mode.
/// @param drivetrain The drivetrain subsystem.
/// @param gripper The gripper subsystem.
/// @param aprilTags The AprilTags subsystem.
AutonomousOneCoralAprilTag::AutonomousOneCoralAprilTag(GripperPoseEnum gripperPoseEnum,
                                                       std::function<std::string ()> getStartPosition,
                                                       std::function<ChassDrivePoseParameters ()> getParameters,
                                                       Drivetrain *drivetrain, Gripper *gripper)
{
    // Determine the starting position based on the selected string position ("L", "M", "R")
    if (getStartPosition().compare("M"))
    {
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),  // Set the gripper pose to Home
                    frc2::WaitCommand(0.5_s),                     // Allow time for the gripper pose to complete
                    GripperPose(gripperPoseEnum, gripper),        // Set the gripper to the appropriate level
                    frc2::WaitCommand(2.0_s),                     // Allow time for the gripper pose to complete
                    //ChassisDrivePose(getParameters, drivetrain),  // Drive closer to the reef
                    AprilTagDriveToCoral(gripperPoseEnum, []() { return true; }, gripper, drivetrain),
                    GripperActivate(gripper),
                    ChassisDrivePose(getParameters().Speed,       // Move robot away from reef
                                     -12.0_in, 0.0_m, 0.0_deg,
                                     15_s, drivetrain));
    }
    else
    {
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),  // Default behavior for other positions
                    frc2::WaitCommand(0.5_s),                     // Allow time for the gripper pose to complete
                    GripperPose(gripperPoseEnum, gripper),        // Set the gripper to the appropriate level
                    frc2::WaitCommand(2.0_s),                     // Allow time for the gripper pose to complete
                    ChassisDrivePose(getParameters, drivetrain),  // Drive closedr to the reef
                    GripperActivate(gripper),
                    ChassisDrivePose(getParameters().Speed,       // Move robot away from reef
                                     -12.0_in, 0.0_m, 0.0_deg,
                                     15_s, drivetrain));
    }

    // // Run the command sequence
    // AddCommands(GripperPose(GripperPoseEnum::Home, gripper),  // Set the gripper pose to Home
    //             frc2::WaitCommand(0.5_s),                     // Allow time for the gripper pose to complete
    //             GripperPose(gripperPoseEnum, gripper),        // Set the gripper to the appropiate level
    //             frc2::WaitCommand(2.0_s),                     // Allow time for the gripper pose to complete
    //             ChassisDrivePose(getParameters, drivetrain),  // Drive to the reef
    //             AprilTagDriveToCoral(gripperPoseEnum, []() { return true; }, aprilTags, gripper, drivetrain),
    //             GripperActivate(gripper)),
    //             ChassisDrivePose(getParameters().Speed,       // Move robot away from reef
    //                              -12.0_in, 0.0_m, 0.0_deg,
    //                              15_s, drivetrain);
}
#pragma endregion
