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
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),                                          // Default behavior for other positions
                    frc2::WaitCommand(0.5_s),                                                             // Allow time for the gripper pose to complete
                    GripperPose(gripperPoseEnum, gripper),                                                // Set the gripper to the appropriate level
                    frc2::WaitCommand(2.0_s),                                                             // Allow time for the gripper pose to complete
                    // ChassisDrivePose(getParameters, drivetrain),                                          // Drive closedr to the reef
                    // frc2::WaitCommand(0.5_s),                                                             // Allow time to read the AprilTag
                    AprilTagDriveToCoral(gripperPoseEnum, []() { return true; }, gripper, drivetrain),    // Drive to the reef
                    GripperActivate(gripper),                                                             // Activate the gripper to place the coral
                    ChassisDrivePose(getParameters().Speed, -12.0_in, 0.0_m, 0.0_deg, 5_s, drivetrain));  // Move robot away from reef
    }
    else
    {
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),                                          // Default behavior for other positions
                    frc2::WaitCommand(0.5_s),                                                             // Allow time for the gripper pose to complete
                    GripperPose(gripperPoseEnum, gripper),                                                // Set the gripper to the appropriate level
                    frc2::WaitCommand(2.0_s),                                                             // Allow time for the gripper pose to complete
                    ChassisDrivePose(getParameters, drivetrain),                                          // Drive closedr to the reef
                    frc2::WaitCommand(0.5_s),                                                             // Allow time to read the AprilTag
                    AprilTagDriveToCoral(gripperPoseEnum, []() { return true; }, gripper, drivetrain),    // Drive to the reef
                    GripperActivate(gripper),                                                             // Activate the gripper to place the coral
                    ChassisDrivePose(getParameters().Speed, -12.0_in, 0.0_m, 0.0_deg, 5_s, drivetrain));  // Move robot away from reef
    }
}
#pragma endregion
