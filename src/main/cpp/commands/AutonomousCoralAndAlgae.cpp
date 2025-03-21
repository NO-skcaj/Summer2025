#include "commands/AutonomousCoralAndAlgae.h"

using namespace ConstantsChassisPoseAutonomous;

#pragma region AutonomousCoralAndAlgae (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousCoralAndAlgae::AutonomousCoralAndAlgae(GripperPoseEnum                            gripperPoseEnum,
                                                 std::function<std::string()>               getStartingPosition,
                                                 std::function<ChassDrivePoseParameters ()> getParameters,
                                                 Drivetrain *drivetrain, Gripper *gripper, AprilTags *aprilTags)
{
    AddCommands(AutonomousOneCoralAprilTag(gripperPoseEnum, getStartingPosition, getParameters, drivetrain, gripper, aprilTags),  // Do all the coral stuff
                ChassisDrivePose(getParameters().Speed, -24.0_in, 0.0_m, 0.0_deg, 15_s, drivetrain),                              // Move robot away from reef
                AprilTagDriveToAlgae(GripperPoseEnum::AlgaeHigh, aprilTags, gripper, drivetrain),                                 // Drive to the reef for Algae
                GripperActivate(gripper),                                                                                         // Get the algae
                ChassisDrivePose(getParameters().Speed, -24.0_in, 0.0_m, 0.0_deg, 15_s, drivetrain),                              // Move robot away from reef
                ChassisDrivePose(AlgaeAndCoralSpeed, AlgaeAndCoralXDistance, AlgaeAndCoralYDistance,                              // Drive to barge
                                 AlgaeAndCoralAngleChange, AutonomousTimeOut, drivetrain),                                        
                GripperPose(GripperPoseEnum::AlgaeBarge, gripper),                                                                // Set the gripper pose for the barge 
                GripperActivate(gripper)                                                                                          // Place the algae into the barge
                );
}
#pragma endregion
