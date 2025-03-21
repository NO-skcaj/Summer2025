#include "commands/AutonomousCoralAndAlgae.h"

using namespace ConstantsChassisPoseAutonomous;

#pragma region AutonomousCoralAndAlgae (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousCoralAndAlgae::AutonomousCoralAndAlgae(GripperPoseEnum                            gripperPoseEnum,
                                                 std::function<std::string()>               getStartingPosition,
                                                 std::function<ChassDrivePoseParameters ()> getParameters,
                                                 Drivetrain *drivetrain, Gripper *gripper, AprilTags *aprilTags)
{
    AddCommands(AutonomousOneCoralAprilTag(gripperPoseEnum, getStartingPosition, getParameters, drivetrain, gripper, aprilTags),   // Do all the coral stuff
                AprilTagDriveToAlgae(GripperPoseEnum::AlgaeHigh, aprilTags, gripper, drivetrain),             // Grab the Algae
                ChassisDrivePose(AlgaeAndCoralSpeed, AlgaeAndCoralXDistance, AlgaeAndCoralYDistance,
                                 AlgaeAndCoralAngleChange, AutonomousTimeOut, drivetrain),                    // Drive To Barge
                GripperPose(GripperPoseEnum::AlgaeBarge, gripper),                                            // Get the robot in position to shoot algae
                GripperActivate(gripper)                                                                      // Shoot the algae into the barge
    );
}
#pragma endregion
