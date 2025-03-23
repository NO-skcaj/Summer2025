#include "commands/AprilTagDriveToAlgae.h"

AprilTagDriveToAlgae::AprilTagDriveToAlgae(GripperPoseEnum gripperPose, Gripper *gripper, Drivetrain *drivetrain)
{
    AddCommands(ChassisDriveToAprilTag(1.0_mps, 0.0_m, 0.0_m, 0.0_deg, 10.0_s, drivetrain),
                GripperPose(gripperPose, gripper),
                GripperActivate(gripper));
}
