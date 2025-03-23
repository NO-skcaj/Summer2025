#include "commands/AprilTagDriveToCoral.h"

using namespace ConstantsChassisAprilTagToPose;

AprilTagDriveToCoral::AprilTagDriveToCoral(GripperPoseEnum               gripperPose,
                                           const std::function<bool ()> &GetJoystickToggle,
                                           Gripper                      *gripper,
                                           Drivetrain                   *drivetrain)
{
    // Get the state of the joystick toggle
    if (GetJoystickToggle())
        AddCommands(GripperPose(gripperPose, gripper),
                    ChassisDriveToAprilTag(ChassisSpeed, CoralReefLeftDistanceOffsetX, CoralReefLeftDistanceOffsetY,
                                           CoralReefLeftAngleOffset, TimeoutTime, drivetrain));
    else
        AddCommands(GripperPose(gripperPose, gripper),
                    ChassisDriveToAprilTag(ChassisSpeed, -CoralReefRightDistanceOffsetX, CoralReefRightDistanceOffsetY,
                                           CoralReefRightAngleOffset, TimeoutTime, drivetrain));
}
