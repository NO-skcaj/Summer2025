#include "commands/SimpleCommands.h"
#include "subsystems/Gripper.h"

using namespace pathplanner;

// Offsetting commands

frc2::CommandPtr OffsetElevator(bool isUp, units::meter_t offset)
{
    return frc2::RunCommand(
        [isUp, offset] {Gripper::GetInstance()->AddElevatorOffset(isUp ? offset : -offset); }, {Gripper::GetInstance()}).ToPtr();
}

frc2::CommandPtr OffsetArm(bool isUp, units::degree_t offset)
{
    return frc2::RunCommand(
        [isUp, offset] {Gripper::GetInstance()->AddArmAngleOffset(isUp ? offset : -offset); }, {Gripper::GetInstance()}).ToPtr();
}

frc2::CommandPtr OffsetWrist(bool isUp, units::degree_t offset )
{
    return frc2::RunCommand(
        [isUp, offset] {Gripper::GetInstance()->AddWristAngleOffset(isUp ? offset : -offset); }, {Gripper::GetInstance()}).ToPtr();
}

// Climb Commands

frc2::CommandPtr ClimbSetVoltage(double isUp, units::volt_t voltage)
{
    auto climb = Climb::GetInstance();
    return frc2::RunCommand(
        [climb, isUp, voltage] { climb->SetVoltage(isUp ? voltage : -voltage); }, {climb}).ToPtr();
}

// Drive Commands

frc2::CommandPtr ChassisSetSwerveWheelAnglesToZero()
{
    return frc2::InstantCommand(
        [] { Drivetrain::GetInstance()->SetWheelAnglesToZero(); }, {Drivetrain::GetInstance()}).ToPtr();
}

frc2::CommandPtr ChassisDrive(std::function<frc::ChassisSpeeds()>  speedsGetter)
{
    return frc2::RunCommand(
        [speedsGetter] { Drivetrain::GetInstance()->Drive(speedsGetter()); }, {Drivetrain::GetInstance()}).ToPtr();
}

frc2::CommandPtr ChassisZeroHeading()
{
    return frc2::InstantCommand(
        [] {Drivetrain::GetInstance()->ZeroHeading(); }, {Drivetrain::GetInstance()}).ToPtr();
}

frc2::CommandPtr FlipFieldCentricity()
{
    return frc2::InstantCommand([] { Drivetrain::GetInstance()->FlipFieldCentric(); }, {Drivetrain::GetInstance()}).ToPtr();
}

// Path following commands

frc2::CommandPtr ChassisDrivePose(std::string CommandName)
{
    return AutoBuilder::followPath(PathPlannerPath::fromPathFile(CommandName));
}

frc2::CommandPtr ChassisDrivePose(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side
{
    return AutoBuilder::pathfindToPose(targetPose, Constants::PathPlanner::Constraints);
}

frc2::CommandPtr ChassisDrivePoseFlipped(frc::Pose2d targetPose) // End goal state relative to the origin, blue alliance side then flipped to red
{
    return AutoBuilder::pathfindToPoseFlipped(targetPose, Constants::PathPlanner::Constraints);
}

// This command will align the robot to the nearest AprilTag
// It will use the AprilTag's pose to determine the target position and rotation
// The robot will drive towards the target position and rotate to face the target rotation
frc2::CommandPtr AlignToNearestTag(frc::Transform2d targetOffset)
{
    // This doesn't need to be a variable. When I wrote this, I just really liked using lambdas. Now, it kinda needs to be because its not in its own class
    std::function<frc::Pose2d(frc::Pose2d, frc::Transform2d)> getTargetWithOffset = 
        [] (frc::Pose2d targetPosition, frc::Transform2d targetOffset)
        {
            // Rotate offset
            return frc::Pose2d{
                targetPosition.X() +                  targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value()) - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),
                targetPosition.Y() +                  targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value()) + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),
                targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()};
        };

    return ChassisDrivePose(getTargetWithOffset(Odometry::GetInstance()->GetNearestTag(), targetOffset));
}

// Gripper Commands

frc2::CommandPtr GripperPose(GripperPoseEnum gripperPose)
{
    return frc2::InstantCommand(
        [gripperPose] { Gripper::GetInstance()->SetPose(gripperPose); }).ToPtr();
}

frc2::CommandPtr SetGripperWheels(std::function<Constants::GripperPose::GripperWheelState()> state)
{
    return frc2::RunCommand([state] { Gripper::GetInstance()->SetGripperWheelsVoltage(state); }, {Gripper::GetInstance()}).ToPtr();
}

frc2::CommandPtr SetGripperWheels(Constants::GripperPose::GripperWheelState state)
{
    return frc2::RunCommand([state] { Gripper::GetInstance()->SetGripperWheelsVoltage(state); }, {Gripper::GetInstance()}).ToPtr();
}

frc2::CommandPtr StopGripperWheels()
{
    return frc2::RunCommand([] { Gripper::GetInstance()->SetGripperWheelsVoltage({true, 0_V}); }, {Gripper::GetInstance()}).ToPtr();
}

// LEDs
frc2::CommandPtr SetLeds(LedMode ledMode, units::second_t howLong)
{
    return frc2::InstantCommand([ledMode] { Leds::GetInstance()->SetMode(ledMode); }, {Leds::GetInstance()})
            .AndThen(frc2::WaitCommand(howLong).ToPtr())
            .AndThen(frc2::InstantCommand([ledMode] { Leds::GetInstance()->SetMode(LedMode::Off); }, {Leds::GetInstance()}).ToPtr());
}

// Sequentials

frc2::CommandPtr PrepareAndAlign(frc::Transform2d prepState1, Constants::GripperPose::GripperPoseEnum prepState2)
{
    return AlignToNearestTag(prepState1)
            .AlongWith(GripperPose(prepState2));
}

frc2::CommandPtr AutoScore(frc::Transform2d scoreState1, Constants::GripperPose::GripperPoseEnum scoreState2)
{
        return PrepareAndAlign(scoreState1, scoreState2)
                .AndThen(std::move(frc2::WaitCommand(2.0_s).ToPtr()))
                .AndThen(std::move(GripperActivate()));
}

frc2::CommandPtr GripperActivate()
{
    Constants::GripperPoseActivate::GripperActivationData stateData = Gripper::GetInstance()->GetActivationState();

    return OffsetArm(true, stateData.ArmOffset)
        .AndThen(frc2::WaitCommand(stateData.ArmMoveWait).ToPtr())
        .AndThen(OffsetElevator(true, stateData.ElevatorOffset))
        .AndThen(frc2::WaitCommand(stateData.ElevatorWait).ToPtr())
        .AndThen(SetGripperWheels([stateData] { return Constants::GripperPose::GripperWheelState{stateData.BothWheels, stateData.GripperVoltage}; }))
        .AndThen(frc2::WaitCommand(stateData.GripperPlaceWait).ToPtr())
        .AndThen(OffsetElevator(true, -stateData.ElevatorFinish))
        .AndThen(OffsetArm(true, -stateData.ArmFinish))
        .AndThen(StopGripperWheels());
}

// Odometry periodic

frc2::CommandPtr UpdateOdometry()
{
    return frc2::InstantCommand{[] {Odometry::GetInstance()->Update();}, {Odometry::GetInstance()}}.ToPtr();
}