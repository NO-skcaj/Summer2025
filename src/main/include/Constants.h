#pragma once

#pragma region Includes
#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#pragma endregion

#pragma region CanConstants
namespace CanConstants
{
    const     auto CanBus                            = "rio";

    constexpr auto SwerveFrontLeftDriveMotorCanId    = 10;
    constexpr auto SwerveFrontLeftAngleMotorCanId    = 11;
    constexpr auto SwerveFrontLeftAngleEncoderCanId  = 20;

    constexpr auto SwerveFrontRightDriveMotorCanId   = 12;
    constexpr auto SwerveFrontRightAngleMotorCanId   = 13;
    constexpr auto SwerveFrontRightAngleEncoderCanId = 21;

    constexpr auto SwerveRearLeftDriveMotorCanId     = 14;
    constexpr auto SwerveRearLeftAngleMotorCanId     = 15;
    constexpr auto SwerveRearLeftAngleEncoderCanId   = 22;

    constexpr auto SwerveRearRightDriveMotorCanId    = 16;
    constexpr auto SwerveRearRightAngleMotorCanId    = 17;
    constexpr auto SwerveRearRightAngleEncoderCanId  = 23;

    constexpr auto ElevatorMotorCanId                = 33;
    constexpr auto ArmMotorCanId                     = 30;
    constexpr auto WristMotorCanId                   = 34;
    constexpr auto GripperMotorCanIdFixed            = 32;
    constexpr auto GripperMotorCanIdFree             = 31;

    constexpr auto ClimbMotorCanId                   = 40;

    constexpr auto MotorConfigurationAttempts        =  5;
}
#pragma endregion

#pragma region ControllerConstants
namespace ControllerConstants
{
    constexpr auto DriverControllerUsbPort =   0;
    constexpr auto JoystickOperatorUsbPort =   1;

    constexpr auto JoystickStrafeIndex     =   0;
    constexpr auto JoystickForwardIndex    =   1;
    constexpr auto JoystickAngleIndex      =   2;  // 4 for xbox controller, 2 for extreme 3d controller(stick controller)
    constexpr auto JoystickThrottleIndex   =   3;

    constexpr auto ThrottleMinimum         = 0.5;

    constexpr auto JoystickDeadZone        = 0.0;
    constexpr auto JoystickRotateDeadZone  = 0.08;

    constexpr auto ExponentForward         = 2.0;
    constexpr auto ExponentStrafe          = 2.0;
    constexpr auto ExponentAngle           = 5.0;
}
#pragma endregion

#pragma region Extreme3DConstants
namespace Extreme3DConstants
{
    constexpr auto HandleTrigger    =  1;
    constexpr auto HandleSide       =  2;
    constexpr auto HandleLowerLeft  =  3;
    constexpr auto HandleLowerRight =  4;
    constexpr auto HandleUpperLeft  =  5;
    constexpr auto HandleUpperRight =  6;
    constexpr auto Handle7          =  7;
    constexpr auto Handle8          =  8;
    constexpr auto Handle9          =  9;
    constexpr auto Handle10         = 10;
    constexpr auto Handle11         = 11;
    constexpr auto Handle12         = 12;
    }
#pragma endregion

#pragma region ControlPanelConstants
namespace ControlPanelConstants
{
    // Digital Inputs
    constexpr auto CoralGnd         =  1;
    constexpr auto CoralStn         =  6;
    constexpr auto CoralL1          =  2;
    constexpr auto CoralL2          =  7;
    constexpr auto CoralL3          =  3;
    constexpr auto CoralL4          = 16;
    constexpr auto CoralSideSelect  = 15;

    constexpr auto AlgaeGnd         = 18;
    constexpr auto AlgaeCoral       = 20;
    constexpr auto AlgaeLow         = 17;
    constexpr auto AlgaeHigh        = 19;
    constexpr auto AlgaeProcessor   =  5;
    constexpr auto AlgaeBarge       =  4;

    constexpr auto BothWheelsActive = 11;
    constexpr auto OperatorWheels   = 13;

    constexpr auto ElevatorUp       = 10;
    constexpr auto ElevatorDown     = 12;

    constexpr auto ClimbUp          = 14;
    constexpr auto ClimbDown        =  8;

    constexpr auto Home             =  9;

    // Analog Inputs
    constexpr auto GripperMotor     =  3;
}
#pragma endregion

#pragma region xBoxConstants
namespace XBoxConstants
{
    constexpr auto A                 =   1;
    constexpr auto B                 =   2;
    constexpr auto X                 =   3;
    constexpr auto Y                 =   4;
    constexpr auto LeftBumper        =   5;
    constexpr auto RightBumper       =   6;
    constexpr auto Back              =   7;
    constexpr auto Start             =   8;
    constexpr auto LeftStickButton   =   9;
    constexpr auto RightStickButton  =  10;

    constexpr auto Pov_0             =   0;
    constexpr auto Pov_45            =  45;
    constexpr auto Pov_90            =  90;
    constexpr auto Pov_135           = 135;
    constexpr auto Pov_180           = 180;
    constexpr auto Pov_225           = 225;
    constexpr auto Pov_270           = 270;
    constexpr auto Pov_315           = 315;
}
#pragma endregion
