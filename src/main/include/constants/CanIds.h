#pragma once

namespace Constants
{

namespace CanIds
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

}