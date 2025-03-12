#pragma once

#pragma region Includes
#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/Gripper.h"
#pragma endregion

//#define USE_APRILTAGS

#pragma region AutonomousConstants
namespace AutonomousConstants
{
    constexpr auto OneCoralSpeed            =  2_mps;

#ifdef USE_APRILTAGS
    // Center starting position for one coral placement
    constexpr auto OneCoralCenterXDistance  =  0.5_m;
    constexpr auto OneCoralCenterYDistance  =  0_m;
    constexpr auto OneCoralAngleChange      =  0_deg;

    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistance    =  1.0_m;
    constexpr auto OneCoralLeftYDistance    =  0.5_m;
    constexpr auto OneCoralLeftAngleChange  =  60_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistance   =  1.0_m;
    constexpr auto OneCoralRightYDistance   = -0.5_m;
    constexpr auto OneCoralRightAngleChange = -60_deg;
#else
    // Center starting position for one coral placement
    constexpr auto OneCoralCenterXDistance  =  1.2_m;
    constexpr auto OneCoralCenterYDistance  =  0_m;
    constexpr auto OneCoralAngleChange      =  0_deg;

    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistance    =  1.2_m;
    constexpr auto OneCoralLeftYDistance    =  1.5_m;
    constexpr auto OneCoralLeftAngleChange  =  60_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistance   =  1.2_m;
    constexpr auto OneCoralRightYDistance   = -1.5_m;
    constexpr auto OneCoralRightAngleChange = -60_deg;
#endif
    constexpr auto OneCoralTimeOut          = 15_s;

    // Algael/Barge starting position for one coral placement
    constexpr auto AlgaeAndCoralSpeed       =  1_mps;
    constexpr auto AlgaeAndCoralXDistance   = -1_m;
    constexpr auto AlgaeAndCoralYDistance   =  0_m;
    constexpr auto AlgaeAndCoralAngleChange = 180_deg;
    constexpr auto AlgaeAndCoralTimeOut     = 15_s;
};
#pragma endregion
