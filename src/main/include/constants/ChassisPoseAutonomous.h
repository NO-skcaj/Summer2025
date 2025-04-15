#pragma once

#include <iostream>
#include <numbers>
#include <string>

#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/Gripper.h"

namespace Constants
{

namespace ChassisPoseAutonomous
{
    constexpr auto AutonomousSpeed                  =  2_mps;
    constexpr auto AutonomousTimeOut                = 15_s;

    /************************ OneCoral AprilTag **********************************/
    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistanceAprilTag    =  66_in;  // 1.0_m;
    constexpr auto OneCoralLeftYDistanceAprilTag    =  16_in;  // 0.5_m;
    constexpr auto OneCoralLeftAngleChangeAprilTag  =  60_deg;

    // Middle starting position for one coral placement
    constexpr auto OneCoralMiddleXDistanceAprilTag  =  32_in;  // 0.5_m;
    constexpr auto OneCoralMiddleYDistanceAprilTag  =  0_m;
    constexpr auto OneCoralAngleChangeAprilTag      =  0_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistanceAprilTag   =  66_in;  //  1.0_m;
    constexpr auto OneCoralRightYDistanceAprilTag   = -16_in;  // -0.5_m;
    constexpr auto OneCoralRightAngleChangeAprilTag = -60_deg;

    /************************* TwoCoral Pose AprilTag ****************************/
    

    /******************** Algae and One Coral AprilTag ***************************/
    // Algael/Barge starting position for one coral placement
    constexpr auto AlgaeAndCoralSpeed               =  2_mps;
    constexpr auto AlgaeAndCoralXDistance           =  39_in;  // 1_m;
    constexpr auto AlgaeAndCoralYDistance           =  0_m;
    constexpr auto AlgaeAndCoralAngleChange         = 180_deg;

    /************************ OneCoral Pose Only *********************************/
    // Left starting position for one coral placement
    constexpr auto OneCoralLeftXDistance            =  72_in;  // 1.829_m;
    constexpr auto OneCoralLeftYDistance            =  39_in;  // 0.9902_m;
    constexpr auto OneCoralLeftAngleChange          =  60_deg;

    // Middle starting position for one coral placement
    constexpr auto OneCoralMiddleXDistance          =  46_in;  // 1.168_m;
    constexpr auto OneCoralMiddleYDistance          =  0_m;
    constexpr auto OneCoralAngleChange              =  0_deg;

    // Right starting position for one coral placement
    constexpr auto OneCoralRightXDistance           =  72_in;  //  1.829_m;
    constexpr auto OneCoralRightYDistance           = -39_in;  // -0.9902_m;
    constexpr auto OneCoralRightAngleChange         = -60_deg;
}

}