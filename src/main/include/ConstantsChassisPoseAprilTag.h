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

#include "subsystems/Gripper.h"
#pragma endregion

#pragma region ConstantsChassisAprilTagToPose
namespace ConstantsChassisAprilTagToPose
{
    constexpr auto ChassisSpeed                     =  2.0_mps;
    constexpr auto TimeoutTime                      =  15_s;

    //*************************** Coral Station *************************************
    constexpr auto CoralStationDistanceOffsetX      =  0.0_m;
    constexpr auto CoralStationDistanceOffsetY      =  0.0_m;
    constexpr auto CoralStationAngleOffset          =  0.0_deg;

    //*************************** Reef **********************************************
    // Coral L1 right
    constexpr auto CoralL1ReefRightDistanceOffsetX  = 20.0_in;  // 0.5_m;    // 24_in;  // 0.6_m;
    constexpr auto CoralL1ReefRightDistanceOffsetY  = -8.0_in;  // -0.2_m;    // 12_in;  // 0.3_m;
    constexpr auto CoralL1ReefRightAngleOffset      =  0.0_deg;

    // Coral L1 left
    constexpr auto CoralL1ReefLeftDistanceOffsetX   =  20.0_in;  // 0.5_m;   // 24_in;  // 0.6_m;
    constexpr auto CoralL1ReefLeftDistanceOffsetY   =  12.0_in;  // 0.3_m;   //-12_in;  // 0.3_m;
    constexpr auto CoralL1ReefLeftAngleOffset       =   0.0_deg;

    // Coral L2 and L3 right
    constexpr auto CoralL23ReefRightDistanceOffsetX =  16.0_in;  // 0.4_m;   // 24_in;  // 0.6_m;
    constexpr auto CoralL23ReefRightDistanceOffsetY = -12.0_in;  // -0.03_m;  // 12_in;  // 0.3_m;
    constexpr auto CoralL23ReefRightAngleOffset     =   0.0_deg;

    // Coral L2 and L3 left
    constexpr auto CoralL23ReefLeftDistanceOffsetX  =  16.0_in;  // 0.4_m;   // 24_in;  // 0.6_m;
    constexpr auto CoralL23ReefLeftDistanceOffsetY  =  12.5_in;  //-12_in;  // 0.3_m;
    constexpr auto CoralL23ReefLeftAngleOffset      =   0.0_deg;

    // Coral L4 right
    constexpr auto CoralL4ReefRightDistanceOffsetX  =  16.0_in;  // 0.4_m;   // 24_in;  // 0.6_m;
    constexpr auto CoralL4ReefRightDistanceOffsetY  =   0.0_in;  // 12_in;  // 0.3_m;
    constexpr auto CoralL4ReefRightAngleOffset      =  0.0_deg;

    // Coral L4 left
    constexpr auto CoralL4ReefLeftDistanceOffsetX   =  16.0_in;  //  0.42_m;  // 24_in;  // 0.6_m;
    constexpr auto CoralL4ReefLeftDistanceOffsetY   =  17.0_in;  //-12_in;  // 0.3_m;
    constexpr auto CoralL4ReefLeftAngleOffset       =   0.0_deg;

    // Reef Algae
    constexpr auto AlgaeReefDistanceOffsetX         =  0.0_m;
    constexpr auto AlgaeReefDistanceOffsetY         =  0.0_m;
    constexpr auto AlgaeReefAngleOffset             =  0.0_deg;

    //*************************** Algae Processor ***********************************
    constexpr auto AlgaeProcessorDistanceOffsetX    =  0.0_m;
    constexpr auto AlgaeProcessorDistanceOffsetY    =  0.0_m;
    constexpr auto AlgaeProcessorAngleOffset        =  0.0_deg;

    //*************************** Algae Barge ***************************************
    constexpr auto AlgaelBargeDistanceOffsetX       =  0.0_m;
    constexpr auto AlgaelBargeDistanceOffsetY       =  0.0_m;
    constexpr auto AlgaelBargeAngleOffset           =  0.0_deg;
}
#pragma endregion
