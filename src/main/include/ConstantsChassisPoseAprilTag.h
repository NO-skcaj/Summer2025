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
    constexpr auto ChassisSpeed                  =  2.0_mps;
    constexpr auto TimeoutTime                   =  15_s;

    constexpr auto CoralStationDistanceOffsetX   =  0.0_m;
    constexpr auto CoralStationDistanceOffsetY   =  0.0_m;
    constexpr auto CoralStationAngleOffset       =  0.0_deg;

    // Right:
    constexpr auto CoralL1ReefRightDistanceOffsetX =  0.5_m;  //24_in;  // 0.6_m;
    constexpr auto CoralL1ReefRightDistanceOffsetY =  -0.2_m; //12_in;  // 0.3_m;
    constexpr auto CoralL1ReefRightAngleOffset     =  0.0_deg;
    
    constexpr auto CoralL23ReefRightDistanceOffsetX =  0.4_m;  // 24_in;  // 0.6_m;
    constexpr auto CoralL23ReefRightDistanceOffsetY =  -0.03_m; //12_in;  // 0.3_m;
    constexpr auto CoralL23ReefRightAngleOffset     =  0.0_deg;
    
    constexpr auto CoralL4ReefRightDistanceOffsetX =  0.4_m;// 24_in;  // 0.6_m;
    constexpr auto CoralL4ReefRightDistanceOffsetY =  0.0_m;//12_in;  // 0.3_m;
    constexpr auto CoralL4ReefRightAngleOffset     =  0.0_deg;

    // Left
    constexpr auto CoralL1ReefLeftDistanceOffsetX  =  0.5_m;//24_in;  // 0.6_m;
    constexpr auto CoralL1ReefLeftDistanceOffsetY  =  0.3_m;//-12_in;  // 0.3_m;
    constexpr auto CoralL1ReefLeftAngleOffset      =  0.0_deg;
    
    constexpr auto CoralL23ReefLeftDistanceOffsetX  = 0.4_m;  // 24_in;  // 0.6_m;
    constexpr auto CoralL23ReefLeftDistanceOffsetY  = 0.32_m; //-12_in;  // 0.3_m;
    constexpr auto CoralL23ReefLeftAngleOffset      = 0.0_deg;
    
    constexpr auto CoralL4ReefLeftDistanceOffsetX  =  0.42_m; // 24_in;  // 0.6_m;
    constexpr auto CoralL4ReefLeftDistanceOffsetY  =  0.43_m; //-12_in;  // 0.3_m;
    constexpr auto CoralL4ReefLeftAngleOffset      =  0.0_deg;

    constexpr auto AlgaeReefDistanceOffsetX      =  0.0_m;
    constexpr auto AlgaeReefDistanceOffsetY      =  0.0_m;
    constexpr auto AlgaeReefAngleOffset          =  0.0_deg;

    constexpr auto AlgaeProcessorDistanceOffsetX =  0.0_m;
    constexpr auto AlgaeProcessorDistanceOffsetY =  0.0_m;
    constexpr auto AlgaeProcessorAngleOffset     =  0.0_deg;

    constexpr auto AlgaelBargeDistanceOffsetX    =  0.0_m;
    constexpr auto AlgaelBargeDistanceOffsetY    =  0.0_m;
    constexpr auto AlgaelBargeAngleOffset        =  0.0_deg;
}
#pragma endregion
