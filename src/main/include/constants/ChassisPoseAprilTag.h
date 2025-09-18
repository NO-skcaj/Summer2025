#pragma once

#include <iostream>
#include <numbers>
#include <string>

#include <frc/geometry/Transform2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "subsystems/Gripper.h"


namespace Constants
{

namespace ChassisAprilTagToPose
{
    constexpr auto ChassisSpeed = 1.0_mps;
    constexpr auto TimeoutTime  = 15_s;

    //*************************** Coral Station *************************************
    constexpr frc::Transform2d CoralStation{ 0.0_m,
                                             0.0_m,
                                             0.0_deg};

    //*************************** Reef **********************************************
    // Coral L1 right
    constexpr frc::Transform2d CoralL1ReefRight{ 20.0_in,
                                                -10.0_in,
                                                 0.0_deg};

    // Coral L1 left
    constexpr frc::Transform2d CoralL1ReefLeft{ 20.0_in,
                                                10.0_in,
                                                 0.0_deg};
    // Coral L2 and L3 right
    constexpr frc::Transform2d CoralL23ReefRight{ 20.0_in,
                                                  -3.0_in,
                                                   0.0_deg};

    // Coral L2 and L3 left
    constexpr frc::Transform2d CoralL23ReefLeft{ 20.0_in,
                                                 10.0_in,
                                                  0.0_deg};

    // Coral L4 right
    constexpr frc::Transform2d CoralL4ReefRight{ 20.0_in,
                                                 -3.0_in,
                                                  0.0_deg};

    // Coral L4 left
    constexpr frc::Transform2d CoralL4ReefLeft{ 20.0_in,
                                                10.0_in,
                                                 0.0_deg};

    // Reef Algae
    constexpr frc::Transform2d AlgaeReef{ 20.0_in,
                                           0.0_in,
                                           0.0_deg};

    //*************************** Algae Processor ***********************************
    constexpr frc::Transform2d AlgaeProcessor{ 20.0_in,
                                                0.0_in,
                                                0.0_deg};

    //*************************** Algae Barge ***************************************
    constexpr frc::Transform2d AlgaeBarge{ 20.0_in,
                                             0.0_in,
                                             0.0_deg};
}

}