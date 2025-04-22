#pragma once

#include "Encoder.h"

#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants/CanIds.h"


namespace hardware
{

// CANCoder class to support the CANCoder
class CANCoder : Encoder
{
    public:
        // Constructor for the CANCoder class
        // The CANCoder is a CAN device, so the CAN ID is passed in
        CANCoder(int CanId) : m_encoder{CanId, Constants::CanIds::CanBus}
        {}

        // Configure the CANCoder
        units::turn_t GetAbsoluteValue() override
        {
            return m_encoder.GetAbsolutePosition().GetValue();
        }

    private:
        ctre::phoenix6::hardware::CANcoder m_encoder;
};

};