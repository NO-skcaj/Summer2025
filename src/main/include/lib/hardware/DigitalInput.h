#pragma once

#include "lib/hardware/Sensor.h"

#include <frc/DigitalInput.h>


namespace hardware
{

class DigitalInput : public Sensor<bool>
{
    public:
        DigitalInput(int CanId) : m_sensor{CanId} {};

        bool operator==(bool operand) override
        {
            return m_sensor.Get() == operand;
        }

        operator bool() override
        {
            return m_sensor.Get();
        }

    private:
        frc::DigitalInput m_sensor;

};

}