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

    private:
        frc::DigitalInput m_sensor;

};

}