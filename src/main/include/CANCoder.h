#include "Encoder.h"

#include <ctre/phoenix6/CANcoder.hpp>


namespace hardware
{

class CANCoder : Encoder
{
public:
CANCoder(int CanId) : m_encoder{CanId}
{}

units::turn_t GetAbsoluteValue() override
{
return m_encoder.GetAbsoluteValue().GetValue();
}

private:
ctre::phoenix6::hardware::CANcoder m_encoder;
}

}