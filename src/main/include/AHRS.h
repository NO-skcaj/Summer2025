#include "Gyro.h"

#include "studica/AHRS.h"

#include <units/angle.h>

namespace hardware
{

class Navx : Gyro
{
Navx() : m_gyro{studica::AHRS::NavXComType::kMXP_SPI};
{}

frc::Rotation3d GetRotation() override
{
return frc::Rotation3d{units::degree_t{m_gyro.GetYaw()}, 0_deg, 0_deg};
}

frc::Rotation3d GetOffset() override
{
return m_offset;
}

void            ResetYaw() override
{
m_gyro.Reset();
}

void            SetOffset(frc::Rotation3d offset) override
{
m_offset = offset;
}

private:
        studica::AHRS m_gyro;

}

}