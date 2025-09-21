#include "lib/hardware/Navx.h"


using namespace hardware;

Navx* Navx::m_instance = nullptr;

Navx* Navx::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new Navx();
    }
    return m_instance;
}

frc::Rotation3d Navx::GetRotation()
{
    if (frc::RobotBase::IsSimulation())
    {
        return frc::Rotation3d{m_simYaw, 0_deg, 0_deg} + m_offset;
    } else
    {
        return m_gyro.GetRotation3d() + m_offset;
    }
}

frc::Rotation3d Navx::GetOffset()
{
    return m_offset;
}

void Navx::ResetYaw()
{
    m_gyro.Reset();
}

void Navx::SetOffset(frc::Rotation3d offset)
{
    m_offset = offset;
}

void Navx::SimPeriodic(units::radians_per_second_t rate)
{
    m_simRate = rate;
    m_simYaw += m_simRate * 0.02_s; // assuming 20ms loop time
}