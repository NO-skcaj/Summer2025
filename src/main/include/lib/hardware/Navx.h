#pragma once

#include "Gyro.h"

#include "studica/AHRS.h"

#include <units/angle.h>


namespace hardware
{

// NavX class to support the NavX gyro
class Navx : Gyro
{
    public:

        // Constructor for the NavX class
        Navx() : m_gyro{studica::AHRS::NavXComType::kMXP_SPI},
                 m_offset{0_deg, 0_deg, 0_deg} {}

        // Get the rotation of the gyro
        frc::Rotation3d GetRotation() override
        {
            return m_gyro.GetRotation3d() + m_offset;
        }

        // Get the offset of the gyro
        frc::Rotation3d GetOffset() override
        {
        return m_offset;
        }

        // Reset the yaw of the gyro
        void ResetYaw() override
        {
            m_gyro.Reset();
        }

        // Set the offset of the gyro
        void SetOffset(frc::Rotation3d offset) override
        {
            m_offset = offset;
        }

    private:

        studica::AHRS m_gyro;     // NavX gyro

        frc::Rotation3d m_offset; // The offset of the gyro
};

}