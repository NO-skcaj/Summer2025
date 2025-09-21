#pragma once

#include "Gyro.h"

#include "studica/AHRS.h"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>

#include <units/angle.h>
#include <units/angular_velocity.h>


namespace hardware
{

// NavX class to support the NavX gyro
class Navx : Gyro
{
    public:
    
        static Navx* GetInstance();

        // Get the rotation of the gyro
        frc::Rotation3d GetRotation() override;

        // Get the offset of the gyro
        frc::Rotation3d GetOffset() override;

        // Reset the yaw of the gyro
        void ResetYaw() override;

        // Set the offset of the gyro
        void SetOffset(frc::Rotation3d offset);

        void SimPeriodic(units::radians_per_second_t rate); // updates in sim

    private:
        // Constructor for the NavX class
        Navx() : m_gyro   {studica::AHRS::NavXComType::kMXP_SPI},
                 m_offset {0_deg, 0_deg, 0_deg},
                 m_simYaw {0.0},
                 m_simRate{0.0} 
        {}

        static Navx* m_instance; // The singleton instance

        studica::AHRS m_gyro;     // NavX gyro

        frc::Rotation3d m_offset; // The offset of the gyro

        units::radian_t             m_simYaw;
        units::radians_per_second_t m_simRate;
};

}