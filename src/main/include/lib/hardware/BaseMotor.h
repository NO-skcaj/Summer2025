#pragma once

#include <memory>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/current.h>


namespace hardware
{

    struct MotorConfiguration
    {
        // The brake mode will hold the motor in place when not powered or not
        enum NeutralMode
        {
            Brake,
            Coast
        };

        NeutralMode      NeutralMode;
        units::ampere_t  CurrentLimit;
        bool             StatorCurrentLimitEnable;
        double           P;
        double           I;
        double           D;
    };

    // This class is used to abstract the motor controller interface
    class BaseMotor
    {
        public:
            virtual void SetSpeed(double                    motorInput) = 0; // output to motor within (-1,1)
            virtual void SetSpeed(units::turns_per_second_t motorInput) = 0; // output to motor within (-1,1)
            virtual void SetSpeed(units::volt_t             motorInput) = 0; // output to motor within (-1,1)

            virtual void SetPosition(units::turn_t motorInput) =0;  // output to motor in turns

            virtual units::turn_t GetPosition() = 0;  // Returns the position of the motor in turns
            
            virtual units::turns_per_second_t GetVelocity() = 0;  // Returns the velocity of the motor in turns per second

            virtual void SimPeriodic() = 0; // Simulate the motor
    };

}