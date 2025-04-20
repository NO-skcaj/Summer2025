#pragma once

#include "BaseMotor.h"

#include <iostream>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <constants/CanIds.h>


namespace hardware
{

    struct TalonMotorConfiguration : MotorConfiguration
    {
        // // The brake mode will hold the motor in place when not powered or not
        // enum NeutralMode
        // {
        //     Brake,
        //     Coast
        // };
        // NeutralMode      NeutralMode;
        // units::ampere_t  CurrentLimit;
        // bool             StatorCurrentLimitEnable;
        // double           P;
        // double           I;
        // double           D;
        double      V; // Extra Pid
        double      A;
        units::turns_per_second_t         MotionMagicCruiseVelocity;
        units::turns_per_second_squared_t MotionMagicAcceleration; 
        units::turns_per_second_cubed_t   MotionMagicJerk;
        
    };         

    class TalonFX : public BaseMotor
    {
        public:
            TalonFX(int CANid) : m_motor{CANid, "rio"} {}

            inline void ConfigureMotor(TalonMotorConfiguration config)  // Configure the motor with default settings
            {
                // Create the drive motor configuration
                ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};
                // Add the "Motor Output" section settings
                ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
                motorOutputConfigs.NeutralMode = //
                    config.NeutralMode == hardware::MotorConfiguration::Brake ? ctre::phoenix6::signals::NeutralModeValue::Brake : 
                                                                                ctre::phoenix6::signals::NeutralModeValue::Coast;
                
                // Add the "Current Limits" section settings
                ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
                currentLimitsConfigs.StatorCurrentLimit       = config.CurrentLimit;
                currentLimitsConfigs.StatorCurrentLimitEnable = config.StatorCurrentLimitEnable;

                // Add the "Slot0" section settings
                ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
                slot0Configs.kP = config.P;
                slot0Configs.kI = config.I;
                slot0Configs.kD = config.D;
                slot0Configs.kV = config.V;
                slot0Configs.kA = config.A;
                
                // Configure Motion Magic
                ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = talonFXConfiguration.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = config.MotionMagicCruiseVelocity;
                motionMagicConfigs.MotionMagicAcceleration   = config.MotionMagicAcceleration;
                motionMagicConfigs.MotionMagicJerk           = config.MotionMagicJerk;

                ApplyConfiguration(talonFXConfiguration);
            }

            inline void ConfigureMotor(ctre::phoenix6::configs::TalonFXConfiguration config)  // Configure the motor with default settings
            {
                ApplyConfiguration(config);
            }

            inline void SetSpeed(double motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.Set(motorInput);
            }

            inline void SetSpeed(units::turns_per_second_t motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(motorInput));
            }

            inline void SetSpeed(units::volt_t motorInput) override // output to motor within (-1,1)
            {
                // Set the motor speed and angle
                m_motor.SetVoltage(motorInput);
            }

            inline void SetPosition(units::turn_t motorInput) override // output to motor in turns
            {
                // Set the arm set position
                m_motor.SetControl(m_motionMagicVoltage.WithPosition(motorInput).WithSlot(0));
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return m_motor.GetPosition().GetValue();
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turns
            {
                return m_motor.GetVelocity().GetValue();
            }

        private:

            inline void ApplyConfiguration(ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration)
            {
                ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
                for (int attempt = 0; attempt < Constants::CanIds::MotorConfigurationAttempts; attempt++)
                {
                    // Apply the configuration to the drive motor
                    status = m_motor.GetConfigurator().Apply(talonFXConfiguration);
                    // Check if the configuration was successful
                    if (status.IsOK())
                    break;
                }
                // Determine if the last configuration load was successful
                if (!status.IsOK())
                    std::cout << "***** ERROR: Could not configure TalonFX motor (" << m_motor.GetDeviceID() <<"). Error: " << status.GetName() << std::endl;
            }


            ctre::phoenix6::hardware::TalonFX             m_motor;  // TalonFX motor controller        
            
            ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
    };

}