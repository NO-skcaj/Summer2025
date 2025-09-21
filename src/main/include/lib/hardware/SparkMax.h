#pragma once

#include "BaseMotor.h"

#include <iostream>

#include <frc/system/plant/DCMotor.h>

#include <rev/SparkSim.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


namespace hardware
{
    /// @brief Configuration structure for the SparkMax motor controller.
    /// @details This structure contains the configuration parameters for the SparkMax motor controller.
    // // The brake mode will hold the motor in place when not powered or not
    // enum MotorConfiguration::NeutralMode
    // {
    //     Brake,
    //     Coast
    // };
    /// @param NeutralMode                   NeutralMode
    /// @param CurrentLimit                  units::ampere_t
    /// @param StatorCurrentLimitEnable      bool
    /// @param P                             double
    /// @param I                             double
    /// @param D                             double
    /// @param Inverted                      bool
    /// @param PositionWrappingEnabled       bool
    /// @param PositionWrappingInputRangeMin double
    /// @param PositionWrappingInputRangeMax double
    struct SparkMaxConfiguration : MotorConfiguration
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
        bool   Inverted;
        bool   PositionWrappingEnabled;
        double PositionWrappingInputRangeMin;
        double PositionWrappingInputRangeMax;
    };

    class SparkMax : public BaseMotor
    {
        public:
            inline SparkMax(int CANid, bool isBrushless) : 
                m_motor{CANid, isBrushless ? rev::spark::SparkMax::MotorType::kBrushless : rev::spark::SparkMax::MotorType::kBrushed},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()}
                // m_motorSim{m_motor, frc::DCMotor::NEO(1)}
            {}

            inline void ConfigureMotor(SparkMaxConfiguration config)  // Configure the motor with default settings
            {
                // Configure the angle motor
                static rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .Inverted(config.Inverted)
                    .SetIdleMode(config.NeutralMode == hardware::MotorConfiguration::NeutralMode::Brake ? rev::spark::SparkBaseConfig::IdleMode::kBrake : rev::spark::SparkBaseConfig::IdleMode::kCoast)
                    .SmartCurrentLimit(config.CurrentLimit.value());

                // sparkMaxConfig.encoder
                //     .PositionConversionFactor(config.PositionConversionFactor)
                //     .VelocityConversionFactor(config.VelocityConversionFactor); // not using these as to stay consistent with the TalonFX vice versa

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(config.P, config.I, config.D)
                    .PositionWrappingEnabled(config.PositionWrappingEnabled)
                    .PositionWrappingInputRange(config.PositionWrappingInputRangeMin, config.PositionWrappingInputRangeMax);

                // Write the configuration to the motor controller
                m_motor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
            }

            inline void SetSpeed(double motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput, rev::spark::SparkMax::ControlType::kDutyCycle);
            }

            inline void SetSpeed(units::turns_per_second_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kVelocity);
            }

            inline void SetSpeed(units::volt_t motorInput) override // output to motor within (-1,1)
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kVoltage);
            }

            inline void SetPosition(units::turn_t motorInput) override // output to motor in turns
            {
                m_turnClosedLoopController.SetReference(motorInput.value(), rev::spark::SparkMax::ControlType::kPosition);
            }

            inline units::turn_t GetPosition() override // Returns the position of the motor in turns
            {
                return units::turn_t{m_angleEncoder.GetPosition()};
            }

            inline units::turns_per_second_t GetVelocity() override // Returns the velocity of the motor in turns
            {
                return units::turns_per_second_t{m_angleEncoder.GetVelocity()};
            }

            inline void OffsetEncoder(units::turn_t offset) // Returns the current of the motor in amps
            {
                m_angleEncoder.SetPosition(offset.value());
            }

            inline void SimPeriodic() override // Simulate the motor
            {
                
            }

        private:
            rev::spark::SparkMax                  m_motor;                    // SparkMax motor controller
            rev::spark::SparkRelativeEncoder      m_angleEncoder;             // Relative encoder onboard the sparkmax
            rev::spark::SparkClosedLoopController m_turnClosedLoopController; // PID Controller for SparkMax

            // rev::spark::SparkSim                  m_motorSim;                 // Simulation model for SparkMax
    };
}