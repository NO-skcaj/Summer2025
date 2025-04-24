#pragma once

#include <typeinfo>

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/geometry/Pose2d.h>

#include "logging/BaseLoggedValue.h"


template <class type>
class LoggedValue : BaseLoggedValue<type>
{
    public:
        LoggedValue(const LoggedValue&) = default;
        LoggedValue(LoggedValue&&)      = default;

        // Constructor to initialize the value
        explicit LoggedValue(type value, std::string_view name) : m_value(value), m_name(name)
        {
            switch (typeid(type).name())
            {
            case typeid(int).name() || typeid(float).name() || typeid(double).name():
                m_logFunction = [this]() {
                    // Log the int value
                    frc::SmartDashboard::PutNumber(double(m_name), m_value);
                };
                break;

            case typeid(std::string).name():
                m_logFunction = [this]() {
                    // Log the string value
                    frc::SmartDashboard::PutString(m_name, m_value);
                };
                break;

            case typeid(bool).name():
                m_logFunction = [this]() {
                    // Log the string value
                    frc::SmartDashboard::PutBoolean(m_name, m_value);
                };
                break;
            
            default:
                assert(false && "Unsupported type for LoggedValue");
                break;
            }
        };

        type Get() override
        {
            return m_value;
        }

        void Set(type value) override
        {
            m_value = value;
            Log();
        }

        void Log() override
        {
            m_logFunction();
        }

    private:
        type                  m_value;
        std::string_view      m_name;

        std::function<void()> m_logFunction;
};