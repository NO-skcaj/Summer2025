#pragma once

#include <typeinfo>
#include <string>
#include <functional>
#include <cassert>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/geometry/Pose2d.h>

#include "lib/logging/BaseLoggedValue.h"
#include "lib/logging/LoggedSwerve.h"

class LoggedValue
{
    public:
        static BaseLoggedValue<frc::Field2d*> CreateLoggedValue(std::string_view name, frc::Field2d* value)
        {
            return BaseLoggedValue<frc::Field2d*>(LoggedField(value, name));
        }

        static BaseLoggedValue<double> CreateLoggedValue(std::string_view name, double value)
        {
            return BaseLoggedValue<double>(LoggedNumber(value, name));
        }

        static BaseLoggedValue<std::string_view> CreateLoggedValue(std::string_view name, std::string_view value)
        {
            return BaseLoggedValue<std::string_view>(LoggedString(value, name));
        }

        static BaseLoggedValue<bool> CreateLoggedValue(std::string_view name, bool value)
        {
            return BaseLoggedValue<bool>(LoggedBool(value, name));
        }

    private:
        class LoggedNumber : public BaseLoggedValue<double>
        {   public:
                LoggedNumber(double value, std::string_view name)
                    : m_value(value), m_name(name), m_logFunction{[this]() {frc::SmartDashboard::PutNumber(m_name, m_value);}} {}

                void                   Set(double value) { m_value = value; }
                std::function<void()>* Get()             { return &m_logFunction; }
                double                 GetValue()        { return m_value; }
            private:
                int                   m_value;
                std::string_view      m_name;
                std::function<void()> m_logFunction;
        };

        class LoggedString : public BaseLoggedValue<std::string_view>
        {
            public:
                LoggedString(std::string_view value, std::string_view name)
                    : m_value(value), m_name(name), m_logFunction{[this]() {frc::SmartDashboard::PutString(m_name, m_value);}} {}
                    
                void                   Set(std::string_view value) { m_value = value; }
                std::function<void()>* Get()                       { return &m_logFunction; }
                std::string_view       GetValue()                  { return m_value; }

            private:
                std::string_view      m_value;
                std::string_view      m_name;
                std::function<void()> m_logFunction;
        };

        class LoggedBool : public BaseLoggedValue<bool>
        {
            public:
                LoggedBool(bool value, std::string_view name)
                    : m_value(value), m_name(name), m_logFunction{[this]() {frc::SmartDashboard::PutBoolean(m_name, m_value);}} {}
                    
                void                   Set(bool value) { m_value = value; }
                std::function<void()>* Get()           { return &m_logFunction; }
                bool                   GetValue()      { return m_value; }
            private:
                bool                  m_value;
                std::string_view      m_name;
                std::function<void()> m_logFunction;
        };

        class LoggedField : public BaseLoggedValue<frc::Field2d*>
        {
            public:
                LoggedField(frc::Field2d* value, std::string_view name) 
                            : m_value(value), m_name(name), m_logFunction{[this]() {frc::SmartDashboard::PutData(m_name, m_value);}} {}

                std::function<void()>* Get() { return &m_logFunction; }

                void Set(frc::Field2d* value) {}

                frc::Field2d* GetValue() { return m_value; }


            private:
                frc::Field2d*         m_value;
                std::string_view      m_name;
                std::function<void()> m_logFunction;

        };
};