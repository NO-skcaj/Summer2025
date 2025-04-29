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

#include "lib/logging/LoggedSwerve.h"

namespace Logger
{
    std::function<void()> CreateLoggedValue(std::string_view name, frc::Field2d* value)
    {
        return [name, value]() { frc::SmartDashboard::PutData(name, value); };
    }

    std::function<void()> CreateLoggedValue(std::string_view name, double* value)
    {
        return [name, value]() {frc::SmartDashboard::PutNumber(name, *value);};
    }

    std::function<void()> CreateLoggedValue(std::string_view name, std::string_view* value)
    {
        return [name, value]() {frc::SmartDashboard::PutString(name, *value);};
    }

    std::function<void()> CreateLoggedValue(std::string_view name, bool* value)
    {
        return [name, value]() {frc::SmartDashboard::PutBoolean(name, *value);};
    }

    std::function<void()> CreateLoggedValue(std::string_view name, LoggedSwerve* value)
    {
        return [name, value]() {frc::SmartDashboard::PutData(name, value);};
    }
};