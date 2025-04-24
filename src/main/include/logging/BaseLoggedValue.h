#pragma once

#include <typeinfo>

#include <string>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableBuilder.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>


class SwerveSendable : public wpi::Sendable {
    virtual std::span<double> GetData();

    void InitSendable(wpi::SendableBuilder &builder) override 
    {
        builder.SetSmartDashboardType("SwerveDrive");

        builder.AddDoubleProperty("Front Left Angle",     [this] -> double { return this->GetData()[0];}, [](double s){});
        builder.AddDoubleProperty("Front Left Velocity",  [this] -> double { return this->GetData()[1];}, [](double s){});
        builder.AddDoubleProperty("Front Right Angle",    [this] -> double { return this->GetData()[2];}, [](double s){});
        builder.AddDoubleProperty("Front Right Velocity", [this] -> double { return this->GetData()[3];}, [](double s){});
        builder.AddDoubleProperty("Back Left Angle",      [this] -> double { return this->GetData()[4];}, [](double s){});
        builder.AddDoubleProperty("Back Left Velocity",   [this] -> double { return this->GetData()[5];}, [](double s){});
        builder.AddDoubleProperty("Back Right Angle",     [this] -> double { return this->GetData()[6];}, [](double s){});
        builder.AddDoubleProperty("Back Right Velocity",  [this] -> double { return this->GetData()[7];}, [](double s){});
        builder.AddDoubleProperty("Robot Angle",          [this] -> double { return this->GetData()[8];}, [](double s){});
    }
};

template <typename type>
class BaseLoggedValue
{
    public:
        virtual type Get()           = 0
        virtual void Set(type value) = 0
        virtual void Log()           = 0
};