#pragma once

#include <iostream>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableBuilder.h>


class LoggedSwerve : public wpi::Sendable {
    virtual std::span<double> GetData() = 0;

    void InitSendable(wpi::SendableBuilder &builder) override 
    {
        builder.SetSmartDashboardType("SwerveDrive");
        
        builder.AddDoubleProperty("Front Left Angle",     [this] { return this->GetData()[0]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Front Left Velocity",  [this] { return this->GetData()[1]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Front Right Angle",    [this] { return this->GetData()[2]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Front Right Velocity", [this] { return this->GetData()[3]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Back Left Angle",      [this] { return this->GetData()[4]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Back Left Velocity",   [this] { return this->GetData()[5]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Back Right Angle",     [this] { return this->GetData()[6]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Back Right Velocity",  [this] { return this->GetData()[7]; }, [](double s){ std::cout << s << '\n'; });
        builder.AddDoubleProperty("Robot Angle",          [this] { return this->GetData()[8]; }, [](double s){ std::cout << s << '\n'; });
    }
};

