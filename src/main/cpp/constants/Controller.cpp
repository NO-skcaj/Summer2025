#include "subsystems/Drivetrain.h"

#include "Constants/Controller.h"

namespace Constants
{

namespace ChassisPose
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{Constants::ChassisPose::MaxAngularSpeed,
                                                                                        Constants::ChassisPose::MaxAngularAcceleration};
}

}