#pragma once

#include <utility>

#include <frc/MathUtil.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include <cameraserver/CameraServer.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

// Subsystems
#include "subsystems/Climb.h"
#include "subsystems/Drivetrain/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/Leds.h"

#include "Controller.h"

// Commands
#include "commands/SimpleCommands.h"

#include "Constants/Controller.h"
#include "Constants/ChassisPoseAprilTag.h"
#include "Constants/ChassisPoseAutonomous.h"
#include "Constants/GripperPose.h"

#include "lib/logging/LoggingManager.h"
#include "lib/logging/LoggedSwerve.h"
#include "lib/logging/LoggerFactory.h"


/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer*       GetInstance();

        // Method to get a pointer to the selected autonomous command
        frc2::Command*             GetAutonomousCommand();

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer                *m_robotContainer;

        // Instantiate the robot subsystems
        Climb*                                m_climb;
        Drivetrain*                           m_drivetrain;
        Gripper*                              m_gripper;
        Odometry*                             m_odometry;
        Leds*                                 m_leds;
        
        // Controllers
        Controller                            m_controller;

        frc::SendableChooser<frc2::Command*>  m_autoChooser;

        cs::VideoSink                         m_server;
        cs::UsbCamera                         m_usbCamera;
        cs::VideoSink                         m_limelightFeed;

        // Logging
        LoggingManager*                      m_loggingManager;

        double                               m_loggedPotentiometer;

};
