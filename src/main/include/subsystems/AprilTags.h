#pragma once

#include <cameraserver/CameraServer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/TimedRobot.h>

#include "Constants.h"

#pragma region ApriltagConstants
namespace ApriltagConstants
{
    // Magic camera values:
    // Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    constexpr auto CameraWidthInPixels     = 699.3778103158814;
    constexpr auto CameraHeightInPixels    = 677.7161226393544;
    constexpr auto CameraCenterXInPixels   = 345.6059345433618;
    constexpr auto CameraCenterYInPixels   = 207.12741326228522;

    constexpr auto CameraResolutionWidth   = 640;
    constexpr auto CameraResolutionHeight  = 480;
    constexpr auto CameraFramesPerSecond   =  30;

    constexpr auto AprilTagLineWitdh       =   2;
    constexpr auto NumberOfAprilTagCorners =   4;
    constexpr auto NumberOfBitsCorrected   =   1;

    constexpr auto LengthOfTagsInches      = 6.5;

    constexpr auto RobotCameraOffset       = 0_m;  // -0.305_m - Side camera mount
    constexpr auto RobotCameraAngleOffset  = 0_deg;
}
#pragma endregion

struct AprilTagInformation
{
    bool   Found;
    int    Identification;
    double X;
    double Y;
    double Z;
    double rotationX;
    double rotationY;
    double rotationZ;
};

class AprilTags : public frc2::SubsystemBase
{
    public:

        explicit            AprilTags();

        void                Periodic() override;

        bool                GetTag(int id, AprilTagInformation &aprilTagInformation);

        AprilTagInformation GetClosestAprilTag();

    private:

        nt::NetworkTableInstance          m_instance;
        std::shared_ptr<nt::NetworkTable> m_aprilTagsTable;

        nt::IntegerArrayTopic             m_aprilTagsIntegerArrayTopic;
        nt::IntegerArraySubscriber        m_aprilTagsIntegerArraySubscriber;

        std::vector<AprilTagInformation>  m_detectedAprilTags;
};
