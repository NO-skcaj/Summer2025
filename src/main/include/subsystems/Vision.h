#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>

#include <photon/targeting/PhotonPipelineResult.h>

#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/RobotBase.h>

namespace Constants 
{

namespace Vision 
{

    constexpr std::string_view kCameraName{"PhotonCamera"};

    constexpr frc::Transform3d kRobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

    const frc::AprilTagFieldLayout kTagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

    const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};

    const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};
}

}

class Vision 
{
    public:
        Vision();

        photon::PhotonPipelineResult GetLatestResult() { return m_latestResult; }

        std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose();

        Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose);

        frc::Field2d& GetSimDebugField() { return visionSim->GetDebugField(); }

        void SimPeriodic(frc::Pose2d robotSimPose);

        void ResetSimPose(frc::Pose2d pose);

    private:
        photon::PhotonPoseEstimator photonEstimator{
            Constants::Vision::kTagLayout,
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants::Vision::kRobotToCam};

        photon::PhotonCamera camera{Constants::Vision::kCameraName};

        std::unique_ptr<photon::VisionSystemSim> visionSim;

        std::unique_ptr<photon::SimCameraProperties> cameraProp;

        std::shared_ptr<photon::PhotonCameraSim> cameraSim;

        // The most recent result, cached for calculating std devs
        photon::PhotonPipelineResult m_latestResult;
};