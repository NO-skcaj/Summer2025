#pragma once

#include <limits>
#include <memory>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <photon/estimation/VisionEstimation.h>

#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>

#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/RobotBase.h>

#include "constants/Vision.h"


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

        frc::Pose3d GetNearestTag(frc::Pose3d robotPose);

    private:
        photon::PhotonPoseEstimator photonEstimator{
            Constants::Vision::TagLayout,
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants::Vision::RobotToCam};

        photon::PhotonCamera camera{Constants::Vision::CameraName};

        std::unique_ptr<photon::VisionSystemSim> visionSim;

        std::unique_ptr<photon::SimCameraProperties> cameraProp;

        std::shared_ptr<photon::PhotonCameraSim> cameraSim;

        // The most recent result, cached for calculating std devs
        photon::PhotonPipelineResult m_latestResult;
};