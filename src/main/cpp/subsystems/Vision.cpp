#include "subsystems/Vision.h"

Vision::Vision() 
{
    photonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

    if (frc::RobotBase::IsSimulation()) 
    {
        visionSim = std::make_unique<photon::VisionSystemSim>("main");

        visionSim->AddAprilTags(Constants::Vision::TagLayout);

        cameraProp = std::make_unique<photon::SimCameraProperties>();

        cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
        cameraProp->SetCalibError(.35, .10);
        cameraProp->SetFPS(15_Hz);
        cameraProp->SetAvgLatency(50_ms);
        cameraProp->SetLatencyStdDev(15_ms);

        cameraSim = std::make_shared<photon::PhotonCameraSim>(&camera, *cameraProp.get());

        visionSim->AddCamera(cameraSim.get(), Constants::Vision::RobotToCam);
        cameraSim->EnableDrawWireframe(true);
    }
}

std::optional<photon::EstimatedRobotPose> Vision::GetEstimatedGlobalPose() {
    std::optional<photon::EstimatedRobotPose> visionEst;

    // Run each new pipeline result through our pose estimator
    for (const auto& result : camera.GetAllUnreadResults()) {
        // cache result and update pose estimator
        auto visionEst = photonEstimator.Update(result);
        m_latestResult = result;

        // In sim only, add our vision estimate to the sim debug field
        if (frc::RobotBase::IsSimulation()) {
        if (visionEst) {
            GetSimDebugField()
                .GetObject("VisionEstimation")
                ->SetPose(visionEst->estimatedPose.ToPose2d());
        } else {
            GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
        }
        }
    }

    return visionEst;
}

Eigen::Matrix<double, 3, 1> Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        Constants::Vision::SingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
        auto tagPose =
            photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(
            estimatedPose.Translation());
        }
    }
    if (numTags == 0) {
        return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
        estStdDevs = Constants::Vision::MultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
        estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max()).finished();
    } else {
        estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
}

void Vision::SimPeriodic(frc::Pose2d robotSimPose) 
{
    this->visionSim->Update(robotSimPose);
}

void Vision::ResetSimPose(frc::Pose2d pose) 
{
    if (frc::RobotBase::IsSimulation()) {
        this->visionSim->ResetRobotPose(pose);
    }
}

frc::Pose3d Vision::GetNearestTag(frc::Pose3d robotPose)
{
    // searches through a list of all tags and returns the closest one
    auto nearestTag = *std::min_element(Constants::Vision::AprilTagLocations::TagsSpan.begin(), Constants::Vision::AprilTagLocations::TagsSpan.end(),
                                        [robotPose, this](frc::Pose3d a, frc::Pose3d b) {
                                            return robotPose.Translation().ToTranslation2d().Distance(a.Translation().ToTranslation2d()) < 
                                                robotPose.Translation().ToTranslation2d().Distance(b.Translation().ToTranslation2d());
                                        });

    return nearestTag;
}