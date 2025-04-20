#pragma once

#include <limits>
#include <memory>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <photon/estimation/VisionEstimation.h>

#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>

#include <photon/targeting/PhotonPipelineResult.h>

#include <frc/geometry/Pose3d.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <frc/RobotBase.h>

namespace Constants 
{

namespace Vision 
{

    constexpr std::string_view        kCameraName{"PhotonCamera"};

    constexpr frc::Transform3d        kRobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

    const frc::AprilTagFieldLayout    kTagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

    const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};

    const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};

    namespace AprilTagLocations
    {
        constexpr frc::Pose3d one      {657.37_in,  25.80_in, 58.50_in, {126_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d two      {657.37_in, 291.20_in, 58.50_in, {234_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d three    {455.15_in, 317.15_in, 51.25_in, {270_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d four     {365.20_in, 241.64_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
        constexpr frc::Pose3d five     {365.20_in,  75.39_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
        constexpr frc::Pose3d six      {530.49_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d seven    {546.87_in, 158.50_in, 12.13_in, {  0_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d eight    {530.49_in, 186.83_in, 12.13_in, { 60_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d nine     {497.77_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d ten      {481.39_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d eleven   {497.77_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d twelve   {33.51_in,   25.80_in, 58.50_in, { 54_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d thirteen {33.51_in,  291.20_in, 58.50_in, {306_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d fourteen {325.68_in, 241.64_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
        constexpr frc::Pose3d fifteen  {325.68_in,  75.39_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
        constexpr frc::Pose3d sixteen  {235.73_in,  -0.15_in, 51.25_in, { 90_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d seventeen{160.39_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d eighteen {144.00_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d nineteen {160.39_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
        constexpr frc::Pose3d twenty   {193.10_in, 186.83_in, 12.13_in, {60_deg,  0_deg,  0_deg}};
        constexpr frc::Pose3d twentyone{209.49_in, 158.50_in, 12.13_in, {0_deg,   0_deg,  0_deg}};
        constexpr frc::Pose3d twentytwo{193.10_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};
 
        constexpr frc::Pose3d tags[22] = {
            one,
            two,
            three,
            four,
            five,
            six,
            seven,
            eight,
            nine,
            ten,
            eleven,
            twelve,
            thirteen,
            fourteen,
            fifteen,
            sixteen,
            seventeen,
            eighteen,
            nineteen,
            twenty,
            twentyone,
            twentytwo
        };

        constexpr std::span<const frc::Pose3d> tagsSpan{std::begin(tags), std::end(tags)};
    }
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