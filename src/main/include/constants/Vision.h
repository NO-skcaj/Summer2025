#pragma once

#include <frc/geometry/Pose3d.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>


namespace Constants 
{

namespace Vision 
{

    constexpr std::string_view        CameraName{"PhotonCamera"};

    constexpr frc::Transform3d        RobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

    const frc::AprilTagFieldLayout    TagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

    const Eigen::Matrix<double, 3, 1> SingleTagStdDevs{4, 4, 8};

    const Eigen::Matrix<double, 3, 1> MultiTagStdDevs{0.5, 0.5, 1};

    namespace AprilTagLocations
    {
        // constexpr frc::Pose3d One      {657.37_in,  25.80_in, 58.50_in, {126_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Two      {657.37_in, 291.20_in, 58.50_in, {234_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Three    {455.15_in, 317.15_in, 51.25_in, {270_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Four     {365.20_in, 241.64_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
        // constexpr frc::Pose3d Five     {365.20_in,  75.39_in, 73.54_in, {  0_deg, 0_deg, 30_deg}};
        // constexpr frc::Pose3d Six      {530.49_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Seven    {546.87_in, 158.50_in, 12.13_in, {  0_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Eight    {530.49_in, 186.83_in, 12.13_in, { 60_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Nine     {497.77_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Ten      {481.39_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Eleven   {497.77_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Twelve   {33.51_in,   25.80_in, 58.50_in, { 54_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Thirteen {33.51_in,  291.20_in, 58.50_in, {306_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Fourteen {325.68_in, 241.64_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
        // constexpr frc::Pose3d Fifteen  {325.68_in,  75.39_in, 73.54_in, {180_deg, 0_deg, 30_deg}};
        // constexpr frc::Pose3d Sixteen  {235.73_in,  -0.15_in, 51.25_in, { 90_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Seventeen{160.39_in, 130.17_in, 12.13_in, {240_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Eighteen {144.00_in, 158.50_in, 12.13_in, {180_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Nineteen {160.39_in, 186.83_in, 12.13_in, {120_deg, 0_deg,  0_deg}};
        // constexpr frc::Pose3d Twenty   {193.10_in, 186.83_in, 12.13_in, {60_deg,  0_deg,  0_deg}};
        // constexpr frc::Pose3d Twentyone{209.49_in, 158.50_in, 12.13_in, {0_deg,   0_deg,  0_deg}};
        // constexpr frc::Pose3d Twentytwo{193.10_in, 130.17_in, 12.13_in, {300_deg, 0_deg,  0_deg}};
 
        // constexpr frc::Pose3d Tags[22] = { 
        //     One,
        //     Two,
        //     Three,
        //     Four,
        //     Five,
        //     Six,
        //     Seven,
        //     Eight,
        //     Nine,
        //     Ten,
        //     Eleven,
        //     Twelve,
        //     Thirteen,
        //     Fourteen,
        //     Fifteen,
        //     Sixteen,
        //     Seventeen,
        //     Eighteen,
        //     Nineteen,
        //     Twenty,
        //     Twentyone,
        //     Twentytwo
        // };
        
        // constexpr std::span<const frc::Pose3d> Tags{std::begin(Tags), std::end(Tags)};
        
        constexpr frc::Pose2d Pose2dOne       {657.37_in,  25.80_in, {126_deg}};
        constexpr frc::Pose2d Pose2dTwo       {657.37_in, 291.20_in, {234_deg}};
        constexpr frc::Pose2d Pose2dThree     {455.15_in, 317.15_in, {270_deg}};
        constexpr frc::Pose2d Pose2dFour      {365.20_in, 241.64_in, {  0_deg}};
        constexpr frc::Pose2d Pose2dFive      {365.20_in,  75.39_in, {  0_deg}};
        constexpr frc::Pose2d Pose2dSix       {530.49_in, 130.17_in, {300_deg}};
        constexpr frc::Pose2d Pose2dSeven     {546.87_in, 158.50_in, {  0_deg}};
        constexpr frc::Pose2d Pose2dEight     {530.49_in, 186.83_in, { 60_deg}};
        constexpr frc::Pose2d Pose2dNine      {497.77_in, 186.83_in, {120_deg}};
        constexpr frc::Pose2d Pose2dTen       {481.39_in, 158.50_in, {180_deg}};
        constexpr frc::Pose2d Pose2dEleven    {497.77_in, 130.17_in, {240_deg}};
        constexpr frc::Pose2d Pose2dTwelve    {33.51_in,   25.80_in, { 54_deg}};
        constexpr frc::Pose2d Pose2dThirteen  {33.51_in,  291.20_in, {306_deg}};
        constexpr frc::Pose2d Pose2dFourteen  {325.68_in, 241.64_in, {180_deg}};
        constexpr frc::Pose2d Pose2dFifteen   {325.68_in,  75.39_in, {180_deg}};
        constexpr frc::Pose2d Pose2dSixteen   {235.73_in,  -0.15_in, { 90_deg}};
        constexpr frc::Pose2d Pose2dSeventeen {160.39_in, 130.17_in, {240_deg}};
        constexpr frc::Pose2d Pose2dEighteen  {144.00_in, 158.50_in, {180_deg}};
        constexpr frc::Pose2d Pose2dNineteen  {160.39_in, 186.83_in, {120_deg}};
        constexpr frc::Pose2d Pose2dTwenty    {193.10_in, 186.83_in, {60_deg,}};
        constexpr frc::Pose2d Pose2dTwentyone {209.49_in, 158.50_in, {0_deg, }};
        constexpr frc::Pose2d Pose2dTwentytwo {193.10_in, 130.17_in, {300_deg}};

        constexpr frc::Pose2d Tags2d[22] = { 
            Pose2dOne,       Pose2dTwo,
            Pose2dThree,     Pose2dFour, 
            Pose2dFive,      Pose2dSix,
            Pose2dSeven,     Pose2dEight,
            Pose2dNine,      Pose2dTen,
            Pose2dEleven,    Pose2dTwelve,
            Pose2dThirteen,  Pose2dFourteen,
            Pose2dFifteen,   Pose2dSixteen,
            Pose2dSeventeen, Pose2dEighteen,
            Pose2dNineteen,  Pose2dTwenty,
            Pose2dTwentyone, Pose2dTwentytwo
        };

        constexpr std::span<const frc::Pose2d> Pose2dTagsSpan{std::begin(Tags2d), std::end(Tags2d)};
    }
}

}