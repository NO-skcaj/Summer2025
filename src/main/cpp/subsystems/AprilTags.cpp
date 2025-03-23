#include "subsystems/AprilTags.h"

static void VisionThread();

#pragma region AprilTags (constructor)
/// @brief Constructor for the AprilTags class.
AprilTags::AprilTags() : m_instance(nt::NetworkTableInstance::GetDefault()),
                         m_aprilTagsTable(m_instance.GetTable("apriltags")),
                         m_aprilTagsIntegerArrayTopic(m_aprilTagsTable->GetIntegerArrayTopic("tags")),
                         m_aprilTagsIntegerArraySubscriber(m_aprilTagsIntegerArrayTopic.Subscribe({}))
{
    SetName("AprilTags");

    SetSubsystem("AprilTags");

    // Need to run the vision program in a separate thread
    std::thread visionThread(VisionThread);
    visionThread.detach();
}
#pragma endregion

#pragma region Periodic
// This method will be called once per scheduler run
void AprilTags::Periodic()
{
    // Read Limelight information to Network Table
    auto targetPose   = LimelightHelpers::getTargetPose_CameraSpace("limelight");
    auto targetPose3d = LimelightHelpers::toPose3D(targetPose);

    frc::SmartDashboard::PutNumber("Limelight Target Pose X",          targetPose3d.X().to<double>());
    frc::SmartDashboard::PutNumber("Limelight Target Pose Y",          targetPose3d.Y().to<double>());
    frc::SmartDashboard::PutNumber("Limelight Target Pose Z",          targetPose3d.Z().to<double>());
    frc::SmartDashboard::PutNumber("Limelight Target Pose Rotation X", targetPose3d.Rotation().X().to<double>() * 180.0 / M_PI);
    frc::SmartDashboard::PutNumber("Limelight Target Pose Rotation Y", targetPose3d.Rotation().Y().to<double>() * 180.0 / M_PI);
    frc::SmartDashboard::PutNumber("Limelight Target Pose Rotation Z", targetPose3d.Rotation().Z().to<double>() * 180.0 / M_PI);

    auto pipelineLatency = LimelightHelpers::getLatency_Pipeline("limelight");
    frc::SmartDashboard::PutNumber("Limelight Pipeline Latency", pipelineLatency);

    // Read Apriltag information to Network Table
    auto foundAprilTags = m_aprilTagsIntegerArraySubscriber.Get();

    // Clear the AprilTag vector
    m_detectedAprilTags.clear();

    // Determine if any AprilTags were detected
    for (auto aprilTagIndex = 0; aprilTagIndex < foundAprilTags.size(); aprilTagIndex++)
    {
        AprilTagInformation aprilTagInformation;

        // Get the AprilTag identification
        aprilTagInformation.Identification = foundAprilTags[aprilTagIndex];

        // Retrieve the entry using the GetEntry method
        nt::NetworkTableEntry entry = m_aprilTagsTable->GetEntry(fmt::format("pose_{}", aprilTagInformation.Identification));

        // Read the value from the entry
        std::vector<double> poseValues = entry.GetDoubleArray({});

        // Check if the entry has values and use them
        if (!poseValues.empty())
        {
            aprilTagInformation.X         = poseValues[0];
            aprilTagInformation.Y         = poseValues[1];
            aprilTagInformation.Z         = poseValues[2];
            aprilTagInformation.rotationX = poseValues[3];
            aprilTagInformation.rotationY = poseValues[4];
            aprilTagInformation.rotationZ = poseValues[5];
        }

        // Add the AprilTag to the found vector
        m_detectedAprilTags.push_back(aprilTagInformation);
    }
}
#pragma endregion

#pragma region GetTag
/// @brief Method to get the specified AprilTag by identification.
/// @param id The AprilTag identificaton to get.
/// @param aprilTagInformation Reference to return the AprilTag information.
/// @return true to indicate that the AprilTag information is available.
bool AprilTags::GetTag(int id, AprilTagInformation &aprilTagInformation)
{
    // Search through all found AprilTags
    for (auto aprilTagIndex = 0; aprilTagIndex < m_detectedAprilTags.size(); aprilTagIndex++)
    {
        // Determine if the specified AprilTag was detected
        if (id == m_detectedAprilTags[aprilTagIndex].Identification)
        {
            // Copy the AprilTag information and return
            aprilTagInformation = m_detectedAprilTags[aprilTagIndex];
            return true;
        }
    }

    // Indicate that the specified AprilTag was not found
    return false;
}
#pragma endregion

#pragma region GetClosestAprilTag
/// @brief Method to get the closest AprilTag.
/// @param aprilTagInformation Reference to return the AprilTag information.
/// @return true to indicate that the AprilTag information is available.
AprilTagInformation AprilTags::GetClosestAprilTag()
{
    AprilTagInformation aprilTagInformation;

    // Indicate that no AprilTag was found
    aprilTagInformation.Found = false;

    // Determine if any AprilTags have been found
    if (m_detectedAprilTags.size() == 0)
        return aprilTagInformation;

    // If only one AprilTag is detected, then return it
    else if (m_detectedAprilTags.size() == 1)
    {
        // Copy the AprilTag information
        aprilTagInformation = m_detectedAprilTags[0];
        aprilTagInformation.Found = true;
        return aprilTagInformation;
    }

    // Assume the first AprilTag is the closest
    aprilTagInformation = m_detectedAprilTags[0];
    aprilTagInformation.Found = true;

    // Search through the remainin found AprilTags
    for (auto aprilTagIndex = 1; aprilTagIndex < m_detectedAprilTags.size(); aprilTagIndex++)
    {
        // Determine if the new AprilTag is closer
        if (m_detectedAprilTags[aprilTagIndex].Z < aprilTagInformation.Z)
        {
            aprilTagInformation = m_detectedAprilTags[aprilTagIndex];
            aprilTagInformation.Found = true;
        }
    }

    // Indicate that an AprilTag was detected
    return aprilTagInformation;
}
#pragma endregion

#pragma region VisionThread
/// @brief Thread to run the AprilTag vision processing.
static void VisionThread()
{
    frc::AprilTagDetector             detector;                                                // Declaring AprilTagDetector
    cv::Mat                           mat;                                                     // Matrix representing the image
    cv::Mat                           grayMat;                                                 // Matrix representing the gray scale image
    std::vector<int64_t>              detectedTags;                                            // Vector containing all the tags currently detected.
    cv::Scalar                        outlineColor                   = cv::Scalar(0, 255, 0);  // Color of the lines outlineing the april tags
    cv::Scalar                        crossColor                     = cv::Scalar(0, 0, 255);  // Color of the cross in the center of the april tags
    nt::NetworkTableInstance          instance                       = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> aprilTagsTable                 = instance.GetTable("apriltags");
    nt::IntegerArrayTopic             aprilTagsIntegerArrayTopic     = aprilTagsTable->GetIntegerArrayTopic("tags");
    nt::IntegerArrayPublisher         aprilTagsIntegerArrayPublisher= aprilTagsIntegerArrayTopic.Publish();

    // look for tag36h11
    detector.AddFamily("tag36h11", ApriltagConstants::NumberOfBitsCorrected);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // Source of magic numbers: (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    frc::AprilTagPoseEstimator::Config poseEstConfig =
    {
        .tagSize = units::length::inch_t(ApriltagConstants::LengthOfTagsInches),
        .fx = ApriltagConstants::CameraWidthInPixels,     // The width of the camera in pixels
        .fy = ApriltagConstants::CameraHeightInPixels,    // The Hight of the camera in pixels
        .cx = ApriltagConstants::CameraCenterXInPixels,   // The center focus point of the camera x pos
        .cy = ApriltagConstants::CameraCenterYInPixels    // The center focus point of the camera y pos
    };

    // Making estimator to estimate the tag's possition wich somehow makes it more accurate.
    frc::AprilTagPoseEstimator estimator = frc::AprilTagPoseEstimator(poseEstConfig);

    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution
    camera.SetVideoMode(cs::VideoMode::PixelFormat::kMJPEG,
                        ApriltagConstants::CameraResolutionWidth,
                        ApriltagConstants::CameraResolutionHeight,
                        ApriltagConstants::CameraFramesPerSecond);

    // Get a CvSink. This is what actualy gets the frame from the camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream = frc::CameraServer::PutVideo("Detected", ApriltagConstants::CameraResolutionWidth, ApriltagConstants::CameraResolutionHeight);

    while (true)
    {
        // Tell the CvSink to grab a frame from the camera and put it in the source mat.
        // If there is an error notify the output.
        if (cvSink.GrabFrame(mat) == 0) // If GrabFrame returns 0 then it couldn't get a frame from the camera.
        {
            // Send the output the error.
            outputStream.NotifyError(cvSink.GetError());
            //std::cout << "***** ERROR: Not able to send stream" << std::endl;

            // If we don't have a frame from the camera, continueing the while loop would be pointless
            continue;
        }

        // Make grayMat the same as mat(The current Frame), but in grayscale.
        cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

        // Stores a bunch of information on the grayMat's size, but we just use it for width and height.
        cv::Size grayMatSize = grayMat.size();
        frc::AprilTagDetector::Results detections = detector.Detect(grayMatSize.width, grayMatSize.height, grayMat.data);

        // Have not seen any tags yet, so detectedTags should not have anything in it.
        detectedTags.clear();

        // If there are no detections then there is no point in looping through all the items in it.
        if (detections.size() > 0)
        {
            // Looping through all the detections. detection is the current detecion that the loop is on.
            for (const frc::AprilTagDetection *detection : detections)
            {
                // Remember the detected tag (add to vector)
                detectedTags.push_back(detection->GetId());

                // Draw lines around the tag
                for (auto cornerIndex = 0; cornerIndex <= (ApriltagConstants::NumberOfAprilTagCorners - 1); cornerIndex++)
                {
                    int nextCornerIndex = (cornerIndex + 1) % ApriltagConstants::NumberOfAprilTagCorners;

                    // Getting the first point that we will draw a line from
                    const frc::AprilTagDetection::Point corner = detection->GetCorner(cornerIndex);

                    // Gettig the second line that we will draw a line to
                    const frc::AprilTagDetection::Point nextCorner = detection->GetCorner(nextCornerIndex);

                    // Drawing the line between the two points(corners) that we just got:
                    line(mat, cv::Point(corner.x, corner.y), cv::Point(nextCorner.x, nextCorner.y), outlineColor, ApriltagConstants::AprilTagLineWitdh);
                }

                // mark the center of the tag
                const frc::AprilTagDetection::Point c = detection->GetCenter();
                int crossSize = 10;

                // Drawing the cross in the center of the tag
                line(mat, cv::Point(c.x - crossSize, c.y), cv::Point(c.x + crossSize, c.y), crossColor, 2);
                line(mat, cv::Point(c.x, c.y - crossSize), cv::Point(c.x, c.y + crossSize), crossColor, 2);

                // identify the tag, and putting the tag name next to the cross in the center of the April tag
                putText(mat, std::to_string(detection->GetId()), cv::Point(c.x + crossSize, c.y), cv::FONT_HERSHEY_SIMPLEX, 1, crossColor, 3);

                // Determine pose
                frc::Transform3d pose = estimator.Estimate(*detection);

                // Add the pose to the Network Table
                frc::Rotation3d rotation = pose.Rotation();
                aprilTagsTable->GetEntry(fmt::format("pose_{}", detection->GetId())).SetDoubleArray(
                        {{ pose.X().value(),
                           pose.Y().value(),
                           pose.Z().value(),
                           rotation.X().value(),
                           rotation.Y().value(),
                           rotation.Z().value() }});
            }
        }

        // Put list of tags onto Network Table
        aprilTagsIntegerArrayPublisher.Set(detectedTags);

        // Give the output stream a new image to display
        outputStream.PutFrame(mat);
    }
}
#pragma endregion