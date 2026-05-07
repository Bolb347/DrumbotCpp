#include "robot/vision/BlobCam.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/MathUtil.h>

// NOTE: Requires the C++ LimelightHelpers header from https://github.com/LimelightVision/limelightlib-wpicpp
#include "robot/LimelightHelpers.h"

#include "robot/RobotContainer.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include <cmath>

namespace vision {

BlobCam::BlobCam(std::string cameraName,
                 subsystems::CommandSwerveDrivetrain* drivetrain,
                 frc::Pose3d cameraPoseRelBot,
                 ctre::phoenix6::hardware::Pigeon2* pigeon)
    : m_cameraName(std::move(cameraName)),
      m_drivetrain(drivetrain),
      m_pigeon(pigeon)
{
    LimelightHelpers::setCameraPose_RobotSpace(
        m_cameraName,
        cameraPoseRelBot.X().value(),
        cameraPoseRelBot.Y().value(),
        cameraPoseRelBot.Z().value(),
        units::degree_t{cameraPoseRelBot.Rotation().X()}.value(),
        units::degree_t{cameraPoseRelBot.Rotation().Y()}.value(),
        units::degree_t{cameraPoseRelBot.Rotation().Z()}.value()
    );

    LimelightHelpers::SetIMUMode(m_cameraName, 1);
    LimelightHelpers::SetRobotOrientation(
        m_cameraName,
        RobotContainer::isBlueAlliance.GetValue() ? 0.0 : 180.0,
        0, 0, 0, 0, 0);
}

void BlobCam::ResetOrientation() {
    LimelightHelpers::SetIMUMode(m_cameraName, 1);
    LimelightHelpers::SetRobotOrientation(
        m_cameraName,
        RobotContainer::isBlueAlliance.GetValue() ? 0.0 : 180.0,
        0, 0, 0, 0, 0);
}

void BlobCam::StopIMUData() {
    m_hasReset = true;
}

void BlobCam::Gather() {
    double currentGyroDegrees = m_drivetrain->GetState().Pose.Rotation().Degrees().value();
    LimelightHelpers::SetRobotOrientation_NoFlush(m_cameraName, currentGyroDegrees, 0, 0, 0, 0, 0);

    LimelightHelpers::PoseEstimate poseEstimate =
        LimelightHelpers::getBotPoseEstimate_wpiBlue(m_cameraName);

    if (poseEstimate.tagCount <= 1) return;

    if (frc::DriverStation::IsAutonomousEnabled()) {
        frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
        double poseError = currentPose.Translation().Distance(
            poseEstimate.pose.Translation()).value();

        if (poseError > 0.5 && poseEstimate.tagCount >= 2) {
            m_drivetrain->AddVisionMeasurement(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                {0.00001, 0.00001, 0.00001});
            return;
        }
    }

    m_drivetrain->AddVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        {0.7, 0.7, 0.5});
}

} // namespace vision
