#include "robot/vision/PhotonBlobCam.h"
#include <frc/geometry/Pose2d.h>

using namespace vision;

PhotonBlobCam::PhotonBlobCam(const std::string& cameraName,
                               subsystems::CommandSwerveDrivetrain* drivetrain,
                               const frc::Pose3d& cameraPoseRelBot)
    : m_camera{cameraName},
      m_poseEstimator{
          kTagLayout,
          photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
          frc::Transform3d{frc::Pose3d{}, cameraPoseRelBot}
      },
      m_drivetrain{drivetrain} {
  m_poseEstimator.SetMultiTagFallbackStrategy(
      photon::PoseStrategy::LOWEST_AMBIGUITY);
}

void PhotonBlobCam::Gather() {
  for (const auto& result : m_camera.GetAllUnreadResults()) {
    m_latestResult = result;

    auto estimate = m_poseEstimator.Update(result);
    if (!estimate.has_value()) continue;

    int tagCount = static_cast<int>(estimate->targetsUsed.size());
    if (tagCount < 1) continue;

    if (tagCount == 1) {
      double ambiguity = result.GetBestTarget().GetPoseAmbiguity();
      if (ambiguity > kMaxAmbiguity) continue;
    }

    frc::Pose2d estimatedPose2d = estimate->estimatedPose.ToPose2d();

    double avgDist = 0.0;
    for (const auto& tgt : estimate->targetsUsed) {
      auto tagPose = kTagLayout.GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        avgDist += tagPose->ToPose2d().Translation()
            .Distance(estimatedPose2d.Translation()).value();
      }
    }
    avgDist /= tagCount;

    if (tagCount == 1 && avgDist > 4.0) continue;

    frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
    double poseError = currentPose.Translation()
        .Distance(estimatedPose2d.Translation()).value();

    if (frc::DriverStation::IsAutonomousEnabled() && poseError > 0.5 && tagCount >= 2) {
      m_drivetrain->AddVisionMeasurement(
          estimatedPose2d,
          estimate->timestamp,
          {0.00001, 0.00001, 0.00001});
      continue;
    }

    wpi::array<double, 3> stdDevs = tagCount >= 2
        ? wpi::array<double, 3>{0.7, 0.7, 0.5}
        : wpi::array<double, 3>{1.5, 1.5, 1.0};
    double scale = 1.0 + (avgDist * avgDist / 30.0);
    stdDevs[0] *= scale;
    stdDevs[1] *= scale;
    stdDevs[2] *= scale;

    m_drivetrain->AddVisionMeasurement(
        estimatedPose2d,
        estimate->timestamp,
        stdDevs);
  }
}