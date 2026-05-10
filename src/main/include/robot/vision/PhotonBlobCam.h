#pragma once
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/DriverStation.h>
#include <string>
#include <optional>
#include "robot/subsystems/CommandSwerveDrivetrain.h"

namespace vision {
class PhotonBlobCam {
 public:
  PhotonBlobCam(const std::string& cameraName,
                subsystems::CommandSwerveDrivetrain* drivetrain,
                const frc::Pose3d& cameraPoseRelBot);

  // Polls camera, estimates pose, and feeds to drivetrain
  void Gather();

  photon::PhotonPipelineResult GetLatestResult() const { return m_latestResult; }
  bool HasTarget() const { return m_latestResult.HasTargets(); }

 private:
  static constexpr double kMaxAmbiguity = 0.2;

  inline static const frc::AprilTagFieldLayout kTagLayout{
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};

  photon::PhotonCamera m_camera;
  photon::PhotonPoseEstimator m_poseEstimator;
  subsystems::CommandSwerveDrivetrain* m_drivetrain;
  photon::PhotonPipelineResult m_latestResult;
};
}