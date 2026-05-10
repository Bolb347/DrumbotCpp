#pragma once
#include <vector>
#include <string>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace vision {

struct VisionConstants {
    static const std::vector<frc::Pose3d> cameraPosesInBotSpace;
    static const std::vector<std::string> cameraNames;
    static const std::vector<frc::Pose3d> photonCameraPosesInBotSpace;
    static const std::vector<std::string> photonCameraNames;
};

} // namespace vision
