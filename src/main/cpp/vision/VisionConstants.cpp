#include "robot/vision/VisionConstants.h"
#include <units/angle.h>
#include <units/length.h>

namespace vision {

const std::vector<frc::Pose3d> VisionConstants::cameraPosesInBotSpace = {
    frc::Pose3d{
        frc::Translation3d{units::inch_t{11.15}, units::inch_t{0.0},  units::inch_t{17.1}},
        frc::Rotation3d{0_deg, 21_deg, 0_deg}
    }
};

const std::vector<frc::Pose3d> VisionConstants::photonCameraPosesInBotSpace = {
    frc::Pose3d{
        frc::Translation3d{units::inch_t{11.15}, units::inch_t{0.0},  units::inch_t{17.1}},
        frc::Rotation3d{0_deg, 21_deg, 0_deg}
    }
};

const std::vector<std::string> VisionConstants::cameraNames = {
    "limelight-a"
};

const std::vector<std::string> VisionConstants::photonCameraNames = {
    "photon-a"
};

} // namespace vision
