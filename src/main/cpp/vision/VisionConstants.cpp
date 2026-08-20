#include "robot/vision/VisionConstants.h"
#include <units/angle.h>
#include <units/length.h>

namespace vision {

const std::vector<frc::Pose3d> VisionConstants::cameraPosesInBotSpace = {
    // Front camera
    frc::Pose3d{
        frc::Translation3d{units::inch_t{11.15}, units::inch_t{0.0},  units::inch_t{17.1}},
        frc::Rotation3d{0_deg, 21_deg, 0_deg}
    },
    // Right camera
    frc::Pose3d{
        frc::Translation3d{units::inch_t{-1.5},  units::inch_t{14.986}, units::inch_t{8.312}},
        frc::Rotation3d{0_deg, 18_deg, -90_deg}
    },
    // Left camera
    frc::Pose3d{
        frc::Translation3d{units::inch_t{-1.5},  units::inch_t{-14.986}, units::inch_t{8.312}},
        frc::Rotation3d{0_deg, 17.8_deg, 90_deg}
    },
};

const std::vector<std::string> VisionConstants::cameraNames = {
    "limelight-a",
    "limelight-b",
    "limelight-c",
};

} // namespace vision
