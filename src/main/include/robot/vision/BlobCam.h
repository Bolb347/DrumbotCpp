#pragma once
#include <string>
#include <frc/geometry/Pose3d.h>
#include <ctre/phoenix6/Pigeon2.hpp>

// Forward declarations
namespace subsystems { class CommandSwerveDrivetrain; }

namespace vision {

class BlobCam {
public:
    BlobCam(std::string cameraName,
            subsystems::CommandSwerveDrivetrain* drivetrain,
            frc::Pose3d cameraPoseRelBot,
            ctre::phoenix6::hardware::Pigeon2* pigeon);

    void ResetOrientation();
    void StopIMUData();
    void Gather();

private:
    std::string m_cameraName;
    subsystems::CommandSwerveDrivetrain* m_drivetrain;
    ctre::phoenix6::hardware::Pigeon2* m_pigeon;
    bool m_hasReset = false;
};

} // namespace vision
