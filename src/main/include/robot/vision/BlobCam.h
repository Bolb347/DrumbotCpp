#pragma once
#include <functional>
#include <string>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <ctre/phoenix6/Pigeon2.hpp>

namespace subsystems { class CommandSwerveDrivetrain; }

namespace vision {

class BlobCam {
public:
    BlobCam(std::string cameraName,
            subsystems::CommandSwerveDrivetrain* drivetrain,
            frc::Translation3d turretPivotInBotSpace,
            frc::Pose3d cameraPoseRelTurretCenter,   // pose when turret = 0°
            ctre::phoenix6::hardware::Pigeon2* pigeon,
            std::function<double()> turretAngleGetter = [] { return 0.0; });

    void ResetOrientation();
    void StopIMUData();
    void Gather();

private:
    void UpdateCameraPose();

    std::string m_cameraName;
    subsystems::CommandSwerveDrivetrain* m_drivetrain;
    ctre::phoenix6::hardware::Pigeon2* m_pigeon;
    frc::Pose3d m_cameraPoseAtZero;
    frc::Translation3d m_turretPivot;
    std::function<double()> m_turretAngleGetter;
    bool m_hasReset = false;
};

} // namespace vision