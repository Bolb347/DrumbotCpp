#pragma once
#include <vector>
#include <frc2/command/SubsystemBase.h>
#include "robot/vision/BlobCam.h"
#include "robot/vision/PhotonBlobCam.h"
#include "CommandSwerveDrivetrain.h"

namespace subsystems {

class Vision : public frc2::SubsystemBase {
public:
    explicit Vision(CommandSwerveDrivetrain* drivetrain);

    void ResetOrientation();
    void StopIMUData();
    void Periodic() override;

private:
    std::vector<vision::BlobCam> m_cameras;
    std::vector<vision::PhotonBlobCam> m_photonCameras;
};

} // namespace subsystems
