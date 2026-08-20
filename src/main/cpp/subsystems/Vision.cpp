#include "robot/subsystems/Vision.h"
#include "robot/vision/VisionConstants.h"

using namespace subsystems;

Vision::Vision(CommandSwerveDrivetrain* drivetrain) {
    const auto& names  = vision::VisionConstants::cameraNames;
    const auto& poses  = vision::VisionConstants::cameraPosesInBotSpace;
    for (size_t i = 0; i < names.size(); ++i) {
        m_cameras.emplace_back(names[i], drivetrain, poses[i], &drivetrain->GetPigeon2());
    }
}

void Vision::ResetOrientation() {
    for (auto& cam : m_cameras) cam.ResetOrientation();
}

void Vision::StopIMUData() {
    for (auto& cam : m_cameras) cam.StopIMUData();
}

void Vision::Periodic() {
    for (auto& cam : m_cameras) cam.Gather();
}
