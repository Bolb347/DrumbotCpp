#include "robot/vision/BlobCam.h"
#include "robot/LimelightHelpers.h"
#include "robot/RobotContainer.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>

namespace vision {

BlobCam::BlobCam(std::string cameraName,
                 subsystems::CommandSwerveDrivetrain* drivetrain,
                 frc::Translation3d turretPivotInBotSpace,
                 frc::Pose3d cameraPoseAtZero,
                 ctre::phoenix6::hardware::Pigeon2* pigeon,
                 std::function<double()> turretAngleGetter)
    : m_cameraName(std::move(cameraName))
    , m_drivetrain(drivetrain)
    , m_pigeon(pigeon)
    , m_cameraPoseAtZero(cameraPoseAtZero)
    , m_turretAngleGetter(std::move(turretAngleGetter))
    , m_turretPivot(turretPivotInBotSpace)
{
    UpdateCameraPose();

    LimelightHelpers::SetIMUMode(m_cameraName, 1);
    LimelightHelpers::SetRobotOrientation(
        m_cameraName,
        RobotContainer::isBlueAlliance.GetValue() ? 0.0 : 180.0,
        0, 0, 0, 0, 0);
}

void BlobCam::UpdateCameraPose() {
    units::radian_t turretYaw = units::degree_t(m_turretAngleGetter());
    frc::Rotation3d turretRot{0_rad, 0_rad, turretYaw};

    frc::Translation3d camRelPivot = m_cameraPoseAtZero.Translation() - m_turretPivot;

    frc::Translation3d rotatedTranslation = m_turretPivot + camRelPivot.RotateBy(turretRot);

    frc::Pose3d rotatedPose{
        rotatedTranslation,
        m_cameraPoseAtZero.Rotation() + turretRot
    };

    LimelightHelpers::setCameraPose_RobotSpace(
        m_cameraName,
        rotatedPose.X().value(),
        rotatedPose.Y().value(),
        rotatedPose.Z().value(),
        units::degree_t{rotatedPose.Rotation().X()}.value(),
        units::degree_t{rotatedPose.Rotation().Y()}.value(),
        units::degree_t{rotatedPose.Rotation().Z()}.value()
    );
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
    UpdateCameraPose();   // <-- recompute every cycle

    double currentGyroDegrees =
        m_drivetrain->GetState().Pose.Rotation().Degrees().value();
    LimelightHelpers::SetRobotOrientation_NoFlush(
        m_cameraName, currentGyroDegrees, 0, 0, 0, 0, 0);

    LimelightHelpers::PoseEstimate poseEstimate =
        LimelightHelpers::getBotPoseEstimate_wpiBlue(m_cameraName);

    if (poseEstimate.tagCount <= 1) return;

    if (frc::DriverStation::IsAutonomousEnabled()) {
        frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
        double poseError = currentPose.Translation()
            .Distance(poseEstimate.pose.Translation()).value();

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