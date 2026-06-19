#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace fetcher {

class TargetFetcher : public frc2::SubsystemBase {
public:
    TargetFetcher();

    static frc::Translation2d GetMostRecentResult();
    void Periodic() override;

    static frc::Translation2d target;

private:
    static std::shared_ptr<nt::NetworkTable> s_headingFetcher;
};

} // namespace fetcher
