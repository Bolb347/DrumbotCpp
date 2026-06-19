#include "robot/fetcher/TargetFetcher.h"

namespace fetcher {

std::shared_ptr<nt::NetworkTable> TargetFetcher::s_headingFetcher =
    nt::NetworkTableInstance::GetDefault().GetTable("TargetPos");

frc::Translation2d TargetFetcher::target{};

TargetFetcher::TargetFetcher() {}

frc::Translation2d TargetFetcher::GetMostRecentResult() {
    double x = s_headingFetcher->GetEntry("x").GetDouble(0.0);
    double y = s_headingFetcher->GetEntry("y").GetDouble(0.0);
    return {units::meter_t{x}, units::meter_t{y}};
}

void TargetFetcher::Periodic() {
    target = GetMostRecentResult();
}

} // namespace fetcher
