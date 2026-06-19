#include "robot/fetcher/DensityFetcher.h"

namespace fetcher {

std::shared_ptr<nt::NetworkTable> DensityFetcher::s_headingFetcher =
    nt::NetworkTableInstance::GetDefault().GetTable("DensityHeading");

DensityFetcher::DensityResult DensityFetcher::GetMostRecentResult() {
    double x         = s_headingFetcher->GetEntry("x").GetDouble(0.0);
    double area      = s_headingFetcher->GetEntry("area").GetDouble(0.0);
    double timestamp = s_headingFetcher->GetEntry("timestamp").GetDouble(0.0);
    return {x, area, timestamp};
}

} // namespace fetcher
