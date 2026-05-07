#pragma once
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace fetcher {

class DensityFetcher {
public:
    struct DensityResult {
        double relativeHeading;
        double timestamp;
        double area;

        DensityResult(double relativeHeading, double timestamp, double area)
            : relativeHeading(relativeHeading), timestamp(timestamp), area(area) {}
    };

    static DensityResult GetMostRecentResult();

private:
    static std::shared_ptr<nt::NetworkTable> s_headingFetcher;
};

} // namespace fetcher
