#pragma once

namespace util {

inline double Clamp(double x, double min, double max) {
    return x < min ? min : (x > max ? max : x);
}

} // namespace util
