#pragma once
#include <functional>

namespace util {

template <typename T>
using ChangeListener = std::function<void(T, T)>;

} // namespace util
