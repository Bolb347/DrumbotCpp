#pragma once
#include <vector>
#include <functional>
#include "ChangeListener.h"

namespace util {

template <typename T>
class ObservableField {
public:
    explicit ObservableField(T initialValue) : m_value(std::move(initialValue)) {}

    T GetValue() const { return m_value; }

    void SetValue(T newValue) {
        T old = m_value;
        m_value = newValue;
        if (old != newValue) {
            for (auto& listener : m_listeners) {
                listener(old, newValue);
            }
        }
    }

    void AddListener(ChangeListener<T> listener) {
        m_listeners.push_back(std::move(listener));
    }

    void RemoveListener(ChangeListener<T> listener) {
        // Note: std::function doesn't support equality comparison;
        // callers should manage listener lifetimes or use indices if removal is needed.
    }

private:
    T m_value;
    std::vector<ChangeListener<T>> m_listeners;
};

} // namespace util
