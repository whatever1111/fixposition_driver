/**
 * @file
 * @brief Implementation of the stamp helpers (see time_utils.hpp)
 */

/* LIBC/STL */
#include <algorithm>
#include <cmath>
#include <vector>

/* PACKAGE */
#include "fixposition_driver_m20/time_utils.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {
constexpr double NANO = 1e9;

//! Median of a copy of the samples
double Median(const std::deque<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    std::vector<double> sorted(values.begin(), values.end());
    const std::size_t middle = sorted.size() / 2;
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(middle), sorted.end());
    const double upper = sorted[middle];
    if ((sorted.size() % 2) != 0) {
        return upper;
    }
    // Even count: average the two central values
    const double lower = *std::max_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(middle));
    return 0.5 * (lower + upper);
}
}  // namespace

Stamp Stamp::FromSec(const double seconds) {
    Stamp stamp;
    if (!std::isfinite(seconds) || (seconds <= 0.0)) {
        return stamp;
    }
    double whole = std::floor(seconds);
    double nano = std::round((seconds - whole) * NANO);
    if (nano >= NANO) {
        whole += 1.0;
        nano -= NANO;
    }
    stamp.sec = static_cast<int32_t>(whole);
    stamp.nsec = static_cast<uint32_t>(nano);
    return stamp;
}

// ---------------------------------------------------------------------------------------------------------------------

HostClockRetimer::HostClockRetimer(const double jump_threshold_sec, const std::size_t window)
    : jump_threshold_{jump_threshold_sec}, window_{std::max<std::size_t>(1, window)} {}

void HostClockRetimer::Reset() { deltas_.clear(); }

double HostClockRetimer::Offset() const { return Median(deltas_); }

double HostClockRetimer::Apply(const double sensor_sec, const double host_now, bool* jumped) {
    if (jumped != nullptr) {
        *jumped = false;
    }
    if (!std::isfinite(sensor_sec) || !std::isfinite(host_now)) {
        return sensor_sec;
    }

    const double delta = host_now - sensor_sec;
    if (!deltas_.empty() && (std::fabs(delta - Median(deltas_)) > jump_threshold_)) {
        // The sensor's clock stepped — the old offset describes a clock that no longer exists
        deltas_.clear();
        if (jumped != nullptr) {
            *jumped = true;
        }
    }

    deltas_.push_back(delta);
    while (deltas_.size() > window_) {
        deltas_.pop_front();
    }

    return sensor_sec + Median(deltas_);
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
