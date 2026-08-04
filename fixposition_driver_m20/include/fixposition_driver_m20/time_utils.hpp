/**
 * @file
 * @brief Stamp arithmetic and sensor-to-host clock retiming — ROS-free
 */

#ifndef __FIXPOSITION_DRIVER_M20_TIME_UTILS_HPP__
#define __FIXPOSITION_DRIVER_M20_TIME_UTILS_HPP__

/* LIBC/STL */
#include <cstddef>
#include <cstdint>
#include <deque>

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief A seconds/nanoseconds stamp
 *
 * The three output topics must carry byte-identical stamps, so stamps are moved around as integers
 * and only converted to double where arithmetic needs it.
 */
struct Stamp {
    int32_t sec = 0;
    uint32_t nsec = 0;

    double ToSec() const { return static_cast<double>(sec) + (static_cast<double>(nsec) * 1e-9); }

    /**
     * @brief Build a stamp from seconds, normalising the nanosecond part
     *
     * Negative times (which cannot happen with an epoch stamp) clamp to zero.
     */
    static Stamp FromSec(double seconds);

    bool operator==(const Stamp& other) const { return (sec == other.sec) && (nsec == other.nsec); }
};

/**
 * @brief Translates sensor stamps onto the host clock axis
 *
 * Tracks a sliding median of (host now - sensor stamp) and adds it to the sensor stamp. The median
 * rejects transport jitter; a residual beyond the jump threshold means the sensor's clock actually
 * stepped, so the estimate is dropped and re-converges from the new samples.
 *
 * Not thread-safe.
 */
class HostClockRetimer {
   public:
    /**
     * @param[in] jump_threshold_sec  residual against the current median that counts as a clock jump
     * @param[in] window              number of samples in the sliding median
     */
    explicit HostClockRetimer(double jump_threshold_sec, std::size_t window = 50);

    /**
     * @brief Correct one sensor stamp
     *
     * @param[in]  sensor_sec  the sensor's stamp [s]
     * @param[in]  host_now    the host clock at reception [s]
     * @param[out] jumped      set to true when this sample reset the estimate; may be null
     *
     * @returns the stamp expressed on the host clock [s]
     */
    double Apply(double sensor_sec, double host_now, bool* jumped);

    void Reset();
    std::size_t Size() const { return deltas_.size(); }

    /**
     * @brief Current correction [s] (0 when nothing has been seen yet)
     */
    double Offset() const;

   private:
    std::deque<double> deltas_;
    double jump_threshold_;
    std::size_t window_;
};

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_TIME_UTILS_HPP__
