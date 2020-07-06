#ifndef COMMON_TIME_TIME_H_
#define COMMON_TIME_TIME_H_

#include <chrono>
#include "common/macro.h"

namespace common {

namespace time {

/**
 * @class Duration
 * @brief the default Duration is of precision nanoseconds (1e-9 seconds).
 */
using Duration = std::chrono::nanoseconds;

/**
 * @class Timestamp
 * @brief the default timestamp uses std::chrono::system_clock. The
 * system_clock is a system-wide realtime clock.
 */
using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

using nanos = std::chrono::nanoseconds;

/**
 * @brief converts the input duration (nanos) to a 64 bit integer, with
 * the unit specified by PrecisionDuration.
 * @param duration the input duration that needs to be converted to integer.
 * @return an integer representing the duration in the specified unit.
 */
template <typename PrecisionDuration>
int64_t AsInt64(const Duration &duration) {
    return std::chrono::duration_cast<PrecisionDuration>(duration).count();
}

inline double ToSecond(const Timestamp &timestamp) {
    return static_cast<double>(AsInt64<nanos>(timestamp.time_since_epoch())) *
            1e-9;
}

/**
 * @class Clock
 * @brief a singleton clock that can be used to get the current
 * timestamp.
 */
class Clock {
public:
    static constexpr int64_t PRECISION =
            std::chrono::system_clock::duration::period::den /
            std::chrono::system_clock::duration::period::num;

    /// PRECISION >= 1000000 means the precision is at least 1us.
    static_assert(PRECISION >= 1000000,
                  "The precision of the system clock should be at least 1 "
                  "microsecond.");

    /**
   * @brief gets the current timestamp.
   * @return a Timestamp object representing the current time.
   */
    static Timestamp Now() {
        return SystemNow();
    }

    /**
   * @brief gets the current time in second.
   * @return the current time in second.
   */
    static double NowInSeconds() { return ToSecond(Clock::Now()); }

private:

    /**
   * @brief Returns the current timestamp based on the system clock.
   * @return the current timestamp based on the system clock.
   */
    static Timestamp SystemNow() {
        return std::chrono::time_point_cast<Duration>(
                    std::chrono::system_clock::now());
    }

    /// Explicitly disable default and move/copy constructors.
    DECLARE_SINGLETON(Clock);
};


}

}

#endif
