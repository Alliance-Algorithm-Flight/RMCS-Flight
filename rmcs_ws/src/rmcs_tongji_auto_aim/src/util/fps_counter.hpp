/**
 * @file src/util/fps_counter.hpp
 * @brief FPS counter utility
 */

#pragma once

#include <chrono>

namespace rmcs_tongji_auto_aim::util {

/**
 * @brief 简单的 FPS 计数器
 */
class FPSCounter {
public:
    /**
         * @brief Construct a new FPSCounter and initialize internal state.
         *
         * Initializes the frame count to 0 and records the current steady clock time
         * as the reference timestamp for FPS measurement. The stored FPS value is
         * initialized to its default (0.0).
         */
        FPSCounter()
        : frame_count_(0)
        , last_time_(std::chrono::steady_clock::now()) {}

    /**
     * @brief Record a single frame and update the stored frames-per-second when at least one second has elapsed.
     *
     * Increments the internal frame count and, if 1000 milliseconds or more have passed since the last update,
     * computes and stores a new FPS value, resets the frame count, and updates the last-sample time.
     *
     * @return `true` if at least 1000 milliseconds have elapsed and the internal FPS value was updated, `false` otherwise.
     */
    bool count() {
        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_time_).count();

        if (elapsed >= 1000) {
            fps_ = frame_count_ * 1000.0 / elapsed;
            frame_count_ = 0;
            last_time_ = now;
            return true;
        }
        return false;
    }

    /**
 * @brief Returns the most recently measured frames per second.
 *
 * @return double Current FPS value in frames per second.
 */
    double getFPS() const { return fps_; }

private:
    int frame_count_;
    double fps_{0.0};
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace rmcs_tongji_auto_aim::util