#include "tracker_test_controller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "noname_controller.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <utility>
#include "./fire_controller.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include <chrono>
#include <memory>
#include <tuple>
using namespace rmcs_auto_aim::fire_controller;
class TrackerTestController::Impl {
public:
    /**
         * @brief Constructs an Impl instance and initializes internal state.
         *
         * Initializes the internal tracker pointer to `nullptr` and sets
         * `enemy_high_speed_mode` to `false`.
         */
        Impl()
        : tracker_(nullptr)
        , enemy_high_speed_mode(false) {};

    /**
 * @brief Checks whether the internal tracker instance is set.
 *
 * @return `true` if an internal tracker is set, `false` otherwise.
 */
bool check() { return tracker_ != nullptr; };
    /**
     * @brief Computes whether firing is permitted and produces a target position from the current tracker state.
     *
     * Uses the tracked car state and armor orientations to select a firing direction, determines a target
     * 3D position offset from the car, and evaluates alignment-based permission to fire.
     *
     * @param sec Time in seconds (used to query time-dependent armor/car estimates).
     * @param tf Reference frame information used for coordinate interpretation.
     * @return std::tuple<bool, rmcs_description::OdomImu::Position>
     *         - first: `true` if firing is permitted based on armor alignment and tracker state, `false` otherwise.
     *         - second: computed target `Position` in the described frame.
     */
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};
        // std::cerr << tracker_->omega() << std::endl;
        if (!enemy_high_speed_mode && abs(tracker_->omega()) > 4 * std::numbers::pi)
            enemy_high_speed_mode = true;
        else if (enemy_high_speed_mode && abs(tracker_->omega()) < 3 * std::numbers::pi)
            enemy_high_speed_mode = false;

        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;
        auto [l1, l2]                               = tracker_->get_frame();
        double min_l                                = std::min(l1, l2);
        Eigen::Vector3d position                    = *tracker_->get_car_position();
        position                                    = position - position.normalized() * min_l;
        double max                                  = -1e7;
        int index                                   = 0;
        auto armors                                 = tracker_->get_armor(sec);
        auto pos_norm = Eigen::Vector2d(position.x(), position.y()).normalized();

        for (int i = 0; i < 4; i++) {

            auto armor_x = (*armors[i].rotation * Eigen::Vector3d::UnitX());
            auto len     = pos_norm.dot(Eigen::Vector2d(armor_x.x(), armor_x.y()).normalized());
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        position.z() = armors[(index + 3) % 4].position->z();
        if (index % 2 == 0) {
            position.z() = armors[index].position->z();
            if (max > 0.99)
                fire_permission = true;
        }
        ret_pos = rmcs_description::OdomImu::Position(position);

        return {fire_permission, ret_pos};
    }

    /**
 * @brief Install or replace the internal CarTracker used by the controller.
 *
 * Replaces any previously set tracker with the provided instance; passing a null
 * shared_ptr clears the stored tracker.
 *
 * @param tracker Shared pointer to a tracker::CarTracker to be stored (ownership taken).
 */
void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }
    /**
 * @brief Gets the current estimated angular velocity of the tracked car.
 *
 * @return double Angular velocity in radians per second as provided by the associated tracker.
 */
double get_omega() { return tracker_->omega(); }
    /**
 * @brief Retrieves the timestamp associated with the current tracker state.
 *
 * @return std::chrono::steady_clock::time_point The tracker's last recorded timestamp.
 */
std::chrono::steady_clock::time_point get_timestamp() { return tracker_->get_timestamp(); }

private:
    std::shared_ptr<tracker::CarTracker> tracker_;
    bool enemy_high_speed_mode = false;
};

/**
 * @brief Update the internal controller state and compute firing permission and target position.
 *
 * @param sec Time step in seconds used for the controller update.
 * @param tf Reference frame information used to compute target geometry.
 * @return std::tuple<bool, rmcs_description::OdomImu::Position>
 *         First element: `true` if firing is permitted, `false` otherwise.
 *         Second element: computed target position in world coordinates.
 */
[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    TrackerTestController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

/**
 * @brief Sets the tracker instance used by this controller.
 *
 * Stores the provided CarTracker so subsequent UpdateController calls use its state.
 *
 * @param tracker Shared pointer to the CarTracker to be used by the controller.
 */
void TrackerTestController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

/**
 * @brief Retrieves the current angular velocity estimate from the active tracker.
 *
 * @return double Current angular velocity (omega) in radians per second.
 */
double TrackerTestController::get_omega() { return pimpl_->get_omega(); }
/**
 * @brief Copy constructor that performs a deep copy of the controller's implementation.
 *
 * Constructs a new TrackerTestController by creating an independent copy of the source controller's
 * internal implementation, preserving the tracked state and configuration.
 *
 * @param car_tracker Source controller to copy.
 */
TrackerTestController::TrackerTestController(const TrackerTestController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

/**
 * @brief Determines whether the controller currently holds a valid tracker.
 *
 * @return true if an internal tracker instance is set, false otherwise.
 */
bool TrackerTestController::check() { return pimpl_->check(); }

/**
 * @brief Retrieves the last timestamp recorded by the associated tracker.
 *
 * @return std::chrono::steady_clock::time_point The tracker's most recent update time as a steady_clock time_point.
 */
std::chrono::steady_clock::time_point TrackerTestController::get_timestamp() {
    return pimpl_->get_timestamp();
}

/**
 * @brief Constructs a TrackerTestController and initializes its internal implementation.
 *
 * Initializes the opaque implementation pointer used to encapsulate controller state and logic.
 */
TrackerTestController::TrackerTestController() { pimpl_ = std::make_unique<Impl>(); }
/**
 * @brief Destroys the TrackerTestController and releases its internal resources.
 *
 * Cleans up the controller's implementation (pimpl) and any associated resources.
 */
TrackerTestController::~TrackerTestController() = default;