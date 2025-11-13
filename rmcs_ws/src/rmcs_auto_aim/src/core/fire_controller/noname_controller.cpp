#include "noname_controller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <iostream>
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <utility>

using namespace rmcs_auto_aim::fire_controller;
class NoNameController::Impl {
public:
    /**
         * @brief Default-constructs the Impl, initializing internal state.
         *
         * Initializes the internal tracker pointer to null and sets enemy high-speed mode to false.
         */
        Impl()
        : tracker_(nullptr)
        , enemy_high_speed_mode(false) {};
    /**
 * @brief Retrieve the timestamp held by the current CarTracker.
 *
 * @return std::chrono::steady_clock::time_point The steady_clock timestamp from the tracked CarTracker.
 */
std::chrono::steady_clock::time_point get_timestamp() { return tracker_->get_timestamp(); }

    /**
 * @brief Checks whether an internal CarTracker has been set.
 *
 * @return `true` if a tracker is set, `false` otherwise.
 */
bool check() { return tracker_ != nullptr; };
    /**
     * @brief Determine target position and whether firing is permitted for the current frame.
     *
     * Updates internal high-speed state based on the tracked object's angular rate and
     * computes a target position and a fire permission flag using either a high-speed
     * selection strategy or a normal predictive armor-targeting strategy.
     *
     * In detail:
     * - Switches to high-speed mode when |omega| > 4π and back to normal when |omega| < 3π.
     * - In high-speed mode, selects an armor target by maximizing directional alignment
     *   between the vehicle-relative direction and each armor's rotated Z axis, applies
     *   a positional offset and uses the selected armor's Z for the result; fire permission
     *   is granted only if the alignment exceeds 0.97 (0.7 used as a fallback threshold).
     * - In normal mode, predicts an armor target via the ArmorTarget predictor and grants
     *   fire permission only if the predicted target is sufficiently aligned with the camera X direction (dot >= 0.97).
     *
     * @param sec Elapsed time or prediction horizon in seconds used for target prediction.
     * @param tf Reference transform/frame used to evaluate camera direction and predictions.
     * @return std::tuple<bool, rmcs_description::OdomImu::Position>
     *         First element: `true` if firing is permitted, `false` otherwise.
     *         Second element: the selected or predicted target position.
     */
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf& tf) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};
        // std::cerr << tracker_->omega() << std::endl;
        if (!enemy_high_speed_mode && abs(tracker_->omega()) > 4 * std::numbers::pi)
            enemy_high_speed_mode = true;
        else if (enemy_high_speed_mode && abs(tracker_->omega()) < 3 * std::numbers::pi)
            enemy_high_speed_mode = false;

        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;
        // enemy_high_speed_mode                       = true;
        if (enemy_high_speed_mode) {
            auto [l1, l2]            = tracker_->get_frame();
            double min_l             = std::min(l1, l2);
            Eigen::Vector3d position = *tracker_->get_car_position();
            position                 = position
                     + Eigen::AngleAxisd(
                           -std::numbers::pi / 6 * (tracker_->omega() > 0 ? 1 : -1),
                           Eigen::Vector3d::UnitZ())
                           * -position.normalized() * min_l;
            double max    = -1e7;
            int index     = 0;
            auto armors   = tracker_->get_armor(sec);
            auto pos_norm = Eigen::Vector2d(position.x(), position.y()).normalized();

            for (int i = 0; i < 4; i++) {

                auto armor_x = (*armors[i].rotation * Eigen::Vector3d::UnitZ());
                auto len     = pos_norm.dot(Eigen::Vector2d(armor_x.x(), armor_x.y()).normalized());
                if (len > max) {
                    index = i;
                    max   = len;
                }
            }
            position.z() = armors[index].position->z();
            if (max < (enemy_high_speed_mode ? 0.97 : 0.7)) {
                fire_permission = false;
                ret_pos         = rmcs_description::OdomImu::Position(position);
            } else {
                fire_permission = true;

                ret_pos = armors[index].position;
            }
        } else {
            ret_pos = tracker::armor::ArmorTarget{tracker::CarTracker(*tracker_)}.Predict(sec, tf);

            auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

            auto armor_x = ret_pos;

            auto len = camera_x->dot(Eigen::Vector3d(*armor_x));

            if (len < 0.97) {
                fire_permission = false;
            } else {
                fire_permission = true;
            }
        }

        return {fire_permission, ret_pos};
    }

    /**
 * @brief Replaces the controller's CarTracker instance used for targeting.
 *
 * Stores the provided shared pointer as the active tracker; passing a null pointer clears it.
 *
 * @param tracker Shared pointer to the tracker::CarTracker to set as the current tracker.
 */
void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }
    /**
 * @brief Get the tracked vehicle's angular velocity.
 *
 * @return double The current angular velocity (omega) in radians per second.
 */
double get_omega() { return tracker_->omega(); }

private:
    std::shared_ptr<tracker::CarTracker> tracker_;
    bool enemy_high_speed_mode = false;
};

/**
 * @brief Update targeting logic and determine whether firing is permitted and where to aim.
 *
 * Processes tracker state and the provided transform to select a target position and decide
 * firing permission, switching behavior when the tracked vehicle enters high-speed mode.
 *
 * @param sec Time step in seconds since the last update.
 * @param tf Current transform used for aiming and alignment checks.
 * @return std::tuple<bool, rmcs_description::OdomImu::Position> `true` if firing is permitted, `false` otherwise; the second element is the selected target position, or a zeroed Position when no valid target is available.
 */
[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    NoNameController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

/**
 * @brief Attach or replace the internal CarTracker used by the controller.
 *
 * Stores the provided shared tracker in the controller's implementation; passing a null
 * shared_ptr clears the current tracker.
 *
 * @param tracker Shared pointer to a tracker::CarTracker to be used for targeting (may be null).
 */
void NoNameController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

/**
 * @brief Retrieves the current angular velocity reported by the active tracker.
 *
 * @return The tracker's angular velocity (omega) in radians per second.
 */
double NoNameController::get_omega() { return pimpl_->get_omega(); }
/**
 * @brief Constructs a NoNameController as a deep copy of another instance.
 *
 * Creates a new controller whose internal implementation state is duplicated from
 * the provided `car_tracker`, including its tracker pointer and mode flags.
 *
 * @param car_tracker Source controller to copy from.
 */
NoNameController::NoNameController(const NoNameController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

/**
 * @brief Indicates whether the controller has an active tracker.
 *
 * @return `true` if an internal CarTracker is set, `false` otherwise.
 */
bool NoNameController::check() { return pimpl_->check(); }

/**
 * @brief Retrieve the controller's last tracked timestamp.
 *
 * @return std::chrono::steady_clock::time_point The most recent timestamp reported by the internal tracker.
 */
std::chrono::steady_clock::time_point NoNameController::get_timestamp() {
    return pimpl_->get_timestamp();
}
/**
 * @brief Constructs a NoNameController and initializes its private implementation.
 *
 * Creates and owns a new Impl instance used to manage tracking and targeting state.
 */
NoNameController::NoNameController() { pimpl_ = std::make_unique<Impl>(); }
/**
 * @brief Destroy the NoNameController and release its owned resources.
 *
 * Defaulted destructor that allows compiler-generated teardown of member objects.
 */
NoNameController::~NoNameController() = default;