#include <algorithm>
#include <memory>
#include <numbers>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/tracker/car/filter/car_frame_kf.hpp"
#include "core/tracker/car/filter/car_frame_z_kf.hpp"
#include "core/tracker/car/filter/car_kf.hpp"
#include "core/tracker/car/filter/car_movement_kf.hpp"
#include "core/tracker/car/filter/car_pos_kf.hpp"

#include "car_tracker.hpp"

namespace rmcs_auto_aim::tracker {
class CarTracker::Impl {
public:
    /**
     * @brief Constructs the Impl and initializes internal Kalman filters and tracking state.
     *
     * Performs default initialization of car_kf_, car_frame_kf_, car_movement_kf_, and the armor list,
     * and primes the frame tracker by calling Update on car_frame_kf_ with the current frame lengths
     * and a zero time delta.
     */
    Impl()
        : car_kf_()
        , car_frame_kf_()
        , car_movement_kf_()
        , armors_() {
        car_frame_kf_.Update({l1, l2}, {}, 0);
    }
    /**
 * @brief Retrieves the car's planar velocity vector.
 *
 * @return Eigen::Vector2d The 2D velocity where the first element is the x-component and the second is the y-component extracted from the internal state.
 */
Eigen::Vector2d velocity() { return {car_kf_.OutPut()(1), car_kf_.OutPut()(3)}; }

    /**
     * @brief Compute the elapsed time since the last recorded timestamp and update the stored timestamp.
     *
     * @param timestamp The new time point to compare against the previously stored timestamp.
     * @return double Elapsed time in seconds between the provided timestamp and the previous timestamp.
     */
    double get_dt(const std::chrono::steady_clock::time_point& timestamp) {
        auto ret          = std::chrono::duration<double>(timestamp - last_update_time_).count();
        last_update_time_ = timestamp;
        return ret;
    }
    /**
 * @brief Retrieves the most recent recorded update timestamp for the tracker.
 *
 * @return std::chrono::steady_clock::time_point The last recorded update time.
 */
std::chrono::steady_clock::time_point get_timestamp() { return last_update_time_; };

    /**
 * @brief Indicates whether armor plates are currently tracked.
 *
 * @return `true` if armor plates are currently tracked, `false` otherwise.
 */
bool check_armor_tracked() const { return self_update_time_ == 0; }

    /**
 * @brief Gets the current yaw rate estimate from the car movement filter.
 *
 * @return Yaw rate (angular velocity about the Z axis) as a double.
 */
double omega() { return car_movement_kf_.OutPut()(2); }

    /**
     * @brief Incorporates a new pose measurement into the tracker, updating internal filters and cached motion state.
     *
     * Updates the detected yaw, advances the car state and movement Kalman filters with the provided measurement and
     * time delta, refreshes cached velocity and acceleration estimates, and resets the self-update timer.
     *
     * @param zk Pose measurement vector; element index 2 contains the measured yaw.
     * @param dt Time elapsed since the previous update, in seconds.
     */
    void update_car(const CarPosKF::ZVec& zk, const double& dt) {
        detected_yaw = zk(2);
        last_acc_ << car_movement_kf_.OutPut()(0), car_movement_kf_.OutPut()(1);
        last_vel_ << last_acc_;

        car_kf_.Update(zk, {}, dt);
        car_movement_kf_.Update(
            {car_kf_.OutPut()(1), car_kf_.OutPut()(3), car_kf_.OutPut()(5)}, {}, dt);

        last_acc_ << (car_movement_kf_.OutPut()(0) - last_acc_(0)) / dt,
            (car_movement_kf_.OutPut()(1) - last_acc_(1)) / dt;
        last_vel_ << (car_movement_kf_.OutPut()(0) + last_vel_(0)) / 2,
            (car_movement_kf_.OutPut()(1) + last_vel_(1)) / 2;

        self_update_time_ = 0;
    }
    /**
     * @brief Computes the four 3D armor plate poses around the tracked car for a given look-ahead time.
     *
     * Builds a projected car center from the current state, velocity, and acceleration, computes a base yaw,
     * then generates four ArmorPlate3d entries spaced at 90-degree intervals with configured lateral half-lengths
     * and per-plate heights.
     *
     * @param dt Look-ahead time in seconds used to project the car's center and orientation; when zero the last
     *           measured yaw is used.
     * @return std::vector<ArmorPlate3d> A vector containing four ArmorPlate3d objects (front/right/back/left order)
     *         positioned around the projected car center with corresponding rotations and heights.
     */
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        auto X  = Eigen::Vector3d{car_kf_.OutPut()(0), car_kf_.OutPut()(2), car_kf_.OutPut()(4)};
        auto Vx = car_movement_kf_.OutPut();
        if (!check_armor_tracked())
            last_acc_ << 0, 0;

        Eigen::Vector3d center{
            X(0) + last_vel_(0) * dt + last_acc_(0) * dt * dt / 2.0,
            X(1) + last_vel_(1) * dt + last_acc_(1) * dt * dt / 2.0, 0};

        if (last_acc_.norm() < 0.5)
            center << X(0) + Vx(0) * dt, X(1) + Vx(1) * dt, 0;

        auto angle = X(2) + dt * Vx(2);
        if (dt == 0)
            angle = detected_yaw;

        add_armor(angle, z1, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z2, center, l2);
        angle += std::numbers::pi / 2;
        add_armor(angle, z3, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z4, center, l2);

        return armors_;
    }

    /**
     * @brief Update the tracked car frame lengths used for armor placement.
     *
     * Sends the provided frame length measurements to the internal frame Kalman filter,
     * then reads and stores the filtered frame lengths after clamping them to the valid range [0.1, 0.6].
     *
     * @param l1 Measured length of the first lateral frame component.
     * @param l2 Measured length of the second lateral frame component.
     */
    void update_frame(double l1, double l2) {
        car_frame_kf_.Update(CarFrameKF::ZVec{l1, l2}, {}, 0);
        auto frame = car_frame_kf_.OutPut();
        this->l1   = std::clamp(frame(0), 0.1, 0.6);
        this->l2   = std::clamp(frame(1), 0.1, 0.6);
    };

    /**
     * @brief Set the per-plate height offsets used when generating armor plate poses.
     *
     * These values are stored as the four armor height parameters used by get_armor() and
     * related computations; they correspond to the armor plates produced in angle order
     * 0, 90°, 180°, and 270° around the vehicle.
     *
     * @param z1 Height for the armor at base angle 0 radians.
     * @param z2 Height for the armor at base angle π/2 radians.
     * @param z3 Height for the armor at base angle π radians.
     * @param z4 Height for the armor at base angle 3π/2 radians.
     */
    void update_z(const double& z1, const double& z2, const double& z3, const double& z4) {

        this->z1 = z1;
        this->z2 = z2;
        this->z3 = z3;
        this->z4 = z4;
    };
    /**
 * @brief Get the stored armor plate height components.
 *
 * @return Eigen::Vector<double, 4> Vector of four heights in order: [z1, z2, z3, z4].
 */
Eigen::Vector<double, 4> get_z() const { return {z1, z2, z3, z4}; }
    /**
 * @brief Retrieve the currently estimated frame lengths.
 *
 * @return std::tuple<double, double> Tuple where the first element is `l1`
 *         (frame length along the first lateral axis) and the second element
 *         is `l2` (frame length along the second lateral axis).
 */
std::tuple<double, double> get_frame() { return {l1, l2}; }
    /**
     * @brief Estimates the car's center position projected forward by a time offset.
     *
     * Projects the tracked car's 2D center forward by the provided time delta using either the
     * tracker movement state or the stored last velocity depending on recent acceleration.
     *
     * @param dt Time offset in seconds to project the position; defaults to 0 (current position).
     * @return rmcs_description::OdomImu::Position The projected 3D position (z set to 0) of the car's center.
     */
    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position(double dt = 0) {
        armors_.clear();
        auto X  = Eigen::Vector3d{car_kf_.OutPut()(0), car_kf_.OutPut()(2), car_kf_.OutPut()(4)};
        auto Vx = car_movement_kf_.OutPut();
        if (!check_armor_tracked())
            last_acc_ << 0, 0;

        Eigen::Vector3d center{X(0) + last_vel_(0) * dt, X(1) + last_vel_(1) * dt, 0};

        if (last_acc_.norm() < 0.5)
            center << X(0) + Vx(0) * dt, X(1) + Vx(1) * dt, 0;
        return rmcs_description::OdomImu::Position(center);
    }

private:
    /**
     * @brief Constructs and appends a 3D armor plate positioned and oriented around the car.
     *
     * The created armor plate is placed at a lateral offset from the provided center at the given
     * angular heading and assigned a rotation that includes the heading rotation about Z plus a
     * 15° tilt about the local Y axis. The plate's vertical coordinate is set to `z` and the
     * appended ArmorPlate3d is created with an Unknown ID.
     *
     * @param angle Heading angle in radians used to rotate the plate around the Z axis.
     * @param z Vertical position (height) to assign to the armor plate.
     * @param center Reference 3D point around which the plate is positioned.
     * @param l Lateral offset distance from `center` along the rotated local X direction.
     */
    void add_armor(double angle, double z, const Eigen::Vector3d& center, const double& l) {
        Eigen::Quaterniond forward_armor =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                15. / 180. * std::numbers::pi,
                *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        Eigen::Vector3d ccenter_{};
        ccenter_ << *rmcs_description::OdomImu::Position(center)
                        - Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                              * *rmcs_description::OdomImu::DirectionVector(
                                  Eigen::Vector3d::UnitX())
                              * l;
        ccenter_.z() = z;

        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown, rmcs_description::OdomImu::Position(ccenter_),
            rmcs_description::OdomImu::Rotation(forward_armor));
    }
    Eigen::Vector2d last_acc_ = {0, 0};
    Eigen::Vector2d last_vel_ = {0, 0};
    CarKF car_kf_;
    CarFrameKF car_frame_kf_;
    CarFrameZKF car_frame_z_kf_;
    CarMovementKF car_movement_kf_;
    double l1 = 0.3, l2 = 0.3;
    double z1 = 0, z2 = 0, z3 = 0, z4 = 0;
    double detected_yaw = 0;
    std::chrono::steady_clock::time_point last_update_time_;
    double self_update_time_ = 10086;

    constexpr static const double alpha_ = 1;

    std::vector<ArmorPlate3d> armors_;
};

/**
 * @brief Constructs a CarTracker and initializes its private implementation.
 *
 * Allocates and installs the internal PImpl instance that encapsulates Kalman
 * filters, frame trackers, and tracking state used by the public API.
 */
CarTracker::CarTracker() { pimpl_ = std::make_unique<Impl>(); }

/**
 * @brief Constructs a new CarTracker as a deep copy of an existing one.
 *
 * Creates a new CarTracker by duplicating the source's internal implementation so the copy
 * has an independent PImpl with the same tracking state.
 *
 * @param car_tracker Source CarTracker to copy from.
 */
CarTracker::CarTracker(const CarTracker& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

/**
 * @brief Advance the internal frame counter and update tracking state.
 *
 * Decrements the internal frame counter used for tracking. If the counter
 * reaches zero or below the tracker transitions to CarTrackerState::Lost.
 * If the counter remains positive and the current state is CarTrackerState::Track,
 * the state transitions to CarTrackerState::NearlyTrack. The frame counter
 * is clamped to the range [0, TrackFrameCount].
 */
void CarTracker::update_self(const double&) {
    frameCount -= 1;
    if (frameCount <= 0)
        state = CarTrackerState::Lost;
    else if (state == CarTrackerState::Track)
        state = CarTrackerState::NearlyTrack;
    frameCount = std::clamp(frameCount, 0, TrackFrameCount);
}

/**
 * @brief Indicates whether armor tracking is active.
 *
 * @return true if armor tracking is active, false otherwise.
 */
bool CarTracker::check_armor_tracked() const { return pimpl_->check_armor_tracked(); }

/**
 * @brief Gets the estimated yaw rate of the tracked car.
 *
 * @return The estimated yaw rate from the internal motion filter.
 */
double CarTracker::omega() { return pimpl_->omega(); }

/**
 * @brief Incorporates a new car pose measurement and advances the tracker's frame counter.
 *
 * Increments the internal frame counter, updates the tracker state based on the new
 * frame count (may transition to Track or NearlyLost), clamps the frame counter to
 * the allowed range, and applies the provided pose measurement to update the
 * tracker's internal tracking state.
 *
 * @param zk Measured car pose as a 3-element vector [x, y, yaw] (position x,y and heading).
 * @param dt Time difference in seconds since the previous measurement.
 */
void CarTracker::update_car(const Eigen::Vector<double, 3>& zk, const double& dt) {
    frameCount += 1;
    if (frameCount >= TrackFrameCount)
        state = CarTrackerState::Track;
    else if (state == CarTrackerState::Lost)
        state = CarTrackerState::NearlyLost;
    frameCount = std::clamp(frameCount, 0, TrackFrameCount);
    pimpl_->update_car(zk, dt);
}

/**
 * @brief Compute 3D poses for the car's armor plates projected forward by a time offset.
 *
 * Builds and returns the set of armor plate poses around the tracked vehicle, using the
 * current estimated position, orientation, velocity, and configured plate heights; positions
 * are projected forward by the specified time delta.
 *
 * @param dt Time in seconds to project the armor plate poses from the current estimate.
 * @return std::vector<ArmorPlate3d> Vector of computed armor plate poses (typically four plates around the vehicle).
 */
std::vector<ArmorPlate3d> CarTracker::get_armor(double dt) { return pimpl_->get_armor(dt); }

/**
 * @brief Update the vehicle frame dimensions used for armor placement.
 *
 * Updates the internal frame length estimates for the car; supplied values outside the valid range
 * are constrained to the tracker’s allowed bounds.
 *
 * @param l1 Longitudinal or first half-length of the frame (will be clamped to [0.1, 0.6]).
 * @param l2 Lateral or second half-length of the frame (will be clamped to [0.1, 0.6]).
 */
void CarTracker::update_frame(double l1, double l2) { return pimpl_->update_frame(l1, l2); }

/**
 * @brief Set the per-plate height offsets used when constructing 3D armor poses.
 *
 * Each parameter specifies the vertical position (z) for one of the four armor plates
 * and is used by the tracker when computing ArmorPlate3d positions.
 *
 * @param z1 Height of armor plate 1 in meters.
 * @param z2 Height of armor plate 2 in meters.
 * @param z3 Height of armor plate 3 in meters.
 * @param z4 Height of armor plate 4 in meters.
 */
void CarTracker::update_z(const double& z1, const double& z2, const double& z3, const double& z4) {
    return pimpl_->update_z(z1, z2, z3, z4);
}

/**
 * @brief Retrieve the configured height offsets for the four armor plates.
 *
 * @return Eigen::Vector<double, 4> A 4-element vector containing the armor heights
 *         in order: z1, z2, z3, z4.
 */
Eigen::Vector<double, 4> CarTracker::get_armor_height() const { return pimpl_->get_z(); }

/**
 * @brief Estimates the vehicle pose projected forward by a time interval.
 *
 * @param dt Time in seconds to project the current estimate forward.
 * @return rmcs_description::OdomImu::Position Estimated vehicle position and orientation after dt seconds.
 */
[[nodiscard]] rmcs_description::OdomImu::Position CarTracker::get_car_position(double dt) {
    return pimpl_->get_car_position(dt);
}

/**
 * @brief Retrieve the current frame lateral lengths used for armor placement.
 *
 * @return std::tuple<double, double> A pair (l1, l2) of frame lengths in meters.
 *         - first: l1 (clamped to the range [0.1, 0.6])
 *         - second: l2 (clamped to the range [0.1, 0.6])
 */
std::tuple<double, double> CarTracker::get_frame() { return pimpl_->get_frame(); }

/**
 * @brief Retrieves the tracked car's 2D velocity in the tracker/world frame.
 *
 * @return Eigen::Vector2d Velocity vector [vx, vy] in meters per second.
 */
Eigen::Vector2d CarTracker::velocity() { return pimpl_->velocity(); }
/**
 * @brief Destroys the CarTracker and cleans up its owned resources.
 *
 * Releases the internal implementation and any associated tracking state.
 */
CarTracker::~CarTracker() = default;

/**
 * @brief Computes the elapsed time since the last update and advances the internal timestamp.
 *
 * @param timestamp Time point to compute the delta against the previously stored timestamp.
 * @return double Elapsed time in seconds since the previous stored timestamp.
 */
double CarTracker::get_dt(const std::chrono::steady_clock::time_point& timestamp) {
    return pimpl_->get_dt(timestamp);
}
/**
 * @brief Get the current tracker state.
 *
 * @return CarTrackerState The current tracking state indicating the tracker's status.
 */
CarTrackerState CarTracker::get_state() { return state; }
} // namespace rmcs_auto_aim::tracker