#pragma once

#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/target_interface.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTarget : public tracker::ITarget {

public:
    /**
         * @brief Construct an ArmorTarget that wraps a CarTracker for armor prediction and state queries.
         *
         * @param car CarTracker used to obtain armor candidates and vehicle state; a copy is stored internally.
         */
        explicit ArmorTarget(const rmcs_auto_aim::tracker::CarTracker& car)
        : car(car) {};
    /**
     * @brief Selects the armor candidate most aligned with the camera forward direction and returns its predicted position.
     *
     * The method evaluates armor candidates at the requested future time and chooses the one whose forward axis has the largest projection
     * onto the camera's forward direction in the provided transform frame.
     *
     * @param sec Time offset in seconds at which to evaluate/predict armor positions.
     * @param tf Transform/frame in which camera and armor directions are compared.
     * @return rmcs_description::OdomImu::Position Position of the selected armor candidate at the requested future time.
     */
    rmcs_description::OdomImu::Position
        Predict(double sec, const rmcs_description::Tf& tf) override {
        double max    = -1e7;
        int index     = 0;
        auto armors_future = car.get_armor(sec + 0.01);
        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);
        for (int i = 0; i < 4; i++) {
            auto armor_x = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::OdomImu::DirectionVector(
                    armors_future[i].rotation->toRotationMatrix() * Eigen::Vector3d::UnitX()),
                tf);
            auto len = camera_x->dot(Eigen::Vector3d(*armor_x));
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        return armors_future[index].position;
    }

    /**
 * @brief Retrieve the car's angular velocity about its vertical axis.
 *
 * @return The car's angular velocity (yaw rate) about the vertical axis.
 */
[[nodiscard]] double get_omega() final { return car.omega(); }
    /**
 * @brief Retrieves the current frame index and fractional progress for the tracked car.
 *
 * @return std::tuple<double, double> First element is the frame index, second element is the fractional progress within that frame (0.0 to 1.0).
 */
[[nodiscard]] std::tuple<double, double> get_frame() final { return car.get_frame(); }
    /**
     * @brief Retrieves the car's current odometry/IMU position.
     *
     * @return rmcs_description::OdomImu::Position The current car position.
     */
    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position() final {
        return car.get_car_position();
    }

private:
    rmcs_auto_aim::tracker::CarTracker car;
};

} // namespace rmcs_auto_aim::tracker::armor