#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarPosKF : public util::EKF<3, 3> {
public:
    /**
     * @brief Initializes the CarPosKF extended Kalman filter with default state and covariance values.
     *
     * Sets the state covariance P_k to identity scaled by 0.1, zeroes the state x_ and measurement z_,
     * and initializes the Jacobian and noise matrices a_, w_, h_, and v_ to identity. The measurement
     * noise covariance r_ is set to identity scaled by 0.1.
     */
    CarPosKF()
        : EKF() {
        // clang-format off
        P_k .setIdentity();
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setIdentity();

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    /**
 * @brief Observation model that maps the filter state into the predicted measurement.
 *
 * The measurement prediction is the identity mapping: the predicted measurement equals the input state.
 *
 * @return ZVec Predicted measurement equal to the provided state vector.
 */
[[nodiscard]] ZVec h(const XVec& x_k, const VVec&) override { return x_k; }

protected:
    /**
     * @brief State transition function implementing an identity motion model.
     *
     * Produces the next state by leaving the current state unchanged.
     *
     * @param x_k Current state vector.
     * @return XVec Next state vector equal to `x_k`.
     */
    [[nodiscard]] XVec f(const XVec& x_k, const UVec&, const WVec&, const double&) override {

        return x_k;
    }

    /**
     * @brief Provides the state transition Jacobian matrix.
     *
     * @return AMat The state transition Jacobian (3x3) used by the filter.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {

        return a_;
    }
    /**
     * Normalize the incoming measurement yaw to the principal range and return the adjusted measurement vector.
     *
     * @param z_k Incoming measurement vector where element 2 is the measured yaw in radians.
     * @return ZVec Measurement vector identical to `z_k` except that element 2 is replaced by the yaw adjusted so the difference
     * between the measurement and the filter's current yaw is wrapped to the range [-pi, pi] and re-applied to the current yaw.
     */
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(2) - X_k(2);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(2) = err + X_k(2);
        return z_new;
    }
    /**
 * @brief Provides the process-noise covariance matrix used by the filter.
 *
 * @return WMat The process-noise covariance matrix.
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
 * @brief Get the observation Jacobian used by the filter.
 *
 * @return HMat The observation Jacobian matrix.
 */
[[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    /**
 * @brief Provides the observation noise covariance matrix for the filter.
 *
 * @return VMat The measurement noise covariance matrix used by the observation model.
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
     * @brief Constructs the process noise covariance matrix for the filter state.
     *
     * The returned matrix is diagonal with the first two diagonal entries set to
     * sigma2_q_xy_ and the third diagonal entry set to sigma2_q_yaw_.
     *
     * @return QMat Diagonal process noise covariance matrix: [sigma2_q_xy_, sigma2_q_xy_, sigma2_q_yaw_].
     */
    [[nodiscard]] QMat Q(const double&) override {

        // clang-format off
        q_ .setIdentity();
        q_*= sigma2_q_xy_;
        q_(2,2) = sigma2_q_yaw_;
        // clang-format on
        return q_;
    }
    /**
     * @brief Constructs the measurement noise covariance matrix for the filter.
     *
     * Sets the diagonal of the returned R matrix to [r_xyz_factor_, r_xyz_factor_, r_ywq_factor_].
     *
     * @return RMat Measurement noise covariance matrix with the configured diagonal values.
     */
    RMat R(const ZVec&) override {
        double x = r_xyz_factor_;
        r_.diagonal() << x, x, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_  = 5e-3;
    static constexpr double sigma2_q_yaw_ = 5e-3;
    static constexpr double r_xyz_factor_ = 5e-2;
    static constexpr double r_ywq_factor_ = 5e-5;

    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_p     = 0.01;
    static constexpr inline const double conv_d     = 0.5;
    static constexpr inline const double conv_theta = 0.1;

    XVec x_{};
    ZVec z_{};
    AMat a_{};
    WMat w_{};
    HMat h_{};
    VMat v_{};
    QMat q_{};
    RMat r_{};
};

} // namespace rmcs_auto_aim::tracker