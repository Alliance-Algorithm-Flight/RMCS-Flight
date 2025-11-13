#pragma once

#include <Eigen/Eigen>
#include <cmath>

#include "util/ekf.hpp"

namespace rmcs_auto_aim::tracker {

class ArmorEKF final : public util::EKF<4, 4> {
public:
    /**
     * @brief Constructs an ArmorEKF and initializes all filter state and noise matrices.
     *
     * Initializes the state covariance P_k to 0.1 * I, state estimate x_ and measurement z_ to zero,
     * measurement Jacobian h_ to zero, and sets a_, w_, v_, and q_ to identity. The measurement noise
     * r_ is initialized to identity and scaled by 0.01.
     */
    ArmorEKF()
        : EKF() {
        // clang-format off
        P_k.setIdentity();
        P_k *= 0.1;
        // clang-format on
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();

        v_.setIdentity();
        r_.setIdentity();
        q_.setIdentity();

        r_ *= 0.01;
    };

    /**
     * @brief Adjusts the incoming measurement's yaw to be within [-pi, pi] relative to the current state.
     *
     * @param z_k Measurement vector where the fourth element (index 3) is the measured yaw in radians.
     * @return ZVec Measurement identical to `z_k` except the fourth element is the yaw wrapped to lie within
     *         [-pi, pi] of the filter's current yaw `X_k(3)`.
     */
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(3) - X_k(3);
        while (err >= std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(3) = err + X_k(3);
        return z_new;
    }
    /**
 * @brief Normalize a state vector for internal consistency; this implementation leaves the state unchanged.
 *
 * @param x_k State vector to normalize.
 * @return XVec Normalized state vector — identical to `x_k` for this implementation.
 */
[[nodiscard]] XVec normalize_x(const XVec& x_k) override { return x_k; }
    //     XVec x_new = x_k;

    //     for (int i = 0; i < 8; i += 2) {
    //         std::clamp(x_new(i + 1), -10., 10.);
    //     }
    //     return x_new;
    /**
     * @brief Apply the state transition model (identity); the state is propagated unchanged.
     *
     * @returns XVec The propagated state, equal to the input state `X_k`.
     */

    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        return X_k;
    }

    /**
     * @brief Computes the predicted measurement vector from the given state.
     *
     * @param X_k State vector where elements are [azimuth, elevation, range, yaw].
     * @return ZVec Predicted measurement vector where
     *         z(0) = cos(azimuth) * cos(elevation) * range,
     *         z(1) = sin(azimuth) * cos(elevation) * range,
     *         z(2) = sin(elevation) * range,
     *         z(3) = yaw.
     */
    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_(0) = cos(X_k(0)) * cos(X_k(1)) * X_k(2);
        z_(1) = sin(X_k(0)) * cos(X_k(1)) * X_k(2);
        z_(2) = sin(X_k(1)) * X_k(2);
        z_(3) = X_k(3);

        return z_;
    }

    /**
     * @brief Returns the state-transition Jacobian matrix.
     *
     * Provides the pre-initialized 4x4 matrix used as the Jacobian of the process model.
     *
     * @return AMat The state-transition Jacobian matrix.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {
        return a_;
    }

    /**
 * @brief Provides the process-noise influence matrix used by the EKF.
 *
 * @return WMat The process noise influence matrix.
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
     * @brief Computes the measurement Jacobian with respect to the state.
     *
     * Produces the 4x4 Jacobian matrix ∂h/∂X for the measurement function:
     * z0 = cos(x0)*cos(x1)*x2,
     * z1 = sin(x0)*cos(x1)*x2,
     * z2 = sin(x1)*x2,
     * z3 = x3.
     *
     * @param X_k Current state vector [x0, x1, x2, x3].
     * @param /*unused*/ Measurement-noise vector (unused).
     * @return HMat 4x4 matrix of partial derivatives where rows correspond to measurements
     *              and columns correspond to state components. The fourth row is [0, 0, 0, 1].
     */
    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        // clang-format off
        h_ << -cos(X_k(1)) * sin(X_k(0)) * X_k(2)  , -sin(X_k(1)) * cos(X_k(0))* X_k(2) , cos(X_k(0)) * cos(X_k(1))  , 0,    //1
             cos(X_k(1)) * cos(X_k(0)) * X_k(2)    , -sin(X_k(1)) * sin(X_k(0))* X_k(2) , sin(X_k(0)) * cos(X_k(1))  , 0,    //2
             0                                     , cos(X_k(1))* X_k(2)                , sin(X_k(1))                , 0,    //3
             0                                     , 0                                  , 0                          , 1;    //4

        return h_;
        // clang-format on
    }

    /**
 * @brief Provide the measurement-noise influence matrix used by the observation model.
 *
 * @return VMat The matrix that maps measurement noise into the measurement space.
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
     * @brief Provides the process noise covariance matrix for the EKF.
     *
     * @return QMat Process noise covariance with diagonal entries [1, 1, 1, 1].
     */
    [[nodiscard]] QMat Q(const double&) override {
        // clang-format on
        q_.diagonal() << 1, 1, 1, 1;
        return q_;
    }
    /**
     * @brief Set the measurement noise covariance matrix for the observation vector.
     *
     * Updates the internal measurement noise covariance's diagonal to [1e-5, 1e-5, 1e-2, 1] and returns the resulting matrix.
     *
     * @return RMat Measurement noise covariance matrix with diagonal entries 1e-5, 1e-5, 1e-2, and 1.
     */
    [[nodiscard]]
    RMat R(const ZVec&) override {
        r_.diagonal() << 1e-5, 1e-5, 1e-2, 1;
        return r_;
    }

protected:
private:
    static constexpr inline const double conv_x     = 0.01;
    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_z     = 0.01;
    static constexpr inline const double conv_theta = 0.01;
    static constexpr inline const double conv_r     = 0.1;

    static constexpr double sigma2_q_xyz_ = 200;
    static constexpr double sigma2_q_yaw_ = 100.0;
    static constexpr double sigma2_q_r_   = 800.0;

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