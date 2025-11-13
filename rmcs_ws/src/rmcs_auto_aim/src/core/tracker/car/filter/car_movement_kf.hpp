#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarMovementKF : public util::EKF<3, 3> {
public:
    /**
     * @brief Constructs a CarMovementKF and initializes its state, covariances, and model matrices with default values.
     *
     * Initializes the filter covariance P_k to 0.1 * I, sets the state x_ and measurement z_ to zero, sets
     * the state-transition Jacobian a_, process-noise mapping w_, observation Jacobian h_, and observation-noise
     * mapping v_ to identity, and sets the measurement covariance r_ to 0.1 * I.
     */
    CarMovementKF()
        : EKF() {
        // clang-format off
        P_k.setIdentity();
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();
        h_.setIdentity();

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    /**
     * @brief Maps the state vector to the measurement vector (position and heading).
     *
     * Converts the input state into the measurement z = [x, y, yaw].
     *
     * @param X_k State vector where X_k(0) is x, X_k(1) is y, and X_k(2) is yaw.
     * @return ZVec Measurement vector with elements `[x, y, yaw]` in that order.
     */
    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0), X_k(1), X_k(2);
        return z_;
    }

protected:
    /**
     * @brief Predicts the next state using an identity motion model.
     *
     * @param X_k Current state vector.
     * @return XVec Predicted next state equal to `X_k`.
     */
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        x_ << X_k;
        return x_;
    }

    /**
     * @brief Provides the state-transition Jacobian matrix.
     *
     * @return AMat The constant 3x3 state-transition Jacobian matrix.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {

        return a_;
    }
    /**
 * @brief Passes an incoming measurement through unchanged.
 *
 * Processes an observed measurement vector and returns it without modification.
 *
 * @param z_k Observed measurement vector.
 * @return ZVec The same measurement vector `z_k`.
 */
[[nodiscard]] ZVec process_z(const ZVec& z_k) override { return z_k; }
    /**
 * @brief Provides the constant process-noise influence matrix.
 *
 * @return WMat The constant matrix that maps process noise into the state space.
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
     * @brief Provides the observation Jacobian for the measurement model.
     *
     * The observation Jacobian is the 3x3 identity matrix, mapping state components directly to measurements.
     *
     * @return HMat Identity observation Jacobian (3x3).
     */
    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        h_.setIdentity();
        // h_ *= dt_;
        return h_;
    }

    /**
 * @brief Provides the constant observation-noise influence matrix used by the filter.
 *
 * The matrix maps observation noise into measurement space and is constant for this model.
 *
 * @return VMat Observation noise mapping matrix.
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
     * @brief Builds the process noise covariance matrix for the motion model.
     *
     * The returned 3x3 covariance has its diagonal set to the configured process
     * noise variances for x, y, and yaw respectively.
     *
     * @return QMat 3x3 process noise covariance with diagonal
     *         [sigma2_q_xy_, sigma2_q_xy_, sigma2_q_yaw_].
     */
    [[nodiscard]] QMat Q(const double&) override {

        double x = sigma2_q_xy_;
        double y = sigma2_q_yaw_;
        // clang-format off
        //      ,vxc        ,vyc        ,theta  ,omega
        q_ <<   x       ,0          ,0      ,
                0           ,x     ,0      ,
                0           ,0          ,y ;
        // clang-format on
        return q_;
    }
    /**
     * @brief Constructs the measurement noise covariance matrix for position and yaw.
     *
     * The returned 3x3 covariance has diagonal entries [r_xyz_factor_, r_xyz_factor_, r_ywq_factor_] and zeros elsewhere.
     *
     * @return RMat Measurement noise covariance matrix with the configured diagonal variances.
     */
    RMat R(const ZVec&) override {
        r_.diagonal() << r_xyz_factor_, r_xyz_factor_, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_  = 1e0;
    static constexpr double sigma2_q_yaw_ = 1e0;
    static constexpr double r_xyz_factor_ = 1e1;
    static constexpr double r_ywq_factor_ = 1e-9;

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