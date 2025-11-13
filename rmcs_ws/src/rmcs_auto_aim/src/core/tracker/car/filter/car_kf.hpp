#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarKF : public util::EKF<6, 3> {
public:
    /**
     * @brief Constructs a CarKF and initializes EKF state, covariances, and model matrices.
     *
     * Initializes the filter's prior covariance, state and measurement vectors, state
     * transition Jacobian, noise influence matrices, measurement model, and noise
     * covariances to sensible defaults used for a 6-state (position, velocity, yaw)
     * / 3-measurement tracker.
     *
     * @details
     * - Sets the initial state covariance P_k to a predefined 6x6 pattern scaled down.
     * - Zeros the internal state (x_) and measurement (z_) vectors.
     * - Sets the state transition Jacobian (a_) and process/measurement influence
     *   matrices (w_, v_) to identity where appropriate.
     * - Configures the measurement matrix (h_) to map states [x, y, yaw] to measurements.
     * - Initializes process and measurement noise covariances (q_, r_) with default
     *   identity-based scales (measurement covariance scaled by 0.1).
     */
    CarKF()
        : EKF() {
        // clang-format off
        P_k <<  .1, 1. , 0., 0. , 0., 0. ,  // 1
                1., 10., 0., 0. , 0., 0. ,  // 2
                0., 0. , .1, 1. , 0., 0. ,  // 3
                0., 0. , 1., 10., 0., 0. ,  // 4
                0., 0. , 0., 0. , .1, 1. ,  // 5
                0., 0. , 0., 0. , 1., 10.;  // 6
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();
        h_(0, 0) = 1;
        h_(1, 2) = 1;
        h_(2, 4) = 1;

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    /**
     * @brief Predicts the measurement vector from the given state using the filter's time step.
     *
     * The state vector is interpreted as pairs of position and rate: indices 0/1 = x, vx; 2/3 = y, vy; 4/5 = yaw, yaw_rate.
     *
     * @param X_k Current state vector.
     * @return ZVec Predicted measurement vector: [x + vx * dt, y + vy * dt, yaw + yaw_rate * dt].
     */
    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0) + X_k(1) * dt_, X_k(2) + X_k(3) * dt_, X_k(4) + X_k(5) * dt_;
        return z_;
    }

protected:
    /**
     * @brief Propagates the 6-state vector forward by a time step using a constant-velocity model.
     *
     * The state layout is [y0, vy0, y1, vy1, y2, vy2]. Positions (indices 0, 2, 4) are advanced by their
     * corresponding velocities multiplied by dt; velocity components (indices 1, 3, 5) are left unchanged.
     *
     * @param X_k Current state vector (positions and velocities interleaved).
     * @param dt Time step used to propagate the state.
     * @return XVec Next state vector with updated positions and preserved velocities.
     */
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) override {
        for (int i = 0; i < 6; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt;
            x_(i + 1) = X_k(i + 1);
        }
        return x_;
    }

    /**
     * @brief Computes the state-transition Jacobian for the 6-state constant-velocity model.
     *
     * Populates the Jacobian matrix with position-velocity coupling terms: for each position-velocity pair
     * (indices 0/1, 2/3, 4/5) the element at (position_index, velocity_index) is set to `dt`.
     *
     * @param dt Time step used to compute the discrete-time Jacobian.
     * @return AMat The state-transition Jacobian matrix with coupling entries set to `dt`.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {

        for (int i = 0; i < 6; i += 2)
            a_(i, i + 1) = dt;
        return a_;
    }
    /**
     * @brief Adjusts a measurement's yaw so its angular difference to the current state lies within [-pi, pi].
     *
     * Ensures the third element (yaw) of the returned measurement is equivalent to the input yaw
     * but wrapped to be within ±pi of the filter's current yaw state (X_k(4)).
     *
     * @param z_k Measurement vector where z_k(2) is the yaw angle.
     * @return ZVec Measurement vector identical to `z_k` except with the yaw normalized relative to the current state.
     */
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(2) - X_k(4);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(2) = err + X_k(4);
        return z_new;
    }
    /**
 * @brief Returns the process-noise influence matrix used by the filter.
 *
 * The matrix maps process noise into state-space during prediction.
 *
 * @return WMat The process noise influence matrix `w_`.
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
     * @brief Computes the measurement Jacobian for the current timestep.
     *
     * Populates and returns the 3x6 measurement matrix where each measurement row i
     * has the velocity column (index i*2 + 1) set to the current timestep dt_.
     *
     * @return HMat The 3x6 measurement Jacobian with velocity-to-measurement entries equal to dt_.
     */
    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        for (int i = 0; i < 3; i += 1)
            h_(i, i * 2 + 1) = dt_;
        return h_;
    }

    /**
 * @brief Accesses the measurement noise influence matrix used by the filter.
 *
 * @return VMat The measurement noise influence matrix `v_`.
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
     * @brief Builds the process noise covariance matrix for the 6-state filter using the time step.
     *
     * Constructs the 6x6 process noise covariance Q where the state is ordered as
     * [x, vx, y, vy, theta, omega]. Position/velocity blocks are scaled by
     * sigma2_q_xy_ and yaw/omega block by sigma2_q_yaw_, with each block's entries
     * computed from powers of the time step to reflect continuous-time integration.
     *
     * @param dt Time step used to scale process noise contributions.
     * @return QMat 6x6 process noise covariance matrix for the given `dt`.
     */
    [[nodiscard]] QMat Q(const double& dt) override {

        double t = dt, x = sigma2_q_xy_, y = sigma2_q_yaw_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
        // clang-format off
        //      xc      ,vxc        ,yc     ,vyc        ,theta  ,omega
        q_ <<   q_x_x   ,q_x_vx     ,0      ,0          ,0      ,0      ,
                q_x_vx  ,q_vx_vx    ,0      ,0          ,0      ,0      ,
                0       ,0          ,q_x_x  ,q_x_vx     ,0      ,0      ,
                0       ,0          ,q_x_vx ,q_vx_vx    ,0      ,0      ,
                0       ,0          ,0      ,0          ,q_y_y  ,q_y_vy ,
                0       ,0          ,0      ,0          ,q_y_vy ,q_vy_vy;
        // clang-format on
        return q_;
    }
    /**
     * @brief Builds the measurement noise covariance matrix used by the filter.
     *
     * The returned matrix is diagonal with the first two diagonal entries set to
     * r_xyz_factor_ and the third entry set to r_ywq_factor_.
     *
     * @return RMat Measurement noise covariance matrix with diagonal
     *         [r_xyz_factor_, r_xyz_factor_, r_ywq_factor_].
     */
    RMat R(const ZVec&) override {
        double x = r_xyz_factor_;
        r_.diagonal() << x, x, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_            = 20;
    static constexpr double sigma2_q_yaw_           = 20;
    static constexpr double r_xyz_factor_           = 1e-2;
    static constexpr double r_ywq_factor_           = 10;
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