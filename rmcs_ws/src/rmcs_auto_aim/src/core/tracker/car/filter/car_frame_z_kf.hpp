#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim::tracker {

class CarFrameZKF : public util::EKF<4, 4> {
public:
    /**
     * @brief Constructs a CarFrameZKF and initializes its state and covariance matrices.
     *
     * Initializes the filter state and measurement to zero; sets the prior covariance
     * P_k to identity scaled by 0.01; sets the state-transition, process-noise
     * influence, observation, and measurement-noise influence matrices (a_, w_, h_, v_)
     * to the identity; sets the process-noise covariance q_ to identity scaled by 10;
     * and sets the measurement-noise covariance r_ to identity scaled by 0.001.
     */
    CarFrameZKF()
        : EKF() {
        P_k.setIdentity();
        P_k *= 0.01;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setIdentity();

        v_.setIdentity();
        q_.setIdentity();
        q_ *= 10;
        r_.setIdentity();
        r_ *= 0.001;
    };

protected:
    /**
     * @brief Propagates the state using an identity transition (no state change).
     *
     * @param X_k Current state vector.
     * @return XVec The propagated state vector, equal to `X_k`.
     */
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        return X_k;
    }

    /**
 * @brief Maps the state to the observation using an identity measurement model.
 *
 * @param X_k Current state vector.
 * @return ZVec Measurement vector equal to X_k.
 */
[[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override { return X_k; }

    /**
     * @brief Provides the state transition Jacobian matrix for the current operating point.
     *
     * @return AMat The 4x4 state transition Jacobian matrix A evaluated at the given state and inputs.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {
        return a_;
    }
    /**
 * @brief Provides the process-noise influence matrix used by the filter.
 *
 * @return WMat The process-noise influence matrix (W).
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
 * @brief Provides the observation matrix (measurement Jacobian) used by the filter.
 *
 * @returns HMat The observation matrix (∂h/∂x) currently stored in the filter.
 */
[[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    /**
 * @brief Provides the measurement-noise influence matrix for the observation model.
 *
 * @return Measurement-noise influence matrix that maps measurement noise into the measurement space.
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
 * @brief Provides the filter's process noise covariance matrix.
 *
 * @return QMat The process noise covariance matrix (`q_`) used by this Kalman filter.
 */
[[nodiscard]] QMat Q(const double&) override { return q_; }
    /**
     * @brief Provide the measurement noise covariance matrix for a given measurement.
     *
     * @param z Measurement vector (not inspected by this implementation).
     * @return RMat Measurement noise covariance matrix corresponding to the measurement.
     */
    [[nodiscard]]
    RMat R(const ZVec&) override {
        return r_;
    }

private:
    static constexpr double sigma2_q_xy_  = 300;
    static constexpr double sigma2_q_yaw_ = 100.0;

    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_p     = 0.01;
    static constexpr inline const double conv_d     = 0.5;
    static constexpr inline const double conv_theta = 0.1;

    XVec x_;
    ZVec z_;
    AMat a_;
    WMat w_;
    HMat h_;
    VMat v_;
    QMat q_;
    RMat r_;
};

} // namespace rmcs_auto_aim::tracker