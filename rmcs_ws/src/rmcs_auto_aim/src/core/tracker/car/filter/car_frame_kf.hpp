#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarFrameKF : public util::EKF<2, 2> {
public:
    /**
     * @brief Constructs a CarFrameKF and initializes state, covariance, system, and noise matrices.
     *
     * Initializes the filter with:
     * - state and measurement vectors set to zero,
     * - posterior covariance P_k set to identity scaled by 0.01,
     * - state transition (a_), process noise influence (w_), observation (h_), and measurement noise influence (v_) matrices set to identity,
     * - process noise covariance q_ set to identity scaled by 0.1,
     * - measurement noise covariance r_ set to identity scaled by 5.
     */
    CarFrameKF()
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
        q_ *= 0.1;
        r_.setIdentity();
        r_ *= 5;
    };

protected:
    /**
     * @brief Identity state transition that predicts the next state equal to the current state.
     *
     * @param X_k Current state vector.
     * @return XVec Predicted state vector equal to `X_k`.
     */
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        return X_k;
    }

    /**
 * @brief Compute the predicted measurement for a given state.
 *
 * The measurement model is identity: the predicted measurement equals the provided state vector.
 * The measurement-noise vector parameter is unused.
 *
 * @param X_k State vector used as the predicted measurement.
 * @return ZVec Predicted measurement equal to X_k.
 */
[[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override { return X_k; }

    /**
     * @brief Provides the state transition Jacobian matrix for the filter.
     *
     * @return AMat The stored state transition Jacobian matrix `a_`.
     */
    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {
        return a_;
    }
    /**
 * @brief Provides the process-noise influence matrix for the state model.
 *
 * @return WMat The process noise influence matrix stored in the filter.
 */
[[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    /**
 * @brief Retrieves the measurement Jacobian matrix.
 *
 * @return HMat Measurement matrix mapping state space to measurement space (measurement Jacobian).
 */
[[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    /**
 * @brief Provides the measurement-noise influence matrix used by the filter.
 *
 * @return VMat The measurement noise influence matrix (`v_`).
 */
[[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    /**
 * @brief Provides the process noise covariance matrix used by the filter.
 *
 * @return QMat Process noise covariance matrix.
 */
[[nodiscard]] QMat Q(const double&) override { return q_; }
    /**
     * @brief Get the measurement noise covariance matrix.
     *
     * @return RMat Measurement noise covariance matrix for the observation model.
     */
    [[nodiscard]]
    RMat R(const ZVec&) override {
        return r_;
    }

private:
    static constexpr double sigma2_q_xy_  = 30000;
    static constexpr double sigma2_q_yaw_ = 10000.0;

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