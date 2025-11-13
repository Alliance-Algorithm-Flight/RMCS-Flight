#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>

#include "armor_pnp_solver.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include "util/profile/profile.hpp"

using namespace rmcs_auto_aim;

class ArmorPnPSolver::StaticImpl {
public:
    /**
     * @brief Estimates 3D poses for a collection of detected armor plates using PnP.
     *
     * Runs a PnP pose estimation for each input armor, converts successful solutions
     * into ArmorPlate3d entries in the provided TF frame, and omits any armor whose
     * PnP fails or whose computed distance exceeds the solver's maximum allowed distance.
     *
     * @param armors Vector of detected 2D armor observations (image points and metadata).
     * @param tf Coordinate transform description used to cast results into the target frame.
     * @return std::vector<ArmorPlate3d> Vector of successfully estimated armor poses in the TF frame.
     */
    static std::vector<ArmorPlate3d>
        SolveAll(const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
        std::vector<ArmorPlate3d> armors3d;

        for (const auto& armor : armors) {
            cv::Mat rvec, tvec;
            auto& objectPoints =
                armor.is_large_armor ? LargeArmorObjectPoints : NormalArmorObjectPoints;
            if (cv::solvePnP(
                    objectPoints, armor.points, util::Profile::get_intrinsic_parameters(),
                    util::Profile::get_distortion_parameters(), rvec, tvec, false,
                    cv::SOLVEPNP_IPPE)) {

                // cv::solvePnPRefineLM(
                //     objectPoints, armor.points,
                //     (cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1),
                //     (cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3), rvec, tvec);

                Eigen::Vector3d position = {
                    tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1)};
                position = position / 1000.0;
                if (position.norm() > MaxArmorDistance) {
                    continue;
                }

                Eigen::Vector3d rvec_eigen = {
                    rvec.at<double>(2), -rvec.at<double>(0), -rvec.at<double>(1)};
                Eigen::Quaterniond rotation = Eigen::Quaterniond{
                    Eigen::AngleAxisd{rvec_eigen.norm(), rvec_eigen.normalized()}
                };

                armors3d.emplace_back(
                    armor.id,
                    fast_tf::cast<rmcs_description::OdomImu>(
                        rmcs_description::CameraLink::Position{position}, tf),
                    fast_tf::cast<rmcs_description::OdomImu>(
                        rmcs_description::CameraLink::Rotation{rotation}, tf));
            } else {
                continue;
            }
        }

        return armors3d;
    }

    /**
     * @brief Estimate the 3D pose of a single armor plate from its 2D image corners.
     *
     * Attempts to solve a PnP problem using the provided camera intrinsics and distortion to
     * produce the armor's position and orientation in the camera frame. If pose estimation
     * fails or the computed position's distance exceeds the maximum allowed distance,
     * an empty result is returned.
     *
     * @param armor ArmorPlate containing the 2D image points and size flag (normal vs large).
     * @param fx Focal length in x (pixels).
     * @param fy Focal length in y (pixels).
     * @param cx Principal point x-coordinate (pixels).
     * @param cy Principal point y-coordinate (pixels).
     * @param k1 Radial distortion coefficient k1.
     * @param k2 Radial distortion coefficient k2.
     * @param k3 Radial distortion coefficient k3.
     * @return ArmorPlate3dWithoutFrame ArmorPlate3dWithoutFrame with armor id, position in meters, and rotation quaternion on success; empty value if PnP fails or the computed distance exceeds the allowed maximum.
     */
    static ArmorPlate3dWithoutFrame Solve(
        const ArmorPlate& armor, const double& fx, const double& fy, const double& cx,
        const double& cy, const double& k1, const double& k2, const double& k3) {

        cv::Mat rvec, tvec;
        auto& objectPoints =
            armor.is_large_armor ? LargeArmorObjectPoints : NormalArmorObjectPoints;
        if (cv::solvePnP(
                objectPoints, armor.points,
                (cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1),
                (cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3), rvec, tvec, false,
                cv::SOLVEPNP_IPPE)) {

            Eigen::Vector3d position = {
                tvec.at<double>(2), -tvec.at<double>(0), -tvec.at<double>(1)};
            position = position / 1000.0;
            if (position.norm() > MaxArmorDistance) {
                return {};
            }

            Eigen::Vector3d rvec_eigen = {
                rvec.at<double>(2), -rvec.at<double>(0), -rvec.at<double>(1)};
            Eigen::Quaterniond rotation = Eigen::Quaterniond{
                Eigen::AngleAxisd{rvec_eigen.norm(), rvec_eigen.normalized()}
            };

            return {armor.id, position, rotation};
        }

        return {};
    }

private:
    inline constexpr static const double MaxArmorDistance = 15.0;

    inline constexpr static const double NormalArmorWidth = 134, NormalArmorHeight = 56,
                                         LargerArmorWidth = 230, LargerArmorHeight = 56;
    inline static const std::vector<cv::Point3d> LargeArmorObjectPoints = {
        cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f)};
    inline static const std::vector<cv::Point3d> NormalArmorObjectPoints = {
        cv::Point3d(-0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f)};
};

/**
 * @brief Estimate 3D poses for multiple detected armor plates using a PnP-based solver.
 *
 * Runs PnP for each provided 2D armor detection, filters out failed solves and poses
 * beyond the solver's maximum allowed distance, and returns the successfully estimated poses
 * transformed into the OdomImu frame using the provided TF description.
 *
 * @param armors Vector of detected armor plates containing 2D image points and metadata.
 * @param tf Camera-to-world TF description used to cast computed positions and rotations into OdomImu.
 * @return std::vector<ArmorPlate3d> Poses for armors with successful and in-range estimations; one entry per successful estimate.
 */
std::vector<ArmorPlate3d> ArmorPnPSolver::SolveAll(
    const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
    return ArmorPnPSolver::StaticImpl::SolveAll(armors, tf);
}

/**
 * @brief Estimate a single armor plate pose from image points using PnP.
 *
 * Solves a Perspective-n-Point problem for the provided armor using the supplied
 * camera intrinsics and radial distortion coefficients. If pose estimation
 * succeeds and the computed distance is within the solver's maximum allowed
 * range, returns the estimated pose (position and orientation) expressed in
 * the camera frame; otherwise returns an empty/invalid ArmorPlate3dWithoutFrame.
 *
 * @param armor Detected armor plate with 2D image corner points and size info.
 * @param fx Camera focal length in x (pixels).
 * @param fy Camera focal length in y (pixels).
 * @param cx Camera principal point x-coordinate (pixels).
 * @param cy Camera principal point y-coordinate (pixels).
 * @param k1 First radial distortion coefficient.
 * @param k2 Second radial distortion coefficient.
 * @param k3 Third radial distortion coefficient.
 * @return ArmorPlate3dWithoutFrame Estimated pose for the armor; valid when PnP succeeds and distance <= 15.0 meters, otherwise an empty/invalid result.
 */
ArmorPlate3dWithoutFrame ArmorPnPSolver::Solve(
    const ArmorPlate& armor, const double& fx, const double& fy, const double& cx, const double& cy,
    const double& k1, const double& k2, const double& k3) {
    return ArmorPnPSolver::StaticImpl::Solve(armor, fx, fy, cx, cy, k1, k2, k3);
}