#include <ceres/jet.h>
#include <cmath>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "opencv2/opencv.hpp"
#include "util/math.hpp"

namespace re_projection {
using namespace std;
using namespace cv;
struct ReProjectionLineRatioError {
    /**
         * @brief Construct a reprojection line-ratio error functor with observed 3D-2D correspondences and camera intrinsics.
         *
         * @param object_point 3D object points to be projected (in object/camera coordinate units).
         * @param image_point Observed 2D image points corresponding to the object points.
         * @param camera_matrix Camera intrinsic matrix.
         * @param dist_coeffs Camera distortion coefficients.
         */
        ReProjectionLineRatioError(
        vector<Point3f> object_point, vector<Point2f> image_point, Mat camera_matrix,
        Mat dist_coeffs)
        : object_point_(std::move(object_point))
        , image_point_(std::move(image_point))
        , camera_matrix_(std::move(camera_matrix))
        , dist_coeffs_(std::move(dist_coeffs)) {}

    template <typename T>
    /**
     * @brief Compute the reprojection line-ratio residual for a candidate pose.
     *
     * Projects the stored 3D object line into the image using the supplied rotation and translation,
     * computes a signed difference of angle-like ratios between observed and projected image line points,
     * and writes the result into residuals[0].
     *
     * @tparam T Ceres scalar type used for evaluation.
     * @param rvec Pointer to a 3-element rotation vector ( Rodrigues ) describing the pose.
     * @param tvec Pointer to a 3-element translation vector describing the pose.
     * @param[out] residuals Array where residuals[0] will be assigned the computed residual.
     * @return `true` on success, `false` if the computed residual is not finite.
     */
    bool operator()(const T* const rvec, const T* const tvec, T* residuals) const {
        // 转换旋转向量和平移向量
        Mat rvec_mat(3, 1, CV_64F, const_cast<T*>(rvec));
        Mat tvec_mat(3, 1, CV_64F, const_cast<T*>(tvec));

        // 3D点转换为投影点
        vector<Point2f> projected_points;
        projectPoints(
            object_point_, rvec_mat, tvec_mat, camera_matrix_, dist_coeffs_, projected_points);
        double ratio1 = rmcs_auto_aim::util::math::ratio(image_point_[0] - projected_points[0])
                      - rmcs_auto_aim::util::math::ratio(image_point_[1] - projected_points[1]);
        while (ratio1 >= std::numbers::pi)
            ratio1 -= std::numbers::pi;
        while (ratio1 <= -std::numbers::pi)
            ratio1 += std::numbers::pi;
        // 计算重投影误差+
        residuals[0] = T();

        if (!ceres::IsFinite(residuals[0])) {
            std::cerr << "Error: Non-finite residuals detected. 1" << std::endl;
            return false;
        }

        return true;
    }

private:
    vector<Point3f> object_point_;
    vector<Point2f> image_point_;
    Mat camera_matrix_;
    Mat dist_coeffs_;
};
struct ReProjectionPointError {
    /**
         * @brief Construct a reprojection error term for a single 3D point and its observed 2D image location.
         *
         * @param object_point Vector containing the 3D object point(s) in the object/camera coordinate frame to be projected.
         * @param image_point Observed 2D image point corresponding to the 3D object point.
         * @param camera_matrix Camera intrinsic matrix.
         * @param dist_coeffs Camera distortion coefficients (may be empty if none).
         */
        ReProjectionPointError(
        vector<Point3f> object_point, Point2f image_point, Mat camera_matrix, Mat dist_coeffs)
        : object_point_(std::move(object_point))
        , image_point_(std::move(image_point))
        , camera_matrix_(std::move(camera_matrix))
        , dist_coeffs_(std::move(dist_coeffs)) {}

    template <typename T>
    /**
     * @brief Computes 2D reprojection residuals for the stored 3D point using the given pose.
     *
     * Projects the stored 3D point with the provided rotation and translation vectors and the stored
     * camera intrinsics/distortion, then writes the image-space residuals (observed minus projected)
     * into `residuals[0]` and `residuals[1]`.
     *
     * @tparam T Ceres scalar type used for automatic differentiation.
     * @param rvec Pointer to a 3-element rotation vector (Rodrigues format).
     * @param tvec Pointer to a 3-element translation vector.
     * @param residuals Output array where:
     *   - residuals[0] = observed_x - projected_x
     *   - residuals[1] = observed_y - projected_y
     * @return true if both residual components are finite and were written successfully, `false` if non-finite values were detected.
     */
    bool operator()(const T* const rvec, const T* const tvec, T* residuals) const {
        // 转换旋转向量和平移向量
        Mat rvec_mat(3, 1, CV_64F, const_cast<T*>(rvec));
        Mat tvec_mat(3, 1, CV_64F, const_cast<T*>(tvec));

        // 3D点转换为投影点
        vector<Point2f> projected_points;
        projectPoints(
            object_point_, rvec_mat, tvec_mat, camera_matrix_, dist_coeffs_, projected_points);

        // 计算重投影误差+
        residuals[0] = T(image_point_.x - projected_points[0].x);
        residuals[1] = T(image_point_.y - projected_points[0].y);

        if (!ceres::IsFinite(residuals[0]) || !ceres::IsFinite(residuals[1])) {
            std::cerr << "Error: Non-finite residuals detected. 2" << std::endl;
            return false;
        }

        return true;
    }

private:
    vector<Point3f> object_point_;
    Point2f image_point_;
    Mat camera_matrix_;
    Mat dist_coeffs_;
};
} // namespace re_projection