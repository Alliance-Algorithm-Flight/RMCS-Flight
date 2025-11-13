#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <tuple>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/pnpsolver/light_bar/light_bar_solver.hpp"
#include "light_bar_solver.hpp"
#include "line.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"
#include "util/optimizer/fibonacci.hpp"
#include "util/profile/profile.hpp"
#include "util/utils.hpp"

using namespace rmcs_auto_aim;

class LightBarSolver::StaticImpl {
public:
    /**
     * @brief Estimate 3D poses for a collection of detected armor plates using light-bar geometry and the camera transform.
     *
     * For each input armor detection, computes a 3D position and orientation by aligning projected 3D light-bar and
     * armor models to the detected 2D image lines. The method derives the camera yaw in the odometry (OdomImu) frame
     * from the provided transform and searches over yaw and distance to produce a best-fit 3D pose for each armor plate.
     *
     * @param armors Detected armor plates to solve for; each entry provides 2D image points and size classification.
     * @param tf Transform helper used to convert camera-frame positions/rotations into the OdomImu (odometry/IMU) frame.
     * @return std::vector<ArmorPlate3d> A list of computed armor poses. Each entry contains the original armor id,
     *         the estimated position expressed in the OdomImu frame, and the estimated orientation (rotation).
     */
    static std::vector<ArmorPlate3d>
        SolveAll(const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
        auto camera_rotation_in_odom = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Rotation(Eigen::Quaterniond::Identity()), tf);
        auto camera_yaw_in_odom = util::math::get_yaw_from_quaternion(*camera_rotation_in_odom);

        const Eigen::AngleAxis rotation = Eigen::AngleAxis(
            15. / 180.0 * std::numbers::pi,
            *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));
        std::vector<ArmorPlate3d> result;
        for (auto& armor : armors) {
            const auto& [top, button, light_bar, dir, top1, button1] = get_full_light_bar(armor);
            const Line image_line{top, button};
            const Line image_line1{top1, button1};
            // util::ImageViewer::draw(image_line1, {0, 255, 255});
            Eigen::Vector3d camera_vec;
            util::compute_yaw_pitch_from_point(
                (image_line.button_ + image_line.top_) / 2,
                util::Profile::get_intrinsic_parameters(),
                util::Profile::get_distortion_parameters(), camera_vec, 3);
            auto target_yaw = util::optimizer::fibonacci(
                camera_yaw_in_odom - std::numbers::pi / 2,
                camera_yaw_in_odom + std::numbers::pi / 2, 1e-2,
                [&image_line, &light_bar, &image_line1, &rotation, &armor, &tf,
                 &camera_vec](const double& yaw) {
                    auto line_rotation = set_armor3d_angle(rotation, yaw);
                    Line3d line3d{
                        *line_rotation, camera_vec,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};

                    LightBar3d line3d1{*line_rotation, camera_vec, light_bar};

                    return line3d.to_line_2d(tf).angle_distance(image_line)
                         + line3d1.to_line_2d(tf).angle_distance(image_line1) * 0.2;
                });
            const auto target_distance = util::optimizer::fibonacci(
                0, 20, 1e-3,
                [&image_line, &light_bar, &image_line1, &rotation, &armor, &tf, target_yaw,
                 &camera_vec](const double& distance) {
                    auto line_rotation = set_armor3d_angle(rotation, target_yaw);
                    auto tmp           = camera_vec * distance;
                    Line3d line3d{
                        *line_rotation, tmp,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
                    LightBar3d line3d1{*line_rotation, tmp, light_bar};
                    return line3d.to_line_2d(tf).length_distance(image_line)
                         + line3d.to_line_2d(tf).length_distance(image_line1) * 0.1;
                });
            target_yaw = util::optimizer::fibonacci(
                target_yaw + std::numbers::pi / 6, target_yaw - std::numbers::pi / 6, 1e-4,
                [&image_line, &image_line1, &light_bar, &rotation, &target_distance, &armor, &tf,
                 &camera_vec](const double& yaw) {
                    auto line_rotation = set_armor3d_angle(rotation, yaw);
                    LightBar3d line3d{*line_rotation, camera_vec * target_distance, light_bar};
                    Line3d line2d{
                        *line_rotation, camera_vec,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
                    // util::ImageViewer::draw(line3d.to_line_2d(tf), {255, 255, 0});
                    return line3d.to_line_2d(tf).line_distance(image_line1)
                         + line3d.to_line_2d(tf).angle_distance(image_line1) * 1
                         + line2d.to_line_2d(tf).angle_distance(image_line) * 1;
                });
            auto armor_angle = set_armor3d_angle(rotation, target_yaw);
            Line3d line3d{
                *armor_angle, camera_vec * target_distance,
                (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
            LightBar3d line3d1{*armor_angle, camera_vec * target_distance, light_bar};
            result.emplace_back(
                armor.id,
                rmcs_description::OdomImu::Position(
                    *fast_tf::cast<rmcs_description::OdomImu>(
                        rmcs_description::CameraLink::Position{camera_vec * target_distance}, tf)
                    + *armor_angle * dir
                          * (armor.is_large_armor ? LargerArmorWidth : NormalArmorWidth) / 1000.0
                          / 2),
                armor_angle);
        }
        return result;
    }

private:
    /**
     * @brief Applies an additional rotation around the Z axis to an existing armor orientation.
     *
     * Composes a new rotation by rotating the provided armor orientation around the Z axis
     * by the specified angle and returns the resulting OdomImu::Rotation.
     *
     * @param inOutArmor3d Existing armor orientation as an Eigen::AngleAxis<double>.
     * @param angle Angle, in radians, to apply about the Z axis.
     * @return rmcs_description::OdomImu::Rotation The composed rotation after applying the Z-axis rotation.
     */
    static inline rmcs_description::OdomImu::Rotation
        set_armor3d_angle(const Eigen::AngleAxis<double>& inOutArmor3d, const double& angle) {
        return rmcs_description::OdomImu::Rotation(
            Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ()) * inOutArmor3d);
    }

    inline constexpr static const double MaxArmorDistance = 15.0;

    inline constexpr static const double NormalArmorWidth = 134, NormalArmorHeight = 56,
                                         LargerArmorWidth = 230, LargerArmorHeight = 56;

    inline const static std::vector<Eigen::Vector3d> SmallLeftLightBar = {
        Eigen::Vector3d(-2e-3, 135e-3, 28e-3), Eigen::Vector3d(-2e-3, 135e-3, -28e-3),
        Eigen::Vector3d(-7e-3, 128e-3, -28e-3), Eigen::Vector3d(-7e-3, 128e-3, 28e-3)};
    inline const static std::vector<Eigen::Vector3d> SmallRightLightBar = {
        Eigen::Vector3d(-7e-3, -128e-3, 28e-3), Eigen::Vector3d(-7e-3, -128e-3, -28e-3),
        Eigen::Vector3d(-2e-3, -135e-3, -28e-3), Eigen::Vector3d(-2e-3, -135e-3, 28e-3)};

    inline const static std::vector<Eigen::Vector3d> LargeLeftLightBar = {
        Eigen::Vector3d(-2e-3, 231e-3, 28e-3), Eigen::Vector3d(-2e-3, 231e-3, -28e-3),
        Eigen::Vector3d(-7e-3, 224e-3, -28e-3), Eigen::Vector3d(-7e-3, 224e-3, 28e-3)};
    inline const static std::vector<Eigen::Vector3d> LargeRightLightBar = {
        Eigen::Vector3d(-7e-3, -224e-3, 28e-3), Eigen::Vector3d(-7e-3, -224e-3, -28e-3),
        Eigen::Vector3d(-2e-3, -231e-3, -28e-3), Eigen::Vector3d(-2e-3, -231e-3, 28e-3)};

    /**
     * @brief Selects the appropriate 2D/3D light-bar endpoints and orientation for an armor plate.
     *
     * @param armor The detected armor plate (2D corner points and size flag) used to choose endpoints and model.
     * @return std::tuple containing, in order:
     *   - primary 2D endpoint A (cv::Point2f),
     *   - primary 2D endpoint B (cv::Point2f),
     *   - reference to the chosen 3D light-bar model points (vector<Eigen::Vector3d>) for small or large, left or right,
     *   - local 3D direction vector (Eigen::Vector3d) indicating light-bar orientation (unit Y or -unit Y),
     *   - secondary 2D endpoint A (cv::Point2f),
     *   - secondary 2D endpoint B (cv::Point2f).
     *
     * The selected 3D model (left/right and small/large) and the ordering of returned 2D endpoints are determined
     * from the armor's 2D geometry and its `is_large_armor` flag.
     */
    inline static std::tuple<
        const cv::Point2f, const cv::Point2f, const std::vector<Eigen::Vector3d>&, Eigen::Vector3d,
        const cv::Point2f, const cv::Point2f>
        get_full_light_bar(const ArmorPlate& armor) {
        cv::Point2f l1 = armor.points[0] - armor.points[1];
        return abs(util::math::clamp_pm_pi(util::math::ratio(l1))) > std::numbers::pi / 2
                 ? std::tuple<
                        const cv::Point2f, const cv::Point2f, const std::vector<Eigen::Vector3d>&,
                        Eigen::Vector3d, const cv::Point2f,
                        const cv::Point2f>{armor.points[0],    armor.points[1],
                        armor.is_large_armor ? LargeRightLightBar : SmallRightLightBar, 
                        -Eigen::Vector3d::UnitY(),
                        armor.points[3],   
                        armor.points[2]}
                 : std::tuple<
                       const cv::Point2f, const cv::Point2f, const std::vector<Eigen::Vector3d>&,
                       Eigen::Vector3d, const cv::Point2f, const cv::Point2f>{
                       armor.points[3],
                       armor.points[2],
                       armor.is_large_armor ? LargeLeftLightBar : SmallLeftLightBar,
                       Eigen::Vector3d::UnitY(),
                       armor.points[0],
                       armor.points[1]};
    }
};

/**
 * @brief Compute 3D poses for detected armor plates using light-bar geometry and a camera transform.
 *
 * @param armors Detected armor plates (image-space observations and metadata) to be processed.
 * @param tf Coordinate transform provider used to obtain the camera pose in the odometry/IMU frame.
 * @return std::vector<ArmorPlate3d> A list of 3D armor representations containing each armor's id, position, and orientation in the odometry/IMU frame.
 */
std::vector<ArmorPlate3d> LightBarSolver::SolveAll(
    const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
    return LightBarSolver::StaticImpl::SolveAll(armors, tf);
}