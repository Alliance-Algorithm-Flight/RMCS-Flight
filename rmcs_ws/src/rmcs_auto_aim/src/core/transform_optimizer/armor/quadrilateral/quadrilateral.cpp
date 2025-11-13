#include <memory>

#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/profile/profile.hpp"

using namespace rmcs_auto_aim::transform_optimizer;

/**
     * @brief Constructs a Quadrilateral from an ArmorPlate.
     *
     * Initializes the internal armor by moving the provided ArmorPlate into the object.
     *
     * @param armorRef ArmorPlate whose contents are moved into the new Quadrilateral.
     */
    Quadrilateral::Quadrilateral(ArmorPlate armorRef)
    : armor(std::move(armorRef)) {}

/**
 * @brief Computes the polar angle of a 2D point or vector measured from the positive x-axis.
 *
 * @tparam T A point-like type exposing numeric members `x` and `y`.
 * @param point The point or vector whose angle is computed (uses `point.y` and `point.x`).
 * @return double Angle in radians in the range [-pi, pi].
 */
static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }

/**
 * @brief Measures the angular difference between this quadrilateral and another.
 *
 * @param s2d The other quadrilateral to compare against.
 * @return double Sum of the absolute differences (in radians) between the two quadrilaterals' corresponding edge direction angles; values are normalized into the interval [-pi, pi] before taking absolute values.
 */
double Quadrilateral::operator-(Quadrilateral s2d) const {
    auto p1 = s2d.armor.points[0] - s2d.armor.points[1];
    auto p2 = s2d.armor.points[2] - s2d.armor.points[3];

    auto p3 = armor.points[1] - armor.points[0];
    auto p4 = armor.points[3] - armor.points[2];

    auto ratio1 = ratio(p1) - ratio(p3);
    auto ratio2 = ratio(p2) - ratio(p4);

    while (ratio1 >= std::numbers::pi)
        ratio1 -= std::numbers::pi;
    while (ratio1 <= -std::numbers::pi)
        ratio1 += std::numbers::pi;

    while (ratio2 >= std::numbers::pi)
        ratio2 -= std::numbers::pi;
    while (ratio2 <= -std::numbers::pi)
        ratio2 += std::numbers::pi;

    return (abs(ratio2) + abs(ratio1));
}

/**
 * @brief Draws the quadrilateral edges and a diagonal onto an image.
 *
 * Draws the four perimeter edges connecting points 0-1, 1-2, 2-3, 3-0 and an additional diagonal between points 2 and 0.
 *
 * @param image Image to draw on; modified in-place.
 * @param color Color used for the lines.
 */
inline void Quadrilateral::draw(cv::InputOutputArray image, const cv::Scalar& color) const {
    cv::line(image, armor.points[0], armor.points[1], color);
    cv::line(image, armor.points[1], armor.points[2], color);
    cv::line(image, armor.points[2], armor.points[3], color);
    cv::line(image, armor.points[3], armor.points[0], color);
    cv::line(image, armor.points[2], armor.points[0], color);
}

struct Quadrilateral3d::Impl {

    /**
         * @brief Constructs an Impl that references the given 3D armor data.
         *
         * @param armor3dRef Reference to the ArmorPlate3d used by this Impl; the Impl stores the reference and does not take ownership.
         */
        explicit Impl(ArmorPlate3d const& armor3dRef)
        : armor3d(armor3dRef) {}

    // bull shit
    inline constexpr static const double NormalArmorWidth = 0.138, NormalArmorHeight = 0.056,
                                         LargerArmorWidth = 0.232, LargerArmorHeight = 0.056;

    inline const static std::vector<Eigen::Vector3d> LargeArmorObjectPoints = {
        Eigen::Vector3d(0.0, 0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, 0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight)};
    inline const static std::vector<Eigen::Vector3d> NormalArmorObjectPoints = {
        Eigen::Vector3d(0.0, 0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, 0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight)};

    /**
     * @brief Compute the four armor corner points expressed in the camera frame.
     *
     * Transforms the selected armor model's object-space corner points into the camera coordinate frame,
     * converts positions to millimeters, and maps them to OpenCV Point3f coordinates using the
     * (x_cam, y_cam, z_cam) -> ( -y_cam, -z_cam, x_cam ) ordering.
     *
     * @param tf Transformation context used to convert poses into the camera link.
     * @param isLargeArmor If true, use the larger armor model points; otherwise use the normal model.
     * @return std::vector<cv::Point3f> Four transformed corner points in camera-frame millimeters with coordinate permutation (-y, -z, x).
     */
    inline std::vector<cv::Point3f>
        get_objective_point(rmcs_description::Tf const& tf, bool isLargeArmor) const {
        auto positionInCamera = fast_tf::cast<rmcs_description::CameraLink>(armor3d.position, tf);

        std::vector<cv::Point3f> points{};
        auto& objectPoints = isLargeArmor ? LargeArmorObjectPoints : NormalArmorObjectPoints;

        for (int i = 0; i < 4; i++) {
            auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
                rmcs_description::OdomImu::DirectionVector(
                    armor3d.rotation->toRotationMatrix() * objectPoints[i]),
                tf);
            auto pos = (*rotationInCamera + *positionInCamera) * 1000;
            points.emplace_back(-pos.y(), -pos.z(), pos.x());
        }
        return points;
    };

    /**
     * @brief Projects the 3D armor plate into image space and returns its 2D quadrilateral.
     *
     * Uses the provided transform to convert the stored 3D armor corners into camera coordinates,
     * projects those points into the image using current camera intrinsics and distortion, and
     * constructs a Quadrilateral from the resulting 2D corner positions.
     *
     * @param tf Transform describing the camera pose relative to the armor plate.
     * @param isLargeArmor True if the armor plate uses the larger model dimensions, false for the normal model.
     * @return Quadrilateral A 2D quadrilateral built from the projected image points and the original armor ID/size flag.
     */
    Quadrilateral ToSquad(const rmcs_description::Tf& tf, bool isLargeArmor) const {
        auto objectPoints = get_objective_point(tf, isLargeArmor);

        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            objectPoints, t, r, intrinsic_parameters, distortion_parameters, imagePoints);

        return Quadrilateral(ArmorPlate(std::move(imagePoints), armor3d.id, isLargeArmor));
    }

    const ArmorPlate3d& armor3d;
};

/**
 * @brief Constructs a Quadrilateral3d from a 3D armor plate description.
 *
 * @param armor3dRef Reference to the source ArmorPlate3d used to initialize the internal implementation for projecting the 3D armor into 2D.
 */
Quadrilateral3d::Quadrilateral3d(ArmorPlate3d const& armor3dRef) {
    pimpl_ = std::make_unique<Impl>(armor3dRef);
}

/**
 * @brief Converts the stored 3D armor representation into a 2D Quadrilateral in image coordinates.
 *
 * @param tf Transformation from object frame to camera frame used for computing object points.
 * @param isLargeArmor Selects the larger armor model when true, otherwise the normal model.
 * @return Quadrilateral 2D quadrilateral obtained by projecting the armor's 3D corner points into the image.
 */
Quadrilateral
    Quadrilateral3d::ToQuadrilateral(const rmcs_description::Tf& tf, bool isLargeArmor) const {
    return pimpl_->ToSquad(tf, isLargeArmor);
}

/**
 * @brief Destroys the Quadrilateral3d instance and releases its implementation resources.
 *
 * Defaulted destructor that cleans up the internal pimpl-managed state.
 */
Quadrilateral3d::~Quadrilateral3d() = default;