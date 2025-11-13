#pragma once

#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs_auto_aim::util::optimizer {

template <typename Func>
/**
 * @brief Locates the minimum of a concave-upward function on [a, b] using a Fibonacci-search strategy.
 *
 * The interval [a, b] is normalized (swapped if b < a) and iteratively shrunk by comparing
 * function values at two interior probe points computed from Fibonacci-like indices until
 * the interval length is ≤ epsilone.
 *
 * @param a Left endpoint of the search interval.
 * @param b Right endpoint of the search interval.
 * @param epsilone Convergence tolerance for the interval length (search stops when b - a <= epsilone).
 * @param concav_upward Callable taking a double and returning a double; the objective function assumed to be concave upward on [a, b].
 * @return Midpoint of the final interval (a + b) / 2, an estimate of the minimizer.
 */
inline static double fibonacci(double a, double b, double epsilone, Func concav_upward) {
    if (b < a)
        std::swap(a, b);

    double fn1 = 2, fn2 = 3;
    while ((b - a) / epsilone > fn2) {
        fn2 = fn1 + fn2;
        fn1 = fn2 - fn1;
    }

    double x1 = a + (fn2 - fn1) / fn2 * (b - a), x2 = a + fn1 / fn2 * (b - a);
    double cv1 = concav_upward(x1);
    double cv2 = concav_upward(x2);
    while ((b - a) > epsilone) {
        fn1 = fn2 - fn1;
        fn2 = fn2 - fn1;
        if (cv1 < cv2) {
            b   = x2;
            x2  = x1;
            x1  = a + (fn2 - fn1) / fn2 * (b - a);
            cv2 = cv1;
            cv1 = concav_upward(x1);
        } else {
            a   = x1;
            x1  = x2;
            x2  = a + fn1 / fn2 * (b - a);
            cv1 = cv2;
            cv2 = concav_upward(x2);
        }
        // RCLCPP_INFO(rclcpp::get_logger(""), "%lf,%lf", x1, x2);
    }
    return (a + b) / 2;
}

} // namespace rmcs_auto_aim::util::optimizer