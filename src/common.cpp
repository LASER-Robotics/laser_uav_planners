/**
 * @file common.cpp
 * @author Krystof Teissing (teisskry@gmail.com)
 * @version 0.1
 * @date 2023-05-25
 * 
 */

#include <laser_uav_planner/common.hpp>

namespace pmm {

// GRAVITY ----------------------------------------------------------------/
// Gravity Value [m/s^2]
Scalar G = 9.8066;

// Gravity Vector [m/s^2]
Vector<3> GVEC = {0, 0, -G};

// DRAG -------------------------------------------------------------------/
Eigen::Matrix3d D_coeffs = -(Eigen::Matrix3d()<< 0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0).finished();

void redistribute(Eigen::Vector3d& vec) {
    // Calculate the norm of the vector
    double original_norm = vec.norm();

    // Calculate the average of the absolute values of the components
    double avg = (std::abs(vec[0]) + std::abs(vec[1]) + std::abs(vec[2])) / 3.0;

    // Adjust the components based on their relationship to the average
    for (int i = 0; i < 3; ++i) {
        if (std::abs(vec[i]) > avg) {
            vec[i] -= std::copysign(0.1*original_norm, vec[i]); // Decrease larger values
        } else {
            vec[i] += std::copysign(0.1*original_norm, vec[i]); // Increase smaller values
        }
    }

    // Normalize the vector to preserve its original norm
    vec = vec.normalized() * original_norm;
}

} // namespace pmm
