#ifndef CUSTOM_MSCKF_POINT_TO_POINT_ICP
#define CUSTOM_MSCKF_POINT_TO_POINT_ICP

#include <Eigen/Eigen>
#include "custom_msckf/include/math.h"
#include <iostream>

namespace custom_msckf{
class PointToPointICP{
    static Eigen::Vector3d ErrorTerm(const Eigen::Vector3d& p, const Eigen::Vector3d&p2_ob, const Eigen::Matrix4d& T);
    static bool GaussNewton(const Eigen::Vector3d& p, const Eigen::Vector3d&p2_ob, Eigen::Matrix4d* x_estimate);
    static bool SolveLeastSquare(const Eigen::Vector3d& p, const Eigen::Vector3d p2_ob, const Eigen::Matrix4d& T, Eigen::Matrix4d* x_estimate, int iter_cnt);
};
}

#endif // CUSTOM_MSCKF_POINT_TO_POINT_ICP