#ifndef CUSTOM_MSCKF_ERROR_TERM
#define CUSTOM_MSCKF_ERROR_TERM

#include <Eigen/Eigen>
#include "custom_msckf/include/math.h"

namespace custom_msckf{
class ErrorTerm{
    static Eigen::Vector3d PointToPointICP(const Eigen::Vector3d& p, const Eigen::Vector3d&p2_ob, const Eigen::Matrix4d& T);
};
}

#endif // CUSTOM_MSCKF_ERROR_TERM