#include "custom_msckf/include/error_term.h"

namespace custom_msckf{
    Eigen::Vector3d ErrorTerm::PointToPointICP(const Eigen::Vector3d& p, const Eigen::Vector3d& p2_ob, const Eigen::Matrix4d& T){
        return p - Math::TransformationPoint(T, p2_ob);
    }
}