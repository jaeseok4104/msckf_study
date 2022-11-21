#include "custom_msckf/include/math.h"

namespace custom_msckf {

Eigen::Vector3d Math::AxisMatrixToVector(const Eigen::Matrix3d& rot_mat) {
    double theta = acos((rot_mat.trace() - 1)/2);
    Eigen::Vector3d rot_vec(rot_mat(3, 2) - rot_mat(2, 3),
                            rot_mat(1, 3) - rot_mat(3, 1),
                            rot_mat(2, 1) - rot_mat(1, 2));
    rot_vec = (theta / 2.0 *sin(theta)) * rot_vec;

    return rot_vec;
}

Eigen::Matrix3d Math::AxisVectorToMatrix(const Eigen::Vector3d& rot_vec) {
    Eigen::Vector3d w = rot_vec.normalized();
    Eigen::Matrix3d k = SkewSymmetricMat(w);
    double theta = rot_vec.norm();
    return Eigen::Matrix3d::Identity() + sin(theta)*k + (1 - cos(theta))*k*k;
}

Eigen::Matrix3d Math::SkewSymmetricMat(const Eigen::Vector3d& rot_vec) {
    Eigen::Matrix3d cross_product_mat;
    cross_product_mat << 0.0, -rot_vec.z(), rot_vec.y(),
                           rot_vec.z(), 0.0, -rot_vec.x(),
                           -rot_vec.y(), rot_vec.x(), 0.0;
    return cross_product_mat;
}

Eigen::Vector3d Math::TransformationPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d p){
    return T.block<3,3>(0,0) * p + T.block<3,1>(0,3);
}

Eigen::Matrix4d Math::TransformationVectorInverse(const Eigen::VectorXd& T_vector){
    
}


}