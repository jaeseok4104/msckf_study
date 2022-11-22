#include "custom_msckf/include/math.h"

namespace custom_msckf {

Eigen::Vector3d Math::AxisMatrixToVector(const Eigen::Matrix3d& rot_mat) {
    double theta = acos((rot_mat.trace() - 1.0)/2.0);
    Eigen::Vector3d rot_vec(rot_mat(2, 1) - rot_mat(1, 2),
                            rot_mat(0, 2) - rot_mat(2, 0),
                            rot_mat(1, 0) - rot_mat(0, 1));
    rot_vec = (theta / (2.0*sin(theta))) * rot_vec;

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

Eigen::Matrix4d Math::TransformationVectorInverse(const Eigen::VectorXd& T_vector) {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    Eigen::Matrix4d T;
    t = T_vector.block<3,1>(3,0);
    R = AxisVectorToMatrix(T_vector.block<3,1>(0,0));
    T = Math::RAndtToT(R.transpose(),-R.transpose()*t);

    return T;
}

Eigen::Matrix4d Math::RAndtToT(const Eigen::Matrix3d& R, const Eigen::Vector3d t) {
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

}