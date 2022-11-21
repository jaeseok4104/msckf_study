#ifndef CUSTOM_MSCKF_MATH
#define CUSTOM_MSCKF_MATH

#include <Eigen/Eigen>
#include <iostream>
#include <math.h>

namespace custom_msckf {

class Math {
public:
    static Eigen::Vector3d AxisMatrixToVector(const Eigen::Matrix3d& rot_mat);
    static Eigen::Matrix3d AxisVectorToMatrix(const Eigen::Vector3d& rot_vec);
    static Eigen::Matrix3d SkewSymmetricMat(const Eigen::Vector3d& rot_vec);
    static Eigen::Vector3d TransformationPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d p);
    static Eigen::Matrix4d TransformationVectorInverse(const Eigen::VectorXd& T_vector);
};

}

#endif // CUSTOM_MSCKF_MATH