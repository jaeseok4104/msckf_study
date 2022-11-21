#include "custom_msckf/include/point_to_point_icp.h"

namespace custom_msckf{
    Eigen::Vector3d PointToPointICP::ErrorTerm(const Eigen::Vector3d& p, const Eigen::Vector3d& p2_ob, const Eigen::Matrix4d& T){
        //P2*T - P1;
        return Math::TransformationPoint(T, p2_ob) - p;
    }

    bool PointToPointICP::GaussNewton(const Eigen::Vector3d& p, const Eigen::Vector3d&p2_ob, Eigen::Matrix4d* x_estimate) {
         Eigen::Vector3d b = PointToPointICP::ErrorTerm(p, p2_ob, *x_estimate);
         if(b.transpose()*b < 0.001)
            return false;
         Eigen::MatrixXd jaccobian(3,6);
         jaccobian << -Math::SkewSymmetricMat(x_estimate->block<3,3>(0,0) * p), Eigen::Matrix3d::Identity();
         Eigen::VectorXd delta_x(6);
         delta_x = (jaccobian.transpose()*jaccobian).inverse()*jaccobian.transpose() * b;
         Eigen::Matrix4d dt(Math::AxisVectorToMatrix(Eigen::Vector3d(delta_x(0), delta_x(1), delta_x(2))), Eigen::Vector3d(delta_x(3), delta_x(4), delta_x(5)));
         dt(3,3) = 1; // Homogeneous coordinate
         *x_estimate = dt * (*x_estimate);
         return true;
    }

    bool PointToPointICP::SolveLeastSquare(const Eigen::Vector3d& p, const Eigen::Vector3d p2_ob, const Eigen::Matrix4d& T, Eigen::Matrix4d* x_estimate, int iter_cnt){
        *x_estimate = T;
        bool result = false;
        for(int iter_num = 0; iter_num < iter_cnt; iter_num++) {
            if(!GaussNewton(p, p2_ob, x_estimate)) {
                result = true;
                break;
            }
            std::cout<< "iter: "<<iter_num<<" / "<<iter_cnt<<" estimated Pose : "<<x_estimate<< std::endl;
        }
    }
}