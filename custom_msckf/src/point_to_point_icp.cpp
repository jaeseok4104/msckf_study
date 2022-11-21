#include "custom_msckf/include/point_to_point_icp.h"

namespace custom_msckf{
    Eigen::Vector3d PointToPointICP::ErrorTerm(const Eigen::Vector3d& p, const Eigen::Vector3d& p2_ob, const Eigen::Matrix4d& T){
        //T*P1 - P2;
        return Math::TransformationPoint(T, p2_ob) - p;
    }

    bool PointToPointICP::GaussNewton(const std::vector<Eigen::Vector3d>& p_vec, const std::vector<Eigen::Vector3d> p2_ob_vec, Eigen::Matrix4d* x_estimate) {
        Eigen::VectorXd b(p_vec.size() * 3);
        for(int i = 0; i < p_vec.size(); i++)
            b.block<3,1>(3 * i, 0) << PointToPointICP::ErrorTerm(p_vec.at(i), p2_ob_vec.at(i), *x_estimate);
        if(b.transpose()*b < 0.001)
            return false;
        Eigen::MatrixXd jaccobian(p_vec.size() * 3, 6);
        for(int i = 0; i < p_vec.size(); i++)
            jaccobian.block<3,6>(3 * i, 0) << Math::SkewSymmetricMat(x_estimate->block<3,3>(0,0) * p_vec.at(i)), Eigen::Matrix3d::Identity();
        // std::cout << "jaccobian : " << std::endl;
        // std::cout << jaccobian << std::endl;
        Eigen::VectorXd delta_x(6);
        delta_x = (jaccobian.transpose()*jaccobian).inverse()*jaccobian.transpose() * b;
        Eigen::VectorXd x_estimation_vector(6);
        x_estimation_vector.block<3,1>(0,0) = Math::AxisMatrixToVector(x_estimate->block<3,3>(0,0));
        x_estimation_vector.block<3,1>(3,0) = x_estimate->block<3,1>(0,3);
        x_estimation_vector = x_estimation_vector + delta_x;
        x_estimate->block<3,3>(0,0) = Math::AxisVectorToMatrix(x_estimation_vector.block<3,1>(0,0));
        x_estimate->block<3,1>(0,3) = x_estimation_vector.block<3,1>(3,0);
        return true;
    }

    bool PointToPointICP::SolveLeastSquare(const std::vector<Eigen::Vector3d>& p_vec, const std::vector<Eigen::Vector3d>& p2_ob_vec, const Eigen::Matrix4d& T, Eigen::Matrix4d* x_estimate, int iter_cnt) {
        *x_estimate = T;
        bool result = false;
        for(int iter_num = 0; iter_num < iter_cnt; iter_num++) {
            if(!GaussNewton(p_vec, p2_ob_vec, x_estimate)) {
                result = true;
                break;
            }
            std::cout<< "iter: "<<iter_num<<" / "<<iter_cnt<< std::endl;
            std::cout << "estimated Pose : " << std::endl;
            std::cout << *x_estimate << std::endl;
        }

        return result;
    }
}