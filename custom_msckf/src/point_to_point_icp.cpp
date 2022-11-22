#include "custom_msckf/include/point_to_point_icp.h"

namespace custom_msckf{
    Eigen::Vector3d PointToPointICP::ErrorTerm(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Matrix4d& T){
        //c1_T_c2*P2 - P1;
        return Math::TransformationPoint(T, p2) - p1;
    }

    bool PointToPointICP::GaussNewton(const std::vector<Eigen::Vector3d>& p_vec, const std::vector<Eigen::Vector3d> p2_ob_vec, Eigen::Matrix4d* x_est_mat) {
        Eigen::VectorXd b(p_vec.size() * 3);
        for(int i = 0; i < p_vec.size(); i++)
            b.block<3,1>(3 * i, 0) << -PointToPointICP::ErrorTerm(p_vec.at(i), p2_ob_vec.at(i), *x_est_mat);
        // std::cout << "error_vector : " << std::endl;
        // std::cout << b << std::endl; 
        std::cout << "residual : " << b.transpose()*b / p_vec.size() << std::endl;
        if(b.transpose()*b < 0.01 * p_vec.size()) { 
            return false;
        }
        
        Eigen::MatrixXd J(p_vec.size() * 3, 6);
        for(int i = 0; i < p_vec.size(); i++)
            J.block<3,6>(3 * i, 0) << -Math::SkewSymmetricMat(x_est_mat->block<3,3>(0,0) * p_vec.at(i)), Eigen::Matrix3d::Identity();
        // std::cout << "jaccobian : " << std::endl;
        // std::cout << jaccobian << std::endl;
        Eigen::VectorXd d_x(6);
        d_x = (J.transpose()*J).inverse()*J.transpose() * b;
        std::cout << "d_x : " << std::endl;
        std::cout << Math::RAndtToT(Math::AxisVectorToMatrix(d_x.block<3,1>(0,0)), d_x.block<3,1>(3,0)) << std::endl; 
        Eigen::VectorXd x_est(6);
        x_est.block<3,1>(0,0) = Math::AxisMatrixToVector(x_est_mat->block<3,3>(0,0));
        x_est.block<3,1>(3,0) = x_est_mat->block<3,1>(0,3);
        x_est = x_est + d_x;
        *x_est_mat = Math::RAndtToT(Math::AxisVectorToMatrix(x_est.block<3,1>(0,0)), x_est.block<3,1>(3,0));
        return true;
    }

    bool PointToPointICP::SolveLeastSquare(const std::vector<Eigen::Vector3d>& p_vec, const std::vector<Eigen::Vector3d>& p2_ob_vec, const Eigen::Matrix4d& T, Eigen::Matrix4d* x_est_mat, int iter_cnt) {
        *x_est_mat = T;
        bool result = false;
        for(int iter_num = 0; iter_num < iter_cnt; iter_num++) {
            std::cout<< std::endl;
            std::cout<< "iter: "<<iter_num + 1<<" / "<<iter_cnt<< std::endl;
            std::cout << "prev estimate Pose : " << std::endl;
            std::cout << *x_est_mat << std::endl;
            if(!GaussNewton(p_vec, p2_ob_vec, x_est_mat)) {
                result = true;
                break;
            }
            std::cout << "estimate Pose : " << std::endl;
            std::cout << *x_est_mat << std::endl;
        }

        return result;
    }
}