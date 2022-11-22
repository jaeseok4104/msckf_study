#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

#include "custom_msckf/include/point_to_point_icp.h"
#include "random"

namespace custom_msckf {
class PointToPointICPTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        const double mean = 0.0;
        const double noise_cov = 0.5;
        const double point_generator_cov = 30;
        std::normal_distribution<double> noise_dist;
        std::normal_distribution<double> point_generator;
        std::default_random_engine generator;
        
        //Make normal distribution of noise and point generator
        noise_dist = std::normal_distribution<double>(mean, noise_cov);
        point_generator = std::normal_distribution<double>(mean, point_generator_cov);

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        R = Eigen::AngleAxisd(M_PI * 0.2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-M_PI * 0.1, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(M_PI * 0.1, Eigen::Vector3d::UnitX());
        // R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
        //     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        //     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        t = Eigen::Vector3d(10, 3, -7);
        T_gt_ = Math::RAndtToT(R, t);
        
        //Make 3d point for 1 frame observation
        for(int i = 0; i < 50; i++)
            p1_vec_.emplace_back(point_generator(generator), point_generator(generator), point_generator(generator));

        Eigen::Matrix4d T_noise;
        Eigen::Matrix3d R_noise;
        Eigen::Vector3d t_noise;
        R_noise = Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitX());
        t_noise = Eigen::Vector3d(noise_dist(generator),noise_dist(generator),noise_dist(generator));
        T_noise = Math::RAndtToT(R_noise, t_noise);
        T_initial_guess_ = T_gt_ * T_noise;
        for(int i = 0; i < p1_vec_.size(); i++)
            p2_ob_vec_.push_back(Math::TransformationPoint(T_initial_guess_.inverse(), p1_vec_.at(i))
                                + Eigen::Vector3d(noise_dist(generator), noise_dist(generator), noise_dist(generator)));
    }

    std::vector<Eigen::Vector3d> p1_vec_;
    std::vector<Eigen::Vector3d> p2_ob_vec_;
    Eigen::Matrix4d T_gt_;
    Eigen::Matrix4d T_initial_guess_;
};

TEST_F(PointToPointICPTest, SolveLeastSquare) {
    Eigen::Matrix4d x_estimation;
    x_estimation = T_initial_guess_;
    PointToPointICP::SolveLeastSquare(p1_vec_, p2_ob_vec_, x_estimation, &x_estimation, 50);
    std::cout << "T_gt : " << std::endl;
    std::cout << T_gt_ << std::endl;
    std::cout << "T_initial_guess : " << std::endl;
    std::cout << T_initial_guess_ << std::endl;
    std::cout << "x_estimation : " << std::endl;
    std::cout << x_estimation << std::endl;
}

}