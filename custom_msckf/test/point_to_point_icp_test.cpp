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
        Eigen::Matrix3d r;
        r = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        T_gt.setIdentity();
        T_gt.block<3,3>(0,0) = r;
        T_gt.block<3,1>(0,3) = Eigen::Vector3d(5,5,5);

        //Make normal distribution of noise and point generator
        noise_dist = std::normal_distribution<double>(mean, noise_cov);
        point_generator = std::normal_distribution<double>(mean, point_generator_cov);
        
        //Make 3d point for 1 frame observation
        for(int i = 0; i < 5; i++)
            p1_vec.emplace_back(point_generator(generator), point_generator(generator), point_generator(generator));

        Eigen::Matrix4d T_noise;
        Eigen::Matrix3d r_noise;
        r_noise = Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(noise_dist(generator), Eigen::Vector3d::UnitX());
        T_noise.setIdentity();
        T_noise.block<3,3>(0,0) = r_noise;
        T_noise.block<3,1>(0,3) = Eigen::Vector3d(noise_dist(generator),noise_dist(generator),noise_dist(generator));
        T_initial_guess = T_gt * T_noise;
        for(int i = 0; i < 5; i++)
            // p2_ob_vec.push_back(Math::TransformationPoint(T_gt.inverse(), p1_vec.at(i)));
            p2_ob_vec.push_back(Math::TransformationPoint(T_initial_guess.inverse(), p1_vec.at(i))
                                + Eigen::Vector3d(noise_dist(generator), noise_dist(generator), noise_dist(generator)));
    }

    const double mean = 0.0;
    const double noise_cov = 0.001;
    const double point_generator_cov = 30;
    std::normal_distribution<double> noise_dist;
    std::normal_distribution<double> point_generator;
    std::default_random_engine generator;

    std::vector<Eigen::Vector3d> p1_vec;
    std::vector<Eigen::Vector3d> p2_ob_vec;
    Eigen::Matrix4d T_gt;
    Eigen::Matrix4d T_initial_guess;
};

TEST_F(PointToPointICPTest, ErrorTerm) {
    Eigen::Matrix4d x_estimation;
    x_estimation = T_initial_guess;
    std::cout << p1_vec.size() << std::endl;
    std::cout << p2_ob_vec.size() << std::endl;
    PointToPointICP::SolveLeastSquare(p1_vec, p2_ob_vec, x_estimation, &x_estimation, 50);
    std::cout << "T_gt : " << std::endl;
    std::cout << T_gt << std::endl;
    std::cout << "T_initial_guess : " << std::endl;
    std::cout << T_initial_guess << std::endl;
    std::cout << "x_estimation : " << std::endl;
    std::cout << x_estimation << std::endl;
}
}