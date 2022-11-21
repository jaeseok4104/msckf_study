#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

#include "custom_msckf/include/math.h"

namespace custom_msckf {
TEST(Math, AxisMatrixToVector) {
    Eigen::Matrix3d r;
    Eigen::Vector3d r_vector;
    r = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    r_vector = Math::AxisMatrixToVector(r);
    Eigen::Matrix3d result = Math::AxisVectorToMatrix(r_vector);

    std::cout << r << std::endl;
    std::cout << r_vector << std::endl;
    std::cout << result << std::endl;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            EXPECT_NEAR(r(i,j),result(i,j),0.0000001);
}

TEST(Math, TransformationPoint) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d r;
    Eigen::Vector3d point;
    Eigen::Vector4d point_homo;
    T.setIdentity();
    // r = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    // T.block<3,3>(0,0) = r;
    T.block<3,1>(0,3) = Eigen::Vector3d(5,0,0);
    point = Eigen::Vector3d(0, 0, 0);
    point_homo = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d result = Math::TransformationPoint(T.inverse(), point);
    Eigen::Vector4d result_homo = T.inverse() * point_homo;

    std::cout << result << std::endl;
    std::cout << result_homo << std::endl;

    for(int i = 0; i < 3; i++)
        EXPECT_NEAR(result(i), result_homo(i),0.0000001);
}

TEST(Math, TransformationVectorInverse) {
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Eigen::VectorXd state_6d(6);
    
    T.setIdentity();
    R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    t = Eigen::Vector3d(5, 5, 5);
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    state_6d.setIdentity();
    state_6d.block<3,1>(0,0) = Math::AxisMatrixToVector(R);
    state_6d.block<3,1>(3,0) = t;
    
    Eigen::Matrix4d result = Math::TransformationVectorInverse(state_6d);
    Eigen::Matrix4d compare = T.inverse();
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            EXPECT_NEAR(result(i,j), compare(i,j),0.0000001);
}

}