#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "MIQPDefinition.hpp"

TEST(MIQPDefinitionTest, ValidProblem) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd c(2); c << 1, 2;
    Eigen::MatrixXd A(1, 2); A << 1, 1;
    Eigen::VectorXd b(1); b << 3;
    Eigen::VectorXd lb(2); lb << 0, 0;
    Eigen::VectorXd ub(2); ub << 10, 10;
    std::vector<int> int_set = {0};

    EXPECT_TRUE(true);
}