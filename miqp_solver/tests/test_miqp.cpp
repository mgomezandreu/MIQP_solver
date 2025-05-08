#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "MIQPDefinition.hpp"

using namespace miqp_solver;

TEST(MIQPDefinitionTest, ValidProblem) {
    Eigen::VectorXd c(2); c << 1, 2;
    Eigen::VectorXd b(1); b << 3;
    Eigen::VectorXd lb(2); lb << 0, 0;
    Eigen::VectorXd ub(2); ub << 10, 10;
    std::vector<int> int_set = {0};

    Eigen::SparseMatrix<double> Q(2, 2);
    Q.insert(0, 0) = 1.0;
    Q.insert(1, 1) = 2.0;
    Q.makeCompressed();

    Eigen::SparseMatrix<double> A(1, 2);
    A.insert(0, 0) = 1.0;
    A.insert(0, 1) = 1.0;
    A.makeCompressed();


    // Expect no assertion errors
    EXPECT_NO_THROW(MIQPDefinition(Q, c, A, b, lb, ub, int_set));
}

TEST(MIQPDefinitionTest, InvalidQ) {
    Eigen::VectorXd c(2); c << 1, 2;
    Eigen::VectorXd b(1); b << 3;
    Eigen::VectorXd lb(2); lb << 0, 0;
    Eigen::VectorXd ub(2); ub << 10, 10;
    std::vector<int> int_set = {0};

    Eigen::SparseMatrix<double> Q(2, 3); // Invalid size
    Q.insert(0, 0) = 1.0;
    Q.insert(1, 1) = 2.0;
    Q.makeCompressed();

    Eigen::SparseMatrix<double> A(1, 2);
    A.insert(0, 0) = 1.0;
    A.insert(0, 1) = 1.0;
    A.makeCompressed();

    // Expect death on creation
    EXPECT_DEATH(MIQPDefinition(Q, c, A, b, lb, ub, int_set), "Q must be a square matrix");
}