#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <vector>
#include <optional>

#include <MIQPDefinition.hpp>
#include <MIQPNode.hpp>

namespace miqp_solver {
    class MIQPSolver {
        public:
            MIQPSolver(const MIQPDefinition& miqp_definition);
            bool solve(Eigen::VectorXd& solution, double& objective_value);

        private:
            OsqpEigen::Solver solver_;
            MIQPDefinition miqp_definition_;


            // Checks if the solution is integral with respect to the integer variables defined in the MIQP Definition
            bool isIntegral(const Eigen::VectorXd& x);

            // Creates a new MIQPNode with the given lower and upper bounds, returns false, Node if the node is infeasible
            std::optional<MIQPNode> createNode(Eigen::VectorXd P, Eigen::VectorXd ub);
    };
}