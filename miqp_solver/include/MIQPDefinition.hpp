#pragma once

#include <Eigen/Dense>   
#include <Eigen/Sparse>

namespace miqp_solver {
    class MIQPDefinition {
        public:
            MIQPDefinition(Eigen::SparseMatrix<double> Q, Eigen::VectorXd c, Eigen::SparseMatrix<double> A, Eigen::VectorXd b, Eigen::VectorXd lb, Eigen::VectorXd ub, std::vector<int> int_list);
            
            // Declare Getters
            const Eigen::SparseMatrix<double>& getQ() const;
            const Eigen::VectorXd& getc() const;
            const Eigen::SparseMatrix<double>& getA() const;
            const Eigen::VectorXd& getb() const;
            const Eigen::VectorXd& getlb() const;
            const Eigen::VectorXd& getub() const;
            const std::vector<int>& getIntList() const;
            

        private:
            // Member variables
            Eigen::SparseMatrix<double> Q_; // Quadratic cost matrix 
            Eigen::VectorXd c_; // Linear cost vector
            Eigen::SparseMatrix<double> A_; // Linear constraint matrix
            Eigen::VectorXd b_; // Linear constraint vector
            Eigen::VectorXd lb_; // Lower bounds
            Eigen::VectorXd ub_; // Upper bounds
            std::vector<int> int_list_; // List of integer variables

    };
}