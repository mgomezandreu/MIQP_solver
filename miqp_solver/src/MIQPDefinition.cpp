#include <MIQPDefinition.hpp>   
#include <cassert>
#include <set>

namespace miqp_solver {
    MIQPDefinition::MIQPDefinition(Eigen::SparseMatrix<double> Q, Eigen::VectorXd c, Eigen::SparseMatrix<double> A, Eigen::VectorXd b, Eigen::VectorXd lb, Eigen::VectorXd ub,
                                std::vector<int> int_list)
        : Q_(Q), c_(c), A_(A), b_(b), lb_(lb), ub_(ub), int_list_(int_list) { 

        assert(Q_.rows() == Q_.cols() && "Q must be a square matrix");
        assert(Q_.rows() == c_.size() && "Q and c must have compatible dimensions");
        assert(A_.rows() == b_.size() && "A and b must have compatible dimensions");
        assert(A_.cols() == Q_.rows() && "A must have the same number of columns as Q");
        assert(lb_.size() == Q_.rows() && "lb and Q must have compatible dimensions");
        assert(ub_.size() == Q_.rows() && "ub and Q must have compatible dimensions");
        assert(int_list_.size() > 0 && "int_list must not be empty");
        for (int i : int_list_) {
            assert(i >= 0 && i < Q_.rows() && "Indices in int_list must be valid");
        }
        assert(int_list_.size() <= Q_.rows() && "int_list cannot have more elements than the number of variables");
        assert(int_list_.size() == std::set<int>(int_list_.begin(), int_list_.end()).size() && "int_list must contain unique indices");
        
    };

    // Const getters
    const Eigen::SparseMatrix<double>& MIQPDefinition::getQ() const {
        return Q_;
    }
    const Eigen::VectorXd& MIQPDefinition::getc() const {
        return c_;
    }
    const Eigen::SparseMatrix<double>& MIQPDefinition::getA() const {
        return A_;
    }
    const Eigen::VectorXd& MIQPDefinition::getb() const {
        return b_;
    }
    const Eigen::VectorXd& MIQPDefinition::getlb() const {
        return lb_;
    }
    const Eigen::VectorXd& MIQPDefinition::getub() const {
        return ub_;
    }
    const std::vector<int>& MIQPDefinition::getIntList() const {
        return int_list_;
    }
}