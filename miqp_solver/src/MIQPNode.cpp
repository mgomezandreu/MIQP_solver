
#include <MIQPNode.hpp>
#include <cassert>

namespace miqp_solver {
    MIQPNode::MIQPNode(Eigen::VectorXd lb, Eigen::VectorXd ub, int depth)
        : lb_(lb), ub_(ub), depth(depth) {
        assert(lb_.size() == ub_.size() && "Lower and upper bounds must have the same size");
        assert(depth >= 0 && "Depth must be non-negative");
    }

    // Const getters
    Eigen::VectorXd MIQPNode::getLb() const {
        return lb_;
    }
    Eigen::VectorXd MIQPNode::getUb() const {
        return ub_;
    }
    Eigen::VectorXd MIQPNode::getSol() const {
        return sol;
    }
    double MIQPNode::getObj() const {
        return obj;
    }

    void MIQPNode::setSol(const Eigen::VectorXd sol, double obj) {
        assert(sol.size() == lb_.size() && "Solution must have the same size as bounds");
        this->sol = sol;
        this->obj = obj;
    }
} // namespace miqp_solver
