#pragma once

#include <Eigen/Dense>

namespace miqp_solver{
    class MIQPNode {    
        public:
            MIQPNode(Eigen::VectorXd lb, Eigen::VectorXd ub, int depth);
        
            Eigen::VectorXd getLb() const;
            Eigen::VectorXd getUb() const;
            Eigen::VectorXd getSol() const;
            double getObj() const;
    
            void setSol(const Eigen::VectorXd sol, double obj);
    
        private:
            Eigen::VectorXd lb_;
            Eigen::VectorXd ub_;
            Eigen::VectorXd sol;
            double obj;
            int depth;
    };
}
